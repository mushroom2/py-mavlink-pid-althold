#!/usr/bin/env python3
"""
Altitude + position-drift hold via SET_ATTITUDE_TARGET (thrust-as-thrust).

Control architecture (GUID_OPTIONS bit 3 = raw thrust):
  altitude PID          -> thrust      (feed-forward around hover)
  forward-velocity PID  -> pitch cmd   (arrests fore/aft drift)
  right-velocity  PID   -> roll  cmd   (arrests left/right drift)
  yaw                   -> held at captured heading (commanded directly)

The three axis commands are packed into one attitude quaternion and sent with
the thrust each control cycle. Horizontal PIDs drive body-frame velocity to 0,
which is what stops the sideways drift you were seeing on lift-off.

SITL:  sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
Run :  python3 alt_hold_pid.py
"""

import math
import time
from collections import deque
from pymavlink import mavutil

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------
CONNECTION    = "udp:127.0.0.1:14550"
RATE_HZ       = 150
HOVER_THRUST  = 0.5
REACH_TOL     = 0.1
HOVER_OFFSET = 0

# --- Altitude PID (error [m]; derivative on climb rate vz) ---
ALT_KP, ALT_KI, ALT_KD, ALT_ILIM = 0.12, 0.045, 0.1, 0.30
THRUST_MIN, THRUST_MAX = 0.0, 0.5

# --- Horizontal velocity PIDs (error [m/s] -> tilt [rad]) ---
VEL_KP_PITCH, VEL_KI_PITCH, VEL_KD_PITCH, VEL_ILIM_PITCH = 0.04, 0.01, 0.005, 0.05
VEL_KP_ROLL, VEL_KI_ROLL, VEL_KD_ROLL, VEL_ILIM_ROLL =  0.04, 0.01, 0.005, 0.05
TILT_MAX = math.radians(5)   # max commanded roll/pitch

# If the copter accelerates INTO the drift instead of arresting it, flip a sign.
PITCH_SIGN = -1.0
ROLL_SIGN  = 1.0

D_WINDOW = 10                 # history-queue length for filtered derivative


# ----------------------------------------------------------------------------
# Generic PID with a history queue (filtered derivative)
# ----------------------------------------------------------------------------
class PID:
    def __init__(self, kp, ki, kd, i_limit, out_limit=None, d_window=D_WINDOW):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit = i_limit
        self.out_limit = out_limit
        self.integral = 0.0
        self.hist = deque(maxlen=d_window)   # recent errors -> smoothed derivative

    def reset(self):
        self.integral = 0.0
        self.hist.clear()

    def update(self, error, dt, meas_rate=None):
        # Integral term (stored already scaled by ki) with anti-windup clamp
        self.integral += self.ki * error * dt
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))

        # Derivative: prefer a measured rate (no noise from differencing);
        # otherwise use a filtered slope over the history queue.
        if meas_rate is not None:
            d = -meas_rate
        else:
            self.hist.append(error)
            if len(self.hist) >= 2:
                d = (self.hist[-1] - self.hist[0]) / (dt * (len(self.hist) - 1))
            else:
                d = 0.0

        out = self.kp * error + self.integral + self.kd * d
        if self.out_limit is not None:
            out = max(-self.out_limit, min(self.out_limit, out))
        return out


class Controller:
    """Bundles the axis PIDs and the yaw target."""
    def __init__(self, hover):
        self.hover = hover
        self.alt   = PID(ALT_KP, ALT_KI, ALT_KD, ALT_ILIM)
        self.pitch = PID(VEL_KP_PITCH, VEL_KI_PITCH, VEL_KD_PITCH, VEL_ILIM_PITCH, out_limit=TILT_MAX)
        self.roll  = PID(VEL_KP_ROLL, VEL_KI_ROLL, VEL_KD_ROLL, VEL_ILIM_ROLL, out_limit=TILT_MAX)
        self.yaw_target = None

    def reset(self):
        self.alt.reset()
        self.pitch.reset()
        self.roll.reset()


# ----------------------------------------------------------------------------
# Math helpers
# ----------------------------------------------------------------------------
def euler_to_quat(roll, pitch, yaw):
    """Aerospace ZYX (yaw-pitch-roll) -> quaternion [w, x, y, z]."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,   # w
        sr * cp * cy - cr * sp * sy,   # x
        cr * sp * cy + sr * cp * sy,   # y
        cr * cp * sy - sr * sp * cy,   # z
    ]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ----------------------------------------------------------------------------
# MAVLink helpers
# ----------------------------------------------------------------------------
def _pid_str(param_id):
    if isinstance(param_id, bytes):
        return param_id.split(b"\x00", 1)[0].decode(errors="ignore")
    return param_id.split("\x00", 1)[0]


def connect(conn_str):
    m = mavutil.mavlink_connection(conn_str)
    print(f"Waiting for heartbeat on {conn_str} ...")
    m.wait_heartbeat()
    print(f"Heartbeat from system {m.target_system} component {m.target_component}")
    return m


def set_param(m, name, value, timeout=5.0):
    value = float(value)
    ptype = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    t_end = time.time() + timeout
    last_send = 0.0
    while time.time() < t_end:
        now = time.time()
        if now - last_send > 1.0:
            m.mav.param_set_send(m.target_system, m.target_component,
                                 name.encode(), value, ptype)
            last_send = now
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg and _pid_str(msg.param_id) == name and abs(msg.param_value - value) < 1e-4:
            print(f"Param OK: {name} = {msg.param_value:g}")
            return msg.param_value
    raise RuntimeError(f"Failed to set/confirm {name} = {value}")


def get_param(m, name, default=None, timeout=3.0):
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  name.encode(), -1)
    t_end = time.time() + timeout
    while time.time() < t_end:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg and _pid_str(msg.param_id) == name:
            return msg.param_value
    return default


def set_mode(m, mode_name):
    mode_id = m.mode_mapping()[mode_name]
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id, 0, 0, 0, 0, 0)
    t_end = time.time() + 5
    while time.time() < t_end:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and hb.custom_mode == mode_id:
            print(f"Mode: {mode_name}")
            return True
    raise RuntimeError(f"Failed to enter mode {mode_name}")


def arm(m, timeout=15.0):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    print("Arming ...")
    t_end = time.time() + timeout
    while time.time() < t_end:
        st = m.recv_match(type="STATUSTEXT", blocking=False)
        if st:
            print(f"  AP: {st.text}")
        if m.motors_armed():
            print("Armed")
            return
        time.sleep(0.5)
    raise RuntimeError("Arming failed (see AP prearm messages above)")


def takeoff(m, alt):
    print(f"Guided takeoff to {alt} m ...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, float(alt))
    t_end = time.time() + 20
    while time.time() < t_end:
        st = get_state(m)
        if st[0] >= alt * 0.9:
            print(f"Airborne at {st[0]:.2f} m")
            return
    raise RuntimeError("Takeoff did not reach target altitude")


def request_position_stream(m, hz):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        int(1e6 / hz), 0, 0, 0, 0, 0)


def get_state(m):
    """Return (alt[m], vx_north[m/s], vy_east[m/s], vz_up[m/s], yaw[rad])."""
    msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
    if msg is None:
        raise RuntimeError("No GLOBAL_POSITION_INT received")
    alt = msg.relative_alt / 1000.0
    vx  = msg.vx / 100.0            # north
    vy  = msg.vy / 100.0            # east
    vz  = -msg.vz / 100.0          # up (msg.vz is +down)
    yaw = math.radians(msg.hdg / 100.0) if msg.hdg != 65535 else 0.0
    return alt, vx, vy, vz, yaw


def send_attitude(m, quat, thrust):
    m.mav.set_attitude_target_send(
        int(time.monotonic() * 1000) & 0xFFFFFFFF,
        m.target_system, m.target_component,
        0b00000111,          # ignore body rates; use attitude + thrust
        quat, 0.0, 0.0, 0.0,
        float(thrust))


# ----------------------------------------------------------------------------
# Flight phase: reach target_alt and hold it for hold_time (with drift arrest)
# ----------------------------------------------------------------------------
def hold_altitude(m, ctrl, target_alt, hold_time):
    print(f"\n=== Target {target_alt} m, hold {hold_time} s ===")
    ctrl.reset()
    dt = 1.0 / RATE_HZ
    reached_at = None
    last_print = 0.0

    while True:
        loop_t0 = time.time()
        alt, vx, vy, vz, yaw = get_state(m)
        if ctrl.yaw_target is None:
            ctrl.yaw_target = yaw     # capture heading once, then hold it

        # --- altitude -> thrust ---
        alt_err = target_alt - alt
        thrust = clamp(ctrl.hover + HOVER_OFFSET + ctrl.alt.update(alt_err, dt, meas_rate=vz),
                       THRUST_MIN, THRUST_MAX)

        # --- horizontal drift arrest: NED velocity -> body frame -> tilt ---
        fwd   =  vx * math.cos(yaw) + vy * math.sin(yaw)   # body-forward speed
        right = -vx * math.sin(yaw) + vy * math.cos(yaw)   # body-right speed
        pitch = PITCH_SIGN * ctrl.pitch.update(-fwd, dt)   # oppose forward vel
        roll  = ROLL_SIGN  * ctrl.roll.update(-right, dt)  # oppose right vel

        q = euler_to_quat(roll, pitch, ctrl.yaw_target)
        send_attitude(m, q, thrust)

        now = time.time()
        if now - last_print > 1.0:
            drift = math.hypot(fwd, right)
            print(f"alt={alt:6.2f}  err={alt_err:+5.2f}  thr={thrust:.3f} | "
                  f"drift={drift:4.2f} m/s  roll={math.degrees(roll):+4.1f} "
                  f"pitch={math.degrees(pitch):+4.1f}")
            last_print = now

        if reached_at is None:
            if abs(alt_err) < REACH_TOL:
                reached_at = now
                print(f"Reached {target_alt} m -- holding {hold_time} s")
        elif now - reached_at >= hold_time:
            print(f"Hold at {target_alt} m complete")
            return

        sleep = dt - (time.time() - loop_t0)
        if sleep > 0:
            time.sleep(sleep)
        if alt == 0.0:
            ctrl.reset()


def land_and_wait(m):
    print("\n=== LAND ===")
    set_mode(m, "LAND")
    print("Waiting for disarm ...")
    m.motors_disarmed_wait()
    print("Disarmed. Scenario complete.")


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------
def main():
    m = connect(CONNECTION)
    # land_and_wait(m)
    set_param(m, "GUID_OPTIONS", 8)
    print(f"GUID_OPTIONS readback = {get_param(m, 'GUID_OPTIONS')}")

    hover = get_param(m, "MOT_THST_HOVER", HOVER_THRUST) or HOVER_THRUST
    print(f"Hover thrust feed-forward: {hover:.3f}")

    request_position_stream(m, RATE_HZ)

    set_mode(m, "GUIDED")
    arm(m)
    ctrl = Controller(hover)
    try:
        hold_altitude(m, ctrl, target_alt=14.5, hold_time=2)
        hold_altitude(m, ctrl, target_alt=15.0, hold_time=15.0)
        hold_altitude(m, ctrl, target_alt=10.0, hold_time=15.0)
        hold_altitude(m, ctrl, target_alt=1, hold_time=2)
    finally:
        land_and_wait(m)


if __name__ == "__main__":
    main()