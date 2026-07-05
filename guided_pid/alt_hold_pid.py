#!/usr/bin/env python3
"""
Altitude + drift hold via SET_ATTITUDE_TARGET (thrust-as-thrust) -- ASYNC version.
"""

import asyncio
import math
import time
from collections import deque
from pymavlink import mavutil
from plot import FlightLog, plot_flight

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------
CONNECTION    = "udp:127.0.0.1:14550"
RATE_HZ       = 10
HOVER_THRUST  = 0.5
REACH_TOL     = 0.1
STREAM_TIMEOUT = 0.3

# --- Altitude PID (error [m]; derivative on climb rate vz) ---
ALT_KP, ALT_KI, ALT_KD, ALT_ILIM = 0.12, 0.045, 0.1, 0.30
THRUST_MIN, THRUST_MAX = 0.0, 0.5

# --- Horizontal velocity PIDs (error [m/s] -> tilt [rad]) ---
VEL_KP_PITCH, VEL_KI_PITCH, VEL_KD_PITCH, VEL_ILIM_PITCH = 0.04, 0.01, 0.005, 0.05
VEL_KP_ROLL,  VEL_KI_ROLL,  VEL_KD_ROLL,  VEL_ILIM_ROLL  = 0.04, 0.01, 0.005, 0.05
TILT_MAX = math.radians(5)

PITCH_SIGN = -1.0
ROLL_SIGN  = 1.0

D_WINDOW = 10
STALE_WARN = 0.5


class PID:
    def __init__(self, kp, ki, kd, i_limit, out_limit=None, d_window=D_WINDOW):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit = i_limit
        self.out_limit = out_limit
        self.integral = 0.0
        self.hist = deque(maxlen=d_window)
        self.p_term = self.i_term = self.d_term = 0.0   # exposed for plotting

    def reset(self):
        self.integral = 0.0
        self.hist.clear()

    def update(self, error, dt, meas_rate=None):
        self.integral += self.ki * error * dt
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))
        if meas_rate is not None:
            d = -meas_rate
        else:
            self.hist.append(error)
            if len(self.hist) >= 2:
                d = (self.hist[-1] - self.hist[0]) / (dt * (len(self.hist) - 1))
            else:
                d = 0.0
        self.p_term = self.kp * error
        self.i_term = self.integral
        self.d_term = self.kd * d
        out = self.p_term + self.i_term + self.d_term
        if self.out_limit is not None:
            out = max(-self.out_limit, min(self.out_limit, out))
        return out


class Controller:
    def __init__(self, hover):
        self.hover = hover
        self.alt   = PID(ALT_KP, ALT_KI, ALT_KD, ALT_ILIM)
        self.pitch = PID(VEL_KP_PITCH, VEL_KI_PITCH, VEL_KD_PITCH, VEL_ILIM_PITCH, out_limit=TILT_MAX)
        self.roll  = PID(VEL_KP_ROLL,  VEL_KI_ROLL,  VEL_KD_ROLL,  VEL_ILIM_ROLL,  out_limit=TILT_MAX)
        self.yaw_target = None

    def reset(self):
        self.alt.reset()
        self.pitch.reset()
        self.roll.reset()


def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _pid_str(param_id):
    if isinstance(param_id, bytes):
        return param_id.split(b"\x00", 1)[0].decode(errors="ignore")
    return param_id.split("\x00", 1)[0]


class VehicleState:
    def __init__(self):
        self.alt = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.yaw = 0.0
        self.last_pos_t = 0.0
        self.mode = None
        self.armed = False
        self.params = {}


class MavlinkCommunicator(object):
    def __init__(self, connection):
        self.conn = connection

    def send_attitude(self, quat, thrust):
        self.conn.mav.set_attitude_target_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            self.conn.target_system, self.conn.target_component,
            0b00000111, quat, 0.0, 0.0, 0.0, float(thrust))

    def request_position_stream(self, hz):
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            int(1e5 / hz), 0, 0, 0, 0, 0)

    async def reader_task(self, state):
        """Continuously drain the link and update shared state. Only caller of recv."""
        while True:
            got = False
            while True:
                msg = self.conn.recv_match(blocking=False)
                if msg is None:
                    break
                got = True
                t = msg.get_type()
                if t == "GLOBAL_POSITION_INT":
                    state.alt = msg.relative_alt / 1000.0
                    state.vx = msg.vx / 100.0
                    state.vy = msg.vy / 100.0
                    state.vz = -msg.vz / 100.0
                    if msg.hdg != 65535:
                        state.yaw = math.radians(msg.hdg / 100.0)
                    state.last_pos_t = time.time()
                elif t == "HEARTBEAT":
                    state.mode = msg.custom_mode
                    state.armed = bool(msg.base_mode &
                                       mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif t == "STATUSTEXT":
                    print(f"  AP: {msg.text}")
                elif t == "PARAM_VALUE":
                    state.params[_pid_str(msg.param_id)] = msg.param_value
            await asyncio.sleep(0 if got else 0.001)

    async def set_param(self, state, name, value, timeout=5.0):
        value = float(value)
        ptype = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        t_end = time.time() + timeout
        while time.time() < t_end:
            self.conn.mav.param_set_send(self.conn.target_system, self.conn.target_component,
                                 name.encode(), value, ptype)
            t_ack = time.time() + 1.0
            while time.time() < t_ack:
                v = state.params.get(name)
                if v is not None and abs(v - value) < 1e-4:
                    print(f"Param OK: {name} = {v:g}")
                    return v
                await asyncio.sleep(0.02)
        raise RuntimeError(f"Failed to set/confirm {name} = {value}")

    async def get_param(self, state, name, default=None, timeout=3.0):
        self.conn.mav.param_request_read_send(self.conn.target_system, self.conn.target_component,
                                      name.encode(), -1)
        t_end = time.time() + timeout
        while time.time() < t_end:
            if name in state.params:
                return state.params[name]
            await asyncio.sleep(0.02)
        return default

    async def set_mode(self, state, mode_name, timeout=5.0):
        mode_id = self.conn.mode_mapping()[mode_name]
        t_end = time.time() + timeout
        while time.time() < t_end:
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id, 0, 0, 0, 0, 0)
            await asyncio.sleep(0.3)
            if state.mode == mode_id:
                print(f"Mode: {mode_name}")
                return
        raise RuntimeError(f"Failed to enter mode {mode_name}")

    async def arm(self, state, timeout=15.0):
        print("Arming ...")
        t_end = time.time() + timeout
        while time.time() < t_end:
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0)
            await asyncio.sleep(0.5)
            if state.armed:
                print("Armed")
                return
        raise RuntimeError("Arming failed (see AP prearm messages above)")

    async def wait_for_position(self, state, timeout=10.0):
        t_end = time.time() + timeout
        while time.time() < t_end:
            if state.last_pos_t > 0:
                return
            await asyncio.sleep(0.05)
        raise RuntimeError("No position estimate received")


class Copter(object):
    def __init__(self, state, comm):
        self.state = state
        self.ctrl = None
        self.comm = comm
        self.last_request_position = 0
        self.log = FlightLog()          # telemetry for plotting

    async def hold_altitude(self, target_alt, hold_time):
        print(f"\n=== Target {target_alt} m, hold {hold_time} s ===")
        self.ctrl.reset()
        period = 1.0 / RATE_HZ
        reached_at = None
        last_print = 0.0
        last_t = time.monotonic()
        next_tick = last_t

        while True:
            alt, vx, vy, vz, yaw = (self.state.alt, self.state.vx, self.state.vy,
                                    self.state.vz, self.state.yaw)
            if self.ctrl.yaw_target is None:
                self.ctrl.yaw_target = yaw

            now_m = time.monotonic()
            dt = now_m - last_t
            last_t = now_m
            if dt <= 0.0:
                dt = period
            dt = min(dt, 0.1)

            alt_err = target_alt - alt
            thrust = clamp(self.ctrl.hover +
                           self.ctrl.alt.update(alt_err, dt, meas_rate=vz),
                           THRUST_MIN, THRUST_MAX)

            fwd   =  vx * math.cos(yaw) + vy * math.sin(yaw)
            right = -vx * math.sin(yaw) + vy * math.cos(yaw)
            pitch = PITCH_SIGN * self.ctrl.pitch.update(-fwd, dt)
            roll  = ROLL_SIGN  * self.ctrl.roll.update(-right, dt)

            self.comm.send_attitude(euler_to_quat(roll, pitch, self.ctrl.yaw_target), thrust)

            drift = math.hypot(fwd, right)
            # log one sample per cycle (P/I/D are the altitude PID's own terms)
            self.log.add(target=target_alt, alt=alt, err=alt_err, thrust=thrust,
                         vz=vz, p=self.ctrl.alt.p_term, i=self.ctrl.alt.i_term,
                         d=self.ctrl.alt.d_term, roll=roll, pitch=pitch, drift=drift)

            now = time.time()
            if now - last_print > 1.0:
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

            if (now - self.state.last_pos_t > STREAM_TIMEOUT) and (now - self.last_request_position > 1):
                print('warning! pos timeout reached')
                self.comm.request_position_stream(RATE_HZ)
                self.last_request_position = now
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                await asyncio.sleep(delay)
            else:
                next_tick = time.monotonic()

    async def land_and_wait(self, timeout=60.0):
        print("\n=== LAND ===")
        await self.comm.set_mode(self.state, "LAND")
        print("Waiting for disarm ...")
        t_end = time.time() + timeout
        while time.time() < t_end:
            if not self.state.armed:
                print("Disarmed. Scenario complete.")
                return
            await asyncio.sleep(0.5)
        print("WARNING: still armed after LAND timeout")

    async def arm(self):
        await self.comm.set_param(self.state, "GUID_OPTIONS", 8)
        await self.comm.set_param(self.state, "FRAME_CLASS", 1)
        await self.comm.set_param(self.state, "FRAME_TYPE", 1)
        print(f"GUID_OPTIONS readback = {await self.comm.get_param(self.state, 'GUID_OPTIONS')}")

        hover = await self.comm.get_param(self.state, "MOT_THST_HOVER", HOVER_THRUST) or HOVER_THRUST
        print(f"Hover thrust feed-forward: {hover:.3f}")

        self.comm.request_position_stream(RATE_HZ)
        await self.comm.wait_for_position(self.state)

        await self.comm.set_mode(self.state, "GUIDED")
        await self.comm.arm(self.state)
        self.ctrl = Controller(hover)


def connect(conn_str):
    m = mavutil.mavlink_connection(conn_str)
    print(f"Waiting for heartbeat on {conn_str} ...")
    m.wait_heartbeat()
    print(f"Heartbeat from system {m.target_system} component {m.target_component}")
    return m


async def main():
    m = connect(CONNECTION)
    state = VehicleState()
    comm = MavlinkCommunicator(m)
    reader = asyncio.create_task(comm.reader_task(state))
    copter = Copter(state, comm)
    try:
        await copter.arm()
        try:
            await copter.hold_altitude(target_alt=14.5, hold_time=2)
            await copter.hold_altitude(target_alt=15.0, hold_time=15.0)
            await copter.hold_altitude(target_alt=10.0, hold_time=15.0)
            await copter.hold_altitude(target_alt=1.0,  hold_time=2)
        finally:
            await copter.land_and_wait()
    finally:
        reader.cancel()
        try:
            await reader
        except asyncio.CancelledError:
            pass
        # render whatever telemetry we collected (works for partial runs too)
        if len(copter.log):
            copter.log.to_csv("flight_log.csv")
            plot_flight(copter.log, save_path="flight_plot.png", show=True)


if __name__ == "__main__":
    asyncio.run(main())