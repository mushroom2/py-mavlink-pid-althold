#!/usr/bin/env python3
"""
Altitude + drift hold via SET_ATTITUDE_TARGET (thrust-as-thrust) -- ASYNC.

All tunable values live in config.yaml and are loaded via settings.Settings.
Run:  python3 alt_hold_pid_async.py [config.yaml]
"""

import asyncio
import math
import sys
import time
from collections import deque

from pymavlink import mavutil

from settings import Settings, ConfigError, Phase
from plot import FlightLog, plot_flight


# ----------------------------------------------------------------------------
# PID (anti-windup, feed-forward, asymmetric limits) -- pure compute
# ----------------------------------------------------------------------------
class PID:
    def __init__(self, kp, ki, kd, i_limit, out_min=None, out_max=None,
                 feedforward=0.0, d_window=10):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit = i_limit
        self.out_min, self.out_max = out_min, out_max
        self.ff = feedforward
        self.integral = 0.0
        self.hist = deque(maxlen=d_window)
        self.p_term = self.i_term = self.d_term = 0.0

    def reset(self):
        self.integral = 0.0
        self.hist.clear()

    def update(self, error, dt, meas_rate=None):
        if meas_rate is not None:
            d = -meas_rate
        else:
            self.hist.append(error)
            d = ((self.hist[-1] - self.hist[0]) / (dt * (len(self.hist) - 1))
                 if len(self.hist) >= 2 else 0.0)

        integ = self.integral + self.ki * error * dt
        integ = max(-self.i_limit, min(self.i_limit, integ))

        self.p_term, self.i_term, self.d_term = self.kp * error, integ, self.kd * d
        out = self.ff + self.p_term + self.i_term + self.d_term

        lo = self.out_min if self.out_min is not None else -1e18
        hi = self.out_max if self.out_max is not None else 1e18
        sat = max(lo, min(hi, out))
        if not ((out > hi and error > 0) or (out < lo and error < 0)):
            self.integral = integ
        return sat


class Controller:
    """Builds the axis PIDs from config; holds the yaw target."""
    def __init__(self, hover, cfg: Settings):
        self.cfg = cfg
        a, p, r = cfg.altitude_pid, cfg.pitch_pid, cfg.roll_pid
        self.hover = hover
        self.alt = PID(a.kp, a.ki, a.kd, a.i_limit, out_min=cfg.thrust_min,
                       out_max=cfg.thrust_max, feedforward=hover, d_window=a.d_window)
        self.pitch = PID(p.kp, p.ki, p.kd, p.i_limit, out_min=-cfg.tilt_max_rad,
                         out_max=cfg.tilt_max_rad, d_window=p.d_window)
        self.roll = PID(r.kp, r.ki, r.kd, r.i_limit, out_min=-cfg.tilt_max_rad,
                        out_max=cfg.tilt_max_rad, d_window=r.d_window)
        self.yaw_target = None

    def reset(self):
        self.alt.reset()
        self.pitch.reset()
        self.roll.reset()


# ----------------------------------------------------------------------------
# Math helpers
# ----------------------------------------------------------------------------
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


# ----------------------------------------------------------------------------
# Shared state (written only by reader_task)
# ----------------------------------------------------------------------------
class VehicleState:
    def __init__(self):
        self.alt = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.yaw = 0.0
        self.last_pos_t = 0.0
        self.pos_n = 0.0            # north [m] from LOCAL_POSITION_NED
        self.pos_e = 0.0            # east  [m]
        self.last_local_t = 0.0
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

    def request_local_stream(self, hz):
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            int(1e6 / hz), 0, 0, 0, 0, 0)

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
                elif t == "LOCAL_POSITION_NED":
                    state.pos_n = msg.x
                    state.pos_e = msg.y
                    state.last_local_t = time.time()
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

    async def wait_for_local(self, state, timeout=10.0):
        t_end = time.time() + timeout
        while time.time() < t_end:
            if state.last_local_t > 0:
                return True
            await asyncio.sleep(0.05)
        print("WARNING: no LOCAL_POSITION_NED -- position moves disabled")
        return False


class Copter(object):
    def __init__(self, state, comm, cfg: Settings):
        self.state = state
        self.comm = comm
        self.cfg = cfg
        self.ctrl = None
        self.last_request_position = 0
        self.log = FlightLog()

    async def hold_altitude(self, phase: Phase):
        """Reach phase.target_m; if the phase has a move, fly to the point
        (distance_m at bearing azimuth_deg from where altitude was reached);
        the hold timer starts only once altitude AND position are satisfied."""
        cfg = self.cfg
        target_alt, hold_time = phase.target_m, phase.hold_s
        label = f"{target_alt} m"
        if phase.has_move:
            label += f", move {phase.distance_m:g} m @ {phase.azimuth_deg:g}deg"
        print(f"\n=== {label}, hold {hold_time} s ===")

        self.ctrl.reset()
        period = cfg.period
        sp = self.state.alt              # slewed altitude setpoint
        h_target = None                  # (north, east) once altitude reached
        captured = False                 # have we captured the horizontal target?
        move_active = phase.has_move
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

            # ---- altitude (slewed setpoint) -> thrust ----
            sp += clamp(target_alt - sp, -cfg.climb_rate_max_ms * dt,
                        cfg.climb_rate_max_ms * dt)
            alt_err = target_alt - alt
            thrust = self.ctrl.alt.update(sp - alt, dt, meas_rate=vz)
            alt_ok = abs(alt_err) < cfg.reach_tol_m
            have_pos = self.state.last_local_t > 0

            # ---- capture the horizontal target the moment altitude is reached ----
            if alt_ok and not captured:
                captured = True
                if have_pos:
                    ref_n, ref_e = self.state.pos_n, self.state.pos_e
                    if move_active:
                        az = math.radians(phase.azimuth_deg)
                        h_target = (ref_n + phase.distance_m * math.cos(az),
                                    ref_e + phase.distance_m * math.sin(az))
                        print(f"[move] to N={h_target[0]:.1f} E={h_target[1]:.1f}")
                    else:
                        h_target = (ref_n, ref_e)      # station-keep here
                else:
                    if move_active:
                        print("WARNING: no position -> skipping move, altitude-only hold")
                    move_active = False                # cannot move without position

            # ---- horizontal: position error -> velocity setpoint (NED) ----
            if h_target is not None:
                en = h_target[0] - self.state.pos_n
                ee = h_target[1] - self.state.pos_e
                dist_err = math.hypot(en, ee)
                if dist_err > 1e-3:
                    v_des = min(cfg.pos_kp * dist_err, cfg.pos_speed_max_ms)
                    vset_n, vset_e = v_des * en / dist_err, v_des * ee / dist_err
                else:
                    vset_n = vset_e = 0.0
            else:
                dist_err = 0.0
                vset_n = vset_e = 0.0                   # drift arrest (climb / no pos)

            # ---- velocity error -> body frame -> tilt ----
            evn, eve = vset_n - vx, vset_e - vy
            err_fwd   =  evn * math.cos(yaw) + eve * math.sin(yaw)
            err_right = -evn * math.sin(yaw) + eve * math.cos(yaw)
            pitch = cfg.pitch_sign * self.ctrl.pitch.update(err_fwd, dt)
            roll  = cfg.roll_sign  * self.ctrl.roll.update(err_right, dt)

            self.comm.send_attitude(euler_to_quat(roll, pitch, self.ctrl.yaw_target), thrust)

            speed = math.hypot(vx, vy)
            self.log.add(target=target_alt, alt=alt, err=alt_err, thrust=thrust,
                         vz=vz, p=self.ctrl.alt.p_term, i=self.ctrl.alt.i_term,
                         d=self.ctrl.alt.d_term, roll=roll, pitch=pitch, drift=speed)

            now = time.time()
            if now - last_print > 1.0:

                extra = f" dist={dist_err:4.1f}m" if move_active else ""
                print(f"alt={alt:6.2f} err={alt_err:+5.2f} thr={thrust:.3f}{extra} | "
                      f"spd={speed:4.2f} roll={math.degrees(roll):+4.1f} "
                      f"pitch={math.degrees(pitch):+4.1f}")
                last_print = now

            # ---- hold timer: needs altitude AND (if moving) arrival at the point ----
            pos_ready = (not move_active) or (h_target is not None
                                              and dist_err < cfg.pos_reach_tol_m)
            if reached_at is None:
                if captured and alt_ok and pos_ready:
                    reached_at = now
                    print(f"Arrived -- holding {hold_time} s")
            elif now - reached_at >= hold_time:
                print("Hold complete")
                return

            if (now - self.state.last_pos_t > cfg.stream_timeout_s) and \
               (now - self.last_request_position > 1):
                print('warning! pos timeout reached')
                self.comm.request_position_stream(cfg.rate_hz)
                self.comm.request_local_stream(cfg.rate_hz)
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

    async def prepare(self):
        # startup ArduPilot params from config (GUID_OPTIONS, FRAME_CLASS, ...)
        for name, val in self.cfg.ardupilot_params.items():
            await self.comm.set_param(self.state, name, val)

        hover = await self.comm.get_param(self.state, "MOT_THST_HOVER",
                                          self.cfg.hover_default) or self.cfg.hover_default
        print(f"Hover thrust feed-forward: {hover:.3f}")

        self.comm.request_position_stream(self.cfg.rate_hz)
        self.comm.request_local_stream(self.cfg.rate_hz)
        await self.comm.wait_for_position(self.state)
        await self.comm.wait_for_local(self.state)

        await self.comm.set_mode(self.state, "GUIDED")
        await self.comm.arm(self.state)
        self.ctrl = Controller(hover, self.cfg)

    async def fly_mission(self):
        for ph in self.cfg.phases:
            await self.hold_altitude(ph)


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------
def connect(address):
    m = mavutil.mavlink_connection(address)
    print(f"Waiting for heartbeat on {address} ...")
    m.wait_heartbeat()
    print(f"Heartbeat from system {m.target_system} component {m.target_component}")
    return m


async def main(cfg: Settings):
    print(cfg.summary())
    m = connect(cfg.connection_address)
    state = VehicleState()
    comm = MavlinkCommunicator(m)
    reader = asyncio.create_task(comm.reader_task(state))
    copter = Copter(state, comm, cfg)
    try:
        await copter.prepare()
        try:
            await copter.fly_mission()
        finally:
            await copter.land_and_wait()
    finally:
        reader.cancel()
        try:
            await reader
        except asyncio.CancelledError:
            pass
        if len(copter.log):
            copter.log.to_csv(cfg.csv_path)
            plot_flight(copter.log, save_path=cfg.plot_path, show=cfg.show_plot)


if __name__ == "__main__":
    config_path = sys.argv[1] if len(sys.argv) > 1 else "config.yaml"
    try:
        cfg = Settings.load(config_path)
    except ConfigError as e:
        sys.exit(f"config error: {e}")
    asyncio.run(main(cfg))