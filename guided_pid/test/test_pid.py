#!/usr/bin/env python3
"""
Unit tests for the altitude/position-hold controller.

Covers the pure-compute pieces (PID, quaternion + geometry helpers) and the
config layer (Settings load / merge / validation). The async MAVLink I/O is not
tested here -- it needs a live link or heavy mocking.

Run:  python3 -m unittest test_flight -v
  or: python3 test_flight.py
"""

import math
import sys
import types
import unittest

# --- make the flight module importable even without pymavlink installed ------
try:
    import pymavlink  # noqa: F401
except ImportError:
    _mav = types.ModuleType("pymavlink")
    _mavutil = types.ModuleType("pymavlink.mavutil")
    _mavutil.mavlink = types.SimpleNamespace(
        MAV_CMD_SET_MESSAGE_INTERVAL=511,
        MAVLINK_MSG_ID_GLOBAL_POSITION_INT=33,
        MAVLINK_MSG_ID_LOCAL_POSITION_NED=32,
        MAV_MODE_FLAG_SAFETY_ARMED=128,
        MAV_CMD_DO_SET_MODE=176,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1,
        MAV_CMD_COMPONENT_ARM_DISARM=400,
        MAV_PARAM_TYPE_REAL32=9,
    )
    _mav.mavutil = _mavutil
    sys.modules["pymavlink"] = _mav
    sys.modules["pymavlink.mavutil"] = _mavutil

from alt_hold_pid_async import (
    PID, Controller, clamp, euler_to_quat,
    target_point, horizontal_velocity_setpoint,
)
from settings import Settings, PIDGains, Phase, ConfigError


# ============================================================================
# Math helpers
# ============================================================================
class TestClamp(unittest.TestCase):
    def test_within(self):
        self.assertEqual(clamp(0.3, 0.0, 1.0), 0.3)

    def test_below_and_above(self):
        self.assertEqual(clamp(-1, 0.0, 1.0), 0.0)
        self.assertEqual(clamp(5, 0.0, 1.0), 1.0)


class TestEulerToQuat(unittest.TestCase):
    def test_identity(self):
        self.assertEqual(euler_to_quat(0, 0, 0), [1.0, 0.0, 0.0, 0.0])

    def test_yaw_90(self):
        w, x, y, z = euler_to_quat(0, 0, math.pi / 2)
        r = math.sqrt(0.5)
        self.assertAlmostEqual(w, r, places=6)
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(z, r, places=6)

    def test_unit_norm(self):
        for roll, pitch, yaw in [(0.1, -0.2, 1.3), (0.5, 0.5, -2.0), (-0.3, 0.7, 0.0)]:
            q = euler_to_quat(roll, pitch, yaw)
            self.assertAlmostEqual(math.sqrt(sum(c * c for c in q)), 1.0, places=6)


class TestTargetPoint(unittest.TestCase):
    def test_cardinals(self):
        cases = [(0, 10, (10, 0)), (90, 10, (0, 10)),
                 (180, 8, (-8, 0)), (270, 5, (0, -5))]
        for az, d, (en, ee) in cases:
            n, e = target_point(0.0, 0.0, az, d)
            self.assertAlmostEqual(n, en, places=6)
            self.assertAlmostEqual(e, ee, places=6)

    def test_offset_from_ref(self):
        n, e = target_point(100.0, -50.0, 90, 10)  # 10 m East of the ref
        self.assertAlmostEqual(n, 100.0, places=6)
        self.assertAlmostEqual(e, -40.0, places=6)

    def test_zero_distance_is_ref(self):
        self.assertEqual(target_point(3.0, 4.0, 123.0, 0.0), (3.0, 4.0))


class TestVelocitySetpoint(unittest.TestCase):
    def test_capped_far_out(self):
        vn, ve, d = horizontal_velocity_setpoint((0, 10), 0, 0, kp=0.5, vmax=2.0)
        self.assertAlmostEqual(d, 10.0)
        self.assertAlmostEqual(math.hypot(vn, ve), 2.0)   # 0.5*10 -> capped at 2
        self.assertAlmostEqual(vn, 0.0)                    # points due East
        self.assertGreater(ve, 0.0)

    def test_tapers_near_target(self):
        vn, ve, d = horizontal_velocity_setpoint((0, 10), 0, 9, kp=0.5, vmax=2.0)
        self.assertAlmostEqual(d, 1.0)
        self.assertAlmostEqual(math.hypot(vn, ve), 0.5)   # 0.5*1 (below cap)

    def test_zero_at_target(self):
        vn, ve, d = horizontal_velocity_setpoint((5, 5), 5, 5, kp=0.5, vmax=2.0)
        self.assertEqual((vn, ve), (0.0, 0.0))
        self.assertLess(d, 1e-3)


# ============================================================================
# PID
# ============================================================================
class TestPID(unittest.TestCase):
    def test_proportional_plus_feedforward(self):
        pid = PID(kp=0.2, ki=0.0, kd=0.0, i_limit=1.0, feedforward=0.34)
        out = pid.update(error=2.0, dt=0.1)
        self.assertAlmostEqual(out, 0.34 + 0.2 * 2.0, places=6)
        self.assertAlmostEqual(pid.p_term, 0.4, places=6)

    def test_derivative_on_measurement(self):
        pid = PID(kp=0.0, ki=0.0, kd=0.1, i_limit=1.0)
        out = pid.update(error=0.0, dt=0.1, meas_rate=5.0)  # d = -kd*vz
        self.assertAlmostEqual(out, -0.5, places=6)
        self.assertAlmostEqual(pid.d_term, -0.5, places=6)

    def test_integral_clamped(self):
        pid = PID(kp=0.0, ki=1.0, kd=0.0, i_limit=0.2)
        for _ in range(10):                       # would reach 1.0 unclamped
            out = pid.update(error=1.0, dt=0.1)
        self.assertAlmostEqual(pid.integral, 0.2, places=6)
        self.assertAlmostEqual(out, 0.2, places=6)

    def test_output_clamped(self):
        pid = PID(kp=1.0, ki=0.0, kd=0.0, i_limit=1.0, out_min=0.0, out_max=0.5)
        self.assertEqual(pid.update(10.0, 0.1), 0.5)
        self.assertEqual(pid.update(-10.0, 0.1), 0.0)

    def test_anti_windup_freezes_at_ceiling(self):
        # Saturated high with positive error -> integral must NOT accumulate.
        pid = PID(kp=0.2, ki=0.09, kd=0.0, i_limit=0.5,
                  out_min=0.0, out_max=0.5, feedforward=0.34)
        for _ in range(20):
            pid.update(error=15.0, dt=0.1)        # thrust pinned at 0.5
        self.assertAlmostEqual(pid.integral, 0.0, places=9)

    def test_anti_windup_releases_on_reversal(self):
        pid = PID(kp=0.2, ki=0.09, kd=0.0, i_limit=0.5,
                  out_min=0.0, out_max=0.5, feedforward=0.34)
        for _ in range(20):
            pid.update(error=15.0, dt=0.1)        # wound-up attempt, frozen at 0
        self.assertAlmostEqual(pid.integral, 0.0, places=9)
        # Small negative error keeps the output off the rail, so the integrator
        # is free again and should start moving negative.
        pid.update(error=-0.5, dt=0.1)
        self.assertLess(pid.integral, 0.0)

    def test_reset(self):
        pid = PID(kp=0.1, ki=0.1, kd=0.1, i_limit=1.0)
        for _ in range(5):
            pid.update(error=1.0, dt=0.1)
        pid.reset()
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(len(pid.hist), 0)


# ============================================================================
# Settings
# ============================================================================
class TestSettings(unittest.TestCase):
    def test_defaults(self):
        cfg = Settings.from_dict({})
        self.assertEqual(cfg.rate_hz, 10.0)
        self.assertEqual(cfg.altitude_pid.kp, 0.22)
        self.assertEqual(len(cfg.phases), 4)

    def test_deep_merge_partial(self):
        cfg = Settings.from_dict({"altitude_pid": {"kp": 0.5}})
        self.assertEqual(cfg.altitude_pid.kp, 0.5)       # overridden
        self.assertEqual(cfg.altitude_pid.ki, 0.09)      # default preserved

    def test_derived_values(self):
        cfg = Settings.from_dict({"control": {"rate_hz": 20},
                                  "attitude": {"tilt_max_deg": 5.0}})
        self.assertAlmostEqual(cfg.period, 0.05, places=9)
        self.assertAlmostEqual(cfg.tilt_max_rad, math.radians(5.0), places=9)

    def test_phase_move_fields(self):
        cfg = Settings.from_dict({"mission": {"phases": [
            {"target_m": 15, "hold_s": 10, "azimuth_deg": 90, "distance_m": 10},
            {"target_m": 5, "hold_s": 3},
        ]}})
        self.assertTrue(cfg.phases[0].has_move)
        self.assertEqual(cfg.phases[0].azimuth_deg, 90.0)
        self.assertEqual(cfg.phases[0].distance_m, 10.0)
        self.assertFalse(cfg.phases[1].has_move)

    def test_validation_rejects_bad_configs(self):
        bad = [
            {"thrust": {"min": 0.6, "max": 0.5}},
            {"altitude_pid": {"kp": -1}},
            {"control": {"rate_hz": 0}},
            {"attitude": {"pitch_sign": 2.0}},
            {"mission": {"phases": []}},
            {"position": {"speed_max_ms": 0}},
            {"mission": {"phases": [{"target_m": 5, "hold_s": 1, "distance_m": -3}]}},
        ]
        for override in bad:
            with self.assertRaises(ConfigError, msg=f"should reject {override}"):
                Settings.from_dict(override)


# ============================================================================
# Integration smoke test: config -> Controller wiring
# ============================================================================
class TestControllerWiring(unittest.TestCase):
    def test_builds_from_config(self):
        cfg = Settings.from_dict({})
        ctrl = Controller(hover=0.34, cfg=cfg)
        self.assertEqual(ctrl.alt.kp, cfg.altitude_pid.kp)
        self.assertEqual(ctrl.alt.ff, 0.34)
        self.assertEqual(ctrl.alt.out_max, cfg.thrust_max)
        self.assertAlmostEqual(ctrl.pitch.out_max, cfg.tilt_max_rad)
        self.assertAlmostEqual(ctrl.roll.out_min, -cfg.tilt_max_rad)

    def test_reset_clears_all_axes(self):
        cfg = Settings.from_dict({})
        ctrl = Controller(hover=0.34, cfg=cfg)
        ctrl.alt.update(5.0, 0.1)
        ctrl.pitch.update(1.0, 0.1)
        ctrl.reset()
        self.assertEqual(ctrl.alt.integral, 0.0)
        self.assertEqual(ctrl.pitch.integral, 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)