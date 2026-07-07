#!/usr/bin/env python3
"""
settings.py -- configuration loading & validation for the altitude-hold app.

All config business logic lives here. The flight code does:

    from settings import Settings
    cfg = Settings.load("config.yaml")
    ...
    cfg.altitude_pid.kp, cfg.thrust.max, cfg.tilt_max_rad, cfg.phases, ...

Design:
  * YAML holds human-friendly values (degrees, m/s, Hz).
  * User YAML is deep-merged over built-in DEFAULTS, so a partial file works.
  * Everything is validated on load with clear error messages -- a bad config
    fails fast at startup instead of mid-flight.
  * Derived values (radians, loop period) are computed here, not in the code.
"""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass, field
from typing import Any

import yaml


# ----------------------------------------------------------------------------
# Built-in defaults (any key omitted from config.yaml falls back to these)
# ----------------------------------------------------------------------------
DEFAULTS: dict[str, Any] = {
    "connection": {"address": "udp:127.0.0.1:14550"},
    "control": {
        "rate_hz": 10.0,
        "reach_tol_m": 0.1,
        "climb_rate_max_ms": 2.5,
        "stream_timeout_s": 0.3,
        "stale_warn_s": 0.5,
    },
    "thrust": {"hover_default": 0.5, "min": 0.0, "max": 0.5},
    "altitude_pid": {"kp": 0.22, "ki": 0.09, "kd": 0.22, "i_limit": 0.20, "d_window": 10},
    "pitch_pid": {"kp": 0.04, "ki": 0.01, "kd": 0.005, "i_limit": 0.05, "d_window": 10},
    "roll_pid": {"kp": 0.04, "ki": 0.01, "kd": 0.005, "i_limit": 0.05, "d_window": 10},
    "attitude": {"tilt_max_deg": 5.0, "pitch_sign": -1.0, "roll_sign": 1.0},
    "ardupilot_params": {"GUID_OPTIONS": 8, "FRAME_CLASS": 1, "FRAME_TYPE": 1},
    "mission": {"phases": [
        {"target_m": 14.5, "hold_s": 2},
        {"target_m": 15.0, "hold_s": 15},
        {"target_m": 10.0, "hold_s": 15},
        {"target_m": 1.0,  "hold_s": 2},
    ]},
    "logging": {"csv_path": "flight_log.csv", "plot_path": "flight_plot.png",
                "show_plot": True},
}


class ConfigError(ValueError):
    """Raised when config.yaml is malformed or out of range."""


def _deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge override into a copy of base."""
    out = copy.deepcopy(base)
    for k, v in (override or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


# ----------------------------------------------------------------------------
# Structured, typed views of the config
# ----------------------------------------------------------------------------
@dataclass(frozen=True)
class PIDGains:
    kp: float
    ki: float
    kd: float
    i_limit: float
    d_window: int

    @classmethod
    def from_dict(cls, name: str, d: dict) -> "PIDGains":
        try:
            g = cls(kp=float(d["kp"]), ki=float(d["ki"]), kd=float(d["kd"]),
                    i_limit=float(d["i_limit"]), d_window=int(d["d_window"]))
        except (KeyError, TypeError, ValueError) as e:
            raise ConfigError(f"{name}: invalid PID gains ({e})") from e
        for f in ("kp", "ki", "kd", "i_limit"):
            if getattr(g, f) < 0:
                raise ConfigError(f"{name}.{f} must be >= 0 (got {getattr(g, f)})")
        if g.d_window < 1:
            raise ConfigError(f"{name}.d_window must be >= 1 (got {g.d_window})")
        return g


@dataclass(frozen=True)
class Phase:
    target_m: float
    hold_s: float


@dataclass(frozen=True)
class Settings:
    # connection
    connection_address: str
    # control
    rate_hz: float
    reach_tol_m: float
    climb_rate_max_ms: float
    stream_timeout_s: float
    stale_warn_s: float
    # thrust
    hover_default: float
    thrust_min: float
    thrust_max: float
    # PIDs
    altitude_pid: PIDGains
    pitch_pid: PIDGains
    roll_pid: PIDGains
    # attitude
    tilt_max_rad: float
    pitch_sign: float
    roll_sign: float
    # startup params + mission + logging
    ardupilot_params: dict
    phases: tuple
    csv_path: str
    plot_path: str
    show_plot: bool

    # ---- derived values ----
    @property
    def period(self) -> float:
        return 1.0 / self.rate_hz

    @property
    def tilt_max_deg(self) -> float:
        return math.degrees(self.tilt_max_rad)

    # ---- construction ----
    @classmethod
    def load(cls, path: str = "config.yaml") -> "Settings":
        try:
            with open(path, "r") as fh:
                raw = yaml.safe_load(fh) or {}
        except FileNotFoundError as e:
            raise ConfigError(f"config file not found: {path}") from e
        except yaml.YAMLError as e:
            raise ConfigError(f"could not parse YAML in {path}: {e}") from e
        if not isinstance(raw, dict):
            raise ConfigError(f"{path}: top level must be a mapping")
        return cls.from_dict(raw)

    @classmethod
    def from_dict(cls, raw: dict) -> "Settings":
        c = _deep_merge(DEFAULTS, raw)

        ctrl, thr, att = c["control"], c["thrust"], c["attitude"]

        phases = tuple(
            Phase(target_m=float(p["target_m"]), hold_s=float(p["hold_s"]))
            for p in c["mission"]["phases"]
        )

        s = cls(
            connection_address=str(c["connection"]["address"]),
            rate_hz=float(ctrl["rate_hz"]),
            reach_tol_m=float(ctrl["reach_tol_m"]),
            climb_rate_max_ms=float(ctrl["climb_rate_max_ms"]),
            stream_timeout_s=float(ctrl["stream_timeout_s"]),
            stale_warn_s=float(ctrl["stale_warn_s"]),
            hover_default=float(thr["hover_default"]),
            thrust_min=float(thr["min"]),
            thrust_max=float(thr["max"]),
            altitude_pid=PIDGains.from_dict("altitude_pid", c["altitude_pid"]),
            pitch_pid=PIDGains.from_dict("pitch_pid", c["pitch_pid"]),
            roll_pid=PIDGains.from_dict("roll_pid", c["roll_pid"]),
            tilt_max_rad=math.radians(float(att["tilt_max_deg"])),
            pitch_sign=float(att["pitch_sign"]),
            roll_sign=float(att["roll_sign"]),
            ardupilot_params=dict(c["ardupilot_params"]),
            phases=phases,
            csv_path=str(c["logging"]["csv_path"]),
            plot_path=str(c["logging"]["plot_path"]),
            show_plot=bool(c["logging"]["show_plot"]),
        )
        s.validate()
        return s

    # ---- validation ----
    def validate(self) -> None:
        if self.rate_hz <= 0:
            raise ConfigError(f"control.rate_hz must be > 0 (got {self.rate_hz})")
        if self.climb_rate_max_ms <= 0:
            raise ConfigError("control.climb_rate_max_ms must be > 0")
        if self.reach_tol_m <= 0:
            raise ConfigError("control.reach_tol_m must be > 0")
        if not (0.0 <= self.thrust_min < self.thrust_max <= 1.0):
            raise ConfigError(
                f"thrust limits must satisfy 0 <= min < max <= 1 "
                f"(got min={self.thrust_min}, max={self.thrust_max})")
        if not (self.thrust_min <= self.hover_default <= self.thrust_max):
            raise ConfigError(
                f"thrust.hover_default {self.hover_default} must lie within "
                f"[min, max] = [{self.thrust_min}, {self.thrust_max}]")
        if not (0 < self.tilt_max_rad < math.radians(45)):
            raise ConfigError("attitude.tilt_max_deg must be in (0, 45)")
        for name, sign in (("pitch_sign", self.pitch_sign), ("roll_sign", self.roll_sign)):
            if sign not in (-1.0, 1.0):
                raise ConfigError(f"attitude.{name} must be +1.0 or -1.0 (got {sign})")
        if not self.phases:
            raise ConfigError("mission.phases must contain at least one phase")
        for i, p in enumerate(self.phases):
            if p.target_m < 0:
                raise ConfigError(f"mission.phases[{i}].target_m must be >= 0")
            if p.hold_s < 0:
                raise ConfigError(f"mission.phases[{i}].hold_s must be >= 0")

    # ---- pretty startup banner ----
    def summary(self) -> str:
        ph = " -> ".join(f"{p.target_m:g}m/{p.hold_s:g}s" for p in self.phases)
        a = self.altitude_pid
        return (
            "Config:\n"
            f"  link          {self.connection_address}\n"
            f"  rate          {self.rate_hz:g} Hz  (period {self.period*1000:.1f} ms)\n"
            f"  alt PID       kp={a.kp} ki={a.ki} kd={a.kd} ilim={a.i_limit}\n"
            f"  thrust        [{self.thrust_min}, {self.thrust_max}]  hover_ff={self.hover_default}\n"
            f"  climb slew    {self.climb_rate_max_ms} m/s\n"
            f"  tilt max      {self.tilt_max_deg:g} deg  (pitch_sign={self.pitch_sign}, roll_sign={self.roll_sign})\n"
            f"  mission       {ph}\n"
        )


if __name__ == "__main__":
    # Self-test: load the real config and exercise validation on bad inputs.
    cfg = Settings.load("config.yaml")
    print(cfg.summary())
    print("phases:", cfg.phases)
    print("ardupilot_params:", cfg.ardupilot_params)

    print("\n-- validation checks --")
    bad_cases = [
        ("thrust min>=max", {"thrust": {"min": 0.6, "max": 0.5}}),
        ("negative kp",     {"altitude_pid": {"kp": -1}}),
        ("zero rate",       {"control": {"rate_hz": 0}}),
        ("bad sign",        {"attitude": {"pitch_sign": 2.0}}),
        ("no phases",       {"mission": {"phases": []}}),
    ]
    for label, override in bad_cases:
        try:
            Settings.from_dict(override)
            print(f"  {label:18s}: NO ERROR (unexpected!)")
        except ConfigError as e:
            print(f"  {label:18s}: rejected -> {e}")