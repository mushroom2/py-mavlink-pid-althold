#!/usr/bin/env python3
"""
flight_plot.py -- telemetry logging + plotting for the altitude-hold controller.

Two pieces, deliberately decoupled:

  FlightLog     Pure data collection (no matplotlib import). The control loop
                calls log.add(...) once per cycle. This same object is what a
                real-time FuncAnimation will read later -- nothing else changes.

  plot_flight   Static matplotlib figure: altitude vs target, error, and the
                altitude PID's P / I / D contributions alongside the thrust.

matplotlib is imported lazily inside plot_flight so that importing FlightLog
(in the flight script) never pulls in a GUI backend.
"""

import time


class FlightLog:
    """Parallel time series filled by the control loop, one sample per cycle."""

    FIELDS = ("t", "target", "alt", "err", "thrust", "vz",
              "p", "i", "d", "roll", "pitch", "drift")

    def __init__(self):
        self.t0 = None
        for f in self.FIELDS:
            setattr(self, f, [])

    def add(self, *, target, alt, err, thrust, vz, p, i, d, roll, pitch, drift):
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        self.t.append(now - self.t0)
        self.target.append(target)
        self.alt.append(alt)
        self.err.append(err)
        self.thrust.append(thrust)
        self.vz.append(vz)
        self.p.append(p)
        self.i.append(i)
        self.d.append(d)
        self.roll.append(roll)
        self.pitch.append(pitch)
        self.drift.append(drift)

    def __len__(self):
        return len(self.t)

    def to_csv(self, path):
        import csv
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(self.FIELDS)
            for row in zip(*(getattr(self, f) for f in self.FIELDS)):
                w.writerow(f"{v:.4f}" for v in row)
        print(f"wrote {path} ({len(self)} samples)")


def _mark_phase_changes(ax, t, target):
    """Light vertical lines wherever the target setpoint steps to a new value."""
    last = None
    for ti, tg in zip(t, target):
        if last is None or abs(tg - last) > 1e-6:
            if last is not None:
                ax.axvline(ti, color="0.75", lw=0.8, ls=":", zorder=0)
            last = tg


def plot_flight(log, save_path="flight_plot.png", show=True):
    """Render altitude / error / PID for a completed (or partial) FlightLog."""
    if len(log) == 0:
        print("FlightLog is empty -- nothing to plot")
        return

    import matplotlib.pyplot as plt

    t = log.t
    fig, (ax_alt, ax_err, ax_pid) = plt.subplots(
        3, 1, sharex=True, figsize=(11, 9),
        gridspec_kw={"height_ratios": [3, 2, 3]})

    # --- Altitude vs target ---
    ax_alt.plot(t, log.target, ls="--", lw=1.3, color="tab:orange", label="target")
    ax_alt.plot(t, log.alt, lw=1.6, color="tab:blue", label="altitude")
    _mark_phase_changes(ax_alt, t, log.target)
    ax_alt.set_ylabel("Altitude [m]")
    ax_alt.set_title("Altitude hold — altitude / error / PID")
    ax_alt.legend(loc="lower right")
    ax_alt.grid(alpha=0.3)

    # --- Tracking error ---
    ax_err.axhline(0, color="k", lw=0.7)
    ax_err.plot(t, log.err, lw=1.3, color="tab:red", label="error (target − alt)")
    _mark_phase_changes(ax_err, t, log.target)
    ax_err.set_ylabel("Error [m]")
    ax_err.legend(loc="upper right")
    ax_err.grid(alpha=0.3)

    # --- Altitude PID contributions + resulting thrust ---
    ax_pid.plot(t, log.p, lw=1.1, label="P")
    ax_pid.plot(t, log.i, lw=1.1, label="I")
    ax_pid.plot(t, log.d, lw=1.1, label="D")
    ax_pid.plot(t, log.thrust, lw=1.6, color="k", label="thrust (cmd)")
    _mark_phase_changes(ax_pid, t, log.target)
    ax_pid.set_ylabel("Alt PID / thrust")
    ax_pid.set_xlabel("time [s]")
    ax_pid.legend(loc="upper right", ncol=4)
    ax_pid.grid(alpha=0.3)

    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=120)
        print(f"saved {save_path}")
    if show:
        plt.show()
    return fig


# --- Self-test: fabricate a plausible flight so the layout can be previewed ---
if __name__ == "__main__":
    import math

    log = FlightLog()
    phases = [(14.5, 2), (15.0, 15), (10.0, 15), (1.0, 3)]
    dt = 1 / 20.0
    alt, vz = 0.0, 0.0
    integ = 0.0
    base_t = time.monotonic()
    sim_t = 0.0
    for target, hold in phases:
        held = 0.0
        while held < hold + 2.5:
            err = target - alt
            integ = max(-0.3, min(0.3, integ + 0.045 * err * dt))
            p, i, d = 0.12 * err, integ, -0.1 * vz
            thrust = max(0.0, min(0.5, 0.42 + p + i + d))
            acc = (thrust - 0.42) * 40.0 - vz * 1.2   # crude 2nd-order response
            vz += acc * dt
            alt += vz * dt
            # fake the monotonic clock so timestamps look real
            log.t0 = base_t
            log.t.append(sim_t)
            for name, val in (("target", target), ("alt", alt), ("err", err),
                              ("thrust", thrust), ("vz", vz), ("p", p), ("i", i),
                              ("d", d), ("roll", 0.4 * math.sin(sim_t)),
                              ("pitch", 0.3 * math.cos(sim_t)),
                              ("drift", abs(0.2 * math.sin(sim_t)))):
                getattr(log, name).append(val)
            sim_t += dt
            if abs(err) < 0.1:
                held += dt
    plot_flight(log, save_path="flight_plot_demo.png", show=False)