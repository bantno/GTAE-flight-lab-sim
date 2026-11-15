"""Fixed-step real-time loop, wall-clock paced, with timing statistics.

What the Simulink model did
---------------------------
AE4803Sim uses a VARIABLE-step solver (ode23, RelTol 1e-2, MaxStep 0.04) with
Simulink's simulation pacing switched on at 1x (EnablePacing = on,
PacingRate = 1). Pacing only ever *slows the sim down* to the wall clock; if a
step takes too long there is no catch-up, no overrun count, and no way to see
how badly it slipped. The 'Real-Time Synchronization' block in the X-Plane
subsystem is present but commented out.

What this does instead
----------------------
A fixed step (default 200 Hz) integrated with RK4, scheduled against
time.perf_counter(), with:

  * every step's execution time and lateness recorded,
  * overruns counted and reported rather than silently absorbed,
  * a bounded catch-up policy so a transient hiccup does not accumulate into
    permanent lag, and a resync (with a count) if it ever falls far behind,
  * the physics on its own thread, so drawing cannot stall it.

Everything here is available to a normal user account. Windows tweaks used:
  * timeBeginPeriod(1)   - raises the system timer resolution (winmm).
  * HIGH_PRIORITY_CLASS  - raises this process's priority (kernel32).
REALTIME_PRIORITY_CLASS needs admin, is not requested, and is not required.
"""
from __future__ import annotations

import ctypes
import threading
import time
from dataclasses import dataclass, field

import numpy as np

from . import tuning
from .controls import PilotCommands


# --- Windows timing helpers (all per-user, no admin) ----------------------
class WindowsTiming:
    """Raises timer resolution and process priority; restores on exit."""

    HIGH_PRIORITY_CLASS = 0x00000080
    ABOVE_NORMAL_PRIORITY_CLASS = 0x00008000

    def __init__(self, timer_ms: int = 1, high_priority: bool = True):
        self.timer_ms = int(timer_ms)
        self.high_priority = high_priority
        self._timer_set = False
        self._prev_priority: int | None = None
        self.notes: list[str] = []

    def __enter__(self) -> "WindowsTiming":
        try:
            winmm = ctypes.WinDLL("winmm")
            if winmm.timeBeginPeriod(self.timer_ms) == 0:
                self._timer_set = True
                self.notes.append(f"timer resolution set to {self.timer_ms} ms")
            else:
                self.notes.append("timeBeginPeriod failed (continuing)")
        except Exception as exc:                              # pragma: no cover
            self.notes.append(f"timeBeginPeriod unavailable: {exc}")

        if self.high_priority:
            try:
                k32 = ctypes.WinDLL("kernel32")
                h = k32.GetCurrentProcess()
                self._prev_priority = k32.GetPriorityClass(h)
                if k32.SetPriorityClass(h, self.HIGH_PRIORITY_CLASS):
                    self.notes.append("process priority: HIGH")
                elif k32.SetPriorityClass(h, self.ABOVE_NORMAL_PRIORITY_CLASS):
                    self.notes.append("process priority: ABOVE_NORMAL")
                else:
                    self.notes.append("could not raise process priority "
                                      "(continuing at NORMAL)")
            except Exception as exc:                          # pragma: no cover
                self.notes.append(f"priority change unavailable: {exc}")
        return self

    def __exit__(self, *exc) -> None:
        if self._timer_set:
            try:
                ctypes.WinDLL("winmm").timeEndPeriod(self.timer_ms)
            except Exception:
                pass
        if self._prev_priority:
            try:
                k32 = ctypes.WinDLL("kernel32")
                k32.SetPriorityClass(k32.GetCurrentProcess(), self._prev_priority)
            except Exception:
                pass


# --- timing statistics ----------------------------------------------------
@dataclass
class TimingStats:
    rate_hz: float
    step_us: list[float] = field(default_factory=list)
    late_us: list[float] = field(default_factory=list)
    steps: int = 0
    overruns: int = 0
    resyncs: int = 0
    wall_start: float = 0.0
    wall_end: float = 0.0
    sim_time: float = 0.0
    max_catchup_steps: int = 0

    def record(self, step_s: float, late_s: float) -> None:
        self.steps += 1
        self.step_us.append(step_s * 1e6)
        self.late_us.append(late_s * 1e6)
        if step_s > 1.0 / self.rate_hz:
            self.overruns += 1

    @property
    def wall_time(self) -> float:
        return max(self.wall_end - self.wall_start, 1e-12)

    @property
    def real_time_factor(self) -> float:
        return self.sim_time / self.wall_time

    def summary(self) -> dict:
        s = np.asarray(self.step_us) if self.step_us else np.zeros(1)
        l = np.asarray(self.late_us) if self.late_us else np.zeros(1)
        h_us = 1e6 / self.rate_hz
        return {
            "rate_hz": self.rate_hz,
            "step_budget_us": h_us,
            "steps": self.steps,
            "sim_time_s": self.sim_time,
            "wall_time_s": self.wall_time,
            "real_time_factor": self.real_time_factor,
            "overruns": self.overruns,
            "overrun_pct": 100.0 * self.overruns / max(self.steps, 1),
            "resyncs": self.resyncs,
            "max_catchup_steps": self.max_catchup_steps,
            "step_us": {
                "mean": float(s.mean()), "p50": float(np.percentile(s, 50)),
                "p95": float(np.percentile(s, 95)), "p99": float(np.percentile(s, 99)),
                "max": float(s.max()),
            },
            "late_us": {
                "mean": float(l.mean()), "p50": float(np.percentile(l, 50)),
                "p95": float(np.percentile(l, 95)), "p99": float(np.percentile(l, 99)),
                "max": float(l.max()),
            },
            "cpu_load_pct": 100.0 * float(s.mean()) / h_us,
        }

    def format(self) -> str:
        d = self.summary()
        return "\n".join([
            "--- real-time timing ---",
            f"  fixed step      : {d['rate_hz']:.1f} Hz  ({d['step_budget_us']:.0f} us budget)",
            f"  steps           : {d['steps']}",
            f"  sim / wall time : {d['sim_time_s']:.2f} s / {d['wall_time_s']:.2f} s"
            f"   (real-time factor {d['real_time_factor']:.5f})",
            f"  step time   [us]: mean {d['step_us']['mean']:7.1f}   p50 {d['step_us']['p50']:7.1f}"
            f"   p95 {d['step_us']['p95']:7.1f}   p99 {d['step_us']['p99']:7.1f}"
            f"   max {d['step_us']['max']:8.1f}",
            f"  lateness    [us]: mean {d['late_us']['mean']:7.1f}   p50 {d['late_us']['p50']:7.1f}"
            f"   p95 {d['late_us']['p95']:7.1f}   p99 {d['late_us']['p99']:7.1f}"
            f"   max {d['late_us']['max']:8.1f}",
            f"  overruns        : {d['overruns']} ({d['overrun_pct']:.3f}% of steps)",
            f"  resyncs         : {d['resyncs']}   max catch-up {d['max_catchup_steps']} steps",
            f"  CPU per step    : {d['cpu_load_pct']:.1f}% of the step budget",
        ])


def _sleep_until(deadline: float, spin_margin: float = 0.0012) -> None:
    """Sleep most of the way, then spin - accurate to a few microseconds."""
    while True:
        remaining = deadline - time.perf_counter()
        if remaining <= 0.0:
            return
        if remaining > spin_margin:
            time.sleep(remaining - spin_margin)
        else:
            while time.perf_counter() < deadline:
                pass
            return


@dataclass
class LoopConfig:
    rate_hz: float = 200.0
    #: How many steps of accumulated lag to work off before giving up and
    #: resyncing the clock. Keeps a transient hiccup from becoming permanent lag.
    max_catchup_steps: int = 5
    high_priority: bool = True
    timer_resolution_ms: int = 1
    spin_margin_s: float = 0.0012


class RealTimeLoop:
    """Runs a FlightModel at a fixed rate on its own thread."""

    def __init__(self, model, input_source, cfg: LoopConfig | None = None,
                 logger=None, xplane=None, on_step=None, on_stop=None,
                 on_poll=None):
        self.model = model
        self.input_source = input_source
        self.cfg = cfg or LoopConfig()
        self.logger = logger
        self.xplane = xplane
        #: called once per completed step, with the Outputs
        self.on_step = on_step
        #: called once per loop iteration with the loop, EVEN WHILE PAUSED -
        #: this is where incoming commands must be handled
        self.on_poll = on_poll
        self.on_stop = on_stop

        self.stats = TimingStats(rate_hz=self.cfg.rate_hz)
        #: every in-flight parameter change, for the flight log
        self.tuning_log = tuning.TuningLog()
        self.state = model.reset()
        self.sim_time = 0.0
        self.paused = False
        self.stop_reason = ""
        #: Auto-stop FREEZES the sim rather than tearing the loop down, so the
        #: loop keeps servicing commands and Restart still works. Tearing down
        #: on auto-stop left nothing alive to receive a restart - the button
        #: did nothing after a stall, which is exactly when you want it.
        self.halted = False
        self.halt_reason = ""

        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._snapshot = None
        self._pilot = PilotCommands()

    # --- control ----------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="ae4803-physics",
                                        daemon=True)
        self._thread.start()

    def halt(self, reason: str) -> None:
        """Freeze the sim but keep the loop alive and responsive."""
        self.halted = True
        self.halt_reason = reason

    def stop(self, reason: str = "stopped by user") -> None:
        if not self.stop_reason:
            self.stop_reason = reason
        self._stop.set()

    def join(self, timeout: float | None = None) -> None:
        if self._thread is not None:
            self._thread.join(timeout)

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def snapshot(self):
        """Most recent Outputs, safe to read from another thread."""
        with self._lock:
            return self._snapshot

    # --- uniform interface shared with procsim.PhysicsProcess -------------
    def set_wing_leveler(self, v: bool) -> None:
        self.model.ap.wing_leveler_on = bool(v)

    def set_altitude_hold(self, v: bool) -> None:
        self.model.ap.altitude_hold_on = bool(v)

    def set_autostop(self, v: bool) -> None:
        self.model.autostop_cfg.enabled = bool(v)

    def restart(self) -> None:
        """Re-fly from the initial condition without tearing anything down.

        Keeps any derivative edits - the usual reason to restart is to fly the
        SAME modified aircraft again from a known start. The tuning log is
        rebuilt so every non-default value is re-stamped at t = 0, otherwise the
        log would carry timestamps from before the restart and be unreadable.
        Accumulated flight-log samples and timing statistics are discarded,
        since they describe a different flight.
        """
        changed = {e["key"]: e["new"] for e in self.tuning_log.entries}
        defaults = {e["key"]: e["old"] for e in reversed(self.tuning_log.entries)}

        self.state = self.model.reset()
        self.sim_time = 0.0
        self.stop_reason = ""
        self.halted = False
        self.halt_reason = ""
        self.stats = TimingStats(rate_hz=self.cfg.rate_hz)
        self.stats.wall_start = time.perf_counter()

        self.tuning_log = tuning.TuningLog()
        for key, new in changed.items():
            if abs(new - defaults.get(key, new)) > 1e-12:
                self.tuning_log.record(0.0, key, defaults[key], new)

        if self.logger is not None:
            self.logger.reset()
        with self._lock:
            self._snapshot = None

    def set_parameter(self, key: str, value: float) -> float:
        """Apply a model parameter change as a step, and record it.

        Safe to call at any time, running or paused: the model re-reads every
        coefficient on each derivative evaluation, so the new value is live on
        the next RK4 stage.
        """
        old = tuning.apply(self.model.p, key, float(value))
        self.tuning_log.record(self.sim_time, key, old, float(value))
        return old

    @property
    def initial_flags(self) -> dict:
        return {"wing_leveler": self.model.ap.wing_leveler_on,
                "altitude_hold": self.model.ap.altitude_hold_on,
                "autostop": self.model.autostop_cfg.enabled}

    def status(self) -> dict:
        st = self.stats
        if not st.steps:
            return {"steps": 0, "overruns": 0, "late_p99_us": 0.0,
                    "real_time_factor": 0.0}
        elapsed = max(time.perf_counter() - st.wall_start, 1e-9) \
            if not st.wall_end else st.wall_time
        return {
            "steps": st.steps,
            "overruns": st.overruns,
            "late_p99_us": float(np.percentile(st.late_us[-2000:], 99)),
            "real_time_factor": self.sim_time / elapsed,
        }

    # --- the loop ---------------------------------------------------------
    def _run(self) -> None:
        h = 1.0 / self.cfg.rate_hz
        with WindowsTiming(self.cfg.timer_resolution_ms,
                           self.cfg.high_priority) as timing:
            self.timing_notes = timing.notes
            t0 = time.perf_counter()
            self.stats.wall_start = t0
            next_deadline = t0

            while not self._stop.is_set():
                _sleep_until(next_deadline, self.cfg.spin_margin_s)
                begin = time.perf_counter()
                lateness = begin - next_deadline

                # Drained EVERY iteration, including while paused. If this only
                # ran with the step, a paused loop could never see the command
                # that resumes it - pause would be a one-way trip.
                if self.on_poll is not None:
                    self.on_poll(self)

                if self.paused or self.halted:
                    next_deadline = begin + h
                    continue

                try:
                    pilot = self._read_input()
                except Exception as exc:                      # pragma: no cover
                    pilot = self._pilot
                    self.stop(f"input error: {exc}")

                self.state = self.model.step(self.sim_time, self.state, h, pilot)
                self.sim_time += h

                out = self.model.out
                with self._lock:
                    self._snapshot = out

                if self.logger is not None:
                    self.logger.record(out)
                if self.xplane is not None:
                    self.xplane.send(out, begin)
                if self.on_step is not None:
                    self.on_step(out)

                if out.autostop and not self.halted:
                    self.halt(f"auto-stop: {out.autostop_reason}")

                end = time.perf_counter()
                self.stats.record(end - begin, lateness)
                self.stats.sim_time = self.sim_time

                # --- schedule the next step, with bounded catch-up ---
                next_deadline += h
                behind = end - next_deadline
                if behind > 0:
                    missed = int(behind / h) + 1
                    self.stats.max_catchup_steps = max(self.stats.max_catchup_steps,
                                                       missed)
                    if missed > self.cfg.max_catchup_steps:
                        next_deadline = end
                        self.stats.resyncs += 1

            self.stats.wall_end = time.perf_counter()

        if self.on_stop is not None:
            self.on_stop(self)

    def _read_input(self) -> PilotCommands:
        src = self.input_source
        if hasattr(src, "set_time"):
            src.set_time(self.sim_time)
        self._pilot = src.poll()
        return self._pilot
