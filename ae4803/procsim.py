"""Run the physics in a separate PROCESS, so drawing cannot stall it.

Running the integrator on a background *thread* still lets the GUI interfere:
CPython's GIL means every Python callback in the Qt event loop - repainting a
strip chart, formatting a readout - blocks the physics thread for its duration.
Measured on this machine at 200 Hz: ~0.7 % of steps overran and the worst
lateness was ~13 ms, i.e. two and a half steps.

With the physics in its own process there is no shared interpreter lock, so the
GUI cannot delay it at all. Communication is deliberately minimal:

  * snapshot : a shared array of doubles plus a sequence counter (a seqlock -
               the writer bumps it before and after writing, the reader retries
               if it saw a write in progress). No locks on the physics side.
  * commands : a small Queue, polled once per step (cheap when empty).
  * results  : a Queue the child uses to hand back timing statistics and the
               saved log path when it stops.

The child owns the joystick and the UDP socket, so neither touches the GUI
process either.
"""
from __future__ import annotations

import multiprocessing as mp
import queue
import time
from dataclasses import dataclass

import numpy as np

#: Scalar fields mirrored into shared memory, in order.
FIELDS: tuple[str, ...] = (
    "t", "phi_deg", "theta_deg", "psi_deg",
    "u", "v", "w", "U", "U_kts",
    "p_dps", "q_dps", "r_dps",
    "alpha_deg", "beta_deg",
    "x", "y", "alt_m", "alt_ft", "hdot_ft_min", "hdot_mps",
    "lat", "lon", "rho", "qbar", "g",
    "elevator_deg", "aileron_deg", "rudder_deg", "flap_deg",
    "rpm", "manifold_pressure", "dpt", "power_kW",
    "cmd_roll", "cmd_pitch", "cmd_yaw", "cmd_throttle",
    "stick_lat", "stick_long", "pedal", "throttle_in",
    "Ve_0", "Ve_1", "Ve_2",
)
#: Loop status appended after the signal fields.
STATUS: tuple[str, ...] = (
    "running", "paused", "steps", "overruns", "late_p99_us",
    "real_time_factor", "sim_time", "autostop", "joystick_warning",
    "param_changes", "restarts",
)
ALL_FIELDS = FIELDS + STATUS
IDX = {name: i for i, name in enumerate(ALL_FIELDS)}
N_FIELDS = len(ALL_FIELDS)


class Snapshot:
    """Read-only view with the same attribute names as model.Outputs."""
    __slots__ = ("_v", "autostop_reason")

    def __init__(self, values: np.ndarray, autostop_reason: str = ""):
        self._v = values
        self.autostop_reason = autostop_reason

    def __getattr__(self, name):
        i = IDX.get(name)
        if i is None:
            raise AttributeError(name)
        return float(self._v[i])

    @property
    def Ve(self) -> np.ndarray:
        return np.array([self._v[IDX["Ve_0"]], self._v[IDX["Ve_1"]],
                         self._v[IDX["Ve_2"]]])

    @property
    def autostop(self) -> bool:
        return bool(self._v[IDX["autostop"]])


@dataclass
class ChildResult:
    stats: dict
    stats_text: str
    stop_reason: str
    log_path: str | None
    xplane_packets: int
    xplane_errors: int
    joystick_warnings: list
    timing_notes: list


def _child_main(cfg: dict, args_dict: dict, shared, seq, cmd_q, res_q) -> None:
    """Entry point for the physics process. Imports happen here so the parent
    never loads pygame/PyQt in the same interpreter as the integrator."""
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

    from types import SimpleNamespace

    from ae4803.realtime import RealTimeLoop, LoopConfig
    from run_sim import build                         # reuse the same wiring

    args = SimpleNamespace(**args_dict)
    model, source, logger, sender, _ = build(cfg, args)

    buf = np.frombuffer(shared.get_obj(), dtype=np.float64)

    def publish(out, loop_ref):
        seq.value += 1                       # odd => write in progress
        for i, name in enumerate(FIELDS):
            if name.startswith("Ve_"):
                buf[i] = out.Ve[int(name[-1])]
            else:
                buf[i] = getattr(out, name)
        st = loop_ref.stats
        buf[IDX["running"]] = 1.0
        buf[IDX["paused"]] = 1.0 if loop_ref.paused else 0.0
        buf[IDX["steps"]] = st.steps
        buf[IDX["overruns"]] = st.overruns
        buf[IDX["sim_time"]] = loop_ref.sim_time
        buf[IDX["autostop"]] = 1.0 if out.autostop else 0.0
        buf[IDX["joystick_warning"]] = float(
            len(getattr(source, "watchdog", None).warnings)
            if getattr(source, "watchdog", None) else 0)
        if st.steps and st.steps % 25 == 0 and st.late_us:
            buf[IDX["late_p99_us"]] = float(np.percentile(st.late_us[-2000:], 99))
            elapsed = max(time.perf_counter() - st.wall_start, 1e-9)
            buf[IDX["real_time_factor"]] = loop_ref.sim_time / elapsed
        seq.value += 1                       # even => consistent

    loop = RealTimeLoop(
        model, source,
        LoopConfig(rate_hz=float(args.rate or cfg.get("solver", {}).get("rate_hz", 200.0)),
                   max_catchup_steps=int(cfg.get("solver", {}).get("max_catchup_steps", 5)),
                   high_priority=bool(cfg.get("solver", {}).get("high_priority", True)),
                   timer_resolution_ms=int(cfg.get("solver", {}).get("timer_resolution_ms", 1))),
        logger=logger, xplane=sender)

    def drain_commands(loop_ref):
        """Runs every loop iteration, including while paused."""
        try:
            while True:
                cmd, val = cmd_q.get_nowait()
                if cmd == "wing_leveler":
                    model.ap.wing_leveler_on = bool(val)
                elif cmd == "altitude_hold":
                    model.ap.altitude_hold_on = bool(val)
                elif cmd == "autostop":
                    model.autostop_cfg.enabled = bool(val)
                elif cmd == "pause":
                    loop_ref.paused = bool(val)
                    buf[IDX["paused"]] = 1.0 if val else 0.0
                elif cmd == "set_param":
                    key, value = val
                    try:
                        loop_ref.set_parameter(key, value)
                        buf[IDX["param_changes"]] = len(loop_ref.tuning_log)
                    except Exception as exc:
                        print(f"set_param {key} failed: {exc}", flush=True)
                elif cmd == "restart":
                    loop_ref.restart()
                    buf[IDX["param_changes"]] = len(loop_ref.tuning_log)
                    buf[IDX["restarts"]] = buf[IDX["restarts"]] + 1.0
                elif cmd == "stop":
                    loop_ref.stop(str(val))
        except queue.Empty:
            pass

    loop.on_step = lambda out: publish(out, loop)
    loop.on_poll = drain_commands
    loop.start()

    duration = args_dict.get("duration")
    try:
        while loop.running:
            if duration and loop.sim_time >= duration:
                loop.stop(f"reached --duration {duration} s")
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        loop.stop("interrupted with Ctrl-C")
    loop.join(3.0)

    buf[IDX["running"]] = 0.0

    log_path = None
    if logger is not None and logger.n_samples:
        logger.tuning_log = loop.tuning_log
        log_path = str(logger.save(metadata={
            "rate_hz": loop.cfg.rate_hz,
            "stopReason": loop.stop_reason or "",
            "inertiaMode": cfg.get("model", {}).get("inertia_mode", "simulink"),
            "paramChangeCount": float(len(loop.tuning_log)),
        }))
    warnings = list(getattr(source, "watchdog", None).warnings) \
        if getattr(source, "watchdog", None) else []
    packets = sender.packets_sent if sender else 0
    errors = sender.errors if sender else 0
    if sender:
        sender.close()
    source.close()

    res_q.put(ChildResult(
        stats=loop.stats.summary() if loop.stats.steps else {},
        stats_text=loop.stats.format() if loop.stats.steps else "(no steps)",
        stop_reason=loop.stop_reason,
        log_path=log_path,
        xplane_packets=packets,
        xplane_errors=errors,
        joystick_warnings=warnings,
        timing_notes=list(getattr(loop, "timing_notes", [])),
    ))


class PhysicsProcess:
    """Parent-side handle. Mimics just enough of RealTimeLoop for the GUI."""

    def __init__(self, cfg: dict, args):
        ctx = mp.get_context("spawn")
        self._shared = ctx.Array("d", N_FIELDS, lock=True)
        self._seq = ctx.Value("l", 0, lock=False)
        self._cmd_q = ctx.Queue()
        self._res_q = ctx.Queue()
        self._view = np.zeros(N_FIELDS)
        self.result: ChildResult | None = None
        self.stop_reason = ""
        self._paused = False
        self.cfg = cfg

        args_dict = {k: v for k, v in vars(args).items()}
        args_dict.pop("config", None)
        self._proc = ctx.Process(
            target=_child_main,
            args=(cfg, args_dict, self._shared, self._seq, self._cmd_q, self._res_q),
            name="ae4803-physics", daemon=False)

    # --- lifecycle --------------------------------------------------------
    def start(self) -> None:
        self._proc.start()

    @property
    def running(self) -> bool:
        return self._proc.is_alive() and bool(self._view[IDX["running"]])

    def stop(self, reason: str = "stopped") -> None:
        if not self.stop_reason:
            self.stop_reason = reason
        try:
            self._cmd_q.put_nowait(("stop", reason))
        except Exception:
            pass

    def join(self, timeout: float = 5.0) -> None:
        self._proc.join(timeout)
        try:
            self.result = self._res_q.get_nowait()
        except Exception:
            pass
        if self._proc.is_alive():
            self._proc.terminate()

    # --- data -------------------------------------------------------------
    def snapshot(self) -> Snapshot | None:
        """Seqlock read: retry while the writer is mid-update."""
        for _ in range(4):
            s0 = self._seq.value
            if s0 & 1:
                continue
            self._view[:] = np.frombuffer(self._shared.get_obj(), dtype=np.float64)
            if self._seq.value == s0:
                break
        if self._view[IDX["steps"]] <= 0:
            return None
        return Snapshot(self._view)

    @property
    def sim_time(self) -> float:
        return float(self._view[IDX["sim_time"]])

    @property
    def paused(self) -> bool:
        return self._paused

    @paused.setter
    def paused(self, v: bool) -> None:
        self._paused = bool(v)
        self._cmd_q.put(("pause", bool(v)))

    def set_wing_leveler(self, v: bool) -> None:
        self._cmd_q.put(("wing_leveler", bool(v)))

    def set_altitude_hold(self, v: bool) -> None:
        self._cmd_q.put(("altitude_hold", bool(v)))

    def set_autostop(self, v: bool) -> None:
        self._cmd_q.put(("autostop", bool(v)))

    def set_parameter(self, key: str, value: float) -> None:
        """Change a model parameter in the physics process, as a step."""
        self._cmd_q.put(("set_param", (str(key), float(value))))

    def restart(self) -> None:
        """Re-fly from the initial condition, keeping any derivative edits."""
        self._cmd_q.put(("restart", True))

    @property
    def restarts(self) -> int:
        return int(self._view[IDX["restarts"]])

    @property
    def param_changes(self) -> int:
        return int(self._view[IDX["param_changes"]])

    @property
    def initial_flags(self) -> dict:
        ap = self.cfg.get("autopilot", {})
        return {"wing_leveler": bool(ap.get("wing_leveler", False)),
                "altitude_hold": bool(ap.get("altitude_hold", False)),
                "autostop": bool(self.cfg.get("autostop", {}).get("enabled", True))}

    def status(self) -> dict:
        v = self._view
        return {"steps": int(v[IDX["steps"]]),
                "overruns": int(v[IDX["overruns"]]),
                "late_p99_us": float(v[IDX["late_p99_us"]]),
                "real_time_factor": float(v[IDX["real_time_factor"]])}
