#!/usr/bin/env python
"""AE4803 Flight Simulator - launcher.

    python run_sim.py                 # gauges + joystick + X-Plane output
    python run_sim.py --no-gui        # headless, prints timing on exit
    python run_sim.py --no-xplane     # don't send UDP
    python run_sim.py --scripted      # fly the built-in doublet profile
    python run_sim.py --duration 120  # stop after 120 s of sim time
    python run_sim.py --devices       # list joysticks and exit

TARGET must be closed before starting: while it runs it hides the physical
Thrustmaster devices from SDL.
"""
from __future__ import annotations

import argparse
import sys
import tomllib
from pathlib import Path

REPO = Path(__file__).resolve().parent
sys.path.insert(0, str(REPO))

from ae4803 import (AutopilotConfig, AutoStopConfig, FlightModel,  # noqa: E402
                    Params)
from ae4803 import replay                                           # noqa: E402
from ae4803.environment import WindModel                            # noqa: E402
from ae4803.flightlog import FlightLogger                           # noqa: E402
from ae4803.joystick import JoystickConfig, JoystickReader, ScriptedJoystick  # noqa: E402
from ae4803.realtime import LoopConfig, RealTimeLoop                # noqa: E402
from ae4803.xplane import XPlaneConfig, XPlaneSender                # noqa: E402

CONFIG = REPO / "config" / "sim.toml"


def load_config(path: Path = CONFIG) -> dict:
    with open(path, "rb") as fh:
        return tomllib.load(fh)


def build(cfg: dict, args) -> tuple:
    s = cfg.get("solver", {})
    m = cfg.get("model", {})
    ap = cfg.get("autopilot", {})
    st = cfg.get("autostop", {})
    xp = cfg.get("xplane", {})
    lg = cfg.get("logging", {})
    wd = cfg.get("wind", {})

    params = Params.load(inertia_mode=m.get("inertia_mode", "simulink"))
    model = FlightModel(
        params=params,
        autopilot=AutopilotConfig(
            wing_leveler_on=bool(ap.get("wing_leveler", False)),
            altitude_hold_on=bool(ap.get("altitude_hold", False)),
            altitude_ref_mode=str(ap.get("altitude_ref_mode", "capture")),
        ),
        autostop=AutoStopConfig(
            enabled=bool(st.get("enabled", True)),
            fix_altitude_test=bool(st.get("fix_altitude_test", False)),
        ),
        wind=WindModel(turbulence_on=bool(wd.get("turbulence", False)),
                       W20=float(wd.get("W20", 5.0)),
                       seed=int(wd.get("seed", 23341))),
    )

    if args.scripted:
        source = ScriptedJoystick(replay.profile)
        print("input: scripted doublet profile (no hardware)")
    else:
        source = JoystickReader(JoystickConfig.load())
        print("input: joystick")
        for line in source.device_report():
            print(f"        {line}")
        if source.missing:
            print(f"        NOTE: unbound controls {source.missing} will read 0. "
                  f"Is TARGET closed? Run 'python tools/list_devices.py'.")

    logger = None
    if lg.get("enabled", True) and not args.no_log:
        logger = FlightLogger(
            log_dir=REPO / lg.get("directory", "flightlogs"),
            params_mat=(REPO / "data" / "sim_init.mat"
                        if lg.get("include_parameters", True) else None),
            decimate=int(lg.get("decimate", 4)),
        )

    sender = None
    if xp.get("enabled", True) and not args.no_xplane:
        sender = XPlaneSender(XPlaneConfig(
            enabled=True, ip=str(xp.get("ip", "127.0.0.1")),
            port=int(xp.get("port", 49000)),
            protocol=str(xp.get("protocol", "vehx")),
            send_datarefs=bool(xp.get("send_datarefs", True)),
            rate_hz=float(xp.get("rate_hz", 30.0)),
            legacy_digits=int(xp.get("legacy_digits", 10)),
        ))
        print(f"X-Plane: {sender.cfg.protocol} -> "
              f"{sender.cfg.ip}:{sender.cfg.port} at {sender.cfg.rate_hz:g} Hz")

    loop = RealTimeLoop(
        model, source,
        LoopConfig(rate_hz=float(args.rate or s.get("rate_hz", 200.0)),
                   max_catchup_steps=int(s.get("max_catchup_steps", 5)),
                   high_priority=bool(s.get("high_priority", True)),
                   timer_resolution_ms=int(s.get("timer_resolution_ms", 1))),
        logger=logger, xplane=sender)
    return model, source, logger, sender, loop


def finish(loop, source, logger, sender, cfg) -> None:
    print()
    print(loop.stats.format())
    if loop.stop_reason:
        print(f"  stop reason     : {loop.stop_reason}")
    if getattr(loop, "timing_notes", None):
        for note in loop.timing_notes:
            print(f"  note            : {note}")
    if sender is not None:
        print(f"  X-Plane packets : {sender.packets_sent} "
              f"({sender.bytes_sent / 1e6:.2f} MB), {sender.errors} errors")
        sender.close()
    if getattr(source, "watchdog", None) and source.watchdog.warnings:
        print("  joystick warnings:")
        for w in source.watchdog.warnings[-5:]:
            print(f"    - {w}")
    source.close()
    if logger is not None and logger.n_samples:
        path = logger.save(metadata={
            "rate_hz": loop.cfg.rate_hz,
            "stopReason": loop.stop_reason or "",
            "inertiaMode": cfg.get("model", {}).get("inertia_mode", "simulink"),
        })
        print(f"  flight log      : {path}  ({logger.n_samples} samples)")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-gui", action="store_true", help="run headless")
    ap.add_argument("--no-xplane", action="store_true", help="do not send UDP")
    ap.add_argument("--no-log", action="store_true", help="do not write a flight log")
    ap.add_argument("--scripted", action="store_true",
                    help="use the built-in doublet profile instead of hardware")
    ap.add_argument("--rate", type=float, default=None, help="override step rate [Hz]")
    ap.add_argument("--duration", type=float, default=None,
                    help="stop after this many seconds of sim time")
    ap.add_argument("--devices", action="store_true",
                    help="list joystick devices and exit")
    ap.add_argument("--threaded", action="store_true",
                    help="run the physics on a thread instead of its own "
                         "process (simpler, but the GUI can then delay it)")
    ap.add_argument("--config", type=Path, default=CONFIG)
    args = ap.parse_args()

    if args.devices:
        from tools.list_devices import main as list_main
        return list_main()

    cfg = load_config(args.config)
    rate = float(args.rate or cfg.get("solver", {}).get("rate_hz", 200.0))
    inertia = cfg.get("model", {}).get("inertia_mode", "simulink")

    # --- GUI + separate physics process (the default) ---
    if not args.no_gui and not args.threaded:
        from PyQt6.QtCore import QTimer
        from PyQt6.QtWidgets import QApplication

        from ae4803.gui.app import MainWindow
        from ae4803.procsim import PhysicsProcess

        print(f"solver: fixed-step RK4 at {rate:g} Hz, inertia_mode = {inertia}")
        print("physics: separate process (use --threaded to run it on a thread)")
        print()

        proc = PhysicsProcess(cfg, args)
        app = QApplication(sys.argv)
        done = {"flag": False}

        def on_quit_proc():
            if done["flag"]:
                return
            done["flag"] = True
            proc.stop("stopped from the GUI")
            proc.join(6.0)
            r = proc.result
            print()
            if r is None:
                print("(physics process did not report back)")
                return
            print(r.stats_text)
            if r.stop_reason:
                print(f"  stop reason     : {r.stop_reason}")
            for note in r.timing_notes:
                print(f"  note            : {note}")
            if r.xplane_packets:
                print(f"  X-Plane packets : {r.xplane_packets}, "
                      f"{r.xplane_errors} errors")
            if r.joystick_warnings:
                print("  joystick warnings:")
                for w in r.joystick_warnings[-5:]:
                    print(f"    - {w}")
            if r.log_path:
                print(f"  flight log      : {r.log_path}")

        win = MainWindow(proc, joystick=None, on_quit=on_quit_proc)
        win.show()
        proc.start()

        if args.duration:
            t = QTimer(win)
            t.timeout.connect(
                lambda: (proc.sim_time >= args.duration) and win.close())
            t.start(200)

        rc = app.exec()
        on_quit_proc()
        return rc

    # --- headless, or GUI with the physics on a thread ---
    model, source, logger, sender, loop = build(cfg, args)

    print(f"solver: fixed-step RK4 at {loop.cfg.rate_hz:g} Hz, "
          f"inertia_mode = {inertia}")
    print()

    if args.no_gui:
        import time
        loop.start()
        try:
            while loop.running:
                if args.duration and loop.sim_time >= args.duration:
                    loop.stop(f"reached --duration {args.duration} s")
                    break
                time.sleep(0.1)
        except KeyboardInterrupt:
            loop.stop("interrupted with Ctrl-C")
        loop.join(2.0)
        finish(loop, source, logger, sender, cfg)
        return 0

    from PyQt6.QtWidgets import QApplication
    from ae4803.gui.app import MainWindow

    app = QApplication(sys.argv)
    done = {"flag": False}

    def on_quit():
        if done["flag"]:
            return
        done["flag"] = True
        loop.join(2.0)
        finish(loop, source, logger, sender, cfg)

    win = MainWindow(loop, joystick=source, xplane=sender, logger=logger,
                     on_quit=on_quit)
    win.show()
    loop.start()

    if args.duration:
        from PyQt6.QtCore import QTimer
        t = QTimer(win)
        t.timeout.connect(lambda: (loop.sim_time >= args.duration) and win.close())
        t.start(200)

    rc = app.exec()
    loop.stop("application exit")
    on_quit()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
