"""Timed, hands-free axis identification - no keyboard input needed.

Runs through a fixed schedule, announcing each phase, and records which axis
moved during each one. Use it when you cannot interact with the console (e.g.
driving the rig remotely): just read the schedule, move the named control
during its window, and leave everything else still.

    python tools/guided_calibrate.py
    python tools/guided_calibrate.py --phase-seconds 8

At the end it prints, for each phase, the axes that moved and by how much,
plus a ready-to-paste config/joystick.toml fragment.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

PHASES = [
    ("roll", "STICK left and right, stop to stop"),
    ("pitch", "STICK forward and back, stop to stop"),
    ("throttle", "THROTTLE lever idle to full and back"),
    ("yaw", "RUDDER PEDALS full left and full right"),
]
#: An axis must move at least this much to count as 'the one'.
MIN_RANGE = 0.15


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--phase-seconds", type=float, default=7.0)
    ap.add_argument("--settle", type=float, default=2.0,
                    help="pause between phases")
    ap.add_argument("--lead", type=float, default=10.0,
                    help="seconds before the first phase starts")
    args = ap.parse_args()

    try:
        import pygame
    except ImportError:
        print("pygame is not installed in this environment.")
        return 2

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No devices. Is TARGET still running? See tools/list_devices.py.")
        return 1

    sticks = []
    for i in range(pygame.joystick.get_count()):
        js = pygame.joystick.Joystick(i)
        js.init()
        sticks.append(js)

    print("Devices:")
    for js in sticks:
        print(f"  {js.get_name()}  ({js.get_numaxes()} axes)")
    print()
    print("SCHEDULE - move ONLY the named control during its window:")
    t = 0.0
    for name, desc in PHASES:
        print(f"  {t:5.1f}s - {t + args.phase_seconds:5.1f}s   {name:9s} {desc}")
        t += args.phase_seconds + args.settle
    print(f"  total {t:.0f} s. Starting in {args.lead:.0f} s...\n")
    sys.stdout.flush()
    time.sleep(args.lead)

    results = {}
    for name, desc in PHASES:
        print(f">>> NOW: {name.upper():9s} - {desc}")
        sys.stdout.flush()
        lo: dict[tuple[int, int], float] = {}
        hi: dict[tuple[int, int], float] = {}
        t_end = time.perf_counter() + args.phase_seconds
        while time.perf_counter() < t_end:
            pygame.event.pump()
            for di, js in enumerate(sticks):
                for a in range(js.get_numaxes()):
                    v = js.get_axis(a)
                    k = (di, a)
                    lo[k] = min(lo.get(k, v), v)
                    hi[k] = max(hi.get(k, v), v)
            time.sleep(0.01)

        moved = sorted(((hi[k] - lo[k], k) for k in lo if hi[k] - lo[k] >= MIN_RANGE),
                       reverse=True)
        results[name] = [(k, lo[k], hi[k], rng) for rng, k in moved]
        if moved:
            for rng, k in moved:
                di, a = k
                print(f"      moved: {sticks[di].get_name()}  axis {a}   "
                      f"range {rng:.3f}  ({lo[k]:+.3f} .. {hi[k]:+.3f})")
        else:
            print("      nothing moved")
        print(f"      (pause {args.settle:.0f}s)")
        sys.stdout.flush()
        time.sleep(args.settle)

    print("\n" + "=" * 68)
    print("RESULT")
    print("=" * 68)
    for name, _ in PHASES:
        hits = results[name]
        if not hits:
            print(f"{name:9s}  NO AXIS MOVED - device missing or not touched")
            continue
        (di, a), lo_v, hi_v, rng = hits[0]
        extra = ""
        if len(hits) > 1:
            extra = f"   (also moved: " + ", ".join(
                f"axis {k[1]} of {sticks[k[0]].get_name()}" for k, _, _, _ in hits[1:]) + ")"
        print(f"{name:9s}  axis {a} of '{sticks[di].get_name()}'   "
              f"range {lo_v:+.3f} .. {hi_v:+.3f}{extra}")

    print("\n--- config/joystick.toml fragment ---")
    for name, _ in PHASES:
        hits = results[name]
        if not hits:
            print(f"# [controls.{name}]  <- no axis identified")
            continue
        (di, a), lo_v, hi_v, _ = hits[0]
        dev = sticks[di].get_name()
        unipolar = name == "throttle"
        print(f"[controls.{name}]")
        print(f'device  = "{dev}"')
        print(f"axis    = {a}")
        print(f"invert  = false   # flip if the control responds backwards")
        print(f"raw_min = {lo_v:.4f}")
        print(f"raw_max = {hi_v:.4f}")
        db = 0.0 if unipolar else 0.01
        print(f"deadband_lower = {-db}")
        print(f"deadband_upper = {db}")
        print(f'output = "{"unipolar" if unipolar else "bipolar"}"')
        print()

    pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
