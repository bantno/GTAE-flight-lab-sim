"""Identify axes by HOLDING a position, rather than sweeping.

Sweep-based calibration needs the operator to be in sync with a clock, which is
awkward when the rig is being driven remotely. This instead asks for a few
static holds and records the mean value of every axis during each one - timing
only has to be roughly right, and the sign of each axis falls straight out.

    python tools/hold_sample.py

Default protocol (edit HOLDS to change):
    1. stick FULL LEFT,  throttle IDLE
    2. stick FULL AFT,   throttle IDLE
    3. stick CENTRED,    throttle FULL FORWARD
    4. everything CENTRED / IDLE   (the reference)
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

HOLDS = [
    ("centre", "everything CENTRED, throttle at IDLE"),
    ("left", "hold stick FULL LEFT  (throttle still idle)"),
    ("aft", "hold stick FULL AFT / back  (throttle still idle)"),
    ("full_throttle", "stick CENTRED, push throttle FULL FORWARD"),
]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hold-seconds", type=float, default=5.0)
    ap.add_argument("--gap", type=float, default=4.0)
    ap.add_argument("--lead", type=float, default=10.0)
    args = ap.parse_args()

    try:
        import pygame
    except ImportError:
        print("pygame is not installed in this environment.")
        return 2

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No devices. Is TARGET running?")
        return 1

    sticks = []
    for i in range(pygame.joystick.get_count()):
        js = pygame.joystick.Joystick(i)
        js.init()
        sticks.append(js)

    labels = []
    for di, js in enumerate(sticks):
        for a in range(js.get_numaxes()):
            labels.append((di, a))

    print("Devices:")
    for js in sticks:
        print(f"  {js.get_name()}  ({js.get_numaxes()} axes)")
    print("\nHOLD each position for the whole window - no need to be precise on timing.\n")
    t = args.lead
    for name, desc in HOLDS:
        print(f"  {t:5.1f}s - {t + args.hold_seconds:5.1f}s   {desc}")
        t += args.hold_seconds + args.gap
    print(f"\n  total {t:.0f} s. First hold starts in {args.lead:.0f} s.\n")
    sys.stdout.flush()
    time.sleep(args.lead)

    means: dict[str, dict] = {}
    for name, desc in HOLDS:
        print(f">>> HOLD: {desc}")
        sys.stdout.flush()
        acc = {k: 0.0 for k in labels}
        n = 0
        t_end = time.perf_counter() + args.hold_seconds
        while time.perf_counter() < t_end:
            pygame.event.pump()
            for (di, a) in labels:
                acc[(di, a)] += sticks[di].get_axis(a)
            n += 1
            time.sleep(0.01)
        means[name] = {k: acc[k] / max(n, 1) for k in labels}
        print(f"      captured ({n} samples), relax for {args.gap:.0f}s")
        sys.stdout.flush()
        time.sleep(args.gap)

    print("\n" + "=" * 74)
    print(f"{'axis':<34}" + "".join(f"{h:>13}" for h, _ in HOLDS))
    print("=" * 74)
    for (di, a) in labels:
        nm = f"{sticks[di].get_name()}  ax{a}"
        print(f"{nm:<34}" + "".join(f"{means[h][(di, a)]:+13.3f}" for h, _ in HOLDS))

    print("\n--- interpretation (deviation from the centre hold) ---")
    base = means["centre"]
    for key, hold_name in (("left", "ROLL"), ("aft", "PITCH"),
                           ("full_throttle", "THROTTLE")):
        devs = sorted(((abs(means[key][k] - base[k]), k) for k in labels), reverse=True)
        d, k = devs[0]
        if d < 0.1:
            print(f"{hold_name:9s}  no axis changed - control not moved or absent")
            continue
        di, a = k
        delta = means[key][k] - base[k]
        print(f"{hold_name:9s}  axis {a} of '{sticks[di].get_name()}'  "
              f"centre {base[k]:+.3f} -> {means[key][k]:+.3f}  (delta {delta:+.3f})")
        for d2, k2 in devs[1:3]:
            if d2 > 0.1:
                print(f"           also: axis {k2[1]} of '{sticks[k2[0]].get_name()}' "
                      f"delta {means[key][k2] - base[k2]:+.3f}")
    pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
