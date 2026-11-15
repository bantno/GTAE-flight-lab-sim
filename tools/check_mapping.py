"""Print the PILOT COMMANDS the current config produces, live.

    python tools/check_mapping.py --seconds 20

Unlike raw axis dumps this shows what the model will actually receive, after
name matching, inversion, calibration and dead zones. Wiggle every control and
confirm each line responds. Also reports the min/max each command reached, so a
control that never leaves 0 is obvious.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ae4803.joystick import JoystickConfig, JoystickReader  # noqa: E402

BAR_W = 21


def bar(v: float, lo: float = -1.0, hi: float = 1.0) -> str:
    v = max(lo, min(hi, v))
    pos = int(round((v - lo) / (hi - lo) * (BAR_W - 1)))
    cells = ["."] * BAR_W
    if lo < 0:
        cells[BAR_W // 2] = "|"
    cells[pos] = "#"
    return "".join(cells)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--rate", type=float, default=10.0)
    args = ap.parse_args()

    reader = JoystickReader(JoystickConfig.load())
    print()
    for line in reader.device_report():
        print("  " + line)
    if reader.missing:
        print(f"  UNBOUND: {reader.missing}")
    print(f"\nWiggle everything for {args.seconds:.0f} s...\n")
    sys.stdout.flush()

    names = ("roll", "pitch", "yaw", "throttle")
    lo = {n: 1e9 for n in names}
    hi = {n: -1e9 for n in names}

    t_end = time.perf_counter() + args.seconds
    last = 0.0
    while time.perf_counter() < t_end:
        c = reader.poll()
        vals = {"roll": c.roll, "pitch": c.pitch, "yaw": c.yaw,
                "throttle": c.throttle}
        for n in names:
            lo[n] = min(lo[n], vals[n])
            hi[n] = max(hi[n], vals[n])
        now = time.perf_counter()
        if now - last >= 1.0 / args.rate:
            last = now
            raw = reader.raw_values
            parts = []
            for n in names:
                rng = (0.0, 1.0) if n == "throttle" else (-1.0, 1.0)
                parts.append(f"{n[:4]:>4} [{bar(vals[n], *rng)}]{vals[n]:+6.3f}"
                             f" raw{raw.get(n, float('nan')):+6.2f}")
            sys.stdout.write("\r" + "  ".join(parts))
            sys.stdout.flush()

    print("\n\n--- travel seen ---")
    ok = True
    for n in names:
        span = hi[n] - lo[n]
        flag = "" if span > 0.2 else "   <-- barely moved; check this mapping"
        if span <= 0.2:
            ok = False
        print(f"  {n:9s} {lo[n]:+7.3f} .. {hi[n]:+7.3f}   span {span:5.3f}{flag}")
    if reader.watchdog.warnings:
        print("\n  stale-axis warnings:")
        for w in reader.watchdog.warnings:
            print(f"    - {w}")
    reader.close()
    print("\nmapping looks usable" if ok else
          "\nsome controls did not move - fix config/joystick.toml before flying")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
