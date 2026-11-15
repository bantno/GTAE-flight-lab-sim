"""Live axis monitor - move one control at a time to identify it.

    python tools/calibrate_axes.py              # all devices, all axes
    python tools/calibrate_axes.py --mapped     # only what joystick.toml binds

Prints a refreshing table of every axis on every device, with a bar, the live
value, and the min/max seen so far. Move one control at a time: the row that
changes is the axis to put in config/joystick.toml.

Ctrl-C to exit; it then prints a config snippet with the observed ranges.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

BAR_W = 25


def bar(v: float) -> str:
    """-1..1 to a centred bar."""
    v = max(-1.0, min(1.0, v))
    mid = BAR_W // 2
    pos = int(round((v + 1.0) / 2.0 * (BAR_W - 1)))
    cells = ["-"] * BAR_W
    cells[mid] = "|"
    cells[pos] = "#"
    return "".join(cells)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mapped", action="store_true",
                    help="only show axes bound in config/joystick.toml")
    ap.add_argument("--rate", type=float, default=20.0)
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

    wanted = None
    if args.mapped:
        from ae4803.joystick import JoystickConfig
        cfg = JoystickConfig.load()
        wanted = {(c.device.lower(), c.axis): name for name, c in cfg.controls.items()}

    lo: dict[tuple[int, int], float] = {}
    hi: dict[tuple[int, int], float] = {}

    print("Move ONE control at a time. The row that moves is that control's axis.")
    print("Ctrl-C to finish.\n")
    try:
        while True:
            pygame.event.pump()
            lines = []
            for di, js in enumerate(sticks):
                name = js.get_name()
                shown = False
                for a in range(js.get_numaxes()):
                    label = ""
                    if wanted is not None:
                        hit = [n for (dev, ax), n in wanted.items()
                               if dev in name.lower() and ax == a]
                        if not hit:
                            continue
                        label = f"  <- {hit[0]}"
                    v = js.get_axis(a)
                    k = (di, a)
                    lo[k] = min(lo.get(k, v), v)
                    hi[k] = max(hi.get(k, v), v)
                    if not shown:
                        lines.append(f"{name}")
                        shown = True
                    lines.append(f"   axis {a:<2d} [{bar(v)}] {v:+6.3f}   "
                                 f"min {lo[k]:+6.3f}  max {hi[k]:+6.3f}{label}")
                if shown:
                    lines.append("")
            sys.stdout.write("\033[2J\033[H" + "\n".join(lines))
            sys.stdout.flush()
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        pass

    print("\n\nObserved ranges - paste the relevant ones into config/joystick.toml:\n")
    for di, js in enumerate(sticks):
        for a in range(js.get_numaxes()):
            k = (di, a)
            if k in lo and abs(hi[k] - lo[k]) > 0.05:
                print(f"# {js.get_name()}  axis {a}")
                print(f"#   raw_min = {lo[k]:.4f}")
                print(f"#   raw_max = {hi[k]:.4f}")
    pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
