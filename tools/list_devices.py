"""List every joystick SDL can see, with axis/button/hat counts.

    python tools/list_devices.py

TARGET must be CLOSED. While TARGET runs it hides the three physical
Thrustmaster devices behind a single virtual controller, which is exactly the
indirection this port removes.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def main() -> int:
    try:
        import pygame
    except ImportError:
        print("pygame is not installed in this environment.")
        return 2

    pygame.init()
    pygame.joystick.init()
    n = pygame.joystick.get_count()

    if n == 0:
        print("SDL sees no joystick devices.\n")
        print("Checklist:")
        print("  1. Is TARGET (TARGET GUI / TARGET Script Editor) still running?")
        print("     Close it completely - it hides the physical devices.")
        print("  2. Are the stick / throttle / pedals plugged in and powered?")
        print("  3. Check Windows 'Set up USB game controllers' (joy.cpl) to")
        print("     confirm Windows itself sees them.")
        return 1

    print(f"SDL sees {n} device(s):\n")
    for i in range(n):
        js = pygame.joystick.Joystick(i)
        js.init()
        print(f"  [{i}] {js.get_name()}")
        print(f"       axes {js.get_numaxes()}   buttons {js.get_numbuttons()}"
              f"   hats {js.get_numhats()}")
        try:
            print(f"       guid {js.get_guid()}")
        except Exception:
            pass
        print()

    print("Put the name (or a distinctive part of it) in config/joystick.toml")
    print("under each control's `device =`. Matching is by NAME, so the binding")
    print("survives a reboot re-ordering the USB devices.")
    print("\nNext: python tools/calibrate_axes.py   to find the axis numbers.")
    pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
