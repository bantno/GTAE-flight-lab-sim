"""Pre-flight check: is this machine ready to run the sim?

    python tools/preflight.py

Checks everything that has actually bitten us, and explains each failure in
plain language with the fix. Exit code 0 = ready to fly, 1 = something needs
attention. `run.bat` calls this before launching.
"""
from __future__ import annotations

import os
import socket
import sys
import tomllib
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

OK, WARN, FAIL = "  OK  ", " WARN ", " FAIL "
_results: list[tuple[str, str, str]] = []


def record(status: str, name: str, detail: str = "") -> None:
    _results.append((status, name, detail))
    print(f"[{status}] {name}")
    if detail:
        for line in detail.splitlines():
            print(f"         {line}")


# --- 1. interpreter and packages -----------------------------------------
def check_python() -> None:
    v = sys.version_info
    if v < (3, 10):
        record(FAIL, f"Python {v.major}.{v.minor}",
               "Need 3.10+ (tomllib). Run setup.bat to build the environment.")
    else:
        record(OK, f"Python {v.major}.{v.minor}.{v.micro}", sys.executable)


def check_packages() -> None:
    for mod, label in (("numpy", "numpy"), ("scipy", "scipy"),
                       ("pyqtgraph", "pyqtgraph")):
        try:
            m = __import__(mod)
            record(OK, f"{label} {getattr(m, '__version__', '?')}")
        except ImportError:
            record(FAIL, label, f"Not installed. Run setup.bat.")
    try:
        from PyQt6 import QtCore
        record(OK, f"PyQt6 {QtCore.QT_VERSION_STR}")
    except ImportError:
        record(FAIL, "PyQt6", "Not installed. Run setup.bat.")


def check_pygame() -> None:
    try:
        import pygame
    except ImportError:
        record(FAIL, "pygame-ce", "Not installed. Run setup.bat.")
        return
    sdl = ".".join(str(x) for x in pygame.get_sdl_version())
    is_ce = "ce" in getattr(pygame, "version", type("v", (), {"ver": ""})).ver.lower() \
        or hasattr(pygame, "IS_CE")
    major, minor = pygame.get_sdl_version()[:2]
    if (major, minor) < (2, 30):
        record(FAIL, f"SDL {sdl}",
               "SDL older than 2.30 does NOT see the T-Pendular-Rudder pedals -\n"
               "they vanish from the device list entirely. You need pygame-ce.\n"
               "Fix:  pip uninstall -y pygame && pip install pygame-ce")
    else:
        record(OK, f"pygame{'-ce' if is_ce else ''} (SDL {sdl})")


# --- 2. project files -----------------------------------------------------
def check_files() -> None:
    for rel, why in (("data/sim_init.mat", "model parameters"),
                     ("config/sim.toml", "simulation config"),
                     ("config/joystick.toml", "joystick mapping")):
        p = REPO / rel
        record(OK if p.is_file() else FAIL, f"{rel} ({why})",
               "" if p.is_file() else f"Missing: {p}")
    try:
        with open(REPO / "config" / "sim.toml", "rb") as fh:
            cfg = tomllib.load(fh)
        mode = cfg.get("model", {}).get("inertia_mode", "?")
        note = ("reproduces AE4803Sim.slx exactly"
                if mode == "simulink"
                else "symmetric tensor - differs from AE4803Sim.slx by ~1% "
                     "in roll/yaw/sideslip, by design")
        record(OK, f"inertia_mode = {mode}", note)
    except Exception as exc:
        record(FAIL, "config/sim.toml parse", str(exc))

    logs = REPO / "flightlogs"
    try:
        logs.mkdir(exist_ok=True)
        probe = logs / ".writetest"
        probe.write_text("x")
        probe.unlink()
        record(OK, "flightlogs/ writable")
    except Exception as exc:
        record(FAIL, "flightlogs/ writable",
               f"Cannot write flight logs: {exc}")


# --- 3. TARGET must be closed --------------------------------------------
def check_target() -> None:
    try:
        import subprocess
        out = subprocess.run(["tasklist"], capture_output=True, text=True,
                             timeout=30).stdout.lower()
    except Exception:
        record(WARN, "TARGET not running", "Could not check the process list.")
        return
    hits = [n for n in ("targetgui", "target.exe", "t.a.r.g.e.t") if n in out]
    if hits:
        record(FAIL, "TARGET is RUNNING",
               "TARGET hides the physical Thrustmaster devices behind one\n"
               "virtual controller. Close it completely (check the system tray)\n"
               "before starting the sim.")
    else:
        record(OK, "TARGET not running")


# --- 4. joystick ----------------------------------------------------------
def check_joystick() -> None:
    os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
    try:
        from ae4803.joystick import JoystickConfig, JoystickReader
    except Exception as exc:
        record(FAIL, "joystick module", str(exc))
        return
    try:
        reader = JoystickReader(JoystickConfig.load(), verbose=False)
    except Exception as exc:
        record(FAIL, "joystick init", str(exc))
        return
    lines = reader.device_report()
    if reader.missing:
        record(WARN, f"joystick: {len(reader.missing)} control(s) unbound",
               "\n".join(lines) + "\n"
               "Those axes will read 0. Check the device is plugged in, then\n"
               "run:  python tools/check_mapping.py")
    elif not reader.available:
        record(FAIL, "joystick: no devices",
               "SDL sees nothing. Is TARGET still running?\n"
               "Diagnose with:  python tools/list_devices.py")
    else:
        record(OK, "joystick: all controls bound", "\n".join(lines))
    reader.close()


# --- 5. X-Plane -----------------------------------------------------------
def check_xplane() -> None:
    try:
        with open(REPO / "config" / "sim.toml", "rb") as fh:
            xp = tomllib.load(fh).get("xplane", {})
    except Exception:
        return
    if not xp.get("enabled", True):
        record(OK, "X-Plane output disabled in config")
        return
    ip, port = xp.get("ip", "127.0.0.1"), int(xp.get("port", 49000))
    try:
        import subprocess
        out = subprocess.run(["tasklist"], capture_output=True, text=True,
                             timeout=30).stdout.lower()
        running = "x-plane" in out or "xplane" in out
    except Exception:
        running = None
    # UDP is connectionless, so we cannot truly probe it; just report intent.
    if running is False:
        record(WARN, f"X-Plane not detected (target {ip}:{port})",
               "The sim will still run and send packets; you just won't see\n"
               "the aircraft. Start X-Plane and load an aircraft.")
    else:
        record(OK, f"X-Plane output -> {ip}:{port}",
               "" if running else "(could not confirm X-Plane is running)")
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.sendto(b"", (ip, port))
        s.close()
    except OSError as exc:
        record(WARN, "UDP send test", f"{exc}")


def main() -> int:
    print("=" * 68)
    print(" AE4803 PySim - pre-flight check")
    print("=" * 68)
    check_python()
    check_packages()
    check_pygame()
    check_files()
    check_target()
    check_joystick()
    check_xplane()

    fails = [r for r in _results if r[0] == FAIL]
    warns = [r for r in _results if r[0] == WARN]
    print("=" * 68)
    if fails:
        print(f" NOT READY - {len(fails)} problem(s) must be fixed:")
        for _, name, _d in fails:
            print(f"   - {name}")
        print("=" * 68)
        return 1
    if warns:
        print(f" READY, with {len(warns)} warning(s) - the sim will start.")
    else:
        print(" READY.")
    print("=" * 68)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
