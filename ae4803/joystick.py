"""Direct joystick input via pygame/SDL2 - no TARGET, no virtual device.

Why this replaces the old path
------------------------------
The Simulink model read a single virtual controller ("Joystick1") that TARGET
synthesised from three physical devices. That produced the two long-standing
faults:

  * axes swapped after a reboot - the binding was by INDEX, and the index
    depended on USB enumeration order;
  * the throttle stopped updating after the rig sat idle - a TARGET/virtual
    device problem, invisible to Simulink.

Here each control is bound to a device BY NAME (substring match on the SDL
device name) plus an axis number from config/joystick.toml, so enumeration
order is irrelevant. A watchdog also flags an axis that has frozen while other
axes are plainly moving, which is what the old throttle fault looked like.

TARGET must be CLOSED: while it runs it hides the physical devices.
"""
from __future__ import annotations

import os
import time
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

from .controls import PilotCommands, dead_zone

CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"
DEFAULT_CONFIG = CONFIG_DIR / "joystick.toml"


@dataclass
class AxisSpec:
    name: str
    device: str
    axis: int
    invert: bool = False
    raw_min: float = -1.0
    raw_max: float = 1.0
    deadband_lower: float = 0.0
    deadband_upper: float = 0.0
    output: str = "bipolar"
    axis_minus: int | None = None

    def scale(self, raw: float) -> float:
        """Raw SDL value -> normalised command, with dead zone applied."""
        if self.invert:
            raw = -raw
            lo, hi = -self.raw_max, -self.raw_min
        else:
            lo, hi = self.raw_min, self.raw_max
        span = hi - lo
        if abs(span) < 1e-12:
            return 0.0
        # 0..1 within the calibrated range
        u = (raw - lo) / span
        u = min(max(u, 0.0), 1.0)
        v = (2.0 * u - 1.0) if self.output == "bipolar" else u
        return dead_zone(v, self.deadband_lower, self.deadband_upper)


@dataclass
class StaleCheckConfig:
    enabled: bool = True
    motion_threshold: float = 0.002
    freeze_seconds: float = 25.0
    repeat_warning_seconds: float = 60.0


@dataclass
class JoystickConfig:
    controls: dict[str, AxisSpec]
    stale: StaleCheckConfig = field(default_factory=StaleCheckConfig)
    rate_hz: float = 60.0
    allow_missing_devices: bool = True
    missing_device_throttle: float = 0.618
    revision: str = ""

    @classmethod
    def load(cls, path: str | Path = DEFAULT_CONFIG) -> "JoystickConfig":
        with open(path, "rb") as fh:
            raw = tomllib.load(fh)
        controls = {}
        for name, c in raw.get("controls", {}).items():
            controls[name] = AxisSpec(
                name=name, device=c["device"], axis=int(c["axis"]),
                invert=bool(c.get("invert", False)),
                raw_min=float(c.get("raw_min", -1.0)),
                raw_max=float(c.get("raw_max", 1.0)),
                deadband_lower=float(c.get("deadband_lower", 0.0)),
                deadband_upper=float(c.get("deadband_upper", 0.0)),
                output=str(c.get("output", "bipolar")),
                axis_minus=(int(c["axis_minus"]) if "axis_minus" in c else None),
            )
        s = raw.get("stale_check", {})
        p = raw.get("polling", {})
        return cls(
            controls=controls,
            stale=StaleCheckConfig(
                enabled=bool(s.get("enabled", True)),
                motion_threshold=float(s.get("motion_threshold", 0.002)),
                freeze_seconds=float(s.get("freeze_seconds", 5.0)),
                repeat_warning_seconds=float(s.get("repeat_warning_seconds", 30.0)),
            ),
            rate_hz=float(p.get("rate_hz", 60.0)),
            allow_missing_devices=bool(p.get("allow_missing_devices", True)),
            missing_device_throttle=float(p.get("missing_device_throttle", 0.618)),
            revision=str(raw.get("meta", {}).get("revision", "")),
        )


class StaleAxisWatchdog:
    """Flags a control that appears to have stopped reporting.

    A live axis always dithers by a count or two. One that is bit-for-bit
    constant for a long stretch, while other axes are being worked, has
    probably stopped updating - the historical TARGET throttle fault.

    Two refinements stop it crying wolf, which matters because a false alarm
    every flight trains people to ignore it:

    * Only axes that have ALREADY moved this session are watched. A control
      nobody has touched yet is not evidence of anything.
    * A control parked deliberately (throttle set and left) is indistinguishable
      from a dead one, so the freeze window is long rather than a few seconds.
    """

    def __init__(self, cfg: StaleCheckConfig):
        self.cfg = cfg
        self._last: dict[str, float] = {}
        self._frozen_since: dict[str, float] = {}
        self._last_warned: dict[str, float] = {}
        self._has_moved: set[str] = set()
        self.warnings: list[str] = []

    def update(self, values: dict[str, float], now: float) -> list[str]:
        """Return any NEW warnings raised on this poll."""
        if not self.cfg.enabled:
            return []
        new: list[str] = []
        moved = {k: abs(v - self._last.get(k, v)) > self.cfg.motion_threshold
                 for k, v in values.items()}
        any_moving = any(moved.values())

        for k, v in values.items():
            prev = self._last.get(k)
            if moved[k]:
                self._has_moved.add(k)
            if prev is None or v != prev:
                self._frozen_since.pop(k, None)
                continue
            if not (any_moving and k in self._has_moved):
                continue
            # bit-for-bit unchanged, has worked before, something else is moving
            t0 = self._frozen_since.setdefault(k, now)
            if now - t0 >= self.cfg.freeze_seconds:
                lw = self._last_warned.get(k, -1e9)
                if now - lw >= self.cfg.repeat_warning_seconds:
                    self._last_warned[k] = now
                    msg = (f"'{k}' has been bit-for-bit frozen for {now - t0:.0f} s "
                           f"while other axes move. If you have not simply parked "
                           f"it, the axis has stopped reporting - check the device.")
                    self.warnings.append(msg)
                    new.append(msg)
        self._last = dict(values)
        return new


class JoystickReader:
    """Reads the configured controls. Degrades gracefully if hardware is absent."""

    def __init__(self, config: JoystickConfig | None = None, verbose: bool = True):
        self.cfg = config or JoystickConfig.load()
        self.verbose = verbose
        self.watchdog = StaleAxisWatchdog(self.cfg.stale)
        self._sticks: dict[str, object] = {}
        self._bound: dict[str, object] = {}
        self._raw: dict[str, float] = {}
        self.missing: list[str] = []
        self.available = False
        self._open()

    # --- device discovery -------------------------------------------------
    def _open(self) -> None:
        # Let SDL deliver joystick input even when our window is not focused -
        # the physics runs in a child process with no window of its own.
        os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
        try:
            import pygame
        except ImportError:                                   # pragma: no cover
            self.missing = sorted(self.cfg.controls)
            if self.verbose:
                print("joystick: pygame not installed - running with zero input")
            return

        pygame.init()
        pygame.joystick.init()
        found = []
        for i in range(pygame.joystick.get_count()):
            js = pygame.joystick.Joystick(i)
            # Joystick.init() is deprecated in pygame-ce (objects self-initialise)
            # but is still required by upstream pygame. Call it only if harmless.
            if hasattr(js, "get_init") and not js.get_init():
                js.init()
            found.append(js.get_name())
            self._sticks[js.get_name()] = js

        if self.verbose:
            if found:
                print("joystick: SDL sees:")
                for n in found:
                    print(f"           - {n}")
            else:
                print("joystick: SDL sees no devices. Is TARGET still running? "
                      "It must be closed for the physical devices to appear.")

        for name, spec in self.cfg.controls.items():
            js = self._match(spec.device)
            if js is None:
                self.missing.append(name)
                if self.verbose:
                    print(f"joystick: no device matching '{spec.device}' for "
                          f"control '{name}'")
            else:
                self._bound[name] = js
        self.available = bool(self._bound)

    def _match(self, pattern: str):
        p = pattern.lower()
        for name, js in self._sticks.items():
            if p in name.lower():
                return js
        return None

    def device_report(self) -> list[str]:
        lines = []
        for name, spec in self.cfg.controls.items():
            js = self._bound.get(name)
            if js is None:
                lines.append(f"{name:9s}  UNBOUND  (wanted '{spec.device}')")
            else:
                lines.append(f"{name:9s}  axis {spec.axis}  <- {js.get_name()}")
        return lines

    # --- polling ----------------------------------------------------------
    def poll(self) -> PilotCommands:
        cmd = PilotCommands()
        if not self._bound:
            cmd.throttle = self.cfg.missing_device_throttle
            return cmd

        import pygame
        pygame.event.pump()

        raw: dict[str, float] = {}
        values: dict[str, float] = {}
        for name, spec in self.cfg.controls.items():
            js = self._bound.get(name)
            if js is None:
                continue
            try:
                v = float(js.get_axis(spec.axis))
                if spec.axis_minus is not None:
                    v -= float(js.get_axis(spec.axis_minus))
            except Exception:
                continue
            raw[name] = v
            values[name] = spec.scale(v)

        self._raw = raw
        self.watchdog.update(raw, time.perf_counter())

        cmd.roll = values.get("roll", 0.0)
        cmd.pitch = values.get("pitch", 0.0)
        cmd.yaw = values.get("yaw", 0.0)
        cmd.throttle = values.get("throttle", self.cfg.missing_device_throttle)
        return cmd

    @property
    def raw_values(self) -> dict[str, float]:
        return dict(self._raw)

    def close(self) -> None:
        try:
            import pygame
            pygame.joystick.quit()
            pygame.quit()
        except Exception:
            pass


class ScriptedJoystick:
    """Stand-in for the hardware: drives the sim from a callable.

    Used by the offline tests and by replay runs, so the whole control chain
    downstream of the stick is exercised without a device attached.
    """

    def __init__(self, fn):
        self.fn = fn
        self.t = 0.0
        self.available = True
        self.missing: list[str] = []
        self.watchdog = StaleAxisWatchdog(StaleCheckConfig(enabled=False))

    def set_time(self, t: float) -> None:
        self.t = float(t)

    def poll(self) -> PilotCommands:
        return self.fn(self.t)

    def device_report(self) -> list[str]:
        return ["scripted input (no hardware)"]

    @property
    def raw_values(self) -> dict[str, float]:
        return {}

    def close(self) -> None:
        pass
