"""Deterministic input profile shared by the Simulink and Python replay runs.

Both sides consume the SAME command sequence, sampled at 1/30 s - the rate the
real Pilot Joystick block runs at - so any difference in the results is the
model, not the input.

Why the pulses are smooth
-------------------------
A hard step input cannot be compared cleanly between a fixed-step and a
variable-step run: Simulink's 'From Workspace' block with a continuous sample
time does not force solver steps onto the data's sample instants, so the step
takes effect at whatever solver step happens to straddle it (up to MaxStep =
0.04 s away). That shows up as a one-sample spike on the control-surface traces
worth the full step height, which tells you nothing about the dynamics.

Using C1-continuous raised-cosine pulses plus linear interpolation on both
sides makes the input agree to ~1e-9, so the residual really is the model. The
original hard-step profile is kept as :func:`profile_step` for the behavioural
sanity check.
"""
from __future__ import annotations

import numpy as np

from .controls import PilotCommands

JOYSTICK_DT = 1.0 / 30.0
TRIM_THROTTLE = 0.618

#: (start, end) of each doublet [s] and the ramp time used to smooth its edges.
PITCH_PULSE = (10.0, 12.0)
ROLL_PULSE = (20.0, 22.0)
YAW_PULSE = (30.0, 32.0)
THROTTLE_PULSE = (40.0, 42.0)
RAMP = 0.5

PITCH_AMP = 0.20
ROLL_AMP = 0.30
YAW_AMP = 0.20
THROTTLE_AMP = 0.182     # 0.618 -> 0.800


def _raised_cosine(t: np.ndarray, t0: float, t1: float, ramp: float) -> np.ndarray:
    """0 outside [t0, t1+ramp], 1 on [t0+ramp, t1], smooth in between."""
    t = np.asarray(t, dtype=float)
    y = np.zeros_like(t)
    rise = (t > t0) & (t < t0 + ramp)
    y[rise] = 0.5 * (1.0 - np.cos(np.pi * (t[rise] - t0) / ramp))
    y[(t >= t0 + ramp) & (t <= t1)] = 1.0
    fall = (t > t1) & (t < t1 + ramp)
    y[fall] = 0.5 * (1.0 + np.cos(np.pi * (t[fall] - t1) / ramp))
    return y


def profile_arrays(t: np.ndarray):
    """Smooth profile evaluated on an array of times."""
    t = np.asarray(t, dtype=float)
    roll = ROLL_AMP * _raised_cosine(t, *ROLL_PULSE, RAMP)
    pitch = PITCH_AMP * _raised_cosine(t, *PITCH_PULSE, RAMP)
    yaw = YAW_AMP * _raised_cosine(t, *YAW_PULSE, RAMP)
    thr = TRIM_THROTTLE + THROTTLE_AMP * _raised_cosine(t, *THROTTLE_PULSE, RAMP)
    return roll, pitch, yaw, thr


def profile(t: float) -> PilotCommands:
    """Smooth doublets on each axis in turn, starting from the trim condition."""
    roll, pitch, yaw, thr = (float(a[0]) for a in profile_arrays(np.array([t])))
    return PilotCommands(roll=roll, pitch=pitch, yaw=yaw, throttle=thr)


def profile_step(t: float) -> PilotCommands:
    """Original hard-step doublets - behavioural check, not numeric comparison."""
    roll = pitch = yaw = 0.0
    thr = TRIM_THROTTLE
    if PITCH_PULSE[0] <= t < PITCH_PULSE[1]:
        pitch = PITCH_AMP
    elif ROLL_PULSE[0] <= t < ROLL_PULSE[1]:
        roll = ROLL_AMP
    elif YAW_PULSE[0] <= t < YAW_PULSE[1]:
        yaw = YAW_AMP
    elif THROTTLE_PULSE[0] <= t < THROTTLE_PULSE[1]:
        thr = TRIM_THROTTLE + THROTTLE_AMP
    return PilotCommands(roll=roll, pitch=pitch, yaw=yaw, throttle=thr)


def sampled(t_end: float, dt: float = JOYSTICK_DT):
    """Return (t, roll, pitch, yaw, throttle) on the 1/30 s sample grid."""
    n = int(round(t_end / dt)) + 1
    t = np.arange(n) * dt
    roll, pitch, yaw, thr = profile_arrays(t)
    return t, roll, pitch, yaw, thr


class InterpolatedInput:
    """Linear interpolation of the sampled profile.

    Identical semantics to Simulink's 'From Workspace' block with
    'Interpolate data' switched ON, which is how make_reference.m configures it.
    """

    def __init__(self, t_end: float, dt: float = JOYSTICK_DT):
        self.t, self.roll, self.pitch, self.yaw, self.thr = sampled(t_end, dt)

    def __call__(self, t: float) -> PilotCommands:
        return PilotCommands(
            roll=float(np.interp(t, self.t, self.roll)),
            pitch=float(np.interp(t, self.t, self.pitch)),
            yaw=float(np.interp(t, self.t, self.yaw)),
            throttle=float(np.interp(t, self.t, self.thr)),
        )


class ZeroOrderHoldInput:
    """Zero-order hold of the sampled profile ('Interpolate data' OFF)."""

    def __init__(self, t_end: float, dt: float = JOYSTICK_DT, step: bool = False):
        self.dt = dt
        n = int(round(t_end / dt)) + 1
        self.t = np.arange(n) * dt
        fn = profile_step if step else profile
        cmds = [fn(float(ti)) for ti in self.t]
        self.roll = np.array([c.roll for c in cmds])
        self.pitch = np.array([c.pitch for c in cmds])
        self.yaw = np.array([c.yaw for c in cmds])
        self.thr = np.array([c.throttle for c in cmds])

    def __call__(self, t: float) -> PilotCommands:
        i = int(np.clip(int(np.floor(t / self.dt + 1e-9)), 0, len(self.t) - 1))
        return PilotCommands(roll=float(self.roll[i]), pitch=float(self.pitch[i]),
                             yaw=float(self.yaw[i]), throttle=float(self.thr[i]))
