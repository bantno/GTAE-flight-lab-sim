"""Pilot/autopilot commands -> control surface deflections and engine settings.

Simulink 'Controls' subsystem (SID 290). The chain for each axis is

    <command, -1..1>  --Saturation--  --Gain--  --+ trim-->  deflection [deg]

                       sat limits      gain      trim
    elevator           [-1, 1]         -20       de0 = -1.86 deg
    aileron            [-1, 1]         -15       da0 =  0.0378 deg
    rudder             [-1, 1]         +25       dr0 = -2.44 deg
    throttle           [ 0, 1]         Throttle2RPM lookup -> n, then
                                       RPM2ManifoldPressure lookup -> pz
    flap               Constant 0 -> Rate Limiter(+/-0.5) -> lookup -> 0 deg

The four deflections are muxed as [elevator, aileron, rudder, flap] in degrees
and converted to radians (Angle Conversion5, deg -> rad) to form `uaero`.
`uprop` is [engineSpeed_rpm, manifoldPressure].

Note: the actuator state-space models in sim_init.mat (Aail/Bail/Cail, Aelv/...,
Arud/...) are NOT in this signal path - the 'Actuators' subsystem (SID 19) is a
plain pass-through (Demux 4 -> Mux 4). They are left unused here too, so the
Python sim matches the Simulink model. See docs/MODEL_NOTES.md.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# --- lookup tables, transcribed from the block masks ----------------------
#: Throttle2RPM (SID 426): bp = (1/6)*(3+(-3:3)), table = ((1+tanh(-3:3))/(1+tanh(2.65))-0.0025)*2300
_THR_BP = (1.0 / 6.0) * (3.0 + np.arange(-3, 4, dtype=float))
_THR_TBL = ((1.0 + np.tanh(np.arange(-3, 4, dtype=float))) / (1.0 + np.tanh(2.65)) - 0.0025) * 2300.0

#: RPM2ManifoldPressure (SID 424)
_MAP_BP = np.array([0.0, 1800.0, 2300.0])
_MAP_TBL = np.array([0.0, 22.5735, 26.0])

#: flap command -> deflection (SID 420)
_FLAP_BP = np.array([0.0, 1.0, 2.0, 3.0])
_FLAP_TBL = np.array([0.0, 4.0, 10.0, 20.0])

ELEVATOR_GAIN = -20.0
AILERON_GAIN = -15.0
RUDDER_GAIN = 25.0


def dead_zone(u: float, lower: float, upper: float) -> float:
    """Simulink Dead Zone block.

    0 inside the band, and OUTSIDE the band the output is shifted so it is
    continuous at the edges (u - upper above, u - lower below).
    """
    if u > upper:
        return u - upper
    if u < lower:
        return u - lower
    return 0.0


def _lookup_clip(x: float, bp: np.ndarray, tbl: np.ndarray) -> float:
    """1-D table lookup, linear interpolation, clipped outside the breakpoints."""
    return float(np.interp(x, bp, tbl))


def throttle_to_rpm(throttle: float) -> float:
    """Throttle2RPM lookup (ExtrapMethod = Clip)."""
    return _lookup_clip(throttle, _THR_BP, _THR_TBL)


def rpm_to_manifold_pressure(rpm: float) -> float:
    """RPM2ManifoldPressure lookup (ExtrapMethod = Clip)."""
    return _lookup_clip(rpm, _MAP_BP, _MAP_TBL)


def flap_command_to_deg(cmd: float) -> float:
    """Flap lookup (SID 420). The model drives it with a constant 0."""
    return _lookup_clip(cmd, _FLAP_BP, _FLAP_TBL)


@dataclass
class PilotCommands:
    """Normalised pilot commands, as produced by the Joystick subsystem."""
    roll: float = 0.0       # command_roll_joystick,    -1..1
    pitch: float = 0.0      # command_pitch_joystick,   -1..1
    yaw: float = 0.0        # command_yaw_joystick,     -1..1
    throttle: float = 0.0   # command_throttle_joystick, 0..1


@dataclass
class SurfaceState:
    """Everything the Controls subsystem emits."""
    uaero_rad: np.ndarray            # [de, da, dr, df] in radians
    uprop: np.ndarray                # [rpm, manifold pressure]
    elevator_deg: float
    aileron_deg: float
    rudder_deg: float
    flap_deg: float
    rpm: float
    manifold_pressure: float


def commands_to_surfaces(cmd: PilotCommands, da0: float, de0: float, dr0: float,
                         flap_cmd: float = 0.0) -> SurfaceState:
    """Command -> deflection, exactly as wired in SID 290."""
    pitch = float(np.clip(cmd.pitch, -1.0, 1.0))        # Saturation  (SID 1131)
    roll = float(np.clip(cmd.roll, -1.0, 1.0))          # Saturation1 (SID 1132)
    yaw = float(np.clip(cmd.yaw, -1.0, 1.0))            # Saturation2 (SID 1133)
    thr = float(np.clip(cmd.throttle, 0.0, 1.0))        # Saturation3 (SID 1134)

    elevator_deg = de0 + ELEVATOR_GAIN * pitch          # Sum4 / 'flip sign2'
    aileron_deg = da0 + AILERON_GAIN * roll             # Sum3 / 'flip sign1'
    rudder_deg = dr0 + RUDDER_GAIN * yaw                # Sum2 / 'flip sign'
    flap_deg = flap_command_to_deg(flap_cmd)

    rpm = throttle_to_rpm(thr)
    mp = rpm_to_manifold_pressure(rpm)

    uaero = np.deg2rad(np.array([elevator_deg, aileron_deg, rudder_deg, flap_deg]))
    return SurfaceState(uaero_rad=uaero, uprop=np.array([rpm, mp]),
                        elevator_deg=elevator_deg, aileron_deg=aileron_deg,
                        rudder_deg=rudder_deg, flap_deg=flap_deg,
                        rpm=rpm, manifold_pressure=mp)


# --- continuous-time PID with filtered derivative -------------------------
@dataclass
class PID:
    """Simulink 'PID Controller' block, continuous time, filtered derivative.

        u = P*e + I*xi + D*N*(e - xf)
        dxi/dt = e
        dxf/dt = N*(e - xf)

    State is carried externally so the controller integrates with the same RK4
    step as the airframe (the Simulink model uses continuous PID blocks, so the
    controller states are part of the ODE, not a separate discrete loop).
    """
    P: float
    I: float = 0.0
    D: float = 0.0
    N: float = 100.0
    limit_output: bool = False
    upper: float = np.inf
    lower: float = -np.inf
    n_states: int = field(init=False)

    def __post_init__(self):
        self.n_states = 2  # [integrator, derivative filter]

    def output(self, e: float, xi: float, xf: float) -> float:
        u = self.P * e + self.I * xi + self.D * self.N * (e - xf)
        if self.limit_output:
            u = float(np.clip(u, self.lower, self.upper))
        return float(u)

    def derivatives(self, e: float, xi: float, xf: float) -> tuple[float, float]:
        return e, self.N * (e - xf)


# PID gains, read off the block masks in SID 1058 / SID 1108.
WL_PHI = PID(P=1.0, I=0.0, D=1.0, N=100.0)                      # PID_wingleveler_phi
WL_BETA = PID(P=1.0, I=0.0, D=1.0, N=100.0)                     # PID_wingleveler_beta
AH_ALT = PID(P=0.08, I=0.02, D=0.08, N=100.0, limit_output=True)  # PID_altitudehold_altitude
AH_PITCH = PID(P=0.04, I=0.002, D=0.0, N=100.0)                 # PID_altitudehold_pitch
AH_SPEED = PID(P=5.0, I=0.0, D=0.0, N=100.0)                    # PID_altitudehold_u

#: Airspeed the altitude-hold throttle loop holds (Constant, SID 1177) [m/s]
AH_SPEED_REF = 45.0
#: Commanded-pitch authority of the altitude loop (Saturation, SID 1190) [deg]
AH_PITCH_LIMIT = 10.0

#: Autopilot controller state layout appended after the 13 rigid-body states.
AP_STATE_NAMES = (
    "wl_phi_xf",    # wing leveler phi  PD derivative filter
    "wl_beta_xf",   # wing leveler beta PD derivative filter
    "ah_alt_xi",    # altitude PID integrator
    "ah_alt_xf",    # altitude PID derivative filter
    "ah_pitch_xi",  # pitch PID integrator
)
N_AP_STATES = len(AP_STATE_NAMES)
