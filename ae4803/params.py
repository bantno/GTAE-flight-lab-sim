"""Model parameters, loaded from the original sim_init.mat.

The .mat file is a verbatim copy of the one the Simulink lab uses, so the
Python sim and AE4803Sim.slx are guaranteed to start from identical numbers.
Nothing here ever writes to the original lab folder.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import scipy.io as sio

DATA_DIR = Path(__file__).resolve().parent.parent / "data"
DEFAULT_INIT_MAT = DATA_DIR / "sim_init.mat"

#: Inertia matrix as typed into the 6DOF (Quaternion) block mask in AE4803Sim:
#:
#:     [5.787969e3  0 1.1764e2; 0 6.92893e3 0; -1.1764e2 0 1.1578329e4]
#:
#: Note it is NOT symmetric - the two Ixz entries have opposite signs, which is
#: not a physically valid inertia tensor. Someone mistyped one of them long ago.
INERTIA_AS_IN_MASK = np.array([[5.787969e3, 0.0, 1.1764e2],
                               [0.0, 6.92893e3, 0.0],
                               [-1.1764e2, 0.0, 1.1578329e4]])

#: What the block ACTUALLY integrates with is the transpose of the mask matrix.
#: Established experimentally against R2024a (see docs/MODEL_NOTES.md):
#:     Simulink(mask = A)    matches Python(I = A^T) to 0.57 % of signal scale
#:     Simulink(mask = A^T)  matches Python(I = A)   to 0.68 %
#:     the matched-matrix cases are both ~3.2 % out, in the lateral states.
#: For any proper (symmetric) inertia this transposition is invisible; it only
#: shows up because this particular mask entry is asymmetric.
INERTIA_SIMULINK = INERTIA_AS_IN_MASK.T.copy()

#: A SYMMETRIC inertia tensor - i.e. a physically realizable one.
#:
#: What is certain: an inertia tensor is symmetric by construction, because
#: I_ij = integral (r^2 delta_ij - r_i r_j) dm and r_i r_j = r_j r_i. So the
#: mask matrix is not a realizable inertia tensor, and the rotational dynamics
#: it produces are not those of ANY rigid body. (Proof: torque-free motion is
#: Ldot = -(I^-1 L) x L; two inertias give identical flows for all L only if
#: (A^-1 - S^-1) L is parallel to L for every L, i.e. A^-1 - S^-1 = c*Identity,
#: which forces A symmetric whenever S is. The mask matrix is not symmetric.)
#:
#: What is NOT verified: the SIGN below. The FDC/Delft Beaver moment equations
#:     Ix*pdot - Jxz*rdot = L + (Iy-Iz)*q*r + Jxz*p*q
#:     Iz*rdot - Jxz*pdot = N + (Ix-Iy)*p*q - Jxz*q*r
#: put -Jxz in both off-diagonal slots for Jxz = +117.64. But texts differ on
#: whether "Jxz" denotes integral(x z dm) or its negative, and the Beaver's
#: actual mass distribution has not been checked against any primary source.
#: Nothing on this machine settles it: all three .slx versions carry the same
#: asymmetric matrix, and the assignment PDF has no inertia data.
#:
#: MEASURE THE STAKES BEFORE TRUSTING EITHER. Over the 60 s replay profile,
#: worst difference as a percentage of signal scale:
#:     -Jxz vs +Jxz (the sign we are unsure of) ... 1.97 %
#:     mask (asymmetric) vs symmetric -Jxz ....... 1.09 %
#:     Jxz = 0 entirely vs symmetric -Jxz ........ 0.98 %
#: The sign uncertainty is LARGER than the asymmetry it is meant to fix. Until
#: the sign is confirmed from a primary source, "symmetric_neg" is not
#: demonstrably closer to the truth than the stock model - it is only
#: demonstrably self-consistent.
INERTIA_SYMMETRIC_NEG = np.array([[5.787969e3, 0.0, -1.1764e2],
                                  [0.0, 6.92893e3, 0.0],
                                  [-1.1764e2, 0.0, 1.1578329e4]])
INERTIA_SYMMETRIC_POS = np.array([[5.787969e3, 0.0, 1.1764e2],
                                  [0.0, 6.92893e3, 0.0],
                                  [1.1764e2, 0.0, 1.1578329e4]])
#: Backwards-compatible alias. Prefer the explicit names above.
INERTIA_PHYSICAL = INERTIA_SYMMETRIC_NEG

#: Default: reproduce the lab model, so Python and Simulink stay comparable.
INERTIA_MODES = {
    "simulink": INERTIA_SIMULINK,              # bug-for-bug with AE4803Sim
    "symmetric_neg": INERTIA_SYMMETRIC_NEG,    # -Jxz in both slots
    "symmetric_pos": INERTIA_SYMMETRIC_POS,    # +Jxz in both slots
    "physical": INERTIA_SYMMETRIC_NEG,         # alias for symmetric_neg
}
INERTIA = INERTIA_SIMULINK


def load_raw(path: str | Path = DEFAULT_INIT_MAT) -> dict:
    """Return every variable in sim_init.mat as plain floats / ndarrays."""
    raw = sio.loadmat(str(path), squeeze_me=True, struct_as_record=False)
    out = {}
    for k, v in raw.items():
        if k.startswith("__"):
            continue
        a = np.asarray(v, dtype=float)
        out[k] = float(a) if a.ndim == 0 else a
    return out


@dataclass
class Geometry:
    S: float          # wing reference area [m^2]
    b: float          # span [m]
    c: float          # mean aerodynamic chord [m]
    mass: float       # [kg]
    inertia: np.ndarray  # 3x3 [kg m^2], used exactly as Simulink stores it

    @property
    def inertia_inv(self) -> np.ndarray:
        return np.linalg.inv(self.inertia)


@dataclass
class AeroCoeffs:
    """FDC / Delft DHC-2 Beaver coefficient set.

    Grouped exactly as the Simulink subsystems build them up:
      datum + actuator increments + body-rate damping + propulsion.
    """
    # --- datum (Datum Coefficients, SID 134) ---
    Cx0: float; Cx_alpha: float; Cx_alpha2: float; Cx_alpha3: float
    Cy0: float; Cy_beta: float
    Cz0: float; Cz_alpha: float; Cz_alpha3: float
    Cl0: float; Cl_beta: float
    Cm0: float; Cm_alpha: float; Cm_alpha2: float; Cm_beta2: float
    Cn0: float; Cn_beta: float; Cn_beta3: float
    # --- aileron (SID 34) ---
    Cy_da: float; Cl_da: float; Cl_da_alpha: float; Cn_da: float
    # --- elevator (SID 49) ---
    Cz_de: float; Cz_de_beta: float; Cm_de: float
    # --- rudder (SID 79) ---
    Cx_dr: float; Cy_dr: float; Cy_dr_alpha: float; Cl_dr: float; Cn_dr: float
    # --- flap (SID 61) ---
    Cx_df: float; Cx_df_alpha: float; Cz_df: float; Cz_df_alpha: float; Cm_df: float
    # --- body-rate damping (SID 96) ---
    Cx_q: float; Cy_p: float; Cy_r: float
    Cz_q: float
    Cl_p: float; Cl_r: float
    Cm_q: float; Cm_r: float
    Cn_p: float; Cn_q: float; Cn_r: float
    # --- propulsion (SID 169) ---
    Cx_dpt: float; Cx_dpt2_alpha: float; Cz_dpt: float
    Cl_alpha2_dpt: float; Cm_dpt: float; Cn_dpt3: float


@dataclass
class Trim:
    """Surface trim offsets [deg] and the initial condition."""
    da0: float
    de0: float
    dr0: float
    uvw0: np.ndarray       # body velocity [m/s]
    xme_0: np.ndarray      # flat-earth position [m], NED (z down -> negative = up)
    euler0: np.ndarray     # [roll, pitch, yaw] [rad]
    pqr0: np.ndarray       # [p, q, r] [rad/s]
    LongLat0: np.ndarray   # [longitude, latitude] [deg]  <- note the order


@dataclass
class Deadbands:
    stick_lat: np.ndarray   # [lower, upper]
    stick_long: np.ndarray
    pedal: np.ndarray


@dataclass
class Params:
    geom: Geometry
    aero: AeroCoeffs
    trim: Trim
    deadbands: Deadbands
    wing_leveler_on: bool
    altitude_hold_on: bool
    raw: dict = field(repr=False, default_factory=dict)

    @classmethod
    def load(cls, path: str | Path = DEFAULT_INIT_MAT,
             inertia_mode: str = "simulink") -> "Params":
        """Load parameters.

        ``inertia_mode`` is "simulink" (default - bug-for-bug compatible with
        AE4803Sim) or "physical" (a valid symmetric inertia tensor).
        """
        d = load_raw(path)
        if inertia_mode not in INERTIA_MODES:
            raise ValueError(f"inertia_mode must be one of {sorted(INERTIA_MODES)}")

        geom = Geometry(
            S=d["beaver_S"], b=d["beaver_b"], c=d["beaver_c"], mass=d["beaver_mass"],
            inertia=INERTIA_MODES[inertia_mode].copy(),
        )

        names = [f.name for f in AeroCoeffs.__dataclass_fields__.values()]
        aero = AeroCoeffs(**{n: d[n] for n in names})

        trim = Trim(
            da0=d["da0"], de0=d["de0"], dr0=d["dr0"],
            uvw0=np.asarray(d["uvw0"], dtype=float).copy(),
            xme_0=np.asarray(d["xme_0"], dtype=float).copy(),
            euler0=np.array([d["beaver_init_roll"], d["beaver_init_pitch"],
                             d["beaver_init_yaw"]], dtype=float),
            pqr0=np.array([d["beaver_init_p"], d["beaver_init_q"],
                           d["beaver_init_r"]], dtype=float),
            LongLat0=np.asarray(d["LongLat0"], dtype=float).copy(),
        )

        db = Deadbands(
            stick_lat=np.asarray(d["deadband_stick_lat"], dtype=float).copy(),
            stick_long=np.asarray(d["deadband_stick_long"], dtype=float).copy(),
            pedal=np.asarray(d["deadband_pedal"], dtype=float).copy(),
        )

        return cls(geom=geom, aero=aero, trim=trim, deadbands=db,
                   wing_leveler_on=bool(d["wingLevelerOn"]),
                   altitude_hold_on=bool(d["altitudeHoldOn"]),
                   raw=d)
