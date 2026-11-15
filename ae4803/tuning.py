"""In-flight parameter tuning: which numbers can be changed, and how.

Why this works at all: nothing in the model is precomputed. `FlightModel`
reads `params.aero.<coeff>` and `params.geom.<...>` fresh inside every
derivative evaluation, and `Geometry.inertia_inv` is a property that inverts
on demand. So assigning a new value takes effect on the very next RK4 stage -
no re-initialisation, no restart, no state reset.

Changes are applied as a STEP, deliberately. The aircraft keeps its velocity
and attitude, so nothing diverges, but whatever trim you were holding will
generally no longer be trim. That discontinuity is usually the point: it
excites the modes you want to watch.

Every change is recorded (see `TuningLog`) and written into the flight log, so
post-processing can tell what the model was at any instant. A log without that
is uninterpretable.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import numpy as np

# --- group names, in display order ---------------------------------------
G_LON = "Longitudinal"
G_LAT = "Lateral / Directional"
G_CTRL = "Control power"
G_DAMP = "Damping"
G_MASS = "Mass & inertia"
G_PROP = "Propulsion"
GROUP_ORDER = (G_LON, G_LAT, G_CTRL, G_DAMP, G_MASS, G_PROP)


@dataclass(frozen=True)
class Tunable:
    """One editable number."""
    key: str                     # stable id, e.g. "aero.Cm_alpha"
    label: str                   # shown in the table
    group: str
    description: str
    units: str = ""
    get: Callable[[object], float] = None       # (params) -> value
    set: Callable[[object, float], None] = None  # (params, value) -> None


def _aero(name: str, group: str, desc: str, units: str = "1/rad") -> Tunable:
    return Tunable(
        key=f"aero.{name}", label=name, group=group, description=desc, units=units,
        get=lambda p, n=name: float(getattr(p.aero, n)),
        set=lambda p, v, n=name: setattr(p.aero, n, float(v)),
    )


def _inertia_diag(i: int, label: str, desc: str) -> Tunable:
    return Tunable(
        key=f"geom.I{label}", label=f"I_{label}", group=G_MASS,
        description=desc, units="kg m^2",
        get=lambda p, i=i: float(p.geom.inertia[i, i]),
        set=lambda p, v, i=i: p.geom.inertia.__setitem__((i, i), float(v)),
    )


def _set_ixz(p, v: float) -> None:
    """Write BOTH off-diagonal slots, i.e. make the tensor symmetric.

    The stock model's matrix is asymmetric (see docs/MODEL_NOTES.md) - that is
    the bug. Editing Ixz here writes a proper symmetric product of inertia, so
    touching this control also makes the tensor physically valid.
    """
    p.geom.inertia[0, 2] = float(v)
    p.geom.inertia[2, 0] = float(v)


#: The curated set. Ordered within each group the way a flight-test course
#: would introduce them.
TUNABLES: tuple[Tunable, ...] = (
    # --- longitudinal ---
    _aero("Cm_alpha", G_LON, "Pitch stiffness. The static stability derivative - "
          "more negative is more statically stable. Drive it toward 0 to watch "
          "the short period soften, past 0 to make it divergent."),
    _aero("Cz_alpha", G_LON, "Lift-curve slope (body axis Z). Sets how hard the "
          "aircraft resists a change in alpha."),
    _aero("Cm_alpha2", G_LON, "Quadratic pitching-moment term in alpha."),
    _aero("Cx_alpha", G_LON, "Axial force vs alpha."),
    _aero("Cm0", G_LON, "Pitching moment at zero alpha - shifts the trim point.",
          units="-"),
    # --- lateral / directional ---
    _aero("Cn_beta", G_LAT, "Weathercock (directional) stability. Positive is "
          "stable; reduce it to loosen Dutch roll, go negative for directional "
          "divergence."),
    _aero("Cl_beta", G_LAT, "Dihedral effect. Negative is stable in roll; it is "
          "what couples sideslip into bank."),
    _aero("Cy_beta", G_LAT, "Side force due to sideslip."),
    # --- control power ---
    _aero("Cm_de", G_CTRL, "Elevator power."),
    _aero("Cl_da", G_CTRL, "Aileron power."),
    _aero("Cn_dr", G_CTRL, "Rudder power. Negative here, which is why a positive "
          "yaw command yaws the nose LEFT."),
    _aero("Cy_dr", G_CTRL, "Side force from rudder."),
    _aero("Cn_da", G_CTRL, "Adverse yaw from aileron."),
    _aero("Cl_dr", G_CTRL, "Roll from rudder."),
    # --- damping ---
    _aero("Cm_q", G_DAMP, "Pitch damping. The big one for short-period damping; "
          "halve it and the response gets noticeably oscillatory.",
          units="1/rad (per qc/V)"),
    _aero("Cl_p", G_DAMP, "Roll damping - sets the roll mode time constant.",
          units="1/rad (per pb/2V)"),
    _aero("Cn_r", G_DAMP, "Yaw damping - the Dutch roll damper.",
          units="1/rad (per rb/2V)"),
    _aero("Cn_p", G_DAMP, "Yaw due to roll rate."),
    _aero("Cl_r", G_DAMP, "Roll due to yaw rate."),
    _aero("Cz_q", G_DAMP, "Heave damping from pitch rate."),
    # --- mass & inertia ---
    Tunable(key="geom.mass", label="mass", group=G_MASS,
            description="Aircraft mass. Changes wing loading, so trim speed and "
                        "every acceleration with it.",
            units="kg",
            get=lambda p: float(p.geom.mass),
            set=lambda p, v: setattr(p.geom, "mass", float(v))),
    _inertia_diag(0, "xx", "Roll inertia - sets the roll mode time constant "
                           "together with Cl_p."),
    _inertia_diag(1, "yy", "Pitch inertia - sets short-period frequency together "
                           "with Cm_alpha."),
    _inertia_diag(2, "zz", "Yaw inertia - sets Dutch roll frequency together "
                           "with Cn_beta."),
    Tunable(key="geom.Ixz", label="I_xz", group=G_MASS,
            description="Roll/yaw product of inertia. NOTE: editing this writes "
                        "BOTH off-diagonal slots, i.e. it makes the tensor "
                        "symmetric - which also repairs the asymmetry the stock "
                        "model ships with. See docs/MODEL_NOTES.md.",
            units="kg m^2",
            get=lambda p: float(p.geom.inertia[0, 2]),
            set=_set_ixz),
    # --- propulsion ---
    _aero("Cx_dpt", G_PROP, "Thrust coefficient into axial force.", units="-"),
    _aero("Cm_dpt", G_PROP, "Pitching moment from thrust - the power/pitch "
          "coupling.", units="-"),
    _aero("Cz_dpt", G_PROP, "Normal force from thrust.", units="-"),
)

BY_KEY: dict[str, Tunable] = {t.key: t for t in TUNABLES}


def all_aero_keys(params) -> list[str]:
    """Every aero coefficient, for the 'everything' tab."""
    return [f"aero.{f}" for f in params.aero.__dataclass_fields__]


def tunable_for(key: str, params) -> Tunable:
    """Return a Tunable for any key, synthesising one for bare aero coeffs."""
    if key in BY_KEY:
        return BY_KEY[key]
    if key.startswith("aero."):
        name = key.split(".", 1)[1]
        if name in params.aero.__dataclass_fields__:
            return _aero(name, G_LON, "")
    raise KeyError(key)


@dataclass
class TuningLog:
    """Record of every parameter change, with the sim time it took effect."""
    entries: list[dict] = field(default_factory=list)

    def record(self, t: float, key: str, old: float, new: float) -> None:
        self.entries.append({"t": float(t), "key": key,
                             "old": float(old), "new": float(new)})

    def to_mat(self) -> dict:
        """Struct-of-arrays, easy to read in MATLAB."""
        if not self.entries:
            return {"t": np.zeros((0, 1)), "key": np.array([], dtype=object),
                    "old": np.zeros((0, 1)), "new": np.zeros((0, 1))}
        return {
            "t": np.array([[e["t"]] for e in self.entries], dtype=float),
            "key": np.array([e["key"] for e in self.entries], dtype=object),
            "old": np.array([[e["old"]] for e in self.entries], dtype=float),
            "new": np.array([[e["new"]] for e in self.entries], dtype=float),
        }

    def __len__(self) -> int:
        return len(self.entries)


def snapshot(params, keys=None) -> dict[str, float]:
    """Current value of each tunable."""
    keys = keys or [t.key for t in TUNABLES]
    return {k: tunable_for(k, params).get(params) for k in keys}


def apply(params, key: str, value: float) -> float:
    """Set one parameter, returning the value it had before."""
    t = tunable_for(key, params)
    old = t.get(params)
    t.set(params, value)
    return old
