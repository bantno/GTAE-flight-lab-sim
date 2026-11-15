"""Aerodynamic coefficient build-up - Simulink 'Aerodynamics' subsystem (SID 27).

    Aero Coefficients = Datum Coefficients (SID 134)
                      + Actuator Increments (SID 30)
                      + Body Rate Damping   (SID 96)

Every expression below was read off the block diagram (blocks + signal lines) in
simulink/systems/system_*.xml, not from the published FDC report, so that the
Python model reproduces this particular model rather than the canonical one.

Coefficient vector order throughout is [Cx, Cy, Cz, Cl, Cm, Cn] (body axes),
and the control vector is uaero = [de, da, dr, df] in RADIANS - matching the
Selector indices 0,1,2,3 used by the Aileron/Elevator/Rudder/Flap blocks.
"""
from __future__ import annotations

import numpy as np

from .params import AeroCoeffs

# Index into uaero, as wired by the Selector blocks in SID 34/49/61/79.
DE, DA, DR, DF = 0, 1, 2, 3


def datum_coefficients(c: AeroCoeffs, alpha: float, beta: float) -> np.ndarray:
    """Datum Coefficients, SID 134.

    Cx = Cx0 + Cx_a*a + Cx_a2*a^2 + Cx_a3*a^3
    Cy = Cy0 + Cy_b*b
    Cz = Cz0 + Cz_a*a + Cz_a3*a^3
    Cl = Cl0 + Cl_b*b
    Cm = Cm0 + Cm_a*a + Cm_a2*a^2 + Cm_b2*b^2
    Cn = Cn0 + Cn_b*b + Cn_b3*b^3
    """
    a2 = alpha * alpha
    a3 = a2 * alpha
    b2 = beta * beta
    b3 = b2 * beta
    return np.array([
        c.Cx0 + c.Cx_alpha * alpha + c.Cx_alpha2 * a2 + c.Cx_alpha3 * a3,
        c.Cy0 + c.Cy_beta * beta,
        c.Cz0 + c.Cz_alpha * alpha + c.Cz_alpha3 * a3,
        c.Cl0 + c.Cl_beta * beta,
        c.Cm0 + c.Cm_alpha * alpha + c.Cm_alpha2 * a2 + c.Cm_beta2 * b2,
        c.Cn0 + c.Cn_beta * beta + c.Cn_beta3 * b3,
    ])


def actuator_increments(c: AeroCoeffs, alpha: float, beta: float,
                        uaero: np.ndarray) -> np.ndarray:
    """Actuator Increments, SID 30 = aileron + elevator + rudder + flap."""
    de, da, dr, df = uaero[DE], uaero[DA], uaero[DR], uaero[DF]

    # Aileron (SID 34): only Cy, Cl, Cn; Cx/Cz/Cm gains are hard-wired to 0.
    ail = np.array([
        0.0,
        c.Cy_da * da,
        0.0,
        c.Cl_da * da + c.Cl_da_alpha * (da * alpha),
        0.0,
        c.Cn_da * da,
    ])

    # Elevator (SID 49): only Cz and Cm.
    elv = np.array([
        0.0,
        0.0,
        c.Cz_de * de + c.Cz_de_beta * (de * beta * beta),
        0.0,
        c.Cm_de * de,
        0.0,
    ])

    # Rudder (SID 79): Cx, Cy, Cl, Cn.
    rud = np.array([
        c.Cx_dr * dr,
        c.Cy_dr * dr + c.Cy_dr_alpha * (dr * alpha),
        0.0,
        c.Cl_dr * dr,
        0.0,
        c.Cn_dr * dr,
    ])

    # Flap (SID 61): Cx, Cz, Cm.
    flp = np.array([
        c.Cx_df * df + c.Cx_df_alpha * (df * alpha),
        0.0,
        c.Cz_df * df + c.Cz_df_alpha * (df * alpha),
        0.0,
        c.Cm_df * df,
        0.0,
    ])

    return ail + elv + rud + flp


def body_rate_damping(c: AeroCoeffs, b: float, cbar: float,
                      V: float, pqr: np.ndarray) -> np.ndarray:
    """Body Rate Damping, SID 96.

        Cdamp = [ (b/2)*p*Pp + cbar*q*Pq + (b/2)*r*Pr ] / V

    Note the pitch term uses the FULL chord (the 'Reference Length' gain is
    beaver_c, not beaver_c/2), while the roll/yaw terms use b/2. That asymmetry
    is what the model has, so it is what we reproduce.
    """
    p, q, r = float(pqr[0]), float(pqr[1]), float(pqr[2])

    Pp = np.array([0.0, c.Cy_p, 0.0, c.Cl_p, 0.0, c.Cn_p])        # SID 108
    Pq = np.array([c.Cx_q, 0.0, c.Cz_q, 0.0, c.Cm_q, c.Cn_q])     # SID 115
    Pr = np.array([0.0, c.Cy_r, 0.0, c.Cl_r, c.Cm_r, c.Cn_r])     # SID 123

    total = (b / 2.0) * p * Pp + cbar * q * Pq + (b / 2.0) * r * Pr
    # Simulink divides by V with no guard; keep it finite for the V -> 0 case.
    return total / (V if abs(V) > 1e-9 else 1e-9)


def aero_coefficients(c: AeroCoeffs, b: float, cbar: float, alpha: float,
                      beta: float, V: float, pqr: np.ndarray,
                      uaero: np.ndarray) -> np.ndarray:
    """Sum of the three contributions (SID 27 Sum block, '+++')."""
    return (datum_coefficients(c, alpha, beta)
            + actuator_increments(c, alpha, beta, uaero)
            + body_rate_damping(c, b, cbar, V, pqr))


def forces_and_moments(coeff6: np.ndarray, qbar: float, S: float,
                       b: float, cbar: float) -> tuple[np.ndarray, np.ndarray]:
    """Aerospace 'Aerodynamic Forces and Moments' block (SID 210), body axes.

    inputAxes = outputForceAxes = outputMomentAxes = Body, and the centre of
    pressure / centre of gravity inputs are both the constant [0 0 0], so the
    moment transfer term vanishes:

        F = qbar*S*[Cx, Cy, Cz]
        M = qbar*S*[b*Cl, cbar*Cm, b*Cn]
    """
    F = qbar * S * coeff6[0:3]
    M = qbar * S * np.array([b * coeff6[3], cbar * coeff6[4], b * coeff6[5]])
    return F, M


def incidence_sideslip_airspeed(v_rel: np.ndarray) -> tuple[float, float, float]:
    """Aerospace 'Incidence, Sideslip, & Airspeed' block (SID 240).

    alpha = atan2(w, u), V = |v|, beta = asin(v / V).
    """
    u, v, w = float(v_rel[0]), float(v_rel[1]), float(v_rel[2])
    V = float(np.sqrt(u * u + v * v + w * w))
    alpha = float(np.arctan2(w, u))
    beta = float(np.arcsin(np.clip(v / V, -1.0, 1.0))) if V > 1e-12 else 0.0
    return alpha, beta, V


def dynamic_pressure(vel: np.ndarray, rho: float) -> float:
    """Aerospace 'Dynamic Pressure' block (SID 239): q = 0.5*rho*v'*v.

    Careful: in AE4803Sim this block is fed the EARTH-frame velocity Ve
    (ACBus signal2), not the body-axis relative velocity. With zero wind the
    magnitudes are identical, but the wiring is reproduced faithfully.
    """
    return 0.5 * rho * float(np.dot(vel, vel))
