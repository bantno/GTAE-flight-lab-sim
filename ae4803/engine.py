"""Engine model - Simulink 'Engine' subsystem (SID 164) = pow (187) + eng (169).

The pilot's throttle lever is turned into (engine speed n, manifold pressure
pz) by two lookup tables in the Controls subsystem; this module turns that pair
plus density and airspeed into the dimensionless thrust parameter dpt, and then
into the six propulsive coefficient increments.
"""
from __future__ import annotations

import numpy as np

from .params import AeroCoeffs


def shaft_power_kW(n_rpm: float, pz_inhg: float, rho: float) -> float:
    """'pow' subsystem (SID 187) up to the 0.7355 gain.

    Fcn blocks, verbatim:
        P   = (u[2] + 7.4) * (u[1] + 2010)              -> SID 199
        P1  = (408.0 - 0.0965*u[1]) * (1.0 - u[3]/1.225) -> SID 200
    Sum1 ('-++'):  -326.5 + 0.00412*P + P1
    Gain:          x 0.7355   (hp -> kW)
    """
    P = (pz_inhg + 7.4) * (n_rpm + 2010.0)
    P1 = (408.0 - 0.0965 * n_rpm) * (1.0 - rho / 1.225)
    return 0.7355 * (-326.5 + 0.00412 * P + P1)


def thrust_parameter(n_rpm: float, pz_inhg: float, rho: float, V: float) -> float:
    """dpt, the FDC Beaver thrust coefficient ('pow', SID 187 output).

    Product block '*////**' with inputs (2, V, V, V, rho, P, 191.18) then
    Sum '|++' with the constant 0.08696:

        dpt = 0.08696 + 2 * 191.18 * P / (rho * V^3)
    """
    P = shaft_power_kW(n_rpm, pz_inhg, rho)
    denom = rho * V ** 3
    if abs(denom) < 1e-9:
        denom = 1e-9 if denom >= 0 else -1e-9
    return 0.08696 + 2.0 * 191.18 * P / denom


def propulsion_coefficients(c: AeroCoeffs, dpt: float, alpha: float) -> np.ndarray:
    """'eng' subsystem (SID 169) -> Cprop = [Cx, Cy, Cz, Cl, Cm, Cn]."""
    return np.array([
        c.Cx_dpt * dpt + c.Cx_dpt2_alpha * (dpt * dpt * alpha),
        0.0,
        c.Cz_dpt * dpt,
        c.Cl_alpha2_dpt * (alpha * alpha * dpt),
        c.Cm_dpt * dpt,
        c.Cn_dpt3 * (dpt ** 3),
    ])
