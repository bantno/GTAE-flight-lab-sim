"""Atmosphere, gravity and wind - the Simulink 'Environment' subsystem (SID 245).

Simulink wiring reproduced here:

    altitude h (from xyH = Xe .* [1 1 -1])
        -> COESA Atmosphere Model      -> rho   (T, a, ps terminated)
        -> WGS84 Gravity Model         -> g (NED 3-vector)
           (fed [45.5 deg, 0 deg, h] from two Constant blocks - the model uses a
            FIXED latitude of 45.5 deg, not the aircraft's actual latitude)
    |Ve|, h, DCM
        -> Wind Models                 -> uvw_wind, pqr_wind

    EnvBus = {g, rho, Vwind{uvw_wind, pqr_wind}}
"""
from __future__ import annotations

import numpy as np

# --- 1976 U.S. Standard Atmosphere (COESA) ---------------------------------
R_AIR = 287.0531          # specific gas constant used by MATLAB atmoscoesa
GAMMA = 1.4
G0 = 9.80665              # standard gravity used for the hydrostatic integration
R_EARTH_GEOP = 6356766.0  # effective earth radius for geopotential conversion

# base geopotential altitude [m], base temperature [K], lapse rate [K/m]
_LAYERS = (
    (0.0,     288.15,  -0.0065),
    (11000.0, 216.65,   0.0),
    (20000.0, 216.65,   0.001),
    (32000.0, 228.65,   0.0028),
    (47000.0, 270.65,   0.0),
    (51000.0, 270.65,  -0.0028),
    (71000.0, 214.65,  -0.002),
    (84852.0, 186.946,  0.0),
)
_P0 = 101325.0


def _layer_base_pressures():
    """Pressure at the base of each layer, integrated hydrostatically."""
    p = [_P0]
    for i in range(len(_LAYERS) - 1):
        Hb, Tb, L = _LAYERS[i]
        Ht = _LAYERS[i + 1][0]
        if L == 0.0:
            p.append(p[i] * np.exp(-G0 * (Ht - Hb) / (R_AIR * Tb)))
        else:
            Tt = Tb + L * (Ht - Hb)
            p.append(p[i] * (Tt / Tb) ** (-G0 / (L * R_AIR)))
    return np.array(p)


_PB = _layer_base_pressures()


def geometric_to_geopotential(h: np.ndarray | float) -> np.ndarray | float:
    """Standard geometric -> geopotential conversion.

    Provided for completeness only. It is deliberately NOT applied in
    :func:`coesa` - see the note there.
    """
    return R_EARTH_GEOP * np.asarray(h, dtype=float) / (R_EARTH_GEOP + np.asarray(h, dtype=float))


def coesa(h: np.ndarray | float):
    """1976 COESA atmosphere, matching MATLAB's ``atmoscoesa``.

    Returns (T [K], a [m/s], p [Pa], rho [kg/m^3]) - the same four outputs, in the
    same order, as the Simulink COESA Atmosphere Model block (Metric (MKS)).
    Altitudes are clipped to the model's valid range, matching the block's
    'Warning' out-of-range action (which clips and keeps running).

    NOTE on altitude: MATLAB's atmoscoesa applies the lapse rates to the input
    altitude *directly*, i.e. it treats the input as geopotential altitude and
    performs no geometric->geopotential conversion. Verified against R2024a:
    atmoscoesa(914.4) gives T = 282.2064 K = 288.15 - 0.0065*914.4 exactly,
    whereas converting first would give 282.2073 K. We reproduce MATLAB's
    behaviour so the Python sim matches AE4803Sim rather than the textbook; at
    the lab's 914.4 m this is a 0.0009 K / 1.4e-5 kg/m^3 difference anyway.
    """
    scalar = np.isscalar(h) or np.ndim(h) == 0
    H = np.atleast_1d(np.asarray(h, dtype=float))
    H = np.clip(H, _LAYERS[0][0], _LAYERS[-1][0])

    bases = np.array([l[0] for l in _LAYERS])
    idx = np.clip(np.searchsorted(bases, H, side="right") - 1, 0, len(_LAYERS) - 1)

    Hb = bases[idx]
    Tb = np.array([l[1] for l in _LAYERS])[idx]
    L = np.array([l[2] for l in _LAYERS])[idx]
    pb = _PB[idx]

    T = Tb + L * (H - Hb)
    isoth = L == 0.0
    p = np.where(
        isoth,
        pb * np.exp(-G0 * (H - Hb) / (R_AIR * np.where(isoth, Tb, 1.0))),
        pb * (np.where(isoth, 1.0, T / Tb)) ** (-G0 / (np.where(isoth, 1.0, L) * R_AIR)),
    )
    rho = p / (R_AIR * T)
    a = np.sqrt(GAMMA * R_AIR * T)

    if scalar:
        return float(T[0]), float(a[0]), float(p[0]), float(rho[0])
    return T, a, p, rho


# --- WGS84 gravity (Taylor Series), as configured in the model -------------
_WGS84_A = 6378137.0                 # semi-major axis [m]
_WGS84_F = 1.0 / 298.257223563       # flattening
_WGS84_E2 = 6.69437999014e-3         # first eccentricity squared
_GAMMA_E = 9.7803253359              # normal gravity at the equator [m/s^2]
_K_SOMIG = 1.931852652458e-3         # Somigliana constant
_M_WGS84 = 3.449786506841e-3         # omega^2 a^2 b / GM

#: The Simulink model hard-codes these into the gravity block's input.
MODEL_LATITUDE_DEG = 45.5
MODEL_LONGITUDE_DEG = 0.0


def gravity_wgs84_taylor(h: np.ndarray | float,
                         lat_deg: float = MODEL_LATITUDE_DEG) -> np.ndarray | float:
    """Normal gravity magnitude [m/s^2], WGS84 Taylor-series expansion.

    Matches MATLAB ``gravitywgs84(h, lat)`` (the block's 'WGS84 Taylor Series'
    model, which is what AE4803Sim uses).
    """
    h = np.asarray(h, dtype=float)
    s2 = np.sin(np.deg2rad(lat_deg)) ** 2
    gamma0 = _GAMMA_E * (1.0 + _K_SOMIG * s2) / np.sqrt(1.0 - _WGS84_E2 * s2)
    return gamma0 * (
        1.0
        - (2.0 / _WGS84_A) * (1.0 + _WGS84_F + _M_WGS84 - 2.0 * _WGS84_F * s2) * h
        + (3.0 / _WGS84_A ** 2) * h ** 2
    )


def gravity_ned(h: float, lat_deg: float = MODEL_LATITUDE_DEG) -> np.ndarray:
    """Gravity as the NED 3-vector the Simulink block emits: [0, 0, g]."""
    return np.array([0.0, 0.0, float(gravity_wgs84_taylor(h, lat_deg))])


# --- Wind ------------------------------------------------------------------
class WindModel:
    """Wind shear + Dryden turbulence + discrete gust.

    In AE4803Sim all three are effectively disabled:
      * Wind Shear Model     W_20 = 0            -> zero
      * Dryden (MIL-F-8785C) W20 = 5 but T_on = off  -> turbulence switched OFF
      * Discrete Wind Gust   Gx = Gy = Gz = off  -> zero

    so the total wind is identically zero. The Dryden implementation is kept
    here (disabled by default) so turbulence can be switched on for an
    experiment without changing the Simulink lab model.
    """

    def __init__(self, turbulence_on: bool = False, W20: float = 5.0,
                 wingspan: float = 4.0, seed: int = 23341,
                 L_high: float = 533.4, dt: float = 0.1):
        self.turbulence_on = turbulence_on
        self.W20 = W20
        self.wingspan = wingspan
        self.L_high = L_high
        self.dt = dt
        self._rng = np.random.default_rng(seed)
        self._ug = np.zeros(3)
        self._pqr = np.zeros(3)

    def reset(self) -> None:
        self._ug[:] = 0.0
        self._pqr[:] = 0.0

    def __call__(self, V: float, h: float, dcm: np.ndarray):
        """Return (uvw_wind [m/s], pqr_wind [rad/s]) in body axes."""
        if not self.turbulence_on:
            return np.zeros(3), np.zeros(3)
        return self._dryden(V, h)

    def _dryden(self, V: float, h: float):
        """Continuous Dryden (+q -r), MIL-F-8785C, integrated with Euler steps.

        Only used when turbulence is explicitly enabled; the lab configuration
        never reaches this path.
        """
        V = max(float(V), 1e-3)
        h_ft = max(h * 3.28084, 10.0)
        W20_ft = self.W20 * 3.28084

        if h_ft < 1000.0:                       # low altitude
            Lw = h_ft
            Lu = Lv = h_ft / (0.177 + 0.000823 * h_ft) ** 1.2
            sw = 0.1 * W20_ft
            su = sv = sw / (0.177 + 0.000823 * h_ft) ** 0.4
        else:                                   # medium/high altitude
            Lu = Lv = Lw = self.L_high * 3.28084
            su = sv = sw = 0.1 * W20_ft

        Lu, Lv, Lw = Lu / 3.28084, Lv / 3.28084, Lw / 3.28084
        su, sv, sw = su / 3.28084, sv / 3.28084, sw / 3.28084

        dt = self.dt
        w = self._rng.normal(size=3) / np.sqrt(dt)
        L = np.array([Lu, Lv, Lw])
        s = np.array([su, sv, sw])
        tau = L / V
        a = np.exp(-dt / tau)
        self._ug = a * self._ug + s * np.sqrt(1.0 - a ** 2) * self._rng.normal(size=3)
        return self._ug.copy(), self._pqr.copy()
