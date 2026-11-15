"""Flat-earth position -> geodetic latitude/longitude/altitude.

Reproduces the Aerospace Blockset 'Flat Earth to LLA' block as instantiated in
the X-Plane Visualization subsystem (SID 285):

    units      = Metric (MKS)
    ptype      = Earth (WGS84)
    F          = 1/298.257223563
    R          = 6378137
    LL0        = [LongLat0(2) LongLat0(1)]   -> [latitude, longitude] in deg
    psi        = 0                           (flat-earth X axis points north)
    refPosPort = off                         (reference altitude input = 0)

Note the index juggling: sim_init.mat stores LongLat0 = [longitude, latitude]
= [-84.394679, 33.772201], and the block reverses it. Georgia Tech, as expected.
"""
from __future__ import annotations

import numpy as np

WGS84_R = 6378137.0
WGS84_F = 1.0 / 298.257223563


class FlatEarthToLLA:
    def __init__(self, lat0_deg: float, lon0_deg: float, psi_deg: float = 0.0,
                 R: float = WGS84_R, f: float = WGS84_F, href: float = 0.0):
        self.lat0 = float(lat0_deg)
        self.lon0 = float(lon0_deg)
        self.psi = np.deg2rad(float(psi_deg))
        self.href = float(href)

        mu0 = np.deg2rad(self.lat0)
        ff = 2.0 * f - f * f
        denom = 1.0 - ff * np.sin(mu0) ** 2
        self._Rn = R / np.sqrt(denom)
        self._Rm = self._Rn * (1.0 - ff) / denom
        self._cos_mu0 = np.cos(mu0)

    @classmethod
    def from_params(cls, long_lat0: np.ndarray, psi_deg: float = 0.0,
                    href: float = 0.0) -> "FlatEarthToLLA":
        """Build from sim_init.mat's LongLat0 = [longitude, latitude]."""
        return cls(lat0_deg=float(long_lat0[1]), lon0_deg=float(long_lat0[0]),
                   psi_deg=psi_deg, href=href)

    def __call__(self, xe: np.ndarray) -> tuple[float, float, float]:
        """``xe`` is the flat-earth NED position [m]; returns (lat, lon, alt).

        Latitude/longitude in degrees, altitude in metres above the reference.
        """
        px, py, pz = float(xe[0]), float(xe[1]), float(xe[2])

        c, s = np.cos(self.psi), np.sin(self.psi)
        dN = c * px - s * py
        dE = s * px + c * py

        dmu = np.arctan2(1.0, self._Rm) * dN
        dlon = np.arctan2(1.0, self._Rn * self._cos_mu0) * dE

        lat = self.lat0 + np.rad2deg(dmu)
        lon = self.lon0 + np.rad2deg(dlon)
        lon = (lon + 180.0) % 360.0 - 180.0          # wrap to [-180, 180)
        alt = -pz + self.href
        return float(lat), float(lon), float(alt)
