"""X-Plane UDP output.

Byte-for-byte reproduction of what AE4803Sim sends. Read off the model rather
than from X-Plane documentation:

VEHX  (MATLAB Function 'VEHA1', chart 500, + etargetslib Byte Pack)
    head  = uint8('VEHX0')       5 bytes - note the fifth byte is ASCII '0'
                                 (0x30), not a NUL
    p     = uint32(n)            aircraft index, 0 for the user aircraft
    lla   = double(u(1:3))       latitude, longitude [deg], altitude [m]
    euler = single(u(4:6))       heading, pitch, roll [deg]  <- psi, theta, phi
    Byte Pack datatypes = {'uint8','uint32','double','single'}, byteAlign = 1
    => 5 + 4 + 24 + 12 = 45 bytes, little-endian, tightly packed.

    The angles ARE reordered inside the block: the subsystem's input vector is
    [lat lon alt roll pitch heading ...], and Mux (SID 1581) feeds VEHX in the
    order lat1, lon1, msl1, psi1, the1, phi1.

DREF  (MATLAB Function, chart 32 and siblings)
    head = uint8('DREF0')                 5 bytes
    y    = single(value)                  4 bytes
    dref = the dataref path
    null = uint8(0)
    pad  = uint8(' ') x (499 - len(dref))
    => 5 + 4 + 500 = 509 bytes.

Destination: 127.0.0.1:49000 (the 'Send X-Plane VEHX' mask), UDP, outbound only.

A note on the brief: the lab write-up describes an older revision of this model
that used SendPositionOrientationToXPlaneOverUDP.m to send a space-separated
"phi theta psi lat lon alt" string to the UWTimedProcessingUDP.xpl plugin on
port 49003. That path is NOT in the current AE4803Sim.slx - there is no
reference to it anywhere in the .slx - so VEHX/DREF on 49000 is the default
here. The legacy sender is implemented as well and can be selected in
config/sim.toml if the older plugin is what is actually loaded in X-Plane.
"""
from __future__ import annotations

import socket
import struct
from dataclasses import dataclass

VEHX_HEADER = b"VEHX0"
DREF_HEADER = b"DREF0"
DEFAULT_IP = "127.0.0.1"
DEFAULT_PORT = 49000
LEGACY_PORT = 49003

#: Datarefs the model drives, in the order its DREF blocks appear.
DREF_ROLL = "sim/flightmodel2/controls/roll_ratio"
DREF_PITCH = "sim/flightmodel2/controls/pitch_ratio"
DREF_HEADING = "sim/flightmodel2/controls/heading_ratio"
DREF_SPEEDBRAKE = "sim/flightmodel2/controls/speedbrake_ratio"
DREF_FLAP = "sim/flightmodel/controls/flaprqst"

#: Cockpit instrument datarefs, beyond what VEHX implies.
#:
#: VEHX only places the aircraft - position and attitude. X-Plane then derives
#: what it can from successive positions, but the panel's airspeed, VSI,
#: tachometer and manifold gauges are driven by X-Plane's own flight model,
#: which we are bypassing. Writing these makes the 3D cockpit agree with our
#: model instead of guessing.
#:
#: IMPORTANT: not every one of these is writable in every X-Plane build - some
#: `sim/cockpit2/...` indicators are recomputed each frame and will ignore a
#: write. Run `python tools/xplane_dref_probe.py` with X-Plane open: it writes
#: each dataref and reads it back over the RREF protocol, so you get a measured
#: list rather than a hopeful one. Prune this tuple to whatever actually sticks.
#:
#: Each entry is (dataref, Outputs attribute, scale) - value = attr * scale.
INSTRUMENT_DREFS: tuple[tuple[str, str, float], ...] = (
    # airspeed indicator (knots)
    ("sim/flightmodel/position/indicated_airspeed", "U_kts", 1.0),
    ("sim/cockpit2/gauges/indicators/airspeed_kts_pilot", "U_kts", 1.0),
    # altimeter (feet)
    ("sim/cockpit2/gauges/indicators/altitude_ft_pilot", "alt_ft", 1.0),
    # vertical speed (feet per minute)
    ("sim/flightmodel/position/vh_ind_fpm", "hdot_ft_min", 1.0),
    ("sim/cockpit2/gauges/indicators/vvi_fpm_pilot", "hdot_ft_min", 1.0),
    # attitude indicator (degrees)
    ("sim/cockpit2/gauges/indicators/pitch_AHARS_deg_pilot", "theta_deg", 1.0),
    ("sim/cockpit2/gauges/indicators/roll_AHARS_deg_pilot", "phi_deg", 1.0),
    # directional gyro / compass (degrees)
    ("sim/cockpit2/gauges/indicators/heading_AHARS_deg_mag_pilot", "psi_deg", 1.0),
    ("sim/cockpit2/gauges/indicators/compass_heading_deg_mag", "psi_deg", 1.0),
    # angle of attack and sideslip
    ("sim/cockpit2/gauges/indicators/aoa_angle_degrees", "alpha_deg", 1.0),
    ("sim/cockpit2/gauges/indicators/sideslip_degrees", "beta_deg", 1.0),
    # engine tachometer and manifold pressure
    ("sim/cockpit2/engine/indicators/engine_speed_rpm", "rpm", 1.0),
    ("sim/cockpit2/engine/indicators/MPR_in_hg", "manifold_pressure", 1.0),
    # throttle lever position (ratio)
    ("sim/cockpit2/engine/actuators/throttle_ratio_all", "cmd_throttle", 1.0),
)


def pack_vehx(lat_deg: float, lon_deg: float, alt_m: float,
              psi_deg: float, theta_deg: float, phi_deg: float,
              index: int = 0) -> bytes:
    """45-byte VEHX packet. Angle order is heading, pitch, roll."""
    return (VEHX_HEADER
            + struct.pack("<I", int(index))
            + struct.pack("<3d", float(lat_deg), float(lon_deg), float(alt_m))
            + struct.pack("<3f", float(psi_deg), float(theta_deg), float(phi_deg)))


def pack_dref(path: str, value: float) -> bytes:
    """509-byte DREF packet."""
    p = path.encode("ascii")
    if len(p) > 499:
        raise ValueError(f"dataref path too long ({len(p)} > 499): {path}")
    return (DREF_HEADER
            + struct.pack("<f", float(value))
            + p + b"\x00" + b" " * (499 - len(p)))


def pack_legacy_string(phi_deg: float, theta_deg: float, psi_deg: float,
                       lat_deg: float, lon_deg: float, alt_m: float,
                       digits: int = 10) -> bytes:
    """The older SendPositionOrientationToXPlaneOverUDP.m message.

    MATLAB builds it with num2str(x, DIGITSPREC), which is %.{digits}g, joined
    by single spaces, in the order phi theta psi lat lon alt (degrees, metres).
    """
    def fmt(x: float) -> str:
        return f"{float(x):.{int(digits)}g}"
    return " ".join(fmt(v) for v in
                    (phi_deg, theta_deg, psi_deg, lat_deg, lon_deg, alt_m)).encode("ascii")


@dataclass
class XPlaneConfig:
    enabled: bool = True
    ip: str = DEFAULT_IP
    port: int = DEFAULT_PORT
    #: "vehx" (current model) or "legacy_string" (UWTimedProcessingUDP.xpl)
    protocol: str = "vehx"
    #: Send control-surface datarefs alongside the position packet.
    send_datarefs: bool = True
    #: Also drive the 3D cockpit instruments (see INSTRUMENT_DREFS).
    send_instruments: bool = True
    #: Restrict INSTRUMENT_DREFS to this list of dataref paths, if non-empty.
    #: Populate it from tools/xplane_dref_probe.py to send only what works.
    instrument_allowlist: tuple[str, ...] = ()
    #: Rate limit; X-Plane does not benefit from more than the display rate.
    rate_hz: float = 30.0
    aircraft_index: int = 0
    legacy_digits: int = 10


class XPlaneSender:
    """Fire-and-forget UDP sender. Never blocks the physics loop."""

    def __init__(self, cfg: XPlaneConfig | None = None):
        self.cfg = cfg or XPlaneConfig()
        self.sock: socket.socket | None = None
        self.packets_sent = 0
        self.bytes_sent = 0
        self.errors = 0
        self._last_send = -1e9
        if self.cfg.enabled:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setblocking(False)

    def _send(self, payload: bytes) -> None:
        if self.sock is None:
            return
        try:
            self.sock.sendto(payload, (self.cfg.ip, self.cfg.port))
            self.packets_sent += 1
            self.bytes_sent += len(payload)
        except OSError:
            # A dropped datagram must never stall the sim.
            self.errors += 1

    def send(self, out, now: float, *, force: bool = False) -> bool:
        """Send one frame from an ae4803.model.Outputs. Returns True if sent."""
        if self.sock is None:
            return False
        interval = 1.0 / self.cfg.rate_hz if self.cfg.rate_hz > 0 else 0.0
        if not force and (now - self._last_send) < interval:
            return False
        self._last_send = now

        if self.cfg.protocol == "legacy_string":
            self._send(pack_legacy_string(out.phi_deg, out.theta_deg, out.psi_deg,
                                          out.lat, out.lon, out.alt_m,
                                          self.cfg.legacy_digits))
            return True

        self._send(pack_vehx(out.lat, out.lon, out.alt_m,
                             out.psi_deg, out.theta_deg, out.phi_deg,
                             self.cfg.aircraft_index))
        if self.cfg.send_datarefs:
            # The model feeds X-Plane the normalised stick positions, not the
            # surface angles (mask: "flight stick inputs ... as signed ratios").
            self._send(pack_dref(DREF_ROLL, out.cmd_roll))
            self._send(pack_dref(DREF_PITCH, out.cmd_pitch))
            self._send(pack_dref(DREF_HEADING, out.cmd_yaw))
            self._send(pack_dref(DREF_FLAP, out.flap_deg / 20.0 if out.flap_deg else 0.0))

        if self.cfg.send_instruments:
            allow = self.cfg.instrument_allowlist
            for path, attr, scale in INSTRUMENT_DREFS:
                if allow and path not in allow:
                    continue
                try:
                    self._send(pack_dref(path, float(getattr(out, attr)) * scale))
                except AttributeError:
                    continue
        return True

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None
