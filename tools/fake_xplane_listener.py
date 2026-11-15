"""Stand-in for X-Plane: receive and decode the sim's UDP packets.

    python tools/fake_xplane_listener.py                 # VEHX/DREF on 49000
    python tools/fake_xplane_listener.py --port 49003 --legacy

Binds to 127.0.0.1 ONLY, so Windows does not raise a firewall prompt (we
cannot approve one on this machine). Run it instead of X-Plane to confirm the
packets are well-formed before the end-to-end test.
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ae4803.xplane import DREF_HEADER, VEHX_HEADER  # noqa: E402


def decode_vehx(data: bytes) -> str:
    if len(data) != 45:
        return f"VEHX? wrong length {len(data)} (expected 45)"
    idx = struct.unpack_from("<I", data, 5)[0]
    lat, lon, alt = struct.unpack_from("<3d", data, 9)
    psi, the, phi = struct.unpack_from("<3f", data, 33)
    return (f"VEHX idx={idx}  lat={lat:+10.6f} lon={lon:+11.6f} alt={alt:8.2f} m  "
            f"psi={psi:+7.2f} the={the:+7.2f} phi={phi:+7.2f} deg")


def decode_dref(data: bytes) -> str:
    if len(data) != 509:
        return f"DREF? wrong length {len(data)} (expected 509)"
    value = struct.unpack_from("<f", data, 5)[0]
    path = data[9:].split(b"\x00", 1)[0].decode("ascii", "replace")
    return f"DREF {path:<48s} = {value:+8.4f}"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=49000)
    ap.add_argument("--legacy", action="store_true",
                    help="expect the plain-string UWTimedProcessingUDP format")
    ap.add_argument("--quiet", action="store_true",
                    help="counters only, do not print every packet")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # 127.0.0.1 only - a loopback bind does not trigger the Windows firewall.
    sock.bind(("127.0.0.1", args.port))
    sock.settimeout(1.0)
    print(f"listening on 127.0.0.1:{args.port}  (Ctrl-C to stop)\n")

    counts = {"VEHX": 0, "DREF": 0, "legacy": 0, "other": 0}
    t0 = time.perf_counter()
    last_report = t0
    try:
        while True:
            try:
                data, _ = sock.recvfrom(2048)
            except socket.timeout:
                continue

            if args.legacy or not (data.startswith(VEHX_HEADER)
                                   or data.startswith(DREF_HEADER)):
                try:
                    text = data.decode("ascii").strip()
                    parts = text.split()
                    if len(parts) == 6:
                        counts["legacy"] += 1
                        line = ("legacy  phi/the/psi = "
                                f"{parts[0]:>10s} {parts[1]:>10s} {parts[2]:>10s}  "
                                f"lat/lon/alt = {parts[3]:>12s} {parts[4]:>12s} {parts[5]:>10s}")
                    else:
                        counts["other"] += 1
                        line = f"other ({len(data)} B): {text[:80]}"
                except UnicodeDecodeError:
                    counts["other"] += 1
                    line = f"other ({len(data)} B, binary)"
            elif data.startswith(VEHX_HEADER):
                counts["VEHX"] += 1
                line = decode_vehx(data)
            else:
                counts["DREF"] += 1
                line = decode_dref(data)

            if not args.quiet:
                print(line)

            now = time.perf_counter()
            if now - last_report >= 5.0:
                dt = now - t0
                print(f"  -- {dt:6.1f} s: " + "  ".join(
                    f"{k}={v} ({v/dt:.1f}/s)" for k, v in counts.items() if v))
                last_report = now
    except KeyboardInterrupt:
        pass

    dt = max(time.perf_counter() - t0, 1e-9)
    print(f"\ntotals after {dt:.1f} s:")
    for k, v in counts.items():
        if v:
            print(f"  {k:<7s} {v:7d}  ({v/dt:6.1f}/s)")
    sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
