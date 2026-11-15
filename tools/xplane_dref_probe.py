"""Find out which cockpit datarefs X-Plane actually lets us write.

Some `sim/cockpit2/...` indicator datarefs are recomputed by X-Plane every
frame and silently ignore a write; others stick. Rather than guess, this
subscribes to each dataref over X-Plane's RREF protocol, writes a test value
with DREF, and reads back to see whether it took.

    (start X-Plane, load an aircraft, then)
    python tools/xplane_dref_probe.py

Prints a WRITABLE / IGNORED / NO DATA verdict per dataref and emits a config
fragment listing only the ones that work, to paste into config/sim.toml as
`instrument_allowlist`.

Protocol notes (X-Plane 11):
    RREF request : b"RREF\\0" + int32 freq + int32 index + char[400] path   (413 B)
    RREF reply   : b"RREF," or b"RREF\\0" then N x (int32 index, float32 value)
    DREF write   : b"DREF\\0" + float32 value + char[500] path              (509 B)

The socket binds to 127.0.0.1 only, so Windows raises no firewall prompt.
X-Plane replies to the port we sent from, which is why one socket does both.
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ae4803.xplane import INSTRUMENT_DREFS, pack_dref  # noqa: E402


def rref_request(path: str, index: int, freq: int) -> bytes:
    p = path.encode("ascii")
    if len(p) > 399:
        raise ValueError("dataref path too long")
    return (b"RREF\x00" + struct.pack("<ii", int(freq), int(index))
            + p + b"\x00" * (400 - len(p)))


def parse_rref(data: bytes) -> dict[int, float]:
    if len(data) < 5 or data[:4] != b"RREF":
        return {}
    out = {}
    body = data[5:]
    for i in range(len(body) // 8):
        idx, val = struct.unpack_from("<if", body, i * 8)
        out[idx] = val
    return out


def collect(sock: socket.socket, seconds: float) -> dict[int, float]:
    """Drain RREF replies for a while, keeping the latest value per index."""
    latest: dict[int, float] = {}
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        try:
            sock.settimeout(max(0.01, end - time.perf_counter()))
            data, _ = sock.recvfrom(8192)
        except socket.timeout:
            break
        except OSError:
            break
        latest.update(parse_rref(data))
    return latest


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=49000)
    ap.add_argument("--settle", type=float, default=1.0)
    args = ap.parse_args()

    dest = (args.ip, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))          # loopback only: no firewall prompt
    print(f"probing {args.ip}:{args.port} from local port {sock.getsockname()[1]}\n")

    paths = [p for p, _, _ in INSTRUMENT_DREFS]

    # subscribe to everything at 10 Hz
    for i, p in enumerate(paths):
        sock.sendto(rref_request(p, i, 10), dest)
    time.sleep(args.settle)
    baseline = collect(sock, args.settle)

    if not baseline:
        print("No RREF replies at all. Check that:")
        print("  * X-Plane is running with an aircraft loaded (not at the menu)")
        print("  * Settings > Network > 'Accept incoming connections' is on")
        print(f"  * the UDP port really is {args.port}")
        sock.close()
        return 1

    # write a distinctive value to each, one at a time, and read back
    results = []
    for i, p in enumerate(paths):
        before = baseline.get(i)
        if before is None:
            results.append((p, "NO DATA", None, None))
            continue
        test = before + 12.345 if abs(before) < 1e5 else 12.345
        for _ in range(6):               # push a few times; X-Plane may re-run its model
            sock.sendto(pack_dref(p, test), dest)
            time.sleep(0.02)
        after = collect(sock, 0.35).get(i, before)
        took = abs(after - test) < max(0.5, abs(test) * 0.01)
        results.append((p, "WRITABLE" if took else "IGNORED", before, after))

    # unsubscribe
    for i, p in enumerate(paths):
        sock.sendto(rref_request(p, i, 0), dest)
    sock.close()

    print(f"{'dataref':<58}{'verdict':<10}{'before':>12}{'after':>12}")
    print("-" * 92)
    for p, verdict, before, after in results:
        b = f"{before:12.3f}" if before is not None else f"{'-':>12}"
        a = f"{after:12.3f}" if after is not None else f"{'-':>12}"
        print(f"{p:<58}{verdict:<10}{b}{a}")

    good = [p for p, v, _, _ in results if v == "WRITABLE"]
    print(f"\n{len(good)} of {len(paths)} datarefs are writable.")
    if good:
        print("\nPaste into config/sim.toml under [xplane]:\n")
        print("instrument_allowlist = [")
        for p in good:
            print(f'    "{p}",')
        print("]")
    else:
        print("\nNone stuck. The aircraft's instruments are probably driven by "
              "X-Plane's own flight model; try an aircraft with simpler gauges, "
              "or accept that VEHX position/attitude is as far as it goes.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
