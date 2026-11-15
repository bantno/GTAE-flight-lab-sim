"""Write the shared replay input profile for the Simulink/Python comparison.

Produces matlab/reference/replay_input.mat, which matlab/make_reference.m feeds
into the model through 'From Workspace' blocks (zero-order hold).
"""
from __future__ import annotations

import sys
from pathlib import Path

import scipy.io as sio

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from ae4803 import replay  # noqa: E402

T_END = 60.0


def main() -> None:
    t, roll, pitch, yaw, thr = replay.sampled(T_END)
    out = Path(__file__).resolve().parent.parent / "matlab" / "reference" / "replay_input.mat"
    out.parent.mkdir(parents=True, exist_ok=True)
    sio.savemat(str(out), {"t": t, "roll": roll, "pitch": pitch,
                           "yaw": yaw, "throttle": thr}, format="5")
    print(f"wrote {out}")
    print(f"  {len(t)} samples at {replay.JOYSTICK_DT*1000:.3f} ms, t_end = {t[-1]:.4f} s")
    for name, arr in (("roll", roll), ("pitch", pitch), ("yaw", yaw), ("throttle", thr)):
        print(f"  {name:9s} min={arr.min():+.3f} max={arr.max():+.3f}")


if __name__ == "__main__":
    main()
