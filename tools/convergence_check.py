"""Three-way check: is the residual a model difference or just integration error?

Compares, against a tight-tolerance Simulink run (ode45, RelTol 1e-10):
  * the lab's own solver settings (ode23, RelTol 1e-2, MaxStep 0.04)
  * the Python fixed-step RK4 at several rates

If Python sits closer to the tight reference than the lab's own loose solver
does, the Python model is faithful and the apparent mismatch is the lab
reference's own integration error.

    python tools/convergence_check.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import scipy.io as sio

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tools.replay_compare import SIGNAL_MAP, SCALE, run_python  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
REFDIR = REPO / "matlab" / "reference"


def load_ref(name: str):
    md = sio.loadmat(str(REFDIR / name), squeeze_me=False, struct_as_record=False)
    return md["ref"][0, 0], float(np.squeeze(md["tEnd"]))


def err_vs(truth, t_series, y_series) -> dict[str, float]:
    """Max |error| as a percentage of each signal's scale."""
    out = {}
    for sl_name, attr in SIGNAL_MAP.items():
        if not hasattr(truth, sl_name):
            continue
        arr = np.asarray(getattr(truth, sl_name), dtype=float)
        t_t, y_t = arr[:, 0], arr[:, 1]
        y_i = np.interp(t_t, t_series, y_series[attr])
        sc = SCALE.get(attr, max(float(np.max(np.abs(y_t))), 1e-9))
        out[attr] = float(np.max(np.abs(y_i - y_t)) / sc * 100.0)
    return out


def main() -> None:
    tight, t_end = load_ref("ref_tight.mat")
    loose, _ = load_ref("ref_run.mat")

    # lab solver (ode23 @ RelTol 1e-2) vs tight
    loose_series, loose_t = {}, None
    for sl_name, attr in SIGNAL_MAP.items():
        if not hasattr(loose, sl_name):
            continue
        arr = np.asarray(getattr(loose, sl_name), dtype=float)
        loose_t = arr[:, 0]
        loose_series[attr] = arr[:, 1]
    cols = {"ode23 (lab)": err_vs(tight, loose_t, loose_series)}

    for rate in (100.0, 200.0, 400.0, 1000.0):
        t_py, py = run_python(t_end, rate)
        cols[f"RK4 {rate:g}Hz"] = err_vs(tight, t_py, py)
        print(f"  ran Python @ {rate:g} Hz")

    names = sorted(SIGNAL_MAP.values(), key=lambda a: -cols["ode23 (lab)"].get(a, 0.0))
    hdr = "".join(f"{k:>14}" for k in cols)
    print("\nmax |error| as % of signal scale, measured against the tight Simulink run")
    print(f"{'signal':<16}{hdr}")
    print("-" * (16 + 14 * len(cols)))
    for a in names:
        if a not in cols["ode23 (lab)"]:
            continue
        row = "".join(f"{cols[k].get(a, float('nan')):13.3f}%" for k in cols)
        print(f"{a:<16}{row}")
    print("-" * (16 + 14 * len(cols)))
    worst = {k: max(v.values()) for k, v in cols.items()}
    print(f"{'WORST':<16}" + "".join(f"{worst[k]:13.3f}%" for k in cols))

    out = REPO / "compare_out" / "convergence.txt"
    out.parent.mkdir(exist_ok=True)
    with open(out, "w", encoding="utf-8") as fh:
        fh.write("max |error| as % of signal scale vs tight Simulink run "
                 "(ode45, RelTol 1e-10)\n\n")
        fh.write(f"{'signal':<16}{hdr}\n")
        for a in names:
            if a not in cols["ode23 (lab)"]:
                continue
            fh.write(f"{a:<16}" + "".join(f"{cols[k].get(a, float('nan')):13.3f}%"
                                          for k in cols) + "\n")
        fh.write(f"\n{'WORST':<16}" + "".join(f"{worst[k]:13.3f}%" for k in cols) + "\n")
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
