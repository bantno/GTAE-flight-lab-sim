"""Replay the shared input profile in Python and compare against Simulink.

    python tools/make_input_profile.py           # 1. write the input
    matlab -batch "addpath('matlab'); make_reference"   # 2. Simulink reference
    python tools/replay_compare.py               # 3. compare + plot

Writes compare_out/ with a difference table and one PNG per signal group.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import scipy.io as sio

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from ae4803 import FlightModel, AutopilotConfig, AutoStopConfig, replay  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
REF = REPO / "matlab" / "reference" / "ref_run.mat"
OUT = REPO / "compare_out"

# Simulink logged-signal name -> attribute on ae4803.model.Outputs
SIGNAL_MAP = {
    "phi__deg_": "phi_deg",
    "theta__deg_": "theta_deg",
    "psi__deg_": "psi_deg",
    "u__m_s_": "u",
    "v__m_s_": "v",
    "w__m_s_": "w",
    "U__m_s_": "U",
    "alt__m_": "alt_m",
    "x__m_": "x",
    "y__m_": "y",
    "p__deg_s_": "p_dps",
    "q__deg_s_": "q_dps",
    "r__deg_s_": "r_dps",
    "alpha__deg_": "alpha_deg",
    "beta__deg_": "beta_deg",
    "hdot__ft_min_": "hdot_ft_min",
    "elevator__deg_": "elevator_deg",
    "aileron__deg_": "aileron_deg",
    "rudder__deg_": "rudder_deg",
    "engineSpeed__rpm_": "rpm",
}

#: Physically meaningful scale for each signal, used to report a normalised
#: error that is comparable across quantities with different units.
SCALE = {
    "phi_deg": 10.0, "theta_deg": 10.0, "psi_deg": 10.0,
    "u": 45.0, "v": 5.0, "w": 10.0, "U": 45.0,
    "alt_m": 100.0, "x": 2700.0, "y": 100.0,
    "p_dps": 10.0, "q_dps": 10.0, "r_dps": 10.0,
    "alpha_deg": 10.0, "beta_deg": 5.0,
    "hdot_ft_min": 1000.0,
    "elevator_deg": 10.0, "aileron_deg": 10.0, "rudder_deg": 10.0,
    "rpm": 2300.0,
}


def run_python(t_end: float, rate_hz: float):
    model = FlightModel(
        autopilot=AutopilotConfig(wing_leveler_on=False, altitude_hold_on=False),
        # The reference run completed 60 s, so auto-stop never fired; disable it
        # here too so a marginal threshold cannot truncate the comparison.
        autostop=AutoStopConfig(enabled=False),
    )
    src = replay.InterpolatedInput(t_end)
    h = 1.0 / rate_hz
    n = int(round(t_end / h))

    state = model.reset()
    rec: dict[str, list[float]] = {k: [] for k in set(SIGNAL_MAP.values())}
    times: list[float] = []

    for k in range(n + 1):
        t = k * h
        pilot = src(t)
        model.derivative(t, state, pilot, record=True)
        o = model.out
        times.append(t)
        for attr in rec:
            rec[attr].append(float(getattr(o, attr)))
        if k < n:
            state = model.step(t, state, h, pilot)

    return np.asarray(times), {k: np.asarray(v) for k, v in rec.items()}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=200.0, help="Python fixed step rate [Hz]")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()

    if not REF.is_file():
        raise SystemExit(f"missing {REF}\nRun matlab -batch \"addpath('matlab'); make_reference\" first.")

    md = sio.loadmat(str(REF), squeeze_me=False, struct_as_record=False)
    ref = md["ref"][0, 0]
    t_end = float(np.squeeze(md["tEnd"]))
    solver = str(np.squeeze(md["solver"]))
    maxstep = str(np.squeeze(md["maxstep"]))

    print(f"Simulink reference: solver={solver}  MaxStep={maxstep}  t_end={t_end:g} s")
    print(f"Python: fixed-step RK4 at {args.rate:g} Hz\n")

    t_py, py = run_python(t_end, args.rate)

    rows = []
    for sl_name, attr in SIGNAL_MAP.items():
        if not hasattr(ref, sl_name):
            print(f"  (reference has no signal {sl_name}, skipped)")
            continue
        arr = np.asarray(getattr(ref, sl_name), dtype=float)
        t_ref, y_ref = arr[:, 0], arr[:, 1]

        # Compare on the Simulink time grid (it is the variable-step one).
        y_py = np.interp(t_ref, t_py, py[attr])
        err = y_py - y_ref
        sc = SCALE.get(attr, max(float(np.max(np.abs(y_ref))), 1e-9))
        rows.append((attr, float(np.max(np.abs(err))), float(np.sqrt(np.mean(err ** 2))),
                     float(np.max(np.abs(err)) / sc * 100.0),
                     float(np.max(np.abs(y_ref)))))

    rows.sort(key=lambda r: -r[3])
    print(f"{'signal':<16}{'max|err|':>12}{'rms err':>12}{'max err %scale':>16}{'max|ref|':>12}")
    print("-" * 68)
    for name, mx, rms, pct, refmax in rows:
        print(f"{name:<16}{mx:12.5g}{rms:12.5g}{pct:15.3f}%{refmax:12.5g}")

    worst = max(r[3] for r in rows) if rows else 0.0
    print("-" * 68)
    print(f"worst normalised error: {worst:.3f}% of signal scale")

    OUT.mkdir(exist_ok=True)
    with open(OUT / "difference_table.txt", "w", encoding="utf-8") as fh:
        fh.write(f"Simulink: solver={solver} MaxStep={maxstep} t_end={t_end:g}s\n")
        fh.write(f"Python:   fixed-step RK4 @ {args.rate:g} Hz\n\n")
        fh.write(f"{'signal':<16}{'max|err|':>12}{'rms err':>12}{'max err %scale':>16}{'max|ref|':>12}\n")
        for name, mx, rms, pct, refmax in rows:
            fh.write(f"{name:<16}{mx:12.5g}{rms:12.5g}{pct:15.3f}%{refmax:12.5g}\n")
    print(f"\nwrote {OUT / 'difference_table.txt'}")

    if not args.no_plots:
        make_plots(ref, t_py, py)


def make_plots(ref, t_py, py) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    groups = {
        "attitude": ["phi_deg", "theta_deg", "psi_deg"],
        "rates": ["p_dps", "q_dps", "r_dps"],
        "velocity": ["u", "v", "w", "U"],
        "flow_angles": ["alpha_deg", "beta_deg"],
        "trajectory": ["alt_m", "x", "y", "hdot_ft_min"],
        "controls": ["elevator_deg", "aileron_deg", "rudder_deg", "rpm"],
    }
    inv = {v: k for k, v in SIGNAL_MAP.items()}

    for gname, attrs in groups.items():
        attrs = [a for a in attrs if hasattr(ref, inv.get(a, ""))]
        if not attrs:
            continue
        fig, axes = plt.subplots(len(attrs), 2, figsize=(12, 2.3 * len(attrs)),
                                 squeeze=False, sharex=True)
        for i, a in enumerate(attrs):
            arr = np.asarray(getattr(ref, inv[a]), dtype=float)
            t_ref, y_ref = arr[:, 0], arr[:, 1]
            y_py = np.interp(t_ref, t_py, py[a])
            axes[i][0].plot(t_ref, y_ref, lw=1.4, label="Simulink")
            axes[i][0].plot(t_ref, y_py, lw=1.0, ls="--", label="Python")
            axes[i][0].set_ylabel(a)
            axes[i][0].grid(alpha=.3)
            if i == 0:
                axes[i][0].legend(loc="best", fontsize=8)
            axes[i][1].plot(t_ref, y_py - y_ref, lw=1.0, color="tab:red")
            axes[i][1].set_ylabel("error")
            axes[i][1].grid(alpha=.3)
        axes[-1][0].set_xlabel("t [s]")
        axes[-1][1].set_xlabel("t [s]")
        fig.suptitle(f"Python vs Simulink - {gname}")
        fig.tight_layout()
        p = OUT / f"compare_{gname}.png"
        fig.savefig(p, dpi=110)
        plt.close(fig)
        print(f"wrote {p}")


if __name__ == "__main__":
    main()
