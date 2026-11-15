"""Flight logging to .mat, compatible with the existing MATLAB post-processing.

The Simulink model's StopFcn is

    save([pwd '/flightlogs/' datestr(now,'mmddyy_HHMMSS') '.mat']);

which dumps the whole base workspace: every sim_init.mat parameter plus the
logged dataset `sigsOut` (note the lower-case s - the config parameter is
SignalLoggingName = 'sigsOut', even though the lab notes call it SigsOut).

`sigsOut` is a Simulink.SimulationData.Dataset, a MATLAB object that only
Simulink can reconstruct. scipy cannot write one. So this module writes:

  * `sigsOut`  - a struct array with the same .Name / .Values.Time / .Values.Data
                 layout the Dataset exposes, so `sigsOut(i).Name` and
                 `sigsOut(i).Values.Data` keep working;
  * `sigs`     - the same signals as a plain struct keyed by a valid MATLAB
                 name, which is far easier to work with;
  * `t`        - the time vector;
  * every parameter from sim_init.mat, exactly as the StopFcn did.

Signal names and units are the 25 the model logs, verified by reading an actual
lab flight log (flightlogs/101626_101022.mat) in MATLAB.

Call `sigsOut_to_struct` in MATLAB if an old script indexes the Dataset through
getElement(); see matlab/load_pysim_log.m.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import scipy.io as sio

#: (log name as Simulink writes it, Outputs attribute, number of columns)
#: Order matches the dataset in a real lab log.
SIGNALS: tuple[tuple[str, str, int], ...] = (
    ("stick_lat_[-1:1]", "stick_lat", 1),
    ("stick_long_[-1:1]", "stick_long", 1),
    ("pedal_[-1:1]", "pedal", 1),
    ("throttle_[0:1]", "throttle_in", 1),
    ("rudder_[deg]", "rudder_deg", 1),
    ("aileron_[deg]", "aileron_deg", 1),
    ("elevator_[deg]", "elevator_deg", 1),
    ("engineSpeed_[rpm]", "rpm", 1),
    ("phi_[deg]", "phi_deg", 1),
    ("psi_[deg]", "psi_deg", 1),
    ("theta_[deg]", "theta_deg", 1),
    ("u_[m/s]", "u", 1),
    ("v_[m/s]", "v", 1),
    ("w_[m/s]", "w", 1),
    ("alt_[m]", "alt_m", 1),
    ("x_[m]", "x", 1),
    ("y_[m]", "y", 1),
    ("p_[deg/s]", "p_dps", 1),
    ("q_[deg/s]", "q_dps", 1),
    ("r_[deg/s]", "r_dps", 1),
    ("Ve_[m/s]", "Ve", 3),
    ("hdot_[ft/min]", "hdot_ft_min", 1),
    ("alpha_[deg]", "alpha_deg", 1),
    ("beta_[deg]", "beta_deg", 1),
    ("U_[m/s]", "U", 1),
)

#: Extra channels the Simulink model does not log but which are useful. They go
#: into `extra` so they cannot collide with anything a lab script expects.
EXTRA_SIGNALS: tuple[tuple[str, str], ...] = (
    ("lat_[deg]", "lat"),
    ("lon_[deg]", "lon"),
    ("rho_[kg/m3]", "rho"),
    ("qbar_[Pa]", "qbar"),
    ("dpt", "dpt"),
    ("power_[kW]", "power_kW"),
    ("manifoldPressure", "manifold_pressure"),
    ("flap_[deg]", "flap_deg"),
    ("cmd_roll", "cmd_roll"),
    ("cmd_pitch", "cmd_pitch"),
    ("cmd_yaw", "cmd_yaw"),
    ("cmd_throttle", "cmd_throttle"),
)


def matlab_safe(name: str) -> str:
    """Same transformation MATLAB's matlab.lang.makeValidName applies."""
    out = []
    for ch in name:
        out.append(ch if (ch.isalnum() or ch == "_") else "_")
    s = "".join(out)
    if not s or not s[0].isalpha():
        s = "x" + s
    return s


@dataclass
class FlightLogger:
    """Accumulates a run in memory, then writes it as a .mat."""
    log_dir: Path = field(default_factory=lambda: Path("flightlogs"))
    params_mat: Path | None = None
    max_samples: int | None = None
    decimate: int = 1

    _t: list[float] = field(default_factory=list, init=False)
    _rows: dict[str, list] = field(default_factory=dict, init=False)
    _n: int = field(default=0, init=False)

    def __post_init__(self):
        self.log_dir = Path(self.log_dir)
        for name, _, _ in SIGNALS:
            self._rows[name] = []
        for name, _ in EXTRA_SIGNALS:
            self._rows[name] = []

    def record(self, out) -> None:
        if self.decimate > 1 and (self._n % self.decimate):
            self._n += 1
            return
        self._n += 1
        if self.max_samples is not None and len(self._t) >= self.max_samples:
            return
        self._t.append(float(out.t))
        for name, attr, width in SIGNALS:
            v = getattr(out, attr)
            self._rows[name].append(np.asarray(v, dtype=float).copy()
                                    if width > 1 else float(v))
        for name, attr in EXTRA_SIGNALS:
            self._rows[name].append(float(getattr(out, attr)))

    @property
    def n_samples(self) -> int:
        return len(self._t)

    def _dataset(self):
        """Build a 1xN MATLAB STRUCT ARRAY standing in for Simulink's Dataset.

        A numpy structured array (rather than an object array of dicts) is what
        savemat turns into a real struct array, so MATLAB can do
        ``sigsOut(i).Name`` and ``sigsOut(i).Values.Data`` exactly as it does
        with the Dataset's elements.
        """
        t = np.asarray(self._t, dtype=float).reshape(-1, 1)
        dt = [("Name", "O"), ("BlockPath", "O"), ("Values", "O")]
        arr = np.zeros((1, len(SIGNALS)), dtype=dt)
        for i, (name, _, width) in enumerate(SIGNALS):
            data = np.asarray(self._rows[name], dtype=float)
            if width == 1:
                data = data.reshape(-1, 1)
            vdt = np.zeros((1, 1), dtype=[("Time", "O"), ("Data", "O")])
            vdt[0, 0]["Time"] = t
            vdt[0, 0]["Data"] = data
            arr[0, i]["Name"] = name
            arr[0, i]["BlockPath"] = "AE4803 PySim"
            arr[0, i]["Values"] = vdt
        return arr

    def save(self, filename: str | None = None,
             timestamp: time.struct_time | None = None,
             metadata: dict | None = None) -> Path:
        """Write the log. Filename defaults to the model's mmddyy_HHMMSS.mat."""
        ts = timestamp or time.localtime()
        if filename is None:
            filename = time.strftime("%m%d%y_%H%M%S.mat", ts)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        path = self.log_dir / filename

        t = np.asarray(self._t, dtype=float).reshape(-1, 1)
        payload: dict[str, object] = {
            "t": t,
            "sigsOut": self._dataset(),
        }

        sigs: dict[str, np.ndarray] = {}
        for name, _, width in SIGNALS:
            data = np.asarray(self._rows[name], dtype=float)
            sigs[matlab_safe(name)] = data if width > 1 else data.reshape(-1, 1)
        payload["sigs"] = sigs

        extra: dict[str, np.ndarray] = {}
        for name, _ in EXTRA_SIGNALS:
            extra[matlab_safe(name)] = np.asarray(self._rows[name],
                                                  dtype=float).reshape(-1, 1)
        payload["extra"] = extra

        payload["logInfo"] = {
            "source": "AE4803 PySim",
            "created": time.strftime("%Y-%m-%d %H:%M:%S", ts),
            "nSamples": float(len(self._t)),
            **{k: v for k, v in (metadata or {}).items()},
        }

        # Reproduce the StopFcn's habit of saving every model parameter too, so
        # existing scripts that expect e.g. beaver_S in the log still find it.
        if self.params_mat is not None and Path(self.params_mat).is_file():
            try:
                raw = sio.loadmat(str(self.params_mat), squeeze_me=True,
                                  struct_as_record=False)
                for k, v in raw.items():
                    if not k.startswith("__") and k not in payload:
                        payload[k] = v
            except Exception:
                pass

        sio.savemat(str(path), payload, format="5", do_compression=True,
                    oned_as="column")
        return path
