# AE4803 Flight Simulator — Python port

A standalone Python real-time simulator reproducing `AE4803Sim.slx` (the FDC /
Delft DHC-2 Beaver model), built to fix two problems with the Simulink lab:

1. **No TARGET.** The joystick, throttle and pedals are read directly through
   SDL and bound **by device name**, so a reboot can no longer swap the axes and
   there is no virtual-device layer to silently stop updating. A watchdog warns
   if an axis freezes while others move — the old throttle fault.
2. **Real time, properly.** A fixed-step RK4 integrator scheduled against the
   wall clock, with the physics in its own **process** so drawing cannot delay
   it. Measured at 200 Hz with the full GUI running: **0 overruns, p99 lateness
   63 µs, real-time factor 1.0002.**

The Simulink model is untouched and remains the authority for the course. This
repo never writes to the lab folder.

---

## Quick start

```bat
cd "%USERPROFILE%\OneDrive - Georgia Institute of Technology\Desktop\FLIGHT-SIM\AE4803 PySim"

REM 1. Close TARGET completely.
REM 2. Check the sim can see the hardware:
"%USERPROFILE%\.venvs\ae4803sim\python.exe" tools\list_devices.py

REM 3. Fly:
"%USERPROFILE%\.venvs\ae4803sim\python.exe" run_sim.py
```

`run_sim.py` opens the gauge window, starts the physics process, reads the
joystick and streams to X-Plane. Closing the window stops the sim, prints the
timing statistics and writes a flight log.

### Options

| flag | effect |
|---|---|
| `--no-gui` | headless; prints timing on exit |
| `--scripted` | fly the built-in doublet profile, no hardware needed |
| `--no-xplane` | don't send UDP |
| `--no-log` | don't write a flight log |
| `--rate 100` | override the step rate |
| `--duration 120` | stop after 120 s of sim time |
| `--threaded` | physics on a thread instead of a process (worse timing) |
| `--devices` | list joysticks and exit |

---

## Environment (no admin rights needed)

Already set up on this machine:

* **Python 3.12.13** at `%USERPROFILE%\.venvs\ae4803sim\python.exe`, created
  with the existing per-user miniconda (`%LOCALAPPDATA%\miniconda3`).
  Python 3.14 is also installed but **pygame has no 3.14 wheel**, which is why
  the env is 3.12.
* Packages, all prebuilt wheels: numpy, scipy, pygame (SDL 2.28.4), PyQt6,
  pyqtgraph, h5py, matplotlib, pytest.
* The venv lives **outside** this folder on purpose — it is ~20 000 files and
  OneDrive would sync every one of them.

To recreate it:

```bat
"%LOCALAPPDATA%\miniconda3\Scripts\conda.exe" create -p "%USERPROFILE%\.venvs\ae4803sim" python=3.12 -y
"%USERPROFILE%\.venvs\ae4803sim\python.exe" -m pip install -r requirements.txt
```

Nothing here needs administrator rights, a driver install, or a system service.
Two optional per-user Windows tweaks are attempted and are not required:
`timeBeginPeriod(1)` (works) and `HIGH_PRIORITY_CLASS` (currently **refused** on
this machine — the sim prints a note and runs at NORMAL priority with no
measurable penalty). `REALTIME_PRIORITY_CLASS` needs admin and is never
requested.

---

## New lab procedure

1. **Close TARGET.** Fully — check the system tray. While TARGET runs it hides
   the three physical Thrustmaster devices behind one virtual controller, and
   SDL will not see them.
2. **Start X-Plane** and confirm the UDP plugin is loaded. The sim sends VEHX +
   DREF packets to `127.0.0.1:49000`, which is what `AE4803Sim.slx` sends.
3. **Run `run_sim.py`.** It prints which device each control bound to. If a
   control says `UNBOUND`, run `tools\list_devices.py`.
4. **Fly.** Toggle the wing leveler / altitude hold with the check boxes.
5. **Close the window.** Timing statistics print to the console and a log is
   written to `flightlogs\mmddyy_HHMMSS.mat`.

### If the axes look wrong

```bat
python tools\calibrate_axes.py
```

Move one control at a time; the row that moves is that control's axis. Put the
device name and axis number into `config\joystick.toml`. Matching is by name,
so the binding survives a reboot.

Note the sign convention: a **positive yaw command yaws the nose left** in this
coefficient set, so the right pedal must produce a *negative* yaw command. Set
`invert` accordingly. (See `docs/MODEL_NOTES.md` §3.)

---

## Configuration

* `config/sim.toml` — step rate, autopilots, auto-stop, X-Plane, logging, wind,
  and `inertia_mode`.
* `config/joystick.toml` — device names, axis numbers, inversion, calibrated
  ranges, dead zones (the ±0.01 from `sim_init.mat`), and the stale-axis
  watchdog.

All model parameters come from `data/sim_init.mat`, a verbatim copy of the lab's
file, so both sims start from identical numbers.

---

## Flight logs

Written to `flightlogs/` with the same `mmddyy_HHMMSS.mat` name the Simulink
`StopFcn` used, containing:

* `sigsOut` — a **1×25 MATLAB struct array** with the same 25 signal names and
  the same `.Name` / `.Values.Time` / `.Values.Data` layout the Simulink
  `Dataset` exposes, so `sigsOut(i).Values.Data` keeps working. (Python cannot
  construct a real `Simulink.SimulationData.Dataset` — that is a MATLAB object.)
* `sigs` — the same signals keyed by valid MATLAB names, easier to use.
* `t`, `extra` (lat/lon/ρ/q̄/dpt/power/…), `logInfo`.
* Every parameter from `sim_init.mat`, as the `StopFcn` saved.

```matlab
[sigs, t, raw] = load_pysim_log('flightlogs/111526_152137.mat');
plot(t, sigs.U__m_s_)
```

---

## Validation against Simulink

```bat
python tools\make_input_profile.py
matlab -batch "addpath('matlab'); make_reference"
python tools\replay_compare.py
```

`make_reference.m` copies the model to a temp folder, loads **that copy**,
swaps the joystick for the recorded input, simulates headless, and closes with
`close_system(model, 0)`. **The lab's `AE4803Sim.slx` is never written to.**

Result against the lab's own solver settings, worst error as a percentage of
each signal's physical scale:

```
worst normalised error: 0.56 %
```

Most states are far better — `u`, `U`, `x`, `alt` agree to < 0.06 %. The
residual is concentrated in `psi` and `y`, which integrate.

`tools\convergence_check.py` goes further: it compares both the lab's loose
solver (ode23, RelTol 1e-2) and Python at 100–1000 Hz against a tight-tolerance
Simulink run (ode45, RelTol 1e-10). This is how the inertia-transposition bug
was found — before the fix the Python error *plateaued* at ~3 % regardless of
step size, which is the signature of a model difference rather than integration
error.

Comparison plots land in `compare_out/`.

---

## Testing

```bat
python -m pytest -q
```

42 tests, ~75 s. Covers parameters, atmosphere/gravity against MATLAB
references, quaternion kinematics, the control chain and lookup tables,
aerodynamic sign conventions, trimmed flight, control response directions,
both autopilots, auto-stop, the X-Plane packet layouts over a real loopback
socket, the log format, the stale-axis watchdog, and the real-time loop.

Everything runs offline — no hardware, no X-Plane, no MATLAB. (The two tests
that compare against MATLAB references skip if the reference files are absent.)

The fake X-Plane listener is useful on its own:

```bat
python tools\fake_xplane_listener.py
```

It binds to `127.0.0.1` only, so Windows does not raise a firewall prompt.

---

## Layout

```
ae4803/
  params.py       parameters from sim_init.mat; inertia variants
  environment.py  COESA atmosphere, WGS84 gravity, Dryden wind
  aero.py         datum + actuator increments + body-rate damping
  engine.py       pow/eng — shaft power, dpt, propulsive coefficients
  dynamics.py     6DOF quaternion equations of motion, RK4
  controls.py     command → deflection, lookup tables, PID
  model.py        the assembled model, autopilots, auto-stop
  geodesy.py      Flat Earth to LLA
  realtime.py     fixed-step wall-clock loop, timing statistics
  procsim.py      physics in a separate process (shared-memory seqlock)
  joystick.py     SDL input bound by device name, stale-axis watchdog
  xplane.py       VEHX + DREF packets; legacy string sender
  flightlog.py    .mat logging in the Simulink dataset layout
  replay.py       the shared comparison input profile
  gui/            gauges and strip charts
tools/            device list, axis calibration, UDP listener, comparison
matlab/           reference generation (never writes to the lab folder)
docs/             MODEL_NOTES.md — what the .slx actually does
```

---

## Known issues in the original model

Reproduced bug-for-bug by default so results stay comparable, each with a
config switch to correct it. Full detail and evidence in `docs/MODEL_NOTES.md`.

1. **The inertia matrix is not a valid inertia tensor** — it is not symmetric,
   and integrating with it does not conserve rotational kinetic energy
   (5 × 10⁻³ relative drift over 200 s of torque-free tumbling, against machine
   precision for a symmetric tensor). `inertia_mode = "physical"` fixes it;
   expect ~2 % difference in the lateral states.
2. **The 6DOF block uses the transpose of the mask inertia matrix** — invisible
   for any proper symmetric inertia, visible here because the mask is
   asymmetric. Matched by default.
3. **The auto-stop altitude test `abs(alt_ft) < 0` can never fire**, so flying
   into the ground never stops the sim. `fix_altitude_test = true` corrects it.
4. The actuator state-space models in `sim_init.mat` are **not in the signal
   path** — the `Actuators` subsystem is a pass-through.
