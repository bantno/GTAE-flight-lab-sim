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

## For students: running the simulator

### Every session — three steps

1. **Close TARGET completely.** Check the system tray; it is not enough to
   close the window. While TARGET runs it hides the stick, throttle and pedals
   behind one virtual controller and the sim will see nothing.
2. **Start X-Plane** and load an aircraft (not sitting at the menu).
3. **Double-click `run.bat`.**

That is the whole procedure. `run.bat` finds the Python environment, runs a
pre-flight check, and starts the sim. Close the window when you are done — the
timing statistics print to the console and your flight log is written to
`flightlogs\mmddyy_HHMMSS.mat`.

If anything is wrong, pre-flight says so in plain language before the sim
starts, e.g. *"TARGET is RUNNING"* or *"joystick: 1 control unbound"*. You can
run it on its own at any time:

```bat
python tools\preflight.py
```

### First time on a given Windows account — one extra step

Run **`setup.bat`** once. It builds a private Python environment under your own
user profile and installs the packages. It needs **no administrator rights**,
installs nothing system-wide, and takes a couple of minutes.

> **Important:** the environment lives in *your* user profile
> (`%USERPROFILE%\.venvs\ae4803sim`), so it is per-Windows-account. If the lab
> machine is shared under one login, setup has already been done and students
> only ever touch `run.bat`. If students log in with their own accounts, each
> one runs `setup.bat` once. They also each need their own copy of this folder,
> because the current one lives in a personal OneDrive.

### In the sim

| control | what it does |
|---|---|
| stick left/right | roll |
| stick fore/aft | pitch |
| rudder pedals | yaw |
| right throttle lever | throttle — start around **60 %** for 45 m/s |
| **Wing leveler** checkbox | holds wings level and ball centred |
| **Altitude hold** checkbox | captures the current altitude and holds it |
| **Auto-stop** checkbox | the model's own limits; untick to explore outside them |
| **Pause** | freezes the sim; needed to edit derivatives |
| **Derivatives** tab | change stability derivatives, paused, mid-flight |

Keep the airspeed needle in the **green band** (35–55 m/s). That is where the
Beaver coefficient set is a good fit; outside it you are extrapolating, and the
model's auto-stop fires below 40 kts or above 140 kts.

### If something goes wrong

| symptom | fix |
|---|---|
| "No Python environment found" | run `setup.bat` once |
| "TARGET is RUNNING" | close it from the system tray |
| A control does nothing | `python tools\check_mapping.py`, move everything, see which reads 0 |
| No devices at all | `python tools\list_devices.py` — almost always TARGET still running |
| Aircraft does not move in X-Plane | check X-Plane has an aircraft loaded and is accepting UDP on 49000 |
| Sim stops by itself | auto-stop fired — usually a stall. The console prints the reason |

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
* Packages, all prebuilt wheels: numpy, scipy, **pygame-ce** (SDL 2.32.10 —
  see the pedals warning below; upstream pygame will NOT see the rudder),
  PyQt6, pyqtgraph, h5py, matplotlib, pytest.
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

### The mapping, as measured on this rig (2026-11-15, verified in flight)

| control | device | axis | notes |
|---|---|---|---|
| roll | `Joystick - HOTAS Warthog` | 0 | full travel ±0.99 |
| pitch | `Joystick - HOTAS Warthog` | 1 | full travel ±0.99 |
| yaw | `T-Pendular-Rudder` | 2 | `invert = true`; axes 0/1 are the toe brakes, resting at +1.0 |
| throttle | `Throttle - HOTAS Warthog` | 2 | right lever, `unipolar` |

> ### ⚠ The rudder pedals require pygame-**ce**, not pygame
>
> The pedals on this rig are a **T-Pendular-Rudder** (TPR, `VID_044F&PID_B68F`),
> not the TFRP named in the lab notes. Upstream `pygame` 2.6.1 bundles
> **SDL 2.28.4, which does not enumerate this device at all** — it is simply
> absent from `pygame.joystick.get_count()`, while Windows, `joy.cpl` and
> Device Manager all show it as present and working. No SDL hint changes this
> (`SDL_JOYSTICK_RAWINPUT`, `_HIDAPI`, `_WGI`, `_ALLOW_BACKGROUND_EVENTS` were
> all tried).
>
> `pygame-ce` 2.5.8 bundles **SDL 2.32.10**, which sees it. The two packages are
> drop-in compatible (both `import pygame`), so the only change is the
> requirement. **Do not "fix" a dependency warning by installing upstream
> pygame — the rudder will silently disappear.**

### If the axes look wrong

```bat
python tools\check_mapping.py      # shows the actual pilot commands, live
python tools\calibrate_axes.py     # raw per-axis bars, if you need them
```

`check_mapping.py` is the better one: it shows what the model actually
receives, after name matching, inversion, calibration and dead zones, and
reports the travel each command reached so a control stuck at 0 is obvious.

Note the sign convention: a **positive yaw command yaws the nose left** in this
coefficient set, so the right pedal must produce a *negative* yaw command —
hence `invert = true` on the yaw axis. (See `docs/MODEL_NOTES.md` §3.)

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

## ⚠ This sim deliberately differs from `AE4803Sim.slx` in one place

**The inertia tensor.** The `.slx` mask holds an *asymmetric* matrix —
`+117.64` at (1,3), `−117.64` at (3,1). No rigid body has an asymmetric inertia
tensor (`I_ij = I_ji` identically), no sign convention can produce one, and the
6DOF block neither repairs it nor ignores it. This sim therefore defaults to a
**symmetric** tensor instead.

The cost: **Python and the Simulink model now disagree by about 1 % of signal
scale in roll, yaw and sideslip.** That is intended. If students compare the
two, tell them why.

* To reproduce the Simulink model exactly: `inertia_mode = "simulink"` in
  `config/sim.toml`.
* Anything validating the port against Simulink **must** use that mode.
  `tools/replay_compare.py` and `tools/convergence_check.py` already do, and
  still report 0.56 % worst error.
* The *sign* of the symmetric value is **not confirmed** — `symmetric_neg`
  follows the FDC moment-equation convention, but that has not been checked
  against Rauw's thesis, and the sign is worth ~2 %, i.e. more than the
  asymmetry it fixes. If the thesis disagrees, switch to `symmetric_pos`.

The bug is **upstream, not the lab's**: MathWorks' own Aerospace Blockset ships
the same matrix in `astDehavillandBeaver.m`, and pasted it verbatim into the
unrelated fictional `astSkyHogg.m`. Full evidence in `docs/MODEL_NOTES.md` §2.1.

## Other known issues in the original model

Reproduced bug-for-bug by default, each with a config switch to correct it.

1. **The 6DOF block uses the transpose of the mask inertia matrix** — invisible
   for any proper symmetric inertia, visible there because the mask is
   asymmetric. Only relevant in `inertia_mode = "simulink"`.
2. **The auto-stop altitude test `abs(alt_ft) < 0` can never fire**, so flying
   into the ground never stops the sim. `fix_altitude_test = true` corrects it.
3. The actuator state-space models in `sim_init.mat` are **not in the signal
   path** — the `Actuators` subsystem is a pass-through.
