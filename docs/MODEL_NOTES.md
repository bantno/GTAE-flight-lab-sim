# Model notes — what AE4803Sim.slx actually does

Everything here was read out of `AE4803Sim.slx` itself (the `.slx` is a zip;
block contents are in `simulink/systems/system_<SID>.xml`) or measured by
running a **scratch copy** of the model in MATLAB R2024a. The original lab
folder was never modified — every MATLAB script copies the model to a temp
directory, loads that, and closes with `close_system(model, 0)`.

Model release: saved with R2024a (the lab notes say R2022b; the folder also has
`AE4803Sim.slx.r2022b` and `AE4803Sim.slx.original`, which are older revisions).

---

## 1. Corrections to the original brief

These differ from the description I was given. Each was verified directly.

| Item | Brief said | Actually is |
|---|---|---|
| Project location | `D:\AE4803 - Flight Test Engineering…` | There is **no D: drive**. The project is at `…\Desktop\FLIGHT-SIM\AE4803 Flight Simulator` |
| Solver `MaxStep` | 0.01 | **4/100 = 0.04** |
| Real-time pacing | "NO real-time pacing is enabled" | **`EnablePacing = on`, `PacingRate = 1`** — Simulink simulation pacing *is* on at 1×. (The *Simulink Desktop Real-Time* `Real-Time Synchronization` block is present but `Commented = on`.) |
| X-Plane output | `SendPositionOrientationToXPlaneOverUDP` → string → `UWTimedProcessingUDP.xpl` on 49003 | **Not used anywhere in the current `.slx`.** The model sends binary **VEHX + DREF packets to 127.0.0.1:49000**. The old `.m` files are still loose in the folder but nothing calls them. |
| Logged dataset | `SigsOut` | **`sigsOut`** (lower-case s) — `SignalLoggingName = 'sigsOut'` |
| Dryden turbulence | "W20 = 5, light; check whether it's on" | **Switched OFF** (`T_on = off`). With wind shear `W_20 = 0` and all gust axes off, **the wind is identically zero.** |
| Analysis Pane | "six dashboard scopes" | **Two** dashboard scopes (60 s span, ±30 and ±1), 12 edit fields, 2 slider switches |
| Actuator state-space | `Aail/Bail/Cail`, `Aelv/…`, `Arud/…` in the signal path | **Unused.** The `Actuators` subsystem (SID 19) is a plain pass-through (Demux 4 → Mux 4). Commands go straight to deflections through static gains. The matrices sit in `sim_init.mat` unreferenced. |

---

## 2. Two genuine modelling errors in the `.slx`

### 2.1 The inertia matrix is not a valid inertia tensor

The 6DOF (Quaternion) block mask (SID 328) contains

```
[5.787969e3  0 1.1764e2; 0 6.92893e3 0; -1.1764e2 0 1.1578329e4]
```

The two `Ixz` entries have **opposite signs**, so the matrix is not symmetric.
An inertia tensor `I = ∫(r²δ − rr)dm` is symmetric by construction, so this is
not a physically possible rigid body.

It is not cosmetic. Integrating torque-free rotation `ω̇ = I⁻¹(−ω×Iω)` from
`ω₀ = [0.3, 0.2, 0.1]` rad/s for 200 s:

| matrix | symmetric? | rotational KE drift | ‖L‖ drift |
|---|---|---|---|
| mask as typed, `A` | no | **5.0 × 10⁻³** | 2.8 × 10⁻¹⁴ |
| what the block uses, `Aᵀ` | no | **8.6 × 10⁻⁴** | 3.2 × 10⁻¹⁴ |
| symmetric, −Ixz | yes | 6.6 × 10⁻¹⁴ | 3.4 × 10⁻¹⁴ |
| symmetric, +Ixz | yes | 6.6 × 10⁻¹⁴ | 3.1 × 10⁻¹⁴ |

Angular momentum is conserved either way (that is automatic for this equation),
but **rotational kinetic energy is not** — the model is quietly doing work on
itself. Reproduce with `pytest -k conserve`.

Note also that the *symmetric part* of the mask matrix is exactly
`diag(Ixx, Iyy, Izz)` — the two `Ixz` entries cancel — so all of the roll/yaw
inertial coupling lives in the skew part, which is the unphysical part.

### Could this just be a sign convention rather than an error?

No. Checked explicitly, because it is the obvious objection.

Texts genuinely do disagree about products of inertia — some write the tensor
off-diagonal as `−∫xz dm`, others as `+∫xz dm`. But **every one of those
conventions yields a symmetric matrix.** The underlying object obeys
`I_ij = ∫(r²δ_ij − r_i r_j)dm` and `r_i r_j = r_j r_i`, so `I_ij = I_ji`
identically. A convention flips *both* off-diagonal entries together; no
convention can flip one and not the other. `+117.64` at (1,3) with `−117.64` at
(3,1) is not a convention, it is two different conventions applied to the two
halves of one matrix.

**Nor is it harmless.** It would be cosmetic if the block symmetrised the
matrix internally, since `sym(A) = sym(Aᵀ) = diag(Ixx, Iyy, Izz)` here. It does
not: running the model with `A` and with `Aᵀ` gives materially different
answers — 3.2 % on `beta`, 1.7 % on `p`, 1.6 % on `phi` and `psi`. Had the block
symmetrised, those two runs would have been bit-identical.

**Why it matters dynamically.** Torque-free motion is `L̇ = −(I⁻¹L) × L`. Two
inertias `A` and `S` give identical flows for every `L` only if
`(A⁻¹ − S⁻¹)L ∥ L` for all `L`, i.e. `A⁻¹ − S⁻¹ = c·Identity`, which forces `A`
symmetric whenever `S` is. The mask matrix is not symmetric, so **no rigid body
reproduces these rotational dynamics.** That is a proof, not an observation.

(An earlier draft argued from rotational-KE drift. That showed one particular
quantity is not conserved; it did not rule out some *other* conserved energy.
The algebraic argument above is the rigorous one.)

### What is NOT established: the sign

Making the tensor symmetric is clearly right. *Which* symmetric value is a
separate question, and it is **not settled**. The FDC/Delft Beaver moment
equations

```
Ix·ṗ − Jxz·ṙ = L + (Iy−Iz)·q·r + Jxz·p·q
Iz·ṙ − Jxz·ṗ = N + (Ix−Iy)·p·q − Jxz·q·r
```

put `−Jxz` in both slots for `Jxz = +117.64`. But that rests on the convention
`Jxz = ∫xz dm`, and the Beaver's real mass distribution has not been checked
against a primary source. Nothing available locally settles it: `AE4803Sim.slx`,
`.slx.original` and `.slx.r2022b` all carry the **same** asymmetric matrix, and
`Simulator_Assignment_1.pdf` has no inertia data at all.

**And the sign matters more than the bug.** Over the 60 s replay profile, worst
difference as a percentage of signal scale:

| comparison | worst difference |
|---|---|
| `−Jxz` vs `+Jxz` — the sign we are unsure of | **1.97 %** |
| mask (asymmetric) vs symmetric `−Jxz` | 1.09 % |
| dropping `Jxz` entirely vs symmetric `−Jxz` | 0.98 % |

The coupling is weak to begin with: `|Jxz|/√(Ixx·Izz) = 0.0144`. All four
candidates sit within ~2 % of one another, and the sign uncertainty is *larger*
than the asymmetry it is meant to repair. A symmetric tensor is therefore
demonstrably **self-consistent**, but not demonstrably **closer to the real
aircraft**, until the sign is confirmed from the FDC source.

Selectable via `inertia_mode` in `config/sim.toml`: `"simulink"` (default,
bug-for-bug), `"symmetric_neg"`, `"symmetric_pos"`; `"physical"` is an alias
for `symmetric_neg`.

**Recommendation:** keep `simulink` as the course default. It keeps Python and
the `.slx` comparable, and the discrepancy makes a good teaching point. Do not
switch to a symmetric tensor on the strength of this note alone — confirm the
sign first.

(Aside: `Iyy` and `Jxz` match the published FDC Beaver exactly, and so does the
mass, 2288.231 kg. `Ixx` and `Izz` are each exactly 419.579 kg·m² larger than
the published values — a deliberate change by someone, not a typo.)

### 2.2 The 6DOF block uses the **transpose** of the mask matrix

Established experimentally, and the test is symmetric so it is not a fitted
guess. Against a tight-tolerance Simulink run (ode45, RelTol 1e-10), worst
error as % of signal scale:

| | |
|---|---|
| Simulink(mask = `A`) vs Python(I = `A`) | 3.14 % |
| Simulink(mask = `A`) vs Python(I = `Aᵀ`) | **0.57 %** |
| Simulink(mask = `Aᵀ`) vs Python(I = `Aᵀ`) | 3.38 % |
| Simulink(mask = `Aᵀ`) vs Python(I = `A`) | **0.68 %** |

So the block integrates with `Aᵀ`, not `A`. For any *proper* (symmetric)
inertia this transposition is invisible — it only surfaces because this mask
entry happens to be asymmetric. `params.INERTIA_SIMULINK = INERTIA_AS_IN_MASK.T`
reproduces it, and is the default so Python and Simulink stay comparable.

### 2.3 The auto-stop altitude test can never fire

Simulation Auto-Stop (SID 853) ORs six conditions. The fifth is

```
abs(alt_ft) < 0
```

`abs()` is never negative, so **flying into the ground never stops the sim.**
Reproduced faithfully by default; set `fix_altitude_test = true` in
`config/sim.toml` to use the evidently-intended `alt_ft < 0`.

The other five are `|Hdot| ≥ 5000` (m/s, on the raw signal), `|α| ≥ 20°`,
`|U| ≥ 140 kts`, `|U| ≤ 40 kts`, `‖pqr‖ ≥ 1000 rad/s`.

---

## 3. Equations, as wired

Coefficient vector order is `[Cx, Cy, Cz, Cl, Cm, Cn]`; `uaero = [δe, δa, δr, δf]`
in radians (Selector indices 0,1,2,3).

**Datum (SID 134)**
```
Cx = Cx0 + Cx_α·α + Cx_α2·α² + Cx_α3·α³
Cy = Cy0 + Cy_β·β
Cz = Cz0 + Cz_α·α + Cz_α3·α³
Cl = Cl0 + Cl_β·β
Cm = Cm0 + Cm_α·α + Cm_α2·α² + Cm_β2·β²
Cn = Cn0 + Cn_β·β + Cn_β3·β³
```

**Actuator increments (SID 30)** — aileron 34, elevator 49, flap 61, rudder 79
```
aileron : Cy += Cy_da·δa ; Cl += Cl_da·δa + Cl_da_α·δa·α ; Cn += Cn_da·δa
elevator: Cz += Cz_de·δe + Cz_de_β·δe·β² ; Cm += Cm_de·δe
rudder  : Cx += Cx_dr·δr ; Cy += Cy_dr·δr + Cy_dr_α·δr·α ; Cl += Cl_dr·δr ; Cn += Cn_dr·δr
flap    : Cx += Cx_df·δf + Cx_df_α·δf·α ; Cz += Cz_df·δf + Cz_df_α·δf·α ; Cm += Cm_df·δf
```

**Body-rate damping (SID 96)**
```
Cdamp = [ (b/2)·p·Pp + c̄·q·Pq + (b/2)·r·Pr ] / V
Pp = [0, Cy_p, 0, Cl_p, 0, Cn_p]
Pq = [Cx_q, 0, Cz_q, 0, Cm_q, Cn_q]
Pr = [0, Cy_r, 0, Cl_r, Cm_r, Cn_r]
```
Note the pitch term uses the **full chord** `c̄` (the `Reference Length` gain is
`beaver_c`, not `beaver_c/2`), while roll and yaw use `b/2`. Reproduced as-is.

**Engine (SID 164 = pow 187 + eng 169)**
```
P   = 0.7355 · [ −326.5 + 0.00412·(pz + 7.4)·(n + 2010)
                 + (408.0 − 0.0965·n)·(1 − ρ/1.225) ]        [kW]
dpt = 0.08696 + 2·191.18·P / (ρ·V³)
Cprop = [Cx_dpt·dpt + Cx_dpt2_α·dpt²·α, 0, Cz_dpt·dpt,
         Cl_α2_dpt·α²·dpt, Cm_dpt·dpt, Cn_dpt3·dpt³]
```

**Forces and moments (SID 206)** — Aerospace block, body→body, cp = cg = 0
```
F = q̄·S·[Cx, Cy, Cz]
M = q̄·S·[b·Cl, c̄·Cm, b·Cn]
```

**Gravity (SID 216)** — `F_grav_body = DCM · (mass · g_ned)`

**Controls (SID 290)**
```
elevator_deg = de0 + (−20)·sat(pitch_cmd, −1, 1)      de0 = −1.86
aileron_deg  = da0 + (−15)·sat(roll_cmd,  −1, 1)      da0 =  0.0378
rudder_deg   = dr0 + (+25)·sat(yaw_cmd,   −1, 1)      dr0 = −2.44
flap_deg     = lookup([0 1 2 3] → [0 4 10 20], ratelimit(0, ±0.5)) = 0
n  = Throttle2RPM(sat(throttle, 0, 1))
pz = RPM2ManifoldPressure(n)
uaero = deg2rad([elevator, aileron, rudder, flap])
```

### Sign convention worth knowing

A **positive yaw command** deflects the rudder positive, and with `Cn_dr < 0`,
`Cy_dr > 0` that yaws the nose **left** with positive sideslip (and the dihedral
effect `Cl_β < 0` then rolls it left). So **the right pedal must map to a
NEGATIVE yaw command** for the nose to go right. Confirm this during the
hardware check and set `invert` in `config/joystick.toml` accordingly.

---

## 4. Signal definitions that are easy to get wrong

* **Two different α and β.** The *aerodynamic* pair (radians) comes from the
  Incidence/Sideslip/Airspeed block fed with `Vb − uvw_wind`. The *logged and
  autopilot* pair (degrees) comes from Data Routing and uses `Vb` alone:
  `α = atan2(w, u)`, `β = asin(v/|Vb|)`. With zero wind they agree.
* **Dynamic pressure uses the earth-frame velocity.** The Dynamic Pressure
  block (SID 239) is fed `Ve` (ACBus signal 2), not the body relative velocity.
  Same magnitude at zero wind; reproduced faithfully anyway.
* **`LongLat0` is [longitude, latitude]** = `[-84.394679, 33.772201]`, and the
  Flat Earth to LLA block reverses it (`LL0 = [LongLat0(2) LongLat0(1)]`).
* **Gravity uses a fixed latitude of 45.5°**, hard-coded by two Constant blocks
  — not the aircraft's actual latitude. Model: *WGS84 Taylor Series*,
  `no_centrifugal = on`, output `[0, 0, g]` in NED.
* **MATLAB's `atmoscoesa` does not convert geometric → geopotential altitude.**
  It applies the lapse rates to the input directly. `atmoscoesa(914.4)` gives
  `T = 282.2064 K = 288.15 − 0.0065·914.4` exactly; converting first would give
  `282.2073`. We match MATLAB, not the textbook (difference at the lab altitude:
  0.0009 K, 1.4e-5 kg/m³).

---

## 5. X-Plane packet formats (verbatim)

**VEHX** — MATLAB Function `VEHA1` (chart 500) + `etargetslib/Byte Pack`
(`datatypes = {'uint8','uint32','double','single'}`, `byteAlign = 1`):

```
b'VEHX0'            5 B   fifth byte is ASCII '0' (0x30), NOT a NUL
uint32 index        4 B   0 = user aircraft
double lat,lon,alt 24 B   degrees, degrees, metres
float  psi,the,phi 12 B   heading, pitch, roll — degrees
                   ----
                   45 B   little-endian, tightly packed
```

The angles **are reordered** inside the block: the subsystem input is
`[lat lon alt roll pitch heading …]`, and Mux SID 1581 feeds VEHX in the order
`lat1, lon1, msl1, psi1, the1, phi1`.

**DREF** — chart 32 and siblings:

```
b'DREF0'                         5 B
float  value                     4 B
dataref path + b'\x00' + spaces  500 B
                                 ----
                                 509 B
```

Datarefs driven: `sim/flightmodel2/controls/{roll,pitch,heading,speedbrake}_ratio`,
`sim/flightmodel/controls/flaprqst`, plus `sim/multiplayer/...` for AI aircraft.

Destination `127.0.0.1:49000`, `num_ai = 1`, `lag = off`.

The legacy string sender is implemented too (`protocol = "legacy_string"`,
port 49003) in case the older `UWTimedProcessingUDP.xpl` plugin is what is
actually loaded in X-Plane.

---

## 6. The 25 logged signals

Read out of a real lab log (`flightlogs/101626_101022.mat`) in MATLAB, in order:

```
stick_lat_[-1:1]  stick_long_[-1:1]  pedal_[-1:1]  throttle_[0:1]
rudder_[deg]  aileron_[deg]  elevator_[deg]  engineSpeed_[rpm]
phi_[deg]  psi_[deg]  theta_[deg]
u_[m/s]  v_[m/s]  w_[m/s]
alt_[m]  x_[m]  y_[m]
p_[deg/s]  q_[deg/s]  r_[deg/s]
Ve_[m/s] (3 cols)  hdot_[ft/min]  alpha_[deg]  beta_[deg]  U_[m/s]
```

The first four run at the joystick's 1/30 s rate; the rest at the solver rate.
`ae4803/flightlog.py` writes exactly these names, in this order.
