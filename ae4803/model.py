"""The complete AE4803 flight simulator model.

Assembles the Simulink block diagram into one continuous-time ODE:

    Controls (290) -> Total Forces and Moments (14) -> Aircraft Dynamics (4)
                       ^                                      |
                       +------ Environment (245) <------------+

State vector = 13 rigid-body states (see dynamics.py) + 5 autopilot
controller states (see controls.py).

Signal conventions follow the model exactly:
  * The aerodynamic alpha/beta/V come from the Incidence, Sideslip & Airspeed
    block fed with (Vb - uvw_wind) - radians.
  * The logged/autopilot alpha/beta come from the Data Routing block and are in
    DEGREES, computed from Vb alone: alpha = atan2(w, u), beta = asin(v/|Vb|).
  * Dynamic pressure uses the EARTH-frame velocity magnitude (see aero.py).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from . import aero, controls, dynamics, engine, environment
from .controls import PilotCommands, SurfaceState
from .geodesy import FlatEarthToLLA
from .params import Params

MPS_TO_KTS = 1.943845249222      # Velocity Conversion, m/s -> kts
MPS_TO_FTMIN = 196.850393700787  # Velocity Conversion1, m/s -> ft/min
M_TO_FT = 3.280839895013123      # Length Conversion, m -> ft

N_STATES = dynamics.N_STATES + controls.N_AP_STATES
IDX_AP = slice(dynamics.N_STATES, N_STATES)


@dataclass
class AutopilotConfig:
    wing_leveler_on: bool = False
    altitude_hold_on: bool = False
    #: "capture" latches the altitude reference the moment the hold is engaged
    #: (what the Memory+Switch circuit is built to do, and what a pilot expects).
    #: "simulink_memory" reproduces the Memory block's 0 initial condition, which
    #: makes the reference 0 ft if the hold is already on at t = 0.
    altitude_ref_mode: str = "capture"


@dataclass
class AutoStopConfig:
    """Simulation Auto-Stop (SID 853). OR of six conditions.

    Thresholds and operators exactly as in the model:
        |Hdot| >= 5000      (Hdot in m/s - note: NOT ft/min, the raw signal)
        |alpha_deg| >= 20
        |U_kts| >= 140
        |U_kts| <= 40
        |alt_ft| < 0        <- can never fire; abs() is never negative
        |pqr| >= 1000       (vector norm, rad/s)
    """
    enabled: bool = True
    hdot_limit: float = 5000.0
    alpha_deg_limit: float = 20.0
    speed_kts_max: float = 140.0
    speed_kts_min: float = 40.0
    pqr_norm_limit: float = 1000.0
    #: The model's altitude test is `abs(alt_ft) < 0`, which is unreachable, so
    #: a crash never stops the sim. Set True to use the evidently-intended
    #: `alt_ft < 0` instead. Default False = bug-for-bug compatible.
    fix_altitude_test: bool = False


@dataclass
class Outputs:
    """Everything the gauges, logger and X-Plane link need, per step."""
    t: float = 0.0
    # attitude
    phi_deg: float = 0.0
    theta_deg: float = 0.0
    psi_deg: float = 0.0
    # body velocity
    u: float = 0.0
    v: float = 0.0
    w: float = 0.0
    U: float = 0.0          # |Vb| [m/s]
    U_kts: float = 0.0
    # body rates [deg/s]
    p_dps: float = 0.0
    q_dps: float = 0.0
    r_dps: float = 0.0
    # flow angles [deg] (Data Routing definitions)
    alpha_deg: float = 0.0
    beta_deg: float = 0.0
    # position / altitude
    x: float = 0.0
    y: float = 0.0
    alt_m: float = 0.0
    alt_ft: float = 0.0
    hdot_ft_min: float = 0.0
    hdot_mps: float = 0.0
    Ve: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # geodetic
    lat: float = 0.0
    lon: float = 0.0
    # environment
    rho: float = 0.0
    qbar: float = 0.0
    g: float = 0.0
    # controls / propulsion
    elevator_deg: float = 0.0
    aileron_deg: float = 0.0
    rudder_deg: float = 0.0
    flap_deg: float = 0.0
    rpm: float = 0.0
    manifold_pressure: float = 0.0
    dpt: float = 0.0
    power_kW: float = 0.0
    # pilot commands actually applied (after autopilot switching)
    cmd_roll: float = 0.0
    cmd_pitch: float = 0.0
    cmd_yaw: float = 0.0
    cmd_throttle: float = 0.0
    # raw joystick commands (logged as stick_lat/long/pedal/throttle)
    stick_lat: float = 0.0
    stick_long: float = 0.0
    pedal: float = 0.0
    throttle_in: float = 0.0
    # forces / moments (useful for debugging, not logged by Simulink)
    force_b: np.ndarray = field(default_factory=lambda: np.zeros(3))
    moment_b: np.ndarray = field(default_factory=lambda: np.zeros(3))
    autostop: bool = False
    autostop_reason: str = ""


class FlightModel:
    """Continuous-time model, integrated by the caller with a fixed-step RK4."""

    def __init__(self, params: Params | None = None,
                 autopilot: AutopilotConfig | None = None,
                 autostop: AutoStopConfig | None = None,
                 wind: environment.WindModel | None = None):
        self.p = params or Params.load()
        self.ap = autopilot or AutopilotConfig(
            wing_leveler_on=self.p.wing_leveler_on,
            altitude_hold_on=self.p.altitude_hold_on,
        )
        self.autostop_cfg = autostop or AutoStopConfig()
        self.wind = wind or environment.WindModel(turbulence_on=False)
        self.lla = FlatEarthToLLA.from_params(self.p.trim.LongLat0)

        self._alt_ref_ft: float | None = None
        self._prev_hold = self.ap.altitude_hold_on
        self.out = Outputs()

    # --- state ------------------------------------------------------------
    def initial_state(self) -> np.ndarray:
        s = np.zeros(N_STATES)
        s[: dynamics.N_STATES] = dynamics.make_initial_state(
            self.p.trim.xme_0, self.p.trim.uvw0, self.p.trim.euler0, self.p.trim.pqr0)
        return s

    def reset(self) -> np.ndarray:
        self._alt_ref_ft = None
        self._prev_hold = self.ap.altitude_hold_on
        self.wind.reset()
        return self.initial_state()

    # --- kinematic / derived signals --------------------------------------
    def _derived(self, state: np.ndarray):
        vb = state[dynamics.IDX_VB]
        q = state[dynamics.IDX_Q]
        w_body = state[dynamics.IDX_W]
        xe = state[dynamics.IDX_XE]

        dcm = dynamics.dcm_from_quat(q)
        ve = dcm.T @ vb
        euler = dynamics.euler_from_dcm(dcm)
        xyh = xe * np.array([1.0, 1.0, -1.0])        # xyH gain [1 1 -1]
        alt_m = float(xyh[2])
        hdot = -float(ve[2])                          # Demux + Gain -1
        return dcm, ve, euler, xyh, alt_m, hdot

    # --- autopilots -------------------------------------------------------
    def _autopilot(self, ap_state: np.ndarray, phi_deg: float, theta_deg: float,
                   beta_deg: float, alt_ft: float, u_mps: float,
                   pilot: PilotCommands):
        """Returns (applied commands, d/dt of the autopilot states)."""
        d = np.zeros(controls.N_AP_STATES)
        cmd = PilotCommands(pilot.roll, pilot.pitch, pilot.yaw, pilot.throttle)

        # --- Wing Leveler (SID 1058): phi -> roll, beta -> yaw ---
        e_phi = 0.0 - phi_deg                        # phi_ref = 0
        e_beta = 0.0 - beta_deg                      # beta_ref = 0
        _, d[0] = controls.WL_PHI.derivatives(e_phi, 0.0, ap_state[0])
        _, d[1] = controls.WL_BETA.derivatives(e_beta, 0.0, ap_state[1])
        if self.ap.wing_leveler_on:
            cmd.roll = controls.WL_PHI.output(e_phi, 0.0, ap_state[0])
            cmd.yaw = controls.WL_BETA.output(e_beta, 0.0, ap_state[1])

        # --- Altitude Hold (SID 1108) ---
        # Memory + Switch latch: capture the reference as the hold engages.
        if self.ap.altitude_hold_on and not self._prev_hold:
            self._alt_ref_ft = (0.0 if self.ap.altitude_ref_mode == "simulink_memory"
                                else alt_ft)
        if not self.ap.altitude_hold_on:
            self._alt_ref_ft = None
        self._prev_hold = self.ap.altitude_hold_on

        alt_ref = self._alt_ref_ft if self._alt_ref_ft is not None else alt_ft
        e_alt = alt_ref - alt_ft
        d[2], d[3] = controls.AH_ALT.derivatives(e_alt, ap_state[2], ap_state[3])
        theta_cmd = controls.AH_ALT.output(e_alt, ap_state[2], ap_state[3])
        theta_cmd = float(np.clip(theta_cmd, -controls.AH_PITCH_LIMIT,
                                  controls.AH_PITCH_LIMIT))

        e_theta = theta_cmd - theta_deg
        d[4], _ = controls.AH_PITCH.derivatives(e_theta, ap_state[4], 0.0)
        pitch_ap = controls.AH_PITCH.output(e_theta, ap_state[4], 0.0)

        e_u = controls.AH_SPEED_REF - u_mps
        throttle_ap = controls.AH_SPEED.output(e_u, 0.0, 0.0)

        if self.ap.altitude_hold_on:
            cmd.pitch = pitch_ap
            cmd.throttle = throttle_ap

        return cmd, d

    # --- the ODE ----------------------------------------------------------
    def derivative(self, t: float, state: np.ndarray, pilot: PilotCommands,
                   record: bool = False) -> np.ndarray:
        p = self.p
        dcm, ve, euler, xyh, alt_m, hdot = self._derived(state)
        vb = state[dynamics.IDX_VB]
        w_body = state[dynamics.IDX_W]
        ap_state = state[IDX_AP]

        # --- Environment (SID 245) ---
        _, _, _, rho = environment.coesa(alt_m)
        g_ned = environment.gravity_ned(alt_m)
        uvw_wind, pqr_wind = self.wind(float(np.linalg.norm(ve)), alt_m, dcm)

        # --- Calculate Parameters (SID 230) ---
        alpha, beta, V = aero.incidence_sideslip_airspeed(vb - uvw_wind)
        pqr_tot = w_body + pqr_wind
        qbar = aero.dynamic_pressure(ve, rho)

        # --- Data Routing signals used by the autopilots (degrees) ---
        phi_deg, theta_deg, psi_deg = np.rad2deg(euler)
        u_b, v_b, w_b = (float(vb[0]), float(vb[1]), float(vb[2]))
        U = float(np.sqrt(u_b * u_b + v_b * v_b + w_b * w_b))
        alpha_log_deg = float(np.rad2deg(np.arctan2(w_b, u_b)))
        beta_log_deg = float(np.rad2deg(np.arcsin(np.clip(v_b / U, -1.0, 1.0)))) if U > 1e-12 else 0.0
        alt_ft = alt_m * M_TO_FT

        # --- Controls (SID 290) ---
        cmd, d_ap = self._autopilot(ap_state, phi_deg, theta_deg, beta_log_deg,
                                    alt_ft, u_b, pilot)
        surf = controls.commands_to_surfaces(cmd, p.trim.da0, p.trim.de0, p.trim.dr0)

        # --- Aerodynamics (SID 27) + Engine (SID 164) ---
        coeff = aero.aero_coefficients(p.aero, p.geom.b, p.geom.c, alpha, beta,
                                       V, pqr_tot, surf.uaero_rad)
        dpt = engine.thrust_parameter(surf.rpm, surf.manifold_pressure, rho, V)
        cprop = engine.propulsion_coefficients(p.aero, dpt, alpha)

        # --- Forces and Moments (SID 206) ---
        F_aero, M_aero = aero.forces_and_moments(coeff + cprop, qbar, p.geom.S,
                                                 p.geom.b, p.geom.c)
        # --- Gravity (SID 216): DCM * (mass * g_ned) ---
        F_grav = dcm @ (p.geom.mass * g_ned)
        F_tot = F_aero + F_grav
        M_tot = M_aero

        # --- Aircraft Dynamics (SID 4) ---
        dy = np.zeros(N_STATES)
        dy[: dynamics.N_STATES] = dynamics.state_derivative(
            state[: dynamics.N_STATES], F_tot, M_tot,
            p.geom.mass, p.geom.inertia, p.geom.inertia_inv, k_quat=1.0)
        dy[IDX_AP] = d_ap

        if record:
            lat, lon, alt_lla = self.lla(state[dynamics.IDX_XE])
            o = self.out
            o.t = t
            o.phi_deg, o.theta_deg, o.psi_deg = float(phi_deg), float(theta_deg), float(psi_deg)
            o.u, o.v, o.w = u_b, v_b, w_b
            o.U, o.U_kts = U, U * MPS_TO_KTS
            o.p_dps, o.q_dps, o.r_dps = (float(x) for x in np.rad2deg(w_body))
            o.alpha_deg, o.beta_deg = alpha_log_deg, beta_log_deg
            o.x, o.y = float(xyh[0]), float(xyh[1])
            o.alt_m, o.alt_ft = alt_m, alt_ft
            o.hdot_mps = hdot
            o.hdot_ft_min = hdot * MPS_TO_FTMIN
            o.Ve = ve.copy()
            o.lat, o.lon = lat, lon
            o.rho, o.qbar, o.g = float(rho), float(qbar), float(g_ned[2])
            o.elevator_deg = surf.elevator_deg
            o.aileron_deg = surf.aileron_deg
            o.rudder_deg = surf.rudder_deg
            o.flap_deg = surf.flap_deg
            o.rpm, o.manifold_pressure = surf.rpm, surf.manifold_pressure
            o.dpt = float(dpt)
            o.power_kW = engine.shaft_power_kW(surf.rpm, surf.manifold_pressure, rho)
            o.cmd_roll, o.cmd_pitch = cmd.roll, cmd.pitch
            o.cmd_yaw, o.cmd_throttle = cmd.yaw, cmd.throttle
            o.stick_lat, o.stick_long = pilot.roll, pilot.pitch
            o.pedal, o.throttle_in = pilot.yaw, pilot.throttle
            o.force_b, o.moment_b = F_tot.copy(), M_tot.copy()
            o.autostop, o.autostop_reason = self.check_autostop(o)

        return dy

    # --- auto-stop --------------------------------------------------------
    def check_autostop(self, o: Outputs) -> tuple[bool, str]:
        c = self.autostop_cfg
        if not c.enabled:
            return False, ""
        if abs(o.hdot_mps) >= c.hdot_limit:
            return True, f"|Hdot| >= {c.hdot_limit}"
        if abs(o.alpha_deg) >= c.alpha_deg_limit:
            return True, f"|alpha| >= {c.alpha_deg_limit} deg"
        if abs(o.U_kts) >= c.speed_kts_max:
            return True, f"|U| >= {c.speed_kts_max} kts"
        if abs(o.U_kts) <= c.speed_kts_min:
            return True, f"|U| <= {c.speed_kts_min} kts"
        if c.fix_altitude_test and o.alt_ft < 0.0:
            return True, "altitude below 0 ft"
        pqr = np.array([o.p_dps, o.q_dps, o.r_dps]) * np.pi / 180.0
        if float(np.linalg.norm(pqr)) >= c.pqr_norm_limit:
            return True, f"|pqr| >= {c.pqr_norm_limit} rad/s"
        return False, ""

    # --- convenience ------------------------------------------------------
    def step(self, t: float, state: np.ndarray, h: float,
             pilot: PilotCommands) -> np.ndarray:
        """One fixed-step RK4 step. Records outputs at the step's start."""
        self.derivative(t, state, pilot, record=True)

        def f(tt, yy):
            return self.derivative(tt, yy, pilot, record=False)

        nxt = dynamics.rk4_step(f, t, state, h)
        # Keep the quaternion clean; k_quat feedback alone drifts over long runs.
        q = nxt[dynamics.IDX_Q]
        n = float(np.linalg.norm(q))
        if n > 0.0:
            nxt[dynamics.IDX_Q] = q / n
        return nxt
