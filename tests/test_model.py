"""Offline model tests - no hardware, no X-Plane, no MATLAB required.

    pytest -q
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest
import scipy.io as sio

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ae4803 import AutopilotConfig, AutoStopConfig, FlightModel, Params, PilotCommands
from ae4803 import aero, controls, dynamics, engine, environment, replay
from ae4803.geodesy import FlatEarthToLLA
from ae4803.params import INERTIA_AS_IN_MASK, INERTIA_PHYSICAL, INERTIA_SIMULINK

REPO = Path(__file__).resolve().parent.parent
TRIM_THROTTLE = 0.618


def fly(model: FlightModel, seconds: float, pilot: PilotCommands,
        rate: float = 200.0):
    """Integrate and return (final Outputs, list of Outputs snapshots)."""
    h = 1.0 / rate
    n = int(round(seconds / h))
    s = model.reset()
    hist = []
    for k in range(n + 1):
        t = k * h
        model.derivative(t, s, pilot, record=True)
        o = model.out
        hist.append((o.t, o.alt_m, o.U, o.theta_deg, o.phi_deg, o.alpha_deg,
                     o.beta_deg, o.psi_deg, o.elevator_deg, o.aileron_deg,
                     o.rudder_deg, o.rpm))
        if k < n:
            s = model.step(t, s, h, pilot)
    return model.out, np.array(hist)


@pytest.fixture(scope="module")
def params():
    return Params.load()


# --- parameters -----------------------------------------------------------
def test_parameters_match_sim_init(params):
    assert params.geom.S == pytest.approx(23.23)
    assert params.geom.b == pytest.approx(14.63)
    assert params.geom.c == pytest.approx(1.5875)
    assert params.geom.mass == pytest.approx(2288.231)
    assert params.trim.da0 == pytest.approx(0.0378)
    assert params.trim.de0 == pytest.approx(-1.86)
    assert params.trim.dr0 == pytest.approx(-2.44)
    assert params.trim.uvw0 == pytest.approx([45.0, 0.0, 0.0])
    assert params.trim.xme_0 == pytest.approx([0.0, 0.0, -914.4])
    # LongLat0 is stored [longitude, latitude] - Georgia Tech
    assert params.trim.LongLat0[0] == pytest.approx(-84.394679)
    assert params.trim.LongLat0[1] == pytest.approx(33.772201)
    assert params.wing_leveler_on is False
    assert params.altitude_hold_on is False


def test_inertia_simulink_is_transpose_of_mask():
    assert np.allclose(INERTIA_SIMULINK, INERTIA_AS_IN_MASK.T)


def test_default_inertia_is_symmetric(params):
    """The shipped default must be a realizable inertia tensor.

    The sim deliberately no longer matches AE4803Sim.slx here; anything
    comparing against Simulink has to ask for inertia_mode="simulink".
    """
    I = params.geom.inertia
    assert np.allclose(I, I.T), "default inertia must be symmetric"
    assert np.all(np.linalg.eigvals(I) > 0)
    sim = Params.load(inertia_mode="simulink").geom.inertia
    assert not np.allclose(sim, sim.T), "simulink mode must stay bug-for-bug"


def test_physical_inertia_is_symmetric_and_the_mask_is_not():
    assert np.allclose(INERTIA_PHYSICAL, INERTIA_PHYSICAL.T)
    assert not np.allclose(INERTIA_AS_IN_MASK, INERTIA_AS_IN_MASK.T)
    assert np.all(np.linalg.eigvals(INERTIA_PHYSICAL) > 0)


def test_asymmetric_inertia_does_not_conserve_rotational_energy():
    """Demonstrates WHY the mask matrix is not a valid inertia tensor."""
    def drift(I, w0=np.array([0.3, 0.2, 0.1]), t_end=50.0, h=1e-3):
        Iinv = np.linalg.inv(I)
        f = lambda w: Iinv @ (-np.cross(w, I @ w))
        w = w0.copy()
        T0 = 0.5 * w @ I @ w
        worst = 0.0
        for _ in range(int(t_end / h)):
            w = dynamics.rk4_step(lambda _t, y: f(y), 0.0, w, h)
            worst = max(worst, abs(0.5 * w @ I @ w - T0) / T0)
        return worst

    assert drift(INERTIA_PHYSICAL) < 1e-10        # exact, as physics requires
    assert drift(INERTIA_AS_IN_MASK) > 1e-5       # the .slx matrix does not


# --- environment ----------------------------------------------------------
def test_atmosphere_matches_matlab_reference():
    ref = REPO / "matlab" / "reference" / "env_ref.mat"
    if not ref.is_file():
        pytest.skip("env_ref.mat not generated")
    d = sio.loadmat(str(ref), squeeze_me=True)
    h = np.atleast_1d(d["h"]).astype(float)
    T, a, P, rho = environment.coesa(h)
    assert np.allclose(T, d["T"], rtol=1e-9)
    assert np.allclose(rho, d["rho"], rtol=1e-5)
    assert np.allclose(P, d["P"], rtol=1e-5)
    g = environment.gravity_wgs84_taylor(h, float(d["lat"]))
    assert np.allclose(g, d["g_taylor"], rtol=1e-10)


def test_sea_level_atmosphere():
    T, a, p, rho = environment.coesa(0.0)
    assert T == pytest.approx(288.15, abs=1e-6)
    assert p == pytest.approx(101325.0, abs=1e-3)
    assert rho == pytest.approx(1.225, abs=1e-4)


def test_wind_is_zero_by_default():
    """AE4803Sim has Dryden T_on = off, shear W_20 = 0, all gusts off."""
    w = environment.WindModel()
    uvw, pqr = w(45.0, 914.4, np.eye(3))
    assert np.allclose(uvw, 0.0)
    assert np.allclose(pqr, 0.0)


# --- kinematics -----------------------------------------------------------
def test_quaternion_euler_roundtrip():
    for eul in ([0, 0, 0], [0.3, -0.2, 1.1], [-1.0, 0.4, -2.5]):
        e = np.array(eul)
        q = dynamics.quat_from_euler(e)
        back = dynamics.euler_from_dcm(dynamics.dcm_from_quat(q))
        assert np.allclose(back, e, atol=1e-12)


def test_dcm_is_orthonormal():
    q = dynamics.quat_from_euler(np.array([0.4, -0.3, 2.0]))
    dcm = dynamics.dcm_from_quat(q)
    assert np.allclose(dcm @ dcm.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(dcm) == pytest.approx(1.0, abs=1e-12)


def test_flat_earth_to_lla_origin_and_north(params):
    lla = FlatEarthToLLA.from_params(params.trim.LongLat0)
    lat, lon, alt = lla(np.array([0.0, 0.0, -914.4]))
    assert lat == pytest.approx(33.772201, abs=1e-9)
    assert lon == pytest.approx(-84.394679, abs=1e-9)
    assert alt == pytest.approx(914.4)
    # 1 km north increases latitude, leaves longitude alone
    lat_n, lon_n, _ = lla(np.array([1000.0, 0.0, -914.4]))
    assert lat_n > lat
    assert lon_n == pytest.approx(lon, abs=1e-12)
    # 1 km east increases longitude at this latitude
    _, lon_e, _ = lla(np.array([0.0, 1000.0, -914.4]))
    assert lon_e > lon


# --- control chain --------------------------------------------------------
def test_deadzone_matches_simulink_semantics():
    assert controls.dead_zone(0.0, -0.01, 0.01) == 0.0
    assert controls.dead_zone(0.005, -0.01, 0.01) == 0.0
    assert controls.dead_zone(0.02, -0.01, 0.01) == pytest.approx(0.01)
    assert controls.dead_zone(-0.02, -0.01, 0.01) == pytest.approx(-0.01)


def test_throttle_lookup_trim_point():
    """Trim throttle 0.618 should give about 1775 rpm."""
    rpm = controls.throttle_to_rpm(TRIM_THROTTLE)
    assert rpm == pytest.approx(1775.0, abs=10.0)
    assert controls.throttle_to_rpm(0.0) == pytest.approx(0.0, abs=0.1)
    assert controls.throttle_to_rpm(1.0) == pytest.approx(2300.0, abs=0.1)
    # clipped outside the breakpoints
    assert controls.throttle_to_rpm(-5.0) == controls.throttle_to_rpm(0.0)
    assert controls.throttle_to_rpm(5.0) == controls.throttle_to_rpm(1.0)


def test_manifold_pressure_lookup():
    assert controls.rpm_to_manifold_pressure(0.0) == pytest.approx(0.0)
    assert controls.rpm_to_manifold_pressure(1800.0) == pytest.approx(22.5735)
    assert controls.rpm_to_manifold_pressure(2300.0) == pytest.approx(26.0)


def test_command_to_surface_gains_and_trims(params):
    t = params.trim
    neutral = controls.commands_to_surfaces(PilotCommands(0, 0, 0, 0),
                                            t.da0, t.de0, t.dr0)
    assert neutral.elevator_deg == pytest.approx(t.de0)
    assert neutral.aileron_deg == pytest.approx(t.da0)
    assert neutral.rudder_deg == pytest.approx(t.dr0)
    assert neutral.flap_deg == pytest.approx(0.0)

    full = controls.commands_to_surfaces(PilotCommands(1, 1, 1, 1),
                                         t.da0, t.de0, t.dr0)
    assert full.elevator_deg == pytest.approx(t.de0 - 20.0)
    assert full.aileron_deg == pytest.approx(t.da0 - 15.0)
    assert full.rudder_deg == pytest.approx(t.dr0 + 25.0)

    # commands saturate before the gain
    over = controls.commands_to_surfaces(PilotCommands(9, 9, 9, 9),
                                         t.da0, t.de0, t.dr0)
    assert over.elevator_deg == pytest.approx(full.elevator_deg)
    # uaero is [de, da, dr, df] in radians
    assert np.allclose(full.uaero_rad,
                       np.deg2rad([full.elevator_deg, full.aileron_deg,
                                   full.rudder_deg, full.flap_deg]))


# --- aerodynamics ---------------------------------------------------------
def test_datum_coefficients_at_zero(params):
    c = params.aero
    d = aero.datum_coefficients(c, 0.0, 0.0)
    assert d == pytest.approx([c.Cx0, c.Cy0, c.Cz0, c.Cl0, c.Cm0, c.Cn0])


def test_lift_increases_with_alpha(params):
    """Cz becomes more negative (more lift, z down) as alpha grows."""
    c = params.aero
    cz = [aero.datum_coefficients(c, np.deg2rad(a), 0.0)[2] for a in (0, 5, 10)]
    assert cz[0] > cz[1] > cz[2]


def test_pitching_moment_is_statically_stable(params):
    """dCm/dalpha < 0 about the trim alpha - the aircraft is statically stable."""
    c = params.aero
    a0 = np.deg2rad(7.5)
    da = 1e-4
    dCm = (aero.datum_coefficients(c, a0 + da, 0.0)[4]
           - aero.datum_coefficients(c, a0 - da, 0.0)[4]) / (2 * da)
    assert dCm < 0.0


def test_body_rate_damping_opposes_rotation(params):
    c = params.aero
    d = aero.body_rate_damping(c, params.geom.b, params.geom.c, 45.0,
                               np.array([0.1, 0.0, 0.0]))
    assert d[3] < 0.0            # roll damping Cl opposes +p
    d = aero.body_rate_damping(c, params.geom.b, params.geom.c, 45.0,
                               np.array([0.0, 0.1, 0.0]))
    assert d[4] < 0.0            # pitch damping Cm opposes +q
    d = aero.body_rate_damping(c, params.geom.b, params.geom.c, 45.0,
                               np.array([0.0, 0.0, 0.1]))
    assert d[5] < 0.0            # yaw damping Cn opposes +r


def test_engine_power_and_thrust_increase_with_throttle(params):
    rho = 1.1215
    lo = controls.throttle_to_rpm(0.3)
    hi = controls.throttle_to_rpm(0.9)
    p_lo = engine.shaft_power_kW(lo, controls.rpm_to_manifold_pressure(lo), rho)
    p_hi = engine.shaft_power_kW(hi, controls.rpm_to_manifold_pressure(hi), rho)
    assert p_hi > p_lo
    d_lo = engine.thrust_parameter(lo, controls.rpm_to_manifold_pressure(lo), rho, 45.0)
    d_hi = engine.thrust_parameter(hi, controls.rpm_to_manifold_pressure(hi), rho, 45.0)
    assert d_hi > d_lo


# --- trimmed flight -------------------------------------------------------
def test_holds_approximately_level_flight_from_trim():
    """From the trim state the aircraft should stay near level flight.

    It is not a perfectly trimmed initial condition - it starts at theta = 0
    with alpha = 0, so it pitches up into its trim alpha and rings down a
    lightly damped phugoid. What matters is that it stays near 914 m and 45 m/s
    rather than diverging.
    """
    m = FlightModel(autopilot=AutopilotConfig(False, False),
                    autostop=AutoStopConfig(enabled=False))
    out, hist = fly(m, 120.0, PilotCommands(0, 0, 0, TRIM_THROTTLE))
    alt, U = hist[:, 1], hist[:, 2]
    assert np.all(np.abs(alt - 914.4) < 30.0), f"altitude excursion {np.ptp(alt):.1f} m"
    assert np.all(np.abs(U - 45.0) < 6.0), f"airspeed excursion {np.ptp(U):.1f} m/s"
    # and it must not be diverging: the second half is no worse than the first
    half = len(alt) // 2
    assert np.ptp(alt[half:]) <= np.ptp(alt[:half]) * 1.3


def test_trim_alpha_is_physically_sensible():
    m = FlightModel(autopilot=AutopilotConfig(False, False),
                    autostop=AutoStopConfig(enabled=False))
    _, hist = fly(m, 120.0, PilotCommands(0, 0, 0, TRIM_THROTTLE))
    alpha = hist[len(hist) // 2:, 5]
    assert 3.0 < alpha.mean() < 12.0


# --- control response directions -----------------------------------------
@pytest.mark.parametrize("cmd,field,expect_sign,label", [
    (PilotCommands(0, 0.3, 0, TRIM_THROTTLE), 3, +1, "pull back -> nose up"),
    (PilotCommands(0, -0.3, 0, TRIM_THROTTLE), 3, -1, "push -> nose down"),
    (PilotCommands(0.3, 0, 0, TRIM_THROTTLE), 4, +1, "right stick -> roll right"),
    (PilotCommands(-0.3, 0, 0, TRIM_THROTTLE), 4, -1, "left stick -> roll left"),
])
def test_control_response_direction(cmd, field, expect_sign, label):
    """Each control must move the aircraft the correct way.

    Compared against the same run with no input, so the natural phugoid and the
    asymmetric trim do not mask the control effect.
    """
    base = FlightModel(autopilot=AutopilotConfig(False, False),
                       autostop=AutoStopConfig(enabled=False))
    _, h0 = fly(base, 3.0, PilotCommands(0, 0, 0, TRIM_THROTTLE))
    m = FlightModel(autopilot=AutopilotConfig(False, False),
                    autostop=AutoStopConfig(enabled=False))
    _, h1 = fly(m, 3.0, cmd)
    delta = h1[-1, field] - h0[-1, field]
    assert np.sign(delta) == expect_sign, f"{label}: delta = {delta:+.3f} deg"
    assert abs(delta) > 0.5, f"{label}: effect too small ({delta:+.3f} deg)"


def test_rudder_sign_convention_is_aerodynamically_consistent():
    """A POSITIVE yaw command deflects the rudder positive, which in this
    coefficient set (Cn_dr < 0, Cy_dr > 0) yaws the nose LEFT and produces
    POSITIVE sideslip - and the dihedral effect (Cl_beta < 0) then rolls left.

    Which physical pedal should produce a positive yaw command is a mapping
    question, settled on the hardware and recorded in config/joystick.toml.
    See docs/MODEL_NOTES.md - right pedal must map to a NEGATIVE yaw command
    for the nose to go right.
    """
    base = FlightModel(autopilot=AutopilotConfig(False, False),
                       autostop=AutoStopConfig(enabled=False))
    _, h0 = fly(base, 3.0, PilotCommands(0, 0, 0, TRIM_THROTTLE))
    m = FlightModel(autopilot=AutopilotConfig(False, False),
                    autostop=AutoStopConfig(enabled=False))
    _, h1 = fly(m, 3.0, PilotCommands(0, 0, 0.3, TRIM_THROTTLE))

    assert h1[-1, 10] > h0[-1, 10]          # rudder deflection went positive
    assert h1[-1, 7] - h0[-1, 7] < 0.0      # psi decreases: nose left
    assert h1[-1, 6] - h0[-1, 6] > 0.0      # beta increases: positive sideslip
    assert h1[-1, 4] - h0[-1, 4] < 0.0      # dihedral effect rolls it left

    # and the opposite command must do the opposite
    m2 = FlightModel(autopilot=AutopilotConfig(False, False),
                     autostop=AutoStopConfig(enabled=False))
    _, h2 = fly(m2, 3.0, PilotCommands(0, 0, -0.3, TRIM_THROTTLE))
    assert h2[-1, 7] - h0[-1, 7] > 0.0
    assert h2[-1, 6] - h0[-1, 6] < 0.0


def test_more_throttle_accelerates():
    slow = FlightModel(autopilot=AutopilotConfig(False, False),
                       autostop=AutoStopConfig(enabled=False))
    _, h0 = fly(slow, 10.0, PilotCommands(0, 0, 0, 0.45))
    fast = FlightModel(autopilot=AutopilotConfig(False, False),
                       autostop=AutoStopConfig(enabled=False))
    _, h1 = fly(fast, 10.0, PilotCommands(0, 0, 0, 0.95))
    assert h1[-1, 2] > h0[-1, 2]


# --- autopilots -----------------------------------------------------------
def test_wing_leveler_reduces_bank():
    """Disturb in roll, then compare holding the stick neutral with/without
    the wing leveler engaged."""
    def run(wl: bool):
        m = FlightModel(autopilot=AutopilotConfig(wing_leveler_on=False),
                        autostop=AutoStopConfig(enabled=False))
        h = 1.0 / 200.0
        s = m.reset()
        t = 0.0
        # 2 s of roll input to build up a bank angle
        for _ in range(int(2.0 / h)):
            s = m.step(t, s, h, PilotCommands(0.4, 0, 0, TRIM_THROTTLE))
            t += h
        m.ap.wing_leveler_on = wl
        for _ in range(int(15.0 / h)):
            s = m.step(t, s, h, PilotCommands(0, 0, 0, TRIM_THROTTLE))
            t += h
        m.derivative(t, s, PilotCommands(0, 0, 0, TRIM_THROTTLE), record=True)
        return abs(m.out.phi_deg)

    assert run(True) < run(False)


def test_altitude_hold_captures_and_holds():
    m = FlightModel(autopilot=AutopilotConfig(False, False),
                    autostop=AutoStopConfig(enabled=False))
    h = 1.0 / 200.0
    s = m.reset()
    t = 0.0
    for _ in range(int(5.0 / h)):
        s = m.step(t, s, h, PilotCommands(0, 0, 0, TRIM_THROTTLE))
        t += h
    m.derivative(t, s, PilotCommands(0, 0, 0, TRIM_THROTTLE), record=True)
    captured = m.out.alt_ft
    m.ap.altitude_hold_on = True

    alts = []
    for _ in range(int(60.0 / h)):
        s = m.step(t, s, h, PilotCommands(0, 0, 0, TRIM_THROTTLE))
        t += h
        alts.append(m.out.alt_ft)
    assert m._alt_ref_ft == pytest.approx(captured, abs=1e-6)
    assert abs(np.mean(alts[-int(20.0 / h):]) - captured) < 250.0


def test_autostop_fires_on_overspeed():
    m = FlightModel(autopilot=AutopilotConfig(False, False),
                    autostop=AutoStopConfig(enabled=True))
    out, _ = fly(m, 2.0, PilotCommands(0, 0, 0, TRIM_THROTTLE))
    assert not out.autostop            # 45 m/s = 87 kts, well inside the band
    out.U_kts = 150.0
    fired, reason = m.check_autostop(out)
    assert fired and "140" in reason


def test_autostop_altitude_test_is_unreachable_by_default():
    """The .slx uses abs(alt_ft) < 0, which can never fire. Documented, not fixed."""
    m = FlightModel(autostop=AutoStopConfig(enabled=True))
    o = m.out
    o.alt_ft, o.U_kts, o.alpha_deg, o.hdot_mps = -5000.0, 87.0, 0.0, 0.0
    o.p_dps = o.q_dps = o.r_dps = 0.0
    assert m.check_autostop(o)[0] is False
    m.autostop_cfg.fix_altitude_test = True
    assert m.check_autostop(o)[0] is True


# --- replay input ---------------------------------------------------------
def test_replay_profile_is_continuous():
    t = np.linspace(0, 60, 60001)
    roll, pitch, yaw, thr = replay.profile_arrays(t)
    for name, y in (("roll", roll), ("pitch", pitch), ("yaw", yaw), ("thr", thr)):
        assert np.max(np.abs(np.diff(y))) < 1e-3, f"{name} has a jump"
    assert thr[0] == pytest.approx(replay.TRIM_THROTTLE)
