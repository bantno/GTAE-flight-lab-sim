"""IO tests: X-Plane packets over a loopback socket, logging, fake joystick."""
from __future__ import annotations

import socket
import struct
import sys
import time
from pathlib import Path

import numpy as np
import pytest
import scipy.io as sio

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ae4803 import AutoStopConfig, AutopilotConfig, FlightModel, PilotCommands, replay
from ae4803.flightlog import SIGNALS, FlightLogger
from ae4803.joystick import ScriptedJoystick, StaleAxisWatchdog, StaleCheckConfig
from ae4803.realtime import LoopConfig, RealTimeLoop
from ae4803.xplane import (DREF_HEADER, VEHX_HEADER, XPlaneConfig, XPlaneSender,
                           pack_dref, pack_legacy_string, pack_vehx)


# --- packet formats -------------------------------------------------------
def test_vehx_packet_layout():
    pkt = pack_vehx(33.772201, -84.394679, 914.4, 12.0, 3.0, -5.0, index=0)
    assert len(pkt) == 45
    assert pkt[:5] == VEHX_HEADER == b"VEHX0"
    assert pkt[4:5] == b"0"                       # ASCII '0', not NUL
    assert struct.unpack_from("<I", pkt, 5)[0] == 0
    lat, lon, alt = struct.unpack_from("<3d", pkt, 9)
    assert (lat, lon, alt) == pytest.approx((33.772201, -84.394679, 914.4))
    psi, the, phi = struct.unpack_from("<3f", pkt, 33)
    # heading, pitch, roll - the model reorders them before packing
    assert (psi, the, phi) == pytest.approx((12.0, 3.0, -5.0), abs=1e-4)


def test_dref_packet_layout():
    path = "sim/flightmodel2/controls/roll_ratio"
    pkt = pack_dref(path, 0.25)
    assert len(pkt) == 509
    assert pkt[:5] == DREF_HEADER == b"DREF0"
    assert struct.unpack_from("<f", pkt, 5)[0] == pytest.approx(0.25)
    assert pkt[9:9 + len(path)].decode() == path
    assert pkt[9 + len(path):10 + len(path)] == b"\x00"
    assert set(pkt[10 + len(path):]) == {ord(" ")}


def test_legacy_string_format():
    msg = pack_legacy_string(1.0, 2.0, 3.0, 33.772201, -84.394679, 914.4).decode()
    parts = msg.split(" ")
    assert len(parts) == 6
    assert float(parts[0]) == pytest.approx(1.0)
    assert float(parts[3]) == pytest.approx(33.772201)
    assert float(parts[5]) == pytest.approx(914.4)


def test_packets_arrive_on_a_loopback_socket():
    """End-to-end over UDP, bound to 127.0.0.1 so no firewall prompt appears."""
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.settimeout(2.0)
    port = rx.getsockname()[1]

    sender = XPlaneSender(XPlaneConfig(enabled=True, ip="127.0.0.1", port=port,
                                       rate_hz=0.0, send_datarefs=True))
    m = FlightModel(autostop=AutoStopConfig(enabled=False))
    s = m.reset()
    m.derivative(0.0, s, PilotCommands(0, 0, 0, 0.618), record=True)
    assert sender.send(m.out, time.perf_counter(), force=True)

    seen = []
    try:
        for _ in range(5):
            data, _ = rx.recvfrom(2048)
            seen.append(data)
    except socket.timeout:
        pass
    finally:
        sender.close()
        rx.close()

    assert any(d.startswith(VEHX_HEADER) and len(d) == 45 for d in seen)
    assert any(d.startswith(DREF_HEADER) and len(d) == 509 for d in seen)
    assert sender.errors == 0


# --- logging --------------------------------------------------------------
def test_flight_log_has_the_simulink_signal_names(tmp_path):
    m = FlightModel(autostop=AutoStopConfig(enabled=False))
    logger = FlightLogger(log_dir=tmp_path)
    s = m.reset()
    h = 1.0 / 100.0
    for k in range(50):
        m.derivative(k * h, s, PilotCommands(0, 0, 0, 0.618), record=True)
        logger.record(m.out)
        s = m.step(k * h, s, h, PilotCommands(0, 0, 0, 0.618))

    path = logger.save(filename="test_log.mat")
    assert path.is_file()

    d = sio.loadmat(str(path), squeeze_me=False, struct_as_record=False)
    assert "sigsOut" in d and "sigs" in d and "t" in d

    # savemat wrote a real 1xN MATLAB struct array, so loadmat gives mat_struct
    # elements with .Name / .Values - the same access pattern as the Dataset.
    ds = d["sigsOut"]
    assert ds.shape == (1, len(SIGNALS))
    names = [str(np.squeeze(ds[0, i].Name)) for i in range(ds.shape[1])]
    expected = [n for n, _, _ in SIGNALS]
    assert names == expected, "log signal names must match the Simulink dataset"

    # Ve is the only 3-wide channel
    ve = np.asarray(ds[0, names.index("Ve_[m/s]")].Values[0, 0].Data)
    assert ve.shape == (50, 3)
    assert d["t"].shape == (50, 1)


def test_log_filename_matches_simulink_timestamp_format(tmp_path):
    logger = FlightLogger(log_dir=tmp_path)
    m = FlightModel(autostop=AutoStopConfig(enabled=False))
    s = m.reset()
    m.derivative(0.0, s, PilotCommands(), record=True)
    logger.record(m.out)
    ts = time.strptime("2026-11-15 14:03:07", "%Y-%m-%d %H:%M:%S")
    path = logger.save(timestamp=ts)
    assert path.name == "111526_140307.mat"      # mmddyy_HHMMSS, as the StopFcn


# --- fake joystick + stale detection -------------------------------------
def test_scripted_joystick_drives_the_model():
    src = ScriptedJoystick(replay.profile)
    src.set_time(10.6)
    cmd = src.poll()
    assert cmd.pitch > 0.0                       # inside the pitch doublet
    src.set_time(0.0)
    assert src.poll().pitch == pytest.approx(0.0)


def test_stale_axis_watchdog_flags_an_axis_that_dies_after_working():
    wd = StaleAxisWatchdog(StaleCheckConfig(enabled=True, motion_threshold=0.002,
                                            freeze_seconds=1.0,
                                            repeat_warning_seconds=10.0))
    t = 0.0
    # phase 1: both axes work normally
    for i in range(20):
        t += 0.1
        wd.update({"roll": 0.01 * i, "throttle": 0.01 * i}, t)
    assert wd.warnings == []

    # phase 2: the throttle dies - bit-for-bit frozen while roll keeps moving
    for i in range(20, 60):
        t += 0.1
        wd.update({"roll": 0.01 * i, "throttle": 0.19}, t)
    assert any("throttle" in w for w in wd.warnings)
    assert not any("roll" in w for w in wd.warnings)


def test_watchdog_ignores_a_control_that_was_never_touched():
    """A parked or untouched control is not evidence of a fault - flagging it
    every flight would just train people to ignore the warning."""
    wd = StaleAxisWatchdog(StaleCheckConfig(enabled=True, freeze_seconds=1.0))
    t = 0.0
    for i in range(60):
        t += 0.1
        wd.update({"roll": 0.01 * i, "throttle": 0.5}, t)
    assert wd.warnings == []


def test_watchdog_quiet_when_everything_moves():
    wd = StaleAxisWatchdog(StaleCheckConfig(enabled=True, freeze_seconds=1.0))
    t = 0.0
    for i in range(40):
        t += 0.1
        wd.update({"roll": 0.01 * i, "throttle": 0.5 + 0.003 * i}, t)
    assert wd.warnings == []


# --- real-time loop -------------------------------------------------------
def test_realtime_loop_runs_and_keeps_time():
    m = FlightModel(autopilot=AutopilotConfig(False, False),
                    autostop=AutoStopConfig(enabled=False))
    loop = RealTimeLoop(m, ScriptedJoystick(replay.profile),
                        LoopConfig(rate_hz=100.0, high_priority=False))
    loop.start()
    time.sleep(2.0)
    loop.stop("test over")
    loop.join(3.0)

    d = loop.stats.summary()
    assert d["steps"] > 150
    assert 0.95 < d["real_time_factor"] < 1.05
    assert d["overrun_pct"] < 10.0


# --- restart, including after auto-stop ----------------------------------
def test_restart_resets_state_and_keeps_derivative_edits():
    m = FlightModel(autopilot=AutopilotConfig(False, False),
                    autostop=AutoStopConfig(enabled=False))
    loop = RealTimeLoop(m, ScriptedJoystick(replay.profile),
                        LoopConfig(rate_hz=200.0, high_priority=False))
    edited = m.p.aero.Cm_q * 0.5
    loop.set_parameter("aero.Cm_q", edited)
    loop.start()
    time.sleep(1.2)
    t_before = loop.sim_time
    loop.restart()
    time.sleep(0.6)
    t_after = loop.sim_time
    loop.stop("test over")
    loop.join(3.0)

    assert t_before > 0.5
    assert t_after < t_before, "sim time did not reset"
    assert m.p.aero.Cm_q == edited, "derivative edit was lost on restart"
    # the tuning log must be re-stamped at t=0, or it carries pre-restart times
    assert all(e["t"] == 0.0 for e in loop.tuning_log.entries)


def test_autostop_halts_but_leaves_the_loop_alive_so_restart_works():
    """Regression: auto-stop used to call stop(), which tore the loop down.

    The Restart button then did nothing - precisely when you most want it,
    after a stall. Auto-stop must FREEZE and keep servicing commands.
    """
    m = FlightModel(autopilot=AutopilotConfig(False, False),
                    autostop=AutoStopConfig(enabled=True))
    loop = RealTimeLoop(m, ScriptedJoystick(lambda t: PilotCommands(0, 1.0, 0, 0.0)),
                        LoopConfig(rate_hz=200.0, high_priority=False))
    loop.start()
    for _ in range(200):                       # wait for auto-stop to fire
        if loop.halted:
            break
        time.sleep(0.05)

    assert loop.halted, "auto-stop never fired"
    assert loop.running, "loop died on auto-stop - restart cannot work"
    t_halt = loop.sim_time
    time.sleep(0.3)
    assert abs(loop.sim_time - t_halt) < 1e-6, "sim time advanced while halted"

    loop.restart()
    time.sleep(0.4)
    assert not loop.halted, "restart did not clear the halt"
    assert loop.sim_time > 0.0 and loop.sim_time < t_halt, "did not re-fly"
    loop.stop("test over")
    loop.join(3.0)
