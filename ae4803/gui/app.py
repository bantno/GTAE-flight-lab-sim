"""Main window: cockpit gauges, numeric readouts, strip charts, controls.

The strip charts mirror the Simulink 'Analysis Pane' (SID 653): two dashboard
scopes with a 60 s span, one at +/-30 (attitude and flow angles) and one at
+/-1 (pilot commands), plus the two slider switches that toggle the autopilots.
Three further charts are added because they are what students actually want to
see: airspeed, altitude and body rates.

The GUI never touches the integrator. It reads an immutable Outputs snapshot
published by the physics thread, on a QTimer at a modest rate, so rendering
cannot stall the sim.
"""
from __future__ import annotations

import collections

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (QCheckBox, QFrame, QGridLayout, QGroupBox,
                             QHBoxLayout, QLabel, QMainWindow, QPushButton,
                             QVBoxLayout, QWidget)

from .gauges import (AirspeedIndicator, Altimeter, AttitudeIndicator,
                     ClimbIndicator, HeadingIndicator, ACCENT, AMBER, BG,
                     DIM, FG, GREEN, RED)

pg.setConfigOption("background", (22, 24, 28))
pg.setConfigOption("foreground", (170, 176, 186))
# Antialiased curves cost a lot of CPU in the GUI thread for no real benefit
# on strip charts, and that CPU is held against the physics thread by the GIL.
pg.setConfigOption("antialias", False)

TIME_SPAN = 60.0          # Dashboard Scope TimeSpan

#: Gauges and numeric readouts are cheap: repaint them often.
GAUGE_RATE_HZ = 25.0
#: Strip charts are expensive - every repaint runs Python in the GUI thread and
#: therefore holds the GIL against the physics thread. Updating them at 10 Hz
#: and downsampling to CHART_POINTS keeps the physics loop's worst-case
#: lateness roughly an order of magnitude lower, with no visible loss.
CHART_RATE_HZ = 10.0
CHART_POINTS = 600

CURVE_COLORS = ["#68b4ff", "#49c17e", "#e8a33c", "#d9534f", "#b57ae0", "#4ec9c0"]


class Readout(QWidget):
    """Label + value, monospaced so digits do not jiggle."""

    def __init__(self, label: str, unit: str = "", width: int = 9, parent=None):
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(6, 1, 6, 1)
        lay.setSpacing(6)
        self.name = QLabel(label)
        self.name.setStyleSheet(f"color: rgb({DIM.red()},{DIM.green()},{DIM.blue()});")
        self.name.setFont(QFont("Segoe UI", 9))
        self.value = QLabel("-")
        self.value.setFont(QFont("Consolas", 11, QFont.Weight.Bold))
        self.value.setStyleSheet(f"color: rgb({FG.red()},{FG.green()},{FG.blue()});")
        self.value.setAlignment(Qt.AlignmentFlag.AlignRight
                                | Qt.AlignmentFlag.AlignVCenter)
        self.value.setMinimumWidth(width * 9)
        self.unit = QLabel(unit)
        self.unit.setFont(QFont("Segoe UI", 8))
        self.unit.setStyleSheet(f"color: rgb({DIM.red()},{DIM.green()},{DIM.blue()});")
        self.unit.setMinimumWidth(38)
        lay.addWidget(self.name)
        lay.addStretch(1)
        lay.addWidget(self.value)
        lay.addWidget(self.unit)

    def set(self, text: str, color=None) -> None:
        self.value.setText(text)
        c = color or FG
        self.value.setStyleSheet(f"color: rgb({c.red()},{c.green()},{c.blue()});")


class StripChart(pg.PlotWidget):
    def __init__(self, title: str, channels: list[str], y_range=None, parent=None):
        super().__init__(parent)
        self.setTitle(title, color="#aab0ba", size="9pt")
        self.showGrid(x=True, y=True, alpha=0.22)
        self.setMouseEnabled(x=False, y=False)
        self.setMenuEnabled(False)
        self.setXRange(-TIME_SPAN, 0.0, padding=0)
        if y_range is not None:
            self.setYRange(*y_range, padding=0)
            self._autoscale = False
        else:
            self._autoscale = True
        self.addLegend(offset=(6, 4), labelTextSize="7pt")
        self.curves = {}
        for i, ch in enumerate(channels):
            self.curves[ch] = self.plot(
                pen=pg.mkPen(CURVE_COLORS[i % len(CURVE_COLORS)], width=1.5), name=ch)
        self.setMinimumHeight(120)

    def update_data(self, t: np.ndarray, data: dict[str, np.ndarray]) -> None:
        if len(t) == 0:
            return
        # Downsample before handing points to pyqtgraph; drawing thousands of
        # vertices per curve is what makes the GUI thread hold the GIL.
        step = max(1, len(t) // CHART_POINTS)
        rel = t[::step] - t[-1]
        for ch, curve in self.curves.items():
            y = data.get(ch)
            if y is not None and len(y) == len(t):
                curve.setData(rel, y[::step])


class MainWindow(QMainWindow):
    def __init__(self, loop, joystick=None, xplane=None, logger=None,
                 on_quit=None, parent=None):
        super().__init__(parent)
        self.loop = loop
        self.joystick = joystick
        self.xplane = xplane
        self.logger = logger
        self.on_quit = on_quit

        self.setWindowTitle("AE4803 Flight Simulator - Python")
        self.resize(1480, 900)
        self.setStyleSheet(
            f"QMainWindow, QWidget {{ background: rgb({BG.red()},{BG.green()},{BG.blue()}); }}"
            f"QGroupBox {{ color: rgb({DIM.red()},{DIM.green()},{DIM.blue()});"
            f"  border: 1px solid #343a44; border-radius: 4px; margin-top: 10px;"
            f"  font-size: 9pt; }}"
            f"QGroupBox::title {{ subcontrol-origin: margin; left: 8px; padding: 0 4px; }}"
            f"QCheckBox, QPushButton {{ color: rgb({FG.red()},{FG.green()},{FG.blue()});"
            f"  font-size: 9pt; }}"
            f"QPushButton {{ background:#2b303a; border:1px solid #434a56;"
            f"  border-radius:3px; padding:5px 12px; }}"
            f"QPushButton:hover {{ background:#353c48; }}"
        )

        # Pre-allocated ring buffers - no per-frame allocation, no fromiter().
        self._cap = int(TIME_SPAN * GAUGE_RATE_HZ) + 4
        self._keys = ("phi", "theta", "alpha", "beta", "psi",
                      "stick_lat", "stick_long", "pedal", "throttle",
                      "U", "alt_m", "p", "q", "r", "hdot")
        self._buf_t = np.zeros(self._cap)
        self._buf = {k: np.zeros(self._cap) for k in self._keys}
        self._head = 0          # next write index
        self._count = 0

        self._build()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(int(1000.0 / GAUGE_RATE_HZ))

        self.chart_timer = QTimer(self)
        self.chart_timer.timeout.connect(self._refresh_charts)
        self.chart_timer.start(int(1000.0 / CHART_RATE_HZ))

    # --- layout -----------------------------------------------------------
    def _build(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # ---- left: gauges + readouts ----
        left = QVBoxLayout()
        left.setSpacing(8)

        gauges = QGridLayout()
        gauges.setSpacing(6)
        self.asi = AirspeedIndicator()
        self.adi = AttitudeIndicator()
        self.alt = Altimeter()
        self.hdg = HeadingIndicator()
        self.vsi = ClimbIndicator()
        gauges.addWidget(self.asi, 0, 0)
        gauges.addWidget(self.adi, 0, 1)
        gauges.addWidget(self.alt, 0, 2)
        gauges.addWidget(self.hdg, 1, 0)
        gauges.addWidget(self.vsi, 1, 1)

        status = QGroupBox("STATUS")
        sv = QVBoxLayout(status)
        sv.setSpacing(1)
        self.r_time = Readout("sim time", "s")
        self.r_rtf = Readout("real-time", "x")
        self.r_over = Readout("overruns", "")
        self.r_late = Readout("late p99", "us")
        self.r_state = Readout("state", "")
        for w in (self.r_time, self.r_rtf, self.r_over, self.r_late, self.r_state):
            sv.addWidget(w)
        gauges.addWidget(status, 1, 2)
        left.addLayout(gauges, 3)

        ro = QGroupBox("FLIGHT DATA")
        grid = QGridLayout(ro)
        grid.setSpacing(1)
        self.readouts: dict[str, Readout] = {}
        spec = [
            ("U", "airspeed", "m/s"), ("alpha", "alpha", "deg"), ("beta", "beta", "deg"),
            ("phi", "roll", "deg"), ("theta", "pitch", "deg"), ("psi", "heading", "deg"),
            ("p", "roll rate", "deg/s"), ("q", "pitch rate", "deg/s"), ("r", "yaw rate", "deg/s"),
            ("alt_m", "altitude", "m"), ("hdot", "climb", "ft/min"), ("rpm", "engine", "rpm"),
            ("elev", "elevator", "deg"), ("ail", "aileron", "deg"), ("rud", "rudder", "deg"),
            ("lat", "latitude", "deg"), ("lon", "longitude", "deg"), ("qbar", "q-bar", "Pa"),
        ]
        for i, (key, label, unit) in enumerate(spec):
            w = Readout(label, unit)
            self.readouts[key] = w
            grid.addWidget(w, i % 6, i // 6)
        left.addWidget(ro, 1)

        # ---- controls ----
        ctrl = QGroupBox("CONTROLS")
        cl = QHBoxLayout(ctrl)
        flags = self.loop.initial_flags
        self.cb_wl = QCheckBox("Wing leveler")
        self.cb_ah = QCheckBox("Altitude hold")
        self.cb_wl.setChecked(flags["wing_leveler"])
        self.cb_ah.setChecked(flags["altitude_hold"])
        self.cb_wl.toggled.connect(self.loop.set_wing_leveler)
        self.cb_ah.toggled.connect(self.loop.set_altitude_hold)
        self.cb_stop = QCheckBox("Auto-stop")
        self.cb_stop.setChecked(flags["autostop"])
        self.cb_stop.toggled.connect(self.loop.set_autostop)
        self.btn_pause = QPushButton("Pause")
        self.btn_pause.clicked.connect(self._toggle_pause)
        self.btn_quit = QPushButton("Stop && save log")
        self.btn_quit.clicked.connect(self._quit)
        for w in (self.cb_wl, self.cb_ah, self.cb_stop):
            cl.addWidget(w)
        cl.addStretch(1)
        cl.addWidget(self.btn_pause)
        cl.addWidget(self.btn_quit)
        left.addWidget(ctrl)

        self.banner = QLabel("")
        self.banner.setFont(QFont("Segoe UI", 10, QFont.Weight.Bold))
        self.banner.setFrameShape(QFrame.Shape.NoFrame)
        self.banner.setVisible(False)
        left.addWidget(self.banner)

        root.addLayout(left, 5)

        # ---- right: strip charts ----
        right = QVBoxLayout()
        right.setSpacing(6)
        self.ch_att = StripChart("attitude & flow angles  [deg]",
                                 ["phi", "theta", "alpha", "beta"], (-30, 30))
        self.ch_cmd = StripChart("pilot commands  [-1..1]",
                                 ["stick_lat", "stick_long", "pedal", "throttle"], (-1, 1))
        self.ch_spd = StripChart("airspeed  [m/s]", ["U"], (20, 80))
        self.ch_alt = StripChart("altitude  [m]", ["alt_m"], None)
        self.ch_rate = StripChart("body rates  [deg/s]", ["p", "q", "r"], (-30, 30))
        for c in (self.ch_att, self.ch_cmd, self.ch_spd, self.ch_alt, self.ch_rate):
            right.addWidget(c)
        root.addLayout(right, 4)

        # Airspeed validity bands drawn onto the strip chart too.
        from .gauges import V_AMBER, V_GREEN, V_STOP_HIGH, V_STOP_LOW
        for lo, hi, col in ((V_GREEN[0], V_GREEN[1], (60, 170, 90, 38)),
                            (V_AMBER[0], V_GREEN[0], (215, 160, 50, 34)),
                            (V_GREEN[1], V_AMBER[1], (215, 160, 50, 34))):
            region = pg.LinearRegionItem(values=(lo, hi), orientation="horizontal",
                                         movable=False, brush=pg.mkBrush(*col))
            region.setZValue(-10)
            self.ch_spd.addItem(region)
        for v in (V_STOP_LOW, V_STOP_HIGH):
            self.ch_spd.addItem(pg.InfiniteLine(pos=v, angle=0,
                                                pen=pg.mkPen("#d9534f", width=1,
                                                             style=Qt.PenStyle.DashLine)))

    # --- interaction ------------------------------------------------------
    def _toggle_pause(self) -> None:
        self.loop.paused = not self.loop.paused
        self.btn_pause.setText("Resume" if self.loop.paused else "Pause")

    def _quit(self) -> None:
        self.loop.stop("stopped from the GUI")
        if self.on_quit is not None:
            self.on_quit()
        self.close()

    def closeEvent(self, event):
        self.loop.stop("window closed")
        if self.on_quit is not None:
            self.on_quit()
        super().closeEvent(event)

    # --- refresh ----------------------------------------------------------
    def _refresh(self) -> None:
        o = self.loop.snapshot()
        if o is None:
            return

        self.asi.value = o.U
        self.adi.pitch, self.adi.roll = o.theta_deg, o.phi_deg
        self.alt.alt_ft, self.alt.alt_m = o.alt_ft, o.alt_m
        self.hdg.heading = o.psi_deg
        self.vsi.rate = o.hdot_ft_min
        for g in (self.asi, self.adi, self.alt, self.hdg, self.vsi):
            g.update()

        from .gauges import V_AMBER, V_GREEN
        if V_GREEN[0] <= o.U <= V_GREEN[1]:
            vcol = GREEN
        elif V_AMBER[0] <= o.U <= V_AMBER[1]:
            vcol = AMBER
        else:
            vcol = RED
        self.readouts["U"].set(f"{o.U:8.2f}", vcol)
        self.readouts["alpha"].set(f"{o.alpha_deg:8.2f}",
                                   AMBER if abs(o.alpha_deg) > 15 else FG)
        self.readouts["beta"].set(f"{o.beta_deg:8.2f}")
        self.readouts["phi"].set(f"{o.phi_deg:8.2f}")
        self.readouts["theta"].set(f"{o.theta_deg:8.2f}")
        self.readouts["psi"].set(f"{o.psi_deg % 360.0:8.2f}")
        self.readouts["p"].set(f"{o.p_dps:8.2f}")
        self.readouts["q"].set(f"{o.q_dps:8.2f}")
        self.readouts["r"].set(f"{o.r_dps:8.2f}")
        self.readouts["alt_m"].set(f"{o.alt_m:8.1f}")
        self.readouts["hdot"].set(f"{o.hdot_ft_min:8.0f}")
        self.readouts["rpm"].set(f"{o.rpm:8.0f}")
        self.readouts["elev"].set(f"{o.elevator_deg:8.2f}")
        self.readouts["ail"].set(f"{o.aileron_deg:8.2f}")
        self.readouts["rud"].set(f"{o.rudder_deg:8.2f}")
        self.readouts["lat"].set(f"{o.lat:9.5f}")
        self.readouts["lon"].set(f"{o.lon:9.5f}")
        self.readouts["qbar"].set(f"{o.qbar:8.1f}")

        d = self.loop.status()
        self.r_time.set(f"{self.loop.sim_time:8.2f}")
        if d["steps"]:
            self.r_rtf.set(f"{d['real_time_factor']:8.4f}")
            self.r_over.set(f"{d['overruns']:8d}",
                            RED if d["overruns"] else GREEN)
            self.r_late.set(f"{d['late_p99_us']:8.1f}")
        self.r_state.set("PAUSED" if self.loop.paused else
                         ("RUNNING" if self.loop.running else "STOPPED"),
                         AMBER if self.loop.paused else
                         (GREEN if self.loop.running else RED))

        # history ring buffer (cheap: scalar stores, no allocation)
        i = self._head
        self._buf_t[i] = o.t
        b = self._buf
        b["phi"][i] = o.phi_deg
        b["theta"][i] = o.theta_deg
        b["alpha"][i] = o.alpha_deg
        b["beta"][i] = o.beta_deg
        b["psi"][i] = o.psi_deg
        b["stick_lat"][i] = o.stick_lat
        b["stick_long"][i] = o.stick_long
        b["pedal"][i] = o.pedal
        b["throttle"][i] = o.throttle_in
        b["U"][i] = o.U
        b["alt_m"][i] = o.alt_m
        b["p"][i] = o.p_dps
        b["q"][i] = o.q_dps
        b["r"][i] = o.r_dps
        b["hdot"][i] = o.hdot_ft_min
        self._head = (i + 1) % self._cap
        self._count = min(self._count + 1, self._cap)

        msgs = []
        if self.joystick is not None and getattr(self.joystick, "watchdog", None) \
                and self.joystick.watchdog.warnings:
            msgs.append(self.joystick.watchdog.warnings[-1])
        elif getattr(o, "joystick_warning", 0):
            # Physics runs in another process; only the warning COUNT is shared.
            n = int(o.joystick_warning)
            msgs.append(f"{n} joystick stale-axis warning(s) - an axis has "
                        f"stopped updating. Full text is printed on exit.")
        if o.autostop:
            msgs.append("AUTO-STOP triggered")
        if not self.loop.running and self.loop.stop_reason:
            msgs.append(self.loop.stop_reason)
        if msgs:
            self.banner.setText("  ⚠  " + "    |    ".join(msgs))
            self.banner.setStyleSheet(
                "background:#3a2a22; color:#e8a33c; border:1px solid #6a4a30;"
                "border-radius:3px; padding:6px;")
            self.banner.setVisible(True)
        else:
            self.banner.setVisible(False)

    def _ordered(self) -> tuple[np.ndarray, dict[str, np.ndarray]]:
        """Unwrap the ring buffer into chronological order."""
        n, i = self._count, self._head
        if n < self._cap:
            sl = slice(0, n)
            return self._buf_t[sl], {k: v[sl] for k, v in self._buf.items()}
        idx = np.r_[i:self._cap, 0:i]
        return self._buf_t[idx], {k: v[idx] for k, v in self._buf.items()}

    def _refresh_charts(self) -> None:
        if self._count < 2:
            return
        t, h = self._ordered()
        for chart in (self.ch_att, self.ch_cmd, self.ch_spd,
                      self.ch_alt, self.ch_rate):
            chart.update_data(t, h)
