"""Custom-painted cockpit instruments.

Mirrors the Simulink 'Cockpit Gauges' subsystem (SID 561), which uses the
Aerospace dashboard blocks: Airspeed Indicator (Limits [40 130] kts),
Altimeter, Artificial Horizon, Climb Rate Indicator (MaximumRate 4000 ft/min)
and Heading Indicator.

The airspeed dial additionally carries model-validity colour bands. The Beaver
coefficient set this sim uses is a fit about 45 m/s; well away from that the
numbers are extrapolation, and the model's own auto-stop fires outside
40-140 kts. The bands make that visible to the student instead of implicit.
"""
from __future__ import annotations

import math

from PyQt6.QtCore import QPointF, QRectF, Qt
from PyQt6.QtGui import (QBrush, QColor, QFont, QFontMetricsF, QPainter,
                         QPainterPath, QPen)
from PyQt6.QtWidgets import QWidget

MPS_TO_KTS = 1.943845249222

# --- model validity bands, in m/s ----------------------------------------
#: Trim / design point of the coefficient set.
V_DESIGN = 45.0
#: Green: coefficients are a good fit here.
V_GREEN = (35.0, 55.0)
#: Amber: usable but extrapolating.
V_AMBER = (25.0, 65.0)
#: The model's own auto-stop limits (40 kts and 140 kts).
V_STOP_LOW = 40.0 / MPS_TO_KTS      # 20.58 m/s
V_STOP_HIGH = 140.0 / MPS_TO_KTS    # 72.02 m/s

BG = QColor(22, 24, 28)
FG = QColor(226, 230, 236)
DIM = QColor(130, 138, 150)
ACCENT = QColor(104, 180, 255)
GREEN = QColor(60, 170, 90)
AMBER = QColor(215, 160, 50)
RED = QColor(205, 70, 65)
SKY = QColor(60, 120, 190)
GROUND = QColor(120, 85, 55)


class Instrument(QWidget):
    """Common round-dial scaffolding."""

    def __init__(self, title: str = "", parent=None):
        super().__init__(parent)
        self.title = title
        self.setMinimumSize(150, 150)

    #: Fraction of the widget height reserved BELOW the dial for the numeric
    #: readout. Keeping the numbers outside the dial is the only way to stop
    #: them colliding with the tick labels, which ring the face at 0.60r.
    CAPTION_FRAC = 0.22

    def _dial(self, p: QPainter):
        w, h = self.width(), self.height()
        avail_h = h * (1.0 - self.CAPTION_FRAC)
        side = min(w, avail_h)
        cx, cy = w / 2.0, avail_h / 2.0
        r = side / 2.0 - 6.0
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        p.setPen(QPen(QColor(70, 76, 86), 2))
        p.setBrush(QBrush(QColor(30, 33, 39)))
        p.drawEllipse(QPointF(cx, cy), r, r)
        return cx, cy, r

    def _caption(self, p: QPainter, main: str, sub: str = "",
                 color: QColor = FG):
        """Numeric readout in the strip below the dial."""
        w, h = self.width(), self.height()
        cap = h * self.CAPTION_FRAC
        top = h - cap
        main_h = cap * (0.60 if sub else 0.95)
        f = self._fit(QFont("Consolas", max(9, int(cap * (0.40 if sub else 0.46))),
                            QFont.Weight.Bold), main, w * 0.96)
        p.setFont(f)
        p.setPen(QPen(color))
        p.drawText(QRectF(0, top, w, main_h),
                   Qt.AlignmentFlag.AlignCenter, main)
        if sub:
            f2 = self._fit(QFont("Segoe UI", max(7, int(cap * 0.26))), sub, w * 0.96)
            p.setFont(f2)
            p.setPen(QPen(DIM))
            p.drawText(QRectF(0, top + cap * 0.58, w, cap * 0.40),
                       Qt.AlignmentFlag.AlignCenter, sub)

    @staticmethod
    def _fit(font: QFont, text: str, max_width: float) -> QFont:
        """Shrink the font until the text fits, so a wide value (a five-digit
        altitude, both attitude angles) cannot spill into the next gauge."""
        for _ in range(12):
            if QFontMetricsF(font).horizontalAdvance(text) <= max_width:
                break
            size = font.pointSize()
            if size <= 7:
                break
            font.setPointSize(size - 1)
        return font

    def _label(self, p: QPainter, cx: float, cy: float, r: float, text: str,
               dy: float = 0.52, size: float = 0.13, color: QColor = DIM):
        f = QFont("Segoe UI", max(7, int(r * size)))
        p.setFont(f)
        p.setPen(QPen(color))
        p.drawText(QRectF(cx - r, cy + r * dy, 2 * r, r * 0.3),
                   Qt.AlignmentFlag.AlignCenter, text)

    @staticmethod
    def _needle(p: QPainter, cx: float, cy: float, r: float, angle_deg: float,
                length: float, width: float, color: QColor, tail: float = 0.12):
        a = math.radians(angle_deg - 90.0)
        tip = QPointF(cx + length * r * math.cos(a), cy + length * r * math.sin(a))
        back = QPointF(cx - tail * r * math.cos(a), cy - tail * r * math.sin(a))
        p.setPen(QPen(color, max(1.5, width * r)))
        p.drawLine(back, tip)


class AirspeedIndicator(Instrument):
    """Airspeed dial in m/s, with model-validity bands and a kts readout."""

    V_MIN, V_MAX = 0.0, 90.0
    A_MIN, A_MAX = 30.0, 330.0          # sweep, degrees clockwise from 12 o'clock

    def __init__(self, parent=None):
        super().__init__("AIRSPEED", parent)
        self.value = 0.0                 # m/s

    def _angle(self, v: float) -> float:
        v = min(max(v, self.V_MIN), self.V_MAX)
        f = (v - self.V_MIN) / (self.V_MAX - self.V_MIN)
        return self.A_MIN + f * (self.A_MAX - self.A_MIN)

    def paintEvent(self, _):
        p = QPainter(self)
        p.fillRect(self.rect(), BG)
        cx, cy, r = self._dial(p)

        def arc(v0: float, v1: float, color: QColor, width: float, inset: float):
            a0, a1 = self._angle(v0), self._angle(v1)
            rect = QRectF(cx - r * inset, cy - r * inset, 2 * r * inset, 2 * r * inset)
            p.setPen(QPen(color, max(2.0, width * r), Qt.PenStyle.SolidLine,
                          Qt.PenCapStyle.FlatCap))
            # Qt angles: 0 at 3 o'clock, counter-clockwise, in 1/16 degree
            start = int((90.0 - a0) * 16)
            span = int(-(a1 - a0) * 16)
            p.drawArc(rect, start, span)

        arc(self.V_MIN, V_AMBER[0], RED, 0.085, 0.86)
        arc(V_AMBER[0], V_GREEN[0], AMBER, 0.085, 0.86)
        arc(V_GREEN[0], V_GREEN[1], GREEN, 0.085, 0.86)
        arc(V_GREEN[1], V_AMBER[1], AMBER, 0.085, 0.86)
        arc(V_AMBER[1], self.V_MAX, RED, 0.085, 0.86)

        # auto-stop limit ticks
        for v in (V_STOP_LOW, V_STOP_HIGH):
            a = math.radians(self._angle(v) - 90.0)
            p.setPen(QPen(RED, max(2.0, 0.025 * r)))
            p.drawLine(QPointF(cx + 0.72 * r * math.cos(a), cy + 0.72 * r * math.sin(a)),
                       QPointF(cx + 0.95 * r * math.cos(a), cy + 0.95 * r * math.sin(a)))

        # ticks and numbers every 10 m/s
        f = QFont("Segoe UI", max(6, int(r * 0.11)))
        p.setFont(f)
        v = self.V_MIN
        while v <= self.V_MAX + 1e-9:
            a = math.radians(self._angle(v) - 90.0)
            major = (abs(v % 10.0) < 1e-9)
            r0 = 0.72 if major else 0.78
            p.setPen(QPen(FG if major else DIM, max(1.0, (0.018 if major else 0.01) * r)))
            p.drawLine(QPointF(cx + r0 * r * math.cos(a), cy + r0 * r * math.sin(a)),
                       QPointF(cx + 0.82 * r * math.cos(a), cy + 0.82 * r * math.sin(a)))
            if major:
                tx = cx + 0.60 * r * math.cos(a)
                ty = cy + 0.60 * r * math.sin(a)
                p.setPen(QPen(FG))
                p.drawText(QRectF(tx - 0.16 * r, ty - 0.11 * r, 0.32 * r, 0.22 * r),
                           Qt.AlignmentFlag.AlignCenter, f"{int(v)}")
            v += 5.0

        # design-point bug at 45 m/s
        a = math.radians(self._angle(V_DESIGN) - 90.0)
        p.setPen(QPen(ACCENT, max(2.0, 0.03 * r)))
        p.drawLine(QPointF(cx + 0.84 * r * math.cos(a), cy + 0.84 * r * math.sin(a)),
                   QPointF(cx + 0.97 * r * math.cos(a), cy + 0.97 * r * math.sin(a)))

        self._needle(p, cx, cy, r, self._angle(self.value), 0.68, 0.028, FG)
        p.setBrush(QBrush(FG))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(QPointF(cx, cy), r * 0.05, r * 0.05)

        self._caption(p, f"{self.value:.1f} m/s",
                      f"{self.value * MPS_TO_KTS:.1f} kt   AIRSPEED")
        p.end()


class AttitudeIndicator(Instrument):
    def __init__(self, parent=None):
        super().__init__("ATTITUDE", parent)
        self.pitch = 0.0
        self.roll = 0.0

    def paintEvent(self, _):
        p = QPainter(self)
        p.fillRect(self.rect(), BG)
        cx, cy, r = self._dial(p)

        path = QPainterPath()
        path.addEllipse(QPointF(cx, cy), r * 0.92, r * 0.92)
        p.save()
        p.setClipPath(path)
        p.translate(cx, cy)
        p.rotate(-self.roll)
        px_per_deg = r * 0.92 / 35.0
        p.translate(0.0, self.pitch * px_per_deg)

        big = r * 3.0
        p.fillRect(QRectF(-big, -big, 2 * big, big), SKY)
        p.fillRect(QRectF(-big, 0.0, 2 * big, big), GROUND)
        p.setPen(QPen(FG, max(1.5, 0.012 * r)))
        p.drawLine(QPointF(-big, 0.0), QPointF(big, 0.0))

        p.setFont(QFont("Segoe UI", max(6, int(r * 0.09))))
        for d in (-30, -20, -10, 10, 20, 30):
            y = -d * px_per_deg
            half = r * (0.30 if d % 20 == 0 else 0.18)
            p.setPen(QPen(FG, max(1.0, 0.009 * r)))
            p.drawLine(QPointF(-half, y), QPointF(half, y))
            if d % 20 == 0:
                p.drawText(QRectF(half + 0.03 * r, y - 0.08 * r, 0.3 * r, 0.16 * r),
                           Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                           f"{abs(d)}")
        p.restore()

        # roll scale
        p.setPen(QPen(FG, max(1.0, 0.012 * r)))
        for d in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
            a = math.radians(-90.0 + d)
            r0 = 0.80 if d % 30 == 0 or d == 0 else 0.86
            p.drawLine(QPointF(cx + r0 * r * math.cos(a), cy + r0 * r * math.sin(a)),
                       QPointF(cx + 0.92 * r * math.cos(a), cy + 0.92 * r * math.sin(a)))

        a = math.radians(-90.0 + self.roll)
        p.setBrush(QBrush(ACCENT))
        p.setPen(Qt.PenStyle.NoPen)
        tipx, tipy = cx + 0.78 * r * math.cos(a), cy + 0.78 * r * math.sin(a)
        p.drawEllipse(QPointF(tipx, tipy), 0.05 * r, 0.05 * r)

        # fixed aircraft symbol
        p.setPen(QPen(QColor(255, 200, 60), max(2.0, 0.022 * r)))
        p.drawLine(QPointF(cx - 0.34 * r, cy), QPointF(cx - 0.10 * r, cy))
        p.drawLine(QPointF(cx + 0.10 * r, cy), QPointF(cx + 0.34 * r, cy))
        p.drawLine(QPointF(cx, cy - 0.05 * r), QPointF(cx, cy + 0.05 * r))

        self._caption(p, f"φ {self.roll:+.1f}°   θ {self.pitch:+.1f}°", "ATTITUDE")
        p.end()


class Altimeter(Instrument):
    def __init__(self, parent=None):
        super().__init__("ALTITUDE", parent)
        self.alt_ft = 0.0
        self.alt_m = 0.0

    def paintEvent(self, _):
        p = QPainter(self)
        p.fillRect(self.rect(), BG)
        cx, cy, r = self._dial(p)

        p.setFont(QFont("Segoe UI", max(6, int(r * 0.13))))
        for i in range(10):
            a = math.radians(i * 36.0 - 90.0)
            p.setPen(QPen(FG, max(1.5, 0.016 * r)))
            p.drawLine(QPointF(cx + 0.74 * r * math.cos(a), cy + 0.74 * r * math.sin(a)),
                       QPointF(cx + 0.88 * r * math.cos(a), cy + 0.88 * r * math.sin(a)))
            tx, ty = cx + 0.60 * r * math.cos(a), cy + 0.60 * r * math.sin(a)
            p.drawText(QRectF(tx - 0.14 * r, ty - 0.11 * r, 0.28 * r, 0.22 * r),
                       Qt.AlignmentFlag.AlignCenter, f"{i}")
        for i in range(50):
            a = math.radians(i * 7.2 - 90.0)
            p.setPen(QPen(DIM, max(1.0, 0.008 * r)))
            p.drawLine(QPointF(cx + 0.81 * r * math.cos(a), cy + 0.81 * r * math.sin(a)),
                       QPointF(cx + 0.88 * r * math.cos(a), cy + 0.88 * r * math.sin(a)))

        hundreds = (self.alt_ft % 1000.0) / 1000.0 * 360.0
        thousands = (self.alt_ft % 10000.0) / 10000.0 * 360.0
        self._needle(p, cx, cy, r, thousands, 0.46, 0.034, FG)
        self._needle(p, cx, cy, r, hundreds, 0.70, 0.020, FG)
        p.setBrush(QBrush(FG)); p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(QPointF(cx, cy), r * 0.05, r * 0.05)

        self._caption(p, f"{self.alt_ft:,.0f} ft",
                      f"{self.alt_m:.1f} m   ALTITUDE")
        p.end()


class HeadingIndicator(Instrument):
    def __init__(self, parent=None):
        super().__init__("HEADING", parent)
        self.heading = 0.0

    def paintEvent(self, _):
        p = QPainter(self)
        p.fillRect(self.rect(), BG)
        cx, cy, r = self._dial(p)

        p.save()
        p.translate(cx, cy)
        p.rotate(-self.heading)
        p.setFont(QFont("Segoe UI", max(6, int(r * 0.13))))
        for d in range(0, 360, 5):
            a = math.radians(d - 90.0)
            major = (d % 30 == 0)
            r0 = 0.74 if major else 0.82
            p.setPen(QPen(FG if major else DIM, max(1.0, (0.016 if major else 0.008) * r)))
            p.drawLine(QPointF(r0 * r * math.cos(a), r0 * r * math.sin(a)),
                       QPointF(0.88 * r * math.cos(a), 0.88 * r * math.sin(a)))
            if major:
                lbl = {0: "N", 90: "E", 180: "S", 270: "W"}.get(d, f"{d // 10}")
                tx, ty = 0.60 * r * math.cos(a), 0.60 * r * math.sin(a)
                p.save()
                p.translate(tx, ty)
                p.rotate(self.heading)
                p.setPen(QPen(ACCENT if d in (0, 90, 180, 270) else FG))
                p.drawText(QRectF(-0.16 * r, -0.11 * r, 0.32 * r, 0.22 * r),
                           Qt.AlignmentFlag.AlignCenter, lbl)
                p.restore()
        p.restore()

        p.setPen(QPen(QColor(255, 200, 60), max(2.0, 0.02 * r)))
        p.drawLine(QPointF(cx, cy - 0.30 * r), QPointF(cx, cy + 0.26 * r))
        p.drawLine(QPointF(cx - 0.18 * r, cy + 0.06 * r), QPointF(cx + 0.18 * r, cy + 0.06 * r))
        p.setBrush(QBrush(ACCENT)); p.setPen(Qt.PenStyle.NoPen)
        p.drawPolygon(QPointF(cx, cy - 0.90 * r), QPointF(cx - 0.06 * r, cy - 0.78 * r),
                      QPointF(cx + 0.06 * r, cy - 0.78 * r))

        self._caption(p, f"{self.heading % 360.0:05.1f}°", "HEADING")
        p.end()


class ClimbIndicator(Instrument):
    """Vertical speed, ft/min. MaximumRate 4000, matching the Simulink block."""

    MAX_RATE = 4000.0

    def __init__(self, parent=None):
        super().__init__("VERT SPEED", parent)
        self.rate = 0.0

    def _angle(self, v: float) -> float:
        v = min(max(v, -self.MAX_RATE), self.MAX_RATE)
        return 90.0 + (v / self.MAX_RATE) * 150.0

    def paintEvent(self, _):
        p = QPainter(self)
        p.fillRect(self.rect(), BG)
        cx, cy, r = self._dial(p)

        p.setFont(QFont("Segoe UI", max(6, int(r * 0.12))))
        step = 500.0
        v = -self.MAX_RATE
        while v <= self.MAX_RATE + 1e-9:
            a = math.radians(self._angle(v) - 90.0)
            major = abs(v % 1000.0) < 1e-9
            r0 = 0.74 if major else 0.80
            p.setPen(QPen(FG if major else DIM, max(1.0, (0.015 if major else 0.008) * r)))
            p.drawLine(QPointF(cx + r0 * r * math.cos(a), cy + r0 * r * math.sin(a)),
                       QPointF(cx + 0.88 * r * math.cos(a), cy + 0.88 * r * math.sin(a)))
            if major:
                tx, ty = cx + 0.59 * r * math.cos(a), cy + 0.59 * r * math.sin(a)
                p.setPen(QPen(FG))
                p.drawText(QRectF(tx - 0.16 * r, ty - 0.11 * r, 0.32 * r, 0.22 * r),
                           Qt.AlignmentFlag.AlignCenter, f"{int(abs(v) // 1000)}")
            v += step

        col = FG if abs(self.rate) < 2000 else AMBER
        self._needle(p, cx, cy, r, self._angle(self.rate), 0.68, 0.024, col)
        p.setBrush(QBrush(FG)); p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(QPointF(cx, cy), r * 0.05, r * 0.05)

        self._caption(p, f"{self.rate:+,.0f}", "ft/min   VERTICAL SPEED",
                      color=FG if abs(self.rate) < 2000 else AMBER)
        
        p.end()
