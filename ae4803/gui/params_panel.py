"""Stability & control derivative editor.

Pause, change numbers, resume. Changes are applied as a STEP and take effect on
the next integrator stage - the model re-reads every coefficient each
derivative evaluation, so nothing needs restarting.

Layout: one tab per group (Longitudinal, Lateral/Directional, Control power,
Damping, Mass & inertia, Propulsion), plus an "All coefficients" tab holding
every aero coefficient in sim_init.mat for the cases the curated set misses.

Each row shows the value loaded from sim_init.mat alongside the live one, and
a x1.0 multiplier box, because "half the pitch damping" is how people actually
think about this, rather than "-7.78".
"""
from __future__ import annotations

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QColor, QFont
from PyQt6.QtWidgets import (QAbstractItemView, QCheckBox, QDoubleSpinBox,
                             QHBoxLayout, QHeaderView, QLabel, QLineEdit,
                             QPushButton, QTableWidget, QTableWidgetItem,
                             QTabWidget, QVBoxLayout, QWidget)

from .. import tuning
from .gauges import ACCENT, AMBER, DIM, FG, GREEN

COL_NAME, COL_DEFAULT, COL_VALUE, COL_SCALE, COL_UNITS = range(5)
CHANGED = QColor(60, 48, 24)


class _Table(QTableWidget):
    """One group's worth of tunables."""
    changed = pyqtSignal(str, float)      # key, new value

    def __init__(self, keys: list[str], params, parent=None):
        super().__init__(len(keys), 5, parent)
        self.params = params
        self.keys = keys
        self.defaults: dict[str, float] = {}
        self._building = True

        self.setHorizontalHeaderLabels(
            ["derivative", "sim_init", "value", "x factor", "units"])
        self.verticalHeader().setVisible(False)
        self.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.setAlternatingRowColors(False)
        hh = self.horizontalHeader()
        hh.setSectionResizeMode(COL_NAME, QHeaderView.ResizeMode.Stretch)
        for c in (COL_DEFAULT, COL_VALUE, COL_SCALE, COL_UNITS):
            hh.setSectionResizeMode(c, QHeaderView.ResizeMode.ResizeToContents)

        self._spins: dict[str, QDoubleSpinBox] = {}
        self._scales: dict[str, QDoubleSpinBox] = {}

        for row, key in enumerate(keys):
            t = tuning.tunable_for(key, params)
            val = t.get(params)
            self.defaults[key] = val

            name = QTableWidgetItem(t.label)
            name.setFont(QFont("Consolas", 9))
            if t.description:
                name.setToolTip(t.description)
            self.setItem(row, COL_NAME, name)

            d = QTableWidgetItem(f"{val:.6g}")
            d.setForeground(DIM)
            d.setFont(QFont("Consolas", 9))
            d.setFlags(Qt.ItemFlag.ItemIsEnabled)
            self.setItem(row, COL_DEFAULT, d)

            spin = QDoubleSpinBox()
            spin.setDecimals(6)
            span = max(abs(val) * 10.0, 10.0)
            spin.setRange(-span, span)
            spin.setSingleStep(max(abs(val) * 0.05, 1e-4))
            spin.setValue(val)
            spin.setFont(QFont("Consolas", 9))
            spin.setMinimumWidth(130)
            spin.valueChanged.connect(
                lambda v, k=key: self._on_value(k, v))
            self.setCellWidget(row, COL_VALUE, spin)
            self._spins[key] = spin

            scale = QDoubleSpinBox()
            scale.setDecimals(3)
            scale.setRange(-10.0, 10.0)
            scale.setSingleStep(0.1)
            scale.setValue(1.0)
            scale.setFont(QFont("Consolas", 9))
            scale.setMinimumWidth(90)
            scale.setToolTip("Multiplier on the sim_init value - 0.5 halves it")
            scale.valueChanged.connect(
                lambda f, k=key: self._on_scale(k, f))
            self.setCellWidget(row, COL_SCALE, scale)
            self._scales[key] = scale

            u = QTableWidgetItem(t.units)
            u.setForeground(DIM)
            u.setFlags(Qt.ItemFlag.ItemIsEnabled)
            self.setItem(row, COL_UNITS, u)

        self.resizeRowsToContents()
        self._building = False

    # --- edits ------------------------------------------------------------
    def _on_value(self, key: str, value: float) -> None:
        if self._building:
            return
        d = self.defaults[key]
        sc = self._scales[key]
        sc.blockSignals(True)
        sc.setValue(value / d if abs(d) > 1e-12 else 1.0)
        sc.blockSignals(False)
        self._mark(key)
        self.changed.emit(key, float(value))

    def _on_scale(self, key: str, factor: float) -> None:
        if self._building:
            return
        value = self.defaults[key] * factor
        spin = self._spins[key]
        spin.blockSignals(True)
        spin.setValue(value)
        spin.blockSignals(False)
        self._mark(key)
        self.changed.emit(key, float(value))

    def _mark(self, key: str) -> None:
        row = self.keys.index(key)
        dirty = abs(self._spins[key].value() - self.defaults[key]) > 1e-12
        item = self.item(row, COL_NAME)
        item.setBackground(CHANGED if dirty else QColor(0, 0, 0, 0))
        item.setForeground(AMBER if dirty else FG)

    # --- external ---------------------------------------------------------
    def revert_all(self) -> list[tuple[str, float]]:
        out = []
        for key in self.keys:
            if abs(self._spins[key].value() - self.defaults[key]) > 1e-12:
                out.append((key, self.defaults[key]))
                s = self._spins[key]
                s.blockSignals(True); s.setValue(self.defaults[key]); s.blockSignals(False)
                sc = self._scales[key]
                sc.blockSignals(True); sc.setValue(1.0); sc.blockSignals(False)
                self._mark(key)
        return out

    def dirty_count(self) -> int:
        return sum(1 for k in self.keys
                   if abs(self._spins[k].value() - self.defaults[k]) > 1e-12)

    def filter_rows(self, text: str) -> None:
        text = text.strip().lower()
        for row, key in enumerate(self.keys):
            label = self.item(row, COL_NAME).text().lower()
            self.setRowHidden(row, bool(text) and text not in label)


class ParameterPanel(QWidget):
    """The whole editor, including the pause interlock."""
    parameter_changed = pyqtSignal(str, float)

    def __init__(self, params, loop, parent=None):
        super().__init__(parent)
        self.params = params
        self.loop = loop

        # The window-level stylesheet paints everything dark but says nothing
        # about these widget classes, so without this the spin boxes render
        # dark-on-dark and the inactive tabs vanish.
        self.setStyleSheet("""
            QTableWidget { background:#1b1e23; alternate-background-color:#1b1e23;
                           color:#e2e6ec; gridline-color:#2e333c;
                           selection-background-color:#2d3a4a; border:none; }
            QHeaderView::section { background:#242931; color:#8a92a0;
                                   padding:4px 6px; border:0px;
                                   border-bottom:1px solid #343a44; font-size:8pt; }
            QTableWidget::item { padding:2px 6px; }
            QDoubleSpinBox { background:#12151a; color:#e2e6ec;
                             border:1px solid #3a414c; border-radius:3px;
                             padding:2px 4px; selection-background-color:#2d3a4a; }
            QDoubleSpinBox:focus { border:1px solid #68b4ff; }
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
                background:#242931; border:0px; width:14px; }
            QDoubleSpinBox::up-arrow { image:none; border-left:4px solid transparent;
                border-right:4px solid transparent; border-bottom:5px solid #8a92a0; }
            QDoubleSpinBox::down-arrow { image:none; border-left:4px solid transparent;
                border-right:4px solid transparent; border-top:5px solid #8a92a0; }
            QLineEdit { background:#12151a; color:#e2e6ec;
                        border:1px solid #3a414c; border-radius:3px; padding:3px 6px; }
            QTabBar::tab { background:#2b303a; color:#ffffff;
                           padding:6px 12px; border:1px solid #434a56;
                           border-bottom:none; font-size:8pt; }
            QTabBar::tab:selected { background:#3d4756; color:#ffffff;
                                    border-color:#5a6578; }
            QTabBar::tab:!selected { color:#ffffff; }
            QTabBar::tab:hover { background:#353c48; }
            QTabWidget::pane { border:1px solid #343a44; top:-1px; }
        """)

        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # --- top bar ---
        bar = QHBoxLayout()
        self.btn_pause = QPushButton("Pause")
        self.btn_pause.setCheckable(True)
        self.btn_pause.toggled.connect(self._toggle_pause)
        bar.addWidget(self.btn_pause)

        self.lbl_state = QLabel("running")
        self.lbl_state.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
        bar.addWidget(self.lbl_state)
        bar.addStretch(1)

        self.search = QLineEdit()
        self.search.setPlaceholderText("filter...")
        self.search.setMaximumWidth(160)
        self.search.textChanged.connect(self._filter)
        bar.addWidget(self.search)

        self.btn_revert = QPushButton("Revert all")
        self.btn_revert.clicked.connect(self._revert_all)
        bar.addWidget(self.btn_revert)
        root.addLayout(bar)

        self.cb_live = QCheckBox("Allow edits while running (otherwise pause first)")
        self.cb_live.setChecked(False)
        self.cb_live.setToolTip(
            "Changing a derivative is a step change. Doing it paused lets you "
            "set several at once and release them together.")
        root.addWidget(self.cb_live)

        # --- tabs ---
        self.tabs = QTabWidget()
        self.tables: list[_Table] = []
        for group in tuning.GROUP_ORDER:
            keys = [t.key for t in tuning.TUNABLES if t.group == group]
            if not keys:
                continue
            tbl = _Table(keys, params)
            tbl.changed.connect(self._on_changed)
            self.tabs.addTab(tbl, group)
            self.tables.append(tbl)

        curated = {t.key for t in tuning.TUNABLES}
        rest = [k for k in tuning.all_aero_keys(params) if k not in curated]
        if rest:
            tbl = _Table(sorted(rest), params)
            tbl.changed.connect(self._on_changed)
            self.tabs.addTab(tbl, f"All others ({len(rest)})")
            self.tables.append(tbl)
        root.addWidget(self.tabs, 1)

        self.status = QLabel("")
        self.status.setWordWrap(True)
        self.status.setFont(QFont("Segoe UI", 8))
        self.status.setStyleSheet(f"color: rgb({DIM.red()},{DIM.green()},{DIM.blue()});")
        root.addWidget(self.status)
        self._refresh_status()

    # --- behaviour --------------------------------------------------------
    def _toggle_pause(self, on: bool) -> None:
        self.loop.paused = bool(on)
        self.btn_pause.setText("Resume" if on else "Pause")
        self._refresh_status()

    def _on_changed(self, key: str, value: float) -> None:
        if not self.cb_live.isChecked() and not self.loop.paused:
            self.status.setText(
                "Pause first, or tick 'allow edits while running'. "
                "(The change was NOT applied.)")
            self.status.setStyleSheet(
                f"color: rgb({AMBER.red()},{AMBER.green()},{AMBER.blue()});")
            return
        self.loop.set_parameter(key, value)
        self.parameter_changed.emit(key, value)
        self._refresh_status(f"{key} -> {value:.6g}")

    def _revert_all(self) -> None:
        if not self.cb_live.isChecked() and not self.loop.paused:
            self.status.setText("Pause first to revert.")
            return
        n = 0
        for tbl in self.tables:
            for key, val in tbl.revert_all():
                self.loop.set_parameter(key, val)
                n += 1
        self._refresh_status(f"reverted {n} parameter(s) to sim_init.mat")

    def _filter(self, text: str) -> None:
        for tbl in self.tables:
            tbl.filter_rows(text)

    def _refresh_status(self, msg: str = "") -> None:
        dirty = sum(t.dirty_count() for t in self.tables)
        paused = bool(getattr(self.loop, "paused", False))
        self.lbl_state.setText("PAUSED" if paused else "running")
        c = AMBER if paused else GREEN
        self.lbl_state.setStyleSheet(f"color: rgb({c.red()},{c.green()},{c.blue()});")
        base = f"{dirty} parameter(s) differ from sim_init.mat."
        if dirty:
            base += " Every change is timestamped into the flight log."
        self.status.setText((msg + "   " if msg else "") + base)
        self.status.setStyleSheet(
            f"color: rgb({DIM.red()},{DIM.green()},{DIM.blue()});")

    def sync_pause_button(self) -> None:
        """Keep the button in step if something else paused the loop."""
        paused = bool(getattr(self.loop, "paused", False))
        if paused != self.btn_pause.isChecked():
            self.btn_pause.blockSignals(True)
            self.btn_pause.setChecked(paused)
            self.btn_pause.setText("Resume" if paused else "Pause")
            self.btn_pause.blockSignals(False)
        self._refresh_status()
