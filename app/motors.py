"""Motors page: per-joint control. 4 leg panels laid out like the robot seen from above.

CALIBRATED: slider/steppers in joint degrees (0 = URDF zero pose, same as the sim).
RAW:        slider/steppers in PCA ticks on the joint's channel, no calibration.
Every row shows joint deg, old 0-180 servo deg and ticks.
"""
from PySide6.QtCore import Qt, QRectF, QPointF, QSize
from PySide6.QtGui import QPainter, QColor, QPen
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QSlider, QPushButton

from spidy import calibration as C
from spidy.kinematics import LEGS, JOINTS, JOINT_NAMES
from .scene import HIGHLIGHT
from .widgets import Panel, Segmented, SimView, button, label, stamp, icon, hline
from . import theme

LEG_NAME = {"fl": "front left", "fr": "front right", "bl": "back left", "br": "back right"}


class CenterSlider(QSlider):
    """Thin slider whose fill grows from a 'zero' value, clicking jumps to the click."""
    def __init__(self, parent=None):
        super().__init__(Qt.Horizontal, parent)
        self.zero = 0
        self.fill = theme.ACCENT
        self.setFixedHeight(26)
        self.setCursor(Qt.PointingHandCursor)

    def _x(self, v):
        lo, hi = self.minimum(), self.maximum()
        return 9 + (self.width() - 18) * (v - lo) / max(hi - lo, 1)

    def _v(self, x):
        lo, hi = self.minimum(), self.maximum()
        return int(round(lo + (hi - lo) * min(max((x - 9) / max(self.width() - 18, 1), 0), 1)))

    def mousePressEvent(self, e):
        self.setSliderDown(True)
        self.setValue(self._v(e.position().x()))

    def mouseMoveEvent(self, e):
        if self.isSliderDown():
            self.setValue(self._v(e.position().x()))

    def mouseReleaseEvent(self, e):
        self.setSliderDown(False)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        cy = self.height() / 2
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(theme.SURFACE_3))
        p.drawRoundedRect(QRectF(9, cy - 2, self.width() - 18, 4), 2, 2)
        z = self._x(min(max(self.zero, self.minimum()), self.maximum()))
        x = self._x(self.value())
        p.setBrush(QColor(self.fill))
        p.drawRoundedRect(QRectF(min(z, x), cy - 2, abs(x - z), 4), 2, 2)
        p.setPen(QPen(QColor(theme.TEXT_3), 1))
        p.drawLine(QPointF(z, cy - 6), QPointF(z, cy + 6))
        p.setPen(QPen(QColor(self.fill if self.isSliderDown() else theme.BORDER_2), 2))
        p.setBrush(QColor(theme.TEXT))
        p.drawEllipse(QPointF(x, cy), 7, 7)
        p.end()


def stepper(text, ico=None):
    b = QPushButton(text)
    b.setFixedSize(40 if len(text) > 1 else 30, 28)
    b.setStyleSheet(f"padding: 0; font-family: '{theme.MONO}'; font-size: 11px; color: {theme.TEXT_2};")
    b.setCursor(Qt.PointingHandCursor)
    if ico:
        b.setText("")
        b.setIcon(icon(ico, 12, theme.TEXT_2))
        b.setIconSize(QSize(12, 12))
    return b


class JointRow(QWidget):
    def __init__(self, ctl, name, parent=None):
        super().__init__(parent)
        self.ctl, self.name, self.i = ctl, name, JOINT_NAMES.index(name)
        g = QGridLayout(self)
        g.setContentsMargins(0, 6, 0, 6)
        g.setHorizontalSpacing(8)
        g.setVerticalSpacing(2)
        self.title = label(name.split("_")[1].capitalize())
        self.title.setStyleSheet("font-weight: 600;")
        self.chan = label("", "faint")
        self.slider = CenterSlider()
        self.slider.valueChanged.connect(self._slid)
        self.slider.sliderPressed.connect(lambda: ctl.sim.highlight(name, HIGHLIGHT))
        self.slider.sliderReleased.connect(lambda: ctl.sim.highlight(None))
        self.value = label("", "value")
        self.value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.sub = label("", "faint")
        self.sub.setStyleSheet(f"font-family: '{theme.MONO}'; font-size: 11px;")
        self.sub.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        steps = QHBoxLayout()
        steps.setSpacing(4)
        for txt, s, ico in (("−10", -10, None), ("", -1, "left"), ("", 1, "right"), ("+10", 10, None)):
            b = stepper(txt, ico)
            if abs(s) == 1:
                b.setAutoRepeat(True)
                b.setAutoRepeatDelay(300)
                b.setAutoRepeatInterval(70)
            b.clicked.connect(lambda _=False, s=s: self._jog(s))
            steps.addWidget(b)
        z = stepper("0")
        z.setToolTip("calibrated zero (raw: 400 ticks)")
        z.clicked.connect(lambda: self._jog(None))
        steps.addWidget(z)
        head = QHBoxLayout()
        head.setSpacing(8)
        head.addWidget(self.title)
        head.addWidget(self.chan)
        head.addStretch()
        head.addWidget(self.value)
        g.addLayout(head, 0, 0, 1, 2)
        g.addWidget(self.slider, 1, 0, 1, 2)
        g.addWidget(self.sub, 2, 0)
        g.addLayout(steps, 2, 1)
        g.setColumnStretch(0, 1)
        self.configure()

    @property
    def jc(self):
        return self.ctl.cal.joints[self.name]

    def configure(self):
        self.slider.blockSignals(True)
        if self.ctl.raw_mode:
            self.slider.setRange(C.HARD_MIN, C.HARD_MAX)
            self.slider.zero = C.RAW_90
            self.slider.fill = theme.AMBER
        else:
            self.slider.setRange(int(self.jc.min_deg * 10), int(self.jc.max_deg * 10))
            self.slider.zero = 0
            self.slider.fill = theme.ACCENT
        self.slider.blockSignals(False)
        self.chan.setText(f"ch {self.jc.channel}")
        self.slider.update()

    def _slid(self, v):
        if self.ctl.raw_mode:
            self.ctl.raw_set(self.jc.channel, v)
        else:
            self.ctl.set_joint(self.name, v / 10)

    def _jog(self, step):
        if self.ctl.raw_mode:
            cur = self.ctl.raw_ticks[self.jc.channel] or C.RAW_90
            self.ctl.raw_set(self.jc.channel, C.RAW_90 if step is None else cur + step)
        else:
            cur = self.ctl.target[self.i]
            self.ctl.set_joint(self.name, 0.0 if step is None else self.jc.clamp_deg(cur + step))

    def refresh(self):
        jc = self.jc
        if self.ctl.raw_mode:
            ticks = self.ctl.raw_ticks[jc.channel] or 0
            deg = jc.to_deg(ticks) if ticks else 0.0
            val = ticks
        else:
            deg = self.ctl.q[self.i]
            ticks = jc.to_ticks(deg)
            val = int(round(self.ctl.target[self.i] * 10))
        if not self.slider.isSliderDown() and self.slider.value() != val:
            self.slider.blockSignals(True)
            self.slider.setValue(val)
            self.slider.blockSignals(False)
        at_limit = not (jc.min_deg < deg < jc.max_deg)
        txt = f"{deg:+.1f}°"
        if self.value.text() != txt:
            self.value.setText(txt)
        if at_limit != getattr(self, "_at_limit", None):        # restyling is expensive: only on change
            self._at_limit = at_limit
            self.value.setStyleSheet(f"color: {theme.AMBER};" if at_limit else "")
        sub = f"{C.servo_deg(ticks):5.1f}° srv · {ticks} t" + ("  · limit" if at_limit else "")
        if self.sub.text() != sub:
            self.sub.setText(sub)


class MotorsTab(QWidget):
    def __init__(self, ctl, parent=None):
        super().__init__(parent)
        self.ctl = ctl
        root = QVBoxLayout(self)
        root.setContentsMargins(18, 16, 18, 18)
        root.setSpacing(14)

        bar = QHBoxLayout()
        bar.setSpacing(8)
        self.mode = Segmented([("cal", "Calibrated"), ("raw", "Raw ticks")],
                              colors={"cal": theme.ACCENT, "raw": theme.AMBER})
        self.mode.changed.connect(lambda k: self._set_raw(k == "raw"))
        bar.addWidget(self.mode)
        self.hint = label("", "faint")
        bar.addWidget(self.hint)
        bar.addStretch()
        bar.addWidget(button("Raw 90", self._raw90, name="ghost", ico="swap",
                             tip="every joint channel to 400 ticks: the 'servo 90' the legs were assembled at"))
        bar.addWidget(button("Cal zero", self._cal_zero, name="ghost", ico="gauge",
                             tip="every joint to its calibrated zero: femur flat, tibia vertical"))
        bar.addWidget(button("Reset sim", ctl.reset_sim, name="ghost", ico="refresh",
                             tip="put the simulated robot back on its feet"))
        bar.addWidget(button("Standf", lambda: self._pose_move(self.ctl.gait.stance_deg, "standf"), ico="up"))
        bar.addWidget(button("Sit", lambda: self._pose_move(self.ctl.poses.get("Sit"), "sit")
                             if "Sit" in self.ctl.poses.poses else None, ico="down"))
        root.addLayout(bar)

        strips = QHBoxLayout()            # four channel strips, like a mixing desk
        strips.setSpacing(14)
        self.rows = []
        for leg in LEGS:
            p = Panel(LEG_NAME[leg], actions=[stamp(leg.upper())])
            p.body.setSpacing(6)
            for k, j in enumerate(JOINTS):
                if k:
                    p.body.addWidget(hline())
                r = JointRow(ctl, f"{leg}_{j}")
                self.rows.append(r)
                p.body.addWidget(r)
            strips.addWidget(p)
        root.addLayout(strips)
        self.view = SimView(ctl.sim, distance=0.32)
        self.view.overlay = "the joint you drag lights up"
        root.addWidget(self.view, 1)
        ctl.mode_changed.connect(self._mode)
        self._mode(ctl.mode)

        ctl.ticked.connect(self.refresh)
        ctl.calibration_changed.connect(self._reconfigure)
        self._set_raw(False)

    def _mode(self, m):
        self.view.badge = {"sim": "SIM · PHYSICS", "real": "REAL · MIRROR", "both": "REAL + SIM SHADOW"}[m]
        self.view.badge_color = theme.MODE_COLOR[m]

    def _set_raw(self, on):
        self.ctl.set_raw_mode(on)
        self.mode.set("raw" if on else "cal")
        self.hint.setText("PCA ticks per channel, no calibration · streaming paused" if on else
                          "joint degrees · + means the same as in the sim")
        self._reconfigure()

    def _reconfigure(self):
        for r in self.rows:
            r.configure()

    def _raw90(self):
        self._set_raw(True)
        pulses = [0] * C.CHANNELS
        for n in JOINT_NAMES:
            pulses[self.ctl.cal.joints[n].channel] = C.RAW_90
        self.ctl.raw_set_many(pulses)
        self.ctl.log.emit("all joint channels -> 400 ticks (raw servo 90)")

    def _cal_zero(self):
        self._set_raw(False)
        self.ctl.move_to([0.0] * 12, 1.2, "cal zero")

    def _pose_move(self, q, label_):
        self._set_raw(False)
        self.ctl.move_to(q, 1.2, label_)

    def refresh(self):
        if self.isVisible():
            for r in self.rows:
                r.refresh()
