"""Tab 3: Calibration wizard.

  1 CHANNELS   wiggle each PCA channel, say which joint moved
  2 ZERO       jog each joint until it matches the URDF zero pose -> center
  3 DIRECTION  move +15 deg, compare with the 3D view -> keep or flip
  4 LIMITS     jog to the safe ends -> min / max (joint degrees)
  5 SCALE      measure the real servo scale (IMU rock test or phone level)
  6 REVIEW     check, save (with backup), push the boot pose to the robot

Steps 1-4 drive single channels RAW (M,<ch>,<ticks>), computed from the
calibration being edited but WITHOUT its limits, so you can find them.
The left 3D view is a separate reference model: it shows what SHOULD happen.
"""
import json

import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (QWidget, QHBoxLayout, QVBoxLayout, QGridLayout, QStackedWidget, QScrollArea,
                               QComboBox, QTableWidget, QTableWidgetItem, QLabel, QDoubleSpinBox, QCheckBox,
                               QHeaderView, QSizePolicy)

from spidy import calibration as C
from spidy import imu
from spidy.kinematics import JOINT_NAMES, angles_to_feet
from spidy.sim import SimWorld
from .widgets import Card, Panel, SimView, StepList, button, label, stamp
from .scene import style, HIGHLIGHT
from . import theme
from .controller import CAL_PATH

HINT = {"hip": "leg points straight out sideways, 90° to the body",
        "knee": "femur horizontal, parallel to the body plate",
        "claw": "tibia vertical, straight down"}


def raw_ticks(jc, deg):
    """Joint degrees -> ticks with this calibration, ignoring its min/max (hard limits only)."""
    return int(round(np.clip(jc.center + jc.direction * jc.ticks_per_deg * deg, C.HARD_MIN, C.HARD_MAX)))


def describe_motion(name, deg=15.0):
    """In words: what should the foot do when this joint goes +deg from zero?"""
    i = JOINT_NAMES.index(name)
    q = np.zeros(12)
    a = angles_to_feet(q)[name[:2]]
    q[i] = np.radians(deg)
    b = angles_to_feet(q)[name[:2]]
    d = b - a
    words = {0: ("FORWARD", "BACKWARD"), 1: ("LEFT", "RIGHT"), 2: ("UP", "DOWN")}
    ax = int(np.argmax(np.abs(d)))
    return f"foot moves {(words[ax][0] if d[ax] > 0 else words[ax][1]).lower()}, about {abs(d[ax]) * 1000:.0f} mm"


class JointPicker(QWidget):
    """Joint combo + prev/next. Calls on_pick(name)."""
    def __init__(self, on_pick, parent=None):
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.combo = QComboBox()
        self.combo.addItems(JOINT_NAMES)
        self.combo.currentTextChanged.connect(on_pick)
        lay.setSpacing(6)
        lay.addWidget(button("", lambda: self.step(-1), ico="left", tip="previous joint"))
        lay.addWidget(self.combo, 1)
        lay.addWidget(button("", lambda: self.step(1), ico="right", tip="next joint"))

    @property
    def name(self):
        return self.combo.currentText()

    def step(self, k):
        self.combo.setCurrentIndex((self.combo.currentIndex() + k) % 12)


class Page(QWidget):
    title = ""

    def __init__(self, wiz):
        super().__init__()
        self.wiz, self.ctl = wiz, wiz.ctl
        self.lay = QVBoxLayout(self)
        self.lay.setContentsMargins(0, 0, 0, 0)
        self.lay.setSpacing(10)
        self.lay.addWidget(label(self.title, "title"))

    def text(self, t):
        lab = label(t, "dim")
        lab.setWordWrap(True)
        lab.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Minimum)   # wrapped text keeps its height
        self.lay.addWidget(lab)
        return lab

    def entered(self):
        pass

    @property
    def cal(self):
        return self.ctl.cal

    def drive(self, name, ticks):
        self.ctl.set_raw_mode(True)
        self.ctl.raw_set(self.cal.joints[name].channel, ticks)

    def show_ref(self, q_deg, highlight=None):
        self.wiz.ref.highlight(highlight, HIGHLIGHT)
        self.wiz.ref.mirror(q_deg)


# ======================================================================== 1
class ChannelsPage(Page):
    title = "Channels"

    def __init__(self, wiz):
        super().__init__(wiz)
        self.text("Robot on a stand, legs free. Press WIGGLE on a channel, watch which joint "
                  "moves, pick that joint. Then APPLY. This settles the old BL/FL/FR/BR order mess for good.")
        self.table = QTableWidget(C.CHANNELS, 3)
        self.table.setHorizontalHeaderLabels(["CH", "JOINT", ""])
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.table.verticalHeader().setDefaultSectionSize(40)
        self.table.setShowGrid(False)
        self.combos = []
        for ch in range(C.CHANNELS):
            self.table.setItem(ch, 0, QTableWidgetItem(str(ch)))
            cb = QComboBox()
            cb.addItems(["—"] + JOINT_NAMES)
            cb.currentTextChanged.connect(lambda t: self._show(t))
            self.combos.append(cb)
            self.table.setCellWidget(ch, 1, cb)
            self.table.setCellWidget(ch, 2, button("wiggle", lambda _=False, c=ch: self.wiggle(c)))
        self.lay.addWidget(self.table, 1)
        row = QHBoxLayout()
        row.addWidget(button("apply mapping", self.apply, name="accent"))
        self.msg = label("")
        row.addWidget(self.msg, 1)
        self.lay.addLayout(row)
        self._seq = []
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._wiggle_step)

    def entered(self):
        by_ch = {j.channel: n for n, j in self.cal.joints.items()}
        for ch, cb in enumerate(self.combos):
            cb.blockSignals(True)
            cb.setCurrentText(by_ch.get(ch, "—"))
            cb.blockSignals(False)
        self.show_ref(np.zeros(12))

    def _show(self, name):
        self.show_ref(np.zeros(12), name if name in JOINT_NAMES else None)

    def wiggle(self, ch):
        self.ctl.set_raw_mode(True)
        base = self.ctl.raw_ticks[ch] or C.RAW_90
        self._seq = [(ch, base + 40), (ch, base - 40), (ch, base + 40), (ch, base - 40), (ch, base)]
        self._show(self.combos[ch].currentText())
        self.timer.start(250)

    def _wiggle_step(self):
        if not self._seq:
            self.timer.stop()
            return
        ch, t = self._seq.pop(0)
        self.ctl.raw_set(ch, t)

    def apply(self):
        chosen = {cb.currentText(): ch for ch, cb in enumerate(self.combos) if cb.currentText() != "—"}
        names = [cb.currentText() for cb in self.combos if cb.currentText() != "—"]
        dup = sorted({n for n in names if names.count(n) > 1})
        missing = sorted(set(JOINT_NAMES) - set(chosen))
        if dup or missing:
            self.msg.setText(f"<span style='color:{theme.RED}'>duplicates: {dup} missing: {missing}</span>")
            return
        for n, ch in chosen.items():
            self.cal.joints[n].channel = ch
        self.ctl.calibration_changed.emit()
        self.wiz.mark_done(0)
        self.msg.setText(f"<span style='color:{theme.GREEN}'>mapping applied (not saved yet)</span>")


# ======================================================================== 2
class ZeroPage(Page):
    title = "Zero pose"

    def __init__(self, wiz):
        super().__init__(wiz)
        self.text("For each joint: jog until the REAL joint matches the 3D view, then SET CENTER. "
                  "Order that works best: all hips, then knees, then claws. A phone level app on "
                  "the femur helps for the knees.")
        self.picker = JointPicker(self.pick)
        self.lay.addWidget(self.picker)
        self.hint = label("", "callout")
        self.hint.setWordWrap(True)
        self.lay.addWidget(self.hint)
        self.read = label("", "big")
        self.lay.addWidget(self.read)
        jog = QHBoxLayout()
        for s in (-20, -5, -1, 1, 5, 20):
            b = button(f"{s:+d}")
            if abs(s) == 1:
                b.setAutoRepeat(True)
                b.setAutoRepeatInterval(80)
            b.clicked.connect(lambda _=False, s=s: self.jog(s))
            jog.addWidget(b)
        self.lay.addLayout(jog)
        row = QHBoxLayout()
        row.addWidget(button("set center here", self.set_center, name="accent"))
        row.addWidget(button("go to saved center", lambda: self.pick(self.picker.name)))
        row.addWidget(button("all joints to zero", self.all_zero,
                             tip="every joint to its current center (raw), to compare the whole pose"))
        self.lay.addLayout(row)
        self.lay.addStretch()
        self.ticks = C.RAW_90

    def entered(self):
        self.pick(self.picker.name)

    def pick(self, name):
        jc = self.cal.joints[name]
        self.ticks = int(round(jc.center))
        self.drive(name, self.ticks)
        self.hint.setText(f"<b style='color:{theme.ACCENT}'>{name}</b>&nbsp;&nbsp;{HINT[name[3:]]}")
        self.show_ref(np.zeros(12), name)
        self._readout()

    def jog(self, s):
        self.ticks = int(np.clip(self.ticks + s, C.HARD_MIN, C.HARD_MAX))
        self.drive(self.picker.name, self.ticks)
        self._readout()

    def _readout(self):
        jc = self.cal.joints[self.picker.name]
        self.read.setText(f"{self.ticks} ticks · {C.servo_deg(self.ticks):.1f}° servo · "
                          f"center {jc.center:.0f} ({self.ticks - jc.center:+.0f})")

    def set_center(self):
        self.cal.joints[self.picker.name].center = float(self.ticks)
        self.ctl.log.emit(f"{self.picker.name}: center = {self.ticks}")
        self.centered = getattr(self, "centered", set()) | {self.picker.name}
        if len(self.centered) == 12:
            self.wiz.mark_done(1)
        self._readout()
        self.picker.step(1)

    def all_zero(self):
        self.ctl.set_raw_mode(True)
        p = [0] * C.CHANNELS
        for n, jc in self.cal.joints.items():
            p[jc.channel] = int(round(jc.center))
        self.ctl.raw_set_many(p)
        self.show_ref(np.zeros(12))


# ======================================================================== 3
class DirectionPage(Page):
    title = "Direction"
    TEST = 15.0

    def __init__(self, wiz):
        super().__init__(wiz)
        self.text("TEST moves the real joint +15° and the 3D view does the same. If they move the "
                  "same way press SAME, otherwise FLIP. Do the zero pose first.")
        self.picker = JointPicker(self.pick)
        self.lay.addWidget(self.picker)
        self.expect = label("", "callout")
        self.expect.setWordWrap(True)
        self.lay.addWidget(self.expect)
        row = QHBoxLayout()
        row.addWidget(button("Test +15°", self.test, name="accent", ico="play"))
        row.addWidget(button("Same as 3D", self.same, ico="check"))
        row.addWidget(button("Opposite, flip", self.flip, ico="swap"))
        self.lay.addLayout(row)
        self.dir_lab = label("", "faint")
        self.dir_lab.setWordWrap(True)
        self.dir_lab.setTextFormat(Qt.RichText)
        self.lay.addWidget(self.dir_lab)
        self.chips = QGridLayout()
        self.chip = {}
        for k, n in enumerate(JOINT_NAMES):
            c = label(n.upper(), "mono")
            c.setAlignment(Qt.AlignCenter)
            self.chip[n] = c
            self.chips.addWidget(c, k // 3, k % 3)
        self.lay.addLayout(self.chips)
        self.lay.addStretch()
        self.checked = set()
        self.anim_t = None
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._anim)

    def entered(self):
        self.pick(self.picker.name)
        self._chips()

    def _dir_text(self, flipped=False):
        jc = self.cal.joints[self.picker.name]
        real = self.ctl.link.connected and self.ctl.mode in ("real", "both")
        note = ("" if real else f"<br><span style='color:{theme.AMBER}'>Not connected to the robot: the test only "
                "moves the 3D reference. Flip changes the calibration for the REAL servo, so the 3D view "
                "never flips.</span>")
        head = f"<b style='color:{theme.ACCENT}'>flipped</b> → " if flipped else ""
        self.dir_lab.setText(f"{head}calibration direction for {self.picker.name}: "
                             f"<b style='color:{theme.TEXT}'>{jc.direction:+d}</b>{note}")

    def pick(self, name):
        jc = self.cal.joints[name]
        self.drive(name, raw_ticks(jc, 0))
        self._dir_text()
        self.expect.setText(f"<b style='color:{theme.ACCENT}'>{name} +15°</b>&nbsp;&nbsp;{describe_motion(name)}")
        self.show_ref(np.zeros(12), name)

    def test(self):
        self.anim_t = 0.0
        self.timer.start(20)

    def _anim(self):
        """0 -> +15 in 0.5 s, hold 1 s, back in 0.5 s. Real joint and 3D view together."""
        self.anim_t += 0.02
        t = self.anim_t
        s = min(t / 0.5, 1) if t < 1.5 else max(0, 1 - (t - 1.5) / 0.5)
        deg = self.TEST * (s * s * (3 - 2 * s))
        name = self.picker.name
        self.drive(name, raw_ticks(self.cal.joints[name], deg))
        q = np.zeros(12)
        q[JOINT_NAMES.index(name)] = deg
        self.show_ref(q, name)
        if t >= 2.0:
            self.timer.stop()

    def same(self):
        self.checked.add(self.picker.name)
        self._chips()
        self.picker.step(1)

    def flip(self):
        jc = self.cal.joints[self.picker.name]
        jc.direction *= -1
        self.ctl.log.emit(f"{self.picker.name}: direction flipped to {jc.direction:+d}")
        self.checked.add(self.picker.name)
        self._chips()
        self._dir_text(flipped=True)
        self.drive(self.picker.name, raw_ticks(jc, 0))

    def _chips(self):
        for n, c in self.chip.items():
            ok = n in self.checked
            c.setStyleSheet(f"border: 1px solid {theme.ACCENT if ok else theme.BORDER_2}; border-radius: 6px;"
                            f"padding: 4px; color: {theme.ACCENT if ok else theme.TEXT_3};"
                            f"background: {'#123026' if ok else 'transparent'};")
        if len(self.checked) == 12:
            self.wiz.mark_done(2)


# ======================================================================== 4
class LimitsPage(Page):
    title = "Limits"

    def __init__(self, wiz):
        super().__init__(wiz)
        self.text("Jog to where the joint would hit the frame, a wire, or the servo starts to buzz, "
                  "back off a few degrees, SET MIN / SET MAX. Limits are in joint degrees and "
                  "everything (poses, gait, sliders) is clamped to them.")
        self.picker = JointPicker(self.pick)
        self.lay.addWidget(self.picker)
        self.read = label("", "big")
        self.lay.addWidget(self.read)
        jog = QHBoxLayout()
        for s in (-10, -1, 1, 10):
            b = button(f"{s:+d}°")
            if abs(s) == 1:
                b.setAutoRepeat(True)
                b.setAutoRepeatInterval(90)
            b.clicked.connect(lambda _=False, s=s: self.jog(s))
            jog.addWidget(b)
        self.lay.addLayout(jog)
        row = QHBoxLayout()
        row.addWidget(button("set min here", lambda: self.set_lim("min"), name="accent"))
        row.addWidget(button("set max here", lambda: self.set_lim("max"), name="accent"))
        row.addWidget(button("reset ±60°", self.reset))
        self.lay.addLayout(row)
        self.lay.addStretch()
        self.deg = 0.0

    def entered(self):
        self.pick(self.picker.name)

    def pick(self, name):
        self.deg = 0.0
        self.jog(0)

    def jog(self, s):
        name = self.picker.name
        self.deg += s
        jc = self.cal.joints[name]
        self.drive(name, raw_ticks(jc, self.deg))
        q = np.zeros(12)
        q[JOINT_NAMES.index(name)] = self.deg
        self.show_ref(q, name)
        self.read.setText(f"{self.deg:+.0f}° · range {jc.min_deg:+.0f}° … {jc.max_deg:+.0f}°")

    def set_lim(self, which):
        jc = self.cal.joints[self.picker.name]
        if which == "min":
            jc.min_deg = min(self.deg, jc.max_deg - 1)
        else:
            jc.max_deg = max(self.deg, jc.min_deg + 1)
        self.ctl.calibration_changed.emit()
        self.wiz.mark_done(3)
        self.jog(0)

    def reset(self):
        jc = self.cal.joints[self.picker.name]
        jc.min_deg, jc.max_deg = -60.0, 60.0
        self.ctl.calibration_changed.emit()
        self.jog(0)


# ======================================================================== 5
class ScalePage(Page):
    title = "Scale"

    def __init__(self, wiz):
        super().__init__(wiz)
        self.scale = label("", "big")
        self.lay.addWidget(self.scale)

        a = Card("a · imu rock test")
        t = label("Robot standing on a flat floor, armed (or target Sim to try it). Front knees go to "
                  "−D then +D, the IMU measures the pitch change, the physics sim works out how far "
                  "the knees really moved.", "dim")
        t.setWordWrap(True)
        t.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Minimum)
        a.body.addWidget(t)
        row = QHBoxLayout()
        row.addWidget(label("D"))
        self.D = QDoubleSpinBox()
        self.D.setRange(5, 25)
        self.D.setValue(15)
        self.D.setSuffix(" °")
        row.addWidget(self.D)
        self.run_btn = button("run test", self.run, name="accent")
        row.addWidget(self.run_btn)
        a.body.addLayout(row)
        self.result = label("")
        self.result.setWordWrap(True)
        self.result.setTextFormat(Qt.RichText)
        a.body.addWidget(self.result)
        self.lay.addWidget(a)

        b = Card("b · phone level")
        t2 = label("Phone level app on a femur: note the angle, Move, type in how much it really changed.", "dim")
        t2.setWordWrap(True)
        t2.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Minimum)
        b.body.addWidget(t2)
        self.picker = JointPicker(lambda n: None)
        self.picker.combo.setCurrentText("fl_knee")
        b.body.addWidget(self.picker)
        row2 = QHBoxLayout()
        self.X = QDoubleSpinBox()
        self.X.setRange(5, 60)
        self.X.setValue(30)
        self.X.setSuffix("°")
        self.meas = QDoubleSpinBox()
        self.meas.setRange(1, 120)
        self.meas.setValue(30)
        self.meas.setSuffix("°")
        row2.addWidget(label("cmd", "faint"))
        row2.addWidget(self.X)
        row2.addWidget(button("move", self.manual_move))
        row2.addWidget(label("real", "faint"))
        row2.addWidget(self.meas)
        row2.addWidget(button("compute", self.manual_compute))
        b.body.addLayout(row2)
        self.lay.addWidget(b)

        row3 = QHBoxLayout()
        self.keep = QCheckBox("Rescale poses + limits too")
        self.keep.setToolTip("keeps every saved pose and limit at the same physical position")
        self.keep.setChecked(True)
        row3.addWidget(self.keep, 1)
        self.apply_btn = button("apply new scale", self.apply, name="accent")
        self.apply_btn.setEnabled(False)
        row3.addWidget(self.apply_btn)
        self.lay.addLayout(row3)
        self.lay.addStretch()

        self.new_tpd = None
        self.samples = None
        self.steps = []
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._step)
        self.ctl.imu_sample.connect(self._imu)

    def entered(self):
        self.ctl.set_raw_mode(False)
        self._show_scale()
        self.show_ref(self.ctl.gait.stance_deg)

    def _show_scale(self):
        t = np.mean([j.ticks_per_deg for j in self.cal.joints.values()])
        extra = f" → <b style='color:{theme.ORANGE}'>{self.new_tpd:.3f}</b>" if self.new_tpd else ""
        self.scale.setText(f"ticks/deg now {t:.3f}{extra}")

    # ---- IMU rock test (state machine on a timer: each step = (seconds, action))
    def run(self):
        c = self.ctl
        if c.mode == "real" and not (c.link.connected and c.armed):
            self.result.setText(f"<span style='color:{theme.RED}'>connect + ARM first (or switch target to SIM to try it)</span>")
            return
        c.set_raw_mode(False)
        c.imu_polling = c.mode in ("real", "both")
        st, D = c.gait.stance_deg, self.D.value()
        lo, hi = imu.front_knee_pose(st, -D), imu.front_knee_pose(st, D)
        self.g = []
        self.steps = [(2.5, lambda: c.move_to(st, 1.5, "scale test")),
                      (2.5, lambda: c.move_to(lo, 1.5, "scale test -D")),
                      (1.0, self._collect_start), (0.0, self._collect_end),
                      (2.5, lambda: c.move_to(hi, 1.5, "scale test +D")),
                      (1.0, self._collect_start), (0.0, self._collect_end),
                      (0.1, lambda: c.move_to(st, 1.5, "scale test end")),
                      (0.0, self._finish)]
        self.run_btn.setEnabled(False)
        self.result.setText("running… don't touch the robot")
        self._step()

    def _step(self):
        self.timer.stop()
        if not self.steps:
            return
        secs, act = self.steps.pop(0)
        act()
        self.timer.start(int(secs * 1000))

    def _collect_start(self):
        self.samples = []

    def _imu(self, acc, _gyro):
        if self.samples is not None and self.ctl.mode != "sim":
            self.samples.append(acc)

    def _collect_end(self):
        if self.ctl.mode == "sim":     # simulated IMU: gravity seen from the sim body
            R = self.ctl.sim.d.body(self.ctl.sim.base).xmat.reshape(3, 3)
            self.samples = [R.T @ np.array([0, 0, 9.81])]
        if not self.samples:
            self.steps = [(0.0, lambda: self._fail("no IMU samples (is the MPU6050 answering?)"))]
        else:
            self.g.append(np.mean(self.samples, axis=0))
        self.samples = None

    def _fail(self, why):
        self.run_btn.setEnabled(True)
        self.result.setText(f"<span style='color:{theme.RED}'>{why}</span>")

    def _finish(self):
        self.run_btn.setEnabled(True)
        D = self.D.value()
        st = self.ctl.gait.stance_deg
        meas = imu.angle_between_deg(*self.g)
        true = imu.true_delta_from_pitch_sim(st, meas)
        tpd = np.mean([j.ticks_per_deg for j in self.cal.joints.values()])
        self.new_tpd = imu.corrected_scale(tpd, D, true)
        self.apply_btn.setEnabled(True)
        self.result.setText(
            f"pitch swing measured <b>{meas:.2f}°</b> (expected {imu.sim_swing_pitch(st, D):.2f}° if the scale were right)<br>"
            f"knees really moved ±<b>{true:.2f}°</b> for ±{D:.1f}° commanded → factor <b>{D / true:.3f}</b>")
        self._show_scale()

    # ---- manual
    def manual_move(self):
        name = self.picker.name
        q = np.array(self.ctl.gait.stance_deg)
        q[JOINT_NAMES.index(name)] = self.X.value()
        self.ctl.move_to(q, 1.0, "scale manual")

    def manual_compute(self):
        tpd = np.mean([j.ticks_per_deg for j in self.cal.joints.values()])
        self.new_tpd = imu.corrected_scale(tpd, self.X.value(), self.meas.value())
        self.apply_btn.setEnabled(True)
        self._show_scale()

    def apply(self):
        if self.new_tpd:
            self.ctl.apply_scale(self.new_tpd, self.keep.isChecked())
            self.wiz.mark_done(4)
            self.new_tpd = None
            self.apply_btn.setEnabled(False)
            self._show_scale()


# ======================================================================== 6
class ReviewPage(Page):
    title = "Review & save"
    COLS = ["joint", "ch", "center", "dir", "ticks/°", "min°", "max°"]

    def __init__(self, wiz):
        super().__init__(wiz)
        self.table = QTableWidget(12, len(self.COLS))
        self.table.setHorizontalHeaderLabels([c.upper() for c in self.COLS])
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.lay.addWidget(self.table, 1)
        self.problems = label("")
        self.problems.setWordWrap(True)
        self.problems.setTextFormat(Qt.RichText)
        self.lay.addWidget(self.problems)
        row = QHBoxLayout()
        row.addWidget(button("save", self.save, name="accent"))
        row.addWidget(button("revert to file", self.revert))
        row.addStretch()
        self.boot = QComboBox()
        row.addWidget(label("boot pose"))
        row.addWidget(self.boot)
        row.addWidget(button("send to robot", self.boot_send,
                             tip="the robot takes this pose at power-on, before the PC connects"))
        self.lay.addLayout(row)

    def entered(self):
        self.ctl.set_raw_mode(False)
        saved = {}
        try:
            with open(CAL_PATH, encoding="utf-8") as f:
                saved = json.load(f)["joints"]
        except (OSError, KeyError, ValueError):
            pass
        for r, n in enumerate(JOINT_NAMES):
            j = self.cal.joints[n]
            vals = [n, j.channel, f"{j.center:.0f}", f"{j.direction:+d}", f"{j.ticks_per_deg:.3f}",
                    f"{j.min_deg:+.1f}", f"{j.max_deg:+.1f}"]
            keys = [None, "channel", "center", "direction", "ticks_per_deg", "min_deg", "max_deg"]
            for c, (v, k) in enumerate(zip(vals, keys)):
                it = QTableWidgetItem(str(v))
                it.setFlags(Qt.ItemIsEnabled)
                if k and n in saved and abs(float(saved[n][k]) - float(getattr(j, k))) > 1e-6:
                    it.setForeground(QColor(theme.AMBER))
                    it.setBackground(QColor(245, 185, 74, 34))
                    it.setToolTip(f"saved: {saved[n][k]}")
                self.table.setItem(r, c, it)
        probs = self.cal.problems()
        self.problems.setText(
            f"<span style='color:{theme.RED}'>problems: {'; '.join(probs)}</span>" if probs
            else f"<span style='color:{theme.GREEN}'>No problems found.</span> "
                 f"<span style='color:{theme.TEXT_3}'>Amber = changed since the last save.</span>")
        self.boot.clear()
        self.boot.addItems(list(self.ctl.poses.poses))
        if "Sit" in self.ctl.poses.poses:
            self.boot.setCurrentText("Sit")
        self.show_ref(self.ctl.gait.stance_deg)

    def save(self):
        if self.cal.problems():
            self.ctl.log.emit("! not saved: fix the problems first")
            return
        self.ctl.save_calibration()
        self.wiz.mark_done(5)
        self.entered()

    def revert(self):
        self.ctl.reload_calibration()
        self.entered()

    def boot_send(self):
        name = self.boot.currentText()
        if not self.ctl.link.connected:
            self.ctl.log.emit("! connect to the robot first")
            return
        self.cal.boot_pose_deg = self.ctl.poses.get(name)
        self.ctl.push_boot_pose(self.cal.boot_pose_deg)


# ======================================================================== tab
STEPS = [("Channels", "which channel is which joint"),
         ("Zero pose", "femur flat, tibia vertical"),
         ("Direction", "does + move the right way"),
         ("Limits", "safe range per joint"),
         ("Scale", "real degrees per tick"),
         ("Review & save", "check, save, boot pose")]


class CalibrationTab(QWidget):
    def __init__(self, ctl, parent=None):
        super().__init__(parent)
        self.ctl = ctl
        self.ref = SimWorld()
        style(self.ref)
        self.ref.cam.distance = 0.5
        root = QHBoxLayout(self)
        root.setContentsMargins(18, 18, 18, 18)
        root.setSpacing(16)

        steps = Panel("steps")
        self.steps = StepList(STEPS)
        self.steps.picked.connect(self._goto)
        steps.body.addWidget(self.steps)
        steps.body.addStretch()
        tip = label("Robot on a stand with the legs free for steps 1-4. "
                    "Nothing is saved until step 6.", "faint")
        tip.setWordWrap(True)
        steps.body.addWidget(tip)
        steps.setFixedWidth(250)
        root.addWidget(steps)

        self.view = SimView(self.ref)
        self.view.badge = "REFERENCE · WHAT IT SHOULD DO"
        self.view.badge_color = theme.BLUE
        root.addWidget(self.view, 1)

        right = Panel()
        right.body.setContentsMargins(18, 16, 18, 16)
        self.stack = QStackedWidget()
        right.body.addWidget(self.stack)
        right.setFixedWidth(480)
        root.addWidget(right)
        self.pages = [ChannelsPage(self), ZeroPage(self), DirectionPage(self),
                      LimitsPage(self), ScalePage(self), ReviewPage(self)]
        for p in self.pages:
            self.stack.addWidget(p)
        self._goto(0)

    def mark_done(self, i):
        self.steps.done.add(i)
        self.steps.update()

    def _goto(self, i):
        self.steps.set_current(i)
        self.stack.setCurrentIndex(i)
        self.pages[i].entered()

    def showEvent(self, e):
        super().showEvent(e)
        self.pages[self.stack.currentIndex()].entered()

    def hideEvent(self, e):
        super().hideEvent(e)
        self.ctl.set_raw_mode(False)        # leaving the wizard: back to calibrated streaming
