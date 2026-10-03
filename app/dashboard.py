"""Drive page: 3D viewport + joystick, posture, servo load, telemetry."""
import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QGridLayout, QSlider, QScrollArea

from spidy.imu import tilt_from_gravity
from spidy.poses import PROFILES
from .widgets import Panel, SimView, LoadBars, Joystick, KeyValue, Segmented, button, label, caps
from . import theme

MODE_BADGE = {"sim": "SIM · PHYSICS", "real": "REAL · MIRROR", "both": "REAL + SIM SHADOW"}


class Dashboard(QWidget):
    def __init__(self, ctl, parent=None):
        super().__init__(parent)
        self.ctl = ctl
        self.g_level_candidate = None
        self.imu_tilt = None
        root = QHBoxLayout(self)
        root.setContentsMargins(18, 18, 18, 18)
        root.setSpacing(16)

        self.view = SimView(ctl.sim)
        self.view.overlay = "drag to orbit  ·  scroll to zoom"
        root.addWidget(self.view, 1)

        side = QVBoxLayout()
        side.setContentsMargins(0, 0, 4, 0)
        side.setSpacing(12)
        holder = QWidget()
        holder.setLayout(side)
        scroll = QScrollArea()
        scroll.setWidget(holder)
        scroll.setWidgetResizable(True)
        scroll.setFixedWidth(330)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        root.addWidget(scroll)

        # ---- drive
        drive = Panel("drive")
        self.stick = Joystick()
        self.stick.setFixedHeight(180)
        self.stick.moved.connect(ctl.walk)
        drive.body.addWidget(self.stick)
        row = QHBoxLayout()
        row.addWidget(label("Speed", "dim"))
        self.speed = QSlider(Qt.Horizontal)
        self.speed.setRange(20, 100)
        self.speed.setValue(int(ctl.speed * 100))
        self.speed.valueChanged.connect(self._speed)
        self.speed_lab = label(f"{int(ctl.speed * 100)}%", "mono")
        self.speed_lab.setFixedWidth(38)
        self.speed_lab.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        row.addWidget(self.speed, 1)
        row.addWidget(self.speed_lab)
        drive.body.addLayout(row)
        side.addWidget(drive)

        # ---- posture
        posture = Panel("posture")
        g = QGridLayout()
        g.setSpacing(8)
        g.addWidget(button("Stand", self._stand, ico="up", tip="smooth move to your Standf pose"), 0, 0)
        g.addWidget(button("Sit", self._sit, ico="down", tip="smooth move to Sit"), 0, 1)
        g.addWidget(button("Sit → stand", lambda: self._seq("Sit to Stand"), ico="play"), 1, 0)
        g.addWidget(button("Stand → sit", lambda: self._seq("Stand to Sit"), ico="play"), 1, 1)
        g.addWidget(button("Reset sim", ctl.reset_sim, name="ghost", ico="refresh",
                           tip="put the simulated robot back on its feet"), 2, 0)
        g.addWidget(button("Set IMU level", self._set_level, name="ghost", ico="gauge",
                           tip="robot on a flat floor: call this tilt 0°"), 2, 1)
        posture.body.addLayout(g)
        self.profile = Segmented([("aggressive", "Aggressive"), ("balanced", "Balanced"),
                                  ("conservative", "Conservative")])
        self.profile.set("balanced")
        for k, n in PROFILES.items():
            self.profile.btns[k].setToolTip(f"at most {n} poses in the sit/stand chain")
        posture.body.addWidget(self.profile)
        r = QHBoxLayout()
        r.addWidget(label("Step", "dim"))
        self.step = QSlider(Qt.Horizontal)
        self.step.setRange(300, 1500)
        self.step.setSingleStep(50)
        self.step.setValue(800)
        self.step_lab = label("800 ms", "mono")
        self.step_lab.setFixedWidth(58)
        self.step_lab.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.step.valueChanged.connect(lambda v: self.step_lab.setText(f"{v} ms"))
        r.addWidget(self.step, 1)
        r.addWidget(self.step_lab)
        posture.body.addLayout(r)
        side.addWidget(posture)

        # ---- load
        self.load_panel = Panel("servo load")
        self.bars = LoadBars()
        self.load_panel.body.addWidget(self.bars)
        self.load_note = label("", "faint")
        self.load_panel.body.addWidget(self.load_note)
        side.addWidget(self.load_panel)

        side.addStretch()

        ctl.ticked.connect(self._update)
        ctl.imu_sample.connect(self._imu)
        ctl.mode_changed.connect(self._mode)
        self._mode(ctl.mode)

    def _seq(self, base):
        """Prefer the IK 'lift' version of a sequence if it exists (feet planted under the hips:
        ~50% peak servo load in the sim vs ~100% for the old chain)."""
        name = f"{base} (lift)" if f"{base} (lift)" in self.ctl.poses.sequences else base
        self.ctl.play_sequence(name, PROFILES[self.profile.key], self.step.value())

    def _speed(self, v):
        self.ctl.speed = v / 100
        self.speed_lab.setText(f"{v}%")

    def _stand(self):
        self.ctl.move_to(self.ctl.gait.stance_deg, 1.5, "stand")

    def _sit(self):
        if "Sit" in self.ctl.poses.poses:
            self.ctl.move_to(self.ctl.poses.get("Sit"), 1.5, "sit")

    def _set_level(self):
        if self.g_level_candidate is not None:
            self.ctl.cal.imu["level"] = [float(v) for v in self.g_level_candidate]
            self.ctl.log.emit("IMU level reference set (save the calibration to keep it)")
        else:
            self.ctl.log.emit("! no IMU data yet (connect, target Real or Both)")

    def _imu(self, acc, gyro):
        self.g_level_candidate = acc
        lvl = self.ctl.cal.imu.get("level")
        self.imu_tilt = tilt_from_gravity(acc, lvl) if lvl else None

    def _mode(self, mode):
        self.view.badge = MODE_BADGE[mode]
        self.view.badge_color = theme.MODE_COLOR[mode]
        self.load_note.setText("predicted by the physics sim" if mode != "real"
                               else "switch to Both to see predicted load")
        self.ctl.imu_polling = mode in ("real", "both")

    def _update(self):
        if not self.isVisible():
            return
        c = self.ctl
        physics = c.mode in ("sim", "both")
        self.bars.set_values(c.sim.torque_pct() if physics else np.zeros(12))
        if c.motion is not None:
            act, col = c.motion_label.capitalize(), theme.BLUE
        elif c.gait.walking:
            act, col = "Walking", theme.ACCENT
        elif c.raw_mode:
            act, col = "Raw mode", theme.AMBER
        else:
            act, col = "Standing by", None
        self.view.pills = [(act.upper(), col)] if col else []
        if c.armed:
            self.view.pills.insert(0, ("ARMED", theme.RED))
        fell = physics and c.sim.fallen()
        self.view.banner = "Sim robot fell over  ·  Reset sim" if fell else ""
        f, t = c.gait.cmd_s
        spd = max(c.speed, 1e-3)
        if not self.stick.drag:
            self.stick.set_indicator(c.gait.cmd[0] / spd, c.gait.cmd[1] / spd)
        lat = c.pinger.last_ms
        self.view.readout = [
            ("forward", f"{f:+.2f}", None),
            ("turn", f"{t:+.2f}", None),
            ("sim tilt", f"{c.sim.tilt_deg():.1f}°" if physics else "—", None),
            ("imu tilt", f"{self.imu_tilt:.1f}°" if self.imu_tilt is not None
             else ("set level" if c.link.connected else "—"), None),
            ("latency", f"{lat:.0f} ms" if (c.link.connected and lat) else "—", None)]
