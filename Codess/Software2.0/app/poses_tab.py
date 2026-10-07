"""Poses page: library with rendered thumbnails, preview (not sent), angle editor, sequences."""
import numpy as np
from PySide6.QtCore import Qt, QSize
from PySide6.QtGui import QImage, QPixmap, QIcon
from PySide6.QtWidgets import (QWidget, QHBoxLayout, QVBoxLayout, QGridLayout, QListWidget, QListWidgetItem, QScrollArea,
                               QDoubleSpinBox, QInputDialog, QSpinBox, QComboBox, QAbstractSpinBox)

from spidy.kinematics import LEGS, JOINTS, JOINT_NAMES
from spidy.sim import SimWorld
from .scene import style
from .widgets import Panel, SimView, button, label, caps, stamp
from . import theme

THUMB = QSize(84, 56)


class Thumbs:
    """Renders a small picture of the robot in a pose (own SimWorld, cached)."""
    def __init__(self):
        self.sim = SimWorld()
        style(self.sim)
        self.sim.cam.distance = 0.36
        self.sim.cam.azimuth = 140
        self.sim.cam.elevation = -24
        self.cache = {}

    def get(self, q):
        key = tuple(np.round(q, 1))
        if key not in self.cache:
            self.sim.mirror(q)
            rgb = self.sim.render(THUMB.width() * 2, THUMB.height() * 2)
            h = rgb.shape[0]
            top, bot = np.array([0x1d, 0x22, 0x2c]), np.array([0x10, 0x12, 0x17])
            bg = (top + (bot - top) * np.linspace(0, 1, h)[:, None]).astype(np.uint8)[:, None, :]
            rgb = np.ascontiguousarray(np.maximum(rgb, bg))      # same trick as SimView: fill the black
            img = QImage(rgb.data, rgb.shape[1], rgb.shape[0], 3 * rgb.shape[1], QImage.Format_RGB888).copy()
            pm = QPixmap.fromImage(img)
            ic = QIcon()
            for mode in (QIcon.Normal, QIcon.Selected, QIcon.Active):
                ic.addPixmap(pm, mode)                           # no blue tint when selected
            self.cache[key] = ic
        return self.cache[key]


class PosesTab(QWidget):
    def __init__(self, ctl, parent=None):
        super().__init__(parent)
        self.ctl = ctl
        self.preview = SimWorld()
        style(self.preview)
        self.preview.cam.distance = 0.5
        self.thumbs = Thumbs()
        root = QHBoxLayout(self)
        root.setContentsMargins(18, 18, 18, 18)
        root.setSpacing(16)

        # ---- library
        lib = Panel("library")
        self.list = QListWidget()
        self.list.setIconSize(THUMB)
        self.list.setSpacing(2)
        self.list.currentTextChanged.connect(self._select)
        lib.body.addWidget(self.list, 1)
        row = QHBoxLayout()
        row.setSpacing(6)
        for ico, fn, tip in (("add", self._new, "new pose from what is commanded now"),
                             ("edit", self._rename, "rename"), ("delete", self._delete, "delete"),
                             ("save", ctl.save_poses, "save the library to config/poses.json")):
            row.addWidget(button("", fn, tip=tip, ico=ico))
        row.addStretch()
        lib.body.addLayout(row)
        lib.setFixedWidth(270)
        root.addWidget(lib)

        # ---- preview
        self.view = SimView(self.preview)
        self.view.badge = "PREVIEW · NOT SENT"
        self.view.badge_color = theme.AMBER
        root.addWidget(self.view, 1)

        # ---- angles + sequences
        right = QVBoxLayout()
        right.setContentsMargins(0, 0, 0, 0)
        right.setSpacing(14)
        self.name_lab = label("", "title")
        ang = Panel("angles · joint degrees")
        ang.body.addWidget(self.name_lab)
        grid = QGridLayout()
        grid.setHorizontalSpacing(14)
        grid.setVerticalSpacing(6)
        self.spins = {}
        pos = {"fl": (0, 0), "fr": (0, 1), "bl": (1, 0), "br": (1, 1)}
        for leg in LEGS:
            box = QGridLayout()
            box.setVerticalSpacing(4)
            box.addWidget(caps(leg), 0, 0, 1, 2)
            for r, j in enumerate(JOINTS, start=1):
                s = QDoubleSpinBox()
                s.setRange(-120, 120)
                s.setDecimals(1)
                s.setSuffix("°")
                s.setButtonSymbols(QAbstractSpinBox.NoButtons)
                s.setAlignment(Qt.AlignRight)
                s.valueChanged.connect(self._edited)
                self.spins[f"{leg}_{j}"] = s
                box.addWidget(label(j, "faint"), r, 0)
                box.addWidget(s, r, 1)
            w = QWidget()
            w.setLayout(box)
            grid.addWidget(w, *pos[leg])
        ang.body.addLayout(grid)
        send = QHBoxLayout()
        self.ms = QSpinBox()
        self.ms.setRange(200, 5000)
        self.ms.setSingleStep(100)
        self.ms.setValue(1200)
        self.ms.setSuffix(" ms")
        self.ms.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.ms.setFixedWidth(84)
        send.addWidget(self.ms)
        send.addWidget(button("Store edits", self._store, ico="save"))
        send.addStretch()
        send.addWidget(button("Send", self._send, name="accent", ico="play",
                              tip="smooth move from where the robot is now"))
        ang.body.addLayout(send)
        right.addWidget(ang)

        seq = Panel("sequences")
        self.seq_combo = QComboBox()
        self.seq_combo.currentTextChanged.connect(self._show_seq)
        seq.body.addWidget(self.seq_combo)
        self.seq_steps = label("", "faint")
        self.seq_steps.setWordWrap(True)
        self.seq_steps.setTextFormat(Qt.RichText)
        seq.body.addWidget(self.seq_steps)
        r2 = QHBoxLayout()
        r2.addWidget(button("Add pose", self._seq_add, ico="add", tip="append the selected pose"))
        r2.addWidget(button("Remove last", self._seq_pop, ico="undo"))
        r2.addStretch()
        r2.addWidget(button("Run", lambda: ctl.play_sequence(self.seq_combo.currentText()), name="accent", ico="play"))
        seq.body.addLayout(r2)
        right.addWidget(seq)
        right.addStretch()
        w = QWidget()
        w.setLayout(right)
        scroll = QScrollArea()                   # scrolls instead of squashing when the console is open
        scroll.setWidget(w)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFixedWidth(392)
        root.addWidget(scroll)
        self._fill()

    # ---- library
    def _fill(self, select=None):
        self.list.blockSignals(True)
        self.list.clear()
        for name, q in self.ctl.poses.poses.items():
            it = QListWidgetItem(self.thumbs.get(q), name)
            it.setSizeHint(QSize(0, THUMB.height() + 10))
            self.list.addItem(it)
        self.list.blockSignals(False)
        if self.list.count():
            items = self.list.findItems(select, Qt.MatchExactly) if select else []
            self.list.setCurrentItem(items[0] if items else self.list.item(0))
        self.seq_combo.blockSignals(True)
        cur = self.seq_combo.currentText()
        self.seq_combo.clear()
        self.seq_combo.addItems(list(self.ctl.poses.sequences))
        if cur:
            self.seq_combo.setCurrentText(cur)
        self.seq_combo.blockSignals(False)
        self._show_seq(self.seq_combo.currentText())

    def _select(self, name):
        if not name:
            return
        self.name_lab.setText(name)
        for n, v in zip(JOINT_NAMES, self.ctl.poses.get(name)):
            s = self.spins[n]
            s.blockSignals(True)
            s.setValue(v)
            s.blockSignals(False)
        self._edited()

    def _current_deg(self):
        return np.array([self.spins[n].value() for n in JOINT_NAMES])

    def _edited(self):
        self.preview.mirror(self.ctl.cal.clamp_deg(self._current_deg()))

    def _cur_name(self):
        return self.list.currentItem().text() if self.list.currentItem() else None

    # ---- actions
    def _new(self):
        name, ok = QInputDialog.getText(self, "New pose", "Name (captures what is commanded right now):")
        if ok and name.strip():
            self.ctl.poses.put(name.strip(), self.ctl.q)
            self._fill(name.strip())

    def _rename(self):
        old = self._cur_name()
        if not old:
            return
        new, ok = QInputDialog.getText(self, "Rename pose", "New name:", text=old)
        if ok and new.strip() and new.strip() != old:
            self.ctl.poses.rename(old, new.strip())
            self._fill(new.strip())

    def _delete(self):
        if self._cur_name():
            self.ctl.poses.delete(self._cur_name())
            self._fill()

    def _store(self):
        name = self._cur_name()
        if name:
            self.ctl.poses.put(name, self._current_deg())
            self.list.currentItem().setIcon(self.thumbs.get(self._current_deg()))
            self.ctl.log.emit("pose updated (save the library to keep it)")

    def _send(self):
        self.ctl.set_raw_mode(False)
        self.ctl.move_to(self._current_deg(), self.ms.value() / 1000, self._cur_name() or "pose")

    # ---- sequences
    def _show_seq(self, name):
        steps = self.ctl.poses.sequences.get(name, [])
        chip = (f"<span style='background:{theme.SURFACE_3}; color:{theme.TEXT};'>&nbsp;{{}}&nbsp;</span>"
                f"<span style='color:{theme.TEXT_3}'> {{}}ms</span>")
        arrow = f"<span style='color:{theme.TEXT_3}'>  →  </span>"
        self.seq_steps.setText(arrow.join(chip.format(p, ms) for p, ms in steps) or "empty")

    def _seq_add(self):
        name = self.seq_combo.currentText()
        if not name:
            name, ok = QInputDialog.getText(self, "New sequence", "Name:")
            if not (ok and name.strip()):
                return
            name = name.strip()
            self.ctl.poses.sequences[name] = []
        if self._cur_name():
            self.ctl.poses.sequences[name].append((self._cur_name(), self.ms.value()))
        self._fill(self._cur_name())
        self.seq_combo.setCurrentText(name)

    def _seq_pop(self):
        steps = self.ctl.poses.sequences.get(self.seq_combo.currentText())
        if steps:
            steps.pop()
            self._show_seq(self.seq_combo.currentText())
