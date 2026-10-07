"""UI kit: icons, panels, segmented control, switch, joystick, load bars, step list,
3D viewport, console. Everything is painted from theme.py colours."""
import time

import numpy as np
from PySide6.QtCore import Qt, QTimer, Signal, QRectF, QPointF, QPropertyAnimation, Property, QSize
from PySide6.QtGui import (QImage, QPainter, QColor, QFont, QPen, QPixmap, QIcon, QPainterPath,
                           QLinearGradient, QRadialGradient)
from PySide6.QtWidgets import (QFrame, QLabel, QVBoxLayout, QHBoxLayout, QWidget, QGridLayout,
                               QPlainTextEdit, QLineEdit, QPushButton, QAbstractButton, QSizePolicy)

from spidy.kinematics import LEGS, JOINTS
from . import theme

# ---------------------------------------------------------------- icons (Segoe Fluent Icons)
I = dict(robot=0xE99A, sliders=0xE9E9, gauge=0xEC4A, layers=0xE81E, play=0xE768, stop=0xE71A,
         refresh=0xE72C, save=0xE74E, add=0xE710, delete=0xE74D, edit=0xE70F, check=0xE73E,
         close=0xE711, wifi=0xE701, usb=0xE88E, power=0xE7E8, warn=0xE7BA, info=0xE946,
         left=0xE76B, right=0xE76C, down=0xE70D, up=0xE70E, ccw=0xE777, cw=0xE72C, pulse=0xE9D9,
         chip=0xE950, console=0xE943, swap=0xE8AB, download=0xE896, undo=0xE777)


def icon_pixmap(name, size=16, color=theme.TEXT):
    dpr = 2
    pm = QPixmap(size * dpr, size * dpr)
    pm.fill(Qt.transparent)
    pm.setDevicePixelRatio(dpr)
    p = QPainter(pm)
    p.setRenderHint(QPainter.TextAntialiasing)
    f = QFont(theme.ICONS)
    f.setPixelSize(size)
    p.setFont(f)
    p.setPen(QColor(color))
    p.drawText(QRectF(0, 0, size, size), Qt.AlignCenter, chr(I[name]))
    p.end()
    return pm


def icon(name, size=16, color=theme.TEXT):
    ic = QIcon()
    ic.addPixmap(icon_pixmap(name, size, color), QIcon.Normal, QIcon.Off)
    ic.addPixmap(icon_pixmap(name, size, theme.TEXT_3), QIcon.Disabled, QIcon.Off)
    return ic


# ---------------------------------------------------------------- text + buttons
def label(text, name=None):
    lab = QLabel(text)
    if name:
        lab.setObjectName(name)
    return lab


def caps(text):
    lab = QLabel(text.upper())
    lab.setObjectName("caps")
    f = lab.font()
    f.setLetterSpacing(QFont.AbsoluteSpacing, 1.2)
    lab.setFont(f)
    return lab


def stamp(text):
    """Small tag."""
    lab = QLabel(text)
    lab.setObjectName("stamp")
    lab.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
    return lab


def button(text, slot=None, name=None, tip=None, checkable=False, ico=None):
    t = text[:1].upper() + text[1:] if text else ""
    if ico and t:
        t = "  " + t                       # Qt has no icon-text gap property
    b = QPushButton(t)
    if name and name != "pad":
        b.setObjectName(name)
    if tip:
        b.setToolTip(tip)
    b.setCheckable(checkable)
    b.setCursor(Qt.PointingHandCursor)
    if ico:
        col = theme.ACCENT_INK if name in ("accent", "primary") else (
              "#ffffff" if name == "estop" else theme.TEXT_2)
        b.setIcon(icon(ico, 14, col))
        b.setIconSize(QSize(14, 14))
    if slot:
        b.clicked.connect(slot)
    return b


def hline():
    f = QFrame()
    f.setObjectName("sep")
    return f


class Panel(QFrame):
    """Rounded panel with a small caps title and optional header widgets (right side)."""
    def __init__(self, title=None, parent=None, actions=()):
        super().__init__(parent)
        self.setObjectName("panel")
        self.body = QVBoxLayout(self)
        self.body.setContentsMargins(14, 12, 14, 14)
        self.body.setSpacing(10)
        if title or actions:
            head = QHBoxLayout()
            head.setSpacing(6)
            if title:
                head.addWidget(caps(title))
            head.addStretch()
            for a in actions:
                head.addWidget(a)
            self.body.addLayout(head)


Card = Panel


class Segmented(QFrame):
    """Pill group of mutually exclusive options. colors: optional {key: hex} for the checked state."""
    changed = Signal(str)

    def __init__(self, options, colors=None, parent=None):
        super().__init__(parent)
        self.setObjectName("seg")
        lay = QHBoxLayout(self)
        lay.setContentsMargins(3, 3, 3, 3)
        lay.setSpacing(2)
        self.colors = colors or {}
        self.btns = {}
        for key, text in options:
            b = QPushButton(text)
            b.setObjectName("segbtn")
            b.setCheckable(True)
            b.setCursor(Qt.PointingHandCursor)
            b.clicked.connect(lambda _=False, k=key: self._click(k))
            self.btns[key] = b
            lay.addWidget(b)
        self.key = None

    def _click(self, key):
        self.set(key)
        self.changed.emit(key)

    def set(self, key):
        self.key = key
        for k, b in self.btns.items():
            b.setChecked(k == key)
            col = self.colors.get(k)
            b.setStyleSheet(f"QPushButton#segbtn:checked {{ color: {col}; }}" if col else "")


class Switch(QAbstractButton):
    """Animated on/off switch."""
    def __init__(self, on_color=theme.ACCENT, parent=None):
        super().__init__(parent)
        self.setCheckable(True)
        self.setCursor(Qt.PointingHandCursor)
        self.on_color = on_color
        self.setFixedSize(42, 24)
        self._pos = 0.0
        self.anim = QPropertyAnimation(self, b"pos_", self)
        self.anim.setDuration(140)
        self.toggled.connect(self._animate)

    def _animate(self, on):
        self.anim.stop()
        self.anim.setStartValue(self._pos)
        self.anim.setEndValue(1.0 if on else 0.0)
        self.anim.start()

    def _get(self):
        return self._pos

    def _set(self, v):
        self._pos = v
        self.update()

    pos_ = Property(float, _get, _set)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        r = QRectF(1, 1, self.width() - 2, self.height() - 2)
        off, on = QColor(theme.SURFACE_3), QColor(self.on_color)
        c = QColor(int(off.red() + (on.red() - off.red()) * self._pos),
                   int(off.green() + (on.green() - off.green()) * self._pos),
                   int(off.blue() + (on.blue() - off.blue()) * self._pos))
        p.setPen(QPen(QColor(theme.BORDER_2), 1))
        p.setBrush(c)
        p.drawRoundedRect(r, r.height() / 2, r.height() / 2)
        d = r.height() - 6
        x = r.left() + 3 + (r.width() - d - 6) * self._pos
        p.setPen(Qt.NoPen)
        p.setBrush(QColor("#ffffff") if self.isEnabled() else QColor(theme.TEXT_3))
        p.drawEllipse(QRectF(x, r.top() + 3, d, d))
        p.end()

    def sizeHint(self):
        return QSize(42, 24)


class Joystick(QWidget):
    """Analog drive stick. Up = forward, left/right = turn, diagonals = arcs.
    moved(forward, turn) while dragging; (0, 0) on release.
    set_indicator() shows the command coming from keys / gamepad as a ghost knob."""
    moved = Signal(float, float)
    DEAD = 0.12

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self.knob = QPointF(0, 0)        # -1..1, y down
        self.ghost = QPointF(0, 0)
        self.drag = False
        self.setCursor(Qt.OpenHandCursor)
        self._spring = QTimer(self)
        self._spring.timeout.connect(self._spring_back)

    def _geom(self):
        s = min(self.width(), self.height())
        c = QPointF(self.width() / 2, self.height() / 2)
        return c, s * 0.42

    def _apply(self, pos):
        c, R = self._geom()
        v = (pos - c) / R
        n = (v.x() ** 2 + v.y() ** 2) ** 0.5
        if n > 1:
            v /= n
        self.knob = v
        fwd, turn = -v.y(), -v.x()
        fwd = 0.0 if abs(fwd) < self.DEAD else fwd
        turn = 0.0 if abs(turn) < self.DEAD else turn
        self.moved.emit(fwd, turn)
        self.update()

    def mousePressEvent(self, e):
        self.drag = True
        self._spring.stop()
        self.setCursor(Qt.ClosedHandCursor)
        self._apply(e.position())

    def mouseMoveEvent(self, e):
        if self.drag:
            self._apply(e.position())

    def mouseReleaseEvent(self, _):
        self.drag = False
        self.setCursor(Qt.OpenHandCursor)
        self.moved.emit(0.0, 0.0)
        self._spring.start(16)

    def _spring_back(self):
        self.knob *= 0.6
        if abs(self.knob.x()) + abs(self.knob.y()) < 0.01:
            self.knob = QPointF(0, 0)
            self._spring.stop()
        self.update()

    def set_indicator(self, fwd, turn):
        g = QPointF(-turn, -fwd)
        if (g - self.ghost).manhattanLength() > 0.01:
            self.ghost = g
            self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        c, R = self._geom()
        g = QRadialGradient(c, R)
        g.setColorAt(0, QColor(theme.SURFACE_3))
        g.setColorAt(1, QColor(theme.SURFACE_2))
        p.setBrush(g)
        p.setPen(QPen(QColor(theme.BORDER_2), 1))
        p.drawEllipse(c, R, R)
        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(QColor(theme.BORDER_2), 1, Qt.DashLine))
        p.drawEllipse(c, R * 0.55, R * 0.55)
        p.setPen(QPen(QColor(theme.BORDER), 1))
        p.drawLine(QPointF(c.x() - R, c.y()), QPointF(c.x() + R, c.y()))
        p.drawLine(QPointF(c.x(), c.y() - R), QPointF(c.x(), c.y() + R))
        f = QFont(theme.ICONS)
        f.setPixelSize(13)
        p.setFont(f)
        p.setPen(QColor(theme.TEXT_3))
        for name, dx, dy in (("up", 0, -1), ("down", 0, 1), ("ccw", -1, 0), ("cw", 1, 0)):
            p.drawText(QRectF(c.x() + dx * R * 0.8 - 10, c.y() + dy * R * 0.8 - 10, 20, 20),
                       Qt.AlignCenter, chr(I[name]))
        if not self.drag and (abs(self.ghost.x()) + abs(self.ghost.y())) > 0.02:
            gp = c + self.ghost * R
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(110, 231, 183, 70))
            p.drawEllipse(gp, R * 0.22, R * 0.22)
        k = c + self.knob * R
        active = self.drag or (abs(self.knob.x()) + abs(self.knob.y())) > 0.05
        if active:
            p.setPen(QPen(QColor(110, 231, 183, 90), 3))
            p.drawLine(c, k)
        p.setPen(QPen(QColor(0, 0, 0, 90), 6))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(k + QPointF(0, 2), R * 0.22, R * 0.22)
        p.setPen(QPen(QColor(theme.ACCENT if active else theme.BORDER_2), 1.5))
        p.setBrush(QColor(theme.TEXT if not active else "#f2fffa"))
        p.drawEllipse(k, R * 0.22, R * 0.22)
        p.end()


class LoadBars(QWidget):
    """12 thin servo-load bars, 4 legs x 3 joints, with a slowly fading peak tick."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.values = np.zeros(12)
        self.peaks = np.zeros(12)
        self.setMinimumHeight(118)

    def set_values(self, pct):
        self.values = np.asarray(pct, float)
        self.peaks = np.maximum(self.peaks * 0.995, self.values)
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        W, H = self.width(), self.height()
        x0 = 30
        colw = (W - x0) / 3
        f = QFont(theme.UI)
        f.setPixelSize(10)
        f.setWeight(QFont.DemiBold)
        f.setLetterSpacing(QFont.AbsoluteSpacing, 1)
        p.setFont(f)
        p.setPen(QColor(theme.TEXT_3))
        for j, name in enumerate(JOINTS):
            p.drawText(QRectF(x0 + j * colw, 0, colw - 10, 14), Qt.AlignLeft, name.upper())
        fm = QFont(theme.MONO)
        fm.setPixelSize(11)
        rowh = (H - 18) / 4
        for i, leg in enumerate(LEGS):
            y = 18 + i * rowh
            p.setFont(f)
            p.setPen(QColor(theme.TEXT_3))
            p.drawText(QRectF(0, y, x0, rowh), Qt.AlignVCenter | Qt.AlignLeft, leg.upper())
            for j in range(3):
                k = 3 * i + j
                x, w = x0 + j * colw, colw - 44
                cy = y + rowh / 2
                p.setPen(Qt.NoPen)
                p.setBrush(QColor(theme.SURFACE_3))
                p.drawRoundedRect(QRectF(x, cy - 3, w, 6), 3, 3)
                v = min(self.values[k], 100) / 100
                if v > 0.005:
                    p.setBrush(QColor(theme.load_color(self.values[k])))
                    p.drawRoundedRect(QRectF(x, cy - 3, max(w * v, 6), 6), 3, 3)
                pk = min(self.peaks[k], 100) / 100
                p.setBrush(QColor(theme.TEXT_2))
                p.drawRect(QRectF(x + w * pk - 1, cy - 5, 2, 10))
                p.setFont(fm)
                p.setPen(QColor(theme.TEXT_2 if self.values[k] < 60 else theme.load_color(self.values[k])))
                p.drawText(QRectF(x + w + 4, y, 38, rowh), Qt.AlignVCenter | Qt.AlignLeft,
                           f"{self.values[k]:.0f}%")
        p.end()


class StepList(QWidget):
    """Vertical numbered stepper. picked(index) on click."""
    picked = Signal(int)
    ROW = 58

    def __init__(self, steps, parent=None):
        super().__init__(parent)
        self.steps = steps                # list of (title, subtitle)
        self.current = 0
        self.done = set()
        self.setMinimumHeight(self.ROW * len(steps) + 8)
        self.setCursor(Qt.PointingHandCursor)

    def set_current(self, i):
        self.current = i
        self.update()

    def mousePressEvent(self, e):
        i = int(e.position().y() // self.ROW)
        if 0 <= i < len(self.steps):
            self.picked.emit(i)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        for i, (title, sub) in enumerate(self.steps):
            y = i * self.ROW
            cur, done = i == self.current, i in self.done
            if cur:
                p.setPen(Qt.NoPen)
                p.setBrush(QColor(theme.SURFACE_2))
                p.drawRoundedRect(QRectF(0, y + 3, self.width(), self.ROW - 6), 10, 10)
            cx, cy = 24, y + self.ROW / 2
            if i < len(self.steps) - 1:
                p.setPen(QPen(QColor(theme.BORDER_2), 1))
                p.drawLine(QPointF(cx, cy + 13), QPointF(cx, cy + self.ROW - 13))
            p.setPen(QPen(QColor(theme.ACCENT if (cur or done) else theme.BORDER_2), 1.5))
            p.setBrush(QColor(theme.ACCENT) if done else QColor(theme.SURFACE))
            p.drawEllipse(QPointF(cx, cy), 11, 11)
            if done:
                f = QFont(theme.ICONS)
                f.setPixelSize(11)
                p.setFont(f)
                p.setPen(QColor(theme.ACCENT_INK))
                p.drawText(QRectF(cx - 11, cy - 11, 22, 22), Qt.AlignCenter, chr(I["check"]))
            else:
                f = QFont(theme.MONO)
                f.setPixelSize(11)
                p.setFont(f)
                p.setPen(QColor(theme.ACCENT if cur else theme.TEXT_3))
                p.drawText(QRectF(cx - 11, cy - 11, 22, 22), Qt.AlignCenter, str(i + 1))
            f = QFont(theme.UI)
            f.setPixelSize(13)
            f.setWeight(QFont.DemiBold if cur else QFont.Normal)
            p.setFont(f)
            p.setPen(QColor(theme.TEXT if cur else theme.TEXT_2))
            p.drawText(QRectF(46, y + 10, self.width() - 50, 20), Qt.AlignLeft | Qt.AlignVCenter, title)
            f.setPixelSize(11)
            f.setWeight(QFont.Normal)
            p.setFont(f)
            p.setPen(QColor(theme.TEXT_3))
            p.drawText(QRectF(46, y + 29, self.width() - 50, 18), Qt.AlignLeft | Qt.AlignVCenter, sub)
        p.end()


class SimView(QWidget):
    """MuJoCo viewport: rounded, gradient backdrop, vignette, HUD pills.
    Left-drag orbits, wheel zooms. The backdrop is composited by Qt in Lighten mode:
    MuJoCo leaves empty pixels pure black, so max(gradient, frame) shows the gradient
    there and the frame everywhere else. Costs no Python time."""
    SCALE = 1.0                      # 1.0 = one MuJoCo pixel per screen pixel (60% looked blocky)

    def __init__(self, sim, fps=24, parent=None, distance=None):
        super().__init__(parent)
        self.sim = sim
        import mujoco
        self.cam = mujoco.MjvCamera()    # own camera: each view can orbit/zoom independently
        for k in ("type", "trackbodyid", "distance", "azimuth", "elevation"):
            setattr(self.cam, k, getattr(sim.cam, k))
        if distance:
            self.cam.distance = distance
        self.badge = "SIM"
        self.badge_color = theme.ACCENT
        self.pills = []                  # extra HUD pills: (text, color)
        self.banner = ""                 # centred warning
        self.overlay = ""
        self.readout = []                # bottom-right telemetry: (key, value, colour or None)
        self._img = None
        self._drag = None
        self.render_ms = 0.0
        self.setMinimumSize(320, 240)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(int(1000 / fps))

    def refresh(self):
        if not self.isVisible():
            return
        # Snap to 32 px buckets: a new size means a new MuJoCo renderer (~0.5 s), so small
        # resizes (window drag, console drawer) reuse the old one. Qt stretches the rest.
        w = max(int(self.width() * self.SCALE) // 32 * 32, 64)
        h = max(int(self.height() * self.SCALE) // 32 * 32, 64)
        t = time.perf_counter()
        rgb = np.ascontiguousarray(self.sim.render(w, h, self.cam))
        self.render_ms = (time.perf_counter() - t) * 1000
        self._img = QImage(rgb.data, rgb.shape[1], rgb.shape[0], 3 * rgb.shape[1], QImage.Format_RGB888).copy()
        self.update()

    def _pill(self, p, x, y, text, color):
        f = QFont(theme.UI)
        f.setPixelSize(11)
        f.setWeight(QFont.DemiBold)
        f.setLetterSpacing(QFont.AbsoluteSpacing, 1)
        p.setFont(f)
        w = p.fontMetrics().horizontalAdvance(text) + 30
        r = QRectF(x, y, w, 24)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(11, 13, 17, 190))
        p.drawRoundedRect(r, 12, 12)
        p.setBrush(QColor(color))
        p.drawEllipse(QPointF(x + 12, y + 12), 3.5, 3.5)
        p.setPen(QColor(theme.TEXT))
        p.drawText(r.adjusted(21, 0, 0, 0), Qt.AlignVCenter | Qt.AlignLeft, text)
        return w

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.SmoothPixmapTransform)
        rect = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)
        path = QPainterPath()
        path.addRoundedRect(rect, 12, 12)
        p.setClipPath(path)
        g = QLinearGradient(0, 0, 0, self.height())
        g.setColorAt(0, QColor("#161a23"))
        g.setColorAt(1, QColor("#08090c"))
        p.fillRect(self.rect(), g)
        if self._img is not None:
            p.setCompositionMode(QPainter.CompositionMode_Lighten)
            p.drawImage(self.rect(), self._img)
            p.setCompositionMode(QPainter.CompositionMode_SourceOver)
        v = QRadialGradient(QPointF(self.width() / 2, self.height() * 0.45), max(self.width(), self.height()) * 0.75)
        v.setColorAt(0.55, QColor(0, 0, 0, 0))
        v.setColorAt(1.0, QColor(0, 0, 0, 150))
        p.fillRect(self.rect(), v)
        x = 14
        x += self._pill(p, x, 14, self.badge, self.badge_color) + 8
        for text, color in self.pills:
            x += self._pill(p, x, 14, text, color) + 8
        if self.banner:
            f = QFont(theme.UI)
            f.setPixelSize(12)
            f.setWeight(QFont.DemiBold)
            p.setFont(f)
            w = p.fontMetrics().horizontalAdvance(self.banner) + 36
            r = QRectF((self.width() - w) / 2, 52, w, 30)
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(255, 92, 108, 225))
            p.drawRoundedRect(r, 15, 15)
            p.setPen(QColor("#ffffff"))
            p.drawText(r, Qt.AlignCenter, self.banner)
        if self.overlay:
            f = QFont(theme.UI)
            f.setPixelSize(11)
            p.setFont(f)
            p.setPen(QColor(theme.TEXT_3))
            p.drawText(QRectF(14, 0, self.width() - 28, self.height() - 12), Qt.AlignBottom | Qt.AlignLeft, self.overlay)
        if self.readout:
            fk, fv = QFont(theme.UI), QFont(theme.MONO)
            fk.setPixelSize(11)
            fv.setPixelSize(12)
            lh, wk, wv = 19, 74, 120
            h = lh * len(self.readout) + 16
            r = QRectF(self.width() - wk - wv - 30, self.height() - h - 14, wk + wv + 16, h)
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(11, 13, 17, 200))
            p.drawRoundedRect(r, 10, 10)
            for i, (k, val, col) in enumerate(self.readout):
                y = r.top() + 8 + i * lh
                p.setFont(fk)
                p.setPen(QColor(theme.TEXT_3))
                p.drawText(QRectF(r.left() + 10, y, wk, lh), Qt.AlignVCenter | Qt.AlignLeft, k)
                p.setFont(fv)
                p.setPen(QColor(col or theme.TEXT))
                p.drawText(QRectF(r.left() + 10 + wk, y, wv - 4, lh), Qt.AlignVCenter | Qt.AlignRight, val)
        p.setClipping(False)
        p.setPen(QPen(QColor(theme.BORDER), 1))
        p.setBrush(Qt.NoBrush)
        p.drawRoundedRect(rect, 12, 12)
        p.end()

    def mousePressEvent(self, e):
        self._drag = (e.position(), e.button())

    def mouseReleaseEvent(self, _):
        self._drag = None

    def mouseMoveEvent(self, e):
        if not self._drag:
            return
        d = e.position() - self._drag[0]
        cam = self.cam
        if self._drag[1] == Qt.LeftButton:
            cam.azimuth -= d.x() * 0.4
            cam.elevation = float(np.clip(cam.elevation - d.y() * 0.4, -89, -3))
        self._drag = (e.position(), self._drag[1])

    def wheelEvent(self, e):
        cam = self.cam
        cam.distance = float(np.clip(cam.distance * (0.9 if e.angleDelta().y() > 0 else 1.1), 0.25, 3.0))


class KeyValue(QWidget):
    """Two-column telemetry list: faint key, mono value."""
    def __init__(self, keys, parent=None):
        super().__init__(parent)
        g = QGridLayout(self)
        g.setContentsMargins(0, 0, 0, 0)
        g.setHorizontalSpacing(12)
        g.setVerticalSpacing(7)
        self.vals = {}
        for r, k in enumerate(keys):
            g.addWidget(label(k, "faint"), r, 0)
            v = label("—", "mono")
            v.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            g.addWidget(v, r, 1)
            self.vals[k] = v

    def set(self, key, text, color=None):
        v = self.vals[key]
        if v.text() != text:
            v.setText(text)
        if v.property("col") != color:
            v.setProperty("col", color)
            v.setStyleSheet(f"color: {color};" if color else "")


class Console(QWidget):
    """Log + a command line for raw protocol lines."""
    send_line = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(14, 10, 14, 12)
        lay.setSpacing(8)
        row = QHBoxLayout()
        row.addWidget(caps("console"))
        self.input = QLineEdit()
        self.input.setPlaceholderText("raw command  ·  M,3,400   STATE   IMU   HELLO   BOOT?")
        self.input.returnPressed.connect(self._send)
        row.addWidget(self.input, 1)
        row.addWidget(button("send", self._send, ico="right"))
        lay.addLayout(row)
        self.text = QPlainTextEdit()
        self.text.setObjectName("console")
        self.text.setReadOnly(True)
        self.text.setMaximumBlockCount(2000)
        lay.addWidget(self.text, 1)

    def _send(self):
        t = self.input.text().strip()
        if t:
            self.send_line.emit(t)
            self.input.clear()

    def append(self, line):
        self.text.appendPlainText(f"{time.strftime('%H:%M:%S')}  {line}")


def grid(widgets, cols):
    w = QWidget()
    g = QGridLayout(w)
    g.setContentsMargins(0, 0, 0, 0)
    for i, x in enumerate(widgets):
        g.addWidget(x, i // cols, i % cols)
    return w
