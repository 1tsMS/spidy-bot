"""Main window: nav rail | header (link, target, arm, e-stop) / pages / status strip + console drawer."""
import sys
import time

from PySide6.QtCore import Qt, QEvent, QObject, QSize, QTimer, QRectF
from PySide6.QtGui import QFont, QKeySequence, QShortcut, QIcon, QPainter, QColor
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QComboBox,
                               QLineEdit, QAbstractSpinBox, QFrame, QStackedWidget, QToolButton,
                               QButtonGroup)

from spidy.link import list_serial_ports
from .controller import RobotController
from .dashboard import Dashboard
from .gamepad import Gamepad
from .motors import MotorsTab
from .poses_tab import PosesTab
from .scene import style
from .widgets import Console, Segmented, Switch, button, label, icon_pixmap, stamp, I
from .wizard import CalibrationTab
from . import theme

KEYS = {Qt.Key_W: (1, 0), Qt.Key_S: (-1, 0), Qt.Key_A: (0, 1), Qt.Key_D: (0, -1)}
PAGES = [("robot", "Drive", "Drive", "Walk, pose and watch the robot"),
         ("sliders", "Motors", "Motors", "Per-joint control, calibrated or raw"),
         ("gauge", "Calibrate", "Calibration", "From a fresh build to a robot you can trust"),
         ("layers", "Poses", "Poses", "Library, preview and sequences")]


class WalkKeys(QObject):
    """WASD held down -> walk command. Ignored while typing in a text box."""
    def __init__(self, ctl):
        super().__init__()
        self.ctl = ctl
        self.held = set()

    def eventFilter(self, obj, e):
        if e.type() in (QEvent.KeyPress, QEvent.KeyRelease) and e.key() in KEYS and not e.isAutoRepeat():
            if isinstance(QApplication.focusWidget(), (QLineEdit, QAbstractSpinBox, QComboBox)):
                return False
            (self.held.add if e.type() == QEvent.KeyPress else self.held.discard)(e.key())
            f = sum(KEYS[k][0] for k in self.held)
            t = sum(KEYS[k][1] for k in self.held)
            self.ctl.walk(max(-1, min(1, f)), max(-1, min(1, t)))
            return True
        return False


class Logo(QWidget):
    """Small mint tile with the robot glyph."""
    def __init__(self):
        super().__init__()
        self.setFixedSize(40, 40)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(theme.ACCENT))
        p.drawRoundedRect(QRectF(2, 2, 36, 36), 10, 10)
        f = QFont(theme.ICONS)
        f.setPixelSize(20)
        p.setFont(f)
        p.setPen(QColor(theme.ACCENT_INK))
        p.drawText(QRectF(2, 2, 36, 36), Qt.AlignCenter, chr(I["robot"]))
        p.end()


def nav_button(ico, text):
    b = QToolButton()
    b.setObjectName("nav")
    b.setText(text)
    b.setCheckable(True)
    b.setCursor(Qt.PointingHandCursor)
    b.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
    ic = QIcon()
    ic.addPixmap(icon_pixmap(ico, 20, theme.TEXT_3), QIcon.Normal, QIcon.Off)
    ic.addPixmap(icon_pixmap(ico, 20, theme.ACCENT), QIcon.Normal, QIcon.On)
    ic.addPixmap(icon_pixmap(ico, 20, theme.TEXT_2), QIcon.Active, QIcon.Off)
    b.setIcon(ic)
    b.setIconSize(QSize(20, 20))
    b.setFixedSize(62, 58)
    return b


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Spidy")
        self.resize(1500, 920)
        self.setMinimumSize(1200, 760)
        self.ctl = c = RobotController()
        style(c.sim)

        root = QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)
        outer = QHBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        outer.addWidget(self._rail())

        main = QVBoxLayout()
        main.setContentsMargins(0, 0, 0, 0)
        main.setSpacing(0)
        outer.addLayout(main, 1)
        main.addWidget(self._header())

        self.stack = QStackedWidget()
        self.pages = [Dashboard(c), MotorsTab(c), CalibrationTab(c), PosesTab(c)]
        for pg in self.pages:
            self.stack.addWidget(pg)
        main.addWidget(self.stack, 1)

        self.drawer = QFrame()
        self.drawer.setObjectName("drawer")
        dl = QVBoxLayout(self.drawer)
        dl.setContentsMargins(0, 0, 0, 0)
        self.console = Console()
        dl.addWidget(self.console)
        self.drawer.setFixedHeight(230)
        self.drawer.hide()
        main.addWidget(self.drawer)
        main.addWidget(self._statusbar())

        c.log.connect(self._log)
        c.rx.connect(lambda l: self._log(f"< {l}"))
        self.console.send_line.connect(self._raw_send)
        c.link_changed.connect(self._link_changed)
        c.armed_changed.connect(self._armed_changed)
        c.mode_changed.connect(self._mode_changed)

        QShortcut(QKeySequence(Qt.Key_Space), self, activated=self._estop, context=Qt.ApplicationShortcut)
        QShortcut(QKeySequence("Ctrl+`"), self, activated=self._toggle_console, context=Qt.ApplicationShortcut)
        self.keys = WalkKeys(c)
        QApplication.instance().installEventFilter(self.keys)

        self.pad = Gamepad(self)
        self.pad.status.connect(self._log)
        self.pad.walk.connect(c.walk)
        self.pad.stand.connect(lambda: c.move_to(c.gait.stance_deg, 1.5, "stand"))
        self.pad.sit.connect(lambda: c.move_to(c.poses.get("Sit"), 1.5, "sit") if "Sit" in c.poses.poses else None)
        self.pad.estop.connect(self._estop)

        self._ticks, self._t0 = 0, time.monotonic()
        c.ticked.connect(self._count_tick)
        self.unread = 0
        self._goto(0)
        self._refresh_ports()
        self.mode.set("sim")

    # ------------------------------------------------------------- rail
    def _rail(self):
        rail = QFrame()
        rail.setObjectName("rail")
        rail.setFixedWidth(78)
        v = QVBoxLayout(rail)
        v.setContentsMargins(8, 14, 8, 14)
        v.setSpacing(6)
        v.addWidget(Logo(), 0, Qt.AlignHCenter)
        v.addSpacing(18)
        self.nav = QButtonGroup(self)
        self.nav.setExclusive(True)
        for i, (ico, short, _, _) in enumerate(PAGES):
            b = nav_button(ico, short)
            b.clicked.connect(lambda _=False, i=i: self._goto(i))
            self.nav.addButton(b, i)
            v.addWidget(b, 0, Qt.AlignHCenter)
        v.addStretch()
        self.btn_console = nav_button("console", "Console")
        self.btn_console.clicked.connect(self._toggle_console)
        v.addWidget(self.btn_console, 0, Qt.AlignHCenter)
        return rail

    def _goto(self, i):
        self.stack.setCurrentIndex(i)
        self.nav.button(i).setChecked(True)
        self.page_title.setText(PAGES[i][2])
        self.page_sub.setText(PAGES[i][3])

    def _toggle_console(self):
        show = not self.drawer.isVisible()
        self.drawer.setVisible(show)
        self.btn_console.setChecked(show)
        if show:
            self.unread = 0
            self.btn_console.setText("Console")

    # ------------------------------------------------------------- header
    def _header(self):
        bar = QFrame()
        bar.setObjectName("topbar")
        bar.setFixedHeight(64)
        h = QHBoxLayout(bar)
        h.setContentsMargins(22, 10, 18, 10)
        h.setSpacing(10)
        titles = QVBoxLayout()
        titles.setSpacing(0)
        self.page_title = label("", "h1")
        self.page_sub = label("", "faint")
        titles.addWidget(self.page_title)
        titles.addWidget(self.page_sub)
        h.addLayout(titles)
        h.addStretch()

        self.transport = Segmented([("usb", "USB"), ("wifi", "Wi-Fi")])
        self.transport.set("usb")
        self.transport.changed.connect(self._refresh_ports)
        self.port = QComboBox()
        self.port.setEditable(True)
        self.port.setMinimumWidth(150)
        self.btn_refresh = button("", self._refresh_ports, name="ghost", tip="rescan serial ports", ico="refresh")
        self.btn_conn = button("Connect", self._toggle_connect, ico="power")
        h.addWidget(self.transport)
        h.addWidget(self.port)
        h.addWidget(self.btn_refresh)
        h.addWidget(self.btn_conn)
        h.addSpacing(14)

        h.addWidget(label("Target", "faint"))
        self.mode = Segmented([("sim", "Sim"), ("real", "Real"), ("both", "Both")], colors=theme.MODE_COLOR)
        self.mode.changed.connect(self.ctl.set_mode)
        for k, tip in [("sim", "physics sim only, nothing is sent to the robot"),
                       ("real", "drive the real robot; the 3D view mirrors what is sent"),
                       ("both", "drive the real robot and run the sim alongside")]:
            self.mode.btns[k].setToolTip(tip)
        h.addWidget(self.mode)
        h.addSpacing(14)

        self.arm_lab = label("Arm", "faint")
        self.arm = Switch(on_color=theme.RED)
        self.arm.setToolTip("allow streaming to the real servos (needs Real or Both + a connection)")
        self.arm.clicked.connect(self._arm)
        h.addWidget(self.arm_lab)
        h.addWidget(self.arm)
        h.addSpacing(10)
        h.addWidget(button("E-stop", self._estop, name="estop", ico="power",
                           tip="all servo outputs off and disarm  ·  Space"))
        return bar

    # ------------------------------------------------------------- status strip
    def _statusbar(self):
        bar = QFrame()
        bar.setObjectName("statusbar")
        bar.setFixedHeight(30)
        h = QHBoxLayout(bar)
        h.setContentsMargins(16, 0, 16, 0)
        h.setSpacing(18)
        self.s_link = label("● offline", "mono")
        self.s_lat = label("", "mono")
        self.s_mode = label("", "mono")
        self.s_loop = label("", "mono")
        self.s_keys = label("W A S D  walk     Space  e-stop     Ctrl+`  console", "faint")
        for w in (self.s_link, self.s_lat, self.s_mode, self.s_loop):
            h.addWidget(w)
        h.addStretch()
        h.addWidget(self.s_keys)
        self.s_timer = QTimer(self)
        self.s_timer.timeout.connect(self._status_tick)
        self.s_timer.start(500)
        return bar

    def _count_tick(self):
        self._ticks += 1

    def _status_tick(self):
        now = time.monotonic()
        hz = self._ticks / max(now - self._t0, 1e-3)
        self._ticks, self._t0 = 0, now
        c = self.ctl
        self.s_loop.setText(f"loop {hz:4.0f} Hz")
        lat = c.pinger.last_ms
        self.s_lat.setText(f"{lat:.0f} ms" if (c.link.connected and lat) else "")
        armed = "  ·  ARMED" if c.armed else ""
        self.s_mode.setText(f"{c.mode}{armed}")
        self.s_mode.setStyleSheet(f"color: {theme.RED if c.armed else theme.MODE_COLOR[c.mode]};")

    # ------------------------------------------------------------- actions
    def _log(self, line):
        self.console.append(line)
        if not self.drawer.isVisible():
            self.unread += 1
            self.btn_console.setText(f"Console {min(self.unread, 99)}")

    def _refresh_ports(self, *_):
        self.port.clear()
        if self.transport.key == "usb":
            self.port.addItems(list_serial_ports() or [""])
        else:
            self.port.addItems(["spidy.local"])

    def _toggle_connect(self):
        c = self.ctl
        if c.link.connected:
            c.disconnect()
            return
        target = self.port.currentText().strip()
        if not target:
            self._log("! pick a port / host first")
            return
        (c.connect_serial if self.transport.key == "usb" else c.connect_tcp)(target)

    def _link_changed(self, ok, name):
        self.btn_conn.setText("Disconnect" if ok else "Connect")
        self.s_link.setText(f"● {name}" if ok else "● offline")
        self.s_link.setStyleSheet(f"color: {theme.ACCENT if ok else theme.TEXT_3};")
        for w in (self.transport, self.port, self.btn_refresh):
            w.setEnabled(not ok)

    def _mode_changed(self, m):
        self.mode.set(m)

    def _arm(self):
        on = self.arm.isChecked()
        if on and self.ctl.mode == "sim":
            self._log("! choose Real or Both before arming")
            self.arm.setChecked(False)
            return
        self.ctl.set_armed(on)

    def _armed_changed(self, on):
        self.arm.setChecked(on)
        self.arm_lab.setText("Armed" if on else "Arm")
        self.arm_lab.setStyleSheet(f"color: {theme.RED};" if on else "")

    def _estop(self):
        self.ctl.estop()

    def _raw_send(self, line):
        self._log(f"> {line}")
        if not self.ctl.link.send(line):
            self._log("! not connected")

    def closeEvent(self, e):
        self.ctl.shutdown()
        super().closeEvent(e)


def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(theme.QSS)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
