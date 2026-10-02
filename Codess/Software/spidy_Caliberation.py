import sys
import time
import os
import json
import math
import socket
import ipaddress
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass

from PyQt5 import QtCore, QtGui, QtWidgets

try:
    import serial
    import serial.tools.list_ports
except Exception:
    serial = None


APP_NAME = "Spidy Bot Controller"
MOTOR_COUNT = 16
POSE_JOINT_COUNT = 12
GROUP_SIZE = 3
STEP_SMALL = 1
STEP_LARGE = 10
ANGLE_MIN = 0
ANGLE_MAX = 180
ANGLE_HOME = 90
SERVO_MIN = 100
SERVO_MAX = 700
DEFAULT_CALIBRATION_FILENAME = "spidy_calibration3.txt"
DEFAULT_POSES_FILENAME = "spidy_poses.txt"
DEFAULT_SPEED_PERCENT = 100
JOINT_NAMES = [
    "FL_HIP", "FL_KNEE", "FL_ANKLE",
    "FR_HIP", "FR_KNEE", "FR_ANKLE",
    "BL_HIP", "BL_KNEE", "BL_ANKLE",
    "BR_HIP", "BR_KNEE", "BR_ANKLE",
    "AUX_1", "AUX_2", "AUX_3", "AUX_4",
]


@dataclass
class MotorState:
    angle: int = ANGLE_HOME
    index: int = 0
    home_angle: int = ANGLE_HOME
    min_angle: int = ANGLE_MIN
    max_angle: int = ANGLE_MAX
    offset_deg: int = 0
    direction: int = 1
    joint: str = ""


def compute_corrected_angle(state: MotorState) -> float:
    angle_corrected = float(state.angle)
    if state.direction < 0:
        angle_corrected = 180.0 - angle_corrected
    offset = float(state.offset_deg)
    if state.direction < 0:
        offset = -offset
    angle_corrected += offset
    return max(float(state.min_angle), min(float(state.max_angle), angle_corrected))


class SerialManager(QtCore.QObject):
    connected = QtCore.pyqtSignal(str)
    disconnected = QtCore.pyqtSignal()
    error = QtCore.pyqtSignal(str)
    line_received = QtCore.pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._ser = None
        self._tcp = None
        self._tcp_rx_buffer = ""
        self._mode = None
        self._reader = QtCore.QTimer(self)
        self._reader.setInterval(20)
        self._reader.timeout.connect(self._read_loop)

    def is_connected(self):
        if self._mode == "serial":
            return self._ser is not None and self._ser.is_open
        if self._mode == "wifi":
            return self._tcp is not None
        return False

    def mode(self):
        return self._mode

    def available_ports(self):
        if serial is None:
            return []
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect_port(self, port, baud=115200):
        if self.is_connected():
            self.disconnect_port()
        if serial is None:
            self.error.emit("pyserial not installed.")
            return
        try:
            self._ser = serial.Serial(port=port, baudrate=baud, timeout=0.01)
            self._mode = "serial"
            time.sleep(0.2)
            self._reader.start()
            self.connected.emit(port)
        except Exception as exc:
            self._ser = None
            self._mode = None
            self.error.emit(str(exc))

    def connect_wifi(self, host, port, timeout_sec=2.0):
        if self.is_connected():
            self.disconnect_port()
        try:
            sock = socket.create_connection((host, int(port)), timeout=timeout_sec)
            sock.setblocking(False)
            self._tcp = sock
            self._tcp_rx_buffer = ""
            self._mode = "wifi"
            self._reader.start()
            self.connected.emit(f"{host}:{int(port)}")
        except Exception as exc:
            self._tcp = None
            self._mode = None
            self.error.emit(str(exc))

    def disconnect_port(self):
        self._reader.stop()
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        if self._tcp is not None:
            try:
                self._tcp.close()
            except Exception:
                pass
        self._ser = None
        self._tcp = None
        self._tcp_rx_buffer = ""
        self._mode = None
        self.disconnected.emit()

    def send_line(self, line):
        if not self.is_connected():
            return
        try:
            payload = (line.strip() + "\n").encode("utf-8")
            if self._mode == "serial" and self._ser is not None:
                self._ser.write(payload)
            elif self._mode == "wifi" and self._tcp is not None:
                self._tcp.sendall(payload)
        except Exception as exc:
            self.error.emit(str(exc))
            self.disconnect_port()

    def _read_loop(self):
        if not self.is_connected():
            return
        if self._mode == "serial":
            try:
                data = self._ser.read(512)
                if not data:
                    return
                text = data.decode("utf-8", errors="ignore")
                for line in text.splitlines():
                    if line.strip():
                        self.line_received.emit(line.strip())
            except Exception as exc:
                self.error.emit(str(exc))
                self.disconnect_port()
            return

        if self._mode == "wifi":
            if self._tcp is None:
                return
            try:
                data = self._tcp.recv(2048)
                if not data:
                    self.disconnect_port()
                    return
                self._tcp_rx_buffer += data.decode("utf-8", errors="ignore")
                while True:
                    pos_n = self._tcp_rx_buffer.find("\n")
                    pos_r = self._tcp_rx_buffer.find("\r")
                    cut = -1
                    if pos_n >= 0 and pos_r >= 0:
                        cut = min(pos_n, pos_r)
                    elif pos_n >= 0:
                        cut = pos_n
                    elif pos_r >= 0:
                        cut = pos_r
                    if cut < 0:
                        break
                    line = self._tcp_rx_buffer[:cut].strip()
                    self._tcp_rx_buffer = self._tcp_rx_buffer[cut + 1:]
                    if line:
                        self.line_received.emit(line)
            except BlockingIOError:
                return
            except Exception as exc:
                self.error.emit(str(exc))
                self.disconnect_port()


class WifiDiscoveryWorker(QtCore.QObject):
    found = QtCore.pyqtSignal(str)
    status = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal(list)
    error = QtCore.pyqtSignal(str)

    def __init__(self, port, timeout_sec=0.2, parent=None):
        super().__init__(parent)
        self.port = int(port)
        self.timeout_sec = float(timeout_sec)

    def _local_ip(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            pass
        try:
            ip = socket.gethostbyname(socket.gethostname())
            if ip and not ip.startswith("127."):
                return ip
        except Exception:
            pass
        return None

    def _probe_host(self, ip):
        sock = None
        try:
            sock = socket.create_connection((ip, self.port), timeout=self.timeout_sec)
            sock.settimeout(self.timeout_sec)
            try:
                data = sock.recv(128)
            except Exception:
                data = b""
            if b"WIFI_CLIENT_OK" in data or b"READY" in data:
                return ip
            return None
        except Exception:
            return None
        finally:
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass

    @QtCore.pyqtSlot()
    def run(self):
        local_ip = self._local_ip()
        if not local_ip:
            self.error.emit("Could not determine local IPv4 address for discovery.")
            self.finished.emit([])
            return

        try:
            net = ipaddress.ip_network(f"{local_ip}/24", strict=False)
        except Exception:
            self.error.emit("Could not build local subnet for discovery.")
            self.finished.emit([])
            return

        hosts = [str(h) for h in net.hosts()]
        self.status.emit(f"Scanning {len(hosts)} hosts on port {self.port}...")

        found = []
        with ThreadPoolExecutor(max_workers=48) as executor:
            futures = {executor.submit(self._probe_host, ip): ip for ip in hosts}
            for future in as_completed(futures):
                ip = future.result()
                if ip and ip not in found:
                    found.append(ip)
                    self.found.emit(ip)

        self.finished.emit(found)


class MotorRow(QtWidgets.QWidget):
    angle_changed = QtCore.pyqtSignal(int, int)
    index_changed = QtCore.pyqtSignal(int, int)
    state_changed = QtCore.pyqtSignal(int)

    def __init__(self, motor_id, state: MotorState, parent=None):
        super().__init__(parent)
        self.motor_id = motor_id
        self.state = state

        self.label = QtWidgets.QLabel(f"Motor {motor_id:02d}")
        self.label.setFixedWidth(90)

        self.angle_label = QtWidgets.QLabel()
        self.angle_label.setAlignment(QtCore.Qt.AlignCenter)
        self.angle_label.setFixedWidth(60)

        self.index_label = QtWidgets.QLabel("Idx")
        self.index_spin = QtWidgets.QSpinBox()
        self.index_spin.setRange(0, MOTOR_COUNT - 1)
        self.index_spin.setValue(self.state.index)
        self.index_spin.setFixedWidth(60)

        self.joint_label = QtWidgets.QLabel("Joint")
        self.joint_combo = QtWidgets.QComboBox()
        self.joint_combo.addItems(JOINT_NAMES)
        if self.state.joint in JOINT_NAMES:
            self.joint_combo.setCurrentText(self.state.joint)
        elif self.state.joint:
            self.joint_combo.addItem(self.state.joint)
            self.joint_combo.setCurrentText(self.state.joint)
        else:
            self.state.joint = self.joint_combo.currentText()

        self.dir_label = QtWidgets.QLabel("Dir")
        self.dir_combo = QtWidgets.QComboBox()
        self.dir_combo.addItems(["+", "-"])
        self.dir_combo.setFixedWidth(50)
        self.dir_combo.setCurrentText("+" if self.state.direction >= 0 else "-")

        self.min_label = QtWidgets.QLabel(f"Min {self.state.min_angle}")
        self.max_label = QtWidgets.QLabel(f"Max {self.state.max_angle}")
        self.btn_set_min = QtWidgets.QPushButton("Set Min")
        self.btn_set_max = QtWidgets.QPushButton("Set Max")

        self.btn_minus10 = QtWidgets.QPushButton("-10")
        self.btn_plus10 = QtWidgets.QPushButton("+10")
        self.btn_minus = QtWidgets.QPushButton("-")
        self.btn_plus = QtWidgets.QPushButton("+")
        self.btn_home = QtWidgets.QPushButton("Set 90")
        self.btn_update_home = QtWidgets.QPushButton("Update 90")

        for b in (
            self.btn_minus10,
            self.btn_plus10,
            self.btn_minus,
            self.btn_plus,
            self.btn_home,
            self.btn_update_home,
            self.btn_set_min,
            self.btn_set_max,
        ):
            b.setFixedWidth(60)

        self.btn_minus.setAutoRepeat(True)
        self.btn_minus.setAutoRepeatDelay(300)
        self.btn_minus.setAutoRepeatInterval(80)
        self.btn_plus.setAutoRepeat(True)
        self.btn_plus.setAutoRepeatDelay(300)
        self.btn_plus.setAutoRepeatInterval(80)

        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(6, 2, 6, 2)
        layout.addWidget(self.label)
        layout.addWidget(self.angle_label)
        layout.addWidget(self.index_label)
        layout.addWidget(self.index_spin)
        layout.addWidget(self.joint_label)
        layout.addWidget(self.joint_combo)
        layout.addWidget(self.dir_label)
        layout.addWidget(self.dir_combo)
        layout.addWidget(self.min_label)
        layout.addWidget(self.btn_set_min)
        layout.addWidget(self.max_label)
        layout.addWidget(self.btn_set_max)
        layout.addWidget(self.btn_minus10)
        layout.addWidget(self.btn_plus10)
        layout.addWidget(self.btn_minus)
        layout.addWidget(self.btn_plus)
        layout.addWidget(self.btn_home)
        layout.addWidget(self.btn_update_home)
        layout.addStretch()

        self.btn_minus10.clicked.connect(lambda: self.set_angle(self.state.angle - STEP_LARGE))
        self.btn_plus10.clicked.connect(lambda: self.set_angle(self.state.angle + STEP_LARGE))
        self.btn_minus.clicked.connect(lambda: self.set_angle(self.state.angle - STEP_SMALL))
        self.btn_plus.clicked.connect(lambda: self.set_angle(self.state.angle + STEP_SMALL))
        self.btn_home.clicked.connect(lambda: self.set_angle(self.state.home_angle))
        self.btn_update_home.clicked.connect(self.update_home)
        self.btn_set_min.clicked.connect(self.update_min)
        self.btn_set_max.clicked.connect(self.update_max)
        self.index_spin.valueChanged.connect(self.set_index)
        self.joint_combo.currentTextChanged.connect(self.set_joint)
        self.dir_combo.currentTextChanged.connect(self.set_direction)
        self._update_angle_label()

    def _update_angle_label(self):
        physical_angle = int(round(compute_corrected_angle(self.state)))
        self.angle_label.setText(f"{physical_angle:03d}°")

    def set_angle(self, angle):
        angle = max(self.state.min_angle, min(self.state.max_angle, angle))
        if angle == self.state.angle:
            return
        self.state.angle = angle
        self._update_angle_label()
        self.angle_changed.emit(self.motor_id, self.state.angle)
        self.state_changed.emit(self.motor_id)

    def sync_angle(self, angle):
        angle = max(self.state.min_angle, min(self.state.max_angle, angle))
        if angle == self.state.angle:
            return
        self.state.angle = angle
        self._update_angle_label()
        self.state_changed.emit(self.motor_id)

    def set_index(self, index):
        index = max(0, min(MOTOR_COUNT - 1, index))
        if index == self.state.index:
            return
        self.state.index = index
        self.index_changed.emit(self.motor_id, self.state.index)
        self.state_changed.emit(self.motor_id)

    def update_home(self):
        self.state.offset_deg = self.state.angle - ANGLE_HOME
        self._update_angle_label()
        self.state_changed.emit(self.motor_id)

    def update_min(self):
        if self.state.angle > self.state.max_angle:
            self.state.max_angle = self.state.angle
            self.max_label.setText(f"Max {self.state.max_angle}")
        self.state.min_angle = self.state.angle
        self.min_label.setText(f"Min {self.state.min_angle}")
        self._update_angle_label()
        self.state_changed.emit(self.motor_id)

    def update_max(self):
        if self.state.angle < self.state.min_angle:
            self.state.min_angle = self.state.angle
            self.min_label.setText(f"Min {self.state.min_angle}")
        self.state.max_angle = self.state.angle
        self.max_label.setText(f"Max {self.state.max_angle}")
        self._update_angle_label()
        self.state_changed.emit(self.motor_id)

    def set_joint(self, name):
        if name == self.state.joint:
            return
        self.state.joint = name
        self.state_changed.emit(self.motor_id)

    def set_direction(self, text):
        self.state.direction = 1 if text == "+" else -1
        self._update_angle_label()
        self.state_changed.emit(self.motor_id)

    def apply_state(self):
        self.index_spin.blockSignals(True)
        self.joint_combo.blockSignals(True)
        self.dir_combo.blockSignals(True)
        self.index_spin.setValue(self.state.index)
        if self.state.joint and self.state.joint in JOINT_NAMES:
            self.joint_combo.setCurrentText(self.state.joint)
        elif self.state.joint:
            if self.joint_combo.findText(self.state.joint) < 0:
                self.joint_combo.addItem(self.state.joint)
            self.joint_combo.setCurrentText(self.state.joint)
        self.dir_combo.setCurrentText("+" if self.state.direction >= 0 else "-")
        self.min_label.setText(f"Min {self.state.min_angle}")
        self.max_label.setText(f"Max {self.state.max_angle}")
        self._update_angle_label()
        self.index_spin.blockSignals(False)
        self.joint_combo.blockSignals(False)
        self.dir_combo.blockSignals(False)


class ControllerTab(QtWidgets.QWidget):
    send_command = QtCore.pyqtSignal(str)
    state_updated = QtCore.pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.states = [MotorState(index=i) for i in range(MOTOR_COUNT)]
        self.rows = []

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        for group_index in range(0, MOTOR_COUNT, GROUP_SIZE):
            box = QtWidgets.QGroupBox(f"Leg Group {group_index // GROUP_SIZE + 1}")
            v = QtWidgets.QVBoxLayout(box)
            for i in range(group_index, group_index + GROUP_SIZE):
                if i >= MOTOR_COUNT:
                    continue
                row = MotorRow(i, self.states[i])
                row.angle_changed.connect(self._handle_angle)
                row.index_changed.connect(self._handle_index)
                row.state_changed.connect(self.state_updated.emit)
                v.addWidget(row)
                self.rows.append(row)
            root.addWidget(box)

        root.addStretch()

    def _handle_angle(self, motor_id, angle):
        state = self.states[motor_id]
        pulse = self.compute_pulse(state)
        self.send_command.emit(f"M,{motor_id},{pulse}")

    def _handle_index(self, motor_id, index):
        self.send_command.emit(f"IDX,{motor_id},{index}")

    def set_all_90(self):
        for motor_id, row in enumerate(self.rows):
            row.sync_angle(ANGLE_HOME)
        self.send_pose()

    def set_all_cal_90(self):
        for motor_id, row in enumerate(self.rows):
            row.sync_angle(ANGLE_HOME)
        self.send_pose()

    def set_motor_cal_90(self, motor_id):
        if motor_id < 0 or motor_id >= len(self.rows):
            return
        self.rows[motor_id].sync_angle(ANGLE_HOME)
        state = self.states[motor_id]
        pulse = self.compute_pulse(state)
        self.send_command.emit(f"M,{motor_id},{pulse}")

    def compute_pulse(self, state: MotorState):
        angle_corrected = compute_corrected_angle(state)
        pulse = SERVO_MIN + (angle_corrected / 180.0) * (SERVO_MAX - SERVO_MIN)
        pulse = int(round(pulse))
        return max(SERVO_MIN, min(SERVO_MAX, pulse))

    def compute_pulse_for_angle(self, motor_id, angle):
        if motor_id < 0 or motor_id >= len(self.states):
            return SERVO_MIN
        state = self.states[motor_id]
        original_angle = state.angle
        state.angle = max(ANGLE_MIN, min(ANGLE_MAX, int(round(angle))))
        pulse = self.compute_pulse(state)
        state.angle = original_angle
        return pulse

    def _compose_pose_command(self, angles, duration_ms=None):
        values = []
        count = min(POSE_JOINT_COUNT, len(angles))
        for motor_id in range(count):
            angle = max(ANGLE_MIN, min(ANGLE_MAX, int(round(angles[motor_id]))))
            values.append(str(angle))
        if len(values) < POSE_JOINT_COUNT:
            for motor_id in range(len(values), POSE_JOINT_COUNT):
                angle = max(ANGLE_MIN, min(ANGLE_MAX, int(round(self.states[motor_id].angle))))
                values.append(str(angle))
        command = "POSE," + ",".join(values)
        if duration_ms is not None:
            command += f",{max(50, min(10000, int(duration_ms)))}"
        return command

    def send_pose(self, angles=None, duration_ms=None):
        if angles is None:
            angles = [self.states[i].angle for i in range(POSE_JOINT_COUNT)]
        command = self._compose_pose_command(angles, duration_ms=duration_ms)
        self.send_command.emit(command)

    def apply_pose_angles(self, angles, duration_ms=None):
        count = min(MOTOR_COUNT, len(angles))
        for motor_id in range(count):
            angle = max(ANGLE_MIN, min(ANGLE_MAX, int(round(angles[motor_id]))))
            self.rows[motor_id].sync_angle(angle)
        self.send_pose(angles, duration_ms=duration_ms)


class CalibrationTab(QtWidgets.QWidget):
    def __init__(self, controller: ControllerTab, parent=None):
        super().__init__(parent)
        self.controller = controller

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        self.table = QtWidgets.QTableWidget(MOTOR_COUNT, 8)
        self.table.setHorizontalHeaderLabels([
            "Motor",
            "Index",
            "Joint",
            "Dir",
            "Offset",
            "Signal",
            "Min",
            "Max",
        ])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.horizontalHeader().setStretchLastSection(True)

        self.btn_save = QtWidgets.QPushButton("Save Analysis")
        self.btn_save.clicked.connect(self._save)
        self.btn_load = QtWidgets.QPushButton("Load Analysis")
        self.btn_load.clicked.connect(self._load)
        self.btn_clear = QtWidgets.QPushButton("Clear Analysis")
        self.btn_clear.clicked.connect(self._clear_analysis)

        root.addWidget(self.table)
        root.addWidget(self.btn_save)
        root.addWidget(self.btn_load)
        root.addWidget(self.btn_clear)
        self.refresh_all()

    def refresh_all(self):
        for motor_id in range(MOTOR_COUNT):
            self.refresh_row(motor_id)

    def refresh_row(self, motor_id):
        state = self.controller.states[motor_id]
        pulse = self.controller.compute_pulse(state)
        values = [
            f"{motor_id:02d}",
            str(state.index),
            state.joint,
            "+" if state.direction >= 0 else "-",
            f"{state.offset_deg:+d}",
            str(pulse),
            str(state.min_angle),
            str(state.max_angle),
        ]
        for col, value in enumerate(values):
            item = self.table.item(motor_id, col)
            if item is None:
                item = QtWidgets.QTableWidgetItem(value)
                self.table.setItem(motor_id, col, item)
            else:
                item.setText(value)

    def _save(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save Analysis",
            "spidy_calibration.txt",
            "Text Files (*.txt);;All Files (*)",
        )
        if not path:
            return
        lines = [
            "Motor\tIndex\tJoint\tDir\tOffset\tSignal\tMin\tMax",
        ]
        for motor_id in range(MOTOR_COUNT):
            state = self.controller.states[motor_id]
            pulse = self.controller.compute_pulse(state)
            lines.append(
                f"{motor_id:02d}\t{state.index}\t{state.joint}\t"
                f"{('+' if state.direction >= 0 else '-') }\t"
                f"{state.offset_deg:+d}\t{pulse}\t{state.min_angle}\t{state.max_angle}"
            )
        with open(path, "w", encoding="utf-8") as handle:
            handle.write("\n".join(lines))

    def _load(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Load Analysis",
            "",
            "Text Files (*.txt);;All Files (*)",
        )
        if not path:
            return
        self.load_analysis_file(path)

    def load_analysis_file(self, path):
        try:
            with open(path, "r", encoding="utf-8") as handle:
                lines = [line.strip() for line in handle if line.strip()]
        except Exception:
            return False
        if not lines:
            return False
        header = lines[0].split("\t")
        expected = ["Motor", "Index", "Joint", "Dir", "Offset", "Signal", "Min", "Max"]
        if header[:8] != expected:
            return False

        loaded_any = False
        for row in lines[1:]:
            parts = row.split("\t")
            if len(parts) < 8:
                continue
            try:
                motor_id = int(parts[0])
                index = int(parts[1])
                offset_deg = int(parts[4])
                min_angle = int(parts[6])
                max_angle = int(parts[7])
            except ValueError:
                continue
            if motor_id < 0 or motor_id >= MOTOR_COUNT:
                continue
            state = self.controller.states[motor_id]
            state.index = index
            state.joint = parts[2]
            state.direction = 1 if parts[3].strip() == "+" else -1
            state.offset_deg = offset_deg
            state.min_angle = min_angle
            state.max_angle = max_angle
            state.angle = ANGLE_HOME
            self.controller.rows[motor_id].apply_state()
            self.controller.send_command.emit(f"IDX,{motor_id},{state.index}")
            self.controller.state_updated.emit(motor_id)
            self.refresh_row(motor_id)
            loaded_any = True
        return loaded_any

    def _clear_analysis(self):
        for motor_id in range(MOTOR_COUNT):
            state = self.controller.states[motor_id]
            state.offset_deg = 0
            state.min_angle = ANGLE_MIN
            state.max_angle = ANGLE_MAX
            state.direction = 1
            state.index = motor_id
            state.angle = ANGLE_HOME
            row = self.controller.rows[motor_id]
            row.apply_state()
            self.refresh_row(motor_id)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(APP_NAME)
        self.resize(1100, 720)
        self.speed_percent = DEFAULT_SPEED_PERCENT
        self._discover_thread = None
        self._discover_worker = None
        self._discover_results = []

        self.serial = SerialManager(self)
        self.serial.connected.connect(self._on_connected)
        self.serial.disconnected.connect(self._on_disconnected)
        self.serial.error.connect(self._log_error)
        self.serial.line_received.connect(self._log_line)

        self._build_ui()
        self._refresh_ports()

    def _build_ui(self):
        self.toolbar = QtWidgets.QToolBar()
        self.toolbar.setMovable(False)
        self.addToolBar(QtCore.Qt.TopToolBarArea, self.toolbar)

        self.btn_connect = QtWidgets.QAction("Connect", self)
        self.btn_disconnect = QtWidgets.QAction("Disconnect", self)
        self.btn_estop = QtWidgets.QAction("Emergency Stop", self)
        self.btn_set_all_90 = QtWidgets.QAction("Set All 90", self)
        self.btn_set_all_cal_90 = QtWidgets.QAction("Set All Cal 90", self)

        self.toolbar.addAction(self.btn_connect)
        self.toolbar.addAction(self.btn_disconnect)
        self.toolbar.addSeparator()
        self.toolbar.addAction(self.btn_estop)
        self.toolbar.addAction(self.btn_set_all_90)
        self.toolbar.addAction(self.btn_set_all_cal_90)
        self.toolbar.addSeparator()

        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setMinimumWidth(160)
        self.toolbar.addWidget(QtWidgets.QLabel("COM: "))
        self.toolbar.addWidget(self.port_combo)

        self.refresh_btn = QtWidgets.QAction("Refresh", self)
        self.toolbar.addAction(self.refresh_btn)
        self.toolbar.addSeparator()

        self.transport_title = QtWidgets.QLabel("Mode:")
        self.transport_combo = QtWidgets.QComboBox()
        self.transport_combo.addItems(["Serial", "WiFi"])
        self.transport_combo.setFixedWidth(90)
        self.toolbar.addWidget(self.transport_title)
        self.toolbar.addWidget(self.transport_combo)

        self.ip_title = QtWidgets.QLabel("IP:")
        self.ip_edit = QtWidgets.QLineEdit("192.168.4.1")
        self.ip_edit.setFixedWidth(130)
        self.wifi_port_title = QtWidgets.QLabel("Port:")
        self.wifi_port_spin = QtWidgets.QSpinBox()
        self.wifi_port_spin.setRange(1, 65535)
        self.wifi_port_spin.setValue(5000)
        self.wifi_port_spin.setFixedWidth(80)
        self.discover_btn = QtWidgets.QPushButton("Discover")
        self.discover_btn.setFixedWidth(90)
        self.toolbar.addWidget(self.ip_title)
        self.toolbar.addWidget(self.ip_edit)
        self.toolbar.addWidget(self.wifi_port_title)
        self.toolbar.addWidget(self.wifi_port_spin)
        self.toolbar.addWidget(self.discover_btn)

        self.speed_title = QtWidgets.QLabel("Speed %:")
        self.speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speed_slider.setRange(1, 100)
        self.speed_slider.setValue(self.speed_percent)
        self.speed_slider.setFixedWidth(140)
        self.speed_spin = QtWidgets.QSpinBox()
        self.speed_spin.setRange(1, 100)
        self.speed_spin.setValue(self.speed_percent)
        self.speed_spin.setFixedWidth(60)
        self.btn_set_speed = QtWidgets.QPushButton("Set Speed")

        self.toolbar.addWidget(self.speed_title)
        self.toolbar.addWidget(self.speed_slider)
        self.toolbar.addWidget(self.speed_spin)
        self.toolbar.addWidget(self.btn_set_speed)

        self.btn_connect.triggered.connect(self._connect)
        self.btn_disconnect.triggered.connect(self._disconnect)
        self.btn_estop.triggered.connect(self._estop)
        self.btn_set_all_90.triggered.connect(self._set_all_90)
        self.btn_set_all_cal_90.triggered.connect(self._set_all_cal_90)
        self.refresh_btn.triggered.connect(self._refresh_ports)
        self.transport_combo.currentTextChanged.connect(self._on_transport_changed)
        self.discover_btn.clicked.connect(self._start_wifi_discovery)
        self.speed_slider.valueChanged.connect(self._on_speed_slider_changed)
        self.speed_spin.valueChanged.connect(self._on_speed_spin_changed)
        self.btn_set_speed.clicked.connect(self._set_speed)

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setTabPosition(QtWidgets.QTabWidget.West)
        self.setCentralWidget(self.tabs)

        self.controller_tab = ControllerTab()
        self.controller_tab.send_command.connect(self._send_line)

        icon = self.style().standardIcon(QtWidgets.QStyle.SP_ComputerIcon)
        self.tabs.addTab(self.controller_tab, icon, "Controller")

        self.calib_tab = CalibrationTab(self.controller_tab)
        self.tabs.addTab(self.calib_tab, icon, "Calibration")

        self.poses_tab = PosesTab(self.controller_tab)
        self.tabs.addTab(self.poses_tab, icon, "Poses")

        self.walk_tab = WalkTab(self.controller_tab, self.poses_tab)
        self.tabs.addTab(self.walk_tab, icon, "Walking")

        self.terminal = QtWidgets.QPlainTextEdit()
        self.terminal.setReadOnly(True)
        self.terminal.setMaximumBlockCount(500)
        self.terminal.setPlaceholderText("Serial log...")

        splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        splitter.addWidget(self.tabs)
        splitter.addWidget(self.terminal)
        splitter.setSizes([520, 160])
        self.setCentralWidget(splitter)

        self.status = self.statusBar()
        self._apply_theme()
        self._update_actions()

        self.controller_tab.state_updated.connect(self.calib_tab.refresh_row)
        self.controller_tab.state_updated.connect(self.poses_tab.refresh_row_all_poses)

        self.poses_tab.load_or_create_default_file()
        self._log(f"Default poses file: {self.poses_tab.poses_file_path}")
        self._load_default_calibration()
        self._on_transport_changed(self.transport_combo.currentText())

    def _apply_theme(self):
        self.setStyleSheet("""
        QMainWindow { background: #0f141a; }
        QToolBar { background: #141b22; spacing: 6px; }
        QToolButton { color: #e5e7eb; background: #1f2937; border-radius: 6px; padding: 6px 10px; }
        QToolButton:hover { background: #263244; }
        QLabel { color: #d1d5db; }
        QGroupBox { border: 1px solid #1f2a37; border-radius: 10px; margin-top: 8px; color: #9ca3af; }
        QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
        QPushButton { background: #1f2937; color: #e5e7eb; border-radius: 6px; padding: 6px 10px; }
        QPushButton:hover { background: #263244; }
        QPlainTextEdit { background: #0b1117; color: #cbd5f5; border: 1px solid #1f2a37; border-radius: 8px; }
        QComboBox { background: #0b1117; color: #e5e7eb; border: 1px solid #1f2a37; border-radius: 6px; padding: 4px 8px; }
        QLineEdit { background: #0b1117; color: #e5e7eb; border: 1px solid #1f2a37; border-radius: 6px; padding: 4px 8px; }
        QSpinBox { background: #0b1117; color: #e5e7eb; border: 1px solid #1f2a37; border-radius: 6px; padding: 2px 6px; }
        QTabWidget::pane { border: none; }
        QTabBar::tab { background: #141b22; color: #cbd5f5; min-height: 60px; min-width: 80px; padding: 8px; }
        QTabBar::tab:selected { background: #1f2937; }
        QTableWidget { background: #0b1117; color: #e5e7eb; gridline-color: #1f2a37; border: 1px solid #1f2a37; }
        QHeaderView::section { background: #141b22; color: #cbd5f5; border: 1px solid #1f2a37; padding: 4px; }
        QTableCornerButton::section { background: #141b22; border: 1px solid #1f2a37; }
        QToolBox::tab { background: #141b22; color: #cbd5f5; border: 1px solid #1f2a37; border-radius: 6px; padding: 6px 8px; }
        QToolBox::tab:selected { background: #1f2937; color: #e5e7eb; }
        """)

    def _log(self, text):
        timestamp = time.strftime("%H:%M:%S")
        self.terminal.appendPlainText(f"[{timestamp}] {text}")

    def _log_line(self, text):
        self._log(f"< {text}")

    def _log_error(self, text):
        self._log(f"! {text}")

    def _on_connected(self, port):
        self._log(f"Connected to {port}")
        self.status.showMessage(f"Connected: {port}")
        self._update_actions()

    def _on_disconnected(self):
        self._log("Disconnected")
        self.status.showMessage("Disconnected")
        self._update_actions()

    def _refresh_ports(self):
        if self.transport_combo.currentText() != "Serial":
            return
        self.port_combo.clear()
        ports = self.serial.available_ports()
        self.port_combo.addItems(ports if ports else [""])

    def _connect(self):
        mode = self.transport_combo.currentText()
        if mode == "Serial":
            port = self.port_combo.currentText().strip()
            if not port:
                self._log_error("No COM port selected.")
                return
            self.serial.connect_port(port)
            return

        host = self.ip_edit.text().strip()
        port = int(self.wifi_port_spin.value())
        if not host:
            self._log_error("No IP address entered.")
            return
        self.serial.connect_wifi(host, port)

    def _disconnect(self):
        self.serial.disconnect_port()

    def _on_transport_changed(self, mode):
        serial_mode = (mode == "Serial")
        self.port_combo.setEnabled(serial_mode)
        self.refresh_btn.setEnabled(serial_mode)
        self.ip_edit.setEnabled(not serial_mode)
        self.wifi_port_spin.setEnabled(not serial_mode)
        self.discover_btn.setEnabled(not serial_mode)

    def _start_wifi_discovery(self):
        if self._discover_thread is not None:
            self._log("Discovery already running...")
            return

        port = int(self.wifi_port_spin.value())
        self._discover_results = []
        self.discover_btn.setEnabled(False)
        self.discover_btn.setText("Scanning...")
        self._log(f"Starting WiFi discovery on port {port}...")

        self._discover_thread = QtCore.QThread(self)
        self._discover_worker = WifiDiscoveryWorker(port=port, timeout_sec=0.2)
        self._discover_worker.moveToThread(self._discover_thread)

        self._discover_thread.started.connect(self._discover_worker.run)
        self._discover_worker.status.connect(lambda msg: self._log(msg))
        self._discover_worker.error.connect(self._log_error)
        self._discover_worker.found.connect(self._on_discovery_found)
        self._discover_worker.finished.connect(self._on_discovery_finished)
        self._discover_worker.finished.connect(self._discover_thread.quit)
        self._discover_thread.finished.connect(self._cleanup_discovery)

        self._discover_thread.start()

    def _on_discovery_found(self, ip):
        if ip not in self._discover_results:
            self._discover_results.append(ip)
            self._log(f"Discovered ESP candidate: {ip}")
            if not self.ip_edit.text().strip() or self.ip_edit.text().strip() == "192.168.4.1":
                self.ip_edit.setText(ip)

    def _on_discovery_finished(self, found_list):
        if found_list:
            self.ip_edit.setText(found_list[0])
            self._log(f"Discovery done: {len(found_list)} device(s) found. Using {found_list[0]}")
        else:
            self._log("Discovery done: no compatible devices found.")

    def _cleanup_discovery(self):
        if self._discover_worker is not None:
            self._discover_worker.deleteLater()
        if self._discover_thread is not None:
            self._discover_thread.deleteLater()
        self._discover_worker = None
        self._discover_thread = None
        self.discover_btn.setText("Discover")
        self._update_actions()

    def _estop(self):
        self._send_line("ESTOP")
        self._log("Emergency stop sent.")

    def _set_all_90(self):
        self.controller_tab.set_all_90()
        self._log("All motors set to 90 degree pulse.")

    def _set_all_cal_90(self):
        self.controller_tab.set_all_cal_90()
        self._log("All motors set to calibrated 90 degree pulse.")

    def _on_speed_slider_changed(self, value):
        self.speed_percent = int(value)
        self.speed_spin.blockSignals(True)
        self.speed_spin.setValue(self.speed_percent)
        self.speed_spin.blockSignals(False)

    def _on_speed_spin_changed(self, value):
        self.speed_percent = int(value)
        self.speed_slider.blockSignals(True)
        self.speed_slider.setValue(self.speed_percent)
        self.speed_slider.blockSignals(False)

    def _set_speed(self):
        self._send_line(f"SPD,{self.speed_percent}")
        self._log(f"Speed set to {self.speed_percent}%")

    def _send_line(self, line):
        self.serial.send_line(line)
        self._log(f"> {line}")

    def _update_actions(self):
        connected = self.serial.is_connected()
        serial_mode = (self.transport_combo.currentText() == "Serial")
        self.btn_connect.setEnabled(not connected)
        self.btn_disconnect.setEnabled(connected)
        self.btn_estop.setEnabled(True)
        self.transport_combo.setEnabled(not connected)
        self.port_combo.setEnabled((not connected) and serial_mode)
        self.refresh_btn.setEnabled((not connected) and serial_mode)
        self.ip_edit.setEnabled((not connected) and (not serial_mode))
        self.wifi_port_spin.setEnabled((not connected) and (not serial_mode))
        self.discover_btn.setEnabled((not connected) and (not serial_mode) and (self._discover_thread is None))

    def _load_default_calibration(self):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        default_path = os.path.join(base_dir, DEFAULT_CALIBRATION_FILENAME)
        if self.calib_tab.load_analysis_file(default_path):
            self._log(f"Default calibration loaded: {DEFAULT_CALIBRATION_FILENAME}")
            self.poses_tab.refresh_all()
        else:
            self._log_error(f"Default calibration not loaded: {DEFAULT_CALIBRATION_FILENAME}")


class PosePanel(QtWidgets.QWidget):
    name_changed = QtCore.pyqtSignal(str)

    def __init__(self, pose_name, pose_angles, controller: ControllerTab, parent=None):
        super().__init__(parent)
        self.pose_name = pose_name
        self.pose_angles = pose_angles
        self.controller = controller

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(8)

        name_row = QtWidgets.QHBoxLayout()
        self.name_label = QtWidgets.QLabel("Pose Name")
        self.name_edit = QtWidgets.QLineEdit(self.pose_name)
        name_row.addWidget(self.name_label)
        name_row.addWidget(self.name_edit)

        self.table = QtWidgets.QTableWidget(MOTOR_COUNT, 4)
        self.table.setHorizontalHeaderLabels(["Motor", "Joint", "Angle", "Signal"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.horizontalHeader().setStretchLastSection(True)

        btn_row = QtWidgets.QHBoxLayout()
        self.btn_capture = QtWidgets.QPushButton("Capture Current")
        self.btn_apply = QtWidgets.QPushButton("Apply Pose")
        btn_row.addWidget(self.btn_capture)
        btn_row.addWidget(self.btn_apply)
        btn_row.addStretch()

        root.addLayout(name_row)
        root.addWidget(self.table)
        root.addLayout(btn_row)

        self.btn_capture.clicked.connect(self.capture_current)
        self.btn_apply.clicked.connect(self.apply_pose)
        self.name_edit.editingFinished.connect(self._emit_name_changed)

        self._build_rows()
        self.refresh_all_rows()

    def _build_rows(self):
        for motor_id in range(MOTOR_COUNT):
            motor_item = QtWidgets.QTableWidgetItem(f"{motor_id:02d}")
            joint_item = QtWidgets.QTableWidgetItem("")
            signal_item = QtWidgets.QTableWidgetItem("")
            self.table.setItem(motor_id, 0, motor_item)
            self.table.setItem(motor_id, 1, joint_item)
            self.table.setItem(motor_id, 3, signal_item)

            spin = QtWidgets.QSpinBox()
            spin.setRange(ANGLE_MIN, ANGLE_MAX)
            spin.setValue(self.pose_angles[motor_id])
            spin.valueChanged.connect(lambda value, m=motor_id: self._on_angle_changed(m, value))
            self.table.setCellWidget(motor_id, 2, spin)

    def _on_angle_changed(self, motor_id, angle):
        self.pose_angles[motor_id] = angle
        self.refresh_row(motor_id)

    def refresh_row(self, motor_id):
        state = self.controller.states[motor_id]
        joint_item = self.table.item(motor_id, 1)
        signal_item = self.table.item(motor_id, 3)
        if joint_item is not None:
            joint_item.setText(state.joint)
        pulse = self.controller.compute_pulse_for_angle(motor_id, self.pose_angles[motor_id])
        if signal_item is not None:
            signal_item.setText(str(pulse))

    def refresh_all_rows(self):
        for motor_id in range(MOTOR_COUNT):
            self.refresh_row(motor_id)

    def refresh_spin_values(self):
        for motor_id in range(MOTOR_COUNT):
            spin = self.table.cellWidget(motor_id, 2)
            if spin is None:
                continue
            spin.blockSignals(True)
            spin.setValue(self.pose_angles[motor_id])
            spin.blockSignals(False)

    def capture_current(self):
        for motor_id in range(MOTOR_COUNT):
            self.pose_angles[motor_id] = int(self.controller.states[motor_id].angle)
        self.refresh_spin_values()
        self.refresh_all_rows()

    def apply_pose(self):
        self.controller.apply_pose_angles(self.pose_angles)

    def _emit_name_changed(self):
        new_name = self.name_edit.text().strip()
        if not new_name:
            self.name_edit.setText(self.pose_name)
            return
        self.pose_name = new_name
        self.name_changed.emit(new_name)


class PosesTab(QtWidgets.QWidget):
    def __init__(self, controller: ControllerTab, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.pose_names = ["Pose 1", "Pose 2", "Pose 3", "Pose 4"]
        self.pose_angles = [[ANGLE_HOME for _ in range(MOTOR_COUNT)] for _ in self.pose_names]
        self.pose_panels = []
        self.poses_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), DEFAULT_POSES_FILENAME)
        self._transition_active = False
        self._transition_indices = []
        self._transition_cursor = 0
        self._transition_target_steps = 4
        self._transition_profile_steps = {
            "Aggressive": 3,
            "Balanced": 4,
            "Conservative": 6,
        }

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        self.pose_data_box = QtWidgets.QGroupBox("Pose Data")
        pose_data_layout = QtWidgets.QVBoxLayout(self.pose_data_box)
        pose_data_layout.setContentsMargins(10, 10, 10, 10)
        pose_data_layout.setSpacing(10)

        top_row = QtWidgets.QHBoxLayout()
        self.file_label = QtWidgets.QLabel(f"File: {self.poses_file_path}")
        self.btn_add_pose = QtWidgets.QPushButton("Add Pose")
        self.btn_delete_pose = QtWidgets.QPushButton("Delete Current Pose")
        self.btn_save_poses = QtWidgets.QPushButton("Save Poses")
        self.btn_reload_poses = QtWidgets.QPushButton("Reload Poses")
        top_row.addWidget(self.file_label)
        top_row.addStretch()
        top_row.addWidget(self.btn_add_pose)
        top_row.addWidget(self.btn_delete_pose)
        top_row.addWidget(self.btn_save_poses)
        top_row.addWidget(self.btn_reload_poses)

        self.toolbox = QtWidgets.QToolBox()
        pose_data_layout.addLayout(top_row)
        pose_data_layout.addWidget(self.toolbox)

        self.transitions_box = QtWidgets.QGroupBox("Transitions")
        transitions_layout = QtWidgets.QHBoxLayout(self.transitions_box)
        transitions_layout.setContentsMargins(10, 10, 10, 10)
        transitions_layout.setSpacing(10)
        self.transition_duration_label = QtWidgets.QLabel("Step Duration (ms)")
        self.transition_duration_spin = QtWidgets.QSpinBox()
        self.transition_duration_spin.setRange(100, 5000)
        self.transition_duration_spin.setValue(800)
        self.transition_mode_label = QtWidgets.QLabel("Optimization")
        self.transition_mode_combo = QtWidgets.QComboBox()
        self.transition_mode_combo.addItems(["Aggressive", "Balanced", "Conservative"])
        self.transition_mode_combo.setCurrentText("Balanced")
        self.btn_sit_to_stand = QtWidgets.QPushButton("Sit → Stand")
        self.btn_stand_to_sit = QtWidgets.QPushButton("Stand → Sit")
        self.transition_status = QtWidgets.QLabel("Idle")
        transitions_layout.addWidget(self.transition_duration_label)
        transitions_layout.addWidget(self.transition_duration_spin)
        transitions_layout.addWidget(self.transition_mode_label)
        transitions_layout.addWidget(self.transition_mode_combo)
        transitions_layout.addWidget(self.btn_sit_to_stand)
        transitions_layout.addWidget(self.btn_stand_to_sit)
        transitions_layout.addStretch()
        transitions_layout.addWidget(self.transition_status)

        root.addWidget(self.pose_data_box)
        root.addWidget(self.transitions_box)

        for pose_id, pose_name in enumerate(self.pose_names):
            panel = self._create_panel(pose_name, self.pose_angles[pose_id])
            self.pose_panels.append(panel)
            self.toolbox.addItem(panel, pose_name)

        self.btn_add_pose.clicked.connect(self._add_pose)
        self.btn_delete_pose.clicked.connect(self._delete_current_pose)
        self.btn_save_poses.clicked.connect(self._save_poses)
        self.btn_reload_poses.clicked.connect(self._reload_poses)
        self.btn_sit_to_stand.clicked.connect(self._run_sit_to_stand_transition)
        self.btn_stand_to_sit.clicked.connect(self._run_stand_to_sit_transition)
        self.transition_mode_combo.currentTextChanged.connect(self._on_transition_mode_changed)
        self._on_transition_mode_changed(self.transition_mode_combo.currentText())

    def _on_transition_mode_changed(self, mode):
        self._transition_target_steps = self._transition_profile_steps.get(mode, 4)

    def refresh_row_all_poses(self, motor_id):
        for panel in self.pose_panels:
            panel.refresh_row(motor_id)

    def refresh_all(self):
        for panel in self.pose_panels:
            panel.refresh_all_rows()

    def _create_panel(self, pose_name, angles):
        panel = PosePanel(pose_name, angles, self.controller)
        panel.name_changed.connect(lambda new_name, p=panel: self._rename_panel(p, new_name))
        return panel

    def _rename_panel(self, panel, new_name):
        index = self.toolbox.indexOf(panel)
        if index < 0:
            return
        self.pose_names[index] = new_name
        self.toolbox.setItemText(index, new_name)

    def _add_pose(self):
        pose_name = f"Pose {len(self.pose_names) + 1}"
        angles = [ANGLE_HOME for _ in range(MOTOR_COUNT)]
        panel = self._create_panel(pose_name, angles)
        self.pose_names.append(pose_name)
        self.pose_angles.append(angles)
        self.pose_panels.append(panel)
        self.toolbox.addItem(panel, pose_name)
        self.toolbox.setCurrentIndex(self.toolbox.count() - 1)

    def _delete_current_pose(self):
        current_index = self.toolbox.currentIndex()
        if current_index < 0:
            return
        if len(self.pose_panels) <= 1:
            self.pose_names[0] = "Pose 1"
            self.pose_angles[0] = [ANGLE_HOME for _ in range(MOTOR_COUNT)]
            panel = self.pose_panels[0]
            panel.pose_name = self.pose_names[0]
            panel.pose_angles = self.pose_angles[0]
            panel.name_edit.setText(self.pose_names[0])
            panel.refresh_spin_values()
            panel.refresh_all_rows()
            self.toolbox.setItemText(0, self.pose_names[0])
            self._save_poses()
            return
        panel = self.pose_panels.pop(current_index)
        self.pose_names.pop(current_index)
        self.pose_angles.pop(current_index)
        self.toolbox.removeItem(current_index)
        panel.deleteLater()
        self._save_poses()

    def _save_poses(self):
        payload = {
            "poses": [
                {
                    "name": self.pose_names[i],
                    "angles": [int(a) for a in self.pose_angles[i]],
                }
                for i in range(len(self.pose_names))
            ]
        }
        try:
            with open(self.poses_file_path, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2)
        except Exception:
            return False
        return True

    def _reload_poses(self):
        self._transition_active = False
        self.transition_status.setText("Idle")
        self._load_poses_from_file(self.poses_file_path)

    def _clear_pose_widgets(self):
        while self.toolbox.count() > 0:
            widget = self.toolbox.widget(0)
            self.toolbox.removeItem(0)
            if widget is not None:
                widget.deleteLater()
        self.pose_panels = []

    def _find_pose_index(self, pose_name):
        target = pose_name.strip().lower()
        for i, name in enumerate(self.pose_names):
            if name.strip().lower() == target:
                return i
        return -1

    def _resolve_sit_to_stand_indices(self):
        wanted = ["Pose 4", "Pose 5", "Pose 6", "Pose 7", "Pose 8"]
        indices = []
        for name in wanted:
            idx = self._find_pose_index(name)
            if idx < 0:
                indices = []
                break
            indices.append(idx)
        if indices:
            sit_index = self._find_pose_index("Sit")
            if sit_index >= 0 and (not indices or indices[0] != sit_index):
                indices.insert(0, sit_index)
            stand_index = self._find_pose_index("Stand")
            if stand_index >= 0 and (not indices or indices[-1] != stand_index):
                indices.append(stand_index)
            return indices
        if len(self.pose_angles) >= 5:
            start = len(self.pose_angles) - 5
            indices = list(range(start, len(self.pose_angles)))
            sit_index = self._find_pose_index("Sit")
            if sit_index >= 0 and (not indices or indices[0] != sit_index):
                indices.insert(0, sit_index)
            stand_index = self._find_pose_index("Stand")
            if stand_index >= 0 and (not indices or indices[-1] != stand_index):
                indices.append(stand_index)
            return indices
        return []

    def _pose_vector(self, pose_index):
        pose = self.pose_angles[pose_index]
        count = min(POSE_JOINT_COUNT, len(pose))
        return [float(pose[i]) for i in range(count)]

    def _distance(self, a, b):
        count = min(len(a), len(b))
        if count <= 0:
            return 0.0
        total = 0.0
        for i in range(count):
            d = a[i] - b[i]
            total += d * d
        return math.sqrt(total)

    def _distance_point_to_segment(self, p, a, b):
        count = min(len(p), len(a), len(b))
        if count <= 0:
            return 0.0

        ab_len_sq = 0.0
        ap_dot_ab = 0.0
        for i in range(count):
            ab = b[i] - a[i]
            ap = p[i] - a[i]
            ab_len_sq += ab * ab
            ap_dot_ab += ap * ab

        if ab_len_sq <= 1e-9:
            return self._distance(p, a)

        t = ap_dot_ab / ab_len_sq
        if t < 0.0:
            t = 0.0
        elif t > 1.0:
            t = 1.0

        proj = [a[i] + (b[i] - a[i]) * t for i in range(count)]
        return self._distance(p[:count], proj)

    def _rdp_reduce_positions(self, vectors, epsilon):
        count = len(vectors)
        if count <= 2:
            return list(range(count))

        keep = [False] * count
        keep[0] = True
        keep[-1] = True
        stack = [(0, count - 1)]

        while stack:
            start, end = stack.pop()
            if end <= start + 1:
                continue

            a = vectors[start]
            b = vectors[end]
            max_dist = -1.0
            max_pos = -1
            for i in range(start + 1, end):
                dist = self._distance_point_to_segment(vectors[i], a, b)
                if dist > max_dist:
                    max_dist = dist
                    max_pos = i

            if max_dist > epsilon and max_pos > 0:
                keep[max_pos] = True
                stack.append((start, max_pos))
                stack.append((max_pos, end))

        return [i for i, flag in enumerate(keep) if flag]

    def _optimize_transition_indices(self, base_indices):
        if len(base_indices) <= self._transition_target_steps:
            return base_indices[:]

        vectors = [self._pose_vector(i) for i in base_indices]
        eps_candidates = [2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 16.0]
        best_positions = list(range(len(base_indices)))

        for eps in eps_candidates:
            reduced = self._rdp_reduce_positions(vectors, eps)
            if len(reduced) <= self._transition_target_steps and len(reduced) >= 3:
                best_positions = reduced
                break
            if len(reduced) < len(best_positions):
                best_positions = reduced

        optimized = [base_indices[pos] for pos in best_positions]

        if len(optimized) < 3 and len(base_indices) >= 3:
            mid = base_indices[len(base_indices) // 2]
            optimized = [base_indices[0], mid, base_indices[-1]]

        dedup = []
        seen_last = None
        for idx in optimized:
            if seen_last is None or idx != seen_last:
                dedup.append(idx)
            seen_last = idx

        return dedup

    def _run_sit_to_stand_transition(self):
        if self._transition_active:
            return
        base_indices = self._resolve_sit_to_stand_indices()
        if not base_indices:
            self.transition_status.setText("Missing Sit/Stand transition poses")
            return
        indices = self._optimize_transition_indices(base_indices)
        self._transition_active = True
        self._transition_indices = indices
        self._transition_cursor = 0
        self.btn_sit_to_stand.setEnabled(False)
        self.btn_stand_to_sit.setEnabled(False)
        self.transition_status.setText(f"Running Sit → Stand ({len(indices)} steps)")
        self._run_next_transition_step()

    def _run_stand_to_sit_transition(self):
        if self._transition_active:
            return
        base_indices = self._resolve_sit_to_stand_indices()
        if not base_indices:
            self.transition_status.setText("Missing Sit/Stand transition poses")
            return
        indices = list(reversed(self._optimize_transition_indices(base_indices)))
        self._transition_active = True
        self._transition_indices = indices
        self._transition_cursor = 0
        self.btn_sit_to_stand.setEnabled(False)
        self.btn_stand_to_sit.setEnabled(False)
        self.transition_status.setText(f"Running Stand → Sit ({len(indices)} steps)")
        self._run_next_transition_step()

    def _run_next_transition_step(self):
        if not self._transition_active:
            return
        if self._transition_cursor >= len(self._transition_indices):
            self._finish_transition()
            return

        pose_index = self._transition_indices[self._transition_cursor]
        self._transition_cursor += 1
        self.toolbox.setCurrentIndex(pose_index)

        duration_ms = int(self.transition_duration_spin.value())
        self.controller.apply_pose_angles(self.pose_angles[pose_index], duration_ms=duration_ms)
        self.transition_status.setText(f"Step {self._transition_cursor}/{len(self._transition_indices)}: {self.pose_names[pose_index]}")

        wait_ms = duration_ms + 80
        QtCore.QTimer.singleShot(wait_ms, self._run_next_transition_step)

    def _finish_transition(self):
        self._transition_active = False
        self.btn_sit_to_stand.setEnabled(True)
        self.btn_stand_to_sit.setEnabled(True)
        self.transition_status.setText("Done")

    def _load_poses_from_file(self, path):
        try:
            with open(path, "r", encoding="utf-8") as handle:
                data = json.load(handle)
        except Exception:
            return False
        poses = data.get("poses", []) if isinstance(data, dict) else []
        if not poses:
            return False

        names = []
        angles_list = []
        for pose in poses:
            name = str(pose.get("name", "Pose")).strip()
            if not name:
                name = "Pose"
            raw_angles = pose.get("angles", [])
            if not isinstance(raw_angles, list):
                continue
            angles = []
            for i in range(MOTOR_COUNT):
                value = raw_angles[i] if i < len(raw_angles) else ANGLE_HOME
                try:
                    angle = int(value)
                except Exception:
                    angle = ANGLE_HOME
                angles.append(max(ANGLE_MIN, min(ANGLE_MAX, angle)))
            names.append(name)
            angles_list.append(angles)

        if not names:
            return False

        self._clear_pose_widgets()
        self.pose_names = names
        self.pose_angles = angles_list
        for i, name in enumerate(self.pose_names):
            panel = self._create_panel(name, self.pose_angles[i])
            self.pose_panels.append(panel)
            self.toolbox.addItem(panel, name)
        self.refresh_all()
        return True

    def load_or_create_default_file(self):
        if self._load_poses_from_file(self.poses_file_path):
            return True
        self._save_poses()
        return True


class WalkTab(QtWidgets.QWidget):
    def __init__(self, controller: ControllerTab, poses_tab: PosesTab, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.poses_tab = poses_tab

        self._running = False
        self._frames = []
        self._cursor = 0

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        box = QtWidgets.QGroupBox("Hardcoded Walk (Pose 10-13 Reference)")
        layout = QtWidgets.QGridLayout(box)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(8)

        self.step_duration_label = QtWidgets.QLabel("Step Duration (ms)")
        self.step_duration_spin = QtWidgets.QSpinBox()
        self.step_duration_spin.setRange(100, 5000)
        self.step_duration_spin.setValue(320)

        self.gap_label = QtWidgets.QLabel("Pause Between Frames (ms)")
        self.gap_spin = QtWidgets.QSpinBox()
        self.gap_spin.setRange(0, 1000)
        self.gap_spin.setValue(50)

        self.btn_start = QtWidgets.QPushButton("Start Walking")
        self.btn_stop = QtWidgets.QPushButton("Stop")
        self.status = QtWidgets.QLabel("Idle")

        self.info = QtWidgets.QLabel(
            "Uses Stand + Pose 10-13 deltas.\n"
            "Sequence: BL -> FR -> BR -> FL with adjacent hip stabilization."
        )

        layout.addWidget(self.step_duration_label, 0, 0)
        layout.addWidget(self.step_duration_spin, 0, 1)
        layout.addWidget(self.gap_label, 0, 2)
        layout.addWidget(self.gap_spin, 0, 3)
        layout.addWidget(self.btn_start, 1, 0, 1, 2)
        layout.addWidget(self.btn_stop, 1, 2, 1, 2)
        layout.addWidget(self.status, 2, 0, 1, 4)
        layout.addWidget(self.info, 3, 0, 1, 4)

        root.addWidget(box)
        root.addStretch()

        self.btn_start.clicked.connect(self.start_walking)
        self.btn_stop.clicked.connect(self.stop_walking)
        self.btn_stop.setEnabled(False)

    def _find_pose_index(self, name):
        target = name.strip().lower()
        for i, n in enumerate(self.poses_tab.pose_names):
            if n.strip().lower() == target:
                return i
        return -1

    def _pose_vector(self, pose_name):
        idx = self._find_pose_index(pose_name)
        if idx < 0:
            return None
        pose = self.poses_tab.pose_angles[idx]
        return [float(pose[i]) for i in range(MOTOR_COUNT)]

    def _leg_base(self, leg_name):
        mapping = {
            "BL": 0,
            "FL": 3,
            "FR": 6,
            "BR": 9,
        }
        return mapping[leg_name]

    def _adjacent_leg(self, stepping_leg):
        mapping = {
            "BL": "FL",
            "FL": "BL",
            "FR": "BR",
            "BR": "FR",
        }
        return mapping[stepping_leg]

    def _copy_leg_delta(self, frame, delta, ref_leg_name, dst_leg_name):
        ref_base = self._leg_base(ref_leg_name)
        dst_base = self._leg_base(dst_leg_name)
        for j in range(3):
            frame[dst_base + j] += delta[ref_base + j]

    def _build_frames(self):
        stand = self._pose_vector("Stand")
        p10 = self._pose_vector("Pose 10")
        p11 = self._pose_vector("Pose 11")
        p12 = self._pose_vector("Pose 12")
        p13 = self._pose_vector("Pose 13")

        if not stand or not p10 or not p11 or not p12 or not p13:
            return None

        d10 = [p10[i] - stand[i] for i in range(MOTOR_COUNT)]
        d11 = [p11[i] - stand[i] for i in range(MOTOR_COUNT)]
        d12 = [p12[i] - stand[i] for i in range(MOTOR_COUNT)]
        d13 = [p13[i] - stand[i] for i in range(MOTOR_COUNT)]
        phase_deltas = [d10, d11, d12, d13]

        order = ["BL", "FR", "BR", "FL"]
        frames = []
        labels = []
        for step_leg in order:
            adj_leg = self._adjacent_leg(step_leg)
            for phase_i, delta in enumerate(phase_deltas):
                frame = stand[:]
                self._copy_leg_delta(frame, delta, "BL", step_leg)
                self._copy_leg_delta(frame, delta, "FL", adj_leg)

                frame = [max(ANGLE_MIN, min(ANGLE_MAX, int(round(a)))) for a in frame]
                frames.append(frame)
                labels.append(f"{step_leg} phase {phase_i + 1}")

        return frames, labels

    def start_walking(self):
        if self._running:
            return

        built = self._build_frames()
        if not built:
            self.status.setText("Missing required poses: Stand, Pose 10, 11, 12, 13")
            return

        self._frames, self._labels = built
        self._cursor = 0
        self._running = True
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.status.setText("Walking...")
        self._run_next_frame()

    def stop_walking(self):
        if not self._running:
            return
        self._running = False
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.status.setText("Stopped")

    def _run_next_frame(self):
        if not self._running:
            return

        if self._cursor >= len(self._frames):
            self._cursor = 0

        frame = self._frames[self._cursor]
        label = self._labels[self._cursor]
        self._cursor += 1

        duration_ms = int(self.step_duration_spin.value())
        self.controller.apply_pose_angles(frame, duration_ms=duration_ms)
        self.status.setText(f"Walking: {label} ({self._cursor}/{len(self._frames)})")

        wait_ms = duration_ms + int(self.gap_spin.value())
        QtCore.QTimer.singleShot(wait_ms, self._run_next_frame)


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
