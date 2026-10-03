"""RobotController: the one object between the GUI and the robot.

Every 20 ms (tick):
    target  <- one source: a running pose move/sequence, the gait, or the manual target
    target  <- clamped to each joint's safe range (calibration)
    q       <- SlewLimiter(target)          never faster than MAX_RATE deg/s
    sim     <- physics(q)  or  mirror(q)    depending on the view mode
    robot   <- P16 pulses from calibration  only when ARMED, connected and target is Real/Both

RAW mode (Motors tab "raw", calibration wizard): streaming stops and single
channels are driven directly with M,<ch>,<ticks>. The sim then shows what the
calibration THINKS those ticks mean.
"""
import os
import time

import numpy as np
from PySide6.QtCore import QObject, QTimer, Signal, Qt

from spidy import calibration as C
from spidy.calibration import Calibration, from_legacy_txt
from spidy.gait import CrawlGait
from spidy.imu import parse_imu
from spidy.kinematics import JOINT_NAMES
from spidy.link import Link, Pinger, resolve_host
from spidy.motion import SlewLimiter, PoseMove, Sequence
from spidy.poses import PoseLibrary, from_legacy_poses
from spidy.sim import SimWorld

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(ROOT, "config")
CAL_PATH = os.path.join(CONFIG, "calibration.json")
POSES_PATH = os.path.join(CONFIG, "poses.json")
LEGACY_DIR = os.path.join(ROOT, "Codess", "Software")

DT = 0.02            # control tick, s (50 Hz, same as the PCA9685 frame rate)
MAX_RATE = 240.0     # deg/s, slew limit on every joint


def load_or_import():
    """config/*.json if present, else import the old tool's files once."""
    os.makedirs(CONFIG, exist_ok=True)
    msgs = []
    if os.path.exists(CAL_PATH):
        cal = Calibration.load(CAL_PATH)
    else:
        cal = from_legacy_txt(os.path.join(LEGACY_DIR, "spidy_calibration3.txt"))
        cal.save(CAL_PATH)
        msgs.append("calibration imported from spidy_calibration3.txt -> config/calibration.json")
    if os.path.exists(POSES_PATH):
        poses = PoseLibrary.load(POSES_PATH)
    else:
        poses = from_legacy_poses(os.path.join(LEGACY_DIR, "spidy_poses.txt"),
                                  os.path.join(LEGACY_DIR, "spidy_calibration3.txt"))
        poses.save(POSES_PATH)
        msgs.append("poses imported from spidy_poses.txt -> config/poses.json")
    return cal, poses, msgs


class RobotController(QObject):
    log = Signal(str)                 # human-readable log line
    rx = Signal(str)                  # raw line from the robot (main thread)
    link_changed = Signal(bool, str)
    armed_changed = Signal(bool)
    mode_changed = Signal(str)
    imu_sample = Signal(object, object)   # accel, gyro (numpy)
    ticked = Signal()                 # after every control tick
    calibration_changed = Signal()

    _rx_from_thread = Signal(str)
    _status_from_thread = Signal(bool, str)

    def __init__(self):
        super().__init__()
        self.cal, self.poses, msgs = load_or_import()
        self.sim = SimWorld()
        self.gait = CrawlGait()
        self.slew = SlewLimiter(MAX_RATE)

        if "Standf" in self.poses.poses:          # the gait stands in YOUR tuned Standf
            self.gait.set_stance(self.poses.get("Standf"))
        self.target = self.gait.stance_deg.copy()  # manual target (deg)
        self.q = self.target.copy()              # what is actually commanded right now
        self.slew.reset(self.q)
        self.sim.reset(self.q)
        self.motion = None                       # PoseMove / Sequence / None
        self.motion_label = ""
        self.mode = "sim"                        # sim | real | both
        self.armed = False
        self.raw_mode = False
        self.raw_ticks = [0] * C.CHANNELS        # last raw tick per channel (raw mode)
        self.speed = 0.6                         # walk speed scale 0..1
        self._walk_after_move = None
        self._arm_wait = None
        self.imu_polling = False
        self.robot_state = None                  # last STATE reply (16 ticks)

        self.link = Link(on_line=self._rx_from_thread.emit, on_status=self._status_from_thread.emit)
        self._rx_from_thread.connect(self._on_line)
        self._status_from_thread.connect(self._on_status)
        self.pinger = Pinger()
        self._t_ping = self._t_imu = 0.0

        self.timer = QTimer(self)
        self.timer.setTimerType(Qt.PreciseTimer)   # Windows' default timer is ~16 ms coarse
        self.timer.timeout.connect(self.tick)
        self.timer.start(int(DT * 1000))
        self._t_last = time.monotonic()
        for m in msgs:
            QTimer.singleShot(0, lambda m=m: self.log.emit(m))

    # ================================================================ main loop
    def tick(self):
        # Real elapsed time, not the nominal 20 ms: if the GUI delays a tick, the sim,
        # the gait and pose moves still run in real time. Capped so a long stall
        # (window drag, breakpoint) doesn't produce one giant step.
        now = time.monotonic()
        dt = min(now - self._t_last, 0.1)
        self._t_last = now
        if self.motion is not None:
            tgt = self.motion.update(dt)
            if self.motion.done:
                self.target = np.array(tgt)
                self.motion = None
                self.motion_label = ""
                if self._walk_after_move is not None:
                    self.gait.set_command(*self._walk_after_move)
                    self._walk_after_move = None
        elif self.gait.walking or np.abs(self.gait.cmd).max() > 0:
            tgt = self.gait.update(dt)
            if not self.gait.walking:
                self.target = np.array(tgt)
        else:
            tgt = self.target

        if self.raw_mode:
            self.q = self._raw_as_deg()
            self.slew.reset(self.q)
        else:
            self.q = self.slew.update(self.cal.clamp_deg(tgt), dt)

        if self.mode in ("sim", "both"):
            self.sim.physics(self.q, dt)
        else:
            self.sim.mirror(self.q)

        if self.armed and self.link.connected and self.mode in ("real", "both") and not self.raw_mode:
            self.link.send_pulses(self.cal.channel_pulses(self.q))

        if self.link.connected and now - self._t_ping > 1.0:
            self._t_ping = now
            self.link.send(self.pinger.make())
        if self.link.connected and self.imu_polling and now - self._t_imu > 0.1:
            self._t_imu = now
            self.link.send("IMU")
        if self._arm_wait and now > self._arm_wait:
            self._finish_arm(None)
        self.ticked.emit()

    def _raw_as_deg(self):
        """Raw channel ticks -> joint degrees, as the current calibration reads them."""
        out = np.array(self.q, float)
        for i, n in enumerate(JOINT_NAMES):
            t = self.raw_ticks[self.cal.joints[n].channel]
            if t:
                out[i] = self.cal.joints[n].to_deg(t)
        return out

    # ================================================================ commands from the GUI
    def set_mode(self, mode):
        self.mode = mode
        if mode in ("sim", "both"):
            self.sim.reset(self.q)               # start the physics from the current pose
        self.mode_changed.emit(mode)

    def set_target(self, q_deg):
        """Manual target (Motors tab). Cancels pose moves and walking."""
        self.cancel_motion()
        self.gait.set_command(0, 0)
        self.target = np.array(q_deg, float)

    def set_joint(self, name, deg):
        q = np.array(self.target, float)
        q[JOINT_NAMES.index(name)] = deg
        self.set_target(q)

    def move_to(self, q_deg, seconds=1.0, label="move"):
        self.gait.set_command(0, 0)
        self.motion = PoseMove(self.q, q_deg, seconds)
        self.motion_label = label

    def play_sequence(self, name, max_steps=None, step_ms=None):
        steps = self.poses.sequence_steps(name, max_steps, step_ms)
        if not steps:
            self.log.emit(f"sequence '{name}' is empty")
            return
        self.gait.set_command(0, 0)
        self.motion = Sequence(self.q, steps)
        self.motion_label = name
        self.log.emit(f"sequence: {name} ({len(steps)} steps"
                      + (f", {step_ms} ms each)" if step_ms else ")"))

    def cancel_motion(self):
        self.motion = None
        self.motion_label = ""
        self._walk_after_move = None

    def walk(self, forward, turn):
        """Live walk command, -1..1 each (scaled by self.speed)."""
        cmd = (forward * self.speed, turn * self.speed)
        if self.raw_mode:
            return
        idle = not self.gait.walking and self.motion is None
        far = np.abs(np.asarray(self.q) - self.gait.stance_deg).max() > 3
        if (forward or turn) and idle and far:
            self.move_to(self.gait.stance_deg, 1.0, "to stance")     # get into stance first
            self._walk_after_move = cmd
            return
        if self._walk_after_move is not None:
            self._walk_after_move = cmd
            return
        self.gait.set_command(*cmd)

    def reset_sim(self):
        self.sim.reset(self.q)

    def estop(self):
        """Cut every servo output and stop everything that moves."""
        self.link.send("OFF")
        self.gait.set_command(0, 0)
        self.gait.amount = 0.0
        self.cancel_motion()
        self.target = np.array(self.q)
        self.set_armed(False)
        self.log.emit("E-STOP: outputs off, disarmed")

    # ---- raw channel control
    def set_raw_mode(self, on):
        if on == self.raw_mode:
            return
        self.raw_mode = on
        if on:
            self.gait.set_command(0, 0)
            self.gait.amount = 0.0
            self.cancel_motion()
            self.raw_ticks = list(self.cal.channel_pulses(self.q))
        else:
            self.target = np.array(self.q)
            self.slew.reset(self.q)

    def raw_set(self, channel, ticks):
        ticks = int(np.clip(ticks, C.HARD_MIN, C.HARD_MAX))
        self.raw_ticks[channel] = ticks
        if self.link.connected and self.mode in ("real", "both"):
            self.link.send(f"M,{channel},{ticks}")

    def raw_set_many(self, pulses16):
        for ch, t in enumerate(pulses16):
            if t:
                self.raw_ticks[ch] = int(np.clip(t, C.HARD_MIN, C.HARD_MAX))
        if self.link.connected and self.mode in ("real", "both"):
            self.link.send_pulses([self.raw_ticks[ch] if pulses16[ch] else 0 for ch in range(C.CHANNELS)])

    # ---- calibration / files
    def apply_scale(self, new_tpd, rescale_poses=True):
        """Set every joint's ticks_per_deg. Limits (and optionally poses) are rescaled
        so they keep the SAME physical position (same ticks)."""
        ratios = []
        for jc in self.cal.joints.values():
            r = jc.ticks_per_deg / new_tpd
            ratios.append(r)
            jc.min_deg, jc.max_deg = round(jc.min_deg * r, 2), round(jc.max_deg * r, 2)
            jc.ticks_per_deg = float(new_tpd)
        r = float(np.mean(ratios))
        if rescale_poses:
            for name in list(self.poses.poses):
                self.poses.put(name, np.array(self.poses.get(name)) * r)
            self.cal.boot_pose_deg = [v * r for v in self.cal.boot_pose_deg]
            if "Standf" in self.poses.poses:
                self.gait.set_stance(self.poses.get("Standf"))
            self.save_poses()
        self.target = np.array(self.target) * r
        self.q = np.array(self.q) * r
        self.slew.reset(self.q)
        self.calibration_changed.emit()
        self.log.emit(f"scale set to {new_tpd:.3f} ticks/deg (x{1 / r:.3f})"
                      + ("; poses and limits rescaled to keep their physical shape" if rescale_poses else ""))

    def save_calibration(self):
        if os.path.exists(CAL_PATH):
            os.makedirs(os.path.join(CONFIG, "backups"), exist_ok=True)
            stamp = time.strftime("%Y%m%d-%H%M%S")
            os.replace(CAL_PATH, os.path.join(CONFIG, "backups", f"calibration-{stamp}.json"))
        self.cal.save(CAL_PATH)
        self.log.emit("calibration saved (old one backed up in config/backups)")

    def reload_calibration(self):
        self.cal = Calibration.load(CAL_PATH)
        self.calibration_changed.emit()
        self.log.emit("calibration reloaded from file")

    def save_poses(self):
        self.poses.save(POSES_PATH)
        self.log.emit("poses saved")

    def push_boot_pose(self, q_deg):
        pulses = self.cal.channel_pulses(q_deg)
        self.link.send("BOOT," + ",".join(map(str, pulses)))
        self.log.emit("boot pose sent to robot flash")

    # ================================================================ link
    def connect_serial(self, port):
        try:
            self.link.open_serial(port)
            self.link.send("HELLO")
        except Exception as e:
            self.log.emit(f"! serial: {e}")

    def connect_tcp(self, host):
        try:
            ip = resolve_host(host)
            self.link.open_tcp(ip)
            self.link.send("HELLO")
        except Exception as e:
            self.log.emit(f"! wifi {host}: {e}")

    def disconnect(self):
        self.set_armed(False)
        self.link.close()

    def set_armed(self, on):
        """Arming = allow streaming to the real servos. First asks the robot where
        its servos are (STATE) so streaming starts from there, not with a jump."""
        if not on:
            self._arm_wait = None
            if self.armed:
                self.armed = False
                self.armed_changed.emit(False)
            return
        if not self.link.connected:
            self.log.emit("! connect to the robot before arming")
            self.armed_changed.emit(False)
            return
        self.link.send("STATE")
        self._arm_wait = time.monotonic() + 1.0

    def _finish_arm(self, state):
        self._arm_wait = None
        if state and all(state[self.cal.joints[n].channel] for n in JOINT_NAMES):
            q0 = [self.cal.joints[n].to_deg(state[self.cal.joints[n].channel]) for n in JOINT_NAMES]
            self.q = np.array(q0)
            self.slew.reset(self.q)
            self.target = np.array(self.q)
            self.log.emit("armed: starting from the robot's current servo positions")
        else:
            self.log.emit("armed: robot outputs were off, servos will move to the current target")
        self.armed = True
        self.armed_changed.emit(True)

    def _on_status(self, connected, text):
        self.log.emit(text)
        if not connected:
            self.set_armed(False)
        self.link_changed.emit(connected, self.link.name if connected else "")

    def _on_line(self, line):
        self.pinger.feed(line)
        if line.startswith("PONG"):
            return
        if line.startswith("IMU,"):
            s = parse_imu(line)
            if s:
                self.imu_sample.emit(*s)
            return
        if line.startswith("STATE,"):
            try:
                self.robot_state = [int(v) for v in line.split(",")[1:17]]
            except ValueError:
                self.robot_state = None
            if self._arm_wait:
                self._finish_arm(self.robot_state)
        self.rx.emit(line)

    def shutdown(self):
        self.timer.stop()
        self.link.close()
        self.sim.close()
