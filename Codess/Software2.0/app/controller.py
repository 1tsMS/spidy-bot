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
import threading

from spidy.link import Link, Pinger, resolve_host, discover
from spidy.motion import SlewLimiter, PoseMove, Sequence
from spidy.poses import PoseLibrary, from_legacy_poses
from spidy.sim import SimWorld

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(ROOT, "config")
CAL_PATH = os.path.join(CONFIG, "calibration.json")
POSES_PATH = os.path.join(CONFIG, "poses.json")
LEGACY_DIR = os.path.join(ROOT, "..", "Software")     # the old tool's files, Codess/Software

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
    link_verified = Signal(str)       # firmware answered HELLO: "fw=2,ip=...,imu=1"
    _discovered = Signal(object)
    _reconnected = Signal(object)     # (ip, ok, error) from the reconnect thread
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
        self.verified = False                    # Spidy firmware answered on this link
        self._hello_until = 0.0                  # keep asking HELLO until this time
        self._t_hello = 0.0
        self._warned_unarmed = 0.0
        self._tcp_ip = None                      # last Wi-Fi robot address (for auto-reconnect)
        self._user_disconnect = False
        self._reconnect_until = 0.0
        self._t_reconnect = 0.0
        self._reconnecting = False
        self._soft_start = None                  # time the soft start began (see _stream_pulses)
        self._boot_logged = False
        self._stream_state = None                # '' = streaming, else why not
        self.tx_frames = 0                       # P16 frames actually handed to the socket

        self.link = Link(on_line=self._rx_from_thread.emit, on_status=self._status_from_thread.emit)
        self._rx_from_thread.connect(self._on_line)
        self._discovered.connect(self._on_discovered)
        self._reconnected.connect(self._on_reconnected)
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

        stream_why = self._stream_blocker()
        if stream_why != self._stream_state:          # say it out loud whenever streaming starts/stops
            if self._stream_state is not None and self.armed or stream_why == "":
                self.log.emit("streaming to the robot: ON" if stream_why == "" else
                              f"streaming to the robot PAUSED: {stream_why}")
            self._stream_state = stream_why
        if stream_why == "":
            if self.link.send_pulses(self._stream_pulses(now)):
                self.tx_frames += 1

        if self.link.connected and now - self._t_ping > 1.0:
            self._t_ping = now
            self.link.send(self.pinger.make())
        if self.link.connected and self.imu_polling and now - self._t_imu > 0.1:
            self._t_imu = now
            self.link.send("IMU")
        if (not self.link.connected and self._reconnect_until and not self._reconnecting
                and now - self._t_reconnect > 2.0):
            if now > self._reconnect_until:
                self._reconnect_until = 0.0
                self.log.emit("! gave up reconnecting after 60 s. Press Connect to try again.")
            else:
                self._t_reconnect = now
                self._reconnecting = True
                ip = self._tcp_ip

                def attempt():
                    try:
                        s = __import__("socket").create_connection((ip, 5000), timeout=2.0)
                        s.close()
                        self._reconnected.emit((ip, True, ""))
                    except OSError as e:
                        self._reconnected.emit((ip, False, str(e)))
                threading.Thread(target=attempt, daemon=True).start()
        if self.link.connected and not self.verified and self._hello_until:
            if now > self._hello_until:
                self._hello_until = 0.0
                self.log.emit("! link is open but no Spidy firmware answers. Wrong port, old firmware, "
                              "or the ESP is still booting? (it can take ~10 s to join Wi-Fi)")
            elif now - self._t_hello > 1.0:          # the ESP may be rebooting / joining Wi-Fi: keep asking
                self._t_hello = now
                self.link.send("HELLO")
        if self._arm_wait and now > self._arm_wait:
            self._finish_arm(None)
        self.ticked.emit()

    def _stream_blocker(self):
        """'' if P16 frames go out this tick, otherwise the reason they don't."""
        if not self.armed:
            return "not armed"
        if not self.link.connected:
            return "not connected"
        if self.mode == "sim":
            return "target is Sim"
        if self.raw_mode:
            return "raw mode (Motors 'Raw ticks' or a Calibration step) drives single channels instead"
        return ""

    SOFT_START_LEG_S = 0.35

    def _stream_pulses(self, now):
        """P16 frame. Right after arming with LIMP servos, legs are switched on one at a
        time (0.35 s apart) instead of all 12 servos jumping at once: that inrush current
        can sag the supply enough to brown out the ESP32."""
        pulses = self.cal.channel_pulses(self.q)
        if self._soft_start is None:
            return pulses
        legs_on = int((now - self._soft_start) / self.SOFT_START_LEG_S) + 1
        if legs_on >= 4:
            self._soft_start = None
            return pulses
        for i, n in enumerate(JOINT_NAMES):
            if i // 3 >= legs_on:
                pulses[self.cal.joints[n].channel] = 0
        return pulses

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

    @property
    def real_blocked(self):
        """Why the REAL robot would not move right now, or '' if it would."""
        if self.mode == "sim":
            return ""
        if not self.link.connected:
            return "not connected"
        if not self.verified:
            return "firmware not answering"
        if not self.armed:
            return "not armed"
        return ""

    def _warn_if_blocked(self):
        why = self.real_blocked
        if why and time.monotonic() - self._warned_unarmed > 3:
            self._warned_unarmed = time.monotonic()
            self.log.emit(f"! the real robot will NOT move: {why}"
                          + (" (flip the ARM switch, top right)" if why == "not armed" else ""))

    def move_to(self, q_deg, seconds=1.0, label="move"):
        self._warn_if_blocked()
        self.gait.set_command(0, 0)
        self.motion = PoseMove(self.q, q_deg, seconds)
        self.motion_label = label

    def play_sequence(self, name, max_steps=None, step_ms=None):
        steps = self.poses.sequence_steps(name, max_steps, step_ms)
        if not steps:
            self.log.emit(f"sequence '{name}' is empty")
            return
        self._warn_if_blocked()
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
        if forward or turn:
            self._warn_if_blocked()
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
    def _start_handshake(self):
        """The link being open proves nothing (a COM port always opens). Only a SPIDY reply
        to HELLO does. Keep asking for 12 s: opening USB serial resets the ESP32, and it
        can spend up to 10 s joining Wi-Fi before it listens."""
        self.verified = False
        self._boot_logged = False
        self._hello_until = time.monotonic() + 12.0
        self._t_hello = 0.0

    def connect_serial(self, port):
        try:
            self.link.open_serial(port)
            self._start_handshake()
        except Exception as e:
            self.log.emit(f"! serial: {e}")

    def connect_tcp(self, host):
        try:
            ip = resolve_host(host)
        except OSError:
            self.log.emit(f"{host} did not resolve (phone hotspots often block mDNS). Scanning the network for Spidy...")
            threading.Thread(target=lambda: self._discovered.emit(discover()), daemon=True).start()
            return
        self._open_tcp(ip)

    def _open_tcp(self, ip):
        try:
            self.link.open_tcp(ip)
            self._tcp_ip = ip
            self._user_disconnect = False
            self._reconnect_until = 0.0
            self._start_handshake()
        except Exception as e:
            self.log.emit(f"! wifi {ip}: {e}")

    def _on_reconnected(self, res):
        ip, ok, err = res
        self._reconnecting = False
        if ok and self._reconnect_until and not self.link.connected:
            self.log.emit(f"robot is back at {ip}, reconnecting")
            self._open_tcp(ip)

    def _note_boot(self, text):
        """READY/SPIDY lines carry boot=<reason>. Anything but a normal power-on means
        the ESP restarted on its own, which is the real cause of most 'random' drops."""
        fields = dict(kv.split("=", 1) for kv in text.split(",") if "=" in kv)
        boot, up = fields.get("boot"), fields.get("up")
        why = {"BROWNOUT": "the supply voltage dipped (usually a servo current spike: check the ESP's 5 V "
                           "rail and grounds, add a big capacitor near the board)",
               "CRASH": "the firmware crashed", "WATCHDOG": "the firmware hung and the watchdog reset it"}
        if fields.get("pca") == "0":
            self.log.emit("! the ESP can't find the PCA9685 servo driver on I2C (SDA/SCL wiring, 3.3 V/GND to "
                          "the PCA logic side, address jumpers). Type I2C? in the console for a bus scan.")
        if fields.get("pca_err", "0") not in ("0", ""):
            self.log.emit(f"! {fields['pca_err']} servo-driver writes failed on I2C. Type PCA? to read the chip back.")
        if boot in why and not self._boot_logged:
            self._boot_logged = True
            self.log.emit(f"! the ESP restarted because of a {boot}: {why[boot]}"
                          + (f" ({up} s ago)" if up else ""))

    def _on_discovered(self, ips):
        if not ips:
            self.log.emit("! no Spidy found on this network (port 5000). Is the ESP powered, on the same "
                          "hotspot, and running spidy_fw? Some hotspots isolate clients from each other.")
            return
        self.log.emit(f"found Spidy at {ips[0]}" + (f" (also: {', '.join(ips[1:])})" if len(ips) > 1 else ""))
        self._open_tcp(ips[0])

    def disconnect(self):
        self._user_disconnect = True
        self._reconnect_until = 0.0
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
        if not self.link.connected or not self.verified:
            self.log.emit("! can't arm: " + ("connect to the robot first" if not self.link.connected
                                             else "the Spidy firmware hasn't answered yet"))
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
            self.log.emit("armed: servos were limp, waking them one leg at a time (soft start)")
            self._soft_start = time.monotonic()
        self.armed = True
        self.armed_changed.emit(True)

    def _on_status(self, connected, text):
        self.log.emit(text)
        if not connected:
            was_armed = self.armed
            self.set_armed(False)
            self.verified = False
            self._hello_until = 0.0
            if self._tcp_ip and not self._user_disconnect and not self._reconnect_until:
                self._reconnect_until = time.monotonic() + 60.0
                self.log.emit(f"! lost the robot{' while ARMED' if was_armed else ''}. Reconnecting to "
                              f"{self._tcp_ip} (it will NOT re-arm by itself)...")
        self.link_changed.emit(connected, self.link.name if connected else "")

    def _on_line(self, line):
        self.pinger.feed(line)
        if line.startswith("PONG"):
            return
        if line.startswith("READY") or line.startswith("SPIDY,"):
            self._note_boot(line)
        if line.startswith("SPIDY,"):
            if not self.verified:
                self.verified = True
                self._hello_until = 0.0
                info = line[6:]
                self.log.emit(f"Spidy firmware answered: {info}")
                self.link_verified.emit(info)
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
