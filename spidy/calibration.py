"""Servo calibration: joint angle (deg, URDF convention) <-> PCA9685 pulse ticks.

One model per joint, one source of truth (this file's JSON, on the PC):

    ticks = center + direction * ticks_per_deg * joint_deg

  center         ticks where the real joint matches the URDF zero pose
                 (femur flat, tibia vertical). Absorbs horn tightening / fit errors.
  direction      +1 / -1: does a +joint angle (same + as the sim) increase ticks?
  ticks_per_deg  servo scale. Same for every MG90S, measured once (wizard, IMU).
  min/max_deg    safe range in JOINT degrees (no more mixed angle spaces).

PCA9685 at 50 Hz: 4096 ticks per 20 ms, so 1 tick = 4.88 us.
"""
import json
from dataclasses import dataclass, asdict, field

import numpy as np

from .kinematics import LEGS, JOINT_NAMES, logical_to_joint_deg

CHANNELS = 16
HARD_MIN, HARD_MAX = 100, 700        # absolute tick limits (old firmware's SERVO_MIN/MAX)
RAW_90 = 400                         # "servo 90": the pulse the legs were assembled at (Set_90 sketch)
OLD_TICKS_PER_LOGICAL = 600 / 180    # old tool: 0..180 logical -> 100..700 ticks


def servo_deg(ticks):
    """Old-style 0-180 'servo degrees' for display: 100 ticks = 0, 400 = 90, 700 = 180."""
    return (ticks - HARD_MIN) / OLD_TICKS_PER_LOGICAL


@dataclass
class JointCal:
    channel: int
    center: float
    direction: int = 1
    ticks_per_deg: float = OLD_TICKS_PER_LOGICAL
    min_deg: float = -60.0
    max_deg: float = 60.0

    def clamp_deg(self, deg):
        return float(np.clip(deg, self.min_deg, self.max_deg))

    def to_ticks(self, deg):
        """Joint angle -> ticks. Clamped to the joint's safe range, then the hard range."""
        t = self.center + self.direction * self.ticks_per_deg * self.clamp_deg(deg)
        return int(round(np.clip(t, HARD_MIN, HARD_MAX)))

    def to_deg(self, ticks):
        return (ticks - self.center) / (self.direction * self.ticks_per_deg)


# Default boot pose = the old "Sit" (logical 90, 120, 100 on every leg)
SIT_DEG = [float(v) for leg in LEGS for v in logical_to_joint_deg(leg, (90, 120, 100))]


@dataclass
class Calibration:
    joints: dict                                     # name -> JointCal
    boot_pose_deg: list = field(default_factory=lambda: list(SIT_DEG))
    imu: dict = field(default_factory=dict)          # level reference etc.
    note: str = ""

    # ---- conversions
    def joint_ticks(self, q_deg):
        """12 joint angles (deg, JOINT_NAMES order) -> {name: ticks}."""
        return {n: self.joints[n].to_ticks(q) for n, q in zip(JOINT_NAMES, q_deg)}

    def channel_pulses(self, q_deg):
        """12 joint angles -> 16 channel pulses for the P16 command (0 = channel unused)."""
        out = [0] * CHANNELS
        for name, t in self.joint_ticks(q_deg).items():
            out[self.joints[name].channel] = t
        return out

    def clamp_deg(self, q_deg):
        return [self.joints[n].clamp_deg(q) for n, q in zip(JOINT_NAMES, q_deg)]

    def problems(self):
        """Human-readable list of things that are wrong with this calibration."""
        msgs = []
        chans = [j.channel for j in self.joints.values()]
        dupes = {c for c in chans if chans.count(c) > 1}
        if dupes:
            msgs.append(f"channels used twice: {sorted(dupes)}")
        for n, j in self.joints.items():
            if not (0 <= j.channel < CHANNELS):
                msgs.append(f"{n}: channel {j.channel} out of range")
            if j.min_deg >= j.max_deg:
                msgs.append(f"{n}: min >= max")
            if not (HARD_MIN <= j.center <= HARD_MAX):
                msgs.append(f"{n}: center {j.center} outside {HARD_MIN}-{HARD_MAX}")
        return msgs

    # ---- files
    def to_dict(self):
        return {"version": 1, "note": self.note,
                "joints": {n: asdict(j) for n, j in self.joints.items()},
                "boot_pose_deg": [round(v, 2) for v in self.boot_pose_deg],
                "imu": self.imu}

    def save(self, path):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def from_dict(cls, d):
        joints = {n: JointCal(**d["joints"][n]) for n in JOINT_NAMES}
        return cls(joints=joints, boot_pose_deg=d.get("boot_pose_deg", list(SIT_DEG)),
                   imu=d.get("imu", {}), note=d.get("note", ""))

    @classmethod
    def load(cls, path):
        with open(path, encoding="utf-8") as f:
            return cls.from_dict(json.load(f))

    def copy(self):
        return Calibration.from_dict(json.loads(json.dumps(self.to_dict())))


# ---------------------------------------------------------------- legacy import
LEGACY_JOINT = {"HIP": "hip", "KNEE": "knee", "ANKLE": "claw"}


def from_legacy_txt(path, deg_per_logical=1.0):
    """Import the old tab-separated calibration (spidy_calibration3.txt).

    Old tool, per motor:  corrected = L + off (dir +)  or  180 - L - off (dir -)
                          pulse     = 100 + corrected * 600/180
    and a pose angle L maps to the joint as  joint_deg = sy * (L - 90) * deg_per_logical.
    Solving for our model gives:
        center        = pulse at L = 90
        direction     = old_dir * sy
        ticks_per_deg = (600/180) / deg_per_logical
    Old min/max limited `corrected` (servo space); converted here to joint degrees."""
    joints = {}
    with open(path, encoding="utf-8") as f:
        rows = [ln.rstrip("\n").split("\t") for ln in f if ln.strip()]
    for r in rows[1:]:
        name = r[2].strip()
        if "_" not in name or name.split("_")[0].lower() not in LEGS:
            continue                                         # AUX channels
        leg, part = name.split("_")
        leg = leg.lower()
        sy = LEGS[leg][1]
        old_dir = 1 if r[3].strip() == "+" else -1
        off = int(r[4])
        lo, hi = int(r[6]), int(r[7])
        corrected_90 = 90 + off if old_dir > 0 else 90 - off
        jc = JointCal(channel=int(r[1]),
                      center=HARD_MIN + corrected_90 * OLD_TICKS_PER_LOGICAL,
                      direction=old_dir * sy,
                      ticks_per_deg=OLD_TICKS_PER_LOGICAL / deg_per_logical)
        d1 = jc.to_deg(HARD_MIN + lo * OLD_TICKS_PER_LOGICAL)
        d2 = jc.to_deg(HARD_MIN + hi * OLD_TICKS_PER_LOGICAL)
        jc.min_deg, jc.max_deg = round(min(d1, d2), 2), round(max(d1, d2), 2)
        joints[f"{leg}_{LEGACY_JOINT[part]}"] = jc
    missing = set(JOINT_NAMES) - set(joints)
    if missing:
        raise ValueError(f"legacy file is missing joints: {sorted(missing)}")
    return Calibration(joints=joints, note=f"imported from {path}")


def legacy_pulse(row_dir, off, lo, hi, logical):
    """The OLD tool's pulse formula, kept only so tests can prove the import is exact."""
    c = logical if row_dir > 0 else 180 - logical
    c += off if row_dir > 0 else -off
    c = min(max(c, lo), hi)
    return HARD_MIN + c * OLD_TICKS_PER_LOGICAL
