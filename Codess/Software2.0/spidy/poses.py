"""Pose library: named poses in JOINT degrees (same numbers as the sim) + sequences.

File format (poses.json):
  {"poses": [{"name": "Stand", "deg": [12 values, JOINT_NAMES order]}, ...],
   "sequences": [{"name": "Sit to Stand", "steps": [{"pose": "Sit", "ms": 800}, ...]}]}
"""
import json

import numpy as np

from .kinematics import LEGS, JOINT_NAMES, logical_to_joint_deg


class PoseLibrary:
    def __init__(self, poses=None, sequences=None):
        self.poses = poses or {}            # name -> list of 12 deg (insertion order kept)
        self.sequences = sequences or {}    # name -> list of (pose name, ms)

    def get(self, name):
        return list(self.poses[name])

    def put(self, name, deg):
        self.poses[name] = [round(float(v), 2) for v in deg]

    def rename(self, old, new):
        self.poses = {(new if k == old else k): v for k, v in self.poses.items()}
        for steps in self.sequences.values():
            steps[:] = [(new if p == old else p, ms) for p, ms in steps]

    def delete(self, name):
        self.poses.pop(name, None)

    def sequence_steps(self, name, max_steps=None, step_ms=None):
        """-> list of (q_deg12, seconds), skipping poses that no longer exist.
        max_steps: keep at most this many poses (see reduce_chain). step_ms: override durations."""
        steps = [(self.get(p), ms / 1000) for p, ms in self.sequences.get(name, []) if p in self.poses]
        if max_steps and len(steps) > max_steps:
            keep = reduce_chain([q for q, _ in steps], max_steps)
            steps = [steps[i] for i in keep]
        if step_ms:
            steps = [(q, step_ms / 1000) for q, _ in steps]
        return steps

    def save(self, path):
        data = {"joint_order": JOINT_NAMES,
                "poses": [{"name": n, "deg": d} for n, d in self.poses.items()],
                "sequences": [{"name": n, "steps": [{"pose": p, "ms": ms} for p, ms in s]}
                              for n, s in self.sequences.items()]}
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=1)

    @classmethod
    def load(cls, path):
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        poses = {p["name"]: list(map(float, p["deg"])) for p in data.get("poses", [])}
        seqs = {s["name"]: [(st["pose"], int(st["ms"])) for st in s["steps"]]
                for s in data.get("sequences", [])}
        return cls(poses, seqs)


# Old tool's "optimisation" profiles for pose chains: how many poses to keep at most.
PROFILES = {"aggressive": 3, "balanced": 4, "conservative": 6}


def _seg_dist(p, a, b):
    """Distance from pose p to the straight line (in joint space) between poses a and b."""
    ab, ap = b - a, p - a
    L = float(ab @ ab)
    t = 0.0 if L < 1e-9 else min(max(float(ap @ ab) / L, 0.0), 1.0)
    return float(np.linalg.norm(p - (a + t * ab)))


def reduce_chain(poses, max_steps):
    """Drop the in-between poses that add the least, keeping at most max_steps (first
    and last always kept). Ramer-Douglas-Peucker in 12-D joint space: a pose that sits
    almost on the straight line between its neighbours is redundant, because a smooth
    move between the neighbours passes close to it anyway. Same idea as the old tool."""
    P = [np.asarray(q, float) for q in poses]
    if len(P) <= max_steps:
        return list(range(len(P)))
    keep = {0, len(P) - 1}
    while len(keep) < max_steps:
        best, best_i = -1.0, None
        ks = sorted(keep)
        for a, b in zip(ks, ks[1:]):                 # biggest deviation inside any gap
            for i in range(a + 1, b):
                d = _seg_dist(P[i], P[a], P[b])
                if d > best:
                    best, best_i = d, i
        if best_i is None:
            break
        keep.add(best_i)
    return sorted(keep)


def from_legacy_poses(poses_path, legacy_cal_path, deg_per_logical=1.0):
    """Import old spidy_poses.txt (16 logical angles per pose, in the old MOTOR order).
    The old calibration file tells which motor id is which joint."""
    with open(legacy_cal_path, encoding="utf-8") as f:
        rows = [ln.split("\t") for ln in f if ln.strip()][1:]
    motor_joint = {}
    for r in rows:
        name = r[2].strip()
        if name.startswith("AUX"):
            continue
        leg, part = name.split("_")
        motor_joint[int(r[0])] = (leg.lower(), {"HIP": 0, "KNEE": 1, "ANKLE": 2}[part])

    with open(poses_path, encoding="utf-8") as f:
        data = json.load(f)
    lib = PoseLibrary()
    for p in data.get("poses", []):
        deg = [0.0] * 12
        for motor, (leg, j) in motor_joint.items():
            logical = p["angles"][motor]
            deg[JOINT_NAMES.index(f"{leg}_{('hip', 'knee', 'claw')[j]}")] = float(
                logical_to_joint_deg(leg, (logical,) * 3, deg_per_logical)[j])
        lib.put(p["name"], deg)

    # The old tool's sit<->stand chain (Sit, Pose 4..8, Stand), if those poses exist
    chain = [n for n in ["Sit", "Pose 4", "Pose 5", "Pose 6", "Pose 7", "Pose 8", "Standf"] if n in lib.poses]
    if len(chain) >= 2:
        lib.sequences["Sit to Stand"] = [(n, 800) for n in chain]
        lib.sequences["Stand to Sit"] = [(n, 800) for n in reversed(chain)]
    return lib
