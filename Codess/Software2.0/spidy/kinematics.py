"""Leg geometry, FK and IK. Same maths as sim_walk/step1_leg_ik.py (explained there).

Conventions (match the URDF):
  body frame: x forward, y left, z up, metres, radians.
  joint order everywhere: fl, fr, bl, br  x  hip, knee, claw  (12 joints)
  every leg shares the same axes (hip +z, knee +x, claw +x), mirroring is done here.
"""
import numpy as np

HIP_X, HIP_Y = 0.060, 0.050      # hip axes from body centre
COXA, FEMUR, TIBIA = 0.0342, 0.0558, 0.0889
FOOT_X, FOOT_Y = -0.005, -0.0012  # foot tip offset (x sx, x sy)

LEGS = {"fl": (1, 1), "fr": (1, -1), "bl": (-1, 1), "br": (-1, -1)}   # sx, sy
JOINTS = ("hip", "knee", "claw")
JOINT_NAMES = [f"{leg}_{j}" for leg in LEGS for j in JOINTS]          # 12 names, actuator order


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def leg_fk(leg, angles):
    """(hip, knee, claw) rad -> foot position in the body frame."""
    sx, sy = LEGS[leg]
    hip, knee, claw = angles
    p = np.array([sx * FOOT_X, sy * FOOT_Y, -TIBIA])
    p = np.array([0, sy * FEMUR, 0]) + rot_x(claw) @ p
    p = np.array([0, sy * COXA, 0]) + rot_x(knee) @ p
    return np.array([sx * HIP_X, sy * HIP_Y, 0]) + rot_z(hip) @ p


def leg_ik(leg, foot):
    """Foot position in the body frame -> (hip, knee, claw) rad.
    Assumes the foot points outward from the hip. Raises ValueError if unreachable."""
    sx, sy = LEGS[leg]
    d = np.asarray(foot, float) - [sx * HIP_X, sy * HIP_Y, 0.0]
    x, y, z = d[0], sy * d[1], d[2]              # mirror right legs into left legs
    fx = sx * FOOT_X

    r = np.hypot(x, y)                           # top view -> hip
    if r < abs(fx):
        raise ValueError(f"{leg}: foot too close to the hip axis")
    out = np.sqrt(r**2 - fx**2)
    hip = np.arctan2(y, x) - np.arctan2(out, fx)
    hip = (hip + np.pi) % (2 * np.pi) - np.pi    # difference of two atan2 can leave +-pi: wrap

    u, v = out - COXA, z                         # side view -> 2-link arm
    tib = np.hypot(FOOT_Y, TIBIA)
    tib0 = np.arctan2(-TIBIA, FOOT_Y)
    D = np.hypot(u, v)
    cos_bend = (D**2 - FEMUR**2 - tib**2) / (2 * FEMUR * tib)
    if abs(cos_bend) > 1:
        raise ValueError(f"{leg}: foot out of reach")
    bend = -np.arccos(cos_bend)
    knee = np.arctan2(v, u) - np.arctan2(tib * np.sin(bend), FEMUR + tib * np.cos(bend))
    claw = bend - tib0
    return np.array([sy * hip, sy * knee, sy * claw])


def feet_to_angles(feet):
    """{leg: foot xyz (body frame)} -> 12 joint angles (rad), actuator order."""
    return np.concatenate([leg_ik(leg, feet[leg]) for leg in LEGS])


def angles_to_feet(q12):
    return {leg: leg_fk(leg, q12[3 * i: 3 * i + 3]) for i, leg in enumerate(LEGS)}


# ---- legacy "logical degrees" (old tool / spidy_poses.txt) -> joint angles
# 90 = modelled zero pose. Same number = same motion on every leg (calib dir already
# applied by the old firmware). +knee raises the femur (implied by the old Sit pose).
def logical_to_joint_deg(leg, logical3, deg_per_logical=1.0):
    sx, sy = LEGS[leg]
    return sy * (np.asarray(logical3, float) - 90.0) * deg_per_logical


STANDF_LOGICAL = (90, 105, 75)     # MS's standing pose: femur up 15, tibia vertical


def standf_deg():
    """12 joint angles (deg) of the Standf pose."""
    return np.concatenate([logical_to_joint_deg(leg, STANDF_LOGICAL) for leg in LEGS])
