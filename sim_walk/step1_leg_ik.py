"""
STEP 1: leg kinematics. FK and IK for one Spidy leg.

  FK (forward kinematics): 3 joint angles  -> where is the foot?
  IK (inverse kinematics): where I want the foot -> 3 joint angles

Every later step only ever says "put the feet HERE" and calls leg_ik().

Run:
  python sim_walk/step1_leg_ik.py              tests, then a viewer: FL foot draws a circle
  python sim_walk/step1_leg_ik.py --headless   tests only
"""
import sys
import numpy as np

# ---------------------------------------------------------------- geometry
# Metres. Read off the MuJoCo model (body positions in spidy.xml).
HIP_X = 0.060     # hip axis distance from the body centre, forward/back
HIP_Y = 0.050     # hip axis distance from the body centre, left/right
COXA = 0.0342     # hip axis   -> knee axis  (sideways, along y)
FEMUR = 0.0558    # knee axis  -> claw axis  (sideways, along y)
TIBIA = 0.0889    # claw axis  -> foot tip   (straight down)
FOOT_X = -0.005   # foot tip sits 5 mm toward the body centre (x sx)
FOOT_Y = -0.0012  # ...and 1.2 mm inward (x sy)

# sx: +1 front / -1 back.  sy: +1 left / -1 right.
# Order matters: it's the actuator order in spidy.xml (fl, fr, bl, br).
LEGS = {"fl": (1, 1), "fr": (1, -1), "bl": (-1, 1), "br": (-1, -1)}


# ---------------------------------------------------------------- your servo angles
# Poses in spidy_poses.txt use "logical degrees" per joint, [hip, knee, claw],
# with the calibration dir already applied, so the same number means the same
# motion on every leg. 90 = the modelled pose (femur flat, tibia straight down).
STANDF = [90, 105, 75]      # your standing pose: knee +15, claw -15
DEG_PER_LOGICAL = 1.0       # NOT MEASURED. If the pulse range is off (see CLAUDE.md),
                            # 1 logical deg may be ~1.46 real deg. Measure, then fix here.


def logical_to_joint(leg, logical):
    """[hip, knee, claw] in logical degrees -> joint angles (rad) for the model.
    +knee raises the femur on every leg (that's what your Sit pose implies).
    The URDF's +knee lifts LEFT legs only, so right legs get a minus sign.
    Hip direction is NOT verified on hardware yet."""
    sx, sy = LEGS[leg]
    hip, knee, claw = np.radians((np.asarray(logical, float) - 90) * DEG_PER_LOGICAL)
    return np.array([sy * hip, sy * knee, sy * claw])


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# ---------------------------------------------------------------- FK
def leg_fk(leg, angles):
    """(hip, knee, claw) in radians -> foot position in the BODY frame (metres)."""
    sx, sy = LEGS[leg]
    hip, knee, claw = angles

    # Where each joint sits in its parent's frame, at zero angles
    hip_pos = np.array([sx * HIP_X, sy * HIP_Y, 0.0])
    knee_pos = np.array([0.0, sy * COXA, 0.0])
    claw_pos = np.array([0.0, sy * FEMUR, 0.0])
    foot_pos = np.array([sx * FOOT_X, sy * FOOT_Y, -TIBIA])

    # Walk the chain from the foot back to the body.
    # Each joint rotates everything after it, then we shift by where the joint sits.
    p = foot_pos
    p = claw_pos + rot_x(claw) @ p
    p = knee_pos + rot_x(knee) @ p
    p = hip_pos + rot_z(hip) @ p
    return p


# ---------------------------------------------------------------- IK
def leg_ik(leg, foot):
    """Foot position in the BODY frame -> (hip, knee, claw) radians.
    Assumes the foot sticks outward from the hip (true for walking).
    Raises ValueError if the foot is out of reach."""
    sx, sy = LEGS[leg]

    # 1) Foot relative to this leg's hip axis.
    d = np.asarray(foot, dtype=float) - [sx * HIP_X, sy * HIP_Y, 0.0]

    # 2) Mirror trick: flip y on right legs so every leg looks like a LEFT leg.
    #    Solve once, then flip the angles back at the end.
    x, y, z = d[0], sy * d[1], d[2]
    fx = sx * FOOT_X                    # foot's small forward offset (x isn't mirrored)

    # 3) HIP (yaw), seen from above.
    #    The leg is a line pointing sideways, but the foot sits fx off that line.
    #    r = horizontal distance hip -> foot. Pythagoras gives how far "out" the foot is.
    r = np.hypot(x, y)
    if r < abs(fx):
        raise ValueError(f"{leg}: foot too close to the hip axis")
    out = np.sqrt(r**2 - fx**2)
    hip = np.arctan2(y, x) - np.arctan2(out, fx)

    # 4) KNEE + CLAW, seen in the leg's own vertical plane.
    #    Now it's a 2-link arm: femur (FEMUR long) then tibia (claw axis -> foot tip).
    u = out - COXA                      # horizontal distance knee axis -> foot
    v = z                               # vertical  distance knee axis -> foot
    tib = np.hypot(FOOT_Y, TIBIA)       # straight-line tibia length
    tib0 = np.arctan2(-TIBIA, FOOT_Y)   # tibia direction at claw = 0 (~straight down)

    D = np.hypot(u, v)                  # knee axis -> foot, straight line
    # Law of cosines: the triangle femur / tibia / D fixes the bend between them.
    cos_bend = (D**2 - FEMUR**2 - tib**2) / (2 * FEMUR * tib)
    if abs(cos_bend) > 1:
        raise ValueError(f"{leg}: foot out of reach (D = {D * 1000:.1f} mm)")
    bend = -np.arccos(cos_bend)         # tibia angle relative to femur. Negative = bends DOWN

    # Femur angle = direction to the foot, minus the angle the bent tibia adds.
    knee = np.arctan2(v, u) - np.arctan2(tib * np.sin(bend), FEMUR + tib * np.cos(bend))
    claw = bend - tib0                  # measure claw from its own zero (tibia straight down)

    # 5) Un-mirror: right legs turn the opposite way about the same axes.
    return np.array([sy * hip, sy * knee, sy * claw])


# ---------------------------------------------------------------- tests
def test_zero_pose():
    for leg in LEGS:
        q = leg_ik(leg, leg_fk(leg, [0, 0, 0]))
        assert np.allclose(q, 0, atol=1e-9), (leg, q)
    print("[ok] zero pose: IK(FK(0)) = 0 for all legs")


def foot_is_outward(leg, q):
    """True if the foot sticks OUT from the hip (always the case when walking).
    A foot folded back under the body has a second IK answer (hip turned ~180 deg),
    and leg_ik() deliberately only returns the normal one."""
    sx, sy = LEGS[leg]
    d = leg_fk(leg, q) - [sx * HIP_X, sy * HIP_Y, 0.0]
    return sy * (rot_z(-q[0]) @ d)[1] > 0.01      # >1 cm outward along the leg


def test_standf():
    """Your Standf: knee +15 and claw -15 cancel, so the tibia should hang vertical:
    foot straight below the claw axis (apart from the small built-in foot offset)."""
    for leg in LEGS:
        q = logical_to_joint(leg, STANDF)
        assert abs(q[1] + q[2]) < 1e-12, (leg, q)    # tibia angle = knee + claw = 0
    foot = leg_fk("fl", logical_to_joint("fl", STANDF))
    print(f"[ok] Standf: tibia vertical. FL foot at x {foot[0] * 1000:.1f}, y {foot[1] * 1000:.1f} mm, "
          f"hip axis {-foot[2] * 1000:.1f} mm above the floor")


def test_round_trip(n=2000):
    """Random angles -> FK -> IK must give the same angles back."""
    rng = np.random.default_rng(0)
    worst, tested = 0.0, 0
    for _ in range(n):
        for leg in LEGS:
            q = rng.uniform([-0.7, -0.9, -0.9], [0.7, 0.9, 0.9])
            if not foot_is_outward(leg, q):
                continue
            worst = max(worst, np.abs(leg_ik(leg, leg_fk(leg, q)) - q).max())
            tested += 1
    assert worst < 1e-9, worst
    print(f"[ok] round trip, {tested} random leg poses: worst angle error {np.degrees(worst):.1e} deg")


def test_against_mujoco(n=200):
    """Our FK vs MuJoCo's FK, on the real model. Catches wrong lengths or signs."""
    import mujoco
    import simrun
    m, d = simrun.load()
    rng = np.random.default_rng(1)
    worst = 0.0
    for _ in range(n):
        q12 = rng.uniform(-0.9, 0.9, 12)
        d.qpos[:7] = [0, 0, 0, 1, 0, 0, 0]       # body at the origin, not rotated
        d.qpos[7:] = q12
        mujoco.mj_kinematics(m, d)               # MuJoCo computes every body position
        for i, leg in enumerate(LEGS):
            ours = leg_fk(leg, q12[3 * i: 3 * i + 3])
            theirs = d.body(f"{leg}_foot").xpos
            worst = max(worst, np.abs(ours - theirs).max())
    # Our constants are rounded to 0.1 mm, the model file has more digits -> ~0.01 mm.
    assert worst < 5e-5, worst
    print(f"[ok] our FK vs MuJoCo FK, {n} random poses: worst error {worst * 1000:.1e} mm")


# ---------------------------------------------------------------- viewer demo
def demo():
    """Pure kinematics, no physics: hang the body in the air, the FL foot draws a circle."""
    import time
    import mujoco
    import mujoco.viewer
    import simrun
    m, d = simrun.load()
    centre = leg_fk("fl", logical_to_joint("fl", STANDF)) + [0, 0, 0.03]   # 3 cm above the Standf foot
    radius = 0.03
    with mujoco.viewer.launch_passive(m, d) as viewer:
        t0 = time.time()
        while viewer.is_running():
            t = time.time() - t0
            # Circle in the x-z plane = the shape of a step: forward, up, back, down
            target = centre + radius * np.array([np.cos(t * 2), 0, np.sin(t * 2)])
            d.qpos[:7] = [0, 0, 0.2, 1, 0, 0, 0]
            d.qpos[7:10] = leg_ik("fl", target)  # only FL moves, others stay at zero
            mujoco.mj_forward(m, d)              # update positions, no physics step
            simrun.mark(viewer, target + [0, 0, 0.2])   # red dot = where we asked the foot to be
            viewer.sync()
            time.sleep(1 / 60)


if __name__ == "__main__":
    test_zero_pose()
    test_standf()
    test_round_trip()
    test_against_mujoco()
    if "--headless" not in sys.argv:
        demo()
