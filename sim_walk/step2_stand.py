"""
STEP 2: stand with IK, and move the BODY while the feet stay planted.

Idea: feet are fixed points on the floor. To move the body, don't think about
joints at all. Just ask: "if the body is HERE, where are the feet as seen from
the body?" Then let IK turn that into angles.

    foot_in_body = rotate(-body_yaw) @ (foot_on_floor - body_position)

Physics is ON from here: gravity, contacts, servo torque limits.

Run:
  python sim_walk/step2_stand.py              viewer: bob, sway, twist (loops)
  python sim_walk/step2_stand.py --headless   one loop, then prints a report
"""
import sys
import numpy as np

import simrun
from step1_leg_ik import LEGS, rot_z, leg_fk, logical_to_joint, STANDF

# The stance is YOUR Standf pose: run FK on it to find where the feet are.
# Femur up 15 deg, tibia vertical -> the claw servo carries almost no torque.
STANCE_BODY = {leg: leg_fk(leg, logical_to_joint(leg, STANDF)) for leg in LEGS}  # feet seen from the body
HEIGHT = -STANCE_BODY["fl"][2]      # hip axis height above the floor (74.5 mm)


def stance_feet():
    """Standf foot spots on the floor (z = 0), with the body centre at x = y = 0."""
    return {leg: np.array([p[0], p[1], 0.0]) for leg, p in STANCE_BODY.items()}


def body_ik(feet_world, body_xyz, body_yaw=0.0):
    """Feet on the floor + where the body is -> feet as seen from the body."""
    R_inv = rot_z(-body_yaw)
    return {leg: R_inv @ (p - body_xyz) for leg, p in feet_world.items()}


# ---------------------------------------------------------------- demo motion
SEG = 4.0                    # seconds per move
HOLD = 2.0                   # settle first
PERIOD = HOLD + 4 * SEG


def body_pose(t):
    """Scripted body pose (x, y, z, yaw) over time. Feet never move."""
    t = t % PERIOD
    x, y, z, yaw = 0.0, 0.0, HEIGHT, 0.0
    if t >= HOLD:
        k = int((t - HOLD) // SEG)                         # which move: 0..3
        wave = np.sin(2 * np.pi * ((t - HOLD) % SEG) / SEG)   # 0 -> +1 -> 0 -> -1 -> 0
        if k == 0:
            z += 0.02 * wave          # 1) up/down 2 cm
        elif k == 1:
            x += 0.02 * wave          # 2) forward/back 2 cm
        elif k == 2:
            y += 0.025 * wave         # 3) left/right 2.5 cm
        else:
            yaw += np.radians(15) * wave   # 4) twist +-15 deg
    return np.array([x, y, z]), yaw


FEET = stance_feet()


def controller(t):
    pos, yaw = body_pose(t)
    return simrun.feet_to_angles(body_ik(FEET, pos, yaw))


# ---------------------------------------------------------------- run
if __name__ == "__main__":
    headless = "--headless" in sys.argv
    m, d = simrun.load()
    simrun.place(m, d, controller(0.0), HEIGHT + 0.001)

    start_feet = {leg: simrun.foot_world(d, leg) for leg in LEGS}
    log = {"tilt": 0.0, "slip": 0.0, "torque": np.zeros(12), "zerr": 0.0}

    def watch(m, d):
        if d.time < 1.0:
            return                               # ignore the first settle
        log["tilt"] = max(log["tilt"], simrun.tilt_deg(d))
        for leg in LEGS:
            slip = np.linalg.norm((simrun.foot_world(d, leg) - start_feet[leg])[:2])
            log["slip"] = max(log["slip"], slip)
        log["torque"] = np.maximum(log["torque"], simrun.torque_pct(d))
        want_z = body_pose(d.time)[0][2]
        log["zerr"] = max(log["zerr"], abs(simrun.base_pos(d)[2] - want_z))

    simrun.run(m, d, controller, PERIOD, headless, on_step=watch)

    if headless:
        tq = log["torque"].reshape(4, 3).max(axis=0)
        print(f"max body tilt          : {log['tilt']:.1f} deg")
        print(f"max foot slip          : {log['slip'] * 1000:.1f} mm")
        print(f"max height error       : {log['zerr'] * 1000:.1f} mm (servo sag)")
        print(f"peak torque hip/knee/claw: {tq[0]:.0f}% / {tq[1]:.0f}% / {tq[2]:.0f}% of stall")
