"""
STEP 5: turning. Same crawl as step 4, but stance feet sweep along ARCS.

Step 4, seen from the body: a foot on the ground slides straight back,
because the body moves straight forward over it.

If the body also ROTATES, a foot that's still on the floor appears to swing
around the body centre the other way. So during stance each foot does both:

    slide back by STEP          (forward motion)
    rotate around the centre    (turning)

  u = 0 just landed ... u = 1 about to lift
  foot = rotate(turn * (0.5 - u)) @ stance_spot + [STEP * (0.5 - u), 0]

TURN = 0 gives exactly step 4. STEP = 0 with TURN != 0 spins on the spot.
The swing just carries the foot from the end of its arc back to the start.

Run:
  python sim_walk/step5_turn.py              viewer: forward, arc left, spin right (loops)
  python sim_walk/step5_turn.py --headless   one pass, prints commanded vs measured turning
"""
import sys
import numpy as np

import simrun
from step1_leg_ik import LEGS, rot_z
from step2_stand import HEIGHT, STANCE_BODY
from step3_body_shift import smooth
from step4_crawl import (LIFT, CYCLE, SWING, TUCK, SETTLE, RAMP,
                         leg_phase, body_sway, gait_amount, gait_phase)

# What to do: (seconds, stroke in m, turn per cycle in degrees, label)
COMMANDS = [
    (8.0, 0.06, 0, "forward"),
    (10.0, 0.04, 12, "arc left"),
    (10.0, 0.00, -15, "spin right on the spot"),
]
BLEND = 1.0      # seconds to blend from one command to the next (no sudden jumps)

# Standf spots, tucked in like step 4
SPOT = {leg: STANCE_BODY[leg] + [0, -sy * TUCK, 0] for leg, (sx, sy) in LEGS.items()}


def command(t):
    """(stroke, turn per cycle in rad) at time t, blended between segments."""
    t -= SETTLE + RAMP
    total = sum(c[0] for c in COMMANDS)
    t %= total
    for i, (dur, step, turn, _) in enumerate(COMMANDS):
        if t < dur:
            nxt = COMMANDS[(i + 1) % len(COMMANDS)]
            k = smooth((t - (dur - BLEND)) / BLEND)     # 0 until the last BLEND seconds
            return (step + (nxt[1] - step) * k,
                    np.radians(turn + (nxt[2] - turn) * k))
        t -= dur


def stance_spot(leg, u, step, turn):
    """Foot position (body frame) at stance progress u: 0 = just landed, 1 = lifting."""
    turn_stance = turn * (1 - SWING)            # how much the body turns while this foot is down
    p = rot_z(turn_stance * (0.5 - u)) @ SPOT[leg]
    return p + [step * (0.5 - u), 0, 0]


def controller(t):
    phi, r = gait_phase(t), gait_amount(t)
    step, turn = command(t) if t > SETTLE + RAMP else command(SETTLE + RAMP)
    step, turn = r * step, r * turn
    sway = r * body_sway(phi)
    feet = {}
    for leg in LEGS:
        p = leg_phase(leg, phi)
        if p < SWING:                            # in the air: end of the arc -> start
            k = p / SWING
            a, b = stance_spot(leg, 1.0, step, turn), stance_spot(leg, 0.0, step, turn)
            foot = a + (b - a) * smooth(k)
            foot[2] += r * LIFT * np.sin(np.pi * k) ** 2
        else:                                    # on the ground: follow the arc
            foot = stance_spot(leg, (p - SWING) / (1 - SWING), step, turn)
        feet[leg] = foot - [sway[0], sway[1], 0]
    return simrun.feet_to_angles(feet)


# ---------------------------------------------------------------- run
if __name__ == "__main__":
    headless = "--headless" in sys.argv
    m, d = simrun.load()
    simrun.place(m, d, controller(0.0), HEIGHT + 0.001)

    t0 = SETTLE + RAMP
    bounds = list(np.cumsum([c[0] for c in COMMANDS]) + t0)
    snaps = []
    log = {"tilt": 0.0, "torque": np.zeros(12), "yaw_prev": 0.0, "yaw": 0.0}

    def watch(m, d):
        y = simrun.base_yaw(d)                   # unwrap so +-180 deg doesn't jump
        log["yaw"] += (y - log["yaw_prev"] + np.pi) % (2 * np.pi) - np.pi
        log["yaw_prev"] = y
        if d.time < t0:
            return
        if not snaps or (len(snaps) <= len(bounds) and d.time >= bounds[len(snaps) - 1]):
            snaps.append((simrun.base_pos(d), log["yaw"]))
        log["tilt"] = max(log["tilt"], simrun.tilt_deg(d))
        log["torque"] = np.maximum(log["torque"], simrun.torque_pct(d))

    simrun.run(m, d, controller, bounds[-1] + 0.01, headless, on_step=watch)

    if headless:
        for i, (dur, step, turn, label) in enumerate(COMMANDS):
            (p0, y0), (p1, y1) = snaps[i], snaps[i + 1]
            print(f"{label:24s}: turned {np.degrees(y1 - y0):+6.1f} deg "
                  f"(asked ~{turn * dur / CYCLE:+4.0f}), moved {np.linalg.norm((p1 - p0)[:2]) * 100:4.1f} cm")
        tq = log["torque"].reshape(4, 3).max(axis=0)
        print(f"max tilt {log['tilt']:.1f} deg, peak torque hip/knee/claw: "
              f"{tq[0]:.0f}% / {tq[1]:.0f}% / {tq[2]:.0f}%")
