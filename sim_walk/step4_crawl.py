"""
STEP 4: crawl (wave gait). The body moves CONTINUOUSLY; one leg swings at a time.

How a crawl works, seen from the body:
  - A foot on the ground slides BACKWARD at a steady speed (the "stroke").
    It isn't really sliding: the foot is still and the body is moving forward over it.
  - When it reaches the back of its stroke, it lifts, swings forward through
    the air and lands at the front again.
  - Every leg does this, each a quarter cycle after the previous one.

  phase of a leg:  0 -------- SWING ---------------------------------- 1
                   [ in the air ][ on the ground, sliding back          ]

Leg order BR -> FR -> BL -> FL: back then front on the same side.

Staying balanced (step 3's lesson) without stopping: the body SWAYS.
  - Sideways: lean left while the right legs take their turns, then lean right.
    Shaped like a rounded square wave: hold the lean, cross over quickly.
  - Fore-aft: lean forward while a back leg is up, back while a front leg is up.
The sway shape was tuned on paper first (stability margin over a whole cycle).

Run:
  python sim_walk/step4_crawl.py              viewer: walks until you close it
  python sim_walk/step4_crawl.py --headless   walks 30 s, prints a report
"""
import sys
import numpy as np

import simrun
from step1_leg_ik import LEGS
from step2_stand import HEIGHT, STANCE_BODY
from step3_body_shift import smooth, triangle_margin

STEP = 0.06         # stroke: how far the body travels while a foot is on the ground (m)
LIFT = 0.025        # swing height (m)
CYCLE = 2.4         # seconds for all 4 legs to step once
SWING = 0.15        # fraction of the cycle each foot spends in the air
PHASE = {"br": 0.0, "fr": 0.25, "bl": 0.5, "fl": 0.75}   # when each leg starts its swing

SWAY_Y = 0.02       # sideways lean (m)
SWAY_X = 0.02       # fore-aft lean (m)
SWAY_SHARP = 2      # how square the sideways sway is. Higher = crosses over faster.
# 4 (sharper) gave more margin, but swinging 4 cm of body in ~0.2 s pushes ~2 N
# sideways on the feet; on the 89 mm tibia that's the claw servo's full stall torque.
TUCK = 0.015        # pull the feet 15 mm in from Standf while walking.
# In Standf the femur is nearly flat, so the knee's lever arm is at its longest.
# On 3 legs that put the knees at 100%. Tucking in shortens that lever (knee ~87%)
# but tilts the tibia, loading the claw (~74%). 15 mm balances the two.
SETTLE = 1.0        # stand still first
RAMP = 2.4          # then fade the gait in over one full cycle (no sudden jumps)


def leg_phase(leg, phi):
    """Where this leg is in its own cycle, 0..1."""
    return (phi - PHASE[leg]) % 1.0


def foot_offset(p, step):
    """Leg phase p -> (dx, dz) of the foot relative to its Standf spot (body frame)."""
    if p < SWING:                              # in the air: back -> front, along an arc
        k = p / SWING
        dx = -step / 2 + step * smooth(k)
        dz = LIFT * np.sin(np.pi * k) ** 2     # sin^2: zero vertical speed at liftoff/touchdown
    else:                                      # on the ground: front -> back, steady speed
        k = (p - SWING) / (1 - SWING)
        dx = step / 2 - step * k
        dz = 0.0
    return dx, dz


def body_sway(phi):
    """Body offset (x, y) at cycle phase phi."""
    y = SWAY_Y * np.clip(SWAY_SHARP * np.sin(2 * np.pi * (phi + 0.05)), -1, 1)   # rounded square wave
    x = SWAY_X * np.cos(4 * np.pi * (phi - 0.1))                       # twice per cycle
    return np.array([x, y])


def gait_amount(t):
    """0 while settling, smoothly up to 1. Scales stroke, lift and sway."""
    return smooth((t - SETTLE) / RAMP)


def gait_phase(t):
    return (max(t - SETTLE, 0.0) / CYCLE) % 1.0


def controller(t):
    phi, r = gait_phase(t), gait_amount(t)
    sway = r * body_sway(phi)
    feet = {}
    for leg in LEGS:
        dx, dz = foot_offset(leg_phase(leg, phi), r * STEP)
        # Body moves by `sway`, so the feet (seen from the body) move the opposite way
        sy = LEGS[leg][1]
        feet[leg] = STANCE_BODY[leg] + [dx - sway[0], -sway[1] - sy * r * TUCK, r * dz]
    return simrun.feet_to_angles(feet)


def swinging_leg(t):
    for leg in LEGS:
        if leg_phase(leg, gait_phase(t)) < SWING:
            return leg
    return None


# ---------------------------------------------------------------- run
if __name__ == "__main__":
    headless = "--headless" in sys.argv
    DURATION = 30.0
    m, d = simrun.load()
    simrun.place(m, d, controller(0.0), HEIGHT + 0.001)

    log = {"tilt": 0.0, "margin": np.inf, "torque": np.zeros(12), "start": None}
    t_walk = SETTLE + RAMP

    def watch(m, d):
        if d.time < t_walk:
            return
        if log["start"] is None:
            log["start"] = simrun.base_pos(d)
        log["tilt"] = max(log["tilt"], simrun.tilt_deg(d))
        log["torque"] = np.maximum(log["torque"], simrun.torque_pct(d))
        leg = swinging_leg(d.time)
        if leg:
            tri = [simrun.foot_world(d, k)[:2] for k in LEGS if k != leg]
            log["margin"] = min(log["margin"], triangle_margin(simrun.com_xy(d), tri))

    simrun.run(m, d, controller, DURATION, headless, on_step=watch)

    if headless:
        moved = simrun.base_pos(d) - log["start"]
        secs = DURATION - t_walk
        planned = STEP / ((1 - SWING) * CYCLE) * secs
        tq = log["torque"].reshape(4, 3).max(axis=0)
        print(f"walked forward : {moved[0] * 100:.1f} cm in {secs:.1f} s "
              f"({moved[0] / secs * 100:.2f} cm/s), planned {planned * 100:.1f} cm")
        print(f"sideways drift : {moved[1] * 100:+.1f} cm")
        print(f"heading drift  : {np.degrees(simrun.base_yaw(d)):+.1f} deg")
        print(f"max tilt       : {log['tilt']:.1f} deg")
        print(f"min margin     : {log['margin'] * 1000:.1f} mm (while a foot is up)")
        print(f"peak torque hip/knee/claw: {tq[0]:.0f}% / {tq[1]:.0f}% / {tq[2]:.0f}%")
