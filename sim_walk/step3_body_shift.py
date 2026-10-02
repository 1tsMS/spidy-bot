"""
STEP 3: lift one leg without falling over. Shift the body first.

Static stability rule: the robot doesn't tip as long as its centre of mass (COM),
projected straight down, is inside the SUPPORT POLYGON (the shape the feet on
the ground make). Lift one of 4 legs and the polygon shrinks to a triangle.
So BEFORE lifting a leg, slide the body until the COM is over the triangle
of the other three.

STABILITY MARGIN = distance from the COM to the nearest triangle edge.
Positive = inside (safe), negative = outside (tipping).

Run:
  python sim_walk/step3_body_shift.py               viewer: lifts each leg in turn (loops)
  python sim_walk/step3_body_shift.py --no-shift    same, but WITHOUT shifting. Watch it tip.
  python sim_walk/step3_body_shift.py --headless    prints the margin for every lift
"""
import sys
import numpy as np

import simrun
from step1_leg_ik import LEGS
from step2_stand import HEIGHT, stance_feet, body_ik

LIFT = 0.03          # how high the lifted foot goes
COM_AHEAD = 0.0089   # the COM sits 8.9 mm ahead of the body centre (battery + buck up front)
ORDER = ["fl", "br", "fr", "bl"]


def triangle_margin(p, tri):
    """Signed distance from point p to the nearest edge of triangle tri (3 x 2D points).
    Positive = inside."""
    centre = np.mean(tri, axis=0)
    dists = []
    for i in range(3):
        a, b = tri[i], tri[(i + 1) % 3]
        edge = b - a
        normal = np.array([-edge[1], edge[0]]) / np.linalg.norm(edge)   # perpendicular to the edge
        if np.dot(centre - a, normal) < 0:
            normal = -normal                     # make it point INTO the triangle
        dists.append(np.dot(p - a, normal))
    return min(dists)


def shift_target(feet, lifted, amount=1.0, yaw=0.0):
    """Where the body centre should go before lifting `lifted`.
    amount = 1.0: COM goes all the way to the middle (centroid) of the triangle
                  made by the other three feet. Most stable.
    amount < 1.0: lean only part of the way there. Less stable, but the body
                  doesn't squash the loaded legs as much (see step 4).
    yaw: body heading. "Ahead" turns with the body (used in step 5)."""
    tri = [feet[leg][:2] for leg in LEGS if leg != lifted]
    centroid = np.mean(tri, axis=0)
    middle = np.mean([p[:2] for p in feet.values()], axis=0)   # centre of all 4 feet
    target_com = middle + amount * (centroid - middle)
    ahead = COM_AHEAD * np.array([np.cos(yaw), np.sin(yaw)])
    return target_com - ahead                    # COM is ahead of the centre, so aim the centre behind


def smooth(s):
    """0 -> 1 with zero speed at both ends (no jerks)."""
    s = np.clip(s, 0, 1)
    return 0.5 - 0.5 * np.cos(np.pi * s)


# ---------------------------------------------------------------- timeline
# Per leg: shift (1 s) -> lift (0.6 s) -> hold (0.8 s) -> lower (0.6 s) -> shift back (1 s)
T_SHIFT, T_LIFT, T_HOLD = 1.0, 0.6, 0.8
T_LEG = 2 * T_SHIFT + 2 * T_LIFT + T_HOLD
SETTLE = 1.0
FEET = stance_feet()
DO_SHIFT = "--no-shift" not in sys.argv


def plan(t):
    """-> (body xy, which leg is lifted, how high). Pure function of time."""
    t = (t - SETTLE) % (T_LEG * 4) if t > SETTLE else -1
    if t < 0:
        return np.zeros(2), None, 0.0
    leg = ORDER[int(t // T_LEG)]
    s = t % T_LEG
    target = shift_target(FEET, leg) if DO_SHIFT else np.zeros(2)

    # Phase boundaries
    a = T_SHIFT                  # end of shift
    b = a + T_LIFT               # foot fully up
    c = b + T_HOLD               # start lowering
    e = c + T_LIFT               # foot down
    if s < a:
        body, h = target * smooth(s / T_SHIFT), 0.0
    elif s < b:
        body, h = target, LIFT * smooth((s - a) / T_LIFT)
    elif s < c:
        body, h = target, LIFT
    elif s < e:
        body, h = target, LIFT * (1 - smooth((s - c) / T_LIFT))
    else:
        body, h = target * (1 - smooth((s - e) / T_SHIFT)), 0.0
    return body, leg, h


def controller(t):
    body_xy, leg, h = plan(t)
    feet = {k: p.copy() for k, p in FEET.items()}
    if leg:
        feet[leg][2] += h                        # raise the swing foot
    body = np.array([body_xy[0], body_xy[1], HEIGHT])
    return simrun.feet_to_angles(body_ik(feet, body))


# ---------------------------------------------------------------- run
if __name__ == "__main__":
    headless = "--headless" in sys.argv
    m, d = simrun.load()
    simrun.place(m, d, controller(0.0), HEIGHT + 0.001)

    # Track the worst real margin and tilt while each leg is fully up.
    worst = {leg: [np.inf, 0.0] for leg in LEGS}

    def watch(m, d):
        _, leg, h = plan(d.time)
        if leg and h >= LIFT * 0.99:
            tri = [simrun.foot_world(d, k)[:2] for k in LEGS if k != leg]
            margin = triangle_margin(simrun.com_xy(d), tri)
            worst[leg][0] = min(worst[leg][0], margin)
            worst[leg][1] = max(worst[leg][1], simrun.tilt_deg(d))

    simrun.run(m, d, controller, SETTLE + 4 * T_LEG, headless, on_step=watch)

    if headless:
        print(f"body shift: {'ON' if DO_SHIFT else 'OFF'}")
        for leg in ORDER:
            mg, tl = worst[leg]
            state = "stable" if mg > 0 else "TIPPING"
            print(f"  lift {leg}: min margin {mg * 1000:+5.1f} mm, max tilt {tl:4.1f} deg  -> {state}")
