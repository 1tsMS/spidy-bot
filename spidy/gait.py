"""Crawl (wave gait) driven by a live command. The maths is sim_walk/step4 + step5.

    gait = CrawlGait()
    gait.set_command(forward=1.0, turn=0.0)     # -1..1 each, change any time
    q_deg = gait.update(dt)                     # call every tick -> 12 joint angles (deg)

When the command goes to zero the gait fades out over RAMP seconds and ends in
the stance pose, so "stop" never leaves a leg in the air.
"""
from dataclasses import dataclass

import numpy as np

from .kinematics import LEGS, leg_fk, feet_to_angles, rot_z, standf_deg


def smooth(s):
    s = np.clip(s, 0.0, 1.0)
    return 0.5 - 0.5 * np.cos(np.pi * s)


@dataclass
class GaitParams:
    cycle: float = 2.4          # s for all four legs to step once
    swing: float = 0.15         # fraction of the cycle a foot is in the air
    lift: float = 0.025         # m, swing height
    max_step: float = 0.06      # m of body travel per stance at forward = 1
    max_turn: float = 15.0      # deg of body turn per cycle at turn = 1
    sway_x: float = 0.02        # m, fore-aft lean
    sway_y: float = 0.02        # m, sideways lean
    sway_sharp: float = 2.0     # how square the side sway is (see step4)
    tuck: float = 0.015         # m, feet pulled in from the stance pose while walking
    ramp: float = 2.4           # s to fade the gait in / out
    cmd_smooth: float = 0.4     # s, time constant for command changes


PHASE = {"br": 0.0, "fr": 0.25, "bl": 0.5, "fl": 0.75}


class CrawlGait:
    def __init__(self, stance_deg=None, params=None):
        self.p = params or GaitParams()
        self.set_stance(standf_deg() if stance_deg is None else stance_deg)
        self.phase = 0.0          # 0..1 through the cycle
        self.amount = 0.0         # 0 = standing, 1 = full gait
        self.cmd = np.zeros(2)    # target (forward, turn), -1..1
        self.cmd_s = np.zeros(2)  # smoothed command actually used

    def set_stance(self, stance_deg):
        """Stance pose (12 joint degrees) -> foot spots the gait walks around."""
        self.stance_deg = np.asarray(stance_deg, float)
        q = np.radians(self.stance_deg)
        self.stance_feet = {leg: leg_fk(leg, q[3 * i: 3 * i + 3]) for i, leg in enumerate(LEGS)}

    def set_command(self, forward, turn):
        self.cmd = np.clip([forward, turn], -1, 1)

    @property
    def walking(self):
        return self.amount > 0

    def update(self, dt):
        p = self.p
        k = min(1.0, dt / p.cmd_smooth)
        self.cmd_s += (self.cmd - self.cmd_s) * k

        want = np.abs(self.cmd).max() > 0.02
        self.amount = float(np.clip(self.amount + (dt if want else -dt) / p.ramp, 0, 1))
        if self.amount == 0:
            self.phase = 0.0
            self.cmd_s[:] = 0
            return self.stance_deg.copy()
        self.phase = (self.phase + dt / p.cycle) % 1.0
        return np.degrees(feet_to_angles(self.feet(self.phase, self.amount, *self.cmd_s)))

    # ---- the gait itself: pure function of (phase, amount, command)
    def feet(self, phi, r, forward, turn):
        p = self.p
        step = r * forward * p.max_step
        turn_st = np.radians(r * turn * p.max_turn) * (1 - p.swing)   # body turn per stance
        sway = r * np.array([p.sway_x * np.cos(4 * np.pi * (phi - 0.1)),
                             p.sway_y * np.clip(p.sway_sharp * np.sin(2 * np.pi * (phi + 0.05)), -1, 1)])
        out = {}
        for leg, (sx, sy) in LEGS.items():
            spot = self.stance_feet[leg] + [0, -sy * r * p.tuck, 0]

            def on_ground(u):          # u: 0 just landed .. 1 lifting
                q = rot_z(turn_st * (0.5 - u)) @ spot
                return q + [step * (0.5 - u), 0, 0]

            lp = (phi - PHASE[leg]) % 1.0
            if lp < p.swing:
                s = lp / p.swing
                a, b = on_ground(1.0), on_ground(0.0)
                f = a + (b - a) * smooth(s)
                f[2] += r * p.lift * np.sin(np.pi * s) ** 2
            else:
                f = on_ground((lp - p.swing) / (1 - p.swing))
            out[leg] = f - [sway[0], sway[1], 0]
        return out
