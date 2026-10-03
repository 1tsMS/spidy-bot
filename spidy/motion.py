"""Small motion helpers used between the GUI and the robot.

SlewLimiter: caps how fast any joint target can change (deg/s). Every target
             goes through it before reaching the sim or the real servos, so a
             dragged slider or a far-away pose never slams a joint.
PoseMove:    smooth timed move from one pose to another (smoothstep easing).
Sequence:    a list of (pose, ms) PoseMoves played back to back.
"""
import numpy as np


class SlewLimiter:
    def __init__(self, max_deg_per_s=200.0):
        self.max_rate = max_deg_per_s
        self.value = None

    def reset(self, q):
        self.value = np.array(q, float)

    def update(self, target, dt):
        target = np.asarray(target, float)
        if self.value is None:
            self.value = target.copy()
            return self.value.copy()
        step = self.max_rate * dt
        self.value += np.clip(target - self.value, -step, step)
        return self.value.copy()


class PoseMove:
    def __init__(self, start, end, seconds):
        self.a, self.b = np.asarray(start, float), np.asarray(end, float)
        self.T = max(seconds, 1e-3)
        self.t = 0.0

    @property
    def done(self):
        return self.t >= self.T

    def update(self, dt):
        self.t = min(self.t + dt, self.T)
        s = self.t / self.T
        e = s * s * (3 - 2 * s)                  # smoothstep: zero speed at both ends
        return self.a + (self.b - self.a) * e


class Sequence:
    def __init__(self, start, steps):
        """steps: list of (q_deg12, seconds)."""
        self.steps = list(steps)
        self.i = 0
        self.current = start
        self.move = PoseMove(start, self.steps[0][0], self.steps[0][1]) if self.steps else None

    @property
    def done(self):
        return self.move is None

    @property
    def label(self):
        return f"{min(self.i + 1, len(self.steps))}/{len(self.steps)}"

    def update(self, dt):
        if self.move is None:
            return self.current
        self.current = self.move.update(dt)
        if self.move.done:
            self.i += 1
            self.move = (PoseMove(self.current, *self.steps[self.i])
                         if self.i < len(self.steps) else None)
        return self.current
