"""MPU6050 helpers, and the servo-scale measurement that uses it.

Orientation is the same everywhere on a rigid body, so the IMU being off-centre
doesn't matter for tilt. We only ever use the ANGLE BETWEEN two gravity vectors,
so it also doesn't matter which way the chip is mounted.

Scale measurement (robot standing on a flat floor):
  1. front knees to -D (commanded), read gravity g1
  2. front knees to +D (commanded), read gravity g2
  3. measured pitch change = angle(g1, g2)
  4. IK predicts the pitch change for any TRUE knee swing -> invert it ->
     true swing. ticks_per_deg_true = ticks_per_deg_used * commanded / true.
"""
import numpy as np

from .kinematics import LEGS, leg_fk


def parse_imu(line):
    """'IMU,ax,ay,az,gx,gy,gz' -> (accel m/s^2 [3], gyro rad/s [3]) or None."""
    if not line.startswith("IMU,"):
        return None
    try:
        v = [float(x) for x in line.split(",")[1:7]]
    except ValueError:
        return None
    return np.array(v[:3]), np.array(v[3:6])


def angle_between_deg(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    c = np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))
    return float(np.degrees(np.arccos(np.clip(c, -1, 1))))


def tilt_from_gravity(g, g_level):
    """Tilt (deg) relative to the reference 'level' gravity vector."""
    return angle_between_deg(g, g_level)


def front_knee_pose(stance_deg, delta_deg):
    """Stance with both FRONT knees moved by delta (deg, '+ = raise femur' on both sides)."""
    q = np.array(stance_deg, float)
    for i, (leg, (sx, sy)) in enumerate(LEGS.items()):
        if sx > 0:
            q[3 * i + 1] += sy * delta_deg       # +knee raises a left femur, -knee a right one
    return q


def body_pitch_deg(q_deg):
    """Body pitch when standing on all four feet in pose q (small-angle plane fit)."""
    q = np.radians(q_deg)
    z, x = {}, {}
    for i, leg in enumerate(LEGS):
        p = leg_fk(leg, q[3 * i: 3 * i + 3])
        z[leg], x[leg] = p[2], p[0]
    dz = (z["fl"] + z["fr"]) / 2 - (z["bl"] + z["br"]) / 2
    dx = (x["fl"] + x["fr"]) / 2 - (x["bl"] + x["br"]) / 2
    return float(np.degrees(np.arctan2(dz, dx)))


def predicted_swing_pitch(stance_deg, true_delta_deg):
    """Pitch change going from front knees -d to +d (true joint degrees)."""
    return abs(body_pitch_deg(front_knee_pose(stance_deg, true_delta_deg))
               - body_pitch_deg(front_knee_pose(stance_deg, -true_delta_deg)))


def true_delta_from_pitch(stance_deg, measured_pitch_deg, max_delta=45.0):
    """Invert predicted_swing_pitch by bisection (it grows monotonically with d)."""
    lo, hi = 0.0, max_delta
    if measured_pitch_deg >= predicted_swing_pitch(stance_deg, hi):
        return hi
    for _ in range(60):
        mid = (lo + hi) / 2
        if predicted_swing_pitch(stance_deg, mid) < measured_pitch_deg:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def sim_swing_pitch(stance_deg, true_delta_deg, settle_s=2.0):
    """Same experiment in the physics sim. Includes servo sag, which the pure
    geometry above ignores (sag differs between knees-up and knees-down, so the
    geometric version overestimates the swing by ~14%)."""
    from .sim import SimWorld                    # local import: only needed here
    s = SimWorld()
    s.reset(stance_deg)
    g = []
    for d in (-true_delta_deg, true_delta_deg):
        q = front_knee_pose(stance_deg, d)
        for _ in range(int(settle_s / 0.02)):
            s.physics(q, 0.02)
        R = s.d.body(s.base).xmat.reshape(3, 3)
        g.append(R.T @ np.array([0, 0, 9.81]))   # gravity as the IMU would see it
    return angle_between_deg(*g)


def true_delta_from_pitch_sim(stance_deg, measured_pitch_deg, lo=1.0, hi=45.0, iters=12):
    """Invert sim_swing_pitch by bisection (~12 short sims, a few seconds)."""
    for _ in range(iters):
        mid = (lo + hi) / 2
        if sim_swing_pitch(stance_deg, mid) < measured_pitch_deg:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def corrected_scale(ticks_per_deg_used, commanded_delta, true_delta):
    return ticks_per_deg_used * commanded_delta / true_delta
