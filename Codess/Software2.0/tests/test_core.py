"""Core tests. Run from the repo root:  python -m unittest discover -s tests -v"""
import os
import unittest

import numpy as np

from spidy import kinematics as K
from spidy import calibration as C

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LEGACY = os.path.join(ROOT, "..", "Software", "spidy_calibration3.txt")
from spidy.sim import MODEL


class TestKinematics(unittest.TestCase):
    def test_round_trip(self):
        rng = np.random.default_rng(0)
        tested = 0
        for _ in range(500):
            for leg, (sx, sy) in K.LEGS.items():
                q = rng.uniform([-0.6, -0.6, -0.6], [0.6, 0.6, 0.6])
                # skip feet folded back under the hip: IK has a 2nd answer there by design
                d = K.leg_fk(leg, q) - [sx * K.HIP_X, sy * K.HIP_Y, 0]
                if sy * (K.rot_z(-q[0]) @ d)[1] < 0.01:
                    continue
                np.testing.assert_allclose(K.leg_ik(leg, K.leg_fk(leg, q)), q, atol=1e-9)
                tested += 1
        self.assertGreater(tested, 1500)

    def test_matches_mujoco(self):
        import mujoco
        m = mujoco.MjModel.from_xml_path(MODEL)
        d = mujoco.MjData(m)
        rng = np.random.default_rng(1)
        for _ in range(50):
            q = rng.uniform(-0.8, 0.8, 12)
            d.qpos[:7] = [0, 0, 0, 1, 0, 0, 0]
            d.qpos[7:] = q
            mujoco.mj_kinematics(m, d)
            for leg, p in K.angles_to_feet(q).items():
                np.testing.assert_allclose(p, d.body(f"{leg}_foot").xpos, atol=5e-5)

    def test_standf_tibia_vertical(self):
        q = np.radians(K.standf_deg()).reshape(4, 3)
        np.testing.assert_allclose(q[:, 1] + q[:, 2], 0, atol=1e-12)


class TestCalibration(unittest.TestCase):
    def setUp(self):
        self.cal = C.from_legacy_txt(LEGACY)

    def test_legacy_import_reproduces_old_pulses(self):
        """Every joint, every logical angle 0..180: new model == old tool's formula."""
        with open(LEGACY) as f:
            rows = [ln.split("\t") for ln in f if ln.strip()][1:]
        checked = 0
        for r in rows:
            if r[2].startswith("AUX"):
                continue
            leg, part = r[2].split("_")
            leg = leg.lower()
            j = ("HIP", "KNEE", "ANKLE").index(part)
            jc = self.cal.joints[f"{leg}_{K.JOINTS[j]}"]
            old_dir = 1 if r[3] == "+" else -1
            for L in range(0, 181):
                old = C.legacy_pulse(old_dir, int(r[4]), int(r[6]), int(r[7]), L)
                deg = K.logical_to_joint_deg(leg, (L, L, L))[j]
                self.assertLessEqual(abs(jc.to_ticks(deg) - old), 0.5 + 1e-9, (r[2], L))
                checked += 1
        self.assertEqual(checked, 12 * 181)

    def test_channels_from_file(self):
        self.assertEqual(self.cal.joints["bl_hip"].channel, 0)
        self.assertEqual(self.cal.joints["fr_hip"].channel, 11)
        self.assertEqual(self.cal.joints["br_claw"].channel, 13)
        self.assertEqual(self.cal.problems(), [])

    def test_round_trip_json(self, tmp="_cal_test.json"):
        try:
            self.cal.save(tmp)
            back = C.Calibration.load(tmp)
            self.assertEqual(back.to_dict(), self.cal.to_dict())
        finally:
            if os.path.exists(tmp):
                os.remove(tmp)

    def test_p16_layout(self):
        p = self.cal.channel_pulses([0.0] * 12)
        used = [i for i, v in enumerate(p) if v]
        self.assertEqual(sorted(used), sorted(j.channel for j in self.cal.joints.values()))
        for n, j in self.cal.joints.items():
            self.assertEqual(p[j.channel], int(round(j.center)))

    def test_clamps(self):
        j = C.JointCal(channel=0, center=400, direction=1, ticks_per_deg=3, min_deg=-10, max_deg=20)
        self.assertEqual(j.to_ticks(100), 460)      # clamped to +20 deg
        self.assertEqual(j.to_ticks(-100), 370)     # clamped to -10 deg
        j2 = C.JointCal(channel=0, center=650, ticks_per_deg=3, max_deg=90)
        self.assertEqual(j2.to_ticks(90), C.HARD_MAX)   # hard limit wins


if __name__ == "__main__":
    unittest.main()
