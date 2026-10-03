"""Pose import + link tests (the link talks to a fake robot on localhost)."""
import os
import socket
import threading
import time
import unittest

import numpy as np

from spidy import kinematics as K
from spidy.poses import from_legacy_poses, PoseLibrary
from spidy.link import Link, Pinger
from spidy.gait import CrawlGait
from spidy.motion import SlewLimiter, Sequence

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OLD = os.path.join(ROOT, "Codess", "Software")


class TestPoses(unittest.TestCase):
    def test_legacy_standf_matches_ik_standf(self):
        lib = from_legacy_poses(os.path.join(OLD, "spidy_poses.txt"),
                                os.path.join(OLD, "spidy_calibration3.txt"))
        np.testing.assert_allclose(lib.get("Standf"), K.standf_deg(), atol=1e-9)
        self.assertIn("Sit to Stand", lib.sequences)
        self.assertEqual(lib.sequences["Sit to Stand"][0][0], "Sit")

    def test_save_load(self, path="_poses_test.json"):
        lib = PoseLibrary()
        lib.put("A", range(12))
        lib.sequences["S"] = [("A", 500)]
        try:
            lib.save(path)
            back = PoseLibrary.load(path)
            self.assertEqual(back.poses, lib.poses)
            self.assertEqual(back.sequences, lib.sequences)
        finally:
            os.remove(path)


class FakeRobot(threading.Thread):
    """Answers like the firmware: PING->PONG, M->M_OK, records P16 lines."""
    def __init__(self):
        super().__init__(daemon=True)
        self.srv = socket.socket()
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(1)
        self.port = self.srv.getsockname()[1]
        self.p16 = []

    def run(self):
        c, _ = self.srv.accept()
        c.sendall(b"READY\n")
        buf = b""
        while True:
            data = c.recv(1024)
            if not data:
                return
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.decode()
                if line.startswith("PING,"):
                    c.sendall(f"PONG,{line[5:]}\n".encode())
                elif line.startswith("M,"):
                    c.sendall(f"M_OK,{line[2:]}\n".encode())
                elif line.startswith("P16,"):
                    self.p16.append(line)


class TestLink(unittest.TestCase):
    def test_round_trip(self):
        bot = FakeRobot()
        bot.start()
        got = []
        link = Link(on_line=got.append)
        link.open_tcp("127.0.0.1", bot.port)
        ping = Pinger()
        link.send(ping.make())
        link.send("M,3,400")
        link.send_pulses([400] * 12 + [0] * 4)
        deadline = time.time() + 2
        while time.time() < deadline and not (len(got) >= 3 and bot.p16):
            time.sleep(0.01)
        for line in got:
            ping.feed(line)
        link.close()
        self.assertIn("READY", got)
        self.assertIn("M_OK,3,400", got)
        self.assertIsNotNone(ping.last_ms)
        self.assertEqual(bot.p16[0], "P16," + ",".join(["400"] * 12 + ["0"] * 4))


class TestMotion(unittest.TestCase):
    def test_slew_limits_rate(self):
        s = SlewLimiter(100)
        s.reset(np.zeros(12))
        out = s.update(np.full(12, 90.0), 0.1)
        np.testing.assert_allclose(out, 10.0)

    def test_sequence_ends_on_last_pose(self):
        seq = Sequence(np.zeros(12), [(np.ones(12), 0.2), (np.full(12, 2.0), 0.2)])
        q = None
        for _ in range(30):
            q = seq.update(0.02)
        self.assertTrue(seq.done)
        np.testing.assert_allclose(q, 2.0)

    def test_gait_stops_in_stance(self):
        g = CrawlGait()
        g.set_command(1, 0)
        for _ in range(200):
            g.update(0.02)
        g.set_command(0, 0)
        for _ in range(200):
            q = g.update(0.02)
        self.assertFalse(g.walking)
        np.testing.assert_allclose(q, K.standf_deg())


if __name__ == "__main__":
    unittest.main()
