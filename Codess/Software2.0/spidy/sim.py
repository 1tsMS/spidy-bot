"""MuJoCo wrapper used by the app.

Two ways to show the robot:
  physics(q_deg, dt)   real simulation: gravity, contacts, servo torque limits.
  mirror(q_deg)        no physics: body held level in the air, joints set exactly.
                       Used to show what is being sent to the REAL robot.
render() gives an RGB image (numpy) for the GUI. Must be called from one thread.
"""
import os

import numpy as np
import mujoco

from .kinematics import angles_to_feet, JOINT_NAMES

# This file: <repo>/Codess/Software2.0/spidy/sim.py. The robot model lives at <repo>/URDF.
REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
MODEL = os.path.join(REPO, "URDF", "spidy_description", "mujoco", "spidy.xml")
STALL = 0.21            # N*m, MG90S


class SimWorld:
    def __init__(self, model_path=MODEL):
        self.m = mujoco.MjModel.from_xml_path(model_path)
        self.m.vis.global_.offwidth = 1920       # allow big offscreen renders
        self.m.vis.global_.offheight = 1200
        self.d = mujoco.MjData(self.m)
        self.base = self.m.body("base_link").id
        self._renderer = None
        self._size = None
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        self.cam.trackbodyid = self.base
        self.cam.distance, self.cam.azimuth, self.cam.elevation = 0.75, 135, -25
        self._rgba0 = self.m.geom_rgba.copy()
        self._mat0 = self.m.geom_matid.copy()

    # ---- state
    def reset(self, q_deg):
        """Place the robot standing on the floor in pose q_deg, nothing moving."""
        mujoco.mj_resetData(self.m, self.d)
        q = np.radians(q_deg)
        feet = angles_to_feet(q)
        height = -min(p[2] for p in feet.values()) + 0.001
        self.d.qpos[:7] = [0, 0, height, 1, 0, 0, 0]
        self.d.qpos[7:] = q
        self.d.ctrl[:] = q
        mujoco.mj_forward(self.m, self.d)

    def physics(self, q_deg, dt):
        """Step the physics for dt seconds with the servos aiming at q_deg."""
        self.d.ctrl[:] = np.radians(q_deg)
        for _ in range(max(1, int(round(dt / self.m.opt.timestep)))):
            mujoco.mj_step(self.m, self.d)

    def mirror(self, q_deg, height=None):
        """Kinematic only: body level, joints exactly at q_deg. Default height puts
        the lowest foot on the floor (the real robot has no position feedback,
        so this shows what we SEND, not what it does)."""
        if height is None:
            feet = angles_to_feet(np.radians(q_deg))
            height = max(-min(p[2] for p in feet.values()), 0.03) + 0.001
        self.d.qpos[:7] = [0, 0, height, 1, 0, 0, 0]
        self.d.qpos[7:] = np.radians(q_deg)
        self.d.qvel[:] = 0
        mujoco.mj_forward(self.m, self.d)

    def fallen(self):
        return self.tilt_deg() > 60 or self.d.body(self.base).xpos[2] < 0.03

    # ---- measurements
    def torque_pct(self):
        return 100 * np.abs(self.d.actuator_force) / STALL

    def joint_deg(self):
        return np.degrees(self.d.qpos[7:]).copy()

    def tilt_deg(self):
        R = self.d.body(self.base).xmat.reshape(3, 3)
        return float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))

    def base_pos(self):
        return self.d.body(self.base).xpos.copy()

    # ---- looks
    def highlight(self, joint_name=None, rgba=(1.0, 0.42, 0.1, 1.0)):
        """Colour everything a joint moves (its body and all children). None = reset."""
        self.m.geom_rgba[:] = self._rgba0
        self.m.geom_matid[:] = self._mat0
        if joint_name is None:
            return
        root = self.m.jnt_bodyid[self.m.joint(joint_name).id]
        moving = {root}
        for b in range(self.m.nbody):            # parents always come before children
            if self.m.body_parentid[b] in moving:
                moving.add(b)
        for g in range(self.m.ngeom):
            if self.m.geom_bodyid[g] in moving and self.m.geom_contype[g] == 0:   # visuals only
                self.m.geom_matid[g] = -1
                self.m.geom_rgba[g] = rgba

    def render(self, width, height, camera=None):
        """camera: an mujoco.MjvCamera, default self.cam (lets several views look differently)."""
        width, height = int(min(width, 1920)), int(min(height, 1200))
        if self._renderer is None or self._size != (width, height):
            if self._renderer is not None:
                self._renderer.close()
            self._renderer = mujoco.Renderer(self.m, height, width)
            self._size = (width, height)
            # Shadows alone cost ~17 of ~30 ms per frame and starved the 50 Hz control
            # loop in the app. Without shadows/reflection/skybox a frame is ~9 ms.
            fl = self._renderer.scene.flags
            fl[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            fl[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
            fl[mujoco.mjtRndFlag.mjRND_SKYBOX] = False
        self._renderer.update_scene(self.d, camera=camera or self.cam)
        return self._renderer.render()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None


def joint_index(name):
    return JOINT_NAMES.index(name)
