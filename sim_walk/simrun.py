"""
Shared plumbing for the step files: load the model, run the sim loop, measure things.
No walking ideas live here. Those are in the step files.
"""
import os
import time
import numpy as np
import mujoco
import mujoco.viewer

from step1_leg_ik import LEGS, leg_ik

MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "URDF", "spidy_description", "mujoco", "spidy.xml")


def load():
    m = mujoco.MjModel.from_xml_path(MODEL)
    return m, mujoco.MjData(m)


def feet_to_angles(feet_body):
    """{leg: foot xyz in body frame} -> 12 joint angles in actuator order."""
    return np.concatenate([leg_ik(leg, feet_body[leg]) for leg in LEGS])


def place(m, d, angles, height):
    """Teleport the robot: body at `height`, level, joints at `angles`, nothing moving."""
    mujoco.mj_resetData(m, d)
    d.qpos[:7] = [0, 0, height, 1, 0, 0, 0]   # x y z + quaternion (w x y z)
    d.qpos[7:] = angles
    d.ctrl[:] = angles
    mujoco.mj_forward(m, d)


def run(m, d, controller, duration, headless, on_step=None):
    """Every physics step (2 ms): d.ctrl = controller(time), then step the physics.
    Headless: run `duration` seconds as fast as possible.
    Viewer: run in real time until you close the window."""
    def step():
        d.ctrl[:] = controller(d.time)
        mujoco.mj_step(m, d)
        if on_step:
            on_step(m, d)

    if headless:
        while d.time < duration:
            step()
        return

    with mujoco.viewer.launch_passive(m, d) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING   # camera follows the body
        viewer.cam.trackbodyid = m.body("base_link").id
        viewer.cam.distance = 0.7
        viewer.cam.elevation = -25
        start = time.time()
        while viewer.is_running():
            while d.time < time.time() - start:   # catch the sim up to the wall clock
                step()
            viewer.sync()
            time.sleep(0.005)


def mark(viewer, pos, rgba=(1, 0, 0, 1), size=0.006):
    """Draw one coloured dot in the viewer (debug marker, no physics)."""
    viewer.user_scn.ngeom = 1
    mujoco.mjv_initGeom(viewer.user_scn.geoms[0], mujoco.mjtGeom.mjGEOM_SPHERE,
                        [size, 0, 0], np.asarray(pos, float), np.eye(3).flatten(),
                        np.asarray(rgba, np.float32))


# ---------------------------------------------------------------- measurements
def base_pos(d):
    return d.body("base_link").xpos.copy()


def base_yaw(d):
    R = d.body("base_link").xmat.reshape(3, 3)
    return np.arctan2(R[1, 0], R[0, 0])


def tilt_deg(d):
    """Angle between the body's 'up' and the world's 'up'. 0 = perfectly level."""
    R = d.body("base_link").xmat.reshape(3, 3)
    return np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))


def com_xy(d):
    """Whole-robot centre of mass, projected on the floor."""
    return d.subtree_com[d.body("base_link").id][:2].copy()


def foot_world(d, leg):
    return d.body(f"{leg}_foot").xpos.copy()


def torque_pct(d):
    """Each servo's torque as % of the 0.21 N*m stall limit."""
    return 100 * np.abs(d.actuator_force) / 0.21
