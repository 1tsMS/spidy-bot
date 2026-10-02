#!/usr/bin/env python3
"""
Sanity checks for the Spidy model. Produces images in mujoco/checks/.

  1. joint_axes.png : FL leg, each joint moved +0.5 rad. Confirms axis + sign.
  2. stand_test     : drop the robot on the floor holding zero pose, then
                      print how hard every servo has to work (torque audit v0).

Run:  MUJOCO_GL=egl python scripts/check_model.py      (osmesa on headless Linux)
"""
import os
import numpy as np
import mujoco
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
OUT = os.path.join(PKG, "mujoco", "checks")
os.makedirs(OUT, exist_ok=True)

m = mujoco.MjModel.from_xml_path(os.path.join(PKG, "mujoco", "spidy.xml"))
d = mujoco.MjData(m)
R = mujoco.Renderer(m, 480, 640)
TORQUE_LIMIT = float(m.actuator_forcerange[0, 1])


def cam(azim=135, elev=-25, dist=0.55, look=(0, 0, 0.06)):
    c = mujoco.MjvCamera()
    c.lookat[:] = look; c.azimuth = azim; c.elevation = elev; c.distance = dist
    return c


def snap(c, label):
    R.update_scene(d, camera=c)
    img = Image.fromarray(R.render())
    ImageDraw.Draw(img).text((10, 10), label, fill=(255, 255, 255))
    return img


def set_pose(q_joints):
    d.qpos[:] = 0
    d.qpos[2] = 0.0909                 # base height, feet on floor
    d.qpos[3] = 1.0                    # identity quaternion
    for name, val in q_joints.items():
        d.qpos[m.joint(name).qposadr[0]] = val
    mujoco.mj_forward(m, d)


# ---------------------------------------------------------------- 1. axes
tiles = []
for label, q in [("zero pose", {}),
                 ("fl_hip_yaw +0.5", {"fl_hip_yaw": 0.5}),
                 ("fl_hip_pitch +0.5", {"fl_hip_pitch": 0.5}),
                 ("fl_knee +0.5", {"fl_knee": 0.5}),
                 ("fr_hip_pitch +0.5 (mirror!)", {"fr_hip_pitch": 0.5}),
                 ("fr_knee +0.5 (mirror!)", {"fr_knee": 0.5})]:
    set_pose(q)
    tiles.append(snap(cam(azim=150, elev=-20, dist=0.5), label))
W, H = tiles[0].size
grid = Image.new("RGB", (W * 3, H * 2), "white")
for i, t in enumerate(tiles):
    grid.paste(t, ((i % 3) * W, (i // 3) * H))
grid.save(os.path.join(OUT, "joint_axes.png"))

# ---------------------------------------------------------------- 2. stand test
mujoco.mj_resetData(m, d)
set_pose({})
d.ctrl[:] = 0.0
log = []
for i in range(int(2.0 / m.opt.timestep)):          # 2 s of sim
    mujoco.mj_step(m, d)
    if i % 50 == 0:
        log.append(d.actuator_force.copy())
tau = np.abs(np.array(log[-10:])).mean(axis=0)       # settled torques
snap(cam(azim=120, elev=-15, dist=0.55), f"stand test t=2s  base z={d.qpos[2]*1000:.1f} mm").save(
    os.path.join(OUT, "stand_test.png"))

print(f"base height after 2 s: {d.qpos[2]*1000:.1f} mm (start 90.9)")
print(f"{'joint':14s} {'torque N*m':>10s} {'% of stall':>10s}  angle error deg")
for i in range(m.nu):
    j = m.actuator(i).name
    err = np.degrees(d.qpos[m.joint(j).qposadr[0]])
    print(f"{j:14s} {tau[i]:10.4f} {100*tau[i]/TORQUE_LIMIT:9.0f}%  {err:+.1f}")
