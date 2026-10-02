#!/usr/bin/env python3
"""
URDF (xacro)  ->  MuJoCo MJCF  (mujoco/spidy.xml)

URDF only describes the robot. To simulate it, MuJoCo also needs:
  * a free joint on base_link (otherwise the body is bolted to the world)
  * a floor + light
  * actuators (URDF has no motors)  -> one position servo per joint
  * joint armature/damping (gearbox inertia + friction of a hobby servo)
We load the URDF with MjSpec, add those things in Python, and save MJCF.

Run:  python scripts/urdf_to_mujoco.py
View: python -m mujoco.viewer --mjcf=mujoco/spidy.xml
"""
import os, re
import mujoco
import xacro

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)

# ---- MG90S model (TUNE THESE from system-ID later) ----
KP = 4.0            # N*m/rad   stiffness of the servo's internal position loop
KV = 0.03           # N*m*s/rad damping of the loop
TORQUE = 0.21       # N*m       stall torque @ 6 V -> actuator force clamp
ARMATURE = 2e-4     # kg*m^2    reflected gearbox/rotor inertia
JOINT_DAMPING = 0.01
FRICTIONLOSS = 0.005
FOOT_FRICTION = 0.9  # PLA tip on a typical floor; rubber feet would be ~1.2


def expand_xacro():
    xacro_file = os.path.join(PKG, "urdf", "spidy.urdf.xacro")
    urdf = xacro.process_file(xacro_file, mappings={"mesh_prefix": "."}).toxml()
    # Tell MuJoCo's URDF importer where meshes are and to KEEP visual meshes
    tag = ('<mujoco><compiler meshdir="%s" strippath="true" discardvisual="false" '
           'balanceinertia="true" fusestatic="false" autolimits="true"/></mujoco>'
           % os.path.join(PKG, "meshes"))
    return re.sub(r"(<robot[^>]*>)", r"\1" + tag, urdf, count=1)


def build():
    spec = mujoco.MjSpec.from_string(expand_xacro())
    spec.modelname = "spidy"
    spec.option.timestep = 0.002
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST

    base = spec.body("base_link")
    base.add_freejoint(name="root")
    base.pos = [0, 0, 0.0889 + 0.002]            # feet just above the floor
    base.add_site(name="imu", pos=[0, 0, 0])

    # Visual meshes: render only, never collide. Primitive geoms: collide.
    for g in spec.geoms:
        if g.type == mujoco.mjtGeom.mjGEOM_MESH:
            g.contype, g.conaffinity, g.group = 0, 0, 1
            g.rgba = [0.25, 0.25, 0.28, 1] if "base" not in g.meshname else [0.6, 0.6, 0.63, 1]
        else:
            g.group = 3
            if g.type == mujoco.mjtGeom.mjGEOM_SPHERE:   # feet
                g.friction = [FOOT_FRICTION, 0.005, 0.0001]
                g.rgba = [0.9, 0.3, 0.1, 1]

    for j in spec.joints:
        if j.type == mujoco.mjtJoint.mjJNT_HINGE:
            j.armature, j.frictionloss = ARMATURE, FRICTIONLOSS
            j.damping = [JOINT_DAMPING, 0, 0]
            a = spec.add_actuator(name=j.name, target=j.name,
                                  trntype=mujoco.mjtTrn.mjTRN_JOINT)
            a.set_to_position(kp=KP, kv=KV)
            a.ctrlrange = j.range
            a.ctrllimited = mujoco.mjtLimited.mjLIMITED_TRUE
            a.forcerange = [-TORQUE, TORQUE]
            a.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE

    # Sensors that the real bot has / will have
    spec.add_sensor(name="imu_quat", type=mujoco.mjtSensor.mjSENS_FRAMEQUAT,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    spec.add_sensor(name="imu_gyro", type=mujoco.mjtSensor.mjSENS_GYRO,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    spec.add_sensor(name="imu_acc", type=mujoco.mjtSensor.mjSENS_ACCELEROMETER,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")

    # World
    spec.worldbody.add_light(pos=[0, 0, 1.5], dir=[0, 0, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)
    tex = spec.add_texture(name="grid", type=mujoco.mjtTexture.mjTEXTURE_2D,
                           builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                           rgb1=[0.82, 0.82, 0.80], rgb2=[0.72, 0.72, 0.70],
                           width=512, height=512)
    spec.add_material(name="grid", textures=["", "grid"], texrepeat=[20, 20])
    spec.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE,
                            size=[2, 2, 0.05], material="grid")
    spec.visual.global_.offwidth = 1280
    spec.visual.global_.offheight = 960

    model = spec.compile()
    out = os.path.join(PKG, "mujoco", "spidy.xml")
    with open(out, "w") as f:
        # make the saved XML portable: absolute mesh dir -> path relative to mujoco/
        f.write(spec.to_xml().replace(os.path.join(PKG, "meshes"), "../meshes"))
    print(f"wrote {out}: nbody={model.nbody} njnt={model.njnt} nu={model.nu} "
          f"mass={model.body_subtreemass[1]*1000:.0f} g")
    return out


if __name__ == "__main__":
    build()
