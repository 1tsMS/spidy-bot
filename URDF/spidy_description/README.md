# spidy_description

URDF + MuJoCo model of Spidy, generated from the Fusion model `MS makda urdf 2`.

## Layout

| Path | What | Edit by hand? |
|---|---|---|
| `urdf/spidy.urdf.xacro` | Robot structure: links, joints, leg macro | **Yes**, this is the URDF to read and learn from |
| `urdf/spidy_generated.xacro` | Numbers: lengths, masses, inertias | No, it's written by `build_description.py` |
| `urdf/spidy.urdf` | Plain expanded URDF (meshes `../meshes`) | No, regenerate with xacro |
| `config/masses.yaml` | Mass budget (**estimates**, weigh and replace) | Yes |
| `config/fusion_transforms.json` | Component placements exported from Fusion | No |
| `meshes/*.stl` | One mesh per link, in that link's frame, metres | No |
| `mujoco/spidy.xml` | MuJoCo model: free base, floor, 12 position servos, IMU | No, it's written by `urdf_to_mujoco.py` |

## Use

```bash
pip install -r requirements.txt
python -m mujoco.viewer --mjcf=mujoco/spidy.xml      # open it, drag the ctrl sliders
python scripts/check_model.py                        # joint-axis images + stand torque audit
```

After weighing parts (edit `config/masses.yaml`):

```bash
python scripts/build_description.py --raw ../raw_world_stl   # meshes + spidy_generated.xacro
xacro urdf/spidy.urdf.xacro mesh_prefix:=../meshes > urdf/spidy.urdf
python scripts/urdf_to_mujoco.py
```

## Conventions

- Frame: x forward, y left, z up. Base origin is the body centre at hip-pitch height.
- Zero pose: the pose in the Fusion model (femur horizontal, tibia vertical).
- Every leg uses the same axes: yaw +z, pitch +x, knee +x. So `+hip_pitch` lifts a left leg and lowers a right leg. The gait code handles the mirroring.
- Joint limits are ±90° placeholders until they're mapped from the servo calibration.

## Leg geometry (from Fusion)

| | mm |
|---|---|
| hip-yaw axes from body centre | x ±60, y ±50 |
| coxa (yaw → pitch) | 34.2 |
| femur (pitch → knee) | 55.8 |
| tibia (knee → foot tip) | 88.9 down; tip sits 5.0 toward body centre (x) and 1.2 inward (y) |
