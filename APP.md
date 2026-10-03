# Spidy Control app

Desktop app to calibrate, pose and walk Spidy, in the MuJoCo sim and on the real robot.

```
pip install -r requirements.txt
python -m app                 # from the repo root
python -m unittest discover -s tests -v
```

## Layout

| Path | What |
|---|---|
| `spidy/` | Core, no GUI: kinematics, gait, calibration, link, sim, IMU, poses |
| `app/` | PySide6 GUI: Dashboard, Motors, Calibration wizard, Poses |
| `config/calibration.json` | The ONLY calibration. Imported once from `Codess/Software/spidy_calibration3.txt` |
| `config/poses.json` | Poses in joint degrees + sequences. Imported once from `spidy_poses.txt` |
| `firmware/spidy_fw/` | New firmware: pulses in, servos out. Needs `secrets.h` (copy `secrets.example.h`) |

## Calibration model

```
ticks = center + direction * ticks_per_deg * joint_deg      (clamped to min/max joint degrees)
```

`joint_deg` is the same angle the sim and IK use (0 = femur flat, tibia vertical).

## Safety

- Nothing reaches the real servos unless the target is REAL/BOTH **and** ARM is on.
- Arming first reads the servos' current pulses (`STATE`) so streaming starts there, without a jump.
- Every target goes through a 240°/s slew limit and each joint's min/max.
- E-STOP (button or Space) turns every output off and disarms.
- If the stream stops for 300 ms, the firmware holds the last pose.

## Keys

W/S forward/back, A/D turn (combine for arcs), Space = E-STOP. Gamepad: left stick, A stand, B sit, Back = E-STOP.
