# Spidy Bot

A 3D-printed 12-servo quadruped (ESP32 + PCA9685 + MG90S), with a MuJoCo model, leg IK, a crawl gait in simulation, and a desktop control app.

**Status:** stands and does slow sit/stand with some help. The crawl and turning work in simulation only. The real robot doesn't walk: the front legs don't have enough servo torque (details below).

## Hardware

| Part | Used |
|---|---|
| Frame | PLA, designed in Fusion 360, 4 legs × 3 joints (hip, knee, claw) |
| Servos | 12× MG90S |
| Servo driver | PCA9685 @ 0x40, 50 Hz, with a custom breakout board on top |
| Controller | ESP32 (uPesy Wroom DevKit), Wi-Fi |
| IMU | MPU6050 |
| Battery | 3S 11.1 V 2200 mAh LiPo |
| Servo supply | 12 A 300 W buck, set to 6 V |
| ESP supply | Mini360 buck, 5 V, fed from the battery side |

Leg lengths: coxa 34.2 mm, femur 55.8 mm, tibia 88.9 mm. Total mass 704 g.

## Software

| Path | What |
|---|---|
| `Codess/Software/` | **Old version.** ESP32 firmware with calibration hardcoded on the board, plus a PyQt5 tool for per-servo calibration and poses. |
| `Codess/Software2.0/` | **New version.** PySide6 app: live MuJoCo view, joystick driving, calibration wizard, pose library, 50 Hz Wi-Fi streaming to the robot. Run `Spidy.bat` or `python run_app.py`. |
| `firmware/spidy_fw/` | Firmware for the new app. All calibration lives on the PC; the ESP only drives pulses, holds the last pose if Wi-Fi drops, and reports its reset reason and I2C health. |
| `Codess/Simulation/sim_walk/` | Walking in MuJoCo step by step: IK, standing, body shift, crawl, turning. |
| `URDF/spidy_description/` | URDF and MuJoCo model generated from the Fusion CAD. |

Wi-Fi credentials go in `secrets.h` (copy `secrets.example.h`); it's gitignored.

## Problems and what fixed them

1. **Servo horns at the wrong angle.** Each servo was centred to 90° in software *before* its horn was screwed on, so all legs started from a known position.
2. **"90°" didn't look like 90° on the real legs.** Horn splines and print tolerances left every joint a few degrees off. Added a per-servo offset and direction, saved to a calibration file.
3. **Couldn't sit and stand.** A direct move overloaded the knees. I found a working standing pose (`Standf`, knees bent, tibias vertical) and an ordered chain of intermediate poses by hand. The new app adds IK-planned sit/stand sequences and a step reducer (aggressive / balanced / conservative).
4. **Power.** The original 18650 cells + XL4015 buck sagged under servo load. Replaced them with a 3S 2200 mAh LiPo and a 12 A buck.
5. **Thin PCA9685 power traces.** The board's own traces can't carry 12 servos' current. Made a breakout board that sits on the PCA9685 like a hat, with header pins and thick solder traces for servo power.
6. **ESP32 rebooting during moves.** Brownouts from servo current spikes. Confirmed by having the firmware report the ESP32's reset reason (`boot=BROWNOUT`). Fixed by:
   - moving the Mini360 input from the 6 V servo rail to the battery side,
   - adding 1000 µF + 100 nF at the ESP,
   - adding 1000 µF on the servo rail at the breakout.
7. **Servo scale was wrong.** The old firmware's "degrees" were ~1.46 real degrees (pulse range 100–700 ticks vs the MG90S's 0.5–2.5 ms). Found it because the real Sit pose only matched the sim at that scale. Corrected in the calibration, with pulses unchanged.

## Open problem: weak front legs

This has been there since day one, and the power and wiring fixes didn't change it.

- **Cause:** the battery and buck sit at the front, so the centre of mass is ~12 mm forward.
- In simulation the front knees need **~43% of a genuine MG90S's stall torque just to stand**, against ~29% for the rear knees.
- Servos can only hold roughly 25–50% of stall continuously, and most MG90S sold are clones with well under the rated 2.2 kg·cm. So the front knees run at or past their limit, and transitions need help.
- Swapping the front servos didn't fix it.
- **Likely fixes:** move the battery ~30 mm back (balances front and rear in sim), and use stronger knee servos (≈3.5 kg·cm, e.g. MG92B).
