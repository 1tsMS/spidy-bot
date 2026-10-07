# sim_walk: making Spidy walk in MuJoCo, one idea per file

Each step builds on the one before and runs on its own. Run them from `Codess/Simulation`.

| File | Idea | Run |
|---|---|---|
| `step1_leg_ik.py` | FK + IK for one leg, tested against MuJoCo. Converts your logical servo degrees (Standf) to joint angles | `python sim_walk/step1_leg_ik.py` |
| `step2_stand.py` | Stand in Standf, feet planted, move the body (height, sway, twist) | `python sim_walk/step2_stand.py` |
| `step3_body_shift.py` | Shift the COM over 3 feet before lifting the 4th | `python sim_walk/step3_body_shift.py` (try `--no-shift`) |
| `step4_crawl.py` | Wave gait: body moves continuously, one leg swings at a time, body sways | `python sim_walk/step4_crawl.py` |
| `step5_turn.py` | Turning: stance feet sweep arcs instead of straight lines | `python sim_walk/step5_turn.py` |

Add `--headless` to any of them to skip the viewer and print a test report instead.

`simrun.py` is shared plumbing (load the model, real-time loop, measurements). No walking ideas live here.

## Assumptions to check on the real bot

- `STANDF = [90, 105, 75]` from `Codess/Software/spidy_poses.txt`, read as knee +15 / claw -15 from the modelled pose.
- +knee raises the femur on every leg (implied by the Sit pose). Hip direction is not verified.
- `DEG_PER_LOGICAL = 1.0`: one logical degree is assumed to be one real degree. Not measured.

## Results (704 g model, servo stall 0.21 N*m)

- Crawl: 2.9 cm/s, 14 mm stability margin, about 1 deg body tilt.
- Servo load while crawling straight: knee ~87%, claw ~74% of stall.
- Turning: arc +47 deg (asked ~50), spin -65 deg (asked ~62). Knees reach ~92% while spinning: the tightest case.
