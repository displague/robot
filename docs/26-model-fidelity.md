# 26 - How close is the model: shape, weight, feet, wind, sensors, and what the body can do about wind (P-040, D-044)

The owner asked whether the shape, friction, weight and wind dynamics are accurate, whether the robot's own sensors
can detect the wind well enough to balance against it, and whether the body can bow, bend or turn its hands to
reduce the load in time. The feet will get an adhesive silicone non-slip pad. Each question below has a measured
answer. "Truth" means the simulator's exact state, which the controller has used until now.

| Item | Modelled as | Verdict | Checked by |
| --- | --- | --- | --- |
| Shape (for drag) | MuJoCo's equivalent inertia box per body, built from the printed shell envelope (`export_mjcf.SHELL`) | frontal area 0.436 m^2 against the shells' 0.484 m^2; drag within about 25 % with Cd (`25`) | `posture_wind.py`'s box formula against MuJoCo's own drag (26.2 against 25.6 N) |
| Wind speed | uniform, the speed at the robot | a forecast over open ground is about 1.7 times higher (`25`) | log law |
| Gusts | OU, sigma 1.5 to 3 m/s, tau 2 s | open ground to built-up; slightly short of fast buffeting | turbulence intensity, integral scale |
| Weight | 20 kg; the roll-ups are 25 and 26 kg | the heavier build carries more wind (below) | `wind_sweep.py` with `ROBOT_MASSES` |
| Feet | friction 1.0, soft contact (creeps) | silicone pad: 0.5 to 1.2, does not creep; the step needs held feet (`25`, D-043) | the friction sweep, `NOSLIP` |
| Sensors | the simulator's CoM and contact forces, no delay | detected well enough if the loop is fast: delay at most 5 ms | `sensor_sweep.py` |
| Wind strategy | ankle lean (D-042) | bowing into a head wind adds 2 m/s if started early; nothing else helps | `posture_wind.py` |

## Feet: the silicone pad

The pad is a cut-to-shape silicone non-slip sheet, 0.04 in (1 mm), adhesive backed (5 x 40 in, about $10; the
closest listing found is Jayen's "5x40 Matte" on Amazon, [camelcamelcamel](https://camelcamelcamel.com/product/B0FGPKNJ5P);
the UPC 717018793922 did not resolve). It is bonded with 3M 9448A, a double-coated tissue tape with an acrylic
adhesive, 0.15 mm thick ([datasheet](https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/6546/9448A-4-50.pdf)).

| Figure | Low | Nom | High | Source |
| --- | --- | --- | --- | --- |
| Sliding friction, pad on floor | 0.5 (dusty or wet) | 0.9 | 1.2 (clean smooth floor, tacky silicone) | rubber on concrete: 0.6 to 1.0 dry, 0.3 to 0.45 wet ([Engineering Toolbox](https://engineeringtoolbox.com/friction-coefficients-d_778.html), [hypertextbook](https://hypertextbook.com/facts/2006/MatthewMichaels.shtml)); no silicone figure was found, and silicone can be below natural rubber, so the low end is set at wet rubber |
| Shear stiffness of the pad | | | | 1 mm silicone (G 0.3 to 1 MPa) under a 100 x 260 mm sole: 30 N of side load is 1.2 kPa of shear, 1 to 4 um of movement. Rigid for every purpose here |

- **What it means for the model.** The sole does not creep, so `NOSLIP=5` (held feet, D-043) is the right model.
  MuJoCo's creep (78 mm in 20 s in wind) was entirely an artifact. The robot uses 0.15 to 0.28 of friction
  (`25`), so a pad at 0.5 holds. The step and the walk on held feet are the open problem (23 and 16 of 42).
- **The tape.** 9448A's acrylic sticks to the pad's own adhesive backing and to PLA or TPU. It does not stick to
  bare silicone: put it only on the pad's adhesive side.
- **Peel.** A pad peels from its edge when the foot rolls onto its heel or toe under load. Round the sole's edges
  or recess the pad 1 mm with the edge tucked in.
- **The foot sensors** (4 per foot, docs/07) go under the pad or a TPU spreader, not on the pad's surface.

## Weight: the 26 kg build carries more wind

The wind map on the 775 roll-up (26.1 kg, `ROBOT_MASSES=sim/masses/rollup_775.json`, one seed):

| Scene | Wind | 20 kg model | 26 kg roll-up |
| --- | --- | --- | --- |
| standing | head, steady | 12 m/s (14 falls) | **14 m/s** |
| standing | head, 1.5 m/s gusts | 10 m/s (1 of 2 seeds) | 10 m/s (12 falls) |
| standing | side, steady / gusts | 10 / 10 m/s | **12 / 12 m/s** |
| one-foot hold | head, steady / gusts | 9 / 9 m/s (swept to 9) | 12 / 10 m/s |
| one-foot hold | side, 1.5 m/s gusts | about 5 m/s | falls at 8 (lower not run) |

The same drag on 30 % more weight tilts it less. The one-foot hold in a gusty side wind is the limit for both.

## Sensors: can the robot feel the wind?

The controller has used the simulator's exact CoM and contact forces. The build has a BNO085 IMU at the pelvis,
encoders on every joint, and 4 load sensors per sole (FSR 402 under a pad, or a load cell, docs/05 and 07).
`tools/sensor_sweep.py` runs any scene on modelled sensors instead:

- **CoM** (IMU, encoders, mass model): sampled at the loop rate, a delay, a constant 10 mm bias (mass-model
  error), a smooth 3 mm wander and fine noise.
- **Each sole corner:** a gain error per sensor, hysteresis, drift and saturation. For the FSR 402 these come
  from Interlink's datasheet: 0.2 to 20 N range, part to part +-6 %, hysteresis +10 %, drift under 5 % per
  decade ([400-series datasheet](https://www.teachengineering.org/content/uconn_/activities/uconn-2662-contractions-calculations-force-resistance-activity/uconn-2662-contractions-calculations-FSR400series-datasheet.pdf)).
  Four of them read at most 80 N per foot, against up to 200 N on one foot, so a pad must carry most of the load
  and the FSR reads a calibrated fraction.

**One error at a time** (40 N push, the hardest gauntlet scene): only the delay matters. 2 ms and 5 ms stand,
10 ms falls. The wander, gains, hysteresis, drift and saturation each stand. Noise raises the energy (44 J to
120 J) but stands.

**The whole set:**

| Scene | Truth | 1 kHz loop, 3 ms, load cells | 1 kHz, 3 ms, FSRs | 200 Hz, 10 ms, load cells or FSRs | 100 Hz, 20 ms, uncalibrated FSRs (15 %), 20 mm bias |
| --- | --- | --- | --- | --- | --- |
| push 20 / 40, lean15, one-foot | all stand | all stand, 1.5 to 3 x the energy | the same | push 40 falls | not run |
| standing in wind (head 8 to 12, side 8 to 10, gusts) | the map | **the same cells** | the same cells | not run | the same cells, 2 to 8 x the energy |
| one-foot hold in wind | the map | the same cells | the same cells | not run | 10 m/s head falls |
| step, still air / 3 m/s gusty | S / S | S / S | S / **F** | not run | **F / F** (lift gate times out) |
| walk, still air / 3 m/s gusty | S / S | S / **F** | S / S | not run | **F / F** |

- **The wind itself is easy to feel.** The lean uses the slow part of the disturbance estimate, filtered over
  0.4 to 1.5 s, so sensor noise averages out. A 10 mm CoM bias is like 2 N of standing wind and the integral
  takes it out. The robot needs no anemometer.
- **What breaks is speed.** A loop that is 10 ms late loses the 40 N push. Docs/04 runs the joint loop at 200 Hz:
  that is enough for the joints, but the balance loop must run at about 1 kHz, IMU to motor command within 5 ms
  (D-044).
- **The planner's lift gate reads the foot load.** With FSRs uncalibrated to 15 % it never sees the trailing foot
  unload and the step stops (`Fg`). Calibrated FSRs (6 %) and load cells score about the same. The FSR's 20 N
  range is the real limit: a load cell per corner (docs/07's alternative, HX711) needs no load-sharing pad.
- **The step and walk lose a run each in gusty 3 m/s on real sensors.** The margin is thin; they share the
  transfer problem of D-043.

## What the body can do about wind

`tools/posture_wind.py static` puts the robot (with its controller) in a posture and computes the drag at
10 m/s, its height, and the CoM offset needed to carry it (drag x height / weight). For standing head-on that
offset is 119 mm.

| Posture | Head-on drag | Offset needed | Side drag |
| --- | --- | --- | --- |
| standing | 25.7 N | 119 mm | 21.9 N |
| squat (knees 30 deg; the ankle's -20 deg stop limits it) | 25.0 N | 114 mm | 22.0 N |
| bow 20 deg (spine) | 23.0 N | 104 mm | 21.9 N |
| bow 35 deg | 19.8 N | **84 mm** | 21.9 N |
| arms folded in | 23.0 N | 109 mm | 21.9 N |
| hands edge-on (forearm yaw 80 deg) | 24.5 N | 114 mm | 22.9 N |
| trunk turned 40 deg plus hips 30 deg | 22.0 N | 100 mm | 19.5 N |

**Then with the controller in the wind** (`posture_wind.py sweep`, 20 kg, the posture taken before the wind):

| Posture | Head, steady | Head, 1.5 m/s gusts | Side |
| --- | --- | --- | --- |
| standing | 12 m/s | under 12 | under 12 |
| bow 20 | **14** | under 12 | under 12 |
| bow 35 | **14** (16 stands 21 s) | **12** | under 12 |
| squat, arms in, twist, squat + bow + arms | worse or no better | worse | worse |

- **Bowing into a head wind works.** It lowers the trunk's area and height and puts the trunk's weight upwind.
  It is worth 2 m/s head-on, about 35 % more drag.
- **The squat does not help.** The ankle stop limits it to 30 mm lower, and a bent knee weakens the ankle lean.
- **Arms and hands are under 10 % of the drag.** Folding them costs more balance than it saves.
- **The twist makes it worse.** It narrows the side area but loads the feet unevenly.
- **From the side nothing helps.** The limit is the stance width and the hip shift. The answer is a wider stance
  (a side step), which the planner does not have.
- **In time?** The wind ramps in over 0.5 to 2.5 s at 14 m/s. A 35 deg bow over 0.5 s that starts at 2.5 s
  stands. The same bow over 1.0 s, or started at 4 s, falls. The spine drive makes 35 deg in 0.5 s (the model's
  motor limits apply). So the bow has to be a reflex: triggered when the lean nears its limit, within about a
  second, not a planned move.

## How many ways the results are checked

| Check | What it verifies | Independent of MuJoCo? |
| --- | --- | --- |
| MuJoCo, the tendon model (`sim_tendon.py`) | everything: the drive trains, the controller, contacts, drag | no, it is the model |
| A drag formula per body in `posture_wind.py` | MuJoCo's drag: 26.2 against 25.6 N | the formula is independent; the boxes are MuJoCo's |
| Published Cd, log law, turbulence figures (`25`) | the drag's size and the wind's meaning | yes, hand checks |
| Grids over mass sets, step lengths, two drive models, seeds | robustness of one model | no |
| Sourced motor ranges (`24`, low / nom / high) | the parts' uncertainty | the figures yes, the model no |
| Modelled sensors (`sensor_sweep.py`) | the controller off the simulator's truth | no |
| The sibling's stand-in (`ai-embedded-dynamic-diversity`) | learned control | no: also MuJoCo, a simpler model |
| Blender | parts and renders only. Its rigid-body wind field is a constant push, not drag, and it has no tendons or controller | not used for physics |

So the wind has one physics engine and hand checks, and no second engine. An independent check would be one of:
the contact and balance in a second engine (the `isaac-lab-conversion` task: PhysX); the shell's drag in CFD
(OpenFOAM on the STL envelope, one run per direction); or, cheapest and most direct, the printed robot on a
stand with a leaf blower and a luggage scale.

## Not yet

- **A bow reflex** in the balance loop (bow when the lean nears `LEAN_MAX`), then the wind map again.
- **A side-wind strategy:** a wider stance or a side step.
- **The balance loop's latency** in docs/04's electronics: a 1 kHz loop on the leg ESP32s.
- **Load cells or FSRs:** a decision after the step works on held feet; for now a load cell per corner is the
  safer part (range, no calibration drift).
- **The pad's friction is a range** from rubber figures, with no silicone measurement. A pad on a ramp gives it
  in a minute after purchase.
- **A second engine or CFD** for the wind.

## Next

Make the step work on held feet (`step-on-held-feet`, D-043), with the pad's friction range (0.5 to 1.2) and
`sensor_sweep.py`'s `lc_fast` case in the grid. Then the bow reflex, judged on the wind map.
