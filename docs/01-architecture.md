---
title: "01 - Architecture"
summary: "System overview, DOF budget, actuator classes, compute topology, honest limits"
section: "Design and hardware"
---

# 01 - Architecture

## System in one picture

```
                 +-----------------------------+
                 |  Brain (optional, Phase 4+) |  Pi 5 / RK1: vision, planning, speech
                 |  USB-CAN or UART            |
                 +--------------+--------------+
                                | CAN bus (1 Mbit/s, 12 V + CAN_H/CAN_L, one 4-wire harness)
      +-------------+-----------+-----------+-------------+-------------+
      |             |                       |             |             |
+-----+-----+ +-----+-----+           +-----+-----+ +-----+-----+ +-----+-----+
| ESP32 L-ARM| | ESP32 R-ARM|           | ESP32 TORSO| | ESP32 L-LEG| | ESP32 R-LEG|
| forearm +  | | forearm +  |           | spine,neck,| | pelvis +   | | pelvis +   |
| upper-arm  | | upper-arm  |           | face, shldr| | thigh bank | | thigh bank |
| winch banks| | winch banks|           | winch banks| |            | |            |
+-----+------+ +-----+------+           +-----+------+ +-----+------+ +-----+------+
      |              |                        |              |              |
  tendons in PTFE sheath  ->  joints with magnetic angle sensors  ->  limbs
```

Each ESP32 node runs the same firmware: N winch channels (PWM + direction, motor encoder, joint
angle sensor), 1 kHz position/tension loops, CAN messages in ("go to angle", "hold tension",
"go limp") and out (angles, currents, faults). A laptop over CAN or Wi-Fi is enough to run
teleoperation and scripted motion. The Pi-class brain is only needed for autonomy.

## ROM parity, not DOF parity

The human body has roughly 200+ degrees of freedom. We match the **range** of every joint and
drive it with the smallest number of motors that still gives useful, human-looking motion.
Where one tendon can move several joints in a natural order (fingers curl from the tip in),
it does. Where gravity or an elastic can do the return stroke, it does.

Three ways a joint gets driven, chosen by load:

| Pattern | Where | Motors per DOF | Notes |
| --- | --- | --- | --- |
| Single tendon + elastic return | fingers, thumb, eyes, jaw, eyebrows | 1 (shared across joints in a finger) | Elastic (TPU band, dental elastic, silicone cord) extends. Cheapest, compliant, safe. |
| Closed-loop cable | wrist, elbow, forearm, shoulder, neck, spine | 1 | Both tendon ends wrap one drum in opposite directions, loop runs round the joint pulley. Stiff both ways, pretensioned, no antagonist motor. |
| Closed-loop cable + tackle | hip, knee, ankle | 1 | Same, with a 2:1 to 4:1 block and tackle to multiply force. Slow but strong. |

## DOF budget (actuated)

| Region | Actuated DOF | Pattern | Winch class | Notes |
| --- | --- | --- | --- | --- |
| Hand (each) | 5 finger flex + 1 thumb opposition + 1 finger spread (shared) = 7 | single tendon | W-mini | Finger abduction via one whiffletree tendon or passive |
| Wrist + forearm (each) | flex/ext, radial/ulnar, pronation/supination = 3 | closed loop | W-mini x2, W-mid x1 | |
| Elbow (each) | 1 | closed loop | W-mid | |
| Shoulder (each) | flex/ext, abd/add, int/ext rotation = 3 | closed loop (+tackle) | W-max | |
| Neck | pitch, yaw, roll = 3 | closed loop | W-mini | |
| Face (optional) | eyes pan/tilt 2, eyelids 1, jaw 1 = 4 | single tendon | W-mini | Pure cosmetics, do last |
| Spine | yaw, pitch, lateral = 3 | closed loop + tackle | W-max | |
| Hip (each) | flex/ext, abd/add, rotation = 3 | closed loop + tackle | W-max | |
| Knee (each) | 1 | closed loop + tackle | W-max | |
| Ankle (each) | dorsi/plantar, inversion/eversion = 2 | closed loop + tackle | W-max | Toes passive (elastic) |
| **Total** | **14 + 6 + 2 + 6 + 3 + 4 + 3 + 6 + 2 + 4 = 50** | | | 46 without the face |

Count by class: W-mini 2x(7+2) + 3 + 4 = 25, W-mid 2x(1+1) = 4, W-max 2x3 + 3 + 2x6 = 21.
Human-looking range everywhere; independent control of every finger joint is deliberately not a goal.

## Actuator classes

All three are the same winch (see `02-tendon-actuator.md`): motor, optional self-locking stage,
drum, line exit into a PTFE socket, friction drag as overload protection, joint-side angle sensor.

| Class | Motor | Peak mech power | Tendon force (cont / stall) | Mass | Price | Used for |
| --- | --- | --- | --- | --- | --- | --- |
| W-mini | N20 micro gearmotor, 6 V, 100:1 or 298:1, 3 mm drum | ~1 W | 13 / 42 N (100:1), 37 / 122 N (298:1) | 10 g + 15 g printed | ~$5 | hands, wrist, neck, face |
| W-mid | 37 mm brushed gearmotor (JGB37-520) with hall encoder, 12 V, 1:90, 6 mm drum, 2:1 tackle | ~3-5 W | 130 / 425 N | 200 g + 60 g | ~$14 | elbow, forearm rotation |
| W-max | 775 brushed motor + printed two-stage 1:40 planetary, 12 V, 15 mm drum, 2:1 or 3:1 tackle (provisional, D-006; fallback 5010-class gimbal BLDC + printed cycloid) | ~110 W | 580 / 1900 N (2:1), 860 / 2900 N (3:1) | 450 g | ~$11 + driver $4 | shoulder, spine, hip, knee, ankle |

The honest finding from `tools/tendon_calc.py`: cheap gearmotors have **enough force** for
every joint once drum radius and tackle are chosen, but **not enough power**, so large joints
driven by 37 mm gearmotors move at a fifth of human speed. Force and speed trade against each
other on the drum; only the motor sets the watts. Pick W-max by power, not torque.

## Regional winch banks vs one central tensioner

The question raised at kickoff was whether one central multi-line tensioner could drive many
tendons. The analysis:

- A single spinning shaft with per-tendon clutches replaces N motors with 1 motor + N clutches.
  A printable clutch that can be modulated for position control needs its own small actuator
  (micro servo or solenoid) plus a per-tendon encoder, so per-tendon cost is not lower than an N20
  motor at ~$4, and control is much harder (slip, heat, no reversing without a second clutch set).
- Where "central" clearly wins is **packaging and wiring**: all the motors of a region in one
  block, one controller, one harness, tendons leaving through one bundle of PTFE tubes.

Decision: **regional banks, one motor per tendon, PTFE-sheathed tendons.** A forearm bank holds
the 7 hand winches + 2 wrist winches in a block the size of a human forearm. A clutch-based
central drive stays on the table for the face only, where 4-6 low-force DOF could share one motor.

Shared-motor tricks we do use: a whiffletree (a floating bar that splits one tendon into two or
four) for finger spread and for adaptive grasp, so one motor closes a hand that conforms to an
object. This is a sewing-machine / sailing bridle idea and is fully printable.

## Tendon routing rules

- Line: braided PE (Dyneema/Spectra) fishing line, never monofilament (nylon stretches 10-20 %).
- Sheath: 2 mm ID / 4 mm OD PTFE tube, the Bowden tube every printer owner has. The sheath takes
  the compression, so a tendon can cross a joint it does not drive without loading it.
- Every tendon drives one joint pulley (closed loop) or one chain of joints (single tendon with
  elastic return). Tendon moment arm is set by a printed pulley or a channel offset at the joint;
  bigger pulley = more torque, more excursion.
- Joint angle is measured **at the joint** (AS5600 magnetic sensor, 6 mm magnet in the axle) so
  tendon stretch, creep and drum wind-up never corrupt position. Motor encoder only for velocity
  and for detecting slip in the drag.
- Every tendon has a hard stop at both ends of travel so homing is "drive gently until the current
  rises", and a periodic re-home absorbs creep.

## Compute

| Node | Board | Job | Why this board |
| --- | --- | --- | --- |
| Limb controllers (5) | ESP32-S3 (or plain ESP32) | up to 12 winches each: PWM out, encoder in, I2C to angle sensors, 1 kHz loops, CAN | $3-5, two cores, hardware CAN (TWAI), plenty of PWM, Wi-Fi for debugging |
| Brain (0 or 1) | Pi 5 or RK1 | camera, planning, speech, logging | Only needed for autonomy; Phase 0-3 use a laptop |

Yes, the ESP32s alone are sufficient for a teleoperated or scripted humanoid. A Pi is not on the
critical path and should not be bought until Phase 4.

## Honest limits

- **Speed**: with the cheap motor classes above, arm motion is 3-5x slower than human, hands
  close in about half a second with light force. Human-speed arms would need ~50 W per shoulder
  and ~100 W per hip/knee, which moves W-max into the $50-80-per-joint actuator class.
- **Walking** is a requirement (D-011). Walking needs 2-5x the power of slow sit-to-stand, which
  is what fixed W-max as the 775 + printed planetary (~110 W) and retired the wiper motor. The
  printed gearbox at 100 W is the biggest unproven element in the project and gets its own bench
  test before any leg is printed. See `07-locomotion.md`.
- **Mass**: InMoov-scale printed humanoids land at 15-30 kg. Our budget is 20 kg including 21
  W-max motors at 0.45 kg. Walking makes that budget hard: leg power scales with the mass the
  legs carry.
- **Stress analysis**: Blender has no finite-element solver. It is used for kinematics, range of
  motion, tendon excursion vs angle, collision, mass properties and rendering. Stress on printed
  parts is checked by hand calculation first and by FreeCAD's FEM workbench (CalculiX) where a
  part is life-critical (hip clevis, knee pulley). See `cad/blender/README.md`.
- **Safety**: a 400 N tendon can cut skin. Every W-max has a friction drag set below the force
  that breaks its printed pulley, every tendon has a sheath, and the E-stop cuts motor power while
  the self-locking stages hold pose (the robot freezes instead of collapsing).
