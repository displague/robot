# 06 - Roadmap

Each phase ends with a physical test that can fail. A failed test rewrites the decision log, not
the roadmap.

## Phase 0 - One finger, one winch (2-3 weeks of evenings, < $40)

Build: one printed index finger (3 joints, 10/7/5 mm moment arms, elastic extensor), one W-mini
(N20 100:1, 3 mm drum), 300 mm of PTFE tube, one ESP32, one DRV8833, one AS5600 at the MCP.

Exit criteria:
- Finger closes from straight to full curl in <= 0.7 s and reopens in <= 1.0 s.
- Pinches a 5 N spring scale at the fingertip and holds it for 10 minutes with <= 100 mA.
- 10,000 open/close cycles with no line failure and no measurable drift after re-homing.
- Measured N20 stall torque and no-load speed replace the catalogue values in the calculator.

This phase validates the whole thesis (line, drum, sheath, lock, sensor, firmware). If it fails
on hold current, add the ratchet. If it fails on line wear, enlarge the drum and radius the exits.

## Phase 1 - Hand and forearm bank (1-2 months, ~$120)

Build: full hand (5 fingers, thumb opposition, spread whiffletree), forearm shell with a 9-winch
bank, wrist universal joint with two closed-loop cables, one ESP32 node running the real firmware.

Exit criteria: picks up a 500 g water bottle, a pen, a coin from a table; holds the bottle for
an hour with no power to the motors; every hand joint reaches the "ours" range in
`03-skeleton-joints.md`.

## Phase 2 - Arm (1-2 months, ~$150)

Build: elbow (W-mid), forearm rotation (W-mid), humerus, and a bench-mounted shoulder with three
W-max of whichever type D-006 chose. This is where W-max gets its first real test.

Exit criteria: lifts 1 kg from the table to shoulder height in <= 4 s; shoulder reaches 165
degrees abduction; a 100 N pull on the hand does not damage anything (the drags slip).

## Phase 3 - Torso and second arm (1-2 months, ~$250)

Build: torso shells, spine joint, pelvis plate on a stand, both arms, two arm nodes + torso node
on CAN, laptop teleoperation with two game controllers or a VR tracker.

Exit criteria: two-handed carry of a 2 kg box; bimanual pour from a bottle to a cup; 2 hours of
teleoperation with no fault.

## Phase 4 - Head, senses, brain (1 month, ~$250)

Build: neck, skull, camera eyes (optional face motion), Pi 5 brain, speech in/out, simple
vision (object detection, hand-eye calibration for grasp).

Exit criteria: "pick up the red cup" works from a voice command, unassisted.

## Phase 5 - Legs, standing (2-3 months, ~$400)

Build: both legs, feet, 12 W-max, IMU, a safety harness from the ceiling.

Exit criteria: sit-to-stand in <= 5 s on a chair; stands for 10 minutes with motors unpowered
(self-locking); shifts weight foot to foot; sits down under control.

## Phase 6 - Walking (stretch)

Needs W-max power roughly 2-3x the Phase 5 requirement and a balance controller on the leg
nodes. Decide after Phase 5 measurements whether the 775/planetary W-max has the headroom or
whether the leg joints upgrade to BLDC/cycloid.

## Parallel tracks (no hardware needed)

- Blender armature with every joint axis and limit, driven by sliders, exported as the kinematic
  description the firmware and the planner share.
- Tendon excursion measurement in Blender feeding `tools/tendon_calc.py`.
- Firmware skeleton: channel abstraction, CAN protocol, homing, logging, bench GUI in Python.
- FreeCAD FEM on the knee pulley and hip clevis before Phase 5 printing.
