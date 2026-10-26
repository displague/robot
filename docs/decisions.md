# Decision log

Format: ID, date, decision, why, what would reverse it.

## D-001 (2026-10-25) Tendon-driven with winches, not servos at the joints

Why: a motor at every joint (InMoov style) puts mass at the extremities, needs a 180-degree servo
per DOF, and gets expensive past 40 DOF. Winches in regional banks keep limbs light, allow any
motor to drive any joint through PTFE sheath, and lock pose for free.
Reverses if: Phase 0 shows line wear or creep that homing cannot absorb.

## D-002 (2026-10-25) Braided PE fishing line in 2/4 mm PTFE Bowden tube

Why: < 2 % stretch, 130-900 N per strand at $10 a spool, slides in PTFE with near-zero friction,
and the tube is the one every printer owner has. Monofilament rejected for 10-25 % stretch.
Reverses if: abrasion at tube exits proves unsolvable with radiused lips (then Kevlar cord).

## D-003 (2026-10-25) One motor per tendon in regional banks; no central clutch-based tensioner

Why: a modulated printable clutch plus its actuator and encoder is not cheaper than a $4 N20, and
position control through slipping clutches is a research project. Packaging benefit of
"central" is kept by putting all winches of a region in one block.
Reverses if: the face needs 6+ DOF and a shared-motor cam/clutch bank proves simpler there.

## D-004 (2026-10-25) ROM parity, underactuated hands (7 motors per hand)

Why: human-looking range with a quarter of the actuators. One tendon per finger with 10/7/5 mm
moment arms produces a natural curl and adaptive grasp.
Reverses if: a task needs independent PIP control (unlikely for a home humanoid).

## D-005 (2026-10-25) W-mini = N20 100:1 without a worm; lock via gearbox + hold current

Why: calculator shows the worm costs half the power and makes a finger take 4 s to close. The
100:1 N20 closes in 0.5 s and still pinches 42 N peak; a 0.1 A hold is cheaper than a worm.
Reverses if: Phase 0 hold test fails (then ratchet + pawl, D-005b).

## D-006 (open) W-max motor class

Candidates: 12 V wiper motor (cheap, worm lock, 1 kg each, 21 kg total: over mass budget);
775 brushed motor + printed 2-stage planetary (cheap, 0.45 kg, ~100 W, printed gears at full
load unproven); 2806 gimbal BLDC + printed cycloid + SimpleFOC (quiet, back-drivable, 0.2 kg,
dearest, most electronics). Leaning 775 + planetary for cost and mass. Decide after a bench test
of one printed planetary at 100 W for 1 hour (Phase 2 entry).

## D-007 (2026-10-25) ESP32 limb nodes on CAN; Pi-class brain optional and deferred to Phase 4

Why: motion control needs nothing more than an ESP32 per limb; the brain is for autonomy only.
Buying it later costs nothing and avoids designing around a board that may change.

## D-008 (2026-10-25) Joint-side AS5600 angle sensing on every DOF

Why: tendon stretch, creep and drum wind-up make motor-side encoders lie about joint angle. $1
per joint buys the truth and makes homing and slip detection trivial.

## D-009 (2026-10-25) Blender for kinematics, ROM, excursion and mass; FreeCAD FEM for stress

Why: Blender has no FE solver. Its strengths (rigging, drivers, Python, rendering) are what the
kinematic side needs. Stress on the few life-critical parts goes to CalculiX via FreeCAD.

## D-010 (2026-10-25) Single waist joint, not a flexible spine

Why: one universal + yaw bearing delivers most visible torso motion at a fraction of the parts.
Reverses if: a use case needs a bending back (none planned).
