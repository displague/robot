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

## D-006 (provisional, 2026-10-25) W-max = 775 brushed motor + printed 1:40 planetary

Why: walking is a goal (P-002), and the calculator's walking scenarios need ~65 W at the ankle
and ~35 W at the hip. The wiper motor peaks near 18 W and weighs 21 kg across the body: retired
for legs. The 775 + printed planetary is ~113 W peak, 0.45 kg, ~$11, and meets every walking
scenario with margin. Risk: printed PETG gears at 100 W.
Confirmed or reversed by: the Phase 2 entry test (one hour at 100 W on a rope brake, then to
destruction; must survive 2x the 20 N m walking peak). Fallback: 5010/5208 gimbal BLDC +
printed cycloid + SimpleFOC, ~2x the price, back-drivable.

## D-011 (2026-10-25) Walking and balancing are requirements, not stretch goals

Why: stated in P-002. Consequences: leg W-max chosen by walking power (D-006); pelvis and foot
IMUs, four force sensors per foot, AS5600 on every leg DOF; balance controller on the leg and
torso ESP32 nodes with no Pi in the loop; all leg motors in the pelvis and thigh so shank and
foot stay light; a series elastic element in the ankle plantarflexor tendon; roadmap Phase 5
(stand, balance) and Phase 6a/6b (static, dynamic walking) with exit tests.
Reverses if: nothing planned; the mass budget is the thing most likely to force a trade.

## D-013 (2026-10-25) Straight tendon runs are lined with hollow piercing needles; nothing is inserted mid-print

Why: suggested in P-003. Stainless hypodermic tubing in piercing-needle form is ~$13 per
hundred, cut to length with a rotary tool, and a press fit (OD + 0.1 mm) holds it in a printed
channel after printing. It removes channel abrasion, the main wear mode of a braided line in
plastic. Printed countersunk mouths take the bend; PTFE tube takes any curved path. The same
rule applies to every insert in the robot: liners, tubes, pins, bearings, magnets, heat-set
inserts all go in post-print, so there is never a print pause.
Reverses if: the cut needle rims prove impossible to deburr reliably at home (then PTFE liners
everywhere, at the cost of a larger channel).

## D-014 (2026-10-25) Reverse stroke: elastic for fingers, closed loop on one drum for everything else; crimp terminations

Why: asked in P-004. A closed loop (both ends of one line on one drum, wound opposite ways,
midpoint anchored to the joint pulley by a crimped stop bead) gives stiff motion both ways from
a single motor with no slack, provided both sides have equal moment arms and both runs are
sheathed. Fingers violate the equal-arm rule (9 / 6.5 / 4.5 mm palmar vs a different dorsal path),
so they keep the elastic antagonist. Pretension via a bicycle barrel adjuster on one sheath end.
Terminations: stopper knots for prototypes, fishing crimp sleeves and crimped stop beads for
production, a needle toggle for the highest loads. Mechanical stops in both directions on every
DOF are the homing reference.
Reverses if: Phase 1 wrist loops show the loop binding or creeping (then look at pulley radius
matching and sheath seating before anything else).

## D-015 (2026-10-25) Passive balance = locked statue + long feet + foot ballast; no bladder; reaction wheel deferred

Why: asked in P-005. A locked robot tips at ~18 N at shoulder height; foot length adds about
1 N per cm, 2 kg of shot per foot adds ~4 N. Passive ankle springs would need 176 N m/rad to
stand alone, beyond any elastic. A liquid bladder worsens lean (free-surface effect) and pumps
too slowly for stepping. A torso reaction wheel is the one "natural" balancer worth revisiting.
Reverses if: Phase 5 push tests show the ankle strategy cannot cover the gap between the statue
margin and the 20 N target (then the reaction wheel moves into Phase 6).

## D-016 (2026-10-25) Springs and compliance policy

Why: asked in P-005. Constant-force spools (or badge reels) replace elastic cords for finger
extensors in the Phase 1 trial; zero-free-length elastic counterbalance on shoulder flexion and
abduction cancels the arm's own 9 N m; series elastic ankle; closed loops pretensioned by barrel
adjuster except on neck and spine where a spring seat may self-tension. Compliance: tendon
stretch plus elastic antagonists plus drag slip as the safety floor, series-spring force
sensing (encoder minus joint sensor) feeding an impedance mode, and a limp mode for falls and
hand-guiding. Locked joints are deliberately non-backdrivable.
Reverses if: impedance mode proves too slow for safe arm contact (then add a true backdrivable
BLDC W-max on the arms).

## D-017 (2026-10-25) Two mechanical primitives: the knuckle chain and the ring joint

Why: asked in P-007. Generalising the finger shows that fork-and-tongue knuckles with tendon
channels cover 36 of the 50 DOF across 26 joint instances, and axial ring joints cover 8 more.
One generator (`chain.py`) now builds all knuckle instances from presets. Two rules came out of
it and are binding for the shell pass: joints that a later tendon must cross use bearings with
stub axles so the tongue centre stays solid and the crossing runs pass through the axis with no
coupling; and section width is sized by that routing check, not by strength.
Reverses if: the stub-axle press fit in PETG proves unable to hold under hip loads (then a
through-shaft with the crossing runs offset symmetrically above and below the axis, with the
resulting pitch-into-yaw coupling compensated in firmware).

## D-012 (2026-10-25) Every set of work records the prompt that caused it

Why: asked in P-002. `docs/prompt-log.md` holds each prompt verbatim with the files and
decisions it produced; commit messages quote the prompt ID and text for the work they contain.

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
