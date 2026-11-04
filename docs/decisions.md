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

## D-018 (2026-10-25) Ring joint: printed ball race on airsoft BBs, rotor as its own pulley, pin-in-track stop

Why: asked in P-009. The eight axial DOF get one generator (`ring.py`). A four-wall PETG race on
6 or 8 mm BBs is free and loaded through a plugged hole; a bought thin-section bearing stays as a
drop-in preset. The closed loop wraps the rotor rim itself, so the moment arm equals the joint
radius. Everything downstream passes through the bore on the axis (twist cost sqrt(L^2 + (rho
theta)^2)). The stop is a pin in an arc track of length 2 x ROM, hidden inside the joint.
Reverses if: the printed race wears or rattles in Phase 2 (then the 6808-class bearing preset,
same hub and rotor interfaces).

## D-019 (2026-10-25) Long bones are bought pipe, not printed shells

Why: suggested in P-009. Humerus, forearm, femur, tibia, spine and neck are undifferentiated
tubes between printed joint ends: Schedule 40 PVC/ABS in 1", 1-1/4", 1-1/2", 2", cut to length,
in printed sockets (OD + 0.4 mm) with two M3 set screws, tendons and wires inside. Stiffer than a
printed shell, a dollar each, and the print-time budget halves. PEX and vinyl rejected as too
flexible; aluminium tube or conduit is the stiffer upgrade.
Reverses if: the socket set screws loosen under leg loads (then a cross bolt through pipe and
socket).

## D-020 (2026-10-25) Simulation pipeline: skeleton.json -> URDF -> MuJoCo now, Isaac Lab later

Why: asked in P-009. One joint table feeds the Blender rig, the URDF and later the firmware, so
signs and limits cannot drift. MuJoCo loads the URDF directly and already reproduces the
balance doc's push margin (stands at 20 N, falls at 30 N). Isaac Lab is the target for Phase 6b
training, with primo's task layout and microban's walk conventions as references.
Reverses if: nothing planned; the next step inside this decision is MuJoCo spatial tendons.

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

## D-021 (2026-10-26) The simulation carries the drive train per DOF, and the driver's current limit sits below the clutch

Why: asked in P-010 ("will the tensions on the lines and other mechanisms be expressed").
`tools/export_mjcf.py` builds, for every DOF, a DC-curve motor on a gear hinge, a self-lock, a
stick/slip drag clutch, a drum, and the two one-sided elastic runs of the closed loop, from the
same calculator figures as the docs, so tensions, slack and slip are read per step. Building it
showed every winch class had stall torque above its clutch slip torque, so a saturated servo
slipped its own clutch; the firmware therefore limits motor current to 90 % of the slip torque,
and the clutch protects against external overloads only. The controller structure that works is
a position loop on the gear (motor encoder) with the joint sensor trimming the gear target, not
a voltage loop on joint error.
Reverses if: bench tests show a clutch that re-sticks cleanly under motor overload (then the
limit can move up to stall).

## D-022 (2026-10-26) Closed-loop pretension is 25 % of the continuous line force, not 10 %

Why: the loop's stiffness is 2k(n r_p)^2 while both runs are taut and half that once the
unloaded run goes slack. At 10 % the standing ankle load slackened one run and the ankle sagged
8 degrees; at 25 % every leg loop stays taut through standing, 20 N pushes and a quarter squat
(`13`). The cost is 72 N resting in each W-max loop, a tenth of the line's break strength.
Reverses if: creep at the higher rest tension proves faster than the re-home interval absorbs.

## D-023 (2026-10-26) W-mini joints (neck, wrist) get 25 mm pulleys; CAD presets to follow

Why: the chain presets' 10 mm neck and 12 mm wrist lugs give 1-1.3 N m/rad through W-mini lines;
the simulated neck cannot lift the head back from its stop and the wrist exceeds the continuous
rating holding 1 kg. The `--stiff-mini` variant (25 mm, 25 % pretension) fixes both in every test.
The `chain.py` wrist and neck presets have not been regenerated yet; the neck yaw already uses a
25 mm ring rotor. Alternative under this decision: move neck pitch/roll to the ring primitive.
Reverses if: a 2:1 tackle at the existing lug radius tests better than a larger lug.

## D-024 (2026-10-26) Standing balance is a centre-of-pressure controller on the capture point; the ankle is a torque source through its lines; no sagittal hip lean

Why: P-011 asked for the hip strategy toward the gauntlet. Building it in `tools/sim_tendon.py`
showed the previous ankle loop (angle offsets from the CoM error) could not hold a sustained
15 N chest push because its velocity estimate was wrong and it never asked the sole for more
than a fraction of what the toe line allows. The loop that works: the CoP the feet must produce
is (1 + k)(x + v/omega) plus an integral on the CoM offset, clamped 30 mm inside the toe and
heel lines; the ankle torque that puts it there is converted to a gear position through the
measured line stiffness (the lines are the series spring), with a disturbance observer that
turns the CoM acceleration the IMU sees, less what the measured CoP explains, into an external
force, so a push is answered in the first 100 ms. With it the ankle alone holds a sustained
15 N and a 30 N impulse; 40 N needs a step. A hip lean against the push was tried in three
forms (slow lean back, fast lean back, flexion toward the push): every one lifted the heels
through the reaction on the pelvis and fell sooner than the ankle alone, so the sagittal hip
term is in the code with zero gain. The ankle may lock (zero power) only after its torque
demand has been steady for a second; locking and unlocking inside the loop rocked the sole.
Reverses if: the real line stiffness differs from k = EA / (n L) by more than the integral can
absorb (then tension sensing at the ankle, already planned for Phase 6b, closes the torque loop).

## D-025 (2026-10-26) The simulation's collision shapes are the printed envelope, and self-collision is on

Why: P-011 asked whether the segments could pass through each other. The MJCF now carries
capsules (thigh 60 mm, shank 50, upper arm 45, forearm 35, neck 35, head 80 radius) and boxes
(pelvis 280 x 160 x 100, trunk 260 x 160 x 400, hand 80 x 30 x 180 mm) instead of 50 mm sticks,
with MuJoCo's default contact filter (parent and child never collide, everything else does).
Zero body-to-body contacts occur in standing, the squat and an arm bend; the hinges sit at the
bone heads so adjacent segments cannot separate or interpenetrate. Masses are unchanged. The
winch gear and drum discs are visual only.
Reverses if: shell geometry from `chain.py` and `ring.py` replaces the primitives (then the
primitives become the collision proxies of the meshes).

## D-026 (2026-10-26) Lateral weight shift is a hip-roll parallelogram with the ankles following geometrically; feet are grounded by load, and the planner declares the swing foot

Why: the first lateral loop let both ankles run CoP control in double support and oscillated,
swinging 100 N between the feet with a 10 mm CoM motion: two planted legs are a closed chain,
and an ankle roll inside it is a very stiff way to move weight. Now the hip rolls shift the
pelvis (P + I + D on the lateral CoM error, slewed at 0.15 rad/s), the ankle rolls follow with
the opposite angle so the soles stay flat, and only a stance foot in single support runs the
lateral CoP loop. A foot counts as support when its force sensors carry more than 5 % of the
weight (15 % to re-enter) and the planner has not declared it the swing foot. With this the
robot puts 95 % of its weight on one foot with the CoM over that sole; the single-support hold
itself is still open (`14`).
Reverses if: a whole-body (QP) controller replaces the cascade; then the chain is handled by
the optimiser and the ankles can take part again.

## D-027 (2026-10-27) Single support: the stance ankle positions the leg, the stance hip levels the pelvis, one capture-point loop commands both; the swing foot comes up behind

Why: the one-foot hold of `14` fell within a second of the lift. The trace (`15`) showed four
separate causes, each masking the next: the stance ankle's target stepped by the shift angle at
the mode switch; the torque-mode ankle dropped its proportional term and went soft at the lift;
the velocity estimate saw the support reference jump; and the knee-first lift pushed the sole
into the floor. With those removed, the controller that holds is: the stance ankle in position
mode keeping the parallelogram term, the stance hip commanded from the lateral capture point
(1.5 rad/m, 1.0 rad/(m s), 0.5 rad/s slew; sign measured: + stance hip roll moves the CoM left
because the ankle target follows the hip and the leg leans over the foot), the swing foot
folded 10 degrees of hip and 50 of knee with the sole level so it hangs 0.2 m behind the stance
foot, the swing hip at 3 degrees of abduction, a foot leaving support only after 0.15 s unloaded
unless the planner declared it the swing foot, and the disturbance observer at 0.25 s in single
support. Exit test of `06` Phase 5 passed in simulation: 10 s on one foot, CoM within 5 mm of
the sole centre in steady state.
Reverses if: stepping needs lateral authority beyond the sole (then a trunk-roll hip strategy
is added on top, with the heel-lift reaction of `14` to manage), or the real sole's CoP
measurement is too noisy for the 0.25 s observer.

## D-028 (2026-10-27) Hip roll on the 60 mm pulley with tackle 3

Why: in single support the stance hip roll carries 15 to 25 N m (the trunk and the swing leg
hanging off it). On the 50 mm pulley with tackle 2 the loop's pretension range is 14 N m, so one
run is slack 27 % of the hold and the loop is half as stiff whenever it matters; on 60 mm with
tackle 3 the range is 26 N m and the run is slack 7 % of the time, with the same 288 N rating on
the line (232 N peak, SF 3.8). The controller of D-027 holds on either; the pulley is for the
lines staying taut, which is what the tension sensing of Phase 6b will rely on. Cost: a 60 mm
pulley at the hip and one more strand. `export_mjcf.HIP_ROLL_60` is now True; `--hip50`
restores the old geometry for comparison.
Reverses if: the hip housing of `11` cannot take a 60 mm pulley at the roll axis (then tackle 3
on 50 mm, range 22 N m, is the fallback).

## D-029 (2026-10-27) The static step is planned from measurement, with the leading knee as a torque source

Why: the first static step (`16`) fell eleven different ways, and the controller of `14`/`15`
needed only small changes (the swing ankle in position mode, the sagittal ankle torque shared by
the CoM position between the soles, CoP limits from the soles on the ground). The rest is the
planner: every leg pose from geometry (the leg leaning most straight, the others' knees from the
law of cosines) on the feet, CoM and hip positions as measured, the hip offsets held through each
transfer; the leading knee of a staggered stance as a torque source carrying its share of the
weight (a bent knee held in position gives as its lines stretch and the foot flickers on and off
the floor); the trailing foot declared the swing foot once a transfer has unloaded it below a
tenth of the weight; placement in two moves (level to 40 mm above the mark, then down); the arms
held 10 degrees clear of the thighs. The step completes on the baseline and on `--hip50`, with
every leg line inside its rating on the baseline (255 N of 288). No winch class or pulley
changes; on `--hip50` the hip roll reaches 304 N, which confirms D-028.
Reverses if: a learned policy (hub task `policy-training-humanoid-tendon32`) steps more reliably
than the planner, or the real knee's line stiffness is too low for the torque mode to hold a share
of the weight.

## D-030 (2026-10-27) The ground-contact debounce advances once per control step

Why: the debounce timer advanced on every call to `Rig.soles()`, and the balance loop, the
planner, the trace and the recorder all call it, so watching the robot changed what it did; the
same step stood in the console and fell in the recorder. It now advances once per step in
`control()` and every other call reads it. `UNLOAD_T` is 0.075 s, which is what the controller of
`15` was tuned on (0.15 s counted twice a step). The gauntlet is unchanged to the joule except the
one-foot hold (still stands, 27 W to 38 W). On the robot this is the firmware's contact state:
one estimate per control tick, read by everything else.
Reverses if: never; a controller whose state depends on how often it is observed is a bug.

## D-031 (2026-10-27) Learned control trains on a fast stand-in and is judged on the tendon model

Why: the tendon model (`13`) is the truth about lines, slack, clutches and locks but runs well under
real time; PPO needs hundreds of millions of steps. `tools/export_standin.py` makes a stand-in of the
same skeleton with each drive train as one position servo (line-loop stiffness, motor back-EMF damping,
clutch slip torque) that runs at about 190,000 control steps per second on this machine's 24 cores,
and exports per DOF the figures for the same damage gates (line force from joint torque, rating,
pretension). Training happens there, in the sibling project (`feat/standin-rl`); a policy counts only
once it passes on the tendon model (`17`). The policy network is the sibling's model core grown from
its champion, and the hand-written controller of `14` to `16` is a teacher (`tools/record_teacher.py`,
`Rig.applied`): cloning it first was the largest effect in the first ladder (90 % standing under pushes
against 45 % from random weights). The first gate run fails (the cloned policy leans over within 10 s
on the tendon model), so the stand-in has to grow the drum locks and the slack run next.
Reverses if: the gap cannot be closed in the stand-in, in which case training moves to a GPU port of
the tendon model itself (MJX or Isaac Lab with the tendon equalities), or the hand-written controller
stays the controller of record and learning is kept to residual corrections on top of it.

## D-032 (2026-10-28) W-mini presets regenerated: neck pitch/roll 25 mm lugs, wrist 18 mm; both sections widened

Why: D-023's fix, now in the CAD. Measured on the tendon model, holding the head standing still: 10 mm
neck lugs 82 N on a 37 N line, 18 mm 37 N, 25 mm 29 N; a 2:1 tackle at 10 mm gave 28 N standing but fell
in the one-foot hold. The wrist with 1 kg in the hand: 12 mm 78 N, 18 mm 72 N, 25 mm 57 N; no lug size
brings the hand's pitch line under 37 N there, so the wrist takes 18 mm (the size its 66 mm section
allows without growing the hand) and the hand load stays a W-mini rating question. A larger lug needs a
wider tongue for the yaw runs: neck 52 x 42 -> 80 x 52 mm (the neck capsule in the model grows from
35 to 40 mm radius), wrist 58 -> 66 mm wide with a 3 mm shell. Pretension is unchanged (the
`--stiff-mini` variant's 25 % was not needed). Effect on the gauntlet: the neck is within its rating
standing, in the 10 and 20 N pushes, the squat and the weight shift, at it in the arm hold (38 N), above
it in the 30 N push, the sustained push and the one-foot hold (42 to 54 N); the 40 N push now stands; the first static step now falls
(the right foot lands 22 mm long, then the planner places the left 333 mm behind), the same failure the
`--stiff-mini` variant showed, so the swing-leg controller is next on this branch.
Alternative: a 2:1 tackle at the old lug (kept the section, fell in the one-foot hold); neck pitch/roll
on the ring primitive (D-023's alternative), not needed at this load.
Reverses if: the printed neck's mass at real infill exceeds the skeleton's 0.3 kg budget enough to move
the head's CoM, or the bench test of the W-mini line rating comes in well above 37 N.

## D-033 (2026-10-28) The swing leg is placed by world-frame IK on the model's own chain; step changes are judged on a 6-variant grid

Why: placing the swing foot from the planner's geometry relative to the CoM and hips lands it 18 to 29 mm
long, and correcting it from the measured foot fought the balance loop (`16`). IK on a scratch copy of the
model from the low-passed measured pose puts the ankle on a path fixed on the floor; landings come within 0
to 13 mm. Holding the free foot's width in the world as well removes the 3 Hz lateral ring of single support
that the parallel-roll scheme fed. Judging on one deterministic run misled every earlier step change: the
open-loop step stands in 1 to 2 of 6 variants of model and step length, so `tools/step_sweep.py` is the test.
The IK stays off by default (`SWING_IK`) until the step stands on the grid, because the weight transfer, not
the landing, is where most variants fall (`18`).
Alternative: a learned swing-leg policy on the stand-in (docs/17), with this IK as its teacher.
Reverses if: the bench robot's joint sensors and IMU are too noisy for a per-step IK at 1 kHz (then the IK
runs on the planner's model and only the slow pelvis attitude is fed back).

## D-034 (2026-10-28) The drums do not self-lock while the robot steps; the step's transfers come from planned geometry in stages

Why: with locks on, the weight shift rocks the body foot to foot at 1 to 2 Hz (+-33 N per foot): a drum locks
when its joint is still and on target and releases when the balance loop moves the target, a stick-slip cycle
inside the loop. Turned off from the first weight shift to the end of the centring, the ripple drops to
+-5 N, the step becomes repeatable, and with the transfers rebuilt (`19`) it stands in 14 of 14 grid variants
with both feet within 4 mm, no line above its rating, and 444 J instead of 1259 J. Locks stay the standing
hold (zero power once the robot stands still).
Alternative: keep locks and widen their hysteresis or lock only joints whose targets are steady (the ankle
roll at 1 deg of tolerance cut the ripple to +-28 N only).
Reverses if: the bench winch shows real holding current costs far above the model's, so that stepping
unlocked drains the battery (then lock per joint, only the stance leg's unloaded axes), or the real drum
releases without the stick-slip the model has.

## D-035 (2026-10-28) The parts list names the parts the model is built from; the neck and wrist motors are bought by name

Why: asked in P-028 ("Are there specific parts that can be recommended"). The line ratings and margins of
docs 13 to 19 come from `tendon_calc.py`'s motor figures, so `05` now names a part per drive with those
figures: the neck and wrist W-minis are Pololu 298:1 HP 6V (the model's motor; no-name N20s deliver 60 to
70 % of it, hobby encoder N20s far less), the fingers and face stay 100:1 of any brand (D-005), the W-max
775 must be rated about 12 000 rpm at 12 V. Prices checked against live listings put the full body at
about $1,940 (the 2026-10-25 estimate was $1,070).
Alternative: every W-mini a no-name N20 (~$1,750), with the neck and wrist re-rated in the model first.
Reverses if: the Phase 0/1 bench test measures a no-name 298:1 N20 at the model's stall torque (then buy
budget motors throughout), or a part in the model changes (then its row in `05` changes in the same commit).

## D-036 (2026-10-28) The mass is rolled up from the parts, not budgeted; the 20 kg model stays the baseline until mass comes off

Why: hub task `printed-mass-rollup` (P-031). `tools/mass_rollup.py` adds every generated part at print
settings, the parts with no generator yet and the bought parts' listed weights: about 26 kg against the 20 kg
budget, half of it the 21 W-max drives (12.4 kg). On those masses (`ROBOT_MASSES`) the model stands through
the gauntlet, but the one-foot hold's stance ankle line peaks at 306 of 288 N and its clutch slips, and the
step stands in 11 of 14 grid variants (14 of 14 at 20 kg). The controllers were tuned at 20 kg, so the
committed model keeps `skeleton.json`'s masses; every change to the legs or the balance is also run with
`ROBOT_MASSES` on the roll-up, and the mass budget is now the roll-up's total.
Alternative: switch the model to the rolled-up masses now and re-tune the step and the hold on them.
Reverses if: mass comes off to about 22 kg or less (then the model moves to the roll-up), or the parts are
built and weighed.

## D-037 (2026-10-28) The landed leg's press is found from the foot's load, not set

Why: hub task `step-mass-robust` (P-035). A fixed 20 mm press on landing was right only for the 20 kg model:
on the rolled-up masses (`20`) it kicked the landed foot back up and rolled the body over the stance foot
(11 and 2 of 14 grid variants), and a zero press broke the 20 kg model. The transfer now settles the landing
first, deepening the press while the landed foot carries less than 15 % of the weight and holding it once
the foot is loaded and the CoM still. The step stands in 42 of 42 variants (2 models x 7 lengths x 3 mass
sets), landings within 4 mm; the rest of the gauntlet is unchanged.
Alternative: a press scaled by the measured stance-leg sag, or a press per mass set.
Reverses if: the real foot's load sensor (FSRs, docs/07) is too noisy or slow to close this loop; then the
press from the measured hip height above the stance ankle.

## D-038 (2026-10-28) The drums lock only when the whole robot is still, and every lock releases on a push

Why: hub task `locks-off-while-balancing` (P-036). D-034's finding holds beyond the step: the per-drum lock
rule hurts any active balancing. With locks as they were the hand-written controller stood 184 of 200 random
pushes; with none, 199. Now no locks while the weight shift and the one-foot hold run, and a whole-body gate
(`LOCK_STILL`): a drum may lock only after the CoM has moved under 20 mm/s for 0.5 s, and every lock releases
once it moves over 80 mm/s. 199 of 200 pushes stand, every gauntlet scene stands, energy is lower in 9 of 11
scenes (the one-foot hold 562 J against 811; standing 12 J against 21), the step grid stays 42 of 42. The
arm-with-a-load scene costs more (162 J against 114): unlocked, the balance loop sways under the load.
Alternative: the gate on the legs only (197 of 200) or on all but the arms (198 of 200).
Reverses if: the real drums' hold current or lock wear makes frequent release costly, or the IMU-based CoM
speed is too noisy to gate on.

## D-039 (2026-10-28) A transfer starts from the crouch the stance leg has

Why: hub task `gait-cycle-static` (P-037). The walk generalises the step (`take_step(r, n_steps)`); from the
second step on the stance leg is crouched when the transfer begins (TP_KEEP_CROUCH), and the transfer's settle
planned it straight, so it straightened at the plan's slew rate, threw the body and kicked the new foot off
the floor. The walk stood in 16 of 42 grid variants (2 of 28 on the rolled-up masses). Starting the transfer
from the stance leg's crouch (none after the first lift) makes it 42 of 42, landings within 6 mm; the single
step is unchanged.
Alternative tried: the CoM further forward after each transfer, a deeper crouch, the stance hip sinking while a
straight swing leg cannot reach the floor (all worse or no better; flags off).
Reverses if: a gait without the kept crouch (straight-legged stance) is wanted; then the swing needs the reach
another way.

## D-040 (2026-10-29) No bench tests before buying: motor figures are sourced ranges and the design must hold across them

Why: the owner will not buy motors until the outcome is reasonably certain and cannot run the bench tests
(P-038). So the figures the model runs on (stall torque, speed, current, gearbox efficiency, line strength)
come from sources (datasheets, the sellers' curves, reference projects that published measurements, the
physics of the motor's own numbers), each with a low and a high value and where it came from, and a build is
only recommended if the walk, the one-foot hold and the push set stand at the pessimistic end of its range.
The bench tests (`bench-load-tests`) become checks after purchase, not gates before it.
Alternative: buy one of each motor and measure (the owner declined: cost and time before the design is sure).
Reverses if: the owner gets the parts in hand; then the measured values replace the ranges.

## D-041 (2026-10-29) A motor case moves the parts, not the settings; lines are anchored by wraps, not knots

Why: docs/24 (P-038). With `ROBOT_MOTORS=low|nom|high` the model runs every drive and line at one end of its
sourced range. What the builder sets stays put across the cases: the pretension (barrel adjuster, from the model's
continuous rating), the clutch's slip force (a third of the spool label, or 0.95 of the motor's stall) and the
line stiffness. So a weaker line shows as a lower safety factor at the same clutch, and a weaker motor as less
margin over it, instead of the model quietly re-tuning itself. And since a knot carrying the load keeps 0.3 to
0.75 of the label (Instron knot tests) and a crimp on bare braid slips, every line is wrapped at least three turns
on the drum and on the joint pulley before its crimp; the line's effective strength is then 0.72 / 0.90 / 1.10 of
the label, never under 2.2 times the clutch force.
Alternative: the clutch and pretension re-derived from each case's figures (hides the weak-line case); knots or
bare crimps (at 0.3 of label the line breaks before the clutch slips).
Reverses if: a bench check after purchase shows the wrapped anchor slipping or the braid failing on the drum
before the knot figures would.

## D-042 (2026-10-29) Wind is MuJoCo's drag on each body; a sustained force is carried by leaning into it

Why: hub task `wind-gauntlet` (P-038, docs/25). Wind is `opt.density` 1.2 and `opt.wind` set every step (the
inertia-box drag of each body's shell: 26 N at 10 m/s head-on, CdA 0.43 m^2, centre of drag 0.95 m up), the mean
plus Ornstein-Uhlenbeck gusts. Standing, the robot fell at 10 m/s head-on (8 stood) for two reasons, fixed:
(1) a wind building up slowly keeps the body still, so the D-038 gate locked every drum and the locked ankles
carried a growing load until the body tipped; the gate now also resets when the disturbance estimate has moved
20 mm since the body went still (`LOCK_RELEASE_D`). (2) The balance loop held the CoM over the sole centre and
carried the force on the CoP alone (about 21 N, 9 m/s); it now leans (`LEAN`): the slow part of the disturbance
estimate (`LEAN_TAU` 0.4 s, lateral `LEAN_TAU_X` 1.5 s, beyond a 20 mm dead band, at most 80 mm) moves the
ankles' planned angle and the lateral shift's target, so the CoP returns toward the sole centre and gravity
carries the force. The capture-point integral alone could not do it: the ankle is a spring about its planned
angle and the integral can hold only ~25 N m of it off the plan. The ankle part fades out in a staggered stance
(`LEAN_STAGGER`), where tilting both ankles fights the legs.
Alternative tried: the lean through the integral alone, faster integral gains (no change); no dead band (the step
and pushes cost 10 to 20 % more); a 0.4 s lateral lean (192 of 200 random pushes).
Reverses if: a learned controller carries wind better, or a stepping strategy (a capture step downwind) replaces
the lean for gusts beyond the sole.

## D-043 (2026-10-29) Contact creep is a model artifact: the step and the walk are judged with the feet held

Why: P-039 (docs/25). MuJoCo's soft contact lets a loaded foot slide slowly under a sustained side force, at any
friction: in a 10 m/s head wind both feet slid 78 mm in 20 s at 16 % of a friction of 1.0, which a real sole does
not do. With the no-slip solver (`NOSLIP=5`, 1.6 mm) the gauntlet and the standing wind map are unchanged, seed by
seed, but the step grid stands 23 of 42 and the walk 16 of 42: the transfer relied on the feet
creeping apart. Until the transfer is fixed, the default stays the soft contact and its results stand with this
caveat. A step or walk change from here on is also run with `NOSLIP=5` and `FLOOR_MU=0.4` (a hard floor's
friction). Once the step passes those grids, `NOSLIP=5` becomes the default.
Alternative: an elliptic cone with impratio 10 (5 mm of creep, no extra cost), kept as a fallback if no-slip is
too slow for the teacher data. A compliant sole in the model (a few mm of elastic shear) is the physical middle
and may replace both.
Reverses if: a printed sole on a bench shows centimetres of creep under a steady side load (it should not).

## D-044 (2026-10-29) Silicone pad feet, a 1 kHz balance loop, and bowing as the next wind strategy

Why: P-040 (docs/26). (1) Feet: an adhesive silicone non-slip pad (1 mm, bonded with 3M 9448A on its adhesive
side) on every sole, friction low / nom / high 0.5 / 0.9 / 1.2 (sourced from rubber figures; no silicone figure
found). It shears by microns, so the held-feet model (`NOSLIP=5`, D-043) is the right one. (2) Sensors: on
modelled sensors (`tools/sensor_sweep.py`) the wind is felt as well as from the simulator's truth; what breaks is
latency: 10 ms loses the 40 N push, 5 ms stands. The balance loop runs at about 1 kHz on the leg nodes, IMU and
foot loads to motor command within 5 ms (docs/04's 200 Hz loop stays for the joints). Calibrated FSRs and load
cells score the same; uncalibrated FSRs (15 %) stop the step at the lift gate. (3) Wind: bowing 35 deg into a
head wind raises standing from 12 to 14 m/s steady and 10 to 12 with gusts, if begun within about a second of
the lean nearing its limit and done in 0.5 s; squatting, folding the arms, turning the hands and twisting do not
help. A bow reflex is the next wind work; nothing posture-based helps from the side.
Alternative: FSR 402s alone (their 20 N range needs a load-sharing pad and per-sensor calibration); a load cell
per corner is the safer part, decided after the step works on held feet.
Reverses if: the pad measures under 0.5 on a ramp, or the balance loop cannot close in 5 ms on the ESP32.

## D-045 (2026-10-29) One main pack in the pelvis; the logic on its own supply; energy per metre judges the W-max

Why: P-041 (docs/27). On the sourced motor figures the 26 kg build walks at 59 W of motor power (peaks 63 A),
standing 14 W; a 111 Wh pack walks about 18 m in 1.2 h, and wind up to what the walk survives changes that by
under 10 %. Nearly all of it is the W-max holding the body through the 775's 66 A stall current. USB power banks
cannot feed the drives (5 V, 1 to 3 A, low-load cut-off), store less per kg than the LiPo, and in the legs add
swing mass on the weakest drive. So: the main pack stays in the pelvis (a second pack or 21700 cells in the spine
pipe if more is needed); the 5 V logic gets its own supply (a buck with hold-up capacitance, or one bank with an
always-on mode) so a 63 A peak cannot brown out the balance loop; and the W-max choice (`wmax-stands-at-low`) is
also judged on walking energy per metre at the sourced nominal. More ESP32 nodes cost about 0.3 W each and are
not a constraint.
Alternative: banks distributed in the bones (rejected above); a larger pack (linear runtime, but its mass costs
energy per metre and W-max margin).
Reverses if: the drives move to a self-locking or higher-ratio stage that holds the stance without current,
making the electronics the main load.

## D-046 (2026-10-31) The transfer has no hold after its sagittal stage

Why: P-045 (docs/27, docs/23). The 1 s hold after the transfer's sagittal stage (`TP_HOLD`) is not needed. Without
it, the step grid and the walk grid stand 42 of 42 on soft contact, and the held-feet walk stands 21 against 23,
within that grid's scatter (11 to 24 across P-045's sweep). The walk takes 92 s instead of 99, and its energy on
the 26 kg build at the sourced nominal drops 4 % (5.90 to 5.68 kJ, 14.7 to 14.2 kJ/m).
Alternative: the other stages shorter (`TP_A` 2 s stands 39, `TP_B` 2 s 41); `COM_REF_BACK` (the CoM target toward
the ankle) saves a third of standing power but drops the arm hold and the walk grid.
Reverses if: a gated transfer needs a settled CoM between its stages.

## D-047 (2026-11-01) Tackles stay at 2:1 and 3:1; no 6:1 block

Why: P-046 (docs/27). The owner asked about a 6:1 hoist. The joints already use 2:1 and 3:1 tackles. A 4:1 ankle
pitch cuts the 26 kg walk's energy by 6 % and keeps both grids at 42 of 42, but the gauntlet's 1 kg arm hold then
falls: the slower ankle cannot catch the lean. 6:1 saves no more than 4:1 and loses two walk variants. A 6:1 hip
roll halves the balance loop's roll speed, and the walk falls. Real sheave friction (about 88 % through a 6-strand
block, not modelled) and n times the line travel make it worse.
Alternative: a 4:1 ankle with the arm hold retuned; `TACKLE_OVERRIDE` keeps that one setting away.
Reverses if: a faster ankle drive or a retuned arm hold makes 4:1 pass the gauntlet.

## D-048 (2026-11-02) Ceramic ring guides at bends and mouths; needles stay in straight runs; sheaves stay in tackles

Why: P-048 (docs/02). Fishing-rod guide rings and textile thread-guide eyelets are polished alumina or SiC made
for braided line rubbing at an angle, at about $0.15 each in repair kits. A needle liner is a tube for straight
runs, and the line's bends today sit on printed countersinks and PTFE ends, which are the softest surfaces it
rubs. A ring is a fixed surface, so it pays the capstan loss on every degree of wrap: fine to about 30 to 45
degrees, but in a 180-degree tackle wrap it would lose a quarter to half of the force against a sheave's 3 to 5 %.
Provisional until `mu` for braid on ceramic is measured.
Alternative: needles and PTFE only (D-013 as it was); sheaves at every bend (heavier, more parts).
Reverses if: the capstan test finds `mu` for braid on ceramic above about 0.25, or pressed rings work loose in
PETG.

## D-049 (2026-11-03) A carried bag hangs on a passive palm hook or forearm strap, not the finger grip

Why: P-049 (docs/28). With the planned 100:1 finger motors, a hook grip on a bag handle holds about 2.4 kg
continuously: each flexor pulls 0.55 times the bag's weight through its 6.5 mm moment arm at the PIP. So bread and
eggs are at the edge, and a gallon of milk is past it. The arm and legs carry more: lifted over 1.5 s, the 20 kg
build stands with 11.3 kg in one hand, and walks with 3.9 kg (a gallon of milk). A printed J-hook at the heel of
the palm, or a strap loop on the forearm, carries the bag at no power, and the hand only places the handle.
Alternative: the 298:1 grip motor in the fingers (about 6.9 kg continuous, but a 1.6 s close).
Reverses if: the fingers get a lock (a ratchet or a self-locking stage) that holds a closed grip without current.

## D-050 (2026-11-03) The stand-in carries the drive's cascade, fitted to the tendon model; the learned stand policy is not used yet

Why: P-050 (docs/29). On step responses the old stand-in's joints were slow and soft (a held knee reached 16 % of a
step in 1 s, the tendon model 91 %): its damping was the motor's back-EMF at the joint, and it had no integral
action, where the firmware's outer loop trims out the line stretch. The stand-in now has 0.1 of that damping, a
30 ms target lag and the outer loop (`outer_loop` in its JSON, run by the training environment), fitted on six
joints (error 0.84 against 2.41). Retrained on three seeds, policies on either stand-in stand 10 s on the tendon
model; the refit's gain on the gate is within the seed spread. No policy passes the 30 N push and the 8 m/s wind
together, and they use 50 to 130 W standing still against 3.6 W, so the hand-written balance stays.
Alternative: keep the plain servo (simpler, and as good on the gate so far); fine-tune on the tendon model.
Reverses if: a policy passes the whole gate within the hand-written controller's power, on more than one seed.

## D-051 (2026-11-04) Learned control order: settle whether the sibling's core helps, then rests and locks, then other bodies

Why: P-051 (docs/30). The gains so far come from cloning the hand-written controller; the sibling's core beat
random weights of its own architecture on one seed, and there is no plain-network baseline. Rests and locks are
the furthest gap (15 to 35 times the power), but under a policy the tendon model's stillness gate never runs, so
no lock can engage in the gate; that is fixed before any training for it. Cross-training on other MuJoCo bodies
(only RPO so far, one seed) is worth its cost only if the core helps. Order: sibling #51 with a plain-network arm;
rests and locks (the gate's stillness and power first, then the stand-in's power and lock, then three seeds); #50;
#49; multi-body training. The sibling's voxel-world issues (#8, #9, #13, #16, #17, #18) are parked.
Alternative: rests and locks first (the user's first idea); the seed study first is cheaper and decides whether
the rest of the sibling work is worth doing on this robot.
Reverses if: #51 shows no difference between the core and a plain network, then the robot's learned control uses
a plain network and the sibling's work for this robot stops at what it already gave.

## D-052 (2026-11-04) The learned stand policy is a plain network; rest is trained with held loads; the drums may lock under a policy

Why: P-052 (docs/31). Over five seeds on the tendon gate, a plain MLP of the core's size beats the sibling's grown
champion core in every arm (cloned: 22 against 17 of 30 named scenes, 98 against 89 of 100 pushes; uncloned: 18
against 0). D-051's reverse condition is met, harder: the robot's learned control uses a plain network, trained
with the sibling's `rl/` trainer; the sibling's core and pretraining stop at what they gave (the trainer, the
gate, RPO transfer). `Rig.still_gate` lets the drums lock under any controller, as the firmware's gate would, and
the gate counts power. Training the fast stand-in to rest failed (exploration noise keeps the drums unlocked;
less noise does not transfer); training with 0 to 4 kg in one hand cut median idle power from 175 W to 38 W (best
10.6 W) and is kept. The hand-written balance stays: no learned policy holds the 8 m/s wind or rests within 2
times its power.
A plain GRU of the same size does as well as the MLP (20 of 30, 100 of 100), so memory is not what the core adds.
Alternative: keep the core for its RPO transfer (a reliable head start on a new body).
Reverses if: a core (or a recurrent network) beats the plain network on the gate over five seeds.

## D-053 (2026-11-04) Getting up is a dynamic rise from a half-kneel; the 8 m/s wind waits for the ankle's torque mode in the policy

Why: P-052 (docs/32). With the ankle's 20 degrees of dorsiflexion no one-leg pose puts the CoM over a flat foot below a
pelvis height of 0.85 m (`tools/rise_static.py`); more ankle range helps only to 0.80 m before the knee and hip drives
run out. A two-knee kneel does not rest; one half-kneel does. A learned lunge from it rises 61 % of the time in the
stand-in, with the forearm lines at 2.3 times their rating. For the wind: drag agrees between the models, widening
the action range and an all-windy tendon fine-tune both failed; the ankle line's stretch under a steady
lean, which the hand-written controller handles in torque mode and the policies do not have, is part of it: with the
hand-written ankles under them, 2 of 10 policies hold the 8 m/s wind (0 alone) and 9 of 10 the 5 m/s (7 alone);
trained with their own ankles in torque mode, 0 of 10. The policies miss the balance loop's ankle strategy, not
its drive mode. As a hybrid (the hand-written ankle balance, the policy on every other joint) two plain policies pass
all six named scenes, at the cost of push recovery (77 of 100 random pushes against 98 alone).
Alternative: more ankle dorsiflexion (a geometry change in the shank-foot joint) for a slower rise; hands in the
get-up.
Reverses if: a policy holds the 8 m/s wind without the hand-written ankles, or a slow rise is found with the hands.
