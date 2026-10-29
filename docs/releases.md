# Releases

Each tag has a GitHub release (github.com/displague/robot/releases) with notes, visuals and assets:
the generated STLs, the MuJoCo models and the recordings. How to make one: `.claude/skills/release/SKILL.md`.

## v0.3.0 (2026-10-29) — A static walk

**State:** the robot walks in simulation: four alternating 100 mm steps and a closing step, standing in 42 of
42 variants of a grid over two drive models, seven step lengths (85 to 115 mm) and three mass sets (20 kg and
the 25 and 26 kg roll-ups), every landing within 6 mm (D-039, docs/23). The step's landing press is found
from the foot's load (D-037); the drum locks engage only when the whole body is still (D-038, docs/22). The
mass is rolled up from the parts (docs/20, 26.1 kg as specified) and the W-max options costed (docs/21). The
parts list carries Amazon prices (docs/05).

**Results:** walk grid 42/42 (93 s, 1113 J, no flags at 20 kg); step grid 42/42 over three mass sets (27/42
before D-037); random pushes 199/200 (184); one-foot hold 562 J (811); standing 12 J (21); the arm scene
162 J (114, worse).

**Known gaps:** slow (16 s per step); the arm scene's sway; 26 kg against a 20 kg budget and the motor
undecided; the 775's catalogue figures inconsistent; no wind, slopes or turning; three STLs non-manifold
(thumb_palm, forearm_6808_rotor, shoulder_rot_rotor).

**Assets:** robot-v0.3.0-stl.zip, robot.xml, robot_standin.xml/.json, rollup_775.json, rollup_worm.json,
walk.gif, walk.mp4.

**Decisions:** D-035 to D-039. **Prompts:** P-028 to P-038.

## v0.2.0 (2026-10-28) — A static step with margin

**State:** the first static step stands in every variant of a 14-run grid (two models, step lengths 85 to
115 mm; 50 to 150 mm also stand), both feet within 4 mm of their marks, no drive line above its rating, at
444 J. The swing leg is placed by world-frame IK on the model's own chain (D-033, docs/18); the weight
transfers come from planned geometry in stages with the shift fed forward, and the drums stay unlocked while
stepping (D-034, docs/19). The neck and wrist got 25 mm and 18 mm pulleys (D-032), which brings the neck
inside its rating. The repo carries its own skills (CLAUDE.md, .claude/skills) and a render tool.

**Results:** step 14/14 on the grid (v0.1.0's step: 2/14); landings 2 to 4 mm (was 18 to 29 mm long); worst
leg line 186 of 288 N (was 255); 444 J at 10 W (was 1259 J at 39 W); neck 29 N standing (was 82 N on 37);
the 40 N push now stands; the rest of the gauntlet unchanged.

**Known gaps:** one step only (no gait cycle yet); 44 s per step; the one-foot hold and the teacher data still
run with locks on; 170 mm steps time out; the learned controller's gaps (sibling #48-#51).

**Assets:** robot-v0.2.0-stl.zip, robot.xml, robot_standin.xml/.json, step-ik.gif, gallery.png, chain_neck.png.

**Decisions:** D-032 to D-034. **Prompts:** P-026, P-027.

## v0.1.0 (2026-10-28) — The tendon humanoid in simulation: drive trains, balance, one foot, first step

**State:** the design (docs 01 to 08), the parametric CAD (knuckle chain, ring joint, winches, finger,
skeleton rig), the tendon-level MuJoCo model with every drive-train element (docs 13), the balance
controller (30 N pushes, sustained 15 N, arm hold, squat, weight shift; docs 14), the one-foot hold
(Phase 5 exit test in simulation; docs 15), the first static step (Phase 6a; docs 16), the fast stand-in
and teacher demonstrations, and the first learned controller in the sibling project's v0.3.0 (docs 17).

**Results:** stand and pushes to 30 N pass; 40 N falls; one-foot hold passes; the static step completes
with the feet 18 and 29 mm long; the hand-written controller stands 177 of 200 random pushes; the learned
policy stands 90 % on the stand-in and fails the tendon gate.

**Known gaps:** the neck over its rating (fixed in D-032 after this tag), landing accuracy, step speed,
the learned controller's stand-in gap.

**Assets:** robot-v0.1.0-stl.zip, robot.xml, robot_standin.xml/.json, gauntlet.gif, onefoot.gif, step.gif.

**Decisions:** D-001 to D-031. **Prompts:** P-001 to P-025.
