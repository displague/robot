# Releases

Each tag has a GitHub release (github.com/displague/robot/releases) with notes, visuals and assets:
the generated STLs, the MuJoCo models and the recordings. How to make one: `.claude/skills/release/SKILL.md`.

## v0.7.0 (2026-11-13) — A helper, a second robot, ears, and a regression gate

**State:** the robot is helped, and helps. A scripted helper holds its wrists on the 0.35 m chair (docs/44, D-070) and
its upper arms on a get-up from a half-kneel, the first on the tendon model (docs/46, D-073). The chair rise's
parameters are trained by search under jitter, and the low-seat set is the default below 0.42 m, so the whole chair
grid stands (docs/45, D-071, D-072). Two robots share one model, and one takes the other's arms at the stand-up
(docs/47, docs/48, D-074 to D-076). Both models have ears, and the sibling's v0.5.0 trains its get-up with a coach:
a voice through those ears and a helper's hands that fade (docs/49, D-077). `tools/regress.py` is a regression gate;
it found and fixed a reset that left the drum locks off (D-078). Also: senses as a research topic (docs/42) and the
training rounds plan (docs/43, D-069).

**Results:** gauntlet 14 of 14, each scene now the same alone or in sequence (chair 835.7 J either way); 0.35 m chair
23 of 24 under 2% jitter; get-up from a half-kneel with the scripted helper 24 of 24, every arm line under 0.9x;
with a yielding helper capped at 60 N a side 16 of 24; two robots, the helper robot taking over at the stand-up, 19
of 24 (7 of 24 with it standing by). In the sibling, three seeds: unaided rise 46% after an early-fade helper against
52% with none; a spoken command is not obeyed.

**Known gaps:** the get-up needs the helper (not at 30 N a side, not withdrawn); the scripted hands still catch the
kneel and carry the rise for the robot helper; an earlier takeover fails more often (10, 4, 0 of 24); the robots end
chest to chest; the helper robot's spine has no margin (clutch at 1.03 of its rating); the voice is unused and not
obeyed; nothing is printed and no number here has met hardware.

**Assets:** robot-v0.7.0-stl.zip, robot.xml, robot_standin.xml/.json, baseline.json, getup_params.json,
duo_params.json, rise_params.json, duo.gif and .mp4, getup.gif, rise35.gif, helper35.gif, coach-arms.png.

**Decisions:** D-069 to D-078. **Prompts:** P-066 to P-075.

## v0.6.0 (2026-11-09) — Chained skills: a bag carried through a walk and a chair

**State:** skills run back to back in random orders with no reset (docs/38, D-062): pick a bag up from a stool, walk,
sit on a 0.45 m chair and stand again, put the bag down. Walking with a bag in hand works once the grip damps the
bag's swing and the balance loop feels its weight (docs/39, D-063). The planner sets a bag down before a sit and
takes it up after (D-065). The chair no longer slips the neck (two strands, D-066) or the spine (the hips paced at
22.5 deg/s, D-067, docs/41). The landing press starts from the landed leg's length, which gives the heavier
layouts their held-feet step (docs/40, D-064). Also since v0.5.0: the brushless W-max as a model variant (D-058,
docs/34), the torso and pelvis as keyed thin shells (D-059, docs/35), and where the mass sits (D-060, D-061,
docs/36, docs/37). The docs are a site, https://displague.github.io/robot/, indexed from their front matter (D-068).

**Results:** gauntlet 14 of 14 (step 533 J, walk 1130 J, chair 842 J, was 1025); chain grid 17 to 18 of 24 (10
before D-063); one hand walks about 2.5 kg, two about 4.5 kg split; the 775 layout's held step 0 to 14 of 14; the 20
kg model on the pessimistic motors: step 7 to 13 of 14, walk 3 to 12; every chair case stands with no slip, on 10
to 48% less energy.

**Known gaps:** the heavier layouts' held walk (4 of 14); bags over about 2.5 kg in one hand (all six chain failures
left); the two-hand pick-up and put-down (`chain.SPLIT`, off); the 0.40 m chair on the layouts; the brushless
layout's lurch at its first release; still nothing printed.

**Assets:** robot-v0.6.0-stl.zip, robot.xml, robot_standin.xml/.json, motor_ranges.json, chain.gif and .mp4.

**Decisions:** D-058 to D-068. **Prompts:** P-058 to P-065.

## v0.5.0 (2026-11-05) — Public, licensed, and sitting on a chair

**State:** the repository is public (D-054), code MIT and the printables CC BY-SA 4.0 (D-055). The robot sits on a
chair and stands again (docs/33, D-056, D-057): on the 20 kg model from 0.40, 0.45 and 0.50 m seats with or without
armrests, on the nominal and the pessimistic motors (0.50 m without armrests lost on the weak ones); never from
0.35 m. The forward reach (the hip's 110 degrees, the ankle's 20 degrees and the spine's drive) is the limit, so the
rise is a trunk swing then a knee-led extension gated on the CoM. The 26 kg build stands again only from 0.50 m.

**Results:** gauntlet 14 of 14 (the chair at 0.45 m added; the other 13 unchanged to the joule); the chair grid above;
seated 17 to 44 W; 10 of 12 nudged rises pass.

**Known gaps:** no 0.35 m rise; the hands cannot use the armrests; the 26 kg build below 0.50 m; the neck's drive slips
through the swing; the seated rest draws power (the elbows); hip lines at a safety factor of 3.2 to 3.5.

**Assets:** robot-v0.5.0-stl.zip, robot.xml, robot_standin.xml/.json, motor_ranges.json, sit40.gif and .mp4.

**Decisions:** D-054 to D-057. **Prompts:** P-054 to P-057.

## v0.4.0 (2026-11-04) — Wind, energy, a bag, and learned control judged on the tendon model

**State:** the model has wind (MuJoCo drag on every body, gusts from any side; the robot leans into it, standing
12 m/s head-on; D-042, docs/25), sourced low / nominal / high figures for every drive and line (D-040, D-041,
docs/24), its fidelity checked (docs/26: a 1 kHz, 5 ms balance loop on modelled sensors), an energy account (about
18 m per charge on the 26 kg build; docs/27), tackles and ring guides settled (D-047, D-048), and a grocery bag
(standing to 11.3 kg, walking to 3.9 kg; palm hook, D-049, docs/28). Learned control is judged on the full tendon
model: a plain network beats the sibling's core (D-052, docs/31); policies rest with held loads; only a hybrid
(the hand-written ankle balance under a policy) holds the 8 m/s wind; getting up is a dynamic rise from a
half-kneel (D-053, docs/32). The hand-written controller stays the robot's controller.

**Results:** gauntlet 13 of 13 (the walk 88 s, 1053 J; the arm scene 238 J, worse); standing in wind 12 m/s head-on, 10 m/s side; the walk 3 m/s any side;
learned stand: plain MLP 22 of 30 named gate scenes over five seeds (grown core 17), hybrid 2 of 5 seeds all six
including the 8 m/s wind; best learned idle 10.6 W (hand-written 5.4 W); get-up 61 % from a half-kneel in the
stand-in; no slow rise below a pelvis height of 0.90 m.

**Known gaps:** no learned policy alone holds the 8 m/s wind; the hybrid costs push recovery (77 of 100) and runs
some lines over their continuous rating; get-up from a fall 0 %, untested on the tendon model, arm lines at 2.3
times their rating; the walk is the wind weak point; still 26 kg against 20 and nothing printed.

**Assets:** robot-v0.4.0-stl.zip, robot.xml, robot_standin.xml/.json, motor_ranges.json, gust.gif,
wind8-alone.gif, wind8-hybrid.gif and their MP4s.

**Decisions:** D-040 to D-053. **Prompts:** P-038 (continued) to P-053.

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
