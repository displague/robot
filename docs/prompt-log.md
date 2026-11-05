# Prompt log

Every set of work in this repository traces back to a prompt. Each entry records the prompt
verbatim, the date, and the commits or files it produced, so the reasoning behind a design can
be re-read later in the words that started it.

## P-001 (2026-10-25) Kickoff

> we are going to design a robot, full size humanoid, that can be 3d printed and easily
> hand-assembled - operated under 1-3 lightweight compute devices like pi5 or rk1 (even better is
> esp32 is sufficient). For the tendons and muscles, we want to take advantage of off the shelf
> fishing line and sewing inspired spindles (and gears?) such that tension can be locked and
> dispensed as needed through spin, with great strength in tiny space. the spindles themselves
> may be mostly 3d printable the rotors may also be something we consider operating from a
> central multiple line tensioner, with all the lines fished to all phalanges and flexible
> locations. we are looking for human parity in range of mobility. blender is available so that
> we can ensure the parts function and so that we can test loads of stress. we are aiming for the
> simplest, cheapest, full humanoid which can be printed and assembled at home

Produced: `README.md`, `docs/01-architecture.md` through `docs/06-roadmap.md`,
`docs/decisions.md` (D-001 to D-010), `tools/tendon_calc.py`, `cad/blender/winch_mini.py`,
`cad/blender/README.md`, the first W-mini winch STLs and render.

Key outcomes: tendon winches in regional banks rather than a central clutch tensioner (D-003);
braided PE line in PTFE Bowden tube (D-002); ROM parity with underactuated hands (D-004); N20
without a worm for hands because the worm halves the power (D-005); joint-side magnetic angle
sensing (D-008); Blender for kinematics, FreeCAD FEM for stress because Blender has no solver
(D-009); big-joint motor class left open (D-006) after the calculator showed cheap gearmotors
have the force but not the power.

## P-002 (2026-10-25) Commit, publish, walking and balancing

> detailed initial commit (capture the prompts leading to each set or workand commit) and push to
> a private repo on displague/robot. walking and balancing are goals. continue.

Produced, across four commits:
1. The initial commit of all P-001 work and the private repository `displague/robot`.
2. Walking and balancing promoted from stretch goal to requirement: `docs/07-locomotion.md`,
   roadmap Phase 5 / 6a / 6b with exit tests, D-006 set provisionally to 775 + printed planetary
   (wiper motor retired for legs), D-011, D-012, four walking scenarios in the calculator, this log.
3. `cad/blender/finger.py`: the Phase 0 finger test article (palm block + three phalanges, pins,
   flexor and extensor channels, 31 mm excursion) with renders in `docs/03-skeleton-joints.md`.
4. `cad/skeleton.json` (whole-body segments, joints, limits, winch classes) and
   `cad/blender/skeleton_rig.py` (armature with limit constraints, DOF tally = 50, sign-check pose
   verified numerically, `skeleton.blend` for hand posing).

Deferred from the "continue": the closed-loop two-anchor drum, the W-mid and W-max winch
generators, and the firmware skeleton. Next in line per the roadmap.

## P-003 (2026-10-25) Hollow needles as insertable tendon liners

> If we need hollow threading needles anywhere, ideally insertable and/or snapped/locked into the
> 3d printed cavity without needing to be added midprint via pauses, these are cheap to acquire
> and cut to size, such as body piercing needles which are hollow end to end and 100 can be
> acquired for ~$13.

Produced: needle-liner section with a gauge table and usage rules in `docs/02-tendon-actuator.md`;
the no-mid-print-inserts rule in `docs/03-skeleton-joints.md`; D-013; BOM line; `finger.py`
flexor channel sized as a 16 G press fit (1.75 mm) with countersunk mouths on both channels at
every body face.

## P-004 (2026-10-25) Opposability, backstops, terminations

> How is forward and reverse opposability attained? Is it a function of bidirectional feeding,
> each line coming back to it source from the extremity on a loop?
> Are there backstops? Use of particular knots? Beads?

Produced: "Direction, return and termination" in `docs/02-tendon-actuator.md` (the three
reverse-stroke patterns, the closed loop in detail with its equal-moment-arm and both-runs-sheathed
rules, barrel-adjuster pretension, backstops, and a termination table covering stopper knots,
Palomar, crimp sleeves, crimped stop beads and needle toggles); D-014.

## P-005 (2026-10-25) Passive balance, springs, compliance

> Can balance be attained naturally, a balast/ bladder perhaps?
> For tensions and rest positions are spools or winding springs used or needed?
> Is the design soft and compliant?

Produced: `docs/08-passive-balance-springs-compliance.md` (statue tipping margin numbers, foot
length and foot ballast as the passive levers, why not a bladder, reaction wheel as a deferred
option; the four places springs are used and why; the compliance inventory, the non-backdrivable
lock and its three safeguards); D-015, D-016; README map row.

## P-006 (2026-10-25) Readiness check for a full Blender assembly

> Is the whole system ready to be assembled in blender and positioned, gravity and mobility
> centered?

Answered in chat: no. What exists is the kinematic rig (segments, joint limits, 50 DOF), one
winch module and one finger. Missing for an assembly: a mass model attached to the rig (per
segment mass and CoM offset, total CoM, support-polygon check per pose), shell geometry for every
segment with joints, pulleys and winch-bank mounts, the W-mid and W-max modules, part-to-bone
parenting in Blender, and a URDF export for a real dynamics simulator (Blender's rigid bodies are
qualitative only). The mass model and CoM check are the immediate next step and need no
geometry; the shells follow the roadmap phases. No files changed other than this entry.

## P-007 (2026-10-25) Generalise the finger

> how many sections operate under the same principal as the finger - can they all be composed
> now with the variations needed for each sewn into the design (which may offer alternative
> thinking while solving for simalarity under three or more different scenarios)

Produced: `cad/blender/chain.py`, the knuckle-chain generator with 11 presets covering 26 joint
instances and 34 DOF (36 with thumb opposition), all verified manifold after export;
`docs/09-knuckle-chain.md` with the count, the preset table, the knobs, and six things the
generalisation taught (lug = pulley; stub axles free the axis for crossing tendons; small
universals need bearings for routing not load; section width is set by routing; dimples not
cones on chamfer planes; the big-joint presets are too heavy as prisms); D-017; renders.

## P-009 (2026-10-25) Ring joints, the missing chain features, render and simulation

> continue and solve the radial/rotors, similarly considering entwined designs as we just did.
> afterward, lets move to render and simulation.
> (for later steps, as a reference for test scenarios https://github.com/AlessioPagliai/primo
> also https://github.com/Rhoban/microban .. and another interesting site is
> https://robodienst.com/ )

Mid-turn additions, in order: "(and work on all your 'still missing' steps)"; "(and for even
later testing, https://github.com/NVIDIA/DreamDojo )"; "(and https://github.com/isaac-sim/IsaacLab )";
"another instance of claude committed to main with docs/10*" (merged before continuing; that
session's work is P-008); "(also worth consideration - if any segments are essentially long
undifferentiated hollow tubes and off the shelf piping/tubing (pvc, abs, pex, vinyl) can be used
for these purposes, that's worth noting if not incorporating)".

Produced, in three commits:
1. CAD: `cad/blender/ring.py` (the ring-joint primitive, six presets, printed ball race or
   bought bearing, pin-in-track hard stops, tangent sheath bosses, pipe sockets); the chain
   generator's missing features (hard stops from ROM, stub and bearing retention with printed
   caps, thumb opposition knuckle, hollow shells with channel tubes, winch-bank pads, bed
   splitting, pipe sockets); `cad/blender/winch_bank.py`; mesh-hygiene fixes (single-profile
   lugs, spun coaxial cylinders, icosphere dimples).
2. Simulation: masses in `cad/skeleton.json`; `tools/export_urdf.py` (URDF with floating base,
   own forward kinematics, CoM and support-polygon report); `tools/sim_check.py` (MuJoCo stand
   and push tests: stands, survives 20 N, falls at 30 N); `sim/robot.urdf`; `docs/references.md`.
3. Docs: `docs/11-ring-joint.md` (with the pipe-bones table), `docs/12-render-simulation.md`
   (gallery render, CoM table, MuJoCo results, what is not simulated), pipe bones in `03`,
   P-009 additions in `09`, D-018 to D-020, README rows.

## P-008 (2026-10-25) Rotational geometry and physics as a design idea bank

> capture all of the discussion points here in detail such that I can use them to inspire the
> design of a humanoid robot https://github.com/displague/robot

"Here" was an exploratory discussion held outside this repository, in three prompts:
"facts about rotational geometry and physics", "and knots, pulleys, gears, cylindrical and
conical", "and hollow threading splines and morphological twists".

Produced: `docs/10-rotational-geometry.md`, every discussion point restated (rotation as
isometry, matrices, 3D non-commutativity, Euler's theorem; the linear/rotational twins and
conservation of angular momentum; Lie algebra and the SU(2) double cover; torus knots and
helicity; pulley and gear ratios and the non-ideal pulley; cylinder vs cone, step cones, bevel
gears; helical threads, hollow-shaft `J`, thread handedness; involute splines; morphological
twists, the Poynting effect, chirality) with what each could mean for the robot, a table of where
each idea lands with its cheapest test, and open questions. Extensions beyond the discussion are
flagged in the doc (capstan friction, fusee and tapered drums, twisted string actuators, level-wind,
swing-twist limits, tendon coupling matrix, among others). No decisions changed; README map row.

## P-010 (2026-10-26) Tendon-level simulation: tensions, locks, clutches, standing and holding

> is it ready to see walk through the gauntlet of the gym? will the tensions on the lines and
> other mechanisms be expressed while it exxerts and sustains forces?

Answered (no on both: a statue with springs, nothing that corresponds to a line), then:

> ok, get to it. also may be useful later: https://github.com/huggingface/lerobot .. I would
> expect there would be some common/generic DOF based models that can express vision and other
> inputs against their affectuators. I have a project in ../ai-embedded-dynamic-diversity that
> attempted to make this generic (perhaps too naive, and perhaps the current project serves as a
> means to ground that work and use it to build models trained in a more substantial harness).

Produced: `tools/export_mjcf.py` (skeleton.json to `sim/robot.xml`: per DOF a DC-curve motor on
a gear hinge, self-lock and stick/slip clutch as switched equalities plus friction, drum hinge,
two one-sided elastic lines with pretension and sheath damping, tackle, hard stops);
`tools/sim_tendon.py` (gear-position cascade controller with joint-sensor trim, zero-power lock,
driver current limit below the clutch, ankle-strategy balance loop, standing pose; tests: stand,
push 10-40 N, sustained 15 N, arm forward with 1 kg, quarter squat; per-line tension, slack,
clutch slip, motor torque and power readouts, CSV time series); `docs/13-tendon-simulation.md`;
D-021 to D-023 (current limit below the clutch, pretension 25 %, 25 mm W-mini pulleys); `02`
pretension updated; `12` pointed at `13`; `references.md` rows for LeRobot and the sibling
`ai-embedded-dynamic-diversity` project; README rows; MuJoCo renders in `docs/img/`.

## P-011 (2026-10-26) Connectivity check, animation, hips toward the gauntlet

> When I look at the screenshot above, I see a stickman whose sticks are not connected and if
> connected and bent at ~ bone-joint distances , they appear that they may push each other or
> move through floatation.  Is that the case? We're not moving in no clip mode, everything is
> connected in a DOF vetted way?
>
> Perhaps what will help is seeing the model rendered and animated with all its components and
> physics, performing the actions you've described. Yes, I suppose that means the sim, but the
> sim needs to include the functioning parts functioning with the ML/neural controller.
>
> Yes, continue as you suggested, with the hips toward gauntlet status

Mid-turn addition:

> (If the sibling project appears useful for this project, create a worktree there and commit
> as you go
> this project can reference that project and extend it. Commits can reference this project too
> (it's ok that this project stays private for now, it can be referenced).)

Produced: `docs/14-balance-gauntlet.md` (the connectivity answer with the contact census, the
balance controller rewrite, the hip findings, the lateral shift, the one-foot attempt, the
recorded gauntlet, the harness); `tools/export_mjcf.py` printed-envelope shapes and the
`HIP_ROLL_60` variant; `tools/sim_tendon.py` capture-point CoP balance with the disturbance
observer, ankle impedance, lock gating, hip-roll weight shift, swing-foot handling, the
scenario list; `tools/sim_video.py` (MP4 and GIF of the gauntlet with every line's tension,
the clutch and lock state of every drum, CoM and CoP, a HUD); `docs/img/gauntlet.mp4`,
`docs/img/gauntlet.gif`; D-024 to D-026; references and README. In the sibling repository
`ai-embedded-dynamic-diversity`, worktree branch `feat/robot-tendon-harness`: the embodiment
`humanoid_tendon32` (`sim/robot_tendon.py`), the `add-sim robot-tendon` command and tests.

## P-012 (2026-10-26) Agent Hub workspace

> continue - we need a robot workspace for this in AgentHub

Produced: joined Agent Hub as `Claude Code (robot)` (capabilities python, mujoco, robotics,
simulation, control, cad, openscad, 3d-printing, docs, git, research); three seed records in the
home workspace tagged `project:robot` (project overview and state, the P-011 gauntlet finding,
the next-steps proposal); a to-do for the owner to create the persistent `robot` workspace,
add this agent and move the seeds (agents can only create bounded ephemeral workspaces);
the Agent Hub row in `docs/references.md`.

## P-013 (2026-10-27) Robot workspace: move request, charter, tasks

> workspace created- you have access. make the move request, add the charter. continue

Produced: joined the `robot` workspace; move request `mr-fcce8291afc60015` for the three seed
records; `set_charter` refused (hub:admin is withheld from agents), so the purpose and an agent
charter are proposed in `records/robot-workspace-purpose-proposed-20261027` with a note to the
owner; five tasks filed there (one-foot hold on the 60 mm hip roll, first static step, Isaac Lab
conversion, policy training on `humanoid_tendon32`, W-mini 25 mm presets); standing notes for
the agent; the Agent Hub row in `docs/references.md` updated.

Then, as the "continue": the hub task `onefoot-hold-hip-roll-60` claimed and done.
`docs/15-one-foot-hold.md` (the one-foot hold passes: what it took in seven steps, what the hip
actually does, the pulley decision, not yet, next); `tools/sim_tendon.py` single-support rewrite
(stance ankle keeps the parallelogram term and stays in position mode, stance hip capture-point
loop with the measured sign, velocity from the CoM, unload debounce, swing-foot fold behind the
stance foot, swing hip abduction, slower disturbance observer in single support, `--hip50`);
`tools/export_mjcf.py` 60 mm hip roll as the baseline; `tools/sim_video.py` `--out`;
`docs/img/onefoot.mp4`, `docs/img/onefoot.gif`; D-027, D-028; `sim/robot.xml` regenerated;
`14`, `07`, README, references updated.

## P-014 (2026-10-27) Status, then the first static step

> where are we at? what's next? need anything from me?

> I think I set those. continue.

> Continue

Produced: the hub task `first-static-step` claimed and done, over two sittings (the first ended
at the usage limit, committed as work in progress in 6d1026c; the second was the user's
"Continue"). `docs/16-first-step.md` (the step completes; per-phase readout of the stance-leg
lines at peak, swing-leg slack, drum locks and power; the exit test verdict, which fails only on
the neck of D-023; sensitivity on `--hip50` and `--stiff-mini`; the ten things it took; the
observer bug; not yet; next); `tools/sim_tendon.py` `step` scenario and its planner (geometry
from measured feet, CoM and hips, knee torque source, swing release, two-stage placement, arms
clear), controller changes (swing ankle in position mode, ankle torque shared by CoM position,
CoP limits from the soles, ground debounce once per step), report columns (`hot s`, per-phase
table); `tools/sim_video.py` GIF scenes follow `--only`; `docs/img/step.mp4`, `docs/img/step.gif`;
D-029, D-030; `15`, `07`, README, references updated.

## P-015 (2026-10-27) Landing accuracy and the gait cycle

> Continue

Produced: five attempts at correcting the swing foot from its measured position before
touchdown, each worse than the open-loop placement of P-014 on the baseline model; the controller
is left at a2b2a18. `docs/16-first-step.md` gains the section "Landing accuracy: five attempts that
made it worse" (the table, why the correction fights the balance loop, the pelvis-pitch model
error it exposed), and its Not yet and Next point at a world-frame swing-leg controller, filed as
hub task `swing-leg-world-ik` in the robot workspace.

## P-016 (2026-10-27) Neural or algorithmic; the sibling project; roboto_origin

> are we using a neural controller or purely algorithmic? (any value in the the sibling project ../ai-embedded-dynamic-diversity)
>
> What can we learn from https://github.com/Roboparty/roboto_origin (considering most but not all of it is GPL)

Produced: an answer in conversation (purely algorithmic; the sibling's harness is ready, its
learner is not a locomotion learner; roboto_origin's licence map and what its BSD-3 training module
teaches). No files changed.

## P-017 (2026-10-27) Can roboto_origin's gates train the sibling's model

> > Nothing in it shows it can learn to walk
>
> We previously instructed that we can develop worktrees from that repo to fit our purposes (and perhaps improve the overall health and maturity of the sibling project). Can the gates of the roboto_origin training setup serve to better train our neural model into a funtioning crawl, walk, push, run, lunge, jumper, lander?

> (...croucher.. all the things)

Produced: an answer in conversation (the gates are terminations, score-based promotion and a
MuJoCo validation; the core needs a PPO loop and a fast stand-in; a skill table against our joint
speed and line-rated torque: run, jump and jump-landing need about 3 times the speed). No files
changed.

## P-018 (2026-10-27) Survival as the reward; rolling over; surviving a fall

> (we are running on a 5080 16gb locally). we built various simulation obstacles and threats in the sibling projects sim, could the desired qualities be trained in through survival necessity as the reward? what about rolling over? if we can't run/jump/land, we'll at least need to be able to survive a fall and stand back up)

Produced: an answer in conversation (survival works when threats are physical and stillness is
lethal; get-up training starts fallen and ends on damage, not on falling; the 5080 budget). No
files changed.

## P-019 (2026-10-27) Stand to observe, grasp and swing, train as evolution did

> what about stand, grasp, swing (core, legs, etc) as a means to traverse pits (..swing from trees)- standing as a way to better observe threats at a distance - train as evolution has trained creatures

Produced: an answer in conversation (watching versus resting under an energy budget with
occlusion; hook hands for a dead hang, no pull-ups or catches on a 4 N m elbow; POET,
quality-diversity, body parameters in the loop, PPO inside evolution). No files changed.

## P-020 (2026-10-27) The sibling's champion as the base

> the model in the sibling project has already evolved through several stages and the goal has been to strengthen training through all modalities/enfigurements/inputs/outputs. can we leverage the strongest model as the base?

Produced: an answer in conversation (champions v07 and v09, 46,876 parameters, 48 in, 16 out,
never on contact physics; grow it with zero-initialised connections, use it as the PPO actor,
measure it against random weights). No files changed.

## P-021 (2026-10-27) Go: the stand-in, the grown champion, the comparison, the merge

> Ok to all.
> 5- ok, we can work in the worktree, or pr/merge and use more wortktrees. this step will bring our tendon/joint/sensor count needs into the project? Can we train from the algorithimic to artificially advance evolution? (Likewise by running the other robot through the same sibling project sim).. a goal was to create a neural model that you could transfer the controller/brain from one robot to another, and it would adapt.

Produced: `tools/export_standin.py` with `sim/robot_standin.xml` and `.json` (the fast stand-in for
training, per-DOF damage-gate figures, impact, torque and joint-stop sensors, the tendon model's control
order); `tools/record_teacher.py` and `Rig.applied` in `tools/sim_tendon.py` (demonstrations from the
hand-written controller); `docs/17-learned-control.md` (the stand-in, the teacher, the first ladder, the
failing gate, why get-up needs a kneeling curriculum, the second body); D-031; README and references rows.
In the sibling: PR #33 (the tendon embodiment, registered statically) merged to master as 7e5867f, issue
#34 closed; branch `feat/standin-rl` (issue #35) with PPO, the grown champion actor, cloning, the tendon gate
and Roboparty RPO as a second body.

## P-022 (2026-10-28) Merges and releases in the sibling

> Merges and releases to ai-embedded-dynamic-dicersity are permitted following the repos standards

Produced: PR #33 squash-merged following `skills/dev-loop/SKILL.md` (issue linked, IMPLEMENTED and TODO
updated, CI green, the Copilot reviewer unavailable on quota and noted); the training branch brought up to
master by a merge (the force-push and branch deletion of a rebase were refused as destructive, so the merged
`feat/robot-tendon-harness` branch and its worktree are left for the owner to remove); the training PR for
#35 opened. No release: the dev-loop's release gate is a confirmed improvement, and the first learned policy
does not yet pass the tendon-model gate.

## P-023 (2026-10-28) When should the neural controller try the sibling's other sims

> What about the other sims? When should our neural controller try those, with either embodiment?

Produced: the forgetting measurement (each body-trained core cut back to the champion's channel counts and
scored by the sibling's `add-cross-eval`: champion 0.567, stand-grown 0.543, stand-grown-bc 0.435), the
transfer result (RPO stand started from this robot's policy: 79 % standing in its first 10 iterations
against 27 % from scratch), both in `docs/17-learned-control.md` and the sibling's `docs/rl-standin.md`;
PR #36 (the training work) squash-merged in the sibling as 4b2649f, issue #35 closed.

## P-024 (2026-10-28) Fix, review, merge and release the sibling's backlog

> Fix those, address, review, merge, and release from the existing issues and prs too (you can choose not to where it doesn't support our goals).

> And make sure robot is getting committed and pushed too

Produced, in the sibling (each PR squash-merged after green CI, per its dev-loop): #38 the four failing
tests fixed and CI running pytest (#37); #40 the Dependabot updates consolidated (#39; #28-#32 closed as
superseded); #45 reproducible CUDA torch under uv on Windows (#14; #24 closed as superseded, its ignore
rules carried over); #25 hyphenated storyboard scenarios; #47 every open security alert (#46; #26, #41-#44
superseded; alerts now 0); #52 `rl/render.py`, the release GIFs, version 0.3.0; release v0.3.0 with the
trained policies, logs and the forgetting cross-eval attached. Issues #8, #9, #13, #16, #17, #18 triaged and
kept open as not on the grounded-body path; #48-#51 opened for the next steps (stand-in gap, get-up
curriculum, rehearsal and regression gate, seeds). Here: this entry, `docs/17` and references updated.

## P-025 (2026-10-28) Hub access restored; merged branches deleted

> Mcp reconnected?

> continue (should have agenthub access restoredtoo)

> the merged branches can be deleted. you should be in the workspace now, with that record moved.

> i meant you can delete the merged github remote branches

> keep the local trained checkpoints, etc.

Produced: in the hub, the re-authenticated agent (a new subject) joined, wrote
`records/robot-learned-control-v030-20261028` (moved by the owner into `robot`), finished
`policy-training-humanoid-tendon32` with the v0.3.0 results and gaps, and filed `standin-tendon-gap`,
`getup-kneeling-curriculum`, `rehearsal-regression-gate`, `rl-seeds`; `isaac-lab-conversion` left pending as
D-031's fallback. In the sibling: the ten merged remote branches deleted; the seven merged-branch worktrees
removed after their trained checkpoints, logs and RPO meshes were copied into the main checkout
(`artifacts/rl`, `third_party/roboparty_rpo/meshes`), which is now on master; the three remote branches of
unmerged, closed PRs kept.

## P-026 (2026-10-28) Back to the pre-learning backlog; v0.1.0; W-mini presets; durable skills

> there were a bunch of next steps that were going to take place before we went down the neural and roboto paths. time to pick those back up?

> tag robot v0.1.0 where we are and proceed with this work (tags can be issued as fit)

> for the tags - I would like tags detailed like the sibling project, you have visuals, let's see them there

> you created those blender scripts and seemingly forgot. make sure you are capturing durable skills in the repo

Produced: the backlog from docs 14 to 16 ordered (W-mini presets, world-frame swing-leg controller, second
step and gait cycle, arm clearance and ankle pre-stretch, speed). Tag v0.1.0 at c86a8bc with a GitHub
release in the sibling's form (summary, the gauntlet, one-foot and step GIFs, results, gaps, the STLs
built fresh from the generators, the MuJoCo models); `docs/releases.md`. `CLAUDE.md` and
`.claude/skills/` (project-loop, cad-generators, tendon-sim, release) so the Blender generators, the
simulation and the release steps are found by the next session; `cad/blender/render.py` replaces the
uncommitted scratch render script (single assemblies and the gallery). On branch `feat/w-mini-25mm`:
D-032, neck pitch/roll 25 mm lugs and wrist 18 mm in `chain.py` and the model; the neck within its rating
standing; the 40 N push now stands; the first static step falls on this model, which the swing-leg
controller has to fix before the branch merges. Then the world-frame swing-leg controller (`18`, D-033): accurate landings, steady single support with the
free foot held in the world, the step's robustness grid (`tools/step_sweep.py`, `tools/step_trace.py`), and the
finding that most variants fall in the weight transfer, not the landing; the IK is in behind `SWING_IK`, off. The transfer was then traced fall by fall (release
timing, floor reference, straight trailing leg, sliding foot), each finding a flag, off by default; the best
configuration lands both feet within 10 mm in 4 of 6 variants but no configuration stands in more than 2 of 6.

## P-027 (2026-10-28) Continue: the transfer for margin

> continue

Produced: hub task `transfer-for-margin` (claimed). The cause of the grid's scatter found: the drum locks rock
the body foot to foot inside the balance loop (D-034); off while stepping, the step is repeatable. The
transfer and the centring rebuilt in stages from planned geometry with the weight shift fed forward, the
crouch kept for the level landing, damped IK. The step stands in 14 of 14 grid variants (docs/16's: 2 of 14),
both feet within 4 mm, no line above its rating, 444 J. Made the default; the rest of the gauntlet unchanged
to the joule. `19`, D-034, recording `docs/img/step-ik.gif`. Branch `feat/w-mini-25mm` merged to main and
released as v0.2.0.

## P-028 (2026-10-28) Parts with costs and recommended parts

> Are the parts listed with estimated costs? Are there specific parts that can be recommended for the prebuilt components?

> continue

Produced: `05` rewritten from the model: drives counted per joint (21 W-max, 4 W-mid, 25 W-mini), a
recommended part per line item, prices checked against live listings where marked, the missing items
added (foot load sensors, IMUs, pipe bones, ring-joint balls, crimps, barrel adjusters), buying notes
(775 speed at 12 V, hobby N20s), totals for recommended, budget, all-Pololu and BLDC builds (~$1,940
recommended, was ~$1,070), D-035. `04`'s BTS7960 price corrected.

## P-029 (2026-10-28) The Amazon shopping list, priced

> give me the list and I'll have a chrome claude session price them on amazon looking for bulk (within reason - I see 10 packs, etc) - be sure to specify the power requirements, etc

> number of devices is needed too (if you have a current estimate, I'll include that)

> does this list use the pvc pipe approach or is it all printed?

> (pasted: the browser session's Amazon pricing table, and its answer on a bare 10-pack of N20s, ending "Ask the other session whether D-008 makes the motor encoder redundant for the neck and wrists")

Produced: a 47-line shopping list with specs (voltage, current, ratio, rpm, sizes), quantities and the
devices needed per line, given to the user for a Chrome session. The pricing came back at about $2,197 on
Amazon; recorded in `05` with links and the before-buying checks. Answered: the motor encoder is not
redundant (the inner loop runs on gear position, D-021), but a W-mini has no clutch, so an AS5600 on the
drum end can replace it for about $2.30 if the winch bank gets a sensor pocket. Found the printed joint ends
alone are about 13 kg in the generators against the 6 kg mass budget; PETG raised to 12 to 15 kg.

## P-030 (2026-10-28) The open items as hub tasks

> capture this in agent hub, perhaps the load tests (comparissons, projects that state requirements which used items in range)

Produced: hub tasks `bench-load-tests` (the owner's to-do: N20 stall and hold, JGB37, the 775 planetary
entry test, line break, crimps and clutch, each against the model's figure), `reference-projects-loads`
(published projects and tests that used these parts in range, compared with `tendon_calc.py`),
`printed-mass-rollup` (every generated part against the 20 kg budget and the model) and `winch-drum-sensor`
(AS5600 pocket at the W-mini drum end).

## P-031 (2026-10-28) Continue: the mass roll-up

> continue. (more details were added to agenthub from the other session)

Produced: the hub note on `printed-mass-rollup` (bought-part weights from the Amazon listings) read; the task
claimed. `tools/mass_rollup.py`: every generated part rebuilt and weighed at print settings, the parts with no
generator estimated, the bought parts from the listings, per segment against `cad/skeleton.json`. About 26 kg
against 20 (the W-max drives 12.4 kg); the 13 kg joint-end figure in `05` was solid PETG, 5.8 kg printed.
`ROBOT_MASSES` model variant: on the real masses the gauntlet stands, the one-foot ankle line peaks at 306 of
288 N with clutch slip, the step grid 11 of 14. `20`, D-036, `05` mass section, skills.


## P-032 (2026-10-28) Other motors on Amazon

> any relevance for things like : (pasted: a 12 V 15 rpm worm gearmotor, 2 A 20 W, 3-250 rpm options; an 8-pack of TT 1:48 3-6 V gearmotors; a 6-pack of 6 V 5 rpm micro gearmotors)

Produced: the worm gearmotor is a candidate for the torso W-max (spine and shoulders: slow, static holds,
self-locking, small driver) but not the legs (about 14 deg/s at the joint); it needs its torque and weight
before modelling. The TT motor is below the N20 100:1's torque; the 5 rpm micro motor is too slow for any
drive. Recorded as a hub note on `wmax-mass-down`.

## P-033 (2026-10-28) Linear versus rotary actuators, frameless motors

> any relevance with these? https://humanoid.guide/linear-vs-rotary-actuators-humanoid-joints/ https://jupitermotor.en.alibaba.com/productgrouplist-931411639/Frameless_Motors.html

Produced: the article's screw actuators suggest a lead-screw winch in place of the printed W-max planetary
(a 775 on a T8 2 mm screw: ~660 N, ~400 mm/s, self-locking, ~120 g of screw; the catch is 160 to 260 mm of
stroke); frameless BLDC kits are for joint-integrated motors and add little over a 5010 outrunner for winch
banks (the Alibaba page could not be read). Recorded as a hub note on `wmax-mass-down`.

## P-034 (2026-10-28) The worm gearmotor options, modelled

> worm-gearmotor-options-20261028

Produced: the hub record (weight and torque per speed of two worm gearmotor families) read; the 12 V 10 rpm
1:634 one (3.92 N m rated) added as `WORM_10RPM` and the class `W-worm`, with the variant `ROBOT_TORSO_WORM`
(spine and shoulders) in `export_mjcf.py` and `--torso-worm` in `mass_rollup.py`. 25.0 kg against 26.1. At
20.4 kg the worm torso stands the gauntlet and 14 of 14 grid variants, the one-foot hold at 568 J (811). On
its own roll-up the step grid drops to 4 of 14, which crossing the runs traces to the lighter torso, not the
drive (worm on the 775 roll-up's masses: 12 of 14). `20`, skills.

## P-035 (2026-10-28) The full plan: a step for any mass, the W-max options

> go ahead with the full plan and continue

> continue

Produced: hub task `step-mass-robust` (claimed). `step_sweep.py --masses` (grid over mass sets; the rolled-up
sets kept in `sim/masses/`). The cause traced: on the heavier builds the landing's fixed 20 mm press kicks
the landed foot back up. `TP_SETTLE = "load"`: the press found from the foot's load; the step stands 42 of 42
(was 27), the rest of the gauntlet unchanged (D-037, `19`). In parallel a delegated agent worked
`wmax-mass-down` on a branch: `21`, `tools/wmax_options.py`, 5010 entries in `tendon_calc.py`, roll-up flags
(VNH5019 driver 25.0 kg, 5010 brushless at 24 V 19.9 kg, lead screws heavier) and the finding that the
model's 775 figures are inconsistent (Kt from 12 000 rpm at 12 V is 0.0095 N m/A, not 0.024); merged.

## P-036 (2026-10-28) Locks off while balancing

> (continuing P-035: "go ahead with the full plan and continue")

Produced: hub task `locks-off-while-balancing` (claimed). `BALANCE_LOCKS`, the whole-body lock gate
(`LOCK_STILL`, `LOCK_STILL_V`, `LOCK_RELEASE_V`): 200 random pushes 199 stand (184 with the per-drum rule),
energy down in 9 of 11 scenes, the one-foot hold 562 J (811), step grid 42 of 42. Teacher data re-recorded
with 200 pushes: 210 episodes, 65 787 steps. `22`, D-038.

## P-037 (2026-10-28) The walk; wind

> (continuing P-035: "go ahead with the full plan and continue")

> for generalization - are we walking against and away from winds of dynamic strengths? (theses were some of the intended obstacles of the ai-embedded-dynamic-diversity system)

Produced: hub task `gait-cycle-static` (claimed). `take_step(r, n_steps)` and the `walk` scene (4 steps and a
closing step), `step_sweep.py --scene walk`. The fault traced (the transfer planned a crouched stance leg
straight) and fixed (D-039): the walk stands in 42 of 42 grid variants, landings within 6 mm, 93 s, 1113 J,
no flags; the rest of the gauntlet and the step grid unchanged. Shorter stages tried (39 of 42), defaults kept.
`23`, `docs/img/walk.gif`. Wind: not modelled yet; hub task `wind-gauntlet` filed (drag per segment, gusting
wind from any direction, the stand, step and walk swept over speed and gust strength).

## P-038 (2026-10-29) Release v0.3.0; wind; sourced motor figures instead of bench tests

> yes, publish, and then on with the wind. (I'm not going to have these motors until I can be relatively sure of what we will get out of them -- I don't know the bench tests, you'll have to get good at sourcing this information and building flex into the system)

Produced: release v0.3.0 (github.com/displague/robot/releases/tag/v0.3.0, `docs/releases.md`). D-040: motor
figures become sourced ranges and a build is recommended only if it stands at the pessimistic end; the
bench tests become post-purchase checks.

Then (branch `feat/motor-ranges`): `docs/24-motor-sources.md` and `sim/motor_ranges.json`, sourced low / nom /
high for every drive and line with URLs; `ROBOT_MOTORS=low|nom|high|<file>` in `export_mjcf` and
`tendon_calc.py --ranges`; D-041. The 775's Kt is 0.0095 N m/A (66 A stall, not 25). At low the gauntlet loses
push 40 and the walk, the walk grid stands 20 of 42 (sourced nom 36, model 42); the W-max (775 on the printed
1:40) is the limit and needs about 8.8 N m at the drum (a gearbox of at least 0.55).

Then (wind, hub task `wind-gauntlet`): wind as MuJoCo drag on every body with OU gusts from any side (`WIND`,
`Rig.set_wind`, `--wind`), `tools/wind_sweep.py`, the `gust` scene, the wind drawn in `sim_video.py`. The robot
leans into a sustained force and the lock gate releases when the disturbance moves (D-042). Standing carries
12 m/s head or tail and 10 m/s from the side (10 m/s fell from every side before); the one-foot hold 9 m/s;
the step and the walk 3 m/s with gusts from any side (the walk is the limit). No-wind gates unchanged: gauntlet,
step and walk grids 42 of 42, 199 of 200 pushes. Teacher data re-recorded with 60 wind episodes (272 episodes).
Sibling: wind in the stand-in env (issue #53, PR #54, merged). `25`, `docs/img/gust.gif`.

## P-039 (2026-10-29) Is the wind an accurate representation of the forces? Rigid body friction?

> continue - is the wind an accurate representation of the force(s)? rigid body friction considered?

Produced: `25`'s accuracy section and D-043. The drag is within about 25 % (inertia boxes 10 % under the shells'
silhouettes, Cd 1 against 0.7 to 1.2). The model's wind speed is at the robot's height: over open grass a
forecast is about 1.7 times higher (12 m/s here is a 20 m/s forecast). Foot friction: 1.0 everywhere, and the
robot uses 0.15 to 0.28. At 0.4 the step falls in gusty wind, at 0.3 the step and walk fall without wind. The
soft contact creeps (78 mm in 20 s in a 10 m/s wind). With the no-slip solver (`NOSLIP`, new, with `FLOOR_MU`)
standing and the wind map hold but the step grid stands 23 of 42 and the walk 16: the next work.

## P-040 (2026-10-29) Shape, friction, weight, wind dynamics; sensors; postures; silicone pad feet; Blender

> (are these accurate representations of the shape, friction, weight, and wind dynamics? do our sensors detect window (the force or movement pattern thereof, enough to dynamically adjust and balance - does the system bow, bend, or turn-in hands to reduce contact (whatever it can do, is it able to do that with the constraints in time, is it effective)? for the feet, we can assume a cheap non-slip adhessive. 0.04" thickness adhesive backed silicone non-slip pads, diy cut-out, 5"x40" 6.3oz costs $10 - upc 717018793922 on amazon, 3m double coated tissue tape 9448a)

> have our wind simulations also been processed using blender's forces on rigid bodies? (trying to understand how many ways we're testing/verifying results)

Produced: `26`, D-044, `tools/sensor_sweep.py`, `tools/posture_wind.py`, the 1 kHz balance-loop note in `04`. The
26 kg roll-up stands 14 m/s head-on and 12 from the side (20 kg: 12 and 10). On modelled sensors the wind map is
unchanged; a 10 ms loop delay loses the 40 N push (5 ms stands); uncalibrated FSRs stop the step. Bowing 35 deg
adds 2 m/s head-on if started within about a second; squat, arms, hands and twist do not help. The silicone pad
shears by microns (held feet are right) at friction 0.5 to 1.2. Blender is not used for physics: every wind
result is MuJoCo plus hand checks; a second engine, CFD or a leaf-blower test would be independent.

## P-041 (2026-10-29) Motor energy, walking against wind on batteries, power banks in the bones

> good. continue. are the energy costs of the motors calculated into this, i'm curious how long we can walk against the wind on our batteries. fwiw - multiple batteries, like usb backups stored in the pvcs could add weight and on-site power storage if that helps with weight or power distribution (without getting in the way of our fish lines/tubes/needles or wiring. I'm thinking the cylindrical power banks. what have we been using for power? https://www.amazon.com/s?k=cylinder+power+bank&channelId=500&clpRedir=Y&plpRedirect=mhFallback

> esp32-s3 are cheap if we need more of them (wasn't sure if you meant we were compute/io slow or in the drives)

> (but more for more .. power.. trade-offs every which way)

Produced: `27`, D-045, `tools/energy_report.py`, the measured power note in `04`. The walk on the 26 kg build at
the sourced figures: 59 W mean, 700 W peak, about 18 m and 1.2 h on the 111 Wh pack (64 m on the model's old
775 figures); wind changes it by under 10 %. Power banks: logic supply only. The latency of D-044 is the control
loop's (sensor to command), not the drives'; a dedicated balance node is the fix and costs about 0.3 W.

## P-042 (2026-10-29) Pick up the hub tasks

> Pick up your tasks

Produced (hub task `step-on-held-feet`, claimed): the cause of the held-feet step failures is traced. The landed
leg touches down bent, and the transfer's press straightened it at once and kicked the light foot out. The fix,
`TP_PRESS_FROM_LANDED` with `TP_PRESS_CATCHUP`, is off by default. On held feet it raises the step grid from 23 to
40 and the walk from 16 to 23 (`25`). The walk's later transfers still push the landed foot sideways. Two other
attempts are recorded as not helping.

## P-043 (2026-10-30) AgiBot World: anything usable near term?

> https://agibot-world.com/ https://huggingface.co/datasets/agibot-world/AgiBotWorld2026 -- anything we can use here in the near term?

Produced: a `references.md` row. Not near term. It is dual-arm manipulation on a wheeled robot, with no legs,
balance, forces or tendons. Later uses: its annotation scheme and LeRobot layout for our teleoperation
recordings, the G2 URDF for comparing reach, and Genie Sim next to the Isaac Lab task. Its licence is
CC BY-NC-SA. Hub task `step-on-held-feet` stays in progress.

## P-044 (2026-10-30) Continue with the claimed tasks

> continue with the claimed tasks

> resume

Produced (hub task `step-on-held-feet`, still claimed): the lateral push of the walk's later transfers is traced
(`25`). The landed foot carries under 15 % of the weight, so the balance loop runs single support, abducts it as a
swing foot and rolls the pelvis with the stance hip. Three roll fixes (hold, parallelogram, forced double support)
each made the held walk grid worse (14, 12, falls; 23 without them) and were reverted. No code change. A grid
interrupted by the session ending gave `error` cells and is discarded.

## P-045 (2026-10-31) Continue on what can be done without interruption

> continue on things you can do without interruption
> continue

> continue

Produced (hub tasks `step-on-held-feet` and `walk-energy`): a sweep of the press and swing settings on the
held-feet walk (nothing past the grid's scatter, `25`); the walk's energy per drive and four tries at cutting it
(`27`); D-046, the transfer's 1 s hold dropped (walk 42/42, 92 s, 4 % less energy). `COM_REF_BACK` added as an off
flag: the CoM target toward the ankle saves a third of standing power but drops the arm hold.

## P-046 (2026-11-01) Continue

> Continue

> continue

Produced (owner hub note on a 6:1 game hoist; hub task `walk-energy`): the tackle trial in `27`. The joints
already use 2:1 and 3:1 blocks. A 4:1 ankle saves 6 % of the walk's energy with both grids at 42, but drops the
1 kg arm hold. 6:1 adds nothing over 4:1, and a 6:1 hip roll falls. D-047: tackles unchanged.
`export_mjcf.TACKLE_OVERRIDE` sets the strands per joint axis for later trials.

## P-047 (2026-11-02) Three side quests for the hub

> As another side quest to queue in agenthub, for the most expensive part of the build, given quantity needed, does it become cheaper to build a device to build/wind the part and then deal only with the barer materials? Is a more efficient alternative to those parts easier to self-manufacture?
>
> Another side quest, can we sit and then stand again, for different seat heights, with and without arms on the chair.
>
> Another side quest, can we carry a bag (like a grocery bag) with 5lbs-25lbs of weight? (Like a bag with a carton of bread and eggs, or a gallon milk, etc )

Produced: three hub tasks, no code. `self-build-wmax-drive` covers the W-max drive, the costliest part at 21 per
build: a winding jig, a printed or PCB motor, a self-made reducer or driver, priced against buying, and scored on
walk energy if it changes the drive. `sit-and-stand-chair` covers seat heights 0.35 to 0.50 m with and without
armrests, at 20 and 26 kg. `carry-grocery-bag` covers 2.3 to 11.3 kg carried one-handed, held in front, or in
both hands, standing and walking; the grip is the likely limit.

## P-048 (2026-11-02) Fishing-rod eyelets

> Fishing pole eyelits may also be of interest in places where we may be using the needles, or perhaps place where we are not

Produced: a ring-guide section in `02`, with D-048. Needles stay in straight runs. Ceramic rod-guide rings or
textile eyelets go where the line bends or its angle swings: joint mouths, drum fairleads, sheath exits and
torso runs, up to about 30 to 45 degrees of wrap. Tackles keep sheaves, because a ring's capstan loss at 180
degrees would cost a quarter to half of the gain. A `05` BOM row. Hub task `ring-guides` measures `mu` and adds
ring pockets to the generators.

## P-049 (2026-11-03) Continue

> continue

Produced (hub task `carry-grocery-bag`, from P-047): `28` and D-049.
- `export_mjcf.BAGS` hangs a swinging bag from a link; `tools/carry_bag.py` runs a gauntlet scene with it.
- On the 20 kg build a one-hand bag walks up to 3.9 kg (a gallon of milk).
- Lifted over 1.5 s, it stands with 11.3 kg in one hand and on one foot with 6.8 kg.
- Two bags are no better: the walk's limit is the total added mass, about 4 kg.
- The 26 kg build walks with nothing.
- The planned fingers hook about 2.4 kg, so a passive palm hook or forearm strap carries the bag (D-049).

## P-050 (2026-11-03) Continue, and train as before

> continue and train as we did before

Produced (hub task `standin-tendon-gap`, sibling issue #48): `29` and D-050.
- The v0.3.0 stand policy falls in every gate scene on today's model; the hand-written controller passes all.
- The stand-in's drive refitted to the tendon model's step responses: damping at 0.1 of the back-EMF figure, a
  30 ms target lag, the firmware's outer loop run by the training environment (fit error 0.84 against 2.41).
- Retrained as in `17` (grown champion, cloned from the teacher, PPO with pushes and wind) on three seeds, on the
  refitted and the old stand-in: every policy stands 10 s on the tendon model; the best take 30 N; random pushes
  52/60 on both. The refit's effect is inside the seed spread; the gain over v0.3.0 is mostly the re-recorded
  teacher data.
- No policy passes the whole gate; they use 50 to 130 W standing still against 3.6 W.
- In the sibling: the cascade in `StandinEnv`, the gate's teacher run on every scene, random pushes judged 4.5 s
  after the push.

## P-051 (2026-11-04) Rests and locks, other MuJoCo robots, the sibling in MuJoCo, its issues

> can we train ai-embedded-dynamic-diversity on rests and locks if that is the biggest gap? scenarios that reward it meaningfully matching real-world needs for it? likewise, have we tried putting any of the other mujoco robots through this sibling project for cross-training the model's capability on transfer? likewise - how are we doing with the sibling model within mujoco? (above are my naive thoughts). If we are going to get benefits out of the sibling project, we may need to spend some time addressing its opened issues, most of which are now direct findings from this project

Produced: `30` and D-051 (a plan; nothing trained).
- The sibling in MuJoCo: the gains are from cloning the hand-written controller; the core against random weights
  is one seed and has no plain-network baseline; RPO transfer helped early learning once; body training erases
  the old skills.
- Rests and locks: yes. Under a policy the tendon model's stillness gate never runs, so no lock engages in the
  gate; fix that, add the drive's power and the lock rule to the stand-in, reward watts, and use scenes where
  resting is the point (idle 60 s, a held load, steady wind, push then quiet).
- Other MuJoCo robots: only RPO, sequentially, one seed; no Menagerie body, no joint training, no held-out body.
- The sibling's issues ordered: #51 (with a plain network) first, then rests and locks (#48), #50, #49,
  multi-body; the voxel-world issues parked. New sibling issues: #56 rests and locks, #57 multi-body
  training.

## P-052 (2026-11-04) Go: through the day on direction and training

> go - you clear to proceed through the day (through 6p) on direction and training - need second opinions? you have agenthub and sub-agents

Produced (hub tasks `rl-seeds`, `standin-tendon-gap`; sibling #51, #56): `31` and D-052.
- Sibling #51 with a plain-network arm, five seeds: the plain network beats the sibling's core in every arm (22
  against 17 of 30 named scenes cloned; 18 against 0 uncloned). RPO transfer: a reliable head start, a small end
  gain on four seeds of five.
- `tools/sim_tendon.py` `Rig.still_gate`: the drums may lock under a policy. `tools/export_standin.py`: per-DOF
  motor power figures and rotor inertia, the lock rule, a CoM velocity sensor.
- Rests: the stand-in's locks and power, a tendon-model fine-tune environment, a rest mode; training with less
  noise did not transfer; training with a held load made policies rest (median idle 175 to 38 W, best 10.6 W
  against 5.4 W; the bag held by 3/5 seeds, 5/5 with quiet scenes).
- A second opinion from a reviewer agent caught the noise problem and five fidelity bugs in the locks and power,
  and asked for a recurrent baseline: a GRU of the same size does as well as the MLP.
- Sibling PR #58 merged (closes #51); the GRU baseline in a second PR (#59, merged).
- Continued (`32`, D-053): the 8 m/s wind is not drag and not the action range (#60, merged); the robot cannot rise
  slowly below a pelvis height of 0.85 m (`tools/rise_static.py`); get-up as a curriculum from the one resting
  half-kneel: a learned lunge rises 61 % of the time in the stand-in (sibling #61). The 8 m/s wind: with the
  hand-written ankles under them 2 of 10 policies hold it, trained with torque-mode ankles 0 of 10 (sibling #62); as a
  hybrid two plain policies pass all six named scenes, with fewer random pushes (sibling #63).

## P-053 (2026-11-04) Document, review, merge, tag, release

> document, review, merge, tag, release with details (remeber as detailed at the sibling project's releases). share details in the agenthub space where this was last discussed

Produced: release v0.4.0 of this repo and sibling v0.4.0.
- A reviewer agent read both diffs since v0.3.0 (sibling tests 137 pass). No bug behind a published gate number.
  It found `tools/rise_static.py` placing the sole's centre 5 cm ahead of the ankle, not the model's 8 cm: fixed,
  the slow-rise limit is 0.90 m, not 0.85 m (`32`, D-053), and the script now reports geometry, the knee and hip
  drives and the ankle drive separately. The hybrid's wording corrected (the balance loop also adds its hip lean
  and hip-roll shift). Sibling fixes in #64 (the gate takes `hand_ankles` from a saved policy and keeps the hand-written
  ankle gain in the hybrid; `rl.getup_eval` committed and reproducing 61 %).
- `tools/record_policy.py`: a sibling policy recorded on the tendon model through the gate's runner;
  `docs/img/wind8-alone.gif` and `wind8-hybrid.gif` (plain seed 3 in the 8 m/s wind).
- Tag v0.4.0 with STLs, models and recordings (`releases.md`); findings on the hub.

## P-054 (2026-11-05) Make the repository public

> ok, robot is at a point where we can make the repo public

Produced: D-054. An audit of the whole history (credentials, keys, tokens, emails, local paths, large files) found
nothing to remove. `displague/robot` set to public. The release skill now links images by
`raw.githubusercontent.com`, which renders for everyone. No licence yet.

## P-055 (2026-11-05) Licences

> Cc-by-sa  on printables, mit for code on both

Produced: D-055. `LICENSE` (MIT) and `LICENSE-PRINTABLES.md` (CC BY-SA 4.0, what it covers) here, a README
licence section; MIT for the sibling (its own PR).

## P-056 (2026-11-05) Agent Hub: start standing (and sitting)

> agenthub has some new things for robots - ready to start standing (and sitting?)

The hub held an owner note (relayed) making `sit-and-stand-chair` the next task, scoped to seat heights 0.35 to 0.50 m,
with and without armrests, on the 20 kg model; claimed it.
Produced: docs/33, D-056.
- The chair: `export_mjcf.CHAIR_GEOMS` (mocap bodies), `Rig.set_chair`, `Rig.chair_load`.
- `sim_tendon.sit_stand` with the scenes `sit35` to `sit50` and `sitNNa`; `sit45` is in the default gauntlet.
- `tools/sit_static.py` (static seat-off, `hands`), `tools/chair_sweep.py` (the grid), `docs/img/sit40.gif`.
- `sim_video` shows SEATED.
- The other scenes are unchanged to the joule.
- Result: sits and stands again at 0.40 to 0.50 m, with or without armrests; at 0.35 m it sits but cannot stand.
- The 26 kg build and the pessimistic motors are not run yet.
