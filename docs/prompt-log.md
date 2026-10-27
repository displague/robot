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
