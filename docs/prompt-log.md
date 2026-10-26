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

Produced: this log; the initial commit and private repository `displague/robot`; walking and
balancing promoted from stretch goal to requirement (`docs/07-locomotion.md`, roadmap Phase 5-6
rewritten, D-006 criteria tightened, walking scenarios added to the calculator); next generators
(`cad/blender/finger.py`, closed-loop drum) and the skeleton joint table that drives the Blender
armature rig.
