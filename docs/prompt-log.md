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
