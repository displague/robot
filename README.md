# Home-printable tendon-driven humanoid

A full-size humanoid that a person can print on a desktop FDM printer, assemble by hand, and
run on one to three cheap boards (ESP32 class for motion, a Pi 5 / RK1 class board only if we
want autonomy). Muscles are fishing line pulled by small spindles ("winches") that can lock
tension without power, routed through PTFE tube like bicycle brake cables, so the strong part
(the motor) can live wherever there is room and the moving part (a finger, a wrist) stays light.

Goal ordering, in case they conflict: **cheap > simple to build > human range of motion > walks and balances > strength > speed.**

## Design principles

1. **ROM parity, not DOF parity.** Every joint reaches the human range of motion. We do not give
   every joint its own motor. Fingers are underactuated (one tendon per finger, elastic return).
2. **One actuator family, three sizes.** The same winch design (motor + drum + line exit + lock)
   scaled to three motor classes. Fewer unique parts, one firmware driver, one calculator.
3. **Tendons carry force, printed parts carry geometry.** Bones are hollow PETG shells with
   integral tendon channels. Nothing printed is loaded in a way that a layer line can peel.
4. **Lock without power.** Holding a grip or a standing pose costs zero current. Self-locking
   gearing (worm or high-ratio spur) plus a friction drag that slips before anything breaks.
5. **Regional winch banks.** Hand tendons are driven from the forearm, arm tendons from the
   torso, leg tendons from the pelvis/thigh, exactly like the body. One motor per tendon, but all
   the motors of a region sit in one printed block with one controller.
6. **Measure, then trust.** Every motor and line figure in `docs/` is a catalogue ballpark until a
   bench test replaces it. The calculator in `tools/` is the single source of those numbers.

## Repository map

| Path | What |
| --- | --- |
| `docs/01-architecture.md` | System overview, DOF budget, actuator classes, compute topology, honest limits |
| `docs/02-tendon-actuator.md` | The winch: physics, lock options, drum rules, line choice, creep, calculator results |
| `docs/03-skeleton-joints.md` | Joint types per body region, ROM targets vs human, materials, print segmentation |
| `docs/04-electronics.md` | Boards, bus, drivers, encoders, power |
| `docs/05-bom-estimate.md` | Parts list with recommended parts, checked prices, totals and mass |
| `docs/06-roadmap.md` | Phases with exit criteria, starting with one finger on one winch, ending in dynamic walking |
| `docs/07-locomotion.md` | Walking and balancing: joint loads from gait, W-max sizing, sensors, controller stages, feet |
| `docs/08-passive-balance-springs-compliance.md` | What balance comes for free (and why not a bladder), where springs are used, how compliant the robot is |
| `docs/09-knuckle-chain.md` | The finger's principle generalised: one generator, 11 presets, 26 joint instances, what the generalisation taught |
| `cad/blender/chain.py` | The knuckle-chain generator behind those presets |
| `docs/10-rotational-geometry.md` | Rotational geometry and physics idea bank: each discussion point and what it could mean for the robot, with cheapest tests |
| `docs/11-ring-joint.md` | The second primitive: axial rotation on a printed ball race, six presets; pipe bones |
| `docs/12-render-simulation.md` | Gallery render, URDF pipeline, centre-of-mass checks, MuJoCo stand and push results |
| `docs/13-tendon-simulation.md` | The drive train per DOF in MuJoCo: motor curve, self-lock, clutch, two elastic lines; tension, slack and slip readouts; standing, push, hold and squat results; three design changes it forced |
| `docs/14-balance-gauntlet.md` | Is it connected (contact census, printed-envelope shapes); the balance controller that holds 30 N pushes and a sustained 15 N; what the hip can and cannot do; weight shift and the one-foot attempt; the recorded gauntlet; the sibling-project harness |
| `docs/15-one-foot-hold.md` | The one-foot hold passes (Phase 5 exit test in simulation): the seven things it took, what the stance hip actually does, the 60 mm hip-roll pulley (D-027, D-028) |
| `docs/16-first-step.md` | The first static step completes (Phase 6a in simulation): per-phase line readout, the exit test verdict, what it took, the observer bug (D-029, D-030) |
| `docs/17-learned-control.md` | Learned control: the fast stand-in, the sibling's model core as a PPO actor, the hand-written controller as teacher, the first ladder (cloning: 90 % standing under pushes), the failing gate on the tendon model, why get-up needs a kneeling curriculum (D-031) |
| `docs/18-swing-leg-world-frame.md` | The swing leg by world-frame IK (accurate landings, steady single support with the free foot held in the world), and the finding that the step has no margin: most variants fall in the weight transfer, whatever the swing leg does (in progress) |
| `docs/19-transfer-for-margin.md` | The step with margin: no drum locks while stepping, staged transfers from planned geometry, the shift fed forward; stands in 14 of 14 grid variants, landings within 4 mm, 444 J (D-034) |
| `docs/20-mass-rollup.md` | Mass roll-up of every printed and bought part: about 26 kg against the 20 kg budget; the model on those masses (the ankle at its rating, the step 11 of 14); where mass can come off (D-036) |
| `docs/24-motor-sources.md` | Sourced low / nominal / high figures for every drive and line (`sim/motor_ranges.json`, `ROBOT_MOTORS`): the 775's Kt settled at 0.0095 N m/A; what stands at the pessimistic end and which part limits it (D-040, D-041) |
| `docs/23-static-walk.md` | The static walk: four 100 mm steps and the closing step, 42 of 42 grid variants over three mass sets, landings within 6 mm, 93 s and 1113 J; each transfer starts from the stance leg's crouch (D-039); `walk.gif` |
| `docs/22-locks-and-balance.md` | Drum locks only once the robot is still, released by any push: 199 of 200 random pushes stand (184), energy down in 9 of 11 scenes, the one-foot hold 562 J (811); teacher data re-recorded (D-038) |
| `docs/21-wmax-options.md` | W-max options weighed: a VNH5019-class driver for the 775 (25.0 kg), a 5010 brushless W-max (19.9 kg), lead-screw winches (heavier); the 775's current constant questioned |
| `tools/step_sweep.py`, `tools/step_trace.py` | The step's robustness grid (two models x three step lengths, landing errors, fall phase) and the per-0.1 s trace falls are found with |
| `tools/mass_rollup.py` | Printed (from the generators' STLs at print settings), estimated and bought mass per body segment against `cad/skeleton.json`; `--json` feeds the `ROBOT_MASSES` model variant |
| `docs/references.md` | External projects to borrow test scenarios and tooling from (primo, microban, Isaac Lab, DreamDojo, LeRobot, the sibling ai-embedded-dynamic-diversity) |
| `cad/blender/ring.py`, `cad/blender/winch_bank.py` | Ring-joint generator; N-winch bank generator |
| `cad/blender/render.py` | Headless PNG renders: one generated assembly, or the gallery of every STL in `cad/out` |
| `CLAUDE.md`, `.claude/skills/` | How to work in this repo: the project loop, the CAD generators, the tendon simulation, releases |
| `docs/releases.md` | Release history; each tag has a GitHub release with visuals, STLs and the models |
| `tools/export_urdf.py`, `tools/sim_check.py`, `sim/robot.urdf` | Joint table to URDF with CoM report; MuJoCo stand and push test on the rigid statue; the generated URDF |
| `tools/export_mjcf.py`, `tools/sim_tendon.py`, `sim/robot.xml` | Joint table to the tendon-actuated MJCF (32 winches, 96 tendons); controller, balance loop and test suite with per-line readouts; the generated model |
| `tools/sim_video.py`, `docs/img/gauntlet.mp4`, `docs/img/gauntlet.gif` | Recorder: the gauntlet with every line's tension, every drum's lock/drive/slip state, CoM and CoP drawn into the MuJoCo render |
| `docs/img/onefoot.mp4`, `docs/img/onefoot.gif` | The weight shift and the 10 s one-foot hold, recorded the same way |
| `docs/img/step.mp4`, `docs/img/step.gif` | The first static step of docs/16, recorded the same way |
| `docs/img/step-ik.mp4`, `docs/img/step-ik.gif` | The step of docs/19 (`python tools/sim_video.py --only step --gif --out step-ik`) |
| `tools/export_standin.py`, `sim/robot_standin.xml`, `sim/robot_standin.json` | The fast stand-in for training: each drive train as one position servo, with the per-DOF figures for the damage gates |
| `tools/record_teacher.py` | Demonstrations from the hand-written controller on the tendon model, in the stand-in's observation layout |
| `docs/decisions.md` | Decision log (what we chose and why; revisit when a test disagrees) |
| `docs/prompt-log.md` | Every prompt that produced a set of work, verbatim, with what it produced |
| `tools/tendon_calc.py` | Force / speed / power / line-safety calculator for any motor + drum + tackle combination; `--ranges`: each winch class at the sourced low / nom / high |
| `sim/motor_ranges.json` | Sourced low / nominal / high figures with URLs for every drive and line, keyed by `tendon_calc.MOTORS`; `ROBOT_MOTORS=low` runs the model on the pessimistic end (docs/24) |
| `tools/wmax_options.py` | The W-max drives (775 with two drivers and two readings of its Kt, the 5010 brushless at 12 and 24 V) on the 15 mm drum against the walking points, and the lead-screw winch per W-max joint (docs/21) |
| `cad/blender/winch_mini.py` | Parametric Blender generator for the W-mini winch (N20 motor); exports STL |
| `cad/out/` | Generated STLs (regenerate, do not hand-edit) |

## Quick start

```powershell
# sizing numbers for every reference joint
python tools/tendon_calc.py

# tendon-level simulation: stand, pushes, arm hold, squat, with every line's tension (pip install mujoco)
python tools/export_mjcf.py --table
python tools/sim_tendon.py                 # or --only stand,push30,onefoot; --hip50 for the 50 mm hip roll of docs/14
python tools/sim_video.py --gif            # docs/img/gauntlet.mp4 and .gif (ffmpeg on PATH)
python tools/sim_video.py --only shift,onefoot --gif --out onefoot   # docs/img/onefoot.mp4 and .gif
python tools/sim_video.py --only step --gif --out step               # docs/img/step.mp4 and .gif
python tools/export_standin.py             # sim/robot_standin.xml and .json (training stand-in, docs/17)
python tools/record_teacher.py --pushes 200   # sim/out/teacher/teacher.npz (demonstrations)

# generate the W-mini winch housing, drum and lid as STL (Blender 4.2+ / 5.x)
& "C:\Program Files\Blender Foundation\Blender 5.1\blender.exe" -b -P cad/blender/winch_mini.py -- --out cad/out
```

## Status

Design phase. Nothing has been printed yet. First hardware milestone is Phase 0 in the roadmap:
one printed finger, one W-mini winch, one ESP32, closing and holding without power.
