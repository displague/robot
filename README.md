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

The docs (design notes and experiments, one per topic) are indexed with a summary each in
[`docs/index.md`](docs/index.md), generated from their front matter; `mkdocs serve` browses them as a site.

| Path | What |
| --- | --- |
| `docs/index.md`, `mkdocs.yml` | The docs' index by section, generated from each doc's front matter (`tools/docs_index.py`); the docs site |
| `cad/blender/chain.py` | The knuckle-chain generator behind those presets |
| `tools/sensor_sweep.py` | Any scene on modelled sensors (IMU/encoder CoM with delay, bias and noise; 4 load sensors per sole with gain, hysteresis, drift, saturation) instead of the simulator's truth (docs/26) |
| `tools/posture_wind.py` | Drag and CoM offset needed per posture, and whether a posture (bow, squat, arms in, twist) stands in a wind (docs/26) |
| `tools/energy_report.py` | A scene's motor energy per winch class, mean / p99 / peak power (docs/27) |
| `tools/carry_bag.py` | One gauntlet scene with a swinging bag in one hand, both, or on a strap at the wrist (docs/28) |
| `tools/wind_sweep.py` | The wind failure map: every scene x direction x mean speed x gust strength x seed in parallel (docs/25) |
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
| `tools/export_standin.py`, `sim/robot_standin.xml`, `sim/robot_standin.json` | The fast stand-in for training: each drive train as one position servo with a target lag and the firmware's outer loop (fitted, docs/29), the per-DOF figures for the damage gates, and the motor power and drum-lock rule (docs/31) |
| `tools/record_policy.py`, `docs/img/wind8-alone.gif`, `docs/img/wind8-hybrid.gif` | A learned policy (the sibling's) recorded on the tendon model through the gate's own runner; the two clips: plain seed 3 in the 8 m/s wind alone (falls) and as the hybrid under the hand-written ankle balance (stands) (docs/32) |
| `tools/sit_static.py`, `tools/chair_sweep.py`, `docs/img/sit40.gif` | Leaving a chair: the static seat-off check (poses with the CoM over the soles against the knee, hip, ankle and spine drives; `hands`: how hard the hands can push on the armrests), the chair grid (sit, stand again, line peaks, slip, seated power, energy per seat height and armrests) and the 0.40 m recording (docs/33) |
| `tools/rise_static.py` | Whether the robot can rise slowly on one leg: the lowest pelvis height with the CoM over a flat foot, by geometry and within the knee, hip and ankle drives, per ankle dorsiflexion (docs/32) |
| `tools/record_teacher.py` | Demonstrations from the hand-written controller on the tendon model, in the stand-in's observation layout |
| `docs/decisions.md` | Decision log (what we chose and why; revisit when a test disagrees) |
| `docs/prompt-log.md` | Every prompt that produced a set of work, verbatim, with what it produced |
| `tools/tendon_calc.py` | Force / speed / power / line-safety calculator for any motor + drum + tackle combination; `--ranges`: each winch class at the sourced low / nom / high |
| `sim/motor_ranges.json` | Sourced low / nominal / high figures with URLs for every drive and line, keyed by `tendon_calc.MOTORS`; `ROBOT_MOTORS=low` runs the model on the pessimistic end (docs/24) |
| `tools/wmax_options.py` | The W-max drives (775 with two drivers and two readings of its Kt, the 5010 brushless at 12 and 24 V) on the 15 mm drum against the walking points, and the lead-screw winch per W-max joint (docs/21) |
| `cad/blender/winch_mini.py` | Parametric Blender generator for the W-mini winch (N20 motor); exports STL |
| `cad/out/` | Generated STLs (regenerate, do not hand-edit) |

## Quick start

```sh
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

# generate the W-mini winch housing, drum and lid as STL (Blender 4.2+ / 5.x on PATH)
blender -b -P cad/blender/winch_mini.py -- --out cad/out

# the docs as a site (docs/index.md is regenerated from the docs' front matter)
pip install -r requirements-docs.txt
mkdocs serve
```

## Status

Design phase. Nothing has been printed yet. First hardware milestone is Phase 0 in the roadmap:
one printed finger, one W-mini winch, one ESP32, closing and holding without power.

## License

Code (the generators, simulation and tools) is [MIT](LICENSE). The printable designs (the generated
STLs and release STL zips, the part geometry and the part renders) are
[CC BY-SA 4.0](LICENSE-PRINTABLES.md): credit displague/robot and share derived designs alike.
