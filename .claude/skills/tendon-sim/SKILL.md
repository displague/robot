---
name: tendon-sim
description: Run and read the robot's MuJoCo tendon model (export_mjcf, sim_tendon gauntlet, sim_video recordings, the fast stand-in and teacher demonstrations). Use whenever a drive-train parameter, controller, pulley, preset or skeleton change needs checking, or a scene must be recorded.
---

# Tendon simulation

Python with `mujoco` (3.17 here). Everything runs from the repo root.

## Pipeline

```bash
python tools/export_mjcf.py          # cad/skeleton.json + JOINTS table -> sim/robot.xml (32 winches, 96 tendons)
python tools/export_standin.py       # -> sim/robot_standin.xml/.json (fast position-servo stand-in for RL)
python tools/sim_tendon.py           # the full gauntlet, several minutes; one line table per scene
python tools/sim_tendon.py --only stand,push30      # scene keys from sim_tendon.scenarios()
python tools/sim_tendon.py --stiff-mini             # variant: every W-mini pulley at least 25 mm
python tools/sim_tendon.py --csv                    # also sim/out/<scene>.csv time series
ROBOT_MASSES=masses.json python tools/sim_tendon.py # variant: per-bone masses from tools/mass_rollup.py --json
ROBOT_TORSO_WORM=1 python tools/sim_tendon.py        # variant: spine and shoulders on the 10 rpm worm gearmotor
```

`ROBOT_MASSES` and `ROBOT_TORSO_WORM` are read by `export_mjcf`, so they also apply to `step_sweep.py` (its
children inherit the environment) and `step_trace.py`. Running `export_mjcf.py` with either set rewrites
`sim/robot.xml` as the variant: re-run it without them before committing.

Scene keys: stand, push10, push20, push30, push40, sustained, arm, squat, shift, onefoot, step. Run long
gauntlets in the background with output to a file in the scratchpad, and keep working meanwhile.

Always regenerate both models after touching `tools/export_mjcf.py`, `cad/skeleton.json` or a preset, and
commit `sim/robot.xml` and `sim/robot_standin.*` with the change.

`docs/05-bom-estimate.md` names the part behind every winch class and motor in `CLASSES` / `MOTORS`
(D-035). Changing a motor, ratio, line or winch count in the model means changing its row and the totals
in `05` in the same commit; `python tools/export_mjcf.py --table` gives the per-DOF classes to count.

## Drive options off the model (desk calculations)

```bash
python tools/tendon_calc.py              # scenarios; MOTORS has 5010_PLAN16 / 5010_PLAN25_24V (brushless: kv, r_ohm, ratio)
python tools/wmax_options.py [--drum|--screw]   # W-max drives on the 15 mm drum vs the walking points; lead-screw winch per joint
```

For a `Motor` with `kv` set, `stall_torque_nm` is the torque at the driver's peak current and peak power is taken
along the voltage line at that current (`motor_peak_power_w`). `wmax_options.py` carries the 775 twice: the model's
figures (Kt 0.024) and Kt from its 12 000 rpm (0.0095, so ~18 A at the D-021 limit, docs/21). Neither tool touches
`sim/robot.xml`; a brushless W-max class in the model does not exist yet.

## Reading a scene's table

`== <scene>: t, pelvis height, tilt -> STANDS|FALLS; electrical energy (mean W)`, then lines worst first:

| Column | Meaning |
| --- | --- |
| peak N / cont N | the line's peak tension against its continuous rating (W-mini 37 N, W-mid 64, W-max 288) |
| SF | break strength / peak; under 4 is flagged |
| slack% | share of time the run is slack |
| slip dg | clutch slip in degrees (the drive was overpowered) |
| tq/stl | motor torque as a fraction of stall |
| over% / lock% | time above rating / time the drum is self-locked |
| J, hot s | electrical energy, seconds above the continuous rating |

The `flags:` line lists what fails a gate. The step scene adds per-phase rows (peak lines, slack over 25 %,
drums locked) and the planner's prints (`R foot placed ... ahead of the other (planned ...)`): the landing
error is the difference.

## Quick parameter sweeps

Do not edit the repo to try a value. In a scratchpad script: `sys.path.insert(0, "<repo>/tools")`,
`import export_mjcf as em`, patch `em.JOINTS[...]` before `import sim_tendon as st`, then
`rig = st.Rig()` and run `st.scenarios()` entries (`key, title, note, fn`) with `rig.reset(); fn(rig)`.
`rig.report(title, top=N)` returns the table rows (name at index 1, peak 2, cont 3) and `rig.standing()`
the verdict. Run several variants as parallel processes. Commit the chosen values, not the sweep script.

## The step: judge changes on the grid, not one run

The step is deterministic and sits on a knife edge: one millimetre flips a run. Any change to the step,
the balance loop or the leg drive trains is judged with

```bash
python tools/step_sweep.py "before: SWING_IK=False" "after: SWING_IK=True IK_LATERAL=True"
python tools/step_trace.py --from 13.5 --to 20 SWING_IK=True     # then trace a failing row
```

The sweep runs every configuration on the D-032 baseline and `--stiff-mini` at 90, 100 and 110 mm steps in
parallel; each cell is `S` or `F<phase>` (where the pelvis first tilted past 20 degrees), `g` if a lift gate
timed out, and the two landing errors in mm. The trace prints loads, CoM error and velocity, pelvis attitude,
hip shift, ankle positions and leg joints (measured/applied) every 0.1 s. The default step is docs/19's (`SWING_IK`, `IK_LATERAL`, `TRANSFER_PLAN`, `STEP_LOCKS=False`); `--open-swing`
runs docs/16's. The other flags of docs/18 and 19 stay for comparison. Use `--lengths 0.085,...,0.115` (14
variants) when comparing configurations: 6 variants were too few to separate them.
Add `--masses 20kg=,775=sim/masses/rollup_775.json,worm=sim/masses/rollup_worm.json` (42 variants) for any
change to the step or the balance: the step once stood on one mass set only (docs/19, D-037). The sweep
starts every run at once (42 processes per configuration), so run configurations one after another.

Lesson (docs/19): if a grid will not settle, look for a limit cycle before tuning stages. Run the scene
that feeds the failing phase on its own, measure the ripple of a load or CoM error, and switch mechanisms
off one at a time (drum locks, joint damping, ankle modes, loop gains); the drum locks were it.

## Recordings

```bash
python tools/sim_video.py --gif                                   # docs/img/gauntlet.mp4 + .gif (ffmpeg on PATH)
python tools/sim_video.py --only shift,onefoot --gif --out onefoot
python tools/sim_video.py --only step --gif --out step
```

Line tension, drum state, CoM and CoP are drawn into the frames. Re-record a scene whose behaviour a
change altered; these GIFs are the release visuals.

## Learned control hooks

`python tools/record_teacher.py [--pushes 24] [--jobs 8]` writes `sim/out/teacher/teacher.npz`
(gitignored): the hand-written controller's targets on the stand-in's observations, for behaviour cloning
in the sibling project (ai-embedded-dynamic-diversity, `rl/`). Re-record after a controller or model change.
