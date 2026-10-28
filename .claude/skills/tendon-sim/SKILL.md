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
```

Scene keys: stand, push10, push20, push30, push40, sustained, arm, squat, shift, onefoot, step. Run long
gauntlets in the background with output to a file in the scratchpad, and keep working meanwhile.

Always regenerate both models after touching `tools/export_mjcf.py`, `cad/skeleton.json` or a preset, and
commit `sim/robot.xml` and `sim/robot_standin.*` with the change.

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
