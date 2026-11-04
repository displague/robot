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
ROBOT_MOTORS=low python tools/sim_tendon.py         # variant: every drive and line at the pessimistic end of its sourced range
ROBOT_MOTORS=high | nom | <file.json>[:case]         # the optimistic end, the sourced nominal, or a file in the same format
```

`ROBOT_MASSES`, `ROBOT_TORSO_WORM` and `ROBOT_MOTORS` are read by `export_mjcf`, so they also apply to `step_sweep.py`
(its children inherit the environment) and `step_trace.py`. Running `export_mjcf.py` with any of them set rewrites
`sim/robot.xml` as the variant: re-run it without them before committing, and check `git diff sim/robot.xml` is empty.

## Motor and line ranges (D-040, docs/24)

`sim/motor_ranges.json` holds low / nom / high for every drive figure (stall torque, no-load speed, stall current,
continuous fraction, gearbox efficiency `eta`, mass; kv and r_ohm for the brushless) keyed by `tendon_calc.MOTORS`
names, and the effective break strength of each line (as terminated: wrapped on drum and pulley), each with its
sources. `low` is the pessimistic end of every figure at once (for stall current the most amps per N m). Unset,
the model runs on `tendon_calc`'s own figures (the committed `sim/robot.xml`); `nom` is the sourced estimate,
which is not the same (the 775's 66 A stall, eta 0.5). A case moves the parts only: pretension, the clutch's
slip force (a third of the spool label) and the line stiffness stay at the model's settings, so a weaker line
shows as a lower SF and a weaker motor as less margin. `python tools/tendon_calc.py --ranges` prints each winch
class per case (drum stall, continuous and clutch line force, SF at the clutch, current at the clutch, the W-max
walking-point speeds). Every new motor or line figure goes into the JSON with a source URL or a derivation.

To find which part limits a failing case, write mixed files in the scratchpad (a copy of the JSON whose `nom`
is nominal except the parts under test at their `low`) and run them as `ROBOT_MOTORS=<file>` on the failing
scenes, then on the walk grid. A build is recommended only if the walk grid, the one-foot hold and push40
stand at `low` (D-040).

Scene keys: stand, push10, push20, push30, push40, lean15, arm, squat, gust (8 m/s head wind with gusts, docs/25), shift, onefoot, step, walk (docs/23,
about 95 s; judge it with `step_sweep.py --scene walk`). Run long
gauntlets in the background with output to a file in the scratchpad, and keep working meanwhile.

Always regenerate both models after touching `tools/export_mjcf.py`, `cad/skeleton.json` or a preset, and
commit `sim/robot.xml` and `sim/robot_standin.*` with the change.

`docs/05-bom-estimate.md` names the part behind every winch class and motor in `CLASSES` / `MOTORS`
(D-035). Changing a motor, ratio, line or winch count in the model means changing its row and the totals
in `05` in the same commit; `python tools/export_mjcf.py --table` gives the per-DOF classes to count.

## Drive options off the model (desk calculations)

```bash
python tools/tendon_calc.py --ranges     # each winch class at the model's figures and at low / nom / high (docs/24)
python tools/tendon_calc.py              # scenarios; MOTORS has 5010_PLAN16 / 5010_PLAN25_24V (brushless: kv, r_ohm, ratio)
python tools/wmax_options.py [--drum|--screw]   # W-max drives on the 15 mm drum vs the walking points; lead-screw winch per joint
```

For a `Motor` with `kv` set, `stall_torque_nm` is the torque at the driver's peak current and peak power is taken
along the voltage line at that current (`motor_peak_power_w`). `wmax_options.py` carries the 775 twice: the model's
figures (Kt 0.024) and Kt from its 12 000 rpm (0.0095, so ~18 A at the D-021 limit, docs/21); docs/24 settled
it at 0.0095 (0.0088 to 0.0100) from nine 775 datasheets, so the second reading is the real one. Neither tool touches
`sim/robot.xml`; a brushless W-max class in the model does not exist yet.

## Wind (docs/25, D-042)

```bash
python tools/sim_tendon.py --only stand --wind 10,1.5,head      # mean m/s, gust sigma m/s, head|tail|left|right|deg
python tools/wind_sweep.py --scenes stand,step --dirs head,left --speeds 4,8,12 --gusts 0,1.5 --seeds 0,1 [K=V]
python tools/sim_video.py --only gust --gif --out gust          # the wind is drawn as a cyan arrow, its speed in the HUD
```

`WIND` (module constant, also a `step_sweep` K=V: `"WIND=dict(mean=5.0,gust=1.5,dir='head')"`) or
`rig.set_wind(spec)` in a scene after `reset()`. Drag is MuJoCo's fluid model (`opt.density`, `opt.wind` each
step) on each body's inertia box; forward is -y, left +x; `dir` is where the wind comes from. The report prints
the wind and the drag (mean, peak). `wind_sweep.py` runs every scene x direction x speed x gust x seed as its
own process (`WIND_JOBS` at once; `stand` there is 20 s) and prints one table per scene and direction. Do not
edit `sim_tendon.py` while a sweep runs: each child imports it when it starts. The balance loop leans into a
sustained force (`LEAN`, `LEAN_TAU`, `LEAN_TAU_X`, `LEAN_DEAD`, `LEAN_MAX`, `LEAN_STAGGER`) and the lock gate
releases when the disturbance estimate moves (`LOCK_RELEASE_D`). `record_teacher.py --winds N` adds N standing
episodes in random winds. The sibling's stand-in has the same wind (`EnvConfig.wind_mean`, `wind_gust`;
`--wind-mean lo,hi --wind-gust lo,hi`; sibling PR #54).

## Contacts (D-043, docs/25)

The default contact is MuJoCo's soft one with friction 1.0, and a loaded foot creeps under a sustained side load
(78 mm in 20 s in a 10 m/s wind). `NOSLIP=5` (noslip solver iterations, about 40 % slower) holds the feet;
`FLOOR_MU=0.4` sets every contact's friction (a hard floor). Both are module constants, so they work as K=V on
`step_sweep.py` and `wind_sweep.py`. Any change to the step or the walk is also judged on
`step_sweep.py ... "held: NOSLIP=5"` and `"hard: NOSLIP=5 FLOOR_MU=0.4"`. Today these stand 23 and 40 of 42 (step)
and 16 and 13 (walk), against 42 on soft contact.
`TP_PRESS_FROM_LANDED=True` (with `TP_PRESS_CATCHUP`, 0.05 m/s) starts the landed leg's press from its measured
length. On held feet the step then stands 40 and the walk 23; it is off by default because the soft-contact walk
drops to 39 (docs/25, P-042). P-045's sweep of the press and swing settings found nothing past the grid's scatter.
For the energy of a walk per drive, sum `rig.stats[name]["e_j"]` after the scene (docs/27: hip roll, ankle
pitch and knee carry three quarters, all holding torque). `COM_REF_BACK` moves the balance loop's CoM target
toward the ankle (off: it costs the arm hold and the walk grid).
`export_mjcf.TACKLE_OVERRIDE` sets the strands at a joint axis (`{"ankle.pitch": 4}`); the line stiffness follows.
From a sweep variant: `"a4: _=__import__('export_mjcf').TACKLE_OVERRIDE.update({'ankle.pitch':4})"`. Always run the
gauntlet too: a 4:1 ankle passes both grids but drops the arm hold (D-047, docs/27).
`export_mjcf.BAGS` hangs a carried bag on a ball joint from a link's tail (`{"hand.L": 3.9}`; non-colliding, the
balance loop sees its mass through `subtree_com`). `tools/carry_bag.py kg= style=down|both|front scene=` runs
one gauntlet scene with it and prints one line (stands, energy, the bag's swing, line peaks and safety factors).
The fingers are not in the model, so the grip is estimated by hand (docs/28).

## Sensors and postures (docs/26, D-044)

```bash
python tools/sensor_sweep.py sweep cases=truth,lc_fast,fsr_fast,fsr_bad scenes=push40,step winds=0:0:head,10:0:head
python tools/posture_wind.py static                                   # drag and CoM offset needed per posture
python tools/posture_wind.py sweep postures=stand,bow35 speeds=12,14 dirs=head,left gusts=0,1.5
T0=2.5 python tools/posture_wind.py one bow35 14 0 head 0.5           # a posture started after the wind
```

`sensor_sweep.py` subclasses `Rig`: the balance loop and the planner read a CoM with delay, bias, wander and noise
and 4 corner load sensors per sole with gain, hysteresis, drift and saturation (`CASES`; `only_<error>` and
`delay<ms>` isolate one error). `lc_fast` (1 kHz, 3 ms, load cells) is the build's target. Judge a controller
change on it as well as on truth: a 10 ms delay already loses push 40.

## Energy (docs/27, D-045)

`python tools/energy_report.py walk 3 head` prints a scene's motor energy (per winch class), mean / p99 / peak
power. Run it with `ROBOT_MOTORS=nom ROBOT_MASSES=sim/masses/rollup_775.json` for the real build: the model's own
775 figures understate the walk's energy 3.4 times. For a battery add about 8 W of electronics and 10 % for the
drivers.

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
Since D-038 the locks are gated on whole-body stillness (`LOCK_STILL`, 0.5 s under 20 mm/s; released above
80 mm/s) and off in the shift and one-foot hold (`BALANCE_LOCKS`). A change to the balance loop is also
judged on the 200 random pushes of `record_teacher.py` (199 stand).

## Recordings

```bash
python tools/sim_video.py --gif                                   # docs/img/gauntlet.mp4 + .gif (ffmpeg on PATH)
python tools/sim_video.py --only shift,onefoot --gif --out onefoot
python tools/sim_video.py --only step --gif --out step
python tools/sim_video.py --only walk --gif --out walk     # then re-encode the GIF at 4x (docs/img/walk.gif: ffmpeg setpts=0.25*PTS, fps 12, 480 wide)
```

Line tension, drum state, CoM and CoP are drawn into the frames. Re-record a scene whose behaviour a
change altered; these GIFs are the release visuals.

## Learned control hooks

`python tools/record_teacher.py [--pushes 24] [--winds 0] [--jobs 8]` writes `sim/out/teacher/teacher.npz`
(gitignored): the hand-written controller's targets on the stand-in's observations, for behaviour cloning
in the sibling project (ai-embedded-dynamic-diversity, `rl/`). Re-record after a controller or model change.

The stand-in's drive (`tools/export_standin.py`) is fitted to the tendon model (docs/29, D-050): `KV_SCALE`,
`TIMECONST` and `outer_loop` in the JSON (the firmware's KQ, KI, trim limit, per-joint gain), which the sibling's
`StandinEnv` runs at its control rate (`EnvConfig.outer_loop`). If `sim_tendon`'s KQ, KI, TRIM_MAX or OUTER_GAIN
change, re-export, and re-check the fit: step one joint's target by 0.03 rad with the robot standing (tendon model
with `balance = False`) and compare the fraction reached at 0.1, 0.4 and 1 s with the stand-in's.
The gate: `PYTHONPATH=src ROBOT_REPO=../robot python -m ai_embedded_dynamic_diversity.rl.tendon_eval <policy.pt|teacher> 20`
in the sibling (stand 10 s, 10/20/30 N, wind 5 and 8 m/s, 20 random pushes; `teacher` runs the hand-written
controller on the same scenes). Under a policy the balance loop is off; `Rig.still_gate = True` runs D-038's
stillness gate anyway (on the CoM speed alone), so drums can lock under a policy, and the sibling's gate sets it
(docs/31). `... tendon_eval <policy.pt|teacher> rest` runs the rest scenes: idle 30 s and a 3.9 kg bag 20 s, power
and locked fraction over the last 20 s (the hand-written controller: 5.4 W idle, 9.2 W with the bag).

Rests and locks in the stand-in (docs/31): the JSON carries per DOF `volt`, `i_stall_a`, `gear_ratio`,
`rotor_j_joint` (the motor and gear inertia in joint terms) and a `locks` dict (the lock rule in joint terms:
tol per DOF, err_max, rate per DOF, still_s, still_v, release_v), and the XML a `com_vel` sensor (subtree linear
velocity of the pelvis: the whole body's CoM). If `LOCK_*`, `LOCK_TOL_SCALE` or a motor changes, re-export. The
sibling's `StandinEnv(locks=True, power_cost=c)` uses them: power V I_stall |(u + w/w_nl) u| with u the joint
torque plus the rotor's inertia torque over stall, zero on a locked drum.
