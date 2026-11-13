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
ROBOT_WMAX=bldc [ROBOT_WMAX_KG=2.09]                 # variant: W-max on the 5010 on 1:25, 24 V: no lock, FOC energy, gains x kg (docs/34)
ROBOT_SHOULDERS_LOW=1                                # variant: the shoulder sheaths 0.85 m (drives in the pelvis; rejected, docs/36)
ROBOT_ITEMS=items.json                               # free items a hand can grip, and two stools (docs/38; tools/chain.py sets them itself)
ROBOT_WMAX=bldc ROBOT_WMAX_RATIO=35                  # the 5010 on another printed ratio (docs/40: 35 and 40 swing too slowly)
```

`ROBOT_MASSES`, `ROBOT_TORSO_WORM`, `ROBOT_MOTORS`, `ROBOT_WMAX` and `ROBOT_SHOULDERS_LOW` are read by `export_mjcf`, so they also apply to `step_sweep.py`
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
length. It is **on by default since D-064** (docs/40). It takes the D-060 layout's held step from 0 to 14 of 14,
costs the gauntlet's step 66 J, and leaves every 20 kg grid level or better. Off, the landed leg is replanned at
its unpressed length and pulls up; a heavier leg then kicks the body sideways over the stance foot. On held feet
with the layout masses the walk still stands only 4 of 14. `TP_B=6 ROLL_KD=0.5` makes it 13 on the 775 layout
but not the brushless one (2), so it is a per-build setting, not a default (docs/40). Gain sweeps (`SS_SLEW`, `SWING_ROLL_TAU`, `SS_KP`,
`WALK_BACK`) stay within the scatter, as P-045's did.
For the energy of a walk per drive, sum `rig.stats[name]["e_j"]` after the scene (docs/27: hip roll, ankle
pitch and knee carry three quarters, all holding torque). `COM_REF_BACK` moves the balance loop's CoM target
toward the ankle (off: it costs the arm hold and the walk grid).
The chair's rise paces the hip extension (`SIT_HIP_RATE`, 22.5 deg/s, D-067). Faster lets the pelvis rotate back
under the lagging trunk, and the spine folds to its stop. `SIT_SPINE_GATE` and `export_mjcf.SPINE_RELIEF` are
variants that were tried and are off (docs/41). The trunk rests on the thighs while seated: that contact is
load-bearing.
The gauntlet's line flag (a safety factor under 4) sits below the clutch's own setting (a third of break), so a
drive working at its clutch is always flagged. The chair's spine pitch is one, at 290 to 350 N (docs/41).
`JOINTS[...]["tackle_by_axis"]` gives one axis its own strand count (the neck's pitch is 2 since D-066, docs/41:
the chair's trunk swing slipped its W-mini on 1). `export_mjcf.TACKLE_OVERRIDE` sets the strands at a joint axis (`{"ankle.pitch": 4}`); the line stiffness follows.
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
judged on the 200 random pushes of `record_teacher.py` (198 stand at v0.6.0; 199 when last recorded).

## Recordings

```bash
python tools/sim_video.py --gif                                   # docs/img/gauntlet.mp4 + .gif (ffmpeg on PATH)
python tools/sim_video.py --only shift,onefoot --gif --out onefoot
python tools/sim_video.py --only step --gif --out step
python tools/sim_video.py --only walk --gif --out walk     # then re-encode the GIF at 4x (docs/img/walk.gif: ffmpeg setpts=0.25*PTS, fps 12, 480 wide)
```

Line tension, drum state, CoM and CoP are drawn into the frames. Re-record a scene whose behaviour a
change altered; these GIFs are the release visuals.

A learned policy (the sibling's `.pt`) on the tendon model, through the sibling gate's own runner so the clip
reproduces the gate's numbers. Run from the sibling root; one recording per process (a second MuJoCo renderer
in one process renders black):

```bash
PYTHONPATH=src ROBOT_REPO=../robot .venv/Scripts/python.exe ../robot/tools/record_policy.py \
    artifacts/rl/seeds/mlp-bc-s3.pt --wind 8 --hybrid --out wind8-hybrid   # also --push N, --secs S, --title T
```

## Learned control hooks

`python tools/record_teacher.py [--pushes 24] [--winds 0] [--jobs 8]` writes `sim/out/teacher/teacher.npz`
(gitignored): the hand-written controller's targets on the stand-in's observations, for behaviour cloning
in the sibling project (ai-embedded-dynamic-diversity, `rl/`). Re-record after a controller or model change.
`--set chair` writes `teacher_chair.npz` (the sit scenes; 6 of 8 stand), `--set chain --chains N` writes
`teacher_chain.npz` (chain.py seeds, each in a fresh worker since each bag is its own model; 17 of 24). They are
separate files because their offsets need about 0.70 rad action spans against 0.27 standing. Every file has `felt`
(steps x 2: the carried weight over the robot's, as `held_load` truth and as the D-063 estimate) beside the 72-channel
`obs`. Smoke check: the sibling's `rl.bc.load_teacher(path, teacher_action_half(path))` on each. Do not edit
`sim_tendon.py` or `export_mjcf.py` while one runs: every task's worker imports them afresh. The release set (docs/43):
`--pushes 200 --winds 24 --jobs 10`, then `--set chair --jobs 8`, then `--set chain --chains 24 --jobs 10`.

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

`tools/rise_static.py [dorsi=N]` answers whether the robot can rise slowly on one leg: the lowest pelvis height with
the CoM over a flat front foot, by geometry, within the knee and hip drives and within the ankle drive, per ankle
dorsiflexion limit (docs/32: 0.90 m at the
joint's 20 degrees). Re-run it after a leg geometry, ankle range or drive change.

## The chair (docs/33)

```bash
python tools/sim_tendon.py --only sit40,sit45a     # sit on a 0.40 m chair (sitNNa: with armrests) and stand again
python tools/chair_sweep.py                        # 0.35-0.50 m x armrests, one process each: sits, stands, line peaks, slip, seated W, J
ROBOT_MASSES=sim/masses/rollup_775.json ROBOT_MOTORS=low python tools/chair_sweep.py   # another build (K=V sets sim_tendon constants)
python tools/sit_static.py [margin=0.09 spine=0.9]  # static seat-off poses against the knee, hip, ankle and spine drives
python tools/sit_static.py hands [armh=0.20]        # can the hands reach the armrests, and how hard can they push
python tools/sim_video.py --only sit40 --gif --out sit40
```

The chair is four mocap bodies in `sim/robot.xml` (`export_mjcf.CHAIR_GEOMS`), out of the way and not colliding until
`Rig.set_chair(seat, arms)` places them (after `reset()`, which clears them). A geom moved or resized at run time is
missed by MuJoCo's collision tree, and a body whose geoms were all off at compile needs its `body_contype` set too;
both are why they are mocap bodies. `Rig.chair_load()` is the seat's and the armrests' vertical force; `rig.mass`
leaves the chair out (counting its 4 g changed the step's energy). Only `sit45` runs in the default gauntlet
(`NAMED_ONLY`); `record_teacher` skips every `sit` scene.

`sit_stand(r, seat, arms)`: down `sit_path` (the hip solved per pose so the CoM keeps a growing share of the seat-off
reach behind the sole centre) with the balance loop on; seated with every target set where the seat stopped it and
`still_gate` on; up by a trunk swing (`SIT_LEAN_T`), then a knee-led rise gated on the measured CoM, with the other
joints following the path at the *measured* knee angle and the ankles in position mode until `SIT_BAL_ON`. Judge a
change on `chair_sweep.py` and its nudges (`SIT_KNEE_RATE=radians(18|35)`, `SIT_LEAN_T=(0.8,1.3,0.065)`): 10 of 12
pass today, so one passing run proves little. The spine's clutch is the binding drive (it sags from 35 to 7 degrees
with the arms forward); a rise that leans on the chair's back looks like standing: check `chair_load()`.
Judge it on the full grid too (docs/33): the 20 kg model and `ROBOT_MASSES=sim/masses/rollup_775.json`, each with
`ROBOT_MOTORS=nom` and `low`. Run the four in sequence (they take a few minutes each at `CHAIR_JOBS=8`) or in
parallel at `CHAIR_JOBS=2` or 3. The 0.50 m case flips with small changes, so read 0.40 and 0.45 m first; the 26 kg
build stands only from 0.50 m (D-057), and from 0.45 m on the D-060 layout (`sim/masses/rollup_775_layout.json`).
The rise is decided by the static reach (`sit_path`'s second return, the CoM behind the sole centre at seat-off;
the model passes at 7.8 cm at 0.45 m). To see what a mass change does, compute the reach per mass set first (one
process each, seconds): docs/36 has the reach per kilogram of each segment. Judge any mass layout on the full grid's held feet too (`"held: NOSLIP=5"`, 7 lengths): docs/37's layout passed soft contact and failed there. `step_sweep.py` starts every run of a call at once (42 runs for three mass sets); split big grids into calls of 14 to 28, or the host may kill them for memory. In a shell, `S=... && (...) &` runs the assignment in the background subshell:
set and export the variable on its own line first.

## Chains: skills back to back, and items to pick up and put down (docs/38)

`tools/chain.py --runs 24 --jobs 12 [--no-handback] [K=V ...]` runs one episode per seed. Each episode does pick,
walk, sit (0.45 m) and place in a random order, with no reset between them. Each seed draws one of six bags
(`chain.BAGS`: shapes, sizes and 0.8 to 4.5 kg). Keep `--jobs` at 12 to 14, or the host kills the runs for
memory. `--one <seed>` prints one run's full log.

`tools/chain_video.py --bag grocery --order pick-walk-sit-place --gif --out chain` records one chained run with
`sim_video.Recorder` (`--seed N` takes a seed's bag and order; K=V overrides as chain.py). The recorder's own GIF
is about 20 MB for a 140 s chain: remake it from the MP4 at 4x speed, 480 px, 10 fps with an ffmpeg palette
(`setpts=PTS/4,fps=10,scale=480:-1,split[a][b];[a]palettegen=max_colors=96[p];[b][p]paletteuse`), about 4 MB.

- **Items** (`export_mjcf.ITEMS`, or `ROBOT_ITEMS` as a JSON file or string) are free bodies with a handle.
  - A hand holds one through a connect equality between `grip.hand.L` and `item.<name>.grip`. `Rig.grip(hand, name, on)` switches it.
  - `Rig.held_load` is the grip's vertical force: the felt weight, ground truth.
  - The items collide only with the floor and the stools (contact bit 2), never with the robot.
- **Stools** are mocap boxes. `Rig.place_stool(k, xy, top)` places one, or parks it when `xy` is None.
- **Placing things where the robot now stands:** `Rig.feet_origin()` is where it stands, relative to the start.
  Set `r.origin` to it before `sit_stand`, and `set_chair` places the chair from there.
- **`Rig.to_stand()` hands back a quiet stand between skills.** It clears the swing flags, the CoM bias and the
  knee torque sources, and eases every target back to the standing pose. The walk leaves the swing flags set,
  the hip shift at 7 degrees and the arms out, so a chair right after it fails without the hand-back.
- **Code that sums the whole world must skip the items.** Use `subtree_com[pelvis]` rather than
  `subtree_com[0]`. Skip floor contacts with contype bit 2 in `cop()`. `Rig.mass` leaves out the items.
  With no items the model and every scene are unchanged.
- **Held items (docs/39, D-063):** `FEEL_LOAD` (on) adds the felt grip weight to the balance loop's CoM.
  `GRIP_DAMP` (0.5 N m s/rad) damps an item's rotation against the hand. A freely pivoting bag with no felt weight
  fails nearly every walk. `GRIP_DAMP` is an explicit torque: 1.0 spikes a light bag and 2.0 blows up, because the
  hand's inertia is small. Both act only when the model has items.
- **Sitting with a bag:** `chain.SIT_SETDOWN` (on, D-065) sets the bag down on a top before a sit and takes it up
  after. A bag held through the sit lands on the floor, and the robot cannot stand. The put-down lets go only once
  the item is still. Today's grid: 18 of 24 chains done. The six failures left are one-handed walks with 3 kg or
  more. `chain.SPLIT` (off) carries a bag over `ONE_HAND_KG` in both hands, split; its two-hand take-up and
  put-down are not reliable yet (docs/39).
- **To diagnose a carried item,** give the balance loop the item's true CoM (`Rig.com_est` overridden) as an upper
  bound. That separates "the loop does not know it" from "the walk cannot carry it".


## The nurturing helper (docs/44, D-070)

`python tools/helper.py --phases` (the per-phase table on the 0.35 m chair), `--fade`, `--one <phase> [--trace]
[give= cap= hands= on_reach= arms_hold= G_FIRM= ...] [--video name]`. `export_mjcf.HELPER` (or `ROBOT_HELPER=1`)
adds two mocap hands, a `wrist.hand.*` site per hand and the `helper.L/R` connect equalities (off); without it the
model is unchanged. `helper.Helper(rig, give, hands, on_reach, cap, drunk, arms_hold)` with `rig.hook = h.chain()`
runs it every step; `h.log` holds the impulse, work, peak, time held and the arm lines' peaks (held and over the rise
window). The robot's side is `sim_tendon.SIT_ARMS_HOLD` (rad: the arms forward till the knees pass it; off by default).
Lessons: hold the wrist, not the grip point (the wrist clutch slips); pull along the arm, never across it (the elbow
overloads); take hold only once the arm is straight; never step a target when the hold ends. Judge the overload law
against the unassisted rise window (the elbows run 1.42x their rating there) and keep the table's D row.

## Training the rise (docs/45, D-071)

- `python tools/helper.py --drunk` is the drunken-sailor table: the wander, the lopsided helper and one hand.
- `python tools/rise_train.py` trains the chair rise's 11 parameters (`PARAMS`) by CEM on the full model, one process
  per run, 13 at a time. A generation of 13 candidates takes about 80 s.
  - The helper's levels (`LEVELS`) are the curriculum. `--start-level 7` is the no-helper control.
  - `--grid`, `--probe` and `--show` read a run's `best.json`.
  - Runs go to `sim/out/rise*` (gitignored); keepers go to `sim/rise_params.json`.
- Lessons:
  - score an unassisted old seat in every generation, or the search trades it away;
  - run a no-curriculum control: here it beat the curriculum;
  - jitter-test a winner before adopting it: `--jitter-test file:key --n 24` (2% jitter, per seat).
- `--robust file:key` retrains with no helper, every run jittered and all four seats scored. Its set is the
  default for seats under 0.42 m: `sim_tendon.SIT_LOW` and `SIT_LOW_BELOW` (D-072). A trainer that sets the `SIT_*`
  globals itself must set `st.SIT_LOW = None`, or the schedule overrides its parameters on the low seats.
  - Don't use `cat > file` without a heredoc in a backgrounded Bash call: it waits on stdin forever.

## Getting up with the helper (docs/46, D-073)

- `python tools/getup.py --settle` checks the half-kneel start (`getup.kneel(r)` homes gears and drums at a pose
  and drops the body onto the floor).
- `GETUP_GRIP=elbow python tools/getup.py --train A --reps 2 [--fade] [--init best.json] --out sim/out/getup_x`
  searches `GPARAMS`, the script's 17 parameters.
  - `--fade` walks the helper through `FADE`.
  - `best.json` keeps `best`, `last`, `mean` and `levels`. Use `last` once the best score saturates at 10.
  - `--jitter-test file:key --n 24` checks a set; `getup.gapply(params)` applies one.
- The get-up's helper is `getup.Lifter`, not `helper.Helper`. Keepers are in `sim/getup_params.json`.
- Lessons:
  - hold under the arms (wrists overload the elbows);
  - lift only what the feet leave, below standing height, or it hangs the robot;
  - shift the weight before lifting a foot, and lower the foot until it carries weight;
  - a script run under multiprocessing spawn must be a file, not stdin.

## The role flip's statics (docs/47, D-074)

- `python tools/role_flip.py --record --set firm` samples docs/46's helper (positions and forces).
- `python tools/role_flip.py --set firm [--scale 0.5] [--contact elbow] [--evals 800]` searches a second robot's
  posture per sample.
  - It compares each pitch joint's static torque with its continuous rating (`sim/robot_standin.json`: line
    continuous force x moment arm), and checks the CoP in its feet and the friction.
  - It runs in about 5 minutes per scale and needs no scipy.
- `--pose t` prints one sample's pose.
- Lesson: the arm's wrist and elbow drives (0.66 and 3.8 N m) cannot carry a lift; contact near the elbow can.

## The regression gate (docs/49, D-078)

`python tools/regress.py` answers "did this change move anything?" in about 2 minutes on 12 processes. Run it
before every commit that touches `sim_tendon.py`, an exporter, `cad/skeleton.json` or a drive constant.

- It checks that `sim/robot.xml` and `sim/robot_standin.xml` are what the exporters write today, then runs each
  gauntlet scene in its own process and compares outcome, flags and energy (within 2%) with `sim/baseline.json`.
- `--more` adds the get-up with the scripted helper and with a robot helper, as counts of 24 jittered runs
  (within 2 runs of the baseline; about 15 minutes).
- `--only walk,sit45` runs some scenes. `--update` rewrites the baseline: only after a change that is meant to move
  it, and say what moved in the commit and the doc.
- A scene must not depend on what ran before it. `Rig.reset()` restores every switch a scene may flip
  (`lock_enabled`, the balance loop's memories, the lock friction). When a scene adds a new switch on the rig,
  reset it there too, and check with a fresh rig against a used one (docs/49 has the method).
- `getup.py --jitter-test sim/getup_params.json:firm` and `duo.py --jitter-test sim/duo_params.json:standup` read
  the committed parameter files directly (with `GETUP_GRIP=elbow`).

## Ears, a coach's voice and a helper's hands in training (docs/49, D-077)

- Both models have two ear sites on the head (`export_mjcf.EARS`, 0.15 m apart); the stand-in also has `ear.L_pos`
  and `ear.R_pos` sensors.
- The sibling's `rl/coach.py` renders a coach's words to those ears as band energies (`--coach voice`) and runs a
  fading helper as a lift on the trunk (`--help-cap 0.6`). Both are for the stand-in's `getup` task:
  `python -m ai_embedded_dynamic_diversity.rl.cli train --task getup --getup-start curriculum --arch mlp
  --coach voice --coach-wait 0.5,12 --help-cap 0.6`.
- `rl.getup_eval <policy> [--guide 0] [--wait 0.5,12] [--mute]` is the unassisted test: never with the hands, with
  the voice as trained unless overridden.
- What worked (docs/49): `--help-cap 0.6 --help-free 0.5 --help-fade-at 0.35`. What did not: a late fade (the policy
  leans on the helper), spoken cues (unused), and waiting for the spoken command (0%). The sibling's
  `skills/grounded-rl/SKILL.md` has the rules.
- If `EARS` moves, re-export the stand-in; policies trained with the voice depend on the ear spacing.

## Two robots in one model (docs/48, D-075, D-076)

- `duo.Duo()` builds the pair: robot A as before (no prefix, so `getup.py` and `helper.py` run on it unchanged) and
  robot B under the prefix `b.`, facing A, on the same `MjData`.
  - `sim_tendon.Rig(model, data, prefix, base, winches)` is one robot of several. Only the first rig resets and steps
    the data; its `step()` calls each partner's `pre_step()` (its `script`, then `control()`) and `readout()`.
  - Code in `Rig` that reads a world position must go through `Rig.loc()` (the rig's heading frame), and code that
    sums contacts must skip the other robot's (`Rig.shared`, `Rig.root`).
  - After any `Rig` change, check one robot is unchanged: `sim_tendon.py --only stand,push30,squat` and
    `getup.py --settle` must print the same as before.
- `python tools/duo.py --one --params sim/duo_params.json:<key> [--trace] [--video duo --gif]` runs one get-up with B
  as the helper; `take=N` sets the first phase (`duo.ORDER`) in which B may take A's arms. `--probe` prints where A's
  parts go.
- `python tools/duo.py --train --levels [--from 6] --init <best.json>:levels.6 --out sim/out/duo_x` searches A's
  script (`getup.GPARAMS`) with B's place and reach (`duo.DPARAMS`), about 70 s a generation at 13 processes.
  `--jitter-test <best.json>:levels.6 --n 24` checks a set.
- Lessons:
  - a W-max clutch slips at 1.03 of the line's continuous rating, so a posture planned near 1.0 folds: plan under 0.7;
  - a helper robot stands close, feet apart, and carries beside its body; it does not reach or bow under load;
  - the helped robot must stand up where it is (`getup.STRIDE`), not travel into the helper;
  - a cradle never pulls down (`Lifter.no_pull_down`);
  - a search that may leave the hard part to the scripted helper will: give no credit unless the robot held, and
    move the takeover earlier level by level;
  - do not edit `duo.py`, `getup.py`, `helper.py` or `sim_tendon.py` while a search runs (its workers re-import them);
  - a long patch script full of quotes fails as a Bash heredoc here: write it with the Write tool and run the file.
