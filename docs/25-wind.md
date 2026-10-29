# 25 - Wind: drag on every body, gusts from any side, and a robot that leans into it (P-038, D-042)

Wind was one of the intended obstacles of the sibling project (its world has wind as a smoothed random control),
and the owner asked whether the robot walks against and away from winds of changing strength (hub task
`wind-gauntlet`). Until now the model had no wind. Now it does. Standing, the robot carries 12 m/s head-on or
from behind and 10 m/s from the side, with 1.5 m/s gusts to 8 to 10 m/s. Walking, it carries 3 m/s from any
side with gusts, 5 m/s steady from behind or the right. The walk is the weak point.

![Standing in an 8 m/s head wind with 1.5 m/s gusts (the cyan arrow is the wind)](img/gust.gif)

## The wind (`tools/sim_tendon.py`)

- **Drag:** MuJoCo's own fluid model, air at 1.2 kg/m^3 (`opt.density`), the wind set each step (`opt.wind`).
  Each body is drag on its inertia box, which for this skeleton is its printed shell (`export_mjcf.SHELL`), with
  Cd 1 on the box faces. Measured on the 20 kg model:

| From | Drag at 10 m/s | CdA | Centre of drag above the floor |
| --- | --- | --- | --- |
| head or tail | 25.6 N | 0.43 m^2 | 0.95 m |
| the side | 22.0 N | 0.37 m^2 | 0.88 m |

  So the robot feels a 10 m/s wind as 13 % of its weight. A person (70 kg, CdA ~0.7) feels it as 6 %: the robot
  is light for its size, and its winds are a person's winds about 1.4 times faster.
- **Speed:** a mean ramped in over 2 s, plus Ornstein-Uhlenbeck gusts on the along-wind speed (sigma = `gust`)
  and the cross-wind speed (0.75 `gust`), time constant 2 s. With 1.5 m/s gusts at 8 m/s, the peaks reach
  11.5 m/s.
- **Direction:** `dir` is where the wind comes from: `head`, `tail`, `left`, `right`, or degrees (0 head, 90 left).
- **Use:** `WIND = dict(mean, gust, tau, dir, seed)` for every scene, `rig.set_wind(...)` inside a scene, or
  `--wind 8,1.5,head` on `sim_tendon.py` and `sim_video.py`. The report prints the mean and peak drag.
- **New gauntlet scene `gust`:** 15 s in an 8 m/s head wind with 1.5 m/s gusts.

## What had to change

At first the standing robot fell at 10 m/s head-on and from behind (8 m/s stood). There were two causes:

1. **The locks.** A wind that builds slowly keeps the body still, so the lock gate of D-038 locked every drum.
   The locked ankles then carried a growing load until the body tipped, and released only once it was falling.
   The gate now also resets when the disturbance estimate has moved 20 mm since the body went still
   (`LOCK_RELEASE_D`). That alone changed nothing in the map, because of the second cause.
2. **No lean.** The balance loop kept the CoM over the sole centre and carried the force with the CoP alone. The
   CoP can move 100 mm toward the heel, so the robot could carry about 21 N, which is 9 m/s. A person leans into
   the wind instead.
   - **The lean (`LEAN`).** The slow part of the disturbance observer's estimate moves the CoM target upwind:
     the lean is that slow part (low-passed over `LEAN_TAU` 0.4 s, lateral `LEAN_TAU_X` 1.5 s), with a 20 mm dead
     band and an 80 mm limit.
   - **Why the integral could not do it.** It has to move the ankles' planned angle. The ankle is a spring about
     its plan (the lines are the spring), and the capture-point integral can hold only about 25 N m of it off the
     plan. Leaning 80 mm takes about 100 N m.
   - **Sideways**, the lean moves the hip-roll shift's target, fed forward like the shift's geometry.
   - **With the lean**, the CoP comes back toward the sole centre and gravity carries the force.

What did not work, or cost too much:

| Variant | Result |
| --- | --- |
| the lean through the capture-point integral only, any gain | no change: the fall at 10 m/s at the same time |
| no dead band | the step and the pushes cost 10 to 20 % more energy (the robot's own motion is read as wind) |
| the lateral lean at 0.4 s | 192 of 200 random pushes (a short side push moved the hips); at 1.5 s, 199 |
| the ankle lean in a staggered stance | tilting both ankles by one angle fights the legs when the feet are a step apart and lifted the landed foot; it now fades out over 40 mm of stagger (`LEAN_STAGGER`) |

Physically, holding the CoP on the sole margin and the CoM at the toe gives about 55 N head-on, 14.7 m/s. The
lean is limited to 80 mm, which gives about 38 N, 12 m/s. That matches the map below.

## The failure map (`tools/wind_sweep.py`, 20 kg model)

Each cell shows the seeds that stood, then where the first run fell (phase@time). Standing is 20 s, two gust
seeds. The other scenes use one seed.

**Standing, 20 s**

| m/s | head, gust 0 / 1.5 / 3 | tail | left | right |
| --- | --- | --- | --- | --- |
| 4 | 2 2 2 | 2 2 2 | 2 2 2 | 2 2 2 |
| 6 | 2 2 2 | 2 2 2 | 2 2 2 | 2 2 2 |
| 8 | 2 2 1 | 2 2 1 | 2 2 2 | 2 2 2 |
| 10 | 2 1 0 | 2 1 0 | 2 2 0 | 2 2 0 |
| 12 | 2 0 0 | 2 0 0 | 0 0 0 | 0 0 0 |
| 14 | 0 0 0 | 0 0 0 | 0 0 0 | 0 0 0 |

Before this doc, 10 m/s steady fell from every side. Gusts of 3 m/s (peaks over 2.5 times the mean's drag) are
the limit at 8 to 10 m/s: the lean cannot follow a gust that comes and goes in 2 s, and the CoP alone carries
only 21 N.

**One foot, step and walk** (S stood, F fell; gust 0 / 1.5 m/s)

| Scene | m/s | head | tail | left | right |
| --- | --- | --- | --- | --- | --- |
| one-foot hold | 3, 5 | S S | S S | S S | S S |
| | 7 | S S | S S | S F | S F |
| | 9 | S S | S S | S F | S F |
| step | 3 | S S | S S | S S | S S |
| | 5 | S F | S S | F F | S S |
| | 7 | F F | F F | S F | S F |
| | 9 | F F | F F | S F | F F |
| walk (4 steps) | 3 | S S | S S | S S | S S |
| | 5 | F F | S F | F F | S F |
| | 7, 9 | F F | F F | F F | F F |

- **The one-foot hold** carries 9 m/s from the front or back, gusts included. From the side, 9 m/s steady holds
  but gusts do not.
- **The step and the walk** fail mostly in the transfer onto the leading foot (phase 4), and at 9 m/s already in
  the lift (phase 2). The map is not monotonic from the side (5 m/s from the left falls where 7 stands): the step
  was tuned on a knife edge (`19`) and wind moves it. The planner's stages wait for loads and CoM positions
  planned for still air, and the lean moves the CoM away from them.

## Regressions (no wind)

| Check | Before (D-038 / D-039) | Now |
| --- | --- | --- |
| gauntlet (now 13 scenes with `gust`) | all stand | all stand |
| step grid, 42 variants | 42 | 42, landings within 4 mm |
| walk grid, 42 variants | 42 | 42, within 6 mm; walk 1113 J (unchanged) |
| 200 random pushes | 199 | 199 |
| sustained 15 N push | 81 J | 54 J (it leans instead of pushing the CoP to the heel) |
| pushes 10 / 20 / 30 / 40 N | 19 / 24 / 26 / 42 J | 22 / 28 / 35 / 44 J |
| arm forward with 1 kg | 162 J | 238 J (the hand load is an outside force: it leans back, and the sway of `22` grows) |
| squat, shift, one-foot hold, step | 33, 43, 562, 471 J | 27, 43, 562, 471 J |

## Learned control

- **Teacher data:** `record_teacher.py --winds 60` adds 60 episodes of 12 s standing in a random wind (mean 0 to
  9 m/s, gusts 0 to 2 m/s, any direction). All 60 stand. The re-record has 272 episodes and 107 243 steps
  (gauntlet scenes with `gust`, 199 of 200 pushes, 60 winds). The observation has no wind channel: the robot has
  no anemometer, so a policy must read the wind from its own motion, as the disturbance observer does.
- **Sibling project (PR #54, issue #53):** the stand-in env has the same wind as domain randomisation
  (`EnvConfig.wind_mean`, `wind_gust`, `wind_tau`; `--wind-mean lo,hi --wind-gust lo,hi`). It is off by default
  and bit-identical when off. Each variant has its own mean, direction and gusts, and the privileged observation
  gets the wind. `tendon_eval` has `wind5` and `wind8` scenes.

## Not yet

- **Walking in wind** above 3 m/s: the planner's gates and stages are geometry for still air. The transfer onto
  the leading foot is where it fails.
- **Gusts beyond the sole:** the answer is a stepping strategy (a capture step downwind), which a static walker
  does not have.
- **The arm scene's sway** is worse with the lean (`22`'s limit cycle).
- **Simplifications:**
  - Drag is per body box, Cd 1. No shielding of one body by another, no lift, no ground boundary layer
    (wind speed is the same at the feet as at the head).
  - The wind's direction is fixed in a run. The sibling's stand-in fixes it per variant.
- **Not run in wind:** the rolled-up masses and the pessimistic motors (`24`). The 26 kg build has more weight
  for the same area, so it should carry more wind; the low motors less.

## Next

Walking in wind: gate the transfer stages on the leaned CoM, not the still-air plan, and judge it on the walk
map at 5 and 7 m/s. Then a capture step for gusts, and wind for the learned controller in the sibling (clone
from the new teacher data, train with `--wind-mean 0,8`).
