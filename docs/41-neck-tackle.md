---
title: "41 - The neck's pitch on two strands: the chair no longer slips it (P-064, D-066)"
summary: "The chair's trunk swing overloaded the neck's W-mini. Its pitch now runs on two strands (D-066). The spine's overload traced to the rise's fast hip extension; pacing it at 22.5 deg/s (D-067) stands every chair case with no slip and less energy"
section: "Skills: wind, bags, chairs, chains"
---

# 41 - The neck's pitch on two strands: the chair no longer slips it (P-064, D-066)

docs/33 found that the chair scene overloads the neck. While the trunk swings forward to seat-off, the neck holds
the head level. Its W-mini line peaked at 62 to 88 N against its 37 N rating, and its clutch slipped up to
534 degrees of the motor. docs/36 and docs/37 then saw the brushless layout lose the 0.50 m chair to the same slip.
Today's gauntlet still flags it at 0.45 m (75 N, a safety factor of 3.0, 23 degrees of slip).

Short answers:
- **The brushless layout now stands from 0.50 m.** It still loaded the neck (76 N, 27 degrees of slip). docs/37's
  631-degree slip came before P-063 took the chair's own mass out of the reach.
- **Letting the head follow the trunk does not help.** At half (`SIT_HEAD_LEVEL=0.5`) the neck slips more on the
  20 kg model (106 degrees), and the 0.50 m rise fails.
- **Two strands at the neck's pitch do.** With `tackle_by_axis={"pitch": 2}`, as the elbow's yaw has its own
  count, the slip is 0 everywhere and the line peak falls to 36 to 52 N. It is now the model (D-066).

## The grid

One chair scene each; the neck line's peak (rated 37 N), the clutch slip and the scene's energy.

| Build, seat | As before | Head level 0.75 | Head level 0.5 | **Pitch on 2 strands** |
| --- | --- | --- | --- | --- |
| 20 kg, 0.45 m | 75 N, 23 deg, 1010 J | 79 N, 64 deg | 82 N, 106 deg | **52 N, 0 deg, 987 J** |
| 20 kg, 0.50 m | 71 N, 0 deg, 1272 J | 78 N, 22 deg | 85 N, 554 deg, **does not stand** | **47 N, 0 deg, 1176 J** |
| brushless layout, 0.45 m | 63 N, 0 deg, 1407 J | 64 N, 0 deg | 69 N, 0 deg | **36 N, 0 deg, 1206 J** |
| brushless layout, 0.50 m | 76 N, 27 deg, 1672 J | 74 N, 10 deg | 56 N, 0 deg | **47 N, 0 deg, 1793 J** |

With the tackle in the model (`sim/robot.xml` regenerated):
- **The gauntlet:** all 14 scenes stand. The 0.45 m chair goes from 1025 J to 954 J, and its neck flag is gone.
  Every other scene is within a few joules.
- **The 20 kg grids:** held walk 14 of 14, soft walk 14, held step 14.
- **The chain grid:** 17 of 24, against 18. The same six heavy one-handed walks fail, and the ball rolled off its
  top once more despite the wait for stillness.
- **The brushless layout at 0.50 m:** it stands with and without armrests (1793 J and 1587 J). The flag is now the
  spine pitch line (above its continuous rating, a safety factor of 2.8), not the neck.

## What it costs

- **Speed:** the neck's pitch runs at half speed, for twice the torque. A head nod does not need the speed.
- **Hardware:** one running sheave on the head side of the pitch line, as the waist's two strands have. The knuckle
  generator (`chain.py --preset neck`) does not draw tackles for any joint, so nothing there changes.
- **Line stiffness:** two strands halve the line's stretch per newton, so the pitch is stiffer at the head.

## The spine pitch: a third strand at the waist does not help

The waist's pitch on three strands (`TACKLE_OVERRIDE={"waist.pitch": 3}`, two today), chair scenes. The spine line's
peak against its continuous rating (288 N on the 775 drive, 184 N on the brushless one):

| Build, seat | 2 strands (today) | 3 strands |
| --- | --- | --- |
| 20 kg, 0.45 m | 320 N, stands | 304 N, stands |
| 20 kg, 0.50 m | 339 N, stands | 296 N, **does not stand** |
| 775 layout, 0.45 m | 338 N (87 deg slip), stands | 310 N, **does not stand** (neck 100 N, 254 deg) |
| 775 layout, 0.50 m | 354 N, stands | 309 N, **does not stand** |
| brushless layout, 0.45 m | 324 N, stands | 316 N, stands |
| brushless layout, 0.50 m | 323 N, stands | 313 N, stands |

The third strand takes off only 5 to 13% and slows the trunk's swing, so half the rises fail. The spine line runs over
its continuous rating on every build, for the second or two of the swing. That is a question for the line and its
rating (docs/12's table), not the tackle.

### Why: the spine is at its clutch

Neither a slower swing (`SIT_LEAN_T` doubled) nor less spine overshoot (`SIT_SPINE_OVER` 15 to 5 degrees) moves the peak.
It stays at 315 to 339 N on all three builds. The clutch slips when the line reaches a third of its break
(`CLUTCH_LINE_FRACTION`): 297 N on the 890 N braid. So the spine drive is pushing against its clutch:
- The spine is position-servoed toward a seat-off pose that it cannot reach under the trunk's load, so it
  saturates.
- More strands or a slower swing only change how it gets there.
- The 775 layout's spine slips (87 to 199 degrees) because its heavier trunk holds it there longer.

The gauntlet's line flag (a safety factor under 4) sits below the clutch's own setting (3). So any drive that works
at its clutch is flagged. The flag is right that the spine runs at its limit. It does not mean the line is at risk:
the clutch caps the line at 297 N, with brief dynamic peaks of up to 1.2 times that.

### Where: the spine cannot bend the trunk forward from a reclined pelvis

A smaller seat-off bend (`SIT_SPINE` 10 or 15 degrees, 20 today) does not lower the peak either (319 to 363 N), and at
10 degrees most rises fail. Tracing the 20 kg model's 0.45 m chair:
- **Going down,** line A holds the trunk from falling forward, up to 294 N, with the spine 1 to 2 degrees past its
  target.
- **Onto the seat,** the pelvis tips back and the spine's flexion line B pulls at 278 to 295 N. The spine stays at
  8 degrees against a 20 degree target.
- **In the rise,** the spine is commanded to 34 degrees: the path plus `SIT_SPINE_OVER`, docs/33's overdrive for
  its sag. B sits at 297 N, the clutch, with the spine near 13 degrees.

So the spine's drive cannot bend the trunk forward from a reclined pelvis. docs/33 overdrove it on purpose, and the
clutch caps the result. The hips do the rest of the lean, and every build stands. A stronger spine drive, or a W-max
in its place, would be the hardware answer. The planner could also lean the pelvis forward before it asks the spine
for anything.

Leaning with the hips first (60% of the lean's time before the spine moves, with and without the overdrive) leaves
the peak at 310 to 354 N. The down and onto-the-seat phases reach the clutch before the lean starts, so planning
cannot remove it. The answer is the spine's drive (tried and reverted).

### What the spine needs

The trunk's gravity torque at the spine pitch joint, from the seated and rising poses (`qfrc_bias`, at rest), against
the clutch's 29.7 N m at the joint (297 N x 50 mm x 2 strands):

| Build | Seated, as reached (7-8 deg) | Seated, at the 20 deg target | Rising, as reached | Rising, at 20 deg |
| --- | --- | --- | --- | --- |
| 20 kg | 29.6 N m | 32.8 | 30.1 | 31.3 |
| 775 layout | 29.5 | 34.4 | 30.3 | 33.2 |
| brushless layout | 27.0 | 30.7 | 27.4 | 29.1 |

- **The spine stops where gravity meets its clutch.** That is the 8 degrees seated, against a 20 degree target.
- **`tendon_calc.py` now has the case** ("SIT spine pitch", 35 N m, 700 N at the 50 mm pulley):
  - the 775 W-max at 2 strands gives 28.8 N m continuous: not enough;
  - at 3 strands, 43.2 N m: enough, with the line at a safety factor of 3.8.
- **In the simulation, 3 strands without the overdrive still run the line at 284 to 310 N** (43 to 46 N m at the
  joint). The 775 layout's 0.50 m rise fails. So the spine carries more than the trunk's static weight in the sit,
  by a third to a half. Where that comes from is not traced. Candidates:
  - the down phase's dynamics;
  - the servo's integral trim;
  - lines that cross the spine.
- **One cause found: the trunk sits on the thighs.** Seated and in the rise, each thigh's capsule presses on the
  spine body's box with 410 to 430 N. The box is 0.26 x 0.16 x 0.40 m from the spine's base, just above the hips.
  That puts a constraint torque of -58 N m at the spine pitch joint, against the lines' +29 N m and gravity's -30.
  The trunk cannot fold forward because its lower front edge meets the thighs.
- **Removing that contact does not lower the peak** (311 to 345 N, the spine's collision off, as an upper bound).
  The spine then folds further and meets its clutch there. So the contact explains the 8 degrees reached, not the
  line's peak.
- **Relieving the edge makes it worse** (`export_mjcf.SPINE_RELIEF`, the collision box's lower front raised):

  | Build, seat | As is | 40 mm relief | 80 mm relief |
  | --- | --- | --- | --- |
  | 20 kg, 0.45 m | 320 N, stands | 365 N, stands | 312 N, stands |
  | 20 kg, 0.50 m | 339 N, stands | 371 N (75 deg slip), stands | 313 N, stands |
  | 775 layout, 0.45 m | 338 N, stands | 380 N, stands | 317 N, **does not stand** |
  | 775 layout, 0.50 m | 354 N, stands | 394 N (165 deg slip), stands | 316 N, stands |
  | brushless layout, 0.45 m | 324 N, stands | 355 N, stands | 331 N, stands |
  | brushless layout, 0.50 m | 323 N, stands | 393 N, **does not stand** | 324 N, stands |

  The thigh contact carries part of the trunk, as a person rests their trunk on their thighs to stand. So the
  trunk's lower front and the top of the thighs are load-bearing seated. **For the hardware:** design them as pads,
  not as a gap to clear. The deeper thigh of D-060 (165 to 175 mm, docs/37) brings that contact sooner.

### When the trunk leaves the thighs, the spine folds to its stop

The rise, traced with the trunk-thigh contact alongside the spine line (0.45 m chair):

| | 20 kg model | 775 layout |
| --- | --- | --- |
| The trunk rests on the thighs | to 12.4 s, 400 to 1000 N | to 13.2 s, 600 to 870 N |
| The hips extend after it leaves | -96 to -74 deg in 0.6 s | -94 to -71 deg in 0.6 s |
| The spine | folds to 39.6 deg (its stop is 40), target 15 to 25 | folds to 41 deg and stays past 15.4 s |
| The spine's extensor line | 230 to 295 N for 1.6 s | 230 to 307 N for over 2 s |

The hips rotate the pelvis back under a trunk that lags, and the spine takes up the difference until it reaches its
stop. The line peak in the rise is the extensor pulling the trunk back from there.

A gate on the rise, like the CoM's (`SIT_SPINE_GATE`: the knees stop while the spine is folded past its path):

| Build, seat | Off | 5 deg | 10 deg |
| --- | --- | --- | --- |
| 20 kg, 0.45 m | 320 N, 0 slip | 307 N, 0 slip | 374 N, 44 deg |
| 20 kg, 0.50 m | 339 N, 0 slip | 302 N, 0 slip | 370 N, 19 deg |
| 775 layout, 0.45 m | 338 N, 87 deg | 330 N, **0 slip** | 383 N, 45 deg |
| 775 layout, 0.50 m | 354 N, 0 slip | 315 N, 0 slip | 372 N, 52 deg |
| brushless layout, 0.45 m | 324 N | 317 N | 388 N |
| brushless layout, 0.50 m | 323 N, stands | 316 N, **does not stand** | 387 N, does not stand |

At 5 degrees the spine stops slipping on every build, and the peak drops by 2 to 11%. But the brushless layout loses
the 0.50 m rise, and the rises take more energy (the 20 kg 0.45 m chair: 1010 to 1264 J). It stays off by default,
as an option.

## The fix: the hips extend no faster than the trunk can follow (D-067)

`SIT_HIP_RATE` caps how fast the rise's hip targets extend. Six chair cases (3 builds x 0.45 and 0.50 m):

| Rate (deg/s) | Rises stood | Spine slip |
| --- | --- | --- |
| none (before) | 6 of 6 | 87 deg (775 layout, 0.45 m) |
| 15 | 1 of 6 | - |
| 20 | 5 of 6 | 0 |
| **22.5** | **6 of 6** | **0** |
| 25 | 6 of 6 | 0 |
| 27.5 | 3 of 6 | up to 51 deg |
| 30 | 3 of 6 | up to 35 deg |

At 22.5 deg/s the rises take less energy:
- 20 kg: 836 J at 0.45 m (was 1010) and 929 J at 0.50 m (was 1176);
- 775 layout: 1024 J and 933 J (was 1309 and 1782);
- brushless layout: 977 J and 1618 J (was 1206 and 1793).

The spine line still peaks at 304 to 328 N: it works at its clutch while the trunk swings up, but nothing slips.
It is now the default:
- **The gauntlet:** all 14 stand; the 0.45 m chair goes from 954 J to 842 J.
- **The other chairs:** 0.40 and 0.50 m, with and without armrests, all stand (836 to 992 J). The 0.35 m chair
  still falls, as in docs/33.
- **The chain grid:** 17 of 24, unchanged.
- At 0.40 m the limit changes nothing: the layouts fail there either way.

## The 0.40 m chair on the layouts

Both layouts sit and fail to rise at 0.40 m, with or without the hip pace. It is the static reach: the CoM's distance
behind the sole centre at seat-off. The 20 kg model stands from 9.1 cm; the brushless layout has 9.6 and the 775
layout 11.5. That is docs/36's mass-layout limit (the pelvis and the arms), not the control's.

## Regression with all of P-064's defaults

The press (D-064), the neck's two strands (D-066) and the hip pace (D-067) together. 20 kg model, 7 lengths x 2
models, out of 14:

| Grid | Before P-064 (docs/37) | Now |
| --- | --- | --- |
| held step | 14 | 14 |
| soft step | 14 | 14 |
| `ROBOT_MOTORS=low` step | 7 | **13** |
| held walk | 14 | 14 |
| soft walk | 14 | 14 |
| `ROBOT_MOTORS=low` walk | 3 | **12** |

Nothing is lost, and the pessimistic motors gain the most. The gauntlet stands 14 of 14. The chain grid is 17 of 24;
its failures are the one-handed 3 and 4.5 kg walks.

## Not yet

- The neck's roll and yaw stay on one strand. Neither was flagged.
- The sheave as a printed part, for the neck and the waist alike.
- The 0.40 m and 0.35 m chairs with the tackle (docs/33's other limits there are the knee and the balance).
- The spine pitch line on the brushless layout's chair (a safety factor of 2.8).
- The ball's put-down: a flat top does not hold a sphere reliably. A dished top or a lip would.

## Next

docs/37's item on the brushless 0.50 m chair is closed. The spine is the next limit in the chair. The rise is fixed by
pacing the hips (D-067). Next is the deeper D-060
thigh in the collision model, which changes when the trunk leaves the thighs.
