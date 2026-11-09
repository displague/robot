# 40 - The layout on held feet: the landing press was the step's failure (P-064, D-064)

docs/37 found that the D-060 layout (the hip drives on the thighs, the battery in the torso) loses held feet.
`NOSLIP=5` is the contact D-043 chose. On it the 775 layout stood 0 of 14 steps and 0 of 14 walks, falling in the
forward weight transfer (phase 4). docs/37's Next was to trace phase 4, the 775 layout at 0.10 m against the 20 kg
model.

Short answers:
- **The fall is sideways, not forward.**
  - The right foot lands where planned (105 mm, planned 102).
  - The transfer then plans the landed leg at its unpressed length. The landing had aimed it 20 mm below the floor,
    so the leg pulls up. Its press rebuilds at 0.02 m/s only while the foot is unloaded.
  - The heavier leg lands harder: 147 N at the first sample against the 20 kg model's 45 N. Its foot is off the floor
    0.3 s later.
  - The CoM runs outward over the stance foot at 0.09 m/s, with the CoP pinned at the sole's outer edge. It passes
    the edge in 0.8 s.
- **The fix already existed.** `TP_PRESS_FROM_LANDED` (P-042, docs/25) starts the press from the landed leg's
  measured length. It was off by default because it cost the soft-contact walk 3 of 42 variants. With it:

  | Grid, 7 lengths x 2 models | Before (docs/37) | `TP_PRESS_FROM_LANDED` |
  | --- | --- | --- |
  | 775 layout, held step | 0 | **14** |
  | 775 layout, held walk | 0 | 4 |
  | 775 layout, soft step | 14 | 14 |
  | 775 layout, soft walk | 12 | 13 |
  | brushless layout, held step | 6 | **10** |
  | brushless layout, held walk | 2 | 4 |
  | 20 kg model, held walk | 14 | 13 |
  | 20 kg model, soft walk | 14 | 14 |

  The `TP_START_REACH` knob had no effect: the load-settle mode does not read it.
- **The held walk is what's left.** The layout's later transfers (w3, w4) fall outward over the new stance foot:
  - Through the weight ramp the trailing foot slides 31 mm outward, even on held feet.
  - When it is released, the CoM jumps to 0.10 m/s outward and passes the stance sole's edge.

Knobs tried on the 775 layout's held walk, with the press fix on (each out of 14, base 4):

| Variant | Walks |
| --- | --- |
| `SS_SLEW=1.0` (the stance hip moves faster) | 7 |
| `SWING_ROLL_TAU=1.5` (the released leg's roll eased in slower) | 7 |
| both | 7 |
| `SS_SLEW=2.0` | 5 |
| `SWING_ROLL_TAU=3.0` | 7 |
| both, doubled | 3 |
| `WALK_BACK=0.8` | 5 |
| `SS_KP=2.5` | 0 |

None of these is clearly above the grid's scatter of two or three. Hand-tuning gains will not fix it. The trailing
foot's outward slide under a falling load is the mechanism to work on.

## The press as the default (D-064)

`TP_PRESS_FROM_LANDED` is now on. On the 20 kg model:

| Grid (out of 14) | Before | Press fix on |
| --- | --- | --- |
| held step | 14 | 14 |
| soft step | 14 | 14 |
| `ROBOT_MOTORS=low` step | 7 | 9 |
| held walk | 14 | 13 |
| soft walk | 14 | 14 |
| `ROBOT_MOTORS=low` walk | 3 | 7 |

- **The gauntlet:** all 14 scenes stand. Only the step and the walk change:
  - the step, 465 J to 531 J (the transfer's foot pitch line peaks 192 N to 223 N);
  - the walk, 1053 J to 1123 J.
- **The chain grid** (docs/38, docs/39 defaults): 12 of 24 against 13.
  - With no bag in a walk or a sit: 9 of 9, against 8.
  - Walks with a bag in hand: 4 of 10, against 6. The tall bag's two walks are lost.
  - Both differences are within the grid's scatter.
- **docs/25's cost** (the soft walk losing 3 of 42 variants) does not show on this grid. That grid used other
  variants.

## What it means for the layout (D-060, D-061)

- **The step on held feet is no longer the layout's problem:** 14 of 14 on the 775 layout and 10 of 14 on the
  brushless one.
- **The walk on held feet still is:** 4 of 14 on both layouts, against 13 for the 20 kg model.
- So the model stays on the 20 kg masses (D-061) until the held walk passes. D-060's battery-up move stays.

## The partial hip banks, and the walk's lateral loop

docs/37's partial banks (`mass_rollup.py --spread --battery-up --hips-on-thighs N`), held feet, press on:

| Build | Held step | Held walk |
| --- | --- | --- |
| 775, 2 hip drives on the thighs | 14 / 14 (docs/37: 8) | 5 / 14 |
| 775, 4 | 14 / 14 (docs/37: 3) | 4 / 14 |
| 775, 6 (the layout) | 14 / 14 (docs/37: 0) | 4 / 14 |
| brushless, 4 | 9 / 14 (docs/37: 4) | 5 / 14 |
| brushless, 6 (the layout) | 10 / 14 (docs/37: 6) | 4 / 14 |

- **Every bank now steps on held feet, and none walks on them.** The held walk fails even with 2 hip drives
  moved. It is a problem of the heavier rolled-up builds (25 to 26 kg against 20), not of where the hip drives sit.
- **The 0.45 m chair wanted all six on the 775 (docs/37).** Now the held step no longer costs anything for them.

At the layout's failing release, the trailing leg's roll follows its command; the -10 degrees is the commanded weight
shift itself. With 95% of the load on the new stance foot, the CoM is still 47 mm inside that sole's centre and
the shift keeps growing. Windup variants on the 775 layout's held walk (base 4 of 14):

| Variant | Walks |
| --- | --- |
| `ROLL_KI=0.25` (half) | 5 |
| `ROLL_KD=0.5` | 6 |
| `LAT_FF=True` (the geometric feed-forward, docs/19) | 6 |
| `ROLL_MAX` 8 degrees | 0 |

None clears the scatter, and a tighter clamp breaks the walk. So the shift is needed, and the failure is what
happens at the release, when the trailing foot stops resisting.

The release gate is not what fires early. It waits for the trailing load under a tenth and the lateral CoM error
under 10 mm. In the failing transfer the trailing foot unloads by itself while the CoM is still 30 mm short of the
new stance sole's centre. The shift has given the body lateral momentum: the load transfers before the CoM arrives,
the foot lifts, and the CoM carries on past the sole's edge. The heavier build's slower lateral swing makes this
worse.

## A slower, damped weight ramp: the 775 layout walks, the brushless one does not

The early unloading comes from lateral momentum, so the next lever was the weight ramp's length (`TP_B`, 3 s) with
more damping on the lateral shift (`ROLL_KD`, 0.2). Held walk, out of 14:

| Variant | 775 layout | Brushless layout |
| --- | --- | --- |
| base | 4 | 4 |
| `TP_B=5` | 7 | - |
| `TP_B=8` | 2 | - |
| `TP_B=5 ROLL_KD=0.5` | 9 | 2 |
| `TP_B=5 ROLL_KD=0.8` | 3 | - |
| `TP_B=5 ROLL_KD=0.5 SS_SLEW=1.0` | 12 | 0 |
| `TP_B=4 ROLL_KD=0.5` | 8 | - |
| **`TP_B=6 ROLL_KD=0.5`** | **13** | 2 |
| `TP_B=6 ROLL_KD=0.4` | 10 | 0 (falls in the first shift) |
| `TP_B=6 ROLL_KD=0.6` | 7 | - |
| `TP_B=7 ROLL_KD=0.5` | 10 | 5 |

`TP_B=6 ROLL_KD=0.5` on the other grids:
- the 775 layout's held step: 14 of 14.
- 20 kg, out of 14:

  | Grid | Default | `TP_B=6 ROLL_KD=0.5` |
  | --- | --- | --- |
  | held walk | 13 | 14 |
  | soft walk | 14 | 14 |
  | `low` walk | 7 | 5 |
  | held step | 14 | 13 |
  | soft step | 14 | 14 |
  | `low` step | 9 | 11 |

- **The 775 layout gets a held walk** (13 of 14, with its neighbours at 7 to 10), at no clear cost to the 20 kg
  model.
- **The brushless layout gets nothing**, and its lateral loop breaks at `ROLL_KD=0.4`. Its walk fails differently
  and was not traced.
- **So the defaults stay.** `TP_B=6 ROLL_KD=0.5` is the 775 layout's setting, to be used when that build is the
  model. The robot's eventual walk is the learned policy's job (the sibling repo), and these grids are its
  teacher's test.

## Not yet

- The held walk on the layouts: the trailing foot slides outward through the weight ramp, and the CoM leaves at
  its release.
- The brushless layout's 0.50 m neck slip (docs/37).

## Next

For the brushless layout, trace why its lateral loop breaks (`ROLL_KD=0.4` falls in the first shift) before
tuning it further. For both, make the walk's release a staged hand-off. Release the trailing foot only once the CoM is
over the new stance sole's centre, not just once the foot is light. Hold the shift's integral through the release,
and ramp the trailing leg's lift. The 26 kg 775 layout's held walk at 0.095 m is the test case (w3). Tried and not
enough on their own:
- seeding the released leg's roll from its measured angle (3 and 6 of 14, with and without a slower filter);
- the gain sweeps above.
