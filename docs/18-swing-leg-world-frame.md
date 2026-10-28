# 18 - The swing leg in the world frame, and what the step really lacks (P-026)

`16` ended with the step completing but its feet landing 18 to 29 mm long, and five attempts at correcting
the swing foot from its measured position all made it worse. The fix proposed there was a swing-leg
controller in the world frame (hub task `swing-leg-world-ik`). This doc records building it, on branch
`feat/w-mini-25mm` together with the D-032 neck, and what it showed: the landing can be made accurate, but the
step has no margin anywhere else, so accuracy alone does not make it stand.

## The controller (`tools/sim_tendon.py`, `SWING_IK`)

Each control step, inverse kinematics on the model's own chain: a scratch copy of the model takes the measured
pose (every joint and the floating pelvis, as the joint sensors and the IMU give it), and Gauss-Newton on
finite differences (three iterations, each evaluation one `mj_kinematics`) solves the swing leg's joints so its
ankle joint sits at a point in the world and its sole at its standing attitude. The foot's path is fixed on
the floor: lift straight up to 40 mm, carry level to the mark, lower 20 mm past the floor until the foot
sensor reads 15 % of the weight. Two variants:

| Variant | Joints solved | Lateral |
| --- | --- | --- |
| `SWING_IK` | hip pitch, knee, ankle pitch | the swing hip roll follows the weight shift, low-passed (`SWING_ROLL_TAU`) |
| `SWING_IK` + `IK_LATERAL` | also hip roll and ankle roll | the ankle held at its lift-off width, the sole level in roll |

`control()` adds the trunk lean to every hip pitch target and the weight shift to every hip roll, so the IK
subtracts them to command the joint angle it solved.

## What it took, and what each step showed

1. **The IK output must not be low-passed.** Read back unfiltered, the pelvis pose carries the swing hip's own
   reaction and the leg rang at about 5 Hz, 40 mm at the foot. Filtering the output instead lagged the
   descent: the foot touched down with no press and stayed unloaded. Filtering the measured pose the IK reads
   (0.3 s) fixes both.
2. **Single support rings at about 3 Hz.** On one foot the pelvis pitches 1 to 1.5 degrees and the free foot
   moves 50 mm sideways in a tenth of a second while every leg joint holds its target within half a degree.
   The cause is lateral: in `16` the free leg's hip roll is slaved to the stance hip's weight-shift command
   (so it lands at stance width), and that command carries the stance loop's ripple, which throws the 3.5 kg
   leg sideways, which feeds the ripple. Holding the free ankle at a fixed width in the world (`IK_LATERAL`)
   makes single support steady: the CoM's lateral error stays within 3 mm and the pelvis roll within half a
   degree, where it swung 2 degrees before.
3. **The landing is accurate.** With the IK the first foot lands within 0 to 13 mm of its mark in most
   variants (open loop: 18 to 28 mm long), the second within 2 to 11 mm in the runs that get that far.
4. **An absolute roll target left behind pushes the next transfer.** The lateral IK leaves the landed leg's
   hip roll about 7 degrees off the weight-shift parallelogram; held there, it drove the CoM sideways at
   170 mm/s past the stance foot. Returning it after landing (`ROLL_RESTORE`) drags the lightly loaded foot
   30 to 60 mm sideways instead.
5. **Joint-rate damping in the hip loop** (`KD_JOINT`) halves the free foot's ripple in single support but
   destabilises the weight shift on the v0.1.0 model; it stays off.

## The step has no margin (`tools/step_sweep.py`)

One run of the step is deterministic, and its outcome flips with a millimetre. So every change is now judged
on a grid: two models (the D-032 baseline and `--stiff-mini`) by three step lengths (90, 100, 110 mm).

| Configuration | Stands | Where the others fall |
| --- | --- | --- |
| open loop (`16`, unchanged) | 1 of 6 | transfer onto the leading foot (4), second lift (1) |
| `SWING_IK` | 0 of 6 | transfer (4), second lift (1), second placement (1) |
| `SWING_IK` + `IK_LATERAL` | 0 of 6 | transfer (1), second lift (1), second placement (1), centring (3) |

On the v0.1.0 model, where `16` reported the step standing, the open-loop step stands in 2 of these 6
variants. Most runs, whatever the swing leg does, fall in phase 4: the weight transfer onto the leading foot
and the wait for the trailing foot to unload. Traced, that transfer does not complete: over its 5 s the
lateral CoM stays about 90 mm short of the leading foot, because the double-support weight shift is slow by
design (`ROLL_KP` 0.2 rad/m, `ROLL_SLEW` 0.15 rad/s; `14`), the load swaps between the feet at about 1 Hz,
and the trailing foot is released into single support with the CoM still moving sideways.

## The transfer, traced

Every phase-4 fall was traced (`tools/step_trace.py`), and each trace showed a different way the transfer
loses the body. Each is now a flag in `tools/sim_tendon.py`, off by default:

| Finding | Flag | Effect on the grid |
| --- | --- | --- |
| The trailing foot is declared the swing foot at the first dip of its load below a tenth of the weight, while the feet are still trading load at about 1 Hz and the CoM is 70 to 90 mm short sideways | `RELEASE_SETTLED`: release only once the lateral CoM has settled as the lift gate requires | with the lateral IK: 0 to 2 of 6 standing |
| The planner measures hip height above the tall foot's ankle; as that foot unloads its ankle rises, the leading leg is bent further and the body tips forward (positive feedback) | `FIXED_FLOOR`: the ankles' standing height as the floor | no change on its own |
| A straight trailing leg sets the hip height and cannot unload; a person straightens the front leg and bends the back knee | `TRANSFER_SWAP`: through the hold the leading leg becomes the straight one and the trailing leg is planned 15 mm short | falls move from phase 4 to phase 7; both feet within 10 mm in 4 of 6 |
| The trailing foot slides inward through the transfer (97 mm to 43 mm from the centre line), and the lateral IK lands it at that narrowed width | `WIDTH_FROM_STANCE`: land at the standing spacing from the stance foot | worse on the grid (0 of 6); open |
| A longer transfer or hold (8 s, 3 s) | `TRANSFER_T`, `TRANSFER_HOLD` | worse: more time in the oscillating stance |
| The leading knee's torque mode | `TRANSFER_KNEE` | no clear effect |

Holding a staggered stance with an even weight share for 8 s stands in every swing-leg variant (load ripple
about 30 N on each foot), so the stance itself is stable; what fails is the end of the transfer, taking the
last of the weight off the trailing foot.

One more attempt was removed again: the transfer's grounded legs by the same world-frame IK from a planned
pelvis path. It collapsed at once, because the landed foot's ankle sat 26 mm above the floor height the IK
assumed and both legs were bent to pull it down. Planning the stance legs in the world frame needs the
landed foot's actual contact (heel, toe or flat) first.

| Configuration (flags) | Stands of 6 | Both feet within 10 mm |
| --- | --- | --- |
| open loop (`16`, default) | 1 | 0 |
| `SWING_IK IK_LATERAL` | 0 | 3 |
| `SWING_IK IK_LATERAL RELEASE_SETTLED` | 2 | 0 |
| `SWING_IK IK_LATERAL TRANSFER_SWAP` (hold 2 s) | 1 | 4 |

## Where this leaves the hub task (superseded by `19`)

`19` found the reason the grid would not settle (the drum locks rock the body foot to foot inside the balance
loop) and rebuilt the transfer; the step now stands in 14 of 14 variants with both feet within 4 mm. What
follows is the state at the end of this doc.

The swing leg does what the task asked: the IK lands the feet within 10 mm in most variants. The exit test
(the step completing on both models) is not met, and it cannot be met by the swing leg: on this grid the
step's falls are in the weight transfers and the centring, whatever the swing leg does, and the differences
between configurations are within what one millimetre of a step length changes. The step as built is a
sequence of hand-tuned stages each with no margin. The next step is a transfer designed for margin rather
than another flag: the CoM moved on a planned lateral path with the hip shift fed forward, the trailing knee
bent from the start of the unload, the landed foot's contact state read from its sole before the stance is
planned on it, and every stage gated on settled quantities. Alternatively, a learned controller cloned from
the IK-placed steps that do land (`17`), judged on this grid.

## Tools

- `python tools/step_sweep.py "name: K=V ..." ...` runs the grid above for any set of overrides in parallel.
- `python tools/step_trace.py --from 13.5 --to 20 [K=V ...]` prints the trace every fall here was found with.
