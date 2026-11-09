# 37 - The layout on the full grid: it loses held feet (P-061, D-061, hub task wmax-mass-down)

docs/36 moved the hip drives onto the thighs and the battery into the torso (D-060). Then the 19.9 kg brushless
build and the 26 kg 775 build stood again from the 0.45 m chair, and they passed the short step and walk grids
(3 lengths, soft contact). This doc runs the full grids: 7 step lengths, held feet and the pessimistic motors.

Short answers:
- **On soft contact and on the pessimistic motors the layout holds.** It does as well as the 20 kg model or better.
  On the pessimistic motors the 775 layout stands 14 of 14 steps, where the 20 kg model stands 7.
- **On held feet the layout fails.** Held feet (`NOSLIP=5`) is the contact D-043 chose as right for the silicone
  pad. There the 775 layout stands 0 of 14 steps and 0 of 14 walks. The old roll-up stands 8 and 3, and the 20 kg
  model 14 and 14. The brushless layout stands 6 steps and 2 walks; the 20 kg model stands 14 and 9.
- **How it fails:** the robot places the front foot 105 mm ahead (planned 102). About 2 s later it falls while
  moving its weight onto that foot (775 layout, base model, 0.10 m, held feet; `F4g` in the grid).
- **This meets one of D-060's reversal conditions** ("the heavier swing leg fails the full step grid"). The hip
  drives on the thighs cost it; the battery move does not (below). Putting only part of the hip bank on the thighs
  does not get both the chair and the held step.
- **The model stays on the 20 kg masses (D-061).** D-036 said it moves to the roll-up when the mass comes to about
  22 kg. The brushless layout is 19.8 kg, but it fails the contact the model is judged on.

## The grids

Commands: `step_sweep.py --lengths 0.085,0.09,0.095,0.1,0.105,0.11,0.115 --scene step|walk --masses ...`. Variants:
`"soft: "`, `"held: NOSLIP=5"`, and `ROBOT_MOTORS=low` on soft contact. Each cell is out of 14 (two models,
seven lengths).

Step:

| Drive | Contact / motors | 20 kg model | Roll-up as before | Layout (D-060) |
| --- | --- | --- | --- | --- |
| brushless (`ROBOT_WMAX=bldc`) | soft | 12 | - | 14 |
| brushless | held feet | 14 | - | **6** |
| brushless | soft, `ROBOT_MOTORS=low` | 13 | - | 13 |
| 775 | soft | 14 | 14 | 14 |
| 775 | held feet | 14 | 8 | **0** |
| 775 | soft, `ROBOT_MOTORS=low` | 7 | 8 | 14 |

Walk:

| Drive | Contact / motors | 20 kg model | Roll-up as before | Layout (D-060) |
| --- | --- | --- | --- | --- |
| brushless | soft | 10 | - | 8 |
| brushless | held feet | 9 | - | **2** |
| brushless | soft, `ROBOT_MOTORS=low` | 8 | - | 5 |
| 775 | soft | 14 | 14 | 12 |
| 775 | held feet | 14 | 3 | **0** |
| 775 | soft, `ROBOT_MOTORS=low` | 3 | 1 | 12 |

The "roll-up as before" column for the brushless build is blank: the brushless snapshot of docs/34 was not rerun.
The grid sits on a knife edge (docs/19): differences of two or three are noise. 0 against 8 and 14 is not.

The held-feet failures on the layouts are nearly all `F4g`: a fall in phase 4, the weight transfer onto the front
foot, with the lift gate then timing out. On held feet the 775 layout's second foot also lands 28 to 48 mm long.
On the pessimistic motors the layout does better than the 20 kg model, perhaps because the lighter pelvis asks
less of the weak hip drives. That was not traced.

## One move at a time, and part of the hip bank (P-062)

The run the host stopped was rerun as three calls of 14. Then `mass_rollup.py --hips-on-thighs N` put only N of the
six hip drives on the thighs. All rows are on held feet, 7 lengths x 2 models, with `--spread`. The reach is at
0.45 m.

| Build | Moves | Step on held feet | Reach | 0.45 m chair |
| --- | --- | --- | --- | --- |
| 775 | spread only | 7 / 14 | - | - |
| 775 | + battery up | 6 / 14 | 12.8 cm | no (docs/33) |
| 775 | + battery up, 2 hip drives on the thighs | 8 / 14 | 11.9 | - |
| 775 | + battery up, 4 hip drives on the thighs | 3 / 14 | 10.9 | no (0.50 m yes) |
| 775 | + 6 hip drives on the thighs, battery as before | **0 / 14** | - | - |
| 775 | + battery up, 6 hip drives on the thighs (the layout) | 0 / 14 | 10.0 | yes |
| brushless | + battery up, 4 hip drives on the thighs | 4 / 14 | 8.6 | yes (0.50 m: the neck slips 631 degrees) |
| brushless | + battery up, 6 (the layout) | 6 / 14 | 8.0 | yes |

- **The hip drives on the thighs lose held feet; the battery move does not.** Moving the battery alone keeps the
  step at the spread roll-up's level (6 against 7).
- **No layout gets both.** The chair from 0.45 m needs the reach at about 8.7 cm on the brushless build and
  10.0 cm on the 775. Each hip drive on the thighs buys about 0.45 cm of reach and costs the held step. The 775
  needs all six for the chair, and with six it steps 0 of 14.
- **So the conflict is in the control, not the layout.** The forward weight transfer on held feet does not carry a
  heavier swing leg. Even the old roll-up stands only 8 of 14 there, against 14 for the 20 kg model.

## Packaging the thigh bank (no generator yet)

Six W-max drives ride on each thigh: the knee, the two ankle drives and three for the hip. They are estimated, not
drawn:
- the brushless drive, a 5010 plus a printed 1:25 and the drum: about 60 mm across and 100 mm long;
- the 775 drive with its 1:40: about 55 mm across and 140 mm long.

The thigh is 430 mm (`cad/skeleton.json`). About 300 mm of it is clear of the hip and knee joints. The hips sit
200 mm apart.

- **Brushless:** a column of three in front of the 48 mm femur and three behind it fit in the 300 mm. The thigh is
  then about 66 mm wide and 175 mm deep.
- **775:** the drives are too long for columns of three. Two columns of two go in front and one column of two
  behind, about 110 mm wide and 165 mm deep.
- **Collisions:** both stay inside the model's 120 mm-wide thigh capsule. So the gap between the thighs, which
  the model collides on, does not shrink. A ring of six drives around the femur would make a 170 to 185 mm round
  thigh, leaving 15 to 30 mm between the thighs. That layout is ruled out.
- **Depth:** the deeper thigh is not modelled. It lifts the pelvis about 25 mm when seated, and the drives behind
  the femur must clear the calf at deep knee flexion.

## Smaller fixes

- `mass_rollup.py`: the brushless W-max is a 1:25 at 24 V (docs/21, docs/34); the code said 1:16 in two places.
  At 24 V the brushless build's battery is a 6S 5 Ah, with the same 111 Wh and mass as the 3S 10 Ah, and the
  roll-up now names it so. The snapshots reproduce unchanged.

## Not yet

- A forward weight transfer that holds on held feet with heavy thighs. That is the old hub task `step-on-held-feet`
  (D-043), now with the layout masses as its hardest case.
- The 0.50 m chair on the brushless layouts (the neck slips); the chair with armrests.
- The battery tray and the thigh bank as generators (the W-max itself has no generator; task item 3, the gearbox).
- The deeper thigh (165 to 175 mm) in the model's collision shapes.

## Next

Trace the forward transfer (phase 4) on held feet: the 775 layout at 0.10 m against the 20 kg model, and where
their CoM, CoP and hip-line forces part. Retune it for the heavy swing leg, then rerun the held grid on the layout.
Keep the battery-up move whatever the outcome (D-061). It costs nothing on held feet and buys 0.6 to 0.7 cm of
reach.
