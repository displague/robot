# 36 - Where the mass sits: hip drives on the thighs, the battery up (P-060, D-060, hub task wmax-mass-down)

docs/34 and docs/35 said that the brushless build loses the 0.45 m chair because its trunk is heavy (5.9 kg
against the model's 3.5). **That was wrong.** The trunk's mass hardly matters for the rise. What matters is how far
the CoM is behind the feet at seat-off, and the pelvis and the light arms push it back. Two moves fix it: the hip
drives ride on the thighs, and the battery and E-stop go into the torso. With them the 19.9 kg brushless build and the
26 kg 775 build both stand again from 0.45 m. The total mass does not change.

Short answers:
- **The static reach is the CoM's distance behind the sole centre at seat-off** (`sit_path`, with the hips at their
  110-degree stop). It decides the rise. The 20 kg model stands from 0.45 m at 7.8 cm. The brushless roll-up was at
  10.9 cm and the 26 kg 775 build at 13.6.
- **Per kilogram, the pelvis costs the most and the hands and forearms help the most** (table below). A segment
  behind the hips (the pelvis, which sits on the seat) moves the CoM back. A segment carried forward (the arms are
  out at seat-off) moves it forward. The trunk leans forward but its CoM stays near the hips, so it is close to
  neutral.
- **The gap to the model (10.5 against 7.8 cm) is mostly the arms.** The model guesses 1.8 kg of forearms and 0.8 kg
  of hands; the roll-up has 1.1 and 0.5 kg. That costs 2.1 of the 2.7 cm. The trunk's extra 1.6 kg *helps*, by
  0.65 cm.
- **Moving the shoulder drives into the pelvis is the wrong way**: the reach grows 1.4 cm, and the shoulder lines
  would be half as stiff. It was the plan in docs/35's Next, and it is rejected.
- **The hip drives on the thighs and the battery in the torso** bring the brushless build to 8.0 cm. Then it stands
  from 0.45 m, passes the whole gauntlet (the 8 m/s wind and the chair included), and stands 6 of 6 on the step grid.
  The 775 build with the same moves stands from 0.45 and 0.50 m, and 12 of 12 on the step grid and 6 of 6 on the walk grid.

## The reach, per kilogram

`ROBOT_WMAX=bldc`, on the brushless roll-up with the spread items (below). 0.5 kg added to one segment at a time
(0.25 kg a side for the paired ones). Seat 0.45 m, base reach 10.5 cm:

| Segment | Reach change per kg | Model kg | Roll-up kg | Share of the gap to the model |
| --- | --- | --- | --- | --- |
| pelvis | +1.2 cm | 3.50 | 3.47 | +0.04 cm |
| spine (trunk) | +0.4 | 3.50 | 5.13 | -0.65 |
| thigh | +0.2 | 4.40 | 3.90 | +0.10 |
| head | -0.6 | 0.80 | 0.54 | -0.16 |
| shank | -0.6 | 1.80 | 1.23 | -0.34 |
| foot | -0.6 | 1.20 | 1.09 | -0.06 |
| upperarm | -0.8 | 1.80 | 2.32 | +0.42 |
| forearm | -1.8 | 1.80 | 1.08 | -1.30 |
| hand | -2.6 | 0.80 | 0.48 | -0.83 |

(Paired segments are both sides. The shares add to -2.8 cm against the measured -2.7. Each share is the mass the
model has over the roll-up, times the reach change per kg.)

The model's distal masses are guesses from D-036's 20 kg budget, and the roll-up's are counted parts. No arm is
printed yet, so the real forearm and hand may come out between the two.

## The roll-up's new flags (`tools/mass_rollup.py`)

| Flag | What it does | Brushless pelvis / trunk / thigh (one), kg |
| --- | --- | --- |
| (none) | as before: every spread item in the trunk | 3.28 / 5.93 / 1.84 |
| `--spread` | wiring (400 g), fasteners (300 g), line and sheath (250 g) and joint sensors (135 g) booked over the segments by their mass, not all in the trunk | 3.47 / 5.13 / 1.95 |
| `--shoulders-low` | the 6 shoulder drives in the pelvis bank, with 12 x 0.45 m more sheath and line (export_mjcf `ROBOT_SHOULDERS_LOW=1`: the shoulder sheath 0.85 m) | 5.34 / 3.33 / 1.95 (with `--spread`) |
| `--hips-on-thighs` | the 6 hip drives ride on the thighs, 3 each, pulling against the pelvis | 1.60 / 5.13 / 2.89 (with `--spread`) |
| `--battery-up` | the battery (543 g) and the E-stop (196 g) in the torso shell | 0.81 / 5.91 / 2.89 (with all three) |

`--spread` is the honest booking: wiring and screws are all over the body. It stays a flag so the earlier snapshots
reproduce. The layout snapshots are `sim/masses/rollup_bldc_layout.json` (`--wmax bldc --spread --hips-on-thighs
--battery-up`, 19.9 kg) and `sim/masses/rollup_775_layout.json` (the 775 with BTS7960, same flags, 26.2 kg).

## Results

Chair, no armrests (stands again from 0.40 / 0.45 / 0.50 m), and the reach at 0.45 m:

| Build | Layout | 0.40 | 0.45 | 0.50 | Reach at 0.45 m |
| --- | --- | --- | --- | --- | --- |
| model, 20 kg | - | yes | yes | yes | 7.8 cm |
| brushless | roll-up as before | no | no | - | 10.9 |
| brushless | spread | no | no | - | 10.5 |
| brushless | shoulders low | no | no | - | 12.3 |
| brushless | spread + shoulders low | no | no | - | 12.0 |
| brushless | spread + hips on thighs | no | **yes** | no | 8.7 |
| brushless | spread + hips on thighs + battery up | no | **yes** | no | 8.0 |
| 775, 26 kg | roll-up as before (docs/33) | no | no | yes | 13.6 |
| 775, 26 kg | spread + hips on thighs + battery up | no | **yes** | **yes** | 10.0 |

On the brushless layouts the 0.50 m seat now fails: the neck's W-mini slips (up to 278 degrees summed) as the trunk
swings. docs/33 found the 0.50 m case flips with small changes. It needs the neck margin, not the layout.

Step and walk grids (2 models x 0.09, 0.10 and 0.11 m, `step_sweep.py --masses`; walk is `--scene walk`):

| Drive | Masses | Step | Walk |
| --- | --- | --- | --- |
| brushless | 20 kg model | 6/6 (docs/34) | 4/6 |
| brushless | roll-up as before | 6/6 (docs/34) | 4/6 (landing errors to 34 mm) |
| brushless | spread | 5/6 | 4/6 |
| brushless | spread + hips on thighs | 4/6 | 3/6 |
| brushless | spread + hips on thighs + battery up | **6/6** | **5/6** |
| 775 | 20 kg model | | 6/6 |
| 775 | roll-up as before | 6/6 | 6/6 |
| 775 | spread + hips on thighs + battery up | **6/6** | **6/6** |

On the brushless build the walk fails on every mass set, the 20 kg model's included. That is the drive's (docs/34),
not the layout's. The heavier thighs do not cost the step or the walk once the battery is up.

The brushless gauntlet at the layout masses stands every scene: the pushes, the arm with 1 kg, the squat, the 8 m/s
wind, the weight shift, one foot (357 J), the step (560 J), the walk (1879 J) and the 0.45 m chair (1339 J).

## Not yet

- The full grids: seven step lengths, held feet, and `ROBOT_MOTORS=low` on both layouts. The chair with
  armrests (its rows have matched the plain ones so far). The 0.50 m chair on the brushless layout (neck slip).
- `cad/skeleton.json` still carries the 20 kg model's masses (D-036): the layout is a roll-up and a variant, not the
  model.
- The shells' bosses and brackets are not placed for the new layout: the battery tray in the torso, and a
  three-drive bank on each thigh. The thigh bank has no generator. Its lines cross the hip to anchors on the pelvis.
- The swing leg is now 0.9 kg heavier a side; the step's swing timing was not retuned and did not need to be at
  these lengths.

## Next

Place the brackets: the battery tray on the torso shell's bosses and a three-drive thigh bank (a `winch_bank.py`
preset). Run the full grids and `ROBOT_MOTORS=low` on `rollup_bldc_layout` and `rollup_775_layout`. Then decide
D-036: move `cad/skeleton.json` to the layout's roll-up masses.
