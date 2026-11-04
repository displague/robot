# 32 - The 8 m/s wind and getting up from a half-kneel (P-052, D-053)

The afternoon of P-052, after `31`. Two open items for learned control: no learned stand policy holds the gate's
8 m/s head wind, and the robot cannot get up (sibling #49).

Short answers:
- **The 8 m/s wind is not the drag and not the action range; its cause is not found.** Drag at 8 m/s is 16.4 N on the tendon model and
  15.6 N on the stand-in. Widening each joint's action range to the teacher's own corrections did not get it, and a
  200-iteration all-windy fine-tune on the tendon model did not either. A policy that falls at 8 m/s on the tendon model
  stood on two of the three windy stand-in variants tried. The cause is not found; the drive under a steady load
  is the leading suspect, untested.
- **The robot cannot rise slowly from below a pelvis height of 0.85 m.** With the ankle's 20 degrees of
  dorsiflexion, no one-leg pose with a flat foot puts the CoM over the sole lower than that (`tools/rise_static.py`).
  Getting up must be a dynamic rise: momentum, the other leg, or the hands.
- **A two-knee kneel does not rest on this robot**; one half-kneel does. Get-up as a curriculum from it: see below.

## The 8 m/s wind

- **The policies' action range clipped the teacher.** Each action spans 0.2 of its joint's range. That clipped the
  hand-written controller's corrections in 11.6 % of its demonstration steps: ankle rolls need up to 2.3 times
  the range, ankle pitch and knees about 2.4 times. The sibling's `--teacher-range` sizes each joint from the
  teacher instead (1.2 times its 99.5th percentile), which cuts the clipping to 1.4 % of steps. Three seeds, each with
  and without a held load: still no 8 m/s wind, and without the load they stand at 300 to 430 W. The default
  stays 0.2.
- **Drag agrees between the two models** (above). In the gate the drag reaches 21 to 28 N with gusts, and the policy
  never leans into it: the CoM drifts back and it goes over at 5.5 s.
- **A wind fine-tune on the tendon model** (every worker windy up to 9 m/s, a lighter power cost, 200 iterations,
  1.5 million steps, from the best held-load policy): still falls at 8 m/s, and standing power went from 36 to
  83 W.

Not found yet. The leading suspect is the ankle drive under a steady load: it is a line loop that stretches, its
outer loop runs at 0.15 gain on the ankles (`OUTER_GAIN`), and the hand-written controller does not position the
ankles at all in this scene but works them in torque mode (the gear placed so the lines carry the planned torque,
docs/15). But the stand-in carries the same outer loop, and the integral trim reaches the target in both, so this
was a hypothesis. A direct test: the policy drives every joint but the ankles, and the hand-written balance
loop drives the ankles alone (torque mode, its hip lean on top). Ten cloned policies (five plain, five held-load),
tendon model:

| Scene | Policy alone | Hand-written ankles under the policy |
| --- | --- | --- |
| 8 m/s wind | 0 of 10 | **2 of 10** (the plain seeds 2 and 3, at 51 and 54 W) |
| 5 m/s wind | 7 of 10 | 9 of 10 |
| 30 N push | 10 of 10 | 8 of 10 |
| stand 10 s | 10 of 10 | 9 of 10 |
| standing power, median | 106 W | 85 W (one seed 253 to 28 W) |

The ankle strategy is part of it: with the hand-written ankles two policies hold the 8 m/s wind, the 5 m/s wind
nearly always holds, and power mostly falls. It is not all of it: eight still fall at 8 m/s, and two lose a scene
they held alone (the policy was not trained with those ankles). Leaving the balance loop on under the policy's own
ankle targets instead made both policies tried worse (the two fight). Next: train with the ankle in torque mode
(hub task `policy-ankle-torque-mode`).

## Getting up: what this robot can do

`tools/rise_static.py` searches one-leg poses (hip, knee, ankle, spine) with the body pitched so the front foot is
flat and the whole-body CoM within 4 cm of the sole's centre, and checks the front knee's and hip's gravity torques
against their drives (the clutch slip at the joint):

| Ankle dorsiflexion allowed | Lowest pelvis height, slow one-leg rise within the drives | Geometry alone |
| --- | --- | --- |
| 0 degrees | 0.95 m | 0.95 m |
| 20 degrees (the joint's limit) | 0.85 m | 0.85 m |
| 30 degrees | 0.80 m | 0.75 m |
| 40 degrees | 0.80 m | 0.65 m |

At the robot's 20 degrees the limit is geometric: the CoM cannot get over the foot. More ankle range helps only down
to 0.80 m before the knee and hip drives run out. A slow rise from a half-kneel (pelvis 0.46 m) is not possible on
this robot as built; a dynamic one might be.

## Get-up as a curriculum (sibling #49)

Starts: a two-knee kneel does not rest. With the shins flat the feet stand on their toes' edge (the ankle cannot
plantarflex past 40 degrees to lay them flat), and with the knees bent further only the knees touch. A search over 32
half-kneels found one that rests upright: front hip flexed 90 degrees, knee 95, ankle -5; back hip +5, knee 100;
pelvis 0.46 m.

Curriculum (`StandinEnv(task="getup", getup_start="curriculum")`): start half-kneeling, then add fallen starts once
the half-kneel succeeds (stands 1 s) on more than half of its episodes.

| Run (plain network, random start) | Steps | Half-kneel success | Fallen success |
| --- | --- | --- | --- |
| 500 iterations, unlock after 50 episodes, 15 % rehearsal | 24.6 M | 0.20 and 0.30 (two seeds) | not unlocked |
| 1,500 iterations, same | 73.7 M | 0.25 and 0.16: unlocked early, then forgot | 0.00 |
| 1,500 iterations, unlock after 200 episodes, 50 % rehearsal | 73.7 M | 0.50 and 0.50 | 0.00 |

Tested deterministically from 64 half-kneel starts (the stand-in, model variants randomised):

| Policy | Rise and stand 1 s | Still standing at 10 s | Ended by the damage gates | Worst line, force / rating | Peak shell impact |
| --- | --- | --- | --- | --- | --- |
| 500 iterations, seed 1 | 41 % | 0 % | 61 of 64 | forearm 2.3 | 3.1 kN |
| 1,500 iterations, seed 0 | **61 %** | 28 % | 9 of 64 | forearm 2.3 | 2.8 kN |
| 1,500 iterations, seed 1 | 55 % | 19 % | 11 of 64 | forearm 2.3 | 1.7 kN |

The rise is a lunge of about half a second: pelvis 0.47 m at the start, 0.87 m at 0.5 s, standing at 1 s (the
sibling's `docs/assets/rl-tendon32-getup-halfkneel.gif`). That matches the static result: only a dynamic rise is
possible. The arms help by swinging, and their lines (forearm, W-mid) run at 2.3 times their continuous rating for
up to the 2 s the damage gate allows; that is too hard on them for the robot. Policies at 500 iterations threw
themselves up and mostly landed lying down.

## Not yet

- The 8 m/s wind for a learned policy: the ankles' stretch under load in the stand-in, or the ankle torque mode
  given to the policy.
- A get-up that works from a fall (0 %), on the tendon model, and within the arms' line ratings (the arm lines'
  overload is cut at 2 s, not prevented).
- Hands in the get-up (the arms are not used yet), and a two-knee kneel with a ratchet or longer plantarflexion.

## Next

Give the policy the ankle torque mode the hand-written controller uses (a torque-equivalent ankle action), and
retrain the get-up with a reward for the momentum of a controlled lunge.
