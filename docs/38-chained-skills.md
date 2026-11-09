# 38 - Skills back to back: a bag picked up and put down, a walk and a chair in random orders (P-063, D-062)

The gauntlet starts every scene from a fresh stand. A day is not like that: the robot sits down after a walk,
stands up holding a bag, walks with it, and puts it down. `tools/chain.py` runs such days. Each seed draws a random
order of pick, walk, sit (0.45 m) and place, plus one of six bags, with no reset between the skills.

Short answers:
- **Chaining found a real bug at once.** The walk ends with the planner's swing flags still set, the hip shift at
  7 degrees and the arms 10 degrees out. A chair right after the walk then sits but cannot stand, though it stands
  from a fresh start. `Rig.to_stand()` now hands back a quiet stand between skills. With it, walk-then-sit stands.
- **Bags can be picked up and put down.** They are free bodies that a hand grips (`export_mjcf.ITEMS`):
  - six of them: a grocery bag, a tall bag, a 4.5 kg box, a 3.9 kg jug, a 0.8 kg wide bag and a ball in a net;
  - two stools at counter height.
  - The pick lifts the bag off a stool with the elbow, so the robot carries its weight. The put-down sets it on a
    top under where it hangs and lets go.
- **Without a bag in hand, 7 of 9 chains are done.** One walk fell after a put-down, and the ball rolled off its
  top.
- **Walking with a bag in the hand fails in 9 of 10 chains, even with the 0.8 kg bag.** docs/28 walked with
  3.9 kg, but that bag was part of the robot's body. Its weight was in the balance loop's CoM, and it hung on a
  damped ball joint.
- **Feeding the felt weight to the balance loop does not fix it.** Results out of 24, from different orders:

  | Felt weight | Chains done |
  | --- | --- |
  | not felt | 10 |
  | raw grip force | 6 |
  | low-passed 0.5 s | 8 |

  The grip force swings to 10 to 15 times the bag's weight in the walk, so the cause is likely the swing, not the
  missing weight. `FEEL_LOAD` stays off.
- **Does any of this exercise audio, vision or felt weight?** Not yet; see "What the policy can sense" below.

## The chain grid

`python tools/chain.py --runs 24 --jobs 12`, the 20 kg model, soft contact, 0.45 m chair. Each row is one seed;
some seeds draw the same order.

| | With the hand-back (`Rig.to_stand`) | Without (`--no-handback`) |
| --- | --- | --- |
| Chains done | **10 / 24** | 6 / 24 |
| No bag in hand during a walk or a sit | 7 / 9 (a walk after a put-down fell; the ball rolled off) | 6 / 9 |
| Bag in hand during a walk | 1 / 10 | 0 / 10 |
| Bag in hand during a sit, not a walk | 2 / 5 | 0 / 5 |

With the felt weight on (`FEEL_LOAD`), the three rows are 5/9, 0/10 and 1/5 for the raw grip force, and 6/9, 0/10
and 2/5 for it low-passed.

The hand-back matters most when a pick or a sit follows a walk; without it, picks after a walk fail. The
differences of one or two chains in a row are within the grid's scatter. 0 against 2 of 5 for sitting with a bag,
and 6 against 10 overall, are probably not.

## What was built

- **`export_mjcf.ITEMS`** (or `ROBOT_ITEMS`, a JSON file or string): free bodies with a handle.
  - **The grip** is a connect equality between a hand's grip site and the handle, off until `Rig.grip` closes it.
    The item swings freely about the grip, as a bag on its handles does.
  - **Contact:** the items and two mocap stools use contact bit 2, so they meet the floor and the stools but
    never the robot.
  - With no items the model is unchanged: `sim/robot.xml` regenerates byte for byte.
- **`Rig` helpers:**
  - `place_stool`, `place_item`, `grip`;
  - `held_load`: the grip's vertical force, the felt weight as ground truth;
  - `felt` and `com_est` (`FEEL_LOAD`, `FEEL_TAU`);
  - `feet_origin`, with `origin` for `set_chair`, so the chair is placed from where the robot now stands;
  - `to_stand`, the hand-back.
- **Fixes for a world with things in it:**
  - The CoP skipped no floor contact, so an item on the floor counted as a foot. A step with a bag parked across
    the room fell.
  - The static reach used the whole world's CoM, so it came out at 90 cm. It now uses the robot's own.
  - Side effect: the reach no longer counts the chair's 4 g of mocap bodies, so 7.8 cm becomes 7.7 cm at 0.45 m.
    The chair scene still stands, at 1025 J instead of 978 J. Every other gauntlet scene is unchanged to the joule.

## What the policy can sense (the owner's question)

The sibling's tendon environment (`robot_tendon.py`) gives the policy:
- joint angles and rates, and the drum angles;
- line tensions, with slack and clutch-slip flags;
- lock state and motor torque;
- the pelvis IMU, the foot loads and the CoP.

There is no camera, no microphone and no touch:
- **Weight in the hand** shows up only indirectly, as wrist and elbow line tension and motor torque. The model now
  also gives the true grip force (`held_load`), which a palm load cell or the finger line tensions could measure.
- **Vision and audio** are not modelled.

So the chains exercise balance, the drives and the proprioceptive channels, not hearing or sight. Hub task
`perception-channels` adds depth vision, contact-event audio and a felt-weight channel as named channel families,
each ablated.

## Not yet

- Walking with a bag in hand: the grip force's swing, the connect's stiffness (`solref` 0.01), and how docs/28's
  ball-jointed bag walked with 3.9 kg.
- Lowering a bag by squatting or reaching to a lower top. A bag picked from the floor.
- Fingers, the pillow and the pour: hub task `hands-pillow-and-pour`. Training across skills (`mixed-skill-
  training-gate`), outside teachers (`external-teacher-transfer`), the physics on the GPU (`tendon-physics-on-gpu`).
- The chain grid on held feet and on the layout masses.

## Next

Find why the carried bag's grip force swings to 10 to 15 times its weight in the walk. Compare a damped connect,
and a ball joint switched in at the grip, against docs/28's bag. Then rerun the chain grid, and judge the felt
weight again once the swing is tamed.
