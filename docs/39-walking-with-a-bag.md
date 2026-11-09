# 39 - Walking with a bag in hand: the balance loop must know the bag, and the grip must damp it (P-064, D-063)

docs/38 found that walking with a gripped bag fails in 9 of 10 chains. The grip force peaked at 10 to 15 times the
bag's weight, so the swing looked like the cause. This doc traces it on single walks (the gauntlet's `walk`
scene). The bag is a free item gripped at the left hand after a 1 s stand.

Short answers:
- **The swing is not the cause.** Until the fall the bag swings less than 8 degrees, and the grip carries 1.0 to
  1.3 times its weight. The 10 to 15 times peaks come after the robot has started to fall, when the bag hits the
  floor.
- **The cause is that the balance loop does not know about the bag.** With no knowledge of it, 0 of 7 bag weights
  from 0.8 to 3.0 kg walk. Given the bag's true, moving centre of mass, as docs/28's bag gave it (that bag was part
  of the robot's body), 5 of 7 walk.
- **Its mass alone, placed at the hand, is not enough.** That walks 1 of 6. The bag's centre wanders ±2 cm about
  the grip (17 mm standard deviation, 8 cm at most), and the loop needs that motion as well.
- **No estimate from the wrist force tracked the swing.** Tried:
  - the force direction as a pointer to the bag;
  - the horizontal pull as a ZMP load at the grip's height;
  - a pendulum model driven by the hand's motion, with and without the force direction mixed in.

  All of these fell sooner than no estimate at all. The pendulum drifts out of phase in seconds. The force
  includes the hand's own accelerations and the box's rotation.
- **What works is to damp the swing and feel the weight.** A grip that resists the bag turning against the hand
  (`GRIP_DAMP`, 0.5 N m s/rad) plus the felt weight in the balance loop (`FEEL_LOAD`) walks 0.8, 1.2 and 2.3 kg.
  3.0 kg falls, as it does even with the bag's true CoM. Friction without the felt weight walks none.
- **On the chain grid:**
  - 13 of 24 chains are done, against 10.
  - Walks with a bag in hand finish 6 of 10, against 1. The 4.5 kg box still fails every walk.
  - Both are now the defaults (D-063). They act only when the model has items, so the gauntlet is unchanged.

## Single walks

`walk` scene, 20 kg model, soft contact. A grocery-bag box (0.10 x 0.25 x 0.30 m) on a 0.10 m handle at
`hand.L`. S stands to the end; F falls (the fall time in s).

What the balance loop is told about the bag:

| kg | Nothing | True mass at the grip | True moving CoM |
| --- | --- | --- | --- |
| 0.8 | F 78 | - | S |
| 1.2 | F 61 | S | F 62 |
| 1.6 | F 17 | F 18 | S |
| 2.0 | F 18 | F 75 | S |
| 2.3 | F 15 | F 16 | S |
| 2.6 | F 19 | F 24 | S |
| 3.0 | F 19 | F 15 | F 47 |

Estimates from the wrist force, 2.3 kg: each falls in 11 to 14 s. The pendulum observer's error is 13 to 38 mm in
the first 10 s, as large as the swing.

Damping the swing (`GRIP_DAMP`, N m s/rad):

| kg | 1.0, not felt | 1.0, felt | 0.5, felt |
| --- | --- | --- | --- |
| 0.8 | F 48 | F 14 (grip spike) | **S** |
| 1.2 | F 49 | S | **S** |
| 2.3 | F 83 | S | **S** |
| 3.0 | F 90 | F 18 | F 19 |

- **The damping alone delays the fall from about 15 s to 50 to 90 s.**
- **At 1.0 the light bag spikes the grip** (1.6 kN at 1.5 s). The damping is an explicit torque between the bag and
  the hand. The hand's smallest inertia, 0.24 g m², limits it to about 0.5 N m s/rad at the 1 ms step; 2.0 goes
  unstable at once.

## The chain grid

`python tools/chain.py --runs 24 --jobs 12 [K=V]`, as docs/38.

| | Chains done | No bag in a walk or sit | Bag in hand in a walk | Bag in hand in a sit, not a walk |
| --- | --- | --- | --- | --- |
| docs/38 (pivot, not felt) | 10 / 24 | 7 / 9 | 1 / 10 | 2 / 5 |
| `GRIP_DAMP=1.0` | 10 / 24 | 8 / 9 | 3 / 10 (walks done; one then failed a sit) | 0 / 5 |
| `GRIP_DAMP=1.0 FEEL_LOAD=True` | 11 / 24 | 8 / 9 | 3 / 10 | 0 / 5 |
| `GRIP_DAMP=0.5 FEEL_LOAD=True` (now the default) | **13 / 24** | 8 / 9 | **6 / 10** | 1 / 5 |

The bag column counts walks done with the bag in hand, even where a later segment failed. The chain's bags in a walk
are the pillow bag (0.8 kg), the ball (1.2), the tall bag (3.0) and the heavy box (4.5):
- **At 0.5 with the felt weight:** the pillow bag walks 3 of 3, the ball 1 of 1, the tall bag 2 of 3 and the heavy
  box 0 of 3.
- **At 1.0:** only the pillow bag walks.

Sitting with a bag: the jug's one sit stood. The differences of one between the rows of the first two columns are
within the grid's scatter. 1 against 6 of 10 is not.

**Sitting with a bag is a separate failure.** Seated, the hand drops to about 0.47 m. The tall bag hangs 0.48 m
below its grip, so it lands on the floor. The robot sits but does not stand, with or without the fixes. Its
reach at seat-off is computed without the bag.

## Sitting with a bag: set it down first (D-065)

`chain.SIT_SETDOWN` (on) is the planner rule a person follows. Before a sit with a bag in hand, the robot sets the
bag down on a top under where it hangs. It then sits, stands, takes a quiet stand, and takes the bag up again from
where it rests. If the handle is more than 60 mm from the hand (`REACH_TOL`), that counts as a failure. After the
stand the handle is 28 to 30 mm from the hand, and the grip closes that gap.

The chain grid, with the press default of D-064:

| | Chains done | Bag in hand into a sit |
| --- | --- | --- |
| The bag held through the sit | 12 / 24 | 0 / 5 |
| **Set down first** | **17 / 24** | **5 / 5** |

The 7 left:
- six walks with the 3.0 kg tall bag or the 4.5 kg box;
- the ball rolling off its top once (`F:place`).

The put-down now waits, for up to 2 s, until the item is still on its top before letting go (a person does the
same). The ball stays put, and the grid is **18 of 24**. The only failures left are the six heavy one-handed walks.

The heavy bags are past what one hand walks with (docs/28 and the single walks above).

## What was built

- `sim_tendon.GRIP_DAMP` (N m s/rad, now 0.5) and `Rig._grip_damp`: on each closed grip, a torque against the
  item's rotation relative to the hand, with its reaction on the hand. It models the fingers' or a palm hook's
  friction on the handles.
- `FEEL_LOAD` now defaults to on (docs/38 had it off).
- No change without items: the gauntlet and `sim/robot.xml` are untouched.

## What it means for the real robot

- **A grip that pivots freely is the worst case.** A palm hook with a rubber pad or a strap (D-049) damps the bag
  against the hand.
- **The robot needs to feel the weight.** A wrist or palm load cell measuring the vertical load is enough when the
  swing is damped. Its direction is not needed.

## Not yet

- Bags over 3 kg in the walk. docs/28's 3.9 kg walk had the bag inside the robot's CoM from the first step. With
  the end-of-day defaults (D-064, D-066, D-067) single walks give:
  - 3.0 kg: stands (it fell before);
  - 3.9 and 4.5 kg: fall, with or without the slower ramp `TP_B=6 ROLL_KD=0.5` (it only delays the fall);
  - on held feet: falls from 3.0 kg up.

  So one hand walks about 3 kg on soft contact, for the grocery bag's shape. The chain's tall bag (3.0 kg, hanging
  lower on a shorter handle) still fails its walks. Waiting up to 3 s for it to hang still after the pick does not
  change that (tried and reverted), so the shape matters as well as the weight.
- **All six shapes at 3.0 kg, single walks:**

  | Shape | Grip to centre (m) | Walk |
  | --- | --- | --- |
  | grocery | 0.25 | stands |
  | heavy box | 0.14 | stands |
  | pillow bag | 0.25 | stands |
  | jug | 0.18 | falls at 41 s |
  | tall | 0.28 | falls at 32 s |
  | ball | 0.26 | falls at 34 s |

  No single property splits them: not the hang length, and not the height off the floor. These are one run each on a
  knife-edge walk, so 3 kg is the edge, where the shape tips it either way. The planner rule follows: one hand walks
  bags up to about 2.5 kg. Heavier ones need both hands or a strap at the body (D-049).
- **A bag in each hand** (two free items, the same shape, today's defaults), single walks:

  | Per hand | Grocery | Tall | Heavy box |
  | --- | --- | --- | --- |
  | 1.5 kg | stands | - | stands |
  | 2.25 kg | stands | stands | falls at 17 s |
  | 3.0 kg | falls at 14 s | - | falls at 17 s |

  Split across both hands, 4.5 kg walks for two of the three shapes on soft contact. On held feet, 2 x 2.25 kg falls
  at 21 s. docs/28 found both hands worse than one (2.3 kg each fell). Damping the grip and feeling the weight turns
  that round. The planner rule: up to about 2.5 kg in one hand, and about 4.5 kg split across two.
- **Sitting with a bag.** Tried: the felt weight at the gripping hand in the sit's planned poses, and `com_est` in
  the stand-up gate. It made things worse. The arms are forward at seat-off, so the bag moved the planned reach
  from 7.7 cm to between -2.6 and +1.9 cm. The planner then leaned too little, and the jug no longer sat. The real
  bag rests on the floor while the robot is seated, so it is not in the hand at seat-off. Reverted. What is needed
  is a planner rule, not a CoM term: set the bag down before sitting, or hold it clear of the floor and lift it only
  once standing.
- An implicit grip damping (a weld with soft rotation, or a joint switched in), so stiffer grips do not spike.
- The ball's 1.2 kg walk in the chain falls although 1.2 kg walks alone. The chain's walk comes right after the pick,
  with the bag possibly still swinging.

## Next

One hand walks about 2.5 kg, and two hands about 4.5 kg split, on soft contact. The split is in the chain as an
option (`chain.SPLIT`, off; `ONE_HAND_KG` 2.5). With it on the grid is 15 of 24, against 17:
- it gains the tall bag's walks;
- it loses three sits: after standing, the right hand comes back up to 158 mm from its handle;
- it loses two put-downs: one robot fell setting both bags down at once.

Next:
- make the two-hand take-up reach for each handle (the right arm's return pose), and set the two bags down one at a
  time;
- take the carried load onto held feet, where 2 x 2.25 kg still falls. That is the same held walk as docs/40's.
