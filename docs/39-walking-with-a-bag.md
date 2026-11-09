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

- Bags over 3 kg in the walk. docs/28's 3.9 kg walk had the bag inside the robot's CoM from the first step.
- Sitting with a bag: lift the forearm while seated, or set the bag down first (a planner rule).
- An implicit grip damping (a weld with soft rotation, or a joint switched in), so stiffer grips do not spike.
- The ball's 1.2 kg walk in the chain falls although 1.2 kg walks alone. The chain's walk comes right after the pick,
  with the bag possibly still swinging.

## Next

Run the chain grid with a settle after the pick, and with the planner keeping a held bag off the floor while
seated (forearm up). Then take the bag-carry case into the held-feet step retune (docs/37 Next).
