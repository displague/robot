---
title: "28 - Carrying a grocery bag: what it holds standing, what it walks with (P-047, P-049, D-049)"
summary: "Carrying a grocery bag: standing to 11.3 kg in one hand, walking to 3.9 kg (20 kg build), the fingers hook about 2.4 kg, so a palm hook (D-049)"
section: "Skills: wind, bags, chairs, chains"
---

# 28 - Carrying a grocery bag: what it holds standing, what it walks with (P-047, P-049, D-049)

The owner asked whether the robot can carry a grocery bag of 5 to 25 lb (2.3 to 11.3 kg), such as bread and eggs or a
gallon of milk. Short answers:
- **Standing, yes, all the way to 25 lb in one hand**, on the 20 kg build if the bag is lifted over about 1.5 s.
- **Walking, up to about a gallon of milk (3.9 kg) in one hand** on the 20 kg build. The 26 kg build (the real one at
  the sourced motor figures) walks with nothing in its hands, and with a 2.3 kg bag it falls.
- **The fingers are the first limit, not the legs.** The 100:1 finger motors hook about 2.4 kg continuously. A
  printed palm hook or a forearm strap carries the bag at no power and is the recommendation (D-049).

## The model

`export_mjcf.BAGS` hangs a bag from a link's tail on a ball joint (`{"hand.L": 3.9}`):
- a 0.10 m handle and a 0.10 x 0.25 x 0.30 m bag
- light damping, for the contents settling
- no collision: the legs pass through it

The balance loop sees the bag's mass through the whole-body CoM, which is the case where the planner has weighed it
(the foot load cells can, at the lift). `tools/carry_bag.py` runs one gauntlet scene with it:

```bash
python tools/carry_bag.py kg=3.9 style=down scene=walk ramp=1.5
```

It prints one line: whether it stands, the energy, the bag's peak swing, and the line forces and safety factors.

Carry styles:
- `down`: one bag in the left hand, arm hanging.
- `both`: one bag in each hand.
- `front`: the left forearm level in front, the bag on a strap at the wrist.

The bag is either there from the first step, or lifted, with its weight coming on over `ramp` seconds.

## Results

Cells read: bag there from the start / lifted over 1.5 s. S stands, F falls.

**20 kg build** (default masses and motors):

| Style | kg | Stand | One foot | Step | Walk |
| --- | --- | --- | --- | --- | --- |
| down | 2.3 | S/S | S/S | S/S | S/S |
| down | 3.9 | S/S | S/S | S/S | S/S |
| down | 6.8 | S/S | S/S | F/F | F/F |
| down | 11.3 | F/S | F/F | F/F | F/F |
| both | 2.3 each | S/S | S/S | S/F | F/F |
| both | 3.9 each | S/S | S/S | F/F | F/F |
| both | 6.8 each | S/S | F/F | F/F | F/F |
| both | 11.3 each | S/S | F/F | F/F | F/F |
| front | 2.3 | S/S | S/S | F/F | F/F |
| front | 3.9 | S/S | S/S | F/F | F/F |
| front | 6.8 | S/S | S/S | F/F | F/F |
| front | 11.3 | F/S | F/S | F/F | F/F |

**26 kg build** (`rollup_775.json`, `ROBOT_MOTORS=nom`):

| Style | kg | Stand | One foot | Step | Walk |
| --- | --- | --- | --- | --- | --- |
| down | 2.3 | S/S | S/S | F/S | F/F |
| down | 3.9 | S/S | S/S | F/F | F/F |
| down | 6.8 | S/S | F/F | F/F | F/F |
| down | 11.3 | F/S | F/F | F/F | F/F |
| both | 2.3 each | S/S | S/S | S/F | F/F |
| both | 3.9 each | S/S | F/F | F/F | F/F |
| both | 6.8 each | S/S | F/F | F/F | F/F |
| both | 11.3 each | S/F | F/F | F/F | F/F |
| front | 2.3 | S/S | S/S | F/F | F/F |
| front | 3.9 | S/S | F/F | F/F | F/F |
| front | 6.8 | S/S | F/F | F/F | F/F |
| front | 11.3 | F/S | F/F | F/F | F/F |

### What limits each case

- **Lifting the bag.** Lifted over 1.5 s, the 20 kg build stands with 11.3 kg in one hand and stands on one foot
  with 6.8 kg. Hung on at once, 11.3 kg slips the left ankle's clutch and falls.
- **The walk's limit is the added mass, not where it hangs.** Two 2.3 kg bags (4.6 kg) fall in the walk, where one
  3.9 kg bag walks. On the 20 kg build the walk carries about 4 kg, a fifth of its weight.
- **A one-hand bag fails on the transfer away from it.** With 6.8 kg on the left, the weight shift onto the right
  foot times out. The left hip roll peaks at 303 N, over its 288 N continuous rating.
- **The front carry stands but does not step.** With the forearm level and nothing in it, the step passes. With
  2.3 kg on it, the step falls: the CoM sits further forward than the step's gates are tuned for.
- **The 26 kg build has no margin.** It walks empty at the sourced figures, but even 2.3 kg fails the walk. Its leg
  drives are already near their limits (docs/24, `wmax-stands-at-low`).

### Line forces (20 kg build, one-hand bag, the walk)

| Line | Empty | 2.3 kg | 3.9 kg | Continuous rating |
| --- | --- | --- | --- | --- |
| Wrist pitch, hand.L_pitch (W-mini) | 17 N | 36 N | **51 N** | 37 N |
| Shoulder roll, upperarm.L_roll | 90 N | 118 N | 145 N | 288 N |
| Spine roll | 109 N | 169 N | 202 N | 288 N |
| Hip roll, far side (thigh.R_roll) | 155 N | 212 N | 261 N | 288 N |
| Ankle pitch, bag side (foot.L_pitch) | 192 N | 189 N | **287 N** | 288 N |

At 3.9 kg the wrist runs over its continuous rating, because the bag swings against it. The bag-side ankle
reaches its rating, which is why 3.9 kg is the walking limit.

### Energy

The walk covers 0.4 m. On the 20 kg build:

| Bag | Walk energy | Per metre | Change |
| --- | --- | --- | --- |
| Empty | 1.05 kJ | 2.6 kJ/m | |
| 2.3 kg | 1.65 to 1.77 kJ | 4.1 to 4.4 kJ/m | +56 to 68 % |
| 3.9 kg | 2.55 kJ | 6.4 kJ/m | +140 % |

It costs the most in holding torque, and grows faster than the mass.

## The grip: estimated, because the fingers are not in the model

A bag handle in a hook grip sits on the middle phalanx, about 14 mm past the PIP joint. The finger's single flexor
tendon has a 6.5 mm moment arm at the PIP (`finger.py`). So each finger's tendon pulls about 2.2 times its share of
the load. With four fingers sharing it, each tendon carries 0.55 times the bag's weight.

| Finger drive | Tendon force, continuous / stall | Bag held, continuous | Held at stall (momentarily) |
| --- | --- | --- | --- |
| N20 100:1 (the plan, D-005) | 13 / 42 N | **2.4 kg** | 7.8 kg |
| N20 298:1 (the "grip" variant) | 37 / 122 N | 6.9 kg | over 11 kg |
| Passive palm hook or forearm strap | none | the arm's limit | |

So bread and eggs are at the edge of the planned fingers, and a gallon of milk is past it. A printed hook does the
job at no power: a J-hook at the heel of the palm, or a strap loop on the forearm (the `front` style). Either
takes the load off the finger and wrist lines, and the hand only has to place the handle (D-049).

## Not yet

- The fingers in the model; the grip and the hand's tendon loads under a real handle.
- The bag colliding with the legs: a 0.25 m bag at the hip swings into the thigh in the step.
- A planner that leans the torso away from a one-hand bag (counter-tilt), and step gates retuned for a load in
  front.
- Weighing the bag at the lift from the foot sensors, rather than knowing its mass.

## Next

A palm hook in the hand generator. Then re-run the walk grid with the bag once the W-max choice
(`wmax-stands-at-low`) gives the legs margin; the carry limit is the same drive question as the 26 kg walk.
