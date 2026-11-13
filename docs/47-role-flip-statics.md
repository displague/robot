---
title: "47 - The role flip, first pass: can the robot be the helper? (P-072, D-074)"
summary: "A static check of a second robot taking docs/46's scripted helper's place, sample by sample along the get-up. Holding the other robot in its hands, it cannot: the wrists run at 5 to 17x their rating and the elbows at 3 to 8x, even at a quarter of the force. Cradling the other robot's upper arms on its forearms, near its own elbows, it can hold 74 of 94 samples within every joint rating, upright and without slipping. The two exceptions are the catch at the start of the lean (147 N to a robot still low: the spine at 2.5x) and the stand-up (the grips 5 cm out of reach)"
section: "Learned control"
---

# 47 - The role flip, first pass: can the robot be the helper? (P-072, D-074)

docs/43's round 3: a robot of the same build becomes the helper (hub task `nurturer-role-flip`). Before two
robots share one simulation, this asks the static question. Could a second robot put its hands where docs/46's
scripted helper put its own, carry the forces the getting-up robot put on them, stay within its joints' ratings,
and not tip over?

```bash
python tools/role_flip.py --record --set firm                 # docs/46's get-up, its helper sampled every 0.1 s
python tools/role_flip.py --set firm [--scale 0.5] [--contact elbow] [--evals 800]
python tools/role_flip.py --set firm --pose 6.0 [--contact elbow]   # one sample's best pose, in full
```

## The check

- **The samples:** docs/46's firm get-up (`sim/getup_params.json`), 94 samples while the helper holds. Each has the
  two grip points and the force the getting-up robot puts on each, up to 195 N together.
- **The helper robot:**
  - stands facing the other robot, feet flat;
  - its left side takes the other's right;
  - its posture is searched per sample: where it stands, its ankles, knees, hips, spine, shoulders, elbows and
    wrists, both sides alike. The search is a small cross-entropy one, warm-started from the last sample.
- **What the search minimises:** the worst joint load, with its contact points on the grip points.
- **The loads:** statics only (nothing is simulated).
  - Each pitch joint's torque comes from gravity and the contact forces, against its continuous rating: the line's
    continuous force times its moment arm (`sim/robot_standin.json`).
  - The legs share their load equally.
- **Upright:** its centre of pressure inside its feet (2 cm in from the edges), and the floor's friction (0.8)
  holding.

## Results

Holding the other robot in its hands (the grip site in each hand):

| Force | Samples within every rating | Worst joint | Wrist, elbow, spine, hip peaks / rating |
| --- | --- | --- | --- |
| As the scripted helper (x1) | 0 of 94 | the wrist | 16.8, 8.4, 4.8, 2.3 |
| Half | 0 of 94 | the wrist | 9.2, 4.6, 2.9, 1.5 |
| A quarter | 9 of 94 | the wrist | 4.8, 2.9, 2.0, 1.1 |

Cradling: it carries the other robot's upper arm on its forearm, near its own elbow (`--contact elbow`):

| Force | Samples within every rating | Spine | Hip | Shoulder | Elbow |
| --- | --- | --- | --- | --- | --- |
| x1 | **74 of 94** | 2.50 | 1.34 | 1.09 | 0.61 |
| Half | 77 of 94 | 1.82 | 1.01 | 0.62 | 0.61 |
| A quarter | 80 of 94 | 1.43 | 0.83 | - | 0.63 |

Per phase, cradling, x1:

| Phase | Samples | Force N | Worst | Joint | Reach error | CoP margin |
| --- | --- | --- | --- | --- | --- | --- |
| Lean | 15 | 147 | 2.50x | spine | 20 mm | 0 |
| Rise | 43 | 91 | 0.89x | shoulder | 17 mm | 0 |
| Shift | 3 | 89 | 0.75x | shoulder | 17 mm | +77 mm |
| Step: lift, swing, plant | 21 | 94 | 0.72x | shoulder | 18 mm | -28 mm at worst |
| Up | 12 | 195 | 1.09x | shoulder | **53 mm** | 0 |

What it shows:
1. **The robot cannot lift another with its hands.** Its wrist (a W-mini, 0.66 N m continuous) and elbow (a W-mid,
   3.8 N m) are rated an order of magnitude below its legs and spine (29 to 43 N m). This is the same finding as on
   the receiving side (docs/32, docs/46), from the other end.
2. **Cradling moves the load to the shoulder, spine and legs, and most of the get-up fits.**
   - The rise, the shift and the step are within every rating.
   - The helper's CoP leaves its feet (less the 2 cm margin) in only 3 samples, by at most 2.8 cm. Its feet do not
     slip (at most 31% of friction).
3. **Two parts do not fit:**
   - **The catch at the start of the lean.** 147 N, to a robot still kneeling, means bending down: the spine at
     2.5x, and still 1.8x at half the force.
   - **The stand-up.** The grip points end 5 cm beyond where the helper can reach, and peak at 195 N.
   - Both point at what the scripted helper does that a robot should not: catch the full kneel from above, and hold
     on to the end.

## Not yet

- **Statics only.** Nothing is simulated with two bodies.
  - The legs' equal share, the warm-started search, and the CoP margin of 2 cm are approximations.
  - A dynamic helper must also follow the other robot's motion and keep its own balance through it.
- **The helper's elbow contact is a point at the joint.** A real cradle is a forearm under an upper arm.
- **The faded get-up (`--set faded`) is not checked.**
- **The hub task `nurturer-role-flip` waits on `training-round-2`.** This is a first pass outside it.

## Next

- **Two robots in one simulation:**
  - the second robot as a prefixed copy of the model (MuJoCo's spec attach), and `sim_tendon.Rig` taking a prefix;
  - its posture from this search as the first script, its contact a connect at the elbow sites;
  - start at the rise, with the scripted helper doing the catch.
- **Shape the get-up for a robot helper:**
  - take the robot's own weight off the catch: the robot leans onto its own hands first, or the helper kneels;
  - let go before the stand-up's last 5 cm.
