# 22 - Drum locks only when the robot is still (P-036, D-038)

The drums self-lock so that a still robot holds its pose at zero power (D-015). `19` found that inside the
balance loop the locks engage and release over and over and rock the body foot to foot (D-034), and turned
them off while the robot steps. This doc applies that finding to everything else (hub task
`locks-off-while-balancing`). The weight shift, the one-foot hold and push recovery all balance actively, and
all three were worse with the locks.

## The policy (`tools/sim_tendon.py`)

- `BALANCE_LOCKS = False`: no locks while the weight shift and the one-foot hold run (the step already had
  `STEP_LOCKS = False`).
- `LOCK_STILL = 0.5`: a drum may lock only once the body's CoM has moved slower than `LOCK_STILL_V` (20 mm/s)
  for 0.5 s, and every lock releases the moment it moves faster than `LOCK_RELEASE_V` (80 mm/s; a 10 N push
  starts it moving at about 100 mm/s). Between the two speeds the timer holds, so a slow sway does not keep
  the robot unlocked. The per-drum rule (on target and still) still decides which drums lock.

## 200 random pushes (10 to 30 N, one to three each, random directions; the teacher's push set)

| Locks | Stand |
| --- | --- |
| as they were (the per-drum rule alone) | 184 of 200 (docs/17 had 177 before D-032) |
| none at all while standing | 199 of 200 |
| the gate on the legs only | 197 of 200 |
| the gate on everything but the arms | 198 of 200 |
| the gate on every drum, 10 mm/s, no hysteresis | 199 of 200 |
| **the gate on every drum, 20 mm/s, release at 80 mm/s (default)** | **199 of 200** |

## The gauntlet (20 kg model)

| Scene | Before: energy, flags | Now: energy, flags |
| --- | --- | --- |
| stand 4 s | 20.8 J | 12.0 J |
| push 10 / 20 / 30 N | 33 / 34 / 39 J; neck above rating at 30 N | 19 / 24 / 26 J; no flags |
| push 40 N | 78 J; neck pitch above rating, SF 3.7 | 42 J; neck pitch above rating |
| sustained 15 N for 5 s | 109 J; neck above rating | 81 J; no flags |
| arm forward, 1 kg | 114 J | **162 J** (below) |
| quarter squat | 83 J | 33 J |
| weight shift | 80 J, load ripple 25 N | 43 J, ripple 7 N (locks off) |
| one-foot hold, 10 s | 811 J, 41 W, worst leg line 237 N | 562 J, 26 W, worst leg line 146 N |
| step | 471 J, no flags | 471 J, no flags |

Every scene still stands; the hand and wrist flags in the arm, shift and one-foot scenes are as before.

The arm scene is the exception. With the arm forward and 1 kg in the hand, the balance loop on unlocked legs
sways the body at 30 to 60 mm/s and never settles; with the legs locked the statue simply holds. Without the
release hysteresis it cost 230 J; with it, 162 J. The sway is a balance-loop limit cycle under a forward load,
which the locks used to hide.

## Teacher data

`tools/record_teacher.py --pushes 200` re-recorded with this policy and the step of `19`/D-037 (the scene list
is the gauntlet's, so the step is in it): 65 787 steps at 50 Hz in 210 episodes, every gauntlet scene
(the 40 N push now stands and is in) and 199 of the 200 random pushes (one falls and is left out). The step
grid of `19` still stands in 42 of 42 variants on this policy.

## Not yet

- The balance loop's sway under a forward arm load (above): a limit cycle to find as `19` found the locks'.
- The gate uses the CoM speed from kinematics and the IMU; on the real robot it needs the same filtering.

## Next

The gait cycle (`gait-cycle-static`) on the 42-variant grid; the learned controller cloned from the new
teacher data in the sibling project (issue #48).
