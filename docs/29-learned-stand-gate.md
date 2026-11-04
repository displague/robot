# 29 - The learned stand policy against the tendon model: the drive refit, retraining, three seeds (P-050, D-050)

`17` ended with a gap: the cloned and PPO-trained stand policy (sibling release v0.3.0) stood 90 % of the time on
the fast stand-in, but on the full tendon model it fell within 10 s standing still. This doc retrains it as `17`
did (the sibling's champion core grown to this robot, cloned from the hand-written controller, then PPO with
pushes) and gates it on the tendon model again. Hub task `standin-tendon-gap`, sibling issue #48.

Short answers:
- **The v0.3.0 policy falls in every gate scene on today's model.** The hand-written controller passes them all.
- **Retrained on today's teacher data, every policy stands 10 s still**, and the best ones take 10 to 30 N pushes
  and a 5 m/s wind. None passes the whole gate yet: 30 N and the 8 m/s wind still fail more often than not.
- **Most of the gain is the teacher data, not the stand-in.** The demonstrations were re-recorded since v0.3.0
  with the drum locks off while balancing (D-038) and with wind episodes. The stand-in's drive was refitted to
  the tendon model here and matches it much better, but over three seeds that changed the gate by about as much
  as the seed did.
- **The learned policies use 50 to 130 W to stand still; the hand-written controller uses 3.6 W.** They never let
  a drum rest. That's the next gate to add.

## What the stand-in had wrong

The task assumed the drum locks were the gap. They are not, in this gate: `rl/tendon_eval.py` runs the tendon
model with its balance loop off, and since D-038 the locks only engage after the whole-body stillness gate,
which that loop drives. So no lock engaged under a policy.

What did differ is how a joint follows its target. A step of 0.03 to 0.05 rad on one joint's target, robot
standing, fraction of the step reached (`probe.py` and `fit.py` in the session scratchpad; the tendon model with
the balance loop off):

| Joint, step +0.03 rad | at 0.1 s | at 0.4 s | at 1 s |
| --- | --- | --- | --- |
| Knee, tendon model | 0.69 | 1.34 | 0.91 |
| Knee, old stand-in | 0.09 | 0.14 | 0.16 |
| Knee, refitted stand-in | 0.67 | 1.22 | 0.50 |
| Hip pitch, tendon model | 0.82 | 0.91 | 0.80 |
| Hip pitch, old stand-in | 0.42 | 0.62 | 0.67 |
| Hip pitch, refitted stand-in | 0.75 | 0.86 | 0.72 |
| Shoulder pitch, tendon model | 1.30 | 1.12 | 1.09 |
| Shoulder pitch, old stand-in | 0.61 | 0.98 | 0.98 |
| Shoulder pitch, refitted stand-in | 1.26 | 1.06 | 0.96 |

The old stand-in was slow and soft. Its damping was the motor's full back-EMF slope, applied at the joint, but
in the real drive that damping acts on the gear, behind the line spring. It also had no integral action, while
the firmware's outer loop (`sim_tendon` KQ 0.5, KI 2/s, trim to 10 degrees) trims out the line stretch, so a
loaded knee on the tendon model reaches its target and on the old stand-in sagged.

The refit (`tools/export_standin.py`, D-050):
- damping at 0.1 of the back-EMF figure (`KV_SCALE`)
- a 30 ms first-order lag on the servo's target (`TIMECONST`, the gear's travel), randomised 0.7 to 1.5 times
  in training
- the outer loop itself, in the JSON as `outer_loop` (KQ, KI, trim limit and the per-joint gain, 0.15 at the
  ankles), run by the sibling's training environment at the 50 Hz control rate
- the servo's control range widened by 0.5 rad so the trim can act at the joint limits

The parameters were picked by a grid search over the step responses of six joints in both directions: summed
squared error 0.84, against 2.41 for the best plain servo (no outer loop) and more for the old one.

## The gate, three seeds

Each run: champion v09 grown to 104 inputs and 32 outputs, cloned for 30 epochs from the teacher data
(`sim/out/teacher/teacher.npz`, 272 episodes, 107,243 steps), then 150 PPO iterations (7.4 million steps) with
pushes, and winds of 0 to 9 m/s with 0 to 2 m/s gusts on most model variants, as in the teacher data. About 12
minutes each on the RTX 5080 and 24 cores. Gated on the tendon model (20 kg build): stand 10 s, a 10, 20 and 30 N
push from the side, a 5 and 8 m/s head wind with 1.5 m/s gusts for 10 s, and 20 random pushes of 10 to 30 N
(each judged 4.5 s after the push; it was 2.5 s, which missed slow falls). S stands, F falls.

| Policy | Stand 10 s | 10 N | 20 N | 30 N | Wind 5 | Wind 8 | Random pushes | Standing still |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Hand-written controller | S | S | S | S | S | S | 20/20 | 3.6 W |
| v0.3.0 policy (`stand-grown-bc`) | F | F | F | F | F | F | 18/20 * | 40 W |
| Refitted stand-in, seed 0 | S | S | S | F | S | F | 18/20 | 67 W |
| Refitted stand-in, seed 1 | S | S | F | F | F | F | 16/20 | 110 W |
| Refitted stand-in, seed 2 | S | S | S | S | F | F | 18/20 | 99 W |
| Old stand-in, seed 0 | S | S | S | S | F | F | 17/20 | 134 W |
| Old stand-in, seed 1 | S | F | F | F | F | F | 17/20 | 117 W |
| Old stand-in, seed 2 | S | S | S | F | F | F | 18/20 | 50 W |

\* judged 2.5 s after the push; it was falling over 10 s in the stand scene.

Totals over three seeds: the refitted stand-in passes 10 of 18 named scenes and 52 of 60 random pushes; the old
stand-in 8 of 18 and 52 of 60. That difference is inside the spread between seeds. The one wind scene any policy
passed was on the refitted stand-in.

No line is over its continuous rating in the scenes the policies stand. The worst line by ratio is the neck pitch
(W-mini, 26 of 37 N standing). In the 8 m/s wind, as the policies fall, it reaches 62 N for 3 s.

On the stand-in itself the old one looks easier: its policies stand 89 to 92 % of the time, against 83 to 87 % on
the refitted one, which is the point of a closer stand-in. Its scores were promising more than the tendon model
gave.

## What it means

The stand policy now stands on the tendon model, which the v0.3.0 one did not, and it takes the pushes the
hand-written controller takes about two times in three. It is not ready to replace the hand-written balance:
- It loses the 30 N push and the 8 m/s wind, which the hand-written controller holds.
- It spends 15 to 35 times the power standing still. The policy keeps every joint target moving by small amounts,
  so the outer loop never settles and no drum rests. The hand-written controller lets the drums lock.

## Not yet

- An energy term in training and a power column in the gate. The stand-in has the joint torques and speeds; a
  motor-current estimate from them would let PPO see the cost of dithering.
- A fine-tune on the tendon model itself (about 140 control steps per second per process, 3,000 on 22 processes:
  1 million steps in 6 minutes), which `17` named as the fallback.
- The policy with the balance loop's ankle torque mode, so the locks can engage under it.
- The forgetting cross-eval of these cores (the sibling's `rehearsal-regression-gate`).

## Next

Add the power estimate to the stand-in's reward and to the gate, retrain on three seeds, and then fine-tune the
best on the tendon model.
