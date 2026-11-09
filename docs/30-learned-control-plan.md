---
title: "30 - Learned control: where the sibling stands in MuJoCo, rests and locks, other bodies, its issues (P-051, D-051)"
summary: "Learned control: how the sibling's model does in MuJoCo, training it to rest and lock the drums, other MuJoCo bodies, the order for its open issues (D-051)"
section: "Learned control"
---

# 30 - Learned control: where the sibling stands in MuJoCo, rests and locks, other bodies, its issues (P-051, D-051)

P-051 asked four things: train the sibling (ai-embedded-dynamic-diversity) to rest and lock the drums, since that
looks like the biggest gap; put other MuJoCo robots through it for transfer; say how the sibling's model is doing
in MuJoCo; and whether its open issues, mostly findings from this repo, need work first. This doc answers from
`17`, `29` and the sibling's `docs/rl-standin.md`, and sets the order. No training was run for it.

## How the sibling's model is doing in MuJoCo

| What | Result | Seeds |
| --- | --- | --- |
| Stand on the fast stand-in, grown champion + cloning + PPO | 83 to 92 % of episodes standing | 3 + 3 (`29`) |
| Stand on the tendon model (the gate) | every policy stands 10 s; best take 30 N; none passes all six scenes | 3 + 3 |
| Power standing still | 50 to 134 W against the hand-written controller's 3.6 W | 6 |
| Get-up from fallen | 0 % (both grown and random weights) | 1 |
| Grown champion against random weights, stand | 63 % against 45 % | 1 |
| RPO (a second MuJoCo humanoid), started from the tendon policy | 79 % against 27 % in the first 10 iterations; 77 % against 72 % at the end | 1 |
| Old-world skills after body training (forgetting) | cross-eval 0.567 falls to 0.435 for the cloned stand core | 1 |

The honest reading:
- **The large effect is cloning the hand-written controller**, not the sibling's core. Most of the gain in `29`
  was the re-recorded teacher data.
- **Whether the sibling's core helps at all is not settled.** Grown against random weights is one seed. There is
  no plain-network baseline (a standard MLP of the same size), so even a seed-robust win over random weights of
  the same architecture would not show the architecture is worth having.
- **Transfer to a second body helped early learning, once.** It did not change where RPO ended.
- **Learning a body erases the old skills** in proportion to what it learns (sibling #50).

## Rests and locks

**Yes, and it is one of the two gaps.** The other is robustness: the 30 N push and the 8 m/s wind. Power is the
gap that is furthest off, 15 to 35 times the hand-written controller.

Why the policies never rest today:
- The stand-in has no lock and no power. Holding a target steady earns nothing, so PPO's dithering costs nothing.
- On the tendon model under a policy, no lock can engage at all. The locks wait for the whole-body stillness gate
  (D-038), and the gate is computed inside the balance loop, which is off when a policy drives. So even a policy
  that held still would read the same power in the gate.

What to build:
1. **The tendon model:** run the stillness gate under a policy (the balance loop off, the gate on), so the locks
   can engage in `tendon_eval`. The gate gains a power column with a pass line, 2 times the hand-written figure
   (7 W standing still).
2. **The stand-in export:** each DOF's motor voltage and stall current, and the lock rule (tolerance, gear rate,
   stillness time). The stand-in already has the joint torque and speed.
3. **The training environment:**
   - the tendon model's power: `V * I_stall * |(tau/T_stall + w/w_noload) * tau/T_stall|` per joint, zero while
     locked;
   - a lock when the joint is on its target and slow, its target has been steady, and the body has been still
     for the stillness time;
   - a power cost in the reward, in watts, so the training number and the gate number are the same.
4. **Scenes that match why the robot rests**, each long enough that resting pays:

| Scene | Real need | What it rewards |
| --- | --- | --- |
| Stand idle 60 s | battery: waiting is most of a day | locking within a few seconds and staying locked |
| Hold a bag or an arm load (`28`) | a static load is where locks save the most | the load held locked, not servoed |
| Steady wind | locked ankles hold a lean (D-038's release on drift) | locking in a steady lean, unlocking as it builds |
| Push, then quiet | recover, then rest again | getting back to locked within a few seconds |

5. **Clone the rests too.** The teacher data was recorded with the locks gated by stillness. If the stand-in shows
   the lock state, the cloned policy can copy when the teacher holds still.

Order: tendon-model gate first (else nothing shows), then the stand-in's power and lock, then retrain on three
seeds. About 1 hour of code and 40 minutes of training.

## Other MuJoCo robots

- **Tried:** Roboparty RPO (23 DOF, motors on the joints, no lines). Trained after the tendon body, not with it;
  one seed.
- **Not tried:** training one network on several bodies at once; any MuJoCo Menagerie body (Unitree G1 and H1,
  Booster T1, Berkeley Humanoid, quadrupeds such as Go2); holding one body out to measure zero-shot transfer.
- The sibling's own embodiments (hexapod, car, drone, polymorph120, humanoid120) live in its voxel world, not in
  MuJoCo physics. Its cross-eval measures transfer there.

For this robot, cross-training is worth it only if it makes the tendon body learn faster or hold up better on
the gate. That needs the seed study (sibling #51) with a plain-network baseline first. Then:
- train the tendon body, RPO and one or two Menagerie humanoids together;
- hold one out, and compare its learning curve with that of a network trained on that body alone.

## The sibling's open issues

| Issue | From | Matters for the robot | Order |
| --- | --- | --- | --- |
| #51 seeds 0-4, grown against random (and a plain network) | `17` | decides whether the sibling's core is worth using | 1 |
| #48 stand-in to tendon gap, and #56 rests and locks | `17`, `29`, this doc | the gate and the power | 2 |
| #50 rehearsal, cross-eval as a regression gate | `17` | only if the old voxel-world skills are kept | 3 |
| #49 get-up through kneeling | `17` | yes, the robot cannot get up yet | 4 |
| #57 multi-body MuJoCo training | this doc | if #51 shows the core helps | 5 |
| #8, #9, #13, #16, #17, #18 | the sibling's voxel world | no | parked |

## Not yet

- None of this is built. Every number above is from `17` and `29`.
- No license check for the Menagerie bodies yet. Most are Apache-2.0 or BSD; each must be checked.

## Next

Sibling #51 with a plain-network arm (5 seeds each: plain network, random weights, grown, grown + cloning;
about 3 hours of training), then rests and locks as above, then the multi-body run if #51 says the core helps.

## Since (P-052)

`31` ran the first two steps. Sibling #51: a plain network beats the core in every arm over five seeds, so D-051's
reverse condition holds (D-052). Rests: the drums can lock under a policy; the stand-in carries the locks and the
power but training it to rest did not transfer; training with a held load made policies rest (best 10.6 W idle).
