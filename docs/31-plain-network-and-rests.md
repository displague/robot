---
title: "31 - Five seeds: a plain network beats the sibling's core; rests and locks; held loads (P-052, D-052)"
summary: "Five seeds: a plain network beats the sibling's core on this robot; the drums lock under a policy; training with a held load makes policies rest (best 10.6 W idle against 5.4 W) (D-052)"
section: "Learned control"
---

# 31 - Five seeds: a plain network beats the sibling's core; rests and locks; held loads (P-052, D-052)

`30` set the order: first settle whether the sibling's model core helps this robot at all (sibling #51, with a
plain-network arm), then train the learned stand policy to rest and lock its drums (sibling #56). Both ran today,
on the sibling's `rl/` trainer, gated on the tendon model.

Short answers:
- **A plain network beats the sibling's core on this robot, in every arm, over five seeds.** Trained the same way,
  with cloning from the hand-written controller then PPO, it passes 22 of 30 named scenes and 98 of 100 random
  pushes; the grown champion core 17 and 89. Without cloning the core never stands on the tendon model; the plain
  network stands in 18 of 30 scenes, as well as the cloned core.
- **The drums can now lock under a policy.** The tendon model's stillness gate ran only inside the balance loop
  (`still_gate`, below); with it on, the best policies lock 50 to 75 % of drum-steps standing.
- **Training a fast-model stand-in to rest did not transfer.** Its exploration noise keeps every drum unlocked, and
  training with less noise produced policies that fall on the tendon model in every scene.
- **Training with a load in one hand did most for rest.** Median idle power 175 W without, 38 W with; the bag held
  by 3 of 5 seeds, against 1 of 5. The best seed idles at 10.6 W (the hand-written controller 5.4 W) and holds
  3.9 kg at 20 W.
- **No learned policy passes the whole gate yet**: every one loses the 8 m/s wind, and none rests as cheaply as
  the hand-written controller. That controller stays.

## The seed study (sibling #51)

Stand task, 150 PPO iterations, wind 0 to 9 m/s on the variants, seeds 0 to 4. The plain network is a two-layer
ELU MLP of 168 units, 51,440 parameters (the grown core has 51,484), feed-forward; the core keeps its recurrent
memory. Gate: the tendon model, drums allowed to lock (below), the six named scenes and 20 random pushes.

| Arm | Named scenes, per seed | Sum of 30 | Random pushes, sum of 100 | Standing power, median (range) | Stand-in standing |
| --- | --- | --- | --- | --- | --- |
| Plain network + cloning | 4 4 5 5 4 | **22** | **98** | 119 W (51-285) | 0.92 |
| Grown champion core + cloning | 4 2 4 4 3 | 17 | 89 | 88 W (67-120) | 0.86 |
| Random-weights core + cloning | 3 4 2 4 0 | 13 | 61 | 51 W (27-273) | 0.86 |
| Plain network, no cloning | 3 4 4 4 3 | 18 | 89 | 420 W (403-531) | 0.90 |
| Grown champion core, no cloning | 0 0 0 0 0 | 0 | 0 | | 0.44 |
| Random-weights core, no cloning | 0 0 0 0 0 | 0 | 0 | | 0.51 |

- The grown champion beats random weights of its own architecture (with cloning 17 against 13; the random core
  collapsed on one seed), so the champion's prior does something; but a plain network beats both.
- Cloning is still the large effect for the core. The plain network clones the hand-written controller about 170
  times more closely (action error 0.00015 against 0.027), so with cloning the gap is partly clone quality.
- A reviewer agent's second opinion: the core has memory and the plain network has none, and the trainer's
  settings were tuned on the core. So a plain recurrent network was added too: a GRU of 80 units (49,872
  parameters), cloned then PPO, five seeds: 4 of 6 named scenes on every seed (20 of 30), 100 of 100 pushes,
  standing power median 154 W, clone error 0.00012. Memory is not what the core adds; with or without it, a plain
  network does at least as well.

RPO transfer (a second MuJoCo humanoid), standing fraction over five seeds:

| RPO start | First 10 iterations | Last 10 iterations |
| --- | --- | --- |
| Random weights | 0.32 (0.29-0.34) | 0.71 (0.70-0.72) |
| From the tendon policy (v0.3.0) | 0.80 on every seed | 0.76-0.78 on four seeds, 0.17 on one |

Starting from another body's policy is a large, reliable head start, and a small gain at the end on four seeds in
five.

## Rests and locks

### The gate

`tools/sim_tendon.py` gains `Rig.still_gate`: with the balance loop off, D-038's stillness gate still runs on the
CoM speed, so the drums can lock under any controller, as the firmware's would. The sibling's gate sets it, and
now reports electrical power and the locked fraction per scene, and two rest scenes, each over its last 20 s:

| | Idle 30 s | Holding 3.9 kg in the left hand, 20 s |
| --- | --- | --- |
| Hand-written controller | 5.4 W, 86 % locked | 9.2 W, 71 % locked |
| Pass line | 7.2 W | 21.6 W |

With locks allowed, the policies already on hand lock 0 to 75 % of drum-steps standing. Their power is spread
over every joint (2 to 7 W each, arms included): each unlock and relock as the target moves costs current.

### The stand-in

`tools/export_standin.py` now exports, per DOF, the motor voltage, stall current, gear ratio and the rotor and gear
inertia in joint terms (`rotor_j_joint`), the lock rule in joint terms (`locks`), and a `com_vel` sensor (the whole
body's CoM velocity). The sibling's `StandinEnv(locks=True, power_cost=c)` counts power as the tendon model does,
including the torque that accelerates the rotor, and locks a drum under the tendon model's rule.

It reads 42 W where the tendon model reads 99 W for the same policy: the right order, low by about 2.3 times. A
held pose settles and locks every drum after about 6 s (the passive stand-in sways first).

### Training it to rest: what did not work

- **Exploration noise keeps the drums unlocked.** PPO's action noise (std 0.135 of the action range, 4 to 8 degrees
  per tick) shakes the rotors: 2.2 kW in training against 50 W for the same policy without noise, and no drum
  ever locks. A power cost then mostly teaches the policy to shrink its noise.
- **Less noise does not transfer.** At a twelfth of the noise the drums lock in training, but every such policy
  falls on the tendon model in every scene, with or without the power cost; ablations (one change at a time) put it
  on the noise alone. Half the environments at a fifth of the noise: also worse (2 of 6 scenes).
- **Fine-tuning on the tendon model itself** (`rl/tendon_env.py`, 144 to 160 rigs on 18 to 20 processes, 600 to
  800 control steps per second here): 150 iterations from the best plain-network policy lowered training power
  from 350 to 165 W, but its noise-free power stayed at 51 W, and it lost the 5 m/s wind. 400 iterations (2.8
  million steps, 65 minutes) from the best held-load policy: standing power 36 to 20 W with the same scenes and
  pushes, but idle 10.6 to 20 W and the bag 20.4 to 23.6 W. Slow, and not yet better overall.
- **A deadband on the targets (rest mode)**: a joint's target held within 2 degrees once the CoM has been under
  0.05 m/s for 0.5 s. It keeps every push and wind result and lowers standing power on four of five plain-network
  seeds (285 to 200, 51 to 36, 119 to 87, 112 to 95 W), but the idle and bag scenes move both ways. Not adopted.

### Scenes that match why the robot rests

The single-change runs pointed at the held load. Plain network with cloning, 0 to 4 kg added to one hand on each
model variant, five seeds, against the same without:

| Training | Seeds | Idle, median (range) | Bag held (of 5) | Best seed: stand, idle, bag |
| --- | --- | --- | --- | --- |
| No load | 5 | 175 W (21-338), one falls | 1 | 51 W, 21 W, falls |
| 0 to 4 kg in one hand | 5 | 38 W (11-215) | 3 | 36 W, **10.6 W**, **20.4 W** |
| The same + half the variants calm and pushed every 8 to 16 s, 20 s episodes | 5 | 86 W (33-130) | **5** | 36 W, 33 W, 35 W |

The best held-load seed passes 5 of 6 named scenes and 20 of 20 pushes, idles at 10.6 W with 80 % of drum-steps
locked, and holds the bag at 20 W, inside its pass line; its idle is twice the hand-written controller's. A held
load is where a still pose pays most, and a policy that has learned to hold one still holds still empty-handed too.
Adding quiet scenes makes every seed hold the bag (5 of 5), with 20 of 20 pushes each, but idles higher.

## What it means

- For this robot the learned stand policy should be a plain network, trained with the sibling's `rl/` trainer
  (`--arch mlp`). The sibling's core and its pretraining do not help here (D-052).
- Rest has to be trained where it happens: under loads, in long quiet scenes, on a model whose drums lock. The fast
  stand-in now has the locks and the power, but its noise problem means the lock behaviour itself is learned best
  on the tendon model, and that is slow.
- The 8 m/s wind is now the gate's hardest scene for every learned policy.

## Not yet

- A policy within the hand-written controller's power. The best is twice it idle.
- The 8 m/s wind for any learned policy.
- The plain network on RPO and on other bodies (sibling #57 now asks this with the plain network).
- Bag scenes in the tendon-model fine-tune; the stand-in's power is 2.3 times low.

## Next

Seeds of the held-load + quiet recipe, a longer tendon-model fine-tune with a bag on some rigs, and the 8 m/s wind
in training at full strength.
