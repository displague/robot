---
title: "17 - Learned control: the fast stand-in, the sibling's model core, and the algorithmic controller as teacher (P-021)"
summary: "Learned control: the fast stand-in, the sibling's model core as a PPO actor, the hand-written controller as teacher, the first ladder (cloning: 90 % standing under pushes), the failing gate on the tendon model, why get-up needs a kneeling curriculum (D-031)"
section: "Learned control"
---

# 17 - Learned control: the fast stand-in, the sibling's model core, and the algorithmic controller as teacher (P-021)

Everything up to `16` is hand-written control. This doc records the first learned controller: a recurrent
policy trained with PPO on a fast stand-in of this robot, judged on the tendon model, in the sibling project
[ai-embedded-dynamic-diversity](https://github.com/displague/ai-embedded-dynamic-diversity) (branch
`feat/standin-rl`, `docs/rl-standin.md` there). The policy network is that project's model core, grown from
its strongest checkpoint to this robot's channel counts, so the question of whether its staged evolution
produced anything reusable gets its first test on a body with contact physics.

## The stand-in (`tools/export_standin.py`)

The tendon model of `13` carries every drive train element and runs well under real time; PPO needs hundreds
of millions of steps. The stand-in is the same skeleton, masses, joint ranges and printed-envelope collision
shapes (`cad/skeleton.json` through `export_urdf.expand`), with each DOF's drive train replaced by one MuJoCo
position servo that keeps what a policy can feel of it:

| Servo parameter | From the drive train |
| --- | --- |
| stiffness kp | the line loop about its target with both runs taut, 2 k (n r_p)^2 (`Rig.torque_to_angle`): hip roll 801, hip pitch 534, ankle 278, knee 222 N m/rad |
| damping kv | the motor's back-EMF slope reflected to the joint, T_stall n_r^2 / w_noload, plus the line damping; it also caps the joint speed near the no-load figure (2.6 to 5.9 rad/s on the W-max joints) |
| torque limit | the clutch slip torque at the joint (30 to 53 N m on the W-max joints): above it the clutch slips, so no drive can push harder |

`sim/robot_standin.json` carries, per DOF, what a training loop needs for the same damage gates the tendon
model reports: the moment arm, the pretension, the linear torque range, the continuous line rating and the
break strength, so the line force for a joint torque is pretension + |tau| / (2 r) while both runs are taut,
else |tau| / r. Sensors: the pelvis IMU, a touch sensor per sole, impact sensors around the head, trunk and
pelvis shells, the torque of every servo and the force of every joint stop. The control order is the tendon
model's (link order), so one policy's outputs mean the same joints on both.

What it leaves out, and the training randomises instead: the slack line run (stiffness down to half), the
self-lock (holding at zero power, irrelevant to a policy), the clutch slip itself (a torque cap here), the
reflected motor inertia behind the line spring (a small joint armature here), and the line's hysteresis.

Standing in the standing pose on its own servos, it settles at a pelvis height of 0.922 m with 100 N on each
foot. On the 24 cores of this machine, MuJoCo's multithreaded rollout runs it at about 190,000 control steps
per second (50 Hz control, four 5 ms physics steps each).

## The algorithmic controller as a teacher (`tools/record_teacher.py`)

The controller of `14` to `16` (capture-point balance, weight shift, one-foot hold, the static step) runs on
the tendon model; `Rig.applied` now records, per DOF, the target `control()` applied each step, balance
terms included. The recorder runs the gauntlet scenarios plus pushes of 10 to 30 N from random directions and
writes, at 50 Hz, the stand-in's observation and those targets as radian offsets from the standing pose.

| Recording | Value |
| --- | --- |
| Episodes kept | 187 (the 40 N push and 23 of the 200 random pushes fall and are left out) |
| Steps | 56,903 (about 19 minutes of demonstrations) |
| Wall time | 59 s on 20 processes |

The 23 falls in 200 random pushes are a finding of their own: the hand-written controller loses about one
sideways push in nine at 10 to 30 N, which is a gap a learned policy can close.

## The first ladder (one seed, 2048 environments)

The policy is the sibling's champion core (v09, 46,876 parameters, 48 inputs, 16 outputs) grown to this
robot's 104 observations and 32 actions (51,484 parameters), or the same architecture from random weights.
PPO with the rsl_rl recipe; stand runs 150 iterations (7.4 million steps, about 5 minutes each), get-up runs
500 (24.6 million steps, about 11 minutes each).

**Stand, with a push of 0.5 m/s every 2 to 4 s** (standing = pelvis above 0.85 m, upright):

| Start | Standing, last 10 iterations | Mean episode, last 10 | Note |
| --- | --- | --- | --- |
| random weights | 45 % | 1.5 s | reached 96 % at iteration 37, then lost it |
| grown champion | 63 % | 1.9 s | |
| grown champion, cloned from the teacher first | **90 %** | **6.0 s** | standing from the first iteration; cloning reached a 0.024 action MSE in 30 epochs |

Cloning from the algorithmic controller is the large effect: the policy starts out standing and PPO keeps it
there under pushes. The grown champion ends ahead of random weights, but on one seed and with a random run
that found and lost a good policy, so that difference is suggestive, not established.

**Get-up from random fallen poses:** neither start learned it in 24.6 million steps (standing 0 % in both).
A scripted probe on the stand-in shows why it is hard here: flat-footed squats deeper than 0.6 rad of knee
flexion tip backward before any rise starts, at under half the knee's torque limit, because the ankle
dorsiflexes only 20 degrees (`03`). A person with that ankle cannot squat flat-footed either; they get up
by kneeling, then a half-kneel lunge. The get-up task needs that as a curriculum (start kneeling, then lower)
rather than a drop to the floor and a reward for height.

## The gate: the cloned policy on the tendon model

`rl/tendon_eval.py` (sibling) runs the policy on the full tendon model, its targets replacing the
hand-written controller's, the balance loop off:

| Scene | Learned policy | Hand-written controller |
| --- | --- | --- |
| stand 10 s | **falls** (a slow lean, 40 degrees of tilt at 10 s) | stands |
| push 10 N | stands | stands |
| push 20 N, 30 N | **fall** | stand |
| 20 random pushes of 10 to 30 N (one each, 4 s) | 18 stand | 20 stand |

The policy that stands 90 % of the time on the stand-in does not yet stand on the tendon model. The gap is
what the stand-in leaves out, and the likely order is: the drum locks (a still joint locks and its target
stops moving it until the gear has drifted), the ankle in torque mode (the teacher's ankle targets are
line-stretch offsets that carry a torque, which the stand-in's servo reproduces only while both runs are
taut), the slack run, and the 1 ms physics against 5 ms. Closing it is the next work: model the lock and
the slack in the stand-in, or fine-tune on the tendon model itself at its real-time rate.

## One brain, two bodies, and what it forgets

The sibling's goal is a controller that moves from one robot to another and adapts. Roboparty's RPO
(roboto_origin, CERN-OHL-W-2.0 description: 34 kg, 23 quasi-direct-drive joints) runs in the same
environment, its per-joint channels padded to this robot's 32 so one network fits both:

| RPO stand task (pushes as above) | Standing, iterations 1 to 10 | Standing, last 10 | Mean episode, last 10 |
| --- | --- | --- | --- |
| from scratch | 27 % | 72 % | 1.1 s |
| started from this robot's cloned stand policy | **79 %** | **77 %** | **2.0 s** |

The policy trained on the 20 kg tendon robot balances the 34 kg direct-drive robot from its first
iterations and stays ahead of training from scratch. One seed again.

What the core gives up for it: cut back to the champion's 48 inputs and 16 outputs and scored on the
sibling's own cross-evaluation (hexapod, car, drone, polymorph120; mild, gust and force scenarios):

| Core | Cross-eval transfer | Mismatch | Car |
| --- | --- | --- | --- |
| champion v09, untouched | 0.567 | 0.20 | 0.49 |
| after get-up training (learned almost nothing) | 0.569 | 0.21 | 0.49 |
| after stand training | 0.543 | 0.25 | 0.45 |
| after cloning and stand training (the best on the body) | **0.435** | **0.74** | **0.28** |

Learning the body overwrites the old skills in proportion to how much it learns. A core meant to carry
across bodies has to keep training on all of them (rehearsal of the old tasks, or a distillation term
toward the champion on them) and has to be scored on the old benchmark as a regression gate after every
body run, which the sibling's promotion discipline already demands.

## Released

The sibling released this work as v0.3.0 (github.com/displague/ai-embedded-dynamic-diversity, release
`v0.3.0`: the stand policy, the RPO policy, every training log and the forgetting cross-eval attached).
The next steps below are its issues #48 (stand-in gap), #49 (get-up), #50 (rehearsal) and #51 (seeds).

## Next

1. Close the stand-in to tendon gap (locks and slack in the stand-in; then a short fine-tune on the tendon
   model).
2. Get-up as a curriculum from kneeling.
3. A second seed for every comparison before reading anything into the champion-versus-random difference.

## Since (P-050)

`29` retrains the stand policy on today's teacher data with the stand-in's drive refitted to the tendon model, on
three seeds. Every policy now stands 10 s on the tendon model, and the best take 10 to 30 N pushes; none yet
passes the whole gate, and they spend 50 to 130 W standing still against the hand-written controller's 3.6 W.
