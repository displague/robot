---
title: "43 - The next training rounds: senses, loose control, the nurturing helper, then a crew (P-067, D-069)"
summary: "The plan the owner queued: four training rounds that bring in the senses of docs/42, relaxed control, and a give-and-take helper faded to withdrawal, then the role flip, shelter tests and robots helping each other up. Overloading a line through the helper fails the episode, and withdrawal needs both a small helper impulse and the old gates passing (D-069)"
section: "Learned control"
---

# 43 - The next training rounds: senses, loose control, the nurturing helper, then a crew (P-067, D-069)

Four proposals arrived on the Agent Hub on 2026-11-10. Three were the owner's ideas, recorded by Antigravity:
- `records/rust-high-frequency-20261110`: Rust for the high-frequency work;
- `records/nurturing-instructor-20261110`: a crutch-like helper with Wing Chun give and take;
- `records/robot-as-nurturer-multi-agent-20261110`: the robot becomes the helper; more robots; a shelter.

The fourth, `records/grok-bot-robot-nurturer-multi-agent-20261110`, was Grok's plan staging them. The owner said to queue
all of them and bake them into the next rounds of training (P-067). This doc is that plan, with each piece's hub task.

## The pieces, in order

| Round | What it trains or builds | Hub task | Depends on |
| --- | --- | --- | --- |
| 0 | Teacher demos re-recorded on the v0.6.0 defaults, plus chain demos | `teacher-demos-regen-v060` | - |
| 0 | The helper v0 on the 0.35 m chair rise, with the hand-written controller | `helper-instructor-chair` | - |
| 0 | A head IMU (docs/42 item 1) | `head-imu-vestibular-channel` | - |
| 0 | Relaxed control (docs/42 item 5) | `relaxed-control-variability` | - |
| 0, parallel | A Rust spike: the 1 kHz balance loop as ESP32 firmware | `rust-balance-loop-esp32` | - |
| 1 | Chained skills; felt weight and the head IMU as ablated channels; helper phases A-B; stiff against relaxed teacher | `training-round-1` | the demos, the helper v0 |
| 1 | The helper on getting up from a fall | `helper-instructor-getup` | the helper v0 |
| 2 | Helper phases C-D (withdrawal); stereo depth and binaural contacts as ablated channels | `training-round-2` | round 1, `perception-channels`, the get-up helper |
| 3 | The role flip: a trained robot is the helper | `nurturer-role-flip` | round 2 |
| 3 | Shelter: stack, lean, cover | `shelter-stack-lean-cover` | round 2 |
| 4 | Three and four robots, helping a fallen one up | `multi-robot-mutual-support` | the flip, `tendon-physics-on-gpu` |

Existing tasks that feed the rounds:
- `mixed-skill-training-gate` (cross-eval so old skills stay);
- `perception-channels` (the channel families);
- `tendon-physics-on-gpu`;
- `hands-pillow-and-pour` (fingers, for the shelter's grasping);
- `standin-tendon-gap`.

## The helper (the nurturing instructor)

**v0 is scripted, not learned:** two mocap "hands" the robot grips through the item grip equalities it already has
(docs/38).
- **Give and take:** each hand follows an admittance law, moving in proportion to the force on it. A firm setting
  barely moves; a yielding one dips under load and comes back. That one knob is the Wing Chun give and take.
- **Reaching:** the helper offers a hand when the robot's hand reaches out.
- **What it logs:**
  - its force impulse;
  - the work it does against the robot's;
  - every line's peak against its rating;
  - slip, energy and falls.

**The first target is the 0.35 m chair rise.** The robot sits on it but cannot stand (docs/33), and the hand-written
controller runs the rest of that scene. So the first test needs no learning at all:
- does the rise succeed with the helper?
- how much does the helper carry?
- how far can it be faded before the rise fails?

Getting up from a fall comes second. It has only been tried by learned policies on the stand-in (docs/32), whose arm
lines ran at 2.3 times their rating.

**Fading:**

| Phase | The helper | Passes when |
| --- | --- | --- |
| A | Always there, nearly perfect, both hands | The task completes; this phase's impulse is the baseline |
| B | Offered only when the robot reaches, and yields more | The task completes with clearly less impulse than A |
| C | One hand, or lopsided and "drunken" | Success and impulse keep improving |
| D | Withdrawn | The unassisted gates pass (below) |

**The reward pays for the robot's own work:** task success, minus a cost on the helper's impulse, minus the usual
line-force, slip and energy costs. Survival stays as a constraint. The helper never gets a "keep it up at all costs"
objective.

## The two laws (D-069)

The owner queued the plan without ruling on Grok's two questions, so these are the defaults. They are open to the
owner's override.
1. **Overload fails the episode.** A line driven past its rating through contact with the helper fails that episode,
   whatever else happens. Helping by overloading the arms (as the stand-in's get-up did) is not help.
2. **Withdrawal needs both conditions:**
   - the helper's impulse is under 10% of the Phase A baseline;
   - the unassisted gates pass at their pre-helper rate or better: the chair grid, the tendon stand gate, the pushes,
     and the wind scenes once in scope.

   A policy that only stands while held is not trained.

## Senses in the rounds (docs/42)

| Round | Channels |
| --- | --- |
| 1 | Felt weight, and a head IMU next to the pelvis one. The helper is felt through contact and the hand lines; no camera is needed yet |
| 2 | Head stereo depth (a wide, coarse periphery and a small fovea), and binaural bands of the robot's own contacts. In Phase B the robot must find the helper by sight |
| 3-4 | The same, to find a fallen robot or scattered supplies; colour only for telling objects apart, at the planner |

Each channel is named and ablated: trained with and without it, compared on the chain grid and the gates. No sense
enters a policy silently.

## Loose control in the rounds

Round 1 trains from two teachers and compares them:
- the stiff hand-written controller, as it is;
- a relaxed variant (`relaxed-control-variability`) that lets task-irrelevant motion drift.

The helper's Phase C is itself loose (lopsided, "drunken"). The test is the same as docs/42's: less energy, more
natural stride-to-stride variation, and no more falls.

## Rust

**The simulation stays in Python.** Its heavy work is MuJoCo's C code. For running many simulations, the GPU
(`tendon-physics-on-gpu`) is the bigger lever than porting the controller.

**The loop that needs to be fast is on the robot:** the leg boards' 1 kHz balance loop, with a 5 ms budget from IMU to
motor command (D-044). The spike writes that loop in Rust for the ESP32-S3 and tests it two ways:
- **against the simulation:** compiled on the host, scene for scene against the Python loop;
- **on a dev board:** for timing only, and only with the owner's go-ahead, since that is hardware.

It runs in parallel and gates nothing.

## Shelter and the crew

These come only after Phase D:
- **Shelter:** stack, lean and cover, one known object first, then scattered supplies. Scored on construction
  progress and on whether the structure persists after a nudge, with survival as a constraint.
- **The role flip:** a policy that passed Phase D drives a second body as the helper.
- **The crew:** three and four robots, one knocked down and the others helping it up. This needs GPU physics, since
  each body is 32 winches.

## Round 0 so far (P-068)

- **The helper on the chair** (`helper-instructor-chair`, done): docs/44.
  - With the helper the 0.35 m rise stands, on the hand-written controller.
  - Firm, it overloads the elbows; yielding (give 0.25 to 0.5), it passes the overload law.
  - The withdrawal baseline is 107 N s (D-070), so withdrawn means under about 11 N s.
- **The teacher demos** (`teacher-demos-regen-v060`, done), re-recorded on the v0.6.0 defaults into
  `sim/out/teacher/` (gitignored). Every file adds `felt`: the carried weight over the robot's weight, as the grip's
  force (truth) and as the balance loop's estimate (D-063). `obs` keeps the stand-in's 72 channels.

| File | Command | Episodes kept | Steps (50 Hz) |
| --- | --- | --- | --- |
| `teacher.npz` | `record_teacher.py --pushes 200 --winds 24` | 235 of 237 (2 random pushes fall) | 85 068 |
| `teacher_chair.npz` | `record_teacher.py --set chair` | 6 of 8 (the 0.35 m seats fall) | 6 781 |
| `teacher_chain.npz` | `record_teacher.py --set chain --chains 24` | 17 of 24 chains | 153 898 |

The chair and chain offsets need action spans of 0.70 rad on average, against 0.27 for the standing set (the sibling's
`teacher_action_half`), so they are separate files. The sibling's `load_teacher` reads all three. The relaxed-control
channel waits for `relaxed-control-variability`.

## Round 1 so far (P-069)

- **The drunken sailor** (docs/45): a 4 N wander is survivable with a yielding helper; 8 N fails the rise through wrist
  slip.
- **The rise trained on the full model** (docs/45, D-071): a parameter search, with the helper as curriculum, and a
  control without it.
  - The 0.35 m seat stands with no helper.
  - The control found it sooner than the curriculum did.
  - The trained sets are marginal and are not the defaults.

## Not yet

- No network is trained: round 1's first slice trained the hand-written rise's parameters (docs/45). The helper is scripted and works with the hand-written controller only.
- The helper is not in the sibling's stand-in.
- The rounds pick one mass model, the 20 kg masses, until the W-max choice lands (D-061).
- The head IMU, relaxed control and the Rust spike (round 0's other tasks) have not started.

## Next

- `helper-instructor-getup`: the helper on getting up from the floor.
- The helper in the sibling's stand-in, so `training-round-1` can learn the 0.35 m rise and fade it.
- `head-imu-vestibular-channel` and `relaxed-control-variability`, before round 1 trains.
