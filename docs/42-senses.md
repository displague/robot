---
title: "42 - Senses: sight, hearing, balance, and why people sway (P-066, research)"
summary: "Research topic: the robot senses only its body (joints, lines, a pelvis IMU, foot loads), with no sight or sound. How eyes, ears and the inner ear split and fuse their inputs, why separate receptors evolved, and why people control loosely. A proposal for head stereo depth, binaural hearing, a head IMU, and a relaxed-control test"
section: "Learned control"
---

# 42 - Senses: sight, hearing, balance, and why people sway (P-066, research)

The owner asked (P-066) whether the robot trains with sight, and at what resolution:
- stereo or not;
- black and white, hues or colour.

They also asked:
- whether the brain treats these as separate layers assembled into one "vision", and what led to separate receptors;
- whether binaural audio belongs in the inputs, with sound propagating correctly;
- whether the robot has the equivalent of inner ears;
- whether sitting, standing and carrying need an awareness of balance (depth, the periphery, the inner ear);
- whether people move like drunken sailors or crabs rather than perfect robots, and whether perfect robotic control is
  harder and more straining on a body.

This doc is the research topic's first pass. It is also on the Agent Hub as `records/senses-research-p066`.

## What the robot senses today

**No sight and no sound.** Neither the hand-written controller nor the learned policies get a camera or a
microphone (docs/38, "What the policy can sense"). Their inputs are:
- **Joints:** joint angles, drum angles, line tensions, slack and clutch-slip flags, lock state, motor torque.
- **Balance:** the pelvis IMU (docs/26 models it with a 5 ms delay, bias and noise).
- **Feet:** four load sensors per sole and the CoP (centre of pressure).
- **Felt weight:** since D-063, the balance loop gets a carried bag's felt weight. It comes from the hand's lines; there
  is no separate hand sensor.

**The inner ear is the part we already have, in the wrong place.** An IMU is a vestibular organ:
- its gyroscope measures angular velocity, like the semicircular canals;
- its accelerometer measures linear acceleration and gravity, like the otoliths.

The build has a BNO085 at the pelvis and ICM-42688s on the torso and feet (docs/05). It has none in the head, where the
inner ear sits. The head IMU matters once there is a camera: it is what lets the eyes stay steady while the body moves.

**Vision is only planned hardware.** docs/04 plans a Pi 5 and one camera module in Phase 4, for planning and speech,
not balance. Hub task `perception-channels` (filed under P-063) plans depth vision, contact-event audio and felt weight
as named, ablated channel families in the sibling's policy. None of it is built.

## Is vision one input or layers?

**Layers, split at the retina and merged late.**
- **Two receptor systems.** Rods give sensitivity in dim light, with no colour. Cones give acuity and colour in
  daylight, in three kinds (S, M, L).
- **Three channels to the brain.** The retina compresses roughly 100 million receptors into about a million optic
  nerve fibres, as at least three parallel channels:
  - **magnocellular:** coarse, fast, motion and flicker, almost colour-blind;
  - **parvocellular:** fine detail and red-green;
  - **koniocellular:** blue-yellow, among others.

  They stay segregated up to the input layer of primary visual cortex ([Callaway 2005](https://pubmed.ncbi.nlm.nih.gov/15905213/)).
- **Cortex** splits again: "where and how" (dorsal, motion, toward MT) and "what" (ventral).
- **The split is real but leaky.** How cleanly the M and P channels map onto percepts is still argued
  ([Edwards, Goodhew and Badcock 2021](https://link.springer.com/article/10.3758/s13423-020-01874-w)).

**Fusion weighs each sense by how reliable it is right now.** People combine vision and touch like a
maximum-likelihood estimator: each cue is weighted by the inverse of its variance, so blurring the image shifts the
weight toward touch ([Ernst and Banks 2002, Nature 415:429](https://ideas.repec.org/a/nat/nature/v415y2002i6870d10.1038_415429a.html)).
Balance does the same across vision, touch and the inner ear. Weights shift within seconds when one source degrades
([Arduino, Schubert and Anson 2024](https://pmc.ncbi.nlm.nih.gov/articles/PMC11568912)).

**Why separate receptors evolved.** Physics forces trade-offs that one receptor cannot cover:
- **sensitivity against acuity:** pooling many rods catches single photons but blurs detail;
- **speed against detail:** a fast channel cannot also carry fine detail through a limited cable;
- **colour against light:** each extra pigment splits the light a receptor catches.

So evolution kept several cheap specialised channels and merged them late:
- **The pigments came from gene duplication.** Most mammals have two cone opsins; Old World primates duplicated one
  to get three ([Osorio and Vorobyev, review](https://pubmed.ncbi.nlm.nih.gov/15312027/)).
- **Selection kept it for foraging.** In wild capuchins, the three-cone females eat ripe fruit faster
  ([Melin et al. 2017](https://pmc.ncbi.nlm.nih.gov/articles/PMC5625910)).
- **Two cones also persist**, which is better for spotting camouflaged prey.

Each channel paid for itself in some task, and none had to be the whole picture.

**The periphery is for balance more than for seeing.** It is low-resolution but motion-sensitive, and the optic
flow it sees steadies quiet standing ([Horiuchi et al. 2017](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5633140/)).
Some walking studies find central flow works too, so the flow's structure may matter more than where on the retina it
falls ([MDPI 2019 review](https://mdpi.com/2076-3417/9/5/934)). For a robot this argues for a wide, coarse, fast
channel next to a narrow, sharp one.

## Hearing

**Binaural, yes.** Two ears locate sound by:
- **ITD (interaural time difference):** up to about 0.6 ms for a human head;
- **ILD (interaural level difference):** the head's shadow, at high frequencies;
- **spectral notches** from the outer ear's shape (captured by the HRTF), for elevation and front against back.

Two bare microphones give left and right but confuse front with back
([Técnico Lisboa, IROS 2006](https://vislab.isr.tecnico.ulisboa.pt/publications/06-iros-ears.pdf)). A shaped ear, or
turning the head, resolves that. Changing the ears or nearby parts changes the HRTF, so robots that learn it
recalibrate it against vision.

**Propagating sound correctly is a separate simulator.** MuJoCo has no acoustics.
- **SoundSpaces 2.0** computes binaural room impulse responses on any mesh by path tracing, validated against real
  rooms ([Chen et al. 2022](https://www.arxiv.org/pdf/2206.08312)). It is built on Habitat, not MuJoCo.
- **A cheaper first step:** our sounds are mostly the robot's own contacts (footfalls, a bag set down, a chair). So
  start with the direct path, using a spherical-head model for ITD and ILD per frequency band. Add room reflections
  later from SoundSpaces responses.

**Hardware:** two I2S MEMS microphones (INMP441 class, a few dollars each) in printed ear shells, and one ESP32 can
sample both at 48 kHz.

## How people control: loosely, on purpose

**Variability is a sign of health, not error.**
- Healthy stride-to-stride timing fluctuates with long-range, fractal correlations.
- Ageing and disease erode that structure, and the loss predicts falls better than the size of the variability
  ([Hausdorff 2007](https://pmc.ncbi.nlm.nih.gov/articles/PMC2267927)).

**The brain corrects only what matters.**
- **Minimal intervention:** optimal feedback control fixes deviations only where they threaten the goal. Variability
  is let free along the redundant ("uncontrolled") directions
  ([Todorov and Jordan 2002](https://papers.neurips.cc/paper_files/paper/2002/hash/8c5f6ecd29a0eb234459190ca51c16dd-Abstract.html)).
- **Muscle recordings agree:** the nervous system tightly controls the task-relevant variables and lets the others drift
  ([Valero-Cuevas, Venkadesan and Todorov 2009](https://pmc.ncbi.nlm.nih.gov/articles/PMC2712269)).

**Tracking everything is the strain.**
- **Learned muscle models do it unprompted:** they fall into high co-contraction until the reward pays for effort
  ([arXiv 2309.02976](https://arxiv.org/pdf/2309.02976v2.pdf)).
- **Letting springs work pays:** on the compliant COMAN humanoid it cut walking energy by 18%
  ([Kormushev et al. 2011](https://calinon.ch/papers/Kormushev-IROS2011.pdf)).

**The robot today is the "perfect robot" end of this.**
- **What it holds:** its balance loop holds the CoM over a planned point, and its servos track every joint target.
- **What that costs:** the static walk takes 88 s and 1130 J for five 100 mm steps (docs/23, the v0.6.0 gauntlet).
- **The one exception:** the drum locks (D-038) are the only place it relaxes.
- **The bag finding fits this picture (docs/39):** walking failed because the balance loop did not *sense* the bag,
  not because the bag swung.

## Proposal

These are the experiments for the hub topic. All are simulation only; nothing is bought.

1. **A vestibular channel in the head.**
   - Add a head IMU to the model and the sensor model (docs/26: delay, bias, noise), as a named channel next to the
     pelvis IMU.
   - Test: does the balance loop or a policy stand better with both, as the head moves (looking down at a stool, the
     chair's trunk swing)?
2. **Stereo depth first, then luminance, colour last.**
   - **What:** two head cameras with a 60 mm baseline. The policy gets:
     - a coarse, wide depth image (e.g. 48 x 64, about 120 degrees), the periphery, for optic flow and footholds;
     - a small central crop (e.g. 32 x 32 at 30 degrees), the fovea, for the stool, the bag and the chair.
   - **Why depth first:**
     - depth has the smallest gap between simulation and reality;
     - learned legged locomotion already works from one low-resolution depth camera, by training on a cheap stand-in
       then distilling ([Agarwal et al. 2022](https://arxiv.org/abs/2211.07638));
     - Apple's HALO uses 48 x 84 log depth (hub `records/halo-apple-tracker-assessment-20261109`).
   - **After depth:** luminance (black and white) for texture and edges.
   - **Colour** only for telling objects apart (which bag), at the planner level and not in the balance loop.
   - **First measurement:** MuJoCo's render cost per step on this CPU. Batched GPU rendering
     ([MuJoCo Warp](https://github.com/google-deepmind/mujoco_warp), which replaced
     [Madrona-MJX](https://madrona-engine.github.io/renderer-blog.html)) is the route if it is too slow; hub task
     `tendon-physics-on-gpu`.
3. **Binaural hearing of the robot's own contacts.**
   - **Sound sources:** contact events (footfalls, a set-down, the chair).
   - **Propagation:** to two ear points, with a spherical-head ITD and ILD per frequency band.
   - **Input:** band energies and ITD per band, not raw waveforms.
   - **Test:** whether a footfall's sound times the swing landing better than the foot load sensors.
4. **Fuse by reliability, and ablate.** Each sense is a named channel family in the sibling's policy, trained with and
   without it on the chain grid (`perception-channels`). The planner-side fusion weights each cue by its measured
   noise, as Ernst and Banks found people do.
5. **Relaxed control (the "drunken sailor" test).**
   - **Change:** loosen the hand-written controller in the directions that do not matter to the task. For example:
     - the trunk's sway while the feet are planted;
     - the swing foot's path away from obstacles;
     - the arms while carrying.
   - **Measure** on the gauntlet and chain grids:
     - energy;
     - clutch slip;
     - stride-to-stride variability;
     - falls.
   - **Expected:** less energy and no more falls. If falls go up, the loose directions were not really free.

## Sources

Gathered on 2026-11-10 from web search results (abstracts and excerpts). Most were not read in full; check the
original before relying on a number.

| Source | Used for |
| --- | --- |
| [Callaway 2005, J Physiol](https://pubmed.ncbi.nlm.nih.gov/15905213/) | The magno, parvo and konio pathways, segregated to V1 |
| [Edwards, Goodhew and Badcock 2021](https://link.springer.com/article/10.3758/s13423-020-01874-w) | The pathway split is real but debated |
| [Ernst and Banks 2002, Nature 415:429](https://ideas.repec.org/a/nat/nature/v415y2002i6870d10.1038_415429a.html) | Senses fused by reliability (maximum likelihood) |
| [Arduino, Schubert and Anson 2024](https://pmc.ncbi.nlm.nih.gov/articles/PMC11568912) | Vestibular anatomy; sensory reweighting; the canals' contested role in standing |
| [Osorio and Vorobyev, primate colour vision review](https://pubmed.ncbi.nlm.nih.gov/15312027/) | Opsins, gene duplication, trichromacy |
| [Melin et al. 2017](https://pmc.ncbi.nlm.nih.gov/articles/PMC5625910) | Trichromat capuchins forage fruit faster |
| [Horiuchi et al. 2017](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5633140/) | Peripheral optic flow steadies quiet standing |
| [Optic flow, central against peripheral (Appl. Sci. 2019)](https://mdpi.com/2076-3417/9/5/934) | Central flow works too; the flow's structure matters |
| [Técnico Lisboa, IROS 2006](https://vislab.isr.tecnico.ulisboa.pt/publications/06-iros-ears.pdf) | ITD, ILD, HRTF; two microphones confuse front and back |
| [SoundSpaces 2.0 (Chen et al. 2022)](https://www.arxiv.org/pdf/2206.08312) | Path-traced binaural room responses on meshes |
| [Hausdorff 2007](https://pmc.ncbi.nlm.nih.gov/articles/PMC2267927) | Fractal stride variability; its loss predicts falls |
| [Todorov and Jordan 2002](https://papers.neurips.cc/paper_files/paper/2002/hash/8c5f6ecd29a0eb234459190ca51c16dd-Abstract.html) | The minimal intervention principle |
| [Valero-Cuevas, Venkadesan and Todorov 2009](https://pmc.ncbi.nlm.nih.gov/articles/PMC2712269) | Task-relevant control in muscle recordings |
| [arXiv 2309.02976](https://arxiv.org/pdf/2309.02976v2.pdf) | Learned muscle models co-contract unless effort is paid for |
| [Kormushev et al. 2011](https://calinon.ch/papers/Kormushev-IROS2011.pdf) | COMAN's passive compliance cut walking energy 18% |
| [Agarwal et al. 2022](https://arxiv.org/abs/2211.07638) | Legged locomotion from one egocentric depth camera, distilled |
| [Madrona batch renderer](https://madrona-engine.github.io/renderer-blog.html), [MuJoCo Warp](https://github.com/google-deepmind/mujoco_warp) | GPU batch rendering for vision training |
| Hub `records/halo-apple-tracker-assessment-20261109` | HALO's 48 x 84 log-depth input |

## Not yet

- **Sensing:** nothing above is in the model; the robot has no head IMU, camera or microphone, even in simulation.
- **Measurements:** render costs are unmeasured. Whether a footfall's sound adds anything over the foot load sensors
  is untested.
- **Sources:** the biology is from reviews and search excerpts, not a systematic read. The canals' role in quiet
  standing, and whether the periphery outweighs central vision for balance, are both contested.

## Next

Start with 1 and 5, which need no new rendering:
- the head IMU as a channel;
- the relaxed-control test on the gauntlet's stand, step and walk, and the chain grid's carries.

Then measure the depth camera's render cost (2) before planning any vision training. The hub's `perception-channels`
task carries 2 to 4 into the sibling.
