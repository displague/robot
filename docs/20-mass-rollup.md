---
title: "20 - Mass roll-up: the robot as specified is about 26 kg, not 20 (P-031)"
summary: "Mass roll-up of every printed and bought part: about 26 kg against the 20 kg budget; the model on those masses (the ankle at its rating, the step 11 of 14); where mass can come off (D-036)"
section: "Drives, mass and energy"
---

# 20 - Mass roll-up: the robot as specified is about 26 kg, not 20 (P-031)

The 20 kg budget in `05` and the masses in `cad/skeleton.json` were estimates written before most parts
existed. The parts list now names every bought part (`05`, with item weights from the Amazon listings) and
the generators build every joint, so the mass can be added up instead of assumed (hub task
`printed-mass-rollup`). It comes to about 26 kg. On those masses the model still stands through the
whole gauntlet, but the ankle reaches its line rating and the step stands in 11 of 14 grid variants instead of 14.

## The tool (`tools/mass_rollup.py`)

```bash
python tools/mass_rollup.py --build --stl <dir>         # run the generators, then roll up
python tools/mass_rollup.py --infill 0.25 --wall 1.2 --md rollup.md --json masses.json
ROBOT_MASSES=masses.json python tools/sim_tendon.py     # the model on the rolled-up masses
```

Printed parts are weighed from each STL's volume V and surface area A: the walls and skins are a solid
shell A x 1.2 mm thick (three 0.4 mm lines), the rest is infill at 25 %. The generators' own figures are solid
PETG. Bought parts carry the listing weights collected from the Amazon listings (the hub note on the task),
or estimates marked `est`. Parts with no generator yet (pelvis and torso frames, head, soles, the W-max
gearbox) are estimates. Every STL in the folder must be listed in the tool's `PARTS` (count and segment) or
`IGNORE`, so a new generator part is not left out silently.

## Result (25 % infill)

| Segment | Printed (generated) | Printed (est) | Bought | Total kg | Model kg | Over |
| --- | --- | --- | --- | --- | --- | --- |
| pelvis | 0.35 | 1.50 | 3.40 | **5.26** | 3.50 | +1.76 |
| spine (torso) | 0.77 | 2.15 | 5.58 | **8.50** | 3.50 | +5.00 |
| neck | 0.17 | 0.00 | 0.05 | **0.22** | 0.30 | -0.08 |
| head | 0.11 | 0.35 | 0.05 | **0.51** | 0.80 | -0.29 |
| clavicles | 0.16 | 0.15 | 0.00 | **0.31** | 0.40 | -0.09 |
| upper arms | 0.84 | 0.16 | 1.19 | **2.19** | 1.80 | +0.39 |
| forearms | 0.60 | 0.00 | 0.42 | **1.02** | 1.80 | -0.78 |
| hands | 0.42 | 0.00 | 0.04 | **0.46** | 0.80 | -0.34 |
| thighs | 1.22 | 0.90 | 3.35 | **5.47** | 4.40 | +1.07 |
| shanks | 0.72 | 0.00 | 0.45 | **1.16** | 1.80 | -0.64 |
| feet | 0.42 | 0.50 | 0.11 | **1.03** | 1.20 | -0.17 |
| **total** | 5.78 | 5.71 | 14.64 | **26.1** | 20.3 | +5.8 |

At 15 % infill the total is 25.3 kg, at 40 % 27.4 kg. Filament to buy: about 13 kg (printed parts plus 15 %).

What the numbers say:

- **The W-max drive is half the robot.** 21 x (775 motor 354 g + BTS7960 with heatsink 65 g + two 608s 24 g +
  an estimated 150 g printed gearbox, drum and clutch) = 12.4 kg. The `05` budget had 9.5 kg for it. The nine
  in the torso (spine and shoulders) are why the spine segment is 5 kg over.
- **The printed joints are not the problem.** The generators' joint ends are 12.0 kg solid but 5.8 kg as
  printed. (`05` briefly said 13 kg, from the solid figures.) The heaviest is the knee's lower half at 281 g,
  around its 100 mm pulley.
- The distal segments are lighter than the model assumed (forearms, shanks, hands); the mass has moved
  into the pelvis and torso, which is better for balance than the total suggests.

## The model on these masses

`ROBOT_MASSES` (an override of `skeleton.json`'s `mass_kg` in `export_mjcf.build()`) puts the rolled-up masses
into the model: 26.25 kg instead of 20.43. The controllers are unchanged.

| Scene | Model as committed (20.4 kg) | Rolled-up masses (26.3 kg) |
| --- | --- | --- |
| stand, pushes 10 to 40 N, sustained, squat | stand | stand |
| one-foot hold, 10 s | stands, tilt 4.4 deg, 811 J | stands, tilt 10.9 deg, 888 J; **stance ankle pitch line 306 N peak (rating 288), its clutch slips 33 deg** |
| static step, 100 mm | stands, worst leg line 186 N, 444 J, no flags | stands, worst leg line 281 N (SF 3.2), 737 J, neck roll above rating |
| step grid, 14 variants | 14 of 14 | **11 of 14**: the 85 mm step on both models and 90 mm on `--stiff-mini` fall in the transfer (phase 4, lift gate timed out) |

The neck's flags in the pushes go away on the heavier model (its head is lighter, 0.51 against 0.8 kg). The
hand flags in the arm and shift scenes are the same on both.

So the drive train carries 26 kg, but only just: the stance ankle is at its line rating and its clutch slips
on one foot, which is the only way to stand on one foot or walk. The real masses are not the model default
yet (D-036): the step and the hold were tuned on 20 kg, and the mass should come off before the baseline moves.

## Where the mass can come off

| Change | Saves | Cost or risk |
| --- | --- | --- |
| A 10 to 15 A H-bridge without the BTS7960's heatsink for each W-max (the firmware limits current to 90 % of the clutch slip, about 7 A on a 775; D-021) | ~1.1 kg | none known; a driver to choose and price |
| The W-max gearbox designed to a mass target (150 g each is a guess; no generator yet) | 0 to 1.5 kg | the Phase 2 gearbox test (`bench-load-tests`) |
| Brushless W-max (5010 class, 109 g against 354 g) with a FOC driver in place of the BTS7960 | ~5 to 6 kg | needs its own drive-train calculation (lower continuous torque, a higher ratio, larger drivers than the SimpleFOC Mini) and about $300 more |
| Fewer W-max: spine roll fixed, one combined shoulder axis (`05` suggested 17) | ~0.6 kg each | lost DOF; ankle roll is used by the weight shift (D-026), so not that one |
| 15 % infill instead of 25 % on everything but the knee, hip and ankle | ~0.6 kg | strength of the large shells, unmeasured |
| Thin-wall pipe (PVC SDR 21 or aluminium) for the femur, tibia and spine | ~0.5 kg | stiffness, socket fit (pipe OD is the same) |
| A 5 Ah battery instead of 10 Ah (the step averages 10 to 17 W) | ~0.27 kg | runtime |

Without changing the motor, the first, fifth, sixth and seventh rows bring it to about 23.5 kg. Reaching 20 kg
needs a lighter W-max motor.

## The torso on worm gearmotors (P-034)

The worm gearmotor class the owner found (hub record `worm-gearmotor-options-20261028`) suits the torso: slow
posture joints that hold. The one that meets the torso's line peaks (232 to 250 N, 3.5 to 3.8 N m on the 15 mm
drum) is the 12 V 10 rpm, 1:634 option rated 40 kg cm (3.92 N m) at 1.4 A, 380 g listed (probably the shipping
weight). In the model (`ROBOT_TORSO_WORM=1`, class `W-worm`, motor `WORM_10RPM` in `tendon_calc.py`; stall taken
as twice rated, which the listing does not state) it drives the spine and both shoulders: 9 of the 21 W-max.
Line rating 261 N continuous, 523 N stall; about 9 deg/s at the spine. It self-locks, so the drum never back-drives.

Mass (`mass_rollup.py --torso-worm`): 25.0 kg against 26.1, the torso 7.4 kg against 8.5 (the motor saves little
at 380 g; the BTS7960s and printed planetaries for those nine go).

| Run | Model as committed (775, 20.4 kg) | Worm torso, 20.4 kg | Worm torso, its roll-up (25.0 kg) |
| --- | --- | --- | --- |
| gauntlet | all stand | all stand; fewer neck flags | all stand except the step (falls in the transfer) |
| one-foot hold | 811 J, 41 W | **568 J, 29 W** | 531 J, 29 W |
| step | 444 J, no flags | 437 J, no flags | falls |
| step grid | 14 of 14 | **14 of 14**, landings within 3 mm | 4 of 14 |

The last column is the mass distribution, not the drive: crossing them, the worm torso on the 775 roll-up's masses
stands 12 of 14 (the 775 on them: 11), and the 775 on the worm roll-up's masses 2 of 14. With the torso 1.1 kg
lighter the transfer onto the leading foot fails again (phase 4, the lift gate times out with the trailing foot
still loaded), so the step as tuned depends on where the mass is. That is the re-tuning D-036 defers until the
masses are settled.

So the worm torso is no worse for the balance (the spine and shoulders are not in the balance loop) and saves
energy, but it saves only ~1.1 kg and its 9 deg/s spine and shoulders make the arms slow. Its stall torque, the
real (unpacked) weight and whether one purchase is one motor are still to check on the bench.

Since D-037 the step no longer depends on the mass distribution: it stands in all 42 variants of the
grid over the three mass sets (`19`). The one-foot hold on the rolled-up masses (ankle line at 306 of 288 N)
is still the case for taking mass off.

## Not yet

- The pelvis and torso frames, head, soles and W-max gearbox have no generator; they are 5.7 kg of guesses.
- The model puts each segment's mass at the bone's midpoint; the battery and the W-max banks are not there.
- The JGB37 weight and several boards' weights are estimates or packaged weights.

## Next

Choose the W-max driver and gearbox targets and re-roll; decide the motor (775 versus brushless) with a
`tendon_calc.py` entry for the 5010 and the bench results (`bench-load-tests`); then move the model to the
rolled-up masses and re-tune the step and the one-foot hold on them.
