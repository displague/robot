---
title: "34 - The brushless W-max in the model (P-058, D-058, hub task wmax-mass-down)"
summary: "The 5010 brushless W-max in the model (`ROBOT_WMAX=bldc`): passes the gauntlet on the 20 kg masses with its servo at the 775's stiffness, holds a foot on 36 % less energy; the 19.9 kg brushless build loses the chair because its trunk is 5.8 kg (D-058)"
section: "Drives, mass and energy"
---

# 34 - The brushless W-max in the model (P-058, D-058, hub task wmax-mass-down)

docs/21 chose, on paper, a 5010 360 KV outrunner on a printed 1:25 at 24 V for the W-max: the only drive that brings
the roll-up under 20 kg (19.9 kg). This puts it in the model and runs it.

Short answers:
- **The drive passes the gauntlet on the 20 kg model's masses once its servo is tuned to the 775's stiffness.**
  It also holds one foot on 36 % less energy, because it pays copper loss rather than the brushed drive's duty times
  volts times amps.
- **Corrected by docs/36 (D-060):** the trunk's mass is close to neutral for the chair. The pelvis and the light arms
  push the CoM back, and the build stands from 0.45 m once the hip drives ride on the thighs and the battery moves up.
- **The 19.9 kg brushless build is not the 20 kg model.** Its trunk is 5.8 kg against the model's 3.5 kg, so its
  CoM is high. With the 775 or the 5010 at these masses the chair rise at 0.45 m falls. Getting the total under
  20 kg is not enough: the mass has to come out of the trunk (next: docs/35, the shell).

## The variant

`ROBOT_WMAX=bldc` (read by `export_mjcf`, like `ROBOT_TORSO_WORM`): every W-max joint on `5010_PLAN25_24V`.
- **No self-lock**: `hold=0`, so its drums never lock (`sim_tendon` skips the lock where `t_hold` is 0) and a
  standing joint holds on current.
- **Energy**: FOC copper loss (3/4 I^2 R, line-to-line R) plus the shaft power out over the gearbox efficiency, the
  current from torque. The brushed rule (duty x bus volts x current) treats the driver's 20 A limit as the stall
  current and would charge the winding as 1.2 ohm, not 0.26.
- **Gains**: the servo's torque per radian of gear error scales with the drive's stall torque, and the 5010 on 1:25
  has 0.48 of the 775's (6.9 against 14.4 N m at the drum). `kg_scale` (default 14.4 / 6.9 = 2.09, override
  `ROBOT_WMAX_KG`) restores the 775's stiffness; a FOC servo would be tuned for its own motor.
- Masses: `sim/masses/rollup_bldc.json` (`mass_rollup.py --wmax bldc`, 19.9 kg) and
  `sim/masses/rollup_775_small.json` (the 775 on the VNH5019-class driver, 25.0 kg).

## Results (gauntlet, one run per scene)

| Scene | 775, model masses | 5010 unscaled, model masses | 5010, model masses | 775, bldc masses | 5010, bldc masses |
| --- | --- | --- | --- | --- | --- |
| stand, pushes 10 to 30 N, sustained, arm, squat, gusts, shift, one foot | stand | stand | stand | stand | stand |
| push 40 N | stands | **falls** | stands | **falls** | stands |
| step, walk | stand | stand | **fall** (grid: stands) | stand | stand |
| chair 0.45 m | stands | stands | stands | **falls** | **falls** |
| one-foot hold, J | 562 | 327 | 360 | 516 | 359 |
| walk, J | 1053 | 1496 | fell | 1168 | 2038 |

The unscaled servo loses the 40 N push. With the gains scaled, the gauntlet's single step run falls on the model's
masses, but the step grid (`step_sweep.py --masses 20kg=,bldc=sim/masses/rollup_bldc.json`, 2 models x 3 lengths x 2
mass sets, which is how a step change is judged, docs/18) stands 12 of 12 at a gain scale of 1.5 and 2.09, and 10 of 12
unscaled. The walk costs about twice the energy on the brushless drive: it pays current to hold every stance joint.

## Not yet

- The full step and walk grids (7 lengths, held feet) and the pessimistic motors (`ROBOT_MOTORS=low`: the 5010's
  continuous fraction is 0.15 there).
- The chair rise on the brushless build: the trunk's mass (docs/35).
- The 5010's figures are listing-based (`sim/motor_ranges.json`); the drive has no generator, and its gearbox is
  the same 150 g guess as the 775's.

## Next

The torso and pelvis as a keyed thin shell (docs/35), its printed mass into the roll-up; then the brushless build's
chair and walk at the new masses, and the W-max motor as a decision.
