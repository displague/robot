# 07 - Walking and balancing

Walking and balancing are goals, not stretch goals (P-002). This changes three things: the
W-max motor must be chosen for walking power, the legs get sensors and a controller the arms do
not need, and the leg mass distribution becomes a design constraint the tendon approach is
unusually good at.

## What walking asks of the joints

Human gait biomechanics, normalised per kilogram of body mass and scaled to the 20 kg robot.
Walking at about 1 m/s is "normal"; the slow column is a cautious 0.3 m/s gait the first
controller will aim for.

| Joint | Peak torque (N m/kg) | Robot peak torque | Peak power, normal walk | Peak power, slow walk | Tendon force on chosen pulley |
| --- | --- | --- | --- | --- | --- |
| Hip flex/ext | 1.0-1.5 | 25 N m | ~35 W | ~12 W | 420 N on 60 mm arm |
| Hip abd/add (stance stabilisation) | 1.0 | 20 N m | ~15 W | ~5 W | 330 N on 60 mm arm |
| Knee flex/ext | 0.5-1.0 (1.5+ for stairs, squat) | 20 N m (30 N m squat) | ~30 W | ~10 W | 400-600 N on 50 mm arm |
| Ankle plantarflexion (push-off) | 1.5-1.7 | 32 N m | ~65 W | ~20 W | 640 N on 50 mm arm |
| Ankle inversion/eversion | 0.3 | 6 N m | small | small | 120 N on 50 mm arm |

Peaks last tenths of a second. Continuous ratings matter for standing on one leg (hip abduction
holds the pelvis: ~20 N m for as long as the step takes) and for stairs.

## W-max for walking (`python tools/tendon_calc.py`, walking scenarios)

| Joint, motor | Need | Tendon force cont / stall | Move time | Power need / motor peak | Verdict |
| --- | --- | --- | --- | --- | --- |
| Ankle push-off, wiper motor, 15 mm drum, 2:1 | 640 N in 0.25 s | 500 / 1667 N | ~1 s | 67 / 18 W | Static walking only |
| Ankle push-off, 775 + printed 1:40 planetary, 15 mm drum, 3:1 | 640 N in 0.25 s | 864 / 2880 N | ~0.2 s | 67 / 113 W | **Works** |
| Knee swing, 775 + planetary, 15 mm drum, 2:1 | 360 N in 0.4 s | 576 / 1920 N | ~0.3 s | 47 / 113 W | Works |
| Hip swing, 775 + planetary, 15 mm drum, 2:1 | 420 N in 0.5 s | 576 / 1920 N | ~0.3 s | 35 / 113 W | Works |

The 775 brushed motor driving a printed two-stage planetary is the cheap path to a walking
humanoid: ~$8 motor, ~$3 of bearings, ~60 g of PETG gears, a BTS7960 driver, roughly 450 g per
actuator and 100 W class. Its risk is the printed gearbox at full load; the Phase 2 entry test
(one planetary at 100 W for an hour, then to destruction) decides it. The fallback is a 5010 /
5208 class gimbal BLDC with a printed cycloid (50-100 W, ~$30 + driver), which also brings
back-drivability and smooth torque control at roughly double the price. The wiper motor is
retired for legs: too slow and 21 kg across the body.

## Why tendons help a walker

- **Light shanks and feet.** Every leg W-max sits in the pelvis or the thigh and reaches the
  knee and ankle through sheathed tendons across the joints. Swing-leg inertia drops, which is
  the quantity that sets hip power during walking.
- **Series elasticity for free.** A tendon is a spring. The ankle plantarflexor tendon gets a
  deliberate elastic element (a loop of silicone cord or a printed TPU spring in series, like an
  Achilles tendon) that stores energy at heel strike and returns it at push-off, so the motor
  needs less peak power than the table says. The drag clutches turn a trip or a shove into a slip
  rather than a stripped gear.
- **Zero-power standing.** Self-locking stages hold a pose; the balance controller only spends
  energy on corrections.

## Sensing

| Sensor | Where | Part | Purpose |
| --- | --- | --- | --- |
| IMU | pelvis (primary), torso, each foot | BNO085 ($20) or ICM-42688 ($4) with fusion on the ESP32 | body attitude, angular rate, foot orientation |
| Foot contact and centre of pressure | 4 per foot at the sole corners | FSR ($2) under a TPU pad, or printed cantilever + HX711 load cell ($2) | ZMP estimate, heel-strike / toe-off events |
| Joint angles | every leg DOF | AS5600 (as on the arms) | state for the controller |
| Tendon tension (optional, Phase 6) | ankle and knee tendons | a sheath-end load cell: the PTFE tube presses on a printed beam with a strain gauge | true joint torque for impedance control |

## Controller

Runs on the two leg ESP32 nodes plus the torso node, which owns the pelvis IMU and publishes
the balance reference at 200 Hz over CAN. No Pi in the loop; the Pi only says where to walk.

1. **Stand** (Phase 5): ankle strategy. Pelvis IMU tilt and sole centre-of-pressure feed a PD
   loop on ankle plantar/dorsiflexion and inversion/eversion. Hip strategy added for larger
   disturbances. Exit test: a 20 N push at the shoulder without a step.
2. **Weight shift and single-leg stance** (Phase 5): hip abduction holds the pelvis level while
   the other foot lifts 50 mm. This is the first real load on the hip W-max. Done in simulation
   at P-013 (`15`): 100 mm lift, 10 s hold, stance hip roll 15 to 25 N m.
3. **Static walking** (Phase 6a): ZMP kept inside the stance foot at all times; precomputed
   joint trajectories at 0.2-0.3 m/s; foot sensors sequence the steps. Slow, obviously
   robotic, and achievable with the force and speed figures above.
4. **Dynamic walking** (Phase 6b): linear inverted pendulum model with capture-point stepping,
   swing-leg trajectories, and the ankle series elastic element doing push-off. Needs the tendon
   tension sensing and the 775/planetary headroom.

Falls: the robot is tested on a ceiling harness until 6b is stable. On a detected fall (pelvis
tilt rate above threshold) the controller goes `limp` on the arms and crouches the legs; the
self-locking stages mean the robot stiffens rather than collapses, which is better for the
robot and worse for whatever it lands on. A fall-safe posture is part of the Phase 6 exit test.

## Feet

Flat printed sole 260 x 100 mm, TPU 95A pad, four force sensors at the corners, a passive toe
hinge with an elastic for a natural roll. A large foot makes static walking easy and makes the
ZMP estimate robust; a smaller foot comes later if dynamic walking works.

## Mass constraint restated

Walking makes the 20 kg budget hard, not soft. Current allocation: 12 leg W-max at 0.45 kg =
5.4 kg; 9 upper W-max = 4 kg; W-mini/W-mid 1.6 kg; printed structure 6 kg; hardware, battery,
electronics 3 kg. Every kilogram saved in the upper body is a kilogram the legs do not have to
carry, so the face, forearm rotation and spine lateral tilt are the first DOF to cut if the first
weigh-in comes out heavy.
