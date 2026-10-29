# 27 - Energy and batteries: how far it walks on a charge, and where the power goes (P-041, D-045)

The owner asked whether the motors' energy is counted, how long the robot can walk against the wind on its
batteries, whether cylindrical USB power banks stored in the PVC bones would help with weight or power
distribution, and what the power plan is. Short answers:
- The motors' energy is counted. It now has a report (`tools/energy_report.py`).
- On the real build (26 kg, the sourced motor figures) a charge walks about **18 m in about 1.2 h**. Wind up to the
  3 to 5 m/s the walk survives changes that by under 10 %.
- Power banks cannot run the drives, and they store less per kilogram than the planned pack. The walk's energy
  goes into holding torque in the W-max drives, and that is where the gain is.

## What is counted

The model's electrical energy is per motor: the applied voltage times the current the motor draws for its torque
(`sim_tendon.Rig.step`, `e_j`: duty x supply voltage x stall current x torque / stall torque). That includes
the motor's copper losses and the power to hold a torque with the motor stalled. It is zero when a drum is
locked by its self-locking stage. Not counted: the drivers' losses (about 10 %), the electronics, and regeneration
(none is assumed). `tools/energy_report.py` prints a scene's motor energy per winch class with the mean,
99th-percentile and peak power. `ROBOT_MOTORS` and `ROBOT_MASSES` apply.

**The motor figures matter most.** The model's own 775 figures (D-021, a stall of 25 A) were contradicted by
nine datasheets (docs/24): the 775 draws about 66 A at stall, so 2.6 times the current for the same torque.
The W-max carries nearly all the walking energy, so the walk costs 3.4 times what the model's figures said.

## The power plan today

- **Main pack** (docs/04, 05): 12 V rail from a 3S LiPo, 10 Ah (111 Wh, 543 g, in the pelvis), or a 3S Li-ion pack
  with a 40 A BMS, or an 18 V tool battery through a buck. A 40 A fuse; bucks to 6 V (W-mini) and 5 V (logic).
- **Electronics** (estimates): 6 ESP32-S3 at 0.15 to 0.35 W, 21 AS5048A encoders at about 0.05 W, the IMUs, 21
  W-max driver boards' logic at about 0.1 W, with the bucks' losses: **about 8 W**, and 5 to 7 W more with the Pi 5.

## How far it walks

Motor energy from the model; total = motors x 1.1 (drivers) + 8 W (electronics, no Pi); the battery is the 111 Wh
pack used to 80 % (89 Wh). The walk is docs/23's: four 100 mm steps and a closing step, 0.4 m in about 100 s.

| Build | Wind | Motors, mean | Peak (99 % / max) | Total | Runtime | Distance |
| --- | --- | --- | --- | --- | --- | --- |
| 20 kg, model's motor figures | still | 11.7 W | 97 / 190 W | 21 W | 4.3 h | 64 m |
| 20 kg, sourced nominal | still | 39.5 W | 318 / 613 W | 52 W | 1.7 h | 26 m |
| 20 kg, sourced nominal | 3 m/s head | 37.8 W | 298 / 643 W | 50 W | 1.8 h | 27 m |
| **26 kg build, sourced nominal** | still | 59.4 W | 373 / 625 W | 73 W | **1.2 h** | **18 m** |
| 26 kg build, sourced nominal | 3 m/s head | 58.9 W | 367 / 698 W | 73 W | 1.2 h | 18 m |
| 26 kg build, sourced nominal | 5 m/s tail | 60.3 W | 345 / 645 W | 74 W | 1.2 h | 17 m |
| 26 kg build, standing | still | 14.4 W | 148 / 305 W | 24 W | 3.7 h | |
| 26 kg build, standing | 10 m/s head | 13.6 W | 120 / 305 W | 23 W | 3.9 h | |

- **Wind is not what drains the battery.** The walk only survives up to 3 m/s head-on and 5 m/s from behind
  (docs/25), and there the drag is 2 to 6 N against 255 N of weight. A head wind slows the walk a little, a tail
  wind costs a little more holding. Standing in a 10 m/s wind costs no more than standing in still air.
- **Holding is what drains it.** The walk is static and slow (about 4 mm/s): every transfer stage waits for the
  load and the CoM to settle, and the leg drives hold the body's weight through the 775s at stall-like
  currents the whole time. The cost of transport (energy / weight x distance) is 55; a person walking is about
  0.2, ASIMO about 3. Every second of waiting costs about 60 W.
- **Peaks.** The 26 kg walk peaks at 700 W, 63 A at 11.1 V, and its 99th percentile is 34 A. Docs/04 planned 300 W
  continuous and 600 W peak on a 40 A fuse: the peaks are above that plan. The fuse holds (a slow fuse rides
  through 63 A for milliseconds) and the LiPo's C rating covers it, but the logic must ride through the sag
  (below).
- **Weight costs energy.** The 26 kg build walks on 50 % more energy than 20 kg (59 against 40 W). It carries
  more wind (docs/26), but that is the wrong trade for the battery and for the W-max, which already falls at the
  pessimistic end of its range (docs/24).

## Power banks in the bones

Cylindrical USB power banks are lithium cells with a 5 V converter:

| | Energy | Mass | Wh/kg | Output | Diameter |
| --- | --- | --- | --- | --- | --- |
| Anker PowerCore 5000 ([Pack Hacker](https://www.packhacker.com/travel-gear/anker/powercore-5000/), [Galaxus](https://www.galaxus.nl/en/s1/product/powercore-5000-mah-10-w-1850-wh-powerbanks-6486510)) | 18.5 Wh | 134 g | 138 (about 117 delivered at 5 V) | 5 V 2 A (10 W) | 33 mm |
| Anker PowerCore+ mini 3350 ([Alza](https://www.alza.de/EN/anker-powercore-mini-3350mah-silver-d4286253.htm)) | 12.4 Wh | 80 g | 155 | 5 V 1 A (5 W) | 23 mm |
| A 10 000 mAh PD 20 W bank, flat ([Sandberg](https://sandberg.cz/en/product/nano-powerbank-10000-pd20w)) | 38.5 Wh | 163 g | 236 | 5 V 3 A, up to 12 V 1.7 A (20 W) | flat |
| The planned 3S LiPo 10 Ah | 111 Wh | 543 g | 204 | 12 V, 30+ A | pelvis |
| 21700 cells, 3S2P (6 x 5 Ah, 18 Wh, about 70 g and 25 A each, e.g. Samsung 50S) | 108 Wh | ~450 g with a BMS | ~240 | 12 V, 50 A | 21 mm |

- **They cannot run the drives.** The legs need 34 A at 12 V, peaking at 63 A. A bank gives 5 V at 1 to 3 A, cuts
  off on overload, and many switch off at low load (a standing robot with its drums locked). It would take about
  40 banks in parallel through boost converters.
- **They do not save weight.** Per kilogram they store less than the LiPo and lose about 15 % in their converter.
  The bones' Schedule 40 bores (1 in: 26.6 mm, 1-1/4 in: 35.1 mm, 1-1/2 in: 40.9 mm, 2 in: 52.5 mm) take a bank or
  cells, but they also carry the lines, sheaths and wiring (docs/03).
- **Mass in the legs costs twice.** A battery in the shank or thigh is mass the hip W-max must swing and lift at
  every step. The W-max is the weak drive (docs/24), and the swing is where the walk fails. The pelvis, where the
  pack is, is the best place: it is near the CoM and costs nothing in the swing. The 2 in spine pipe is the
  second-best.
- **Where a bank does help: a separate logic supply.** One bank on the 5 V logic (ESP32s, IMU, encoders; about
  6 W) keeps the controller alive when a 63 A peak sags the 12 V rail. That is the failure mode a balance loop
  cannot afford. A PowerCore 5000 runs it about 2.5 h, longer than the main pack lasts walking. The bank must not
  switch off at low load (choose one with an always-on or low-current mode), and the grounds are joined at one
  point. A buck from the main pack with a hold-up capacitor does the same job without a second charger.
- **More energy, if needed,** comes cheapest as a second main pack in the pelvis or a 21700 3S2P pack in the spine
  pipe, not as banks. Each extra 100 Wh is about 0.45 kg and adds about 1.2 h of walking, but also more energy per
  metre (mass).

## The trade-offs

| Lever | Energy | Cost |
| --- | --- | --- |
| A W-max that holds without current: lock the stance leg's drums between stages, or a worm/self-locking stage on the leg drives | the largest: the walk is mostly holding | the lock gate is off while walking (it waits for stillness, D-038); a locked joint cannot balance |
| A W-max with less current per N m (a higher ratio, a 4S pack, the 5010 brushless; hub task `wmax-stands-at-low`) | the 66 A 775 is 3.4 x the model's energy | speed, mass, money |
| A faster walk (shorter transfer stages) | less time holding: at 60 W, every second saved per step is 60 J | stood 39 of 42 when tried (docs/23) |
| Less mass (20 kg against 26) | 33 % less | the W-max options, thin-wall pipe (docs/20) |
| More battery | linear in runtime | 0.45 kg per 100 Wh, and that mass costs energy too |
| More ESP32 nodes (a balance node, D-044) | about 0.3 W each | nothing measurable: 0.5 % of the walk |

## Not yet

- Electronics power is an estimate, not a measurement; regeneration and the drivers' losses are factors.
- Energy at the pessimistic motor end (`ROBOT_MOTORS=low`), where the walk falls.
- The walk's energy with the stance drums locked between stages.
- A battery's position in the mass model (the pack sits at the pelvis bone's midpoint, docs/20).

## Next

The W-max decision (`wmax-stands-at-low`) now has a second criterion: energy per metre at the sourced nominal.
Then locks in stance while walking.
