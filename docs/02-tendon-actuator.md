# 02 - The tendon actuator (winch)

One design, three sizes. A motor turns a small drum; braided fishing line winds on the drum and
leaves through a PTFE tube socket tangent to the drum. A self-locking stage holds tension with
no power. A friction drag slips before anything printed breaks. The joint, not the motor, carries
the angle sensor.

```
   motor ---> [gearbox] ---> [worm stage, optional] ---> drum ---> PTFE socket ---> sheathed tendon ---> joint pulley
                                                       |
                                               friction drag (overload)
```

## Physics (the four numbers that matter)

| Quantity | Formula | Design lever |
| --- | --- | --- |
| Tendon force | F = T_drum / r_drum x tackle | smaller drum or more tackle strands = more force |
| Line speed | v = omega_drum x r_drum / tackle | bigger drum or fewer strands = more speed |
| Mechanical power | P = F x v | **only the motor sets this**; drum and tackle just split it |
| Excursion | s = sum over crossed joints of (angle_rad x moment_arm) | sets drum turns and sheath length |

A brushed DC motor delivers peak mechanical power at half its no-load speed and half its stall
torque: P_peak ~ T_stall x omega_noload / 4. That single figure tells you which class a joint
needs before you ever pick a drum.

### Reference results (`python tools/tendon_calc.py`)

| Joint / configuration | Need | Tendon force cont / stall | Full-ROM time | Power need / motor peak | Verdict |
| --- | --- | --- | --- | --- | --- |
| Finger, N20 100:1, 3 mm drum | 15 N, 0.6 s | 13 / 42 N | 0.47 s | 0.9 / 1.3 W | **Good**: fast close, light squeeze, 42 N peak pinch |
| Finger, N20 298:1, 3 mm drum | 40 N, 0.6 s | 37 / 122 N | 1.6 s | 2.3 / 1.0 W | Strong but slow; a "grip" variant |
| Finger, N20 100:1 + 10:1 printed worm | 40 N | 71 / 236 N | 4.1 s | 2.3 / 0.6 W | Worm costs 55 % of the power; too slow for fingers |
| Wrist, N20 298:1, 4 mm drum, 2:1 tackle | 100 N, 0.8 s | 55 / 184 N | 1.9 s | 2.3 / 1.0 W | Marginal; W-mini is the floor for wrists |
| Elbow, JGB37 1:90, 6 mm drum, 2:1 tackle | 137 N, 1.5 s | 128 / 425 N | 3.4 s | 6.7 / 3.3 W | Holds 1 kg; moves at half target speed |
| Shoulder, JGB37 1:270, 8 mm drum, 3:1 | 370 N, 2 s | 295 / 984 N | 21 s | 23 / 2.4 W | **Fails on power**: shoulder is W-max territory |
| Knee sit-to-stand, wiper motor, 15 mm drum, 2:1 | 400 N, 2 s | 500 / 1667 N | 3.7 s | 24 / 18 W | Works, slowly; 0.9 kg per knee |
| Knee sit-to-stand, JGB37 1:270, 4:1 tackle | 400 N, 2 s | 394 / 1313 N | 24 s | 24 / 2.4 W | Force yes, power no |

Loads assumed: 20 kg robot, 1 kg payload in hand, forearm 0.8 kg, whole arm 3 kg. Motor figures
are Pololu-class catalogue values; no-name N20s deliver roughly two thirds of them.

## Locking tension without power

| Option | Holds with zero power | Efficiency | Back-drivable (safety / compliance) | Printable | Verdict |
| --- | --- | --- | --- | --- | --- |
| High-ratio spur gearbox (N20 298:1+) | mostly (creeps under sustained load) | 70-80 % | barely | n/a (bought) | Default for W-mini. Add a small hold current when it matters. |
| Printed single-start worm after the gearbox | yes, hard lock | 35-55 % | no | yes (PETG worm, nylon or PETG wheel, grease) | Use where holding matters more than speed: thumb opposition, neck, spine. Not fingers. |
| Wiper motor (worm built in) | yes | ~50 % | no | n/a | Free lock for W-max if wiper motors are chosen |
| Ratchet + pawl, pawl lifted by a micro servo | yes, one direction | ~95 % | no | yes | Fishing-reel anti-reverse. Good for "grip and hold" fingers if the spur creep proves annoying |
| Friction drag (Belleville or printed spring on the drum) | n/a | n/a | slips at set force | yes | Not a lock: **overload protection**. Every W-mid and W-max gets one. |

Rule: W-mini = spur gearbox only (speed matters, loads are small, a 0.1 A hold is cheap).
W-mid = spur + ratchet or hold current. W-max = worm (wiper) or brake, plus drag.

## Drum design rules

- **Drum diameter >= 10x line diameter** for fatigue life of braided PE. 3 mm core radius with
  0.36 mm (50 lb) line is 17x. Good.
- **Keep wraps in one layer.** Drum width >= turns x line diameter x 1.3. For a finger (1.8 turns
  of 0.36 mm) that is 1 mm; a 5 mm wide drum is generous. For an elbow (4 turns of 0.55 mm) 3 mm.
  For excursions over ~10 turns, print a helical groove (level-wind) so the effective radius
  stays constant and the position loop stays linear.
- **Anchor the line through the flange**, knot on the outside in a counterbore, not through the
  core (a knot in the core makes a bump that changes the radius every revolution).
- **Tangent exit.** The PTFE socket axis is tangent to the drum core at the plane of the drum. The
  line then enters the tube straight and the tube does the steering.
- **D-bore on the motor shaft, stub axle on the far side** riding in the housing, so the motor
  bearing is not cantilevered by tendon load.
- **Closed-loop drum**: two line anchors, lines wound in opposite senses from the drum centre
  outward; the joint pulley has the same two anchors. Pretension by winding both on with the
  joint at mid-range and a 2-5 N spring scale on each strand.

## Line

| Line | Diameter | Break | Stretch | Creep | Verdict |
| --- | --- | --- | --- | --- | --- |
| Braided PE (Dyneema/Spectra), 4 or 8 carrier | 0.28-0.75 mm | 130-900 N | <2 % | slow, real under sustained >30 % load | **Use this** |
| Fluorocarbon | 0.3-0.9 mm | similar | 5-10 % | low | too stretchy for position control |
| Nylon monofilament | 0.3-0.9 mm | similar | 10-25 % | high | no |
| Kevlar/aramid cord | 0.5-1 mm | 200-500 N | <2 % | none | fine alternative, abrades on sharp edges more than PE, dearer |

Working load <= 25 % of break strength (safety factor 4) because every pass over a pulley or
through a tube costs strength. The calculator reports the factor per scenario.

Creep is handled, not fought: hard stops at both ends of every tendon, home on power-up by
driving into the stop at low current, re-home any joint whose joint-sensor-to-motor-encoder ratio
has drifted by more than a threshold. Dyneema creep at our loads is fractions of a millimetre per
day and the homing routine absorbs it.

Knots: a Palomar or a doubled figure-eight in braid holds ~80-90 % of break strength. Melt the
tag end with a lighter into a mushroom; it cannot pull through a 1.2 mm hole.

## Why the "closing in air, squeezing on contact" profile suits a DC motor

Closing a finger in free air costs only the elastic return (~3-5 N). A 100:1 N20 does that at
~90 mm/s, so the hand closes in half a second. On contact the speed drops along the motor's
torque-speed line until it stalls at ~42 N tendon force (~5 N at the fingertip). The driver
limits current to a safe hold value and the gearbox holds the grip. No worm needed, no second
speed, and the same tendon gives speed where speed is wanted and force where force is wanted.

## What the W-mini module looks like

`cad/blender/winch_mini.py` generates it: a 41 x 18 x 13 mm open-top housing that an N20 slides
into from the rear, a bulkhead with the motor's two M1.6 holes and the shaft bore, a drum chamber
with the drum on the shaft's D-flat and a 2 mm stub riding in the end wall, a tangent 4 mm PTFE
socket boss on the side, and a screw-on lid. Nine of them side by side are a forearm bank.

![W-mini winch: housing, drum, lid (exploded)](img/winch_mini.png)

Generated parts (PETG, solid): housing 6 g, drum 0.4 g, lid 1.8 g. All three meshes are manifold.

Next versions: a 2-anchor closed-loop drum, a ratchet variant, the W-mid housing for a 37 mm motor
with a 2:1 tackle block, and the drag clutch (a printed wave spring between drum and a friction
disc keyed to the shaft).
