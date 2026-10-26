# 08 - Passive balance, springs, and compliance (P-005)

Three questions from P-005: can balance be had passively (ballast, a bladder); are spools or
winding springs needed for tension and rest positions; is the design soft and compliant.

## Passive balance

### What a standing biped gets for free

A standing robot with every leg winch locked is a rigid statue. It stays up as long as the
centre of mass (CoM) stays over the foot polygon, and costs zero power (D-005, D-006). Its
margin against a push is set by foot length and mass distribution, not by any controller:

| Quantity | Value (20 kg robot, CoM at 0.9 m, foot half-length 0.13 m, push at 1.4 m) |
| --- | --- |
| Restoring moment about the foot edge | 20 x 9.81 x 0.13 = 25.5 N m |
| Quasi-static push that tips the statue | 25.5 / 1.4 = **18 N** |
| Same with 0.15 m foot half-length | 21 N |
| Same with 2 kg of ballast in each foot (24 kg total) | 25.5 + 4 x 9.81 x 0.13 = 30.6 N m -> 22 N |
| Gravitational stiffness m g h that passive ankle springs would have to beat | 176 N m/rad = 3.1 N m per degree |

Lessons:

- **Foot length is the cheapest balance.** Every centimetre of foot is worth about a newton of
  push margin. The 260 x 100 mm foot is the first lever; a 300 mm foot is on the table.
- **Ballast helps, modestly.** Weight in the feet adds restoring moment without adding push
  lever, and it is free: a pocket in the sole for steel shot or sand, about 1-2 kg per foot. It
  also lowers the CoM, which makes the robot *fall faster* once past the edge but *resist more*
  before it. Worth doing. Everything else heavy (battery, pelvis W-max bank) already lives low.
- **Passive ankle springs cannot make it stable on their own.** To stand without control, ankle
  stiffness must exceed m g h, 176 N m/rad. On a 50 mm ankle pulley that is 3.5 kN per radian of
  line, 300 N per ankle at 10 degrees of lean. No elastic does that; a locked winch does. So the
  passive stance is the lock, and springs are for centring and damping around it.
- **A bladder is the wrong ballast.** A free liquid surface moves the CoM the wrong way during a
  lean (the ship free-surface effect) unless baffled, and pumping water between tanks shifts the
  CoM far too slowly for a 0.5 s step. Liquid would only make sense as a passive damper in a
  sealed, baffled tank, and sand does the same job without leaks.
- **A reaction wheel is the real "natural" balancer** if one is wanted later: a 1 kg rim at
  0.15 m in the torso on a cheap BLDC gives a short burst of corrective torque independent of the
  feet. Adds about $40 and 1.3 kg and a spinning mass. Option for Phase 6b, not part of the
  baseline.

Humans are not passively stable either: ankle stiffness is 60-90 % of m g h and the rest is
reflex. The plan copies that: locked statue when still, ankle and hip strategy when disturbed.

### Where the mass goes (restated as a balance rule)

Feet (ballast, sensors) > pelvis (W-max bank, battery) > thighs (W-max) > torso (W-max for
shoulders and spine) > arms > head. Nothing heavy above the shoulders. Shank and foot structure
stay light so swing inertia is low; the foot ballast is a deliberate exception because it sits
still during stance and the swing cost is accepted for the stance gain.

## Springs, spools, rest positions

The winch drum itself needs no spring: it is self-locking, so "rest position" is wherever it
stopped, at zero power. Springs appear in four places, each for a specific job:

| Place | Element | Job | Why a spring and not a motor |
| --- | --- | --- | --- |
| Finger and thumb extensors | elastic cord now; **constant-force spring spool** (tape-measure style, 1-3 N, $1-2) or a **retractable badge reel** ($0.50) at the back of the hand as the Phase 1 trial | return stroke and slack take-up | An elastic's force grows with stretch and steals grip at full curl; a constant-force spool returns the same 3 N everywhere and never fatigues like TPU |
| Shoulder flexion and abduction (later hip flexion) | **zero-free-length spring counterbalance**: an elastic cord anchored on the torso, routed through the joint axis, pulling on the upper arm (the Anglepoise lamp principle) | cancel the arm's own weight (3 kg at 0.3 m = 9 N m) so the W-max only carries payload | Doubles the useful continuous torque of a power-limited motor for cents; tuned so the spring torque tracks sin(angle) like gravity does |
| Ankle plantarflexor tendon | series elastic element (silicone cord loop or coil spring in line with the tendon) | store heel-strike energy, return it at push-off, soften impacts | The motor cannot deliver push-off power fast enough on its own; the spring does the fast part |
| Closed-loop pretension | barrel adjuster (stiff, manual) or a **spring-loaded sheath seat** (compression spring behind the tube end) | keep the loop tight as the line creeps | The spring version re-tensions itself but adds compliance; use it only where position precision does not matter (neck, spine), the adjuster everywhere else |

Not used: clock springs on the drum to "pre-wind" tendons, and spring-return on closed loops
(the loop needs none). Pretension on fingers is set by the elastic, on loops by the adjuster.

## Compliance

The design is compliant in the ways that matter for safety and grasping, stiff in the ways
that matter for standing, and the firmware can move it between the two.

| Source | Amount | Effect |
| --- | --- | --- |
| Tendon stretch (braided PE, <2 % at a quarter of break) | 1-3 mm on a 300 mm run at working load | a few degrees of give under load; invisible to control because the angle sensor is at the joint (D-008) |
| Sheath compression and seat settling | ~1 mm | same |
| Elastic antagonists (fingers) | full underactuated conformity | a hand closes around whatever shape it meets; fingers can be pushed open against 3 N |
| Series elastic element (ankle, optionally knee) | designed 5-10 mm at peak load | impact softening and energy return |
| Friction drag on W-mid and W-max | slips above a set torque | an arm pushed hard gives way instead of breaking its pulley; a trip at the knee does not strip a gear |
| TPU pads on fingertips, palms, soles | millimetres | contact compliance and grip |

What is **not** compliant: a locked winch. The gear train is non-backdrivable, so a joint that
is holding cannot be pushed through its motor; it can only give by tendon stretch or drag slip.
That is the price of zero-power holding and it is also the standing strategy. For a human-sized
robot it needs three safeguards, all planned:

1. **Drag thresholds set at human-safe levels.** An arm drag that slips at 15 N m is 25 N at the
   hand: a firm push. Leg drags set well above standing load but below pulley failure.
2. **Force sensing for free, then impedance control.** With a known series spring in the tendon,
   the difference between motor encoder and joint sensor times the spring rate *is* the tendon
   force. No load cell needed. The node runs an impedance loop: when sensed force exceeds the
   commanded stiffness, the motor pays out. Bandwidth is limited by the gearmotor, so this is
   compliance at the tens-of-hertz level, fine for handling, not a substitute for the drag.
3. **A `limp` mode** that spins the motor to follow the joint, used on the arms during a fall and
   for hand-guiding the robot to teach poses.

Not soft-robotic: the skeleton is a rigid shell with soft layers where it touches the world.
Foam or TPU skins can go over the shells later without changing anything underneath.

## Decisions taken here

- D-015: foot ballast pockets and foot length are the passive balance measures; no bladder;
  reaction wheel deferred.
- D-016: constant-force spool trial for finger extensors; zero-free-length elastic
  counterbalance on shoulder flexion and abduction; series spring force sensing and impedance
  mode as the compliance mechanism, drags as the safety floor.
