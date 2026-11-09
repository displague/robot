---
title: "Rotational geometry and physics: a design idea bank"
summary: "Rotational geometry and physics idea bank: each discussion point and what it could mean for the robot, with cheapest tests"
section: "Design and hardware"
---

# Rotational geometry and physics: a design idea bank

Source: a three-prompt exploratory discussion (P-008), "facts about rotational geometry and
physics", then "knots, pulleys, gears, cylindrical and conical", then "hollow threading, splines
and morphological twists". This page captures every point from that discussion in our own words,
then says what each one could mean for this robot.

Status: **inspiration, not decisions.** Nothing here changes `decisions.md`. Items marked
**[beyond the discussion]** are extensions added while mapping the ideas onto the robot; they
are flagged so they can be checked separately. Every figure is arithmetic on stated inputs, not a
measurement (README principle 6).

Notation: plain text math. `r` radius or moment arm, `theta` angle in rad, `omega` angular
velocity, `alpha` angular acceleration, `tau` torque, `I` moment of inertia, `L` angular momentum
(or a length, where stated), `J` polar second moment of area, `mu` friction coefficient.

---

## 1. Rotation as geometry

### Captured

- **A rotation is an isometry (rigid motion).** Every point moves; distances, angles and shape do
  not change. Size, shape and intrinsic properties survive a rotation; only orientation changes.
- **Rotation matrices.** A 2D counter-clockwise rotation by `theta` is
  `[[cos theta, -sin theta], [sin theta, cos theta]]`. Computers and engineers rotate points and
  frames this way; 3D rotations are 3x3 orthonormal matrices with determinant +1 (the group SO(3)).
- **2D rotations commute, 3D rotations do not.** 30 then 40 degrees in a plane equals 40 then
  30. In 3D, 90 degrees about X then 90 about Y leaves a book facing a different way than Y then
  X. Order is part of the answer.
- **Euler's rotation theorem.** Any rigid displacement that keeps one point fixed equals a single
  rotation by some angle about some fixed axis (axis-angle).

### For the robot

1. **Axis order is a design parameter, not a convention.** Every multi-axis joint (wrist,
   shoulder, hip, ankle, waist universal + yaw, D-010) composes rotations in the order its
   hardware stacks them. `cad/skeleton.json` should record that order per joint, the Blender rig
   (`skeleton_rig.py`) should use the matching rotation mode, and firmware forward kinematics
   must multiply in the same order. A mismatch looks like a sign error only at large angles.
2. **Put the gimbal singularity where the body never goes.** A three-axis stack loses one DOF when
   its middle axis reaches +/-90 degrees. Choose the stacking order of shoulder and hip so that
   configuration sits outside everyday poses (for the shoulder, human ROM makes this impossible to
   avoid fully; see item 3). **[beyond the discussion]**
3. **Limit 3-DOF joints with swing-twist, not an Euler box.** By Euler's theorem the shoulder's
   orientation is one axis-angle. Decompose it into a *swing* (where the arm points, a cone) and a
   *twist* (rotation about the arm's own axis). Human shoulder and hip ROM are naturally a
   swing cone plus a twist range; a per-axis Euler limit box is too permissive in some
   directions and too strict in others. Blender's Limit Rotation constraint is an Euler box, so
   the rig check under-reports this. **[beyond the discussion]**
4. **Plan orientation moves along one axis.** Head turns, wrist re-orientation and hand
   approach look natural and take the shortest path when interpolated as a single axis-angle
   (quaternion slerp) rather than three independent joint ramps.
5. **Rigid-motion invariance is the reason segment frames work.** Mass, CoM offset and inertia
   per segment (the P-006 mass model) are computed once in the segment's own frame and carried
   by the joint transforms; nothing about the segment needs recomputing per pose.

---

## 2. Linear motion and its rotational twins

### Captured

| Linear | Rotational twin | Meaning |
|---|---|---|
| position `x` | angle `theta` | how far it has turned, in rad |
| velocity `v` | angular velocity `omega` | how fast it spins, rad/s |
| mass `m` | moment of inertia `I` | resistance to spin, depends on how far mass is from the axis (point mass: `I = m r^2`) |
| force `F` | torque `tau = r x F` | the twisting effect of a force |
| momentum `p = m v` | angular momentum `L = I omega` | rotary inertia in motion |

- **Conservation of angular momentum.** With no external torque, `I1 omega1 = I2 omega2`. A
  skater who pulls the arms in lowers `I` and spins faster.

### For the robot

1. **A tendon drive is a chain of these twins.** Motor torque becomes line force at the drum
   (`F = tau_motor / r_drum`), line force becomes joint torque at the joint's moment arm
   (`tau_joint = F r_joint`), and speeds go the other way (`v = omega_drum r_drum`,
   `omega_joint = v / r_joint`). The drum-to-joint ratio is `r_joint / r_drum`, exactly the
   pulley ratio in section 5. `tools/tendon_calc.py` already does this; the framing makes clear
   that the drum radius and the joint moment arm are the two cheapest "gears" we own.
2. **Inertia goes as distance squared, so mass placement dominates.** A 0.2 kg motor at 0.5 m
   from the hip adds `0.2 x 0.5^2 = 0.05 kg m^2` to leg swing; the same motor at 0.15 m adds
   `0.0045 kg m^2`, eleven times less. This is the physics behind regional winch banks
   (principle 5) and behind D-011's "all leg motors in pelvis and thigh". Use it as a scoring
   rule in the mass model: every gram below the knee is worth about ten grams near the hip.
3. **Arm swing is a free reaction wheel.** A swinging leg puts angular momentum about the
   vertical axis into the body; human arms swing opposite to cancel it so the stance foot need not
   resist the yaw with friction. Our arms already exist and are already actuated, so counter-phase
   arm swing (or torso counter-rotation at the waist yaw) is the cheapest yaw balancer available
   and should be in the Phase 6 gait before any added wheel. **[beyond the discussion]**
4. **If the reaction wheel (D-015, deferred) ever comes back, make it a rim.** Torque from a wheel
   is `tau = I_wheel alpha_wheel`, and a thin ring has `I = M R^2`, twice a solid disc of the same
   mass and radius (`1/2 M R^2`, section 6). A printed ring channel filled with steel shot (the
   same shot as the D-015 foot ballast) puts nearly all the mass at the rim.
5. **Skater effect for whole-body motion.** Tucking limbs speeds up a turn-in-place; extending
   them slows any rotation, including an unwanted one during a stumble. Worth a line in the
   fall/limp-mode policy (D-016): limb posture changes the body's spin rate even when no joint
   can push on the ground. **[beyond the discussion]**
6. **Reflected inertia is the gear ratio squared.** **[beyond the discussion]** A rotor of inertia
   `I_rotor` behind an `N:1` reduction feels like `N^2 I_rotor` at the output. At 100:1 that is
   10,000x. It decides how hard a locked joint hits on impact and how backdrivable (D-016) a
   joint can ever be; it belongs in the calculator next to force and speed.

---

## 3. Advanced cross-over facts

### Captured

- **Lie algebra.** Angular velocity is the *infinitesimal generator* of rotation: a tiny rotation
  moves a point by `dr = dtheta x r`. The cross products in torque and velocity are the structure
  of the Lie algebra so(3) that generates the group SO(3).
- **Spin and the 720-degree return.** A macroscopic object is unchanged after 360 degrees. A
  spin-1/2 particle needs 720 degrees to return to its original quantum state, because its
  symmetry group SU(2) double-covers SO(3).

### For the robot

1. **Product-of-exponentials kinematics.** Treat each joint as a unit screw axis (the Lie
   algebra element) and write forward kinematics as `T = exp([S1] theta1) ... exp([Sn] thetan) M`.
   Every knuckle-chain preset (`chain.py`, D-017) already knows its pin axes; exporting them as
   screw axes into `skeleton.json` gives one kinematics description that Blender, a URDF export
   and ESP32 firmware can all use. **[beyond the discussion]**
2. **IMU integration on an ESP32.** Integrate gyro rates as `q <- q * exp(omega dt / 2)` and
   renormalise: small, exact for constant rate over a step, no trig tables beyond one sin/cos.
   This is the pelvis and foot IMU path in D-011.
3. **Double cover in practice: `q` and `-q` are the same orientation.** Unit quaternions are
   SU(2), so every orientation has two representations. Before averaging, filtering or slerping
   two quaternions, flip one if their dot product is negative, or the interpolation takes the
   350-degree way round. Cheap bug to prevent, expensive to find on a balancing robot.
4. **The tendon coupling matrix is a Jacobian.** A tendon that crosses joints with moment arms
   `r1, r2, r3` changes length by `dl = r1 dtheta1 + r2 dtheta2 + r3 dtheta3`; for many tendons,
   `dl = R^T dtheta` and joint torques are `tau = R f` for line tensions `f`. The finger's
   10 / 7 / 5 mm arms (D-004) are one column of `R`. Writing `R` out per region makes coupling,
   underactuation and the crossing rules of D-017 checkable by a script. **[beyond the discussion]**
5. **The plate (Dirac belt) trick is the macroscopic face of the double cover.** A hand can
   turn a held plate through 720 degrees about the vertical while the arm untwists back to its
   start; after only 360 the arm is wound. For us: a grasped object can be turned beyond the
   forearm's pronation/supination ROM by coordinating shoulder, elbow and forearm, with no
   continuous-rotation joint and no slip ring. The same topology underlies "anti-twister"
   mechanisms that let a tethered body rotate continuously while the tether carrier turns at half
   speed. **[beyond the discussion]**

---

## 4. Knots: rotation in topology

### Captured

- **A knot is a closed curve embedded in 3D.** Its links to rotational geometry are symmetry and
  invariants (quantities unchanged by any smooth deformation).
- **Torus knots** lie on the surface of a torus and are fixed by two integers `(p, q)`: how many
  times the curve winds around each of the torus's two rotational directions.
- **Helicity.** When magnetic field lines or vortex lines in a fluid or plasma are knotted or
  linked, the field carries helicity, a measure of twist and linking that is conserved under
  smooth motion.

### For the robot

1. **Linking number as a routing check.** Every closed-loop tendon (D-014) is literally a closed
   curve, and each one threads past rotating members. The linking number between a tendon loop
   and a joint axis (or another loop) is an integer that no amount of joint motion within ROM can
   change. A Blender script can compute it (Gauss linking integral over the routed polylines) and
   assert zero wherever a loop must not wrap a rotating part. **[beyond the discussion]**
2. **Keep tendons that cross a roll joint on its axis.** A run at radius `rho` from a roll axis,
   over length `L`, twisted by `theta`, lengthens to `sqrt(L^2 + (rho theta)^2)` (it becomes a
   helix, section 7). For a forearm (`L = 100 mm`, `theta = 180 deg`): `rho = 3 mm` gives 0.44 mm
   of unwanted excursion, `rho = 8 mm` gives 3.1 mm. This is the quantitative form of D-017's
   stub-axle rule, applied to forearm and waist yaw.
3. **Torus-knot winding patterns.** A line that wraps a ring joint (D-017's axial ring joints) is a
   curve on a torus. Choosing `(p, q)` deliberately is a way to lay several lines on one ring
   without crossings, or to wind a printed lattice (shell rib, compliant rope) that is balanced in
   both rotational directions. Speculative; a sketch in Blender would show whether it helps.
4. **Braided line has helicity of its own.** A braid under load tries to untwist, which can spin
   a crimped bead or termination. Fishing tackle solves this with a **barrel or ball-bearing
   swivel**, cheap and already sized for the line. Add swivels where a termination is seen to
   rotate in Phase 0/1 tests. **[beyond the discussion]**
5. **Knots as terminations** are covered in D-014 (stopper knots, Palomar, crimps).

---

## 5. Pulleys and gears: transferring torque

### Captured

- **No slip means equal surface speed.** At the contact (tooth mesh or belt) `v1 = v2`, and with
  `v = omega r`: `omega1 r1 = omega2 r2`, so `omega1 / omega2 = r2 / r1`.
- **Gears** count teeth: `omega1 N1 = omega2 N2`. Meshed external gears turn in **opposite**
  directions.
- **Belt or chain pulleys** turn in the **same** direction.
- **Real pulleys have inertia.** The textbook pulley is massless and frictionless, so tension is
  equal on both sides. A real pulley with `I != 0` needs a tension difference to accelerate:
  `tau = r (T1 - T2) = I alpha`.

### For the robot

1. **Every tendon path is a belt drive.** Drum and joint pulley are two pulleys on one belt; the
   ratio is `r_joint / r_drum` (section 2). A closed loop (D-014) is an open belt (same
   direction). Swapping which run goes to which side of the joint pulley is a crossed belt
   (reversed direction). So the left/right mirror of a limb is a routing choice (or a sign in
   firmware), and one drum design serves both sides with no mirrored prints.
2. **Tension is not equal around anything real.** Two effects, both missing from the "massless,
   frictionless" picture:
   - Pulley inertia: `T1 - T2 = I alpha / r`, significant only for large redirect pulleys during
     fast moves.
   - Friction around bends **[beyond the discussion]**: the capstan relation
     `T_out = T_in exp(mu phi)` for total wrap angle `phi`. With `mu = 0.1` (PE line in PTFE, to be
     measured), a 180-degree run loses about 27 %, and 360 degrees of accumulated bends about 47 %
     (`exp(0.1 x 2 pi) = 1.87`). Bowden friction is likely the single largest loss in the system.
     Proposal: the calculator takes total bend angle per tendon as an input, and Phase 0 measures
     `mu` with a spring scale on each side of a known bend.
3. **Gears, where we use them,** follow `omega1 N1 = omega2 N2`, with the printed planetary of
   D-006 as the main case. Direction reversal per external mesh matters for winch handedness.

---

## 6. Cylindrical versus conical

### Captured

- **Cylinders keep a constant radius.** A spur gear or drum pulley has the same `r` all along
  its axis. A solid cylinder about its axis has `I = 1/2 M R^2`.
- **Cones make radius a variable.** Cones, frustums and tapers redirect rotation or change
  mechanical advantage along the axis.
- **Step cone pulleys** (old lathe headstocks) stack cylinders of decreasing size; moving the belt
  from step to step changes the speed/torque ratio without a gearbox.
- **Bevel gears** cut teeth on a cone frustum so shafts can meet at an angle, typically 90 degrees,
  as in a car differential. Because radius varies along each tooth, pitch lines and load sharing
  are harder to calculate.

### For the robot

1. **A tapered drum is a gearbox that shifts by itself.** If the line winds from a large radius
   onto a small one as the joint closes, the start of the stroke is fast and weak and the end is
   slow and strong. For a hand that is exactly the grasp we want: quick approach, firm final
   squeeze. Going from `r = 8 mm` to `r = 4 mm` halves speed and doubles force at the end of the
   stroke from the same motor. Reverse the taper for a joint that needs most torque early, such as
   knee extension from a deep squat. **[beyond the discussion]**
2. **The fusee.** Clockmakers solved "constant output from a varying spring" with a cone carrying
   a helical groove: radius `r(theta)` chosen so `F(theta) r(theta)` is constant. For us: shape the
   drum (or the joint pulley) so a finger's rising elastic antagonist force (D-014), or a limb's
   gravity torque varying with `cos theta`, is cancelled along the stroke. This is a printable
   part: the groove profile comes straight from the force curve. **[beyond the discussion]**
3. **Non-circular joint pulleys (cams)** are the same idea at the joint end: a moment arm that
   changes with angle. Human muscles have moment arms that vary with joint angle; a cam lug on a
   knuckle-chain preset reproduces that. Pairs with the zero-free-length counterbalance of D-016.
4. **Stepped drum** (two or three radii on one winch, line terminated on the step chosen at
   assembly) gives one W-mini part number several force/speed variants, keeping "one actuator
   family, three sizes" honest while widening each size's range.
5. **Single-layer grooved winding keeps a drum cylindrical.** Multi-layer winding grows the
   effective radius layer by layer, which is an uncontrolled taper that makes excursion depend on
   how much line is already wound. Helical grooves at a pitch slightly above line diameter keep
   `r` constant and excursion linear in drum angle.
6. **Bevel gears and differentials.** A 90-degree turn lets a motor lie along a limb and drive a
   drum or joint across it (hip yaw from the pelvis, for example). A **differential wrist**
   **[beyond the discussion]**: two motors drive two bevel gears (or two tendon pulleys) on one
   joint; turning them together gives pitch, opposite gives yaw, and both motors share the load
   in either motion. The tendon-only equivalent is a whiffletree, which is already how one tendon
   drives a whole underactuated finger.
7. **Rim mass for the flywheel case:** solid disc `1/2 M R^2`, thin ring `M R^2` (section 2, item 4).

---

## 7. Hollow threading: helices and torsion

### Captured

- **A screw thread is an inclined plane wrapped around a cylinder (or cone).** Unrolled, one turn
  is a right triangle: base `pi d`, height the pitch `P` (axial advance per revolution). The
  thread follows a helix.
- **Hollow costs little torsional stiffness.** Torsional rigidity scales with the polar moment
  `J`. Solid: `J = pi/2 R^4`. Tube: `J = pi/2 (Ro^4 - Ri^4)`. Material far from the axis carries
  almost all the torque, so removing the core saves weight and loses little stiffness.
- **Threads are handed.** A right-hand thread tightens clockwise. Where a shaft spins
  continuously (bicycle pedals, angle grinders), one side uses a left-hand thread so rotation and
  inertia tighten the joint instead of backing it out.

### For the robot

1. **Hollow bones, quantified.** With `Ro = 10 mm`:

   | `Ri` | material kept | `J` kept |
   |---|---|---|
   | 5 mm | 75 % | 94 % |
   | 7 mm | 51 % | 76 % |
   | 8 mm | 36 % | 59 % |

   This supports principle 3 (hollow PETG shells). For FDM it translates directly into slicer
   settings: torsion and bending stiffness live in the **perimeters**, so more walls and less
   infill is the right trade for every bone. **[beyond the discussion]**
2. **The hollow is a conduit.** The same bore that costs nothing structurally carries tendons
   and wiring along the axis, which is where section 4 item 2 says they belong when crossing a
   roll joint. Hollow-bore ring joints and hollow bearings (D-017) are the robot form of a hollow
   shaft.
3. **Self-locking is a lead-angle condition.** **[beyond the discussion]** The unrolled triangle
   gives the lead angle `tan(lambda) = P / (pi d)`. A screw or worm cannot be back-driven when
   `tan(lambda) < mu`, and its forward efficiency is `tan(lambda) / tan(lambda + atan(mu))`.
   Example: single-start module-1 worm, 10 mm pitch diameter: `tan(lambda) = 0.1` (5.7 degrees);
   with `mu = 0.2` it locks, and efficiency is about 33 %. This is the geometry behind D-005's
   finding that the worm costs about half the power, and the knob to turn if a lock is needed
   with less loss (raise lambda just to the edge of `mu`, which then must be measured).
4. **Drum grooves are threads.** The groove is a helix with pitch about line diameter plus a gap;
   the line exit point travels `P` per drum turn, which sets the fleet angle into the line exit or
   PTFE tube. Keep total drum travel times pitch within the exit's tolerance, or add a level-wind
   (section 8, item 3).
5. **Handedness rules for printed threads and caps.** Any screw or threaded cap that sits on a
   rotating drum should be handed so that line pull and drum inertia tighten it. With the drum
   design shared across left and right (section 5, item 1), the drum's wind direction is fixed,
   so a single handedness rule covers both sides.

---

## 8. Splines: lock rotation, free the axis

### Captured

- **Splines** are many teeth around a shaft meshing with matching slots in a hub. They carry far
  more torque than one key or pin, which can slip or shear.
- **Involute profiles.** High-performance splines use the same involute curve as gears, so
  contact force stays evenly spread across teeth as load rises, reducing stress concentration and
  fatigue.
- **Rotation locked, axial motion free.** A car's driveshaft lengthens and shortens over bumps on
  its splines while still transmitting full torque.

### For the robot

1. **Motor-to-drum couplings.** An N20 D-flat into PETG is a weak point under reversing load. A
   printed internal spline on the drum, mating a small metal splined or knurled adapter on the
   shaft (or a printed spline sleeve on a coupler), spreads the load over many teeth. FDM limits
   tooth size; use a coarse module and verify by test before trusting fine involutes.
   **[beyond the discussion]**
2. **Indexed assembly.** A spline with `N` teeth can only assemble at multiples of `360/N`
   degrees (24 teeth: 15 degrees; 36 teeth: 10 degrees). Use it to set joint zero, drum phase or
   magnet orientation for the AS5600 (D-008) so the sensor's 0/360 wrap sits outside the joint's ROM,
   repeatably, without a calibration jig.
3. **Spline + lead screw = level-wind.** Like a baitcasting reel: the drum (or the line guide)
   slides on a spline while a lead screw geared to the drum moves it one line pitch per turn. Line
   always leaves at the same point, fleet angle stays zero, and the drum can be long and thin.
   Fishing tackle again supplies the reference design. **[beyond the discussion]**
4. **Quick-release modules.** Splined interfaces that lock rotation but allow axial removal suit
   swapping a winch bank or a hand without re-homing, provided the spline is indexed (item 2).
5. **Telescoping members** where a segment must transmit twist while changing length (none
   planned today; candidate if the waist or neck grows a sliding DOF).

---

## 9. Morphological twists

### Captured

- **A morphological twist** is a shape whose whole cross-section rotates steadily along its long
  axis.
- **Pre-twisted beams and blades.** Turbine blades, rotors and drill bits are permanently
  twisted. A rotor tip moves faster than the root, so the twist keeps the angle of attack optimal
  along the whole length.
- **Poynting effect.** Twist a nonlinear elastic material (rubber, soft tissue) hard enough and it
  does more than twist: it lengthens, or develops internal axial tension and pushes outward.
  Large rotational strain distorts linear geometry.
- **Chiral twists in nature.** DNA, plant tendrils and umbilical cords use twisted, handed
  geometry to carry bending and pulling without kinking, tangling or snapping.

### For the robot

1. **Fingers that converge.** Human fingers, curled, point toward a common spot near the base of
   the thumb because each finger's flexion axes are slightly rotated along its length: a
   morphological twist of the joint axes. A per-joint axis roll knob in `chain.py` presets would
   reproduce it and improve grasp closure on small objects. **[beyond the discussion]**
2. **Twist built into the legs.** Human legs carry torsion (the foot points slightly outward in
   gait), which widens the support polygon sideways. A few degrees of toe-out designed into the
   shank/foot shells is free lateral margin for D-015's passive balance. **[beyond the discussion]**
3. **Helical tendon routing.** A tendon that spirals around a limb converts a pull into combined
   flexion and roll, as the biceps both flexes the elbow and supinates the forearm. Wrapping a
   tendon helically around a ring joint is a way to drive forearm pronation/supination, or to
   add a coupled roll to a single-tendon motion. Section 4 item 2's helix formula gives the
   excursion. **[beyond the discussion]**
4. **Twisted string actuation: a muscle made of fishing line.** **[beyond the discussion]**
   Twisting two strands of line together shortens them: for strand radius `r` and twist angle
   `theta` over length `L`, contraction is about `L - sqrt(L^2 - (theta r)^2)`. The reduction
   ratio starts near infinite (very strong) and falls as the twist builds (faster), with no
   gearbox: a small fast motor pulls hard directly. It uses the same braided line (D-002), but line
   wear from strand-on-strand friction and the nonlinear ratio are the open risks. A candidate
   alternative to a gearmotor winch for the hand bank, worth one bench test.
5. **Coiled nylon fishing-line muscles.** **[beyond the discussion]** Over-twisted nylon
   monofilament coils into a spring that contracts when heated (published by Haines et al., 2014).
   Interesting because the raw material is literally fishing line, but efficiency is low (around
   1-2 % in the literature) and speed is limited by cooling. Noted for completeness; not a fit for
   walking, perhaps for a facial or slow postural DOF.
6. **Poynting effect in soft parts.** TPU bushings, foot pads and the series elastic element
   (D-011) will push or pull axially when twisted hard. Either allow for it (axial clearance) or use
   it (a twist-to-push compliant element). **[beyond the discussion]**
7. **Chirality cancels torque.** Pairing left- and right-handed helices (counter-wound springs,
   opposite-lay twisted strings, tendons spiralled in opposite directions on either side of a limb)
   cancels the net twist each would apply alone, the same reason braided line is more stable
   than a single twisted ply. **[beyond the discussion]**
8. **Pre-twisted compliant flexures.** A printed thin beam with a built-in twist has different
   stiffness in its two bending directions along its length, which may suit finger extensor
   springs or toe flexures. Speculative.

---

## 10. Where each idea lands

| Idea | Section | Touches | When to try | Cheapest test |
|---|---|---|---|---|
| Axis order recorded per joint | 1 | `skeleton.json`, `skeleton_rig.py`, firmware FK | Now (no hardware) | Rig: compare FK at large angles against Blender |
| Swing-twist limits for shoulder and hip | 1 | Rig limits, firmware ROM guard | Before Phase 3 arms | Script the cone check against human ROM tables |
| Mass-at-distance scoring in the mass model | 2 | P-006 mass model, D-011 | Now | Spreadsheet of `m r^2` per segment |
| Counter-phase arm swing for yaw | 2 | Phase 6 gait | Phase 6a | Measure stance-foot yaw torque with and without |
| Reflected inertia in the calculator | 2 | `tendon_calc.py` | Now | Add `N^2 I_rotor` column |
| Screw-axis export (product of exponentials) | 3 | `chain.py`, `skeleton.json` | Now | Generate, compare with the rig |
| Quaternion sign check | 3 | IMU filter firmware | Phase 5 | Unit test |
| Tendon coupling matrix `R` | 3 | Hand, wrist routing | Phase 1 | Measure excursion per joint on the Phase 0 finger |
| Linking-number routing check | 4 | Blender routing scripts | Shell pass | Script on routed polylines |
| Tendons on axis at roll joints | 4, 7 | Forearm, waist | Shell pass | Helix formula per run |
| Swivels at spinning terminations | 4 | D-014 terminations | Phase 0 | Watch the bead under load |
| Capstan loss per tendon | 5 | `tendon_calc.py`, Phase 0 | Phase 0 | Spring scales either side of a known bend |
| Mirror limbs by routing, one drum design | 5 | Winch banks | Phase 1 | Paper check |
| Tapered / fusee drum | 6 | W-mini drum | Phase 0 variant | Print a second drum, compare close time and pinch force |
| Stepped drum variants | 6 | W-mini family | Phase 1 | Same housing, swap drum |
| Differential wrist | 6 | Wrist | Phase 1-3 | Two W-minis on one wrist |
| Perimeter-heavy slicing for bones | 7 | All shells | Now | Twist test two prints, same mass, different walls |
| Lead-angle lock tuning | 7 | D-005 fallback, W-max | If hold test fails | Printed worms at two lead angles |
| Spline coupling and indexed assembly | 8 | Drum-shaft, joint zero, AS5600 | Phase 0-1 | Print and torque to failure |
| Level-wind | 8 | Long thin drums | If fleet angle bites | Borrow a baitcaster mechanism |
| Converging finger axes | 9 | `chain.py` presets | Phase 1 hand | Small-object grasp test |
| Toe-out leg torsion | 9 | Shank/foot shells | Phase 5 | Push test with and without |
| Twisted string actuator | 9 | Hand bank alternative | Side experiment | One finger on a TSA vs W-mini |

## 11. Open questions this raises

- What is `mu` for braided PE in PTFE, and in a needle liner (D-013)? It decides whether capstan
  loss or motor power is the real limit on long leg tendons.
- Does a tapered drum's changing radius upset excursion-based position control enough to matter,
  given D-008's joint-side sensing makes the joint angle the truth anyway?
- Can a printed spline in PETG outlast the N20's stall torque reversals, or does the drum need a
  metal insert (post-print, per D-013's rule)?
- Is twisted-string line wear tolerable at home-robot duty cycles, and does it beat a $4 N20 on
  force per gram and per dollar?
