# 04 - Electronics and compute

## Topology

Five identical limb nodes on one CAN bus, one optional brain. Every node is the same board
design (or the same breakout modules on a printed carrier) so there is one firmware image and
spares are interchangeable.

| Node | Winches | Motor classes |
| --- | --- | --- |
| Left arm | 7 hand + 2 wrist (W-mini), 1 forearm + 1 elbow (W-mid) = 11 | 9 mini, 2 mid |
| Right arm | same = 11 | 9 mini, 2 mid |
| Torso | 6 shoulder + 3 spine (W-max), 3 neck + 4 face (W-mini) = 16 | 7 mini, 9 max |
| Left leg | 3 hip + 1 knee + 2 ankle (W-max) = 6 | 6 max |
| Right leg | same = 6 | 6 max |

The torso node is the busiest; split it into torso-upper (neck, face, shoulders) and torso-lower
(spine) if 16 channels is awkward. Nothing else changes.

## Per node

| Function | Part | Qty per node | ~Price | Notes |
| --- | --- | --- | --- | --- |
| MCU | ESP32-S3 DevKit (or ESP32-WROOM) | 1 | $4 | dual core, hardware CAN (TWAI), 20+ GPIO |
| CAN transceiver | SN65HVD230 or TJA1050 breakout | 1 | $1 | |
| PWM expansion | PCA9685 16-ch I2C | 1-2 | $2 | one 16-ch board drives 8 bidirectional channels (2 pins each) through the H-bridges |
| H-bridge, W-mini | DRV8833 (2 ch, 1.5 A each) | 1 per 2 mini | $1.5 | 6 V rail |
| H-bridge, W-mid | TB6612FNG (2 ch, 1.2 A cont / 3 A peak) | 1 per 2 mid | $2 | 12 V rail |
| H-bridge, W-max | BTS7960 (43 A) or IBT-2 module | 1 per max | $9 (checked 2026-10-28, `05`) | 12 V rail; current sense output used for stall detection |
| Joint angle sensor | AS5600 magnetic (12-bit, I2C) + 6x2.5 mm diametric magnet | 1 per DOF | $1 | fixed I2C address, so: |
| I2C multiplexer | TCA9548A (8 ch) | 1-2 | $1 | up to 16 AS5600s per node |
| Motor encoder | magnetic encoder PCB that ships with N20 / hall encoder on JGB37 | 1 per motor | included | velocity + slip detection only |
| Current sense | INA219 per rail or the H-bridge's own sense | 1-2 | $1 | |
| Buck converters | 12 V -> 6 V (5 A) for W-mini, 12 V -> 5 V (3 A) logic | 2 | $3 | |

Cost per node: about $40 in electronics for an arm node before motors. BLDC-based W-max
(if chosen) replaces the BTS7960 with a SimpleFOC mini driver (~$8) per motor.

## Control loops (firmware, identical on every node)

- 1 kHz inner loop per channel: PWM from a PI current/torque estimate (bus current or H-bridge
  sense), clamped to a per-channel hold limit.
- 200 Hz outer loop: PID on joint angle from the AS5600, feed-forward from the motor encoder.
- Modes per channel: `position` (go to angle), `tension` (hold tendon force, used for grasp and
  for pretension), `limp` (coast), `hold` (brake at current position, drive to zero current if
  the stage is self-locking).
- Protections: stall timeout, over-current, joint-sensor-vs-encoder mismatch (slip or broken
  line), temperature estimate from the current history (I^2 t), and limits from the hard stops.
- Homing: drive each tendon gently into its slack-side stop, zero the motor encoder, then move
  to mid-range and record the joint sensor value. Re-run on request or on mismatch.
- CAN protocol: one 8-byte frame per channel command, one per channel status at 50 Hz, node
  heartbeat, E-stop broadcast. Keep it dumb enough that a laptop with a $10 USB-CAN adapter can
  drive the whole robot from Python.

## Power

- 12 V main rail. Candidate sources: 3S Li-ion pack (3x 18650 x 4 parallel, ~130 Wh, with BMS),
  or a cordless-tool battery with a printed adapter (Ryobi 18 V / Makita 18 V through a buck, or
  a 12 V Milwaukee M12). Tool batteries are the home-friendly answer: charger, BMS and fuse are
  already solved.
- Budget: 25 W-mini x 0.5 A at 6 V worst case = 75 W if every finger stalls at once; 4 W-mid
  at 12 V x 2 A = 96 W; 21 W-max at 12 V x 5 A running = 1.3 kW if everything moves at once, which
  it never does. Plan for 300 W continuous, 600 W peak with a 40 A fuse, and sequence motion in
  firmware so no more than six W-max are accelerating simultaneously.
- Standing still costs nothing: self-locking stages hold pose with zero motor current.
- E-stop: hardware line that opens the main motor contactor (relay or MOSFET), logic stays up.

## Compute decision

| Phase | Compute |
| --- | --- |
| 0-3 (finger, hand, arm, torso) | ESP32 nodes + laptop over USB-CAN or Wi-Fi |
| 4 (head, camera, speech) | add one Pi 5 (8 GB) with a camera module and a USB-CAN adapter |
| 5+ (legs, balance) | the IMU and balance loop live on the leg ESP32s; the Pi stays a planner |

An RK1 on a Turing Pi carrier is a drop-in alternative to the Pi 5 if more NPU is wanted later;
nothing in the design depends on which.
