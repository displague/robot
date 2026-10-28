# 05 - Parts, cost and mass (pre-test)

Every drive in this list is the one the tendon model (`tools/export_mjcf.py`, `tools/tendon_calc.py`) is built
from, so the line ratings and margins in docs 13 to 19 hold for the parts named here and not for weaker ones.
Prices were checked against live listings on 2026-10-28 where the **Price** column says *checked* (sources at
the end); the rest are the 2026-10-25 estimates, still to be checked. Update both from real purchases.

## Drives, counted from the model

| Region | DOF | Class | Motor in the model |
| --- | --- | --- | --- |
| Spine (pitch, roll, yaw) | 3 | W-max | 775 + printed 1:40 planetary |
| Shoulders (3 each) | 6 | W-max | " |
| Hips (3 each) | 6 | W-max | " |
| Knees | 2 | W-max | " |
| Ankles (pitch, roll) | 4 | W-max | " |
| Elbow and forearm (2 each) | 4 | W-mid | JGB37-520 1:90, 12 V, hall encoder |
| Neck (pitch, roll, yaw) | 3 | W-mini | N20 **298:1** HP 6 V |
| Wrists (pitch, roll) | 4 | W-mini | N20 **298:1** HP 6 V |
| Hands (7 each), face | 18 | W-mini | N20 100:1 6 V (D-005); not in the model yet |

21 W-max, 4 W-mid, 25 W-mini: the model's 32 winches plus the hands and face. The neck and wrist line
ratings (37 N continuous) come from the Pololu-class HP figures; a no-name N20 delivers about 60 to 70 % of
them (`tendon_calc.py`), which is why those seven are bought by name (D-035).

## Drives: recommended parts

| Item | Recommended part | Qty | Price | Total |
| --- | --- | --- | --- | --- |
| W-mini, neck and wrist | Pololu 298:1 Micro Metal Gearmotor HP 6V with extended motor shaft (#2218) | 7 + 1 spare | $24.95, checked | $200 |
| Encoders for the above | Pololu magnetic encoder pair kit for micro metal gearmotors (12 CPR) | 4 kits | ~$9, estimate | $36 |
| W-mini, hands and face | N20 100:1 6 V with magnetic encoder, no-name (budget); Pololu 100:1 HP 6V extended shaft (#2214) if the Phase 0 finger test is short of pinch | 18 + 2 spare | $4.5, estimate; Pololu $24.95, checked | $90 (Pololu: $499 + encoders) |
| W-mid | JGB37-520 12 V "110 rpm" (1:90) with hall encoder | 4 | $13 to $17, checked | $68 |
| W-max motor | 775 brushed, 12 V, ball bearing, **rated about 12 000 rpm at 12 V** | 21 | $13 to $40, checked; budget $15 | $315 |
| W-max gearbox bearings | printed 1:40 planetary (D-006), bearings and pins | 21 | $3, estimate | $63 |

Buying notes:

- **775 speed.** Many 775 listings are 6 000 rpm at 12 V and 12 000 rpm only at 24 V. The model assumes about
  12 000 rpm at 12 V (300 rpm at the drum after 1:40). A 6 000 rpm motor halves joint speed; run it on 24 V
  or re-run `tendon_calc.py` with its figures before buying 21.
- **Not hobby N20s for the neck and wrist.** Retail "N20 with magnetic encoder" motors (the Adafruit family,
  for example) draw about 0.2 A at stall, against 1.6 A for the HP motor the model uses: a fraction of the
  torque.
- **W-max is still provisional** (D-006): the printed planetary must pass the Phase 2 rope-brake test before
  buying 21. Buy two 775s for that test first.

BLDC alternative for W-max (the D-006 fallback, back-drivable):

| Item | Part | Qty | Price | Total |
| --- | --- | --- | --- | --- |
| Motor | 5010 360 KV outrunner (multirotor class) | 21 | ~$24, checked | $500 |
| Driver | SimpleFOCMini | 21 | $8 to $10, checked | $190 |
| Encoder | AS5048A | 21 | ~$8, estimate | $170 |

It replaces the 775s, their bearings and the BTS7960s (about +$300 net). The SimpleFOCMini is a small
driver (a few amps); the ankles' ~65 W walking peak needs a larger board, so this table is a floor.

## Electronics

| Item | Recommended part | Qty | Price | Total |
| --- | --- | --- | --- | --- |
| Node MCU | ESP32-S3 DevKitC | 6 (5 + spare) | ~$4, estimate | $25 |
| CAN, I2C mux, PWM | SN65HVD230, TCA9548A, PCA9685 breakouts | ~15 boards | estimate | $25 |
| W-mini drivers | DRV8833 | 13 | estimate | $20 |
| W-mid drivers | TB6612FNG | 2 | estimate | $4 |
| W-max drivers | BTS7960 43 A module (IBT-2) | 21 | $8 to $10, checked | $190 |
| Joint angle | AS5600 module with diametric magnet | 50 | $2.70 each in 10-packs + shipping, checked | $135 to $205 |
| Pelvis IMU | Adafruit BNO085 (#4754): fusion on the chip | 1 | $29.50, checked | $30 |
| Torso and foot IMUs | ICM-42688-P breakout, fusion on the ESP32 | 3 | ~$8, estimate | $25 |
| Foot load | Interlink FSR 402 under a TPU pad, 4 per foot | 8 + 2 spare | $2.99, checked | $30 |
| Power | bucks (12 to 6 V 5 A, 12 to 5 V 3 A), current sense, 40 A fuse, E-stop, JST-XH wiring | | estimate | $60 |
| Battery | 18 V tool battery + printed adapter + buck, or 3S4P 18650 with BMS | 1 | estimate | $60 to $90 |
| Brain (Phase 4) | Pi 5 8 GB + camera + USB-CAN | 1 | estimate | $130 |

The foot load sensors and IMUs were in `07` but not in this list before; the balance controller of docs
14 to 19 reads both.

## Mechanical

| Item | Recommended part | Qty | Total |
| --- | --- | --- | --- |
| PETG filament | any reputable 1 kg spools | 10 to 12 kg | $200 |
| TPU filament | 95A | 1 kg | $25 |
| Bones (D-019) | Schedule 40 PVC or ABS DWV: 1" (humerus, forearm, neck), 1-1/4" (tibia), 1-1/2" (femur), 2" (spine) | ~2.5 m | $30, estimate |
| Ring-joint balls (D-018) | 6 mm and 8 mm airsoft BBs (seamless) | ~200 | $15, estimate |
| Bearings | 608 (60), 625 and small (40) | 100 | $30 |
| Rod and wire | 8 mm steel rod, 5 mm rod, 2 mm music wire | | $25 |
| Fasteners | M3 heat-set inserts and screws, M2 screws | ~1000 | $40 |
| Sheath | PTFE tube 2 mm ID / 4 mm OD | 30 m | $20 |
| Channel liners (D-013) | hollow piercing needles, 16 G and 12 G | 200 | $26 |
| Line (D-002) | braided PE, 50, 100 and 200 lb (PowerPro Spectra class); **verify the break rating** against the model's 222 / 445 / 890 N with a spring scale | 3 spools | $60, estimate |
| Terminations (D-014) | fishing crimp sleeves sized to each line, crimp pliers | 300 | $15, estimate |
| Pretension (D-014) | bicycle barrel adjusters (M5) | 32 + spares | $20, estimate |
| Finger elastics | silicone tubing / elastic cord | | $10 |

## Totals

| Build | Drives | Electronics (no Pi) | Mechanical | Total |
| --- | --- | --- | --- | --- |
| **Recommended**: Pololu neck and wrist, no-name N20 hands and face, 775 W-max | $770 | $650 | $520 | **~$1,940** |
| Budget: every W-mini a no-name N20 (neck and wrist below the model's ratings) | $580 | $650 | $520 | ~$1,750 |
| Every W-mini Pololu HP with encoders | $1,270 | $650 | $520 | ~$2,440 |
| Recommended with BLDC W-max (no BTS7960s) | $1,250 | $460 | $520 | ~$2,230 |
| Add the Pi 5 brain to any row | | +$130 | | |
| Upper body only (hands, arms, torso, head; 9 W-max; no legs, no Pi, no foot sensors) | $560 | $430 | $400 | ~$1,390 |

The 2026-10-25 estimate was ~$1,070. The difference is real prices for the BTS7960s and AS5600s, the
sensors and bones that were missing, and buying the neck and wrist motors by name.

## Mass

Full body, 775 option: 9.5 kg W-max + 1.6 kg mini/mid + 6 kg printed structure + 1 kg
bearings/hardware + 1 kg battery + 0.5 kg electronics = **~20 kg**. On budget, with no margin.
Shaving the structure to 5 kg and the W-max count to 17 (fixed ankle inversion, fixed spine
lateral) is the first thing to try if the first leg comes out heavy.

## Sources (checked 2026-10-28)

- Pololu 298:1 HP 6V: [#994](https://www.pololu.com/product/994/specs), [#2218 extended shaft](https://pololu.com/product/2218/resources); [100:1 HP #2214](https://www.pololu.com/product/2214/pictures)
- N20 hobby motor with encoder: [Jameco / Adafruit 1:100](https://www.jameco.com/z/4639-Adafruit-Industries-N20-DC-Motor-with-Magnetic-Encoder-6V-with-1-100-Gear-Ratio_2519863.html)
- JGB37-520: [eBay](https://www.ebay.com/itm/325349454260), [Makers Electronics](https://makerselectronics.com/?p=531484)
- 775: [Phipps Electronics](https://www.phippselectronics.com/product/775-12v-24v-double-ball-bearing-high-speed-dc-motor/), [eBay](https://www.ebay.com/itm/126581133785)
- BTS7960: [Walmart](https://www.walmart.com/ip/BTS7960-43A-High-Power-Motor-Driver-Module-Smart-Car-Driver-Module-for-Current-Limit/8904873796)
- AS5600 10-pack: [eBay](https://www.ebay.com/itm/10pcs-AS5600-Magnetic-Encoder-Sensor-Module-12bit-Precision-for-Arduino/174320770372)
- BNO085: [Adafruit #4754](https://www.adafruit.com/products/4754)
- FSR 402: [Interlink](https://www.interlinkelectronics.com/products/fsr-model-402)
- SimpleFOCMini: [Walmart](https://www.walmart.com/ip/SimpleFOCMini-Brushless-DC-Motor-Driver-Board-FOC-Control-Driver-SVPWM-Control-Algorithm-Driver-Board-Module/13474754822); 5010 360 KV: [Newegg](https://www.newegg.com/p/1W7-00TR-029V9)
