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
- **Sourced ranges (`24`, D-040).** The 775 draws about 66 A at stall (Kt 0.0095), not 25: holding the clutch
  limit takes 24 A (36 A at the pessimistic end), above what `21` sized the VNH5019 for. The JGB37-520 1:90 is
  1.1 N m stall (0.77 to 1.47), not 1.7, and is also sold with a 66 rpm winding: buy the 110 rpm one. At the
  pessimistic end the 775 on the printed 1:40 does not walk; it needs a gearbox of at least 0.55 efficiency.
- **W-max is still provisional** (D-006): the printed planetary must pass the Phase 2 rope-brake test before
  buying 21. Buy two 775s for that test first.

BLDC alternative for W-max (the D-006 fallback, back-drivable):

| Item | Part | Qty | Price | Total |
| --- | --- | --- | --- | --- |
| Motor | 5010 360 KV outrunner (multirotor class) | 21 | ~$24, checked | $500 |
| Driver | SimpleFOCMini | 21 | $8 to $10, checked | $190 |
| Encoder | AS5048A | 21 | ~$8, estimate | $170 |

It replaces the 775s, their bearings and the BTS7960s (about +$300 net). The SimpleFOCMini is a small
driver (a few amps); the ankles' ~65 W walking peak needs a larger board, so this table is a floor. `21` sizes it:
a 5010 on a printed 1:25 at 24 V with an ST B-G431B-ESC1 (40 A peak, 9.2 g, $19 to $39) per drive, 19.9 kg in all.
For the 775, `21` recommends a VNH5019-class driver (12 A / 30 A, 6.5 g) in place of the BTS7960 (-1.2 kg).

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
| PETG filament | any reputable 1 kg spools | about 13 kg (`20`: printed parts at 25 % infill + 15 %) | $145 to $200 |
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

**Superseded by the roll-up (`20`, `tools/mass_rollup.py`): about 26 kg as specified.** Printed parts 11.5 kg
at 25 % infill (5.8 kg of it from the generators, 5.7 kg estimated for parts with no generator), bought parts
14.6 kg. The W-max drive alone (775, BTS7960 with heatsink, bearings, printed gearbox) is 12.4 kg against the
9.5 kg above. The 13 kg figure that stood here was the generators' solid-PETG mass; printed, the joint ends
are 5.8 kg.

## Motor encoders and the drum sensor

The firmware's inner loop is a position loop on the gear output (D-021; `control()` in
`tools/sim_tendon.py`); the joint AS5600 (D-008) only trims its target. So the motor encoder is not made
redundant by the joint sensor. But a W-mini has no clutch (`02`): its drum turns with the gear output, so a
second AS5600 facing a magnet on the drum end reads the gear position directly (4096 counts per drum turn,
about 5 um of line on the 3 mm drum, absolute). That lets the W-minis be bare N20s without encoders (about
$1.90 each in 10-packs on Amazon instead of $14 to $20) for about $2.30 a drum in sensor and magnet, if
the winch bank (`cad/blender/winch_bank.py`) gets a sensor pocket at each drum end, which it has not yet. The
W-mid and W-max keep their encoders: their clutch sits between the gear and the drum.

## Amazon pricing, 2026-10-28

Priced line by line on amazon.com by a browser session (listed prices before tax and shipping; variant
listings at the price shown on the search page). Bulk packs up to about twice the need. **Build total
about $2,197** for lines 1 to 4, 6 to 8, 10 to 27 and 29 to 46, which includes items this doc's totals do
not (bench supply, wire, connector kit, crimpers, scales; 16 kg PETG).

| # | Item | Listing | Unit $ | Line $ | Note |
| --- | --- | --- | --- | --- | --- |
| 1 | N20 298:1, neck and wrist (8) | [GM12-N20 6V 1:298 hall encoder](https://www.amazon.com/dp/B07ZWW66BR) | 20.11 | 160.88 | no Amazon N20 states stall current or torque; Pololu #2218 direct is $199.60 for 8 and meets the spec |
| 2 | N20 100:1, fingers and face (20) | [Waveshare N20 metal gear, hall encoder](https://www.amazon.com/dp/B0CW1TCCTL) | 13.99 | 279.80 | stall torque not stated |
| 3 | JGB37-520 12 V 110 rpm (5) | [listing](https://www.amazon.com/dp/B0B17ZFQ44) | 16.01 | 80.05 | |
| 4 | 775 12 V 12 000 rpm ball bearing (23) | [listing](https://www.amazon.com/dp/B0D2RKP1F6) | 12.99 | 298.77 | the one listing that states 12 000 rpm at 12 V; the cheaper "12 to 24 V 12000 rpm" 2-packs are 12 000 at 24 V |
| 6 | DRV8833 (15) | [10-pack](https://www.amazon.com/dp/B0GX9F695H) + [5-pack](https://www.amazon.com/dp/B0GX94VQ5F) | 1.59 | 23.87 | |
| 7 | TB6612FNG (3) | [3-pack](https://www.amazon.com/dp/B0GX9G92DV) | 3.33 | 9.98 | |
| 8 | BTS7960 (24) | [4-pack](https://www.amazon.com/dp/B0FND3VTGK) x 6 | 5.75 | 137.94 | |
| 10 | XL4016 8 A buck (4) | [4-pack](https://www.amazon.com/dp/B0DSYXLKZT) | 3.84 | 15.36 | |
| 11 | 5 V 3 A buck (6) | [6-pack](https://www.amazon.com/dp/B0GYJDP96Q) | 1.50 | 8.99 | |
| 12 | INA219 (6) | [6-pack](https://www.amazon.com/dp/B0FG2CHL6R) | 2.13 | 12.79 | |
| 13 | Battery | [3S LiPo 10 Ah, 2-pack](https://www.amazon.com/dp/B09YM8P25C) | 60.00 | 119.99 | closest only: no prebuilt 3S Li-ion with a 40 A BMS; LiPo needs a balance charger. Build-your-own: [DALY 3S 40 A BMS](https://www.amazon.com/dp/B0CZQJCG5Z) $26.16 + cells |
| 14 | 12 V 30 A bench supply | [ALITOVE 360 W](https://www.amazon.com/dp/B06XJVYDDW) | 23.99 | 23.99 | |
| 15 | 40 A fuse + holder | [4-pack](https://www.amazon.com/dp/B08BWRSPY1) | 2.50 | 9.99 | |
| 16 | E-stop + relay | [latching E-stop](https://www.amazon.com/dp/B07LFXB8PF) + [200 A continuous relay](https://www.amazon.com/dp/B07T48QKQB) | | 22.58 | |
| 17 | Wire | [12 AWG](https://www.amazon.com/dp/B07CZZ11K2), [18 AWG](https://www.amazon.com/dp/B07HGTKQ89), [24 AWG](https://www.amazon.com/dp/B0C7TJG9YB) | | 58.12 | |
| 18 | JST-XH kit + crimper | [listing](https://www.amazon.com/dp/B0G2L3LGWD) | | 24.99 | |
| 19 | ESP32-S3 N16R8 (6) | [3-pack](https://www.amazon.com/dp/B0CGYXJB6Y) x 2 | 6.33 | 37.98 | |
| 20 | SN65HVD230 (6) | [6-pack](https://www.amazon.com/dp/B0GRXVCJ46) | 2.15 | 12.88 | |
| 21 | PCA9685 (8) | [listing](https://www.amazon.com/dp/B07RMTN4NZ) | 5.99 | 47.92 | |
| 22 | TCA9548A (12) | [6-pack](https://www.amazon.com/dp/B08JGSMRF9) x 2 | 1.67 | 19.98 | |
| 23 | AS5600 (60) | [10-pack](https://www.amazon.com/dp/B0D3WMK5MT) x 6 | 1.90 | 114.00 | magnet not included; avoid bare-chip "AS5600-ASOM SOP-8" listings |
| 24 | 6 x 2.5 mm diametric magnets (100) | [50-pack](https://www.amazon.com/dp/B0H1578MWG) x 2 | 0.42 | 41.96 | |
| 25 | BNO085 | [GY-BNO085](https://www.amazon.com/dp/B0CL26J81F) | 22.99 | 22.99 | |
| 26 | ICM-42688 (4) | [GY-601N1](https://www.amazon.com/dp/B0GCRW1NPQ) | 17.06 | 68.24 | -P variant not stated |
| 27 | FSR 402 (10) | [10-pack](https://www.amazon.com/dp/B0DGPMDJS1) | 3.11 | 31.08 | |
| 29 | PETG (16 kg) | [ELEGOO 4 kg](https://www.amazon.com/dp/B0D86BXVL6) x 4 | 9.00/kg | 143.96 | |
| 30 | TPU 95A 1 kg | [SUNLU](https://www.amazon.com/dp/B0BXNWK6NS) | | 20.99 | |
| 31 | 608-2RS (70) | [50](https://www.amazon.com/dp/B0GLVL76MN) + [20](https://www.amazon.com/dp/B0F83XSSP7) | 0.33 | 22.97 | |
| 32 | 625ZZ (50) | [25-pack](https://www.amazon.com/dp/B0CJFRSK7D) x 2 | 0.32 | 15.78 | |
| 33 | 6 mm BBs 0.20 g | [bag](https://www.amazon.com/dp/B0HGRWN3Z6) | | 9.99 | |
| 34 | 8 mm POM balls (100) | [uxcell](https://www.amazon.com/dp/B0FJ1SV5ZW) | 0.07 | 6.69 | |
| 35 | Rod and wire | [8 mm](https://www.amazon.com/dp/B0C2MKDWD7), [5 mm](https://www.amazon.com/dp/B0F32VCRFV) x 2, [spring wire kit](https://www.amazon.com/dp/B0B491L6LV) | | 54.96 | stainless spring wire, not music wire |
| 36 | M3 inserts (500) | [Yaocom](https://www.amazon.com/dp/B0F43D2TTH) | 0.03 | 16.99 | 4 mm long is fine (the generators' insert holes run deep); check the insert's outside diameter suits their 4.2 mm hole (`INSERT_D`, `chain.py`) |
| 37 | Screw kits | [M3](https://www.amazon.com/dp/B0FGV5FCBN) + [M2](https://www.amazon.com/dp/B0FG2CC91J) | | 19.98 | |
| 38 | Sheath 2 x 4 mm (30 m) | [PFA 10 m](https://www.amazon.com/dp/B0DRP6RCX7) x 3 | 1.24/m | 37.29 | PFA, not PTFE; true PTFE about $90 for 30 m |
| 39 | Piercing needles 16 G, 12 G | [16 G](https://www.amazon.com/dp/B0FL6NTDH2) + [12 G](https://www.amazon.com/dp/B0DDPL9YW6) | 0.10 | 19.98 | |
| 40 | 50 lb braid | [Reaction Tackle](https://www.amazon.com/dp/B09CW6L3PF) | | 20.98 | |
| 41 | 100 lb braid | [KastKing Superpower](https://www.amazon.com/dp/B0D1R1YGT6) | | 10.18 | 4-strand |
| 42 | 200 lb braid | [HERCULES 8-strand](https://www.amazon.com/dp/B0791D3SZN) | | 16.99 | base price; 200 lb variant may cost more |
| 43 | Crimp kit | [pliers + 500 sleeves](https://www.amazon.com/dp/B08PZ34MYZ) | | 29.99 | sleeve IDs not checked against 1.0 / 1.2 / 1.6 mm |
| 44 | M5 barrel adjusters (40) | [10-pack](https://www.amazon.com/dp/B07FDZYB9Z) x 4 | 0.70 | 27.96 | |
| 45 | Elastics | [silicone tube](https://www.amazon.com/dp/B08PTXMNCN) + [cord](https://www.amazon.com/dp/B0CBTRJV19) | | 16.18 | |
| 46 | Scales | [10 N spring](https://www.amazon.com/dp/B00VYNT6Q6) + [50 kg luggage](https://www.amazon.com/dp/B09PQB73JD) | | 19.97 | |
| 47 | Sole pads | silicone non-slip sheet, adhesive backed, 0.04 in, 5 x 40 in (the owner's pick, UPC 717018793922) | | ~10 | cut to the soles; friction 0.5 to 1.2 (D-044, docs/26); not in the total above |
| 48 | Pad tape | 3M 9448A double coated tissue tape (acrylic) | | | on the pad's adhesive side only, it does not stick to bare silicone; not priced, not in the total |

Options: 5010 360 KV x 21 [GoolRC](https://www.amazon.com/dp/B0C2YRNX1Z) $587.58; SimpleFOC Mini x 21
[listing](https://www.amazon.com/dp/B0DRV88Z16) $167.79. Phase 4: Pi 5 8 GB, camera, USB-CAN about $256
(the Pi listing is above MSRP). Pipe (line 47): home-improvement store.

Before buying:

1. **Measure an N20 first.** No Amazon N20 states stall current or torque, and the neck and wrist ratings
   depend on them (D-035). A bare 10-pack (6 V, 104 rpm, full metal, about $19) costs less than one
   encoder motor: lock the shaft at 6 V and read the current, hang a weight on a lever for the torque. Pass
   is about 1.5 A and 4 kg-cm (Pololu HP 298:1). Its stated 0.04 A no-load current points to a low-power
   winding (the HP draws 0.1 A), so expect it to fail; then Pololu for the neck and wrist.
2. **Battery:** a 3S LiPo with a balance charger, or a 3S Li-ion pack built with a 40 A BMS.
3. **The 775** listing above is the only one found stating 12 000 rpm at 12 V; buy two for the gearbox test
   first (D-006).

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
