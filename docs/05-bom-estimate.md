# 05 - Cost and mass estimate (rough, pre-test)

Prices are typical AliExpress / Amazon-bulk, October 2026, rounded. Masses are estimates for
the 20 kg budget. Update both columns from real purchases.

## Actuators

| Item | Qty | Unit | Total | Mass each | Mass total |
| --- | --- | --- | --- | --- | --- |
| N20 gearmotor 100:1 with encoder (W-mini) | 25 (+5 spare) | $4.5 | $135 | 25 g with printed housing | 0.6 kg |
| JGB37-520 encoder motor 1:90 (W-mid) | 4 | $12 | $48 | 260 g with housing | 1.0 kg |
| W-max motor, wiper-motor option | 21 | $20 | $420 | 1.0 kg | 21 kg (**over budget**) |
| W-max motor, 775 + printed planetary option | 21 | $8 + $3 bearings | $230 | 0.45 kg | 9.5 kg |
| W-max motor, 2806 BLDC + cycloid + AS5048 option | 21 | $20 + $8 driver | $590 | 0.2 kg | 4.2 kg |

W-max is the single largest cost and mass decision in the project. The 775 + printed planetary
route is the one that fits the budget and the mass target; it is also the one that needs the
most testing (printed gears at 100 W). See decision D-006.

## Electronics

| Item | Qty | Total |
| --- | --- | --- |
| ESP32-S3 dev boards | 6 (5 + spare) | $25 |
| CAN transceivers, I2C muxes, PCA9685 | ~15 boards | $25 |
| DRV8833 / TB6612 / BTS7960 drivers | 13 / 2 / 21 | $20 / $4 / $85 |
| AS5600 + magnets | 50 | $50 |
| Bucks, current sensors, fuses, E-stop, wiring, connectors (JST-XH) | | $60 |
| Battery: 3S4P 18650 pack with BMS, or tool-battery adapter + charger | 1 | $60-90 |
| Pi 5 8 GB + camera + USB-CAN (Phase 4) | 1 | $130 |

## Mechanical

| Item | Qty | Total |
| --- | --- | --- |
| PETG filament | 10-12 kg | $200 |
| TPU filament | 1 kg | $25 |
| 608 bearings | 60 | $15 |
| 625 bearings, misc small bearings | 40 | $15 |
| 8 mm steel rod, 5 mm rod, 2 mm music wire | | $25 |
| M3 inserts and screws, M2 screws | ~1000 | $40 |
| PTFE tube 2/4 mm | 30 m | $20 |
| Braided PE line 50/100/200 lb | 3 spools | $30 |
| Silicone tubing / elastics | | $10 |

## Totals

| Build | Electronics + mechanical | Actuators | Total |
| --- | --- | --- | --- |
| Upper body only (hands, arms, torso, head; no legs, no Pi) | ~$600 | ~$300 (25 mini, 4 mid, 9 max as 775) | **~$900** |
| Full body, 775-planetary W-max, no Pi | ~$650 | ~$415 | **~$1,070** |
| Full body with Pi 5 brain | | | **~$1,200** |
| Full body, BLDC W-max | | ~$770 | ~$1,450 |

Mass, full body, 775 option: 9.5 kg W-max + 1.6 kg mini/mid + 6 kg printed structure + 1 kg
bearings/hardware + 1 kg battery + 0.5 kg electronics = **~20 kg**. On budget, with no margin.
Shaving the structure to 5 kg and the W-max count to 17 (fixed ankle inversion, fixed spine
lateral) is the first thing to try if the first leg comes out heavy.
