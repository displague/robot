# 21 - W-max options: a small driver now, a brushless W-max for 20 kg

The mass roll-up (`20`) put the robot at 26.1 kg against 20, and the 21 W-max drives are 12.4 kg of it: each one a
775 motor (354 g), a BTS7960 with its heatsink (65 g), two 608s (24 g) and an estimated 150 g printed 1:40
planetary, drum and clutch. This is desk work on three ways to cut that: a driver without the heatsink, a 5010
brushless motor, and a lead screw in place of the gearbox and drum. Nothing in the model changes (`sim/robot.xml`
is as committed); the figures come from two tools:

```bash
python tools/wmax_options.py                     # drum drives side by side, and the lead-screw winch per joint
python tools/tendon_calc.py                      # now with 5010_PLAN16 / 5010_PLAN25_24V and two BLDC WALK rows
python tools/mass_rollup.py --stl <dir> --wmax-driver small     # 775 on a VNH5019-class driver
python tools/mass_rollup.py --stl <dir> --wmax bldc             # 5010 + B-G431B-ESC1 + printed 1:16
```

Fixed by the model: drum radius 15 mm, 200 lb braid (890 N), the clutch slips at a third of break (297 N on the
line), the firmware limits motor current to 90 % of that (267 N, D-021). The walking points of `07` as drum-side
line force and speed: ankle push-off 213 N at 312 mm/s (640 N at the joint through 3:1, 67 W), hip swing 210 N at
168 mm/s (35 W).

## First, the 775's current is probably 2.5 times what the model says

The model's 775 is 0.6 N m stall at 25 A and 12 000 rpm at 12 V. Those cannot all hold: a 12 000 rpm 12 V motor
has a speed constant of 12 V / 1257 rad/s, so Kt = 0.0095 N m/A, and 0.6 N m needs 63 A. The model's figures
imply Kt = 0.024. A reseller listing for a 12 000 rpm 775 gives about 50 A stall (and is itself inconsistent; no
manufacturer datasheet was found). With Kt 0.0095 and 50 A (0.48 N m, 0.24 ohm):

- the current at the firmware limit is **about 18 A, not 7 A** (plus 1 to 2 A no-load current);
- the continuous line force, at the same copper loss as the model's 7.5 A (27 W), is 161 N, not 288 N;
- the walking points still pass (the speed side of the motor is right), at 14 A during push-off.

The torque-speed line the simulation uses is unaffected (it is in torque and speed); the electrical energy figures
and the continuous rating are. One stall test on the bench (`bench-load-tests`) settles it. The driver choice below
is made to work either way.

## The drum drives side by side (`tools/wmax_options.py --drum`)

| Drive | Peak line N (motor) | Cont line N | No-load line mm/s | Ankle 213 N: mm/s (need 312) | Hip 210 N: mm/s (need 168) | W at the 267 N limit | A at the limit | A cont | g per drive |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 775 + 1:40, BTS7960 (model figures) | 960 | 288 | 471 | 367 | 368 | 91 | 7.0 | 7.5 | 593 |
| 775 + 1:40, VNH5019 (model figures) | 960 | 288 | 471 | 367 | 368 | 91 | 7.0 | 7.5 | 538 |
| 775 + 1:40, VNH5019 (Kt from 12 000 rpm) | 760 | 161 | 471 | 339 | 341 | 82 | 17.6 | 10.6 | 538 |
| 5010 + 1:16, B-G431B-ESC1, 12 V | 294 | 118 | 382 | 305 | 306 | 76 | 18.1 | 8.0 | 295 |
| 5010 + 1:25, B-G431B-ESC1, 24 V (6S) | 460 | 184 | 489 | 457 | 458 | 120 | 11.6 | 8.0 | 295 |

Peak line force is the motor's (stall for the 775, the driver's 20 A for the 5010) before the clutch, which caps
every row at 297 N. "W at the limit" is the mechanical power with the line at the firmware limit: the most the drive
gives in service, against 65 W at the ankle and 35 W at the hip. All rows clear the hip; the 12 V 5010 is 2 % short
of the ankle's 0.25 s push-off (0.26 s) and its continuous 118 N is below the standing ankle load of the 26 kg model
(119 to 146 N), so it needs either 24 V or more ratio.

## 1. Driver for the 775: a 10 to 15 A H-bridge, no heatsink

The BTS7960 module is 43 A with a 65 g heatsink. The firmware never asks for more than the current at the clutch
limit: 7 A on the model's figures, about 18 A for a second or so on the Kt-from-speed reading, and 8 to 11 A
continuous. Options:

| Module | Continuous / peak | Volts | Price | Mass | Notes |
| --- | --- | --- | --- | --- | --- |
| **Pololu VNH5019 carrier (#1451)** | 12 A / 30 A | 5.5 to 24 | $19.95 (one Pololu snapshot; another shows $29.95) | 6.5 g (Pololu) | current-sense output (~140 mV/A) for the firmware limit; over-current and thermal shutdown; no heatsink |
| Cytron MD10C | 13 A / 30 A for 10 s | 5 to 30 | about $13 to $15 (EU and Asia retailers) | ~18 g (one listing, unverified) | no current sense on the board; 75 x 43 mm |
| VNH2SP30 "mini" clone modules | 14 A / 30 A | 5.5 to 16 | $5 to $22 (retailer listings) | ~8 g, est | the chip is discontinued (ST, Pololu); clones of unknown origin; 16 V max is tight on a charged 3S (12.6 V) |

The VNH5019 is the choice: it is rated for the continuous current on either reading of the 775, its 30 A peak covers
the 18 A limit if the Kt-from-speed reading is right, and it has the current-sense output the D-021 limit needs (the
BTS7960 has one too; the MD10C does not). Its cost is price: 21 at $20 to $30 is $420 to $630 against $140 for 21
BTS7960s. The MD10C is the cheaper fallback with an external sense resistor or an INA219 per drive.

Mass (`mass_rollup.py --wmax-driver small`, 10 g per driver with headers and wires): **25.0 kg** against 26.1.

## 2. Brushless: a 5010 360 KV outrunner on a printed 1:16 or 1:25

`tendon_calc.py` now has `5010_PLAN16` and `5010_PLAN25_24V` (fields `kv`, `r_ohm`, `ratio` on `Motor`; for a
brushless motor the stall torque is the torque at the driver's peak current and the peak power is taken along the
voltage line at that current, not at half stall).

| Figure | Value | Source |
| --- | --- | --- |
| Mass | 109 g | listing (`05`), not weighed |
| Kt | 8.27 / KV = 0.023 N m/A | from KV |
| Resistance | 0.15 ohm line to line | **estimate**: the 5010 listings found give none (a T-Motor MN501-S 360 KV is 45 mohm, a smaller MN3510 188 mohm) |
| No-load speed | 0.9 x 360 x V: 3900 rpm at 12 V | from KV, 10 % FOC headroom |
| Peak current | 20 A, seconds (0.46 N m) | **estimate**; multirotor 5010s are sold for 20 to 40 A ESCs with propeller cooling |
| Continuous current | 8 A (0.18 N m), about 7 W of copper loss in a fan-less torso | **estimate**: measure the temperature rise |
| Gearbox | printed two-stage planetary, 1:16 (4 x 4) at 12 V or 1:25 (5 x 5) at 24 V, efficiency 0.6 as the 775's | ratio chosen here |

The 1:16 at 12 V puts the clutch limit (18 A) inside the 20 A peak and keeps the ankle's push-off speed, but its
continuous force is 118 N. The 1:25 at 24 V (a 6S battery) has the same no-load line speed as the 775 (489 mm/s),
184 N continuous, 120 W at the limit and needs 11.6 A for it. The 5010 is back-drivable through either: no
self-lock, so a standing joint holds on current (about 6 A at the ankle's 146 N on 1:25), which the 775's drum lock
in the model does not pay.

Drivers that carry 12 to 20 A on three phases (the SimpleFOC Mini is 3 A):

| Driver | Current | Volts | Price | Mass | Notes |
| --- | --- | --- | --- | --- | --- |
| **ST B-G431B-ESC1** | 40 A peak (tested with propeller cooling); continuous not stated | 3S to 6S (one listing: 11.1 to 22.2 V) | $19 to $39 (DigiKey listings disagree) | 9.2 g (ST) | STM32G431 on board, so it can run the drive's position loop and talk CAN; SimpleFOC supports it |
| MKS XDrive Mini (ODrive 3.6 clone), dual axis | 50 A per axis, claimed | 12 to 48 | ~$35 for two axes (one blog) | not found | one board per two W-max; ODrive firmware |
| ODrive Micro | 3.5 A continuous | | $89 | 6.8 g | too small, like the SimpleFOC Mini |

The B-G431B-ESC1 at 9.2 g with its own microcontroller is the per-drive choice; its continuous current without
airflow and its 6S maximum (a charged 6S is 25.2 V) are to verify. Mass per drive: 109 + 9.2 + 3 (AS5048A) + 24
(608s) + 150 (gearbox, est) = 295 g against 593 g.

Mass (`mass_rollup.py --wmax bldc`): **19.9 kg**, under the 20 kg budget; the pelvis at its model mass and the
thighs 0.7 kg under. Price: 21 x ($28 motor + $19 to $39 driver + $8 encoder) = about $1,150 to $1,600, against
about $410 for 775s and BTS7960s.

## 3. A lead screw in place of the gearbox and drum

The nut pulls the closed loop's two runs (one on each side of the nut, the far run around an idler), so the stroke
is the line excursion of the joint: tackle x pulley radius x range (`tools/wmax_options.py --screw`, from
`export_mjcf.JOINTS` and `cad/skeleton.json`).

| Screw on a 775 | Reduction | Line N at stall | No-load line mm/s | Ankle / hip mm/s (need 312 / 168) | Peak W at the nut | A for the 267 N limit (model / Kt from speed) | Holds unpowered |
| --- | --- | --- | --- | --- | --- | --- | --- |
| T8, 2 mm lead, brass nut, eta 0.35 | 1:8 | ~4000 to 5000 | 50 | 47 / 47 | 63 | 1.3 / 3.4 | yes, self-locking |
| SFU1204 ball screw, 4 mm lead, eta 0.9 | 1:2.4 | ~1500 to 1900 | 333 | 287 / 288 | 161 | 3.5 / 8.7 | no, back-drives |

A 775 cannot turn either screw directly: at 12 000 rpm a T8 nut slides at 5 m/s and the ball screw is at DN 144 000.
The reductions keep the T8 under ~1500 rpm (brass nut, est) and the ball screw under ~5000 rpm (DN ~60 000, est),
and the T8's then make it a slow posture drive (50 mm/s line), not a leg drive. Both would put several kN on a 890 N
line at stall with no clutch between, so the current limit becomes the only protection, at 1 to 9 A.

| Joint, axis | Tackle x pulley | Range | Stroke | T8 module length, mass | SFU1204 module length, mass |
| --- | --- | --- | --- | --- | --- |
| waist pitch / roll / yaw | 2 x 50 mm | 60 / 50 / 90 deg | 105 / 87 / 157 mm | 224 to 294 mm, 611 to 652 g | 244 to 314 mm, 727 to 795 g |
| shoulder pitch / roll / yaw | 2 x 40 mm | 215 / 195 / 160 deg | 300 / 272 / 223 mm | 360 to 437 mm, 690 to 735 g | 380 to 457 mm, 859 to 933 g |
| hip pitch | 2 x 60 mm | 125 deg | 262 mm | 399 mm, 713 g | 419 mm, 896 g |
| hip roll | 3 x 60 mm | 65 deg | 204 mm | 341 mm, 679 g | 361 mm, 840 g |
| hip yaw | 2 x 50 mm | 70 deg | 122 mm | 259 mm, 632 g | 279 mm, 761 g |
| knee | 2 x 50 mm | 135 deg | 236 mm | 373 mm, 698 g | 393 mm, 871 g |
| ankle pitch / roll | 3 x 50 mm | 60 / 35 deg | 157 / 92 mm | 294 / 229 mm, 652 / 614 g | 314 / 249 mm, 795 / 731 g |

Module length is inline (775 67 mm, coupler and bearing 40 mm, stroke, nut, end bearing 15 mm); mass is the 775,
a 10 g driver, the screw (T8 0.33 g/mm, SFU1204 0.72 g/mm, est), the nut (12 g, 95 g, est), two 608s, a printed
housing and nut guide (est) and the reduction (est). All 21: **14.1 kg (T8) and 17.5 kg (ball screw)** against
12.4 kg now. The screw removes the 150 g gearbox but adds more in steel, and the shoulder's 300 mm stroke makes a
module longer than the upper arm. Not a mass saving; the T8 is only worth it where a self-locking slow drive is
wanted, which the worm gearmotor (`20`, P-034) already does at 380 g with no screw.

## The roll-ups (`mass_rollup.py`, parts built fresh, 25 % infill)

| Option | Flags | Total kg | Over the 20 kg budget |
| --- | --- | --- | --- |
| 775 + BTS7960 (as specified) | none | 26.1 | +5.8 |
| 775 + VNH5019-class driver | `--wmax-driver small` | **25.0** | +4.7 |
| worm torso, 775 + BTS7960 elsewhere | `--torso-worm` | 25.0 | +4.7 |
| worm torso, 775 + VNH5019 elsewhere | `--torso-worm --wmax-driver small` | 24.4 | +4.1 |
| 5010 BLDC W-max | `--wmax bldc` | **19.9** | -0.4 |
| worm torso, 5010 elsewhere | `--torso-worm --wmax bldc` | 21.4 | +1.1 |
| 5010 BLDC, 15 % infill | `--wmax bldc --infill 0.15` | 19.0 | -1.3 |

(Over is against the model's 20.3 kg.) The worm torso and the brushless W-max do not combine: the worm gearmotor at
380 g is heavier than the 295 g brushless drive it would replace.

## Recommendation

1. **Now: the VNH5019 in place of the BTS7960** for every 775 W-max. It works on either reading of the 775's
   current, saves 1.2 kg, and costs $300 to $500 more for 21 (the MD10C is the cheaper fallback, without current
   sense). Before buying 21, stall one 775 on the bench and measure Kt: if it is 0.0095, the D-021 limit is 18 A and
   the continuous rating in `tendon_calc.py` (288 N) should come down to about 160 N.
2. **For 20 kg: the 5010 brushless W-max on a printed 1:25 and a 6S (24 V) battery**, one B-G431B-ESC1 per drive.
   It is the only option that reaches the budget (19.9 kg), it meets both walking points with margin and has 184 N
   continuous. It costs about $750 to $1,200 more than the 775 path, a 6S battery with a 12 V buck for the W-mid and
   worm loads, and a hold current (it is back-drivable). The 12 V, 1:16 version is the fallback if the battery stays
   3S: the same mass, 2 % short on the push-off and 118 N continuous.
3. **Not the lead screw**: heavier than the drive it replaces, too slow (T8) or not self-locking (ball screw), and no
   clutch.

## Not yet

- The 775's Kt and stall current, and the 5010's resistance, peak and continuous current: all from listings or
  estimates. The 5010's 109 g is the listing's; the GoolRC listing was not weighed.
- The B-G431B-ESC1's price (DigiKey listings at $19 and $39), its continuous current without airflow and its 6S
  ceiling; the MD10C's weight; the VNH2SP30 clones' prices.
- The printed 1:16 and 1:25 planetaries have no generator; 150 g each is the same guess as the 1:40.
- None of this is in the model: no `W-bldc` class, and the electrical energy of the 775 rows in the simulation uses
  the 25 A stall current.

## Next

Stall one 775 and one 5010 on the bench (`bench-load-tests`): torque on a lever, current, temperature rise at 8 A.
If the 775's Kt is 0.0095, correct `775_PLAN40` (stall current, continuous fraction) and re-run the gauntlet. Then a
`W-bldc` class in `export_mjcf.py` (back-drivable, no lock, current-limited) as a variant like `ROBOT_TORSO_WORM`, run
the gauntlet and the step grid on it at the 19.9 kg roll-up masses, and decide the W-max motor as a decision.
