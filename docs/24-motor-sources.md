# 24 - Motor and line figures from sources, and the model at the pessimistic end (D-040, D-041)

The owner will not buy motors before the outcome is reasonably certain and cannot run bench tests (P-038,
D-040). So every drive and line figure the model uses now has a low, a nominal and a high value, each with a
source or a stated derivation, in `sim/motor_ranges.json`. The model runs on any end of those ranges:

```bash
python tools/tendon_calc.py --ranges              # each winch class: model figures, then low / nom / high
ROBOT_MOTORS=low python tools/sim_tendon.py       # the gauntlet with every drive and line at its pessimistic end
ROBOT_MOTORS=low python tools/step_sweep.py --scene walk --masses 20kg=,775=sim/masses/rollup_775.json,worm=sim/masses/rollup_worm.json --lengths 0.085,0.09,0.095,0.1,0.105,0.11,0.115 "low: "
ROBOT_MOTORS=<file.json>[:case] ...                # a file in the same format (mixed cases, a new part)
```

`low` is the pessimistic end of every figure at once: least torque, speed, gearbox efficiency, continuous rating
and line strength, and the most amps per newton metre. `nom` is the best sourced estimate. Unset, the model runs
on `tendon_calc.py`'s own figures, which are what `sim/robot.xml` is (unchanged by this work); they are not the
same as `nom`, see below. A case changes the parts, not the builder's settings: the pretension, the clutch's slip
force (a third of the spool label) and the line stiffness stay where the model has them (D-041).

Research was done from the web on 2026-10-29. Measured data (dynamometer curves, FRC motor sheets, Instron line
tests) was preferred over listings. Nothing here was tested by us.

## The 775 (W-max): Kt is 0.0095 N m/A, and the stall current is about 66 A, not 25

`21` found the model's 775 inconsistent. Settled: Kt follows from the speed. A 12 000 rpm motor on 12 V has
Kv = 1000 rpm/V, Kt = 60 / (2 pi Kv) = **0.0095 N m/A**. On nine 775-class sheets that give speed, stall torque and
stall current together, the stall-point Kt is 0.86 to 1.05 times the speed one:

| Motor | No-load rpm / A at 12 V | Stall N m / A | Kt from speed | Kt from stall | Km (N m/sqrt W) | Source |
| --- | --- | --- | --- | --- | --- | --- |
| VEX 775pro | 18 730 / 0.7 | 0.71 / 134 | 0.0061 | 0.0053 | 0.018 | [FRC sheet](https://www.chiefdelphi.com/uploads/default/original/3X/0/2/02a0dc9f352a3d7bf4d126012baef9d037ee2393.pdf) |
| AndyMark RedLine | 21 020 / 3.8 | 0.70 / 130 | 0.0055 | 0.0056 | 0.018 | [AndyMark](https://cdn.andymark.com/am-3775a) |
| Banebots RS775-18 at 12 V | 13 000 / 1.8 | 0.80 / 86.7 | 0.0088 | 0.0092 | 0.025 | [2012 FRC motor info](https://wiki.penfieldrobotics.com/wiki/images/1/10/2012_MotorInfo.pdf), [Banebots](https://banebots.com/m7-rs775-18/) |
| Banebots RS775-12 | 7 300 to 8 700 / 1.1 | 0.43 / 30 | 0.013 to 0.016 | 0.0149 | 0.021 to 0.025 | [Banebots](https://banebots.com/m7-rs775-12/) |
| Kinmore RS-775PH-8011R | 17 500 / 2.0 | 0.78 / 123 | 0.0066 | 0.0064 | 0.021 | [Kinmore](https://kinmore.com/products/rs-770-775h/) |
| Kinmore RS-770PHF-8523R | 15 000 / 2.0 | 0.93 / 126 | 0.0076 | 0.0075 | 0.024 | same |
| Techtonics 775 | 7 000 / 0.8 | 0.54 / 36 | 0.0164 | 0.0153 | 0.027 | [Techtonics](https://techtonics.in/product/rs-775-motor-7000-rpm-12v-high-torque/) |
| NFP 775SHP-4538 | 5 000 / 0.6 | 0.34 / 18 | 0.0229 | 0.0197 | 0.024 | [NFP](https://nfpshop.com/product/42mm-12v-18v-24v-ball-bearing-775-performance-motor-used-for-robotics-quadcopter-industrials) |
| Handsontec 775, 12 000 rpm | 12 000 / 1.2 | 0.66 (0.79 at 14.4 V) / not given | 0.0095 | | 0.023 | [Handsontec](https://www.handsontec.com/dataspecs/motor_fan/775-Motor.pdf) |

With Kt fixed by the speed, the motor constant Km = Kt / sqrt(R) of these sheets (0.018 to 0.028) fixes the
resistance (0.28 / 0.18 / 0.14 ohm), the stall current 12 V / R (45 / 66 / 87 A) and the stall torque
Kt (I - I0) (0.40 / 0.62 / 0.80 N m). The model's 0.6 N m is right; its 25 A is not. **The current for any torque
is 2.6 times the model's**, so every electrical energy figure in docs 13 to 23 is low by about that factor, and
holding the line at the clutch limit takes 24 A, not 7.7. No source rates a cheap 775 for continuous current: a
stalled 775pro tolerates 11 to 25 W of copper loss ([Chief Delphi](https://www.chiefdelphi.com/t/stall-testing-775-pro-surprisingly-poor-result/166538)),
which at 0.18 ohm is 8 to 12 A, and 15 A with the fan turning: 7 / 10 / 15 A. A 6000 rpm 775 (the common 12 V
listing) is Kt 0.019, 0.41 N m, 22 A: half the speed, as `05` warns.

## Tables per part

Torque and speed are at the gearbox output; "drum" is stall torque times the gearbox efficiency the model applies.

### W-max: 775 at 12 V + printed two-stage 1:40 planetary (`775_PLAN40`)

| Figure | Low | Nom | High | Model | Source |
| --- | --- | --- | --- | --- | --- |
| Motor stall torque | 0.40 N m | 0.62 | 0.80 | 0.60 | Kt and Km, above |
| x 40 | 16.0 N m | 24.8 | 32.0 | 24.0 | |
| No-load speed | 270 rpm | 300 | 325 | 300 | Handsontec 12 000; RS775-18 13 000 at 12 V; -10 % tolerance (AndyMark) |
| Stall current | 45 A | 66 | 87 | 25 | 12 V / R |
| Continuous current | 7 A | 10 | 15 | 7.5 (0.3 of stall) | copper loss, above |
| Printed planetary efficiency | 0.35 | 0.50 | 0.65 | 0.60 | [COMPAct dyno](https://arxiv.org/html/2510.07197v2): one PLA stage 0.60 to 0.80, a 14:1 compound about 0.60; [a two-stage 16:1](https://github.com/jagathsrujan/3d-printable-planetary-gearbox) about 0.50; two stages of 0.6 to 0.8 are 0.36 to 0.64. Informal stall tests say 0.78 to 0.95 ([HowToMechatronics](https://howtomechatronics.com/how-it-works/what-is-the-best-3d-printed-drive-for-your-next-robotic-project/)) |
| Drum stall torque | **5.6 N m** | 12.4 | 20.8 | 14.4 | |
| Mass | 366 g | 350 | 337 | 354 | RedLine, Handsontec, RS775-18 |

### W-mid: JGB37-520 12 V 1:90, "110 rpm" (`JGB37_1:90`)

| Figure | Low | Nom | High | Model | Source |
| --- | --- | --- | --- | --- | --- |
| Stall torque (output, gearbox loss in it) | 0.77 N m | 1.10 | 1.47 | 1.7 x 0.75 = 1.28 | [Hiwonder](https://www.hiwonder.com/products/hall-encoder-dc-geared-motor) 15 kg cm; [NFP](https://precisionminidrives.com/product/37mm-diameter-miniature-dc-gear-motor-nfp-jgb37-520-dc-6v-12v-24v) 12 kg cm scaled with ratio (no loss), x 0.65 for the gearbox at low |
| No-load speed | 94 rpm | 110 | 127 | 100 | NFP, +-15 % |
| Stall current | 3.2 A | 2.4 | 2.3 | 3.5 | Hiwonder, NFP, [RAM](https://www.ram-e-shop.com/shop/dc-jgb37-12v-set-110rpm-jgb37-520-dc-geared-motor-12vdc-110-rpm-complete-set-with-encoder-9326) |
| Rated (continuous) torque | 0.255 N m | 0.27 | 0.294 | 0.38 | Hiwonder 2.6, NFP 3.0 kg cm |
| Gearbox efficiency | 0.55 | 0.65 | 0.80 | 0.75 | derived: a [supplier](https://rcdrone.top/products/jgb37-520-dc-gear-motor) stall table across ratios; [Maxon](https://www.maxongroup.com/maxon/view/product/143767) spur gearheads 0.59 to 0.66 at 4 to 5 stages. Folded into the stall torque (eta 1.0 in the JSON) |
| Mass | 170 g | 160 | 144 | 200 | NFP 144 to 170, [Yahboom](https://category.yahboom.net/products/md520) 150; Hiwonder's 200 is shipping weight |

The same name is sold with a 6000 rpm winding: 66 rpm and 8 kg cm at 1:90 ([OpenImpulse](https://www.openimpulse.com/blog/jgb37-520-gearmotor-selector/)). Buy the 110 rpm one.

### W-mini: Pololu 298:1 Micro Metal Gearmotor HP 6 V (`N20_298:1`)

| Figure | Low | Nom | High | Model | Source |
| --- | --- | --- | --- | --- | --- |
| Stall torque (output, measured with the gearbox) | 0.31 N m | 0.39 | 0.49 | 0.49 x 0.75 = 0.37 | [Pololu datasheet rev 6.2](https://www.pololu.com/file/0J1487/pololu-micro-metal-gearmotors-rev-6-2.pdf) 40 kg mm (dynamometer, extrapolated); 0.49 is the pre-2020 70 oz in still on [resellers](https://core-electronics.com.au/298-1-micro-metal-gearmotor-hp.html); low 0.8 x (Pololu: units stall well before the extrapolated point as they heat) |
| No-load speed | 80 rpm | 100 | 120 | 100 | [Pololu](https://www.pololu.com/product/2218/specs), +-20 % |
| Stall current | 1.6 A | 1.6 | 1.6 | 1.6 | Pololu |
| Continuous | 0.064 N m | 0.088 | 0.147 | 0.11 | Pololu advises at most 25 % of stall current (0.4 A, 9 kg mm on the measured line); low the max-efficiency point |
| Gearbox limit | 0.196 N m | | | | [Pololu](https://www.pololu.com/category/60/micro-metal-gearmotors): 20 kg mm, instantaneous, "keep loads well under" |
| Mass | 10 g | 9.5 | 9.5 | 10 | Pololu |

The 298:1 gearbox alone is about 0.5 to 0.6 efficient (Pololu's torque per amp across ratios, our derivation; the
whole gearmotor peaks at 31 %); the model's 0.75 applied to the old 0.49 lands near the new measured 0.39 by luck.
No measured comparison of a no-name N20 298:1 against Pololu was found; vendor claims are 33 to 75 % of its stall.

### W-worm: 12 V 10 rpm 1:634 worm gearmotor, 5840-31ZY class (`WORM_10RPM`)

| Figure | Low | Nom | High | Model | Source |
| --- | --- | --- | --- | --- | --- |
| Stall torque (output) | 3.92 N m | 5.4 | 6.9 | 7.84 | gear-limited: [NFP 31ZY-S](https://nfpmotor.com/6v-12v-24v-dc-single-output-shaft-reversible-electric-gear-motor-model-nfp-5840-31zy-s) caps stall at 40 kg cm, [NFP 31ZY-DSH](https://nfpmotor.com/5840-gear-motor-of-high-torque-with-self-locking-double-shafts-model-nfp-5840-31zy-dsh) 70 kg cm |
| Rated torque | 3.92 N m at 1.4 A | | | | [Greartisan](https://www.amazon.com/clp/B07YBXMTWC) 40 kg cm, +-10 % current |
| No-load speed | 8.5 rpm | 10 | 11.5 | 10 | +-15 % (NFP) |
| Current at stall torque | 1.55 A | 1.8 | 2.25 | 2.7 | Greartisan's line, 0.25 A + 35 kg cm per A |
| Mass | 380 g | 370 | 360 | 380 | NFP 360, Amazon 13.1 oz |

The 31ZY motor through 1:634 could push 125 to 220 kg cm; the gear teeth are the limit, so "twice rated" (the
model's 7.84 N m) has no source. Overall efficiency at the rated point is about 22 % (3.7 W out, 16.8 W in);
a steel-on-plastic worm stage alone is 0.26 to 0.41 by tan(lead) / tan(lead + friction angle).

### Brushless candidates: 5010 360 KV + printed 1:16 (12 V) or 1:25 (24 V)

| Figure | Low | Nom | High | Model | Source |
| --- | --- | --- | --- | --- | --- |
| Kv | 324 | 360 | 396 | 360 | label, +-10 % |
| Kt per amp of phase amplitude (8.27 / Kv) | 0.0209 | 0.0230 | 0.0255 | 0.023 | [ODrive](https://discourse.odriverobotics.com/t/question-about-torque-calculation/4295); 9.55 / Kv is per DC amp |
| Resistance, line to line | 0.35 ohm | 0.26 | 0.20 | 0.15 | 0.24 measured on a generic 5010 360 KV ([SimpleFOC forum](https://community.simplefoc.com/t/what-current-and-voltage-limits-to-use-for-this-motor/4245)); [RCTimer](https://rctimer.com/rctimer-5010-260kv-multirotor-brushless-motor40mm-shaft-p0132.html) siblings scaled |
| Peak current (driver) | 20 A | 20 | 20 | 20 | a firmware choice; the motors are sold for 20 to 40 A with propeller air |
| Continuous, no airflow | 3 A | 6 | 9 | 8 | SimpleFOC forum guesses 3 A; 6 and 9 from an assumed 4 to 5 K/W (no source) |
| Stall at 20 A, x 16 / x 25 | 6.7 / 10.5 N m | 7.4 / 11.5 | 8.2 / 12.8 | 7.4 / 11.5 | |
| Mass | 100 g | 92 | 80 | 109 | RCTimer, rcshopbd listings |

"5010 360 KV" is two motors: the generic 12N14P (92 g, 0.24 ohm) and the premium 24N28P such as the T-Motor
MN501-S (175 g, 0.045 ohm, [ligpower](https://www.ligpower.com/product/mn501-s-kv360-motor-navigator-type.html)),
which has about 2.4 times the torque per square root watt. The ranges are the generic one's; the model's 0.15
ohm was between the two.

### Line: braided PE (`braid_50lb`, `braid_100lb`, `braid_200lb`)

| Figure | Low | Nom | High | Model | Source |
| --- | --- | --- | --- | --- | --- |
| Straight pull / label | 0.85 | 1.0 | 1.25 | 1.0 | branded braids 1.04 to 2.0 at 20 lb, 1.12 to 1.26 at 40 to 50 lb ([Sport Fishing, Instron](https://www.sportfishingmag.com/?p=46781), [TackleTour](https://tackletour.com/reviewsufix832braid.html)); a cheap "50 lb" 0.86 ([NESA](https://nesa.co.uk/threads/cheap-braid-test.152946)); no tests at 100 or 200 lb |
| Termination, wrapped three turns before the crimp | 0.85 | 0.9 | 0.9 | 1.0 | capstan: 6 to 15 % of the tension reaches the crimp (friction assumed 0.1 to 0.15); HMPE keeps its strength bent at D/d above 8 ([RYA](https://www.rya.org.uk/knowledge/safety/look-after-yourself/hmpe-rope)); our drums are 17 to 40 |
| Effective break, 50 / 100 / 200 lb | 160 / 320 / 641 N | 200 / 400 / 801 | 244 / 490 / 979 | 222 / 445 / 890 | product of the two |
| Knot or crimp carrying the full load | 0.3 x label | 0.5 | 0.75 | | palomar, uni 0.31 to 0.79 ([Sport Fishing](https://www.sportfishingmag.com/best-fishing-knots-main-line-to-terminal-gear/)); overhand 0.49 ([TackleTour](https://www.tackletour.com/reviewsufix832braidpg2.html)); crimps on bare braid slip, no numbers |
| Strain at a quarter of break | 0.7 % | 1.0 | 2.0 | 2.0 | [FishTalk](https://fishtalkmag.com/blog/fishing-line-stretch-test-stretching-truth): PowerPro 0.7 to 1.3 % at a third of label |

A knot carrying the load at 0.3 of label breaks below the clutch (a third of label), so the lines are anchored by
wraps (D-041). The model's line is softer than the sources (2 % against about 1 %); left as it is.

## The line rating per winch class (`tendon_calc.py --ranges`)

The clutch slips at a third of the label, or 0.95 of the motor's stall line force if that is lower. SF at the
clutch is the effective break over the clutch force: the most the line can see in service.

| Class | Case | Drum stall N m | Line stall N | Continuous N | Clutch N | Break N | SF at clutch | No-load mm/s | A at the clutch |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| W-max | model | 14.4 | 960 | 288 | 297 | 890 | 3.0 | 471 | 7.7 |
| | **low** | 5.6 | 373 | **47** | 297 | 641 | **2.2** | 424 | **35.8** |
| | nom | 12.4 | 827 | 107 | 297 | 801 | 2.7 | 471 | 23.7 |
| | high | 20.8 | 1387 | 222 | 297 | 979 | 3.3 | 511 | 18.6 |
| W-mid | model | 1.28 | 212 | 64 | 148 | 445 | 3.0 | 63 | 2.4 |
| | **low** | 0.77 | 128 | **42** | 122 | 320 | **2.6** | 59 | 3.0 |
| | nom | 1.10 | 183 | 45 | 148 | 400 | 2.7 | 69 | 1.9 |
| W-mini | model | 0.37 | 122 | 37 | 74 | 222 | 3.0 | 31 | 1.0 |
| | **low** | 0.31 | 103 | **17** | 74 | 160 | **2.2** | 25 | 1.1 |
| | nom | 0.39 | 130 | 30 | 74 | 200 | 2.7 | 31 | 0.9 |
| W-worm | model | 7.84 | 523 | 261 | 297 | 890 | 3.0 | 16 | 1.5 |
| | **low** | 3.92 | 261 | **196** | 248 | 641 | **2.6** | 13 | 1.5 |
| | nom | 5.4 | 360 | 263 | 297 | 801 | 2.7 | 16 | 1.5 |

What the pessimistic end means:

- **W-max**: 47 N continuous against 288. Its stall line force (373 N) still clears the clutch, but the drive's
  power is 40 W at the drum against 113 W: the ankle push-off point of `07` (213 N) runs at 182 mm/s against the
  312 needed, the hip swing (210 N) at 186 against 168. Holding the line at the clutch takes 36 A, more than the
  VNH5019's 30 A peak that `21` chose; at nom it is 24 A.
- **W-mid**: the elbow loses 40 % of its stall, the continuous rating is 42 N against 64.
- **W-mini**: 17 N continuous against 37. Its clutch at 74 N on the 3 mm drum is 0.222 N m at the gearbox, above
  Pololu's 0.196 N m gear limit in every case: the W-mini clutch should slip at about 59 N (0.9 of the limit).
- **Lines**: with wrapped anchors no class falls below SF 2.2 at the clutch. With a knot taking the load it would
  be 0.9 (W-max low): the line breaks before the clutch slips.

## The model across the range

### The gauntlet (`ROBOT_MOTORS`, 20 kg skeleton masses)

| Scene | Model figures | Sourced nom | Low |
| --- | --- | --- | --- |
| stand 4 s | stands, 12 J | stands, 38 J | stands, 79 J |
| push 10 N | stands, 19 J | stands, 58 J | stands, 161 J |
| push 20 N | stands, 24 J | stands, 70 J | stands, 204 J |
| push 30 N | stands, 26 J | stands, 83 J | stands, 241 J |
| push 40 N | stands, 42 J | stands, 143 J | **falls**, 907 J |
| sustained 15 N for 5 s | stands, 81 J | stands, 267 J | stands, 729 J |
| arm forward, 1 kg in the hand | stands, 162 J | stands, 508 J | stands, 2002 J |
| quarter squat | stands, 33 J | stands, 108 J | stands, 276 J |
| weight shift | stands, 43 J | stands, 128 J | stands, 333 J |
| one-foot hold, 10 s | stands, 562 J | stands, 1833 J | stands, 8609 J (400 W mean) |
| first static step | stands, 471 J | stands, 1591 J | stands, 6402 J |
| static walk | stands, 1113 J | stands, 3754 J | **falls** at 132 s, 66 kJ |

At low the W-max drives sit at the firmware current limit (torque 0.72 of stall) and above their continuous rating
most of the time. In the push the spine pitch line peaks at 305 N and the body goes over; in the walk the second
transfer takes 30 s instead of 11, the hip lines go slack and loaded in turn (thigh.R_pitch 385 N peak, SF 1.7,
slack 21 % of the time), and it falls while centring. The neck's clutch slips 297 degrees as the body tumbles.

Energy is the other result: at sourced nom every scene costs 3.1 to 3.4 times the model's, because the 775 draws
2.6 times the current per newton metre and the gearbox loses more. The one-foot hold at low averages 400 W.

### The walk grid (42 variants: two models, seven step lengths, three mass sets)

| Case | Stands | 20 kg | 775 roll-up | worm roll-up | How the rest fail |
| --- | --- | --- | --- | --- | --- |
| model figures (D-039) | 42 / 42 | 14 | 14 | 14 | |
| sourced nom | 36 / 42 | 14 | 11 | 11 | a lift gate times out in steps 2 to 4 |
| **low** | **20 / 42** | 5 | 1 | 14 | lift gates time out in the first transfer (7) and steps 2 to 4 (14); one falls centring |
| W-max at low, the rest nom | 17 / 42 | 5 | 1 | 11 | as low |
| the rest at low, W-max nom | 38 / 42 | 14 | 11 | 13 | steps 3 |
| the 775 at low, its gearbox and the rest nom | 34 / 42 | 14 | 9 | 11 | lift gates in steps 2 to 4 |
| everything at low but the gearbox at 0.50 (8.0 N m at the drum) | 37 / 42 | 12 | 13 | 12 | lift gates in steps 3 and 4 (3), two fall (step 4, centring) |

The grid sits on a knife edge (`19`): differences of three or four variants between cases are noise; 20 against
36 to 42 is not.

Push 40 N, the sharpest test of drive power:

| W-max at the drum | The rest | Push 40 N |
| --- | --- | --- |
| 5.6 N m (775 low, gearbox 0.35) | nom | falls |
| 7.2 N m (775 low, gearbox 0.45) | nom | falls |
| 8.0 N m (775 low, gearbox 0.50) | nom | stands |
| 8.7 N m (775 nom, gearbox 0.35) | nom | stands |
| 8.0 N m (775 low, gearbox 0.50) | low | falls (the one-foot hold and the walk stand) |
| 8.8 N m (775 low, gearbox 0.55) | low | stands |
| 9.6 / 10.4 N m (gearbox 0.60 / 0.65) | low | stands |

**The W-max is what falls.** Every other part at its pessimistic end, lines included, costs little (38 of 42, in
the noise of the grid's knife edge). The lines' low end moves no motion at all (the clutch and the stiffness are
set from the label); it shows only in the safety factors, which reach 1.7 on the hip pitch line in the falling walk. The W-max at low is 5.6 N m and 270 rpm at the drum: 40 W, a third of
the model's 113 W. The low motor alone or the low gearbox alone is survivable on the push; both together are not.
With every other part at its low end, the W-max needs **about 8.8 N m at the drum at 270 rpm** (62 W, the low 775
on a gearbox of at least 0.55) for the whole set to stand: push 40, the one-foot hold and the walk grid at the
sourced-nominal level.

## Recommendation

1. **Do not buy 21 775s against the printed 1:40 on the strength of the model.** At the pessimistic end of the
   sources the robot does not walk. The 775 itself is well sourced (nine sheets agree on Kt); the printed
   planetary's efficiency (0.35 to 0.65, no dynamometer test of a printed two-stage) and the cheap motor's
   resistance are the wide ranges. The W-max needs about **8.8 N m and 270 rpm at the drum** at its pessimistic
   end (the walk grid alone holds at 8.0): a printed gearbox has to be shown to reach 0.55, or the drive changed.
2. **The W-max driver needs 36 A peak** at the pessimistic end (24 A nominal), not 18: the VNH5019's 30 A is short;
   a current limit at the driver's peak in firmware gives up clutch force instead.
3. **Anchor every line with three wraps on the drum and pulley before the crimp** (D-041); never a load-bearing knot.
4. **Lower the W-mini clutch to about 59 N** (Pololu's 0.196 N m gear limit on the 3 mm drum).
5. **Budget 3 to 4 times the energy** the model's figures gave (docs 13 to 23) for the battery.

## Not yet

- No dynamometer test of a printed two-stage planetary; the W-max's widest range. A bought metal planetary
  (e.g. a 1:40 or 1:50 775 gearbox) has published efficiency and would narrow it.
- No measured stall of a cheap 12 000 rpm AliExpress 775, of a JGB37-520 or of the 5840-31ZY worm; no measured
  comparison of no-name N20s with Pololu; no Mabuchi or Johnson primary datasheet (403s).
- The 5010's continuous current without airflow and its peak torque (saturation) are unmeasured anywhere found.
- Braid break tests at 100 and 200 lb, crimp tests on braid, the friction of braid on a printed drum.
- The model's `tendon_calc` figures are left as they were (the committed `sim/robot.xml`); the sourced `nom` is a
  case, not the default, so docs 13 to 23 stay reproducible.
- The worm torso and the brushless W-max at low; the 200 pushes of `record_teacher.py` at low.

## Next

The W-max drive at its pessimistic end: try a higher bus voltage (a 4S battery, 14.8 V: speed and torque up by a
quarter), a lower ratio with the drum and a bought metal planetary in place of the printed one, on push 40 and
the walk grid at `low`, and make the sourced `nom` the model's default once a W-max that stands at `low` is
chosen. Then wind (`wind-gauntlet`).
