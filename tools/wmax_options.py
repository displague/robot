#!/usr/bin/env python3
"""
wmax_options.py - the W-max drive options of docs/21, side by side on the model's 15 mm drum, and the lead-screw
winch (a screw and nut in place of the drum) per W-max joint.

    python tools/wmax_options.py            # both tables (Markdown)
    python tools/wmax_options.py --drum     # drum drives only
    python tools/wmax_options.py --screw    # lead-screw winch only

Fixed by the model (tools/export_mjcf.py, D-021): drum radius 15 mm, 200 lb braid (890 N), the clutch slips at a
third of break (297 N on the line), the firmware limits motor current to 90 % of the clutch slip (267 N).
Walking points (docs/07, tendon_calc.py WALK scenarios) as drum-side line force and speed:
    ankle push-off  640 N at the joint / 3:1 tackle = 213 N, 26 mm x 3 in 0.25 s = 312 mm/s   (~67 W)
    hip swing       420 N / 2 = 210 N, 42 mm x 2 in 0.5 s = 168 mm/s                          (~35 W)

Motor constants. The 775 appears twice: as the model has it (0.6 N m stall at 25 A, so Kt 0.024 N m/A) and as
its 12 000 rpm at 12 V implies (Ke = Kt = 12 V / 1257 rad/s = 0.0095 N m/A; 50 A stall from a reseller listing,
so 0.48 N m and 0.24 ohm). The two cannot both hold: a 12 000 rpm 12 V motor has Kt near 0.0095, so the current
at a given torque is about 2.5 times the model's. Measure one (bench-load-tests) before trusting either.
Figures marked est in docs/21 are estimates; the rest are listings or arithmetic.
"""
import math
import os
import sys
from dataclasses import dataclass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

R_DRUM = 0.015
BREAK_N = 890.0
CLUTCH_N = BREAK_N / 3                    # 297 N
LIMIT_N = 0.9 * CLUTCH_N                  # 267 N, D-021
WALK = {"ankle": (213.0, 0.312), "hip": (210.0, 0.168)}      # (N, m/s) on the drum-side line


@dataclass
class Drive:
    name: str
    kt: float          # N m per A at the motor
    r: float           # ohm (brushed: terminal; BLDC: line to line, DC-equivalent)
    v: float           # bus volts the speed figure is for
    w0_rpm: float      # motor no-load rpm
    i_peak: float      # brushed: stall current; BLDC: driver/motor peak current
    i_cont: float      # continuous current (heat)
    ratio: float
    eta: float
    mass_g: float      # per drive: motor + driver + bearings + printed gearbox, drum, clutch
    driver: str
    brushed: bool = True

    def drum_torque(self, i):
        return self.kt * i * self.ratio * self.eta

    def line_speed(self, i):
        """drum-side line speed (m/s) at motor current i"""
        w = self.w0_rpm / 60 * 2 * math.pi * (1 - i * self.r / self.v)
        return max(w, 0) / self.ratio * R_DRUM

    def current_for(self, force_n):
        return force_n * R_DRUM / (self.kt * self.ratio * self.eta)

    def peak_power(self):
        i = min(self.i_peak, self.v / (2 * self.r))
        return self.drum_torque(i) * self.line_speed(i) / R_DRUM


DRIVES = [
    # 775 as the model has it: 0.6 N m / 25 A stall, 12 000 rpm; continuous 0.3 of stall (7.5 A)
    Drive("775 + 1:40, BTS7960 (model figures)", 0.024, 0.48, 12, 12000, 25, 7.5, 40, 0.6,
          354 + 65 + 23.8 + 150, "BTS7960 43 A, 65 g"),
    Drive("775 + 1:40, VNH5019 (model figures)", 0.024, 0.48, 12, 12000, 25, 7.5, 40, 0.6,
          354 + 10 + 23.8 + 150, "VNH5019 12 A / 30 A, ~10 g"),
    # 775 from its speed constant: Kt 0.0095, 50 A stall (0.24 ohm); continuous at the model's copper loss
    # (7.5^2 x 0.48 = 27 W -> 10.6 A)
    Drive("775 + 1:40, VNH5019 (Kt from 12 000 rpm)", 0.0095, 0.24, 12, 12000, 50, 10.6, 40, 0.6,
          354 + 10 + 23.8 + 150, "VNH5019 12 A / 30 A, ~10 g"),
    # 5010 360 KV: Kt ~ 8.27/KV = 0.023, R 0.15 ohm (est), speed 0.9 x KV x V (FOC headroom), 20 A peak,
    # 8 A continuous (est, no airflow); B-G431B-ESC1 9.2 g + AS5048A 3 g
    Drive("5010 + 1:16, B-G431B-ESC1, 12 V", 0.023, 0.15, 10.8, 360 * 10.8, 20, 8, 16, 0.6,
          109 + 9.2 + 3 + 23.8 + 150, "B-G431B-ESC1 40 A peak, 9.2 g", brushed=False),
    Drive("5010 + 1:25, B-G431B-ESC1, 24 V (6S)", 0.023, 0.15, 21.6, 360 * 21.6, 20, 8, 25, 0.6,
          109 + 9.2 + 3 + 23.8 + 150, "B-G431B-ESC1 40 A peak, 9.2 g", brushed=False),
]


def drum_rows():
    rows = []
    for d in DRIVES:
        f_peak = d.drum_torque(d.i_peak) / R_DRUM
        f_cont = d.drum_torque(d.i_cont) / R_DRUM
        i_lim = d.current_for(LIMIT_N)
        p_lim = LIMIT_N * d.line_speed(min(i_lim, d.i_peak)) if i_lim <= d.i_peak else 0.0
        walk = {}
        for k, (f, v) in WALK.items():
            i = d.current_for(f)
            walk[k] = (d.line_speed(i) if i <= d.i_peak else 0.0, v, i)
        rows.append(dict(d=d, f_peak=f_peak, f_cont=f_cont, v0=d.line_speed(0), i_lim=i_lim, p_lim=p_lim,
                         p_peak=d.peak_power(), walk=walk))
    return rows


def print_drum():
    print("## Drum drives on the 15 mm drum (clutch %.0f N, firmware limit %.0f N)\n" % (CLUTCH_N, LIMIT_N))
    print("| Drive | Peak line N (motor) | Cont line N | No-load line mm/s | Ankle 213 N: mm/s (need 312), A | "
          "Hip 210 N: mm/s (need 168), A | W at 267 N limit | Peak W (drum) | A at limit | A cont | Driver | g per drive |")
    print("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for r in drum_rows():
        d = r["d"]
        a, h = r["walk"]["ankle"], r["walk"]["hip"]
        print("| %s | %.0f | %.0f | %.0f | %.0f, %.1f | %.0f, %.1f | %.0f | %.0f | %.1f | %.1f | %s | %.0f |" % (
            d.name, r["f_peak"], r["f_cont"], 1000 * r["v0"], 1000 * a[0], a[2], 1000 * h[0], h[2],
            r["p_lim"], r["p_peak"], r["i_lim"], d.i_cont, d.driver, d.mass_g))
    print()


# ------------------------------------------------------------------- lead-screw winch
@dataclass
class Screw:
    name: str
    lead_mm: float
    eta: float
    g_per_mm: float        # screw shaft mass per mm (est)
    nut_g: float           # nut (est)
    nut_len_mm: float
    max_rpm: float         # nut / screw speed limit (est): T8 brass nut sliding speed, ball screw DN ~70 000
    self_locking: bool


SCREWS = [
    Screw("T8 x 2 mm lead, brass nut", 2.0, 0.35, 0.33, 12, 15, 1500, True),
    Screw("SFU1204 ball screw, 4 mm lead", 4.0, 0.90, 0.72, 95, 35, 5000, False),
]
MOTOR_775 = dict(stall_nm=0.6, w0_rpm=12000, kt=0.024, kt_phys=0.0095, mass=354)
BELT_ETA = 0.95


def strokes():
    """(joint, axis, tackle, pulley mm, range deg, stroke mm) for every W-max DOF of the model"""
    import json
    import export_mjcf as em
    sk = json.load(open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "cad", "skeleton.json")))
    out = []
    for j in sk["joints"]:
        if j.get("winch") != "W-max":
            continue
        for dof in j["dof"]:
            wp = em.winch_params(j["name"], dof["axis"], "W-max")
            rng = dof["max"] - dof["min"]
            out.append((j["name"], dof["axis"], wp["tackle"], 1000 * wp["r_p"], rng,
                        wp["tackle"] * 1000 * wp["r_p"] * math.radians(rng)))
    return out


def screw_module(s: Screw, stroke):
    """inline module: 775 (67 mm) + coupler and bearing block (40) + stroke + nut + end bearing (15);
    mass: 775 + small H-bridge 10 g + screw (stroke + 60 mm) + nut + 2 x 608 + printed housing and nut guide
    (60 g + 0.25 g/mm, est) + reduction (T8 1:8 printed gears 60 g, ball screw 1:2.4 belt 30 g, est)"""
    length = 67 + 40 + stroke + s.nut_len_mm + 15
    red_g = 60 if s.self_locking else 30
    mass = MOTOR_775["mass"] + 10 + (stroke + 60) * s.g_per_mm + s.nut_g + 23.8 + 60 + 0.25 * (stroke + s.nut_len_mm + 70) + red_g
    return length, mass


def print_screw():
    print("## Lead-screw winch on a 775 (line force on the drum-side line; the screw replaces gearbox, drum and clutch)\n")
    print("| Screw | Reduction to keep under the nut limit | Line N at motor stall (model / Kt from speed) | "
          "No-load line mm/s | Ankle 213 N / hip 210 N, mm/s, Kt from speed (need 312 / 168) | Peak W at the nut | "
          "A for the 267 N limit (model / Kt from speed) | Holds unpowered |")
    print("| --- | --- | --- | --- | --- | --- | --- | --- |")
    for s in SCREWS:
        g = math.ceil(MOTOR_775["w0_rpm"] / s.max_rpm * 10) / 10
        eta = s.eta * (BELT_ETA if g > 1 else 1)
        f_model = MOTOR_775["stall_nm"] * g * 2 * math.pi * eta / (s.lead_mm / 1000)
        f_phys = MOTOR_775["kt_phys"] * 50 * g * 2 * math.pi * eta / (s.lead_mm / 1000)
        v0 = MOTOR_775["w0_rpm"] / g / 60 * s.lead_mm
        p = MOTOR_775["stall_nm"] * MOTOR_775["w0_rpm"] / 60 * 2 * math.pi / 4 * eta
        i_m = LIMIT_N * (s.lead_mm / 1000) / (2 * math.pi * eta * g) / MOTOR_775["kt"]
        i_p = LIMIT_N * (s.lead_mm / 1000) / (2 * math.pi * eta * g) / MOTOR_775["kt_phys"]
        va, vh = (v0 * (1 - WALK[k][0] / f_phys) for k in ("ankle", "hip"))
        print("| %s | 1:%.1f | %.0f / %.0f | %.0f | %.0f / %.0f | %.0f | %.1f / %.1f | %s |" % (
            s.name, g, f_model, f_phys, v0, va, vh, p, i_m, i_p, "yes (self-locking)" if s.self_locking else "no (back-drives)"))
    print("\nDirect drive (no reduction) would turn the screw at 12 000 rpm: the T8 nut slides at %.1f m/s, the "
          "ball screw is at DN %d; both far past their limits.\n" % (math.pi * 8 * 12000 / 60 / 1000, 12 * 12000))
    print("| Joint | Axis | Tackle | Pulley mm | Range deg | Stroke mm | T8 module mm, g | SFU1204 module mm, g |")
    print("| --- | --- | --- | --- | --- | --- | --- | --- |")
    tot = {s.name: [0, 0.0] for s in SCREWS}
    for jn, ax, n, rp, rng, st in strokes():
        cells = []
        for s in SCREWS:
            L, m = screw_module(s, st)
            mult = 2 if jn in ("shoulder", "hip", "knee", "ankle") else 1
            tot[s.name][0] += mult
            tot[s.name][1] += mult * m
            cells.append("%.0f, %.0f" % (L, m))
        print("| %s | %s | %d | %.0f | %d | %.0f | %s | %s |" % (jn, ax, n, rp, rng, st, cells[0], cells[1]))
    print()
    for s in SCREWS:
        n, m = tot[s.name]
        print("%s: %d W-max modules, %.1f kg (%.0f g average) against %.1f kg for 21 x 775 + BTS7960 + 1:40." % (
            s.name, n, m / 1000, m / n, 21 * DRIVES[0].mass_g / 1000))
    print()


def main():
    if "--screw" not in sys.argv:
        print_drum()
    if "--drum" not in sys.argv:
        print_screw()


if __name__ == "__main__":
    main()
