#!/usr/bin/env python3
"""
tendon_calc.py - sizing calculator for the fishing-line / spindle (winch) tendon stack.

Chain:  motor -> gearbox -> [optional worm stage] -> drum -> [optional block & tackle] -> tendon -> joint pulley

Formulas
  drum torque      T_d  = T_motor_stall * eta_gear * worm_ratio * eta_worm
  tendon force     F    = T_d / r_drum * tackle
  line speed       v    = (rpm_noload/60 * 2*pi) / worm_ratio * r_drum / tackle   (no-load, optimistic)
  joint torque     T_j  = F * r_joint_moment_arm
  excursion        s    = sum(theta_i * r_i)  over the joints the tendon crosses
  line safety      SF   = (break_strength * strands) / F_working

Run:  python tools/tendon_calc.py          (prints the reference scenarios quoted in docs/)
      python tools/tendon_calc.py --json   (machine-readable)
      python tools/tendon_calc.py --ranges (each winch class at the low / nom / high of sim/motor_ranges.json, docs/24)

All motor figures are BALLPARK catalogue values. Measure the motors you actually buy.
"""
import json
import math
import sys
from dataclasses import dataclass


# ---------------------------------------------------------------- motors (ballpark; verify)
@dataclass
class Motor:
    name: str
    stall_torque_nm: float      # at gearbox output, nominal voltage
    noload_rpm: float           # at gearbox output
    stall_current_a: float
    voltage: float
    mass_g: float
    price_usd: float
    cont_fraction: float = 0.30  # fraction of stall torque usable continuously without cooking motor/gearbox
    # Brushless (current-limited) motors only: stall_torque_nm is then the torque at the driver's peak current
    # (stall_current_a), not a true stall, and peak power is taken along the voltage line at that current.
    kv: float = 0.0              # rpm per volt; 0 = brushed (peak power at half stall, half no-load speed)
    r_ohm: float = 0.0           # winding resistance, line to line
    ratio: float = 1.0           # gearbox ratio included in stall_torque_nm and noload_rpm


def motor_peak_power_w(m: Motor, eta: float) -> float:
    """Peak mechanical power at the gearbox output.
    Brushed: T_stall * w_noload / 4. Brushless: at the peak current (or V / 2R if that is lower), with the speed
    the voltage left after the I R drop allows (DC-equivalent model of FOC; ~10 % optimistic)."""
    w0 = m.noload_rpm / 60 * 2 * math.pi
    if not m.kv:
        return m.stall_torque_nm * w0 / 4 * eta
    v_eff = w0 * m.ratio / (m.kv / 60 * 2 * math.pi)            # the voltage the no-load speed implies
    i = min(m.stall_current_a, v_eff / (2 * m.r_ohm))
    t_out = m.stall_torque_nm * i / m.stall_current_a
    w = (v_eff - i * m.r_ohm) / v_eff * w0
    return t_out * w * eta


MOTORS = {
    # These are the figures the committed model runs on. Sourced low / nom / high for the ones the model uses are in
    # sim/motor_ranges.json (D-040, docs/24) and differ: Pololu 298:1 HP is 0.39 N m stall now (0.49 was the
    # pre-2020 figure), JGB37 1:90 1.1 N m (0.77 to 1.47) at 2.4 A, the 775 is 66 A at stall (Kt 0.0095, not 25 A),
    # the worm is gear-limited at 3.9 to 6.9 N m. Run them with ROBOT_MOTORS=low|nom|high, or --ranges here.
    # Pololu-class HP N20 at 6 V; cheap no-name N20s run ~60-70% of these numbers.
    "N20_30:1":    Motor("N20 micro gearmotor 30:1 6V",   0.064, 1000, 1.6, 6,  10,  4.0),
    "N20_100:1":   Motor("N20 micro gearmotor 100:1 6V",  0.21,   320, 1.6, 6,  10,  4.0),
    "N20_298:1":   Motor("N20 micro gearmotor 298:1 6V",  0.49,   100, 1.6, 6,  10,  4.0),
    "N20_1000:1":  Motor("N20 micro gearmotor 1000:1 6V", 0.88,    32, 1.6, 6,  10,  4.0),
    # 25 mm / 37 mm brushed gearmotors with hall encoders (JGA25-370, JGB37-520), 12 V
    "JGA25_1:78":  Motor("JGA25-370 encoder motor 1:78 12V",  0.55, 130, 1.5, 12,  90,  8.0),
    "JGB37_1:90":  Motor("JGB37-520 encoder motor 1:90 12V",  1.7,  100, 3.5, 12, 200, 12.0),
    "JGB37_1:270": Motor("JGB37-520 encoder motor 1:270 12V", 3.5,   35, 3.5, 12, 200, 12.0),
    # Automotive windshield-wiper motor: worm gearbox built in (self-locking), heavy, cheap, strong
    "WIPER":       Motor("12V wiper motor (worm, 2-speed)", 25.0,  55, 18.0, 12, 900, 20.0),
    # 775 brushed motor (12 V, ~0.6 Nm stall, ~12000 rpm) + printed two-stage 1:40 planetary.
    # Torque below is BEFORE gearbox efficiency; pass eta_gear ~0.6 for printed PETG gears.
    "775_PLAN40":  Motor("775 motor + printed 1:40 planetary", 24.0, 300, 25.0, 12, 450, 11.0),
    # 12 V worm gearmotor, 58 mm gearbox, 8 mm shaft, 10 rpm 1:634 (Greartisan B07YBXMTWC; hub record
    # worm-gearmotor-options-20261028): seller's rated 40 kg cm (3.92 N m) at 1.4 A, at the output (worm
    # losses included: pass eta 1.0). Stall not stated: taken as twice rated, current to match. Self-locking.
    "WORM_10RPM":  Motor("12V worm gearmotor 10 rpm 1:634", 7.84, 10, 2.7, 12, 380, 27.0, cont_fraction=0.5),
    # 5010 360 KV multirotor outrunner (109 g listed, docs/05) on FOC + printed two-stage 1:16 planetary (4 x 4), 12 V.
    # Kt ~ 8.27 / KV = 0.023 N m/A (per amp of phase current amplitude). Torque here is at the driver's 20 A peak:
    # 0.023 x 20 x 16 = 7.36 N m BEFORE gearbox efficiency (pass eta ~0.6). No-load 0.9 x 360 x 12 / 16 = 243 rpm.
    # Continuous 8 A (0.4 of peak): heat-limited, a fan-less outrunner inside the torso; estimate, not measured.
    # R 0.15 ohm line to line: estimate (5010-class listings do not state it). docs/21.
    "5010_PLAN16": Motor("5010 360KV BLDC + printed 1:16 planetary", 7.36, 243, 20.0, 12, 109, 28.0,
                         cont_fraction=0.4, kv=360, r_ohm=0.15, ratio=16),
    # the same on a 6S (24 V) bus with 1:25 (5 x 5): more continuous force at the same speed. docs/21.
    "5010_PLAN25_24V": Motor("5010 360KV BLDC + printed 1:25 planetary, 24 V", 11.5, 311, 20.0, 24, 109, 28.0,
                             cont_fraction=0.4, kv=360, r_ohm=0.15, ratio=25),
}

# ---------------------------------------------------------------- lines (braided PE / Dyneema; verify spool label)
LINES = {
    # name: (nominal diameter mm, break strength N)
    "braid_30lb":  (0.28, 133),
    "braid_50lb":  (0.36, 222),
    "braid_80lb":  (0.48, 356),
    "braid_100lb": (0.55, 445),
    "braid_150lb": (0.70, 667),
    "braid_200lb": (0.75, 890),
}


@dataclass
class Scenario:
    name: str
    motor: str
    drum_radius_mm: float
    worm_ratio: float = 1.0
    eta_gear: float = 0.75           # spur gearbox efficiency (N20 ~0.7-0.8)
    eta_worm: float = 0.45           # printed single-start worm, greased: 0.35-0.55
    tackle: float = 1.0              # block-and-tackle mechanical advantage (number of load-bearing strands)
    joint_moment_arm_mm: float = 10  # tendon moment arm at the joint being evaluated
    excursion_mm: float = 30         # total tendon travel needed for full ROM
    required_force_n: float = 50     # working tendon force the task needs
    line: str = "braid_80lb"
    target_time_s: float = 1.0       # how fast full ROM should happen


def evaluate(s: Scenario) -> dict:
    m = MOTORS[s.motor]
    r = s.drum_radius_mm / 1000.0
    t_drum_stall = m.stall_torque_nm * s.eta_gear * s.worm_ratio * (s.eta_worm if s.worm_ratio > 1 else 1.0)
    f_stall = t_drum_stall / r * s.tackle
    f_cont = f_stall * m.cont_fraction
    omega_noload = m.noload_rpm / 60 * 2 * math.pi / s.worm_ratio
    v_noload = omega_noload * r / s.tackle * 1000  # mm/s
    # crude DC-motor model: speed falls linearly with load from no-load to stall
    load_frac = min(s.required_force_n / f_stall, 0.99) if f_stall > 0 else 1
    v_loaded = v_noload * (1 - load_frac)
    if m.kv:   # brushless on a current-limited driver: speed falls only by the I R drop, until the peak current
        v_eff = m.noload_rpm * m.ratio / m.kv
        v_loaded = v_noload * (1 - load_frac * m.stall_current_a * m.r_ohm / v_eff) if load_frac < 0.99 else 0.0
    t_full = s.excursion_mm / v_loaded if v_loaded > 0 else float("inf")
    dia, brk = LINES[s.line]
    sf = brk * s.tackle / s.required_force_n
    drum_turns = s.excursion_mm * s.tackle / (2 * math.pi * s.drum_radius_mm)
    bend_ratio = (2 * s.drum_radius_mm) / dia   # want >= ~10 for braid fatigue life
    # Power is the real budget: drum/tackle only trade force against speed, they never add watts.
    p_required = s.required_force_n * (s.excursion_mm / 1000) / s.target_time_s
    eta_total = s.eta_gear * (s.eta_worm if s.worm_ratio > 1 else 1.0)
    p_motor_peak = motor_peak_power_w(m, eta_total)
    return {
        "scenario": s.name,
        "motor": m.name,
        "required_mech_power_w": round(p_required, 2),
        "motor_peak_mech_power_w": round(p_motor_peak, 2),
        "power_ok": p_motor_peak >= p_required,
        "drum_torque_stall_nm": round(t_drum_stall, 3),
        "tendon_force_stall_n": round(f_stall, 1),
        "tendon_force_cont_n": round(f_cont, 1),
        "required_force_n": s.required_force_n,
        "meets_required_continuously": f_cont >= s.required_force_n,
        "meets_required_at_stall": f_stall >= s.required_force_n,
        "joint_torque_cont_nm": round(f_cont * s.joint_moment_arm_mm / 1000, 2),
        "line_speed_noload_mm_s": round(v_noload, 1),
        "line_speed_at_required_load_mm_s": round(v_loaded, 1),
        "full_rom_time_s": round(t_full, 2),
        "meets_time_target": t_full <= s.target_time_s,
        "drum_turns_for_full_rom": round(drum_turns, 2),
        "line": s.line,
        "line_safety_factor": round(sf, 1),
        "line_safety_ok": sf >= 4,
        "drum_to_line_dia_ratio": round(bend_ratio, 1),
        "motor_mass_g": m.mass_g,
        "motor_price_usd": m.price_usd,
    }


# ---------------------------------------------------------------- reference scenarios quoted in docs/
SCENARIOS = [
    # Finger: one tendon through MCP(90deg,r=10mm) + PIP(100deg,r=7mm) + DIP(70deg,r=5mm) -> ~34 mm excursion.
    # 10 N fingertip at 80 mm lever -> 0.8 Nm at MCP -> 80 N tendon with 10 mm moment arm (strong grasp).
    # 5 N fingertip (everyday manipulation) -> 40 N.
    Scenario("finger / N20 298:1, 3 mm drum, no worm", "N20_298:1", 3.0,
             joint_moment_arm_mm=10, excursion_mm=34, required_force_n=40, line="braid_50lb", target_time_s=0.6),
    Scenario("finger / N20 100:1, 3 mm drum, no worm (fast close, 15 N squeeze)", "N20_100:1", 3.0,
             joint_moment_arm_mm=10, excursion_mm=34, required_force_n=15, line="braid_50lb", target_time_s=0.6),
    Scenario("finger / N20 100:1 + 10:1 printed worm (self-locking), 3 mm drum", "N20_100:1", 3.0, worm_ratio=10,
             joint_moment_arm_mm=10, excursion_mm=34, required_force_n=40, line="braid_50lb", target_time_s=0.6),
    Scenario("finger / N20 30:1 + 10:1 printed worm (self-locking), 3 mm drum", "N20_30:1", 3.0, worm_ratio=10,
             joint_moment_arm_mm=10, excursion_mm=34, required_force_n=40, line="braid_50lb", target_time_s=0.6),
    # Wrist flexion: ~1.5 Nm to lift a 1 kg object held in hand, 15 mm moment arm -> 100 N, 70deg -> 18 mm.
    Scenario("wrist / N20 298:1, 4 mm drum, 2:1 tackle", "N20_298:1", 4.0, tackle=2,
             joint_moment_arm_mm=15, excursion_mm=18, required_force_n=100, line="braid_80lb", target_time_s=0.8),
    # Elbow: 1 kg in hand at 0.30 m + 0.8 kg forearm at 0.15 m -> 4.1 Nm; 30 mm pulley -> 137 N; 140deg -> 73 mm.
    Scenario("elbow / JGB37 1:90, 6 mm drum, 2:1 tackle", "JGB37_1:90", 6.0, tackle=2,
             joint_moment_arm_mm=30, excursion_mm=73, required_force_n=137, line="braid_100lb", target_time_s=1.5),
    Scenario("elbow / JGA25 1:78, 5 mm drum, 3:1 tackle", "JGA25_1:78", 5.0, tackle=3,
             joint_moment_arm_mm=30, excursion_mm=73, required_force_n=137, line="braid_100lb", target_time_s=1.5),
    # Shoulder flexion: arm (3 kg at 0.3 m) + 1 kg at 0.6 m -> 14.7 Nm; 40 mm pulley -> 370 N; 180deg -> 126 mm
    Scenario("shoulder / JGB37 1:270, 8 mm drum, 3:1 tackle", "JGB37_1:270", 8.0, tackle=3,
             joint_moment_arm_mm=40, excursion_mm=126, required_force_n=370, line="braid_200lb", target_time_s=2.0),
    # Knee, sit-to-stand, 20 kg robot, CoM 0.2 m behind knee axis, 2 legs -> ~20 Nm per knee; 50 mm pulley -> 400 N
    Scenario("knee (slow stand) / wiper motor, 15 mm drum, 2:1 tackle", "WIPER", 15.0, tackle=2, eta_gear=0.5,
             joint_moment_arm_mm=50, excursion_mm=122, required_force_n=400, line="braid_200lb", target_time_s=2.0),
    Scenario("knee (slow stand) / JGB37 1:270, 8 mm drum, 4:1 tackle", "JGB37_1:270", 8.0, tackle=4,
             joint_moment_arm_mm=50, excursion_mm=122, required_force_n=400, line="braid_200lb", target_time_s=2.0),
    # ---- walking (docs/07-locomotion.md): 20 kg robot, gait peaks scaled from human biomechanics
    # Ankle push-off: 1.6 Nm/kg -> 32 Nm, 50 mm pulley -> 640 N, 30 deg in 0.25 s -> 26 mm
    Scenario("WALK ankle push-off / wiper motor, 15 mm drum, 2:1 tackle", "WIPER", 15.0, tackle=2, eta_gear=0.5,
             joint_moment_arm_mm=50, excursion_mm=26, required_force_n=640, line="braid_200lb", target_time_s=0.25),
    Scenario("WALK ankle push-off / 775 + printed 1:40 planetary, 15 mm drum, 3:1 tackle", "775_PLAN40", 15.0,
             tackle=3, eta_gear=0.6, joint_moment_arm_mm=50, excursion_mm=26, required_force_n=640,
             line="braid_200lb", target_time_s=0.25),
    # Knee swing: 20 Nm peak, 50 mm pulley -> 400 N (use 360 N walking), 60 deg in 0.4 s -> 52 mm
    Scenario("WALK knee swing / 775 + printed 1:40 planetary, 15 mm drum, 2:1 tackle", "775_PLAN40", 15.0,
             tackle=2, eta_gear=0.6, joint_moment_arm_mm=50, excursion_mm=52, required_force_n=360,
             line="braid_200lb", target_time_s=0.4),
    # Hip swing: 25 Nm, 60 mm pulley -> 420 N, 40 deg in 0.5 s -> 42 mm
    Scenario("WALK hip swing / 775 + printed 1:40 planetary, 15 mm drum, 2:1 tackle", "775_PLAN40", 15.0,
             tackle=2, eta_gear=0.6, joint_moment_arm_mm=60, excursion_mm=42, required_force_n=420,
             line="braid_200lb", target_time_s=0.5),
    # ---- brushless W-max (docs/21): the same joints on a 5010 + printed 1:16 planetary
    Scenario("WALK ankle push-off / 5010 BLDC + printed 1:16 planetary, 15 mm drum, 3:1 tackle", "5010_PLAN16", 15.0,
             tackle=3, eta_gear=0.6, joint_moment_arm_mm=50, excursion_mm=26, required_force_n=640,
             line="braid_200lb", target_time_s=0.25),
    Scenario("WALK hip swing / 5010 BLDC + printed 1:16 planetary, 15 mm drum, 2:1 tackle", "5010_PLAN16", 15.0,
             tackle=2, eta_gear=0.6, joint_moment_arm_mm=60, excursion_mm=42, required_force_n=420,
             line="braid_200lb", target_time_s=0.5),
]


def ranges_table():
    """--ranges: each winch class of the model (export_mjcf.CLASSES) on the model's figures and on the low / nom /
    high cases of sim/motor_ranges.json (D-040, docs/24). The clutch and the label are the builder's settings and
    stay put; the motor, gearbox and the line's real strength move."""
    import export_mjcf as em
    rows = []
    for case in ("model", "low", "nom", "high"):
        motors, lines, eta = em.motor_case("" if case == "model" else case)
        for cls, c in em.CLASSES.items():
            m, r = motors[c["motor"]], c["r_drum"]
            t_stall = m.stall_torque_nm * eta[cls]
            f_stall = t_stall / r
            brk, label = lines[c["line"]][1], em.LINES_MODEL[c["line"]][1]
            f_slip = min(em.CLUTCH_LINE_FRACTION * label, 0.95 * f_stall)
            v0 = m.noload_rpm / 60 * 2 * math.pi * r * 1000
            amps = m.stall_current_a * f_slip / f_stall          # current to hold the line at the clutch (brushed)
            row = dict(case=case, cls=cls, motor=c["motor"], t_stall=t_stall, f_stall=f_stall,
                       f_cont=f_stall * m.cont_fraction, f_slip=f_slip, brk=brk, sf_slip=brk / f_slip, v0=v0,
                       amps=amps)
            if cls == "W-max":       # docs/21's walking points, drum-side: ankle push-off 213 N, hip swing 210 N
                row["ankle_mm_s"] = v0 * max(0.0, 1 - 213 / f_stall)
                row["hip_mm_s"] = v0 * max(0.0, 1 - 210 / f_stall)
            rows.append(row)
    return rows


def main():
    if "--ranges" in sys.argv:
        print("class   case   T_stall drum (N m)  F_stall  F_cont  F_clutch  break  SF@clutch  no-load mm/s  A@clutch"
              "  ankle 213 N mm/s (need 312)  hip 210 N mm/s (need 168)")
        for r in sorted(ranges_table(), key=lambda r: r["cls"]):
            print("%-7s %-6s %8.3f %16.0f %7.0f %7.0f %6.0f %8.1f %12.0f %10.1f   %s" % (
                r["cls"], r["case"], r["t_stall"], r["f_stall"], r["f_cont"], r["f_slip"], r["brk"], r["sf_slip"],
                r["v0"], r["amps"],
                "%5.0f %26.0f" % (r["ankle_mm_s"], r["hip_mm_s"]) if "ankle_mm_s" in r else ""))
        return
    rows = [evaluate(s) for s in SCENARIOS]
    if "--json" in sys.argv:
        print(json.dumps(rows, indent=2))
        return
    for r in rows:
        print(f"\n== {r['scenario']}")
        print(f"   motor: {r['motor']}  ({r['motor_mass_g']} g, ~${r['motor_price_usd']})")
        print(f"   power: task needs {r['required_mech_power_w']} W mech, motor peaks at"
              f" {r['motor_peak_mech_power_w']} W -> power OK: {r['power_ok']}")
        print(f"   tendon force: stall {r['tendon_force_stall_n']} N | continuous {r['tendon_force_cont_n']} N"
              f" | need {r['required_force_n']} N  -> cont OK: {r['meets_required_continuously']},"
              f" peak OK: {r['meets_required_at_stall']}")
        print(f"   joint torque (continuous): {r['joint_torque_cont_nm']} Nm")
        print(f"   line speed: {r['line_speed_noload_mm_s']} mm/s no-load,"
              f" {r['line_speed_at_required_load_mm_s']} mm/s loaded -> full ROM in {r['full_rom_time_s']} s"
              f" (target met: {r['meets_time_target']}), {r['drum_turns_for_full_rom']} drum turns")
        print(f"   line {r['line']}: SF {r['line_safety_factor']} (ok: {r['line_safety_ok']}),"
              f" drum/line dia ratio {r['drum_to_line_dia_ratio']}")


if __name__ == "__main__":
    main()
