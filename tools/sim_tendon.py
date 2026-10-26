#!/usr/bin/env python3
"""
sim_tendon.py - run the tendon-actuated model (sim/robot.xml from tools/export_mjcf.py) through the
standing, pushing and holding tests, and report what every line, clutch and motor is doing.

Per DOF the model has a motor (DC torque-speed line) on a gear hinge, a friction clutch to a drum hinge,
and two one-sided elastic lines from the drum to the joint pulley. The controller here is the simplest
thing the firmware will do: a position servo per joint (voltage = Kp * error - Kd * rate), and when a
joint is on target and still, the motor is switched off and the self-locking stage holds (a dry-friction
holding torque on the gear hinge). That is the zero-power hold of docs/02 and docs/08.

Readouts per line: tension (0 when slack), peak tension against the line's continuous rating and break
strength. Per clutch: slip angle. Per motor: torque against stall, electrical power, time above the
continuous torque. Per test: does it stand, pelvis height and tilt.

Run:  python tools/sim_tendon.py               (needs: pip install mujoco)
      python tools/sim_tendon.py --csv         also writes sim/out/<test>.csv time series
      python tools/sim_tendon.py --stiff-mini  variant: 25 mm W-mini pulleys, 25 % pretension
"""
import math
import os
import sys

try:
    import mujoco
except ImportError:
    print("mujoco not installed: pip install mujoco")
    sys.exit(1)
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from export_mjcf import build   # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KG, KDG = 3.0, 0.02             # inner gear loop: volts-fraction per rad of gear error, per rad/s
KQ, KI, TRIM_MAX = 0.5, 2.0, math.radians(10)   # outer joint loop: proportional, integral (1/s), trim clamp
OUTER_GAIN = {"foot": 0.15}      # per bone: the ankle loop is 560 N m/rad through its lines, 1 deg is the whole
                                 # centre-of-pressure budget of the sole, so it is corrected gently
LOCK_TOL, LOCK_RATE = math.radians(1.0), 0.3    # joint-equivalent tolerance and gear rate to switch off and lock
STAND_LEAN = math.radians(-5.0)  # ankle dorsiflexion of the standing pose: puts the CoM over the sole centre
BAL_KP, BAL_KD, BAL_KI = 1.0, 0.5, 0.5   # ankle strategy: rad of ankle per m of CoM offset from the sole centre,
                                         # per m/s, and per m s (slow re-planning of the lean)
BAL_DEADBAND, BAL_MAX, BAL_TAU = 0.005, math.radians(1.5), 0.05   # m, rad (P+D part), s (CoM estimate low-pass)
BAL_IMAX, BAL_OUT_TAU = math.radians(4.0), 0.05  # rad (integral clamp), s (low-pass on the commanded offset: the
                                                 # ankle is 560 N m/rad through its lines, a step rocks the sole)
LOCK_TOL_SCALE = {"foot": 0.2}   # the ankle locks and re-locks within 0.2 deg so each correction is a few N m
LEAN_RAMP, BAL_START = 0.0, 0.5  # s: optional ramp of the lean from a vertical start (the heel-edge CoP allows
                                 # ~11 N m of ankle torque, a step lifts the toes); when the balance loop starts
SOLE_CENTRE = (0.0, -0.08, -0.04)  # sole centre relative to the ankle (export_urdf.FOOT_OFFSET)
GEAR_FRICTION = 0.02             # of stall: running friction of the gear train when powered
CURRENT_LIMIT = 0.9              # of the clutch slip torque: the driver's current limit


class Rig:
    def __init__(self):
        xml, winches, links = build()
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.w = []
        for name, p, host in winches:
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
            gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name + "_gear")
            self.w.append(dict(
                name=name, p=p,
                q=m.jnt_qposadr[jid], dq=m.jnt_dofadr[jid], gq=m.jnt_qposadr[gid], gdof=m.jnt_dofadr[gid],
                tA=mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, name + "_lineA"),
                tB=mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, name + "_lineB"),
                tC=mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, name + "_clutch"),
                act=mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name + "_motor"),
                eq_stick=mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, name + "_stick"),
                eq_lock=mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, name + "_lock"),
                ddof=m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name + "_drum")],
                dq_=m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name + "_drum")],
                lo=m.jnt_range[jid][0], hi=m.jnt_range[jid][1]))
        self.lock_enabled = True
        self.balance = True
        self.pelvis = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self.feet = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in ("foot.L", "foot.R")]
        self.reset()

    def reset(self):
        """Start in the standing pose: ankles dorsiflexed by STAND_LEAN, the body leaned forward by the same
        angle about the ankle axis so the soles are flat and the CoM sits over the sole centre, every gear
        and drum at the angle that pose needs (the robot was homed there)."""
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        th = -STAND_LEAN
        ankle_z = 0.08
        d.qpos[0:3] = (0.0, -(0.95 - ankle_z) * math.sin(th), ankle_z + (0.95 - ankle_z) * math.cos(th) + 0.01)
        d.qpos[3:7] = (math.cos(th / 2), math.sin(th / 2), 0.0, 0.0)
        for w in self.w:
            if w["name"].startswith("foot.") and w["name"].endswith("_pitch"):
                p = w["p"]
                ratio = p["tackle"] * p["r_p"] / p["r_d"]
                d.qpos[w["q"]] = STAND_LEAN
                d.qpos[w["gq"]] = d.qpos[w["dq_"]] = ratio * STAND_LEAN
        self.stats = {w["name"]: dict(fA=0.0, fB=0.0, slackA=0, slackB=0, slip=0.0, tq=0.0, over=0, e_j=0.0,
                                      locked=0, n=0) for w in self.w}
        self.steps = 0
        self.rows = []
        self.target = {w["name"]: 0.0 for w in self.w}
        for side in ("L", "R"):
            self.target["foot.%s_pitch" % side] = STAND_LEAN
        self.ankle_offset, self._com_f, self._com_v, self._bal_i = 0.0, None, 0.0, 0.0
        self.com_margin = (0.0, 0.0)
        for w in self.w:
            w["_trim"], w["_locked"], w["_slipping"], w["_backdriven"] = 0.0, False, False, 0
            m.eq_data[w["eq_stick"]][0] = 0.0          # gear and drum start together (the clutch has not slipped)
            m.eq_data[w["eq_lock"]][0] = d.qpos[w["gq"]]
            d.eq_active[w["eq_stick"]] = 1
            d.eq_active[w["eq_lock"]] = 0

    # ---- physics of the drive train read back from the state
    def line_force(self, t):
        m, d = self.model, self.data
        hi = m.tendon_lengthspring[t][1]
        return max(0.0, m.tendon_stiffness[t] * (d.ten_length[t] - hi))

    def support(self):
        """CoM against the feet on the ground: (com_y - sole-centre y, heel margin, toe margin), metres.
        Forward is -Y, so a positive margin means the CoM is inside the sole on that side."""
        d = self.data
        com = d.subtree_com[self.pelvis]
        ys = []
        for b in self.feet:
            c = d.xpos[b] + d.xmat[b].reshape(3, 3) @ np.array(SOLE_CENTRE)
            if c[2] < 0.07:
                ys.append(c[1])
        if not ys:
            return 0.0, -1.0, -1.0
        sy = sum(ys) / len(ys)
        return com[1] - sy, (sy + 0.13) - com[1], com[1] - (sy - 0.13)

    def balance_loop(self):
        """Ankle strategy: move both ankle targets so the CoM comes back over the sole centre. The CoM
        comes from the IMU and the joint sensors on the robot; here from the simulator."""
        e, heel, toe = self.support()
        self.com_margin = (heel, toe)
        dt = self.model.opt.timestep
        if self._com_f is None:
            self._com_f = e
        a = dt / (BAL_TAU + dt)
        self._com_v = (1 - a) * self._com_v + a * (e - self._com_f) / dt
        self._com_f += a * (e - self._com_f)
        ef = self._com_f - math.copysign(min(abs(self._com_f), BAL_DEADBAND), self._com_f)
        # CoM behind the sole centre (e > 0): dorsiflex more (negative foot pitch) to lean forward
        if self.balance and self.data.time > BAL_START:
            off = max(-BAL_MAX, min(BAL_MAX, -BAL_KP * ef - BAL_KD * self._com_v))
            self._bal_i = max(-BAL_IMAX, min(BAL_IMAX, self._bal_i - BAL_KI * ef * dt))
            off += self._bal_i
        else:
            off = 0.0
        ramp = 1.0 if LEAN_RAMP <= 0 else min(1.0, self.data.time / LEAN_RAMP)
        b = dt / (BAL_OUT_TAU + dt)
        self.ankle_offset += b * (off - (1.0 - ramp) * STAND_LEAN - self.ankle_offset)

    def control(self):
        """Cascade, as the firmware will do it: an inner position loop on the gear (motor encoder), damped
        by the motor's own back-EMF, and an outer loop that trims the gear target from the joint sensor
        (AS5600, D-008) so line stretch and clutch creep do not become position error."""
        m, d = self.model, self.data
        dt = m.opt.timestep
        self.balance_loop()
        # torque carried by each active equality (constraint rows are rebuilt every step)
        eqf = {}
        mask = d.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY
        for i, f in zip(d.efc_id[mask], d.efc_force[mask]):
            eqf[int(i)] = eqf.get(int(i), 0.0) + float(f)
        for w in self.w:
            p = w["p"]
            q, dq = d.qpos[w["q"]], d.qvel[w["dq"]]
            phi, dphi = d.qpos[w["gq"]], d.qvel[w["gdof"]]
            # clutch: stuck until the torque through it exceeds the slip torque, sliding on the friction
            # tendon until gear and drum move together again, then re-stuck at the new offset
            if d.eq_active[w["eq_stick"]]:
                if abs(eqf.get(w["eq_stick"], 0.0)) > p["t_clutch"]:
                    d.eq_active[w["eq_stick"]] = 0
                    w["_slipping"] = True
            elif abs(dphi - d.qvel[w["ddof"]]) < 0.2:
                m.eq_data[w["eq_stick"]][0] = phi - d.qpos[w["dq_"]]
                d.eq_active[w["eq_stick"]] = 1
                w["_slipping"] = False
            tgt = self.target[w["name"]]
            if w["name"].startswith("foot.") and w["name"].endswith("_pitch"):
                tgt += self.ankle_offset
            tgt = min(max(tgt, w["lo"]), w["hi"])
            err = tgt - q
            # outer loop: integral trim on the joint error, clamped (the "re-home absorbs creep" of docs/01)
            g = OUTER_GAIN.get(w["name"].split(".")[0].split("_")[0], 1.0)
            w["_trim"] = max(-TRIM_MAX, min(TRIM_MAX, w.get("_trim", 0.0) + g * KI * err * dt))
            # gear target from the loop geometry: +phi tightens line B, which pulls q positive
            ratio = p["tackle"] * p["r_p"] / p["r_d"]
            phi_des = ratio * (tgt + g * KQ * err + w["_trim"])
            gerr = phi_des - phi
            ctrl = KG * gerr - KDG * dphi
            # lock with hysteresis: engage when the gear is on target and still, release when it has drifted
            was = w.get("_locked", False)
            tol = LOCK_TOL * LOCK_TOL_SCALE.get(w["name"].split(".")[0].split("_")[0], 1.0)
            locked = (self.lock_enabled and abs(dphi) < LOCK_RATE and abs(err) < 3 * LOCK_TOL
                      and abs(gerr) < (tol * ratio * (2 if was else 1)))
            if locked:
                ctrl = 0.0
                m.dof_frictionloss[w["gdof"]] = p["t_hold"]
                if not w["_locked"]:                    # engage: hold the gear where it is
                    m.eq_data[w["eq_lock"]][0] = phi
                    d.eq_active[w["eq_lock"]] = 1
                elif abs(eqf.get(w["eq_lock"], 0.0)) > p["t_hold"]:
                    d.eq_active[w["eq_lock"]] = 0       # back-driven through the holding torque: it creeps
                    w["_backdriven"] = w.get("_backdriven", 0) + 1
                elif not d.eq_active[w["eq_lock"]] and abs(dphi) < 0.05:
                    m.eq_data[w["eq_lock"]][0] = phi    # creep stopped: lock again here
                    d.eq_active[w["eq_lock"]] = 1
            else:
                d.eq_active[w["eq_lock"]] = 0
                m.dof_frictionloss[w["gdof"]] = GEAR_FRICTION * p["t_stall"]
            # the driver's current limit: motor torque stays below the clutch's slip torque (the clutch is
            # for external overloads, not for the motor to slip itself); torque = T_stall * (ctrl - w/w_noload)
            wn = dphi / p["w_noload"]
            lim = CURRENT_LIMIT * p["t_clutch"] / p["t_stall"]
            ctrl = max(wn - lim, min(wn + lim, ctrl))
            d.ctrl[w["act"]] = max(-1.0, min(1.0, ctrl))
            w["_locked"] = locked

    def step(self, record=False):
        m, d = self.model, self.data
        self.control()
        mujoco.mj_step(m, d)
        self.steps += 1
        dt = m.opt.timestep
        row = [d.time]
        for w in self.w:
            s, p = self.stats[w["name"]], w["p"]
            fA, fB = self.line_force(w["tA"]), self.line_force(w["tB"])
            tq = d.actuator_force[w["act"]]
            s["fA"], s["fB"] = max(s["fA"], fA), max(s["fB"], fB)
            s["slackA"] += fA <= 0.0
            s["slackB"] += fB <= 0.0
            s["slip"] += abs(d.qvel[w["gdof"]] - d.qvel[w["ddof"]]) * dt if w.get("_slipping") else 0.0
            s["tq"] = max(s["tq"], abs(tq))
            s["over"] += abs(tq) > p["motor"].cont_fraction * p["t_stall"]
            ctrl = d.ctrl[w["act"]]
            current = p["i_stall"] * tq / p["t_stall"]
            s["e_j"] += abs(ctrl * p["volt"] * current) * dt
            s["locked"] += w["_locked"]
            s["n"] += 1
            if record:
                row += [fA, fB, tq]
        if record:
            self.rows.append(row)

    def run(self, seconds, record=False):
        for _ in range(int(round(seconds / self.model.opt.timestep))):
            self.step(record)

    def pelvis_state(self):
        d = self.data
        z = d.xpos[self.pelvis][2]
        up = d.xmat[self.pelvis].reshape(3, 3)[:, 2]
        return z, math.degrees(math.acos(max(-1.0, min(1.0, up[2]))))

    def standing(self):
        z, tilt = self.pelvis_state()
        return z > 0.80 and tilt < 20

    def move(self, targets, seconds, record=False):
        """Ramp joint targets linearly over `seconds` (the planner's job; a step would slam the drive train)."""
        start = {k: self.target[k] for k in targets}
        n = int(round(seconds / self.model.opt.timestep))
        for i in range(1, n + 1):
            for k, v in targets.items():
                self.target[k] = start[k] + (v - start[k]) * i / n
            self.step(record)

    def push(self, body, force_xyz, seconds):
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body)
        for _ in range(int(round(seconds / self.model.opt.timestep))):
            self.data.xfrc_applied[bid][:3] = force_xyz
            self.step()
        self.data.xfrc_applied[bid] = 0

    # ---- reporting
    def report(self, title, groups=None, top=8):
        z, tilt = self.pelvis_state()
        t = self.data.time
        energy = sum(s["e_j"] for s in self.stats.values())
        print("\n== %s: t=%.1f s, pelvis %.3f m, tilt %.1f deg -> %s; electrical energy %.1f J (%.1f W mean)"
              % (title, t, z, tilt, "STANDS" if self.standing() else "FALLS", energy, energy / max(t, 1e-9)))
        rows = []
        for w in self.w:
            s, p = self.stats[w["name"]], w["p"]
            fmax = max(s["fA"], s["fB"])
            rows.append((fmax / p["f_cont"], w["name"], fmax, p["f_cont"], p["break_n"] / max(fmax, 1e-9),
                         100.0 * (s["slackA"] + s["slackB"]) / (2 * s["n"]), math.degrees(s["slip"]),
                         s["tq"] / p["t_stall"], 100.0 * s["over"] / s["n"], 100.0 * s["locked"] / s["n"],
                         s["e_j"]))
        rows.sort(reverse=True)
        print("   %-20s %8s %8s %6s %7s %7s %7s %7s %7s %7s" %
              ("line (worst first)", "peak N", "cont N", "SF", "slack%", "slip dg", "tq/stl", "over%", "lock%", "J"))
        for r in rows[:top]:
            print("   %-20s %8.0f %8.0f %6.1f %7.1f %7.1f %7.2f %7.1f %7.1f %7.1f" % r[1:])
        worst = rows[0]
        flags = []
        if worst[2] > worst[3]:
            flags.append("%s above continuous rating" % worst[1])
        if worst[4] < 4:
            flags.append("%s line safety factor %.1f < 4" % (worst[1], worst[4]))
        slips = [r for r in rows if r[6] > 1.0]
        if slips:
            flags.append("clutch slip: " + ", ".join("%s %.0f deg" % (r[1], r[6]) for r in slips))
        print("   flags: " + ("; ".join(flags) if flags else "none"))
        return rows

    def write_csv(self, name):
        if not self.rows:
            return
        os.makedirs(os.path.join(ROOT, "sim", "out"), exist_ok=True)
        path = os.path.join(ROOT, "sim", "out", name + ".csv")
        hdr = ["t"] + sum([[w["name"] + "_A_N", w["name"] + "_B_N", w["name"] + "_motor_Nm"] for w in self.w], [])
        np.savetxt(path, np.array(self.rows), delimiter=",", header=",".join(hdr), comments="")
        print("   wrote %s" % path)


def main():
    csv = "--csv" in sys.argv
    if "--stiff-mini" in sys.argv:
        import export_mjcf
        export_mjcf.STIFF_MINI = True
    rig = Rig()
    print("mujoco %s: %d bodies, %d DOF (%d joint + %d winch), %d tendons, %d motors, mass %.1f kg"
          % (mujoco.__version__, rig.model.nbody, rig.model.nv, len(rig.w), 2 * len(rig.w), rig.model.ntendon,
             rig.model.nu, sum(rig.model.body_mass)))

    # 1. stand: servo for 2 s, then the locks take over
    rig.reset()
    rig.run(4.0, record=csv)
    rig.report("stand 4 s (servo, then lock on target)")
    if csv:
        rig.write_csv("stand")

    # 2. push impulses at the torso, forward
    for f in (10, 20, 30, 40):
        rig.reset()
        rig.run(1.5)
        rig.push("spine", (0, -f, 0), 0.2)
        rig.run(3.0)
        rig.report("push %d N for 0.2 s at the torso" % f, top=5)

    # 3. sustained lean: 15 N held at the torso for 5 s
    rig.reset()
    rig.run(1.5)
    rig.push("spine", (0, -15, 0), 5.0)
    rig.report("sustained 15 N at the torso for 5 s", top=6)

    # 4. arm raised forward over 1.5 s, then 10 N (1 kg) hanging from the hand for 4 s
    rig.reset()
    rig.run(1.0)
    rig.move({"upperarm.L_pitch": math.radians(-90)}, 1.5)
    rig.run(1.0)
    rig.push("hand.L", (0, 0, -10), 4.0)
    rig.report("left arm forward, 1 kg in the hand for 4 s", top=6)

    # 5. quarter squat: hips -10, knees 30, ankles -20 (soles stay flat, CoM 5 mm ahead of the sole centre
    #    by the exporter's FK), reached over 1.5 s and held 4 s
    rig.reset()
    rig.run(1.0)
    rig.move({"thigh.%s_pitch" % s: math.radians(-10) for s in "LR"}
             | {"shank.%s_pitch" % s: math.radians(30) for s in "LR"}
             | {"foot.%s_pitch" % s: math.radians(-20) for s in "LR"}, 1.5, record=csv)
    rig.run(4.0, record=csv)
    rig.report("quarter squat held 4 s", top=6)
    if csv:
        rig.write_csv("squat")


if __name__ == "__main__":
    main()
