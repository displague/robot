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
OUTER_GAIN = {"foot": 0.15}      # per bone (only used when the balance loop is off: with it on, the ankle runs in
                                 # torque mode and carries no trim)
LOCK_TOL, LOCK_RATE = math.radians(1.0), 0.3    # joint-equivalent tolerance and gear rate to switch off and lock
STAND_LEAN = math.radians(-5.0)  # ankle dorsiflexion of the standing pose: puts the CoM over the sole centre
# Balance (docs/07 stages 1-2). Sagittal: a centre-of-pressure controller on the capture point. The CoP the
# feet must produce is p = (1 + CP_GAIN) * (x + v / omega) + integral, clamped inside the sole; the ankle torque
# that puts it there is m g (p - ankle), converted to an ankle angle offset through the measured line stiffness
# (the lines are the series spring). What the sole cannot supply (p beyond COP_HIP) goes to the hip as a
# trunk lean, integrated slowly, leaking back when the ankle can cope again. Lateral: the same CoP law on the
# ankle rolls, and a hip-roll posture shift integrates the lateral error (a weight shift between the feet).
CP_GAIN, CP_KI, CP_IMAX = 1.0, 0.5, 0.12   # -, 1/s, m: capture-point gain, integral on the CoM offset, clamp
BAL_TAU = 0.03                             # s: CoM position/velocity estimate low-pass (IMU + joint sensors)
DOB_TAU, DOB_MAX = 0.08, 0.15              # s, m: disturbance observer (external horizontal force from the CoM
                                           # acceleration against the measured CoP), expressed as a CoM offset
COP_LIMIT_Y, COP_LIMIT_X = 0.10, 0.04      # m: CoP kept inside the toe/heel line, and within the stance sole laterally
COP_DS_X = 0.035                           # m: in double support each ankle only moves its own CoP this far (the
                                           # closed chain makes ankle roll a very stiff way to move weight between feet)
COP_HIP, HIP_GAIN, HIP_RATE, HIP_LEAK, HIP_MAX = 0.04, 0.0, 0.0, 0.2, math.radians(25)  # m, rad per m of excess,
                                 # rad/s per m of excess (integral), 1/s (leak of the integral), rad (clamp)
HIP_SLEW = 0.35                  # rad/s: the trunk lean moves no faster than this (a fast lean throws the hips)
ANKLE_QUIET = 1.0                # s: the ankle may lock once its torque demand has been steady this long
ROLL_KP, ROLL_KI, ROLL_KD = 0.2, 0.5, 0.2     # hip-roll shift: rad per m of lateral error, per m s, per m/s (the
                                 # closed chain of two planted feet moves ~35 mm of CoM per degree of hip roll)
ROLL_MAX = math.radians(20)      # rad: hip-roll shift clamp
ROLL_SLEW = 0.15                 # rad/s: the weight shift is slow by design (static walking, docs/06 Phase 6a)
SS_KP, SS_KI = 1.5, 1.0          # single support: stance hip roll per m of lateral capture point, per m s of CoM
                                 # error. Sign: + stance hip roll moves the CoM left (measured with the stance ankle
                                 # in position mode, docs/15); the loop is written as negative feedback on that
SS_SLEW = 0.5                    # rad/s: the stance hip may move faster than the weight shift (capture-point time
                                 # constant 1/omega = 0.3 s)
SWING_CLEAR = math.radians(3)    # rad: the swing hip's roll once its foot is off the ground: the leg hangs from the
                                 # (level) pelvis with this much abduction. Held parallel to the leaning stance leg
                                 # its mass sat out wide and the loop ran out of hip range; hanging straight down
                                 # it touched the stance shank and its hip loop wound up against the contact
DOB_TAU_SS = 0.25                # s: the disturbance observer is slower in single support (one rocking sole makes a
                                 # noisy CoP and the hip-roll loop's own accelerations read as a push)
CP_GAIN_X = 0.5                  # lateral capture-point gain (double support is not one pendulum)
SS_ANKLE_TORQUE = False          # single support: the stance ankle roll as a torque source (True) or a position
                                 # (False: it holds the leg at the parallelogram angle and the stance hip moves the CoM)
BAL_START = 0.5                  # s: when the balance loop starts (the locks settle first)
LOCK_TOL_SCALE = {"foot": 0.2}   # the ankle locks and re-locks within 0.2 deg so each correction is a few N m
SOLE_CENTRE = (0.0, -0.08, -0.04)  # sole centre relative to the ankle (export_urdf.FOOT_OFFSET)
SOLE_HALF = (0.05, 0.13)         # half width (x) and half length (y) of the sole (export_urdf.FOOT_BOX)
GROUND_LOAD = 0.05               # of the weight: a foot carrying less is in the air for the balance loop
UNLOAD_T = 0.15                  # s: for that long, unless the planner declared it the swing foot (a bounce in the
                                 # weight shift unloaded a foot for a few ms and the support reference jumped)
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
        self.by_name = {w["name"]: w for w in self.w}
        self.lock_enabled = True
        self.balance = True
        self.hook = None                 # called after every step (the video recorder)
        self.record_all = False
        self.pelvis = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self.feet = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in ("foot.L", "foot.R")]
        self.floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.mass = float(sum(m.body_mass))
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
        # the ankles start pre-stretched for the standing torque (CoM over the sole centre, 80 mm ahead of the
        # ankle), so the first half second is not a fall onto the lines
        pre = self.torque_to_angle("foot.L_pitch", self.mass * 9.81 * -SOLE_CENTRE[1] / 2)
        for w in self.w:
            if w["name"].startswith("foot.") and w["name"].endswith("_pitch"):
                p = w["p"]
                ratio = p["tackle"] * p["r_p"] / p["r_d"]
                d.qpos[w["q"]] = STAND_LEAN
                d.qpos[w["gq"]] = d.qpos[w["dq_"]] = ratio * (STAND_LEAN + pre)
        self.stats = {w["name"]: dict(fA=0.0, fB=0.0, slackA=0, slackB=0, slip=0.0, tq=0.0, over=0, e_j=0.0,
                                      locked=0, n=0) for w in self.w}
        self.steps = 0
        self.rows = []
        self.target = {w["name"]: 0.0 for w in self.w}
        for side in ("L", "R"):
            self.target["foot.%s_pitch" % side] = STAND_LEAN
        # balance state: ankle pitch/roll torque-equivalent offsets, hip pitch lean, hip roll shift, estimates
        self.ankle_offset, self.ankle_roll, self.hip_lean, self.hip_shift = pre, {"L": 0.0, "R": 0.0}, 0.0, 0.0
        self._single, self._shift_hold, self._swing_free = False, 0.0, False
        self._grounded, self._unload_t = [True, True], [0.0, 0.0]
        self.swing = None                # planner: the foot being lifted ("L"/"R"), not a support foot
        self.hip_roll_cmd = {"L": 0.0, "R": 0.0}   # per hip: the shift (double support) or the levelling (stance)
        self._quiet, self._off_prev, self._hip_i = 0.0, pre, 0.0
        self._v_prev, self._dist = np.zeros(2), np.zeros(2)
        self._com_f, self._com_v, self._cp_i, self._roll_i = None, np.zeros(2), np.zeros(2), 0.0
        self.weight = (0.5, 0.5)         # planner: desired share of the weight on (left, right)
        self.com_margin = (0.0, 0.0)
        self.cop_cmd = np.zeros(2)
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

    def soles(self):
        """World position of each sole centre and whether that foot is on the ground (its force sensors carry
        more than GROUND_LOAD of the weight)."""
        d = self.data
        _, loads = self.cop()
        out = []
        dt = self.model.opt.timestep
        for i, (b, load) in enumerate(zip(self.feet, loads)):
            c = d.xpos[b] + d.xmat[b].reshape(3, 3) @ np.array(SOLE_CENTRE)
            thr = (GROUND_LOAD if self._grounded[i] else 3 * GROUND_LOAD) * self.mass * 9.81
            self._unload_t[i] = self._unload_t[i] + dt if load <= thr else 0.0
            swing = self.swing == "LR"[i]                      # the planner knows which foot it lifted
            self._grounded[i] = not swing and (load > thr or self._unload_t[i] < UNLOAD_T)
            out.append((c, self._grounded[i]))
        return out

    def support(self):
        """CoM against the feet: (offset of the CoM from the reference point as (x, y), heel margin, toe margin),
        metres. The reference point is the sole centres weighted by the planner's desired weight share, over the
        feet on the ground. Forward is -Y, left is +X; a positive margin means the CoM is inside the sole."""
        com = self.data.subtree_com[self.pelvis]
        s = self.soles()
        wts = [self.weight[i] if s[i][1] else 0.0 for i in range(2)]
        if sum(wts) <= 0:
            wts = [1.0 if s[i][1] else 0.0 for i in range(2)]
        if sum(wts) <= 0:
            return np.zeros(2), -1.0, -1.0
        ref = sum(w * s[i][0] for i, w in enumerate(wts)) / sum(wts)
        e = com[:2] - ref[:2]
        return e, (ref[1] + SOLE_HALF[1]) - com[1], com[1] - (ref[1] - SOLE_HALF[1])

    def cop(self):
        """Centre of pressure from the floor contacts, world (x, y), and the vertical load per foot (N)."""
        m, d = self.model, self.data
        f6 = np.zeros(6)
        num, den, loads = np.zeros(2), 0.0, [0.0, 0.0]
        for i in range(d.ncon):
            c = d.contact[i]
            if self.floor not in (c.geom1, c.geom2):
                continue
            mujoco.mj_contactForce(m, d, i, f6)
            fn = f6[0]
            num += fn * c.pos[:2]
            den += fn
            other = c.geom2 if c.geom1 == self.floor else c.geom1
            for k, b in enumerate(self.feet):
                if m.geom_bodyid[other] == b:
                    loads[k] += fn
        return (num / den if den > 1.0 else None), loads

    def balance_loop(self):
        """Balance from the CoM estimate (IMU + joint sensors on the robot; the simulator here). Sagittal:
        capture-point CoP on the ankle pitch, excess to a hip lean. Lateral: the same on the ankle rolls,
        plus a slow hip-roll weight shift. Sets ankle_offset, ankle_roll (torque-equivalent angle offsets),
        hip_lean and hip_shift (rad), applied to the targets in control()."""
        m, d = self.model, self.data
        dt = m.opt.timestep
        e, heel, toe = self.support()
        self.com_margin = (heel, toe)
        com = d.subtree_com[self.pelvis]
        if self._com_f is None:
            self._com_f, self._e_prev = e.copy(), com[:2].copy()
        a = dt / (BAL_TAU + dt)
        # velocity from the CoM itself (the IMU's job): the support reference jumps when a foot leaves the ground
        self._com_v = (1 - a) * self._com_v + a * (com[:2] - self._e_prev) / dt
        self._e_prev = com[:2].copy()
        self._com_f = self._com_f + a * (e - self._com_f)
        if not (self.balance and d.time > BAL_START):
            return
        s = self.soles()
        grounded = [c for c, on in s if on]
        if not grounded:
            return
        mass = self.mass
        ref = com[:2] - e
        h = max(0.3, com[2] - 0.08)
        omega = math.sqrt(9.81 / h)
        # disturbance observer: the CoM acceleration the IMU sees, less what the measured CoP explains, is an
        # external force; carried as the CoM offset it is equivalent to, so a push is answered at once
        acc = (self._com_v - self._v_prev) / dt
        self._v_prev = self._com_v.copy()
        cop_meas, _ = self.cop()
        if cop_meas is not None:
            resid = acc / omega ** 2 - (self._com_f - (cop_meas - ref))
            b = dt / ((DOB_TAU_SS if len(grounded) == 1 else DOB_TAU) + dt)
            self._dist = np.clip(self._dist + b * (resid - self._dist), -DOB_MAX, DOB_MAX)
        # capture-point CoP demand (m, relative to the reference point), integral on the CoM offset
        cp = self._com_f + self._dist + self._com_v / omega
        self._cp_i = np.clip(self._cp_i + CP_KI * self._com_f * dt, -CP_IMAX, CP_IMAX)
        p = np.array([(1.0 + CP_GAIN_X) * cp[0], (1.0 + CP_GAIN) * cp[1]]) + self._cp_i
        # keep the CoP inside what the grounded soles can supply
        xs = [c[0] for c in grounded]
        lim_x = (min(xs) - SOLE_HALF[0] + COP_LIMIT_X - ref[0], max(xs) + SOLE_HALF[0] - COP_LIMIT_X - ref[0])
        p_y = float(np.clip(p[1], -COP_LIMIT_Y, COP_LIMIT_Y))
        p_x = float(np.clip(p[0], lim_x[0], lim_x[1]))
        single = len(grounded) == 1
        self.cop_cmd = ref + (p_x, p_y)
        # --- sagittal: ankle pitch torque for that CoP, shared by the grounded ankles, as a line-stretch angle.
        # CoP forward of the ankle (-Y) needs plantarflexion torque; +q is plantarflexion (toes down).
        ankle_y = np.mean([d.xpos[b][1] for b, (c, on) in zip(self.feet, s) if on])
        tau_y = mass * 9.81 * (ankle_y - (ref[1] + p_y)) / len(grounded)
        self.ankle_offset = self.torque_to_angle("foot.L_pitch", tau_y)
        self._quiet = self._quiet + dt if abs(self.ankle_offset - self._off_prev) < math.radians(0.02) else 0.0
        self._off_prev = self.ankle_offset
        # excess beyond the ankle's comfortable CoP goes to the hip as a trunk lean (+ hip pitch = trunk back),
        # integrated, leaking back toward upright
        excess = p[1] - math.copysign(min(abs(p[1]), COP_HIP), p[1])
        self._hip_i += (-HIP_RATE * excess - HIP_LEAK * self._hip_i) * dt
        self._hip_i = max(-HIP_MAX, min(HIP_MAX, self._hip_i))
        want = max(-HIP_MAX, min(HIP_MAX, -HIP_GAIN * excess + self._hip_i))
        self.hip_lean += max(-HIP_SLEW * dt, min(HIP_SLEW * dt, want - self.hip_lean))
        # --- lateral: ankle roll torque for the lateral CoP, and the hip-roll weight shift on the lateral error
        # each grounded ankle moves the CoP within its own sole toward the demanded point; in double support
        # only a little (COP_DS_X), the weight transfer between the feet is the hip shift's job
        _, loads = self.cop()
        want_x = ref[0] + p_x
        for side, b, (c, on), load in zip("LR", self.feet, s, loads):
            if single and on and SS_ANKLE_TORQUE:
                dx = max(-COP_LIMIT_X, min(COP_LIMIT_X, (ref[0] + p[0]) - d.xpos[b][0]))
                self.ankle_roll[side] = self.torque_to_angle("foot.%s_roll" % side, mass * 9.81 * dx)
            else:
                self.ankle_roll[side] = None     # position mode: the parallelogram, or the swing foot held level
        if single:
            # the parallelogram is gone. Two-link lateral model: the stance ankle holds the leg at its angle (position
            # mode, the lines both taut) and the stance hip moves the CoM over the sole from the lateral capture
            # point; the swing hip is eased to its plan once its foot has left the ground, and stays there
            if not self._single:
                self._shift_hold, self._ss_i, self._swing_free = self.hip_shift, 0.0, False
            ex, vx = float(self._com_f[0]), float(self._com_v[0])
            self._ss_i = max(-ROLL_MAX, min(ROLL_MAX, self._ss_i + SS_KI * ex * dt))
            self.hip_shift = max(-ROLL_MAX, min(ROLL_MAX, self._shift_hold - SS_KP * (ex + vx / omega) - self._ss_i))
            for side, (c, on), load in zip("LR", s, loads):
                cur = self.hip_roll_cmd[side]
                if on:
                    self.hip_roll_cmd[side] = cur + max(-SS_SLEW * dt, min(SS_SLEW * dt, self.hip_shift - cur))
                else:
                    # a lifting foot keeps the shift until it has actually left the ground, then abducts a little
                    # (SWING_CLEAR); a brush with the floor does not bring the shift back
                    if load < GROUND_LOAD * mass * 9.81:
                        self._swing_free = True
                    want = SWING_CLEAR if self._swing_free else self._shift_hold
                    self.hip_roll_cmd[side] = cur + max(-ROLL_SLEW * dt, min(ROLL_SLEW * dt, want - cur))
        else:
            if self._single:
                self._roll_i = -self.hip_shift - ROLL_KP * self._com_f[0]
            self._roll_i = max(-ROLL_MAX, min(ROLL_MAX, self._roll_i + ROLL_KI * self._com_f[0] * dt))
            want = -max(-ROLL_MAX, min(ROLL_MAX, ROLL_KP * self._com_f[0] + ROLL_KD * self._com_v[0] + self._roll_i))
            self.hip_shift += max(-ROLL_SLEW * dt, min(ROLL_SLEW * dt, want - self.hip_shift))
            self.hip_roll_cmd = {"L": self.hip_shift, "R": self.hip_shift}
        self._single = single

    def torque_to_angle(self, winch, tau):
        """Joint angle offset that stretches the loop's lines to carry `tau`: both runs taut up to the pretension
        range, then the loaded run alone (half the stiffness). Sign: +offset pulls the joint positive."""
        p = self.by_name[winch]["p"]
        n_rp = p["tackle"] * p["r_p"]
        k_both, tau_lin = 2 * p["k"] * n_rp ** 2, 2 * p["pre"] * n_rp
        a = abs(tau)
        if a <= tau_lin:
            return tau / k_both
        return math.copysign(tau_lin / k_both + (a - tau_lin) / (k_both / 2), tau)

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
            bone, axis = w["name"].split("_")[0].split(".")[0], w["name"].split("_")[-1]
            torque_mode = False
            side = w["name"].split(".")[1][0] if "." in w["name"] else ""
            if bone == "foot" and self.balance and (axis == "pitch" or self.ankle_roll[side] is not None):
                # ankle impedance: the gear is placed so the lines carry the balance torque at the planned
                # ankle angle (the lines are the series spring, K per rad about the plan); the CoP loop's
                # integral takes out what the stiffness term adds when the body sways
                torque_mode = True
                # roll: the parallelogram term stays (the leg keeps its angle to the pelvis when the other foot
                # leaves the ground), the CoP torque is added on top
                tgt += self.ankle_offset if axis == "pitch" else (self.ankle_roll[side] - self.hip_roll_cmd[side])
            elif bone == "foot" and axis == "roll":
                tgt -= self.hip_roll_cmd[side]          # parallelogram: the sole stays flat under the shift
            elif bone == "thigh" and axis == "pitch":
                tgt += self.hip_lean
            elif bone == "thigh" and axis == "roll":
                tgt += self.hip_roll_cmd[side]
            tgt = min(max(tgt, w["lo"]), w["hi"])
            err = tgt - q
            # outer loop: integral trim on the joint error, clamped (the "re-home absorbs creep" of docs/01)
            g = 0.0 if torque_mode else OUTER_GAIN.get(bone, 1.0)
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
                      and (not torque_mode or self._quiet > ANKLE_QUIET)
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
            if record or self.record_all:
                row += [fA, fB, tq]
        if record or self.record_all:
            self.rows.append(row)
        if self.hook:
            self.hook(self)

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


def scenarios():
    """The gauntlet: (key, title, note, function(rig)). Shared by the console runner and the video recorder."""
    def stand(r):
        r.run(4.0)

    def push(f):
        def fn(r):
            r.run(1.5)
            r.push("spine", (0, -f, 0), 0.2)
            r.run(3.0)
        return fn

    def lean(r):
        r.run(1.5)
        r.push("spine", (0, -15, 0), 5.0)
        r.run(1.0)

    def arm(r):
        r.run(1.0)
        r.move({"upperarm.L_pitch": math.radians(-90)}, 1.5)
        r.run(1.0)
        r.push("hand.L", (0, 0, -10), 4.0)

    def squat(r):
        r.run(1.0)
        r.move({"thigh.%s_pitch" % s: math.radians(-10) for s in "LR"}
               | {"shank.%s_pitch" % s: math.radians(30) for s in "LR"}
               | {"foot.%s_pitch" % s: math.radians(-20) for s in "LR"}, 1.5)
        r.run(4.0)

    def shift(r, hold=2.0):
        r.run(1.5)
        n = int(3.0 / r.model.opt.timestep)
        for i in range(n):
            r.weight = (0.5 + 0.5 * (i + 1) / n, 0.5 - 0.5 * (i + 1) / n)
            r.step()
        r.run(hold)

    def onefoot(r):
        shift(r, 2.0)
        # the planner lifts only once the right foot carries under a tenth of the weight (foot sensors) and
        # the CoM is within 10 mm of the left sole centre (IMU + kinematics), then tells the balance loop
        for _ in range(6000):
            _, loads = r.cop()
            e, _, _ = r.support()
            if loads[1] < 0.1 * r.mass * 9.81 and abs(e[0]) < 0.01:
                break
            r.step()
        r.swing = "R"
        # fold the leg with the sole kept level: a little hip flexion and a lot of knee, so the foot comes up
        # behind the stance foot (100 mm of clearance, 0.2 m back) instead of beside it, where it touched. A
        # knee-first lift pushed the sole into the floor and shoved the body
        hip, knee = math.radians(10), math.radians(50)
        r.move({"thigh.R_pitch": -hip, "shank.R_pitch": knee, "foot.R_pitch": STAND_LEAN - (knee - hip)}, 1.0)
        r.run(10.0)

    out = [("stand", "stand 4 s (servo, then lock on target)", "zero-power hold: the W-max drums go blue", stand)]
    for f in (10, 20, 30, 40):
        out.append(("push%d" % f, "push %d N for 0.2 s at the torso" % f,
                    "ankle strategy on the capture point; the ankle lines take the load", push(f)))
    out += [("lean15", "sustained 15 N at the torso for 5 s", "disturbance observer: the CoP goes forward at once", lean),
            ("arm", "left arm forward, 1 kg in the hand for 4 s", "shoulder and wrist lines under a held load", arm),
            ("squat", "quarter squat held 4 s", "hip, knee and ankle loops all loaded", squat),
            ("shift", "weight shift onto the left foot", "hip-roll shift and ankle-roll CoP control", shift),
            ("onefoot", "one-foot stance: right foot lifted, held 10 s", "Phase 5 exit test (docs/06)", onefoot)]
    return out


def main():
    csv = "--csv" in sys.argv
    only = sys.argv[sys.argv.index("--only") + 1].split(",") if "--only" in sys.argv else None
    if "--stiff-mini" in sys.argv:
        import export_mjcf
        export_mjcf.STIFF_MINI = True
    if "--hip50" in sys.argv:
        import export_mjcf
        export_mjcf.HIP_ROLL_60 = False      # the 50 mm, tackle 2 hip roll of docs/14, for comparison
    rig = Rig()
    print("mujoco %s: %d bodies, %d DOF (%d joint + %d winch), %d tendons, %d motors, mass %.1f kg"
          % (mujoco.__version__, rig.model.nbody, rig.model.nv, len(rig.w), 2 * len(rig.w), rig.model.ntendon,
             rig.model.nu, sum(rig.model.body_mass)))
    for key, title, note, fn in scenarios():
        if only and key not in only:
            continue
        rig.reset()
        rig.record_all = csv
        fn(rig)
        rig.report(title, top=6)
        if csv:
            rig.write_csv(key)


if __name__ == "__main__":
    main()
