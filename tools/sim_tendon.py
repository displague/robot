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
      python tools/sim_tendon.py --open-swing  the step as in docs/16 (planner geometry, open-loop swing, locks on)
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
KD_JOINT = {}                    # s: joint-rate damping in the outer loop per bone (rad of target per rad/s; docs/18)
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
COP_MARGIN_Y, COP_LIMIT_X = 0.03, 0.04     # m: CoP kept this far inside the heel and toe lines of the soles on the
                                           # ground (not a fixed distance from the reference: with the CoM biased
                                           # ahead for a step the demand reached past the toe), and this far inside
                                           # the stance sole laterally
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
UNLOAD_T = 0.075                 # s: for that long, unless the planner declared it the swing foot (a bounce in the
                                 # weight shift unloaded a foot for a few ms and the support reference jumped). It
                                 # read 0.15 until P-014, but the timer advanced on every call and the controller
                                 # called it twice a step: 0.075 s is what all the tuning ran on (docs/16)
STEP_LEN = 0.10                  # m: the first static step (docs/06 Phase 6a, hub task first-static-step)
STAND_HIP = 0.076                # m: the hip is this far ahead of the ankle in the standing pose (the 5 deg lean)
ARM_CLEAR = math.radians(10)     # rad: shoulder abduction that keeps the hands off the thighs while stepping
LAND_LOAD = 0.15                 # of the weight: the swing foot counts as placed once it carries this much
PLACE_T = 1.5                    # s: the swing leg unfolds onto the floor over this long (stops at touchdown)
TRANSFER_BACK = 1.0              # the transfer onto the leading foot ends with the CoM this many half-steps behind its
                                 # sole centre (the old fold swung the trailing leg's mass forward at lift-off)
FIXED_FLOOR = False              # the planner's leg heights from the floor (the ankles' standing height, measured once)
                                 # rather than from the tall foot's ankle, which rises as that foot unloads and fed the
                                 # leading leg's bend back into itself (docs/18)
RELEASE_SETTLED = False          # the transfer declares the trailing foot the swing foot only once the lateral CoM has
                                 # settled as the lift gate requires (|e_x| < 10 mm, |v_x| < 20 mm/s), not at the first
                                 # dip of its load below a tenth of the weight (docs/18)
WIDTH_FROM_STANCE = False        # IK_LATERAL lands the foot at the standing spacing from the stance foot (True) or at
                                 # its own lift-off width (False); see docs/18 for why True is not yet better
TRANSFER_PLAN = True             # the transfers by planned geometry in stages (docs/19): CoM forward at an even share
                                 # with a small crouch, then the weight across, the trailing leg shortened as the CoM
                                 # arrives, released when settled, the stance leg straightened; the centring likewise
STEP_LOCKS = False               # False: the drums do not self-lock from the step's weight shift to the end of its
                                 # centring (locks hold poses at zero power; engaging and releasing them inside the
                                 # balance loop rocks the body foot to foot at 1-2 Hz, +-33 N; docs/19)
LAT_FF = False                   # the double-support weight shift with a geometric feed-forward base (docs/19)
IK_DAMP, IK_STEP = 0.01, 0.15   # the swing IK's damping (m-rad mixed) and largest joint step per iteration (rad)
TP_LAND_REACH = 0.0              # m: the centring starts the landed leg's press here, not at the landing's 20 mm below the
                                 # floor (pressing an outward-leaning leg shoves the hip over the stance foot; docs/19)
TP_KEEP_CROUCH = True            # TRANSFER_PLAN: the stance leg stays crouched through the next swing so a level landing
                                 # has reach to press with; the centring straightens it
TP_SHIFT_FF = True               # the planned transfers feed the weight share's lateral move into the hip shift
TP_CROUCH = 0.012                # m: both hips this far below the lower straight-leg height during a transfer
TP_A, TP_B, TP_HOLD = 3.0, 3.0, 1.0   # s: the sagittal stage, the weight ramp, the holds
TRANSFER_SWAP, TRAIL_LIFT = False, 0.015   # through the transfer's hold the leading leg becomes the straight one and
                                 # the trailing leg is planned TRAIL_LIFT short, so its knee bends and its foot unloads
                                 # (a straight trailing leg sets the hip height and cannot unload; docs/18)
STAGGER_ONLY, STAGGER_HOLD = False, 8.0   # the `stagger` scene: first placement, then hold the staggered stance
TRANSFER_KNEE = True             # the leading knee is a torque source through the transfer (docs/16 item 6)
TRANSFER_HOLD = 1.0              # s: the transfer onto the leading foot holds its end point this long
TRANSFER_T = 5.0                 # s: a weight transfer between staggered feet (the hip-roll shift is slow, docs/14)
LAND_REACH = 0.02                # m: the landing pose aims this far below the floor; touchdown ends the placement
HOVER_H, HOVER_T = 0.04, 1.0     # m, s: the swing foot is first carried level to this far above its mark, in this long
SWING_IK = True                  # the swing leg placed by world-frame inverse kinematics (docs/18); False: the planner
                                 # geometry of docs/16 (targets relative to the CoM and hips, open loop)
TRACE = False
SWING_KI, SWING_CMAX = 0.0, 0.05  # 1/s, m: the swing ankle's world-frame aim moves by the integral of its measured error
                                 # from the path (the leg sags below its targets under its own weight), clamped
SWING_IK_TAU = 0.3               # s: low-pass on the measured body pose the IK reads (the swing hip's reaction
                                 # pitches the pelvis; read back unfiltered, the leg rang at 5 Hz, +-40 mm at the foot)
IK_LATERAL = True                # True: the IK also holds the swing ankle's width and sole roll in the world frame;
                                 # False: the swing hip roll follows the weight shift through SWING_ROLL_TAU (docs/18)
SWING_ROLL_TAU = 0.5             # s: low-pass on the swing hip's parallel roll command (unfiltered it followed the
                                 # stance loop's 3 Hz ripple and threw the free leg +-45 mm sideways)
ROLL_RESTORE = 0.05              # rad/s: after an IK landing, the swing leg's roll targets return to their pre-lift
                                 # values this fast during the transfer (the weight shift takes up the difference)
PLAN_SLEW = 0.6                  # rad/s: fastest the planner moves a leg target during a transfer
REACH_0 = 0.003                  # m: the bent leg of a staggered stance is planned this much long; its knee is a
                                 # torque source (the share of the weight the CoM implies), so the leg is a spring
                                 # the weight settles onto rather than a position that hovers or presses
ANKLE_BLEND = 0.3                # s: a foot that has just landed takes up its share of the ankle torque over this
                                 # long (a swing foot's ankle is in position mode, sole level, carrying no offset)
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
        mujoco.mj_forward(m, self.data)
        j = lambda n: self.data.xanchor[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
        self.leg = float(np.linalg.norm(j("thigh.L_pitch") - j("foot.L_pitch")))   # hip to ankle, m
        self.thigh_len = float(np.linalg.norm(j("thigh.L_pitch") - j("shank.L_pitch")))
        self.shank_len = float(np.linalg.norm(j("shank.L_pitch") - j("foot.L_pitch")))

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
        self.phase, self.phases = None, []      # planner: the current phase label; per-phase readouts, in order
        self.stats = {w["name"]: dict(fA=0.0, fB=0.0, slackA=0, slackB=0, slip=0.0, tq=0.0, over=0, e_j=0.0,
                                      hot=0.0, hot_max=0.0,
                                      locked=0, n=0) for w in self.w}
        self.steps = 0
        self.rows = []
        self.phase, self.phases = None, []
        self.target = {w["name"]: 0.0 for w in self.w}
        for side in ("L", "R"):
            self.target["foot.%s_pitch" % side] = STAND_LEAN
        # balance state: ankle pitch/roll torque-equivalent offsets, hip pitch lean, hip roll shift, estimates
        self.ankle_offset, self.ankle_roll, self.hip_lean, self.hip_shift = pre, {"L": 0.0, "R": 0.0}, 0.0, 0.0
        self._single, self._shift_hold, self._swing_free = False, 0.0, False
        self._swing_roll_f = None
        self._grounded, self._unload_t = [True, True], [0.0, 0.0]
        self._ank_w = [1.0, 1.0]         # each ankle's share of the sagittal balance torque (0 in the air)
        self._load_f = [0.5 * self.mass * 9.81] * 2    # filtered vertical load per foot (its force sensors)
        self.ankle_off = {"L": pre, "R": pre}          # per-ankle pitch offset: that foot's share of the torque
        self.swing = None                # planner: the foot being lifted ("L"/"R"), not a support foot
        self.swing_land = False          # planner: the swing leg is being placed (its hip tracks the stance hip)
        self.swing_parallel = False      # planner: a foot off the floor keeps its leg parallel to the stance leg
                                         # (same vertical reach, so it neither presses the floor nor hangs wide)
        self.knee_tau = {"L": None, "R": None}   # planner: a knee carrying this torque (N m, + flexion) as a torque
                                         # source about its planned angle (the bent leg of a staggered stance)
        self.com_bias = np.zeros(2)      # planner: where the CoM should sit relative to the support reference (m)
        self.hip_roll_cmd = {"L": 0.0, "R": 0.0}   # per hip: the shift (double support) or the levelling (stance)
        self.applied = dict(self.target)  # per DOF: the joint target control() applied last step, balance terms in
        self._quiet, self._off_prev, self._hip_i = 0.0, pre, 0.0
        self._v_prev, self._dist = np.zeros(2), np.zeros(2)
        self._com_f, self._com_v, self._cp_i, self._roll_i = None, np.zeros(2), np.zeros(2), 0.0
        self.weight = (0.5, 0.5)         # planner: desired share of the weight on (left, right)
        self.com_share = (0.5, 0.5)      # balance loop: the share the CoM position between the soles implies
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

    def soles(self, update=False):
        """World position of each sole centre and whether that foot is on the ground (its force sensors carry
        more than GROUND_LOAD of the weight)."""
        d = self.data
        _, loads = self.cop()
        out = []
        dt = self.model.opt.timestep
        for i, (b, load) in enumerate(zip(self.feet, loads)):
            c = d.xpos[b] + d.xmat[b].reshape(3, 3) @ np.array(SOLE_CENTRE)
            if update:
                thr = (GROUND_LOAD if self._grounded[i] else 3 * GROUND_LOAD) * self.mass * 9.81
                self._unload_t[i] = self._unload_t[i] + dt if load <= thr else 0.0
                swing = self.swing == "LR"[i]                  # the planner knows which foot it lifted
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
        e = com[:2] - ref[:2] - self.com_bias
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
        for i, (c, on) in enumerate(s):
            # the planner's swing foot carries no ankle torque (position mode, sole level); any other foot shares
            # it, blending back in after a landing. By the planner's flag, not the load: a lightly loaded foot in
            # a weight shift must not flip modes
            self._ank_w[i] = 0.0 if self.swing == "LR"[i] else min(1.0, self._ank_w[i] + dt / ANKLE_BLEND)
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
        cop_meas, loads = self.cop()
        for i in range(2):
            self._load_f[i] += a * (loads[i] - self._load_f[i])
        if cop_meas is not None:
            resid = acc / omega ** 2 - (self._com_f - (cop_meas - ref))
            b = dt / ((DOB_TAU_SS if len(grounded) == 1 else DOB_TAU) + dt)
            self._dist = np.clip(self._dist + b * (resid - self._dist), -DOB_MAX, DOB_MAX)
        # capture-point CoP demand (m, relative to the reference point), integral on the CoM offset
        cp = self._com_f + self._dist + self._com_v / omega
        self._cp_i = np.clip(self._cp_i + CP_KI * self._com_f * dt, -CP_IMAX, CP_IMAX)
        p = np.array([(1.0 + CP_GAIN_X) * cp[0], (1.0 + CP_GAIN) * cp[1]]) + self._cp_i
        # keep the CoP inside what the grounded soles can supply
        xs, ys = [c[0] for c in grounded], [c[1] for c in grounded]
        lim_x = (min(xs) - SOLE_HALF[0] + COP_LIMIT_X - ref[0], max(xs) + SOLE_HALF[0] - COP_LIMIT_X - ref[0])
        lim_y = (min(ys) - SOLE_HALF[1] + COP_MARGIN_Y - ref[1], max(ys) + SOLE_HALF[1] - COP_MARGIN_Y - ref[1])
        p_y = float(np.clip(p[1], lim_y[0], lim_y[1]))
        p_x = float(np.clip(p[0], lim_x[0], lim_x[1]))
        single = len(grounded) == 1
        self.cop_cmd = ref + (p_x, p_y)
        # --- sagittal: ankle pitch torque for that CoP, shared by the grounded ankles, as a line-stretch angle.
        # CoP forward of the ankle (-Y) needs plantarflexion torque; +q is plantarflexion (toes down).
        ankle_y = np.mean([d.xpos[b][1] for b, (c, on) in zip(self.feet, s) if on])
        # shared by the feet by where the CoM sits between the soles (statically that is each foot's load). A
        # lightly loaded foot asked for half the torque rocks onto its toe and pushes the body; sharing by the
        # measured load feeds back (the loaded foot pushes itself off, the load slams across at 2 Hz); sharing
        # by the planner's weight runs ahead of the real transfer and leaves the loaded foot with no torque
        tau_y = mass * 9.81 * (ankle_y - (ref[1] + p_y))
        seg = s[1][0][:2] - s[0][0][:2]
        lam = float(np.clip(np.dot(com[:2] - s[0][0][:2], seg) / max(1e-6, np.dot(seg, seg)), 0.0, 1.0))
        self.com_share = (1.0 - lam, lam)      # the static load share the CoM position implies (planner reads it)
        share = [self._ank_w[0] * (1.0 - lam), self._ank_w[1] * lam]
        tot = sum(share)
        for i, side in enumerate("LR"):
            self.ankle_off[side] = self.torque_to_angle("foot.%s_pitch" % side, tau_y * share[i] / tot if tot > 0 else 0.0)
        self.ankle_offset = self.torque_to_angle("foot.L_pitch", tau_y / len(grounded))
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
                self._swing_roll_f = None
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
                    if self.swing_land or self.swing_parallel:
                        # parallel to the stance leg: it lands at stance width; low-passed (SWING_ROLL_TAU)
                        b = dt / (SWING_ROLL_TAU + dt) if SWING_IK else 1.0
                        self._swing_roll_f = cur if self._swing_roll_f is None else self._swing_roll_f
                        self._swing_roll_f += b * (self.hip_shift - self._swing_roll_f)
                        want = self._swing_roll_f
                    elif self._swing_free:
                        want = SWING_CLEAR
                    else:
                        want = self._shift_hold
                    self.hip_roll_cmd[side] = cur + max(-ROLL_SLEW * dt, min(ROLL_SLEW * dt, want - cur))
        else:
            # LAT_FF (docs/19): the shift the desired CoM point needs, from geometry (its lateral offset from the
            # middle of the two soles over the leg length), as the base; the loop's terms only trim around it.
            # Without it the loop is integral-dominant behind a slew limit and rings at 1.5 Hz (+-25 N per foot)
            ff = 0.0
            if LAT_FF:
                mid_x = 0.5 * (s[0][0][0] + s[1][0][0])
                ff = (ref[0] - mid_x) / self.leg
            if self._single:
                self._roll_i = ff - self.hip_shift - ROLL_KP * self._com_f[0]
            self._roll_i = max(-ROLL_MAX, min(ROLL_MAX, self._roll_i + ROLL_KI * self._com_f[0] * dt))
            want = max(-ROLL_MAX, min(ROLL_MAX, ff - (ROLL_KP * self._com_f[0] + ROLL_KD * self._com_v[0]
                                                      + self._roll_i)))
            self.hip_shift += max(-ROLL_SLEW * dt, min(ROLL_SLEW * dt, want - self.hip_shift))
            for side in "LR":
                cur = self.hip_roll_cmd[side]
                self.hip_roll_cmd[side] = cur + max(-SS_SLEW * dt, min(SS_SLEW * dt, self.hip_shift - cur))
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
        self.soles(update=True)          # the ground-contact debounce advances here, once per step (docs/16)
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
            if (bone == "foot" and self.balance and self.swing != side
                    and (axis == "pitch" or self.ankle_roll[side] is not None)):
                # ankle impedance: the gear is placed so the lines carry the balance torque at the planned
                # ankle angle (the lines are the series spring, K per rad about the plan); the CoP loop's
                # integral takes out what the stiffness term adds when the body sways
                torque_mode = True
                # roll: the parallelogram term stays (the leg keeps its angle to the pelvis when the other foot
                # leaves the ground), the CoP torque is added on top
                tgt += self.ankle_off[side] if axis == "pitch" else (self.ankle_roll[side] - self.hip_roll_cmd[side])
            elif bone == "foot" and axis == "roll":
                tgt -= self.hip_roll_cmd[side]          # parallelogram: the sole stays flat under the shift
            elif bone == "shank" and self.knee_tau.get(side) is not None:
                torque_mode = True                      # knee impedance: the lines carry the planned torque at the
                tgt += self.torque_to_angle(w["name"], self.knee_tau[side])   # planned angle (docs/16)
            elif bone == "thigh" and axis == "pitch":
                tgt += self.hip_lean
            elif bone == "thigh" and axis == "roll":
                tgt += self.hip_roll_cmd[side]
            tgt = min(max(tgt, w["lo"]), w["hi"])
            self.applied[w["name"]] = tgt               # what the joint was asked for (the teacher's action)
            err = tgt - q
            # outer loop: integral trim on the joint error, clamped (the "re-home absorbs creep" of docs/01)
            g = 0.0 if torque_mode else OUTER_GAIN.get(bone, 1.0)
            w["_trim"] = max(-TRIM_MAX, min(TRIM_MAX, w.get("_trim", 0.0) + g * KI * err * dt))
            # gear target from the loop geometry: +phi tightens line B, which pulls q positive
            ratio = p["tackle"] * p["r_p"] / p["r_d"]
            phi_des = ratio * (tgt + g * KQ * err - g * KD_JOINT.get(bone, 0.0) * dq + w["_trim"])
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
            s["hot"] = s["hot"] + dt if max(fA, fB) > p["f_cont"] else 0.0
            s["hot_max"] = max(s["hot_max"], s["hot"])
            if self.phase is not None:
                ph = self.phases[-1][1]
                q = ph.setdefault(w["name"], dict(f=0.0, slack=0, locked=0, e_j=0.0, n=0))
                q["f"] = max(q["f"], fA, fB)
                q["slack"] += (fA <= 0.0) + (fB <= 0.0)
                q["locked"] += w["_locked"]
                q["e_j"] += abs(ctrl * p["volt"] * current) * dt
                q["n"] += 1
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

    def ramp(self, seconds, targets=None, weight=None, bias=None, until=None):
        """The planner's step primitive: ramp joint targets, the desired weight share and the CoM bias together
        over `seconds`; stop early when `until(rig)` is true (touchdown). Returns True if it stopped early."""
        targets = targets or {}
        start = {k: self.target[k] for k in targets}
        w0, b0 = self.weight, self.com_bias.copy()
        n = int(round(seconds / self.model.opt.timestep))
        for i in range(1, n + 1):
            f = i / n
            for k, v in targets.items():
                self.target[k] = start[k] + (v - start[k]) * f
            if weight is not None:
                self.weight = tuple(w0[j] + (weight[j] - w0[j]) * f for j in range(2))
            if bias is not None:
                self.com_bias = b0 + (np.asarray(bias, dtype=float) - b0) * f
            self.step()
            if until is not None and until(self):
                return True
        return False

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
                         s["e_j"], s["hot_max"]))
        rows.sort(reverse=True)
        print("   %-20s %8s %8s %6s %7s %7s %7s %7s %7s %7s %6s" %
              ("line (worst first)", "peak N", "cont N", "SF", "slack%", "slip dg", "tq/stl", "over%", "lock%", "J",
               "hot s"))
        for r in rows[:top]:
            print("   %-20s %8.0f %8.0f %6.1f %7.1f %7.1f %7.2f %7.1f %7.1f %7.1f %6.2f" % r[1:])
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
        if self.phases:
            self.phase_report()
        return rows

    def set_phase(self, label):
        """Start a new phase of the per-phase readout (the planner calls it at each step of a sequence)."""
        self.phase = label
        self.phases.append((label, {}, self.data.time))

    def phase_report(self, legs=("thigh", "shank", "foot")):
        """Per phase: duration, electrical power, the leg lines nearest their rating, the leg lines slack most
        of the time, and the leg drums locked most of the time."""
        print("   per phase (leg lines: hip pitch/roll/yaw = thigh, knee = shank, ankle = foot):")
        ends = [t for _, _, t in self.phases[1:]] + [self.data.time]
        for (label, ph, t0), t1 in zip(self.phases, ends):
            if not ph:
                continue
            e = sum(q["e_j"] for q in ph.values())
            leg = {k: q for k, q in ph.items() if k.split(".")[0].split("_")[0] in legs}
            cont = {w["name"]: w["p"]["f_cont"] for w in self.w}
            hot = sorted(leg, key=lambda k: -leg[k]["f"] / cont[k])[:3]
            slack = sorted((k for k in leg if leg[k]["slack"] / (2 * leg[k]["n"]) > 0.25),
                           key=lambda k: -leg[k]["slack"] / leg[k]["n"])
            locked = [k for k in leg if leg[k]["locked"] / leg[k]["n"] > 0.9]
            print("   %-24s %5.1f s %5.1f W | peak %s | slack>25%% %s | locked>90%% %d of %d leg drums"
                  % (label, t1 - t0, e / max(t1 - t0, 1e-9),
                     ", ".join("%s %.0f N" % (k, leg[k]["f"]) for k in hot),
                     ", ".join("%s %.0f%%" % (k, 50.0 * leg[k]["slack"] / leg[k]["n"]) for k in slack[:4]) or "none",
                     len(locked), len(leg)))

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

    def unloaded(r, i):
        # the planner lifts only once the foot carries under a tenth of the weight (foot sensors), the CoM is
        # within 10 mm of the planned point and moving under 20 mm/s (IMU + kinematics) and the pelvis is level
        # within 1.5 deg: a lift from the tail of the weight shift (pelvis rolled 4 deg, hip loop still moving)
        # put the stance hip at its clamp within a second
        d = r.data
        for _ in range(int(10.0 / r.model.opt.timestep)):
            _, loads = r.cop()
            e, _, _ = r.support()
            rm = d.xmat[r.pelvis].reshape(3, 3)
            roll = math.degrees(math.atan2(rm[2, 0], rm[0, 0]))
            if (loads[i] < 0.1 * r.mass * 9.81 and abs(e[0]) < 0.01 and abs(r._com_v[0]) < 0.02
                    and abs(roll) < 1.5):
                return
            r.step()
        print("   lift gate timed out (load %.0f N, e_x %.0f mm, v_x %.0f mm/s, roll %.1f deg)"
              % (loads[i], e[0] * 1e3, r._com_v[0] * 1e3, roll))

    def level(thigh, knee):
        # ankle pitch that keeps the sole level for a thigh pitch (- is flexion, foot forward) and a knee flexion
        # (foot back): the sole follows the shank, which leans by the pelvis pitch plus both
        return STAND_LEAN - knee - thigh

    def take_step(r):
        """The first static step (docs/06 Phase 6a): shift left, lean half a step forward, lift the right and place
        it STEP_LEN ahead, shift onto it, move the pelvis back half a step, lift the left and place it level with
        the right, centre. Geometry (docs/16): the leg leaning most is straight and sets the hip height, every
        other grounded leg bends its knee to reach the floor; the trunk keeps its standing pitch through the hip
        targets; the CoM bias tells the balance loop where the pelvis is going and the leg targets follow it."""
        lt, ls, big = r.thigh_len, r.shank_len, r.leg
        half = 0.5 * STEP_LEN
        sole = -SOLE_CENTRE[1]                            # the sole centre is this far ahead of the ankle
        # the lift: 25 deg of hip and 38 deg of knee put the foot 50 mm up and 80 mm ahead of the hip with the
        # sole level (ankle at -18 deg, inside its -20 deg stop). The one-foot hold's fold (10 and 50, foot up
        # behind) needs -45 deg at the ankle, so its sole hangs toe-down from the stop and the toe catches the
        # stance heel on the way back; here the foot goes forward, where the step is going
        fold = dict(hip=math.radians(25), knee=math.radians(38))
        # the straight knee (q = 0) is the reference: the anchors are not collinear, so the law of cosines
        # gives it a nonzero included angle k0 and the thigh a fixed offset beta0 from the hip-ankle line
        k0 = math.acos(max(-1.0, min(1.0, (big * big - lt * lt - ls * ls) / (2 * lt * ls))))
        beta0 = math.asin(ls * math.sin(k0) / big)
        theta0 = math.asin(STAND_HIP / big)               # lean of the standing leg (hip ahead of the ankle)

        def leg(d, h):
            # (thigh, knee, ankle) targets for a grounded leg whose hip is d ahead of its ankle at height h,
            # relative to the standing pose (thigh 0, knee 0, ankle STAND_LEAN, trunk pitch kept)
            r2 = d * d + h * h
            k = math.acos(max(-1.0, min(1.0, (r2 - lt * lt - ls * ls) / (2 * lt * ls))))
            theta = math.atan2(d, h)                      # lean of the hip-ankle line, + hip ahead
            beta = math.asin(ls * math.sin(k) / math.sqrt(r2))
            q_t = (theta - beta) - (theta0 - beta0)       # the thigh leans less than the line by beta
            q_k = max(0.0, k - k0)
            return q_t, q_k, level(q_t, q_k)

        def anchor(name):
            return r.data.xanchor[mujoco.mj_name2id(r.model, mujoco.mjtObj.mjOBJ_JOINT, name)]

        ik = mujoco.MjData(r.model)
        swing_q = {}
        body_f = None                                     # the low-passed measured pose the IK reads
        aim = np.zeros(2)                                 # (y, z) correction of the swing ankle's aim, world frame
        x0, att0 = [0.0, 0.0], [(0.0, 0.0), (0.0, 0.0)]   # each swing ankle's width and sole attitude, taken at lift-off
        restore = {}                                      # roll targets to return to after an IK landing (docs/18)

        def sole_att(dd, side):
            # the sole's pitch (about +x) and roll (about +y) in the world, rad
            R = dd.xmat[r.feet["LR".index(side)]].reshape(3, 3)
            return math.atan2(R[2, 1], R[1, 1]), math.atan2(-R[2, 0], R[2, 2])

        def ik_solve(side, x, y, z, att):
            # world-frame IK on the model's own chain: from the measured pose (every joint and the floating pelvis,
            # as the joint sensors and the IMU give it, low-passed), the swing leg's hip roll and pitch, knee and
            # ankle pitch and roll that put its ankle joint at (x, y, z) in the world and the sole at `att` (its
            # standing attitude: level). Gauss-Newton on finite differences, each evaluation one mj_kinematics on
            # a scratch copy; the rest of the body stays where it is measured. Hip yaw is left where it is
            names = ["thigh.%s_roll" % side, "thigh.%s_pitch" % side, "shank.%s_pitch" % side,
                     "foot.%s_pitch" % side, "foot.%s_roll" % side]
            if not IK_LATERAL:
                names = names[1:4]                        # hip roll and ankle roll stay with the balance loop
            adr = [r.by_name[n]["q"] for n in names]
            aj = mujoco.mj_name2id(r.model, mujoco.mjtObj.mjOBJ_JOINT, "foot.%s_pitch" % side)
            ik.qpos[:] = body_f
            q = np.array([swing_q.get(n, r.data.qpos[a]) for n, a in zip(names, adr)])

            def resid(qq):
                for a, v in zip(adr, qq):
                    ik.qpos[a] = v
                mujoco.mj_kinematics(r.model, ik)
                p_, r_ = sole_att(ik, side)
                e = np.array([ik.xanchor[aj][0] - x, ik.xanchor[aj][1] - y, ik.xanchor[aj][2] - z,
                              p_ - att[0], r_ - att[1]])
                return e if IK_LATERAL else e[1:4]

            lo = np.array([r.by_name[n]["lo"] for n in names])
            hi = np.array([r.by_name[n]["hi"] for n in names])
            nq = len(names)
            for _ in range(3):
                e = resid(q)
                J = np.empty((nq, nq))
                for j in range(nq):
                    dq = np.zeros(nq)
                    dq[j] = 1e-4
                    J[:, j] = (resid(q + dq) - e) / 1e-4
                # damped least squares with a step limit: at a straight knee (a target out of reach, as when the
                # landing aims below the floor) the plain solve took huge steps and the leg flailed (docs/19)
                dq = J.T @ np.linalg.solve(J @ J.T + IK_DAMP ** 2 * np.eye(nq), e)
                dq *= min(1.0, IK_STEP / max(1e-9, float(np.max(np.abs(dq)))))
                q = np.clip(q - dq, lo, hi)
            return dict(zip(names, q))

        def swing_ik(side, path, seconds, until=None):
            # carry the swing ankle along path(g) -> (y, z), g from 0 to 1 over `seconds`, sole at its standing
            # pitch, the targets solved every control step from the measured body (docs/18). The balance loop is
            # free to move the body: the foot stays on its world path regardless
            nonlocal body_f, aim
            n = int(round(seconds / r.model.opt.timestep))
            a = r.model.opt.timestep / max(SWING_IK_TAU, r.model.opt.timestep)
            if body_f is None or len(swing_q) == 0:
                body_f = r.data.qpos.copy()
                aim[:] = 0.0
            aj = mujoco.mj_name2id(r.model, mujoco.mjtObj.mjOBJ_JOINT, "foot.%s_pitch" % side)
            dt = r.model.opt.timestep
            for i in range(1, n + 1):
                g = i / n
                g = g * g * (3 - 2 * g)                   # smoothstep: no jerk at either end
                # the body pose as the IK sees it: the measured one, low-passed (quaternion by normalised lerp)
                body_f += a * (r.data.qpos - body_f)
                body_f[3:7] /= np.linalg.norm(body_f[3:7])
                y, z = path(g)
                # the aim integrates the measured ankle's error from the path (joint sensors + kinematics); the IK
                # has already taken out the body's motion, so what is left is the leg's own tracking error
                err = np.array([y, z]) - r.data.xanchor[aj][1:3]
                aim[:] = np.clip(aim + SWING_KI * err * dt, -SWING_CMAX, SWING_CMAX)
                sol = ik_solve(side, x0["LR".index(side)], y + aim[0], z + aim[1], att0["LR".index(side)])
                for k, v in sol.items():
                    swing_q[k] = v
                    # control() adds the trunk lean to every hip pitch target and the weight shift to every hip roll
                    # (and takes it off the ankle roll: the parallelogram); the IK wants the joint angle itself
                    off = {"thigh.%s_pitch" % side: r.hip_lean, "thigh.%s_roll" % side: r.hip_roll_cmd[side],
                           "foot.%s_roll" % side: -r.hip_roll_cmd[side]}.get(k, 0.0)
                    r.target[k] = v - off
                r.step()
                if TRACE and i % 100 == 0:
                    an = r.data.xanchor[aj]
                    print("     ik t=%.2f g=%.2f want (%.0f, %.0f, %.0f) ankle (%.0f, %.0f, %.0f) aim (%.0f, %.0f) | %s | load %.0f"
                          % (r.data.time, g, 1e3 * x0["LR".index(side)], 1e3 * y, 1e3 * z, 1e3 * an[0], 1e3 * an[1],
                             1e3 * an[2], 1e3 * aim[0], 1e3 * aim[1],
                             " ".join("%s %.1f/%.1f" % (k.split(".")[1], math.degrees(v),
                                                        math.degrees(r.data.qpos[r.by_name[k]["q"]])) for k, v in sol.items()),
                             r.cop()[1]["LR".index(side)]))
                if until is not None and until(r):
                    return True
            return False

        def com_ahead():
            # the CoM ahead of the left ankle's start (IMU + kinematics)
            return float(y_l0 - r.data.subtree_com[r.pelvis][1])

        def measure(s):
            # where the ankles actually are, ahead of the left ankle's start (joint sensors + kinematics)
            for i, side in enumerate("LR"):
                s[i] = y_l0 - anchor("foot.%s_pitch" % side)[1]

        def hip_height(side, floor):
            # that side's hip above the floor, the floor being the `floor` foot's ankle (joint sensors + kinematics),
            # or with FIXED_FLOOR the ankles' standing height
            z = z_floor[0] if FIXED_FLOOR and z_floor else anchor("foot.%s_pitch" % floor)[2]
            return anchor("thigh.%s_pitch" % side)[2] - z

        def hip_offsets():
            # each hip ahead of the CoM, measured
            c = com_ahead()
            return {side: y_l0 - anchor("thigh.%s_pitch" % side)[1] - c for side in "LR"}

        def legs(f, s, tall, sides="LR", reach=0.0, off=None):
            # joint targets for the CoM f ahead of the left ankle, the ankles s = (left, right) ahead of it. The
            # `tall` leg is straight and sets the hip height by its own geometry; any other leg bends its knee to
            # the hip height actually measured (the lateral lean of the loaded leg lowers it ~9 mm below what
            # the sagittal geometry says, and that changes through a transfer), plus `reach` below the floor.
            # Each hip sits where it is measured relative to the CoM, not on it: in a swing the hips ran up to
            # 40 mm from the CoM, the swing foot landed off its mark and was then dragged after touchdown.
            # A transfer passes the offsets it measured at its start (`off`): re-measured every step they close
            # a loop through the trunk's sway that dragged a lightly loaded planted foot 75 mm
            off = off or hip_offsets()
            d = {side: f + off[side] - s[i] for i, side in enumerate("LR")}
            h_tall = math.sqrt(max(0.0, big * big - d[tall] ** 2))
            out = {}
            for side in sides:
                h = h_tall if side == tall else min(hip_height(side, tall) + reach,
                                                   math.sqrt(max(0.0, big * big - d[side] ** 2)))
                t, k, a = leg(d[side], h)
                out["thigh.%s_pitch" % side], out["shank.%s_pitch" % side], out["foot.%s_pitch" % side] = t, k, a
            return out

        def planned_legs(f, s, crouch, reach, off, grounded="LR"):
            # leg targets from the plan alone (docs/19): each hip off[side] ahead of the CoM f, each ankle at s; the
            # hips at the lower straight-leg height of the grounded legs less `crouch`, each leg reaching
            # reach[side] beyond it (+ presses, - lifts). No measured height enters, so no loop through the trunk
            d = {side: f + off[side] - s[i] for i, side in enumerate("LR")}
            h = min(math.sqrt(max(0.0, big * big - d[sd] ** 2)) for sd in grounded) - crouch
            out = {}
            for side in "LR":
                t, k, a = leg(d[side], h + reach[side])
                out["thigh.%s_pitch" % side], out["shank.%s_pitch" % side], out["foot.%s_pitch" % side] = t, k, a
            return out

        def slew(plan, dt):
            for k, v in plan.items():
                r.target[k] += max(-PLAN_SLEW * dt, min(PLAN_SLEW * dt, v - r.target[k]))

        def set_weight(w):
            # the planner's weight share, with its lateral move fed forward into the hip-roll weight shift: the
            # support reference moves dx sideways, so the shift needs dx / leg length more (the double-support loop
            # alone integrates the error at a fraction of a degree per second; docs/19)
            if TP_SHIFT_FF and not LAT_FF:
                xs = [c[0][0] for c in r.soles()]
                dx = sum((w[j] - r.weight[j]) * xs[j] for j in range(2))
                r._roll_i -= dx / big
            r.weight = w

        def restore_rolls(side, dt):
            # the lateral IK leaves the landed leg's rolls at absolute angles off the weight-shift parallelogram;
            # return them only while that foot carries a third of the weight, so the change moves the body, not
            # a lightly loaded foot across the floor (docs/18 item 4)
            if r.cop()[1]["LR".index(side)] > 0.3 * r.mass * 9.81:
                for k, v in restore.items():
                    r.target[k] += max(-ROLL_RESTORE * dt, min(ROLL_RESTORE * dt, v - r.target[k]))

        def transfer_plan(f1, s, to, reach0=LAND_REACH):
            # onto the `to` foot in stages (docs/19), each a quantity the balance loop has already shown it holds:
            # A, the CoM forward to f1 at an even weight share while both hips sink TP_CROUCH (the staggered stance
            # holds, docs/18); B, the share ramped onto `to` as the first weight shift does it, the trailing leg
            # shortened only as far as the CoM has actually arrived over `to`; release once the trailing load is
            # under a tenth and the lateral CoM has settled; C, the stance leg straightened with the other free
            dt = r.model.opt.timestep
            it = "LR".index(to)
            frm = "LR"[1 - it]
            measure(s)
            f0, off, w0 = com_ahead(), hip_offsets(), r.weight
            for k in ("L", "R"):
                r.knee_tau[k] = None
            nA = int(round(TP_A / dt))
            for i in range(1, nA + int(round(TP_HOLD / dt)) + 1):
                g = min(1.0, i / nA)
                g = g * g * (3 - 2 * g)
                f = f0 + (f1 - f0) * g
                set_weight(tuple(w0[j] + (0.5 - w0[j]) * g for j in range(2)))
                r.com_bias = np.asarray(bias(f, s, r.weight), dtype=float)
                reach = {to: reach0 + (REACH_0 - reach0) * g, frm: REACH_0}
                slew(planned_legs(f, s, TP_CROUCH * g, reach, off), dt)
                restore_rolls(to, dt)
                r.step()
            nB = int(round(TP_B / dt))
            released, t_rel, i = False, 0.0, 0
            w_to = [0.0, 0.0]
            w_to[it] = 1.0
            while True:
                i += 1
                g = min(1.0, i / nB)
                set_weight(tuple(0.5 + (w_to[j] - 0.5) * g for j in range(2)))
                r.com_bias = np.asarray(bias(f1, s, r.weight), dtype=float)
                arrived = float(np.clip((r.com_share[it] - 0.6) / 0.3, 0.0, 1.0))
                load = r.cop()[1][1 - it]
                if (not released and g >= 1.0 and load < 0.1 * r.mass * 9.81
                        and abs(r._com_f[0]) < 0.01 and abs(r._com_v[0]) < 0.02):
                    released, t_rel = True, r.data.time
                    r.swing = frm
                straight = (0.0 if TP_KEEP_CROUCH else min(1.0, (r.data.time - t_rel) / 1.5)) if released else 0.0
                reach = {to: REACH_0, frm: REACH_0 - (REACH_0 + TRAIL_LIFT) * (1.0 if released else arrived)}
                slew(planned_legs(f1, s, TP_CROUCH * (1.0 - straight), reach, off,
                                  grounded=to if released else "LR"), dt)
                restore_rolls(to, dt)
                r.step()
                if (released and (straight >= 1.0 or (TP_KEEP_CROUCH and r.data.time - t_rel > 1.0))
                        or i * dt > TP_B + 10.0):
                    break
            f_now[0] = f1
            measure(s)
            _, loads = r.cop()
            print("   transfer done at t=%.2f s: loads %.0f/%.0f N, %s, CoM error (%.0f, %.0f) mm"
                  % (r.data.time, loads[0], loads[1], "released" if released else "NOT released",
                     1e3 * r.support()[0][0], 1e3 * r.support()[0][1]))

        def centre_plan(f1, s):
            # the centring in stages (docs/19): 0, both hips sink TP_CROUCH and the landed left foot is pressed until
            # it carries load and the body is still (or 3 s): a straight leg has no range left to press with; 1, the
            # share ramped back to even with its lateral move fed forward; 2, the CoM forward to f1; 3, both legs
            # straightened, then held. The landed leg's rolls return to the parallelogram while it is loaded
            dt = r.model.opt.timestep
            measure(s)
            f0, off, w0 = com_ahead(), hip_offsets(), r.weight
            press = {"L": REACH_0, "R": REACH_0}
            t0 = r.data.time
            while True:
                g = min(1.0, (r.data.time - t0) / 1.0)
                g = g * g * (3 - 2 * g)
                slew(planned_legs(f0, s, TP_CROUCH * (1.0 if TP_KEEP_CROUCH else g),
                                  {"L": TP_LAND_REACH + (REACH_0 - TP_LAND_REACH) * g, "R": REACH_0}, off), dt)
                restore_rolls("L", dt)
                r.step()
                if g >= 1.0 and ((r.cop()[1][0] > 0.15 * r.mass * 9.81 and np.linalg.norm(r._com_v) < 0.02)
                                 or r.data.time - t0 > 3.0):
                    break
            nB = int(round(TP_B / dt))
            for i in range(1, nB + 1):
                g = i / nB
                g = g * g * (3 - 2 * g)
                set_weight(tuple(w0[j] + (0.5 - w0[j]) * g for j in range(2)))
                r.com_bias = np.asarray(bias(f0, s, r.weight), dtype=float)
                slew(planned_legs(f0, s, TP_CROUCH, press, off), dt)
                restore_rolls("L", dt)
                r.step()
            nA = int(round(TP_A / dt))
            for i in range(1, 2 * nA + int(round(2.0 / dt)) + 1):
                g = min(1.0, i / nA)
                g = g * g * (3 - 2 * g)
                g2 = min(1.0, max(0.0, i / nA - 1.0))
                g2 = g2 * g2 * (3 - 2 * g2)
                f = f0 + (f1 - f0) * g
                r.com_bias = np.asarray(bias(f, s, r.weight), dtype=float)
                slew(planned_legs(f, s, TP_CROUCH * (1.0 - g2), press, off), dt)
                restore_rolls("L", dt)
                r.step()

        def move_com(seconds, f1, s, tall, weight=None, hold=0.0, reach0=LAND_REACH, release=None, knee=True,
                     swap=False):
            # the planner's transfer: the CoM target, the weight share and the leg targets move together over
            # `seconds` (then `hold`), the legs recomputed every step from the measured hip height (a closed
            # chain: targets set once at the start press one foot into the floor or lift it as the lean
            # changes), each joint slewed at PLAN_SLEW. The bent leg's reach is closed on the foot sensors: it
            # is pushed down until that foot carries the share the CoM position implies (a knee held at a fixed
            # angle gives under the load as its lines stretch, the foot unloads and lifts, and the support
            # flips back to one foot). Not the planned share: forcing the load ahead of the CoM lifts the
            # other foot and tips the body over the loaded one
            n = int(round((seconds + hold) / r.model.opt.timestep))
            dt = r.model.opt.timestep
            w0 = r.weight
            measure(s)
            f_0 = com_ahead()
            off = hip_offsets()
            lead = "LR"[0] if tall == "R" else "R"
            li = "LR".index(lead)
            sole_b = r.feet[li]
            knee_j = mujoco.mj_name2id(r.model, mujoco.mjtObj.mjOBJ_JOINT, "shank.%s_pitch" % lead)
            for i in range(1, n + 1):
                g = min(1.0, i * dt / seconds)
                settled = (not RELEASE_SETTLED or (abs(r._com_f[0]) < 0.01 and abs(r._com_v[0]) < 0.02))
                if (release and g >= 1.0 and r.swing != release and settled
                        and r.cop()[1]["LR".index(release)] < 0.1 * r.mass * 9.81):
                    # the transfer has taken the weight off this foot: the planner declares it the swing foot
                    # now, so the controller enters single support once. Left to the load debounce, a foot
                    # carrying 0 to 40 N flipped the controller between one foot and two every few tenths of a
                    # second, and one flip ran the stance hip to its clamp with the pelvis rolled 45 deg
                    r.swing = release
                f = f_0 + (f1 - f_0) * g
                w = tuple(w0[j] + (weight[j] - w0[j]) * g for j in range(2)) if weight is not None else w0
                r.weight = w
                r.com_bias = np.asarray(bias(f, s, w), dtype=float)
                # the leading knee carries its share of the weight: the ground reaction (vertical, through the
                # sole centre, the share the CoM position implies) times its horizontal arm to the knee, as an
                # extension torque (negative: + is flexion)
                sole_c = r.data.xpos[sole_b] + r.data.xmat[sole_b].reshape(3, 3) @ np.array(SOLE_CENTRE)
                arm = abs(r.data.xanchor[knee_j][1] - sole_c[1])
                share = 1.0 if r.swing == tall else r.com_share[li]   # all of it once the other foot is released
                # with the feet level both legs are nearly straight: a straight knee has no lever to push with
                # and the torque mode left the leg weak (the foot hopped 0 to 150 N and slid 75 mm), so a level
                # stance runs both knees in position mode, as the plain weight shift does
                gh = max(0.0, (i * dt - seconds) / hold) if (swap and hold > 0) else 0.0   # hold progress
                r.knee_tau[lead] = -share * r.mass * 9.81 * arm if (knee and gh == 0.0) else None
                # the reach eases from the landing pose's (the foot was aimed below the floor and has just
                # touched it) to the planted value: jumping straight to the planted value lifted the new foot
                # 17 mm off the floor it had just landed on
                if gh > 0.0:
                    # the leading leg straight and setting the height; the trailing leg bent TRAIL_LIFT short
                    plan = legs(f, s, lead, reach=-TRAIL_LIFT * min(1.0, 2.0 * gh), off=off)
                else:
                    plan = legs(f, s, tall, reach=reach0 + (REACH_0 - reach0) * g, off=off)
                for k, v in plan.items():
                    r.target[k] += max(-PLAN_SLEW * dt, min(PLAN_SLEW * dt, v - r.target[k]))
                # the IK left the landed leg's rolls at absolute angles; control() adds the weight shift on top, so
                # held there they push the CoM sideways through the transfer (170 mm/s past the stance foot)
                for k, v in restore.items():
                    r.target[k] += max(-ROLL_RESTORE * dt, min(ROLL_RESTORE * dt, v - r.target[k]))
                r.step()
            f_now[0] = f1
            measure(s)
            _, loads = r.cop()
            print("   transfer done at t=%.2f s: loads %.0f/%.0f N, lead knee %s, CoM error (%.0f, %.0f) mm"
                  % (r.data.time, loads[0], loads[1],
                     "position mode" if r.knee_tau[lead] is None else "%.1f N m" % r.knee_tau[lead],
                     1e3 * r.support()[0][0], 1e3 * r.support()[0][1]))

        def bias(f, s, w):
            # CoM bias (m, forward is -y) that puts the CoM f ahead of the left ankle for the weight share w
            return (0.0, -(f - sum(w[i] * (s[i] + sole) for i in range(2))))

        def place(swing, s, s_new):
            i = "LR".index(swing)
            stance = "LR"[1 - i]
            measure(s)
            f = com_ahead()
            r.swing, r.swing_land = swing, False
            r.set_phase("%s lift %s" % ("2" if swing == "R" else "5", swing))
            if SWING_IK:
                # lift: the ankle straight up to HOVER_H above the floor, sole level, at its standing width (world
                # frame, docs/18); the stance leg straight at its lean
                swing_q.clear()
                restore.clear()
                keep = TRANSFER_PLAN and TP_KEEP_CROUCH and swing == "L"
                if IK_LATERAL:
                    restore.update({k: r.target[k] for k in ("thigh.%s_roll" % swing, "foot.%s_roll" % swing)})
                if keep:
                    r.run(0.3)                            # the stance leg keeps its crouch (docs/19)
                else:
                    r.ramp(0.3, legs(f, s, stance, stance))
                a0 = anchor("foot.%s_pitch" % swing).copy()
                # the mark's width: the stance foot's lateral position plus the standing spacing (the trailing foot
                # can slide inward through a transfer; landing it where it slid to narrows the stance step by step)
                k = 1 - i
                x0[i] = (float(anchor("foot.%s_pitch" % stance)[0]) + (width0[i] - width0[k]) if WIDTH_FROM_STANCE
                         else float(a0[0]))
                att0[i] = sole_att(r.data, swing)              # on the floor still: the sole level
                z_f = anchor("foot.%s_pitch" % stance)[2]
                swing_ik(swing, lambda g: (a0[1], a0[2] + (z_f + HOVER_H - a0[2]) * g), 1.2)
                swing_ik(swing, lambda g: (a0[1], z_f + HOVER_H), 0.5)
            else:
                # lift: fold the swing leg (sole level, foot up ahead); the stance leg straight at its lean
                r.ramp(1.5, {"thigh.%s_pitch" % swing: -fold["hip"], "shank.%s_pitch" % swing: fold["knee"],
                             "foot.%s_pitch" % swing: level(-fold["hip"], fold["knee"])} | legs(f, s, stance, stance))
                r.run(0.5)
            # place, in two moves: first carry the foot level to HOVER_H above its mark, then lower it straight
            # down onto a landing pose aimed LAND_REACH below the floor (the hip height moves with the stance
            # lean, so geometry alone leaves the sole hovering); the foot sensors end it at touchdown. In one
            # move the sole met the floor before it had travelled back from the fold and landed 48 mm long.
            # The hip runs parallel to the stance leg throughout, so the foot comes down at stance width
            r.swing_land = True
            r.set_phase("%s place %s" % ("3" if swing == "R" else "6", swing))
            s2 = list(s)
            s2[i] = s_new
            f = com_ahead()
            touch = lambda r: r.cop()[1][i] > LAND_LOAD * r.mass * 9.81
            if SWING_IK:
                # the same two moves in the world frame: the mark is fixed on the floor relative to the left ankle's
                # start, the foot's height relative to the floor the stance ankle stands on
                a0 = anchor("foot.%s_pitch" % swing).copy()
                y_m, z_f = y_l0 - s_new, anchor("foot.%s_pitch" % stance)[2]
                swing_ik(swing, lambda g: (a0[1] + (y_m - a0[1]) * g, a0[2] + (z_f + HOVER_H - a0[2]) * g), HOVER_T)
                swing_ik(swing, lambda g: (y_m, z_f + HOVER_H), 0.3)
                landed = swing_ik(swing, lambda g: (y_m, z_f + HOVER_H - (HOVER_H + LAND_REACH) * g), PLACE_T,
                                  until=touch)
            else:
                r.ramp(HOVER_T, legs(f, s2, stance, swing, reach=-HOVER_H))
                r.run(0.3)
                land = legs(f, s2, stance, swing, reach=LAND_REACH)
                landed = r.ramp(PLACE_T, land, until=touch)
            r.swing, r.swing_land = None, False
            measure(s)
            print("   %s foot %s at t=%.2f s, %.0f mm ahead of the other (planned %.0f)"
                  % (swing, "placed" if landed else "NOT placed", r.data.time, 1e3 * (s[i] - s[1 - i]),
                     1e3 * (s_new - s[1 - i])))

        y_l0 = anchor("foot.L_pitch")[1]
        z_floor = []
        width0 = [0.0, 0.0]
        s = [0.0, 0.0]                                    # ankles ahead of the left ankle's start
        f0 = sole                                         # standing: CoM over the sole centres
        f_now = [f0]                                      # where the planner has put the CoM
        # a foot off the floor keeps its leg parallel to the stance leg: at 3 deg of abduction (the one-foot hold's
        # setting) a straight free leg is vertically longer than the leaning stance leg, presses the floor with
        # 150 N and the support flips between one foot and two until the stance CoP pins at an edge
        r.swing_parallel = True
        lock_was = r.lock_enabled
        if not STEP_LOCKS:
            r.lock_enabled = False
        r.set_phase("1 shift onto L")
        # the arms held 10 deg clear of the thighs: hanging, the hands rest on the thighs and the wrist and
        # shoulder roll lines carry the legs' sway (wrist rolls above their rating for 4 s, shoulder roll 283 N)
        r.move({"upperarm.L_roll": -ARM_CLEAR, "upperarm.R_roll": ARM_CLEAR}, 1.0)
        z_floor.append(0.5 * (anchor("foot.L_pitch")[2] + anchor("foot.R_pitch")[2]))   # both feet flat on the floor
        width0[:] = [float(anchor("foot.%s_pitch" % side)[0]) for side in "LR"]          # their standing widths
        shift(r, 2.0)                                                               # weight onto the left foot
        unloaded(r, 1)
        place("R", s, STEP_LEN)                           # right foot a step ahead, knee bent to reach
        if STAGGER_ONLY:
            # the staggered stance on its own (docs/18): the CoM to the middle of the two soles, an even share, held
            r.set_phase("4 stagger hold")
            move_com(TRANSFER_T, 0.5 * (s[0] + s[1]) + sole, s, "L", weight=(0.5, 0.5), hold=STAGGER_HOLD)
            return
        # onto the right foot, the pelvis half a step forward (the CoM half a step behind the right sole centre:
        # the heel margin is 80 mm, and the next lift swings the left leg's mass forward); the left leg stays
        # straight and sets the height, the right knee follows
        # (the targets come from the feet as measured: the leading foot slid 46 mm forward during this transfer
        # once, and a plan from the stale position landed the next foot behind it and the CoM on a heel)
        r.set_phase("4 transfer onto R")
        if TRANSFER_PLAN:
            transfer_plan(s[1] + sole - TRANSFER_BACK * half, s, "R")
        else:
            move_com(TRANSFER_T, s[1] + sole - TRANSFER_BACK * half, s, "L", weight=(0.0, 1.0), hold=TRANSFER_HOLD,
                     release="L", knee=TRANSFER_KNEE, swap=TRANSFER_SWAP)
        unloaded(r, 0)
        r.knee_tau["R"] = None                            # the right leg is the stance now: position mode
        measure(s)
        place("L", s, s[1])                               # left foot level with it (the right leg straightens)
        r.set_phase("7 centre")
        if TRANSFER_PLAN:
            centre_plan(s[1] + sole, s)
        else:
            move_com(TRANSFER_T, s[1] + sole, s, "R", weight=(0.5, 0.5), hold=3.0, knee=False)   # centre over both
        if r.lock_enabled != lock_was:
            r.lock_enabled = lock_was                     # standing again: the drums may hold at zero power
            r.run(2.0)

    out = [("stand", "stand 4 s (servo, then lock on target)", "zero-power hold: the W-max drums go blue", stand)]
    for f in (10, 20, 30, 40):
        out.append(("push%d" % f, "push %d N for 0.2 s at the torso" % f,
                    "ankle strategy on the capture point; the ankle lines take the load", push(f)))
    out += [("lean15", "sustained 15 N at the torso for 5 s", "disturbance observer: the CoP goes forward at once", lean),
            ("arm", "left arm forward, 1 kg in the hand for 4 s", "shoulder and wrist lines under a held load", arm),
            ("squat", "quarter squat held 4 s", "hip, knee and ankle loops all loaded", squat),
            ("shift", "weight shift onto the left foot", "hip-roll shift and ankle-roll CoP control", shift),
            ("onefoot", "one-foot stance: right foot lifted, held 10 s", "Phase 5 exit test (docs/06)", onefoot),
            ("step", "first static step: right foot 100 mm ahead, then the left level with it",
             "Phase 6a (docs/06): shift, lift, place, shift, lift, place, centre", take_step)]
    if STAGGER_ONLY:
        out = [("stagger", "staggered stance: right foot placed ahead, CoM to the middle, held",
                "the transfer's stance on its own (docs/18)", take_step)]
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
    if "--trace" in sys.argv:
        global TRACE
        TRACE = True
    if "--open-swing" in sys.argv:
        global SWING_IK, IK_LATERAL, TRANSFER_PLAN, STEP_LOCKS
        SWING_IK = IK_LATERAL = TRANSFER_PLAN = False   # the step of docs/16, for comparison
        STEP_LOCKS = True
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
