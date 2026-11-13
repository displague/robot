"""
duo.py - two robots in one simulation (docs/48): the role flip. Robot A gets up from a half-kneel (getup.py); robot B,
the same build, is its helper.

The model is sim/robot.xml twice: B is attached as a copy whose names carry the prefix "b." (MuJoCo's spec attach),
standing at BASE and facing A. Each has its own sim_tendon.Rig on the one MjData: A's step() runs B's controller and
readouts too (Rig.partners). B carries A's upper arms near its own elbows (D-074): a soft connect equality per side
between A's elbow site and B's (`duo.L`, `duo.R`, named by A's side).

B's side (Carer):
  * a posture solver (Posture): ankle, knee, hip, spine and shoulder (both sides alike) put its elbows on the goal
    points, keep its centre of pressure at its soles' centre under the load it measures at the elbows, keep every
    loaded joint's static torque under T_MAX of its rating, and otherwise stay near a resting posture. One descent
    step every TICK on its own kinematics; the drives follow.
  * its balance loop counts the load it carries as weight at its elbows (as FEEL_LOAD does for a bag).
  * it stands with its feet apart (STANCE), astride A's front foot.
A's side is getup.py's script, standing up in a stride (getup.STRIDE). The helper's law is getup.Lifter's (hold toward
an anchor, lift only what A's feet leave, ease off), but its hands are B's elbows (Cradle): the scripted hands make the
catch at the lean, B takes over once its elbows are on A's, from phase `take` of ORDER on, and from then the anchor
moves to B's perch and the cradle never pulls down (D-076).

    python tools/duo.py --probe                       # docs/46's scripted get-up with B standing by: the geometry
    python tools/duo.py --one [take=6 catch=1 base=-0.62 K=V ...] [--params best.json:levels.6] [--trace]
                        [--video name [--gif]]
    python tools/duo.py --train --levels [--from 6] [--gens 30] [--init best.json:levels.6] --out sim/out/duo_x
    python tools/duo.py --jitter-test sim/out/duo_x/best.json:levels.6 [--n 24]
"""
import contextlib
import io
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
os.environ.setdefault("GETUP_GRIP", "elbow")
import getup  # noqa: E402  (imports helper, which sets export_mjcf.HELPER)
import export_mjcf  # noqa: E402
import mujoco  # noqa: E402
import sim_tendon as st  # noqa: E402

D = math.radians
PREFIX = "b."
BASE_Y = -0.63                   # m: where B's ankles stand (A's pelvis kneels at 0; forward is -y)
STANCE = D(8)                    # rad: B's hips abducted, its feet 0.44 m apart: they straddle A's front foot
OTHER = {"L": "R", "R": "L"}     # A's left arm rests on B's right
SOLREF = (0.03, 1.0)             # the cradle's connect: a padded forearm, not a weld
ARMS = ("clavicle.", "upperarm.", "forearm.", "hand.")
A_ELBOW = D(-120)                # rad: A folds its forearms up while it is held, clear of B's thighs and trunk
FOLD_T = 0.8                     # s: A folds its forearms before the lean
TICK = 10                        # steps between posture solves (100 Hz)
RATE = D(20)                     # rad/s: the fastest a posture angle moves
K_BIAS, BIAS_MAX = 2.0, 0.06      # 1/s, m: the integral on the elbows' measured position error
TAKE_TOL = 0.025                  # m: B takes A's arms once its elbows are this close to them for TAKE_S
TAKE_S = 0.2
ORDER = ("stand: lean", "stand: rise", "stand: shift", "stand: step lift", "stand: step swing", "stand: step plant",
         "stand: step land", "stand: up")   # the get-up's phases; `take` (0-7) is the first in which B may take A's arms
TAKE = 7                         # by default at the stand-up (the level that stands 19 of 24, docs/48)
LEVELS = (7, 6, 5, 3, 2, 1)      # the search's curriculum: B takes over later in the get-up first, then earlier
F_TAU = 0.1                      # s: the measured load's low-pass
W_POS, W_COP = 0.005, 0.004      # m: the solver's scales for the elbows' position error and the CoP's offset
REACH = 0.38                     # m: its elbows go no further ahead of its soles' centre than this
Z_LOW = 1.0                      # m: nor lower than this above them
E_MAX = 0.03                     # m: beyond this the position error's cost grows only linearly (a far goal pulls no
                                 # harder than a 3 cm error)
LOADED = ("spine_pitch", "upperarm.L_pitch", "thigh.L_pitch", "shank.L_pitch", "foot.L_pitch")
T_FREE, W_TAU = 0.2, 0.1         # a loaded joint's static torque is free up to T_FREE of its continuous rating; past
                                 # it, W_TAU of rating costs the solver as much as W_POS of position
T_MAX, W_WALL = 0.7, 0.005       # past T_MAX the cost is a wall: the clutch slips at 1.03 of the rating (W-max)
PERCH_FWD, PERCH_Z, PERCH_T = 0.06, 1.13, 2.5   # m, m, s: once B holds A's arms, the helper's anchor moves (over
                                 # PERCH_T) to B's strong spot: its elbows PERCH_FWD ahead of its soles' centre at height
                                 # PERCH_Z, its upper arms near vertical. A comes to B; B does not reach for A
F_RES = 20.0                     # N: the load (down, both elbows) it plans for beyond the measured one
P_DES = 0.0                      # m: where it keeps its CoP, ahead (-) of its soles' centre
REST = dict(ankle=math.degrees(st.STAND_LEAN), knee=0.0, hip=0.0, spine=0.0, shoulder=-20.0)   # degrees
SIGMA = dict(ankle=5.0, knee=40.0, hip=30.0, spine=5.0, shoulder=60.0)   # degrees: how freely each leaves REST
B_FOREARM = -80.0                # degrees: B's elbows' bend, its forearms forward under A's upper arms
LIMITS = dict(ankle=(-15, 10), knee=(0, 110), hip=(-100, 15), spine=(-5, 20), shoulder=(-150, 45))
KEYS = ("ankle", "knee", "hip", "spine", "shoulder")
JOINT = dict(ankle="foot.%s_pitch", knee="shank.%s_pitch", hip="thigh.%s_pitch", shoulder="upperarm.%s_pitch")
TRACE = False


def wide():
    """The roll targets of B's wide stance: hips out, soles flat."""
    return {"thigh.L_roll": -STANCE, "thigh.R_roll": STANCE, "foot.L_roll": STANCE, "foot.R_roll": -STANCE}


def build(base_y=None):
    """The two-robot model and export_mjcf's winch list (the same for both)."""
    xml, winches, _ = export_mjcf.build()
    a = mujoco.MjSpec.from_string(xml)
    b = mujoco.MjSpec.from_string(xml)
    for g in list(b.geoms):
        if g.name == "floor":
            b.delete(g)
    for el in list(b.lights):
        b.delete(el)
    frame = a.worldbody.add_frame(pos=[0.0, BASE_Y if base_y is None else base_y, 0.0], quat=[0, 0, 0, 1])
    a.attach(b, prefix=PREFIX, frame=frame)
    for s in "LR":
        e = a.add_equality(name="duo." + s, type=mujoco.mjtEq.mjEQ_CONNECT, objtype=mujoco.mjtObj.mjOBJ_SITE,
                           name1="elbow." + s, name2=PREFIX + "elbow." + OTHER[s], active=False)
        e.solref = SOLREF
    # the arms pass through each other: the cradle is the connect, not a contact (a forearm under an upper arm)
    for x in ARMS:
        for y in ARMS:
            for s1 in "LR":
                for s2 in "LR":
                    a.add_exclude(bodyname1=x + s1, bodyname2=PREFIX + y + s2)
    return a.compile(), winches, xml


class Duo:
    """Robot A (`a`, no prefix: getup.py and helper.py run on it unchanged) and robot B (`b`) in one model."""

    def __init__(self, base_y=None):
        self.base = (0.0, BASE_Y if base_y is None else base_y, math.pi)
        self.model, winches, self.xml = build(self.base[1])
        with contextlib.redirect_stdout(io.StringIO()):
            self.a = st.Rig(model=self.model, winches=winches)
            self.b = st.Rig(model=self.model, data=self.a.data, prefix=PREFIX, base=self.base, winches=winches)
        a, b = self.a, self.b
        self.data = a.data
        a.partners = [b]
        a.shared = b.shared = True
        a.on_reset = lambda rig: rig.data is b.data and self.reset_b()     # A's reset() wipes the data: B after it
        self.reset_b()
        m = self.model
        self.eq = {s: m.equality("duo." + s).id for s in "LR"}
        self.a_elbow = {s: m.site("elbow." + s).id for s in "LR"}
        self.b_elbow = {s: m.site(PREFIX + "elbow." + OTHER[s]).id for s in "LR"}     # by A's side

    def reset_b(self):
        """B at its base, standing with its feet apart (STANCE), every gear and drum homed there."""
        b = self.b
        b.reset()
        q = wide()
        getup._home(b, q)
        b.target.update(q)
        b.applied.update(q)

    def touching(self):
        """The contacts between the two robots: [(A's body, B's body, normal force N)]."""
        m, d = self.model, self.data
        out, f6 = [], np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            r1, r2 = m.body_rootid[b1], m.body_rootid[b2]
            if {int(r1), int(r2)} == {self.a.root, self.b.root}:
                mujoco.mj_contactForce(m, d, i, f6)
                if r1 != self.a.root:
                    b1, b2 = b2, b1
                out.append((m.body(b1).name, m.body(b2).name, float(f6[0])))
        return out


class Fell(Exception):
    """Raised from the step hook once either robot is down: the run is over."""


class Posture:
    """B's own kinematics with its feet flat, in its heading frame (forward -y), from its soles' centre: a private
    one-robot model, nothing simulated."""

    def __init__(self, xml):
        m = self.m = mujoco.MjModel.from_xml_string(xml)
        self.d = mujoco.MjData(m)
        adr = lambda n: int(m.jnt_qposadr[m.joint(n).id])   # noqa: E731
        self.adr = {k: [adr(JOINT[k] % s) for s in "LR"] for k in JOINT}
        self.adr["spine"] = [adr("spine_pitch")]
        self.adr["elbow"] = [adr("forearm.%s_pitch" % s) for s in "LR"]
        self.wide = [(adr(k), v) for k, v in wide().items()]
        self.site = m.site("elbow.L").id
        self.foot = m.body("foot.L").id
        self.pelvis = m.body("pelvis").id
        self.mass = float(m.body_subtreemass[self.pelvis])
        # the statics of the joints that carry the load (docs/47): each one's continuous torque, and the bodies it
        # carries (its subtree; for a leg joint everything else, shared by the two legs)
        spec = json.load(open(os.path.join(ROOT, "sim", "robot_standin.json")))
        cont = {x["name"]: x["line_cont_n"] * x["moment_arm_m"] for x in spec["dofs"]}
        bodies = [b for b in range(m.nbody) if m.body_rootid[b] == self.pelvis]

        def under(x, b):
            while x != 0:
                if x == b:
                    return True
                x = m.body_parentid[x]
            return False

        self._rest = np.array([D(REST[k]) for k in KEYS])
        self._sigma = np.array([D(SIGMA[k]) for k in KEYS])
        self._lo = np.array([D(LIMITS[k][0]) for k in KEYS])
        self._hi = np.array([D(LIMITS[k][1]) for k in KEYS])
        self.loaded = []
        for name in LOADED:
            j = m.joint(name).id
            sub = [x for x in bodies if under(x, m.jnt_bodyid[j])]
            leg = name.startswith(("thigh.", "shank.", "foot."))
            idx = np.array([x for x in bodies if x not in sub] if leg else sub)
            self.loaded.append((name, j, idx, m.body_mass[idx].copy(), 0.5 if leg else 1.0, cont[name]))

    def fk(self, u):
        """u: (ankle, knee, hip, spine, shoulder) rad. Returns (elbow y, elbow z, CoM y) from the sole's centre."""
        m, d = self.m, self.d
        d.qpos[:] = m.qpos0
        for k, v in zip(KEYS, u):
            for a in self.adr[k]:
                d.qpos[a] = v
        for a in self.adr["elbow"]:
            d.qpos[a] = D(B_FOREARM)
        for a, v in self.wide:
            d.qpos[a] = v
        pitch = -(u[0] + u[1] + u[2])                 # the body pitch that keeps the soles flat
        d.qpos[3:7] = (math.cos(pitch / 2), math.sin(pitch / 2), 0.0, 0.0)
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        sole = d.xpos[self.foot] + d.xmat[self.foot].reshape(3, 3) @ np.array(st.SOLE_CENTRE)
        c = d.site_xpos[self.site] - sole
        self._c = d.site_xpos[self.site].copy()
        return np.array((c[1], c[2], d.subtree_com[self.pelvis][1] - sole[1]))

    def ratios(self, f):
        """After fk(): each loaded joint's static torque over its continuous rating, with the load f (y, z; N, both
        elbows together) on the elbows. Sagittal statics, as role_flip.Statics.loads."""
        d = self.d
        out = []
        for name, j, idx, mass, share, cont in self.loaded:
            p = d.xanchor[j]
            tau = -9.81 * float(mass @ (d.xipos[idx, 1] - p[1])) + (self._c[1] - p[1]) * f[1] - (self._c[2] - p[2]) * f[0]
            out.append(share * tau / cont)
        return np.array(out)

    def resid(self, u, goal, f):
        """(elbow error y, z; CoP offset from P_DES) for the goal (y, z) and the load on the elbows f (y, z; N)."""
        cy, cz, com = self.fk(u)
        w = self.mass * 9.81
        cop = (w * com - cy * f[1] + cz * f[0]) / max(w - f[1], 0.3 * w)
        r = np.abs(self.ratios(f))
        over = np.maximum(0.0, r - T_FREE)       # the joints past T_FREE of their rating
        self._wall = float(np.sum(np.maximum(0.0, r - T_MAX) ** 2))
        return np.concatenate(((cy - max(goal[0], -REACH), cz - max(goal[1], Z_LOW), cop - P_DES), over))

    def cost(self, u, goal, f):
        """The solver's cost at u: the elbows' error (linear beyond E_MAX, so a far goal never outweighs the balance),
        the CoP's offset, the loaded joints past T_FREE, and the distance from REST."""
        g = self.resid(u, goal, f)
        n = float(np.linalg.norm(g[:2]))
        pos = n * n if n < E_MAX else E_MAX * (2 * n - E_MAX)
        rest = (u - self._rest) / self._sigma
        return (pos / W_POS ** 2 + (g[2] / W_COP) ** 2 + float(g[3:] @ g[3:]) / W_TAU ** 2 + self._wall / W_WALL ** 2
                + float(rest @ rest)), g

    def step(self, u, goal, f, max_du):
        """One descent step from u: per angle, a Newton step on the cost along that angle (from three values), or
        max_du downhill where the cost is not convex; each angle moves at most max_du and stays within LIMITS."""
        c0, g0 = self.cost(u, goal, f)
        h = D(0.5)
        new = u.copy()
        for i in range(len(u)):
            du = np.zeros(len(u))
            du[i] = h
            cp, cm = self.cost(u + du, goal, f)[0], self.cost(u - du, goal, f)[0]
            g1, g2 = (cp - cm) / (2 * h), (cp + cm - 2 * c0) / (h * h)
            step = -g1 / g2 if g2 > 1e-9 else -math.copysign(max_du, g1)
            new[i] = min(max(u[i] + min(max(step, -max_du), max_du), self._lo[i]), self._hi[i])
        return new, g0


class Carer:
    """Robot B's policy (its Rig.script, run before every step): idle, approach A's elbows, hold, retire."""

    def __init__(self, duo):
        self.duo, self.b = duo, duo.b
        self.body = Posture(duo.xml)
        self.u = np.array([D(REST[k]) for k in KEYS])
        self.mode, self.goal = "idle", None
        self.f = np.zeros(3)             # the load on B's elbows (A's push on them), world, filtered
        self.err, self.near = 1.0, 0.0
        self.resid = np.zeros(3 + len(LOADED))
        self.bias = np.zeros(2)
        self.peak, self.slip0 = {}, None
        self.fell = False
        self._n = 0
        self._next = 0.0

    def load(self):
        d = self.duo.data
        f = np.zeros(3)
        for s in "LR":
            e = self.duo.eq[s]
            if d.eq_active[e]:
                i = np.flatnonzero((d.efc_id == e) & (d.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY))
                if len(i):
                    f -= d.efc_force[i[0]:i[0] + 3]
        return f

    def cradle(self, s):
        return self.duo.data.site_xpos[self.duo.b_elbow[s]]

    def ready(self):
        return self.near >= TAKE_S

    def update(self, b):
        d, m = b.data, b.model
        dt = m.opt.timestep
        self.f += dt / (F_TAU + dt) * ((self.load() if self.mode == "hold" else 0.0) - self.f)
        mid = 0.5 * (self.cradle("L") + self.cradle("R"))
        kg = max(0.0, -self.f[2]) / 9.81
        b._felt_f = (kg, mid.copy()) if kg > 0.05 else (0.0, None)   # its balance loop counts the load as weight
        z, tilt = b.pelvis_state()
        if z < 0.6 or tilt > 60:         # a helper bows: fallen is lower or flatter than that
            self.fell = True
        if self.goal is not None and self.mode in ("approach", "hold"):
            self.err = max(float(np.linalg.norm(self.cradle(s) - self.goal[s])) for s in "LR")
            self.near = self.near + dt if self.err < TAKE_TOL else 0.0
        if self.mode == "hold":
            for w in b.w:
                r = max(b.line_force(w["tA"]), b.line_force(w["tB"])) / w["p"]["f_cont"]
                if r > self.peak.get(w["name"], 0.0):
                    self.peak[w["name"]] = r
        self._n += 1
        if self._n % TICK:
            return
        if self.mode in ("approach", "hold") and self.goal is not None:
            so = [d.xpos[f] + d.xmat[f].reshape(3, 3) @ np.array(st.SOLE_CENTRE) for f in b.feet]
            rel = b.loc(0.5 * (self.goal["L"] + self.goal["R"]) - 0.5 * (so[0] + so[1]))
            fl = b.loc(self.f) + np.array((0.0, 0.0, -F_RES))   # it stands ready for F_RES more than it carries
            # what the solver's model misses (line stretch, the balance loop's own lean): an integral on the measured
            # elbows against the goal, while the goal is within reach
            act = b.loc(mid - 0.5 * (so[0] + so[1]))
            if rel[1] > -REACH and rel[2] > Z_LOW:
                self.bias = np.clip(self.bias + K_BIAS * TICK * dt * (rel[1:3] - act[1:3]), -BIAS_MAX, BIAS_MAX)
            self.u, self.resid = self.body.step(self.u, rel[1:3] + self.bias, fl[1:3], RATE * TICK * dt)
        else:
            rest = np.array([D(REST[k]) for k in KEYS])
            self.u = self.u + np.clip(rest - self.u, -RATE * TICK * dt, RATE * TICK * dt)
        for k, v in zip(KEYS, self.u):
            if k == "spine":
                b.target["spine_pitch"] = float(v)
            else:
                for s in "LR":
                    b.target[JOINT[k] % s] = float(v)
        for s in "LR":
            b.target["forearm.%s_pitch" % s] = D(B_FOREARM)
        if TRACE and d.time >= self._next:
            self._next = d.time + 0.25
            z, tilt = b.pelvis_state()
            so = [d.xpos[f] + d.xmat[f].reshape(3, 3) @ np.array(st.SOLE_CENTRE) for f in b.feet]
            act = b.loc(mid - 0.5 * (so[0] + so[1]))
            pred = self.body.fk(self.u)
            q = [math.degrees(d.qpos[b.by_name[n]["q"]]) for n in ("foot.L_pitch", "shank.L_pitch", "thigh.L_pitch",
                                                                   "spine_pitch", "upperarm.L_pitch")]
            print("      elbow from soles: actual y %+.3f z %.3f, solver y %+.3f z %.3f; joints %s"
                  % (act[1], act[2], pred[0], pred[1], " ".join("%+4.0f" % x for x in q)), file=sys.__stdout__,
                  flush=True)
            ra = self.duo.a
            sa = ra.soles()
            print("  A t %5.2f %-17s pelvis y %+.2f z %.2f tilt %4.1f  soles y L %+.2f R %+.2f  loads %3.0f %3.0f N  elbow y %+.2f z %.2f"
                  % (d.time, ra.phase, d.xpos[ra.pelvis][1], d.xpos[ra.pelvis][2], ra.pelvis_state()[1], sa[0][0][1],
                     sa[1][0][1], *ra.cop()[1], *d.site_xpos[self.duo.a_elbow["L"]][1:3]), file=sys.__stdout__)
            print("    B %-8s err %.3f  resid %+.3f %+.3f cop %+.3f over %s  load y %+4.0f z %+4.0f N  u %s  pelvis %.2f tilt %4.1f"
                  % (self.mode, self.err, self.resid[0], self.resid[1], self.resid[2],
                     " ".join("%.2f" % x for x in self.resid[3:]), b.loc(self.f)[1], self.f[2],
                     " ".join("%+4.0f" % math.degrees(x) for x in self.u), z, tilt), file=sys.__stdout__, flush=True)


class Cradle(getup.Lifter):
    """getup.Lifter with a robot for hands. With `catch`, the scripted hands hold A from the lean as in docs/46, and
    B takes over once its elbows are on A's (ready), from phase `take` of ORDER on; without, nobody holds A until B
    does. From then
    the Lifter's commanded points are B's goals, and the force it feels is the cradle's."""

    def __init__(self, rig, duo, carer, catch=True, take=None, **cfg):
        super().__init__(rig, **cfg)
        self.take_from = TAKE if take is None else int(take)
        self.duo, self.carer, self.catch = duo, carer, catch
        self.by = "script"
        self.t_take = self.take_phase = None
        self.imp = dict(script=0.0, robot=0.0)
        self.peak_by = dict(script=0.0, robot=0.0)
        self.sides = self.hands
        if not catch:
            self.hands = ""

    def anchor(self, s):
        x = super().anchor(s)
        if self.by != "robot":
            return x
        u = min(1.0, (self.r.data.time - self.t_take) / PERCH_T)
        u = u * u * (3 - 2 * u)
        return x + u * (self.perch[s] - x)

    def take(self):
        d = self.r.data
        for s in "LR":
            if self.attached(s):
                d.eq_active[self.eq[s]] = 0
                d.mocap_pos[self.mid[s]] = (3.0 if s == "L" else 4.0, 5.0, -1.0)
            self.eq[s] = self.duo.eq[s]
        self.hands = self.sides
        for s in self.hands:
            self.attach(s)
        self.by = "robot"
        self.t_take, self.take_phase = float(d.time), self.r.phase
        self.no_pull_down = True
        b = self.duo.b
        so = [d.xpos[f] + d.xmat[f].reshape(3, 3) @ np.array(st.SOLE_CENTRE) for f in b.feet]
        mid = 0.5 * (so[0] + so[1])
        self.perch = {s: np.array((d.site_xpos[self.grip[s]][0], mid[1] + PERCH_FWD, PERCH_Z)) for s in "LR"}
        self.carer.mode = "hold"
        self.carer.slip0 = sum(v["slip"] for v in self.duo.b.stats.values())

    def update(self, r):
        d = r.data
        c = self.carer
        phase = r.phase or ""
        if self.by == "script" and not self.done and phase.startswith("stand: "):
            c.mode = "approach"
            c.goal = {s: d.site_xpos[self.grip[s]].copy() for s in "LR"}
            if c.ready() and phase in ORDER and ORDER.index(phase) >= self.take_from and self.path is not None:
                self.take()
        i0, p0 = self.log["impulse"]["L"] + self.log["impulse"]["R"], self.log["peak"]
        self.log["peak"] = 0.0
        super().update(r)
        self.imp[self.by] += self.log["impulse"]["L"] + self.log["impulse"]["R"] - i0
        self.peak_by[self.by] = max(self.peak_by[self.by], self.log["peak"])
        self.log["peak"] = max(p0, self.log["peak"])
        if self.by == "robot" and not self.done:
            c.goal = {s: self.x[s].copy() for s in "LR"}
        if self.done and c.mode != "retire":
            c.mode = "retire"


def run(cfg=None, which="firm", video=None, gif=False):
    """One get-up with B as the helper. cfg: catch (the scripted hands hold A until B takes over), base (B's ankles'
    y), and getup.Lifter's give, cap. Returns the result (also printed as a RESULT line)."""
    cfg = dict(cfg or {})
    params = json.load(open(os.path.join(ROOT, "sim", "getup_params.json")))[which]
    getup.gapply(dict(params["params"], **cfg.pop("gp", {})))
    getup.ELBOW = A_ELBOW
    cfg.pop("applied", None)
    duo = Duo(cfg.pop("base", None))
    a, b, d = duo.a, duo.b, duo.data
    carer = Carer(duo)
    b.script = carer.update
    with contextlib.redirect_stdout(io.StringIO()):
        getup.kneel(a)
    a.set_phase("kneel: fold")              # the forearms up first: swung forward straight, the hands hit B's legs
    a.move({"forearm.%s_pitch" % s: A_ELBOW for s in "LR"}, FOLD_T)
    h = Cradle(a, duo, carer, catch=bool(cfg.pop("catch", True)), **dict(params["helper"], **cfg))
    a.lifter = h
    hits = {}

    first = {}

    zp = [0.0]

    def watch(r):
        za = d.xpos[a.pelvis][2]
        zp[0] = max(zp[0], za)
        if (r.phase or "").startswith("stand: ") and (za < 0.3 or carer.fell):
            raise Fell()
        if a.steps % 5:
            return
        for x in duo.touching():
            hits[x[:2]] = max(hits.get(x[:2], 0.0), x[2])
            if x[:2] not in first and x[2] > 5.0:
                first[x[:2]] = d.time
                if TRACE:
                    print("  ** t %.2f %s: %s touches B's %s" % (d.time, r.phase, x[0], x[1][2:]), file=sys.__stdout__)

    rec = None
    if video:
        import duo_video
        out = os.path.join(ROOT, "docs", "img", video + ".mp4")
        rec = duo_video.Recorder(duo, out, gif=out[:-4] + ".gif" if gif else None)
        rec.scene("duo", "One robot helps another up from a half-kneel",
                  "orange hands: the scripted helper, until robot B (standing) has A's upper arms on its elbows")
        a.hook = lambda r: (h.update(r), watch(r), rec.on_step(r))
    else:
        a.hook = lambda r: (h.update(r), watch(r))
    getup.TRACE = TRACE
    with contextlib.redirect_stdout(io.StringIO() if not TRACE else sys.__stdout__):
        try:
            res = getup.getup(a)
        except Fell:
            z, tilt = a.pelvis_state()
            res = dict(stood=False, progress=0.0, z=float(z), tilt=float(tilt), z_peak=float(zp[0]),
                       shift_share=float(getattr(a, "shift_share", 1.0)), stage=int(getattr(a, "stage", 0)), t_stand=0.0)
    if rec:
        print("wrote %s (%d frames)" % (out, rec.close()))
    cont = {w["name"]: w["p"]["f_cont"] for w in a.w}
    top = sorted(carer.peak.items(), key=lambda kv: -kv[1])[:5]
    bz, bt = b.pelvis_state()
    res.update(b_stood=bool(bz > 0.8 and not carer.fell), b_z=float(bz), b_tilt=float(bt),
               t_take=h.t_take, take_phase=h.take_phase, imp_script=h.imp["script"], imp_robot=h.imp["robot"],
               peak_script=h.peak_by["script"], peak_robot=h.peak_by["robot"],
               a_arm={k: v / cont[k] for k, v in h.log["arm_on"].items()},
               b_worst=top, b_slip=(math.degrees(sum(v["slip"] for v in b.stats.values()) - carer.slip0)
                                    if carer.slip0 is not None else 0.0),
               hits={"%s/%s" % k: round(v) for k, v in sorted(hits.items(), key=lambda kv: -kv[1])[:6]})
    print("RESULT " + json.dumps(res))
    return res


# --- the search: A's script (getup.GPARAMS) and B's place and reach together, B in the loop -----------------------
DPARAMS = [("BASE_Y", -0.60, -0.90, -0.45), ("B.REACH", 0.25, 0.08, 0.40), ("B.F_RES", 20.0, 0.0, 60.0),
           ("B.Z_LOW", 1.0, 0.85, 1.10), ("B.PERCH_FWD", 0.06, -0.05, 0.25), ("B.PERCH_Z", 1.13, 0.95, 1.25),
           ("B.PERCH_T", 2.5, 1.0, 5.0), ("B.B_FOREARM", -80.0, -120.0, -20.0)]
JITTER = 0.02
START = dict(STRIDE=0.30, ARM=-60.0, Z_TOP=-0.15)        # the search's start, on top of sim/getup_params.json's firm set


def dapply(p):
    """Split a parameter set: getup's (returned), this module's ("B." names) and B's place (returned as base)."""
    gp, base = {}, None
    for k, v in p.items():
        if k == "BASE_Y":
            base = v
        elif k.startswith("B."):
            globals()[k[2:]] = v
        else:
            gp[k] = v
    return gp, base


def drun(job):
    """One get-up for the search: (params, config, jitter seed or None). Quiet; returns run()'s result."""
    p, cfg, seed = job
    if seed is not None:
        rng = np.random.default_rng(seed)
        p = {k: v + JITTER * (abs(v) if abs(v) > 0.05 else 0.05) * rng.standard_normal() for k, v in p.items()}
    gp, base = dapply(p)
    with contextlib.redirect_stdout(io.StringIO()):
        try:
            return run(dict(cfg, gp=dict(gp, **cfg.get("gp", {})), base=cfg.get("base", base)))
        except Exception as e:          # a diverged run scores as a fall
            return dict(stood=False, b_stood=False, z_peak=0.0, stage=0, t_stand=0.0, imp_script=1.0, imp_robot=0.0,
                        b_worst=[], a_arm={}, hits={}, t_take=None, error=repr(e))


def success(res):
    """Both stand at the end, and B held A on the way."""
    return bool(res["stood"] and res["b_stood"] and res.get("t_take") is not None)


def dscore(res):
    """10 if both stand at the end (A 3 s after B lets go) and B held A; else A's progress as getup.gscore (height,
    stage, time stood; at most 6), less 3 if B fell. Plus up to 4 for the share of the helper's impulse B carried;
    minus 2 per unit of B's worst line over its rating, of A's worst arm line over 1.42 (D-070), and 1 per 200 N of
    the hardest contact between them over 60 N."""
    if success(res):
        s = 10.0
    else:
        s = min(6.0, 4.0 * min(1.0, res["z_peak"] / 0.9) + 2.0 * res.get("stage", 0) / 4.0
                + 3.0 * res.get("t_stand", 0.0) / getup.HOLD_S) - (0.0 if res["b_stood"] else 3.0)
        if res.get("t_take") is None:    # B stood by: not the experiment
            s = min(s, 2.0)
    s += 4.0 * res["imp_robot"] / max(res["imp_robot"] + res["imp_script"], 1e-9)
    s -= 2.0 * max(0.0, max([v for _, v in res["b_worst"]] or [0.0]) - 1.0)
    s -= 2.0 * max(0.0, max(res["a_arm"].values() or [0.0]) - getup.ARM_REF)
    s -= max(0.0, max(res["hits"].values() or [0.0]) - 60.0) / 200.0
    return s


def train(gens, pop, jobs, seed, out, cfg, init=None, reps=2, levels=None):
    """A cross-entropy search over getup.GPARAMS and DPARAMS, as getup.gtrain: `reps` jittered runs per candidate,
    scored by their mean. best.json keeps the best, the last generation's best and the mean."""
    import multiprocessing as mp
    os.makedirs(out, exist_ok=True)
    rng = np.random.default_rng(seed)
    P = list(getup.GPARAMS) + DPARAMS
    lo, hi = np.array([x[2] for x in P]), np.array([x[3] for x in P])
    names = [x[0] for x in P]
    firm = json.load(open(os.path.join(ROOT, "sim", "getup_params.json")))["firm"]["params"]
    base = dict({x[0]: x[1] for x in P}, **firm)
    base.update(START)
    base.update(init or {})
    to_u = lambda p: (np.array([p[k] for k in names]) - lo) / (hi - lo)   # noqa: E731
    from_u = lambda u: {k: float(v) for k, v in zip(names, lo + np.clip(u, 0, 1) * (hi - lo))}   # noqa: E731
    mean, std = to_u(base), np.full(len(P), 0.10)
    best = None
    n_el = max(2, pop // 4)
    level, per_level = 0, {}
    with mp.get_context("spawn").Pool(jobs, maxtasksperchild=4) as pool:
        for g in range(gens):
            if levels:
                cfg = dict(cfg, take=levels[level])
            us = [mean] + [mean + std * rng.standard_normal(len(P)) for _ in range(pop - 1)]
            ps = [from_u(u) for u in us]
            sds = rng.integers(1 << 30, size=(pop, reps))
            rr = pool.map(drun, [(p, cfg, int(sds[i, k])) for i, p in enumerate(ps) for k in range(reps)], chunksize=1)
            groups = [rr[i * reps:(i + 1) * reps] for i in range(pop)]
            sc = [float(np.mean([dscore(x) for x in gr])) for gr in groups]
            ok = [all(success(x) for x in gr) for gr in groups]
            order = np.argsort(sc)[::-1]
            i0 = int(order[0])
            w = min(groups[i0], key=dscore)
            if best is None or sc[i0] > best["score"]:
                best = dict(score=sc[i0], params=ps[i0], res=w, gen=g)
            print("%s gen %2d best %5.2f mean %5.2f both stand %2d/%d (runs %d/%d)  best's worst run: A stage %d %s, B %s, "
                  "B's share %.0f%%, take %s, B line %.2fx, hit %.0f N"
                  % (time.strftime("%H:%M:%S"), g, sc[i0], float(np.mean(sc)), sum(ok), pop, sum(success(x) for x in rr), len(rr),
                     w.get("stage", 0), "stands" if w["stood"] else "falls", "stands" if w["b_stood"] else "falls",
                     100 * w["imp_robot"] / max(w["imp_robot"] + w["imp_script"], 1e-9),
                     ("%.1f s" % w["t_take"]) if w.get("t_take") else "-", max([v for _, v in w["b_worst"]] or [0.0]),
                     max(w["hits"].values() or [0.0])), flush=True)
            el = [us[i] for i in order[:n_el]]
            frac = float(np.mean([ok[i] for i in order[:n_el]]))
            if levels and ok[i0]:
                per_level[str(levels[level])] = dict(params=ps[i0], gen=g, score=sc[i0], cfg=cfg)
            mean = 0.3 * mean + 0.7 * np.mean(el, axis=0)
            std = np.clip(0.5 * std + 0.5 * np.std(el, axis=0), 0.02, 0.25)
            if levels:
                print("        take from %d (%s): the elite succeeding %.2f" % (levels[level], ORDER[levels[level]], frac),
                      flush=True)
                if frac >= 0.75 and level < len(levels) - 1:
                    level += 1
                    best = None
                    mean, std = us[i0], np.maximum(std, 0.05)
                    print("  -> take from %d (%s)" % (levels[level], ORDER[levels[level]]), flush=True)
            json.dump(dict(best=best, cfg=cfg, last=dict(params=ps[i0], gen=g, score=sc[i0]), mean=from_u(mean),
                           levels=per_level), open(os.path.join(out, "best.json"), "w"), indent=1)
    return best


def jitter_test(path, key, n, jobs, cfg=None):
    """`key` of a search's best.json ("best", "last", "mean") n times, every parameter jittered by JITTER."""
    import multiprocessing as mp
    p, c0 = load_params(path + ":" + key)
    cfg = dict(c0, **(cfg or {}))
    with mp.get_context("spawn").Pool(jobs, maxtasksperchild=4) as pool:
        rs = pool.map(drun, [(p, cfg, 1000 + i) for i in range(n)], chunksize=1)
    both = [r for r in rs if success(r)]
    share = [r["imp_robot"] / max(r["imp_robot"] + r["imp_script"], 1e-9) for r in rs]
    bw = [max([v for _, v in r["b_worst"]] or [0.0]) for r in rs]
    print("%s %s %s: both stand, B having held A, %d of %d (A %d, B %d; jitter %.0f%%); B's share of the impulse %.0f%% mean; B's worst "
          "line %.2fx max, %.2fx median; A's worst arm line %.2fx max; hardest contact %.0f N median"
          % (path, key, json.dumps(cfg), len(both), n, sum(r["stood"] for r in rs), sum(r["b_stood"] for r in rs),
             100 * JITTER, 100 * float(np.mean(share)), max(bw), float(np.median(bw)),
             max(max(r["a_arm"].values() or [0.0]) for r in rs),
             float(np.median([max(r["hits"].values() or [0.0]) for r in rs]))), flush=True)
    return rs


def load_params(spec):
    """`file:key` of a search's best.json: key is best, last, mean or levels.<n>. Returns (params, config)."""
    path, key = spec.rsplit(":", 1)
    d = json.load(open(path))
    x = d
    for k in key.split("."):
        x = x[k]
    return (x["params"] if "params" in x else x), dict(d.get("cfg", {}), **x.get("cfg", {}))


def show(res):
    print("A %s (stage %d, stood %.1f s after)   B %s (pelvis %.2f m, tilt %.0f deg)"
          % ("STANDS" if res["stood"] else "FALLS", res["stage"], res["t_stand"],
             "stands" if res["b_stood"] else "FALLS", res["b_z"], res["b_tilt"]))
    if res["t_take"] is None:
        print("B never took A's arms; the scripted hands: %.0f N s, peak %.0f N" % (res["imp_script"], res["peak_script"]))
    else:
        tot = max(res["imp_script"] + res["imp_robot"], 1e-9)
        print("B took over at %.1f s (%s): it carried %.0f of %.0f N s (%.0f%%), peak %.0f N; the scripted hands' "
              "peak %.0f N" % (res["t_take"], res["take_phase"], res["imp_robot"], tot, 100 * res["imp_robot"] / tot,
                               res["peak_robot"], res["peak_script"]))
    print("B's lines while holding (peak / continuous): " + ", ".join("%s %.2fx" % (k, v) for k, v in res["b_worst"])
          + "; clutch slip %.0f deg" % res["b_slip"])
    print("A's arm lines while held: %.2fx worst" % max(res["a_arm"].values() or [0.0]))
    print("contacts between them (N): %s" % res["hits"])


def probe(which="firm"):
    """docs/46's scripted get-up in the two-robot model, B standing by: where A's parts go, for placing B."""
    params = json.load(open(os.path.join(ROOT, "sim", "getup_params.json")))[which]
    getup.gapply(params["params"])
    duo = Duo(-1.3)
    a, b, m, d = duo.a, duo.b, duo.model, duo.data
    with contextlib.redirect_stdout(io.StringIO()):
        getup.kneel(a)
    h = getup.Lifter(a, **params["helper"])
    a.lifter = h
    last = [None, 0.0]

    def hook(r):
        h.update(r)
        if r.phase != last[0] or d.time >= last[1]:
            last[0], last[1] = r.phase, d.time + 1.0
            so = r.soles()
            el = d.site_xpos[duo.a_elbow["L"]]
            bz, bt = b.pelvis_state()
            print("t %5.2f %-18s A pelvis y %+.2f z %.2f  elbow y %+.2f z %.2f  soles y L %+.2f R %+.2f  "
                  "spine y %+.2f | B pelvis z %.2f tilt %4.1f" % (
                      d.time, r.phase, d.xpos[a.pelvis][1], d.xpos[a.pelvis][2], el[1], el[2], so[0][0][1],
                      so[1][0][1], d.xpos[m.body("spine").id][1], bz, bt), file=sys.__stdout__, flush=True)

    a.hook = hook
    with contextlib.redirect_stdout(io.StringIO()):
        res = getup.getup(a)
    print("A:", res)
    print("B stands:", b.standing(), "pelvis", b.pelvis_state())


def _parse(argv):
    cfg, rest = {}, []
    g = globals()
    for x in argv:
        if "=" in x and not x.startswith("--"):
            k, v = x.split("=", 1)
            if k in ("catch", "base", "give", "cap", "drunk", "seed", "take"):
                cfg[k] = eval(v)
            elif k in [x[0] for x in getup.GPARAMS]:
                cfg.setdefault("gp", {})[k] = float(v)
            elif k.isupper() and k in g:
                g[k] = eval(v, vars(math) | g)
            elif k.isupper() and hasattr(getup, k):
                setattr(getup, k, eval(v, vars(math) | vars(getup)))
            else:
                setattr(st, k, eval(v, vars(st)))
        else:
            rest.append(x)
    return cfg, rest


if __name__ == "__main__":
    argv = sys.argv[1:]
    if "--probe" in argv:
        probe()
    elif "--train" in argv:
        argv.remove("--train")
        cfg, rest = _parse(argv)
        opt = lambda k, dflt: rest[rest.index(k) + 1] if k in rest else dflt   # noqa: E731
        init = load_params(opt("--init", ""))[0] if "--init" in rest else None
        lv = [x for x in LEVELS if x <= int(opt("--from", LEVELS[0]))]
        train(int(opt("--gens", 20)), int(opt("--pop", 13)), int(opt("--jobs", 13)), int(opt("--seed", 0)),
              opt("--out", "sim/out/duo"), cfg, init=init, reps=int(opt("--reps", 2)),
              levels=lv if "--levels" in rest else None)
    elif "--jitter-test" in argv:
        cfg, rest = _parse([x for x in argv if x != "--jitter-test"])
        path, key = rest[0].rsplit(":", 1)
        opt = lambda k, dflt: rest[rest.index(k) + 1] if k in rest else dflt   # noqa: E731
        jitter_test(path, key, int(opt("--n", 24)), int(opt("--jobs", 12)), cfg)
    elif "--one" in argv:
        argv.remove("--one")
        cfg, rest = _parse(argv)
        TRACE = "--trace" in rest
        video = rest[rest.index("--video") + 1] if "--video" in rest else None
        if "--params" in rest:              # a search's set: --params sim/out/duo_x/best.json:levels.6
            p, c0 = load_params(rest[rest.index("--params") + 1])
            gp, base = dapply(p)
            cfg = dict(c0, **dict(dict(gp=dict(gp, **cfg.pop("gp", {})), base=base), **cfg))
        show(run(cfg, video=video, gif="--gif" in rest))
    else:
        print(__doc__)
