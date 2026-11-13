"""
getup.py - getting up from a half-kneel on the full tendon model, with the nurturing helper (docs/46, hub task
helper-instructor-getup).

The start is the stand-in's resting half-kneel (docs/32, the sibling's KNEEL_STARTS): front hip flexed 90 degrees,
knee 95, ankle -5; back hip +5, knee 100; spine 10. `kneel(r)` homes the rig there the way reset() homes it standing
(every gear and drum at the angle the pose needs), drops it onto the floor from the highest root height that touches,
and lets it settle with the drives holding the pose.

The rise is scripted, in phases the helper knows (helper.py arms on "stand: lean" and lets go on "stand: up"):
    stand: lean   the arms reach forward to ARM, the trunk leans to SPINE_LEAN; the helper takes the wrists
    stand: rise   every joint eases from the kneel to the standing pose over RISE_T s, gated while the CoM is more
                  than GATE behind the front sole (the helper pulls by how far it is behind)
    stand: shift  the weight onto the front foot, the helper drawing forward, till the back foot is nearly unloaded
    stand: step   the back foot lifts, swings through and is planted beside the front foot (the helper holding)
    stand: up     every joint to the standing pose, the balance loop on the ankles; then the helper lets go

    python tools/getup.py --settle                   # the half-kneel alone: does it rest?
    python tools/getup.py --one B [--trace] [K=V ...] [--video name [--gif]]
    python tools/getup.py --phases [--jobs 7]        # the per-phase table
    python tools/getup.py --train A [--gens 30] [--pop 13] [--jobs 13] [--reps 2] [--init best.json]
                                                     # search the script's GPARAMS with that helper (--reps: jittered;
                                                     # --fade: the helper fades through FADE as the elite stands)
    python tools/getup.py --jitter-test sim/out/getup_A5/best.json:last [--n 24]
    GETUP_GRIP=elbow ...                             # the helper holds the upper arms (D-073), not the wrists
"""
import contextlib
import io
import json
import math
import os
import re
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import helper  # noqa: E402  (sets export_mjcf.HELPER before the model is built)
import mujoco  # noqa: E402
import sim_tendon as st  # noqa: E402

D = math.radians
FRONT, BACK = "L", "R"
KNEEL = {"thigh.F_pitch": D(-90), "shank.F_pitch": D(95), "foot.F_pitch": D(-5), "thigh.B_pitch": D(5),
         "shank.B_pitch": D(100), "foot.B_pitch": 0.0, "spine_pitch": D(10)}
SETTLE_S = 1.5
ARM = D(-100)          # the arms' reach for the helper (shoulder pitch; -90 is forward, level)
SPINE_LEAN = D(30)     # the trunk's lean before the rise
LEAN_T = 1.5           # s
RISE_T = 4.0           # s: the kneel to the standing pose, ungated
GATE = 0.12            # m: the rise waits while the CoM is this far behind the front sole's centre
SWING = (D(60), D(-15))   # the back leg's knee and hip as it lifts its foot to swing it through
LIFT_T, SWING_T, PLANT_T = 0.6, 0.8, 0.6   # s: the step's three parts
LAND_T, LAND_LOAD, LAND_HIP = 1.0, 0.3, D(10)   # s at most; the share of the weight that ends it; the hip's
                                                 # extension past the front hip's as the foot reaches for the floor
UP_T = 1.5             # s
SHIFT = dict(ankle=D(-20), knee=D(25), back_hip=D(20), spine=D(25))   # the weight shift onto the front foot
SHIFT_T = 2.5          # s at most; it ends once the back foot carries under SHIFT_LOAD of the weight
SHIFT_LOAD = 0.25
SHIFT_FWD = 0.20       # m: the helper's anchor draws forward this far over the shift
SAMPLE_DT = 0.1        # s: the helper's hand positions and forces are sampled this often (Lifter.samples)
Z_TOP = 0.0            # m: the anchor's ceiling above the standing grip height
LIFT_SLACK = 10.0      # N per hand: the lift allowed beyond the weight the feet leave uncarried
HOLD_S = 3.0           # s standing at the end
TRACE = False


def pose(kneel=KNEEL, front=FRONT, back=BACK):
    return {k.replace(".F_", ".%s_" % front).replace(".B_", ".%s_" % back): v for k, v in kneel.items()}


def _home(r, q):
    """Joint angles and their gears and drums at `q` (rad, by DOF), as reset() does for the ankles."""
    d, m = r.data, r.model
    for w in r.w:
        if w["name"] not in q:
            continue
        p = w["p"]
        ratio = p["tackle"] * p["r_p"] / p["r_d"]
        d.qpos[w["q"]] = q[w["name"]]
        d.qpos[w["gq"]] = d.qpos[w["dq_"]] = ratio * q[w["name"]]
        m.eq_data[w["eq_lock"]][0] = d.qpos[w["gq"]]


def kneel(r, kn=None):
    """Reset the rig into the half-kneel, resting on the floor, drives holding it. Returns the targets."""
    r.reset()
    m, d = r.model, r.data
    tg = dict(r.target)
    tg.update(pose(kn or KNEEL))
    for s in "LR":
        tg.setdefault("upperarm.%s_pitch" % s, 0.0)
    _home(r, tg)
    d.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
    d.qpos[0:2] = 0.0
    lo, hi = 0.0, 1.2
    for _ in range(24):                     # the highest root height at which the body touches the floor
        d.qpos[2] = 0.5 * (lo + hi)
        mujoco.mj_forward(m, d)
        touching = any(m.geom_bodyid[c.geom1] == 0 or m.geom_bodyid[c.geom2] == 0 for c in d.contact[:d.ncon])
        lo, hi = (d.qpos[2], hi) if touching else (lo, d.qpos[2])
    d.qpos[2] = lo + 0.005
    d.qvel[:] = 0.0
    mujoco.mj_forward(m, d)
    r.target.update(tg)
    r.applied = dict(r.target)
    r.balance, r.still_gate = False, True
    r.set_phase("kneel: settle")
    r.run(SETTLE_S)
    return tg


def front_behind(r):
    """How far the CoM is behind the front sole's centre (m; +: behind; forward is -y)."""
    so = r.soles()
    i = 0 if FRONT == "L" else 1
    return float(r.data.subtree_com[r.pelvis][1] - so[i][0][1])


def shift(r):
    """The weight onto the front foot: its ankle and knee bend forward, the back hip pushes, the trunk leans, and the
    helper draws forward; until the back foot carries under SHIFT_LOAD of the weight. Returns that share."""
    B, F = BACK, FRONT
    r.set_phase("stand: shift")
    start = {k: r.target[k] for k in ("foot.%s_pitch" % F, "shank.%s_pitch" % F, "thigh.%s_pitch" % B, "spine_pitch")}
    goal = {"foot.%s_pitch" % F: SHIFT["ankle"], "shank.%s_pitch" % F: SHIFT["knee"],
            "thigh.%s_pitch" % B: SHIFT["back_hip"], "spine_pitch": SHIFT["spine"]}
    dt = r.model.opt.timestep
    n = int(SHIFT_T / dt)
    ib = 0 if B == "L" else 1
    share = 1.0
    for i in range(1, n + 1):
        for k in goal:
            r.target[k] = start[k] + (goal[k] - start[k]) * i / n
        r.step()
        loads = r.cop()[1]
        share = float(loads[ib]) / (r.mass * 9.81)
        if share < SHIFT_LOAD and i * dt > 0.3:
            break
    return share


def step(r):
    """The back foot swings through to beside the front foot, the helper holding the robot meanwhile."""
    B, F = BACK, FRONT
    r.shift_share = shift(r)
    r.stage = 2 if r.shift_share < SHIFT_LOAD else 1
    r.set_phase("stand: step lift")
    r.move({"shank.%s_pitch" % B: SWING[0], "thigh.%s_pitch" % B: SWING[1]}, LIFT_T)
    r.set_phase("stand: step swing")
    hf = float(r.data.qpos[r.by_name["thigh.%s_pitch" % F]["q"]])
    r.move({"thigh.%s_pitch" % B: hf - D(15), "shank.%s_pitch" % B: SWING[0]}, SWING_T)
    r.set_phase("stand: step plant")
    q = lambda k: float(r.data.qpos[r.by_name[k % F]["q"]])
    r.move({"thigh.%s_pitch" % B: q("thigh.%s_pitch"), "shank.%s_pitch" % B: q("shank.%s_pitch"),
            "foot.%s_pitch" % B: q("foot.%s_pitch")}, PLANT_T)
    r.set_phase("stand: step land")          # lower the foot until it carries LAND_LOAD of the weight
    ib = 0 if B == "L" else 1
    goal = {"shank.%s_pitch" % B: 0.0, "thigh.%s_pitch" % B: q("thigh.%s_pitch") + LAND_HIP,
            "foot.%s_pitch" % B: st.STAND_LEAN}
    r.ramp(LAND_T, targets=goal, until=lambda _r: _r.cop()[1][ib] > LAND_LOAD * _r.mass * 9.81)
    so = r.soles()
    landed = r.cop()[1][ib] > 0.5 * LAND_LOAD * r.mass * 9.81
    if r.stage == 2 and abs(so[0][0][1] - so[1][0][1]) < 0.15 and landed:     # the feet side by side, both down
        r.stage = 3


def getup(r):
    """The scripted rise from the half-kneel (kneel(r) first). Sets r.getup_result."""
    k0 = dict(r.target)
    stand = {w["name"]: (st.STAND_LEAN if w["name"].startswith("foot.") and w["name"].endswith("_pitch") else 0.0)
             for w in r.w}
    r.set_phase("stand: lean")
    if getattr(r, "lifter", None) is not None:
        r.lifter.plan(arms_forward_wrists(r, ARM))
    lean = dict(k0)
    lean["spine_pitch"] = SPINE_LEAN
    for s in "LR":
        lean["upperarm.%s_pitch" % s] = ARM
        lean["forearm.%s_pitch" % s] = 0.0
    r.move(lean, LEAN_T)
    r.set_phase("stand: rise")
    start = dict(r.target)
    keys = [k for k in stand if not k.startswith(("upperarm.", "forearm."))]
    dt = r.model.opt.timestep
    s_ = 0.0
    z_peak = 0.0
    _next = [0.0]
    for _ in range(int(3 * RISE_T / dt)):
        e = front_behind(r)
        gate = float(np.clip((GATE - e) / max(GATE, 1e-6) * 4.0, 0.0, 1.0)) if GATE else 1.0
        s_ = min(1.0, s_ + gate * dt / RISE_T)
        for k in keys:
            r.target[k] = start[k] + (stand[k] - start[k]) * s_
        r.step()
        z_peak = max(z_peak, r.pelvis_state()[0])
        if TRACE and r.data.time >= _next[0]:
            _next[0] = r.data.time + 0.25
            z, tilt = r.pelvis_state()
            print("  t %5.2f rise %.2f  pelvis %.3f tilt %5.1f  CoM behind front sole %+.3f  gate %.2f"
                  % (r.data.time, s_, z, tilt, e, gate))
        if s_ >= 1.0:
            break
        if r.data.qpos[2] < 0.25:
            break
    r.stage = 1 if s_ >= 1.0 else 0
    if s_ >= 1.0:
        step(r)
    r.set_phase("stand: up")
    r.balance, r._com_f = True, None
    r.move(stand, UP_T)
    r.set_phase("stand: done")
    if getattr(r, "stage", 0) == 3 and r.standing():
        r.stage = 4
    t_stand = 0.0
    dt = r.model.opt.timestep
    for _ in range(int(HOLD_S / dt)):
        r.step()
        if not r.standing():
            break
        t_stand += dt
    r.t_stand = t_stand
    z, tilt = r.pelvis_state()
    r.getup_result = dict(stood=bool(r.standing()), progress=float(s_), z=float(z), tilt=float(tilt),
                          z_peak=float(z_peak), shift_share=float(getattr(r, "shift_share", 1.0)),
                          stage=int(getattr(r, "stage", 0)), t_stand=float(getattr(r, "t_stand", 0.0)))
    return r.getup_result


def settle():
    r = st.Rig()
    with contextlib.redirect_stdout(io.StringIO()):
        kneel(r)
    z, tilt = r.pelvis_state()
    print("half-kneel after %.1f s: pelvis %.3f m, tilt %.1f deg, CoM behind the front sole %+.3f m" %
          (SETTLE_S, z, tilt, front_behind(r)))
    r.run(2.0)
    z, tilt = r.pelvis_state()
    print("            after %.1f s: pelvis %.3f m, tilt %.1f deg" % (SETTLE_S + 2, z, tilt))


GRIP = os.environ.get("GETUP_GRIP", "wrist")   # where the helper holds: "wrist" (the hands), or "elbow" (the upper arms)
K_FIRM, K_SOFT = 3000.0, 300.0     # N/m per hand: the lifter's stiffness toward its anchor, firm and yielding
B_LIFT = 150.0                      # N s/m: how fast a hand gives under a force difference
LIFT = 0.45                         # m: the anchor rises this much over the rise (the pelvis's 0.46 to ~0.9 m)
LIFT_FWD = 0.0                      # m: and moves this far forward (-y)


class Lifter(helper.Helper):
    """The get-up helper: it takes both wrists at the start of the lean and holds them toward an anchor that moves
    from where the wrists hang to where the arms reach forward (over LEAN_T), then rises by LIFT over RISE_T. Each
    hand moves as v = (F_robot + F_own) / B_LIFT, F_own = K (anchor - hand), capped at `cap`: the stiffness K is the
    give (K_FIRM .. K_SOFT by `give`). The robot's pull on it, its impulse and the arm lines are logged as in
    helper.Helper; it lets go at "stand: up" or on a fall."""

    def __init__(self, rig, give=0.0, hands="LR", cap=150.0, drunk=0.0, seed=0, **_):
        super().__init__(rig, give=give, hands=hands, on_reach=False, cap=cap, drunk=drunk, seed=seed,
                         arms_hold=False)
        self.path = None
        self.t_up = None
        self.fwd = 0.0
        self.samples, self._sample_t = [], 0.0
        if GRIP == "elbow":             # under the arms: it holds each upper arm at the elbow, off the elbow's line
            m = rig.model
            self.eq = {s: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "helper.%s.elbow" % s) for s in "LR"}
            self.grip = {s: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "elbow." + s) for s in "LR"}

    def k(self, s):
        return K_FIRM * (K_SOFT / K_FIRM) ** self.give[s]

    def plan(self, fwd):
        """fwd: {side: the wrist's position with the arms forward}. Set by getup() at the lean."""
        d = self.r.data
        self.path = {s: (d.site_xpos[self.grip[s]].copy(), np.asarray(fwd[s], dtype=float)) for s in "LR"}
        self.z_top = standing_grip_z(self.r, ARM)       # the anchor never rises above the standing grip height
        self.t0 = d.time

    def anchor(self, s):
        x0, x1 = self.path[s]
        t = self.r.data.time - self.t0
        if t < LEAN_T:
            return x0 + (x1 - x0) * (t / LEAN_T)
        u = min(1.0, (t - LEAN_T) / RISE_T)
        x2 = x1 + np.array((0.0, -LIFT_FWD, LIFT)) * u + np.array((0.0, -self.fwd, 0.0))
        x2[2] = min(x2[2], self.z_top + Z_TOP)
        return x2

    def update(self, r):
        d = r.data
        dt = r.model.opt.timestep
        phase = r.phase or ""
        if self.done or self.path is None:
            return
        if phase == "stand: done" or (phase.startswith("stand: ") and d.qpos[2] < 0.25):
            self.done = True
            self.log["e1"] = self.energy()
            for s in "LR":
                self.release(s)
            return
        if self.log["e0"] is None:
            self.log["e0"] = self.energy()
        on = any(self.attached(s) for s in "LR")
        loads = r.cop()[1]
        need = max(0.0, r.mass * 9.81 - float(sum(loads)))
        if phase == "stand: shift":
            self.fwd = min(SHIFT_FWD, self.fwd + SHIFT_FWD * dt / SHIFT_T)
        if phase == "stand: up":
            if self.t_up is None:
                self.t_up = d.time
            fade = max(0.0, 1.0 - (d.time - self.t_up) / UP_T)     # it eases off over the stand-up, then lets go
        else:
            fade = 1.0
        for w in self.arm:
            f = max(r.line_force(w["tA"]), r.line_force(w["tB"]))
            a = self.log["arm_rise"]
            a[w["name"]] = max(a.get(w["name"], 0.0), f)
            if on:
                a = self.log["arm_on"]
                a[w["name"]] = max(a.get(w["name"], 0.0), f)
        for s in self.hands:
            if not self.attached(s):
                self.attach(s)
                continue
            if self.drunk:
                kk = dt / helper.WANDER_TAU
                self.wander[s] += -self.wander[s] * kk + self.drunk * math.sqrt(2 * kk) * self.rng.standard_normal(3)
            f = self.force(s)
            own = self.k(s) * (self.anchor(s) - self.x[s]) * fade
            if own[2] > 0.0:                       # it lifts no more than the feet leave uncarried (D-073)
                own[2] = min(own[2], need / max(1, len(self.hands)) + LIFT_SLACK)
            n = np.linalg.norm(own)
            if n > self.cap:
                own *= self.cap / n
            v = (f + own + self.wander[s]) / B_LIFT
            sp = np.linalg.norm(v)
            if sp > helper.V_MAX:
                v *= helper.V_MAX / sp
            self.x[s] = self.x[s] + v * dt
            d.mocap_pos[self.mid[s]] = self.x[s]
            fn = float(np.linalg.norm(f))
            self.log["impulse"][s] += fn * dt
            self.log["work"] += float(np.dot(-f, v)) * dt
            self.log["peak"] = max(self.log["peak"], fn)
        if on:
            self.log["t_on"] += dt
            if d.time >= self._sample_t:            # the helper's side, for a robot taking its place (docs/47)
                self._sample_t = d.time + SAMPLE_DT
                self.samples.append(dict(t=float(d.time), phase=phase,
                                         x={s: self.x[s].tolist() for s in self.hands if self.attached(s)},
                                         f={s: self.force(s).tolist() for s in self.hands if self.attached(s)}))


def standing_grip_z(r, arm):
    """The grip's height with the robot in its standing pose (reset()) and the shoulders at `arm`."""
    m = r.model
    dd = mujoco.MjData(m)
    r2 = st.Rig.__new__(st.Rig)
    r2.__dict__.update(r.__dict__)
    r2.data = dd
    with contextlib.redirect_stdout(io.StringIO()):
        st.Rig.reset(r2)
    for s in "LR":
        dd.qpos[r.by_name["upperarm.%s_pitch" % s]["q"]] = arm
        dd.qpos[r.by_name["forearm.%s_pitch" % s]["q"]] = 0.0
    mujoco.mj_kinematics(m, dd)
    site = "elbow.L" if GRIP == "elbow" else "wrist.hand.L"
    return float(dd.site_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, site)][2])


def arms_forward_wrists(r, arm):
    """Where the wrists would be with the shoulders at `arm` and the elbows straight, the body as it is now."""
    m = r.model
    dd = mujoco.MjData(m)
    dd.qpos[:] = r.data.qpos
    for s in "LR":
        dd.qpos[r.by_name["upperarm.%s_pitch" % s]["q"]] = arm
        dd.qpos[r.by_name["forearm.%s_pitch" % s]["q"]] = 0.0
    mujoco.mj_kinematics(m, dd)
    site = "elbow.%s" if GRIP == "elbow" else "wrist.hand.%s"
    return {s: dd.site_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, site % s)].copy() for s in "LR"}


PHASES = {
    "A": dict(give=0.0, hands="LR", cap=150.0),
    "B": dict(give=0.5, hands="LR", cap=100.0),
    "C1": dict(give=0.5, hands="L", cap=100.0),
    "C2": dict(give=(0.3, 0.8), hands="LR", cap=100.0, drunk=4.0),
    "D": dict(hands=""),
}
TITLES = dict(A="firm, both hands", B="yielding, both hands", C1="one hand", C2="lopsided, drunken 4 N",
              D="withdrawn")


def run_one(cfg, video=None, gif=False, title=""):
    r = st.Rig()
    cfg = dict(cfg)
    with contextlib.redirect_stdout(io.StringIO()):
        kneel(r)
    cfg.pop("arms_hold", None)
    h = Lifter(r, **cfg)
    r.lifter = h
    rec = None
    if video:
        import sim_video
        out = os.path.join(os.path.dirname(HERE), "docs", "img", video + ".mp4")
        rec = sim_video.Recorder(r, out, gif=out[:-4] + ".gif" if gif else None, gif_scenes=("getup",))
        rec.scene("getup", "Up from a half-kneel: %s" % title, "the helper: orange hands; it gives with the pull on it")
        r.hook = h.chain(rec.on_step)
    else:
        r.hook = h.chain()
    res = getup(r)
    rows = r.report("getup %s" % title, top=6) if not TRACE else []
    if rec:
        print("wrote %s (%d frames)" % (out, rec.close()))
    lg = h.log
    cont = {w["name"]: w["p"]["f_cont"] for w in r.w}
    res.update(impulse=lg["impulse"]["L"] + lg["impulse"]["R"], peak=lg["peak"], work=lg["work"], t_on=lg["t_on"],
               energy=sum(v["e_j"] for v in r.stats.values()),
               slip=sum(math.degrees(v["slip"]) for v in r.stats.values()),
               arm_on={k: v / cont[k] for k, v in lg["arm_on"].items()},
               arm_rise={k: v / cont[k] for k, v in lg["arm_rise"].items()},
               worst=rows[0][1] if rows else "", worst_ratio=(rows[0][2] / rows[0][3]) if rows else 0.0)
    print("RESULT " + json.dumps(res))
    return res


def _parse(argv):
    over, rest = {}, []
    for a in argv:
        if "=" in a and not a.startswith("--"):
            k, v = a.split("=", 1)
            if k in ("give", "cap", "hands", "drunk", "seed"):
                over[k] = v if k == "hands" else eval(v)
            elif k.isupper() and k in globals():
                globals()[k] = eval(v, vars(math) | globals())
            elif k.isupper() and hasattr(helper, k):
                setattr(helper, k, eval(v))
            else:
                setattr(st, k, eval(v, vars(st)))
        else:
            rest.append(a)
    return over, rest


def _opt(args, k, default=None):
    if k in args:
        i = args.index(k)
        v = args[i + 1]
        del args[i:i + 2]
        return v
    return default


def one(argv):
    global TRACE
    over, args = _parse(argv)
    video = _opt(args, "--video")
    TRACE = "--trace" in args
    helper.TRACE = "--trace-helper" in args
    name = args[0]
    cfg = dict(PHASES.get(name, PHASES["A"]), **over)
    return run_one(cfg, video, "--gif" in args, TITLES.get(name, name))


def table(runs, extra):
    jobs = int(_opt(extra, "--jobs", 7))
    res = {}
    for i in range(0, len(runs), jobs):
        procs = [(lab, subprocess.Popen([sys.executable, os.path.abspath(__file__), "--one"] + a + extra,
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                        cwd=os.path.dirname(HERE))) for lab, a in runs[i:i + jobs]]
        for lab, p in procs:
            out = p.communicate()[0]
            m = re.search(r"^RESULT (.*)$", out, re.M)
            res[lab] = json.loads(m.group(1)) if m else dict(error=(out.strip().splitlines() or ["no output"])[-1])
    ref = next((v for k, v in res.items() if k.startswith("D") and "arm_rise" in v), None)
    print("%-28s %-6s %6s %8s %7s %7s %6s %7s %6s  %-24s %s" % ("phase", "rise", "prog", "imp N s", "peak N", "work J",
          "on s", "energy", "slip", "worst arm line held", "overload"))
    for lab, v in res.items():
        if "error" in v:
            print("%-28s error: %s" % (lab, v["error"]))
            continue
        over = [k for k, x in v["arm_on"].items() if x > 1.0 and x > helper.ARM_MARGIN * (ref["arm_rise"].get(k, 0.0)
                                                                                         if ref else 0.0)]
        worst = max(v["arm_on"].items(), key=lambda kv: kv[1]) if v["arm_on"] else ("-", 0.0)
        print("%-28s %-6s %6.2f %8.1f %7.0f %7.0f %6.1f %7.0f %6.0f  %-18s %5.2f  %s"
              % (lab, "stands" if v["stood"] else "FAILS", v["progress"], v["impulse"], v["peak"], v["work"],
                 v["t_on"], v["energy"], v["slip"], worst[0], worst[1], ("FAIL: " + ", ".join(over)) if over else "ok"))
    return res


# --- the search over the scripted get-up's parameters (as rise_train.py does for the chair) ---------------------
# name, default, low, high; angles in degrees (DEG), set on this module (or SHIFT/SWING entries)
GPARAMS = [
    ("ARM", -90.0, -160.0, -60.0),
    ("SPINE_LEAN", 10.0, 0.0, 35.0),
    ("RISE_T", 4.0, 2.0, 7.0),
    ("LIFT", 0.45, 0.2, 0.7),
    ("LIFT_FWD", 0.0, -0.2, 0.3),
    ("SHIFT.ankle", -20.0, -20.0, -5.0),
    ("SHIFT.knee", 25.0, 0.0, 45.0),
    ("SHIFT.back_hip", 20.0, 0.0, 15.0 + 25.0),
    ("SHIFT.spine", 25.0, 0.0, 40.0),
    ("SHIFT_FWD", 0.20, 0.0, 0.45),
    ("SWING.0", 60.0, 30.0, 100.0),
    ("SWING.1", -15.0, -50.0, 10.0),
    ("SWING_T", 0.8, 0.4, 1.5),
    ("PLANT_T", 0.6, 0.3, 1.5),
    ("UP_T", 1.5, 0.8, 4.0),
    ("LIFT_SLACK", 10.0, 0.0, 40.0),
    ("Z_TOP", 0.0, -0.35, 0.0),
]
GDEG = {"ARM", "SPINE_LEAN", "SHIFT.ankle", "SHIFT.knee", "SHIFT.back_hip", "SHIFT.spine", "SWING.0", "SWING.1"}


def gapply(p):
    g = globals()
    for k, v in p.items():
        v = D(v) if k in GDEG else v
        if k.startswith("SHIFT."):
            g["SHIFT"] = dict(g["SHIFT"], **{k[6:]: v})
        elif k.startswith("SWING."):
            sw = list(g["SWING"])
            sw[int(k[6:])] = v
            g["SWING"] = tuple(sw)
        else:
            g[k] = v
    g["GATE"] = 0.0


def gdefaults():
    return {k: d for k, d, lo, hi in GPARAMS}


def grun(job):
    """One get-up: (params, helper config[, jitter seed]). Returns run_one's result, quietly. With a seed, every
    parameter is jittered by JITTER (relative, or absolute on the angles and Z_TOP)."""
    p, cfg = job[:2]
    if len(job) > 2 and job[2] is not None:
        rng = np.random.default_rng(job[2])
        p = {k: v + JITTER * (abs(v) if abs(v) > 0.05 else 0.05) * rng.standard_normal() for k, v in p.items()}
    gapply(p)
    with contextlib.redirect_stdout(io.StringIO()):
        return run_one(cfg)


JITTER = 0.02
ARM_REF = 1.42    # the elbows' unassisted peak in the chair rise (docs/44): the overload law's reference here too


def gscore(res):
    """10 if it stands once let go; else up to 4 for the height reached, 2 for how far through the step it got and 3
    for how long it stood after the helper let go;
    minus 2 per unit of the worst arm line held over its rating beyond 1.42 (the chair's unassisted elbows)."""
    s = 10.0 if res["stood"] else (4.0 * min(1.0, res["z_peak"] / 0.9) + 2.0 * (res.get("stage", 0) / 4.0)
                                   + 3.0 * res.get("t_stand", 0.0) / HOLD_S)
    worst = max(res["arm_on"].values() or [0.0])
    s -= 2.0 * max(0.0, worst - ARM_REF)
    if worst > ARM_REF * helper.ARM_MARGIN:     # the overload law (D-069, as D-070 applies it): a hard cost
        s -= 8.0
    return s


FADE = [dict(give=0.25, hands="LR", cap=150.0), dict(give=0.5, hands="LR", cap=150.0),
        dict(give=0.5, hands="LR", cap=60.0), dict(give=0.75, hands="LR", cap=60.0),
        dict(give=0.75, hands="LR", cap=30.0), dict(give=1.0, hands="LR", cap=30.0),
        dict(give=1.0, hands="LR", cap=15.0), dict(hands="")]   # --fade: the helper's curriculum, to withdrawn


def gtrain(gens, pop, jobs, seed, out, cfg, init=None, reps=1, levels=None):
    import multiprocessing as mp
    os.makedirs(out, exist_ok=True)
    rng = np.random.default_rng(seed)
    lo = np.array([x[2] for x in GPARAMS])
    hi = np.array([x[3] for x in GPARAMS])
    to_u = lambda p: (np.array([p[k] for k, *_ in GPARAMS]) - lo) / (hi - lo)
    from_u = lambda u: {k: float(v) for (k, *_), v in zip(GPARAMS, lo + np.clip(u, 0, 1) * (hi - lo))}
    mean, std = to_u(dict(gdefaults(), **(init or {}))), np.full(len(GPARAMS), 0.15 if init is None else 0.08)
    best = None
    logf = open(os.path.join(out, "log.jsonl"), "a")
    n_el = max(2, pop // 4)
    level, per_level = 0, {}
    with mp.get_context("spawn").Pool(jobs, maxtasksperchild=4) as pool:
        for g in range(gens):
            if levels:
                cfg = levels[level]
            us = [mean] + [mean + std * rng.standard_normal(len(GPARAMS)) for _ in range(pop - 1)]
            ps = [from_u(u) for u in us]
            if reps > 1:
                sds = rng.integers(1 << 30, size=(pop, reps))
                rr = pool.map(grun, [(p, cfg, int(sds[i, k])) for i, p in enumerate(ps) for k in range(reps)],
                              chunksize=1)
                groups = [rr[i * reps:(i + 1) * reps] for i in range(pop)]
                sc = [float(np.mean([gscore(x) for x in gr])) for gr in groups]
                rs = [min(gr, key=gscore) for gr in groups]              # each candidate's worst run, for the log
                ok = [all(x["stood"] for x in gr) for gr in groups]
                stood_all = sum(ok)
            else:
                rs = pool.map(grun, [(p, cfg) for p in ps], chunksize=1)
                sc = [gscore(r) for r in rs]
                ok = [r["stood"] for r in rs]
                stood_all = sum(ok)
            order = np.argsort(sc)[::-1]
            i0 = int(order[0])
            if best is None or sc[i0] > best["score"]:
                best = dict(score=sc[i0], params=ps[i0], res=rs[i0], gen=g)
            worst = max(rs[i0]["arm_on"].values() or [0.0])
            print("gen %2d best %5.2f mean %5.2f stood %2d/%d  best: z %.2f stage %d arms %.2fx imp %.0f N s"
                  % (g, sc[i0], float(np.mean(sc)), stood_all, pop, rs[i0]["z_peak"],
                     rs[i0].get("stage", 0), worst, rs[i0]["impulse"]), flush=True)
            logf.write(json.dumps(dict(gen=g, best=sc[i0], mean=float(np.mean(sc)), params=ps[i0],
                                       stood=sum(r["stood"] for r in rs))) + "\n")
            logf.flush()
            el = [us[i] for i in order[:n_el]]
            frac = float(np.mean([ok[i] for i in order[:n_el]]))
            if levels:
                if ok[i0]:
                    per_level[str(level)] = dict(params=ps[i0], gen=g, cfg=cfg,
                                                 impulse=float(np.mean([x["impulse"] for x in (groups[i0] if reps > 1
                                                                                                else [rs[i0]])])))
                print("        level %d (%s): elite standing %.2f" % (level, json.dumps(cfg), frac), flush=True)
            mean = 0.3 * mean + 0.7 * np.mean(el, axis=0)
            std = np.clip(0.5 * std + 0.5 * np.std(el, axis=0), 0.03, 0.25)
            if levels and frac >= 0.75 and level < len(levels) - 1:
                level += 1
                mean = us[i0]
                std = np.maximum(std, 0.06)
                print("  -> level %d" % level, flush=True)
            json.dump(dict(best=best, cfg=cfg, last=dict(params=ps[i0], gen=g), mean=from_u(mean), level=level,
                           levels=per_level), open(os.path.join(out, "best.json"), "w"), indent=1)
    return best


def jitter_test(path, key, n, jobs, grip=None):
    """`key` of a search's best.json ("best", "last" or "mean") run n times, every parameter jittered by JITTER."""
    import multiprocessing as mp
    d = json.load(open(path))
    p = d[key]["params"] if "params" in d[key] else d[key]
    with mp.get_context("spawn").Pool(jobs, maxtasksperchild=4) as pool:
        rs = pool.map(grun, [(p, d["cfg"], 1000 + i) for i in range(n)], chunksize=1)
    worst = [max(r["arm_on"].values() or [0.0]) for r in rs]
    print("%s %s: %d of %d stand (jitter %.0f%%); impulse %.0f N s mean; worst arm line %.2fx max; stage %s"
          % (path, key, sum(r["stood"] for r in rs), n, 100 * JITTER, np.mean([r["impulse"] for r in rs]), max(worst),
             np.bincount([r["stage"] for r in rs], minlength=5).tolist()), flush=True)
    return rs


if __name__ == "__main__":
    argv = sys.argv[1:]
    if "--settle" in argv:
        settle()
    elif argv and argv[0] == "--one":
        one(argv[1:])
    elif "--jitter-test" in argv:
        i = argv.index("--jitter-test")
        path, key = argv[i + 1].rsplit(":", 1)
        jitter_test(path, key, int(_opt(argv, "--n", 24)), int(_opt(argv, "--jobs", 12)))
    elif "--train" in argv:
        argv.remove("--train")
        over, rest = _parse(argv)
        name = rest[0] if rest and not rest[0].startswith("--") else "A"
        cfg = dict(PHASES[name], **over)
        gtrain(int(_opt(rest, "--gens", 30)), int(_opt(rest, "--pop", 13)), int(_opt(rest, "--jobs", 13)),
               int(_opt(rest, "--seed", 0)), _opt(rest, "--out", "sim/out/getup_" + name), cfg,
               init=(dict(json.load(open(_opt(rest, "--init")))["best"]["params"], **{k: v for k, v in over.items()
                                                                                     if k.isupper()})
                     if "--init" in rest else None),
               reps=int(_opt(rest, "--reps", 1)), levels=FADE if "--fade" in rest else None)
    elif "--phases" in argv:
        argv.remove("--phases")
        table([("%s %s" % (k, TITLES[k]), [k]) for k in PHASES], argv)
    else:
        print(__doc__)
