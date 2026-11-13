"""
helper.py - the nurturing helper v0 (docs/44, D-070; hub task helper-instructor-chair): a scripted helper that holds
the robot's wrists while it rises from a chair, gives way in proportion to the force on it, and is faded out.

The helper is two mocap "hands" in the model (export_mjcf.HELPER), each tied to a robot wrist by a connect equality
when it takes hold. Each helper hand moves as

    v = v_shoulder + (F_robot + P u) / B       P = min(G * behind, CAP)

F_robot: the robot's pull on it; u: the robot's arm direction, shoulder to wrist (the helper pulls in tension along the
arm only); behind: how far the CoM is behind the soles' centre, less MARGIN (no pull once the CoM is over the feet).
So it follows the body, gives in proportion to the force on it, and pulls only as much as the balance needs. One knob,
`give` (0 firm, 1 yielding), sets G (per hand, half of G_FIRM..G_SOFT) and B (B_FIRM..B_SOFT): the Wing Chun give and
take. `drunk` adds a slow random force (an Ornstein-Uhlenbeck wander, N).

It arms when the rise's lean starts, takes a wrist once it is REACH of the arm's length ahead of the shoulder (the arm
out and straight), and lets go when the knees pass `arms_hold` degrees (the robot holds its arms forward till then:
sim_tendon.SIT_ARMS_HOLD) or the robot falls.

Phases (docs/43, docs/44, D-069, D-070):
    A   firm, both hands
    B   yielding, both hands
    C1  one hand (the left) only
    C2  lopsided and "drunken": the left firmer than the right, plus a wandering force
    D0  control: the arms held forward, nobody there
    D   withdrawn: no helper, the default chair controller

    python tools/helper.py --phases [--seat 0.35] [--jobs 7] [K=V ...]   # the per-phase table
    python tools/helper.py --fade [--seat 0.35]                          # the firm helper weakened step by step
    python tools/helper.py --drunk [--seat 0.35]                         # the drunken sailor: wander, lopsided, one hand
    python tools/helper.py --one B [--seat 0.35] [--trace] [give=0.5 cap=60 ...] [--video name [--gif]]

K=V: give=, cap=, hands=, drunk=, on_reach=, arms_hold= override the phase; upper-case names set this module's
constants (G_FIRM=, B_FIRM=, MARGIN=, REACH=), others sim_tendon's. Every --one prints a RESULT line of JSON.

The two laws (D-069), as applied here (D-070):
  * Overload: an arm line's peak while held must stay within its rating, or within ARM_MARGIN of the same line's peak
    over the rise window with no helper (Phase D of the same table): the elbows already run at 1.42x their rating in
    the unassisted rise.
  * Withdrawal: the impulse under 10 % of the firmest setting that obeys the overload law, AND the unassisted gates
    passing at their old rate. Nothing learns here, so Phase D is the unassisted baseline, not a trained withdrawal.
"""
import json
import math
import os
import re
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import export_mjcf  # noqa: E402

export_mjcf.HELPER = True
import mujoco  # noqa: E402
import sim_tendon as st  # noqa: E402

G_FIRM, G_SOFT = 800.0, 100.0       # N/m: the pull (both hands) per metre the CoM is behind the feet, firm and yielding
B_FIRM, B_SOFT = 150.0, 25.0        # N s/m: the damping against the hand moving relative to the robot's shoulder
ARMS_HOLD = 90.0                    # degrees of knee: the robot keeps its arms forward while held, until the knees pass it
MARGIN = 0.03                       # m: the CoM this far behind the soles' centre needs no help
V_MAX = 0.8                         # m/s: the helper hand's top speed
REACH = 0.96                        # a wrist this share of the arm's length ahead of its shoulder is reaching out: the
                                    # arm out and nearly straight. Taking the wrists before (as the lean starts, the
                                    # elbow still straightening) drove the elbow line to 2.3x its rating (docs/44)
ARM_CHAIN = ("upperarm.", "forearm.", "hand.")
ARM_MARGIN = 1.10                   # the overload law's allowance over the unassisted window's peak
TRACE = False                       # --trace: print the rise every 0.25 s
WANDER_TAU = 0.7                    # s: the drunken wander's time constant

PHASES = {
    "A": dict(give=0.0, hands="LR", on_reach=True, cap=100.0, drunk=0.0),
    "B": dict(give=0.5, hands="LR", on_reach=True, cap=60.0, drunk=0.0),
    "C1": dict(give=0.5, hands="L", on_reach=True, cap=60.0, drunk=0.0),
    "C2": dict(give=(0.3, 0.8), hands="LR", on_reach=True, cap=60.0, drunk=8.0),
    "D0": dict(hands=""),           # control: the arms held forward as for a helper, but nobody there
    "D": None,
}
TITLES = dict(A="always there, firm, both hands", B="offered on reach, yielding", C1="one hand",
              C2="lopsided and drunken", D0="arms forward, no helper", D="withdrawn")


class Helper:
    """The helper on one rig, built after reset(); update(rig) runs every step (Helper.chain)."""

    def __init__(self, rig, give=0.0, hands="LR", on_reach=False, cap=100.0, drunk=0.0, seed=0, arms_hold=ARMS_HOLD):
        m = rig.model
        self.r = rig
        self.hands = hands
        g = give if isinstance(give, (tuple, list)) else (give, give)
        self.give = dict(L=float(g[0]), R=float(g[1]))
        self.on_reach, self.cap, self.drunk = on_reach, float(cap), float(drunk)
        self.rng = np.random.default_rng(seed)
        self.arms_hold = arms_hold
        self.ref = None
        self.eq = {s: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "helper." + s) for s in "LR"}
        self.mid = {s: m.body_mocapid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "helper." + s)] for s in "LR"}
        self.grip = {s: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "wrist.hand." + s) for s in "LR"}
        self.shoulder = {s: m.body("upperarm." + s).id for s in "LR"}
        self.arm = [w for w in rig.w if w["name"].startswith(ARM_CHAIN)]
        d = rig.data   # the arm's length, shoulder to wrist, as it hangs straight in the standing pose
        mujoco.mj_kinematics(m, d)
        self.arm_len = float(np.linalg.norm(d.site_xpos[self.grip["L"]] - d.xpos[self.shoulder["L"]]))
        self.x = {s: None for s in "LR"}
        self.sh_prev = {s: None for s in "LR"}
        self.wander = {s: np.zeros(3) for s in "LR"}
        self.armed = self.done = False
        self.trace, self._next = TRACE, 0.0
        self.log = dict(impulse=dict(L=0.0, R=0.0), work=0.0, peak=0.0, t_on=0.0, t_attach=None, t_release=None,
                        arm_on={}, arm_rise={}, e0=None, e1=None)

    def gb(self, s):
        """This hand's share of the pull gain (N per m the CoM is behind) and its damping, from its give."""
        g = self.give[s]
        return 0.5 * G_FIRM * (G_SOFT / G_FIRM) ** g, B_FIRM * (B_SOFT / B_FIRM) ** g

    def behind(self):
        """How far the CoM is behind the soles' centre, less MARGIN (m; +: the robot would sit back down). `ref`, if
        set, replaces the soles' centre with its own measure (getup.py: the front sole)."""
        r = self.r
        if self.ref is not None:
            return float(self.ref(r)) - MARGIN
        so = r.soles()
        return float(r.data.subtree_com[r.pelvis][1] - 0.5 * (so[0][0][1] + so[1][0][1])) - MARGIN

    def force(self, s):
        """The robot's pull on helper hand s (N, world): minus the connect constraint's force on its second site."""
        d = self.r.data
        e = self.eq[s]
        i = np.flatnonzero((d.efc_id == e) & (d.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY))
        return -np.array(d.efc_force[i[0]:i[0] + 3]) if len(i) else np.zeros(3)

    def attached(self, s):
        return bool(self.r.data.eq_active[self.eq[s]])

    def attach(self, s):
        d = self.r.data
        self.x[s] = d.site_xpos[self.grip[s]].copy()
        d.mocap_pos[self.mid[s]] = self.x[s]
        d.eq_active[self.eq[s]] = 1
        if self.log["t_attach"] is None:
            self.log["t_attach"] = d.time

    def release(self, s):
        d = self.r.data
        d.eq_active[self.eq[s]] = 0
        d.mocap_pos[self.mid[s]] = (3.0 if s == "L" else 4.0, 5.0, -1.0)

    def reaching(self, s):
        d = self.r.data
        return d.xpos[self.shoulder[s]][1] - d.site_xpos[self.grip[s]][1] > REACH * self.arm_len   # forward is -y

    def energy(self):
        return sum(v["e_j"] for v in self.r.stats.values())

    def update(self, r):
        d = r.data
        dt = r.model.opt.timestep
        phase = r.phase or ""
        if not self.armed and phase == "stand: lean":
            self.armed = True
            self.log["e0"] = self.energy()
        if not self.armed:
            return
        on = any(self.attached(s) for s in "LR")
        km = 0.5 * (d.qpos[r.by_name["shank.L_pitch"]["q"]] + d.qpos[r.by_name["shank.R_pitch"]["q"]])
        past = self.arms_hold and phase == "stand: rise" and km < math.radians(self.arms_hold)   # up: it lets go
        if not self.done and (phase == "stand: up" or past or not r.standing() and d.qpos[2] < 0.35):
            self.done = True
            self.log["e1"] = self.energy()
            self.log["t_release"] = d.time
            for s in "LR":
                self.release(s)
            return
        if self.done:
            return
        # the arm lines over the rise window, with or without the helper (the overload law's baseline)
        for w in self.arm:
            f = max(r.line_force(w["tA"]), r.line_force(w["tB"]))
            a = self.log["arm_rise"]
            a[w["name"]] = max(a.get(w["name"], 0.0), f)
            if on:
                a = self.log["arm_on"]
                a[w["name"]] = max(a.get(w["name"], 0.0), f)
        for s in self.hands:
            if not self.attached(s):
                if not self.on_reach or self.reaching(s):
                    self.attach(s)
                continue
            if self.drunk:
                k = dt / WANDER_TAU
                self.wander[s] += -self.wander[s] * k + self.drunk * math.sqrt(2 * k) * self.rng.standard_normal(3)
            f = self.force(s)
            gg, bb = self.gb(s)
            sh = d.xpos[self.shoulder[s]].copy()
            v_sh = np.zeros(3) if self.sh_prev[s] is None else (sh - self.sh_prev[s]) / dt
            self.sh_prev[s] = sh
            u = self.x[s] - sh
            u /= max(np.linalg.norm(u), 1e-6)
            fs = min(gg * max(self.behind(), 0.0), self.cap) * u + self.wander[s]
            v = v_sh + (f + fs) / bb
            sp = np.linalg.norm(v)
            if sp > V_MAX:
                v *= V_MAX / sp
            self.x[s] = self.x[s] + v * dt
            d.mocap_pos[self.mid[s]] = self.x[s]
            fn = float(np.linalg.norm(f))
            self.log["impulse"][s] += fn * dt
            self.log["work"] += float(np.dot(-f, v)) * dt     # the helper's work on the robot (+: it lifted or pulled it)
            self.log["peak"] = max(self.log["peak"], fn)
        if on:
            self.log["t_on"] += dt
        if self.trace and d.time >= self._next:
            self._next = d.time + 0.25
            km = 0.5 * (d.qpos[r.by_name["shank.L_pitch"]["q"]] + d.qpos[r.by_name["shank.R_pitch"]["q"]])
            so = r.soles()
            com = r.data.subtree_com[r.pelvis]
            z, tilt = r.pelvis_state()
            fl = [self.force(s) if self.attached(s) else np.zeros(3) for s in "LR"]
            print("  t %5.2f %-12s knee %5.1f  pelvis %.3f tilt %5.1f  CoM-sole y %+.3f  seat %4.0f N  "
                  "pull L (%+4.0f %+4.0f) R (%+4.0f %+4.0f) N  behind %+.3f m"
                  % (d.time, phase[:12], math.degrees(km), z, tilt, com[1] - 0.5 * (so[0][0][1] + so[1][0][1]),
                     r.chair_load()[0], -fl[0][1], -fl[0][2], -fl[1][1], -fl[1][2],
                     self.behind()))

    def chain(self, hook=None):
        """A step hook running the helper, then `hook` (a recorder's)."""
        return (lambda r: (self.update(r), hook(r))) if hook else self.update


def run_one(cfg, seat, video=None, gif=False, title=""):
    r = st.Rig()
    r.reset()
    cfg = dict(cfg) if cfg else dict(hands="", arms_hold=False)   # Phase D: the same logging, nobody there
    cfg.setdefault("arms_hold", ARMS_HOLD)
    st.SIT_ARMS_HOLD = math.radians(cfg["arms_hold"]) if cfg["arms_hold"] else 0.0
    st.SIT_LOW = None          # docs/44's tables: the helper on the default rise, not the trained low-seat set (D-072)
    h = Helper(r, **cfg)
    rec = None
    if video:
        import sim_video
        out = os.path.join(os.path.dirname(HERE), "docs", "img", video + ".mp4")
        rec = sim_video.Recorder(r, out, gif=out[:-4] + ".gif" if gif else None, gif_scenes=("chair",))
        rec.scene("chair", "Rise from a %.2f m chair: %s" % (seat, title),
                  "the helper: orange hands; it gives in proportion to the pull on it")
        r.hook = h.chain(rec.on_step)
    else:
        r.hook = h.chain()
    st.sit_stand(r, seat, False)
    rows = r.report("helper %s on the %.2f m chair" % (title, seat), top=6)
    if rec:
        n = rec.close()
        print("wrote %s (%d frames)" % (out, n))
    lg = h.log
    rise_e = (lg["e1"] if lg["e1"] is not None else h.energy()) - (lg["e0"] or 0.0)
    cont = {w["name"]: w["p"]["f_cont"] for w in r.w}
    slip = sum(math.degrees(v["slip"]) for v in r.stats.values())
    res = dict(stood=bool(r.sit_result["stood"]), sat=bool(r.sit_result["sat"]), reach=r.sit_result["reach"],
               impulse=lg["impulse"]["L"] + lg["impulse"]["R"], imp_L=lg["impulse"]["L"], imp_R=lg["impulse"]["R"],
               peak=lg["peak"], work=lg["work"], t_on=lg["t_on"], rise_e=rise_e,
               energy=sum(v["e_j"] for v in r.stats.values()), slip=slip,
               arm_on={k: v / cont[k] for k, v in lg["arm_on"].items()},
               arm_rise={k: v / cont[k] for k, v in lg["arm_rise"].items()},
               worst=rows[0][1] if rows else "", worst_ratio=(rows[0][2] / rows[0][3]) if rows else 0.0)
    print("RESULT " + json.dumps(res))
    return res


def _parse(argv):
    """K=V: the phase's own keys (give, cap, hands, drunk, on_reach, arms_hold), this module's constants (G_FIRM,
    B_FIRM, MARGIN, ...), or sim_tendon's."""
    over, rest = {}, []
    for a in argv:
        if "=" in a and not a.startswith("--"):
            k, v = a.split("=", 1)
            if k in ("give", "cap", "hands", "drunk", "on_reach", "arms_hold", "seed"):
                over[k] = v if k == "hands" else eval(v)
            elif k.isupper() and k in globals():
                globals()[k] = eval(v)
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
    over, args = _parse(argv)
    seat = float(_opt(args, "--seat", 0.35))
    video = _opt(args, "--video")
    gif = "--gif" in args
    global TRACE
    TRACE = "--trace" in args
    name = args[0]
    cfg = PHASES.get(name)
    if over:
        cfg = dict(cfg or dict(PHASES["A"]), **over)
    return run_one(cfg, seat, video, gif, TITLES.get(name, name))


def _table(runs, seat, extra):
    """Run each (label, argv) in its own process, `jobs` at a time; print the per-phase table."""
    jobs = int(_opt(extra, "--jobs", 6))
    res = {}
    for i in range(0, len(runs), jobs):
        procs = [(lab, subprocess.Popen([sys.executable, os.path.abspath(__file__), "--one"] + a + ["--seat", str(seat)]
                                        + extra, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                        cwd=os.path.dirname(HERE))) for lab, a in runs[i:i + jobs]]
        for lab, p in procs:
            out = p.communicate()[0]
            m = re.search(r"^RESULT (.*)$", out, re.M)
            res[lab] = json.loads(m.group(1)) if m else dict(error=(out.strip().splitlines() or ["no output"])[-1])
    base = next((v for k, v in res.items() if k.startswith("A") and "impulse" in v), None)
    unassisted = next((v for k, v in res.items() if k.startswith("D") and "arm_rise" in v), None)
    print("\nseat %.2f m" % seat)
    print("%-26s %-6s %7s %7s %7s %6s %7s %7s %6s  %-26s %s" % ("phase", "rise", "imp Ns", "% of A", "peak N",
          "work J", "on s", "rise J", "slip", "worst arm line held / rating", "overload"))
    for lab, v in res.items():
        if "error" in v:
            print("%-26s error: %s" % (lab, v["error"]))
            continue
        over = []
        for k, ratio in v["arm_on"].items():
            ref = unassisted["arm_rise"].get(k, 0.0) if unassisted else 0.0
            if ratio > 1.0 and ratio > ARM_MARGIN * ref:
                over.append(k)
        worst = max(v["arm_on"].items(), key=lambda kv: kv[1]) if v["arm_on"] else ("-", 0.0)
        pct = "%6.0f%%" % (100 * v["impulse"] / base["impulse"]) if base and base["impulse"] > 0 else "      -"
        print("%-26s %-6s %7.1f %7s %7.0f %6.0f %7.1f %7.0f %6.1f  %-18s %5.2f   %s"
              % (lab, "stands" if v["stood"] else "FAILS", v["impulse"], pct, v["peak"], v["work"], v["t_on"],
                 v["rise_e"], v["slip"], worst[0], worst[1], ("FAIL: " + ", ".join(over)) if over else "ok"))
    if unassisted:
        w = sorted(unassisted["arm_rise"].items(), key=lambda kv: -kv[1])[:3]
        print("unassisted rise window, arm lines / rating: " + ", ".join("%s %.2f" % kv for kv in w))
    return res


def main():
    argv = sys.argv[1:]
    if argv and argv[0] == "--one":
        return one(argv[1:])
    seat = float(_opt(argv, "--seat", 0.35))
    if "--phases" in argv:
        argv.remove("--phases")
        return _table([("%s %s" % (k, TITLES[k]), [k]) for k in PHASES], seat, argv)
    if "--fade" in argv:
        argv.remove("--fade")
        runs = [("A give %.2f cap %3.0f" % (g, c), ["A", "give=%r" % g, "cap=%r" % c])
                for g, c in ((0.0, 200), (0.25, 150), (0.5, 100), (0.5, 60), (0.75, 40), (0.75, 25), (0.9, 15))]
        return _table(runs + [("D withdrawn", ["D"])], seat, argv)
    if "--drunk" in argv:
        argv.remove("--drunk")
        runs = [("B25 drunk 0", ["B", "give=0.25", "cap=150"])]
        runs += [("B25 drunk %g s%d" % (dr, sd), ["B", "give=0.25", "cap=150", "drunk=%r" % dr, "seed=%d" % sd])
                 for dr in (4.0, 8.0, 16.0) for sd in (0, 1)]
        runs += [("B50 drunk %g" % dr, ["B", "drunk=%r" % dr]) for dr in (4.0, 8.0)]
        runs += [("lopsided 0.1/0.5", ["B", "give=(0.1, 0.5)", "cap=150"]),
                 ("lopsided 0.1/0.5 drunk 8", ["B", "give=(0.1, 0.5)", "cap=150", "drunk=8.0"]),
                 ("one hand give 0.25", ["C1", "give=0.25", "cap=150"]),
                 ("one hand give 0", ["C1", "give=0.0", "cap=200"])]
        return _table(runs + [("D withdrawn", ["D"])], seat, argv)
    print(__doc__)


if __name__ == "__main__":
    main()
