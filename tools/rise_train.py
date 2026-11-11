"""
rise_train.py - training round 1's first slice (docs/45, hub task training-round-1): the robot's own chair rise is
trained on the full tendon model with the helper (tools/helper.py, docs/44) as scaffolding, and the helper is faded
as the rise improves, toward withdrawal.

What learns: the robot's side of the rise, the hand-written chair controller's parameters (the knees' lead and rate,
the CoM gate, the hips' flexion and rate, the spine's over-command and bend, the arms' forward reach and when they
lower, the trunk's lean time, when the balance loop takes the ankles). A cross-entropy search (CEM) over them, each
candidate run in its own process on the full model; no stand-in, no network.

The helper is the curriculum (docs/43 phases B-D):
    level 0  give 0.25, cap 150 N   (D-070's baseline: the firmest helper that obeys the overload law)
    level 1  give 0.5,  cap 60 N    (Phase B)
    level 2  give 0.75, cap 40 N
    level 3  give 0.9,  cap 15 N
    levels 4-6  give 0.9, cap 6, 3, 1 N
    level 7  withdrawn: the arms held forward, nobody there
With --gate (the default; --no-gate turns it off) each candidate also rises from the 0.40 m seat with no helper, the
grid's lowest seat that stood before: failing it costs 6 and keeps the candidate out of the passing elite (the first
run, without it, traded that seat away; docs/45).
Every candidate runs twice per generation: the helper as set, and the same helper "drunk" (a wandering force of
DRUNK N (4: at 8 the scripted rise already fails, docs/45), a fresh seed each time: the drunken sailor), so the rise cannot lean on a perfectly steady hand.
The level goes up once ADVANCE of the elite stand in every run of a generation.

Score per run: 10 if it stood, else up to 5 for how far the knees straightened; minus 3 x the helper's impulse over
the 107 N s baseline (D-070); minus 8 if an arm line broke the overload law (D-069 as applied, D-070); minus a small
energy cost. The gates afterwards: the chair grid without a helper (0.35 to 0.50 m, with and without armrests) with
the trained parameters against the defaults; the rest of the gauntlet does not use these parameters.

    python tools/rise_train.py [--gens 40] [--pop 13] [--jobs 13] [--seed 0] [--out sim/out/rise] [--no-gate]
                               [--start-level 7]   # the control: no curriculum, straight at withdrawn
    python tools/rise_train.py --grid sim/out/rise/best.json      # the chair grid with the trained parameters
    python tools/rise_train.py --show sim/out/rise_gate/best.json --level 7 [--video rise35]   # record it, no helper
    python tools/rise_train.py --probe sim/out/rise/best.json [--level 3]   # that level's best against weaker helpers
"""
import contextlib
import io
import json
import math
import multiprocessing as mp
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# name, default, low, high; the angles in degrees
PARAMS = [
    ("SIT_LEAD", 12.0, 4.0, 25.0),
    ("SIT_KNEE_RATE", 25.0, 10.0, 50.0),
    ("SIT_GATE", 0.02, 0.0, 0.06),
    ("SIT_HIP_K", 3.0, 0.0, 8.0),
    ("SIT_HIP_RATE", 22.5, 10.0, 45.0),
    ("SIT_SPINE_OVER", 15.0, 0.0, 30.0),
    ("SIT_SPINE", 20.0, 10.0, 35.0),
    ("SIT_ARM", -90.0, -120.0, -60.0),
    ("LEAN_T", 1.0, 0.5, 2.0),        # scales both SIT_LEAN_T times
    ("SIT_BAL_ON", 30.0, 15.0, 50.0),
    ("ARMS_HOLD", 90.0, 60.0, 110.0),
]
DEG = {"SIT_LEAD", "SIT_KNEE_RATE", "SIT_HIP_RATE", "SIT_SPINE_OVER", "SIT_SPINE", "SIT_ARM", "SIT_BAL_ON"}
LEVELS = [dict(give=0.25, cap=150.0), dict(give=0.5, cap=60.0), dict(give=0.75, cap=40.0), dict(give=0.9, cap=15.0),
          dict(give=0.9, cap=6.0), dict(give=0.9, cap=3.0), dict(give=0.9, cap=1.0), None]
GATE = (0.40, False)   # the old gate scored in every generation: this seat, no helper, must still stand (D-069 law 2)
DRUNK = 4.0
BASE_IMPULSE = 107.0                 # N s: D-070's baseline
ADVANCE = 0.75
SEAT = 0.35


def defaults():
    return {k: d for k, d, lo, hi in PARAMS}


def _apply(p):
    import sim_tendon as st
    for k, v in p.items():
        if k == "LEAN_T":
            st.SIT_LEAN_T = (1.0 * v, 1.5 * v, 0.065)
        elif k == "ARMS_HOLD":
            continue
        else:
            setattr(st, k, math.radians(v) if k in DEG else v)


def run(job):
    """One rise: job = (params, level config or None, drunk N, seed, seat, armrests, helper on). Returns a dict."""
    p, lvl, drunk, seed, seat, arms, use_helper = job
    import helper
    import sim_tendon as st
    _apply(p)
    t0 = time.time()
    with contextlib.redirect_stdout(io.StringIO()):
        r = st.Rig()
        r.reset()
        hold = p.get("ARMS_HOLD", 90.0) if use_helper else 0.0
        st.SIT_ARMS_HOLD = math.radians(hold) if hold else 0.0
        cfg = dict(hands="", arms_hold=hold or False)
        if use_helper and lvl is not None:
            cfg = dict(hands="LR", on_reach=True, give=lvl["give"], cap=lvl["cap"], drunk=drunk, seed=seed,
                       arms_hold=hold)
        h = helper.Helper(r, **cfg)
        prog = dict(km0=None, km=None, z=0.0)

        def track(_r):
            if (_r.phase or "") == "stand: rise":
                d = _r.data
                km = 0.5 * (d.qpos[_r.by_name["shank.L_pitch"]["q"]] + d.qpos[_r.by_name["shank.R_pitch"]["q"]])
                prog["km0"] = km if prog["km0"] is None else prog["km0"]
                prog["km"] = km if prog["km"] is None else min(prog["km"], km)
        r.hook = h.chain(track)
        st.sit_stand(r, seat, arms)
    lg = h.log
    cont = {w["name"]: w["p"]["f_cont"] for w in r.w}
    rise_e = (lg["e1"] if lg["e1"] is not None else h.energy()) - (lg["e0"] or 0.0)
    km0, km = prog["km0"] or 1.0, prog["km"] if prog["km"] is not None else 1.0
    return dict(stood=bool(r.sit_result["stood"]), sat=bool(r.sit_result["sat"]),
                impulse=lg["impulse"]["L"] + lg["impulse"]["R"], t_on=lg["t_on"], rise_e=rise_e,
                progress=float(np.clip(1.0 - km / max(km0, 1e-6), 0.0, 1.0)),
                arm_on={k: v / cont[k] for k, v in lg["arm_on"].items()},
                arm_rise={k: v / cont[k] for k, v in lg["arm_rise"].items()},
                slip=sum(math.degrees(v["slip"]) for v in r.stats.values()), wall=time.time() - t0)


def overloaded(res, ref, margin=1.10):
    return [k for k, v in res["arm_on"].items() if v > 1.0 and v > margin * ref.get(k, 0.0)]


def score(res, ref):
    s = 10.0 if res["stood"] else 5.0 * res["progress"]
    s -= 3.0 * res["impulse"] / BASE_IMPULSE
    if overloaded(res, ref):
        s -= 8.0
    return s - 0.002 * res["rise_e"]


def _to_unit(p):
    return np.array([(p[k] - lo) / (hi - lo) for k, d, lo, hi in PARAMS])


def _from_unit(u):
    u = np.clip(u, 0.0, 1.0)
    return {k: float(lo + x * (hi - lo)) for (k, d, lo, hi), x in zip(PARAMS, u)}


def train(gens, pop, jobs, seed, out, gate=True, start=0):
    os.makedirs(out, exist_ok=True)
    rng = np.random.default_rng(seed)
    logf = open(os.path.join(out, "log.jsonl"), "a")
    with mp.get_context("spawn").Pool(jobs, maxtasksperchild=4) as pool:
        ref = pool.apply(run, ((defaults(), None, 0.0, 0, SEAT, False, False),))["arm_rise"]   # unassisted, defaults
        print("unassisted reference: forearm %.2fx" % max(v for k, v in ref.items() if k.startswith("forearm")),
              flush=True)
        mean, std = _to_unit(defaults()), np.full(len(PARAMS), 0.15)
        level, best = start, {}
        n_elite = max(2, pop // 4)
        for g in range(gens):
            us = [mean] + [mean + std * rng.standard_normal(len(PARAMS)) for _ in range(pop - 1)]
            ps = [_from_unit(u) for u in us]
            lvl = LEVELS[level]
            jobs_ = []
            for i, p in enumerate(ps):
                sd = int(rng.integers(1 << 30))
                jobs_ += [(p, lvl, 0.0, sd, SEAT, False, True), (p, lvl, DRUNK if lvl else 0.0, sd, SEAT, False, True)]
                if gate:
                    jobs_.append((p, None, 0.0, 0, GATE[0], GATE[1], False))
            per = 3 if gate else 2
            t0 = time.time()
            rs = pool.map(run, jobs_, chunksize=1)
            sc = []
            for i in range(pop):
                a, b = rs[per * i], rs[per * i + 1]
                sc.append(0.5 * (score(a, ref) + score(b, ref)) - (6.0 if gate and not rs[per * i + 2]["stood"] else 0.0))
            order = np.argsort(sc)[::-1]
            elite = order[:n_elite]
            ok = [rs[per * i]["stood"] and rs[per * i + 1]["stood"] and not overloaded(rs[per * i], ref)
                  and not overloaded(rs[per * i + 1], ref) and (not gate or rs[per * i + 2]["stood"]) for i in range(pop)]
            frac = float(np.mean([ok[i] for i in elite]))
            i0 = int(order[0])
            if level not in best or sc[i0] > best[level]["score"]:
                best[level] = dict(score=sc[i0], params=ps[i0], runs=rs[per * i0:per * i0 + per], gen=g)
            rec = dict(gen=g, level=level, best=sc[i0], mean_score=float(np.mean(sc)), elite_ok=frac,
                       stood=int(sum(r["stood"] for r in rs)), runs=len(rs),
                       imp_best=0.5 * (rs[per * i0]["impulse"] + rs[per * i0 + 1]["impulse"]), params=ps[i0],
                       std=float(std.mean()), wall=time.time() - t0)
            logf.write(json.dumps(rec) + "\n")
            logf.flush()
            print("gen %2d level %d  best %6.2f  mean %6.2f  stood %2d/%d  elite ok %.2f  imp %.0f N s  std %.3f  %.0f s"
                  % (g, level, sc[i0], rec["mean_score"], rec["stood"], len(rs), frac, rec["imp_best"], rec["std"],
                     rec["wall"]), flush=True)
            new_mean = np.mean([us[i] for i in elite], axis=0)
            new_std = np.std([us[i] for i in elite], axis=0)
            mean = 0.3 * mean + 0.7 * new_mean
            std = np.clip(0.5 * std + 0.5 * new_std, 0.03, 0.25)
            if frac >= ADVANCE and level < len(LEVELS) - 1:
                level += 1
                mean = us[i0]                        # start the next level from the best that passed this one
                std = np.maximum(std, 0.08)
                print("  -> level %d" % level, flush=True)
            json.dump(dict(level=level, best={str(k): v for k, v in best.items()}, ref=ref),
                      open(os.path.join(out, "best.json"), "w"), indent=1)
    return best


def grid(path, jobs):
    """The chair grid with no helper, the trained parameters (each level's best) against the defaults."""
    data = json.load(open(path))
    sets = [("defaults", defaults())] + [("level %s" % k, v["params"]) for k, v in sorted(data["best"].items())]
    seats = [(s, a) for s in (0.35, 0.40, 0.45, 0.50) for a in (False, True)]
    with mp.get_context("spawn").Pool(jobs, maxtasksperchild=4) as pool:
        for name, p in sets:
            rs = pool.map(run, [(p, None, 0.0, 0, s, a, False) for s, a in seats], chunksize=1)
            print("%-10s " % name + "  ".join("%.2f%s %s" % (s, "a" if a else " ", "stands" if r["stood"] else "FAILS")
                                               for (s, a), r in zip(seats, rs)), flush=True)
            # with the trained arms-forward hold as well (the robot's side of the helper, nobody there)
            if name != "defaults":
                q = dict(p)
                rs = pool.map(run, [(q, None, 0.0, 0, s, a, True) for s, a in seats], chunksize=1)
                print("%-10s " % (name + " hold") + "  ".join("%.2f%s %s" % (s, "a" if a else " ", "stands" if r["stood"]
                                                                              else "FAILS")
                                                               for (s, a), r in zip(seats, rs)), flush=True)


def probe(path, level, jobs):
    """The trained parameters of `level` against a helper weakened past the curriculum: give 0.9, cap falling to 0."""
    data = json.load(open(path))
    p = data["best"][str(level)]["params"]
    ref = data["ref"]
    caps = (15.0, 10.0, 6.0, 3.0, 1.0)
    jobs_ = [(p, dict(give=0.9, cap=c), dr, sd, SEAT, False, True) for c in caps for dr, sd in ((0.0, 0), (DRUNK, 1))]
    jobs_ += [(defaults(), dict(give=0.9, cap=c), 0.0, 0, SEAT, False, True) for c in (15.0, 6.0)]
    with mp.get_context("spawn").Pool(jobs, maxtasksperchild=4) as pool:
        rs = pool.map(run, jobs_, chunksize=1)
    for (q, lvl, dr, sd, *_), r in zip(jobs_, rs):
        print("%-8s cap %4.0f N drunk %g: %-6s impulse %5.1f N s  held %4.1f s  worst arm line %.2fx  %s"
              % ("defaults" if q is not p else "level %d" % level, lvl["cap"], dr, "stands" if r["stood"] else "FAILS",
                 r["impulse"], r["t_on"], max(r["arm_on"].values() or [0.0]),
                 ("overload: " + ", ".join(overloaded(r, ref))) if overloaded(r, ref) else "ok"), flush=True)


def show(path, level, seat, video):
    """Record one rise with `level`'s trained parameters and no helper to docs/img/<video>.mp4 and .gif."""
    import helper
    import sim_tendon as st
    import sim_video
    p = json.load(open(path))["best"][str(level)]["params"]
    _apply(p)
    st.SIT_ARMS_HOLD = 0.0
    r = st.Rig()
    r.reset()
    h = helper.Helper(r, hands="", arms_hold=False)
    out = os.path.join(os.path.dirname(HERE), "docs", "img", video + ".mp4")
    rec = sim_video.Recorder(r, out, gif=out[:-4] + ".gif", gif_scenes=("chair",))
    rec.scene("chair", "Rise from a %.2f m chair, no helper: the trained rise" % seat,
              "its parameters trained on the full model (docs/45)")
    r.hook = h.chain(rec.on_step)
    st.sit_stand(r, seat, False)
    print("stood" if r.sit_result["stood"] else "did not stand", "; wrote %s (%d frames)" % (out, rec.close()))


def _opt(argv, k, default):
    if k in argv:
        i = argv.index(k)
        v = argv[i + 1]
        del argv[i:i + 2]
        return v
    return default


if __name__ == "__main__":
    argv = sys.argv[1:]
    jobs = int(_opt(argv, "--jobs", 13))
    g = _opt(argv, "--grid", None)
    pr = _opt(argv, "--probe", None)
    sh = _opt(argv, "--show", None)
    if sh:
        show(sh, int(_opt(argv, "--level", 7)), float(_opt(argv, "--seat", SEAT)), _opt(argv, "--video", "rise35"))
    elif pr:
        probe(pr, int(_opt(argv, "--level", 3)), jobs)
    elif g:
        grid(g, jobs)
    else:
        train(int(_opt(argv, "--gens", 40)), int(_opt(argv, "--pop", 13)), jobs, int(_opt(argv, "--seed", 0)),
              _opt(argv, "--out", "sim/out/rise"), gate="--no-gate" not in argv, start=int(_opt(argv, "--start-level", 0)))
