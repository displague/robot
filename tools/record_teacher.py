#!/usr/bin/env python3
"""
record_teacher.py - the algorithmic controller as a teacher: demonstrations for a learned policy (docs/17).

Runs the gauntlet scenarios of sim_tendon.py on the full tendon model (the controller of docs/14-16: capture-
point balance, weight shift, one-foot hold, the static step) plus pushes from random directions, and records
at 50 Hz what a policy on the fast stand-in (tools/export_standin.py) would observe, together with the joint
targets the controller applied (Rig.applied). The observation layout and the action scaling are the stand-in
environment's (ai-embedded-dynamic-diversity, rl/standin_env.py):

    obs = [joint angle - standing pose (32), 0.1 x joint rate (32), gravity in the pelvis frame (3),
           0.25 x pelvis angular rate (3), foot loads / weight (2)]                                  72
    offset = applied target - standing pose, radians                                              32

The environment's last 32 observation channels (the previous action) and its action are the offset divided by
that environment's action span, which depends on the task, so the loader builds them (rl/bc.py there).

Scenarios that end in a fall (the 40 N push) are left out: the teacher is only worth copying where it works.

Wind (docs/25): --winds N adds N episodes of 12 s standing in a random wind (mean 0 to 9 m/s, gusts 0 to 2 m/s, any
direction); the observation has no wind channel (the robot carries no anemometer), so the policy must read it
from its own motion as the controller's disturbance observer does.

Chairs and chains (docs/43, hub task teacher-demos-regen-v060): --set chair records the chair scenes (sit on a
0.40/0.45/0.50 m seat, with and without armrests, and stand again; the 0.35 m seat falls and is left out) into
teacher_chair.npz; --set chain --chains N records N chained episodes of tools/chain.py (seeds 0..N-1: a bag picked,
walked, set down for a sit, put down) into teacher_chain.npz, each in a fresh process (each bag is its own model).
They are separate files because their offsets (the knees at 100 degrees) would widen a standing task's action spans.

Every file also holds `felt` (steps x 2): the weight the hands carry over the robot's weight, as the grip's force
(Rig.held_load, ground truth) and as the balance loop's low-passed estimate (D-063, Rig._felt_f). Zero with nothing
held. It is not in `obs`, whose 72-channel layout stays the stand-in's; a policy that uses it adds it as a channel.

Run:  python tools/record_teacher.py [--pushes 24] [--winds 0] [--jobs 8]   -> sim/out/teacher/teacher.npz
      python tools/record_teacher.py --set chair [--jobs 8]               -> sim/out/teacher/teacher_chair.npz
      python tools/record_teacher.py --set chain --chains 24 [--jobs 12]  -> sim/out/teacher/teacher_chain.npz
"""
import math
import multiprocessing as mp
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_tendon as st   # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RATE_HZ = 50
SKIP = set()               # scenarios to leave out; falls are left out anyway (the 40 N push stands since D-038)


def _push_scenario(seed):
    rng = np.random.default_rng(seed)

    def fn(r):
        r.run(1.0 + rng.uniform(0.0, 1.0))
        for _ in range(int(rng.integers(1, 4))):
            ang = rng.uniform(-math.pi, math.pi)
            f = rng.uniform(10.0, 30.0)
            r.push("spine", (f * math.cos(ang), f * math.sin(ang), 0.0), 0.2)
            r.run(rng.uniform(1.5, 2.5))
    return fn


def _wind_spec(seed):
    rng = np.random.default_rng(seed)
    return dict(mean=float(rng.uniform(0.0, 9.0)), gust=float(rng.uniform(0.0, 2.0)),
                dir=float(rng.uniform(0.0, 360.0)), seed=int(seed))


def record(job):
    key, fn = job[:2]
    item = job[2] if len(job) > 2 else None
    if item:                                         # a chain episode: its bag in the model (tools/chain.py)
        import chain
        import export_mjcf
        export_mjcf.ITEMS = chain.items_for(item)
        export_mjcf.STOOLS = 4
        st.STOOL_HALF = export_mjcf.STOOL_HALF
    r = st.Rig()
    names = [w["name"] for w in r.w]
    lo = np.array([w["lo"] for w in r.w])
    hi = np.array([w["hi"] for w in r.w])
    default = np.array([r.target[n] for n in names])        # the standing pose (ankles at STAND_LEAN)
    m, d = r.model, r.data
    every = max(1, int(round(1.0 / (RATE_HZ * m.opt.timestep))))
    weight = r.mass * 9.81
    rows_o, rows_a, rows_f = [], [], []
    count = [0]

    def hook(rig):
        count[0] += 1
        if count[0] % every:
            return
        q = np.array([d.qpos[w["q"]] for w in rig.w])
        qd = np.array([d.qvel[w["dq"]] for w in rig.w])
        R = d.xmat[rig.pelvis].reshape(3, 3)
        grav = R.T @ np.array([0.0, 0.0, -1.0])
        gyro = d.qvel[3:6]                                    # the free joint's angular rate, pelvis frame
        _, loads = rig.cop()
        a = np.array([rig.applied[n] for n in names]) - default
        rows_o.append(np.concatenate([q - default, 0.1 * qd, grav, 0.25 * gyro, np.array(loads) / weight]))
        rows_a.append(a)
        held = sum(rig.held_load(h, n) for h in ("hand.L", "hand.R") for n in rig.items) if rig.items else 0.0
        rows_f.append((held / weight, getattr(rig, "_felt_f", (0.0, None))[0] / rig.mass))

    r.reset()
    r.hook = hook
    done = fn(r)
    ok = done is not False and r.standing()
    return (key, np.array(rows_o, dtype=np.float32), np.array(rows_a, dtype=np.float32), ok,
            np.array(rows_f, dtype=np.float32).reshape(-1, 2))


def _chain_scenario(seed):
    """A chain.py episode (seed's bag and order) as a scenario: False if a segment failed."""
    import chain
    item, order = chain.order_for(seed)

    def fn(r):
        segs, _, _ = chain.segments(st, r, item)
        hook = r.hook
        for name in order:
            try:
                segs[name](r)
                if chain.HANDBACK:
                    r.to_stand()
            except chain.Fail:
                return False
            finally:
                r.hook = hook                        # a failed sit leaves its own hook behind
            if not r.standing():
                return False
        return True
    return item, order, fn


def main():
    pushes = int(sys.argv[sys.argv.index("--pushes") + 1]) if "--pushes" in sys.argv else 24
    winds = int(sys.argv[sys.argv.index("--winds") + 1]) if "--winds" in sys.argv else 0
    jobs = int(sys.argv[sys.argv.index("--jobs") + 1]) if "--jobs" in sys.argv else 8
    which = sys.argv[sys.argv.index("--set") + 1] if "--set" in sys.argv else "gauntlet"
    chains = int(sys.argv[sys.argv.index("--chains") + 1]) if "--chains" in sys.argv else 24
    if which == "chair":
        work = [k for k, title, note, fn in st.scenarios() if k.startswith("sit")]
        fname = "teacher_chair.npz"
    elif which == "chain":
        work = ["chain_%02d" % i for i in range(chains)]
        fname = "teacher_chain.npz"
    else:
        work = [k for k, title, note, fn in st.scenarios() if k not in SKIP and not k.startswith("sit")]   # no chair
        work += ["push_rand_%02d" % i for i in range(pushes)]
        work += ["wind_rand_%02d" % i for i in range(winds)]
        fname = "teacher.npz"
    # scenario functions are closures, so each worker rebuilds its list and picks its own by key; a chain's bag is
    # its own model, so every task gets a fresh worker
    with mp.Pool(jobs, maxtasksperchild=1) as pool:
        results = pool.map(_record_by_key, [(k, pushes) for k in work], chunksize=1)
    r0 = st.Rig()
    names0 = [w["name"] for w in r0.w]
    lo0, hi0 = np.array([w["lo"] for w in r0.w]), np.array([w["hi"] for w in r0.w])
    out = os.path.join(ROOT, "sim", "out", "teacher")
    os.makedirs(out, exist_ok=True)
    obs, act, ep, keys, kept, felt = [], [], [], [], [], []
    for i, (key, o, a, ok, f) in enumerate(results):
        print("%-16s %5d steps  %s" % (key, len(o), "stands" if ok else "FALLS (left out)"))
        if not ok or len(o) == 0:
            continue
        obs.append(o)
        act.append(a)
        felt.append(f)
        ep.append(np.full(len(o), len(kept)))
        kept.append(key)
    np.savez_compressed(os.path.join(out, fname), obs=np.concatenate(obs), offset=np.concatenate(act),
                        episode=np.concatenate(ep), keys=np.array(kept), rate_hz=RATE_HZ, dof_names=np.array(names0),
                        joint_lo=lo0, joint_hi=hi0, felt=np.concatenate(felt))
    print("wrote sim/out/teacher/%s: %d steps in %d of %d episodes" % (fname, sum(len(o) for o in obs), len(kept),
                                                                    len(results)))


def _record_by_key(arg):
    key, pushes = arg
    table = {k: fn for k, title, note, fn in st.scenarios()}
    table.update({"push_rand_%02d" % i: _push_scenario(1000 + i) for i in range(pushes)})
    st.WIND = None
    if key.startswith("chain_"):
        item, order, fn = _chain_scenario(int(key.split("_")[-1]))
        return record(("%s_%s_%s" % (key, item, "-".join(order)), fn, item))
    if key.startswith("wind_rand_"):
        st.WIND = _wind_spec(2000 + int(key.split("_")[-1]))
        return record((key, lambda r: r.run(12.0)))
    return record((key, table[key]))


if __name__ == "__main__":
    main()
