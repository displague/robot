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

Run:  python tools/record_teacher.py [--pushes 24] [--jobs 8]   -> sim/out/teacher/teacher.npz
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
SKIP = {"push40"}          # falls


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


def record(job):
    key, fn = job
    r = st.Rig()
    names = [w["name"] for w in r.w]
    lo = np.array([w["lo"] for w in r.w])
    hi = np.array([w["hi"] for w in r.w])
    default = np.array([r.target[n] for n in names])        # the standing pose (ankles at STAND_LEAN)
    m, d = r.model, r.data
    every = max(1, int(round(1.0 / (RATE_HZ * m.opt.timestep))))
    weight = r.mass * 9.81
    rows_o, rows_a = [], []
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

    r.reset()
    r.hook = hook
    fn(r)
    ok = r.standing()
    return key, np.array(rows_o, dtype=np.float32), np.array(rows_a, dtype=np.float32), ok


def main():
    pushes = int(sys.argv[sys.argv.index("--pushes") + 1]) if "--pushes" in sys.argv else 24
    jobs = int(sys.argv[sys.argv.index("--jobs") + 1]) if "--jobs" in sys.argv else 8
    work = [(k, fn) for k, title, note, fn in st.scenarios() if k not in SKIP]
    work += [("push_rand_%02d" % i, _push_scenario(1000 + i)) for i in range(pushes)]
    # scenario functions are closures, so each worker rebuilds its list and picks its own by key
    with mp.Pool(jobs) as pool:
        results = pool.map(_record_by_key, [(k, pushes) for k, _ in work])
    r0 = st.Rig()
    names0 = [w["name"] for w in r0.w]
    lo0, hi0 = np.array([w["lo"] for w in r0.w]), np.array([w["hi"] for w in r0.w])
    out = os.path.join(ROOT, "sim", "out", "teacher")
    os.makedirs(out, exist_ok=True)
    obs, act, ep, keys, kept = [], [], [], [], []
    for i, (key, o, a, ok) in enumerate(results):
        print("%-16s %5d steps  %s" % (key, len(o), "stands" if ok else "FALLS (left out)"))
        if not ok or len(o) == 0:
            continue
        obs.append(o)
        act.append(a)
        ep.append(np.full(len(o), len(kept)))
        kept.append(key)
    np.savez_compressed(os.path.join(out, "teacher.npz"), obs=np.concatenate(obs), offset=np.concatenate(act),
                        episode=np.concatenate(ep), keys=np.array(kept), rate_hz=RATE_HZ, dof_names=np.array(names0),
                        joint_lo=lo0, joint_hi=hi0)
    print("wrote sim/out/teacher/teacher.npz: %d steps in %d episodes" % (sum(len(o) for o in obs), len(kept)))


def _record_by_key(arg):
    key, pushes = arg
    table = {k: fn for k, title, note, fn in st.scenarios()}
    table.update({"push_rand_%02d" % i: _push_scenario(1000 + i) for i in range(pushes)})
    return record((key, table[key]))


if __name__ == "__main__":
    main()
