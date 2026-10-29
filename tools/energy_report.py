"""Energy of one scene (docs/27): motor electrical energy in total and per winch class, time, distance walked,
mean, 99th-percentile and peak electrical power.

Run:
    python tools/energy_report.py walk 0 head
    ROBOT_MOTORS=nom ROBOT_MASSES=sim/masses/rollup_775.json python tools/energy_report.py walk 3 head

Arguments: scene key (as sim_tendon.scenarios(); `stand` is 20 s), mean wind m/s, direction. Motor energy only:
add the electronics (about 8 W) and the drivers' losses (about 10 %) for a battery (docs/27)."""
import io, contextlib, os, sys, collections
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_tendon as st

scene, mean, wdir = sys.argv[1], float(sys.argv[2]), sys.argv[3]
st.WIND = dict(mean=mean, gust=0.0, dir=wdir, seed=0) if mean else None
rig = st.Rig()
rig.reset()
y0 = rig.data.subtree_com[rig.pelvis][1]
pw, last = [], [0.0]


def hook(r):
    e = sum(s["e_j"] for s in r.stats.values())
    pw.append((e - last[0]) / r.model.opt.timestep)
    last[0] = e
rig.hook = hook
fns = {k: fn for k, t, n, fn in st.scenarios()}
fns["stand"] = lambda r: r.run(20.0)
with contextlib.redirect_stdout(io.StringIO()):
    fns[scene](rig)
T = rig.data.time
dist = -(rig.data.subtree_com[rig.pelvis][1] - y0)
by = collections.Counter()
for w in rig.w:
    by[w["p"]["cls"]] += rig.stats[w["name"]]["e_j"]
E = sum(by.values())
pw = np.array(pw)
ank = rig.by_name.get("foot.L_pitch") or rig.w[0]
print("RESULT %s motors=%s wind=%s%s %s T=%.0fs com_dy=%.3fm E=%.0fJ mean=%.1fW p99=%.0fW peak=%.0fW i_stall(%s)=%.0fA | %s" % (
    scene, os.environ.get("ROBOT_MOTORS", "model"), mean, wdir, "S" if rig.standing() else "F", T, dist, E, E / T,
    np.percentile(pw, 99), pw.max(), ank["name"], ank["p"]["i_stall"],
    " ".join("%s=%.0f" % kv for kv in sorted(by.items(), key=lambda x: -x[1]))))
