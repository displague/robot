"""Wind sweep (docs/25): where each scene fails under a steady or gusting wind, by direction, mean speed and gust
strength, every run a separate process in parallel.

Run:
    python tools/wind_sweep.py                                   # stand, onefoot, step, walk; head/tail/left/right
    python tools/wind_sweep.py --scenes stand,step --dirs head,tail --speeds 0,4,8,12 --gusts 0,2 --seeds 0,1
    python tools/wind_sweep.py ... "LEAN=False"                  # K=V: module constants of sim_tendon, as step_sweep

`stand` here is 20 s of standing (the gauntlet's is 4 s); the rest are the gauntlet's scenes. The wind's mean ramps
in over 2 s from 0.5 s; gusts are Ornstein-Uhlenbeck (sigma = gust along the wind, 0.75 gust across, tau 2 s).
Output: one table per scene and direction, speeds down, gusts across; each cell the runs that stood out of the
seeds, F<phase> for the first fall's phase. ROBOT_MASSES and the other model variables apply as everywhere.
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def one(args):
    """Child: run one scene in one wind, print one machine-readable line."""
    import io
    import contextlib
    sys.path.insert(0, HERE)
    import sim_tendon as st
    scene, mean, gust, wdir, seed = args[:5]
    for kv in args[5:]:
        k, v = kv.split("=", 1)
        setattr(st, k, eval(v, vars(st)))
    st.WIND = dict(mean=float(mean), gust=float(gust), dir=wdir, seed=int(seed)) if float(mean) or float(gust) else None
    rig = st.Rig()
    fell = []

    def hook(r):
        if not fell and r.data.xmat[r.pelvis][8] < 0.94:          # pelvis z axis more than 20 deg from vertical
            fell.append((r.phase or "-").split(" ")[0] + "@%.0fs" % r.data.time)

    fns = {k: fn for k, title, note, fn in st.scenarios()}
    fns["stand"] = lambda r: r.run(20.0)
    rig.reset()
    rig.hook = hook
    with contextlib.redirect_stdout(io.StringIO()):
        fns[scene](rig)
    w = rig.wind_stat
    print("RESULT %s %s %.1f %.1f %.0f" % ("S" if rig.standing() else "F", fell[0] if fell else "-",
                                          w[0] / max(w[2], 1), w[1], sum(s["e_j"] for s in rig.stats.values())))


def main():
    argv = sys.argv[1:]
    if argv and argv[0] == "--one":
        return one(argv[1:])
    opts = dict(scenes="stand,onefoot,step,walk", dirs="head,tail,left,right", speeds="0,4,6,8,10,12",
                gusts="0,2,4", seeds="0")
    for k in list(opts):
        if "--" + k in argv:
            i = argv.index("--" + k)
            opts[k] = argv[i + 1]
            del argv[i:i + 2]
    jobs = int(os.environ.get("WIND_JOBS", os.cpu_count() or 8))
    sc, dirs = opts["scenes"].split(","), opts["dirs"].split(",")
    sp, gu, se = [opts[k].split(",") for k in ("speeds", "gusts", "seeds")]
    runs = [(c, v, g, d, s) for c in sc for d in dirs for v in sp for g in gu for s in se
            if not (float(v) == 0 and float(g) == 0 and d != dirs[0])]       # no wind is the same from any side
    res, live = {}, []
    todo = list(runs)
    while todo or live:
        while todo and len(live) < jobs:
            r = todo.pop(0)
            p = subprocess.Popen([sys.executable, os.path.abspath(__file__), "--one", *r, *argv],
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            live.append((r, p))
        r, p = live.pop(0)
        out = p.communicate()[0]
        line = [x for x in out.splitlines() if x.startswith("RESULT")]
        res[r] = line[0].split()[1:] if line else ["E", out.strip().splitlines()[-1][:40] if out.strip() else "?"]
    print("config: %s" % (" ".join(argv) or "defaults"))
    for c in sc:
        for d in dirs:
            print("\n%s, wind from %s   (cells: stood/seeds, first fall phase@time; drag mean/peak N)" % (c, d))
            print("   m/s  " + "".join("%-26s" % ("gust %s" % g) for g in gu))
            for v in sp:
                cells = []
                for g in gu:
                    rr = [res.get((c, v, g, d if (float(v) or float(g)) else dirs[0], s)) for s in se]
                    rr = [x for x in rr if x]
                    ok = sum(x[0] == "S" for x in rr)
                    fall = next((x[1] for x in rr if x[0] != "S"), "")
                    drag = "%s/%s" % (rr[0][2], rr[0][3]) if rr and len(rr[0]) > 3 else ""
                    cells.append("%-26s" % ("%d/%d %s %s" % (ok, len(rr), fall, drag)))
                print("   %-4s " % v + "".join(cells))


if __name__ == "__main__":
    main()
