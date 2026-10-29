"""Postures against wind: drag, centre of drag and overturning need per posture (static, inertia-box drag as MuJoCo
computes it), and whether the posture stands in a wind (dynamic, the real controller).
Run:
    python tools/posture_wind.py static                 # drag, centre of drag, CoM offset needed per posture at 10 m/s
    python tools/posture_wind.py sweep postures=stand,bow35 speeds=12,14 dirs=head gusts=0,1.5 move=0.5
    T0=2.5 python tools/posture_wind.py one bow35 14 0 head 0.5   # posture started at T0 s (the wind ramps 0.5-2.5 s)
"""
import io, contextlib, math, os, subprocess, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

D = math.radians
POSTURES = {
    "stand": {},
    "squat": {"thigh.%s_pitch" % s: D(-10) for s in "LR"} | {"shank.%s_pitch" % s: D(30) for s in "LR"}
             | {"foot.%s_pitch" % s: D(-20) for s in "LR"},
    "bow20": {"spine_pitch": D(20)},
    "bow35": {"spine_pitch": D(35)},
    "back20": {"spine_pitch": D(-20)},
    "arms_in": {"forearm.L_pitch": D(-120), "forearm.R_pitch": D(-120),
                "upperarm.L_pitch": D(-20), "upperarm.R_pitch": D(-20)},
    "hands_edge": {"forearm.L_yaw": D(80), "forearm.R_yaw": D(-80)},
    "twist": {"spine_yaw": D(40), "thigh.L_yaw": D(30), "thigh.R_yaw": D(30)},
}
POSTURES["squat_bow"] = POSTURES["squat"] | POSTURES["bow20"]
POSTURES["squat_bow_arms"] = POSTURES["squat_bow"] | POSTURES["arms_in"]


def drag(m, d, u_world, rho=1.2):
    """Total drag force (world) and its moment about the world origin, the inertia-box model as MuJoCo computes it
    for a still body in wind u: f_i = 0.5 rho A_i |v_i| v_i in the inertia frame, v = u (air relative to body)."""
    F, M = np.zeros(3), np.zeros(3)
    for b in range(1, m.nbody):
        mass = m.body_mass[b]
        if mass < 1e-3:
            continue
        I = m.body_inertia[b]
        r = [math.sqrt(max(0.0, 3 / (2 * mass) * (I[(i + 1) % 3] + I[(i + 2) % 3] - I[i]))) for i in range(3)]
        R = d.ximat[b].reshape(3, 3)
        v = R.T @ u_world
        f = np.array([0.5 * rho * 4 * r[(i + 1) % 3] * r[(i + 2) % 3] * abs(v[i]) * v[i] for i in range(3)])
        fw = R @ f
        F += fw
        M += np.cross(d.xipos[b], fw)
    return F, M


def settle(posture, move_s, extra=()):
    import sim_tendon as st
    for kv in extra:
        k, v = kv.split("=", 1)
        setattr(st, k, eval(v, vars(st)))
    rig = st.Rig()
    rig.reset()
    return st, rig


def static():
    import sim_tendon as st
    print("%-15s %-6s %6s %6s %7s %8s %8s  %s" % ("posture", "from", "F N", "z_cd", "CoM z", "need mm", "U_eq", "stands"))
    base = {}
    for name, tg in POSTURES.items():
        rig = st.Rig()
        rig.reset()
        with contextlib.redirect_stdout(io.StringIO()):
            rig.run(1.5)
            if tg:
                rig.move(tg, 1.0)
            rig.run(3.0)
        m, d = rig.model, rig.data
        com = d.subtree_com[rig.pelvis]
        mg = rig.mass * 9.81
        for wname, u in (("head", np.array([0, 10.0, 0])), ("left", np.array([-10.0, 0, 0]))):
            F, M = drag(m, d, u)
            ax = 1 if wname == "head" else 0
            # overturning moment about the floor line under the CoM, in the wind's direction
            fh = F[ax]
            z = (M[0] / -F[1]) if wname == "head" else (M[1] / F[0])
            need = abs(fh) * z / mg
            key = (name, wname)
            base.setdefault(wname, need)
            print("%-15s %-6s %6.1f %6.2f %7.2f %8.0f %8.2f  %s" % (
                name, wname, abs(fh), z, com[2], 1000 * need, math.sqrt(base[wname] / need),
                "S" if rig.standing() else "F"))


def one(posture, mean, gust, wdir, move_s, *extra):
    import sim_tendon as st
    for kv in extra:
        k, v = kv.split("=", 1)
        setattr(st, k, eval(v, vars(st)))
    st.WIND = dict(mean=float(mean), gust=float(gust), dir=wdir, seed=0, start=0.5, ramp=2.0)
    rig = st.Rig()
    rig.reset()
    fell = []

    def hook(r):
        if not fell and r.data.xmat[r.pelvis][8] < 0.94:
            fell.append("@%.0fs" % r.data.time)
    rig.hook = hook
    tg = POSTURES[posture]
    with contextlib.redirect_stdout(io.StringIO()):
        if tg and float(move_s) == 0:          # already in the posture before the wind
            rig.run(0.3)
        rig.run(float(os.environ.get("T0", "0.2")))
        if tg:
            rig.move(tg, max(float(move_s), 0.4))
        rig.run(20.0)
    w = rig.wind_stat
    print("RESULT %s %s %.1f %.0f" % ("S" if rig.standing() else "F", fell[0] if fell else "-",
                                      w[0] / max(w[2], 1), sum(s["e_j"] for s in rig.stats.values())))


def sweep(postures, speeds, dirs, gusts, move_s="0.8", extra=()):
    runs = [(p, v, g, dr, move_s) for p in postures for dr in dirs for v in speeds for g in gusts]
    live, res, todo = [], {}, list(runs)
    while todo or live:
        while todo and len(live) < int(os.environ.get("JOBS", 12)):
            r = todo.pop(0)
            live.append((r, subprocess.Popen([sys.executable, __file__, "one", *r, *extra],
                                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)))
        r, p = live.pop(0)
        out = p.communicate()[0]
        ln = [x for x in out.splitlines() if x.startswith("RESULT")]
        res[r] = ln[0].split()[1:] if ln else ["E", out.strip()[-60:]]
    for dr in dirs:
        for g in gusts:
            print("\nwind from %s, gust %s (cells: S/F fall-time drag J)" % (dr, g))
            print("%-15s" % "posture" + "".join("%-22s" % ("%s m/s" % v) for v in speeds))
            for p in postures:
                print("%-15s" % p + "".join("%-22s" % " ".join(res[(p, v, g, dr, move_s)]) for v in speeds))


if __name__ == "__main__":
    if sys.argv[1] == "static":
        static()
    elif sys.argv[1] == "one":
        one(*sys.argv[2:])
    elif sys.argv[1] == "sweep":
        a = dict(x.split("=", 1) for x in sys.argv[2:] if "=" in x and x.split("=")[0] in
                 ("postures", "speeds", "dirs", "gusts", "move"))
        extra = [x for x in sys.argv[2:] if x.split("=")[0] not in ("postures", "speeds", "dirs", "gusts", "move")]
        sweep(a.get("postures", ",".join(POSTURES)).split(","), a.get("speeds", "10,12,14,16").split(","),
              a.get("dirs", "head,left").split(","), a.get("gusts", "0").split(","), a.get("move", "0.8"), extra)
