"""Robustness sweep for the static step (docs/18): every configuration on two models and three step lengths,
in parallel, with the landing errors and the phase each run falls in.

A single run of the step is deterministic and sits on a knife edge (the open-loop step of docs/16 stands in
2 of 6 of these variants), so a change is judged on this grid, never on one run.

Run:
    python tools/step_sweep.py "open: SWING_IK=False" "ik: SWING_IK=True" ["name: K=V K=V ..."]
    python tools/step_sweep.py --lengths 0.09,0.10,0.11 --models base,stiff "ik: SWING_IK=True"
    python tools/step_sweep.py --masses 20kg=,775=m775.json,worm=mworm.json "now: "   # x mass distributions

Each K=V sets a module constant of sim_tendon before the rig is built (V is a Python expression). Models:
base (the D-032 baseline), stiff (--stiff-mini). Output, one row per configuration:
    name   stands n/N  worst landing error of the standing runs | per run: S or F<phase>[g]: errors (mm)
where the errors are each foot's landing minus its plan, F4 means the pelvis first tilted past 20 degrees in
phase 4, and g means a lift gate timed out.
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def one(args):
    """Child process: run the step once with overrides, print one machine-readable line."""
    import io
    import contextlib
    sys.path.insert(0, HERE)
    model = args[0]
    if model == "stiff":
        import export_mjcf
        export_mjcf.STIFF_MINI = True
    import sim_tendon as st
    for kv in args[1:]:
        k, v = kv.split("=", 1)
        setattr(st, k, eval(v, vars(st)))
    rig = st.Rig()
    fell = []

    def hook(r):
        if not fell and r.data.xmat[r.pelvis][8] < 0.94:          # pelvis z axis more than 20 deg from vertical
            fell.append(r.phase.split(" ")[0])

    for key, title, note, fn in st.scenarios():
        if key != "step":
            continue
        rig.reset()
        rig.hook = hook
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            fn(rig)
        txt = buf.getvalue()
        errs = [int(a) - int(b) for a, b in
                re.findall(r"placed at t=[\d.]+ s, (-?\d+) mm ahead of the other \(planned (-?\d+)\)", txt)]
        print("RESULT %s %s %s %s" % ("S" if rig.standing() else "F", fell[0] if fell else "-",
                                      "g" if "timed out" in txt else "-", ",".join("%+d" % e for e in errs) or "-"))


def main():
    argv = sys.argv[1:]
    if argv and argv[0] == "--one":
        return one(argv[1:])
    lengths, models = [0.09, 0.10, 0.11], ["base", "stiff"]
    if "--lengths" in argv:
        i = argv.index("--lengths")
        lengths = [float(x) for x in argv[i + 1].split(",")]
        del argv[i:i + 2]
    if "--models" in argv:
        i = argv.index("--models")
        models = argv[i + 1].split(",")
        del argv[i:i + 2]
    masses = [("", os.environ.get("ROBOT_MASSES", ""))]
    if "--masses" in argv:                     # name=path,... ; an empty path is skeleton.json's masses
        i = argv.index("--masses")
        masses = [tuple(x.split("=", 1)) for x in argv[i + 1].split(",")]
        del argv[i:i + 2]
    models = [(m, mn, mp) for mn, mp in masses for m in models]
    cfgs = [(c.split(":", 1)[0].strip(), c.split(":", 1)[1].split() if ":" in c else []) for c in argv]
    jobs = {}
    for name, kv in cfgs:
        for model in models:
            for L in lengths:
                cmd = [sys.executable, os.path.abspath(__file__), "--one", model[0]] + kv + ["STEP_LEN=%r" % L]
                env = dict(os.environ, ROBOT_MASSES=os.path.abspath(model[2]) if model[2] else "")
                jobs[(name, model, L)] = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                                          text=True, cwd=os.path.dirname(HERE), env=env)
    width = max(len(n) for n, _ in cfgs)
    print("models %s, step lengths %s m" % (" ".join(m + ("@" + mn if mn else "") for m, mn, _ in models),
                                            " ".join("%.3f" % L for L in lengths)))
    for name, _ in cfgs:
        cells, n_ok, worst = [], 0, 0
        for model in models:
            for L in lengths:
                out = jobs[(name, model, L)].communicate()[0]
                m = re.search(r"RESULT (\S) (\S+) (\S) (\S+)", out)
                if not m:
                    cells.append("error")
                    continue
                ok, ph, gate, errs = m.groups()
                es = [int(e) for e in errs.split(",")] if errs != "-" else []
                if ok == "S":
                    n_ok += 1
                    worst = max([worst] + [abs(e) for e in es])
                    cells.append("S:%s" % errs)
                else:
                    cells.append("F%s%s:%s" % (ph, "g" if gate == "g" else "", errs))
        print("%-*s  stands %d/%d  worst %3d mm | %s" % (width, name, n_ok, len(models) * len(lengths), worst,
                                                       "  ".join(cells)))


if __name__ == "__main__":
    main()
