"""Sit on a chair and stand again, over seat heights and armrests (docs/33, hub task sit-and-stand-chair).

Runs sim_tendon's chair scenes (sit35 .. sit50, sitNNa with armrests), each in its own process, and prints one row per
case: whether it sat and stood again, the peak knee and hip pitch line force against the continuous rating and the
safety factor (break / peak), the worst line overall, the clutch slip summed over every drum, and the energy (to
seated, and in all).

    python tools/chair_sweep.py                       # 0.35, 0.40, 0.45, 0.50 m, with and without armrests
    python tools/chair_sweep.py seats=0.40,0.45 arms=0
    ROBOT_MASSES=sim/masses/rollup_775.json ROBOT_MOTORS=low python tools/chair_sweep.py   # another build
    python tools/chair_sweep.py SIT_KNEE_RATE=0.5     # K=V: a sim_tendon module constant for every case
"""
import concurrent.futures as cf
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def child(key, sets):
    sys.path.insert(0, HERE)
    import sim_tendon as st
    for kv in sets:
        k, v = kv.split("=", 1)
        setattr(st, k, eval(v, vars(math)))
    r = st.Rig()
    r.reset()
    fns = {k: fn for k, _, _, fn in st.scenarios()}
    import io, contextlib
    with contextlib.redirect_stdout(io.StringIO()):
        fns[key](r)
    out = dict(r.sit_result)
    rows = {}
    for w in r.w:
        s, p = r.stats[w["name"]], w["p"]
        f = max(s["fA"], s["fB"])
        rows[w["name"]] = (f, p["f_cont"], p["break_n"] / max(f, 1e-9), math.degrees(s["slip"]))
    pick = lambda pre: max(((v[0] / v[1], k) for k, v in rows.items() if k.startswith(pre)))[1]
    out["knee"] = (pick("shank."),) + rows[pick("shank.")]
    out["hip"] = (pick("thigh.") ,) + rows[pick("thigh.")]
    worst = max(rows, key=lambda k: rows[k][0] / rows[k][1])
    out["worst"] = (worst,) + rows[worst]
    out["slip"] = sum(v[3] for v in rows.values())
    out["slips"] = sorted(((v[3], k) for k, v in rows.items() if v[3] > 1.0), reverse=True)[:3]
    out["energy"] = sum(s["e_j"] for s in r.stats.values())
    rest = [ph for name, ph, _ in r.phases if name == "sit: rest"]
    out["rest_w"] = (sum(v["e_j"] for v in rest[0].values()) / (max(v["n"] for v in rest[0].values())
                     * r.model.opt.timestep)) if rest else float("nan")
    print("RESULT " + json.dumps(out))


def main():
    a = sys.argv[1:]
    if a and a[0] == "--child":
        child(a[1], a[2:])
        return
    seats, arms, sets = ("0.35", "0.40", "0.45", "0.50"), ("0", "1"), []
    for x in a:
        k, v = x.split("=", 1)
        if k == "seats":
            seats = tuple(v.split(","))
        elif k == "arms":
            arms = tuple(v.split(","))
        else:
            sets.append(x)
    keys = ["sit%02d%s" % (round(float(s) * 100), "a" if ar == "1" else "") for s in seats for ar in arms]

    def run(key):
        p = subprocess.run([sys.executable, __file__, "--child", key] + sets, capture_output=True, text=True)
        line = [l for l in p.stdout.splitlines() if l.startswith("RESULT ")]
        return key, json.loads(line[0][7:]) if line else p.stderr[-400:]

    print("seat m | armrests | sits | stands | knee line peak/cont N, SF | hip line peak/cont N, SF | worst line | "
          "slip deg (all drums) | energy J to seated / all | seated W | static reach cm")
    with cf.ThreadPoolExecutor(int(os.environ.get("CHAIR_JOBS", "8"))) as ex:
        for key, res in ex.map(run, keys):
            if isinstance(res, str):
                print(key, "ERROR", res)
                continue
            k, h, w = res["knee"], res["hip"], res["worst"]
            print("%.2f   | %-8s | %-4s | %-6s | %-10s %3.0f/%3.0f, %4.1f | %-10s %3.0f/%3.0f, %4.1f | %s %.0f/%.0f N | %4.0f %s | %4.0f / %4.0f | %3.0f | %.1f"
                  % (res["seat"], "yes" if res["arms"] else "no", "yes" if res["sat"] else "no",
                     "yes" if res["stood"] else "no", k[0].split("_")[0], k[1], k[2], k[3],
                     h[0], h[1], h[2], h[3], w[0], w[1], w[2], res["slip"],
                     "(" + ", ".join("%s %.0f" % (n, s) for s, n in res["slips"]) + ")" if res["slips"] else "",
                     res["e_sat"], res["energy"], res["rest_w"], res["reach"] * 100))


if __name__ == "__main__":
    main()
