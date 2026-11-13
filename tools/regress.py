"""
regress.py - the regression gate: has a change moved what the simulation does?

It checks three things against sim/baseline.json, and exits 1 on any difference:
  1. the generated models are current: sim/robot.xml and sim/robot_standin.xml/.json are what tools/export_mjcf.py
     and tools/export_standin.py write today (a stale model is the commonest silent error);
  2. the gauntlet: each scene of sim_tendon.scenarios() run in its own process, its outcome (STANDS or not), its
     flags, and its electrical energy within ENERGY_TOL of the baseline;
  3. with --more, the skills that take longer, as counts over 24 jittered runs, within COUNT_TOL runs of the
     baseline: the get-up with the scripted helper (docs/46) and with a robot as the helper (docs/48).

    python tools/regress.py                 # models and gauntlet (about 2 minutes on 12 processes)
    python tools/regress.py --more          # and the get-up and two-robot jitter tests (about 15 minutes more)
    python tools/regress.py --only walk,sit45
    python tools/regress.py --update        # after a change that is meant to move them: rewrite the baseline,
                                            # and say what moved in the commit and the doc

A parallel run uses 12 processes (the repo's limit is 12 to 14); --jobs changes it.
"""
import concurrent.futures as cf
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
BASELINE = os.path.join(ROOT, "sim", "baseline.json")
ENERGY_TOL = 0.02          # of a scene's energy: other machines round differently
COUNT_TOL = 2              # runs of 24 a jittered count may move
HEAD = re.compile(r"^== (?P<title>.*): t=(?P<t>[\d.]+) s, pelvis (?P<z>[\d.]+) m, tilt (?P<tilt>[\d.]+) deg -> "
                  r"(?P<out>[A-Z]+[^;]*); electrical energy (?P<j>[\d.]+) J")
MORE = {   # name -> (command, the line that carries the count)
    "getup_firm": ([os.path.join(HERE, "getup.py"), "--jitter-test", "sim/getup_params.json:firm"],
                   re.compile(r"(\d+) of (\d+) stand")),
    "duo_standup": ([os.path.join(HERE, "duo.py"), "--jitter-test", "sim/duo_params.json:standup"],
                    re.compile(r"both stand, B having held A, (\d+) of (\d+)")),
}


def models():
    """{file: 'current' | 'stale'}: the committed models against what the exporters write now."""
    sys.path.insert(0, HERE)
    import export_mjcf
    import export_standin
    out = {}
    same = lambda path, text: os.path.exists(path) and open(path).read() == text
    out["sim/robot.xml"] = "current" if same(os.path.join(ROOT, "sim", "robot.xml"), export_mjcf.build()[0]) else "stale"
    xml = export_standin.build()[0]
    out["sim/robot_standin.xml"] = "current" if same(os.path.join(ROOT, "sim", "robot_standin.xml"), xml) else "stale"
    return out


def scene(key):
    p = subprocess.run([sys.executable, os.path.join(HERE, "sim_tendon.py"), "--only", key], capture_output=True,
                       text=True, cwd=ROOT)
    head = flags = None
    for line in p.stdout.splitlines():
        m = HEAD.match(line)
        if m:
            head = m
        elif line.strip().startswith("flags:"):
            flags = line.split("flags:", 1)[1].strip()
    if head is None:
        return key, dict(outcome="NO RESULT", error=(p.stderr or p.stdout)[-300:])
    return key, dict(outcome=head["out"].strip(), energy_j=float(head["j"]), t_s=float(head["t"]),
                     pelvis_m=float(head["z"]), flags=flags or "none")


def more(name):
    cmd, pat = MORE[name]
    p = subprocess.run([sys.executable] + cmd, capture_output=True, text=True, cwd=ROOT,
                       env=dict(os.environ, GETUP_GRIP="elbow"))       # the helper holds the upper arms (D-073)
    for line in reversed(p.stdout.splitlines()):
        m = pat.search(line)
        if m:
            return name, dict(count=int(m.group(1)), of=int(m.group(2)))
    return name, dict(count=None, error=(p.stderr or p.stdout)[-300:])


def compare(base, now):
    """The differences as lines; empty when nothing moved."""
    diff = []
    for f, s in now.get("models", {}).items():
        if s != "current":
            diff.append("%s is stale: run its exporter (tools/export_mjcf.py or tools/export_standin.py)" % f)
    for k, r in now.get("gauntlet", {}).items():
        b = base.get("gauntlet", {}).get(k)
        if b is None:
            diff.append("%s: not in the baseline (%s)" % (k, r["outcome"]))
        elif r["outcome"] != b["outcome"]:
            diff.append("%s: %s, was %s" % (k, r["outcome"], b["outcome"]))
        else:
            if abs(r["energy_j"] - b["energy_j"]) > ENERGY_TOL * max(b["energy_j"], 1.0):
                diff.append("%s: %.1f J, was %.1f J" % (k, r["energy_j"], b["energy_j"]))
            if r["flags"] != b["flags"]:
                diff.append("%s: flags now '%s', were '%s'" % (k, r["flags"], b["flags"]))
    for k, r in now.get("more", {}).items():
        b = base.get("more", {}).get(k)
        if b is None:
            diff.append("%s: not in the baseline (%s of %s)" % (k, r.get("count"), r.get("of")))
        elif r.get("count") is None or abs(r["count"] - b["count"]) > COUNT_TOL:
            diff.append("%s: %s of %s, was %s of %s" % (k, r.get("count"), r.get("of"), b["count"], b["of"]))
    return diff


def main():
    argv = sys.argv[1:]
    opt = lambda k, d=None: argv[argv.index(k) + 1] if k in argv else d
    jobs = int(opt("--jobs", 12))
    sys.path.insert(0, HERE)
    import sim_tendon as st
    keys = [k for k, _, _, _ in st.scenarios() if k not in st.NAMED_ONLY]
    if "--only" in argv:
        keys = [k for k in opt("--only").split(",")]
    now = dict(models=models(), gauntlet={}, more={})
    with cf.ThreadPoolExecutor(jobs) as ex:
        for k, r in ex.map(scene, keys):
            now["gauntlet"][k] = r
            print("%-10s %-8s %8.1f J  %s" % (k, r["outcome"], r.get("energy_j", float("nan")), r.get("flags", r.get("error"))),
                  flush=True)
    if "--more" in argv:
        for name in MORE:                       # one at a time: each already runs 12 processes
            k, r = more(name)
            now["more"][k] = r
            print("%-12s %s of %s" % (k, r.get("count"), r.get("of")), flush=True)
    for f, s in now["models"].items():
        print("%-24s %s" % (f, s))
    base = json.load(open(BASELINE)) if os.path.exists(BASELINE) else {}
    if "--update" in argv:
        failed = [k for k, r in list(now["gauntlet"].items()) + list(now["more"].items()) if "error" in r]
        if failed:                              # a test that did not run is not a baseline
            print("not written: no result from %s" % ", ".join(failed))
            sys.exit(1)
        base.setdefault("gauntlet", {}).update(now["gauntlet"])
        base.setdefault("more", {}).update(now["more"])
        with open(BASELINE, "w") as f:
            json.dump(base, f, indent=1, sort_keys=True)
        print("wrote sim/baseline.json (%d scenes, %d longer tests)" % (len(base["gauntlet"]), len(base["more"])))
        stale = [f for f, s in now["models"].items() if s != "current"]
        sys.exit(1 if stale else 0)
    diff = compare(base, now)
    print("\n".join(["", "MOVED:"] + ["  " + d for d in diff]) if diff else "\nnothing moved against sim/baseline.json")
    sys.exit(1 if diff else 0)


if __name__ == "__main__":
    main()
