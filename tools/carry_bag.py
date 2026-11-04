"""Carry a grocery bag (docs/28): a bag hangs on a ball joint from the hand (or the wrist, forearm level), and a
gauntlet scene runs with it. One case per child process, so a grid is a loop in the shell.

    python tools/carry_bag.py kg=3.9 style=down scene=walk
    python tools/carry_bag.py kg=6.8 style=both scene=onefoot ramp=3     # the weight comes on over 3 s
    ROBOT_MASSES=sim/masses/rollup_775.json ROBOT_MOTORS=nom python tools/carry_bag.py kg=11.3 style=front scene=stand

style: down  - one bag in the left hand, arm hanging (the grip carries it)
       both  - one bag in each hand, arms hanging
       front - left forearm level in front, the bag on a strap at the wrist (no grip; the elbow holds it)
Prints one line: standing or falling, time, energy, the bag's peak swing, and the lines that matter with their
peak force and safety factor (break / peak). Extra K=V pairs set sim_tendon globals as in step_sweep.
"""
import contextlib
import io
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import export_mjcf as ex  # noqa: E402
import sim_tendon as st   # noqa: E402
import mujoco             # noqa: E402

LINES = ("hand.L_pitch", "forearm.L_pitch", "upperarm.L_pitch", "upperarm.L_roll", "spine_roll", "spine_pitch",
         "thigh.L_roll", "thigh.R_roll", "shank.L_pitch", "foot.L_pitch", "foot.L_roll", "foot.R_roll")


def main():
    kg, style, scene, ramp = 3.9, "down", "stand", 0.0
    for a in sys.argv[1:]:
        k, v = a.split("=", 1)
        if k == "kg":
            kg = float(v)
        elif k == "style":
            style = v
        elif k == "scene":
            scene = v
        elif k == "ramp":
            ramp = float(v)          # s: the bag's weight comes on over this long from the start (a lift)
        else:
            setattr(st, k, eval(v, vars(st)))
    if kg > 0:
        if style == "both":
            ex.BAGS.update({"hand.L": kg, "hand.R": kg})
        elif style == "front":
            ex.BAGS.update({"forearm.L": kg})
        else:
            ex.BAGS.update({"hand.L": kg})
    rig = st.Rig()
    rig.reset()
    fns = {k: fn for k, _, _, fn in st.scenarios()}
    fn = fns[scene]
    m, d = rig.model, rig.data
    bags = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "bag." + b) for b in ex.BAGS]
    swing = [0.0]
    full = [(m.body_mass[b], m.body_inertia[b].copy()) for b in bags]

    def hook(r):
        if ramp > 0 and d.time <= ramp + 0.01:
            f = max(0.002, min(1.0, d.time / ramp))
            for b, (mb, ib) in zip(bags, full):
                m.body_mass[b], m.body_inertia[b] = mb * f, ib * f
        for b in bags:
            z = d.xmat[b].reshape(3, 3)[:, 2]            # the bag's axis: 0 deg is hanging plumb
            swing[0] = max(swing[0], math.degrees(math.acos(max(-1.0, min(1.0, z[2])))))
    rig.hook = hook

    def run(r):
        if style == "front":
            r.run(0.5)
            r.move({"forearm.L_pitch": math.radians(-90)}, 1.5)    # forearm level, the bag on a strap at the wrist
        fn(r)
    with contextlib.redirect_stdout(io.StringIO()):
        run(rig)
    t = d.time
    e = sum(s["e_j"] for s in rig.stats.values())
    by = {w["name"]: w for w in rig.w}
    cells = []
    for n in LINES:
        if n not in by:
            continue
        s, p = rig.stats[n], by[n]["p"]
        f = max(s["fA"], s["fB"])
        cells.append("%s %.0fN/%.1f" % (n, f, p["break_n"] / max(f, 1e-9)))
    worst = min(((by[w["name"]]["p"]["break_n"] / max(rig.stats[w["name"]]["fA"], rig.stats[w["name"]]["fB"], 1e-9),
                  w["name"]) for w in rig.w))
    print("%-5s %-5s %5.1f kg %-8s %-6s t=%5.1f E=%6.0f J (%5.1f W) swing %4.1f deg | worst SF %.1f %s | %s"
          % (style, os.environ.get("ROBOT_MOTORS", "def"), kg, scene, "STANDS" if rig.standing() else "FALLS", t, e,
             e / max(t, 1e-9), swing[0], worst[0], worst[1], "; ".join(cells)))


if __name__ == "__main__":
    main()
