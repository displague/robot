"""
chain.py - skills run back to back in one episode, in random orders, with no reset between them (docs/38).

The gauntlet starts every scene from a fresh stand. Here one run strings scenes together: pick a bag up from a
stool, walk, sit on a chair and stand again, put the bag down on another stool, in an order drawn from a seed. Each
scene starts from wherever the last one left the robot; the chair and the stools are placed from where the robot
then stands (Rig.feet_origin). The bags are free bodies gripped by a hand (export_mjcf.ITEMS), of several shapes,
sizes and weights (BAGS below), one per run.

    python tools/chain.py --runs 24 --jobs 12              # 24 seeds, 12 at a time
    python tools/chain.py --runs 24 --jobs 12 NOSLIP=5     # K=V module overrides of sim_tendon, as step_sweep
    python tools/chain.py --one 7                          # one seed, the full log

Output per run: the seed, the bag, the order, and per segment ok or the first failing one (F:<segment>), with the
time, the bag's peak load on the hand and the energy.
"""
import json

import numpy as np
import math
import os
import random
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

# the bags: shape, MuJoCo half sizes (m), kg, handle (m). A grocery bag, a tall bag, a small heavy box, a
# cylinder (a jug or a paint can on a bail), a light wide pillow-sized bag, a ball in a net
BAGS = [
    dict(name="grocery", shape="box", size=[0.05, 0.125, 0.15], kg=2.3, handle=0.10),
    dict(name="tall", shape="box", size=[0.06, 0.10, 0.20], kg=3.0, handle=0.08),
    dict(name="heavybox", shape="box", size=[0.08, 0.08, 0.08], kg=4.5, handle=0.06),
    dict(name="jug", shape="cylinder", size=[0.07, 0.13], kg=3.9, handle=0.05),
    dict(name="pillowbag", shape="box", size=[0.10, 0.25, 0.15], kg=0.8, handle=0.10),
    dict(name="ball", shape="sphere", size=[0.11], kg=1.2, handle=0.15),
]
HAND = "hand.L"
SEAT = 0.45
HANDBACK = True              # chain.py ... HANDBACK=0 (via --no-handback): each skill starts from where the last left it
LIFT = math.radians(-50)     # elbow flexion that lifts the bag off the stool (- is flexion, as SIT_LAP)
SIT_SETDOWN = True           # docs/40: a bag in hand is set down on a top before a sit and taken up again after it
REACH_TOL = 0.06             # m: the furthest the hand's grip may be from the resting bag's handle to take it up again


def segments(st, r, item):
    """The segments of a run, by name: each a function(r) that returns a short note or raises Fail."""
    scenes = {k: fn for k, _, _, fn in st.scenarios()}
    it = [b for b in BAGS if b["name"] == item][0]
    from export_mjcf import item_half_z
    hz = item_half_z(it)
    state = dict(held=False)

    def pick(r):
        r.run(0.5)
        p = r.grip_site(HAND)
        bottom = p[2] - it["handle"] - 2 * hz
        r.place_stool(0, p[:2], bottom - 0.002)
        r.place_item(item, p + (0.0, 0.0, 0.002))
        r.run(0.3)
        r.grip(HAND, item, True)
        r.run(0.3)
        r.move({"forearm.L_pitch": LIFT}, 1.0)
        r.run(0.5)
        r.place_stool(0, None)
        r.move({"forearm.L_pitch": 0.0}, 1.0)        # the arm hangs again, the bag below the hand
        r.run(1.0)
        state["held"] = True
        return "load %.0f N" % r.held_load(HAND, item)

    def bottom_xy():
        """The item's lowest point (z) and its body's centre (x, y), from its geom's pose and shape."""
        m, d = r.model, r.data
        gid = m.geom("item." + item).id
        c, R, sz = d.geom_xpos[gid], d.geom_xmat[gid].reshape(3, 3), m.geom_size[gid]
        if it["shape"] == "box":
            low = c[2] - sum(abs(R[2, i]) * sz[i] for i in range(3))
        elif it["shape"] == "sphere":
            low = c[2] - sz[0]
        else:
            ca = abs(R[2, 2])
            low = c[2] - ca * sz[1] - sz[0] * math.sqrt(max(0.0, 1 - ca * ca)) - (sz[0] * ca if it["shape"] == "capsule" else 0)
        return low, c[:2].copy()

    def place(r):
        # set the bag down on a counter-height top under where it hangs (arm down, settled), then let go
        r.run(1.0)
        low, xy = bottom_xy()
        r.place_stool(1, xy, low - 0.003)
        r.run(0.3)
        q, v = r.item_qadr(item)
        for _ in range(int(round(2.0 / r.model.opt.timestep))):   # let go once it is still on the top (a ball rolls)
            if np.linalg.norm(r.data.qvel[v:v + 3]) < 0.02 and np.linalg.norm(r.data.qvel[v + 3:v + 6]) < 0.2:
                break
            r.step()
        r.grip(HAND, item, False)
        r.run(1.5)
        state["held"] = False
        low2, xy2 = bottom_xy()
        top = low - 0.003
        q, v = r.item_qadr(item)
        if abs(low2 - top) > 0.02 or abs(r.data.qvel[v + 2]) > 0.05:
            raise Fail("the bag is not resting on the top (its bottom %.0f mm, the top %.0f mm)" % (low2 * 1e3, top * 1e3))
        return "on the top at %.2f m" % top

    def take_up(r):
        """Take the bag up again from the top it rests on, where the hand now is (after a sit)."""
        r.run(0.5)
        gap = float(np.linalg.norm(r.grip_site(HAND) - r.item_grip(item)))
        if gap > REACH_TOL:
            raise Fail("the bag's handle is %.0f mm from the hand" % (gap * 1e3))
        r.grip(HAND, item, True)
        r.run(0.3)
        r.move({"forearm.L_pitch": LIFT}, 1.0)
        r.run(0.5)
        r.place_stool(1, None)
        r.move({"forearm.L_pitch": 0.0}, 1.0)
        r.run(1.0)
        state["held"] = True
        return gap

    def walk(r):
        scenes["walk"](r)
        return ""

    def sit(r):
        down = SIT_SETDOWN and state["held"]
        if down:                                   # the planner sets the bag down first (docs/40)
            place(r)
            r.to_stand()
        r.origin = r.feet_origin()
        st.sit_stand(r, SEAT, False)
        r.set_chair(None)
        if not r.sit_result["stood"]:
            raise Fail("did not stand again" if r.sit_result["sat"] else "did not sit")
        note = "reach %.1f cm" % (r.sit_result["reach"] * 100)
        if down:
            r.to_stand()
            note += ", the bag set down and taken up again (%.0f mm off)" % (take_up(r) * 1e3)
        return note

    def stand(r):
        r.run(2.0)
        return ""

    return dict(pick=pick, place=place, walk=walk, sit=sit, stand=stand), state


class Fail(Exception):
    pass


def order_for(seed):
    """A random order with the bag picked before it is placed; walking and sitting may come with the bag or not."""
    rnd = random.Random(seed)
    item = rnd.choice(BAGS)["name"]
    rest = ["walk", "sit"] + (["walk"] if rnd.random() < 0.5 else [])
    rnd.shuffle(rest)
    i, j = sorted(rnd.sample(range(len(rest) + 2), 2))
    order = rest[:]
    order.insert(i, "pick")
    order.insert(j, "place")
    return item, order


def one(argv):
    global HANDBACK
    if "--no-handback" in argv:
        argv = [a for a in argv if a != "--no-handback"]
        HANDBACK = False
    seed = int(argv[0])
    item, order = order_for(seed)
    import export_mjcf
    export_mjcf.ITEMS = [dict(b) for b in BAGS if b["name"] == item]
    sys.path.insert(0, HERE)
    import sim_tendon as st
    st.STOOL_HALF = export_mjcf.STOOL_HALF
    for kv in argv[1:]:
        k, v = kv.split("=", 1)
        setattr(st, k, eval(v, vars(st)))
    r = st.Rig()
    r.reset()
    segs, state = segments(st, r, item)
    peak = [0.0]

    def hook(_r):
        peak[0] = max(peak[0], _r.held_load(HAND, item))
    r.hook = hook
    notes, failed = [], None
    for name in order:
        try:
            note = segs[name](r)
            if HANDBACK:
                r.to_stand()                      # a quiet two-footed stand before the next skill (Rig.to_stand)
            if not r.standing():
                raise Fail("fell")
            notes.append("%s ok%s" % (name, (" (" + note + ")") if note else ""))
        except Fail as e:
            failed = name
            notes.append("%s FAILED: %s" % (name, e))
            break
    e = sum(s["e_j"] for s in r.stats.values())
    print("   " + "; ".join(notes))
    print("RESULT %d %s %s %s %.1f %.0f %.0f" % (seed, item, "-".join(order), failed or "-", r.data.time, peak[0], e))


def main():
    argv = sys.argv[1:]
    if argv and argv[0] == "--one":
        return one(argv[1:])
    runs, jobs = 12, 12
    if "--runs" in argv:
        i = argv.index("--runs")
        runs = int(argv[i + 1])
        del argv[i:i + 2]
    if "--jobs" in argv:
        i = argv.index("--jobs")
        jobs = int(argv[i + 1])
        del argv[i:i + 2]
    if "--no-handback" in argv:
        argv.remove("--no-handback")
        argv.append("--no-handback")
    ok, rows = 0, []
    for start in range(0, runs, jobs):
        procs = [(s, subprocess.Popen([sys.executable, os.path.abspath(__file__), "--one", str(s)] + argv,
                                      stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                      cwd=os.path.dirname(HERE)))
                 for s in range(start, min(runs, start + jobs))]
        for s, p in procs:
            out = p.communicate()[0]
            m = re.search(r"RESULT (\d+) (\S+) (\S+) (\S+) ([\d.]+) (\d+) (\d+)", out)
            if not m:
                rows.append("%3d error: %s" % (s, out.strip().splitlines()[-1] if out.strip() else "no output"))
                continue
            seed, item, order, failed, t, peak, e = m.groups()
            ok += failed == "-"
            rows.append("%3s %-9s %-32s %-8s %5.0f s  hand peak %3s N  %5s J" %
                        (seed, item, order, "ok" if failed == "-" else "F:" + failed, float(t), peak, e))
    print("\n".join(rows))
    print("chains done %d/%d" % (ok, runs))


if __name__ == "__main__":
    main()
