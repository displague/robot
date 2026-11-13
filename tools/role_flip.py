"""
role_flip.py - can the robot be the helper? (docs/47, hub task nurturer-role-flip)

docs/46's get-up helper (getup.Lifter) is a pair of scripted hands. Before two robots share one simulation, this asks
the static question: could a second robot of the same build put its hands where those hands were and carry the force
they carried, within its own joints' ratings, without tipping over?

1. `--record`: run docs/46's get-up (sim/getup_params.json, the helper under the arms) and save the helper hands'
   positions and the forces the getting-up robot put on them, every 0.1 s (getup.Lifter.samples).
2. For each sample, a helper robot faces the getting-up one, its feet flat on the floor. A small cross-entropy search
   over its posture (where it stands, ankle, knee, hip, spine, shoulder, elbow, wrist; both sides alike) puts its hands
   on the grip points and minimises its worst joint load:
   - each joint's torque from gravity and the hand forces (statics), against its continuous rating (the line's
     continuous force times its moment arm, sim/robot_standin.json);
   - the legs share the load equally;
   - its centre of pressure must stay inside its feet, and the floor's friction (0.8) must hold.
3. Prints the trajectory: per phase, the worst joint and its load, and the CoP margin.

    python tools/role_flip.py --record [--set firm|faded]     # -> sim/out/role_flip/samples_<set>.json
    python tools/role_flip.py [--set firm] [--scale 1.0] [--evals 400]
    python tools/role_flip.py --pose <t>                      # print the helper's best pose at time t
    python tools/role_flip.py --contact elbow                 # the helper carries the load at its elbows (cradling)

`--scale` multiplies the hand forces (0.5: a helper carrying half of what the scripted one did).
"""
import contextlib
import io
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
os.environ.setdefault("GETUP_GRIP", "elbow")
import getup  # noqa: E402
import mujoco  # noqa: E402
import sim_tendon as st  # noqa: E402
from export_urdf import FOOT_BOX, FOOT_OFFSET  # noqa: E402

D = math.radians
OUT = os.path.join(ROOT, "sim", "out", "role_flip")
CONTACT = "grip.hand.%s"    # where the helper robot carries the load: its hands, or "elbow.%s" (it cradles the other
                            # robot's upper arm on its own forearm, near the elbow: --contact elbow)
MU = 0.8
COP_MARGIN = 0.02           # m: the CoP this far inside the sole's edge counts as inside
# the helper's posture: name, low, high (degrees, metres for "y")
POSE = [("y", -1.2, 0.2), ("ankle", -20.0, 40.0), ("knee", 0.0, 120.0), ("hip", -100.0, 15.0), ("spine", -20.0, 40.0),
        ("shoulder", -170.0, 45.0), ("elbow", -140.0, 0.0), ("wrist", -70.0, 60.0)]
PITCH = ["thigh.%s_pitch", "shank.%s_pitch", "foot.%s_pitch", "spine_pitch", "neck_pitch", "upperarm.%s_pitch",
         "forearm.%s_pitch", "hand.%s_pitch"]
LEG = ("thigh.", "shank.", "foot.")


def record(which):
    params = json.load(open(os.path.join(ROOT, "sim", "getup_params.json")))[which]
    getup.gapply(params["params"])
    r = st.Rig()
    with contextlib.redirect_stdout(io.StringIO()):
        getup.kneel(r)
        h = getup.Lifter(r, **params["helper"])
        r.lifter = h
        r.hook = h.chain()
        res = getup.getup(r)
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, "samples_%s.json" % which)
    json.dump(dict(stood=res["stood"], helper=params["helper"], samples=h.samples), open(path, "w"))
    print("%s: stood %s, %d samples -> %s" % (which, res["stood"], len(h.samples), path))


class Statics:
    """The helper robot's kinematics and statics (a second copy of the model; nothing is simulated)."""

    def __init__(self):
        with contextlib.redirect_stdout(io.StringIO()):
            self.rig = st.Rig()
        m = self.m = self.rig.model
        self.d = mujoco.MjData(m)
        spec = json.load(open(os.path.join(ROOT, "sim", "robot_standin.json")))
        self.cont = {x["name"]: x["line_cont_n"] * x["moment_arm_m"] for x in spec["dofs"]}
        self.jid = {n: m.joint(n).id for n in self.cont}
        root = m.body("pelvis").id
        self.bodies = [b for b in range(m.nbody) if m.body_rootid[b] == root]
        self.mass = np.array([m.body_mass[b] for b in self.bodies])
        self.grip = {s: m.site((CONTACT % s)).id for s in "LR"}
        self.foot = {s: m.body("foot." + s).id for s in "LR"}
        # each joint's body, and the bodies on the far side of it (the subtree for the upper body; for a leg joint,
        # everything but the foot-side subtree, the load it carries in stance)
        self.side = {}
        for n, j in self.jid.items():
            b = m.jnt_bodyid[j]
            sub = {x for x in self.bodies if self._under(x, b)}
            self.side[n] = (set(self.bodies) - sub) if n.startswith(LEG) else sub
        self.hand_body = {s: m.site_bodyid[self.grip[s]] for s in "LR"}

    def _under(self, x, b):
        m = self.m
        while x != 0:
            if x == b:
                return True
            x = m.body_parentid[x]
        return False

    def place(self, p):
        """Set the pose (degrees; p["y"]: where the left sole's centre stands), feet flat, facing +y (toward the
        getting-up robot, which faces -y)."""
        m, d = self.m, self.d
        d.qpos[:] = 0.0
        q = {}
        for s in "LR":
            q["foot.%s_pitch" % s], q["shank.%s_pitch" % s], q["thigh.%s_pitch" % s] = (D(p["ankle"]), D(p["knee"]),
                                                                                          D(p["hip"]))
            q["upperarm.%s_pitch" % s], q["forearm.%s_pitch" % s], q["hand.%s_pitch" % s] = (D(p["shoulder"]),
                                                                                           D(p["elbow"]),
                                                                                           D(p["wrist"]))
        q["spine_pitch"] = D(p["spine"])
        for k, v in q.items():
            d.qpos[m.jnt_qposadr[m.joint(k).id]] = v
        pitch = -(D(p["ankle"]) + D(p["hip"]) + D(p["knee"]))       # the body pitch that keeps the soles flat
        qy = np.array((0.0, 0.0, 0.0, 1.0))                          # yaw 180 degrees: facing +y
        qp = np.array((math.cos(pitch / 2), math.sin(pitch / 2), 0.0, 0.0))
        quat = np.zeros(4)
        mujoco.mju_mulQuat(quat, qy, qp)
        d.qpos[3:7] = quat
        d.qpos[0:3] = 0.0
        mujoco.mj_kinematics(m, d)
        sole = self.sole("L")
        d.qpos[0:3] = (-sole[0] + 0.0 * sole[0], p["y"] - sole[1], -sole[2] + FOOT_BOX[2] / 2)
        d.qpos[0] = 0.0
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)

    def sole(self, s):
        d = self.d
        b = self.foot[s]
        return d.xpos[b] + d.xmat[b].reshape(3, 3) @ np.array(FOOT_OFFSET)

    def loads(self, forces):
        """forces: {helper side: force on that hand (N, world)}. Returns ({joint: |torque| / rating}, CoP margin m,
        friction use)."""
        d = self.d
        g = np.array((0.0, 0.0, -9.81))
        com = {b: d.xipos[b] for b in self.bodies}
        hand = {s: d.site_xpos[self.grip[s]] for s in "LR"}
        out = {}
        for n, j in self.jid.items():
            if not n.endswith("_pitch") or n.startswith(("neck",)):
                continue
            p, ax = d.xanchor[j], d.xaxis[j]
            side = self.side[n]
            tau = sum(np.dot(ax, np.cross(com[b] - p, self.m.body_mass[b] * g)) for b in side)
            for s, f in forces.items():
                if self.hand_body[s] in side:
                    tau += np.dot(ax, np.cross(hand[s] - p, f))
            if n.startswith(LEG):
                tau *= 0.5                       # the two legs share it (the other leg's own mass aside)
                # the other leg hangs off the pelvis on this one's far side: counted in `side` already, halved with it
            out[n] = abs(tau) / self.cont[n]
        fe = sum(self.m.body_mass[b] * g for b in self.bodies) + sum(forces.values())
        me = sum(np.cross(com[b], self.m.body_mass[b] * g) for b in self.bodies)
        me = me + sum(np.cross(hand[s], f) for s, f in forces.items())
        rz = -fe[2]
        if rz <= 0:
            return out, -1.0, 9.9
        py = -me[0] / rz                         # the CoP: the floor's reaction balances the moment about x
        ys = [self.sole(s)[1] for s in "LR"]
        half = FOOT_BOX[1] / 2 - COP_MARGIN
        margin = min(py - (min(ys) - half), (max(ys) + half) - py)
        fric = math.hypot(fe[0], fe[1]) / (MU * rz)
        return out, margin, fric


def solve(stx, sample, scale, evals, start=None, rng=None):
    """The helper's posture for one sample: hands on the grip points, the worst joint load least."""
    rng = rng or np.random.default_rng(0)
    lo = np.array([x[1] for x in POSE])
    hi = np.array([x[2] for x in POSE])
    # the helper's left hand takes the getting-up robot's right side (they face each other)
    tgt = {"L": np.array(sample["x"]["R"]), "R": np.array(sample["x"]["L"])}
    frc = {"L": scale * np.array(sample["f"]["R"]), "R": scale * np.array(sample["f"]["L"])}

    def cost(u):
        p = dict(zip([x[0] for x in POSE], lo + np.clip(u, 0, 1) * (hi - lo)))
        stx.place(p)
        err = sum(np.linalg.norm(stx.d.site_xpos[stx.grip[s]] - tgt[s]) for s in "LR")
        ld, margin, fric = stx.loads(frc)
        worst = max(ld.values())
        c = worst + 200.0 * err + 50.0 * max(0.0, -margin) + 5.0 * max(0.0, fric - 1.0)
        return c, p, worst, ld, err, margin, fric

    mean = np.full(len(POSE), 0.5) if start is None else start
    std = np.full(len(POSE), 0.3 if start is None else 0.08)
    best = None
    pop = 20
    for _ in range(max(1, evals // pop)):
        us = [np.clip(mean + std * rng.standard_normal(len(POSE)), 0, 1) for _ in range(pop)]
        rs = [(cost(u), u) for u in us]
        rs.sort(key=lambda x: x[0][0])
        if best is None or rs[0][0][0] < best[0][0]:
            best = rs[0]
        el = np.array([u for _, u in rs[:5]])
        mean = 0.3 * mean + 0.7 * el.mean(axis=0)
        std = np.clip(0.6 * std + 0.4 * el.std(axis=0), 0.01, 0.3)
    (c, p, worst, ld, err, margin, fric), u = best
    return dict(t=sample["t"], phase=sample["phase"], pose=p, worst=worst, worst_joint=max(ld, key=ld.get),
                loads=ld, err=err, margin=margin, fric=fric, force=float(sum(np.linalg.norm(v) for v in frc.values()))), u


def main():
    argv = sys.argv[1:]

    def opt(k, default):
        if k in argv:
            i = argv.index(k)
            v = argv[i + 1]
            del argv[i:i + 2]
            return v
        return default
    which = opt("--set", "firm")
    global CONTACT
    if opt("--contact", "hand") == "elbow":
        CONTACT = "elbow.%s"
    if "--record" in argv:
        return record(which)
    scale = float(opt("--scale", 1.0))
    evals = int(opt("--evals", 400))
    at = opt("--pose", None)
    data = json.load(open(os.path.join(OUT, "samples_%s.json" % which)))
    samples = [x for x in data["samples"] if len(x["x"]) == 2]
    if at is not None:
        samples = [min(samples, key=lambda x: abs(x["t"] - float(at)))]
    stx = Statics()
    rng = np.random.default_rng(1)
    rows, u = [], None
    for smp in samples:
        res, u = solve(stx, smp, scale, evals, u, rng)
        if res["err"] > 0.03:                     # lost the hands: search afresh
            res2, u2 = solve(stx, smp, scale, 2 * evals, None, rng)
            if res2["worst"] + 200 * res2["err"] < res["worst"] + 200 * res["err"]:
                res, u = res2, u2
        rows.append(res)
        if at is not None:
            print(json.dumps(res, indent=1))
    if at is not None:
        return
    print("set %s, forces x%.2f, %d samples (every %.1f s)" % (which, scale, len(rows), getup.SAMPLE_DT))
    print("%-18s %5s %7s %9s  %-22s %6s %8s %6s" % ("phase", "n", "force N", "worst", "worst joint", "reach",
                                                      "CoP m", "fric"))
    for ph in dict.fromkeys(r["phase"] for r in rows):
        rr = [r for r in rows if r["phase"] == ph]
        w = max(rr, key=lambda r: r["worst"])
        print("%-18s %5d %7.0f %8.2fx  %-22s %5.0fmm %+8.3f %6.2f" % (
            ph, len(rr), max(r["force"] for r in rr), w["worst"], w["worst_joint"],
            1000 * max(r["err"] for r in rr), min(r["margin"] for r in rr), max(r["fric"] for r in rr)))
    peak = {}
    for r in rows:
        for k, v in r["loads"].items():
            peak[k] = max(peak.get(k, 0.0), v)
    print("peak load / continuous rating: " + ", ".join("%s %.2f" % kv for kv in sorted(peak.items(),
                                                                                       key=lambda kv: -kv[1])[:8]))
    ok = [r for r in rows if r["worst"] <= 1.0 and r["err"] < 0.03 and r["margin"] >= 0 and r["fric"] <= 1.0]
    print("samples a helper robot could hold within every rating, upright and gripping: %d of %d" % (len(ok), len(rows)))
    json.dump(rows, open(os.path.join(OUT, "flip_%s_x%.2f_%s.json" % (which, scale, CONTACT.split(".")[0])), "w"))


if __name__ == "__main__":
    main()
