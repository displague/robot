#!/usr/bin/env python3
"""
export_urdf.py - cad/skeleton.json -> sim/robot.urdf, plus a centre-of-mass / support-polygon check.

The URDF is the hand-off to MuJoCo, PyBullet and Isaac Lab (docs/references.md). Links are the
bones of skeleton.json (left side mirrored), with box visuals/collisions along the bone and the
mass model's inertia. Each DOF becomes one revolute joint about a world axis of the rest pose
(pitch = X, roll = Y, yaw = Z, the convention in skeleton.json "_axes"); multi-DOF joints get
massless intermediate links. The pelvis floats above a world link. Hand and face DOF are not
exported (hands are rigid bodies in simulation for now).

Run:
    python tools/export_urdf.py                 -> sim/robot.urdf, rest-pose CoM report
    python tools/export_urdf.py --pose demo     -> also the CoM of the sign-check pose
"""
import json
import math
import os
import sys

WORLD_AXIS = {"pitch": (1, 0, 0), "roll": (0, 1, 0), "yaw": (0, 0, 1)}
FOOT_BOX = (0.10, 0.26, 0.04)          # printed sole: 100 x 260 mm, 40 mm tall
FOOT_OFFSET = (0.0, -0.08, -0.04)      # sole centre relative to the ankle: forward and down
DEMO_POSE = {"upperarm.L": {"pitch": -90}, "upperarm.R": {"roll": 90}, "forearm.R": {"pitch": -90},
             "thigh.L": {"pitch": -90}, "shank.L": {"pitch": 90}, "foot.R": {"pitch": 30}, "neck": {"yaw": 60}}


def bone_name(base, suffix):
    return base + ("." + suffix if suffix else "")


def sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def add(a, b):
    return [a[i] + b[i] for i in range(3)]


def scale(a, s):
    return [a[i] * s for i in range(3)]


def rot_axis(axis, deg):
    """3x3 rotation about a world axis, right-hand rule."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    if axis == "pitch":
        return [[1, 0, 0], [0, c, -s], [0, s, c]]
    if axis == "roll":
        return [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


IDENT = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


def expand(data):
    """Mirror the left-side bones and joints into a flat list of links with their DOF."""
    defs = {b["name"]: b for b in data["bones"]}
    joints_by_bone = {j["bone"]: j for j in data["joints"] if j.get("bone")}
    links = []
    for b in data["bones"]:
        sides = (("L", 1), ("R", -1)) if b["mirror"] else (("", 1),)
        for suffix, s in sides:
            name = bone_name(b["name"], suffix)
            parent = None
            if b["parent"]:
                psuf = suffix if defs[b["parent"]]["mirror"] else ""
                parent = bone_name(b["parent"], psuf)
            head = [b["head"][0] * s, b["head"][1], b["head"][2]]
            tail = [b["tail"][0] * s, b["tail"][1], b["tail"][2]]
            dofs = []
            j = joints_by_bone.get(b["name"])
            if j:
                for d in j["dof"]:
                    if d["axis"] in WORLD_AXIS:
                        lo, hi = d["min"], d["max"]
                        if s < 0 and d["axis"] in ("roll", "yaw"):
                            lo, hi = -hi, -lo
                        dofs.append({"name": d["name"], "axis": d["axis"], "lo": lo, "hi": hi})
            links.append({"name": name, "parent": parent, "head": head, "tail": tail, "dofs": dofs,
                          "mass": b.get("mass_kg", 0.5), "base": b["name"]})
    return links


def fk(links, pose):
    """World rotation and head position of every link for a pose {link: {axis: deg}}."""
    by = {l["name"]: l for l in links}
    R, pos = {}, {}
    for l in links:   # bones are listed parents-first in skeleton.json
        if l["parent"] is None:
            Rp, pp, hp = IDENT, l["head"], l["head"]
        else:
            Rp, pp, hp = R[l["parent"]], pos[l["parent"]], by[l["parent"]]["head"]
        Rj = IDENT
        for d in l["dofs"]:
            ang = pose.get(l["name"], {}).get(d["axis"], 0.0)
            ang = max(d["lo"], min(d["hi"], ang))
            Rj = matmul(Rj, rot_axis(d["axis"], ang))
        R[l["name"]] = matmul(Rp, Rj)
        pos[l["name"]] = add(pp, matvec(Rp, sub(l["head"], hp)))
    return R, pos


def com(links, pose):
    R, pos = fk(links, pose)
    total, acc = 0.0, [0.0, 0.0, 0.0]
    for l in links:
        d = sub(l["tail"], l["head"])
        c = add(pos[l["name"]], matvec(R[l["name"]], scale(d, 0.5)))
        total += l["mass"]
        acc = add(acc, scale(c, l["mass"]))
    return scale(acc, 1 / total), total, pos, R


def support_polygon(pos, R):
    """Axis-aligned bounding rectangle of the corners of the soles that are on the ground (sole centre
    within 2 cm of the floor): a conservative support polygon. A lifted foot does not count."""
    xs, ys = [], []
    for name in ("foot.L", "foot.R"):
        c = add(pos[name], matvec(R[name], list(FOOT_OFFSET)))
        if c[2] > FOOT_BOX[2] / 2 + 0.05:   # sole centre more than 5 cm up: this foot is in the air
            continue
        for sx in (-1, 1):
            for sy in (-1, 1):
                corner = matvec(R[name], [sx * FOOT_BOX[0] / 2, sy * FOOT_BOX[1] / 2, 0])
                xs.append(c[0] + corner[0])
                ys.append(c[1] + corner[1])
    if not xs:
        return 0.0, 0.0, 0.0, 0.0
    return min(xs), max(xs), min(ys), max(ys)


def box_inertia(m, sx, sy, sz):
    return (m / 12 * (sy * sy + sz * sz), m / 12 * (sx * sx + sz * sz), m / 12 * (sx * sx + sy * sy))


def link_xml(l):
    d = sub(l["tail"], l["head"])
    if l["base"] == "foot":
        size, origin = FOOT_BOX, FOOT_OFFSET
    else:
        size = tuple(max(abs(v), 0.05) for v in d)
        origin = tuple(v / 2 for v in d)
    ixx, iyy, izz = box_inertia(l["mass"], *size)
    o = "%.4f %.4f %.4f" % origin
    s = "%.4f %.4f %.4f" % size
    return ("  <link name=\"%s\">\n"
            "    <inertial><origin xyz=\"%s\"/><mass value=\"%.3f\"/>"
            "<inertia ixx=\"%.5f\" ixy=\"0\" ixz=\"0\" iyy=\"%.5f\" iyz=\"0\" izz=\"%.5f\"/></inertial>\n"
            "    <visual><origin xyz=\"%s\"/><geometry><box size=\"%s\"/></geometry></visual>\n"
            "    <collision><origin xyz=\"%s\"/><geometry><box size=\"%s\"/></geometry></collision>\n"
            "  </link>\n" % (l["name"], o, l["mass"], ixx, iyy, izz, o, s, o, s))


def urdf(links):
    by = {l["name"]: l for l in links}
    out = ["<?xml version=\"1.0\"?>\n<robot name=\"tendon_humanoid\">\n", "  <link name=\"world\"/>\n"]
    for l in links:
        out.append(link_xml(l))
    for l in links:
        if l["parent"] is None:
            out.append("  <joint name=\"float\" type=\"floating\"><parent link=\"world\"/><child link=\"%s\"/>"
                       "<origin xyz=\"%.4f %.4f %.4f\"/></joint>\n" % ((l["name"],) + tuple(l["head"])))
            continue
        origin = sub(l["head"], by[l["parent"]]["head"])
        if not l["dofs"]:
            out.append("  <joint name=\"%s_fixed\" type=\"fixed\"><parent link=\"%s\"/><child link=\"%s\"/>"
                       "<origin xyz=\"%.4f %.4f %.4f\"/></joint>\n" % ((l["name"], l["parent"], l["name"]) + tuple(origin)))
            continue
        parent = l["parent"]
        for k, d in enumerate(l["dofs"]):
            last = k == len(l["dofs"]) - 1
            child = l["name"] if last else "%s__%s" % (l["name"], d["axis"])
            if not last:
                out.append("  <link name=\"%s\"><inertial><mass value=\"0.001\"/>"
                           "<inertia ixx=\"1e-6\" ixy=\"0\" ixz=\"0\" iyy=\"1e-6\" iyz=\"0\" izz=\"1e-6\"/></inertial></link>\n" % child)
            o = origin if k == 0 else (0.0, 0.0, 0.0)
            ax = WORLD_AXIS[d["axis"]]
            out.append("  <joint name=\"%s_%s\" type=\"revolute\"><parent link=\"%s\"/><child link=\"%s\"/>"
                       "<origin xyz=\"%.4f %.4f %.4f\"/><axis xyz=\"%d %d %d\"/>"
                       "<limit lower=\"%.4f\" upper=\"%.4f\" effort=\"30\" velocity=\"3\"/></joint>\n"
                       % ((l["name"], d["axis"], parent, child) + tuple(o) + ax
                          + (math.radians(d["lo"]), math.radians(d["hi"]))))
            parent = child
    out.append("</robot>\n")
    return "".join(out)


def report(links, pose, label):
    c, total, pos, R = com(links, pose)
    x0, x1, y0, y1 = support_polygon(pos, R)
    inside = x0 <= c[0] <= x1 and y0 <= c[1] <= y1
    print("%-10s mass %.1f kg  CoM (%.3f, %.3f, %.3f) m  support x[%.2f, %.2f] y[%.2f, %.2f]  CoM inside: %s" %
          (label, total, c[0], c[1], c[2], x0, x1, y0, y1, inside))
    return inside


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with open(os.path.join(root, "cad", "skeleton.json")) as f:
        data = json.load(f)
    links = expand(data)
    os.makedirs(os.path.join(root, "sim"), exist_ok=True)
    path = os.path.join(root, "sim", "robot.urdf")
    with open(path, "w") as f:
        f.write(urdf(links))
    n_rev = sum(len(l["dofs"]) for l in links)
    print("wrote %s: %d links, %d revolute joints (+ floating base)" % (path, len(links), n_rev))
    report(links, {}, "rest")
    if "--pose" in sys.argv:
        report(links, DEMO_POSE, "demo")
    # one-legged stance: left leg lifted, nothing else moved (expect CoM outside the right foot) ...
    report(links, {"thigh.L": {"pitch": -90}, "shank.L": {"pitch": 90}}, "left-up")
    # ... and the same with the weight shifted over the stance leg: right hip adducts and the waist leans
    # right, which is what the Phase 5 hip strategy has to do before any foot leaves the ground
    report(links, {"thigh.L": {"pitch": -90}, "shank.L": {"pitch": 90},
                   "thigh.R": {"roll": -12}, "spine": {"roll": 18}}, "left-up+shift")


if __name__ == "__main__":
    main()
