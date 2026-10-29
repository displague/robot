"""Mass roll-up: every generated part at the infill it will print with, the bought parts, and the parts not
generated yet, per body segment, against cad/skeleton.json (the masses the tendon model uses) and the 20 kg
budget. Also the filament to buy.

Printed mass from each STL's volume V and surface area A: walls and top/bottom skins are a shell A x WALL
thick, printed solid; the rest is infill at INFILL:  m = rho (min(V, A t) + INFILL (V - min(V, A t))).
The generators' own "mass" figures are solid PETG (100 %).

Run:
    python tools/mass_rollup.py [--stl cad/out] [--build] [--infill 0.25] [--wall 1.2] [--md out.md]

--build runs the Blender generators into --stl first (chain, ring, winch_bank). Every STL must be matched by
PARTS below (or IGNORE); an unmatched one is reported, so a new generator part cannot be left out silently.
Sources for bought masses: docs/05 (Amazon listings, 2026-10-28; the item weights in the hub note on
tasks/printed-mass-rollup), PVC Schedule 40 from its dimensions. Rows marked est are estimates.
"""
import argparse
import json
import math
import os
import re
import struct
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RHO = 1.27e-3                     # PETG, g/mm^3
MIRRORED = ("clavicle", "upperarm", "forearm", "hand", "thigh", "shank", "foot")
BLENDER = os.environ.get("BLENDER", r"C:\Program Files\Blender Foundation\Blender 5.1\blender.exe")

# --------------------------------------------------------------- generated parts: STL name -> (count, segment)
# count is pieces on the robot (instances x pieces per instance); segment names follow cad/skeleton.json, and
# mirrored segments (upperarm, forearm, hand, thigh, shank, foot, clavicle) are totals over both sides here.
# Bearing caps: one per bearing (a universal joint has 4, a hinge 2).
PARTS = {
    "finger_palm": (8, "hand"), "finger_proximal": (8, "hand"), "finger_middle": (8, "hand"),
    "finger_distal": (8, "hand"),
    "thumb_palm": (2, "hand"), "thumb_cmc": (2, "hand"), "thumb_proximal": (2, "hand"), "thumb_distal": (2, "hand"),
    "toe_sole": (2, "foot"), "toe_plate": (2, "foot"),
    "wrist_base": (2, "forearm"), "wrist_cross": (2, "hand"), "wrist_end": (2, "hand"), "wrist_bearing_cap": (8, "hand"),
    "ankle_base": (2, "shank"), "ankle_cross": (2, "foot"), "ankle_end": (2, "foot"), "ankle_bearing_cap": (8, "foot"),
    "neck_base": (1, "spine"), "neck_cross": (1, "neck"), "neck_end": (1, "head"), "neck_bearing_cap": (4, "neck"),
    "waist_base": (1, "pelvis"), "waist_cross": (1, "spine"), "waist_end": (1, "spine"), "waist_bearing_cap": (4, "spine"),
    "elbow_upper": (2, "upperarm"), "elbow_lower": (2, "forearm"), "elbow_bearing_cap": (4, "forearm"),
    "knee_upper": (2, "thigh"), "knee_lower": (2, "shank"), "knee_bearing_cap": (4, "shank"),
    "hip_base": (2, "pelvis"), "hip_cross": (2, "thigh"), "hip_end": (2, "thigh"), "hip_bearing_cap": (8, "thigh"),
    "shoulder_base": (2, "clavicle"), "shoulder_cross": (2, "upperarm"), "shoulder_end": (2, "upperarm"),
    "shoulder_bearing_cap": (8, "upperarm"),
    "forearm_hub": (2, "forearm"), "forearm_rotor": (2, "forearm"),
    "shoulder_rot_hub": (2, "upperarm"), "shoulder_rot_rotor": (2, "upperarm"),
    "hip_rot_hub": (2, "thigh"), "hip_rot_rotor": (2, "thigh"),
    "neck_yaw_hub": (1, "neck"), "neck_yaw_rotor": (1, "neck"),
    "waist_yaw_hub": (1, "spine"), "waist_yaw_rotor": (1, "spine"),
    # W-mini banks of 5: 2 per forearm (7 hand + 2 wrist winches), 1 in the torso (neck), 1 in the head (face)
    "bank5_housing": (6, None), "bank5_lid": (6, None), "bank_drum": (25, None),
}
BANKS = {"forearm": 4, "spine": 1, "head": 1}            # where the 6 W-mini banks sit
IGNORE = {"forearm_6808_hub", "forearm_6808_rotor", "forearm_6808_cap",                      # unused alternative
          "winch_mini_housing", "winch_mini_drum", "winch_mini_lid", "elbow_lower_p1", "elbow_lower_p2"}

# --------------------------------------------------------------- printed parts with no generator yet (est, g)
UNGENERATED = [
    ("pelvis frame (battery and hip W-max bank mount)", "pelvis", 600),
    ("torso frame (spine/shoulder W-max bank, electronics)", "spine", 800),
    ("head shell and face mechanisms", "head", 350),
    ("clavicle brackets", "clavicle", 150),
    ("foot soles (long feet, D-015)", "foot", 500),
    ("W-max gearbox: printed 1:40 planetary, drum, clutch, housing, x21", None, 21 * 150),
    ("W-mid housing and drum, x4", "upperarm", 4 * 40),
]
WMAX_AT = {"pelvis": 6, "spine": 9, "thigh": 6}          # hips in the pelvis; spine + shoulders in the torso;
                                                          # knee + ankle pitch/roll on each thigh
# --------------------------------------------------------------- bought parts (g each)
PVC_KG_M = {"1": 0.446, "1-1/4": 0.605, "1-1/2": 0.722, "2": 0.970}   # Schedule 40, from OD and wall, 1.40 g/cc


def bought():
    rows = []

    def add(name, seg, n, g, src):
        rows.append((name, seg, n, g, src))
    for seg, n in WMAX_AT.items():
        add("775 motor 12 V 12000 rpm", seg, n, 354, "listing")
        add("BTS7960 driver with heatsink", seg, n, 65, "listing")
        add("608 bearings in the gearbox, 2 per W-max", seg, 2 * n, 11.9, "listing")
    add("JGB37-520 encoder gearmotor", "upperarm", 4, 175, "est")
    add("N20 298:1 with encoder (neck, wrist)", "spine", 3, 13, "listing")
    add("N20 298:1 with encoder (neck, wrist)", "forearm", 4, 13, "listing")
    add("N20 100:1 with encoder (hands)", "forearm", 14, 13, "est")
    add("N20 100:1 with encoder (face)", "head", 4, 13, "est")
    # joint bearings: universal joints 4, hinges 2 (608 unless noted)
    add("608 joint bearings (ankle)", "foot", 8, 11.9, "listing")
    add("608 joint bearings (knee)", "shank", 4, 11.9, "listing")
    add("608 joint bearings (hip)", "thigh", 8, 11.9, "listing")
    add("608 joint bearings (waist)", "spine", 4, 11.9, "listing")
    add("608 joint bearings (elbow)", "forearm", 4, 11.9, "listing")
    add("608 joint bearings (shoulder)", "upperarm", 8, 11.9, "listing")
    add("625 joint bearings (wrist)", "hand", 8, 4.6, "est")
    add("625 joint bearings (neck)", "neck", 4, 4.6, "est")
    add("ring-joint balls 6 mm", "upperarm", 56, 0.2, "listing")
    add("ring-joint balls 6 mm", "forearm", 32, 0.2, "listing")
    add("ring-joint balls 6 mm", "neck", 13, 0.2, "listing")
    add("ring-joint balls 8 mm", "thigh", 48, 0.4, "listing")
    add("ring-joint balls 8 mm", "spine", 32, 0.4, "listing")
    # pipe bones (D-019, docs/11): length x kg/m
    add("PVC 1\" humerus 0.20 m", "upperarm", 2, 200 * PVC_KG_M["1"], "calc")
    add("PVC 1\" forearm 0.15 m", "forearm", 2, 150 * PVC_KG_M["1"], "calc")
    add("PVC 1-1/2\" femur 0.33 m", "thigh", 2, 330 * PVC_KG_M["1-1/2"], "calc")
    add("PVC 1-1/4\" tibia 0.33 m", "shank", 2, 330 * PVC_KG_M["1-1/4"], "calc")
    add("PVC 2\" spine 0.30 m", "spine", 1, 300 * PVC_KG_M["2"], "calc")
    add("PVC 1\" neck 0.06 m", "neck", 1, 60 * PVC_KG_M["1"], "calc")
    # electronics: 5 nodes (ESP32 11, CAN 4, PCA9685 x1-2 ~9, TCA9548A x2 3, INA219 6, 5 V buck 3, DRV8833/TB6612)
    add("node electronics, arm", "upperarm", 2, 60, "est")
    add("node electronics, torso", "spine", 1, 70, "est")
    add("node electronics, leg", "thigh", 2, 50, "est")
    add("6 V 8 A buck (XL4016)", "spine", 1, 44, "listing")
    add("6 V 8 A buck (XL4016)", "upperarm", 2, 44, "listing")
    add("AS5600 + magnet (per DOF, spread)", "spine", 50, 2.7, "listing")
    add("BNO085 pelvis IMU", "pelvis", 1, 5, "est")
    add("ICM-42688 IMU (torso, feet)", "foot", 2, 5, "est")
    add("ICM-42688 IMU (torso, feet)", "spine", 1, 5, "est")
    add("FSR 402, 4 per foot", "foot", 8, 1, "est")
    add("battery 3S LiPo 10 Ah", "pelvis", 1, 543, "listing")
    add("E-stop, 200 A relay, fuse", "pelvis", 1, 196, "listing")
    add("wiring (12 and 18 AWG runs, signal), est", "spine", 1, 400, "est")
    add("M3 inserts and screws, est", "spine", 1, 300, "est")
    add("line, PTFE sheath, crimps, adjusters", "spine", 1, 250, "est")
    return rows


def stl_volume_area(path):
    with open(path, "rb") as f:
        data = f.read()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        v = np.array([list(map(float, m)) for m in re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data)])
        tri = v.reshape(-1, 3, 3)
    else:
        n = struct.unpack("<I", data[80:84])[0]
        rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
        tri = rec["v"].astype(np.float64).reshape(-1, 3, 3)
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    vol = abs(np.einsum("ij,ij->i", a, np.cross(b, c)).sum()) / 6.0
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1).sum()
    return vol, area


def build(out):
    os.makedirs(out, exist_ok=True)
    for script, extra in (("chain.py", []), ("ring.py", []), ("winch_bank.py", ["--n", "5"])):
        print("building %s ..." % script, flush=True)
        subprocess.run([BLENDER, "-b", "-P", os.path.join(ROOT, "cad", "blender", script), "--", "--out", out] + extra,
                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, cwd=ROOT)


def skeleton_masses():
    s = json.load(open(os.path.join(ROOT, "cad", "skeleton.json")))
    out = {}
    for b in s["bones"]:
        out[b["name"]] = b["mass_kg"] * 1000 * (2 if b["name"] in MIRRORED else 1)
    return out, s["mass_budget_kg"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stl", default=os.path.join(ROOT, "cad", "out"))
    ap.add_argument("--build", action="store_true")
    ap.add_argument("--infill", type=float, default=0.25)
    ap.add_argument("--wall", type=float, default=1.2, help="mm of solid walls and skins (3 lines of 0.4 mm)")
    ap.add_argument("--md", help="also write the tables as Markdown to this file")
    ap.add_argument("--json", help="write {bone: kg per side} for the model (ROBOT_MASSES=<file>, export_mjcf.py)")
    args = ap.parse_args()
    if args.build:
        build(args.stl)

    seg = {}

    def put(s, kind, g):
        seg.setdefault(s, {"printed": 0.0, "bought": 0.0, "est": 0.0})[kind] += g

    lines = []
    p = lambda s="": (lines.append(s), print(s))
    stls = sorted(f[:-4] for f in os.listdir(args.stl) if f.endswith(".stl"))
    unmatched = [n for n in stls if n not in PARTS and n not in IGNORE]
    missing = [n for n in PARTS if n not in stls]
    p("## Printed parts from the generators (infill %.0f %%, walls and skins %.1f mm, PETG)" % (100 * args.infill, args.wall))
    p()
    p("| Part | On robot | Solid g each | Printed g each | Printed g total | Segment |")
    p("| --- | --- | --- | --- | --- | --- |")
    tot_solid = tot_printed = 0.0
    for n in stls:
        if n not in PARTS:
            continue
        cnt, s = PARTS[n]
        vol, area = stl_volume_area(os.path.join(args.stl, n + ".stl"))
        shell = min(vol, area * args.wall)
        g_solid, g = RHO * vol, RHO * (shell + args.infill * (vol - shell))
        tot_solid += cnt * g_solid
        tot_printed += cnt * g
        if s is None:                                    # winch-bank parts: spread over the banks' segments
            per = cnt / sum(BANKS.values())
            for bs, nb in BANKS.items():
                put(bs, "printed", g * per * nb)
            s = "banks"
        else:
            put(s, "printed", cnt * g)
        p("| %s | %d | %.0f | %.0f | %.0f | %s |" % (n, cnt, g_solid, g, cnt * g, s))
    p()
    p("Generated: %.2f kg printed (%.2f kg if printed solid)." % (tot_printed / 1000, tot_solid / 1000))
    if unmatched:
        p("**Unmatched STLs (add them to PARTS or IGNORE):** " + ", ".join(unmatched))
    if missing:
        p("**Expected but not built:** " + ", ".join(missing))
    p()
    p("## Printed parts with no generator yet (estimates)")
    p()
    p("| Part | Segment | g |")
    p("| --- | --- | --- |")
    tot_ungen = 0.0
    for name, s, g in UNGENERATED:
        tot_ungen += g
        if s is None:
            for ws, n in WMAX_AT.items():
                put(ws, "est", g * n / 21)
            s = "with the W-max"
        else:
            put(s, "est", g)
        p("| %s | %s | %.0f |" % (name, s, g))
    p()
    p("## Bought parts")
    p()
    p("| Part | Segment | Qty | g each | g total | Source |")
    p("| --- | --- | --- | --- | --- | --- |")
    tot_bought = 0.0
    for name, s, n, g, src in bought():
        tot_bought += n * g
        put(s, "bought", n * g)
        p("| %s | %s | %d | %.1f | %.0f | %s |" % (name, s, n, g, n * g, src))
    p()
    model, budget = skeleton_masses()
    p("## Per segment against the model (cad/skeleton.json; mirrored segments are both sides)")
    p()
    p("| Segment | Printed (generated) | Printed (est) | Bought | Total kg | Model kg | Over |")
    p("| --- | --- | --- | --- | --- | --- | --- |")
    grand = 0.0
    for s in model:
        d = seg.get(s, {"printed": 0, "bought": 0, "est": 0})
        t = d["printed"] + d["est"] + d["bought"]
        grand += t
        p("| %s | %.2f | %.2f | %.2f | **%.2f** | %.2f | %+.2f |" %
          (s, d["printed"] / 1000, d["est"] / 1000, d["bought"] / 1000, t / 1000, model[s] / 1000, (t - model[s]) / 1000))
    p("| **total** | %.2f | %.2f | %.2f | **%.2f** | %.2f | %+.2f |" %
      (tot_printed / 1000, tot_ungen / 1000, tot_bought / 1000, grand / 1000, sum(model.values()) / 1000,
       grand / 1000 - sum(model.values()) / 1000))
    p()
    fil = (tot_printed + tot_ungen) * 1.15
    p("Budget %d kg; estimate %.1f kg. Filament to buy: %.1f kg (printed parts + 15 %% for supports, brims and "
      "failed prints)." % (budget, grand / 1000, fil / 1000))
    if args.json:
        per = {}
        for s in model:
            d = seg.get(s, {"printed": 0, "bought": 0, "est": 0})
            per[s] = round((d["printed"] + d["est"] + d["bought"]) / 1000 / (2 if s in MIRRORED else 1), 3)
        with open(args.json, "w") as f:
            json.dump(per, f, indent=1)
    if args.md:
        with open(args.md, "w") as f:
            f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
