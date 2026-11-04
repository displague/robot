"""Can the robot rise slowly on one leg? (docs/32)

For poses of the front leg (hip, knee, ankle) and the spine, with the body pitched so the front foot is flat and the
whole-body CoM over its sole, the front knee's, hip's and ankle's gravity torques against their drives' limits (the
stand-in's torque_max: the clutch slip at the joint). Prints, for each ankle dorsiflexion limit, the lowest pelvis
height reachable that way: by geometry alone, within the knee and hip drives, and within the ankle drive too. Below it a rise needs momentum, the other leg or the hands.

    python tools/rise_static.py                 # the stand-in as exported (sim/robot_standin.xml, .json)
    python tools/rise_static.py dorsi=50        # also try ankles that dorsiflex further than the joint allows
"""
import itertools
import json
import math
import os
import sys

import mujoco
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FRONT = ("thigh.L", "shank.L", "foot.L")
sys.path.insert(0, os.path.join(ROOT, "tools"))
from export_urdf import FOOT_OFFSET  # noqa: E402

SOLE_AHEAD = -FOOT_OFFSET[1]   # m: the sole's centre ahead of the ankle (0.08, the model's own)
SOLE_HALF = 0.04           # m: how far the CoM may sit from it


def main():
    dorsi = 20
    for a in sys.argv[1:]:
        k, v = a.split("=", 1)
        if k == "dorsi":
            dorsi = int(v)
    m = mujoco.MjModel.from_xml_path(os.path.join(ROOT, "sim", "robot_standin.xml"))
    spec = json.load(open(os.path.join(ROOT, "sim", "robot_standin.json")))
    tmax = {dd["name"]: dd["torque_max"] for dd in spec["dofs"]}
    d = mujoco.MjData(m)
    qa = {dd["name"]: m.jnt_qposadr[m.joint(dd["name"]).id] for dd in spec["dofs"]}
    bid = lambda b: m.body(b).id
    R = math.radians
    g = 9.81
    best, geo, ank = {}, {}, {}
    for th, kn, an, sp in itertools.product(range(-100, 1, 5), range(0, 121, 5), range(-dorsi, 41, 5), (-20, 0, 20, 40)):
        d.qpos[:] = 0
        d.qpos[3] = 1.0
        d.qpos[qa["thigh.L_pitch"]], d.qpos[qa["shank.L_pitch"]] = R(th), R(kn)
        d.qpos[qa["foot.L_pitch"]], d.qpos[qa["spine_pitch"]] = R(an), R(sp)
        d.qpos[qa["thigh.R_pitch"]], d.qpos[qa["shank.R_pitch"]] = R(10), R(100)   # the back leg folded, unloaded
        mujoco.mj_kinematics(m, d)
        fz = d.xmat[bid("foot.L")].reshape(3, 3)
        beta = math.atan2(fz[1, 2], fz[2, 2])
        level = False
        for sgn in (-1, 1):                       # pitch the body so the front foot is flat
            d.qpos[3:7] = (math.cos(sgn * beta / 2), math.sin(sgn * beta / 2), 0, 0)
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            fz = d.xmat[bid("foot.L")].reshape(3, 3)
            if abs(math.atan2(fz[1, 2], fz[2, 2])) < R(2):
                level = True
                break
        if not level or d.xmat[bid("pelvis")].reshape(3, 3)[2, 2] < 0.3:
            continue
        foot = d.xpos[bid("foot.L")]
        if abs(d.subtree_com[0][1] - (foot[1] - SOLE_AHEAD)) > SOLE_HALF:
            continue

        def ratio(body, dof):
            sub = [b for b in range(1, m.nbody)
                   if not any(m.body(b).name.startswith(f) for f in FRONT[FRONT.index(body):])]
            mm = sum(m.body_mass[b] for b in sub)
            cy = sum(m.body_mass[b] * d.xipos[b][1] for b in sub) / mm
            return mm * g * abs(cy - d.xpos[bid(body)][1]) / tmax[dof]

        h = round((d.xpos[bid("pelvis")][2] - foot[2] + 0.08) / 0.05) * 0.05
        lim = max(0, -an)
        geo[lim] = min(geo.get(lim, 9.0), h)
        r = max(ratio("shank.L", "shank.L_pitch"), ratio("thigh.L", "thigh.L_pitch"))
        if r < 1.0:
            best[lim] = min(best.get(lim, 9.0), h)
            if ratio("foot.L", "foot.L_pitch") < 1.0:
                ank[lim] = min(ank.get(lim, 9.0), h)

    def low(tab, lim):
        v = [x for k, x in tab.items() if k <= lim]
        return "%.2f m" % min(v) if v else "none"
    print("lowest pelvis height for a slow one-leg rise (CoM within %.0f cm of the sole centre, %.0f cm ahead of the ankle)"
          % (SOLE_HALF * 100, SOLE_AHEAD * 100))
    print("ankle dorsiflexion | geometry alone | within the knee and hip drives | and the ankle drive")
    for lim in sorted(geo):
        print("%2d deg             | %s         | %s                         | %s" % (lim, low(geo, lim), low(best, lim), low(ank, lim)))


if __name__ == "__main__":
    main()
