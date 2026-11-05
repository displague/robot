"""Can the robot leave a chair slowly? (docs/33)

For symmetric poses of the legs (hip, knee), the spine and the arms, with the body pitched so both feet are flat on
the floor, the seat height at which the thighs just touch the seat (their lowest point) and whether the whole-body CoM
is over the soles there (seat-off without momentum). For each such pose the knee's, hip's and ankle's gravity torques
(each leg carrying half) and the spine's (the upper body leaned over it) against their drives' limits (the
stand-in's torque_max: the clutch slip at the joint). The spine is the one that binds: arms held forward move the
CoM forward but load it past its clutch (sim_tendon's sit scenes found it sagging from 40 to 7 degrees).

With armrests the hands push down on rests beside the hips: a vertical force F (both hands) at the hands moves the
centre of the load the feet carry forward, so a CoM behind the soles can still leave the seat. The arms' limit is
the shoulder and elbow drives holding F.

Prints per seat height the best pose without and with armrests: the knee, hip, ankle and spine torque ratios (1.0 is
the clutch) and the hand force needed; `spine=0.9` keeps only poses whose spine torque is under that share.

    python tools/sit_static.py                      # seat heights 0.35, 0.40, 0.45, 0.50 m
    python tools/sit_static.py seats=0.30,0.40 dorsi=30 margin=0.04 spine=0.9
    python tools/sit_static.py hands [armh=0.20]    # can the hands reach the armrests, and how hard can they push?

The armrest rows assume the hands reach the rests and push what the balance needs. `hands` checks that on the full
tendon model: for seated and leaning poses at each seat height, every shoulder, elbow and wrist pitch that puts the
hand's tip on the rest top (sim_tendon's chair: ARM_H above the seat, from CHAIR_FRONT + 0.06 m back), and the
largest push per hand within 80 % of the shoulder's, elbow's and wrist's clutches.
"""
import itertools
import json
import math
import os
import sys

import mujoco

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
from export_urdf import FOOT_BOX, FOOT_OFFSET  # noqa: E402
from export_mjcf import SHELL  # noqa: E402

SOLE_AHEAD = -FOOT_OFFSET[1]   # m: the sole's centre ahead of the ankle
SOLE_MARGIN = 0.09             # m: the CoM (or the feet's load centre) within this of the sole centre, fore and aft
                               # (0.09: 4 cm ahead of the heel; the sole is 0.13 m from its centre to the heel)
SPINE_MAX = 9.0                # the most of the spine's clutch a pose may load it to (9: no limit)
THIGH_R = SHELL["thigh"][1]    # m: the thigh shell's radius (the part that rests on the seat)
THIGH_SLOPE = 0.43 * math.sin(math.radians(25))   # m: the knee at most this far below the hip (a seat edge)
FLOOR_BELOW_FOOT = -FOOT_OFFSET[2] + FOOT_BOX[2] / 2   # m: the sole's underside below the foot body's origin


def hands(seats, armh):
    import numpy as np
    import sim_tendon as st
    r = st.Rig()
    m = r.model
    dd = mujoco.MjData(m)
    D = math.radians
    clutch = {n: r.by_name[n]["p"]["t_clutch"] * r.by_name[n]["p"]["tackle"] * r.by_name[n]["p"]["r_p"]
              / r.by_name[n]["p"]["r_d"] for n in ("upperarm.L_pitch", "forearm.L_pitch", "hand.L_pitch")}
    print("clutch at the joint, N m: shoulder %.1f, elbow %.1f, wrist %.1f"
          % tuple(clutch[k] for k in ("upperarm.L_pitch", "forearm.L_pitch", "hand.L_pitch")))
    hand_len = 0.18                              # export_mjcf.SHELL["hand"]: the hand's length
    for seat in seats:
        best = None
        for th, kn, sp in itertools.product((-90, -100, -110), range(90, 126, 5), (0, 20)):
            pose = st._sym_pose(r, D(th), D(kn), D(-20), D(sp), 0.0)
            off, hz = st._pose_com(r, dd, pose)
            if abs(hz - 0.06 - seat) > 0.025:
                continue                         # not sitting on this seat
            rest_top = hz - 0.06 + armh
            for ua, fa, ha in itertools.product(range(-90, 46, 5), range(-140, 1, 5), range(-70, 61, 10)):
                for s_ in "LR":
                    dd.qpos[r.by_name["upperarm.%s_pitch" % s_]["q"]] = D(ua)
                    dd.qpos[r.by_name["forearm.%s_pitch" % s_]["q"]] = D(fa)
                    dd.qpos[r.by_name["hand.%s_pitch" % s_]["q"]] = D(ha)
                mujoco.mj_kinematics(m, dd)
                hb, fb = m.body("hand.L").id, r.feet[0]
                tip = dd.xpos[hb] + dd.xmat[hb].reshape(3, 3) @ np.array((0, 0, -hand_len))
                sole = dd.xpos[fb] + dd.xmat[fb].reshape(3, 3) @ np.array(st.SOLE_CENTRE)
                fz, ty = sole[2] - 0.02, tip[1] - dd.xpos[fb][1]
                if not (abs(tip[2] - fz - rest_top) < 0.015 and st.CHAIR_FRONT + 0.06 <= ty <= st.CHAIR_FRONT + 0.40):
                    continue
                if min(dd.xpos[m.body(b).id][2] - fz for b in ("forearm.L", "hand.L")) < rest_top - 0.01:
                    continue                     # the arm itself below the rest top: it would be through the rest
                F = min(0.8 * clutch[j] / max(abs(tip[1] - dd.xanchor[m.joint(j).id][1]), 1e-3) for j in clutch)
                if best is None or F > best[0]:
                    best = (F, th, kn, sp, ua, fa, ha, ty, off)
        if best:
            print("seat %.2f m: at most %.0f N per hand (hip %d knee %d spine %d | shoulder %d elbow %d wrist %d deg), "
                  "the hand %.2f m behind the ankle, the CoM %.1f cm behind the sole centre" % ((seat,) + best[:1] + best[1:7]
                  + (best[7], best[8] * 100)))
        else:
            print("seat %.2f m: the hands cannot reach the rest top" % seat)


def main():
    seats, dorsi = (0.35, 0.40, 0.45, 0.50), 20
    if sys.argv[1:2] == ["hands"]:
        armh = float(dict(a.split("=", 1) for a in sys.argv[2:]).get("armh", 0.20))
        hands(seats, armh)
        return
    for a in sys.argv[1:]:
        k, v = a.split("=", 1)
        if k == "seats":
            seats = tuple(float(x) for x in v.split(","))
        elif k == "dorsi":
            dorsi = int(v)
        elif k == "margin":
            global SOLE_MARGIN
            SOLE_MARGIN = float(v)
        elif k == "spine":
            global SPINE_MAX
            SPINE_MAX = float(v)
    m = mujoco.MjModel.from_xml_path(os.path.join(ROOT, "sim", "robot_standin.xml"))
    spec = json.load(open(os.path.join(ROOT, "sim", "robot_standin.json")))
    tmax = {dd["name"]: dd["torque_max"] for dd in spec["dofs"]}
    d = mujoco.MjData(m)
    qa = {dd["name"]: m.jnt_qposadr[m.joint(dd["name"]).id] for dd in spec["dofs"]}
    bid = lambda b: m.body(b).id
    R, g = math.radians, 9.81
    M = float(sum(m.body_mass))

    def is_under(b, anc):
        while b:
            if b == anc:
                return True
            b = m.body_parentid[b]
        return False
    best = {}                                  # (seat, arms) -> (score, row)
    closest = {}                               # seat -> least CoM offset behind the sole centre found
    for th, kn, sp, arm in itertools.product(range(-110, -29, 5), range(40, 136, 5), range(0, 41, 10), (0, -45, -90)):
        d.qpos[:] = 0
        d.qpos[3] = 1.0
        for s in "LR":
            d.qpos[qa["thigh.%s_pitch" % s]], d.qpos[qa["shank.%s_pitch" % s]] = R(th), R(kn)
            d.qpos[qa["upperarm.%s_pitch" % s]] = R(arm)
        d.qpos[qa["spine_pitch"]] = R(sp)
        # the body pitch that keeps the shank-to-floor angle such that the ankle can make the sole flat: the sole
        # follows the shank; the ankle angle then is the body pitch's complement
        for pitch in range(-60, 81, 1):          # body pitch about x (+ tips the pelvis forward, toward -y)
            q = R(pitch)
            d.qpos[3:7] = (math.cos(q / 2), math.sin(q / 2), 0, 0)
            # ankle angle that levels the foot: foot pitch in the world = body pitch + thigh + knee + ankle (all about x)
            an = -(q + R(th) + R(kn))
            if not (R(-dorsi) - 1e-9 <= an <= R(40)):
                continue
            for s in "LR":
                d.qpos[qa["foot.%s_pitch" % s]] = an
            mujoco.mj_kinematics(m, d)
            fz = d.xmat[bid("foot.L")].reshape(3, 3)
            if abs(math.atan2(fz[1, 2], fz[2, 2])) > R(1.5):
                continue                         # the sole not flat (the sign convention check)
            mujoco.mj_comPos(m, d)
            foot = d.xpos[bid("foot.L")]
            floor = foot[2] - FLOOR_BELOW_FOOT
            hip, knee = d.xpos[bid("thigh.L")], d.xpos[bid("shank.L")]
            # the buttock, under the hip, rests on the seat (a knee lower than the hip: the thigh slopes down past the
            # seat's edge; a knee higher: the thigh slopes up off the seat)
            seat = hip[2] - THIGH_R - floor
            if knee[2] < hip[2] - THIGH_SLOPE or d.xmat[bid("spine")].reshape(3, 3)[2, 2] < 0.3:
                continue                         # thighs sloping down more than a seat edge allows
            near = [s for s in seats if abs(seat - s) <= 0.025]
            if not near:
                continue
            h = near[0]
            sole = foot[1] - SOLE_AHEAD          # forward is -y
            com = d.subtree_com[0][1]
            off = com - sole                     # + : the CoM behind the sole centre
            closest[h] = min(closest.get(h, 9.0), off)
            # armrests: the hands at the forearm's tail; a vertical push F at y_hand carries the moment the feet do not
            hand = d.xpos[bid("hand.L")]
            need = 0.0
            if off > SOLE_MARGIN:
                if hand[1] - sole <= off:        # the hands must be further back than the CoM to help
                    need = None
                else:                            # the feet's load centre (sole + margin) from F: M g off = F (y_h - y_s) + (Mg - F) margin
                    need = M * g * (off - SOLE_MARGIN) / (hand[1] - sole - SOLE_MARGIN)
            elif off < -SOLE_MARGIN:
                continue                         # CoM ahead of the soles: tips forward
            for arms in (False, True):
                if not arms and need != 0.0:
                    continue
                if arms and need is None:
                    continue
                F = need if arms else 0.0
                if F > 0.6 * M * g:
                    continue
                Wf = M * g - F                   # what the feet carry; each leg half

                def tq(body):
                    # gravity torque at `body`'s joint from everything above it (the far side of the leg), less the hands'
                    sub = [b for b in range(1, m.nbody) if not m.body(b).name.startswith(("thigh.L", "shank.L", "foot.L")
                           [("thigh.L", "shank.L", "foot.L").index(body):])]
                    mm = sum(m.body_mass[b] for b in sub)
                    cy = sum(m.body_mass[b] * d.xipos[b][1] for b in sub) / mm
                    # by symmetry each leg carries half of the rest of the body, and the hands half of F each side
                    return abs(0.5 * (mm - m.body_mass[bid("foot.R")]) * g * (cy - d.xpos[bid(body)][1])
                               - 0.5 * F * (hand[1] - d.xpos[bid(body)][1]))
                r_knee = tq("shank.L") / tmax["shank.L_pitch"]
                r_hip = tq("thigh.L") / tmax["thigh.L_pitch"]
                r_ank = 0.5 * Wf * abs(sole - foot[1] + (off if not arms else 0.0)) / tmax["foot.L_pitch"]
                # the arms: F/2 per hand pushing down; elbow and shoulder torques from the hand's lever
                ua, fa = d.xpos[bid("upperarm.L")], d.xpos[bid("forearm.L")]
                r_sh = 0.5 * F * abs(hand[1] - ua[1]) / tmax["upperarm.L_pitch"]
                r_el = 0.5 * F * abs(hand[1] - fa[1]) / tmax["forearm.L_pitch"]
                # the spine: everything above it, leaned over its joint
                up = [b for b in range(1, m.nbody) if is_under(b, bid("spine"))]
                mu = sum(m.body_mass[b] for b in up)
                cu = sum(m.body_mass[b] * d.xipos[b][1] for b in up) / mu
                r_sp = mu * g * abs(cu - d.xanchor[m.joint("spine_pitch").id][1]) / tmax["spine_pitch"]
                if r_sp > SPINE_MAX:
                    continue
                score = max(r_knee, r_hip, r_ank, r_sh, r_el, r_sp)
                row = dict(th=th, kn=kn, sp=sp, arm=arm, an=round(math.degrees(an)), pitch=pitch, off=off, F=F,
                           knee=r_knee, hip=r_hip, ank=r_ank, sh=r_sh, el=r_el, spn=r_sp,
                           pz=d.xpos[bid("pelvis")][2] - floor)
                key = (h, arms)
                if key not in best or score < best[key][0]:
                    best[key] = (score, row)
    print("seat-off without momentum: the best pose per seat height (torque / the drive's clutch limit; 1.0 slips)")
    print("seat m | armrests | hip  knee ankle spine arms deg | CoM behind sole cm | hands N | knee  hip  ankle spine shoulder elbow | max")
    for h in seats:
        for arms in (False, True):
            if (h, arms) not in best:
                print("%.2f   | %-8s | no pose (CoM at best %.0f cm behind the sole centre)"
                      % (h, "yes" if arms else "no", closest.get(h, float("nan")) * 100))
                continue
            s, r = best[(h, arms)]
            print("%.2f   | %-8s | %4d %4d %5d %5d %4d     | %5.1f              | %6.0f  | %.2f  %.2f  %.2f  %.2f  %.2f     %.2f  | %.2f"
                  % (h, "yes" if arms else "no", r["th"], r["kn"], r["an"], r["sp"], r["arm"], r["off"] * 100, r["F"],
                     r["knee"], r["hip"], r["ank"], r["spn"], r["sh"], r["el"], s))


if __name__ == "__main__":
    main()
