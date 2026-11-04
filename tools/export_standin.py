#!/usr/bin/env python3
"""
export_standin.py - cad/skeleton.json -> sim/robot_standin.xml + sim/robot_standin.json: the fast stand-in
of the tendon humanoid, for training policies at scale (docs/17).

The tendon model of export_mjcf.py carries every drive train element (motor curve, self-lock, clutch,
drum, two elastic line runs that go slack) and runs well under real time. A policy needs hundreds of
millions of steps, so it trains on this stand-in and is judged on the tendon model. The stand-in is the
same skeleton, masses, joint ranges and printed-envelope collision shapes, with each DOF's drive train
replaced by one MuJoCo position servo that keeps what a policy can feel of it:

  * stiffness kp = the line loop's stiffness about its target with both runs taut, 2 k (n r_p)^2
    (sim_tendon.Rig.torque_to_angle); the loop is half as stiff once one run goes slack, which the
    training randomises rather than models
  * damping kv = the motor's back-EMF slope reflected to the joint, T_stall n_r^2 / w_noload, plus the
    line damping (export_mjcf.LINE_DAMPING), scaled by KV_SCALE (below)
  * the drive's cascade (P-050, docs/29): the tendon model's firmware loop is an inner gear servo and an
    outer joint loop (sim_tendon KQ, KI, TRIM_MAX, OUTER_GAIN) whose integral trim takes out the line
    stretch under load, so a held joint reaches its target. The stand-in reproduces it as a target lag
    (TIMECONST, the gear's travel) on the servo and the outer loop run by the training loop at its control
    rate from `outer_loop` in the JSON. KV_SCALE and TIMECONST were fitted to the tendon model's step
    responses standing (docs/29); the plain servo was slow and soft (a held knee reached 16 % of a 0.03 rad
    step in 1 s, the tendon model 91 %)
  * torque limit = the clutch slip torque reflected to the joint (above it the clutch slips, so no drive
    can push harder)
  * per DOF in the JSON: the line force for a joint torque, the continuous line rating, the break
    strength, the no-load speed and the stall torque, so a training loop can apply the same damage
    gates the tendon model reports (line above its continuous rating, clutch slip, joint stop)

Sensors: pelvis orientation, gyro and accelerometer (the IMU), and a touch sensor on each sole.

Run:  python tools/export_standin.py   -> sim/robot_standin.xml, sim/robot_standin.json
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import export_mjcf as em                                                    # noqa: E402
from export_urdf import expand, sub, WORLD_AXIS, FOOT_BOX, FOOT_OFFSET     # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMPACT_BODIES = ("head", "spine", "pelvis")   # touch sensors for falls: the shell parts that take impacts
KV_SCALE = 0.1           # of the back-EMF damping: the motor's damping acts on the gear, behind the line spring
TIMECONST = 0.03         # s: first-order lag of the servo's target (the gear's travel and the inner loop)
CTRL_MARGIN = 0.5        # rad beyond the joint range the servo target may go (the outer loop's trim)
JOINT_ARMATURE = 0.01     # kg m^2 at the joint: the motor's inertia sits behind the line spring in the real
                           # drive train, so the joint does not feel its full reflected value at impact


def dof_params(joint_name, axis, cls_name):
    p = em.winch_params(joint_name, axis, cls_name)
    n_rp = p["tackle"] * p["r_p"]
    ratio = n_rp / p["r_d"]                       # drum radians per joint radian
    k_both = 2 * p["k"] * n_rp ** 2               # N m / rad, both runs taut
    stall = p["t_stall"] * ratio                  # N m at the joint
    w_nl = p["w_noload"] / ratio                  # rad/s at the joint
    kv = KV_SCALE * stall / w_nl + em.LINE_DAMPING * k_both
    slip = p["t_clutch"] * ratio                  # N m at the joint where the clutch slips
    return dict(cls=cls_name, kp=k_both, kv=kv, torque_max=min(slip, stall), stall_nm=stall,
                noload_rad_s=w_nl, moment_arm_m=n_rp, pretension_n=p["pre"], line_cont_n=p["f_cont"],
                line_break_n=p["break_n"], linear_torque_nm=2 * p["pre"] * n_rp)


def standin(links, data):
    joint_defs = {j["bone"]: j for j in data["joints"] if j.get("bone")}
    by = {l["name"]: l for l in links}
    children = {}
    for l in links:
        children.setdefault(l["parent"], []).append(l)
    dofs = []

    def body(l, indent):
        pad = "  " * indent
        jd = joint_defs.get(l["base"])
        if l["parent"] is None:
            out = ['%s<body name="%s" pos="%.4f %.4f %.4f">\n' % ((pad, l["name"]) + tuple(l["head"])),
                   '%s  <freejoint name="float"/>\n' % pad,
                   '%s  <site name="imu" pos="0 0 0" size="0.01"/>\n' % pad]
        else:
            o = sub(l["head"], by[l["parent"]]["head"])
            out = ['%s<body name="%s" pos="%.4f %.4f %.4f">\n' % ((pad, l["name"]) + tuple(o))]
        for d in l["dofs"]:
            ax = WORLD_AXIS[d["axis"]]
            name = "%s_%s" % (l["name"], d["axis"])
            out.append('%s  <joint name="%s" type="hinge" axis="%d %d %d" range="%.4f %.4f" limited="true" '
                       'damping="0.05" armature="%.4f"/>\n'
                       % (pad, name, ax[0], ax[1], ax[2], math.radians(d["lo"]), math.radians(d["hi"]),
                          JOINT_ARMATURE))
            q = dof_params(jd["name"], d["axis"], jd["winch"])
            q.update(name=name, lo=math.radians(d["lo"]), hi=math.radians(d["hi"]))
            dofs.append(q)
        out.append(em.geom_shape(l, em.TINT.get(jd["winch"], "0.75 0.75 0.75 1") if jd else "0.6 0.6 0.6 1"))
        if l["base"] == "foot":
            out.append('%s  <site name="%s_sole" type="box" size="%.4f %.4f 0.006" pos="%.4f %.4f %.4f"/>\n'
                       % (pad, l["name"], FOOT_BOX[0] / 2, FOOT_BOX[1] / 2, FOOT_OFFSET[0], FOOT_OFFSET[1],
                          FOOT_OFFSET[2] - FOOT_BOX[2] / 2))
        if l["name"] in IMPACT_BODIES:
            # an impact sensor around the printed shell: a touch site a little larger than the shell
            kind, dim = em.SHELL[l["base"]]
            d = sub(l["tail"], l["head"])
            if kind == "box":
                size = tuple(max(abs(v), w) / 2 + 0.01 for v, w in zip(d, dim))
                out.append('%s  <site name="%s_impact" type="box" size="%.4f %.4f %.4f" pos="%.4f %.4f %.4f"/>\n'
                           % ((pad, l["name"]) + size + tuple(v / 2 for v in d)))
            else:
                length = math.sqrt(sum(v * v for v in d))
                out.append('%s  <site name="%s_impact" type="sphere" size="%.4f" pos="%.4f %.4f %.4f"/>\n'
                           % ((pad, l["name"], max(dim, length / 2) + 0.01) + tuple(v / 2 for v in d)))
        for c in children.get(l["name"], []):
            out.extend(body(c, indent + 1))
        out.append("%s</body>\n" % pad)
        return out

    root = [l for l in links if l["parent"] is None][0]
    walk = body(root, 2)
    # actuators (the control vector) in the tendon model's order (export_mjcf: link order), so one policy's
    # outputs mean the same joints on both models
    order = ["%s_%s" % (l["name"], d["axis"]) for l in links for d in l["dofs"]]
    dofs.sort(key=lambda d: order.index(d["name"]))
    acts = ['    <position name="%s" joint="%s" kp="%.2f" kv="%.3f" timeconst="%.3f" ctrlrange="%.4f %.4f" '
            'ctrllimited="true" forcerange="%.3f %.3f" forcelimited="true"/>\n'
            % (d["name"], d["name"], d["kp"], d["kv"], TIMECONST, d["lo"] - CTRL_MARGIN, d["hi"] + CTRL_MARGIN,
               -d["torque_max"], d["torque_max"])
            for d in dofs]
    feet = [l["name"] for l in links if l["base"] == "foot"]
    sensors = (['    <framequat name="pelvis_quat" objtype="site" objname="imu"/>\n',
                '    <gyro name="pelvis_gyro" site="imu"/>\n',
                '    <accelerometer name="pelvis_acc" site="imu"/>\n']
               + ['    <touch name="%s_touch" site="%s_sole"/>\n' % (f, f) for f in feet]
               + ['    <touch name="%s_impact" site="%s_impact"/>\n' % (b, b) for b in IMPACT_BODIES]
               + ['    <actuatorfrc name="%s_tau" actuator="%s"/>\n' % (d["name"], d["name"]) for d in dofs]
               + ['    <jointlimitfrc name="%s_stop" joint="%s"/>\n' % (d["name"], d["name"]) for d in dofs])
    out = ['<mujoco model="tendon_humanoid_standin">\n',
           '  <compiler angle="radian"/>\n',
           '  <option timestep="0.005" gravity="0 0 -9.81" integrator="implicitfast"/>\n',
           '  <default>\n    <geom friction="1.0 0.005 0.0001" rgba="0.75 0.75 0.75 1"/>\n  </default>\n',
           '  <worldbody>\n    <geom name="floor" type="plane" size="20 20 0.1" rgba="0.9 0.9 0.9 1"/>\n']
    out.extend(walk)
    out.append('  </worldbody>\n  <actuator>\n')
    out.extend(acts)
    out.append('  </actuator>\n  <sensor>\n')
    out.extend(sensors)
    out.append('  </sensor>\n</mujoco>\n')
    return "".join(out), dofs, feet


def outer_loop(dofs):
    """The tendon model's outer joint loop, for the training loop to run at its control rate: servo target =
    target + gain (kq err + trim), trim += gain ki err dt with |trim| <= trim_max, err = target - q."""
    import sim_tendon as st
    gain = [st.OUTER_GAIN.get(d["name"].split("_")[0].split(".")[0], 1.0) for d in dofs]
    return dict(kq=st.KQ, ki=st.KI, trim_max=st.TRIM_MAX, gain=gain, timeconst=TIMECONST, kv_scale=KV_SCALE)


def build():
    with open(os.path.join(ROOT, "cad", "skeleton.json")) as f:
        data = json.load(f)
    links = expand(data)
    return standin(links, data)


def main():
    xml, dofs, feet = build()
    with open(os.path.join(ROOT, "sim", "robot_standin.xml"), "w") as f:
        f.write(xml)
    spec = dict(model="sim/robot_standin.xml", source="tools/export_standin.py", timestep=0.005,
                feet=feet, imu_site="imu", outer_loop=outer_loop(dofs), impact_bodies=list(IMPACT_BODIES), dofs=dofs,
                sensors="pelvis_quat 4, pelvis_gyro 3, pelvis_acc 3, <foot>_touch per foot, <body>_impact per "
                        "impact body, <dof>_tau per DOF (actuator torque), <dof>_stop per DOF (joint-stop force)",
                notes="kp: line loop stiffness, both runs taut; kv: motor back-EMF + line damping; torque_max: "
                      "clutch slip at the joint; line force for a joint torque tau: pretension + |tau| / (2 "
                      "moment_arm) while |tau| < linear_torque, else |tau| / moment_arm")
    with open(os.path.join(ROOT, "sim", "robot_standin.json"), "w") as f:
        json.dump(spec, f, indent=1)
    print("wrote sim/robot_standin.xml and .json: %d DOF, feet %s" % (len(dofs), feet))
    if "--table" in sys.argv:
        print("%-20s %-6s %8s %8s %8s %8s %8s" % ("dof", "class", "kp", "kv", "Tmax", "w_nl", "Tline"))
        for d in dofs:
            print("%-20s %-6s %8.1f %8.2f %8.1f %8.1f %8.1f" % (d["name"], d["cls"], d["kp"], d["kv"],
                  d["torque_max"], d["noload_rad_s"], d["line_cont_n"] * d["moment_arm_m"]))


if __name__ == "__main__":
    main()
