#!/usr/bin/env python3
"""
export_mjcf.py - cad/skeleton.json -> sim/robot.xml: the tendon-actuated MuJoCo model.

Every DOF of the joint table becomes the real drive train of docs/02, element for element:

    motor ---> gear output hinge ---> friction clutch ---> drum hinge ---> line A ----> joint pulley
    (DC curve)  (self-locking,          (tendon with          (drum radius)   line B <---  (closed loop,
                 reflected inertia)      dry friction)                                     equal moment arms)

  * The motor is a MuJoCo general actuator on the gear hinge with the DC torque-speed line:
    torque = T_stall * ctrl - (T_stall / w_noload) * w, ctrl in [-1, 1] = voltage fraction. T_stall and
    w_noload are the drum-side figures of tools/tendon_calc.py (after gearbox efficiency).
  * Self-locking is a second joint equality on the gear hinge that the controller activates when the
    motor is switched off, released when the torque through it exceeds the holding torque (a worm never
    reaches it; a 298:1 spur creeps), with a dry-friction torque on the hinge for that sliding phase.
  * The drag clutch is a joint equality (gear = drum, the stuck phase) that the controller releases when
    the torque through it exceeds the slip torque, plus a fixed tendon (gear angle - drum angle) with
    frictionloss = that same torque, which carries the sliding phase; the controller re-engages the
    equality when the slip stops. Slip torque is set so the line sees one third of its break strength
    (docs/02: "slips before anything breaks"). Soft friction alone creeps under a static load, which a
    stuck clutch does not, hence the two-element model.
  * The closed loop is two fixed tendons per DOF, A and B, with lengths +-(n r_p q - r_d phi): joint
    angle q at tackle n times pulley radius r_p, drum angle phi at drum radius r_d. Each has a one-
    sided spring (dead band below its slack length) so a line can go slack and carry no force, and a
    stiffness EA / (n L_bowden) from the line's stretch; the slack length is set slightly negative so
    both lines carry the pretension at rest (barrel adjuster, D-014).
  * Hard stops are the joint ranges. Feet are boxes. Hands are rigid. Face and fingers are not here.

Run:  python tools/export_mjcf.py [--table]   -> sim/robot.xml (and the per-DOF drive-train table)
      python tools/sim_tendon.py              -> stand, push, hold tests with tendon readouts
"""
import json
import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from export_urdf import expand, sub, FOOT_BOX, FOOT_OFFSET, WORLD_AXIS   # noqa: E402
from tendon_calc import MOTORS as MOTORS_MODEL, LINES as LINES_MODEL      # noqa: E402
MOTORS, LINES = dict(MOTORS_MODEL), dict(LINES_MODEL)   # the figures in use: tendon_calc's, or a ROBOT_MOTORS case

# ------------------------------------------------------------------- winch classes (docs/01 table)
# rotor_J: motor rotor inertia (kg m^2), ballpark from rotor mass and radius; reflected through the
# gearbox it is what the drum feels. hold: holding torque at the gear output when unpowered as a
# fraction of stall (worm = effectively infinite, spur 298:1 creeps; D-021 says measure it).
CLASSES = {
    "W-mini": dict(motor="N20_298:1", eta=0.75, r_drum=0.003, line="braid_50lb",  rotor_J=1.6e-8, ratio=298, hold=0.8),
    "W-mid":  dict(motor="JGB37_1:90", eta=0.75, r_drum=0.006, line="braid_100lb", rotor_J=3.6e-6, ratio=90,  hold=0.8),
    "W-max":  dict(motor="775_PLAN40", eta=0.60, r_drum=0.015, line="braid_200lb", rotor_J=1.1e-5, ratio=40,  hold=0.8),
    # variant (TORSO_WORM): a bought self-locking worm gearmotor on the drum; hold far above stall (a worm
    # cannot be back-driven), figures at the output so eta 1.0
    "W-worm": dict(motor="WORM_10RPM", eta=1.00, r_drum=0.015, line="braid_200lb", rotor_J=2.0e-6, ratio=634, hold=3.0),
}
# variant (ROBOT_WMAX=bldc, docs/34): every W-max joint on a 5010 360 KV outrunner, FOC, printed 1:25, 24 V (docs/21).
# Back-drivable: hold 0, so its drums never lock and a standing joint holds on current. rotor_J: the outrunner's
# bell (~50 g at ~25 mm). bldc: the energy is copper loss plus shaft power, not the brushed duty x V x I.
WMAX_VARIANT = os.environ.get("ROBOT_WMAX", "")
WMAX_RATIO = float(os.environ.get("ROBOT_WMAX_RATIO", 25))   # bldc: the printed gearbox's ratio (25 in docs/34; docs/40)
if WMAX_VARIANT == "bldc":
    _r = WMAX_RATIO
    CLASSES["W-max"] = dict(motor="5010_PLAN25_24V", eta=0.60, r_drum=0.015, line="braid_200lb", rotor_J=3.0e-5,
                            ratio=_r, hold=0.0, bldc=True)
    # its servo tuned to the 775's drum stiffness (torque per rad of gear error): the gains scale with stall torque
    CLASSES["W-max"]["kg_scale"] = float(os.environ.get("ROBOT_WMAX_KG", MOTORS_MODEL["775_PLAN40"].stall_torque_nm
                                                        / (MOTORS_MODEL["5010_PLAN25_24V"].stall_torque_nm * _r / 25)))
elif WMAX_VARIANT:
    raise SystemExit("ROBOT_WMAX: bldc or unset, not %r" % WMAX_VARIANT)
# line stretch: 2 % at a quarter of break (docs/08) -> EA = break / 0.08
STRETCH_AT_QUARTER_BREAK = 0.02
PRETENSION_FRACTION = 0.25        # of the continuous line force. docs/02 said ~10 %; the simulation (docs/13)
                                  # showed the loop's stiffness halves once the unloaded side goes slack, and
                                  # 25 % keeps both sides taut under standing loads (D-021)
LINE_DAMPING = 0.02               # s: sheath friction and line hysteresis as viscous damping, b = this x k.
                                  # Unmeasured; 0.003 leaves the body swaying +-4 deg on its ankle lines for many
                                  # seconds, 0.02-0.03 settles it. Needs the implicit integrator (set below).
CLUTCH_LINE_FRACTION = 1 / 3.0    # clutch slips when the line reaches this fraction of break

# ------------------------------------------------------------------- per joint: tackle, pulley radii, Bowden run
# pulley radius per DOF axis (m) = the joint's tendon moment arm: knuckle-chain lug radius (chain.py presets:
# wrist 18, neck 25 mm since D-032; they were 12 and 10) or ring rotor radius (ring.py: forearm 28, neck_yaw 25 mm), tendon_calc moment arms
# for the W-max joints; bowden = sheath length from the bank to the joint (m); tackle = strands at the joint.
# --stiff-mini (sim_tendon.py) raises the W-mini pulleys to 25 mm and their pretension to 25 % (see docs/13).
JOINTS = {
    "waist":    dict(tackle=2, pulley={"pitch": 0.050, "roll": 0.050, "yaw": 0.050}, bowden=0.30),
    "neck":     dict(tackle=1, pulley={"pitch": 0.025, "roll": 0.025, "yaw": 0.025}, bowden=0.30,    # D-032
                     tackle_by_axis={"pitch": 2}),   # D-066: the pitch on 2 strands (the chair's trunk swing slipped it)
    "shoulder": dict(tackle=2, pulley={"pitch": 0.040, "roll": 0.040, "yaw": 0.040}, bowden=0.40),
    "elbow":    dict(tackle=2, pulley={"pitch": 0.030, "yaw": 0.028}, bowden=0.50, tackle_by_axis={"yaw": 1}),
    "wrist":    dict(tackle=1, pulley={"pitch": 0.018, "roll": 0.018}, bowden=0.30),                  # D-032
    "hip":      dict(tackle=2, pulley={"pitch": 0.060, "roll": 0.050, "yaw": 0.050}, bowden=0.30),
    "knee":     dict(tackle=2, pulley={"pitch": 0.050}, bowden=0.50),
    "ankle":    dict(tackle=3, pulley={"pitch": 0.050, "roll": 0.050}, bowden=0.60),
}


SHOULDERS_LOW = bool(os.environ.get("ROBOT_SHOULDERS_LOW"))   # variant (docs/36): the shoulder W-max drives in the
                     # pelvis bank, so their sheaths run ~0.45 m further (0.85 m) and the lines are about half as stiff
if SHOULDERS_LOW:
    JOINTS["shoulder"]["bowden"] = 0.85

STIFF_MINI = False   # variant: every W-mini pulley at least 25 mm (docs/13 finding; since D-032 the neck is at 25 mm
                     # in the baseline and the wrist at 18, so this now only raises the wrist)
TORSO_WORM = bool(os.environ.get("ROBOT_TORSO_WORM"))   # variant: spine and shoulders on W-worm (docs/20; hub
                     # record worm-gearmotor-options-20261028); an environment variable so step_sweep's children see it
MOTOR_SET = os.environ.get("ROBOT_MOTORS")   # variant (D-040, docs/24): low | nom | high of sim/motor_ranges.json, or
                     # <file.json>[:case]; every drive's stall torque, speed, current, continuous fraction and gearbox
                     # efficiency, and every line's break strength, at that end of its sourced range (low = the
                     # pessimistic end of each figure). Unset: tendon_calc's figures. Applied at import and by build().
ETA_MODEL = {k: c["eta"] for k, c in CLASSES.items()}


def motor_case(spec):
    """(MOTORS, LINES, {class: eta}) for a ROBOT_MOTORS value; tendon_calc's own figures for an empty one."""
    motors, lines, eta = dict(MOTORS_MODEL), dict(LINES_MODEL), dict(ETA_MODEL)
    if WMAX_RATIO != 25:     # the 5010 on another printed ratio: torque up and speed down by it (docs/40)
        m5 = motors["5010_PLAN25_24V"]
        motors["5010_PLAN25_24V"] = dataclasses.replace(m5, stall_torque_nm=m5.stall_torque_nm * WMAX_RATIO / 25,
                                                        noload_rpm=m5.noload_rpm * 25 / WMAX_RATIO, ratio=WMAX_RATIO)
    if not spec:
        return motors, lines, eta
    path, case = spec, "nom"
    if spec in ("low", "nom", "high"):
        path, case = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "sim",
                                  "motor_ranges.json"), spec
    elif ":" in os.path.basename(spec):
        path, case = spec.rsplit(":", 1)
    with open(path) as f:
        ranges = json.load(f)
    fields = ("stall_torque_nm", "noload_rpm", "stall_current_a", "cont_fraction", "mass_g", "kv", "r_ohm")
    for name, r in ranges["motors"].items():
        if name in motors:
            motors[name] = dataclasses.replace(motors[name], **{k: r[k][case] for k in fields if k in r})
        for cls, c in CLASSES.items():            # the gearbox efficiency of the drive this motor sits in
            if c["motor"] == name and "eta" in r:
                eta[cls] = r["eta"][case]
    for name, r in ranges.get("lines", {}).items():
        if name in lines:
            lines[name] = (lines[name][0], r["break_n"][case])
    return motors, lines, eta


def use_motor_case(spec):
    """Point this module's MOTORS, LINES and CLASSES[...]["eta"] at a ROBOT_MOTORS case."""
    global MOTORS, LINES
    MOTORS, LINES, eta = motor_case(spec)
    for cls, e in eta.items():
        CLASSES[cls]["eta"] = e


use_motor_case(MOTOR_SET)   # at import too: export_standin and wmax_options call winch_params without build()
HIP_ROLL_60 = True   # hip roll on the 60 mm pulley with tackle 3 (D-028, docs/15); False restores the 50 mm, tackle 2 of docs/14
EARS = {"L": (0.075, 0.0, 0.09), "R": (-0.075, 0.0, 0.09)}   # m in the head's frame: two microphones 0.15 m apart
                                                              # (docs/42, docs/49); forward is -y, +x the robot's left
TACKLE_OVERRIDE = {}  # "joint.axis" -> strands at the joint, over JOINTS (e.g. {"ankle.pitch": 6}; docs/27)
CHAIR_GEOMS = {"chair_seat": (0.23, 0.20, 0.30), "chair_arm.L": (0.03, 0.17, 0.10),   # half sizes, m; placed by
               "chair_arm.R": (0.03, 0.17, 0.10), "chair_back": (0.23, 0.02, 0.25)}     # sim_tendon.Rig.set_chair
BAGS = {}            # link -> kg: a carried bag hanging from that link's tail on a ball joint (docs/28)
BAG_HANDLE = 0.10     # m from the grip to the top of the bag
BAG_SIZE = (0.10, 0.25, 0.30)   # x (across the body), y (front to back), z: a grocery bag, m
BAG_DAMPING = 0.05    # N m s/rad on the ball joint: the contents settling
# free objects the hands grip and let go (docs/38): each a free body with a handle, gripped by a connect equality
# between a hand's grip site and the handle's site (off until Rig.grip turns it on), resting on stools (mocap boxes).
# They collide only with the stools and the floor (contype/conaffinity bit 2), not with the robot, as BAGS do not.
# A list of {"name", "shape": box|cylinder|sphere|capsule, "size": MuJoCo half sizes (box: x y z; cylinder and
# capsule: r h; sphere: r), "kg", "handle": m from the grip to the top}; from ROBOT_ITEMS (a JSON file or string)
# or set by a tool. Empty: the model is unchanged.
ITEMS = []
STOOLS = 2            # mocap stools for the items, placed by Rig.place_stool
STOOL_HALF = (0.20, 0.20, 0.40)   # a counter-sized top, 0.4 m square
GRIP_HANDS = ("hand.L", "hand.R")
# the nurturing helper (docs/44, D-070): two mocap "hands" the robot's hands grip by connect equalities (off until
# tools/helper.py attaches them). They do not collide and weigh nothing in the robot's mass; the helper's admittance
# law moves them (tools/helper.py). False: the model is unchanged.
HELPER = bool(os.environ.get("ROBOT_HELPER"))


def winch_params(joint_name, axis, cls_name):
    """Everything the MJCF and the controller need for one DOF's drive train."""
    c = CLASSES[cls_name]
    j = JOINTS[joint_name]
    m = MOTORS[c["motor"]]
    n = j.get("tackle_by_axis", {}).get(axis, j["tackle"])
    r_p = j["pulley"][axis]
    if HIP_ROLL_60 and joint_name == "hip" and axis == "roll":
        n, r_p = 3, 0.060
    n = TACKLE_OVERRIDE.get("%s.%s" % (joint_name, axis), n)
    pre_frac = PRETENSION_FRACTION
    if STIFF_MINI and cls_name == "W-mini":
        r_p, pre_frac = max(r_p, 0.025), 0.25
    r_d = c["r_drum"]
    dia, brk = LINES[c["line"]]
    t_stall = m.stall_torque_nm * c["eta"]                    # drum-side stall torque
    w_noload = m.noload_rpm / 60 * 2 * math.pi               # drum-side no-load speed
    f_stall = t_stall / r_d                                  # line force at stall (before the clutch)
    f_cont = f_stall * m.cont_fraction
    # What the builder sets (pretension, clutch slip) and the line's stretch come from the model's own figures
    # (tendon_calc: the spool label, the nominal motor), so a ROBOT_MOTORS case changes the parts, not the settings:
    # a weaker line then shows as a lower safety factor at the same clutch, a weaker motor as less margin over it.
    m0, brk0 = MOTORS_MODEL[c["motor"]], LINES_MODEL[c["line"]][1]
    f_cont0 = m0.stall_torque_nm * ETA_MODEL[cls_name] / r_d * m0.cont_fraction
    ea = brk0 / (4 * STRETCH_AT_QUARTER_BREAK)               # N, line axial stiffness per unit length
    k = ea / (n * j["bowden"])                                # N/m, measured along the drum-side line
    pre = pre_frac * f_cont0                                  # N on each line at rest (barrel adjuster)
    t_clutch = CLUTCH_LINE_FRACTION * brk0 * r_d             # drum torque at slip, set against the label
    return dict(cls=cls_name, motor=m, tackle=n, r_p=r_p, r_d=r_d, line=c["line"], break_n=brk,
                t_stall=t_stall, w_noload=w_noload, f_stall=f_stall, f_cont=f_cont, k=k, pre=pre,
                t_clutch=min(t_clutch, 0.95 * t_stall), t_hold=c["hold"] * t_stall,
                armature=c["rotor_J"] * c["ratio"] ** 2, i_stall=m.stall_current_a, volt=m.voltage,
                bldc=bool(c.get("bldc")), r_ohm=m.r_ohm or 0.0, eta=c["eta"], kg_scale=c.get("kg_scale", 1.0),
                # drum + clutch plate inertia: a few percent of the reflected rotor inertia keeps the
                # soft friction constraint of the clutch well conditioned (a massless drum creeps)
                drum_J=max(1e-4, 0.05 * c["rotor_J"] * c["ratio"] ** 2),
                bowden=j["bowden"])


# body shapes: radius of the printed shell per segment (capsule from bone head to tail), or a box
# (x width, y depth, z height) for the trunk segments and the hand. These are the collision shapes too,
# so the self-collision check is against the printed envelope, not a 50 mm stick. Masses are unchanged.
SPINE_RELIEF = 0.0   # m: the trunk's collision box relieved this high at its lower front edge (docs/41); 0: one box
SHELL = {"pelvis": ("box", (0.28, 0.16, 0.10)), "spine": ("box", (0.26, 0.16, 0.40)),
         "neck": ("capsule", 0.040), "head": ("capsule", 0.08), "clavicle": ("capsule", 0.02),
         "upperarm": ("capsule", 0.045), "forearm": ("capsule", 0.035), "hand": ("box", (0.08, 0.03, 0.18)),
         "thigh": ("capsule", 0.06), "shank": ("capsule", 0.05)}
TINT = {"W-worm": "0.50 0.55 0.66 1", "W-max": "0.55 0.58 0.62 1", "W-mid": "0.62 0.60 0.52 1", "W-mini": "0.66 0.62 0.60 1"}


BOX_GEOM = '      <geom type="box" size="%.4f %.4f %.4f" pos="%.4f %.4f %.4f" mass="%.3f" rgba="%s"/>\n'
CAPSULE_GEOM = '      <geom type="capsule" size="%.4f" fromto="%.4f %.4f %.4f %.4f %.4f %.4f" mass="%.3f" rgba="%s"/>\n'


def geom_shape(l, tint="0.75 0.75 0.75 1"):
    d = sub(l["tail"], l["head"])
    if l["base"] == "foot":
        size, origin = FOOT_BOX, FOOT_OFFSET
        return BOX_GEOM % (size[0] / 2, size[1] / 2, size[2] / 2, origin[0], origin[1], origin[2], l["mass"], tint)
    kind, dim = SHELL[l["base"]]
    if kind == "box":
        size = tuple(max(abs(v), w) for v, w in zip(d, dim))
        origin = tuple(v / 2 for v in d)
        if l["base"] == "spine" and SPINE_RELIEF > 0:
            # docs/41: the trunk shell's lower front edge relieved where the thighs come up seated. The mass box stays
            # as it was but collides with nothing; two weightless boxes collide: the back half at full height, the
            # front half (-y is forward) with its bottom raised by SPINE_RELIEF
            hx, hy, hz = size[0] / 2, size[1] / 2, size[2] / 2
            c = SPINE_RELIEF
            return (BOX_GEOM.replace('rgba', 'contype="0" conaffinity="0" rgba')
                    % (hx, hy, hz, origin[0], origin[1], origin[2], l["mass"], tint)
                    + BOX_GEOM.replace('rgba', 'group="3" rgba')
                    % (hx, hy / 2, hz, origin[0], origin[1] + hy / 2, origin[2], 0.0, tint)
                    + BOX_GEOM.replace('rgba', 'group="3" rgba')
                    % (hx, hy / 2, hz - c / 2, origin[0], origin[1] - hy / 2, origin[2] + c / 2, 0.0, tint))
        return BOX_GEOM % (size[0] / 2, size[1] / 2, size[2] / 2, origin[0], origin[1], origin[2], l["mass"], tint)
    # capsule: the hemispherical ends overlap the neighbours at the joint, as a printed shell does
    length = math.sqrt(sum(v * v for v in d))
    inset = min(dim, 0.45 * length)
    a = tuple(v / length * inset for v in d)
    b = tuple(v - v / length * inset for v in d)
    return CAPSULE_GEOM % ((dim,) + a + b + (l["mass"], tint))


def winch_body_xml(pad, name, p, pos):
    return ('%s  <body name="%s_gear" pos="%s">\n'
            '%s    <joint name="%s_gear" type="hinge" axis="0 0 1" armature="%.5f"/>\n'
            '%s    <geom type="cylinder" size="0.006 0.004" mass="0.002" contype="0" conaffinity="0" rgba="0.8 0.3 0.1 1"/>\n'
            '%s  </body>\n'
            '%s  <body name="%s_drum" pos="%s">\n'
            '%s    <joint name="%s_drum" type="hinge" axis="0 0 1" armature="%.6f"/>\n'
            '%s    <geom type="cylinder" size="%.4f 0.006" mass="0.002" contype="0" conaffinity="0" rgba="0.2 0.5 0.9 1"/>\n'
            '%s  </body>\n'
            % (pad, name, pos, pad, name, p["armature"], pad, pad, pad, name, pos, pad, name,
               p["drum_J"], pad, max(p["r_d"], 0.004), pad))


def item_half_z(it):
    sz = it["size"]
    return {"box": sz[2] if len(sz) > 2 else 0, "cylinder": sz[-1], "capsule": sz[-1] + sz[0], "sphere": sz[0]}[it["shape"]]


def items_xml(equalities):
    """The stools and the free items (docs/38); appends each hand-to-item grip to equalities."""
    out = []
    for k in range(STOOLS if ITEMS else 0):
        out.append('    <body name="stool.%d" mocap="true" pos="%d 5 -1">\n'
                   '      <geom name="stool.%d" type="box" size="%.3f %.3f %.3f" mass="0.001" contype="2" '
                   'conaffinity="2" rgba="0.45 0.5 0.4 1"/>\n    </body>\n' % ((k, k + 1, k) + STOOL_HALF))
    for i, it in enumerate(ITEMS):
        n, h, hz = it["name"], it.get("handle", 0.10), item_half_z(it)
        size = " ".join("%.4f" % v for v in it["size"])
        out.append('    <body name="item.%s" pos="%d 6 %.3f">\n      <freejoint name="item.%s"/>\n'
                   '      <site name="item.%s.grip" size="0.008"/>\n'
                   '      <geom type="capsule" size="0.004" fromto="0 0 0 0 0 %.4f" mass="0.01" contype="0" '
                   'conaffinity="0" rgba="0.9 0.9 0.6 1"/>\n'
                   '      <geom name="item.%s" type="%s" size="%s" pos="0 0 %.4f" mass="%.3f" contype="2" '
                   'conaffinity="2" rgba="0.85 0.7 0.45 1"/>\n    </body>\n'
                   % (n, i + 1, h + 2 * hz + 0.01, n, n, -h, n, it["shape"], size, -h - hz, it["kg"]))
        for hand in GRIP_HANDS:
            equalities.append('    <connect name="grip.%s.%s" site1="grip.%s" site2="item.%s.grip" active="false" '
                              'solref="0.01 1"/>\n' % (hand, n, hand, n))
    return out


def mjcf(links, data):
    joint_defs = {j["bone"]: j for j in data["joints"] if j.get("bone")}
    by = {l["name"]: l for l in links}
    children = {}
    for l in links:
        children.setdefault(l["parent"], []).append(l)
    # every DOF gets a winch; the winch bodies sit on the parent link (the bank is proximal)
    winches = []
    for l in links:
        jd = joint_defs.get(l["base"])
        for d in l["dofs"]:
            cls = "W-worm" if TORSO_WORM and jd["name"] in ("waist", "shoulder") else jd["winch"]
            p = winch_params(jd["name"], d["axis"], cls)
            winches.append(("%s_%s" % (l["name"], d["axis"]), p, l["parent"]))

    def body(l, indent):
        pad = "  " * indent
        if l["parent"] is None:
            out = ['%s<body name="%s" pos="%.4f %.4f %.4f">\n' % ((pad, l["name"]) + tuple(l["head"])),
                   '%s  <freejoint name="float"/>\n' % pad]
        else:
            o = sub(l["head"], by[l["parent"]]["head"])
            out = ['%s<body name="%s" pos="%.4f %.4f %.4f">\n' % ((pad, l["name"]) + tuple(o))]
        for d in l["dofs"]:
            ax = WORLD_AXIS[d["axis"]]
            out.append('%s  <joint name="%s_%s" type="hinge" axis="%d %d %d" range="%.4f %.4f" limited="true" damping="0.05"/>\n'
                       % (pad, l["name"], d["axis"], ax[0], ax[1], ax[2], math.radians(d["lo"]), math.radians(d["hi"])))
        out.append(geom_shape(l, TINT.get(jd["winch"], "0.75 0.75 0.75 1") if jd else "0.6 0.6 0.6 1"))
        k = 0
        for name, p, host in winches:
            if host == l["name"]:
                k += 1
                pos = "%.3f %.3f %.3f" % (0.06 if l["head"][0] >= 0 else -0.06, 0.02 * k, 0.0)
                out.append(winch_body_xml(pad, name, p, pos))
        for c in children.get(l["name"], []):
            out.extend(body(c, indent + 1))
        if l["name"] == "head":
            for side, e in EARS.items():
                out.append('%s  <site name="ear.%s" pos="%.4f %.4f %.4f" size="0.012" rgba="0.2 0.5 0.9 1"/>\n'
                           % ((pad, side) + e))
        if (ITEMS or HELPER) and l["name"] in GRIP_HANDS:
            g = sub(l["tail"], l["head"])
            out.append('%s  <site name="grip.%s" pos="%.4f %.4f %.4f" size="0.008"/>\n' % ((pad, l["name"]) + tuple(g)))
            if HELPER:      # the helper holds the wrist (the hand's joint), so the small wrist drive carries none of it
                out.append('%s  <site name="wrist.%s" pos="0 0 0" size="0.008"/>\n' % (pad, l["name"]))
        if HELPER and l["name"].startswith("forearm."):   # the elbow, where a helper may hold the upper arm (docs/46)
            out.append('%s  <site name="elbow.%s" pos="0 0 0" size="0.008"/>\n' % (pad, l["name"][-1]))
        if l["name"] in BAGS:
            # the bag swings from the grip at the link's tail; it does not collide (the legs pass through it)
            g = sub(l["tail"], l["head"])
            out.append('%s  <body name="bag.%s" pos="%.4f %.4f %.4f">\n' % ((pad, l["name"]) + tuple(g)))
            out.append('%s    <joint name="bag.%s" type="ball" damping="%.3f"/>\n' % (pad, l["name"], BAG_DAMPING))
            out.append('%s    <geom type="capsule" size="0.004" fromto="0 0 0 0 0 %.4f" mass="0.01" contype="0" '
                       'conaffinity="0" rgba="0.9 0.9 0.6 1"/>\n' % (pad, -BAG_HANDLE))
            out.append('%s    <geom type="box" size="%.4f %.4f %.4f" pos="0 0 %.4f" mass="%.3f" contype="0" '
                       'conaffinity="0" rgba="0.85 0.7 0.45 1"/>\n'
                       % (pad, BAG_SIZE[0] / 2, BAG_SIZE[1] / 2, BAG_SIZE[2] / 2, -BAG_HANDLE - BAG_SIZE[2] / 2,
                          BAGS[l["name"]]))
            out.append("%s  </body>\n" % pad)
        out.append("%s</body>\n" % pad)
        return out

    root = [l for l in links if l["parent"] is None][0]
    walk = body(root, 2)

    tendons, actuators, equalities = [], [], []
    for name, p, host in winches:
        # stick phase of the clutch and the self-lock as rigid couplings the controller switches
        equalities.append('    <joint name="%s_stick" joint1="%s_gear" joint2="%s_drum" polycoef="0 1 0 0 0" solref="0.005 1"/>\n'
                          '    <joint name="%s_lock" joint1="%s_gear" active="false" polycoef="0 0 0 0 0" solref="0.005 1"/>\n'
                          % (name, name, name, name, name))
        n_rp, r_d = p["tackle"] * p["r_p"], p["r_d"]
        slack = -p["pre"] / p["k"]
        tendons.append('    <fixed name="%s_clutch" frictionloss="%.4f" solreffriction="0.01 1" '
                       'solimpfriction="0.99 0.999 0.001">\n'
                       '      <joint joint="%s_gear" coef="1"/><joint joint="%s_drum" coef="-1"/>\n    </fixed>\n'
                       % (name, p["t_clutch"], name, name))
        for side, sgn in (("A", 1), ("B", -1)):
            tendons.append('    <fixed name="%s_line%s" stiffness="%.1f" springlength="-100 %.6f" damping="%.2f">\n'
                           '      <joint joint="%s" coef="%.5f"/><joint joint="%s_drum" coef="%.5f"/>\n    </fixed>\n'
                           % (name, side, p["k"], slack, LINE_DAMPING * p["k"], name, sgn * n_rp, name, -sgn * r_d))
        actuators.append('    <general name="%s_motor" joint="%s_gear" ctrllimited="true" ctrlrange="-1 1" biastype="affine" '
                         'gainprm="%.4f" biasprm="0 0 %.6f"/>\n'
                         % (name, name, p["t_stall"], -p["t_stall"] / p["w_noload"]))

    out = ['<mujoco model="tendon_humanoid">\n',
           '  <compiler angle="radian"/>\n',
           '  <option timestep="0.001" gravity="0 0 -9.81" integrator="implicitfast"/>\n',
           '  <visual><global offwidth="1280" offheight="960"/></visual>\n',
           '  <default>\n    <geom friction="1.0 0.005 0.0001" rgba="0.75 0.75 0.75 1"/>\n  </default>\n',
           '  <worldbody>\n    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.9 0.9 0.9 1"/>\n']
    out.extend(walk)
    # a chair (docs/33): seat, armrests and back as mocap bodies (placed at run time; a geom moved at run time is
    # missed by the collision tree), out of the way and not colliding until a scene places them
    # (sim_tendon.Rig.set_chair). After the robot, so its body ids stay; near-massless
    for name, half in CHAIR_GEOMS.items():
        out.append('    <body name="%s" mocap="true" pos="0 5 -1">\n'
                   '      <geom name="%s" type="box" size="%.3f %.3f %.3f" mass="0.001" contype="0" conaffinity="0" '
                   'rgba="0.55 0.4 0.25 0"/>\n    </body>\n' % ((name, name) + half))
    out.extend(items_xml(equalities))
    if HELPER:
        for hand in GRIP_HANDS:
            side = hand[-1]
            out.append('    <body name="helper.%s" mocap="true" pos="%d 5 -1">\n'
                       '      <site name="helper.%s.grip" size="0.008"/>\n'
                       '      <geom type="sphere" size="0.035" contype="0" conaffinity="0" mass="0.001" '
                       'rgba="0.95 0.55 0.2 0.9"/>\n'
                       '      <geom type="capsule" size="0.025" fromto="0 0 0 0 -0.25 -0.05" contype="0" conaffinity="0" '
                       'mass="0.001" rgba="0.95 0.55 0.2 0.6"/>\n    </body>\n' % (side, 3 if side == "L" else 4, side))
            equalities.append('    <connect name="helper.%s" site1="wrist.%s" site2="helper.%s.grip" active="false" '
                              'solref="0.02 1"/>\n' % (side, hand, side))
            equalities.append('    <connect name="helper.%s.elbow" site1="elbow.%s" site2="helper.%s.grip" '
                              'active="false" solref="0.02 1"/>\n' % (side, side, side))
    out.append('  </worldbody>\n  <tendon>\n')
    out.extend(tendons)
    out.append('  </tendon>\n  <equality>\n')
    out.extend(equalities)
    out.append('  </equality>\n  <actuator>\n')
    out.extend(actuators)
    out.append('  </actuator>\n</mujoco>\n')
    return "".join(out), winches


if os.environ.get("ROBOT_ITEMS"):                 # variant (docs/38): free items to grip, a JSON file or string
    _it = os.environ["ROBOT_ITEMS"]
    ITEMS = json.load(open(_it)) if os.path.exists(_it) else json.loads(_it)
MASSES = os.environ.get("ROBOT_MASSES")   # variant: a JSON file {bone: kg per side} (tools/mass_rollup.py --json)
                                          # replacing skeleton.json's mass_kg; children of step_sweep inherit it


def build():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    use_motor_case(MOTOR_SET)
    with open(os.path.join(root, "cad", "skeleton.json")) as f:
        data = json.load(f)
    if MASSES:
        with open(MASSES) as f:
            masses = json.load(f)
        for b in data["bones"]:
            b["mass_kg"] = masses.get(b["name"], b["mass_kg"])
    links = expand(data)
    xml, winches = mjcf(links, data)
    return xml, winches, links


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    xml, winches, links = build()
    path = os.path.join(root, "sim", "robot.xml")
    with open(path, "w") as f:
        f.write(xml)
    print("wrote %s: %d links, %d DOF, %d winches (%d tendons, %d motors)"
          % (path, len(links), len(winches), len(winches), 3 * len(winches), len(winches)))
    if "--table" in sys.argv:
        print("%-22s %-6s n  r_p(mm) r_d(mm)  Tstall(Nm) Fstall(N) Fcont(N) break(N) k(kN/m) pre(N) Tclutch(Nm) Fslip(N)" % ("dof", "class"))
        for name, p, host in winches:
            print("%-22s %-6s %d  %5.0f   %5.1f   %8.3f  %8.0f  %7.0f  %6.0f  %6.1f  %5.1f  %8.3f  %7.0f"
                  % (name, p["cls"], p["tackle"], p["r_p"] * 1e3, p["r_d"] * 1e3, p["t_stall"], p["f_stall"],
                     p["f_cont"], p["break_n"], p["k"] / 1e3, p["pre"], p["t_clutch"], p["t_clutch"] / p["r_d"]))


if __name__ == "__main__":
    main()
