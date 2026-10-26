"""
skeleton_rig.py - build the whole-body armature from cad/skeleton.json.

- One Blender bone per body segment (left side mirrored to the right).
- One Limit Rotation constraint per jointed bone, with the limits from the JSON mapped from the
  semantic world axes (pitch = world X, roll = world Y, yaw = world Z) onto the bone's local axes.
- A simple stick mesh skinned to the armature so poses show up in headless renders.
- Prints the DOF table and the per-winch-class tally (must match docs/01-architecture.md).
- Renders rest pose, all-joints-at-max and all-joints-at-min, and saves skeleton.blend.

Run headless:
    blender -b -P cad/blender/skeleton_rig.py -- --json cad/skeleton.json --out cad/out
Open cad/out/skeleton.blend afterwards to pose it by hand; the constraints clamp every joint.
"""
import json
import math
import os
import sys

import bmesh
import bpy
from mathutils import Vector

WORLD = {"pitch": Vector((1, 0, 0)), "roll": Vector((0, 1, 0)), "yaw": Vector((0, 0, 1))}
SIDES_MIRROR = (("L", 1), ("R", -1))


def bone_name(base, suffix):
    return base + ("." + suffix if suffix else "")


def build(data):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    arm = bpy.data.armatures.new("robot")
    obj = bpy.data.objects.new("robot", arm)
    bpy.context.scene.collection.objects.link(obj)
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    defs = {b["name"]: b for b in data["bones"]}

    bpy.ops.object.mode_set(mode="EDIT")
    for b in data["bones"]:
        for suffix, s in (SIDES_MIRROR if b["mirror"] else (("", 1),)):
            eb = arm.edit_bones.new(bone_name(b["name"], suffix))
            eb.head = Vector((b["head"][0] * s, b["head"][1], b["head"][2]))
            eb.tail = Vector((b["tail"][0] * s, b["tail"][1], b["tail"][2]))
            eb.roll = 0.0
            if b["parent"]:
                psuf = suffix if defs[b["parent"]]["mirror"] else ""
                eb.parent = arm.edit_bones[bone_name(b["parent"], psuf)]
    bpy.ops.object.mode_set(mode="POSE")

    limits = {}  # pose bone name -> {local axis index: (lo, hi) radians}
    for j in data["joints"]:
        if not j.get("bone"):
            continue
        for suffix, s in (SIDES_MIRROR if j["mirror"] else (("", 1),)):
            pb = obj.pose.bones[bone_name(j["bone"], suffix)]
            pb.rotation_mode = "XYZ"
            m = pb.bone.matrix_local.to_3x3()
            local_axes = [m.col[i].normalized() for i in range(3)]
            lim = {}
            for d in j["dof"]:
                if d["axis"] not in WORLD:
                    continue
                lo, hi = math.radians(d["min"]), math.radians(d["max"])
                if s < 0 and d["axis"] in ("roll", "yaw"):   # mirrored side
                    lo, hi = -hi, -lo
                w = WORLD[d["axis"]]
                dots = [la.dot(w) for la in local_axes]
                i = max(range(3), key=lambda k: abs(dots[k]))
                if dots[i] < 0:
                    lo, hi = -hi, -lo
                lim[i] = (lo, hi)
            c = pb.constraints.new("LIMIT_ROTATION")
            c.owner_space = "LOCAL"
            for i, flag in enumerate(("use_limit_x", "use_limit_y", "use_limit_z")):
                setattr(c, flag, True)
                lo, hi = lim.get(i, (0.0, 0.0))
                setattr(c, "min_" + "xyz"[i], lo)
                setattr(c, "max_" + "xyz"[i], hi)
            limits[pb.name] = lim
    bpy.ops.object.mode_set(mode="OBJECT")
    return obj, limits


def skin(arm_obj, thickness=0.035):
    """One box per bone, rigidly weighted to that bone, so poses render in the Workbench engine."""
    bm = bmesh.new()
    deform = bm.verts.layers.deform.new()
    me = bpy.data.meshes.new("robot_skin")
    skin_obj = bpy.data.objects.new("robot_skin", me)
    bpy.context.scene.collection.objects.link(skin_obj)
    for b in arm_obj.data.bones:
        vg = skin_obj.vertex_groups.new(name=b.name)
        t = thickness / 2
        corners = [Vector((x, y, z)) for x in (-t, t) for y in (0.0, b.length) for z in (-t, t)]
        verts = [bm.verts.new(b.matrix_local @ c) for c in corners]
        for v in verts:
            v[deform][vg.index] = 1.0
        faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
        for f in faces:
            bm.faces.new([verts[i] for i in f])
    bm.to_mesh(me)
    bm.free()
    mod = skin_obj.modifiers.new("arm", "ARMATURE")
    mod.object = arm_obj
    skin_obj.parent = arm_obj
    return skin_obj


def pose_extreme(arm_obj, limits, which):
    for name, lim in limits.items():
        pb = arm_obj.pose.bones[name]
        rot = [0.0, 0.0, 0.0]
        for i, (lo, hi) in lim.items():
            rot[i] = hi if which == "max" else lo
        pb.rotation_euler = rot
    for pb in arm_obj.pose.bones:
        if pb.name not in limits:
            pb.rotation_euler = (0, 0, 0)


def pose_named(arm_obj, limits, values):
    """values: {bone name: {"pitch"/"roll"/"yaw": degrees}} mapped onto local axes like the limits."""
    pose_extreme(arm_obj, {}, "max")  # reset
    for name, axes in values.items():
        pb = arm_obj.pose.bones[name]
        m = pb.bone.matrix_local.to_3x3()
        local_axes = [m.col[i].normalized() for i in range(3)]
        rot = [0.0, 0.0, 0.0]
        for ax, deg in axes.items():
            w = WORLD[ax]
            dots = [la.dot(w) for la in local_axes]
            i = max(range(3), key=lambda k: abs(dots[k]))
            rot[i] = math.radians(deg) * (1 if dots[i] > 0 else -1)
        pb.rotation_euler = rot


# Sign-convention check pose (see cad/skeleton.json "_axes"): left arm straight forward, right arm out to
# the side with the elbow bent, left leg lifted forward with the knee bent as if climbing a step,
# right ankle plantarflexed, head turned left. The constraints clamp anything out of range.
DEMO_POSE = {
    "upperarm.L": {"pitch": -90},
    "upperarm.R": {"roll": 90},
    "forearm.R": {"pitch": -90},
    "thigh.L": {"pitch": -90},
    "shank.L": {"pitch": 90},
    "foot.R": {"pitch": 30},
    "neck": {"yaw": 60},
}


def setup_render(out_png):
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.display.shading.light = "STUDIO"
    scene.display.shading.color_type = "SINGLE"
    scene.display.shading.single_color = (0.75, 0.55, 0.3)
    scene.render.resolution_x, scene.render.resolution_y = 900, 1200
    scene.world = bpy.data.worlds.new("w")
    scene.world.color = (1, 1, 1)
    cam_data = bpy.data.cameras.new("cam")
    cam = bpy.data.objects.new("cam", cam_data)
    scene.collection.objects.link(cam)
    scene.camera = cam
    target = Vector((0, 0, 0.95))
    cam.location = Vector((1.8, -3.2, 1.5))
    cam.rotation_euler = (target - cam.location).to_track_quat("-Z", "Y").to_euler()
    cam_data.lens = 60
    scene.render.filepath = out_png


def report(data):
    print("\nDOF table (ours vs human):")
    tally, total = {}, 0
    for j in data["joints"]:
        n = 2 if j["mirror"] else 1
        for d in j["dof"]:
            cls = d.get("winch", j["winch"])
            tally[cls] = tally.get(cls, 0) + n
            total += n
            human = d.get("human")
            hs = (" human [%d, %d]" % tuple(human)) if human else ""
            print("  %-9s %-16s [%5d, %5d]%s  x%d  %s" % (j["name"], d["name"], d["min"], d["max"], hs, n, cls))
    print("actuated DOF: %d   by class: %s" % (total, ", ".join("%s %d" % kv for kv in sorted(tally.items()))))
    return total, tally


def main(json_path, out_dir):
    with open(json_path) as f:
        data = json.load(f)
    os.makedirs(out_dir, exist_ok=True)
    arm_obj, limits = build(data)
    skin(arm_obj)
    report(data)
    setup_render(os.path.join(out_dir, "skeleton_rest.png"))
    bpy.ops.render.render(write_still=True)
    for which in ("max", "min"):
        pose_extreme(arm_obj, limits, which)
        bpy.context.scene.render.filepath = os.path.join(out_dir, "skeleton_%s.png" % which)
        bpy.ops.render.render(write_still=True)
    pose_named(arm_obj, limits, DEMO_POSE)
    bpy.context.view_layer.update()
    print("\nDemo pose check (armature space, metres; forward is -Y, robot left is +X):")
    for name in DEMO_POSE:
        pb = arm_obj.pose.bones[name]
        d = pb.tail - pb.head
        print("  %-11s tail - head = (%+.2f, %+.2f, %+.2f)" % (name, d.x, d.y, d.z))
    bpy.context.scene.render.filepath = os.path.join(out_dir, "skeleton_demo.png")
    bpy.ops.render.render(write_still=True)
    pose_extreme(arm_obj, {}, "max")  # back to rest
    blend = os.path.join(out_dir, "skeleton.blend")
    bpy.ops.wm.save_as_mainfile(filepath=blend)
    print("saved", blend, "and skeleton_{rest,max,min,demo}.png")


if __name__ == "__main__":
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    jp = argv[argv.index("--json") + 1] if "--json" in argv else "cad/skeleton.json"
    out = argv[argv.index("--out") + 1] if "--out" in argv else "cad/out"
    main(os.path.abspath(jp), os.path.abspath(out))
