"""
finger.py - parametric generator for a three-joint tendon-driven finger (Phase 0 test article).

Four parts: palm block (MCP tongue, PTFE socket), proximal, middle and distal phalanx.
Each phalanx has a fork at its proximal end and a tongue at its distal end, joined by a 2 mm pin.
One flexor tendon runs in a palmar channel through all three; the joint axes sit dorsal of the
channel by the moment arm (MCP 9, PIP 6.5, DIP 4.5 mm), so a single pull curls the finger
distal-first. An elastic extensor cord runs in a dorsal channel and rides over each knuckle.

Frame per part: X proximal -> distal, Z dorsal (+) / palmar (-), Y across the finger.
Units: mm. Parts are exported in place (straight pose) so the assembly can be viewed together,
and the script reports total tendon excursion for the calculator.

Run headless:
    blender -b -P cad/blender/finger.py -- --out cad/out
"""
import math
import os
import sys

import bmesh
import bpy

P = {
    "W": 16.0,                  # finger width (Y)
    "H": 16.0,                  # finger height (Z)
    "L": {"proximal": 45.0, "middle": 28.0, "distal": 24.0},
    "palm_len": 30.0,
    # joint axes, Z offset from the phalanx centreline (dorsal positive)
    "axis_z": {"mcp": 5.0, "pip": 2.5, "dip": 0.5},
    "flexor_z": -4.0,           # palmar channel centreline -> moment arm = axis_z - flexor_z
    # The flexor channel is lined with a hollow body-piercing needle cut to length (16 G: 1.65 mm OD,
    # ~1.2 mm ID), pushed in after printing. Channel = needle OD + press-fit allowance.
    "liner_od": 1.65,
    "liner_press": 0.10,
    "mouth_chamfer_r": 1.0,     # countersink at each channel mouth so the line never meets a sharp edge
    "mouth_chamfer_depth": 1.2,
    "extensor_z": 5.5,          # dorsal channel; the cord rides over the knuckle lug (axis + lug_r) at each joint
    "extensor_d": 2.0,
    "lug_r": 4.5,               # knuckle radius around the pin
    "lug_w": 4.0,               # fork lug thickness (Y)
    "lug_clear": 0.3,           # per side between tongue and fork
    "pin_d": 2.0,
    "pin_press": 1.9,           # hole in the fork lugs (press fit)
    "pin_clear": 2.3,           # hole in the tongue (free)
    "body_gap": 0.6,            # gap between body face and the lug radius
    "chamfer": 6.0,             # palmar corner chamfer at each joint for flexion clearance
    "knot_pocket_d": 3.5,
    "tube_od": 4.0,
    "tube_clear": 0.3,
    "tube_socket_depth": 8.0,
    "mount_hole_d": 3.2,
    "segments": 48,
}

ROM_DEG = {"mcp": 90, "pip": 100, "dip": 70}


# ----------------------------------------------------------------------------- helpers
def _activate(obj):
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj


def box(name, x0, x1, y0, y1, z0, z1, rot_y=0.0, pivot=None):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
    o = bpy.context.active_object
    o.name = name
    o.scale = (x1 - x0, y1 - y0, z1 - z0)
    _activate(o)
    bpy.ops.object.transform_apply(scale=True)
    if rot_y:
        px, pz = pivot
        # rotate about a Y-parallel axis through (px, *, pz)
        o.location = (o.location.x - px, o.location.y, o.location.z - pz)
        bpy.ops.object.transform_apply(location=True)
        o.rotation_euler = (0, rot_y, 0)
        bpy.ops.object.transform_apply(rotation=True)
        o.location = (px, 0, pz)
        bpy.ops.object.transform_apply(location=True)
    return o


def cyl(name, axis, r, a0, a1, c1, c2, segments=None):
    length, mid = a1 - a0, (a0 + a1) / 2
    if axis == "x":
        loc, rot = (mid, c1, c2), (0, math.pi / 2, 0)
    elif axis == "y":
        loc, rot = (c1, mid, c2), (math.pi / 2, 0, 0)
    else:
        loc, rot = (c1, c2, mid), (0, 0, 0)
    bpy.ops.mesh.primitive_cylinder_add(vertices=segments or P["segments"], radius=r, depth=length,
                                        location=loc, rotation=rot)
    o = bpy.context.active_object
    o.name = name
    _activate(o)
    bpy.ops.object.transform_apply(rotation=True)
    return o


def cone_x(name, r_wide, r_narrow, depth, x_face, facing, y, z):
    """Countersink cone along X: wide end on the face at x_face, narrowing into the part.
    facing = -1 when the face looks toward -X (proximal mouth), +1 when it looks toward +X."""
    # The narrow end is made 0.05 mm larger than the channel and the wide end starts 0.1 mm outside the
    # face, with a different segment count, so no cone surface or vertex coincides with the channel
    # cylinder (coincident geometry makes the exact boolean produce non-manifold output).
    r_narrow += 0.05
    over = 0.1
    depth += over
    if facing < 0:
        r1, r2, xc = r_wide, r_narrow, x_face - over + depth / 2   # local -Z (wide) -> world -X
    else:
        r1, r2, xc = r_narrow, r_wide, x_face + over - depth / 2
    bpy.ops.mesh.primitive_cone_add(vertices=P["segments"] - 11, radius1=r1, radius2=r2, depth=depth,
                                    location=(xc, y, z), rotation=(0, math.pi / 2, 0))
    o = bpy.context.active_object
    o.name = name
    _activate(o)
    bpy.ops.object.transform_apply(rotation=True)
    return o


def boolean(target, tool, op):
    _activate(target)
    m = target.modifiers.new(name=op, type="BOOLEAN")
    m.operation, m.solver, m.object = op, "EXACT", tool
    with bpy.context.temp_override(object=target, active_object=target, selected_objects=[target]):
        bpy.ops.object.modifier_apply(modifier=m.name)
    bpy.data.objects.remove(tool, do_unlink=True)
    return target


def cut(t, tool):
    return boolean(t, tool, "DIFFERENCE")


def join(t, tool):
    return boolean(t, tool, "UNION")


def translate(obj, dx):
    obj.location.x += dx
    _activate(obj)
    bpy.ops.object.transform_apply(location=True)


def cleanup(obj, dist=0.02):
    """Merge near-coincident vertices and dissolve degenerate slivers the exact boolean can leave where
    two cylinders cross (e.g. a mount hole through a tendon channel). Returns the non-manifold edge count."""
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=dist)
    bmesh.ops.dissolve_degenerate(bm, dist=dist, edges=bm.edges)
    n = sum(1 for e in bm.edges if not e.is_manifold)
    bm.to_mesh(obj.data)
    bm.free()
    obj.data.update()
    return n


def volume_mm3(obj):
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.transform(obj.matrix_world)
    v = bm.calc_volume(signed=False)
    bm.free()
    return v


def export_stl(obj, path):
    _activate(obj)
    if hasattr(bpy.ops.wm, "stl_export"):
        bpy.ops.wm.stl_export(filepath=path, export_selected_objects=True, apply_modifiers=True)
    else:
        bpy.ops.export_mesh.stl(filepath=path, use_selection=True)


# ----------------------------------------------------------------------------- features
def add_fork(part, axis_z):
    """Two side lugs around a pin at (x=0, z=axis_z), bridged to the body which starts at x = lug_r + gap."""
    W, lr, lw = P["W"], P["lug_r"], P["lug_w"]
    x_body = lr + P["body_gap"]
    for s in (-1, 1):
        y0, y1 = (W / 2 - lw, W / 2) if s > 0 else (-W / 2, -W / 2 + lw)
        part = join(part, cyl("fork_lug", "y", lr, y0, y1, 0.0, axis_z))
        part = join(part, box("fork_web", 0.0, x_body + 0.01, y0, y1, max(axis_z - lr, -P["H"] / 2), min(axis_z + lr, P["H"] / 2)))
    part = cut(part, cyl("fork_pin", "y", P["pin_press"] / 2, -W, W, 0.0, axis_z))
    return part


def add_tongue(part, x_axis, axis_z):
    """Centre lug around a pin at (x_axis, axis_z), bridged back to the body which ends at x_axis - lug_r - gap."""
    W, lr, lw, c = P["W"], P["lug_r"], P["lug_w"], P["lug_clear"]
    y0, y1 = -W / 2 + lw + c, W / 2 - lw - c
    x_body = x_axis - lr - P["body_gap"]
    part = join(part, cyl("tongue_lug", "y", lr, y0, y1, x_axis, axis_z))
    part = join(part, box("tongue_web", x_body - 0.01, x_axis, y0, y1, max(axis_z - lr, -P["H"] / 2), min(axis_z + lr, P["H"] / 2)))
    part = cut(part, cyl("tongue_pin", "y", P["pin_clear"] / 2, -W, W, x_axis, axis_z))
    return part


def chamfer_palmar(part, x_corner, direction):
    """Cut a 45-degree chamfer with leg length P['chamfer'] off the palmar corner at x_corner.

    A square of side c*sqrt(2) rotated 45 degrees and centred on the corner point removes exactly the
    triangle with legs c from the body, whichever way the corner faces (direction is kept for clarity).
    """
    c = P["chamfer"]
    zb = -P["H"] / 2
    s = c * math.sqrt(2)
    tool = box("chamfer", x_corner - s / 2, x_corner + s / 2, -P["W"], P["W"], zb - s / 2, zb + s / 2,
               rot_y=math.radians(45), pivot=(x_corner, zb))
    return cut(part, tool)


def phalanx(name, L, prox_axis_z, dist_axis_z, is_distal=False):
    W, H, lr = P["W"], P["H"], P["lug_r"]
    x0 = lr + P["body_gap"]
    x1 = L if is_distal else L - lr - P["body_gap"]
    part = box(name, x0, x1, -W / 2, W / 2, -H / 2, H / 2)
    if is_distal:  # rounded fingertip
        part = join(part, cyl("tip", "y", H / 2, -W / 2, W / 2, x1, 0.0))
    part = add_fork(part, prox_axis_z)
    if not is_distal:
        part = add_tongue(part, L, dist_axis_z)
        part = chamfer_palmar(part, x1, -1)
    part = chamfer_palmar(part, x0, +1)
    # flexor channel (palmar, sized for the needle liner) and extensor channel (dorsal), full length of the body
    liner_r = (P["liner_od"] + P["liner_press"]) / 2
    x_end = (L - 7.0) if is_distal else L + H   # in the fingertip the channels stop at the knot pockets
    part = cut(part, cyl("flexor", "x", liner_r, -1, x_end, 0.0, P["flexor_z"]))
    part = cut(part, cyl("extensor", "x", P["extensor_d"] / 2, -1, x_end, 0.0, P["extensor_z"]))
    # countersunk mouths on both channels at the body faces (the needle stops 1 mm short of each mouth)
    mouths = [(x0, -1)] + ([] if is_distal else [(x1, +1)])
    # The flexor runs below the chamfer line, so its mouth is on the 45-degree chamfer plane, which at
    # z = flexor_z sits (flexor_z + H/2) inboard of the body face; the extensor mouth is on the flat face.
    flex_inset = P["flexor_z"] + H / 2
    for xf, facing in mouths:
        for zc, rc, inset in ((P["flexor_z"], liner_r, flex_inset), (P["extensor_z"], P["extensor_d"] / 2, 0.0)):
            part = cut(part, cone_x("mouth", rc + P["mouth_chamfer_r"], rc, P["mouth_chamfer_depth"],
                                    xf - facing * inset, facing, 0.0, zc))
    if is_distal:
        # flexor knot pocket from the palmar face near the tip; extensor cord anchor pocket from the dorsal face
        part = cut(part, cyl("knot_pocket", "z", P["knot_pocket_d"] / 2, -H, P["flexor_z"] + 0.6, L - 7.0, 0.0))
        part = cut(part, cyl("ext_pocket", "z", P["knot_pocket_d"] / 2, P["extensor_z"] - 0.6, H, L - 7.0, 0.0))
    return part


def palm_block():
    W, H, lr = P["W"], P["H"], P["lug_r"]
    L = P["palm_len"]
    x1 = L - lr - P["body_gap"]
    part = box("finger_palm", 0.0, x1, -W / 2, W / 2, -H / 2, H / 2)
    part = add_tongue(part, L, P["axis_z"]["mcp"])
    part = chamfer_palmar(part, x1, -1)
    liner_r = (P["liner_od"] + P["liner_press"]) / 2
    part = cut(part, cyl("flexor", "x", liner_r, -1, L + 1, 0.0, P["flexor_z"]))
    part = cut(part, cyl("tube_socket", "x", (P["tube_od"] + P["tube_clear"]) / 2, -1, P["tube_socket_depth"], 0.0, P["flexor_z"]))
    part = cut(part, cyl("extensor", "x", P["extensor_d"] / 2, -1, L + 1, 0.0, P["extensor_z"]))
    flex_inset = P["flexor_z"] + H / 2
    for zc, rc, inset in ((P["flexor_z"], liner_r, flex_inset), (P["extensor_z"], P["extensor_d"] / 2, 0.0)):
        part = cut(part, cone_x("mouth", rc + P["mouth_chamfer_r"], rc, P["mouth_chamfer_depth"], x1 - inset, +1, 0.0, zc))
    for x in (10.5, 16.3):  # mounting holes through Z for the test stand (clear of the socket and the chamfer)
        # 36 segments: a different tessellation from the X-axis channels avoids vertex coincidences in the boolean
        part = cut(part, cyl("mount", "z", P["mount_hole_d"] / 2, -H, H, x, 0.0, segments=36))
    return part


# ----------------------------------------------------------------------------- main
def main(out_dir):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    az = P["axis_z"]
    parts = []
    palm = palm_block()
    parts.append(palm)
    prox = phalanx("finger_proximal", P["L"]["proximal"], az["mcp"], az["pip"])
    translate(prox, P["palm_len"])
    parts.append(prox)
    mid = phalanx("finger_middle", P["L"]["middle"], az["pip"], az["dip"])
    translate(mid, P["palm_len"] + P["L"]["proximal"])
    parts.append(mid)
    dist = phalanx("finger_distal", P["L"]["distal"], az["dip"], None, is_distal=True)
    translate(dist, P["palm_len"] + P["L"]["proximal"] + P["L"]["middle"])
    parts.append(dist)

    os.makedirs(out_dir, exist_ok=True)
    arms = {j: az[j] - P["flexor_z"] for j in az}
    exc = sum(math.radians(ROM_DEG[j]) * arms[j] for j in az)
    print("\nFinger: palm %.0f + phalanges %.0f/%.0f/%.0f mm, section %.0f x %.0f mm" %
          (P["palm_len"], P["L"]["proximal"], P["L"]["middle"], P["L"]["distal"], P["W"], P["H"]))
    print("flexor moment arms mm: " + ", ".join("%s %.1f" % (j, arms[j]) for j in az))
    print("full-curl tendon excursion: %.1f mm (ROM %s)" % (exc, ROM_DEG))
    for o in parts:
        bad = cleanup(o)
        v = volume_mm3(o)
        path = os.path.join(out_dir, o.name + ".stl")
        export_stl(o, path)
        print("  %-18s volume %6.0f mm^3  (~%.1f g solid PETG)  non-manifold edges %d -> %s" %
              (o.name, v, v * 1.27e-3, bad, path))


if __name__ == "__main__":
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    out = argv[argv.index("--out") + 1] if "--out" in argv else "cad/out"
    main(os.path.abspath(out))
