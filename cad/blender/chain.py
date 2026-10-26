"""
chain.py - the knuckle-chain primitive: segments joined by fork-and-tongue pin joints, driven by
tendons in lined channels. One generator, many presets (P-007).

A chain is a list of segments and the joints between them. Every joint is a fork (two lugs) on one
segment and a tongue (centre lug) on the other, on a pin or on bearings. The tongue lug doubles as
the joint pulley: for a single-tendon joint the elastic rides over it; for a closed-loop joint it
carries a line groove and a pocket for the crimped stop bead. Variations "sewn in":

  axis       'y' = pitch joint (lugs across the width), 'z' = yaw joint (lugs across the height)
  off        axis offset from the section centreline, dorsal positive (asymmetric moment arms)
  fit        {'type': 'pin', ...} or {'type': 'bearing', 'od', 'w', 'bore'}
  tongue_on  'prox' (tongue on the proximal segment, tendon passes through: fingers, toes)
             'dist' (tongue on the distal = moving segment, loop terminates on it: every loop joint)
  loop       None, or {'d': line diameter} -> groove + bead pocket on the tongue lug
  chamfer    clearance chamfer legs on the faces the joint swings toward

Frame per segment: X proximal -> distal, Z dorsal/up, Y across. Units mm. Each preset exports one
STL per segment, in place (straight chain), plus a report.

Run headless:
    blender -b -P cad/blender/chain.py -- --out cad/out            (all presets)
    blender -b -P cad/blender/chain.py -- --out cad/out --preset wrist
"""
import copy
import math
import os
import sys

import bmesh
import bpy

SEG = 48            # default cylinder tessellation
PETG_G_PER_MM3 = 1.27e-3
TUBE_OD, TUBE_CLEAR, TUBE_SOCKET = 4.0, 0.3, 8.0


# ----------------------------------------------------------------------------- primitives
def _activate(obj):
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj


def box(name, x0, x1, y0, y1, z0, z1, rot=None, pivot=None):
    """Axis-aligned box; rot=('y'|'z', radians) spins it about an axis through pivot=(px, py, pz)."""
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
    o = bpy.context.active_object
    o.name = name
    o.scale = (x1 - x0, y1 - y0, z1 - z0)
    _activate(o)
    bpy.ops.object.transform_apply(scale=True)
    if rot:
        axis, ang = rot
        px, py, pz = pivot
        o.location = (o.location.x - px, o.location.y - py, o.location.z - pz)
        bpy.ops.object.transform_apply(location=True)
        o.rotation_euler = (0, ang, 0) if axis == "y" else (0, 0, ang)
        bpy.ops.object.transform_apply(rotation=True)
        o.location = (px, py, pz)
        bpy.ops.object.transform_apply(location=True)
    return o


def cyl(name, axis, r, a0, a1, c1, c2, segments=SEG):
    """Cylinder along axis from a0 to a1; (c1, c2) are the other two coordinates in x,y,z order."""
    length, mid = a1 - a0, (a0 + a1) / 2
    if axis == "x":
        loc, rot = (mid, c1, c2), (0, math.pi / 2, 0)
    elif axis == "y":
        loc, rot = (c1, mid, c2), (math.pi / 2, 0, 0)
    else:
        loc, rot = (c1, c2, mid), (0, 0, 0)
    bpy.ops.mesh.primitive_cylinder_add(vertices=segments, radius=r, depth=length, location=loc, rotation=rot)
    o = bpy.context.active_object
    o.name = name
    _activate(o)
    bpy.ops.object.transform_apply(rotation=True)
    return o


def cone_x(name, r_wide, r_narrow, depth, x_face, facing, y, z):
    """Countersink along X, wide end on the face at x_face. facing -1: face looks toward -X."""
    r_narrow += 0.05
    over = 0.1
    depth += over
    if facing < 0:
        r1, r2, xc = r_wide, r_narrow, x_face - over + depth / 2
    else:
        r1, r2, xc = r_narrow, r_wide, x_face + over - depth / 2
    bpy.ops.mesh.primitive_cone_add(vertices=SEG - 11, radius1=r1, radius2=r2, depth=depth,
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


# ----------------------------------------------------------------------------- joint features
def lug_span(joint, W, H):
    """Lateral extent (along the lug axis) of fork lugs and tongue, and the perpendicular limits."""
    full = W if joint["axis"] == "y" else H          # extent along the lug axis
    lw, c = joint["lug_w"], joint.get("clear", 0.3)
    fork = [(-full / 2, -full / 2 + lw), (full / 2 - lw, full / 2)]
    tongue = (-full / 2 + lw + c, full / 2 - lw - c)
    return fork, tongue


def lug_cyl(name, joint, x_axis, a0, a1):
    """Cylinder along the lug axis through the joint axis at x_axis."""
    if joint["axis"] == "y":
        return cyl(name, "y", joint["lug_r"], a0, a1, x_axis, joint["off"])
    return cyl(name, "z", joint["lug_r"], a0, a1, x_axis, joint["off"])


def web_box(name, joint, xa, xb, a0, a1, W, H):
    """Box bridging a lug to the body between x = xa..xb, spanning a0..a1 along the lug axis and
    +-lug_r (clamped to the section) perpendicular to it."""
    r, off = joint["lug_r"], joint["off"]
    if joint["axis"] == "y":
        return box(name, xa, xb, a0, a1, max(off - r, -H / 2), min(off + r, H / 2))
    return box(name, xa, xb, max(off - r, -W / 2), min(off + r, W / 2), a0, a1)


WEB_OVERLAP = 2.0   # how far a lug web reaches into the solid body (a robust union, not a coplanar sliver)


def add_fork(part, joint, x_axis, body_x, W, H):
    """Two lugs at x_axis bridged to the body face at body_x."""
    fork, _ = lug_span(joint, W, H)
    xa, xb = (x_axis, body_x + WEB_OVERLAP) if body_x > x_axis else (body_x - WEB_OVERLAP, x_axis)
    for a0, a1 in fork:
        part = join(part, lug_cyl("fork_lug", joint, x_axis, a0, a1))
        part = join(part, web_box("fork_web", joint, xa, xb, a0, a1, W, H))
    fit = joint["fit"]
    if fit["type"] == "bearing":
        # bearing seats from the outer faces, a 2 mm shoulder inside, shaft hole through
        for (a0, a1), outer in zip(fork, (-1, 1)):
            depth = fit["w"] + 0.1
            s0, s1 = (a0 - 1, a0 + depth) if outer < 0 else (a1 - depth, a1 + 1)
            part = cut(part, lug_cyl("seat", dict(joint, lug_r=fit["od"] / 2 + 0.05), x_axis, s0, s1))
        part = cut(part, lug_cyl("shaft", dict(joint, lug_r=fit["bore"] / 2 + 0.1), x_axis, -max(W, H), max(W, H)))
    else:
        part = cut(part, lug_cyl("pin", dict(joint, lug_r=fit["press"] / 2), x_axis, -max(W, H), max(W, H)))
    return part


def add_tongue(part, joint, x_axis, body_x, W, H):
    """Centre lug at x_axis bridged to the body face at body_x; groove and bead pocket for loop joints."""
    _, (t0, t1) = lug_span(joint, W, H)
    xa, xb = (x_axis, body_x + WEB_OVERLAP) if body_x > x_axis else (body_x - WEB_OVERLAP, x_axis)
    part = join(part, lug_cyl("tongue_lug", joint, x_axis, t0, t1))
    part = join(part, web_box("tongue_web", joint, xa, xb, t0, t1, W, H))
    if joint.get("loop"):
        d = joint["loop"]["d"]
        gr, gd = d / 2 + 0.3, d * 0.8                 # groove half-width, depth
        r = joint["lug_r"]
        mid = (t0 + t1) / 2
        part = cut(part, lug_cyl("groove_out", dict(joint, lug_r=r + 1), x_axis, mid - gr, mid + gr))
        part = join(part, lug_cyl("groove_in", dict(joint, lug_r=r - gd), x_axis, mid - gr - 0.01, mid + gr + 0.01))
        # bead pocket at the top of the rim, on the side away from the body (the loop midpoint sits here)
        pk = 2.5 * d + 2
        side = -1 if body_x > x_axis else 1
        if joint["axis"] == "y":
            part = cut(part, box("bead_pocket", x_axis + side * (r - pk), x_axis + side * (r + 1),
                                 mid - gr - 1, mid + gr + 1, joint["off"] - pk / 2, joint["off"] + pk / 2))
        else:
            part = cut(part, box("bead_pocket", x_axis + side * (r - pk), x_axis + side * (r + 1),
                                 joint["off"] - pk / 2, joint["off"] + pk / 2, mid - gr - 1, mid + gr + 1))
    fit = joint["fit"]
    if fit["type"] == "pin":
        part = cut(part, lug_cyl("tongue_hole", dict(joint, lug_r=fit["clear"] / 2), x_axis, -max(W, H), max(W, H)))
    else:
        # Stub axles: two short shafts, one per fork lug, each pressed into a blind bore in the tongue.
        # The tongue centre stays solid, so tendons for the next joint can cross this axis through it.
        depth = stub_depth(fit)
        hr = fit["bore"] / 2 - 0.05   # press fit for the stub
        part = cut(part, lug_cyl("stub_bore", dict(joint, lug_r=hr), x_axis, t0 - 1, t0 + depth))
        part = cut(part, lug_cyl("stub_bore", dict(joint, lug_r=hr), x_axis, t1 - depth, t1 + 1))
    return part


def stub_depth(fit):
    return fit["w"] if fit["type"] == "bearing" else 0.0


def free_centre_half(joint, W, H):
    """Half-width of the tongue region left solid between the two stub bores (0 for pin joints)."""
    _, (t0, t1) = lug_span(joint, W, H)
    return (t1 - t0) / 2 - stub_depth(joint["fit"])


def chamfer_edge(part, x_corner, side, which, c, W, H):
    """45-degree chamfer of leg c along the body edge at x_corner.
    which='z': edge on the +-z face (side), running along Y. which='y': edge on the +-y face, along Z."""
    s = c * math.sqrt(2)
    if which == "z":
        zc = side * H / 2
        tool = box("chamfer", x_corner - s / 2, x_corner + s / 2, -W, W, zc - s / 2, zc + s / 2,
                   rot=("y", math.radians(45)), pivot=(x_corner, 0, zc))
    else:
        yc = side * W / 2
        tool = box("chamfer", x_corner - s / 2, x_corner + s / 2, yc - s / 2, yc + s / 2, -H, H,
                   rot=("z", math.radians(45)), pivot=(x_corner, yc, 0))
    return cut(part, tool)


def chamfer_sides(joint, single_tendon):
    """Which body edges get a clearance chamfer at this joint."""
    if joint["axis"] == "y":
        return [("z", -1)] if single_tendon else [("z", -1), ("z", 1)]
    return [("y", -1), ("y", 1)]


def mouth_inset(ch, chamfers, c, W, H):
    """How far inboard of the body face a channel's mouth sits because of a chamfer there."""
    inset = 0.0
    for which, side in chamfers:
        coord, half = (ch["z"], H / 2) if which == "z" else (ch["y"], W / 2)
        dist_from_edge = half - side * coord      # distance of the channel centre from the chamfered face
        inset = max(inset, c - dist_from_edge)
    return max(inset, 0.0)


# ----------------------------------------------------------------------------- a segment
def build_segment(spec, i, x_start):
    W, H = spec["W"], spec["H"]
    seg = spec["segments"][i]
    L = seg["L"]
    jp = spec["joints"][i - 1] if i > 0 else None                      # proximal joint
    jd = spec["joints"][i] if i < len(spec["joints"]) else None       # distal joint
    single = spec.get("single_tendon", False)

    x0 = (jp["lug_r"] + jp.get("gap", 0.6)) if jp else 0.0
    x1 = (L - jd["lug_r"] - jd.get("gap", 0.6)) if jd else L
    name = "%s_%s" % (spec["name"], seg["name"])
    part = box(name, x0, x1, -W / 2, W / 2, -H / 2, H / 2)
    for x in seg.get("mounts", []):   # through-holes first, while the faces they pierce are still simple
        part = cut(part, cyl("mount", "z", 1.6, -H, H, x, 0.0, segments=36))
    if seg.get("tip"):
        part = join(part, cyl("tip", "y", H / 2, -W / 2, W / 2, x1, 0.0))

    # joints
    cham_p, cham_d = [], []
    if jp:
        kind = "fork" if jp["tongue_on"] == "prox" else "tongue"
        part = (add_fork if kind == "fork" else add_tongue)(part, jp, 0.0, x0, W, H)
        cham_p = chamfer_sides(jp, single)
        for which, side in cham_p:
            part = chamfer_edge(part, x0, side, which, jp["chamfer"], W, H)
    if jd:
        kind = "tongue" if jd["tongue_on"] == "prox" else "fork"
        part = (add_fork if kind == "fork" else add_tongue)(part, jd, L, x1, W, H)
        cham_d = chamfer_sides(jd, single)
        for which, side in cham_d:
            part = chamfer_edge(part, x1, side, which, jd["chamfer"], W, H)

    # channels
    for ch in seg.get("channels", []):
        r = ch["d"] / 2
        xe = ch.get("x_end", L + H)
        xs = ch.get("x_start", -1.0)
        part = cut(part, cyl("ch_" + ch["name"], "x", r, xs, xe, ch["y"], ch["z"]))
        for face, cham, c, facing in ((x0, cham_p, jp["chamfer"] if jp else 0, -1), (x1, cham_d, jd["chamfer"] if jd else 0, +1)):
            if (facing < 0 and xs > face) or (facing > 0 and xe < face):
                continue   # channel does not reach this face
            inset = mouth_inset(ch, cham, c, W, H)
            if ch.get("socket_at") == ("prox" if facing < 0 else "dist"):
                sr = (TUBE_OD + TUBE_CLEAR) / 2
                s0, s1 = (face - 1, face + TUBE_SOCKET) if facing < 0 else (face - TUBE_SOCKET, face + 1)
                part = cut(part, cyl("socket", "x", sr, s0, s1, ch["y"], ch["z"]))
            elif inset > 0:
                # mouth on a 45-degree chamfer plane: a spherical dimple centred on the exit point
                # (a cone's flat rim grazing the inclined plane leaves sliver faces)
                bpy.ops.mesh.primitive_uv_sphere_add(segments=24, ring_count=12, radius=r + 0.9,
                                                     location=(face - facing * inset, ch["y"], ch["z"]))
                sph = bpy.context.active_object
                sph.name = "mouth_dimple"
                part = cut(part, sph)
            else:
                part = cut(part, cone_x("mouth", r + 1.0, r, 1.2, face, facing, ch["y"], ch["z"]))
        if ch.get("pocket"):   # knot / bead pocket from the near face at the channel end
            zs = -H if ch["z"] < 0 else H
            part = cut(part, cyl("pocket", "z", 1.75, min(zs, ch["z"] + math.copysign(0.6, ch["z"])),
                                 max(zs, ch["z"] + math.copysign(0.6, ch["z"])), xe, ch["y"]))
    translate(part, x_start)
    return part


def build_chain(spec, out_dir):
    x = 0.0
    parts = []
    for i in range(len(spec["segments"])):
        parts.append(build_segment(spec, i, x))
        x += spec["segments"][i]["L"]
    rows = []
    for o in parts:
        bad = cleanup(o)
        v = volume_mm3(o)
        export_stl(o, os.path.join(out_dir, o.name + ".stl"))
        rows.append((o.name, v, bad))
    return parts, rows


# ----------------------------------------------------------------------------- joint helpers
def pin(press, clear):
    return {"type": "pin", "press": press, "clear": clear}


BEARING_625 = {"type": "bearing", "od": 16.0, "w": 5.0, "bore": 5.0}
BEARING_608 = {"type": "bearing", "od": 22.0, "w": 7.0, "bore": 8.0}


def J(axis, lug_r, lug_w, fit, tongue_on, off=0.0, loop=None, chamfer=6.0):
    return {"axis": axis, "lug_r": lug_r, "lug_w": lug_w, "fit": fit, "tongue_on": tongue_on,
            "off": off, "loop": loop, "chamfer": chamfer}


def loop_channels(prefix, axis, lug_r, d, liner_od, W, H, **kw):
    """Two sheathed runs tangent to the tongue pulley on either side of the axis."""
    bore = liner_od + 0.1
    o = lug_r - d * 0.8 - d / 2 + 0.3    # groove bottom + half line
    if axis == "y":
        return [dict(name=prefix + "_a", y=0.0, z=+o, d=bore, **kw), dict(name=prefix + "_b", y=0.0, z=-o, d=bore, **kw)]
    return [dict(name=prefix + "_a", y=+o, z=0.0, d=bore, **kw), dict(name=prefix + "_b", y=-o, z=0.0, d=bore, **kw)]


# ----------------------------------------------------------------------------- presets
def finger_preset(name="finger", scale=1.0):
    s = scale
    flex = dict(name="flexor", y=0.0, z=-4.0 * s, d=1.75)
    ext = dict(name="extensor", y=0.0, z=5.5 * s, d=2.0)
    return {
        "name": name, "W": 16 * s, "H": 16 * s, "single_tendon": True,
        "instances": 8, "dof_per_instance": 1,
        "segments": [
            dict(name="palm", L=30 * s, channels=[dict(flex, socket_at="prox"), dict(ext)], mounts=[10.5 * s, 16.3 * s]),
            dict(name="proximal", L=45 * s, channels=[dict(flex), dict(ext)]),
            dict(name="middle", L=28 * s, channels=[dict(flex), dict(ext)]),
            dict(name="distal", L=24 * s, tip=True,
                 channels=[dict(flex, x_end=17 * s, pocket=True), dict(ext, x_end=17 * s, pocket=True)]),
        ],
        "joints": [
            J("y", 4.5 * s, 4.0 * s, pin(1.9, 2.3), "prox", off=5.0 * s),
            J("y", 4.5 * s, 4.0 * s, pin(1.9, 2.3), "prox", off=2.5 * s),
            J("y", 4.5 * s, 4.0 * s, pin(1.9, 2.3), "prox", off=0.5 * s),
        ],
    }


def thumb_preset():
    p = finger_preset("thumb", 1.1)
    p["instances"], p["dof_per_instance"] = 2, 1
    p["segments"] = [p["segments"][0], dict(p["segments"][1], name="proximal", L=40), dict(p["segments"][3], name="distal", L=30)]
    p["segments"][2]["channels"] = [dict(c, x_end=23) for c in p["segments"][2]["channels"]]
    p["joints"] = [p["joints"][0], dict(p["joints"][1], off=1.0)]
    return p


def toe_preset():
    """Passive toe plate: one wide pin joint, elastic only, no tendon channel."""
    return {
        "name": "toe", "W": 90, "H": 14, "single_tendon": True, "instances": 2, "dof_per_instance": 0,
        "segments": [dict(name="sole", L=40, channels=[dict(name="elastic", y=0.0, z=4.0, d=2.2)]),
                     dict(name="plate", L=50, tip=True, channels=[dict(name="elastic", y=0.0, z=4.0, d=2.2, x_end=40, pocket=True)])],
        "joints": [J("y", 5.0, 12.0, pin(2.9, 3.3), "prox", off=2.0, chamfer=5.0)],
    }


def universal_preset(name, W, H, lug_r, lug_w, fit, line_d, liner_od, L=(40, 30, 40), instances=2, chamfer=8.0):
    """Two loop joints with axes at 90 degrees: pitch then yaw. Wrist, ankle, neck, waist."""
    j1 = J("y", lug_r, lug_w, fit, "dist", loop={"d": line_d}, chamfer=chamfer)
    j2 = J("z", lug_r, lug_w, fit, "dist", loop={"d": line_d}, chamfer=chamfer)
    base_ch = loop_channels("pitch", "y", lug_r, line_d, liner_od, W, H, socket_at="dist") + \
        loop_channels("yaw", "z", lug_r, line_d, liner_od, W, H)
    mid_ch = loop_channels("yaw", "z", lug_r, line_d, liner_od, W, H, socket_at="dist")
    # The yaw runs cross the pitch axis through the solid centre of the pitch tongue (stub axles).
    # They must fit between the two stub bores, or the section has to grow.
    need = abs(mid_ch[0]["y"]) + mid_ch[0]["d"] / 2 + 1.0
    have = free_centre_half(j1, W, H)
    if have < need:
        raise ValueError("%s: yaw runs need +-%.1f mm of free tongue centre, have %.1f; widen W" % (name, need, have))
    return {
        "name": name, "W": W, "H": H, "instances": instances, "dof_per_instance": 2,
        "segments": [dict(name="base", L=L[0], channels=base_ch),
                     dict(name="cross", L=L[1], channels=mid_ch),
                     dict(name="end", L=L[2])],
        "joints": [j1, j2],
    }


def hinge_preset(name, W, H, lug_r, lug_w, fit, line_d, liner_od, L=(60, 60), instances=2, chamfer=10.0):
    """One big loop joint on bearings: elbow, knee."""
    j = J("y", lug_r, lug_w, fit, "dist", loop={"d": line_d}, chamfer=chamfer)
    return {
        "name": name, "W": W, "H": H, "instances": instances, "dof_per_instance": 1,
        "segments": [dict(name="upper", L=L[0], channels=loop_channels("flex", "y", lug_r, line_d, liner_od, W, H, socket_at="dist")),
                     dict(name="lower", L=L[1])],
        "joints": [j],
    }


PRESETS = {
    "finger": finger_preset,
    "thumb": thumb_preset,
    "toe": toe_preset,
    # universal joints: W must leave room for the yaw runs to cross the pitch tongue centre (checked at build)
    "wrist": lambda: universal_preset("wrist", 58, 40, 12, 7.5, BEARING_625, 0.55, 2.11, chamfer=8),
    "ankle": lambda: universal_preset("ankle", 76, 60, 18, 9.5, BEARING_608, 0.75, 2.77, L=(60, 40, 50), chamfer=12),
    "neck": lambda: universal_preset("neck", 52, 40, 10, 7.5, BEARING_625, 0.48, 1.65, L=(40, 30, 40), instances=1, chamfer=6),
    "waist": lambda: universal_preset("waist", 120, 80, 30, 9.5, BEARING_608, 0.75, 2.77, L=(60, 60, 60), instances=1, chamfer=16),
    "elbow": lambda: hinge_preset("elbow", 60, 70, 30, 9.5, BEARING_608, 0.55, 2.11, chamfer=10),
    "knee": lambda: hinge_preset("knee", 80, 110, 50, 9.5, BEARING_608, 0.75, 2.77, L=(80, 80), chamfer=14),
    "hip": lambda: universal_preset("hip", 82, 80, 20, 9.5, BEARING_608, 0.75, 2.77, L=(60, 40, 60), chamfer=12),
    "shoulder": lambda: universal_preset("shoulder", 76, 70, 18, 9.5, BEARING_608, 0.75, 2.77, L=(50, 40, 50), chamfer=10),
}


# ----------------------------------------------------------------------------- main
def main(out_dir, only=None):
    os.makedirs(out_dir, exist_ok=True)
    names = [only] if only else list(PRESETS)
    total_inst, total_dof = 0, 0
    print("\nknuckle-chain presets")
    for n in names:
        bpy.ops.wm.read_factory_settings(use_empty=True)
        spec = copy.deepcopy(PRESETS[n]())
        parts, rows = build_chain(spec, out_dir)
        inst, dof = spec["instances"], spec["dof_per_instance"]
        total_inst += inst
        total_dof += inst * dof
        mass = sum(v for _, v, _ in rows) * PETG_G_PER_MM3
        bad = sum(b for _, _, b in rows)
        print("  %-9s segments %d joints %d  instances %2d  DOF %2d  solid mass %6.0f g/instance  non-manifold %d" %
              (n, len(spec["segments"]), len(spec["joints"]), inst, inst * dof, mass, bad))
        for name, v, b in rows:
            print("      %-22s %8.0f mm^3  non-manifold %d" % (name, v, b))
    print("chain instances %d, actuated DOF on knuckle chains %d (of 50; the other 8 are ring joints, 4 face, 2 whiffletree)" %
          (total_inst, total_dof))


if __name__ == "__main__":
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    out = argv[argv.index("--out") + 1] if "--out" in argv else "cad/out"
    only = argv[argv.index("--preset") + 1] if "--preset" in argv else None
    main(os.path.abspath(out), only)
