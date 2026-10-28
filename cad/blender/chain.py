"""
chain.py - the knuckle-chain primitive: segments joined by fork-and-tongue pin joints, driven by
tendons in lined channels. One generator, many presets (P-007, extended at P-008).

A chain is a list of segments and the joints between them. Every joint is a fork (two lugs) on one
segment and a tongue (centre lug) on the other, on a pin or on bearings. The tongue lug doubles as
the joint pulley: for a single-tendon joint the elastic rides over it; for a closed-loop joint it
carries a line groove and a pocket for the crimped stop bead. Variations "sewn in":

  axis       'y' = pitch joint (lugs across the width), 'z' = yaw joint (lugs across the height)
  off        axis offset from the section centreline (asymmetric moment arms)
  fit        {'type': 'pin', ...} or {'type': 'bearing', 'od', 'w', 'bore'} (seats in the fork lugs,
             stub axles into the tongue, radial M3 insert to lock each stub, a printed retaining cap)
  tongue_on  'prox' (tendon passes through: fingers, toes) / 'dist' (loop ends on the moving tongue)
  loop       None, or {'d': line diameter} -> groove + bead pocket on the tongue lug
  rom        (lo, hi) degrees of the distal segment relative to the proximal one, positive toward +Z
             (axis y) or +Y (axis z) -> printed hard stops: the tongue lug grows to just under the
             other body's face over the forbidden arc, so the body face is the stop (homing drives
             into it, D-014)
  chamfer    clearance chamfer legs on the faces the joint swings toward

Segment extras: shell (wall thickness; hollow body with tubes left around every channel), pads
(M3 insert holes on the dorsal face for winch-bank or shell mounting), split_at (cut into bed-sized
pieces with alignment pin holes), mounts, tip, channels (liners, PTFE sockets, knot pockets).

Frame per segment: X proximal -> distal, Z dorsal/up, Y across. Units mm. Each preset exports one
STL per piece, in place (straight chain), plus a report.

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
WEB_OVERLAP = 2.0   # how far a lug web reaches into the solid body (a robust union, not a coplanar sliver)
INSERT_D = 4.2      # M3 heat-set insert hole
BED = 240.0         # largest printable length


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


SPIN = math.radians(3.7)   # half-step twist for cuts coaxial with another feature, so no vertex rays coincide


def cyl(name, axis, r, a0, a1, c1, c2, segments=SEG, spin=0.0):
    """Cylinder along axis from a0 to a1; (c1, c2) are the other two coordinates in x,y,z order.
    spin rotates the vertex ring about the cylinder's own axis (use it for anything coaxial with
    another cylinder: sockets around channels, bores inside lugs, grooves on lugs)."""
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
    if spin:
        bm = bmesh.new()
        bm.from_mesh(o.data)
        import mathutils
        bmesh.ops.rotate(bm, verts=bm.verts, cent=(0, 0, 0), matrix=mathutils.Matrix.Rotation(spin, 3, "Z"))
        bm.to_mesh(o.data)
        bm.free()
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


def sector_prism(name, axis, cx, off, r_in, r_out, phi0, phi1, a0, a1):
    """Annular sector about a lug axis ('y' or 'z') through (x=cx, perpendicular offset=off), angles in
    degrees from +X toward +Z (axis y) or +Y (axis z), extruded along the lug axis from a0 to a1."""
    n = max(6, int(abs(phi1 - phi0) / 3.1) + 1)   # 3.1-degree steps: never on the lug arc's vertex rays
    pts = []
    for i in range(n):
        p = math.radians(phi0 + (phi1 - phi0) * i / (n - 1))
        pts.append((cx + r_out * math.cos(p), off + r_out * math.sin(p)))
    for i in reversed(range(n)):
        p = math.radians(phi0 + (phi1 - phi0) * i / (n - 1))
        pts.append((cx + r_in * math.cos(p), off + r_in * math.sin(p)))
    bm = bmesh.new()
    if axis == "y":
        verts = [bm.verts.new((x, a0, v)) for (x, v) in pts]
        vec = (0, a1 - a0, 0)
    else:
        verts = [bm.verts.new((x, v, a0)) for (x, v) in pts]
        vec = (0, 0, a1 - a0)
    face = bm.faces.new(verts)
    res = bmesh.ops.extrude_face_region(bm, geom=[face])
    moved = [g for g in res["geom"] if isinstance(g, bmesh.types.BMVert)]
    bmesh.ops.translate(bm, verts=moved, vec=vec)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    o = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(o)
    _activate(o)
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


def intersect(t, tool):
    return boolean(t, tool, "INTERSECT")


def duplicate(obj, name):
    o = obj.copy()
    o.data = obj.data.copy()
    o.name = name
    bpy.context.scene.collection.objects.link(o)
    return o


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
    """Lateral extents (along the lug axis) of the fork lugs and the tongue."""
    full = W if joint["axis"] == "y" else H
    lw, c = joint["lug_w"], joint.get("clear", 0.3)
    fork = [(-full / 2, -full / 2 + lw), (full / 2 - lw, full / 2)]
    tongue = (-full / 2 + lw + c, full / 2 - lw - c)
    return fork, tongue


def lug_cyl(name, joint, x_axis, a0, a1, r=None, segments=47):
    """A cylinder coaxial with the joint's lug: always spun and odd-count so its rays never coincide with
    the lug profile's arc vertices."""
    r = joint["lug_r"] if r is None else r
    return cyl(name, "y" if joint["axis"] == "y" else "z", r, a0, a1, x_axis, joint["off"], segments, spin=SPIN)


def lug_prism(name, joint, x_axis, body_x, a0, a1, W, H):
    """Lug and web as ONE extruded profile: a circle of lug_r about the axis merged with the rectangle that
    reaches WEB_OVERLAP into the body. One mesh means no coplanar unions between lug, web and body faces.
    Faces that would be flush with the section are inset 0.2 mm for the same reason."""
    r, off = joint["lug_r"], joint["off"]
    half = H / 2 if joint["axis"] == "y" else W / 2          # section half-extent perpendicular to the lug axis
    full = W / 2 if joint["axis"] == "y" else H / 2          # section half-extent along the lug axis
    top = min(off + r, half - 0.2) if off + r >= half - 0.2 else off + r
    bot = max(off - r, -half + 0.2) if off - r <= -half + 0.2 else off - r
    a0 = a0 + 0.2 if abs(a0 + full) < 1e-6 else a0
    a1 = a1 - 0.2 if abs(a1 - full) < 1e-6 else a1
    toward = 1 if body_x > x_axis else -1                     # which side of the axis the body is on
    xb = body_x + toward * WEB_OVERLAP
    # arc on the side away from the body, from the top tangent point round to the bottom one
    a_top = math.asin(max(-1.0, min(1.0, (top - off) / r)))
    a_bot = math.asin(max(-1.0, min(1.0, (off - bot) / r)))
    if toward > 0:   # body at +X: arc runs through 180 deg
        th0, th1 = math.pi - a_top, math.pi + a_bot
    else:            # body at -X: arc runs through 0 deg
        th0, th1 = a_top, -a_bot
    n = 40
    pts = [(xb, top)]
    for i in range(n + 1):
        th = th0 + (th1 - th0) * i / n
        pts.append((x_axis + r * math.cos(th), off + r * math.sin(th)))
    pts.append((xb, bot))
    bm = bmesh.new()
    if joint["axis"] == "y":
        verts = [bm.verts.new((x, a0, v)) for (x, v) in pts]
        vec = (0, a1 - a0, 0)
    else:
        verts = [bm.verts.new((x, v, a0)) for (x, v) in pts]
        vec = (0, 0, a1 - a0)
    face = bm.faces.new(verts)
    res = bmesh.ops.extrude_face_region(bm, geom=[face])
    moved = [g for g in res["geom"] if isinstance(g, bmesh.types.BMVert)]
    bmesh.ops.translate(bm, verts=moved, vec=vec)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    o = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(o)
    _activate(o)
    return o


def stub_depth(fit):
    return fit["w"] if fit["type"] == "bearing" else 0.0


def jgap(joint):
    """Body-face clearance beyond the lug radius. Joints with hard stops get a larger gap so the stop
    sector that grows out of the lug has a real bearing face (2 mm) instead of a sliver."""
    return joint.get("gap", 2.5 if joint.get("rom") else 0.6)


def free_centre_half(joint, W, H):
    _, (t0, t1) = lug_span(joint, W, H)
    return (t1 - t0) / 2 - stub_depth(joint["fit"])


def add_fork(part, joint, x_axis, body_x, W, H):
    """Two lugs at x_axis bridged to the body face at body_x. Bearing fit: seats from the outer faces,
    2 mm shoulder, shaft hole, and two pilot holes per lug for the retaining cap."""
    fork, _ = lug_span(joint, W, H)
    for a0, a1 in fork:
        part = join(part, lug_prism("fork_lug", joint, x_axis, body_x, a0, a1, W, H))
    fit = joint["fit"]
    if fit["type"] == "bearing":
        for (a0, a1), outer in zip(fork, (-1, 1)):
            depth = fit["w"] + 0.1
            s0, s1 = (a0 - 1, a0 + depth) if outer < 0 else (a1 - depth, a1 + 1)
            part = cut(part, lug_cyl("seat", joint, x_axis, s0, s1, r=fit["od"] / 2 + 0.05))
            rp = fit["od"] / 2 + 2.5
            for sx in (-1, 1):   # cap screw pilots, 6 mm deep from the outer face
                p0, p1 = (a0 - 1, a0 + 6) if outer < 0 else (a1 - 6, a1 + 1)
                part = cut(part, cyl("cap_pilot", "y" if joint["axis"] == "y" else "z", 0.8, p0, p1,
                                     x_axis + sx * rp, joint["off"], 16))
        part = cut(part, lug_cyl("shaft", joint, x_axis, -max(W, H), max(W, H), r=fit["bore"] / 2 + 0.1))
    else:
        part = cut(part, lug_cyl("pin", joint, x_axis, -max(W, H), max(W, H), r=fit["press"] / 2))
    return part


def add_tongue(part, joint, x_axis, body_x, W, H):
    """Centre lug at x_axis bridged to the body face at body_x. Hard-stop sector over the forbidden arc,
    groove and bead pocket for loop joints, pin clearance hole or stub-axle blind bores with M3 locks."""
    _, (t0, t1) = lug_span(joint, W, H)
    part = join(part, lug_prism("tongue_lug", joint, x_axis, body_x, t0, t1, W, H))
    body_on_minus = body_x < x_axis    # this segment's body lies at -X of the axis (tongue at the distal end)
    # hard stops: the lug grows to just under the other body's face over the arc the joint may not enter
    if joint.get("rom"):
        lo, hi = joint["rom"]
        if body_on_minus:            # other segment's axis direction = theta
            phi0, phi1 = hi, lo + 360
        else:                        # other segment's direction = 180 - theta
            phi0, phi1 = 180 - lo, 540 - hi
        r_stop = joint["lug_r"] + jgap(joint) - 0.4
        # end faces pulled 0.37 deg inside the arc so they never lie on one of the lug's 7.5-deg vertex lines
        part = join(part, sector_prism("stop", joint["axis"], x_axis, joint["off"], joint["lug_r"] - 1.0,
                                       r_stop, phi0 + 0.37, phi1 - 0.37, t0 + 0.5, t1 - 0.5))
    if joint.get("loop"):
        d = joint["loop"]["d"]
        gr, gd = d / 2 + 0.3, d * 0.8
        r = joint["lug_r"]
        mid = (t0 + t1) / 2
        part = cut(part, lug_cyl("groove_out", joint, x_axis, mid - gr, mid + gr, r=r + 3))
        part = join(part, lug_cyl("groove_in", joint, x_axis, mid - gr - 0.01, mid + gr + 0.01, r=r - gd))
        pk = 2.5 * d + 2
        side = -1 if body_x > x_axis else 1
        if joint["axis"] == "y":
            part = cut(part, box("bead_pocket", x_axis + side * (r - pk), x_axis + side * (r + 3),
                                 mid - gr - 1, mid + gr + 1, joint["off"] - pk / 2, joint["off"] + pk / 2))
        else:
            part = cut(part, box("bead_pocket", x_axis + side * (r - pk), x_axis + side * (r + 3),
                                 joint["off"] - pk / 2, joint["off"] + pk / 2, mid - gr - 1, mid + gr + 1))
    fit = joint["fit"]
    if fit["type"] == "pin":
        part = cut(part, lug_cyl("tongue_hole", joint, x_axis, -max(W, H), max(W, H), r=fit["clear"] / 2))
    else:
        depth = stub_depth(fit)
        hr = fit["bore"] / 2 - 0.05
        for b0, b1, mid_b in ((t0 - 1, t0 + depth, t0 + depth / 2), (t1 - depth, t1 + 1, t1 - depth / 2)):
            part = cut(part, lug_cyl("stub_bore", joint, x_axis, b0, b1, r=hr))
            # radial M3 insert hole from the outer face into the bore: a grub screw locks the stub
            if joint["axis"] == "y":
                part = cut(part, cyl("stub_lock", "z", INSERT_D / 2, joint["off"], H / 2 + 1, x_axis, mid_b, 19, spin=SPIN))
            else:
                part = cut(part, cyl("stub_lock", "y", INSERT_D / 2, joint["off"], W / 2 + 1, x_axis, mid_b, 19, spin=SPIN))
    return part


def make_bearing_cap(spec, fit):
    """Retaining cap for a bearing lug: a disc with a lip, two screw holes at the pilot radius."""
    name = "%s_bearing_cap" % spec["name"]
    rp = fit["od"] / 2 + 2.5
    cap = cyl(name, "y", rp + 3.0, 0.0, 2.0, 0.0, 0.0)
    cap = join(cap, cyl("lip", "y", fit["od"] / 2 - 0.3, -1.0, 0.01, 0.0, 0.0))        # presses the outer race
    cap = cut(cap, cyl("shaft", "y", fit["bore"] / 2 + 1.0, -2, 3, 0.0, 0.0))
    for sx in (-1, 1):
        cap = cut(cap, cyl("screw", "y", 1.1, -2, 3, sx * rp, 0.0, 16))
    return cap


def chamfer_edge(part, x_corner, side, which, c, W, H):
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
    if joint["axis"] == "y":
        return [("z", -1)] if single_tendon else [("z", -1), ("z", 1)]
    return [("y", -1), ("y", 1)]


def mouth_inset(ch, chamfers, c, W, H):
    inset = 0.0
    for which, side in chamfers:
        coord, half = (ch["z"], H / 2) if which == "z" else (ch["y"], W / 2)
        inset = max(inset, c - (half - side * coord))
    return max(inset, 0.0)


# ----------------------------------------------------------------------------- a segment
def build_segment(spec, i, x_start):
    W, H = spec["W"], spec["H"]
    seg = spec["segments"][i]
    L = seg["L"]
    jp = spec["joints"][i - 1] if i > 0 else None
    jd = spec["joints"][i] if i < len(spec["joints"]) else None
    single = spec.get("single_tendon", False)

    x0 = (jp["lug_r"] + jgap(jp)) if jp else 0.0
    x1 = (L - jd["lug_r"] - jgap(jd)) if jd else L
    name = "%s_%s" % (spec["name"], seg["name"])
    part = box(name, x0, x1, -W / 2, W / 2, -H / 2, H / 2)
    for x in seg.get("mounts", []):
        part = cut(part, cyl("mount", "z", 1.6, -H, H, x, 0.0, segments=36))
    if seg.get("tip"):
        part = join(part, cyl("tip", "y", H / 2, -W / 2, W / 2, x1, 0.0))
    # off-the-shelf pipe socket at a free end (P-009): the long bone between two printed joint ends is a
    # bought tube (Schedule 40 PVC/ABS, conduit...). Bore = pipe OD + 0.4, two radial M3 inserts as set screws.
    pipe = seg.get("pipe")
    pipe_depth = 0.0
    if pipe:
        od, pipe_depth = pipe["od"], pipe.get("depth", 40.0)
        if min(W, H) < od + 0.4 + 6.0:
            raise ValueError("%s: pipe OD %.1f needs a section >= %.1f" % (name, od, od + 6.4))
        at_prox = (jp is None)
        if at_prox:
            xs, xe = -1.0, pipe_depth
            screws = (8.0, pipe_depth - 8.0)
        else:
            xs, xe = L - pipe_depth, L + 1.0
            screws = (L - 8.0, L - pipe_depth + 8.0)
        part = cut(part, cyl("pipe_socket", "x", od / 2 + 0.2, xs, xe, 0.0, 0.0, segments=45, spin=SPIN))
        for sx in screws:
            part = cut(part, cyl("pipe_screw", "z", INSERT_D / 2, od / 2 - 1.0, H / 2 + 1, sx, 0.0, 19, spin=SPIN))

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

    # hollow shell: void inside the body, tubes left around every channel and mount, pads stay solid
    t = seg.get("shell", spec.get("shell", 0.0))
    chans = seg.get("channels", [])
    # the void starts behind the chamfer zone at each end so the shell is never breached by a chamfer
    end_p = max(t, jp["chamfer"] if jp else 0.0) if t else 0.0
    end_d = max(t, jd["chamfer"] if jd else 0.0) if t else 0.0
    if pipe and t:   # keep the void clear of the pipe socket
        if jp is None:
            end_p = max(end_p, pipe_depth + 3.0)
        else:
            end_d = max(end_d, pipe_depth + 3.0)
    if t and (x1 - x0) > end_p + end_d + 2 * t:
        vx0, vx1 = x0 + end_p, x1 - end_d
        part = cut(part, box("void", vx0, vx1, -W / 2 + t, W / 2 - t, -H / 2 + t, H / 2 - t))
        for ch in chans:
            part = join(part, cyl("tube_" + ch["name"], "x", ch["d"] / 2 + 1.6, vx0 - 0.5, vx1 + 0.5, ch["y"], ch["z"],
                                  segments=43, spin=SPIN))
        for x in seg.get("mounts", []):
            part = join(part, cyl("boss", "z", 3.6, -H / 2 + t - 0.5, H / 2 - t + 0.5, x, 0.0, 36))
            part = cut(part, cyl("mount", "z", 1.6, -H, H, x, 0.0, 36))
        for (px, py) in seg.get("pads", []):
            part = join(part, cyl("pad_boss", "z", INSERT_D / 2 + 2.0, H / 2 - t - 8.0, H / 2 - t + 0.5, px, py, 24))

    # channels
    for ch in chans:
        r = ch["d"] / 2
        xe = ch.get("x_end", L + H)
        xs = ch.get("x_start", -1.0)
        part = cut(part, cyl("ch_" + ch["name"], "x", r, xs, xe, ch["y"], ch["z"]))
        for face, cham, c, facing in ((x0, cham_p, jp["chamfer"] if jp else 0, -1), (x1, cham_d, jd["chamfer"] if jd else 0, +1)):
            if (facing < 0 and xs > face) or (facing > 0 and xe < face):
                continue
            inset = mouth_inset(ch, cham, c, W, H)
            if ch.get("socket_at") == ("prox" if facing < 0 else "dist"):
                sr = (TUBE_OD + TUBE_CLEAR) / 2
                s0, s1 = (face - 1, face + TUBE_SOCKET) if facing < 0 else (face - TUBE_SOCKET, face + 1)
                part = cut(part, cyl("socket", "x", sr, s0, s1, ch["y"], ch["z"], segments=41, spin=SPIN))
            elif inset > 0:
                # an icosphere (no poles, no rings coaxial with the channel), centred 0.3 mm outside the
                # chamfer plane and turned by odd angles so no vertex lands on the plane or on an axis
                bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=2, radius=r + 1.0,
                                                      location=(face - facing * inset + facing * 0.3, ch["y"], ch["z"]),
                                                      rotation=(math.radians(7), math.radians(13), math.radians(29)))
                sph = bpy.context.active_object
                sph.name = "mouth_dimple"
                _activate(sph)
                bpy.ops.object.transform_apply(rotation=True)
                part = cut(part, sph)
            else:
                part = cut(part, cone_x("mouth", r + 1.0, r, 1.2, face, facing, ch["y"], ch["z"]))
        if ch.get("pocket"):
            zs = -H if ch["z"] < 0 else H
            part = cut(part, cyl("pocket", "z", 1.75, min(zs, ch["z"] + math.copysign(0.6, ch["z"])),
                                 max(zs, ch["z"] + math.copysign(0.6, ch["z"])), xe, ch["y"]))
    # winch-bank / shell mounting pads: M3 insert holes from the dorsal face
    for (px, py) in seg.get("pads", []):
        part = cut(part, cyl("pad", "z", INSERT_D / 2, H / 2 - 7.0, H / 2 + 1, px, py, 24))

    # split into bed-sized pieces with alignment pin holes
    pieces = []
    cuts = list(seg.get("split_at", []))
    if not cuts and L > BED:
        cuts = [L * (k + 1) / math.ceil(L / BED) for k in range(math.ceil(L / BED) - 1)]
    if cuts:
        bounds = [x0 - 1] + cuts + [L + H]
        for k in range(len(bounds) - 1):
            piece = duplicate(part, "%s_p%d" % (name, k + 1))
            piece = intersect(piece, box("keep", bounds[k], bounds[k + 1], -W, W, -H - 10, H + 10))
            for xc in cuts:
                for sy in (-1, 1):   # 3 mm alignment pins, 10 mm into each side of every cut
                    piece = cut(piece, cyl("align", "x", 1.55, xc - 10, xc + 10, sy * W / 4, 0.0, 20))
            pieces.append(piece)
        bpy.data.objects.remove(part, do_unlink=True)
    else:
        pieces = [part]
    for p in pieces:
        translate(p, x_start)
    return pieces


def build_chain(spec, out_dir):
    x = 0.0
    parts = []
    for i in range(len(spec["segments"])):
        parts.extend(build_segment(spec, i, x))
        x += spec["segments"][i]["L"]
    caps = {}
    for j in spec["joints"]:
        if j["fit"]["type"] == "bearing":
            caps[j["fit"]["od"]] = j["fit"]
    for fit in caps.values():
        parts.append(make_bearing_cap(spec, fit))
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


def J(axis, lug_r, lug_w, fit, tongue_on, off=0.0, loop=None, chamfer=6.0, rom=None):
    return {"axis": axis, "lug_r": lug_r, "lug_w": lug_w, "fit": fit, "tongue_on": tongue_on,
            "off": off, "loop": loop, "chamfer": chamfer, "rom": rom}


def loop_channels(prefix, axis, lug_r, d, liner_od, W, H, **kw):
    bore = liner_od + 0.1
    o = lug_r - d * 0.8 - d / 2 + 0.3
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
            J("y", 4.5 * s, 4.0 * s, pin(1.9, 2.3), "prox", off=5.0 * s, rom=(-90, 10)),
            J("y", 4.5 * s, 4.0 * s, pin(1.9, 2.3), "prox", off=2.5 * s, rom=(-105, 0)),
            J("y", 4.5 * s, 4.0 * s, pin(1.9, 2.3), "prox", off=0.5 * s, rom=(-70, 0)),
        ],
    }


def thumb_preset():
    """Thumb column: opposition knuckle (yaw, at the palm), then MCP and IP (pitch)."""
    s = 1.1
    W = H = 16 * s
    # flexor/extensor at +-3 mm so they pass through the opposition joint's tongue (|z| < 4), clear of its
    # offset pin; the extensor still rides over each knuckle lug, which sets its moment arm
    flex = dict(name="flexor", y=0.0, z=-2.8, d=1.75)
    ext = dict(name="extensor", y=0.0, z=2.8, d=2.0)       # top tangent at 3.8, clear of the tongue face at 4.0
    opp = dict(name="opposition", y=-5.0, z=0.0, d=1.75)       # moment arm = 4 (axis offset) + 5 = 9 mm
    opp_ret = dict(name="opp_return", y=5.0, z=0.0, d=2.0)
    return {
        "name": "thumb", "W": W, "H": H, "single_tendon": True, "instances": 2, "dof_per_instance": 2,
        "segments": [
            dict(name="palm", L=30, channels=[dict(flex, socket_at="prox"), dict(ext), dict(opp, socket_at="prox"), dict(opp_ret)],
                 mounts=[10.5, 16.3]),
            dict(name="cmc", L=24, channels=[dict(flex), dict(ext), dict(opp, x_end=17, pocket=True), dict(opp_ret, x_end=17, pocket=True)]),
            dict(name="proximal", L=40, channels=[dict(flex), dict(ext)]),
            dict(name="distal", L=30, tip=True, channels=[dict(flex, x_end=23, pocket=True), dict(ext, x_end=23, pocket=True)]),
        ],
        "joints": [
            J("z", 5.0, 4.5, pin(1.9, 2.3), "prox", off=4.0, rom=(0, 60)),
            J("y", 5.0, 4.4, pin(1.9, 2.3), "prox", off=5.5, rom=(-55, 10)),
            J("y", 5.0, 4.4, pin(1.9, 2.3), "prox", off=1.0, rom=(-75, 0)),
        ],
    }


def toe_preset():
    return {
        "name": "toe", "W": 90, "H": 14, "single_tendon": True, "instances": 2, "dof_per_instance": 0,
        "segments": [dict(name="sole", L=40, channels=[dict(name="elastic", y=0.0, z=4.0, d=2.2)]),
                     dict(name="plate", L=50, tip=True, channels=[dict(name="elastic", y=0.0, z=4.0, d=2.2, x_end=40, pocket=True)])],
        "joints": [J("y", 5.0, 12.0, pin(2.9, 3.3), "prox", off=2.0, chamfer=5.0, rom=(-10, 70))],
    }


PIPE = {   # Schedule 40 PVC / ABS DWV outside diameters, mm (also sold as 'nominal' sizes)
    "3/4": 26.7, "1": 33.4, "1-1/4": 42.2, "1-1/2": 48.3, "2": 60.3,
}


def universal_preset(name, W, H, lug_r, lug_w, fit, line_d, liner_od, rom1, rom2, L=(40, 30, 40),
                     instances=2, chamfer=8.0, shell=0.0, pads=(), pipe_base=None, pipe_end=None):
    """Two loop joints with axes at 90 degrees. Wrist, ankle, neck, waist, and the two knuckle axes of
    hip and shoulder. pipe_base / pipe_end: bought-tube socket at the free end of the base / end segment."""
    j1 = J("y", lug_r, lug_w, fit, "dist", loop={"d": line_d}, chamfer=chamfer, rom=rom1)
    j2 = J("z", lug_r, lug_w, fit, "dist", loop={"d": line_d}, chamfer=chamfer, rom=rom2)
    base_ch = loop_channels("pitch", "y", lug_r, line_d, liner_od, W, H, socket_at="dist") + \
        loop_channels("yaw", "z", lug_r, line_d, liner_od, W, H)
    mid_ch = loop_channels("yaw", "z", lug_r, line_d, liner_od, W, H, socket_at="dist")
    need = abs(mid_ch[0]["y"]) + mid_ch[0]["d"] / 2 + 1.0
    have = free_centre_half(j1, W, H)
    if have < need:
        raise ValueError("%s: yaw runs need +-%.1f mm of free tongue centre, have %.1f; widen W" % (name, need, have))
    base = dict(name="base", L=L[0], channels=base_ch, pads=list(pads))
    end = dict(name="end", L=L[2])
    if pipe_base:
        base["pipe"] = {"od": PIPE[pipe_base]}
    if pipe_end:
        end["pipe"] = {"od": PIPE[pipe_end]}
    return {
        "name": name, "W": W, "H": H, "instances": instances, "dof_per_instance": 2, "shell": shell,
        "segments": [base, dict(name="cross", L=L[1], channels=mid_ch, shell=0.0), end],
        "joints": [j1, j2],
    }


def hinge_preset(name, W, H, lug_r, lug_w, fit, line_d, liner_od, rom, L=(60, 60), instances=2, chamfer=10.0,
                 shell=0.0, pads=(), split_at=(), pipe_upper=None, pipe_lower=None):
    j = J("y", lug_r, lug_w, fit, "dist", loop={"d": line_d}, chamfer=chamfer, rom=rom)
    upper = dict(name="upper", L=L[0], pads=list(pads),
                 channels=loop_channels("flex", "y", lug_r, line_d, liner_od, W, H, socket_at="dist"))
    lower = dict(name="lower", L=L[1], split_at=list(split_at))
    if pipe_upper:
        upper["pipe"] = {"od": PIPE[pipe_upper]}
    if pipe_lower:
        lower["pipe"] = {"od": PIPE[pipe_lower]}
    return {
        "name": name, "W": W, "H": H, "instances": instances, "dof_per_instance": 1, "shell": shell,
        "segments": [upper, lower],
        "joints": [j],
    }


def grid(x0, nx, y0, ny, pitch=20.0):
    return [(x0 + i * pitch, y0 + j * pitch) for i in range(nx) for j in range(ny)]


PRESETS = {
    "finger": finger_preset,
    "thumb": thumb_preset,
    "toe": toe_preset,
    # Long bones between printed joint ends are bought pipe (P-009): forearm and humerus 1" (33.4 mm),
    # tibia 1-1/4" (42.2), femur 1-1/2" (48.3), spine 2" (60.3). Sockets sit at the free ends.
    "wrist": lambda: universal_preset("wrist", 66, 42, 18, 7.5, BEARING_625, 0.55, 2.11, (-70, 60), (-25, 15), chamfer=8, shell=3.0,
                                      pipe_base="1"),
    "ankle": lambda: universal_preset("ankle", 76, 60, 18, 9.5, BEARING_608, 0.75, 2.77, (-40, 20), (-20, 15),
                                      L=(60, 40, 50), chamfer=12, shell=3.0, pipe_base="1-1/4"),
    "neck": lambda: universal_preset("neck", 80, 52, 25, 7.5, BEARING_625, 0.48, 1.65, (-50, 50), (-35, 35),
                                     L=(50, 30, 40), instances=1, chamfer=6, pipe_base="1"),
    "waist": lambda: universal_preset("waist", 120, 80, 30, 9.5, BEARING_608, 0.75, 2.77, (-20, 40), (-25, 25),
                                      L=(60, 60, 60), instances=1, chamfer=16, shell=3.0, pads=grid(15, 2, -20, 3),
                                      pipe_end="2"),
    "elbow": lambda: hinge_preset("elbow", 60, 70, 30, 9.5, BEARING_608, 0.55, 2.11, (-140, 0), L=(80, 60),
                                  chamfer=10, shell=3.0, pads=grid(15, 2, -10, 2), pipe_upper="1", pipe_lower="1"),
    "knee": lambda: hinge_preset("knee", 80, 110, 50, 9.5, BEARING_608, 0.75, 2.77, (-135, 0), L=(80, 70),
                                 chamfer=14, shell=3.0, pads=grid(10, 2, -20, 3), pipe_upper="1-1/2", pipe_lower="1-1/4"),
    "hip": lambda: universal_preset("hip", 82, 80, 20, 9.5, BEARING_608, 0.75, 2.77, (-110, 15), (-25, 40),
                                    L=(60, 40, 60), chamfer=12, shell=3.0, pads=grid(10, 2, -20, 3), pipe_end="1-1/2"),
    "shoulder": lambda: universal_preset("shoulder", 76, 70, 18, 9.5, BEARING_608, 0.75, 2.77, (-170, 45), (-30, 165),
                                         L=(50, 40, 50), chamfer=10, shell=3.0, pads=grid(10, 2, -20, 3), pipe_end="1"),
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
        print("  %-9s pieces %d joints %d  instances %2d  DOF %2d  mass %6.0f g/instance (shell %s)  non-manifold %d" %
              (n, len(parts), len(spec["joints"]), inst, inst * dof, mass, spec.get("shell") or "solid", bad))
        for name, v, b in rows:
            print("      %-26s %8.0f mm^3  non-manifold %d" % (name, v, b))
    print("chain instances %d, actuated DOF on knuckle chains %d (ring joints add 8, face 4, whiffletree 2)" %
          (total_inst, total_dof))


if __name__ == "__main__":
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    out = argv[argv.index("--out") + 1] if "--out" in argv else "cad/out"
    only = argv[argv.index("--preset") + 1] if "--preset" in argv else None
    main(os.path.abspath(out), only)
