"""
ring.py - the ring joint primitive: rotation of a segment about its own axis (P-008).

Covers the eight axial DOF the knuckle chain cannot: forearm pronation/supination, shoulder and
hip internal/external rotation, neck yaw, waist yaw. Same treatment as chain.py: one generator,
presets with the variations sewn in.

Parts (axis along X, proximal at low X):
  hub   - fixed to the proximal bone: a tube with a mounting flange, the inner race (or a seat for a
          commercial bearing), a stop pin, and two sheath-socket bosses holding the tendon runs
          tangent to the rotor's drive groove. Hollow: the next joints' tendons and the wiring pass
          through the bore along the axis, where a twist of +-80 degrees costs almost no length.
  rotor - fixed to the distal bone: a ring with the outer race, a drive groove with a crimped-bead
          pocket (the closed loop wraps the rotor itself: the joint is its own pulley), an arc channel
          for the stop pin that sets the range of motion, a ball loading hole, and a mounting flange.
  cap   - (commercial-bearing variant only) retaining ring, two screws.

Bearing variants:
  {'type': 'balls', 'd': 6.0}        printed four-wall race running on airsoft BBs (6 mm, or 8 mm for
                                     the hip and waist): $3 per thousand, loaded through a plugged hole
  {'type': 'bearing', 'od', 'id', 'w'} a thin-section ball bearing (6808, 6810 class) in printed seats

Run headless:
    blender -b -P cad/blender/ring.py -- --out cad/out            (all presets)
    blender -b -P cad/blender/ring.py -- --out cad/out --preset forearm
"""
import copy
import math
import os
import sys

import bmesh
import bpy
from mathutils import Matrix

SEG = 64
PETG_G_PER_MM3 = 1.27e-3
TUBE_OD, TUBE_CLEAR, TUBE_SOCKET = 4.0, 0.3, 8.0
FLANGE_T = 4.0


# ----------------------------------------------------------------------------- primitives (as chain.py)
def _activate(obj):
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj


def box(name, x0, x1, y0, y1, z0, z1):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
    o = bpy.context.active_object
    o.name = name
    o.scale = (x1 - x0, y1 - y0, z1 - z0)
    _activate(o)
    bpy.ops.object.transform_apply(scale=True)
    return o


_SPIN_K = [0]


def cyl(name, axis, r, a0, a1, c1, c2, segments=SEG):
    """Every cylinder gets its own small twist about its axis (a running multiple of 3.7 degrees), so the
    many coaxial cylinders a ring joint is made of never share a vertex ray."""
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
    _SPIN_K[0] += 1
    spin = math.radians(3.7 * (_SPIN_K[0] % 11))
    if spin:
        bm = bmesh.new()
        bm.from_mesh(o.data)
        bmesh.ops.rotate(bm, verts=bm.verts, cent=(0, 0, 0), matrix=Matrix.Rotation(spin, 3, "Z"))
        bm.to_mesh(o.data)
        bm.free()
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


def annulus_cut(part, r_out, r_in, x0, x1, segments=SEG):
    """Remove the ring r_in..r_out between x0 and x1 (about the X axis)."""
    part = cut(part, cyl("ann_out", "x", r_out, x0, x1, 0.0, 0.0, segments))
    return join(part, cyl("ann_in", "x", r_in, x0 - 0.01, x1 + 0.01, 0.0, 0.0, segments))


def sector_prism(name, r_in, r_out, phi0, phi1, x0, x1, n=None):
    """One mesh: an annular sector (angles in degrees from +Z toward +Y) extruded along X."""
    n = n or max(6, int(abs(phi1 - phi0) / 3) + 1)
    bm = bmesh.new()
    pts = []
    for i in range(n):
        phi = math.radians(phi0 + (phi1 - phi0) * i / (n - 1))
        pts.append((r_out * math.sin(phi), r_out * math.cos(phi)))
    for i in reversed(range(n)):
        phi = math.radians(phi0 + (phi1 - phi0) * i / (n - 1))
        pts.append((r_in * math.sin(phi), r_in * math.cos(phi)))
    verts = [bm.verts.new((x0, y, z)) for (y, z) in pts]
    face = bm.faces.new(verts)
    res = bmesh.ops.extrude_face_region(bm, geom=[face])
    moved = [g for g in res["geom"] if isinstance(g, bmesh.types.BMVert)]
    bmesh.ops.translate(bm, verts=moved, vec=(x1 - x0, 0, 0))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    o = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(o)
    _activate(o)
    return o


def arc_channel(part, r_c, phi0, phi1, r_tool, x0, x1):
    """Cut a stop-pin track: an annular sector r_c +- r_tool over phi0..phi1, with round ends."""
    part = cut(part, sector_prism("track", r_c - r_tool, r_c + r_tool, phi0, phi1, x0, x1))
    for phi in (phi0, phi1):
        a = math.radians(phi)
        part = cut(part, cyl("track_end", "x", r_tool, x0, x1, r_c * math.sin(a), r_c * math.cos(a), 24))
    return part


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


# ----------------------------------------------------------------------------- layout
def layout(p):
    L = {}
    b = p["bearing"]
    L["R_ro"] = p["R_ro"]                      # rotor outer radius = drive radius
    if b["type"] == "balls":
        d = b["d"]
        L["R_h"] = p["R_h"]                    # hub outer radius
        L["R_ri"] = L["R_h"] + 0.4             # rotor inner radius (running clearance)
        L["ball_c"] = L["R_h"] + 0.2           # ball centre radius
        L["race_w"] = d + 0.3
        L["race_depth"] = d / 2 - 0.2          # into each part
        L["w_rotor"] = max(p["w_rotor"], d + 8)
    else:
        L["R_h"] = b["id"] / 2 - 0.03
        L["R_ri"] = b["od"] / 2 + 0.05
        L["w_rotor"] = max(p["w_rotor"], b["w"] + 6)
    L["x_r0"] = FLANGE_T + 1.0                 # rotor starts 1 mm clear of the hub flange
    L["x_r1"] = L["x_r0"] + L["w_rotor"]
    L["x_race"] = (L["x_r0"] + L["x_r1"]) / 2
    L["x_g"] = L["x_r1"] - 4.0                 # drive groove plane, near the distal end of the rotor
    L["hub_len"] = L["x_r1"] - 0.5                # ends just short of the rotor's distal flange
    L["R_f"] = max(1.35 * L["R_ro"], L["R_ro"] + 10)   # flange radius: reaches the socket bars
    if p.get("pipe_od"):                                # and clears the pipe socket boss plus its M3 ring
        L["R_f"] = max(L["R_f"], p["pipe_od"] / 2 + 3.4 + 8.0)
    L["gd"] = p["line_d"] * 0.8                # groove depth
    L["sr"] = (TUBE_OD + TUBE_CLEAR) / 2       # socket bore radius
    L["R_s"] = (L["R_ri"] + L["R_ro"]) / 2     # stop track radius
    return L


# ----------------------------------------------------------------------------- parts
def make_hub(p, L):
    name = p["name"] + "_hub"
    # the flange provides the proximal face; the hub tube starts inside it so no two faces are coplanar
    hub = cyl(name, "x", L["R_f"], 0.0, FLANGE_T, 0.0, 0.0)
    hub = join(hub, cyl("tube", "x", L["R_h"], FLANGE_T - 1.0, L["hub_len"], 0.0, 0.0))
    b = p["bearing"]
    if b["type"] == "balls":
        hub = annulus_cut(hub, L["R_h"] + 1, L["R_h"] - L["race_depth"],
                          L["x_race"] - L["race_w"] / 2, L["x_race"] + L["race_w"] / 2)
    else:
        # bearing seat: inner race sits on the hub between a shoulder and the flange side
        hub = annulus_cut(hub, L["R_h"] + 1, L["R_h"] - 1.5, L["x_race"] + b["w"] / 2 + 0.1, L["hub_len"] + 1)
    # stop pin on the flange face at the bottom (angle 180 from +Z), reaching into the rotor's track
    pin_r = p["stop_pin_r"]
    hub = join(hub, cyl("stop_pin", "x", pin_r, FLANGE_T - 0.01, L["x_r0"] + 3.0, 0.0, -L["R_s"]))
    # socket bosses: the two runs leave the rotor tangentially at the +-Y sides heading -Z;
    # each boss is a bar from the flange along X, carrying a Z-axis socket at y = +-R_g, below the rotor
    R_g = L["R_ro"] - L["gd"] + p["line_d"] / 2
    z_top = -0.7 * L["R_ro"]
    z_bot = z_top - TUBE_SOCKET - 4
    for s in (-1, 1):
        y = s * R_g
        hub = join(hub, box("bar", 0.01, L["x_g"] + L["sr"] + 1.5, y - (L["sr"] + 1.5), y + (L["sr"] + 1.5), z_bot, z_top))
        hub = cut(hub, cyl("socket", "z", L["sr"], z_bot - 1, z_bot + TUBE_SOCKET, L["x_g"], y))
        hub = cut(hub, cyl("line", "z", p["line_d"] / 2 + 0.4, z_bot + TUBE_SOCKET - 0.5, z_top + 1, L["x_g"], y))
    # bore for crossing tendons and wiring, and four M3 insert holes in the flange
    hub = cut(hub, cyl("bore", "x", p["R_b"], -1, L["hub_len"] + 1, 0.0, 0.0))
    # M3 insert holes at top and both sides (the bottom carries the stop pin and the socket bars)
    rr = L["R_f"] - 5
    for a in (0.0, math.pi / 2, -math.pi / 2):
        hub = cut(hub, cyl("m3", "x", 2.0, -1, FLANGE_T + 1, rr * math.sin(a), rr * math.cos(a), 24))
    hub = add_pipe_socket(hub, p, L, proximal=True)
    return hub


def add_pipe_socket(part, p, L, proximal):
    """Boss on the flange's free face that takes a bought pipe (P-009): bore = pipe OD + 0.4, 3 mm wall,
    30 mm deep, two radial M3 insert holes as set screws. The pipe butts on the flange; tendons and
    wires continue through the hub bore."""
    od = p.get("pipe_od")
    if not od:
        return part
    r_in, r_out, depth = od / 2 + 0.2, od / 2 + 3.4, 30.0
    if proximal:
        x0, x1 = -depth, 1.0
        xs = (-8.0, -depth + 8.0)
    else:
        x0, x1 = L["x_r1"] + FLANGE_T - 1.0, L["x_r1"] + FLANGE_T + depth
        xs = (L["x_r1"] + FLANGE_T + 8.0, L["x_r1"] + FLANGE_T + depth - 8.0)
    part = join(part, cyl("pipe_boss", "x", r_out, x0, x1, 0.0, 0.0))
    part = cut(part, cyl("pipe_bore", "x", r_in, x0 - 1, x1 - 1.0 if proximal else x1 + 1, 0.0, 0.0))
    for sx in xs:
        part = cut(part, cyl("pipe_screw", "z", 2.1, r_in - 1.0, r_out + 1.0, sx, 0.0, 19))
    return part


def make_rotor(p, L):
    name = p["name"] + "_rotor"
    rot = cyl(name, "x", L["R_ro"], L["x_r0"], L["x_r1"], 0.0, 0.0)
    rot = join(rot, cyl("flange", "x", L["R_f"], L["x_r1"] - 0.01, L["x_r1"] + FLANGE_T, 0.0, 0.0))
    b = p["bearing"]
    # the rotor body is bored to the race; the distal flange keeps only the tendon bore (R_b), so a pipe
    # in the distal socket butts on the flange instead of sliding into the rotor
    if b["type"] == "balls":
        rot = cut(rot, cyl("id", "x", L["R_ri"], L["x_r0"] - 1, L["x_r1"] + 0.01, 0.0, 0.0))
        # outer race: a groove in the bore (a plain cut; the bore is already open)
        rot = cut(rot, cyl("race", "x", L["R_ri"] + L["race_depth"],
                           L["x_race"] - L["race_w"] / 2, L["x_race"] + L["race_w"] / 2, 0.0, 0.0))
        # ball loading hole at the bottom, plugged after filling (a short M4 grub screw or a glued pin)
        rot = cut(rot, cyl("load_hole", "z", b["d"] / 2 + 0.3, -L["R_ro"] - 1, 0.0, L["x_race"], 0.0, 24))
    else:
        rot = cut(rot, cyl("id", "x", p["R_b"] + 3, L["x_r0"] - 1, L["x_r1"] + 0.01, 0.0, 0.0))
        # bearing seat from the proximal face up to a shoulder
        rot = cut(rot, cyl("seat", "x", L["R_ri"], L["x_r0"] - 1, L["x_race"] + b["w"] / 2 + 0.1, 0.0, 0.0))
        for k in range(2):   # cap screw pilots on the proximal face
            a = math.radians(90 + 180 * k)
            rr = L["R_ri"] + 3
            rot = cut(rot, cyl("pilot", "x", 0.8, L["x_r0"] - 1, L["x_r0"] + 6, rr * math.sin(a), rr * math.cos(a), 16))
    rot = cut(rot, cyl("flange_bore", "x", p["R_b"], L["x_r1"] - 1, L["x_r1"] + FLANGE_T + 1, 0.0, 0.0))
    rot = add_pipe_socket(rot, p, L, proximal=False)
    # drive groove and bead pocket (pocket at the top, mid-rotation)
    gr = p["line_d"] / 2 + 0.3
    rot = annulus_cut(rot, L["R_ro"] + 1, L["R_ro"] - L["gd"], L["x_g"] - gr, L["x_g"] + gr)
    pk = 2.5 * p["line_d"] + 2
    rot = cut(rot, box("bead_pocket", L["x_g"] - gr - 1, L["x_g"] + gr + 1, -pk / 2, pk / 2, L["R_ro"] - pk, L["R_ro"] + 1))
    # stop track on the proximal face: the hub's pin sits at 180 deg; the track spans 180 +- (ROM + pin)
    half = p["rom_deg"] + math.degrees(math.atan2(p["stop_pin_r"] + 0.3, L["R_s"]))
    rot = arc_channel(rot, L["R_s"], 180 - half, 180 + half, p["stop_pin_r"] + 0.3, L["x_r0"] - 1, L["x_r0"] + 3.3)
    # four M3 insert holes in the distal flange
    for k in range(4):
        a = math.radians(45 + 90 * k)
        rr = L["R_f"] - 6
        rot = cut(rot, cyl("m3", "x", 2.0, L["x_r1"] - 1, L["x_r1"] + FLANGE_T + 1, rr * math.sin(a), rr * math.cos(a), 24))
    return rot


def make_cap(p, L):
    """Retaining ring for the commercial-bearing variant: holds the outer race in the rotor seat."""
    b = p["bearing"]
    name = p["name"] + "_cap"
    cap = cyl(name, "x", L["R_ri"] + 6, 0.0, 2.0, 0.0, 0.0)
    cap = cut(cap, cyl("hole", "x", b["od"] / 2 - 2.0, -1, 3, 0.0, 0.0))
    for k in range(2):
        a = math.radians(90 + 180 * k)
        rr = L["R_ri"] + 3
        cap = cut(cap, cyl("screw", "x", 1.1, -1, 3, rr * math.sin(a), rr * math.cos(a), 16))
    return cap


# ----------------------------------------------------------------------------- presets
def P(name, R_ro, R_h, R_b, w_rotor, rom_deg, line_d, bearing, instances, stop_pin_r=2.0, pipe_od=None):
    return dict(name=name, R_ro=R_ro, R_h=R_h, R_b=R_b, w_rotor=w_rotor, rom_deg=rom_deg, line_d=line_d,
                bearing=bearing, instances=instances, stop_pin_r=stop_pin_r, pipe_od=pipe_od)


BALLS6 = {"type": "balls", "d": 6.0}
BALLS8 = {"type": "balls", "d": 8.0}
BEARING_6808 = {"type": "bearing", "od": 52.0, "id": 40.0, "w": 7.0}
PIPE = {"3/4": 26.7, "1": 33.4, "1-1/4": 42.2, "1-1/2": 48.3, "2": 60.3}   # Schedule 40 PVC/ABS OD, mm

PRESETS = {
    # pipe_od: bought-tube sockets on both flanges (forearm/humerus 1", femur 1-1/2", spine 2")
    "forearm":      lambda: P("forearm", 28, 18, 12, 14, 80, 0.55, BALLS6, 2, pipe_od=PIPE["1"]),
    "forearm_6808": lambda: P("forearm_6808", 32, 20, 14, 14, 80, 0.55, BEARING_6808, 0, pipe_od=PIPE["1"]),
    "shoulder_rot": lambda: P("shoulder_rot", 42, 30, 20, 18, 80, 0.75, BALLS6, 2, pipe_od=PIPE["1"]),
    "hip_rot":      lambda: P("hip_rot", 50, 35, 22, 22, 35, 0.75, BALLS8, 2, stop_pin_r=3.0, pipe_od=PIPE["1-1/2"]),
    "neck_yaw":     lambda: P("neck_yaw", 25, 15, 9, 12, 80, 0.48, BALLS6, 1, pipe_od=PIPE["1"]),
    "waist_yaw":    lambda: P("waist_yaw", 60, 45, 30, 24, 45, 0.75, BALLS8, 1, stop_pin_r=3.0, pipe_od=PIPE["2"]),
}


def build(p, out_dir):
    L = layout(p)
    parts = [make_hub(p, L), make_rotor(p, L)]
    if p["bearing"]["type"] == "bearing":
        parts.append(make_cap(p, L))
    rows = []
    for o in parts:
        bad = cleanup(o)
        v = volume_mm3(o)
        export_stl(o, os.path.join(out_dir, o.name + ".stl"))
        rows.append((o.name, v, bad))
    balls = 0
    if p["bearing"]["type"] == "balls":
        balls = int(2 * math.pi * L["ball_c"] / (p["bearing"]["d"] + 0.2)) - 2   # leave two gaps for loading
    return L, rows, balls


def main(out_dir, only=None):
    os.makedirs(out_dir, exist_ok=True)
    names = [only] if only else list(PRESETS)
    print("\nring-joint presets")
    tot_inst = 0
    for n in names:
        bpy.ops.wm.read_factory_settings(use_empty=True)
        p = copy.deepcopy(PRESETS[n]())
        L, rows, balls = build(p, out_dir)
        mass = sum(v for _, v, _ in rows) * PETG_G_PER_MM3
        bad = sum(b for _, _, b in rows)
        tot_inst += p["instances"]
        print("  %-13s rotor OD %3.0f  bore %3.0f  width %2.0f  ROM +-%d  %s%s  instances %d  solid mass %4.0f g  non-manifold %d" %
              (n, 2 * L["R_ro"], 2 * p["R_b"], L["w_rotor"], p["rom_deg"],
               p["bearing"]["type"], (" x%d balls" % balls) if balls else "", p["instances"], mass, bad))
        for name, v, b in rows:
            print("      %-24s %8.0f mm^3  non-manifold %d" % (name, v, b))
    print("ring instances %d (8 DOF)" % tot_inst)


if __name__ == "__main__":
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    out = argv[argv.index("--out") + 1] if "--out" in argv else "cad/out"
    only = argv[argv.index("--preset") + 1] if "--preset" in argv else None
    main(os.path.abspath(out), only)
