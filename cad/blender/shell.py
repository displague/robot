"""
shell.py - the torso and pelvis as thin keyed shells (docs/35, D-059): the structure and the cover in one.

The idea (P-059) is a lifecaster's mother mold: a thin rigid shell in sections that part cleanly along planned lines
and register on keys. Here each body box is a rounded thin wall with an internal rib grid, split into panels that
print flat (outer face down, no supports), joined by an overlapping built-up edge:

- one panel carries a lap strip that slides inside its neighbour's wall with a print gap;
- hemispherical keys on the strip snap into dimples in the neighbour's wall (registration, no shifting);
- M3 screws from outside through the neighbour's wall into heat-set inserts in pads on the strip;
- bosses with heat-set inserts on the back face carry the drive banks, the battery and the electronics, which
  the rolled-up "frame" estimates (D-036) held before.

Frame: X across the body (left-right), Y front-back (+Y front), Z up, the box centred on the origin. Units mm.
Torso (250 mm wide, 10 under export_mjcf.SHELL, so each panel fits a 256 mm bed): split front/back (Y = 0) and
upper/lower, front panels carrying the Y seam's strips, lower panels the Z seam's. Pelvis: split left/right at the
midline, each half a tub that prints on its side; the left half carries the strip.

Run headless:
    blender -b -P cad/blender/shell.py -- --out cad/out [--preset torso|pelvis] [--wall 1.2]
"""
import math
import os
import sys

import bmesh
import bpy

P = {
    "wall": 1.2,          # shell wall (PETG; three 0.4 mm perimeters)
    "r": 25.0,            # outer corner and edge radius
    "seg": 6,             # bevel segments per rounded edge
    "seam_gap": 0.3,      # between neighbouring panels' walls
    "gap": 0.15,          # between a lap strip and the neighbour's wall
    "lap_w": 10.0,        # strip overlap past the seam
    "lap_t": 1.6,         # strip thickness
    "rib_t": 1.2,
    "rib_d": 6.0,         # rib depth inward from the wall
    "rib_pitch": 60.0,
    "key_r": 0.8,         # key bump radius (into a 0.95 mm dimple)
    "screw_pitch": 70.0,  # along each straight seam; keys halfway between screws
    "pad_d": 8.0, "pad_h": 3.5, "insert_d": 4.0, "insert_depth": 4.5,   # M3 heat-set insert, 4.0 x 4 mm
    "clear_d": 3.4,       # M3 clearance through the neighbour's wall
    "boss_d": 9.0, "boss_h": 7.0, "boss_pitch": 60.0,                    # drive-bank bosses on the back face
    "bed": (256.0, 256.0, 256.0),
}
PETG = 1.27e-3            # g per mm^3

# size (x, y, z) from export_mjcf.SHELL; holes: (face axis, side, (u, v) on the face, radius)
PRESETS = {
    "torso": dict(size=(250.0, 160.0, 400.0), seams=[("y", 0.0, -1), ("z", 0.0, +1)],
                  names=[("back", "front"), ("lower", "upper")],
                  holes=[("z", +1, (0.0, 0.0), 32.0),      # neck
                         ("z", -1, (0.0, 0.0), 46.0),      # waist joint
                         ("x", +1, (0.0, 130.0), 36.0),    # shoulders (u = y, v = z)
                         ("x", -1, (0.0, 130.0), 36.0)]),
    "pelvis": dict(size=(280.0, 160.0, 100.0), seams=[("x", 0.0, -1)], names=[("right", "left")],
                   holes=[("z", +1, (0.0, 0.0), 46.0),     # waist joint
                          ("z", -1, (90.0, 0.0), 36.0),    # hips (u = x, v = y)
                          ("z", -1, (-90.0, 0.0), 36.0)]),
}


# ----------------------------------------------------------------------------- helpers
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


def rbox(name, size, inset):
    """The rounded body box offset inward by `inset` (same centre, edge radius r - inset)."""
    sx, sy, sz = (s - 2 * inset for s in size)
    o = box(name, -sx / 2, sx / 2, -sy / 2, sy / 2, -sz / 2, sz / 2)
    m = o.modifiers.new("bevel", "BEVEL")
    m.width = P["r"] - inset
    m.segments = P["seg"]
    m.limit_method = "NONE"
    with bpy.context.temp_override(object=o, active_object=o, selected_objects=[o]):
        bpy.ops.object.modifier_apply(modifier=m.name)
    return o


def cyl(name, axis, r, a0, a1, c1, c2, segments=33):
    """Cylinder along axis from a0 to a1; (c1, c2) the other two coordinates in x, y, z order."""
    mid = (a0 + a1) / 2
    loc, rot = {"x": ((mid, c1, c2), (0, math.pi / 2, 0)), "y": ((c1, mid, c2), (math.pi / 2, 0, 0)),
                "z": ((c1, c2, mid), (0, 0, 0))}[axis]
    bpy.ops.mesh.primitive_cylinder_add(vertices=segments, radius=r, depth=a1 - a0, location=loc, rotation=rot)
    o = bpy.context.active_object
    o.name = name
    _activate(o)
    bpy.ops.object.transform_apply(rotation=True)
    return o


def sphere(name, r, loc):
    bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=2, radius=r, location=loc)
    o = bpy.context.active_object
    o.name = name
    return o


def boolean(target, tool, op, keep=False):
    _activate(target)
    m = target.modifiers.new(name=op, type="BOOLEAN")
    m.operation = op
    m.solver = "EXACT"
    m.object = tool
    with bpy.context.temp_override(object=target, active_object=target, selected_objects=[target]):
        bpy.ops.object.modifier_apply(modifier=m.name)
    if not keep:
        bpy.data.objects.remove(tool, do_unlink=True)
    return target


def cut(t, tool, keep=False):
    return boolean(t, tool, "DIFFERENCE", keep)


def join(t, tool, keep=False):
    return boolean(t, tool, "UNION", keep)


def meet(t, tool, keep=False):
    return boolean(t, tool, "INTERSECT", keep)


def dup(obj, name):
    o = obj.copy()
    o.data = obj.data.copy()
    o.name = name
    bpy.context.scene.collection.objects.link(o)
    return o


def band(name, size, a, b):
    """The layer between insets a and b (a < b) of the rounded box."""
    return cut(rbox(name, size, a), rbox(name + "_in", size, b))


def cleanup(obj, dist=0.01):
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
    v = bm.calc_volume(signed=False)
    bm.free()
    return v


def export_stl(obj, path):
    _activate(obj)
    if hasattr(bpy.ops.wm, "stl_export"):
        bpy.ops.wm.stl_export(filepath=path, export_selected_objects=True, apply_modifiers=True)
    else:
        bpy.ops.export_mesh.stl(filepath=path, use_selection=True)


def fits(dims, bed):
    """Fits the bed square, or turned diagonally on it (the panels are long and flat)."""
    d, b = sorted(dims), sorted(bed)
    if all(x <= y for x, y in zip(d, b)):
        return "fits"
    lo, hi = d[0], d[1:]          # flat on the bed: the smallest dimension is the height
    a, c = sorted(hi)
    bx, by = bed[0], bed[1]
    for th in range(1, 90):       # a rectangle a x c turned by th inside bx x by
        t = math.radians(th)
        if a * math.cos(t) + c * math.sin(t) <= bx and a * math.sin(t) + c * math.cos(t) <= by and lo <= bed[2]:
            return "fits turned %d deg" % th
    return "TOO BIG"


# ----------------------------------------------------------------------------- seam features
def seam_points(size, axis, c, d, keep_out):
    """Feature points along the straight (flat-face) runs of the seam at `axis` = c, whose strip reaches toward d
    (-1 or +1): on every face parallel to the seam axis, a run along the face's free axis, alternating screws and
    keys. Each point is on the strip's outer face (the wall's inner surface plus the gap), half a lap past the seam.
    Returns [(kind, normal axis, sign, (x, y, z))]."""
    r, w = P["r"], P["wall"] + P["gap"]
    half = P["screw_pitch"] / 2
    ia = "xyz".index(axis)
    out = []
    for n in "xyz":
        if n == axis:
            continue
        m = [a for a in "xyz" if a not in (axis, n)][0]
        i_n, i_m = "xyz".index(n), "xyz".index(m)
        lo, hi = -size[i_m] / 2 + r + 5, size[i_m] / 2 - r - 5
        k = max(1, int((hi - lo) // half))
        for sgn in (-1, 1):
            for j in range(k + 1):
                p = [0.0, 0.0, 0.0]
                p[ia] = c + d * P["lap_w"] / 2
                p[i_n] = sgn * (size[i_n] / 2 - w)
                p[i_m] = lo + j * (hi - lo) / k
                out.append(("screw" if j % 2 == 0 else "key", n, sgn, tuple(p)))
    return [o for o in out if all(math.dist(o[3], cc) > rr + 8.0 for cc, rr in keep_out)]


def hole_centres(size, holes):
    W, D, H = size
    out = []
    for ax, sgn, (u, v), rr in holes:
        c = {"z": (u, v, sgn * H / 2), "x": (sgn * W / 2, u, v), "y": (u, sgn * D / 2, v)}[ax]
        out.append((c, rr))
    return out


def screw_and_key(owner, other, kind, ax, sgn, p):
    """A pad and insert hole on the strip (owner) and a clearance hole in the neighbour's wall (other), or a key
    bump on the strip and its dimple in the neighbour's wall."""
    i = "xyz".index(ax)
    if kind == "key":
        owner = join(owner, sphere("key", P["key_r"], p))
        other = cut(other, sphere("dimple", P["key_r"] + 0.15, p))
        return owner, other
    s0 = p[i]                                    # the strip's outer face
    s1 = s0 - sgn * P["lap_t"]                   # its inner face
    a0, a1 = sorted((s1 + sgn * 0.2, s1 - sgn * P["pad_h"]))
    c = [p[j] for j in range(3) if j != i]
    owner = join(owner, cyl("pad", ax, P["pad_d"] / 2, a0, a1, *c))
    b0, b1 = sorted((s0 + sgn * 0.1, s0 - sgn * P["insert_depth"]))
    owner = cut(owner, cyl("insert", ax, P["insert_d"] / 2, b0, b1, *c))
    w0, w1 = sorted((s0, s0 + sgn * (P["wall"] + P["gap"] + 1.0)))
    other = cut(other, cyl("clear", ax, P["clear_d"] / 2, w0 - 0.5, w1, *c))
    return owner, other


# ----------------------------------------------------------------------------- build
def build(name, spec, out_dir):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    size = spec["size"]
    W, D, H = size
    t, g2, L = P["wall"], P["seam_gap"] / 2, P["lap_w"]
    zs = next((c for ax, c, _ in spec["seams"] if ax == "z"), None)
    xs = next((c for ax, c, _ in spec["seams"] if ax == "x"), None)
    big = 2 * max(size)

    # wall and ribs
    body = band("body", size, 0.0, t)
    plates = None
    keep = L + 3.0                                       # no ribs where a strip lies

    def add(o):
        nonlocal plates
        plates = o if plates is None else join(plates, o)
    n = int(W // P["rib_pitch"])
    for i in range(n):
        x = -W / 2 + (i + 0.5) * W / n
        add(box("rx", x - P["rib_t"] / 2 + 0.013, x + P["rib_t"] / 2 + 0.013, -big, big, -big, big))
    n = int(H // P["rib_pitch"])
    for i in range(n):
        z = -H / 2 + (i + 0.5) * H / n
        if zs is not None and abs(z - zs) < keep:
            continue
        add(box("rz", -big, big, -big, big, z - P["rib_t"] / 2 + 0.017, z + P["rib_t"] / 2 + 0.017))
    for y in (-D / 4, D / 4):
        add(box("ry", -big, big, y - P["rib_t"] / 2 + 0.011, y + P["rib_t"] / 2 + 0.011, -big, big))
    if any(ax == "y" for ax, _, _ in spec["seams"]):
        plates = cut(plates, box("ky", -big, big, -keep, keep, -big, big))
    if xs is not None:
        plates = cut(plates, box("kx", xs - keep, xs + keep, -big, big, -big, big))
    if zs is not None:
        plates = cut(plates, box("kz", -big, big, -big, big, zs - keep, zs + keep))
    ribs = meet(band("ribzone", size, t - 0.3, t + P["rib_d"]), plates)
    body = join(body, ribs)

    # drive-bank bosses on the back face (insert holes from the inside)
    bosses = []
    nb = int((W - 2 * P["r"]) // P["boss_pitch"]) + 1
    for i in range(nb):
        x = (i - (nb - 1) / 2) * P["boss_pitch"]
        for j in range(int((H - 2 * P["r"]) // P["boss_pitch"]) + 1):
            z = -H / 2 + P["r"] + 10 + j * P["boss_pitch"]
            if zs is not None and abs(z - zs) < keep + P["boss_d"]:
                continue
            if xs is not None and abs(x - xs) < keep + P["boss_d"]:
                continue
            if z > H / 2 - P["r"] - 5:
                continue
            bosses.append((x, z))
    for x, z in bosses:
        y0 = -D / 2 + t - 0.3
        body = join(body, cyl("boss", "y", P["boss_d"] / 2, y0, -D / 2 + t + P["boss_h"], x, z))
        body = cut(body, cyl("boss_in", "y", P["insert_d"] / 2, -D / 2 + t + P["boss_h"] - P["insert_depth"],
                             -D / 2 + t + P["boss_h"] + 0.5, x, z))

    # panels: one per side of every seam; seams = [(axis, coordinate, d)], the strip reaching toward d
    seams = spec["seams"]
    holes = hole_centres(size, spec["holes"])
    panels = {}
    for code in range(2 ** len(seams)):
        sides = tuple(1 if (code >> k) & 1 else -1 for k in range(len(seams)))
        lim = [[-big, big], [-big, big], [-big, big]]
        for (ax, c, _), sd in zip(seams, sides):
            lim["xyz".index(ax)] = [c + g2, big] if sd > 0 else [-big, c - g2]
        nm = "%s_shell_%s" % (name, "_".join(spec["names"][k][0 if sd < 0 else 1] for k, sd in enumerate(sides)))
        o = meet(dup(body, nm), box("region", *lim[0], *lim[1], *lim[2]))
        o.name = nm
        panels[sides] = o
    bpy.data.objects.remove(body, do_unlink=True)

    def strip(ax, c, d, lim):
        """The lap strip across the seam at ax = c reaching toward d: part 1 inside the neighbour's wall (with the
        gap), part 2 fused into the owner's own wall; lim = the strip's extent along the other axes."""
        i = "xyz".index(ax)
        l1, l2 = [list(v) for v in lim], [list(v) for v in lim]
        l1[i] = sorted((c + d * L, c - d * 1.0))
        l2[i] = sorted((c, c - d * L))
        s1 = meet(band("b1", size, t + P["gap"], t + P["gap"] + P["lap_t"]), box("s1", *l1[0], *l1[1], *l1[2]))
        s2 = meet(band("b2", size, t - 0.3, t + P["lap_t"] + 0.45), box("s2", *l2[0], *l2[1], *l2[2]))
        return join(s2, s1)

    for k, (ax, c, d) in enumerate(seams):
        for sides, owner in list(panels.items()):
            if sides[k] != -d:                      # the strip belongs to the side it reaches away from
                continue
            other_sides = tuple(-sd if j == k else sd for j, sd in enumerate(sides))
            lim = [[-big, big], [-big, big], [-big, big]]
            for j, (ax2, c2, _) in enumerate(seams):   # stop short of the other seams (their strips are there)
                if j != k:
                    lim["xyz".index(ax2)] = [c2 + keep, big] if sides[j] > 0 else [-big, c2 - keep]
            owner = join(owner, strip(ax, c, d, lim))
            other = panels[other_sides]
            for kind, n, sgn, p in seam_points(size, ax, c, d, holes):
                if all(lim[q][0] < p[q] < lim[q][1] for q in range(3)):
                    owner, other = screw_and_key(owner, other, kind, n, sgn, p)
            panels[sides], panels[other_sides] = owner, other

    # openings
    for ax, sgn, (u, v), rr in spec["holes"]:
        i = "xyz".index(ax)
        face = size[i] / 2
        for k, o in panels.items():
            tool = cyl("hole", ax, rr, sgn * face - 30 if sgn > 0 else -face - 1, face + 1 if sgn > 0 else -face + 30,
                       u, v, 41)
            panels[k] = cut(o, tool)

    os.makedirs(out_dir, exist_ok=True)
    total = 0.0
    print("\n%s shell: %.0f x %.0f x %.0f mm, wall %.1f mm, ribs %.1f x %.0f mm at %.0f, %d bank bosses" %
          (name, W, D, H, t, P["rib_t"], P["rib_d"], P["rib_pitch"], len(bosses)))
    for k, o in panels.items():
        bad = cleanup(o)
        v = volume_mm3(o)
        total += v
        dims = tuple(o.dimensions)
        path = os.path.join(out_dir, o.name + ".stl")
        export_stl(o, path)
        print("  %-28s %8.0f mm^3  %5.0f g  %4.0f x %4.0f x %4.0f mm  %-18s non-manifold %d" %
              (o.name, v, v * PETG, dims[0], dims[1], dims[2], fits(dims, P["bed"]), bad))
    print("  %s total %.0f g printed solid (the walls are all perimeter)" % (name, total * PETG))


if __name__ == "__main__":
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    out = argv[argv.index("--out") + 1] if "--out" in argv else "cad/out"
    if "--wall" in argv:
        P["wall"] = float(argv[argv.index("--wall") + 1])
    only = argv[argv.index("--preset") + 1] if "--preset" in argv else None
    for nm in ([only] if only else list(PRESETS)):
        build(nm, PRESETS[nm], os.path.abspath(out))
