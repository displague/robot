"""Render generated parts to PNG, headless. Two modes:

  One assembly, built by its generator in this Blender session and rendered where the generator leaves it:
    blender -b -P cad/blender/render.py -- --gen chain --preset wrist --png docs/img/chain_wrist.png
    blender -b -P cad/blender/render.py -- --gen ring --preset forearm --png docs/img/ring_forearm.png
    blender -b -P cad/blender/render.py -- --gen winch_mini --png docs/img/winch_mini.png

  A gallery of every STL in a folder, one tile per assembly prefix (neck_*, wrist_*, ...), laid out on a grid:
    blender -b -P cad/blender/render.py -- --gallery cad/out --png docs/img/gallery.png

Options: --size 1400x800 (pixels), --view iso|front|side|top. Workbench engine with studio light and a flat
grey world, so renders are quick (seconds) and consistent between runs. Parts are coloured by their index in
the assembly (grey, bronze, slate, sand, ...), the same palette as the renders already in docs/img.
The generator's STLs go to a temporary folder in --gen mode; cad/out is not touched.
"""
import bpy
import math
import os
import sys
import tempfile
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
PALETTE = [(0.55, 0.56, 0.58), (0.62, 0.45, 0.28), (0.42, 0.52, 0.60), (0.70, 0.64, 0.50),
           (0.45, 0.58, 0.47), (0.60, 0.42, 0.45), (0.50, 0.50, 0.66)]
VIEWS = {"iso": (1.0, -1.2, 0.8), "front": (0.0, -1.0, 0.15), "side": (1.0, 0.0, 0.15), "top": (0.05, -0.05, 1.0), "gallery": (0.35, -1.0, 0.9)}


def args():
    a = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    opt = {"gen": None, "preset": None, "gallery": None, "png": "render.png", "size": "1400x800", "view": "iso"}
    for k in list(opt):
        if "--" + k in a:
            opt[k] = a[a.index("--" + k) + 1]
    return opt


def meshes():
    return [o for o in bpy.context.scene.objects if o.type == "MESH"]


def colour(objs):
    for i, o in enumerate(sorted(objs, key=lambda o: o.name)):
        o.color = PALETTE[i % len(PALETTE)] + (1.0,)


def bounds(objs):
    bpy.context.view_layer.update()          # matrix_world is stale after a location change until this
    lo = Vector((1e9, 1e9, 1e9))
    hi = -lo
    for o in objs:
        for c in o.bound_box:
            w = o.matrix_world @ Vector(c)
            lo = Vector(map(min, lo, w))
            hi = Vector(map(max, hi, w))
    return lo, hi


def frame(objs, view, aspect):
    lo, hi = bounds(objs)
    centre, size = (lo + hi) / 2, (hi - lo).length
    d = Vector(VIEWS[view]).normalized()
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    bpy.context.scene.collection.objects.link(cam)
    cam.data.type = "ORTHO"
    cam.data.ortho_scale = size * 0.8 * max(aspect, 1.0)   # the projected extent is under the diagonal
    cam.data.clip_end = size * 20
    cam.location = centre + d * size * 3
    cam.rotation_euler = (-d).to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.camera = cam


def render(png, size, view):
    w, h = (int(v) for v in size.split("x"))
    sc = bpy.context.scene
    sc.render.engine = "BLENDER_WORKBENCH"
    sc.display.shading.light = "STUDIO"
    sc.display.shading.color_type = "OBJECT"
    sc.display.shading.show_cavity = True
    sc.display.shading.show_shadows = False
    sc.render.resolution_x, sc.render.resolution_y, sc.render.resolution_percentage = w, h, 100
    sc.render.film_transparent = False
    if sc.world is None:
        sc.world = bpy.data.worlds.new("world")
    sc.world.color = (0.80, 0.80, 0.80)
    sc.render.image_settings.file_format = "PNG"
    objs = meshes()
    colour(objs)
    frame(objs, view, w / h)
    sc.render.filepath = os.path.abspath(png)
    os.makedirs(os.path.dirname(sc.render.filepath) or ".", exist_ok=True)
    bpy.ops.render.render(write_still=True)
    print("rendered %s (%d parts)" % (png, len(objs)))


def build(gen, preset):
    """Run a generator's main() here; it leaves its parts assembled in the scene."""
    sys.path.insert(0, HERE)
    mod = __import__(gen)
    tmp = tempfile.mkdtemp(prefix="render_")
    if preset:
        mod.main(tmp, preset)
    else:
        mod.main(tmp)


PART_WORDS = {"base", "bearing", "cap", "cross", "end", "housing", "lid", "drum", "lower", "upper", "p1", "p2",
              "distal", "middle", "proximal", "palm", "hub", "rotor", "cmc", "plate", "sole"}


def assembly_of(stem):
    """neck_bearing_cap -> neck, hip_rot_hub -> hip_rot, forearm_6808_cap -> forearm_6808, elbow_lower_p1 -> elbow."""
    t = stem.split("_")
    while len(t) > 1 and t[-1] in PART_WORDS:
        t.pop()
    return "_".join(t)


def gallery(folder):
    """Every STL in folder, grouped into assemblies by name (assembly_of). The generators export parts in their
    assembled positions, so each group is moved as one unit; groups are packed in rows by their real widths.
    A part that was also exported split for the bed (x_p1, x_p2) is shown once, unsplit."""
    bpy.ops.wm.read_factory_settings(use_empty=True)
    stems = sorted(f[:-4] for f in os.listdir(folder) if f.lower().endswith(".stl"))
    groups = {}
    for st in stems:
        if st.endswith(("_p1", "_p2")) and st[:-3] in stems:
            continue
        groups.setdefault(assembly_of(st), []).append(st)
    tiles = []
    for name, parts in groups.items():
        objs = []
        for st in parts:
            before = set(bpy.context.scene.objects)
            bpy.ops.wm.stl_import(filepath=os.path.join(folder, st + ".stl"))
            objs.extend(set(bpy.context.scene.objects) - before)
        colour(objs)
        lo, hi = bounds(objs)
        tiles.append((name, objs, hi - lo, lo))
    gap = 70.0
    area = sum((d.x + gap) * (d.y + gap) for _, _, d, _ in tiles)
    row_w = math.sqrt(area) * 1.4
    x = y = row_h = 0.0
    for name, objs, d, lo in sorted(tiles, key=lambda t: -t[2].y):
        if x > 0 and x + d.x > row_w:
            x, y, row_h = 0.0, y - row_h - gap, 0.0
        shift = Vector((x - lo.x, y - d.y - lo.y, -lo.z))
        for o in objs:
            o.location += shift
        x += d.x + gap
        row_h = max(row_h, d.y)
    print("gallery: %d assemblies from %s: %s" % (len(tiles), folder, ", ".join(sorted(groups))))


if __name__ == "__main__":
    o = args()
    if o["gallery"]:
        gallery(o["gallery"])
        if o["view"] == "iso":
            o["view"] = "gallery"
    else:
        build(o["gen"], o["preset"])
    # the generators' color settings are not ours: colour by part after building
    render(o["png"], o["size"], o["view"])
