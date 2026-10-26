"""
winch_mini.py - parametric generator for the W-mini tendon winch (N20 gearmotor + spool).

Coordinate frame: X along the motor axis (motor at low X, drum at high X), Z up.
Housing prints open-top (as modelled), drum prints flange-down, lid prints flat.
Units: 1 Blender unit = 1 mm, STL is exported unscaled.

Run headless (Blender 4.2+ / 5.x):
    blender -b -P cad/blender/winch_mini.py -- --out cad/out
Or open in Blender and run from the Text Editor to inspect the parts.

Outputs: winch_mini_housing.stl, winch_mini_drum.stl, winch_mini_lid.stl, and a volume report.
"""
import math
import os
import sys

import bmesh
import bpy

# ----------------------------------------------------------------------------- parameters (mm)
P = {
    # N20 gearmotor: 12 x 10 mm body, D shaft 3 mm, two M1.6 holes 9 mm apart on the gearbox face
    "motor_w": 12.0,
    "motor_h": 10.0,
    "motor_len": 26.0,       # gearbox + can + encoder PCB clearance; rear of housing is open anyway
    "motor_clear": 0.3,
    "shaft_d": 3.0,
    "shaft_flat_depth": 0.5,  # distance from the round surface to the flat (3.0 shaft -> 2.5 across the flat)
    "shaft_bore_clear": 0.15,
    "mount_hole_d": 1.7,
    "mount_hole_spacing": 9.0,
    # drum
    "core_r": 3.0,
    "flange_r": 5.0,
    "core_w": 5.0,
    "flange_t": 1.5,
    "anchor_hole_d": 1.2,
    "anchor_cbore_d": 2.6,
    "anchor_cbore_depth": 0.8,
    "stub_d": 2.0,
    "stub_len": 2.0,
    "stub_bore_clear": 0.15,
    # housing
    "wall": 2.5,
    "floor": 2.5,
    "end_wall": 3.0,
    "drum_clear": 0.5,
    "tube_od": 4.0,
    "tube_clear": 0.3,
    "tube_socket_depth": 6.0,
    "tube_boss_wall": 1.5,
    "line_hole_d": 1.4,
    # lid
    "lid_t": 2.0,
    "lid_screw_pilot_d": 1.6,   # M2 self-tapping into PETG
    "lid_screw_clear_d": 2.2,
    "lid_screw_inset": 1.4,
    "lid_screw_depth": 6.0,
    "segments": 64,
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


def cyl(name, axis, r, a0, a1, c1, c2, segments=None):
    """Cylinder along `axis` ('x','y' or 'z') from a0 to a1; (c1, c2) are the other two coords in x,y,z order."""
    length = a1 - a0
    mid = (a0 + a1) / 2
    if axis == "x":
        loc, rot = (mid, c1, c2), (0, math.pi / 2, 0)
    elif axis == "y":
        loc, rot = (c1, mid, c2), (math.pi / 2, 0, 0)
    else:
        loc, rot = (c1, c2, mid), (0, 0, 0)
    bpy.ops.mesh.primitive_cylinder_add(vertices=segments or P["segments"], radius=r, depth=length, location=loc, rotation=rot)
    o = bpy.context.active_object
    o.name = name
    _activate(o)
    bpy.ops.object.transform_apply(rotation=True)
    return o


def boolean(target, tool, op):
    _activate(target)
    m = target.modifiers.new(name=op, type="BOOLEAN")
    m.operation = op
    m.solver = "EXACT"
    m.object = tool
    with bpy.context.temp_override(object=target, active_object=target, selected_objects=[target]):
        bpy.ops.object.modifier_apply(modifier=m.name)
    bpy.data.objects.remove(tool, do_unlink=True)
    return target


def cut(target, tool):
    return boolean(target, tool, "DIFFERENCE")


def join(target, tool):
    return boolean(target, tool, "UNION")


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
    else:  # Blender < 4.2
        bpy.ops.export_mesh.stl(filepath=path, use_selection=True)


# ----------------------------------------------------------------------------- derived layout
def layout():
    L = {}
    L["inner_w"] = max(P["motor_w"] + 2 * P["motor_clear"], 2 * P["flange_r"] + 2 * P["drum_clear"])
    L["W"] = L["inner_w"] + 2 * P["wall"]
    L["axis_z"] = P["floor"] + P["motor_clear"] + P["motor_h"] / 2
    L["H"] = L["axis_z"] + max(P["motor_h"] / 2 + P["motor_clear"], P["flange_r"] + P["drum_clear"])
    L["pocket_x1"] = P["motor_len"] + P["motor_clear"]
    L["bulk_x0"] = L["pocket_x1"]
    L["bulk_x1"] = L["bulk_x0"] + P["wall"]
    L["drum_len"] = 2 * P["flange_t"] + P["core_w"]
    L["cham_x0"] = L["bulk_x1"]
    L["cham_x1"] = L["cham_x0"] + L["drum_len"] + 2 * P["drum_clear"]
    L["end_x1"] = L["cham_x1"] + P["end_wall"]
    L["L"] = L["end_x1"]
    L["drum_x0"] = L["cham_x0"] + P["drum_clear"]
    L["drum_x1"] = L["drum_x0"] + L["drum_len"]
    L["core_x0"] = L["drum_x0"] + P["flange_t"]
    L["core_x1"] = L["core_x0"] + P["core_w"]
    L["exit_x"] = (L["core_x0"] + L["core_x1"]) / 2
    L["exit_z"] = L["axis_z"] - P["core_r"]           # bottom tangent, line leaves toward +Y
    L["boss_r"] = (P["tube_od"] + P["tube_clear"]) / 2 + P["tube_boss_wall"]
    L["boss_y0"] = L["inner_w"] / 2                   # inner face of +Y wall
    L["boss_y1"] = L["W"] / 2 + P["tube_socket_depth"] - P["wall"] + P["tube_boss_wall"]
    return L


# ----------------------------------------------------------------------------- parts
def make_housing(L):
    h = box("winch_mini_housing", 0, L["L"], -L["W"] / 2, L["W"] / 2, 0, L["H"])
    # motor pocket, open at the rear (x=0) and at the top
    h = cut(h, box("pocket", -1, L["pocket_x1"], -P["motor_w"] / 2 - P["motor_clear"], P["motor_w"] / 2 + P["motor_clear"],
                   P["floor"], L["H"] + 1))
    # drum chamber, open at the top
    h = cut(h, box("chamber", L["cham_x0"], L["cham_x1"], -L["inner_w"] / 2, L["inner_w"] / 2,
                   L["axis_z"] - P["flange_r"] - P["drum_clear"], L["H"] + 1))
    # bulkhead: shaft bore and the two M1.6 mounting holes
    h = cut(h, cyl("shaft_bore", "x", (P["shaft_d"] + 0.6) / 2, L["bulk_x0"] - 1, L["bulk_x1"] + 1, 0, L["axis_z"]))
    for s in (-1, 1):
        h = cut(h, cyl("mount_hole", "x", P["mount_hole_d"] / 2, L["bulk_x0"] - 1, L["bulk_x1"] + 1,
                       s * P["mount_hole_spacing"] / 2, L["axis_z"]))
    # end wall: blind bore for the drum stub
    h = cut(h, cyl("stub_bore", "x", (P["stub_d"] + 2 * P["stub_bore_clear"]) / 2, L["cham_x1"] - 0.5,
                   L["cham_x1"] + P["stub_len"] + 0.5, 0, L["axis_z"]))
    # PTFE socket boss on the +Y wall, tangent to the bottom of the drum core
    h = join(h, cyl("boss", "y", L["boss_r"], L["boss_y0"], L["boss_y1"], L["exit_x"], L["exit_z"]))
    h = cut(h, cyl("tube_socket", "y", (P["tube_od"] + P["tube_clear"]) / 2,
                   L["boss_y1"] - P["tube_socket_depth"], L["boss_y1"] + 1, L["exit_x"], L["exit_z"]))
    h = cut(h, cyl("line_hole", "y", P["line_hole_d"] / 2, L["boss_y0"] - 1, L["boss_y1"], L["exit_x"], L["exit_z"]))
    # lid screw pilots in the four corners
    for x in (P["lid_screw_inset"], L["L"] - P["lid_screw_inset"]):
        for y in (-L["W"] / 2 + P["lid_screw_inset"], L["W"] / 2 - P["lid_screw_inset"]):
            h = cut(h, cyl("pilot", "z", P["lid_screw_pilot_d"] / 2, L["H"] - P["lid_screw_depth"], L["H"] + 1, x, y))
    return h


def make_drum(L):
    z = L["axis_z"]
    d = cyl("winch_mini_drum", "x", P["flange_r"], L["drum_x0"], L["drum_x0"] + P["flange_t"], 0, z)
    d = join(d, cyl("core", "x", P["core_r"], L["core_x0"] - 0.01, L["core_x1"] + 0.01, 0, z))
    d = join(d, cyl("flange2", "x", P["flange_r"], L["core_x1"], L["drum_x1"], 0, z))
    d = join(d, cyl("stub", "x", P["stub_d"] / 2, L["drum_x1"] - 0.01, L["drum_x1"] + P["stub_len"], 0, z))
    # D-bore from the motor side through flange + core (far flange stays solid)
    bore_r = (P["shaft_d"] + 2 * P["shaft_bore_clear"]) / 2
    d = cut(d, cyl("bore", "x", bore_r, L["drum_x0"] - 1, L["core_x1"], 0, z))
    flat_z = z + P["shaft_d"] / 2 - P["shaft_flat_depth"] + P["shaft_bore_clear"]
    d = join(d, box("flat", L["drum_x0"], L["core_x1"], -bore_r - 0.01, bore_r + 0.01, flat_z, z + bore_r + 0.01))
    # line anchor: axial hole through the far flange with a knot counterbore on its outer face
    ar = (P["core_r"] + P["flange_r"]) / 2
    d = cut(d, cyl("anchor", "x", P["anchor_hole_d"] / 2, L["core_x1"] - 0.5, L["drum_x1"] + 1, 0, z + ar))
    d = cut(d, cyl("cbore", "x", P["anchor_cbore_d"] / 2, L["drum_x1"] - P["anchor_cbore_depth"], L["drum_x1"] + 1, 0, z + ar))
    return d


def make_lid(L):
    lid = box("winch_mini_lid", 0, L["L"], -L["W"] / 2, L["W"] / 2, L["H"], L["H"] + P["lid_t"])
    for x in (P["lid_screw_inset"], L["L"] - P["lid_screw_inset"]):
        for y in (-L["W"] / 2 + P["lid_screw_inset"], L["W"] / 2 - P["lid_screw_inset"]):
            lid = cut(lid, cyl("clear", "z", P["lid_screw_clear_d"] / 2, L["H"] - 1, L["H"] + P["lid_t"] + 1, x, y))
    return lid


# ----------------------------------------------------------------------------- main
def main(out_dir):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    L = layout()
    parts = [make_housing(L), make_drum(L), make_lid(L)]
    os.makedirs(out_dir, exist_ok=True)
    print("\nW-mini winch, overall housing %.1f x %.1f x %.1f mm (L x W x H), drum axis at z=%.1f" %
          (L["L"], L["W"], L["H"], L["axis_z"]))
    print("drum: core r=%.1f, flange r=%.1f, length %.1f; one turn = %.1f mm of line" %
          (P["core_r"], P["flange_r"], L["drum_len"], 2 * math.pi * P["core_r"]))
    for o in parts:
        v = volume_mm3(o)
        path = os.path.join(out_dir, o.name + ".stl")
        export_stl(o, path)
        print("  %-22s volume %7.0f mm^3  (~%.1f g solid PETG)  -> %s" % (o.name, v, v * 1.27e-3, path))


if __name__ == "__main__":
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    out = "cad/out"
    if "--out" in argv:
        out = argv[argv.index("--out") + 1]
    main(os.path.abspath(out))
