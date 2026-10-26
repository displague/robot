"""
winch_bank.py - a row of N W-mini winches as one printed block (a "regional winch bank", D-003).

Built from winch_mini's parameters: N open-top housings side by side along Y sharing their walls,
one lid, N drums. The tendon exit is moved from the side wall to the FLOOR (a tangent socket boss
under each drum pointing -Z), so the sockets of a whole row point the same way and the sheaths
leave the bank as one bundle toward the limb. Two mounting tabs with M3 clearance holes on a 20 mm
pitch match the pad grids on the chain segments.

Run headless:
    blender -b -P cad/blender/winch_bank.py -- --out cad/out --n 5
"""
import math
import os
import sys

import bpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import winch_mini as wm  # noqa: E402

PAD_PITCH = 20.0


def make_housing_floor_exit(L, P):
    """winch_mini housing with the PTFE socket in a boss under the floor instead of on the side wall."""
    h = wm.box("housing", 0, L["L"], -L["W"] / 2, L["W"] / 2, 0, L["H"])
    h = wm.cut(h, wm.box("pocket", -1, L["pocket_x1"], -P["motor_w"] / 2 - P["motor_clear"], P["motor_w"] / 2 + P["motor_clear"],
                         P["floor"], L["H"] + 1))
    h = wm.cut(h, wm.box("chamber", L["cham_x0"], L["cham_x1"], -L["inner_w"] / 2, L["inner_w"] / 2,
                         L["axis_z"] - P["flange_r"] - P["drum_clear"], L["H"] + 1))
    h = wm.cut(h, wm.cyl("shaft_bore", "x", (P["shaft_d"] + 0.6) / 2, L["bulk_x0"] - 1, L["bulk_x1"] + 1, 0, L["axis_z"]))
    for s in (-1, 1):
        h = wm.cut(h, wm.cyl("mount_hole", "x", P["mount_hole_d"] / 2, L["bulk_x0"] - 1, L["bulk_x1"] + 1,
                             s * P["mount_hole_spacing"] / 2, L["axis_z"]))
    h = wm.cut(h, wm.cyl("stub_bore", "x", (P["stub_d"] + 2 * P["stub_bore_clear"]) / 2, L["cham_x1"] - 0.5,
                         L["cham_x1"] + P["stub_len"] + 0.5, 0, L["axis_z"]))
    # floor exit: the line leaves the drum at its +Y tangent heading -Z; boss under the floor, socket along Z
    y_exit = P["core_r"]
    sr = (P["tube_od"] + P["tube_clear"]) / 2
    boss_r = sr + P["tube_boss_wall"]
    z_bot = -(P["tube_socket_depth"] + 2.0)
    h = wm.join(h, wm.cyl("boss", "z", boss_r, z_bot, 0.01, L["exit_x"], y_exit, 40))
    h = wm.cut(h, wm.cyl("tube_socket", "z", sr, z_bot - 1, z_bot + P["tube_socket_depth"], L["exit_x"], y_exit, 41))
    h = wm.cut(h, wm.cyl("line_hole", "z", P["line_hole_d"] / 2, z_bot + P["tube_socket_depth"] - 0.5,
                         L["axis_z"] - P["flange_r"], L["exit_x"], y_exit, 19))
    for x in (P["lid_screw_inset"], L["L"] - P["lid_screw_inset"]):
        for y in (-L["W"] / 2 + P["lid_screw_inset"], L["W"] / 2 - P["lid_screw_inset"]):
            h = wm.cut(h, wm.cyl("pilot", "z", P["lid_screw_pilot_d"] / 2, L["H"] - P["lid_screw_depth"], L["H"] + 1, x, y, 17))
    return h


def make_bank(n, out_dir):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    P, L = wm.P, wm.layout()
    pitch = L["W"] - P["wall"] + 0.3               # adjacent housings share one wall (0.3 mm thicker: no coplanar faces)
    span = pitch * (n - 1) + L["W"]
    bank = None
    for i in range(n):
        h = make_housing_floor_exit(L, P)
        h.location.y = i * pitch
        wm._activate(h)
        bpy.ops.object.transform_apply(location=True)
        bank = h if bank is None else wm.join(bank, h)
    bank.name = "bank%d_housing" % n
    # mounting tabs at both ends of the row, M3 clearance holes on the pad pitch
    y0, y1 = -L["W"] / 2, -L["W"] / 2 + span
    for ys, sgn in ((y0, -1), (y1, 1)):
        tab = wm.box("tab", 4.0, L["L"] - 4.0, ys + sgn * 12.0 if sgn > 0 else ys - 12.0, ys if sgn > 0 else ys + 0.01, 0.0, 3.0)
        if sgn > 0:
            tab = wm.box("tab", 4.0, L["L"] - 4.0, ys - 0.01, ys + 12.0, 0.0, 3.0)
        bank = wm.join(bank, tab)
        for x in (L["L"] / 2 - PAD_PITCH / 2, L["L"] / 2 + PAD_PITCH / 2):
            bank = wm.cut(bank, wm.cyl("m3", "z", 1.7, -1, 4, x, ys - 6.0 if sgn < 0 else ys + 6.0, 19))
    lid = wm.box("bank%d_lid" % n, 0, L["L"], y0, y1, L["H"], L["H"] + P["lid_t"])
    for i in range(n):
        for x in (P["lid_screw_inset"], L["L"] - P["lid_screw_inset"]):
            for yy in (-L["W"] / 2 + P["lid_screw_inset"], L["W"] / 2 - P["lid_screw_inset"]):
                lid = wm.cut(lid, wm.cyl("clear", "z", P["lid_screw_clear_d"] / 2, L["H"] - 1, L["H"] + P["lid_t"] + 1,
                                         x, yy + i * pitch, 17))
    drum = wm.make_drum(L)
    drum.name = "bank_drum"
    os.makedirs(out_dir, exist_ok=True)
    print("\nW-mini bank x%d: %.1f x %.1f x %.1f mm (L x W x H) + lid %.0f, floor sockets on a %.1f mm pitch, tabs on %.0f mm holes" %
          (n, L["L"], span + 24, L["H"], P["lid_t"], pitch, PAD_PITCH))
    for o in (bank, lid, drum):
        v = wm.volume_mm3(o)
        path = os.path.join(out_dir, o.name + ".stl")
        wm.export_stl(o, path)
        print("  %-18s volume %7.0f mm^3  (~%.0f g solid PETG) -> %s" % (o.name, v, v * 1.27e-3, path))


if __name__ == "__main__":
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    out = argv[argv.index("--out") + 1] if "--out" in argv else "cad/out"
    n = int(argv[argv.index("--n") + 1]) if "--n" in argv else 5
    make_bank(n, os.path.abspath(out))
