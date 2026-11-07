---
name: cad-generators
description: Build, change and render the robot's printable parts with the Blender Python generators in cad/blender (knuckle chain, ring joint, winches, finger, skeleton rig). Use whenever a part, preset, pulley/lug size or STL changes, or a part render is needed.
---

# CAD generators (Blender 5.1, headless)

The parts are parametric Blender Python scripts, not OpenSCAD and not hand-made meshes. Never hand-edit
`cad/out/*.stl`: edit the preset table and re-run. `cad/out/` is gitignored, so the STLs exist only after a
run; a release builds them fresh (see the `release` skill).

Blender on this machine:

```bash
B="/c/Program Files/Blender Foundation/Blender 5.1/blender.exe"
```

| Script | Builds | Run |
| --- | --- | --- |
| `chain.py` | knuckle chain, 11 presets: finger, thumb, toe, wrist, ankle, neck, waist, elbow, knee, hip, shoulder | `"$B" -b -P cad/blender/chain.py -- --out cad/out [--preset neck]` |
| `ring.py` | ring joint (axial rotation), 6 presets: forearm, forearm_6808, hip_rot, shoulder_rot, neck_yaw, waist_yaw | `... ring.py -- --out cad/out [--preset forearm]` |
| `winch_mini.py` | W-mini housing, drum, lid | `... winch_mini.py -- --out cad/out` |
| `winch_bank.py` | N W-mini housings as one block | `... winch_bank.py -- --out cad/out --n 5` |
| `finger.py` | Phase 0 finger | `... finger.py -- --out cad/out` |
| `shell.py` | torso (4 panels) and pelvis (2 halves) as keyed thin shells: ribbed wall, lap strips, keys, M3 inserts, bank bosses (docs/35) | `... shell.py -- --out cad/out [--preset torso] [--wall 0.8]` |
| `skeleton_rig.py` | whole-body armature from `cad/skeleton.json`, posed to its limits | `... skeleton_rig.py -- --json cad/skeleton.json --out cad/out` |
| `render.py` | PNG of one generated assembly, or a gallery of every STL in a folder | below |

## Presets and their constraints

- `chain.py` presets: `universal_preset(name, W, H, lug_r, lug_w, fit, line_d, liner_od, rom1, rom2, ...)`
  and `hinge_preset(...)` at the bottom of the file. **`lug_r` is the tendon moment arm**: it must match
  `JOINTS[...]["pulley"]` in `tools/export_mjcf.py` (metres there, mm here). Change both together.
- A universal joint's yaw line runs pass through the tongue's free centre. If `lug_r` grows, the generator
  refuses with `yaw runs need +-X mm of free tongue centre, have Y; widen W`. Widen `W` (and `H` if the lug
  no longer fits the section) until it builds. Since D-032: neck 80 x 52 at 25 mm, wrist 66 x 42 at 18 mm.
- `shell=3.0` hollows the base and end segments (tubes kept around channels). The cross piece is always
  solid, and a short segment may be too small to hollow (the neck's are).
- The printed "mass" is solid PETG (100 % infill); real prints at normal infill weigh roughly half. The
  simulation's masses come from `cad/skeleton.json` (`mass_kg`), not from these figures.
- **Mass roll-up** (`tools/mass_rollup.py`, docs/20): every generated part at print settings (walls and skins
  1.2 mm solid, 25 % infill by default; `--infill`, `--wall`), the parts with no generator yet (estimates),
  and the bought parts, per body segment against `cad/skeleton.json`. `--build` runs the generators first
  (chain, ring, winch_bank and shell into `--stl`, default `cad/out`); `--frame` books the pre-shell 600 + 800 g frame guesses instead of the shells; `--json masses.json` writes per-bone masses for the model variant
  `ROBOT_MASSES=masses.json` (tendon-sim skill); `--torso-worm` rolls up the worm-gearmotor torso variant; `--wmax-driver small` puts the 775s on a VNH5019-class
  driver (10 g) instead of the BTS7960 (65 g); `--wmax bldc` makes the W-max a 5010 + B-G431B-ESC1 + printed 1:16
  (docs/21). The flags combine. A new generator part must be added to `PARTS` (count and
  segment) or `IGNORE`, or the tool lists it as unmatched.
- Every export is re-imported and its non-manifold edges counted; anything but `non-manifold 0` is a bug.
  The boolean lessons (odd segment counts, no coplanar unions, spun cylinders) are at the end of
  `docs/09-knuckle-chain.md`.
- To try a preset change without touching the repo, copy the script to the scratchpad, edit the copy, and
  run it with `--out` pointing into the scratchpad.

## Renders (`render.py`)

```bash
"$B" -b -P cad/blender/render.py -- --gen chain --preset wrist --png docs/img/chain_wrist.png
"$B" -b -P cad/blender/render.py -- --gen ring --preset forearm --png docs/img/ring_forearm.png
"$B" -b -P cad/blender/render.py -- --gallery cad/out --png docs/img/gallery.png --size 2400x1300
```

Workbench engine, a few seconds each. `--view iso|front|side|top`. The gallery groups STLs into
assemblies by name (`neck_bearing_cap` belongs to `neck`, `hip_rot_hub` to `hip_rot`) and moves each
assembly as a unit, since the generators export parts in their assembled positions. Re-render the
affected PNGs and the gallery whenever a generator changes; they are the visuals the docs and releases
use. Look at the PNG (Read it) before committing: framing and grouping bugs only show in the image.

## After a geometry change

1. Rebuild the preset into `cad/out`, check `non-manifold 0`.
2. If a moment arm or section changed: update `tools/export_mjcf.py` (`JOINTS`, collision radii), then
   follow the `tendon-sim` skill (re-export, gauntlet).
3. Re-render, update the doc that shows the part, record the change as a decision (D-nnn).
