# CAD workflow

## Generators

Each part family is a Python script that builds the geometry from a parameter table at the top
and exports STL. Edit the table, re-run, re-slice. Never hand-edit the STLs in `cad/out/`.

```powershell
& "C:\Program Files\Blender Foundation\Blender 5.1\blender.exe" -b -P cad/blender/winch_mini.py -- --out cad/out
```

Open the script in Blender's Text Editor and press Run Script to see the parts in the viewport.

| Script | Parts |
| --- | --- |
| `winch_mini.py` | W-mini housing, drum, lid (N20 motor) |
| `finger.py` | Phase 0 finger: palm block + three phalanges with flexor/extensor channels, pins, chamfers |
| `chain.py` | The knuckle-chain primitive with 11 presets (finger, thumb, toe, wrist, ankle, neck, waist, elbow, knee, hip, shoulder); `--preset name` builds one. See `docs/09-knuckle-chain.md` |
| `skeleton_rig.py` | Builds the whole-body armature from `cad/skeleton.json` with joint limits, poses every joint to its extremes, renders |

| `ring.py` | The ring-joint primitive (axial rotation): hub, rotor, optional cap; six presets; `--preset forearm`. See `docs/11-ring-joint.md` |
| `winch_bank.py` | N W-mini housings as one block with floor exits, lid and mounting tabs; `--n 5` |

Planned: `winch_mid.py` (37 mm motor, 2:1 tackle block), `winch_max.py` (775 + planetary),
`drum_closed_loop.py`, part-to-bone assembly of the generated pieces on the skeleton rig.

Generated STLs are checked after export by re-importing them and counting non-manifold edges.
Lessons that keep the exact boolean solver happy are listed at the end of `docs/09-knuckle-chain.md`.

## Whole-body kinematics

`cad/skeleton.json` is the single source of truth for segment lengths, joint axes and limits.
`skeleton_rig.py` reads it, builds a Blender armature with limit-rotation constraints, and can
export a `.blend` for interactive posing. The firmware's joint table and the planner's model are
generated from the same file so the three never disagree.

## What Blender does well for this project

- Parametric mesh generation with booleans (this folder).
- Armature rig with the robot's real joint axes, offsets and limits; sliders drive every DOF so
  reach, collisions and gimbal lock are visible before printing.
- Tendon excursion vs. angle: model the tendon as a curve constrained to the pulley and joint
  geometry, read its length with a Python driver, feed `tools/tendon_calc.py`.
- Mass properties with the 3D Print Toolbox add-on (volume) x material density.
- Print checks: 3D Print Toolbox for manifold errors, overhang angles, wall thickness.

## What Blender does not do

Blender has no finite-element solver, so "testing loads of stress" cannot be done inside it.
The path for the few parts that carry life-critical loads (hip clevis, knee pulley, shoulder
yaw ring):

1. Hand calculation (bending stress on the clevis arm, bearing stress on the pin, tooth shear on
   printed gears) with a 3x safety factor on PETG yield (~50 MPa, use 15 MPa allowable in the
   layer-weak direction).
2. FreeCAD FEM workbench with CalculiX for the real geometry: import the STL (or better, rebuild
   the part in FreeCAD/CadQuery as a solid), fix the mounting faces, apply the tendon force at the
   pulley groove, mesh, solve, read von Mises against the allowable.

For gears and worms, where tolerance matters more than organic shape, consider moving the
generator to CadQuery/build123d (exact B-rep, STEP output, involute profiles) and keep Blender
for assemblies and kinematics. The parameter tables transfer directly.

## Print settings baseline

PETG, 0.2 mm layers, 4 walls, 30 % gyroid, housings printed open-top as modelled, drums flange-
down, 100 % infill on drums and pulleys. Scale the drum D-bore by +-0.1 mm until it presses onto
the N20 shaft by hand with firm resistance.
