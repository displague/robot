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

Planned: `winch_mid.py` (37 mm motor, 2:1 tackle block), `drum_closed_loop.py`, `finger.py`
(3-joint finger with tendon channels), `joint_pin.py` (bushing/pin joint with AS5600 seat).

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
