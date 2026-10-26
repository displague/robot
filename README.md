# Home-printable tendon-driven humanoid

A full-size humanoid that a person can print on a desktop FDM printer, assemble by hand, and
run on one to three cheap boards (ESP32 class for motion, a Pi 5 / RK1 class board only if we
want autonomy). Muscles are fishing line pulled by small spindles ("winches") that can lock
tension without power, routed through PTFE tube like bicycle brake cables, so the strong part
(the motor) can live wherever there is room and the moving part (a finger, a wrist) stays light.

Goal ordering, in case they conflict: **cheap > simple to build > human range of motion > walks and balances > strength > speed.**

## Design principles

1. **ROM parity, not DOF parity.** Every joint reaches the human range of motion. We do not give
   every joint its own motor. Fingers are underactuated (one tendon per finger, elastic return).
2. **One actuator family, three sizes.** The same winch design (motor + drum + line exit + lock)
   scaled to three motor classes. Fewer unique parts, one firmware driver, one calculator.
3. **Tendons carry force, printed parts carry geometry.** Bones are hollow PETG shells with
   integral tendon channels. Nothing printed is loaded in a way that a layer line can peel.
4. **Lock without power.** Holding a grip or a standing pose costs zero current. Self-locking
   gearing (worm or high-ratio spur) plus a friction drag that slips before anything breaks.
5. **Regional winch banks.** Hand tendons are driven from the forearm, arm tendons from the
   torso, leg tendons from the pelvis/thigh, exactly like the body. One motor per tendon, but all
   the motors of a region sit in one printed block with one controller.
6. **Measure, then trust.** Every motor and line figure in `docs/` is a catalogue ballpark until a
   bench test replaces it. The calculator in `tools/` is the single source of those numbers.

## Repository map

| Path | What |
| --- | --- |
| `docs/01-architecture.md` | System overview, DOF budget, actuator classes, compute topology, honest limits |
| `docs/02-tendon-actuator.md` | The winch: physics, lock options, drum rules, line choice, creep, calculator results |
| `docs/03-skeleton-joints.md` | Joint types per body region, ROM targets vs human, materials, print segmentation |
| `docs/04-electronics.md` | Boards, bus, drivers, encoders, power |
| `docs/05-bom-estimate.md` | Rough cost and mass |
| `docs/06-roadmap.md` | Phases with exit criteria, starting with one finger on one winch, ending in dynamic walking |
| `docs/07-locomotion.md` | Walking and balancing: joint loads from gait, W-max sizing, sensors, controller stages, feet |
| `docs/decisions.md` | Decision log (what we chose and why; revisit when a test disagrees) |
| `docs/prompt-log.md` | Every prompt that produced a set of work, verbatim, with what it produced |
| `tools/tendon_calc.py` | Force / speed / power / line-safety calculator for any motor + drum + tackle combination |
| `cad/blender/winch_mini.py` | Parametric Blender generator for the W-mini winch (N20 motor); exports STL |
| `cad/out/` | Generated STLs (regenerate, do not hand-edit) |

## Quick start

```powershell
# sizing numbers for every reference joint
python tools/tendon_calc.py

# generate the W-mini winch housing, drum and lid as STL (Blender 4.2+ / 5.x)
& "C:\Program Files\Blender Foundation\Blender 5.1\blender.exe" -b -P cad/blender/winch_mini.py -- --out cad/out
```

## Status

Design phase. Nothing has been printed yet. First hardware milestone is Phase 0 in the roadmap:
one printed finger, one W-mini winch, one ESP32, closing and holding without power.
