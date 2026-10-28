# Releases

Each tag has a GitHub release (github.com/displague/robot/releases) with notes, visuals and assets:
the generated STLs, the MuJoCo models and the recordings. How to make one: `.claude/skills/release/SKILL.md`.

## v0.1.0 (2026-10-28) — The tendon humanoid in simulation: drive trains, balance, one foot, first step

**State:** the design (docs 01 to 08), the parametric CAD (knuckle chain, ring joint, winches, finger,
skeleton rig), the tendon-level MuJoCo model with every drive-train element (docs 13), the balance
controller (30 N pushes, sustained 15 N, arm hold, squat, weight shift; docs 14), the one-foot hold
(Phase 5 exit test in simulation; docs 15), the first static step (Phase 6a; docs 16), the fast stand-in
and teacher demonstrations, and the first learned controller in the sibling project's v0.3.0 (docs 17).

**Results:** stand and pushes to 30 N pass; 40 N falls; one-foot hold passes; the static step completes
with the feet 18 and 29 mm long; the hand-written controller stands 177 of 200 random pushes; the learned
policy stands 90 % on the stand-in and fails the tendon gate.

**Known gaps:** the neck over its rating (fixed in D-032 after this tag), landing accuracy, step speed,
the learned controller's stand-in gap.

**Assets:** robot-v0.1.0-stl.zip, robot.xml, robot_standin.xml/.json, gauntlet.gif, onefoot.gif, step.gif.

**Decisions:** D-001 to D-031. **Prompts:** P-001 to P-025.
