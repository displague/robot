---
name: release
description: Tag and publish a robot release on GitHub the way the sibling project does (annotated tag, detailed notes with visuals, attached STLs, models and recordings). Use when the user asks for a tag or release, or a milestone (a doc's Next item) lands.
---

# Releases

Tags are `vMAJOR.MINOR.PATCH`, annotated, on `main`, and every tag gets a GitHub release with notes in the
sibling project's form (see its `skills/dev-loop/references/release-history.md` and its v0.3.0 release).
Tags can be issued whenever a milestone fits (user, 2026-10-28). Minor bump for a milestone (a new
capability in simulation, a printable preset set), patch for fixes.

## Steps

1. Main is clean and pushed; the docs, decisions and prompt log describe the state.
2. Visuals current: re-render changed parts (`cad-generators` skill), re-record changed scenes
   (`tendon-sim` skill), commit them.
3. `git tag -a vX.Y.Z -m "<title>" -m "<one-paragraph state>"`, then `git push origin vX.Y.Z`.
4. Notes file in the scratchpad (never pipe text with emoji through the cp1252 console), sections:
   - **Summary**: what the robot can do at this tag, and what is not yet real (nothing printed, etc.).
   - A visuals table of the recordings and renders. The repo is public (D-054): reference images as
     `https://raw.githubusercontent.com/displague/robot/vX.Y.Z/docs/img/<file>`.
   - **What changed** (numbered, naming docs and decisions), **Results** (a table of gauntlet and step
     verdicts and the numbers that moved), **Known gaps**, **Attached**.
5. Assets: `robot-vX.Y.Z-stl.zip` built fresh from the generators at the tag. `cad/out` is gitignored, so
   `git archive` of it is empty: run the generators at the tagged source into a scratch `cad/out` and
   zip that. Also `sim/robot.xml`, `sim/robot_standin.xml/.json` and the GIFs.
   The STL zip is CC BY-SA 4.0 and the code MIT (D-055): say so in the notes' Attached section.
6. `gh release create vX.Y.Z -R displague/robot --title "vX.Y.Z — <title>" --notes-file <notes> --verify-tag <assets...>`
7. Add the entry to `docs/releases.md` (newest first) and name the release in the prompt-log entry.
