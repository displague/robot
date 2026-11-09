---
name: project-loop
description: How work is recorded in the robot repo (prompt log, one commit per prompt quoting it, numbered decisions and docs, branches, durable skills, the sibling ai-embedded-dynamic-diversity, the Agent Hub workspace). Use at the start of any session and before every commit.
---

# Working loop

1. **Every user prompt** goes verbatim into `docs/prompt-log.md` as the next `P-nnn` (date, the prompt
   quoted, then what it produced).
2. **One commit per set of work**, the message quoting the prompt verbatim (D-012), ending with the
   attribution lines the session supplies. Push after committing; the user wants the remote current.
3. **Decisions** are numbered `D-nnn` in `docs/decisions.md`: date, title, Why, the alternative,
   Reverses if. Reference them from code comments and docs.
4. **Docs** are numbered by topic (`docs/NN-*.md`); each experiment doc ends with "Not yet" and "Next"
   sections, which are the backlog. Each starts with YAML front matter (`title`: its H1, `summary`: one or two
   sentences of what it found and its decisions, `section`: one of `tools/docs_index.SECTIONS`). Then run
   `python tools/docs_index.py` and commit the regenerated `docs/index.md` (CI runs `--check`; `mkdocs serve`
   browses the site, `pip install -r requirements-docs.txt`). Docs are not listed in the README; new tools are.
5. **Branches**: small work on `main`. Work that breaks a gauntlet scene until a follow-up fixes it goes
   on a `feat/*` branch, pushed, and merged when the gauntlet passes again.
6. **Durable know-how goes in `.claude/skills/`**, not only in chat or memory: when a tool, generator or
   workflow is created or learned, add or update its skill in the same commit. Scratch scripts that turn
   out to be needed again (renders, sweeps) become repo tools with a skill entry.
7. **Sibling project** `../ai-embedded-dynamic-diversity` (github displague/ai-embedded-dynamic-diversity):
   learned control (`rl/`). Work there in a new worktree from master and follow its
   `skills/dev-loop/SKILL.md` (issue, PR, squash merge; merges and releases are permitted). Its venv has
   MuJoCo and CUDA torch: `PYTHONPATH=src ROBOT_REPO=../robot ../ai-embedded-dynamic-diversity/.venv/Scripts/python.exe`.
8. **Agent Hub** (https://ah.meyu.us/, workspace `robot`): next steps are hub tasks; claim before
   working, finish with an honest result (what passed, what did not, commits). Findings are records
   tagged `project:robot`.

## Tools at a glance

| Area | Skill |
| --- | --- |
| Printable parts, presets, renders | `cad-generators` |
| MuJoCo tendon model, gauntlet, recordings, teacher data | `tendon-sim` |
| Tags and GitHub releases | `release` |
| Drive-train numbers (motor, line, drum) | `python tools/tendon_calc.py`, the single source of the figures in docs |
