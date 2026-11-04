#!/usr/bin/env python3
"""
record_policy.py - record a learned policy (the sibling ai-embedded-dynamic-diversity's rl/) running on the full
tendon model, drawn as tools/sim_video.py draws the gauntlet (line tensions, drum states, CoM, CoP, HUD).

The policy runs through the sibling's gate runner (rl/tendon_eval.TendonPolicyRunner), so what is recorded is
what the gate scores: the same observation, rate, locks and (with --hybrid) the hand-written ankle balance
under the policy (docs/32, D-053). One recording per process (a second MuJoCo renderer in one process
renders black).

Run from the sibling repo root, its venv, with its src on the path:
  PYTHONPATH=src ROBOT_REPO=../robot .venv/Scripts/python.exe ../robot/tools/record_policy.py \
      artifacts/rl/seeds/mlp-bc-s3.pt --wind 8 --hybrid --out wind8-hybrid
      -> docs/img/wind8-hybrid.mp4 and .gif
Options: --wind M (head wind M m/s, gusts 1.5 m/s; default none), --push N (an N newton push from behind
at 1.5 s), --secs S (default 10), --hybrid, --out NAME, --title TEXT.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))


def main():
    args = sys.argv[1:]
    if not args or args[0].startswith("--"):
        raise SystemExit(__doc__)

    def opt(name, default=None):
        return args[args.index(name) + 1] if name in args else default

    from ai_embedded_dynamic_diversity.rl.tendon_eval import TendonPolicyRunner
    import sim_video

    pol = args[0]
    hybrid = "--hybrid" in args
    wind = float(opt("--wind", 0.0))
    push = float(opt("--push", 0.0))
    secs = float(opt("--secs", 10.0))
    out = opt("--out", "policy")
    runner = TendonPolicyRunner(pol, hand_ankles=hybrid)
    st = runner.st
    sim_video.st = st                        # the sim_tendon module the runner loaded (one copy of its globals)
    if wind > 0:
        st.WIND = dict(mean=wind, gust=1.5, dir="head", seed=0)
    runner.reset()
    who = os.path.splitext(os.path.basename(pol))[0]
    title = opt("--title", ("Hybrid (hand-written ankle balance under %s)" if hybrid else "Learned policy %s") % who)
    note = ("wind %.0f m/s head, gusts 1.5 m/s" % wind if wind else "") + ("  push %.0f N at 1.5 s" % push if push else "")
    mp4 = os.path.join(ROOT, "docs", "img", out + ".mp4")
    rec = sim_video.Recorder(runner.rig, mp4, fps=20, gif=mp4[:-4] + ".gif", gif_scenes=("policy",))
    pol_hook = runner._hook
    runner.rig.hook = lambda r: (pol_hook(r), rec.on_step(r))
    rec.scene("policy", title, note.strip())
    r = runner.rig
    if push:
        r.run(1.5)
        r.push("spine", (0, -push, 0), 0.2)
        r.run(max(secs - 1.5, 0.0))
    else:
        r.run(secs)
    z, tilt = r.pelvis_state()
    print("%s: %s, pelvis %.3f m, tilt %.1f deg" % (out, "stands" if r.standing() else "falls", z, tilt))
    n = rec.close()
    print("wrote %s (%d frames) and %s" % (mp4, n, mp4[:-4] + ".gif"))


if __name__ == "__main__":
    main()
