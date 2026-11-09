"""
chain_video.py - record one chained run (docs/38, tools/chain.py) with the gauntlet's recorder (tools/sim_video.py):
every line's tension, the drums' states, CoM and CoP, the segment named in the corner.

    python tools/chain_video.py --bag grocery --order pick-walk-sit-place --gif --out chain
    python tools/chain_video.py --seed 7 --gif            # a chain.py seed's bag and order
    python tools/chain_video.py ... NOSLIP=5              # K=V module overrides of sim_tendon, as chain.py

Writes docs/img/<out>.mp4 (and .gif with --gif, every segment). Prints the per-segment result as chain.py does.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import chain  # noqa: E402

TITLES = dict(pick="Pick the bag up from a stool", walk="Walk: four 100 mm steps and the closing step",
              sit="Sit on a 0.45 m chair and stand again", place="Put the bag down on a top", stand="Stand")


def main():
    args = sys.argv[1:]

    def opt(k, default=None):
        if k in args:
            i = args.index(k)
            v = args[i + 1]
            del args[i:i + 2]
            return v
        return default
    gif = "--gif" in args
    if gif:
        args.remove("--gif")
    seed, bag, order = opt("--seed"), opt("--bag"), opt("--order")
    name = opt("--out", "chain")
    fps = int(opt("--fps", 30))
    if seed is not None:
        item, order = chain.order_for(int(seed))
    else:
        item, order = bag or "grocery", (order or "pick-walk-sit-place").split("-")
    if isinstance(order, str):
        order = order.split("-")
    import export_mjcf
    export_mjcf.ITEMS = chain.items_for(item)
    export_mjcf.STOOLS = 4
    import sim_tendon as st
    st.STOOL_HALF = export_mjcf.STOOL_HALF
    for kv in args:
        k, v = kv.split("=", 1)
        setattr(st, k, eval(v, vars(st)))
    import sim_video
    r = st.Rig()
    r.reset()
    root = os.path.dirname(HERE)
    out = os.path.join(root, "docs", "img", name + ".mp4")
    rec = sim_video.Recorder(r, out, fps=fps, gif=out[:-4] + ".gif" if gif else None, gif_scenes=tuple(order))
    segs, state, carry = chain.segments(st, r, item)
    bag_note = "%s bag, %.1f kg" % (item, [b for b in chain.BAGS if b["name"] == item][0]["kg"])
    notes, failed = [], None
    for k, seg in enumerate(order):
        rec.scene(seg, "%d/%d  %s" % (k + 1, len(order), TITLES.get(seg, seg)), bag_note)
        try:
            note = segs[seg](r)
            if chain.HANDBACK:
                r.to_stand()
            if not r.standing():
                raise chain.Fail("fell")
            notes.append("%s ok%s" % (seg, (" (" + note + ")") if note else ""))
        except chain.Fail as e:
            failed = seg
            notes.append("%s FAILED: %s" % (seg, e))
            r.run(1.0)
            break
    n = rec.close()
    print("   " + "; ".join(notes))
    print("%s %s: %s, %.1f s; wrote %s (%d frames)%s" % (item, "-".join(order), "ok" if not failed else "F:" + failed,
                                                       r.data.time, out, n, " and .gif" if gif else ""))


if __name__ == "__main__":
    main()
