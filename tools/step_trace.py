"""Trace a scenario every 0.1 s over a time window: the quantities every fall in docs/16 and docs/18 was found
with (foot loads, CoM error and velocity, pelvis attitude, hip shift and roll commands, ankle positions, leg
joint angles against their targets, the swing foot). Use it on the run a step_sweep row says fails.

Run:
    python tools/step_trace.py --from 13.5 --to 20 [--every 0.1] [--scene step] [--stiff-mini] [K=V ...]
"""
import contextlib
import io
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def main():
    argv = sys.argv[1:]

    def opt(name, default):
        if name in argv:
            i = argv.index(name)
            v = argv[i + 1]
            del argv[i:i + 2]
            return v
        return default

    t0, t1 = float(opt("--from", "0")), float(opt("--to", "1e9"))
    every, scene = float(opt("--every", "0.1")), opt("--scene", "step")
    if "--stiff-mini" in argv:
        argv.remove("--stiff-mini")
        import export_mjcf
        export_mjcf.STIFF_MINI = True
    import mujoco
    import sim_tendon as st
    for kv in argv:
        k, v = kv.split("=", 1)
        setattr(st, k, eval(v, vars(st)))
    rig = st.Rig()
    m, d = rig.model, rig.data
    jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    last = [-1e9]
    deg = math.degrees

    def hook(r):
        if not (t0 <= d.time <= t1) or d.time - last[0] < every - 1e-9:
            return
        last[0] = d.time
        R = d.xmat[r.pelvis].reshape(3, 3)
        pitch, roll = deg(math.atan2(R[2, 1], R[1, 1])), deg(math.atan2(R[2, 0], R[0, 0]))
        _, loads = r.cop()
        q = lambda n: deg(d.qpos[r.by_name[n]["q"]])
        a = lambda n: deg(r.applied.get(n, 0.0))
        ank = {s: d.xanchor[jid("foot.%s_pitch" % s)] * 1e3 for s in "LR"}
        print("%6.2f %-12s load %3.0f/%3.0f  e (%4.0f,%4.0f) v (%5.0f,%5.0f)  pelvis p %5.1f r %5.1f  shift %5.1f"
              "  ankle L (%4.0f,%5.0f,%4.0f) R (%4.0f,%5.0f,%4.0f)  hip p L %5.1f/%5.1f R %5.1f/%5.1f"
              "  knee L %5.1f/%5.1f R %5.1f/%5.1f  roll L %5.1f R %5.1f  swing %s"
              % (d.time, r.phase[:12], loads[0], loads[1], 1e3 * r._com_f[0], 1e3 * r._com_f[1],
                 1e3 * r._com_v[0], 1e3 * r._com_v[1], pitch, roll, deg(r.hip_shift),
                 *ank["L"], *ank["R"], q("thigh.L_pitch"), a("thigh.L_pitch"), q("thigh.R_pitch"), a("thigh.R_pitch"),
                 q("shank.L_pitch"), a("shank.L_pitch"), q("shank.R_pitch"), a("shank.R_pitch"),
                 q("thigh.L_roll"), q("thigh.R_roll"), r.swing))

    print("  t    phase        loads N    CoM error / velocity mm, mm/s; angles deg (measured/applied); ankles mm (x lateral, y back, z)")
    for key, title, note, fn in st.scenarios():
        if key != scene:
            continue
        rig.reset()
        rig.hook = hook
        fn(rig)
        rig.report(title, top=4)


if __name__ == "__main__":
    main()
