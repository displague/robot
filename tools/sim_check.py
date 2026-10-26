#!/usr/bin/env python3
"""
sim_check.py - load sim/robot.urdf into MuJoCo, add a floor, hold every joint at its rest angle with a
stiff spring (the "locked winch" statue of docs/08), drop the robot 1 cm, simulate, and report whether
it stands. Then push it at shoulder height and report the push it survives.

Run:  python tools/sim_check.py            (needs: pip install mujoco)
"""
import os
import sys

try:
    import mujoco
except ImportError:
    print("mujoco not installed: pip install mujoco")
    sys.exit(1)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(ROOT, "sim", "robot.urdf")


def build(stiffness=400.0, damping=20.0):
    spec = mujoco.MjSpec.from_file(URDF)
    spec.option.timestep = 0.002
    floor = spec.worldbody.add_geom()
    floor.type = mujoco.mjtGeom.mjGEOM_PLANE
    floor.size = [5, 5, 0.1]
    floor.name = "floor"
    for g in spec.geoms:
        g.friction = [1.0, 0.005, 0.0001]
    model = spec.compile()
    # locked winch = a stiff spring holding the rest angle (gives a little, like a tendon); set on the
    # compiled model so this works across MjSpec API versions
    for j in range(model.njnt):
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE:
            model.jnt_stiffness[j] = stiffness
            model.dof_damping[model.jnt_dofadr[j]] = damping
    return model


def settle(model, data, seconds):
    for _ in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)


def pelvis_state(model, data):
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    z = data.xpos[bid][2]
    up = data.xmat[bid].reshape(3, 3)[:, 2]   # pelvis z axis in world
    import math
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up[2]))))
    return z, tilt


def push_test(force, seconds=0.2):
    model = build()
    data = mujoco.MjData(model)
    data.qpos[2] += 0.01
    settle(model, data, 1.5)
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "spine")
    n = int(seconds / model.opt.timestep)
    for _ in range(n):
        data.xfrc_applied[bid] = [0, -force, 0, 0, 0, 0]   # push forward (-Y) at the torso
        mujoco.mj_step(model, data)
    data.xfrc_applied[bid] = 0
    settle(model, data, 3.0)
    z, tilt = pelvis_state(model, data)
    return z, tilt


def main():
    model = build()
    data = mujoco.MjData(model)
    data.qpos[2] += 0.01
    settle(model, data, 3.0)
    z, tilt = pelvis_state(model, data)
    print("mujoco %s: %d bodies, %d DOF, mass %.1f kg" % (mujoco.__version__, model.nbody, model.nv, sum(model.body_mass)))
    print("statue after 3 s: pelvis height %.3f m (rest 0.950), tilt %.1f deg -> %s" %
          (z, tilt, "STANDS" if z > 0.85 and tilt < 10 else "FALLS"))
    for f in (10, 20, 30, 40, 60):
        z, tilt = push_test(f)
        print("push %3d N for 0.2 s at the torso: pelvis %.3f m, tilt %5.1f deg -> %s" %
              (f, z, tilt, "stands" if z > 0.85 and tilt < 10 else "falls"))


if __name__ == "__main__":
    main()
