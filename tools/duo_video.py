"""
duo_video.py - sim_video's recorder for two robots in one model (docs/48): a fixed side camera on both, and robot B's
lines and drums drawn as robot A's are. The HUD's state line is A's; B's lines are listed as "B <name>".

Used by tools/duo.py --one ... --video <name> [--gif].
"""
import collections
import os
import sys

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_video  # noqa: E402

LOOKAT = (0.0, -0.32, 0.72)
DISTANCE, AZIMUTH, ELEVATION = 3.3, 200, -10


class Recorder(sim_video.Recorder):
    def __init__(self, duo, path, gif=None, key="duo"):
        super().__init__(duo.a, path, gif=gif, gif_scenes=(key,))
        m = duo.model
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.cam.lookat[:] = LOOKAT
        self.cam.distance, self.cam.azimuth, self.cam.elevation = DISTANCE, AZIMUTH, ELEVATION
        for w in duo.b.w:
            drum = m.body(duo.b.prefix + w["name"] + "_drum").id
            jid = m.joint(duo.b.prefix + w["name"]).id
            view = collections.ChainMap({"name": "B " + w["name"]}, w)      # the live winch state, under B's name
            self.draw.append((view, drum, m.body_geomadr[drum], m.jnt_bodyid[jid], m.jnt_axis[jid].copy()))
