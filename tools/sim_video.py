#!/usr/bin/env python3
"""
sim_video.py - record the tendon model (tools/sim_tendon.py) running the gauntlet, with every line, clutch and
motor shown doing its work.

What is drawn on top of the MuJoCo render:
  - each winch's two line runs, from the drum to the joint it drives: grey and thin when slack, green to
    yellow to red as the tension goes from zero to the line's continuous rating and beyond, thicker with load
  - the drum disc of each winch: blue when the self-lock holds it with the motor off, orange when the motor is
    driving (brightness with torque), red while the clutch slips
  - the centre of mass (blue ball), the measured centre of pressure (orange ball on the floor) and the CoP the
    balance loop asked for (white ball)
  - a HUD: scene title, time, stands/falls, electrical power, CoM offset, the worst five lines as bars

Output: an MP4 (ffmpeg on PATH, libx264) and optionally a GIF for the docs.

Run:  python tools/sim_video.py                      -> docs/img/gauntlet.mp4
      python tools/sim_video.py --gif --fps 20       also docs/img/gauntlet.gif (highlights only)
      python tools/sim_video.py --only stand,push30  scenes by key (see sim_tendon.scenarios)
"""
import math
import os
import shutil
import subprocess
import sys

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_tendon as st  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
W, H = 960, 720


def tension_rgba(ratio):
    """0 -> green, 0.5 -> yellow, >= 1 -> red."""
    r = min(1.0, max(0.0, ratio))
    if r < 0.5:
        return (2 * r, 1.0, 0.1, 1.0)
    return (1.0, max(0.0, 2.0 - 2 * r), 0.1, 1.0)


class Recorder:
    def __init__(self, rig, path, fps=30, size=(W, H), gif=None, gif_scenes=()):
        self.rig, self.fps, self.size = rig, fps, size
        m = rig.model
        self.renderer = mujoco.Renderer(m, size[1], size[0])
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        self.cam.trackbodyid = rig.pelvis
        self.cam.distance, self.cam.azimuth, self.cam.elevation = 3.0, 325, -12
        self.opt = mujoco.MjvOption()
        self.font = ImageFont.load_default(size=18)
        self.small = ImageFont.load_default(size=14)
        self.title, self.note = "", ""
        self.next_t = 0.0
        self.e_prev, self.power = 0.0, 0.0
        self.frames = 0
        self.path = path
        self.gif, self.gif_scenes, self.gif_frames = gif, set(gif_scenes), []
        self.key = ""
        # per winch: drum geom id, drum and joint body ids, joint axis in the body frame for the line offset
        self.draw = []
        for w in rig.w:
            drum = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, w["name"] + "_drum")
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, w["name"])
            self.draw.append((w, drum, m.body_geomadr[drum], m.jnt_bodyid[jid], m.jnt_axis[jid].copy()))
        self.base_rgba = m.geom_rgba.copy()
        os.makedirs(os.path.dirname(path), exist_ok=True)
        ffmpeg = shutil.which("ffmpeg")
        if not ffmpeg:
            raise SystemExit("ffmpeg not found on PATH")
        self.proc = subprocess.Popen(
            [ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", "%dx%d" % size,
             "-r", str(fps), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "24", path],
            stdin=subprocess.PIPE)
        rig.hook = self.on_step

    def scene(self, key, title, note=""):
        self.key, self.title, self.note = key, title, note
        self.next_t = 0.0

    def on_step(self, rig):
        if rig.data.time + 1e-9 < self.next_t:
            return
        self.next_t = rig.data.time + 1.0 / self.fps
        self.frame()

    def add_ball(self, scn, pos, radius, rgba):
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([radius, 0, 0]), np.asarray(pos, dtype=float),
                            np.eye(3).flatten(), np.asarray(rgba, dtype=np.float32))
        scn.ngeom += 1

    def add_line(self, scn, a, b, width, rgba):
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.eye(3).flatten(),
                            np.asarray(rgba, dtype=np.float32))
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, width, np.asarray(a, dtype=float),
                             np.asarray(b, dtype=float))
        scn.ngeom += 1

    def frame(self):
        rig = self.rig
        m, d = rig.model, rig.data
        # colour the drums by state before the scene is built
        worst = []
        for w, drum, gid, jbody, axis in self.draw:
            p = w["p"]
            if w.get("_slipping"):
                m.geom_rgba[gid] = (1.0, 0.1, 0.1, 1.0)
            elif w.get("_locked"):
                m.geom_rgba[gid] = (0.2, 0.4, 1.0, 1.0)
            else:
                k = min(1.0, abs(d.actuator_force[w["act"]]) / p["t_stall"] / 0.3)
                m.geom_rgba[gid] = (0.5 + 0.5 * k, 0.3 + 0.3 * k, 0.1, 1.0)
        self.renderer.update_scene(d, self.cam, self.opt)
        scn = self.renderer.scene
        for w, drum, gid, jbody, axis in self.draw:
            p = w["p"]
            a = d.xpos[drum]
            b = d.xpos[jbody]
            # the two runs leave the drum on either side and reach the pulley on either side of its axis
            off = (d.xmat[jbody].reshape(3, 3) @ axis) * p["r_p"]
            for t, sgn in ((w["tA"], 1), (w["tB"], -1)):
                f = rig.line_force(t)
                ratio = f / p["f_cont"]
                worst.append((ratio, w["name"], f, p["f_cont"]))
                if f <= 0.0:
                    self.add_line(scn, a + sgn * off * 0.3, b + sgn * off, 0.0015, (0.55, 0.55, 0.6, 0.5))
                else:
                    self.add_line(scn, a + sgn * off * 0.3, b + sgn * off, 0.002 + 0.004 * min(ratio, 1.5),
                                  tension_rgba(ratio))
        com = d.subtree_com[rig.pelvis]
        self.add_ball(scn, com, 0.03, (0.2, 0.4, 1.0, 0.9))
        self.add_ball(scn, (com[0], com[1], 0.003), 0.015, (0.2, 0.4, 1.0, 0.9))
        cop, loads = rig.cop()
        if cop is not None:
            self.add_ball(scn, (cop[0], cop[1], 0.003), 0.02, (1.0, 0.55, 0.1, 1.0))
        if rig.balance and d.time > st.BAL_START:
            self.add_ball(scn, (rig.cop_cmd[0], rig.cop_cmd[1], 0.003), 0.012, (1.0, 1.0, 1.0, 1.0))
        img = Image.fromarray(self.renderer.render())
        self.hud(img, worst, loads)
        self.proc.stdin.write(img.tobytes())
        self.frames += 1
        if self.gif and self.key in self.gif_scenes and self.frames % 3 == 0:
            self.gif_frames.append(img.resize((480, 360), Image.BILINEAR))

    def hud(self, img, worst, loads):
        rig = self.rig
        dr = ImageDraw.Draw(img, "RGBA")
        e_tot = sum(s["e_j"] for s in rig.stats.values())
        dt = 1.0 / self.fps
        self.power = 0.8 * self.power + 0.2 * (e_tot - self.e_prev) / dt
        self.e_prev = e_tot
        z, tilt = rig.pelvis_state()
        e, heel, toe = rig.support()
        dr.rectangle((0, 0, img.width, 92), fill=(0, 0, 0, 150))
        dr.text((12, 8), self.title, font=self.font, fill=(255, 255, 255))
        state = "STANDS" if rig.standing() else "FALLS"
        dr.text((12, 34), "t = %5.2f s   %s   pelvis %.2f m  tilt %.0f deg   motors %4.0f W   CoM %+.0f mm fwd of sole centre"
                % (rig.data.time, state, z, tilt, self.power, -e[1] * 1e3), font=self.small,
                fill=(255, 120, 120) if state == "FALLS" else (220, 220, 220))
        dr.text((12, 54), self.note, font=self.small, fill=(200, 200, 200))
        dr.text((12, 72), "feet %3.0f / %3.0f N   lines: grey slack, green..red = 0..continuous rating; drums: blue locked, orange driving, red clutch slip"
                % (loads[0], loads[1]), font=self.small, fill=(170, 170, 170))
        # worst five lines as bars
        worst.sort(reverse=True)
        x0, y0 = img.width - 330, 104
        dr.rectangle((x0 - 10, y0 - 8, img.width - 8, y0 + 5 * 22 + 6), fill=(0, 0, 0, 140))
        for i, (ratio, name, f, fc) in enumerate(worst[:5]):
            y = y0 + i * 22
            dr.text((x0, y), "%-18s %4.0f N" % (name, f), font=self.small, fill=(230, 230, 230))
            c = tuple(int(255 * v) for v in tension_rgba(ratio)[:3])
            dr.rectangle((x0 + 200, y + 4, x0 + 200 + int(110 * min(ratio, 1.1)), y + 14), fill=c)
            dr.line((x0 + 300, y + 2, x0 + 300, y + 16), fill=(255, 255, 255))

    def close(self):
        self.proc.stdin.close()
        self.proc.wait()
        self.rig.hook = None
        if self.gif and self.gif_frames:
            self.gif_frames[0].save(self.gif, save_all=True, append_images=self.gif_frames[1:],
                                    duration=int(3000 / self.fps), loop=0, optimize=True)
        return self.frames


def main():
    args = sys.argv[1:]
    fps = int(args[args.index("--fps") + 1]) if "--fps" in args else 30
    only = args[args.index("--only") + 1].split(",") if "--only" in args else None
    name = args[args.index("--out") + 1] if "--out" in args else "gauntlet"
    out = os.path.join(ROOT, "docs", "img", name + ".mp4")
    gif = os.path.join(ROOT, "docs", "img", name + ".gif") if "--gif" in args else None
    if "--hip50" in args:
        import export_mjcf
        export_mjcf.HIP_ROLL_60 = False
    rig = st.Rig()
    rec = Recorder(rig, out, fps=fps, gif=gif, gif_scenes=("stand", "push30", "lean15", "onefoot"))
    for key, title, note, fn in st.scenarios():
        if only and key not in only:
            continue
        rec.scene(key, title, note)
        rig.reset()
        fn(rig)
        rig.report(title, top=5)
    n = rec.close()
    print("wrote %s (%d frames at %d fps)%s" % (out, n, fps, " and " + gif if gif else ""))


if __name__ == "__main__":
    main()
