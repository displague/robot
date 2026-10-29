"""The balance loop and planner on modelled sensors instead of the simulator's truth (docs/26, P-040).

CoM (IMU + joint encoders + the mass model): sampled at RATE, DELAY late, a constant bias (mass model error), a slow
wander (OU, kinematics and posture-dependent mass error) and white noise; linearly interpolated between samples
as an estimator would. Foot load and CoP: 4 corner sensors per sole (docs/07: 4 per foot at the sole corners);
each contact force is split to the corners bilinearly; each corner reads gain * f (gain error per sensor),
hysteresis (reads low by HYST of the load while unloading), drift (+DRIFT per decade of loaded seconds),
saturation SAT per corner, white noise; same RATE and DELAY.

Run:
    python tools/sensor_sweep.py sweep cases=truth,lc_fast,fsr_fast,fsr_bad scenes=push40,stand winds=10:0:head,8:1.5:left
    python tools/sensor_sweep.py one fsr_fast step 0 0 head          # one run, one RESULT line
Cases are CASES below (truth, ideal, only_<error>, delay<ms>, loadcell, fsr, lc_fast, fsr_fast, fsr_bad); JOBS sets
the parallel runs. Each cell: S/F, g if a lift gate timed out, the first fall (phase@time), electrical J."""
import io, contextlib, math, os, subprocess, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mujoco
import sim_tendon as st

CASES = {
    "truth": None,
    # load cells (HX711 + half-bridge per corner): 1 % gain, no hysteresis, 0.5 N noise
    "loadcell": dict(rate=200, delay=0.010, com_bias=0.010, com_wander=0.003, com_noise=0.0005,
                     gain=0.01, hyst=0.0, drift=0.0, sat=400.0, f_noise=0.5),
    # FSR 402 behind a load-sharing pad, calibrated: 6 % part-to-part, 10 % hysteresis, 5 %/decade drift,
    # each corner saturating at 80 N of corner load (the pad carries the rest up to the FSR's 20 N)
    "fsr": dict(rate=200, delay=0.010, com_bias=0.010, com_wander=0.003, com_noise=0.0005,
                gain=0.06, hyst=0.10, drift=0.05, sat=80.0, f_noise=1.0),
    # FSRs, worse: 15 % gain spread (uncalibrated), saturating at 50 N, 20 mm CoM bias, 20 ms delay
    "fsr_bad": dict(rate=100, delay=0.020, com_bias=0.020, com_wander=0.005, com_noise=0.001,
                    gain=0.15, hyst=0.10, drift=0.05, sat=50.0, f_noise=2.0),
}
_Z = dict(rate=1000, delay=0.0, com_bias=0.0, com_wander=0.0, com_noise=0.0, gain=0.0, hyst=0.0, drift=0.0,
          sat=1e4, f_noise=0.0)
CASES["ideal"] = dict(_Z)
for k, v in dict(rate=200, delay=0.010, com_bias=0.010, com_wander=0.003, com_noise=0.0005, gain=0.06, hyst=0.10,
                 drift=0.05, sat=80.0, f_noise=1.0).items():
    CASES["only_" + k] = _Z | {k: v}
for ms in (2, 5, 10, 20):
    CASES["delay%d" % ms] = _Z | dict(delay=ms / 1000)
# the build's sensors: a 1 kHz ESP32 loop, delay 3 ms (BNO085 fusion and ADC), smooth CoM error, quantised encoders
CASES["lc_fast"] = CASES["loadcell"] | dict(rate=1000, delay=0.003, com_noise=0.0002)
CASES["fsr_fast"] = CASES["fsr"] | dict(rate=1000, delay=0.003, com_noise=0.0002)
SH = st.SOLE_HALF
CORNERS = np.array([(sx * SH[0], sy * SH[1], 0.0) for sx in (-1, 1) for sy in (-1, 1)])


class _Proxy:
    def __init__(self, d, com, pelvis):
        object.__setattr__(self, "_d", d)
        sc = d.subtree_com.copy()
        sc[pelvis] = com
        object.__setattr__(self, "subtree_com", sc)

    def __getattr__(self, k):
        return getattr(self._d, k)


class SensedRig(st.Rig):
    def __init__(self, case, seed=0):
        self.sense = CASES[case]
        self._srng = np.random.default_rng(1000 + seed)
        super().__init__()

    def reset(self):
        super().reset()
        if not self.sense:
            return
        c, rng = self.sense, self._srng
        a = rng.uniform(0, 2 * math.pi)
        self._bias = c["com_bias"] * np.array([math.cos(a), math.sin(a), 0.0])
        self._wander = np.zeros(3)
        self._gain = 1 + c["gain"] * rng.standard_normal((2, 4))
        self._prev_f = np.zeros((2, 4))
        self._loaded_t = np.zeros((2, 4))
        self._buf = []                      # (t, com, corner world positions (2,4,3), corner readings (2,4))
        self._next = 0.0
        self._sample()

    def _corners_true(self):
        m, d = self.model, self.data
        f6 = np.zeros(6)
        out = np.zeros((2, 4))
        pos = np.zeros((2, 4, 3))
        for k, b in enumerate(self.feet):
            R = d.xmat[b].reshape(3, 3)
            c0 = d.xpos[b] + R @ np.array(st.SOLE_CENTRE)
            pos[k] = c0 + CORNERS @ R.T
        for i in range(d.ncon):
            c = d.contact[i]
            if self.floor not in (c.geom1, c.geom2):
                continue
            other = c.geom2 if c.geom1 == self.floor else c.geom1
            for k, b in enumerate(self.feet):
                if m.geom_bodyid[other] != b:
                    continue
                mujoco.mj_contactForce(m, d, i, f6)
                R = d.xmat[b].reshape(3, 3)
                c0 = d.xpos[b] + R @ np.array(st.SOLE_CENTRE)
                p = R.T @ (c.pos - c0)
                u = np.clip((p[0] / SH[0] + 1) / 2, 0, 1)
                v = np.clip((p[1] / SH[1] + 1) / 2, 0, 1)
                w = [(1 - u) * (1 - v), (1 - u) * v, u * (1 - v), u * v]
                out[k] += f6[0] * np.array(w)
        return out, pos

    def _sample(self):
        c, rng, d = self.sense, self._srng, self.data
        dt = 1.0 / c["rate"]
        f, pos = self._corners_true()
        # OU wander of the CoM estimate, tau 1 s
        # smooth wander (posture-dependent mass-model and kinematic error): OU at tau 1 s through a 0.3 s lag
        self._wander += -self._wander * dt + c["com_wander"] * math.sqrt(2 * dt) * rng.standard_normal(3) * [1, 1, 0]
        self._wander_s = getattr(self, "_wander_s", np.zeros(3))
        self._wander_s += (self._wander - self._wander_s) * dt / 0.3
        com = d.subtree_com[self.pelvis] + self._bias + self._wander_s + c["com_noise"] * rng.standard_normal(3) * [1, 1, 0]
        self._loaded_t = np.where(f > 2.0, self._loaded_t + dt, 0.0)
        drift = 1 + c["drift"] * np.log10(1 + self._loaded_t)
        hyst = np.where(f < self._prev_f - 0.5, 1 - c["hyst"], 1.0)
        self._prev_f = f
        r = np.clip(self._gain * drift * hyst * f + c["f_noise"] * rng.standard_normal(f.shape), 0, c["sat"])
        r = np.where(r < 1.0, 0.0, r)
        self._buf.append((d.time, com, pos, r))
        if len(self._buf) > 64:
            self._buf.pop(0)
        self._next = d.time + dt

    def _meas(self):
        """The reading for now: the samples DELAY ago, interpolated (CoM) or held (forces)."""
        t = self.data.time - self.sense["delay"]
        b = self._buf
        j = len(b) - 1
        while j > 0 and b[j][0] > t:
            j -= 1
        if j + 1 < len(b) and b[j + 1][0] > b[j][0]:
            a = min(1.0, max(0.0, (t - b[j][0]) / (b[j + 1][0] - b[j][0])))
            com = (1 - a) * b[j][1] + a * b[j + 1][1]
        else:
            com = b[j][1]
        return com, b[j][2], b[j][3]

    def step(self, record=False):
        r = super().step(record)
        if self.sense and self.data.time >= self._next - 1e-9:
            self._sample()
        return r

    def cop(self):
        if not self.sense:
            return super().cop()
        _, pos, r = self._meas()
        loads = [float(r[0].sum()), float(r[1].sum())]
        den = sum(loads)
        if den <= 1.0:
            return None, loads
        cop = (r[:, :, None] * pos).sum(axis=(0, 1))[:2] / den
        return cop, loads

    def _with_com(self, fn, *a):
        if not self.sense:
            return fn(*a)
        real = self.data
        self.data = _Proxy(real, self._meas()[0], self.pelvis)
        try:
            return fn(*a)
        finally:
            self.data = real

    def support(self):
        return self._with_com(super().support)

    def balance_loop(self):
        return self._with_com(super().balance_loop)


def one(case, scene, mean, gust, wdir, seed="0"):
    mean, gust = float(mean), float(gust)
    st.WIND = dict(mean=mean, gust=gust, dir=wdir, seed=int(seed)) if mean or gust else None
    rig = SensedRig(case, int(seed))
    rig.reset()
    fell = []

    def hook(r):
        if not fell and r.data.xmat[r.pelvis][8] < 0.94:
            fell.append((r.phase or "-").split(" ")[0] + "@%.0fs" % r.data.time)
    rig.hook = hook
    fns = {k: fn for k, t, n, fn in st.scenarios()}
    fns["stand"] = lambda r: r.run(20.0)
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        fns[scene](rig)
    gate = "g" if "gate timed out" in out.getvalue() else ""
    print("RESULT %s%s %s %.0f" % ("S" if rig.standing() else "F", gate, fell[0] if fell else "-",
                                   sum(s["e_j"] for s in rig.stats.values())))


def sweep(cases, scenes, winds, seeds):
    runs = [(c, s, *w, sd) for s in scenes for w in winds for c in cases for sd in seeds]
    live, res, todo = [], {}, list(runs)
    while todo or live:
        while todo and len(live) < int(os.environ.get("JOBS", 14)):
            r = todo.pop(0)
            live.append((r, subprocess.Popen([sys.executable, __file__, "one", *r],
                                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)))
        r, p = live.pop(0)
        out = p.communicate()[0]
        ln = [x for x in out.splitlines() if x.startswith("RESULT")]
        res[r] = " ".join(ln[0].split()[1:]) if ln else "E " + out.strip()[-80:].replace("\n", " ")
    print("%-9s %-16s" % ("scene", "wind") + "".join("%-24s" % c for c in cases))
    for s in scenes:
        for w in winds:
            for sd in seeds:
                print("%-9s %-16s" % (s, ",".join(w) + " s" + sd) + "".join("%-24s" % res[(c, s, *w, sd)] for c in cases))


if __name__ == "__main__":
    if sys.argv[1] == "one":
        one(*sys.argv[2:])
    else:
        a = dict(x.split("=", 1) for x in sys.argv[2:])
        winds = [tuple(w.split(":")) for w in a.get("winds", "0:0:head").split(",")]
        sweep(a.get("cases", ",".join(CASES)).split(","), a.get("scenes", "stand").split(","), winds,
              a.get("seeds", "0").split(","))
