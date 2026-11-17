"""
Vertical control of a direct simple shear (DSS) device - a self-contained simulation.

During the shear of a DSS test the vertical boundary of the specimen is controlled in one of three ways:

  1. Constant height (constant volume, ASTM D6528 / D8296): the height is held; the vertical stress changes.
     A dead-band loop on the height reading pushes the motor back whenever the reading leaves the band,
     rests after each push, and caps every push so that no single push changes the load by more than ~1 kPa.
  2. Constant normal stress: the vertical stress is held; the height changes. "Governed steps": each step is
     a counted number of motor turns, sized from the error and a learned load-per-turn, never past the target.
  3. Constant normal stiffness (CNS): the target follows the height, T = max(2 kPa, s'vc + K * dh), and is
     held as in (2). K = infinity is constant height, K = 0 is constant normal stress.

The rig model is simple on purpose: a stiff loading frame in series with a specimen whose stiffness grows with
stress, a DC gear motor that never moves less than its gears' release when it breaks free, noisy load and height
readings, and a specimen that contracts and then dilates as it is sheared. All numbers are illustrative, but of a
realistic order.

Run:   python vertical_control_sim.py      (standard library only - nothing to install)
Out:   vertical_control_sim.svg and a printed summary of each control mode.

Author: Morteza Shafiei (Mortezashafiei). MIT License.
"""
import math
import random
import statistics
from collections import deque

# ----------------------------------------------------------------------------------------------- parameters
DT = 0.02             # s, the controller tick (a microcontroller loop at 50 Hz)
T_END = 3 * 3600.0    # s, a 3-hour shear
RATE = 5.0 / 3600.0   # %/s, shear strain rate (5 %/h, as suggested for a constant-volume monotonic shear)
SIG_VC = 400.0        # kPa, vertical stress at the end of the consolidation (s'vc)
C_SOIL = 2.5          # 1/mm, specimen stiffness K_s = C_SOIL * sigma_v (kPa/mm)
K_FRAME = 4000.0      # kPa/mm, loading frame + load cell stiffness
LEAD = 0.0015         # mm of piston travel per motor turn (after the gearboxes and the worm drive)
RELEASE = 0.12        # turns, the most extra travel when the gears break free (at SIG_VC; grows with load)
NOISE_SIG = 0.15      # kPa, noise of one load reading
NOISE_H = 0.00008     # mm, noise of one height reading (24-bit converter, ~0.1 um)
SENS = 1.0            # V/mm, LVDT + signal conditioner sensitivity (1 mV = 1 um)


def soil_contraction(gamma):
    """The specimen's own volume-change tendency (mm, + = it wants to compress) at shear strain gamma (%):
    contractive first, dilative at large strain - a typical shape for a lightly overconsolidated clay."""
    return 0.16 * (1.0 - math.exp(-gamma / 3.0)) - 0.004 * gamma


def slope(points):
    """Least-squares slope of [(x, y), ...]."""
    n = len(points)
    mx = sum(p[0] for p in points) / n
    my = sum(p[1] for p in points) / n
    sxx = sum((p[0] - mx) ** 2 for p in points)
    return sum((p[0] - mx) * (p[1] - my) for p in points) / sxx if sxx else 0.0


# ----------------------------------------------------------------------------------------------- the rig model
class Rig:
    """Piston -> frame spring -> top cap -> specimen spring. Positions in mm, + = down (compression)."""

    def __init__(self, seed=1):
        self.rng = random.Random(seed)
        self.sig = SIG_VC      # kPa, true vertical stress
        self.s = 0.0           # mm, specimen compression since the start of the shear (height change = -s)
        self.e = 0.0           # mm, the soil's own contraction tendency so far
        self.moves = []        # (time, turns, load change) of every move, for the summary

    def _apply(self, dp, de):
        ks = C_SOIL * max(self.sig, 1.0)
        ds = (K_FRAME * dp + ks * de) / (K_FRAME + ks)       # the top cap moves down by ds
        self.sig += K_FRAME * (dp - ds)                      # frame force = specimen force
        self.s += ds

    def shear(self, gamma):
        e_new = soil_contraction(gamma)
        self._apply(0.0, e_new - self.e)
        self.e = e_new

    def move(self, turns, t):
        """One move of the motor: the turns asked plus up to RELEASE turns when the gears break free."""
        if turns == 0:
            return 0.0
        sign = 1.0 if turns > 0 else -1.0
        done = turns + sign * RELEASE * (self.sig / SIG_VC) * self.rng.random()
        before = self.sig
        self._apply(done * LEAD, 0.0)
        self.moves.append((t, done, self.sig - before))
        return done

    def read_sigma(self):
        return self.sig + self.rng.gauss(0.0, NOISE_SIG)

    def read_height_v(self):
        """Height change as the LVDT reads it (V, + = taller)."""
        return (-self.s + self.rng.gauss(0.0, NOISE_H)) * SENS


# ----------------------------------------------------------------------------------------------- controllers
class ConstantHeight:
    """A dead-band loop on the height reading (what the microcontroller does in AUTO), with a rest after each push
    and each push capped so that it cannot change the load by more than about 1 kPa: TKMAX = 0.9 * 1 kPa / k_pt."""

    def __init__(self, band_v=0.0003, rest_s=4.5, kpa_per_push=1.0):
        self.band = band_v
        self.rest = rest_s
        self.kmax_kpa = kpa_per_push
        self.kpt = 0.5            # kPa per motor turn - a first guess, learned from every push
        self.sp = None
        self.next_ok = 0.0

    def tick(self, rig, t):
        h = rig.read_height_v()
        if self.sp is None:
            self.sp = h
        if t < self.next_ok:
            return
        err = h - self.sp                         # + = taller than the setpoint -> push down (load)
        if abs(err) <= self.band:
            return
        need = abs(err) / SENS / LEAD             # turns that would close the error
        turns = min(need, 0.9 * self.kmax_kpa / self.kpt) * (1 if err > 0 else -1)
        before = rig.sig
        done = rig.move(turns, t)
        if abs(done) > 0.05:                      # learn the load per turn from this push
            self.kpt = 0.7 * self.kpt + 0.3 * abs(rig.sig - before) / abs(done)
        self.next_ok = t + self.rest

    def target(self, rig):
        return None


class GovernedSteps:
    """Holds the vertical load at a target with counted steps of the motor (counted by a tachometer). A step when
    the error expected one second ahead is at least half a step; its size from the error and the learned load per
    turn, 0.12 - 0.60 turn; never past the target; a turn-around only after the load has been one band past the
    target for 30 s, or is two bands past it."""

    def __init__(self, band=1.0, q=0.5, settle_s=1.0, cns_k=None):
        self.band, self.q, self.settle = band, q, settle_s
        self.cns_k = cns_k                        # kPa/mm for CNS, None = constant normal stress
        self.kpt = 0.5                            # kPa per turn, first guess (learned)
        self.buf = deque(maxlen=5)                # 5 readings ~ 0.5 s at 10 readings a second
        self.hist = deque(maxlen=20)              # 2 s of medians, for the drift
        self.hbuf = deque(maxlen=20)              # 2 s of height readings, for the CNS target
        self.next_ok = 0.0
        self.last_dir = 0
        self.past_since = None
        self.h0 = None
        self.pending = None                       # (turns, load before) of the last step, to learn k_pt

    def target(self, rig):
        if self.cns_k is None or not self.hbuf:
            return SIG_VC
        dh = statistics.median(self.hbuf) / SENS  # mm, + = dilation
        return max(2.0, SIG_VC + self.cns_k * dh)

    def tick(self, rig, t):
        if int(round(t / DT)) % 5:                # the load is read 10 times a second
            return
        self.buf.append(rig.read_sigma())
        hv = rig.read_height_v()
        if self.h0 is None:
            self.h0 = hv
        self.hbuf.append(hv - self.h0)
        med = statistics.median(self.buf)
        self.hist.append((t, med))
        if t < self.next_ok or len(self.buf) < 5:
            return
        if self.pending is not None:              # the last step has acted: learn k_pt from it
            turns, before = self.pending
            dsig = abs(med - before)
            if dsig > 0.2 and abs(turns) > 0.05:
                self.kpt = 0.7 * self.kpt + 0.3 * dsig / abs(turns)
            self.pending = None
        T = self.target(rig)
        e = T - med                               # + = load too low -> load
        drift = slope(self.hist) if len(self.hist) >= 10 else 0.0   # kPa/s
        e1 = e - drift * 1.0                      # the error expected one second ahead
        past = (self.last_dir > 0 and e < -self.band) or (self.last_dir < 0 and e > self.band)
        self.past_since = (self.past_since if self.past_since is not None else t) if past else None
        if abs(e1) < self.q / 2:
            return
        want = 1 if e1 > 0 else -1
        if self.last_dir and want != self.last_dir:          # the turn-around rule
            long_past = self.past_since is not None and t - self.past_since >= 30.0
            if not (long_past or abs(e) >= 2 * self.band):
                return
        turns = min(max((abs(e1) + self.q / 2) / self.kpt, 0.12), 0.60)
        if abs(e) / self.kpt >= 0.12:                        # never sized to go past the target
            turns = min(turns, abs(e) / self.kpt)
        done = rig.move(want * turns, t)
        self.pending = (done, med)
        self.last_dir = want
        self.buf.clear()
        self.next_ok = t + self.settle


# ----------------------------------------------------------------------------------------------- one shear
def run(controller, seed=1):
    rig = Rig(seed)
    out = {"t": [], "sig": [], "target": [], "dh_um": []}
    for i in range(int(T_END / DT)):
        t = i * DT
        rig.shear(RATE * t)
        controller.tick(rig, t)
        if i % 50 == 0:                           # keep one point a second
            out["t"].append(t / 60.0)
            out["sig"].append(rig.sig)
            out["target"].append(controller.target(rig))
            out["dh_um"].append(-rig.s * 1000.0)
    return rig, out


def summary(name, rig, d):
    dsig = [abs(m[2]) for m in rig.moves] or [0.0]
    lines = [name, "  motor moves: %d, total turns %.1f, largest load change of one move %.2f kPa"
             % (len(rig.moves), sum(abs(m[1]) for m in rig.moves), max(dsig))]
    late = [i for i, t in enumerate(d["t"]) if t > 1.0]
    if d["target"][0] is None:
        lines.append("  height held within %.2f um (max |dh| after the first minute); sigma_v %.0f -> %.0f kPa, "
                     "so du = %.0f kPa" % (max(abs(d["dh_um"][i]) for i in late), d["sig"][0], d["sig"][-1],
                                           d["sig"][0] - d["sig"][-1]))
    else:
        err = [d["sig"][i] - d["target"][i] for i in late]
        lines.append("  sigma_v - target: rms %.2f kPa, max %.2f kPa; height change at the end %.0f um"
                     % (math.sqrt(sum(x * x for x in err) / len(err)), max(abs(x) for x in err), d["dh_um"][-1]))
    return "\n".join(lines)


# ----------------------------------------------------------------------------------------------- a small SVG plotter
def nice_ticks(lo, hi, n=5):
    if hi - lo < 1e-9:
        lo, hi = lo - 1, hi + 1
    raw = (hi - lo) / n
    mag = 10 ** math.floor(math.log10(raw))
    step = min((m * mag for m in (1, 2, 2.5, 5, 10) if m * mag >= raw), default=10 * mag)
    first = math.floor(lo / step) * step
    ticks, v = [], first
    while v <= hi + step * 0.5:
        ticks.append(round(v, 10))
        v += step
    return ticks


def panel(x0, y0, w, h, xs, series, title, ylabel, xlabel):
    """One chart as SVG elements. series: [(ys, color, dashed, label)]."""
    ys_all = [y for s in series for y in s[0] if y is not None]
    yt = nice_ticks(min(ys_all), max(ys_all))
    xt = nice_ticks(min(xs), max(xs))
    ylo, yhi, xlo, xhi = yt[0], yt[-1], xt[0], xt[-1]
    px = lambda v: x0 + (v - xlo) / (xhi - xlo) * w
    py = lambda v: y0 + h - (v - ylo) / (yhi - ylo) * h
    el = ['<text x="%.1f" y="%.1f" class="title">%s</text>' % (x0 + w / 2, y0 - 12, title)]
    for v in yt:
        el.append('<line x1="%.1f" x2="%.1f" y1="%.1f" y2="%.1f" class="grid"/>' % (x0, x0 + w, py(v), py(v)))
        el.append('<text x="%.1f" y="%.1f" class="tick" text-anchor="end">%g</text>' % (x0 - 6, py(v) + 4, v))
    for v in xt:
        el.append('<line x1="%.1f" x2="%.1f" y1="%.1f" y2="%.1f" class="grid"/>' % (px(v), px(v), y0, y0 + h))
        el.append('<text x="%.1f" y="%.1f" class="tick" text-anchor="middle">%g</text>' % (px(v), y0 + h + 16, v))
    el.append('<line x1="%.1f" x2="%.1f" y1="%.1f" y2="%.1f" class="axis"/>' % (x0, x0 + w, y0 + h, y0 + h))
    el.append('<line x1="%.1f" x2="%.1f" y1="%.1f" y2="%.1f" class="axis"/>' % (x0, x0, y0, y0 + h))
    el.append('<text x="%.1f" y="%.1f" class="label" text-anchor="middle">%s</text>' % (x0 + w / 2, y0 + h + 36, xlabel))
    el.append('<text transform="translate(%.1f,%.1f) rotate(-90)" class="label" text-anchor="middle">%s</text>'
              % (x0 - 46, y0 + h / 2, ylabel))
    for k, (ys, color, dashed, label) in enumerate(series):
        pts = " ".join("%.1f,%.1f" % (px(x), py(y)) for i, (x, y) in enumerate(zip(xs, ys))
                       if y is not None and i % 5 == 0)
        el.append('<polyline points="%s" fill="none" stroke="%s" stroke-width="1.4"%s/>'
                  % (pts, color, ' stroke-dasharray="5 4"' if dashed else ""))
        if label:
            ly = y0 + 14 + 16 * k
            el.append('<line x1="%.1f" x2="%.1f" y1="%.1f" y2="%.1f" stroke="%s" stroke-width="2"%s/>'
                      % (x0 + w - 120, x0 + w - 100, ly - 4, ly - 4, color, ' stroke-dasharray="5 4"' if dashed else ""))
            el.append('<text x="%.1f" y="%.1f" class="tick">%s</text>' % (x0 + w - 94, ly, label))
    return el


def main():
    cases = [("Constant height (constant volume)", ConstantHeight()),
             ("Constant normal stress (governed steps)", GovernedSteps()),
             ("CNS, K = 500 kPa/mm (governed steps)", GovernedSteps(cns_k=500.0))]
    W, H, w, h = 1320, 660, 330, 200
    el = []
    for j, (name, ctl) in enumerate(cases):
        rig, d = run(ctl, seed=1 + j)
        print(summary(name, rig, d))
        x0 = 90 + j * 430
        top = [(d["sig"], "#8b1a1a", False, "vertical stress")]
        if d["target"][0] is not None:
            top.insert(0, (d["target"], "#7b8794", True, "target"))
        el += panel(x0, 80, w, h, d["t"], top, name, "σv (kPa)", "")
        el += panel(x0, 380, w, h, d["t"], [(d["dh_um"], "#1f5e8c", False, None)], "",
                    "height change (µm, + = taller)", "time (min) · γ at 5 %/h")
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d" '
           'font-family="sans-serif">' % (W, H, W, H),
           '<style>.title{font-size:14px;fill:#1f2933;text-anchor:middle}.tick{font-size:11px;fill:#52606d}'
           '.label{font-size:12px;fill:#1f2933}.grid{stroke:#e4e7eb;stroke-width:1}.axis{stroke:#9aa5b1;stroke-width:1}'
           '</style>',
           '<rect width="100%" height="100%" fill="#ffffff"/>',
           '<text x="%d" y="28" font-size="15" fill="#1f2933" text-anchor="middle">Vertical control of a DSS specimen '
           'during a 3-hour shear (simulation, illustrative parameters)</text>' % (W // 2)] + el + ["</svg>"]
    with open("vertical_control_sim.svg", "w", encoding="utf-8") as f:
        f.write("\n".join(svg))
    print("wrote vertical_control_sim.svg")


if __name__ == "__main__":
    main()
