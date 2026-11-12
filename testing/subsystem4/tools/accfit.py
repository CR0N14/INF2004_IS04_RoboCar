#!/usr/bin/env python3
"""
accfit.py - Subsystem 4 sensor calibration from a Stage 2 log.

Reads the console log of s4_stage2_lsm303d.uf2 (S4_CFG_CONSOLE
S4_CONSOLE_RAW) and works out the values imu_params.h needs:

  - accelerometer zero-g offsets and per-axis scale (IMU_ACC_ZG_*,
    IMU_ACC_SCALE_*), fitted to the still windows only;
  - the axis mapping check: every labelled hold must tilt the way its
    label says (--expect), and the script lists every right-handed
    mapping that fits;
  - magnetometer hard-iron offsets (IMU_MAG_OFF_B*): a sphere fit to
    every orientation and a circle fit to the flat turns.  If the field
    was uniform (the vertical field is the same in every hold) the
    sphere is used; if not, the candidate that keeps the flat-turn
    heading steadiest.  The turns also give IMU_MAG_SCALE_B{X,Y} and
    the turn direction;
  - the largest magnetometer reading against the S4-M07 limit: any axis
    above 1,600 mG means the magnetometer must go to +-4 gauss.

It refuses (exit 2) unless the log was taken with the production
settings and every accelerometer axis was held near +1 g and -1 g.
Values are printed in sensor axes (as in the log) and in body axes.

    python3 accfit.py LOG
    python3 accfit.py LOG --expect flat,front-up,flat,back-up,...
    python3 accfit.py LOG --mag-skip 0-57

Labels for --expect (one per hold, in order; repeats of the same hold
are merged): flat, upside-down, front-up, back-up, left-up, right-up.
Add "90" (front-up90) for a hold standing on its end or side;
front-down = back-up, left-down = right-up, right-down = left-up.

Pure Python 3, no numpy.
"""

import argparse
import difflib
import itertools
import math
import re
import sys

# The production settings the Stage 2 image writes (imu_cfg.h defaults):
# 400 Hz, +-4 g, 50 Hz anti-alias, FIFO stream; magnetometer 100 Hz,
# +-2 gauss.  The 50 Hz filter shifts every axis by about +50 mg against
# 773 Hz, so data taken with any other set-up must not be mixed in.
PROD_CTRL = "40 8f c8 00 00 74 00 00"
PROD_FIFO = "40"

LINE_S = 0.5            # one readout line every 500 ms

# The USB console never waits for the host: until a host reads, it keeps
# the first 4095 bytes (lib/libtm/sysdepend/pico_rp2040/usb/usb_console.c,
# 4096-byte ring) plus TinyUSB's 256-byte CDC buffer and one 64-byte
# packet (tusb_config.h), and drops the rest.  A log that breaks at
# exactly this byte was started late.
USB_HELD_BYTES = 4095 + 256 + 64
ONE_G = 1000.0          # mg
MAG_FULL_SCALE = 2000.0 # mG at +-2 gauss (0.080 mG/LSB, datasheet Table 3)
# S4-M07 pass/fail: an axis above this means switch to +-4 gauss.  The
# 2026-11-05 diagnosis log reached 1,746 mG on one axis nose up (diag 32).
MAG_SWITCH_MG = 1600.0
# Lines record.sh adds; they are not Pico output
RECORDER_MARK = "--- record.sh:"

# Mapping in imu_params.h: body (x fwd, y left, z up) = (+X, -Y, -Z)
MAPPING = ((0, 1), (1, -1), (2, -1))

VALUE_RE = re.compile(
    r"s4 t=(?P<t>\d+\.\d) @(?P<addr>[0-9a-f]{2}) acc mg "
    r"X=(?P<ax>[+-]?\d+) Y=(?P<ay>[+-]?\d+) Z=(?P<az>[+-]?\d+) "
    r"\|a\|=\d+ \(raw (?P<rx>[+-]?\d+) (?P<ry>[+-]?\d+) (?P<rz>[+-]?\d+)\)"
    r"\s+mag mG X=(?P<mx>[+-]?\d+) Y=(?P<my>[+-]?\d+) Z=(?P<mz>[+-]?\d+) "
    r"\|B\|=\d+ \(raw (?P<qx>[+-]?\d+) (?P<qy>[+-]?\d+) (?P<qz>[+-]?\d+)\)"
    r"\s+fifo (?P<fmin>\d+)/(?P<favg>\d+)/(?P<fmax>\d+)"
    r"\s+fails (?P<fails>\d+)")
CFG_RE = re.compile(r"t=(?P<t>\d+\.\d) cfg \(er (?P<er>-?\d+)\): CTRL0-7 "
                    r"(?P<ctrl>(?:[0-9a-f]{2} ){7}[0-9a-f]{2})\s+"
                    r"FIFO_CTRL (?P<fifo>[0-9a-f]{2})")
ID_RE = re.compile(r"t=(?P<t>\d+\.\d) id: .*-> (?P<res>.*)$")
I2C_RE = re.compile(r"t=(?P<t>\d+\.\d) i2c: (?P<x>\d+) transfers to the IMU, "
                    r"(?P<f>\d+) failed")
BURST_OK_RE = re.compile(r"t=(?P<t>\d+\.\d) fifo burst check: \d+ samples in "
                         r"one read, level (?P<b>\d+) -> (?P<a>\d+): "
                         r"(?P<res>wraps, burst OK|does NOT wrap)")
BURST_SKIP_RE = re.compile(r"t=(?P<t>\d+\.\d) fifo burst check: not run "
                           r"\(er (?P<er>-?\d+), (?P<w>\d+) waiting\)")
ENTRY_RE = re.compile(r"s4 t=\d+\.\d")


class Line:
    def __init__(self, m):
        self.t = float(m["t"])
        self.addr = m["addr"]
        self.acc = [float(m["ax"]), float(m["ay"]), float(m["az"])]
        self.mag = [float(m["mx"]), float(m["my"]), float(m["mz"])]
        self.mag_raw = [int(m["qx"]), int(m["qy"]), int(m["qz"])]
        self.fifo = (int(m["fmin"]), int(m["favg"]), int(m["fmax"]))
        self.fails = int(m["fails"])


# ---- small linear algebra (no numpy) ---------------------------------

def solve(a, b):
    """Solve a x = b by Gaussian elimination with partial pivoting."""
    n = len(b)
    m = [list(row) + [b[i]] for i, row in enumerate(a)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-12:
            raise ValueError("singular system: the data do not constrain "
                             "every parameter")
        m[c], m[p] = m[p], m[c]
        for r in range(n):
            if r != c:
                f = m[r][c] / m[c][c]
                for k in range(c, n + 1):
                    m[r][k] -= f * m[c][k]
    return [m[i][n] / m[i][i] for i in range(n)]


def inverse(a):
    n = len(a)
    cols = [solve(a, [1.0 if r == c else 0.0 for r in range(n)])
            for c in range(n)]
    return [[cols[c][r] for c in range(n)] for r in range(n)]


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def dot(u, v):
    return sum(a * b for a, b in zip(u, v))


def cross(u, v):
    return [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0]]


def mean3(vs):
    return [sum(v[k] for v in vs) / len(vs) for k in range(3)]


def rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else 0.0


def gauss_newton(model, p0, free, iters=200):
    """Least squares: model(p) -> (residuals, jacobian rows).  Returns the
    parameters, the residuals and the 1-sigma error of each parameter."""
    p = list(p0)
    idx = [i for i in range(len(p)) if free[i]]
    for _ in range(iters):
        r, jac = model(p)
        a = [[sum(row[i] * row[j] for row in jac) for j in idx] for i in idx]
        g = [sum(row[i] * ri for row, ri in zip(jac, r)) for i in idx]
        step = solve(a, [-x for x in g])
        for k, i in enumerate(idx):
            p[i] += step[k]
        if max(abs(s) for s in step) < 1e-10:
            break
    r, jac = model(p)
    a = [[sum(row[i] * row[j] for row in jac) for j in idx] for i in idx]
    dof = len(r) - len(idx)
    s2 = (sum(x * x for x in r) / dof) if dof > 0 else float("nan")
    cov = inverse(a)
    sig = [0.0] * len(p)
    for k, i in enumerate(idx):
        sig[i] = math.sqrt(max(cov[k][k] * s2, 0.0))
    return p, r, sig


# ---- accelerometer: offsets and per-axis scale ------------------------

def acc_model(points):
    """|(a - o) / s| = 1 g; p = [ox, oy, oz, sx, sy, sz]."""
    def model(p):
        r = []
        jac = []
        for a in points:
            g = [(a[k] - p[k]) / p[3 + k] for k in range(3)]
            n = norm(g)
            r.append(n - ONE_G)
            jac.append([-g[k] / (n * p[3 + k]) for k in range(3)] +
                       [-g[k] * g[k] / (n * p[3 + k]) for k in range(3)])
        return r, jac
    return model


def acc_correct(p, a):
    return [(a[k] - p[k]) / p[3 + k] for k in range(3)]


def fit_acc(points, free=(True,) * 6, start=(0.0, 0.0, 0.0, 1.0, 1.0, 1.0)):
    return gauss_newton(acc_model(points), list(start), list(free))


# ---- magnetometer: sphere ----------------------------------------------

def fit_sphere(points):
    # Linear start: |m|^2 = 2 c.m + k
    rows = [[2 * m[0], 2 * m[1], 2 * m[2], 1.0] for m in points]
    rhs = [dot(m, m) for m in points]
    ata = [[sum(r[i] * r[j] for r in rows) for j in range(4)]
           for i in range(4)]
    atb = [sum(r[i] * y for r, y in zip(rows, rhs)) for i in range(4)]
    c0 = solve(ata, atb)
    r0 = math.sqrt(c0[3] + dot(c0[:3], c0[:3]))

    def model(p):
        r = []
        jac = []
        for m in points:
            d = [m[k] - p[k] for k in range(3)]
            n = norm(d)
            r.append(n - p[3])
            jac.append([-d[k] / n for k in range(3)] + [-1.0])
        return r, jac
    return gauss_newton(model, c0[:3] + [r0], [True] * 4)


# ---- orientation -------------------------------------------------------

def to_body(v, mapping):
    return [sign * v[axis] for axis, sign in mapping]


def mapping_name(mapping):
    return "(" + ", ".join(("+" if s > 0 else "-") + "XYZ"[a]
                           for a, s in mapping) + ")"


def right_handed_mappings():
    out = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            cols = [[0, 0, 0] for _ in range(3)]
            for b in range(3):
                cols[b][perm[b]] = signs[b]
            if dot(cross(cols[0], cols[1]), cols[2]) > 0:
                out.append(tuple(zip(perm, signs)))
    return out


def rest_rotation(up):
    """Smallest rotation taking the unit vector up onto +z (Rodrigues),
    as the firmware's at-rest calibration does."""
    z = [0.0, 0.0, 1.0]
    axis = cross(up, z)
    s = norm(axis)
    c = dot(up, z)
    if s < 1e-12:
        return [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
    k = [x / s for x in axis]
    kx = [[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]]
    kk = [[sum(kx[i][m] * kx[m][j] for m in range(3)) for j in range(3)]
          for i in range(3)]
    return [[(1.0 if i == j else 0.0) + s * kx[i][j] + (1 - c) * kk[i][j]
             for j in range(3)] for i in range(3)]


def rotate(rm, v):
    return [dot(row, v) for row in rm]


def classify(u):
    """u: unit 'up' vector in the rest-aligned body frame."""
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, u[2]))))
    if tilt < 5.0:
        return "flat", tilt
    if tilt > 135.0:
        return "upside-down", tilt
    if abs(u[0]) >= abs(u[1]):
        name = "front-up" if u[0] > 0 else "back-up"
    else:
        name = "left-up" if u[1] > 0 else "right-up"
    return name + ("90" if tilt >= 60.0 else ""), tilt


SYNONYMS = {"front-down": "back-up", "left-down": "right-up",
            "right-down": "left-up", "back-down": "front-up"}


def parse_expect(text):
    out = []
    for tok in text.split(","):
        tok = tok.strip().lower()
        suffix = "90" if tok.endswith("90") else ""
        base = tok[:-2] if suffix else tok
        base = SYNONYMS.get(base, base)
        if base not in ("flat", "upside-down", "front-up", "back-up",
                        "left-up", "right-up"):
            raise SystemExit("unknown label in --expect: " + tok)
        if base in ("flat", "upside-down"):
            suffix = ""
        out.append(base + suffix)
    return out


def holds(windows, classes):
    """Merge consecutive windows with the same class."""
    out = []
    for w, c in zip(windows, classes):
        if out and out[-1][0] == c:
            out[-1][1].append(w)
        else:
            out.append((c, [w]))
    return out


def label_holds(windows, corr_means, mapping, rest_index):
    up_rest = to_body(corr_means[rest_index], mapping)
    up_rest = [x / norm(up_rest) for x in up_rest]
    if up_rest[2] < math.cos(math.radians(30.0)):
        return None             # the car at rest is not 'up' here
    rm = rest_rotation(up_rest)
    classes = []
    tilts = []
    for m in corr_means:
        b = to_body(m, mapping)
        u = rotate(rm, [x / norm(b) for x in b])
        c, tilt = classify(u)
        classes.append(c)
        tilts.append((tilt, u))
    return classes, tilts, rm


# ---- log parsing -------------------------------------------------------

def read_log(path):
    lines = []
    cfgs = []
    ids = []
    i2c = []
    bursts = []
    skips = []
    joins = []
    offset = 0                  # bytes so far, line ends as captured
    with open(path, errors="replace", newline="") as f:
        for raw in f:
            starts = [m.start() for m in ENTRY_RE.finditer(raw)]
            if len(starts) > 1:
                joins.append((raw[starts[0]:starts[0] + 12].strip(),
                              raw[starts[-1]:starts[-1] + 12].strip(),
                              offset + starts[-1]))
            if not raw.startswith(RECORDER_MARK):
                offset += len(raw)
            # Parse each "s4 t=" entry on its own: a lost stretch of text
            # can join two entries on one physical line.
            bounds = starts + [len(raw)]
            pieces = [raw[bounds[i]:bounds[i + 1]]
                      for i in range(len(starts))] or [raw]
            for piece in pieces:
                m = VALUE_RE.search(piece)
                if m:
                    lines.append(Line(m))
                    continue
                for rx, dst in ((CFG_RE, cfgs), (ID_RE, ids), (I2C_RE, i2c),
                                (BURST_OK_RE, bursts),
                                (BURST_SKIP_RE, skips)):
                    m = rx.search(piece)
                    if m:
                        dst.append(m)
                        break
    return lines, cfgs, ids, i2c, bursts, skips, joins


def still_windows(lines, step_mg, step_mag, min_lines):
    """Runs of lines whose accelerometer mean differs from both neighbours
    (0.5 s apart, no gap) by at most step_mg on every axis, and whose
    magnetometer mean by at most step_mag (a flat turn does not move the
    accelerometer)."""
    n = len(lines)
    still = [False] * n
    for i in range(1, n - 1):
        ok = True
        for j in (i - 1, i + 1):
            if abs(abs(lines[j].t - lines[i].t) - LINE_S) > 0.01:
                ok = False
            elif max(abs(lines[j].acc[k] - lines[i].acc[k])
                     for k in range(3)) > step_mg:
                ok = False
            elif max(abs(lines[j].mag[k] - lines[i].mag[k])
                     for k in range(3)) > step_mag:
                ok = False
        still[i] = ok
    wins = []
    i = 0
    while i < n:
        if still[i]:
            j = i
            while (j + 1 < n and still[j + 1] and
                   abs(lines[j + 1].t - lines[j].t - LINE_S) < 0.01):
                j += 1
            if j - i + 1 >= min_lines:
                wins.append(lines[i:j + 1])
            i = j + 1
        else:
            i += 1
    return wins


def cluster(dirs, max_deg):
    """Greedy grouping of unit vectors closer than max_deg."""
    groups = []
    cos_max = math.cos(math.radians(max_deg))
    for i, d in enumerate(dirs):
        for g in groups:
            if dot(d, g["dir"]) >= cos_max:
                g["members"].append(i)
                break
        else:
            groups.append({"dir": d, "members": [i]})
    return [g["members"] for g in groups]


def parse_ranges(specs):
    out = []
    for spec in specs or []:
        a, b = spec.split("-")
        out.append((float(a), float(b)))
    return out


def in_ranges(t, ranges):
    return any(a <= t <= b for a, b in ranges)


# ---- magnetometer report ---------------------------------------------

def to_sensor(b, mapping):
    s = [0.0, 0.0, 0.0]
    for k, (axis, sign) in enumerate(mapping):
        s[axis] = sign * b[k]
    return s


def level_of(v, rm):
    """Sensor-axis vector -> body axes -> the rest-aligned (level) frame."""
    return rotate(rm, to_body(v, MAPPING))


def fit_circle(pts):
    rows = [[2 * x, 2 * y, 1.0] for x, y in pts]
    rhs = [x * x + y * y for x, y in pts]
    ata = [[sum(r[i] * r[j] for r in rows) for j in range(3)]
           for i in range(3)]
    atb = [sum(r[i] * b for r, b in zip(rows, rhs)) for i in range(3)]
    cx, cy, k = solve(ata, atb)
    rad = math.sqrt(k + cx * cx + cy * cy)
    return cx, cy, rad, [math.hypot(x - cx, y - cy) - rad for x, y in pts]


def fit_ellipse(pts, cx, cy, rad):
    """Axis-aligned ellipse in the level plane: centre and both semi-axes."""
    def model(p):
        r = []
        jac = []
        e_mean = (p[2] + p[3]) / 2.0
        for x, y in pts:
            u = [(x - p[0]) / p[2], (y - p[1]) / p[3]]
            n = norm(u)
            r.append((n - 1.0) * e_mean)
            jac.append([-u[0] / (n * p[2]) * e_mean,
                        -u[1] / (n * p[3]) * e_mean,
                        -u[0] * u[0] / (n * p[2]) * e_mean,
                        -u[1] * u[1] / (n * p[3]) * e_mean])
        return r, jac
    return gauss_newton(model, [cx, cy, rad, rad], [True] * 4)


def horiz_spread(pts, c):
    """Spread of the horizontal field |m - c| over flat-turn points: what
    a centre error does to heading."""
    r = [math.hypot(m[0] - c[0], m[1] - c[1]) for m in pts]
    mean = sum(r) / len(r)
    dev = [x - mean for x in r]
    return mean, rms(dev), max(abs(x) for x in dev)


def mag_report(lines, wins, classes, corr_means, p, rm, skip):
    """Hard-iron offsets, field consistency, turns and range.  Returns the
    recommended offsets in body axes."""
    win_of = {id(ln): wi for wi, w in enumerate(wins) for ln in w}

    # Sphere points: one mean per still window, plus moving lines that
    # turned slowly enough for their 500 ms mean to stay on the sphere
    # (a 100 mG step is about 25 deg on a 250 mG circle: under 1 % low)
    pts = []
    n_still = 0
    for w in wins:
        if not in_ranges(w[0].t, skip):
            pts.append(mean3([ln.mag for ln in w]))
            n_still += 1
    for i in range(1, len(lines) - 1):
        ln = lines[i]
        if id(ln) in win_of or in_ranges(ln.t, skip):
            continue
        if all(abs(lines[j].t - ln.t) < LINE_S + 0.01 and
               norm([lines[j].mag[k] - ln.mag[k] for k in range(3)]) <= 100.0
               for j in (i - 1, i + 1)):
            pts.append(ln.mag)
    (sx, sy, sz, rad), mres, msig = fit_sphere(pts)
    sph = [sx, sy, sz]
    print("== Magnetometer, motors off")
    print("Sphere fit (hard iron, every orientation): %d points (%d still-"
          "window means, %d slow moving lines)%s" %
          (len(pts), n_still, len(pts) - n_still,
           ", skipping t = " + ", ".join("%g-%g" % r for r in skip)
           if skip else ""))
    print("  centre, sensor axes X %+.1f Y %+.1f Z %+.1f mG (+- %.1f %.1f "
          "%.1f); body axes %s" % (sx, sy, sz, msig[0], msig[1], msig[2],
                                   fmt3(to_body(sph, MAPPING), 6, 1)))
    print("  field |B| %.0f mG; residual RMS %.1f mG, largest %.1f mG "
          "(a uniform field and a sound sensor give under about 10)" %
          (rad, rms(mres), max(abs(x) for x in mres)))

    # In a uniform field the vertical component is the same whatever the
    # orientation
    print()
    print("Field in each still window, after the sphere centre (vertical = "
          "along the accelerometer's 'up'):")
    vert = []
    for wi, w in enumerate(wins):
        if in_ranges(w[0].t, skip):
            continue
        d = [m - c for m, c in zip(mean3([ln.mag for ln in w]), sph)]
        up = [x / norm(corr_means[wi]) for x in corr_means[wi]]
        bv = dot(d, up)
        vert.append(bv)
        print("  %6.1f-%-6.1f %-12s |B| %4.0f  vertical %+5.0f  horizontal "
              "%4.0f mG" % (w[0].t, w[-1].t, classes[wi], norm(d), bv,
                            math.sqrt(max(dot(d, d) - bv * bv, 0.0))))
    v_spread = max(vert) - min(vert) if vert else 0.0
    print("  vertical field spread %.0f mG (%+.0f to %+.0f): %s" %
          (v_spread, min(vert), max(vert),
           "uniform" if v_spread <= 30.0 else
           "NOT uniform: the field at the sensor changed between holds"))

    # Heading of every flat line (firmware formula: level frame,
    # atan2(y, x); anticlockwise turns make it fall)
    flat_pts = []
    for i, ln in enumerate(lines):
        if in_ranges(ln.t, skip):
            continue
        g = rotate(rm, to_body(acc_correct(p, ln.acc), MAPPING))
        if math.degrees(math.acos(max(-1.0, min(1.0, g[2] / norm(g))))) > 10:
            continue
        m = level_of(ln.mag, rm)
        c = level_of(sph, rm)
        flat_pts.append((i, ln, m, math.degrees(math.atan2(m[1] - c[1],
                                                           m[0] - c[0]))))
    runs = []
    for fp in flat_pts:
        if runs and fp[0] == runs[-1][-1][0] + 1:
            runs[-1].append(fp)
        else:
            runs.append([fp])

    print()
    print("Flat stretches (heading from the sphere centre):")
    turn_pts = []
    chunks = []
    for run in runs:
        total = 0.0
        acc_h = 0.0
        chunk = []
        for a, b in zip(run, run[1:]):
            dh = (b[3] - a[3] + 180.0) % 360.0 - 180.0
            if b[1].t - a[1].t > LINE_S + 0.01:
                print("  across the gap %.1f-%.1f s: heading %+.0f deg, "
                      "vertical (level z) %+.0f -> %+.0f mG" %
                      (a[1].t, b[1].t, dh, a[2][2], b[2][2]))
            total += dh
            acc_h += dh
            if id(b[1]) not in win_of:
                chunk.append(b[2])
            if abs(acc_h) >= 360.0:
                chunks.append(chunk)
                chunk = []
                acc_h = 0.0
        if abs(total) >= 270.0:
            zs = [fp[2][2] for fp in run]
            print("  %.1f-%.1f s: heading %+.0f deg (%s), vertical (level z) "
                  "%+.0f to %+.0f mG" %
                  (run[0][1].t, run[-1][1].t, total,
                   "anticlockwise seen from above" if total < 0 else
                   "clockwise seen from above", min(zs), max(zs)))
            for fp in run:
                if id(fp[1]) not in win_of:
                    turn_pts.append(fp[2])
            for wi in sorted({win_of[id(fp[1])] for fp in run
                              if id(fp[1]) in win_of}):
                turn_pts.append(level_of(mean3([ln.mag for ln in wins[wi]]),
                                         rm))
            if len(chunk) >= 5 and abs(acc_h) >= 270.0:
                chunks.append(chunk)

    off_sph = to_body(sph, MAPPING)
    if len(turn_pts) < 8:
        print("  no full turn found: using the sphere centre")
        return off_sph

    # The flat turns are the case that matters for heading on the course:
    # fit their circle in the level plane
    print()
    print("Flat turns, level frame (the horizontal field is what heading "
          "uses):")
    for n, ch in enumerate(chunks):
        if len(ch) >= 5:
            cx, cy, r, res = fit_circle([(m[0], m[1]) for m in ch])
            print("  turn %d: circle centre x %+6.1f y %+6.1f, horizontal "
                  "field %.0f mG, residual RMS %.1f mG (%d lines)" %
                  (n + 1, cx, cy, r, rms(res), len(ch)))
    cx, cy, r_h, res = fit_circle([(m[0], m[1]) for m in turn_pts])
    print("  all turns: circle centre x %+6.1f y %+6.1f, horizontal field "
          "%.0f mG, residual RMS %.1f mG, largest %.1f" %
          (cx, cy, r_h, rms(res), max(abs(x) for x in res)))
    ep, eres, _ = fit_ellipse([(m[0], m[1]) for m in turn_pts], cx, cy, r_h)
    ex, ey = ep[2], ep[3]
    # An ellipse of axis ratio q left uncorrected bends heading by up to
    # about (1 - q) / 2 rad
    cost = math.degrees(abs(1.0 - ey / ex) / 2.0)
    worth = cost > 3.0 and rms(eres) < 0.7 * rms(res)
    print("  axis-aligned ellipse: semi-axes x %.0f, y %.0f mG (ratio %.3f), "
          "residual RMS %.1f mG; left uncorrected it costs up to %.1f deg of "
          "heading -> IMU_MAG_SCALE_BX/BY_PERMIL %s" %
          (ex, ey, ey / ex, rms(eres), cost,
           "%.0f / %.0f" % ((ex + ey) / 2 / ex * 1000,
                            (ex + ey) / 2 / ey * 1000) if worth else
           "keep 1000 (not worth correcting, or the ellipse fits no better "
           "than the circle)"))

    # Two candidates, scored on the flat turns
    c_sph = level_of(sph, rm)
    c_turn = [cx, cy, c_sph[2]]
    rmt = [[rm[j][i] for j in range(3)] for i in range(3)]
    off_turn = rotate(rmt, c_turn)
    print()
    print("Candidate offsets, scored on the flat turns (spread of the "
          "horizontal field; heading error up to asin(spread / field)):")
    best = None
    for name, c_lvl, off in (("sphere centre", c_sph, off_sph),
                             ("turn circle + sphere vertical", c_turn,
                              off_turn)):
        h, s_rms, s_max = horiz_spread(turn_pts, c_lvl)
        print("  %-30s body %s mG: field %.0f, spread RMS %.1f, largest "
              "%.1f mG (up to %.0f deg)" %
              (name, fmt3(off, 6, 1), h, s_rms, s_max,
               math.degrees(math.asin(min(1.0, s_max / h)))))
        if best is None or s_rms < best[0]:
            best = (s_rms, name, off)
    uniform = v_spread <= 30.0 and rms(mres) <= 15.0
    choice = ("sphere centre", off_sph) if uniform else (best[1], best[2])
    print("-> %s: %s" % (choice[0],
                         "the field was uniform, so the full sphere fit is "
                         "best" if uniform else
                         "the field was not uniform, so use the offsets that "
                         "keep the flat-turn heading steadiest"))
    print("   body %s mG = sensor %s mG" %
          (fmt3(choice[1], 6, 1), fmt3(to_sensor(choice[1], MAPPING), 6, 1)))

    # Range
    peak_ln = max(lines, key=lambda ln: max(abs(x) for x in ln.mag))
    peak = max(abs(x) for x in peak_ln.mag)
    peak_raw = max(abs(x) for ln in lines for x in ln.mag_raw)
    bound = [abs(c) + rad for c in to_sensor(choice[1], MAPPING)]
    print()
    print("Range: largest single-axis reading (500 ms mean) %.0f mG (%d "
          "counts) at t = %.1f s; worst case any orientation (|offset| + "
          "|B| per axis) %s mG; full scale +-%.0f mG" %
          (peak, peak_raw, peak_ln.t, fmt3(bound, 4, 0), MAG_FULL_SCALE))
    print("-> S4-M07 range check (any axis above %.0f mG: switch to +-4 "
          "gauss): %s" % (MAG_SWITCH_MG,
                          "PASS, +-2 gauss stays" if peak <= MAG_SWITCH_MG
                          else "FAIL, switch the magnetometer to +-4 gauss"))
    print("   (the worst case above is an estimate; the check uses the "
          "largest reading)")
    return choice[1]


# ---- report ------------------------------------------------------------

def fmt3(v, width=6, prec=0):
    return " ".join("%+*.*f" % (width, prec, x) for x in v)


def log_health(lines, cfgs, ids, i2c, bursts, skips, joins):
    print("== Log")
    print("%d readout lines, t = %.1f to %.1f s" %
          (len(lines), lines[0].t, lines[-1].t))
    addrs = sorted({ln.addr for ln in lines})
    print("address: %s in every readout line" % ", ".join("0x" + a
                                                          for a in addrs)
          if len(addrs) == 1 else "ADDRESS CHANGED: %s" % addrs)
    bad_ids = [m for m in ids if "LSM303D OK" not in m["res"]]
    print("chip identified %d times, %d failures" % (len(ids), len(bad_ids)))
    print("failed reads at the end: %d" % lines[-1].fails)
    for ln_a, ln_b in zip(lines, lines[1:]):
        if ln_b.t - ln_a.t > LINE_S + 0.01:
            print("GAP: no readout from t = %.1f to %.1f s (%d lines "
                  "missing)" % (ln_a.t, ln_b.t,
                                round((ln_b.t - ln_a.t) / LINE_S) - 1))
            before = [m for m in i2c if float(m["t"]) <= ln_a.t]
            after = [m for m in i2c if float(m["t"]) >= ln_b.t]
            if before and after:
                b = before[-1]
                a = after[0]
                rate_gap = ((int(a["x"]) - int(b["x"])) /
                            (float(a["t"]) - float(b["t"])))
                rates = [(int(y["x"]) - int(x["x"])) /
                         (float(y["t"]) - float(x["t"]))
                         for x, y in zip(i2c, i2c[1:])
                         if float(x["t"]) >= ln_b.t]
                rate_rest = sorted(rates)[len(rates) // 2] if rates else 0
                print("     I2C transfers across the gap: %.0f/s (%s to %s "
                      "s), elsewhere %.0f/s: %s" %
                      (rate_gap, b["t"], a["t"], rate_rest,
                       "the firmware kept running, the text was lost "
                       "between the Pico and the file"
                       if abs(rate_gap - rate_rest) < 0.05 * rate_rest else
                       "the firmware did not run normally"))
    for first, second, at in joins:
        print("JOINED: text lost inside one line: %s ... %s, after byte %d"
              % (first, second, at))
        if at == USB_HELD_BYTES:
            print("     = what the Pico's USB console holds while no host "
                  "reads (4095-byte ring + 256 + 64 in TinyUSB): the capture "
                  "started late, and the Pico dropped what it printed until "
                  "then")
    bad_cfg = [m for m in cfgs if m["er"] != "0" or m["ctrl"] != PROD_CTRL
               or m["fifo"] != PROD_FIFO]
    print("settings read back %d times: %s" %
          (len(cfgs), "all production (CTRL0-7 %s, FIFO_CTRL %s)" %
           (PROD_CTRL, PROD_FIFO) if not bad_cfg else
           "%d NOT production" % len(bad_cfg)))
    ok = sum(1 for m in bursts if m["res"].startswith("wraps"))
    print("FIFO burst check: %d ran, %d wrap OK, %d do NOT wrap, %d not run"
          % (len(bursts), ok, len(bursts) - ok, len(skips)))
    if skips:
        why = sorted({"er %s, %s waiting" % (m["er"], m["w"]) for m in skips})
        print("     not run: %s (fewer samples waiting than the check reads;"
              " no read was attempted)" % "; ".join(why))
    fifo = {}
    for ln in lines:
        fifo[ln.fifo] = fifo.get(ln.fifo, 0) + 1
    common = max(fifo, key=fifo.get)
    print("most common FIFO min/avg/max per 10 ms: %d/%d/%d (%d of %d lines)"
          % (common + (fifo[common], len(lines))))
    return bool(bad_cfg)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--expect", help="hold labels in order (see above)")
    ap.add_argument("--still-mg", type=float, default=10.0,
                    help="largest accelerometer change to a neighbouring "
                    "line, mg (default 10)")
    ap.add_argument("--still-mag", type=float, default=15.0,
                    help="largest magnetometer change to a neighbouring "
                    "line, mG (default 15)")
    ap.add_argument("--min-lines", type=int, default=4,
                    help="shortest still window, lines (default 4 = 2 s)")
    ap.add_argument("--cluster-deg", type=float, default=12.0,
                    help="windows closer than this are one orientation")
    ap.add_argument("--mag-skip", action="append", metavar="T0-T1",
                    help="leave these seconds out of the magnetometer fit")
    ap.add_argument("--force", action="store_true",
                    help="fit even if the checks fail")
    args = ap.parse_args()

    lines, cfgs, ids, i2c, bursts, skips, joins = read_log(args.log)
    if len(lines) < 10:
        sys.exit("only %d readout lines in %s" % (len(lines), args.log))
    refuse = []
    if log_health(lines, cfgs, ids, i2c, bursts, skips, joins):
        refuse.append("the log was not taken with the production settings")

    wins = still_windows(lines, args.still_mg, args.still_mag, args.min_lines)
    means = [mean3([ln.acc for ln in w]) for w in wins]
    print()
    print("== Still windows: %d (each line within %.0f mg and %.0f mG of its "
          "neighbours, %d+ lines)" % (len(wins), args.still_mg,
                                      args.still_mag, args.min_lines))

    for k, axis in enumerate("XYZ"):
        hi = max(m[k] for m in means)
        lo = min(m[k] for m in means)
        if hi < 700 or lo > -700:
            refuse.append("%s not held near both +1 g and -1 g (still "
                          "windows reach %+.0f / %+.0f mg)" % (axis, hi, lo))
    for r in refuse:
        print(("FORCED past: " if args.force else "REFUSED: ") + r)
    if refuse and not args.force:
        sys.exit(2)

    # One point per distinct orientation, so the long flat holds do not
    # outweigh the six positions
    groups = cluster([[x / norm(m) for x in m] for m in means],
                     args.cluster_deg)
    points = []
    for g in groups:
        pts = [ln.acc for i in g for ln in wins[i]]
        points.append(mean3(pts))

    # Linear start: axis-aligned ellipsoid A x^2 + B y^2 + C z^2 + D x +
    # E y + F z = 1
    rows = [[a[0] ** 2, a[1] ** 2, a[2] ** 2, a[0], a[1], a[2]]
            for a in points]
    ata = [[sum(r[i] * r[j] for r in rows) for j in range(6)]
           for i in range(6)]
    atb = [sum(r[i] for r in rows) for i in range(6)]
    q = solve(ata, atb)
    off0 = [-q[3 + k] / (2 * q[k]) for k in range(3)]
    gg = 1 + sum(q[3 + k] ** 2 / (4 * q[k]) for k in range(3))
    start = off0 + [math.sqrt(gg / q[k]) / ONE_G for k in range(3)]

    p, res, sig = fit_acc(points, start=start)
    p_off, res_off, _ = fit_acc(points, free=(True,) * 3 + (False,) * 3)
    p_z7, res_z7, _ = fit_acc(points, free=(True,) * 3 + (False,) * 3,
                              start=(0.0, 0.0, 0.0, 1.0, 1.0, 0.93))

    corr_means = [acc_correct(p, m) for m in means]
    rest_index = 0
    lab = label_holds(wins, corr_means, MAPPING, rest_index)
    if lab is None:
        sys.exit("the first still window is not 'up' with mapping %s" %
                 mapping_name(MAPPING))
    classes, tilts, rm = lab
    window_group = {}
    for gi, g in enumerate(groups):
        for i in g:
            window_group[i] = gi

    print("  #   t from-to      n  orient  acc mg X Y Z (as logged)  "
          "|a| log  |a| fit  class        tilt")
    for i, w in enumerate(wins):
        a_fit = norm(corr_means[i])
        print("%3d %6.1f-%-6.1f %3d   %3d   %s   %6.0f   %6.0f   %-12s %5.1f"
              % (i, w[0].t, w[-1].t, len(w), window_group[i],
                 fmt3(means[i]), norm(means[i]), a_fit, classes[i],
                 tilts[i][0]))

    print()
    print("== Accelerometer fit: %d orientations (one point each), "
          "production settings" % len(points))
    names = ("X", "Y", "Z")
    print("sensor axes   zero-g mg            scale per mille")
    for k in range(3):
        print("  %s           %+7.1f +- %4.1f       %7.1f +- %4.1f" %
              (names[k], p[k], sig[k], p[3 + k] * 1000, sig[3 + k] * 1000))
    print("residual |a| - 1 g over the orientations: RMS %.1f mg, largest "
          "%.1f mg" % (rms(res), max(abs(x) for x in res)))
    line_res = [norm(acc_correct(p, ln.acc)) - ONE_G for w in wins
                for ln in w]
    print("residual over all %d still lines: RMS %.1f mg, largest %+.1f mg"
          % (len(line_res), rms(line_res),
             max(line_res, key=abs)))
    body_off = to_body(p[:3], MAPPING)
    body_scale = [p[3 + a] * 1000 for a, _ in MAPPING]
    print("body axes %s: zero-g %s mg, scale %s per mille" %
          (mapping_name(MAPPING), fmt3(body_off, 6, 1),
           " ".join("%.1f" % s for s in body_scale)))
    print()
    print("Same points, other models:")
    print("  offsets only, scale 1000:        offsets %s mg, residual RMS "
          "%.1f mg, largest %.1f" % (fmt3(p_off[:3], 6, 1), rms(res_off),
                                     max(abs(x) for x in res_off)))
    print("  Z 7 %% low (930), others 1000:     offsets %s mg, residual RMS "
          "%.1f mg, largest %.1f" % (fmt3(p_z7[:3], 6, 1), rms(res_z7),
                                     max(abs(x) for x in res_z7)))
    print("  full fit (offsets and scale):    residual RMS %.1f mg" %
          rms(res))

    # |a| at rest after correction
    print()
    flat = [i for i, c in enumerate(classes) if c == "flat"]
    print("|a| at rest (flat windows) after correction: %s mg" %
          ", ".join("%.0f" % norm(corr_means[i]) for i in flat))
    up = to_body(corr_means[rest_index], MAPPING)
    pitch = math.degrees(math.atan2(up[0], math.hypot(up[1], up[2])))
    roll = math.degrees(math.atan2(up[1], up[2]))
    print("board at rest (first flat window), after correction: pitch "
          "%+.1f deg (%s), roll %+.1f deg (%s); the self-test's at-rest "
          "rotation takes this out" %
          (pitch, "nose down" if pitch < 0 else "nose up", roll,
           "left side up" if roll > 0 else "right side up"))

    # Mapping and labels
    print()
    print("== Axis mapping %s and labels" % mapping_name(MAPPING))
    hl = holds(wins, classes)
    got = [c for c, _ in hl]
    print("holds found: " + ", ".join(
        "%s (%.0f-%.0f s)" % (c, ws[0][0].t, ws[-1][-1].t) for c, ws in hl))
    if args.expect:
        want = parse_expect(args.expect)
        sm = difflib.SequenceMatcher(a=want, b=got, autojunk=False)
        all_ok = (want == got)
        for op, i1, i2, j1, j2 in sm.get_opcodes():
            if op != "equal":
                print("  MISMATCH: expected %s, found %s" %
                      (want[i1:i2] or "-", got[j1:j2] or "-"))
        print("every hold matches its label: %s" % ("YES" if all_ok else "NO"))
        fits = []
        for mp in right_handed_mappings():
            lab_m = label_holds(wins, corr_means, mp, rest_index)
            if lab_m is None:
                continue
            if [c for c, _ in holds(wins, lab_m[0])] == want:
                fits.append(mapping_name(mp))
        print("right-handed mappings (of 24) that fit every label: %s" %
              (", ".join(fits) if fits else "none"))

    # ---- magnetometer ----
    print()
    skip = parse_ranges(args.mag_skip)
    off_body = mag_report(lines, wins, classes, corr_means, p, rm, skip)

    print()
    print("== imu_params.h")
    for k, n in enumerate("XYZ"):
        print("#define IMU_ACC_ZG_%s_MG         %d" % (n, round(p[k])))
    for k, n in enumerate("XYZ"):
        print("#define IMU_ACC_SCALE_%s_PERMIL  %d" % (n, round(p[3 + k] *
                                                               1000)))
    for k, n in enumerate("XYZ"):
        print("#define IMU_MAG_OFF_B%s_MG       %d" % (n, round(off_body[k])))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except ValueError as e:         # a fit the data cannot support
        sys.exit("accfit.py: %s" % e)
