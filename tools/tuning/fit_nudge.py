#!/usr/bin/env python3
"""Check (or estimate) the body's inertia and CoM height from balancing
captures, by simulating the robot + controller and matching the capture.

    python fit_nudge.py robot.json capture.csv [more.csv@1500 ...] [--free-com] [--free-kt]

The captures should be ~30 s of balancing with a few pushes (as for
check_log.py); captures with different gains together are better. The drivers' current
loop I is taken from robot.json; if a capture was recorded with another
value, add it to the file name as capture.csv@1500.

The model is started from the measured state every 0.1 s, run for 0.5 s with
the capture's own gains, and compared with what the robot did. The body
inertia and the IMU's tilt offset are always fitted; --free-com and --free-kt
fit the CoM height and the torque constant too.

Treat the result as a cross-check of the CAD, not a replacement: the CoM
height in particular is only weakly determined while balancing (the
controller hides gravity's effect), so the script also prints how much worse
the robot.json values fit than the best ones (a few percent is "the same"),
and, for each fitted value, whether the captures actually pin it down.
"""

import argparse
import math

import numpy as np
from scipy.optimize import least_squares

import sbr_model as sm

H = 250    # samples per stretch (0.5 s at 500 Hz)
STEP = 50  # a new stretch every 0.1 s


def closed_loop(r: sm.Robot, D, g: sm.Gains, starts, th0, bp=False):
    """Vectorised over stretches: simulate from the measured state."""
    ts = r.ts
    m, rr, l = r.body_mass, r.wheel_radius, r.com_height
    jt, a_, c0 = sm._mass_matrix(r, l, r.body_inertia)
    T = D["tilt"][starts] - th0
    Td = D["rate"][starts].copy()
    wr = D["w"][starts].copy()
    pa = wr + Td
    wf = wr.copy()
    pos = D["pos"][starts].copy()
    sp = D["sp"][starts]
    ke = r.ke
    x = D["i"][starts] / 2 * r.resistance + ke * wr  # current loop settled at the measured current
    q = [D["i"][starts] / 2] * max(0, r.delay_steps)
    out = np.zeros((3, len(starts), H))
    for k in range(H):
        tilt = T + th0
        out[0, :, k] = tilt
        out[1, :, k] = Td
        out[2, :, k] = wf
        lqr = g.total_current(r, pos, tilt - sp, wf, Td, balance_priority=bp)
        half = np.clip(lqr / 2, -r.max_current, r.max_current)
        q.append(half)
        u = q.pop(0)
        wr = pa - Td
        i = (r.cl_p * u + x - ke * wr) / (r.resistance + r.cl_p)
        x = x + r.cl_i * (u - i) * ts
        tq = 2 * (r.kt * i - r.friction * np.tanh(wr / 0.5))
        c = c0 * np.cos(T)
        r1 = tq + c0 * np.sin(T) * Td * Td
        r2 = m * sm.G * l * np.sin(T) - tq
        det = jt * a_ - c * c
        pdd = (a_ * r1 - c * r2) / det
        tdd = (jt * r2 - c * r1) / det
        pa = pa + pdd * ts
        Td = Td + tdd * ts
        T = T + Td * ts
        wf = wf + ((pa - Td) - wf) * ts / r.speed_filter
        pos = np.clip(pos + wf * ts, -g.max_pos_err, g.max_pos_err)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("robot")
    ap.add_argument("captures", nargs="+")
    ap.add_argument("--free-com", action="store_true", help="fit the CoM height too")
    ap.add_argument("--free-kt", action="store_true", help="fit the torque constant too")
    ap.add_argument("--cl-i", type=float, help="drivers' current loop I during the captures")
    a = ap.parse_args()
    r0 = sm.load_robot(a.robot)
    if a.cl_i is not None:
        r0 = r0.copy(cl_i=a.cl_i)

    data = []
    for arg in a.captures:
        path, _, cli = arg.partition("@")
        g = sm.gains_from_capture(path)
        bp = sm.parse_capture_header(path).get("balPriority", "0") == "1"  # firmware before this setting: off
        d = sm.load_capture(path)
        D = sm.resample(d, r0.ts)
        bal = np.interp(D["t"], d.t_ms.values / 1000.0, (d.state.values == "balancing").astype(float)) > 0.999
        starts = np.array([s for s in range(500, len(D["t"]) - H - 10, STEP) if bal[s:s + H].all()])
        if len(starts) < 10:
            print(f"{path}: not enough balancing to use")
            continue
        idx = starts[:, None] + np.arange(H)[None, :]
        meas = np.stack([D["tilt"][idx], D["rate"][idx], D["w"][idx]])
        data.append((path, D, g, starts, meas, float(cli) if cli else r0.cl_i, bp))
    if not data:
        raise SystemExit("no usable captures")
    scale = np.array([math.radians(0.3), math.radians(8), 2.0])[:, None, None]

    names = ["body inertia", "tilt offset"] + (["CoM height"] if a.free_com else []) + (["Kt"] if a.free_kt else [])

    def robot_for(p):
        kw = dict(body_inertia=p[0])
        j = 2
        if a.free_com:
            kw["com_height"] = p[j]
            j += 1
        if a.free_kt:
            kw["kt"] = p[j]
        return r0.copy(**kw), p[1]

    def residuals(p):
        r, th0 = robot_for(p)
        return np.concatenate([((closed_loop(r.copy(cl_i=ci), D, g, st, th0, bp) - meas) / scale).ravel()
                               for _, D, g, st, meas, ci, bp in data])

    x0 = [r0.body_inertia, 0.0] + ([r0.com_height] if a.free_com else []) + ([r0.kt] if a.free_kt else [])
    lb = [1e-5, -0.1] + ([0.005] if a.free_com else []) + ([0.005] if a.free_kt else [])
    ub = [0.05, 0.1] + ([0.3] if a.free_com else []) + ([0.5] if a.free_kt else [])
    fit = least_squares(residuals, x0, bounds=(lb, ub), loss="soft_l1", f_scale=3)
    rb, th0 = robot_for(fit.x)

    def cost(p):
        e = residuals(p)
        return float(np.sum(2 * (np.sqrt(1 + (e / 3) ** 2) - 1)))

    c_fit = cost(fit.x)
    c_json = cost(np.array([r0.body_inertia, th0] + ([r0.com_height] if a.free_com else [])
                           + ([r0.kt] if a.free_kt else [])))
    print(sm.describe(r0))
    print(f"Fitted from {len(data)} capture(s), {sum(len(x[3]) for x in data)} stretches:")
    print(f"  body inertia {rb.body_inertia * 1e9:.0f} g mm^2 (robot.json {r0.body_inertia * 1e9:.0f})")
    if a.free_com:
        print(f"  CoM height {rb.com_height * 1000:.1f} mm (robot.json {r0.com_height * 1000:.1f})")
    if a.free_kt:
        print(f"  Kt {rb.kt:.4f} N m/A (robot.json {r0.kt:.4f})")
    print(f"  IMU tilt offset {math.degrees(th0):+.2f} deg (true upright reads as this)")
    print(f"robot.json values fit {100 * (c_json / c_fit - 1):.0f}% worse than the fitted ones "
          f"(under ~10%: no real evidence against them).")
    # How well is each value determined? Cost with it 30% lower / 40% higher.
    for j, name in enumerate(names):
        if name == "tilt offset":
            continue
        rise = []
        for f in (0.7, 1.4):
            p = fit.x.copy()
            p[j] *= f
            rise.append(100 * (cost(p) / c_fit - 1))
        verdict = "well determined" if min(rise) > 10 else "poorly determined: the captures hardly constrain it"
        print(f"  {name}: fit {rise[0]:.0f}% / {rise[1]:.0f}% worse at -30% / +40% -> {verdict}")


if __name__ == "__main__":
    main()
