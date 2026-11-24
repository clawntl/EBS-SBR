"""Self-righting simulator for EBS-SBR.

Lying down, the robot rests on its hoops (the egg-shaped outer rings) with the
wheels in the air. The firmware (`Righting` state, firmware/src/LQRControl.cpp)
gets it up by rocking it on the hoop: the wheels are spun up as reaction
wheels, with the reaction torque on the body in the direction it is swinging
(bang-bang, with a little hysteresis), so every half swing adds energy. Once
it has rocked to within the handover angle, moving towards upright, the
wheels are on the ground and the normal balance controller takes over.

This script simulates exactly that, so the self-righting settings can be tried
here first:

  * the hoop phase: the robot rolls on its hoop (outline in hoop_outline.csv,
    plus the tyres), the wheels spin freely in the air as reaction wheels;
  * the wheel phase: the inverted pendulum on its wheels, with Coulomb
    friction (the tyres can slip), and back onto the hoop if it falls back;
  * the motors and drivers as in sbr_model.py (PI current loop, back-EMF,
    voltage limit), the controller at the control rate with one step of
    delay.

Not modelled: bouncing on the hoop or the floor, the hoop slipping, the two
hoops not lining up, turning. Treat a pass here as "should work", not proof.

Usage:
  python self_right.py                      # both sides, settings from the firmware defaults
  python self_right.py --current 1.0 --wheel-cap 150
  python self_right.py --sweep              # a table over current, wheel cap, Kt and grip
  python self_right.py --side back --csv run.csv   # also write the time series
  python self_right.py --step Ring.step     # remake hoop_outline.csv from a STEP (needs cadquery-ocp)
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

import sbr_model as sm

HERE = os.path.dirname(os.path.abspath(__file__))
G = sm.G


# ---------------------------------------------------------------------------
# Geometry: what the robot touches the floor with
# ---------------------------------------------------------------------------


def load_outline(path: str) -> np.ndarray:
    """Hoop outline [m] as (h, up) rows, origin on the axle."""
    pts = np.loadtxt(path, delimiter=",", comments="#", skiprows=0, dtype=str)
    rows = [r for r in pts if not r[0].strip().startswith("h_mm")]
    return np.array([[float(a), float(b)] for a, b in rows]) / 1000.0


@dataclass
class Shape:
    """Rolling tables over the tilt angle, for the whole robot lying or
    rocking: the height of the CoM above the floor, its derivative, the
    pitch inertia about the contact point, and whether the floor touches
    the tyre or the hoop."""
    th: np.ndarray          # tilt grid [rad]
    height: np.ndarray      # CoM height above the floor [m]
    dheight: np.ndarray
    inertia: np.ndarray     # about the contact point [kg m^2], spinning parts excluded
    dinertia: np.ndarray
    on_wheel: np.ndarray    # bool
    axle_height: np.ndarray # axle above the floor [m]
    touch: float            # tilt at which the tyres touch (positive side) [rad]
    rest: float             # resting tilt on the hoop (positive side) [rad]


def make_shape(r: sm.Robot, hoop: np.ndarray) -> Shape:
    m = r.mass_total
    L = r.com_height * r.body_mass / m          # whole-robot CoM above the axle
    # Whole robot about its own CoM (spinning inertia excluded; the wheels'
    # mass sits on the axle).
    i_cm = r.body_inertia + r.body_mass * (r.com_height - L) ** 2 + 2 * r.rot_mass * L * L
    a = np.linspace(0, 2 * np.pi, 1440, endpoint=False)
    tyre = np.c_[r.wheel_radius * np.cos(a), r.wheel_radius * np.sin(a)]
    pts = np.vstack([hoop, tyre])
    is_tyre = np.r_[np.zeros(len(hoop), bool), np.ones(len(tyre), bool)]
    th = np.radians(np.arange(-150.0, 150.0001, 0.05))
    height, inertia, on_wheel, axle_h = [], [], [], []
    com = np.array([0.0, L])
    for f in th:
        up = np.array([-math.sin(f), math.cos(f)])   # world "up" in body coordinates, tilted by f towards +h
        s = pts @ up
        k = int(np.argmin(s))
        height.append(com @ up - s[k])
        axle_h.append(-s[k])
        inertia.append(i_cm + m * float(np.sum((com - pts[k]) ** 2)))
        on_wheel.append(bool(is_tyre[k]))
    height = np.array(height)
    inertia = np.array(inertia)
    on_wheel = np.array(on_wheel)
    pos = th > 0
    edge = np.where(pos[:-1] & on_wheel[:-1] & ~on_wheel[1:])[0]
    touch = float(th[edge[0]]) if len(edge) else math.radians(36)
    side = (th > touch) & (th < math.radians(120))
    rest = float(th[side][np.argmin(height[side])])
    return Shape(th, height, np.gradient(height, th), inertia, np.gradient(inertia, th), on_wheel,
                 np.array(axle_h), touch, rest)


# ---------------------------------------------------------------------------
# Settings (firmware names in brackets)
# ---------------------------------------------------------------------------


@dataclass
class Settings:
    current: float = 1.5        # srCurrent [A per wheel]
    wheel_cap: float = 200.0    # srWheelCap [rad/s]
    handover: float = 34.0      # srHandover [deg]
    timeout: float = 6.0        # srTimeout [s]
    k0: float = -0.23
    k1: float = -20.0
    k3: float = -0.24
    k4: float = -2.0
    max_pos_err: float = 2.0
    deact: float = 42.0         # deactAngle [deg]
    runaway: float = 60.0       # runawaySpeed [rad/s]
    mu: float = 0.6             # tyre grip (Coulomb friction coefficient)


HYST = math.radians(3.0)        # SR_RATE_HYSTERESIS
HOLD_FACTOR = 1.5               # HOLD_LIMIT_FACTOR
RUNAWAY_MS = 0.3


@dataclass
class Result:
    side: str
    ok: bool
    why: str
    t_wheels: float = float("nan")     # tyres first touch
    t_handover: float = float("nan")
    swings: int = 0                    # direction changes while rocking
    handover_rate: float = float("nan")  # deg/s
    handover_wheel: float = float("nan") # rad/s, relative to the body
    overshoot: float = float("nan")      # deg past upright (other side)
    travel: float = float("nan")         # m, after handover
    slip_ms: float = 0.0
    peak_wheel: float = 0.0


def simulate(r: sm.Robot, shape: Shape, s: Settings, side: int = 1, t_end_after: float = 3.0,
             trace: Optional[List[tuple]] = None) -> Result:
    """One attempt from rest on `side` (+1 lying on its back = positive tilt,
    -1 on its front)."""
    dt = 5e-5
    ctrl_every = max(1, int(round(r.ts / dt)))
    m, mb, mw = r.mass_total, r.body_mass, 2 * r.rot_mass
    l, ib, rw = r.com_height, r.body_inertia, r.wheel_radius
    iw = 2 * r.rot_inertia
    j_axle = ib + mb * l * l
    ke, R, P, I, V = r.ke, r.resistance, r.cl_p, r.cl_i, r.voltage_limit

    def tab(arr, th):
        return float(np.interp(th, shape.th, arr))

    # State. Hoop phase: th, w (body rate), wr (wheel speed relative to the body).
    # Wheel phase adds x, xd; the wheel's absolute speed is w + wr.
    th = side * shape.rest
    w = 0.0
    wr = 0.0
    x = xd = 0.0
    on_wheels = False
    xi = 0.0                       # current loop integrator (both motors alike)
    cmd = 0.0                      # per-wheel current command being applied
    pending = 0.0                  # command computed this step, applied next (1-step delay)
    mode = "rock"
    direction = -side              # wanted direction of the body torque: towards upright first
    phi = 0.0
    swings = 0
    res = Result("back" if side > 0 else "front", False, "")
    t = 0.0
    step = 0
    t_hand = None
    run_since = None
    min_after = 0.0
    i_max = min(r.max_current, 2.0)

    while True:
        # ---- controller, at the control rate -------------------------------
        if step % ctrl_every == 0:
            cmd = pending
            if mode == "rock":
                if w > HYST and direction < 0:
                    direction, swings = 1, swings + 1
                elif w < -HYST and direction > 0:
                    direction, swings = -1, swings + 1
                u = -direction * s.current
                if abs(wr) >= s.wheel_cap and (u > 0) == (wr > 0):
                    u = 0.0
                if abs(th) < math.radians(s.handover) and (th * w < 0):
                    mode = "balance"
                    t_hand = t
                    phi = 0.0
                    res.t_handover = t
                    res.handover_rate = math.degrees(w)
                    res.handover_wheel = wr
                    res.swings = swings
                elif t > s.timeout:
                    res.why = "timed out"
                    return res
                else:
                    pending = u
            if mode == "balance":
                pos_err = max(-s.max_pos_err, min(s.max_pos_err, phi))
                tilt = -(s.k1 * th + s.k4 * w)
                hold = -(s.k0 * pos_err + s.k3 * wr)
                if r.balance_priority:
                    hm = HOLD_FACTOR * 2 * i_max
                    hold = max(-hm, min(hm, hold))
                pending = max(-i_max, min(i_max, (tilt + hold) / 2))
                if abs(th) > math.radians(s.deact):
                    res.why = "fell over after the handover"
                    return res
                if abs(wr) > max(s.runaway, 60):
                    run_since = t if run_since is None else run_since
                    if t - run_since >= RUNAWAY_MS:
                        res.why = "runaway stop after the handover"
                        return res
                else:
                    run_since = None
                min_after = min(min_after, th * side)
                if t - t_hand >= t_end_after:
                    res.ok = True
                    res.why = "up"
                    res.overshoot = math.degrees(-min_after)
                    res.travel = x
                    return res
            if trace is not None:
                trace.append((t, math.degrees(th), math.degrees(w), wr, x, xd, cmd, mode, on_wheels))

        # ---- motors: PI current loop with back-EMF and the voltage limit ----
        i = (P * cmd + xi - ke * wr) / (R + P)
        v = R * i + ke * wr
        if abs(v) > V:
            v = math.copysign(V, v)
            i = (v - ke * wr) / R
        xi = max(-V, min(V, xi + I * (cmd - i) * dt))
        fric = r.friction * (math.tanh(wr / 0.5))
        tau = 2 * (r.kt * i - fric)          # both motors, on the wheels (reaction on the body: -tau)
        res.peak_wheel = max(res.peak_wheel, abs(wr))

        # ---- mechanics ------------------------------------------------------
        wheel_support = abs(th) <= shape.touch
        if not wheel_support:
            if on_wheels:
                on_wheels = False                     # back onto the hoop
            j = tab(shape.inertia, th)
            dj = tab(shape.dinertia, th)
            dh = tab(shape.dheight, th)
            thdd = (-tau - 0.5 * dj * w * w - m * G * dh) / j
            wrdd = tau / iw - thdd
            w += thdd * dt
            th += w * dt
            wr += wrdd * dt
        else:
            if not on_wheels:
                on_wheels = True
                if math.isnan(res.t_wheels):
                    res.t_wheels = t
                xd = w * tab(shape.axle_height, th)   # rolling on the hoop moved the axle
            psid = w + wr                             # absolute wheel speed
            c, sn = math.cos(th), math.sin(th)
            vs = xd - rw * psid                       # tyre slip speed
            a = np.array([[mb + mw, mb * l * c], [mb * l * c, j_axle]])

            def solve(F):
                xdd_, thdd_ = np.linalg.solve(a, [F + mb * l * sn * w * w, -tau + mb * G * l * sn])
                return xdd_, thdd_, (tau - rw * F) / iw

            if abs(vs) < 1e-3:
                A = np.array([[mb + mw, mb * l * c, -1.0], [mb * l * c, j_axle, 0.0], [1.0, 0.0, rw * rw / iw]])
                xdd, thdd, F = np.linalg.solve(A, [mb * l * sn * w * w, -tau + mb * G * l * sn, rw * tau / iw])
                psidd = (tau - rw * F) / iw
                N = m * G - mb * l * (thdd * sn + w * w * c)
                if abs(F) > s.mu * max(N, 0):
                    F = math.copysign(s.mu * max(N, 0), F)
                    xdd, thdd, psidd = solve(F)
                    res.slip_ms += dt * 1000
            else:
                F = -math.copysign(s.mu * m * G, vs)
                for _ in range(3):
                    xdd, thdd, psidd = solve(F)
                    N = m * G - mb * l * (thdd * sn + w * w * c)
                    F = -math.copysign(s.mu * max(N, 0), vs)
                res.slip_ms += dt * 1000
            xd += xdd * dt
            x += xd * dt
            w += thdd * dt
            th += w * dt
            psid += psidd * dt
            if abs(vs) >= 1e-3 and np.sign(xd - rw * psid) != np.sign(vs):
                xd = rw * psid                        # slip stopped
            wr = psid - w
        if mode == "balance":
            phi += wr * dt
        t += dt
        step += 1
        if abs(th) > math.radians(120):
            res.why = "rolled over the top of the hoop"
            return res


# ---------------------------------------------------------------------------


def outline_from_step(step_path: str, out_csv: str) -> None:
    """Hoop outline from a STEP of the ring, positioned as on the robot: the
    origin on the axle, the ring in the wheel plane, -x up (as Ring.step).
    Sections it through its middle and keeps the convex hull."""
    try:
        from OCP.STEPControl import STEPControl_Reader
        from OCP.BRepAlgoAPI import BRepAlgoAPI_Section
        from OCP.gp import gp_Pln, gp_Pnt, gp_Dir
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopAbs import TopAbs_EDGE
        from OCP.TopoDS import TopoDS
        from OCP.BRepAdaptor import BRepAdaptor_Curve
        from OCP.Bnd import Bnd_Box
        from OCP.BRepBndLib import BRepBndLib
    except ImportError:
        sys.exit("--step needs the OpenCascade bindings: python -m pip install cadquery-ocp")
    from scipy.spatial import ConvexHull
    rd = STEPControl_Reader()
    rd.ReadFile(step_path)
    rd.TransferRoots()
    shp = rd.OneShape()
    box = Bnd_Box()
    BRepBndLib.Add_s(shp, box)
    zmid = (box.CornerMin().Z() + box.CornerMax().Z()) / 2
    sec = BRepAlgoAPI_Section(shp, gp_Pln(gp_Pnt(0, 0, zmid), gp_Dir(0, 0, 1)))
    sec.Build()
    ex = TopExp_Explorer(sec.Shape(), TopAbs_EDGE)
    pts = []
    while ex.More():
        cv = BRepAdaptor_Curve(TopoDS.Edge(ex.Current()))
        for u in np.linspace(cv.FirstParameter(), cv.LastParameter(), 400):
            p = cv.Value(u)
            pts.append((p.Y(), -p.X()))          # (h, up)
        ex.Next()
    pts = np.array(pts)
    hull = pts[ConvexHull(pts).vertices]
    hull = hull[np.argsort(np.arctan2(hull[:, 0], hull[:, 1]))]
    with open(out_csv, "w") as f:
        f.write(f"# Outer hoop from {os.path.basename(step_path)}: convex hull of its mid-thickness section.\n")
        f.write("# Body frame, origin on the axle: up_mm = towards the top of the robot, h_mm = fore/aft.\n")
        f.write("h_mm,up_mm\n")
        for h, u in hull:
            f.write(f"{h:.2f},{u:.2f}\n")
    print(f"Wrote {out_csv}: {len(hull)} points")


def describe(shape: Shape, r: sm.Robot) -> None:
    L = r.com_height * r.body_mass / r.mass_total
    i_rest = int(np.argmin(abs(shape.th - shape.rest)))
    i_touch = int(np.argmin(abs(shape.th - shape.touch)))
    i_up = int(np.argmin(abs(shape.th)))
    print(f"Whole-robot CoM {L * 1000:.1f} mm above the axle. Lying on the hoop it rests at "
          f"+-{math.degrees(shape.rest):.1f} deg (CoM {shape.height[i_rest] * 1000:.1f} mm up); "
          f"the tyres touch at +-{math.degrees(shape.touch):.1f} deg "
          f"(CoM {shape.height[i_touch] * 1000:.1f} mm, i.e. "
          f"{r.mass_total * G * (shape.height[i_touch] - shape.height[i_rest]) * 1000:.0f} mJ to get there); "
          f"upright {shape.height[i_up] * 1000:.1f} mm.")


def fmt(res: Result) -> str:
    if not res.ok:
        extra = f" (tyres touched at {res.t_wheels:.2f} s)" if not math.isnan(res.t_wheels) else ""
        return f"{res.side:5s} FAIL: {res.why}{extra}"
    return (f"{res.side:5s} up: tyres touch {res.t_wheels:.2f} s, handover {res.t_handover:.2f} s after "
            f"{res.swings} swing reversals ({res.handover_rate:+.0f} deg/s, wheels {res.handover_wheel:+.0f} rad/s); "
            f"overshoot {res.overshoot:.1f} deg, moved {res.travel * 1000:+.0f} mm, slipped {res.slip_ms:.0f} ms, "
            f"wheels peaked at {res.peak_wheel:.0f} rad/s")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--robot", default=os.path.join(HERE, "robot.json"))
    ap.add_argument("--hoop", default=os.path.join(HERE, "hoop_outline.csv"))
    ap.add_argument("--step", help="remake the hoop outline (--hoop) from this STEP file and exit")
    ap.add_argument("--side", choices=["back", "front", "both"], default="both")
    ap.add_argument("--current", type=float, default=1.5, help="srCurrent, A per wheel")
    ap.add_argument("--wheel-cap", type=float, default=200, help="srWheelCap, rad/s")
    ap.add_argument("--handover", type=float, default=34, help="srHandover, deg")
    ap.add_argument("--timeout", type=float, default=6, help="srTimeout, s")
    ap.add_argument("--gains", type=float, nargs=4, metavar=("K0", "K1", "K3", "K4"), default=[-0.23, -20, -0.24, -2.0])
    ap.add_argument("--max-pos-err", type=float, default=2.0)
    ap.add_argument("--mu", type=float, default=0.6, help="tyre grip on the floor")
    ap.add_argument("--kt", type=float, help="override the motor Kt from robot.json")
    ap.add_argument("--max-current", type=float, help="override maxCurrent (A per wheel) from robot.json")
    ap.add_argument("--sweep", action="store_true", help="table over current, wheel cap, Kt and grip")
    ap.add_argument("--csv", help="write the time series of the run (one side) to this file")
    a = ap.parse_args()

    if a.step:
        outline_from_step(a.step, a.hoop)
        return

    r = sm.load_robot(a.robot)
    if a.kt:
        r = r.copy(kt=a.kt)
    if a.max_current:
        r = r.copy(max_current=a.max_current)
    shape = make_shape(r, load_outline(a.hoop))
    print(sm.describe(r))
    describe(shape, r)
    s = Settings(current=a.current, wheel_cap=a.wheel_cap, handover=a.handover, timeout=a.timeout,
                 k0=a.gains[0], k1=a.gains[1], k3=a.gains[2], k4=a.gains[3], max_pos_err=a.max_pos_err, mu=a.mu)
    sides = {"back": [1], "front": [-1], "both": [1, -1]}[a.side]

    if a.sweep:
        print("\nOne side (they're symmetric in the model).")
        print("  Kt    grip  A     cap    handover  swings  rate / wheels at handover   overshoot  moved  slip  peak wheels")
        for kt in (0.045, 0.054):
            rr = r.copy(kt=kt)
            for mu in (0.5, 0.8):
                for cur in (1.0, 1.5, 1.8):
                    for cap in (100, 150, 200, 400):
                        ss = Settings(**{**s.__dict__, "current": cur, "wheel_cap": cap, "mu": mu})
                        res = simulate(rr, shape, ss, 1)
                        row = (f"{res.t_handover:5.2f} s {res.swings:2d}  {res.handover_rate:+5.0f} deg/s {res.handover_wheel:+5.0f} rad/s  "
                               f"{res.overshoot:4.1f} deg {res.travel * 1000:+5.0f} mm {res.slip_ms:4.0f} ms {res.peak_wheel:4.0f} rad/s"
                               if res.ok else "FAIL: " + res.why)
                        print(f"  {kt:.3f} {mu:4.1f} {cur:5.1f} {cap:5.0f}   {row}")
        return

    print(f"\nRocking {s.current:g} A, wheel cap {s.wheel_cap:g} rad/s, handover {s.handover:g} deg, "
          f"gains K0 {s.k0:g} K1 {s.k1:g} K3 {s.k3:g} K4 {s.k4:g}, maxCurrent {r.max_current:g} A, grip {s.mu:g}:")
    for sd in sides:
        tr: Optional[list] = [] if a.csv else None
        print("  " + fmt(simulate(r, shape, s, sd, trace=tr)))
        if a.csv and tr:
            with open(a.csv, "w") as f:
                f.write("t,tilt_deg,tilt_rate_dps,wheel_rel_rads,x_m,xd_ms,cmd_a,mode,on_wheels\n")
                for row in tr:
                    f.write(",".join(str(v) if not isinstance(v, float) else f"{v:.5g}" for v in row) + "\n")
            print(f"  wrote {a.csv}")


if __name__ == "__main__":
    main()
