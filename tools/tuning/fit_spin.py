#!/usr/bin/env python3
"""Motor torque constant from a wheel spin test.

Record the test on the robot (web page, Drivers tab, "Wheel spin test"): lay
the robot down so both wheels turn freely; 0.3 A with 100 ms steps works well
(at 0.1 A friction and cogging dominate). Download the CSV from Record.

    python fit_spin.py robot.json capture.csv [more.csv ...] [--cl-i 3000]

Fits each wheel's speed with the motor + current-loop model, knowing the
wheel's spinning inertia (robot.json, from CAD), and prints the torque
constant Kt (N m per commanded amp), the friction, and the lag of the wheel
speed reported over CAN (the drivers' velocity filter plus timing) to put in
robot.json.
--cl-p / --cl-i give the drivers' current loop gains at the time of the test
if they differ from robot.json (the capture doesn't record them).
"""

import argparse
import math
import sys

import numpy as np
from scipy.optimize import least_squares

import sbr_model as sm


def fit_wheel(t, cmd, speed, r: sm.Robot):
    dt = np.diff(t, prepend=t[0] - 0.002)
    active = np.nonzero(np.abs(cmd) > 1e-6)[0]
    end_t = t[active[-1]] + 0.4 if len(active) else t[-1]
    m = t <= end_t
    J = r.rot_inertia

    def sim(p, with_loop=True):
        kt, fc, speed_filter = p
        ke = kt / 1.5
        w = x = wf = 0.0
        out = np.empty(len(t))
        for k in range(len(t)):
            u = cmd[k]
            if with_loop:
                i = (r.cl_p * u + x - ke * w) / (r.resistance + r.cl_p)
                x += r.cl_i * (u - i) * dt[k]
            else:
                i = u
            w += (kt * i - fc * math.tanh(w)) / J * dt[k]
            wf += (w - wf) * dt[k] / speed_filter
            out[k] = wf
        return out

    res = least_squares(lambda p: (sim(p) - speed)[m], [0.05, 0.001, 0.008],
                        bounds=([0.003, 0, 0.001], [0.5, 0.05, 0.05]), x_scale=[0.01, 0.001, 0.005])
    rms = float(np.sqrt(np.mean(((sim(res.x) - speed)[m]) ** 2)))
    return res.x[0], res.x[1], res.x[2], rms, float(np.abs(speed[m]).max())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("robot")
    ap.add_argument("captures", nargs="+")
    ap.add_argument("--cl-p", type=float, help="driver current loop P during the test (default: robot.json)")
    ap.add_argument("--cl-i", type=float, help="driver current loop I during the test (default: robot.json)")
    a = ap.parse_args()
    r = sm.load_robot(a.robot)
    if a.cl_p is not None:
        r = r.copy(cl_p=a.cl_p)
    if a.cl_i is not None:
        r = r.copy(cl_i=a.cl_i)
    print(f"Wheel spinning inertia {r.rot_inertia * 1e9:.0f} g mm^2 per side, motor {r.resistance:g} ohm, "
          f"current loop P {r.cl_p:g} I {r.cl_i:g}")
    kts, fcs, lags = [], [], []
    for path in a.captures:
        d = sm.load_capture(path)
        if "tipped" not in set(d.state):
            print(f"{path}: not a spin test (the robot wasn't lying down)", file=sys.stderr)
        t = d.t_ms.values / 1000.0
        for side, cur in (("wheel_l", "cur_l"), ("wheel_r", "cur_r")):
            kt, fc, lag, rms, peak = fit_wheel(t, d[cur].values, d[side].values, r)
            kts.append(kt)
            fcs.append(fc)
            lags.append(lag)
            print(f"{path}  {side}: Kt {kt:.4f} N m/A, friction {fc * 1000:.2f} mN m ({fc / kt:.3f} A), "
                  f"wheel speed lag {lag * 1000:.1f} ms, peak {peak:.0f} rad/s, fit error {rms:.1f} rad/s rms")
    kt, fc, lag = float(np.mean(kts)), float(np.mean(fcs)), float(np.mean(lags))
    print(f"\nFor robot.json: \"kt_nm_per_a\": {kt:.4f}, \"friction_nm\": {fc:.5f}, "
          f"\"wheel_speed_filter_s\": {lag:.3f}")
    if max(kts) > 1.25 * min(kts):
        print("The wheels disagree by more than 25%: check both drivers have the same current loop settings, "
              "and that the wheels were spinning freely.")


if __name__ == "__main__":
    main()
