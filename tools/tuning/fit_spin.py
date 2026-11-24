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

With driver firmware 1.6+ the capture also has the currents the drivers
MEASURED (iq_l / iq_r). Then Kt is also fitted straight from those
(torque = wheel inertia x acceleration = Kt x measured current - friction),
which needs no current-loop model and works in longer tests where the motors
run out of voltage (e.g. 0.5 A, 1000 ms). It also prints how the measured
current compares with the command at low wheel speed.
"""

import argparse
import json
import math
import re
import sys

import numpy as np
from scipy.optimize import least_squares

import sbr_model as sm


def fit_wheel(t, cmd, speed, r: sm.Robot, vlim: float):
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
                v = r.resistance * i + ke * w
                if abs(v) > vlim:          # out of voltage (long steps reach the top speed)
                    i = (math.copysign(vlim, v) - ke * w) / r.resistance
                x = max(-vlim, min(vlim, x + r.cl_i * (u - i) * dt[k]))
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


def fit_wheel_measured(t, iq, speed, tilt_rate_dps, J, lag):
    """Kt and friction from the measured q current: J dw/dt = Kt iq - fc sign(w).
    w is the wheel's absolute speed (relative speed + body rate: lying on its
    hoop the body rocks). The measured current goes through the reported
    speed's lag (robot.json wheel_speed_filter_s) so the two line up. None if
    there are no currents."""
    from scipy.signal import savgol_filter
    ok = np.isfinite(iq)
    if ok.sum() < 50:
        return None
    iq = np.where(ok, iq, 0.0)
    w = speed + np.radians(np.nan_to_num(tilt_rate_dps))
    dt = np.diff(t, prepend=t[0] - 0.002)
    iqf = np.empty_like(iq)
    acc = 0.0
    for k in range(len(iq)):
        acc += (iq[k] - acc) * min(1.0, dt[k] / max(lag, 1e-4))
        iqf[k] = acc
    a = savgol_filter(w, 9, 2, deriv=1, delta=float(np.median(dt)))
    m = ok & (np.abs(w) > 3.0)
    if m.sum() < 30:
        return None
    A = np.c_[iqf[m], -np.sign(w[m])]
    (kt, fc), *_ = np.linalg.lstsq(A, J * a[m], rcond=None)
    rms = float(np.sqrt(np.mean((A @ [kt, fc] - J * a[m]) ** 2)))
    return float(kt), float(fc), rms


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
    cl = json.load(open(a.robot)).get("current_loop", {})
    kts, fcs, lags = [], [], []
    for path in a.captures:
        d = sm.load_capture(path)
        # The motors' voltage limit at the battery voltage of this test.
        batt = None
        with open(path) as f:
            for line in f:
                if not line.startswith("#"):
                    break
                mm = re.search(r"battery_v: ([0-9.]+)", line)
                if mm and float(mm.group(1)) > 5:
                    batt = float(mm.group(1))
        vlim, how = sm.q_voltage_limit({**cl, "supply_v": batt} if batt else cl)
        print(f"{path}: motor voltage up to {vlim:.2f} V ({how})")
        if "tipped" not in set(d.state):
            print(f"{path}: not a spin test (the robot wasn't lying down)", file=sys.stderr)
        t = d.t_ms.values / 1000.0
        for side, cur, meas in (("wheel_l", "cur_l", "iq_l"), ("wheel_r", "cur_r", "iq_r")):
            kt, fc, lag, rms, peak = fit_wheel(t, d[cur].values, d[side].values, r, vlim)
            kts.append(kt)
            fcs.append(fc)
            lags.append(lag)
            print(f"{path}  {side}: Kt {kt:.4f} N m/A, friction {fc * 1000:.2f} mN m ({fc / kt:.3f} A), "
                  f"wheel speed lag {lag * 1000:.1f} ms, peak {peak:.0f} rad/s, fit error {rms:.1f} rad/s rms")
            if meas in d.columns:
                iq = d[meas].values.astype(float)
                fm = fit_wheel_measured(t, iq, d[side].values, d["tilt_rate"].values, r.rot_inertia, r.speed_filter)
                if fm:
                    ktm, fcm, rmsm = fm
                    cmd = d[cur].values
                    low = np.isfinite(iq) & (np.abs(cmd) > 0.05) & (np.abs(d[side].values) < 20)
                    ratio = (f", measured / commanded at low speed {np.median(iq[low] / cmd[low]):.2f}"
                             if low.sum() > 10 else "")
                    print(f"{'':{len(path)}}  {side}: from the MEASURED current: Kt {ktm:.4f} N m/A, "
                          f"friction {fcm * 1000:.2f} mN m{ratio} (fit error {rmsm * 1000:.2f} mN m rms)")
    kt, fc, lag = float(np.mean(kts)), float(np.mean(fcs)), float(np.mean(lags))
    print(f"\nFor robot.json: \"kt_nm_per_a\": {kt:.4f}, \"friction_nm\": {fc:.5f}, "
          f"\"wheel_speed_filter_s\": {lag:.3f}")
    if max(kts) > 1.25 * min(kts):
        print("The wheels disagree by more than 25%: check both drivers have the same current loop settings, "
              "and that the wheels were spinning freely.")


if __name__ == "__main__":
    main()
