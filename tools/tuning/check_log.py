#!/usr/bin/env python3
"""Summarise a balancing capture: how well it holds still, how it recovers
from pushes, and why it fell (if it did).

    python check_log.py capture.csv [more.csv ...] [--robot robot.json]

Record 20-30 s on the robot (Tune tab, Record) while giving it a few pushes
of different strengths, with a pause between them. For each capture:

  * at rest: tilt-rate jitter (deg/s rms) and its main frequency, current
    noise, and how much of the time the position hold was at its limit
    (maxPosErr - if that's often, the robot is creeping and fighting it);
  * pushes: for each one, the peak tilt rate (how hard the push was), the
    peak tilt, time to settle, and whether the motors hit the current limit;
  * falls: what the motors were doing just before (at the current limit,
    wheel speeds beyond what the supply voltage allows full current at).

With --robot, the model's prediction for the capture's gains is printed too
(its oscillation frequency should be near the measured one at rest).
"""

import argparse

import numpy as np
from scipy.signal import welch

import sbr_model as sm


def quiet_mask(d, max_rate=15.0, window=500):
    return ((d.tilt_rate.abs().rolling(window, center=True).max() < max_rate).values
            & (d.state.values == "balancing"))


def pushes(d, threshold=60.0):
    t = d.t_ms.values / 1000.0
    r = d.tilt_rate.values
    e = (d.tilt - d.setpoint).values
    st = d.state.values
    sat_lvl = None
    out = []
    i = 0
    n = len(d)
    while i < n - 1:
        if st[i] == "balancing" and abs(r[i]) > threshold:
            k = i
            settled = None
            while k < n - 150:
                if st[k] != "balancing":
                    break
                if np.all(np.abs(e[k:k + 150]) < 0.7) and np.all(np.abs(r[k:k + 150]) < 15):
                    settled = k
                    break
                k += 5
            end = settled if settled is not None else min(n - 1, k)
            seg = slice(i, end + 1)
            fell = settled is None and end < n - 1 and st[end] != "balancing"
            out.append(dict(t=t[i], peak_rate=float(np.abs(r[seg]).max()), peak_tilt=float(np.abs(e[seg]).max()),
                            settle=(t[settled] - t[i]) if settled is not None else None, fell=fell,
                            i0=i, i1=end))
            i = (settled + 150) if settled is not None else end + 1
        else:
            i += 1
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("captures", nargs="+")
    ap.add_argument("--robot", help="robot.json, to compare with the model")
    ap.add_argument("--cl-i", type=float, help="drivers' current loop I during the capture (default: robot.json)")
    a = ap.parse_args()
    robot = sm.load_robot(a.robot) if a.robot else None
    if robot and a.cl_i is not None:
        robot = robot.copy(cl_i=a.cl_i)

    for path in a.captures:
        d = sm.load_capture(path)
        hdr = sm.parse_capture_header(path)
        g = sm.gains_from_capture(path)
        imax = float(hdr.get("maxCurrent", 1.5))
        t = d.t_ms.values / 1000.0
        I = (d.cur_l + d.cur_r).values
        print(f"\n=== {path} ({t[-1] - t[0]:.1f} s)")
        print(f"Gains: {g}   maxCurrent {imax:g} A")
        counts = d.state.value_counts().to_dict()
        print("States: " + ", ".join(f"{k} {v / len(d) * 100:.0f}%" for k, v in counts.items()))

        q = quiet_mask(d)
        if q.sum() > 2048:
            x = d.tilt_rate.values[q]
            f, p = welch(x - x.mean(), fs=500, nperseg=1024)
            m = (f > 1) & (f < 30)
            rms = np.sqrt(p[m].sum() * (f[1] - f[0]))
            print(f"At rest ({q.mean() * 100:.0f}% of the capture): tilt rate {rms:.2f} deg/s rms, mostly at "
                  f"{f[m][np.argmax(p[m])]:.1f} Hz; current noise {np.std(I[q]):.3f} A rms")
        else:
            print("At rest: not enough quiet time to measure (needs a few seconds without pushes)")
        bal = d.state.values == "balancing"
        if g and bal.any():
            at_lim = (np.abs(d.pos_err.values[bal]) >= 0.98 * g.max_pos_err).mean() * 100
            print(f"Position hold at its limit (maxPosErr {g.max_pos_err:g}) {at_lim:.0f}% of the time")
            if q.sum() > 500 and g.k1:
                # At rest the position term balances a set point error: K0 pos = -K1 (tilt error).
                pos_rest = float(np.median(d.pos_err.values[q]))
                off = np.degrees(-g.k0 * pos_rest / g.k1)
                print(f"  At rest pos_err sits at {pos_rest:+.2f} rad (median): the position hold is making up for "
                      f"a set point about {off:+.2f} deg off" + ("  <- worth checking the set point / trim" if abs(off) > 0.3 else ""))

        ps = pushes(d)
        if ps:
            print("Pushes:")
            for p in ps:
                seg = slice(p["i0"], p["i1"] + 1)
                sat = (np.abs(d.cur_l.values[seg]) >= imax - 0.01).mean() * 100
                res = "FELL" if p["fell"] else (f"settled in {p['settle']:.2f} s" if p["settle"] is not None
                                                 else "didn't settle in the capture")
                print(f"  {p['t']:6.1f} s: push {p['peak_rate']:4.0f} deg/s, leaned {p['peak_tilt']:5.1f} deg, "
                      f"{res}, at the current limit {sat:.0f}%")
            ok = [p for p in ps if p["settle"] is not None]
            if ok:
                print(f"  Recovered {len(ok)}/{len(ps)}: median lean {np.median([p['peak_tilt'] for p in ok]):.1f} deg, "
                      f"median settle {np.median([p['settle'] for p in ok]):.2f} s, hardest push recovered "
                      f"{max(p['peak_rate'] for p in ok):.0f} deg/s")

        st = d.state.values
        for i in np.where((st[:-1] == "balancing") & (st[1:] != "balancing"))[0]:
            a0 = max(0, i - 400)
            seg = slice(a0, i + 1)
            sat = (np.abs(d.cur_l.values[seg]) >= imax - 0.01).mean() * 100
            wmax = float(np.abs(d.phi_dot.values[seg]).max())
            print(f"Stopped balancing at {t[i]:.2f} s -> {st[i + 1]} (tilt {d.tilt.values[i]:.1f} deg). In the last "
                  f"0.8 s: at the current limit {sat:.0f}% of the time, wheels up to {wmax:.0f} rad/s")
            if robot:
                w_v = (robot.voltage_limit - imax * robot.resistance) / robot.ke
                if wmax > w_v:
                    print(f"  Above ~{w_v:.0f} rad/s the drivers can't push {imax:g} A through the motor "
                          f"(voltage limit {robot.voltage_limit:g} V): the push was more than the motors could catch.")

        if robot and g:
            print("Model, these gains:")
            for f in (0.7, 1.0, 1.3):
                r = robot.copy(com_height=robot.com_height * f)
                print(f"  CoM {r.com_height * 1000:5.1f} mm: {sm.analyse(r, g)}")


if __name__ == "__main__":
    main()
