#!/usr/bin/env python3
"""Balance controller gains (K0, K1, K3, K4, maxPosErr) for a robot.

    python design_gains.py robot.json [--compare capture.csv | --compare K0,K1,K3,K4]

Designs gains for the robot in robot.json (an LQR on the firmware's four
gains, at the controller's real rate, with the drivers' current loop, the
wheel-speed filter and a control delay included). Because the CoM height is
usually the least certain number, the design and every check are done at the
CoM height in robot.json and +-30% of it (--com-spread).

For each gain set it prints:
  * damping: the worst oscillation's damping ratio (0 = keeps ringing,
    0.3+ is good, 0.5+ very calm) and its frequency;
  * slowest mode: how long it takes to settle back onto its spot;
  * margin: how far the loop is from instability (1 = very robust, under
    ~0.3 = fragile; a small change in the robot could make it oscillate);
  * pushes: a simulated push (a sudden tilt rate), with the current limit
    and the drivers' voltage limit: peak tilt, peak current, how far it
    drifts and how long it takes to settle.

--compare adds the gains you're running now, from a capture's header or
written out, so you can see what changes. --gentle / --firm set how hard the
design may use the motors (per-motor current a push is allowed to ask for).
"""

import argparse
import math

import numpy as np

import sbr_model as sm


def parse_gains(s: str):
    if s.lower().endswith(".csv"):
        g = sm.gains_from_capture(s)
        if g is None:
            raise SystemExit(f"{s}: no gains in the header")
        return g
    k = [float(x) for x in s.split(",")]
    if len(k) not in (4, 5):
        raise SystemExit("--compare K0,K1,K3,K4[,maxPosErr]")
    return sm.Gains(*k)


def report(name, g, robots, kicks, ideal):
    print(f"\n{name}: {g}")
    for r in robots:
        a = sm.analyse(r, g, ideal)
        print(f"  CoM {r.com_height * 1000:5.1f} mm: {a}")
    r = robots[len(robots) // 2]
    for k in kicks:
        print(f"  push {k:4.0f} deg/s: {sm.simulate(r, g, kick_dps=k, ideal_current=ideal).summary(r)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("robot")
    ap.add_argument("--compare", action="append", default=[], help="capture.csv or K0,K1,K3,K4[,maxPosErr]")
    ap.add_argument("--com-spread", type=float, default=0.3, help="check the CoM height +- this fraction (default 0.3)")
    ap.add_argument("--gentle", type=float, default=0.5, help="per-motor current a push may ask for, gentle design [A]")
    ap.add_argument("--firm", type=float, default=0.75, help="same, firmer design [A]")
    ap.add_argument("--max-tilt", type=float, default=3.0, help="tilt you'd accept during a push [deg]")
    ap.add_argument("--max-pos", type=float, default=3.0, help="wheel position error you'd accept [rad]")
    ap.add_argument("--kicks", default="60,120,200", help="simulated pushes [deg/s]")
    ap.add_argument("--ideal-current", action="store_true", help="assume the drivers' current follows instantly")
    a = ap.parse_args()

    r0 = sm.load_robot(a.robot)
    print(sm.describe(r0))
    s = a.com_spread
    robots = [r0.copy(com_height=r0.com_height * f) for f in (1 - s, 1.0, 1 + s)]
    kicks = [float(x) for x in a.kicks.split(",") if x]

    for c in a.compare:
        report(f"Now ({c})", parse_gains(c), robots, kicks, a.ideal_current)

    start = sm.Gains(-0.15, -28, -0.18, -1.5)
    results = []
    for label, amps in (("Gentle", a.gentle), ("Firm", a.firm)):
        g = sm.design(robots, start, max_tilt_deg=a.max_tilt, max_pos=a.max_pos, max_current=amps,
                      ideal_current=a.ideal_current)
        g.max_pos_err = sm.suggest_max_pos_err(g)
        results.append((label, g))
        report(f"{label} design ({amps:g} A per motor)", g, robots, kicks, a.ideal_current)

    print("\nTo try one, set these on the Tune tab (they're the firmware's settings names):")
    for label, g in results:
        print(f"  {label:6s}  K0 {g.k0:.3f}   K1 {g.k1:.2f}   K3 {g.k3:.3f}   K4 {g.k4:.3f}   maxPosErr {g.max_pos_err:g}")
    print("Start with Gentle. Record a 30 s capture with a few pushes and check it with check_log.py.")


if __name__ == "__main__":
    main()
