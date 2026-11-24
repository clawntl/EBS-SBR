"""Model of a two-wheeled self-balancing robot for tuning the EBS-SBR balance
controller. Shared by the scripts in this folder (fit_spin.py,
design_gains.py, check_log.py, fit_nudge.py, self_right.py).

What is modelled (pitch / forward motion only; turning is ignored):

  * the robot: a body (everything that doesn't spin with the wheels) on an
    axle, and two wheels. Each wheel's rotating group is the wheel, tyre and
    motor rotor;
  * the motors: torque = Kt x actual current, minus a little friction;
  * the drivers' current loop (SimpleFOC FOC current control): a PI
    controller on a motor of resistance R, with back-EMF, so the actual
    current lags the commanded current;
  * the controller (firmware/src/LQRControl.cpp): u = -(K0 pos_err + K1 tilt
    + K3 wheel_speed + K4 tilt_rate) amps in total, split over the two
    wheels, clamped to +-maxCurrent per wheel; the wheel speed arrives
    through the drivers' velocity filter and a control-loop delay.

Units are SI (m, kg, rad, s, A, N m) unless a name says otherwise. Gains are
in the firmware's units: K0 A/rad (wheel angle), K1 A/rad, K3 A/(rad/s), K4
A/(rad/s), all for the TOTAL current of both wheels.

Sign convention (matches the firmware and the captures): positive tilt
leans the robot towards the direction a positive wheel speed rolls it, so a
positive current catches a positive tilt.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

G = 9.81

# ---------------------------------------------------------------------------
# Robot description (robot.json)
# ---------------------------------------------------------------------------


@dataclass
class Robot:
    name: str = "robot"
    mass_total: float = 0.765          # kg, whole robot as weighed
    wheel_radius: float = 0.030        # m
    track: float = 0.110               # m, tyre centre to centre (not used by the pitch model)
    rot_mass: float = 0.0448           # kg per side: wheel + tyre + rotor
    rot_inertia: float = 1.44e-5       # kg m^2 per side, about the axle
    com_height: float = 0.062          # m, BODY CoM above the axle (wheels excluded)
    body_inertia: float = 1.9e-3       # kg m^2, body about its own CoM, pitch axis
    kt: float = 0.052                  # N m per commanded A, per motor
    resistance: float = 2.8            # ohm, motor resistance PER PHASE (GM3506: 5.6 ohm line-to-line)
    friction: float = 0.0013           # N m per motor (Coulomb)
    cl_p: float = 3.0                  # driver current loop P [V/A]
    cl_i: float = 3000.0               # driver current loop I [V/(A s)]
    voltage_limit: float = 5.9         # V, largest q-axis voltage the drivers can apply (see q_voltage_limit)
    rate_hz: float = 500.0             # control loop rate
    delay_steps: int = 1               # control steps between measurement and applied current
    speed_filter: float = 0.006        # s, wheel speed filter + telemetry delay seen by the controller
    max_current: float = 1.5           # A per wheel (maxCurrent setting)
    balance_priority: bool = True      # balPriority setting (firmware: tilt terms first at the limit)
    notes: Dict[str, str] = field(default_factory=dict)

    @property
    def ts(self) -> float:
        return 1.0 / self.rate_hz

    @property
    def body_mass(self) -> float:
        return self.mass_total - 2 * self.rot_mass

    @property
    def ke(self) -> float:
        """Back-EMF constant on the q axis [V/(rad/s)]. With SimpleFOC's
        amplitude-invariant transform, torque = 1.5 x Ke x Iq."""
        return self.kt / 1.5

    def copy(self, **changes) -> "Robot":
        d = self.__dict__.copy()
        d.update(changes)
        return Robot(**d)


def body_from_cad(cad: dict, mass_total: float, rot_mass: float, rot_inertia: float) -> Tuple[float, float]:
    """Body CoM height and inertia from Fusion's whole-robot properties.

    cad: {"mass_g", "com_height_mm" (CoM distance from the axle, Fusion
    "Center of Mass" component along 'up' with the origin on the axle),
    "inertia_axle_gmm2" (Fusion "Moment of Inertia at Origin" about the axle
    axis)}. The CAD mass is scaled to the weighed mass (inertia scaled the
    same way, CoM kept). Returns (com_height m, body inertia about its CoM).
    """
    m_cad = cad["mass_g"] / 1000.0
    k = mass_total / m_cad
    com_total = cad["com_height_mm"] / 1000.0          # whole robot, wheels included
    i_axle = cad["inertia_axle_gmm2"] * 1e-9 * k       # kg m^2
    m_body = mass_total - 2 * rot_mass
    # The rotating groups sit on the axle: no contribution to the CoM height,
    # and only their own axial inertia to the pitch inertia.
    l = com_total * mass_total / m_body
    i_body_axle = i_axle - 2 * rot_inertia
    return l, i_body_axle - m_body * l * l


def q_voltage_limit(cl: dict) -> Tuple[float, str]:
    """Largest q-axis voltage the drivers can put on the motor, from
    robot.json's current_loop block, and a description of it.

    The PWM outputs swing between 0 and the phase voltage range
    (SimpleFOC driver.voltage_limit). How much of that reaches the motor
    depends on the modulation:
      * "svpwm" (BLDC-Driver firmware 1.5+): the range is max_duty x the
        supply (followed live), capped at voltage_limit_v
        (MOTOR_VOLTAGE_LIMIT); the motor gets up to range / sqrt(3).
      * "sine" (firmware 1.4 and older): the range is voltage_limit_v (10 V),
        capped at the supply; the motor only gets range / 2.
    capture-304 (2026-11-23, firmware 1.4) matched "sine": the free wheels
    topped out at ~137 rad/s, i.e. 5 V of back-EMF."""
    mod = cl.get("modulation", "sine").lower()
    supply = cl.get("supply_v", 11.1)
    vlim = cl.get("voltage_limit_v", 10.0 if mod == "sine" else 12.0)
    if mod == "svpwm":
        rng = min(vlim, cl.get("max_duty", 0.92) * supply)
        return rng / math.sqrt(3), f"Space Vector PWM, {supply:g} V supply -> phases 0-{rng:.2f} V"
    rng = min(vlim, supply)
    return rng / 2, f"Sine PWM, phases 0-{rng:.2f} V"


def load_robot(path: str) -> Robot:
    with open(path) as f:
        j = json.load(f)
    rot = j["rotating_per_side"]
    mass_total = j["mass_total_kg"]
    rot_mass = rot["mass_kg"]
    rot_inertia = rot["inertia_kgm2"]
    if "body" in j and j["body"].get("com_height_m") is not None:
        l = j["body"]["com_height_m"]
        ib = j["body"]["inertia_cm_kgm2"]
    else:
        l, ib = body_from_cad(j["cad"], mass_total, rot_mass, rot_inertia)
    m = j.get("motor", {})
    cl = j.get("current_loop", {})
    c = j.get("control", {})
    return Robot(
        name=j.get("name", "robot"), mass_total=mass_total, wheel_radius=j["wheel_radius_m"],
        track=j.get("track_m", 0.11), rot_mass=rot_mass, rot_inertia=rot_inertia,
        com_height=l, body_inertia=ib,
        kt=m.get("kt_nm_per_a", 0.05), resistance=m.get("resistance_ohm", 2.8),
        friction=m.get("friction_nm", 0.0013),
        cl_p=cl.get("p", 3.0), cl_i=cl.get("i", 300.0), voltage_limit=q_voltage_limit(cl)[0],
        rate_hz=c.get("rate_hz", 500.0), delay_steps=int(c.get("delay_steps", 1)),
        speed_filter=c.get("wheel_speed_filter_s", 0.006), max_current=c.get("max_current_a", 1.5),
        balance_priority=bool(c.get("balance_priority", True)),
    )


def describe(r: Robot) -> str:
    return (f"{r.name}: {r.mass_total*1000:.0f} g, wheel r {r.wheel_radius*1000:.1f} mm, body CoM {r.com_height*1000:.1f} mm "
            f"above the axle, body inertia {r.body_inertia*1e9:.0f} g mm^2 (about its CoM), "
            f"rotating {r.rot_mass*1000:.1f} g / {r.rot_inertia*1e9:.0f} g mm^2 per side, Kt {r.kt:.4f} N m/A, "
            f"current loop P {r.cl_p:g} I {r.cl_i:g} on R {r.resistance:g} ohm, motor voltage up to {r.voltage_limit:.2f} V")


# ---------------------------------------------------------------------------
# Gains
# ---------------------------------------------------------------------------


@dataclass
class Gains:
    k0: float  # wheel position   [A/rad]
    k1: float  # tilt              [A/rad]
    k3: float  # wheel speed       [A/(rad/s)]
    k4: float  # tilt rate         [A/(rad/s)]
    max_pos_err: float = 10.0  # rad

    def total_current(self, r: "Robot", pos, tilt, wheel, rate, balance_priority=None):
        """The firmware's balance current (total, both wheels) before the
        per-wheel clamp, including balance priority (LQRControl.cpp,
        HOLD_LIMIT_FACTOR = 1.5): the position/speed part is limited to
        1.5 x the total current limit. Works on numpy arrays."""
        tilt_part = -(self.k1 * tilt + self.k4 * rate)
        hold_part = -(self.k0 * pos + self.k3 * wheel)
        bp = r.balance_priority if balance_priority is None else balance_priority
        if bp:
            hold_max = 1.5 * 2 * r.max_current
            hold_part = np.clip(hold_part, -hold_max, hold_max)
        return tilt_part + hold_part

    def as_tuple(self):
        return (self.k0, self.k1, self.k3, self.k4)

    def __str__(self):
        return f"K0 {self.k0:.3f}  K1 {self.k1:.2f}  K3 {self.k3:.3f}  K4 {self.k4:.3f}  maxPosErr {self.max_pos_err:g}"


def parse_capture_header(path: str) -> Dict[str, str]:
    """The '# settings: K1=-25 ...' line of a capture as a dict."""
    out = {}
    with open(path) as f:
        for line in f:
            if not line.startswith("#"):
                break
            if line.startswith("# settings:"):
                body = line.split(":", 1)[1].split("(at the start")[0]
                for kv in body.split():
                    if "=" in kv:
                        k, v = kv.split("=", 1)
                        out[k] = v
    return out


def gains_from_capture(path: str) -> Optional[Gains]:
    s = parse_capture_header(path)
    try:
        return Gains(float(s["K0"]), float(s["K1"]), float(s["K3"]), float(s["K4"]), float(s.get("maxPosErr", 10)))
    except KeyError:
        return None


# ---------------------------------------------------------------------------
# Linear model (for design and stability checks)
# ---------------------------------------------------------------------------
# Continuous states: 0 pos (integral of the filtered wheel speed, as the
# firmware's pos_err), 1 tilt, 2 absolute wheel rate, 3 tilt rate, 4 filtered
# relative wheel speed (what the controller sees), 5 current-loop integrator.
# Input: commanded current per motor. Then discretised and a delay chain of
# delay_steps samples is appended.

N_PLANT = 6


def _mass_matrix(r: Robot, l: float, ib: float):
    jt = 2 * r.rot_inertia + r.mass_total * r.wheel_radius ** 2
    a = ib + r.body_mass * l * l
    c = r.body_mass * r.wheel_radius * l
    return jt, a, c


def plant_continuous(r: Robot, ideal_current: bool = False):
    l, ib, m = r.com_height, r.body_inertia, r.body_mass
    jt, a_, c = _mass_matrix(r, l, ib)
    mi = np.linalg.inv(np.array([[jt, c], [c, a_]]))
    n = N_PLANT
    A = np.zeros((n, n))
    B = np.zeros((n, 1))
    wr = np.array([0, 0, 1, -1, 0, 0.0])  # relative wheel speed = abs wheel rate - tilt rate
    e = np.eye(n)
    if ideal_current:
        i_x, i_u = np.zeros(n), 1.0
    else:
        # current loop: i = (P u + x - Ke wr) / (R + P), x' = I (u - i)
        i_x = (e[5] - r.ke * wr) / (r.resistance + r.cl_p)
        i_u = r.cl_p / (r.resistance + r.cl_p)
    tq_x, tq_u = 2 * r.kt * i_x, 2 * r.kt * i_u          # both motors
    for row, (ma, mb) in zip((2, 3), mi):
        # [abs wheel acc; tilt acc] = Minv [torque; m g l tilt - torque]
        A[row] = ma * tq_x + mb * (m * G * l * e[1] - tq_x)
        B[row] = ma * tq_u - mb * tq_u
    A[0, 4] = 1.0
    A[1, 3] = 1.0
    A[4] = (wr - e[4]) / r.speed_filter
    if ideal_current:
        A[5, 5] = -100.0  # unused, kept stable
    else:
        A[5] = -r.cl_i * i_x
        B[5] = r.cl_i * (1 - i_u)
    return A, B


def plant_discrete(r: Robot, ideal_current: bool = False):
    from scipy.linalg import expm
    A, B = plant_continuous(r, ideal_current)
    n = A.shape[0]
    M = np.zeros((n + 1, n + 1))
    M[:n, :n] = A
    M[:n, n:] = B
    E = expm(M * r.ts)
    Ad, Bd = E[:n, :n], E[:n, n:]
    nd = max(0, int(r.delay_steps))
    if nd == 0:
        return Ad, Bd
    N = n + nd
    A2 = np.zeros((N, N))
    B2 = np.zeros((N, 1))
    A2[:n, :n] = Ad
    A2[:n, n:n + 1] = Bd          # oldest queued command drives the plant
    for k in range(nd - 1):
        A2[n + k, n + k + 1] = 1.0
    B2[N - 1, 0] = 1.0
    return A2, B2


def feedback_row(g: Gains, n: int) -> np.ndarray:
    """Per-motor command u = -F x (the firmware's total current / 2)."""
    f = np.zeros((1, n))
    f[0, 0], f[0, 1], f[0, 4], f[0, 3] = g.k0, g.k1, g.k3, g.k4
    return f / 2


@dataclass
class Analysis:
    stable: bool
    damping: float          # lowest damping ratio of the oscillatory modes below 25 Hz
    damping_hz: float       # frequency of that mode
    settle_s: float         # time constant of the slowest mode
    disk_margin: float      # min |1 + L(e^jwT)| at the plant input (1 = robust, ~0 = on the edge)

    def __str__(self):
        if not self.stable:
            return f"UNSTABLE (mode at {self.damping_hz:.1f} Hz, damping {self.damping:+.2f})"
        return (f"damping {self.damping:.2f} at {self.damping_hz:.1f} Hz, slowest mode {self.settle_s:.1f} s, "
                f"margin |1+L|min {self.disk_margin:.2f}")


def analyse(r: Robot, g: Gains, ideal_current: bool = False) -> Analysis:
    Ad, Bd = plant_discrete(r, ideal_current)
    n = Ad.shape[0]
    F = feedback_row(g, n)
    ev = np.linalg.eigvals(Ad - Bd @ F)
    s = np.log(ev.astype(complex)) / r.ts
    s = s[(np.abs(s) > 1e-3) & (np.abs(s) < 2 * np.pi * 25)]
    stable = bool(np.abs(ev).max() < 1.0)
    osc = s[s.imag > 0.5]
    if len(osc):
        zs = [(-p.real / abs(p), abs(p) / (2 * np.pi)) for p in osc]
        z, hz = min(zs)
    else:
        z, hz = 1.0, 0.0
    slow = -s.real.max() if len(s) else 1.0
    ws = np.logspace(-1, np.log10(np.pi / r.ts * 0.95), 1500)
    eye = np.eye(n)
    dm = min(abs(1 + (F @ np.linalg.solve(np.exp(1j * w * r.ts) * eye - Ad, Bd))[0, 0]) for w in ws)
    return Analysis(stable, z, hz, (1 / slow) if slow > 0 else math.inf, dm)


def design(robots: Sequence[Robot], start: Gains, max_tilt_deg=3.0, max_tilt_rate=1.0, max_pos=3.0,
           max_speed=15.0, max_current=0.5, ideal_current=False) -> Gains:
    """Fixed-structure LQR: the four firmware gains minimising the average
    LQR cost over `robots` (e.g. the robot with its uncertain CoM height at a
    few values). Weights by Bryson's rule: each max_* is the size of that
    state (or of the per-motor current) you'd accept. Smaller max_current =
    gentler, smaller gains."""
    from scipy.linalg import solve_discrete_lyapunov
    from scipy.optimize import minimize

    plants = [plant_discrete(r, ideal_current) for r in robots]
    q = np.zeros(N_PLANT)
    q[0] = 1 / max_pos ** 2
    q[1] = 1 / math.radians(max_tilt_deg) ** 2
    q[3] = 1 / max_tilt_rate ** 2
    q[4] = 1 / max_speed ** 2
    R = np.array([[1 / max_current ** 2]])

    def cost(k):
        g = Gains(*k)
        tot = 0.0
        for A, B in plants:
            n = A.shape[0]
            F = feedback_row(g, n)
            Acl = A - B @ F
            if np.abs(np.linalg.eigvals(Acl)).max() >= 0.99999:
                return 1e9
            Q = np.zeros((n, n))
            Q[:N_PLANT, :N_PLANT] = np.diag(q)
            P = solve_discrete_lyapunov(Acl.T, Q + F.T @ R @ F)
            x0 = np.zeros(n)
            x0[[0, 1, 3]] = [0.5, 0.03, 0.5]   # a push: tilt, tilt rate, position
            tot += float(x0 @ P @ x0)
        return tot / len(plants)

    if cost(start.as_tuple()) >= 1e9:
        # start from something stable
        for cand in [(-0.15, -28, -0.18, -1.5), (-0.1, -30, -0.1, -1.0), (-0.05, -40, -0.1, -2.0)]:
            if cost(cand) < 1e9:
                start = Gains(*cand)
                break
    res = minimize(cost, np.array(start.as_tuple(), float), method="Nelder-Mead",
                   options=dict(maxiter=8000, xatol=1e-6, fatol=1e-7))
    k = res.x
    return Gains(k[0], k[1], k[2], k[3])


def suggest_max_pos_err(g: Gains, bias_amps: float = 0.5) -> float:
    """maxPosErr so that the position term can't ask for more than bias_amps
    on its own (the position hold drags its reference beyond this)."""
    if g.k0 == 0:
        return 10.0
    return float(min(10.0, max(0.5, round(bias_amps / abs(g.k0) * 2) / 2)))


# ---------------------------------------------------------------------------
# Non-linear simulation (pushes, saturation)
# ---------------------------------------------------------------------------


@dataclass
class SimResult:
    t: np.ndarray
    tilt: np.ndarray        # rad
    tilt_rate: np.ndarray   # rad/s
    wheel: np.ndarray       # rad/s (relative, filtered: as the firmware sees it)
    pos: np.ndarray         # rad
    current: np.ndarray     # A total commanded
    fell: bool

    def summary(self, r: Robot) -> str:
        if self.fell:
            return "FELL"
        th = np.degrees(self.tilt)
        big = np.where((np.abs(th) > 0.5) | (np.abs(self.pos) > 0.5))[0]
        settle = self.t[big[-1]] if len(big) else 0.0
        sat = np.mean(np.abs(self.current) >= 2 * r.max_current - 1e-6) * 100
        return (f"peak tilt {np.abs(th).max():4.1f} deg, peak current {np.abs(self.current).max():.2f} A "
                f"(at the limit {sat:.0f}% of the time), drift {np.abs(self.pos).max() * r.wheel_radius * 100:4.1f} cm, "
                f"settled {settle:.2f} s")


def simulate(r: Robot, g: Gains, kick_dps: float = 0.0, tilt0_deg: float = 0.0, t_end: float = 3.0,
             ideal_current: bool = False) -> SimResult:
    """The robot balancing at rest, given a tilt-rate kick (a push) at t=0."""
    ts = r.ts
    m, rr = r.body_mass, r.wheel_radius
    jt, a_, c0 = _mass_matrix(r, r.com_height, r.body_inertia)
    l = r.com_height
    T = math.radians(tilt0_deg)
    Td = math.radians(kick_dps)
    pa = Td            # absolute wheel rate: wheels not turning relative to the body yet
    wf = 0.0
    pos = 0.0
    x = 0.0
    queue = [0.0] * max(0, r.delay_steps)
    n = int(t_end / ts)
    out = np.full((n, 5), np.nan)
    fell = False
    for k in range(n):
        lqr = float(g.total_current(r, pos, T, wf, Td))
        half = max(-r.max_current, min(r.max_current, lqr / 2))
        queue.append(half)
        u = queue.pop(0)
        wr = pa - Td
        if ideal_current:
            i = u
        else:
            i = (r.cl_p * u + x - r.ke * wr) / (r.resistance + r.cl_p)
            v = r.cl_p * (u - i) + x
            if abs(v) > r.voltage_limit:           # drivers' q-axis voltage limit
                v = math.copysign(r.voltage_limit, v)
                i = (v - r.ke * wr) / r.resistance
            x = max(-r.voltage_limit, min(r.voltage_limit, x + r.cl_i * (u - i) * ts))
        tq = 2 * (r.kt * i - r.friction * math.tanh(wr / 0.5))
        c = c0 * math.cos(T)
        r1 = tq + c0 * math.sin(T) * Td * Td
        r2 = m * G * l * math.sin(T) - tq
        det = jt * a_ - c * c
        pdd = (a_ * r1 - c * r2) / det
        tdd = (jt * r2 - c * r1) / det
        pa += pdd * ts
        Td += tdd * ts
        T += Td * ts
        wf += ((pa - Td) - wf) * ts / r.speed_filter
        pos = max(-g.max_pos_err, min(g.max_pos_err, pos + wf * ts))
        out[k] = (T, Td, wf, pos, 2 * half)
        if abs(T) > math.radians(40):
            fell = True
            break
    t = np.arange(n) * ts
    return SimResult(t, out[:, 0], out[:, 1], out[:, 2], out[:, 3], out[:, 4], fell)


# ---------------------------------------------------------------------------
# Captures
# ---------------------------------------------------------------------------


def load_capture(path: str):
    import pandas as pd
    return pd.read_csv(path, comment="#")


def resample(d, ts: float):
    """Uniformly resampled arrays (SI units) from a capture DataFrame."""
    t = d.t_ms.values / 1000.0
    tu = np.arange(t[0], t[-1], ts)
    f = lambda c: np.interp(tu, t, d[c].values)
    return dict(t=tu, tilt=np.radians(f("tilt")), rate=np.radians(f("tilt_rate")), w=f("phi_dot"),
                i=f("cur_l") + f("cur_r"), sp=np.radians(f("setpoint")), pos=f("pos_err"))


if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        raise SystemExit("usage: python sbr_model.py robot.json   (prints the robot as the model sees it)")
    r = load_robot(sys.argv[1])
    print(describe(r))
    m, l = r.body_mass, r.com_height
    i_axle = r.body_inertia + m * l * l
    print(f"Body {m * 1000:.0f} g; pitch inertia about the axle {i_axle * 1e9:.0f} g mm^2; "
          f"falls over with a time constant of about {1000 * math.sqrt(i_axle / (m * G * l)):.0f} ms "
          f"(the faster this is, the harder it is to balance)")
    print(f"Current loop follows a step in about {1000 * (r.resistance + r.cl_p) / max(r.cl_i, 1e-9):.1f} ms; "
          f"full {r.max_current:g} A is available up to about "
          f"{(r.voltage_limit - r.max_current * r.resistance) / r.ke:.0f} rad/s of wheel speed, "
          f"and none beyond {r.voltage_limit / r.ke:.0f} rad/s")
