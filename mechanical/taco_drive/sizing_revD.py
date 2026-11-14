"""Rev D speculative sizing: taco-fold disc driven by piezo stations ("piezo comb").

Problem: the rim rises and falls by delta (mm) every wave cycle; piezo strokes are microns.
Concept: each station has
  - a follower plunger that rides on the sheet on a light spring (passive, takes the mm motion),
  - a piezo LOCK that grips the plunger stem to the frame (normally open, um stroke),
  - an amplified piezo PUSHER between the locked stem and the pad (stroke s, ~100-500 um).
Near the leading edge of the fold the sheet is only microns above the anvil land. The station locks
when its gap equals g_lock and the pusher presses the sheet down onto the land, which is what moves
the fold axis forward; it then keeps pressing as the clamp. The station pitch must be no larger than
the axis advance one push produces, so the next station is within reach. Trailing stations unlock
and their followers ride the sheet back up.

Geometry scales with the disc diameter D (all lengths proportional, Ø100 = the Rev B disc), which
keeps the ratio and bending stress constant.
"""
import math

import numpy as np

E, NU, RHO = 193e3, 0.3, 7.85e-9          # MPa, -, t/mm^3
MU = 0.30

STACKS = {   # name: (free stroke um, blocking N, mass g incl. amplifier)
    "3x3x9":  (9.0, 270.0, 1.0),
    "5x5x18": (18.0, 750.0, 4.0),
    "7x7x36": (36.0, 1450.0, 14.0),
}


def geom(D, ratio=140.0):
    s = D / 100.0
    r0, r_h, x_f, r_c, t = 50 * s, 12 * s, 46 * s, 48 * s, 0.15 * s
    L = x_f - r_h
    dx = r_c / ratio
    delta = math.sqrt(dx * L / 0.6)
    sigma = 3 * E * t * delta / L**2
    return dict(D=D, r0=r0, r_h=r_h, x_f=x_f, r_c=r_c, t=t, L=L, delta=delta, sigma=sigma, ratio=ratio)


def gap(phi, g):
    """Sheet height above the land at clamp radius, angle phi from the fold axis."""
    s = g["r_c"] * math.cos(phi) - g["r_h"]
    if s >= g["L"]:
        return 0.0
    q = max(s, 0.0) / g["L"]
    return g["delta"] * (1 - (3 * q * q - 2 * q**3))


def phi_at_gap(target, g):
    lo, hi = 0.0, math.pi / 2
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if gap(mid, g) < target else (lo, mid)
    return lo


def design(D, stack, amp, ratio=140.0, follow_N=None, follow_mass_g=None):
    g = geom(D, ratio)
    s0, F0, m_stack = STACKS[stack]
    stroke = 0.75 * amp * s0 * 1e-3                # mm, amplified free stroke
    blocking = 0.75 * F0 / amp                      # N at the pad
    g_lock = 0.5 * stroke                           # lock with half the stroke left for clamping
    phi_L = math.acos(g["x_f"] / g["r_c"])           # half-width of the flat land
    phi_g = phi_at_gap(g_lock, g)
    adv = phi_g - phi_L                             # axis advance per push, rad
    n_st = 2 * math.ceil(math.pi / adv)             # per side, even so lobes are symmetric
    clamp = 0.5 * blocking                          # force left after closing g_lock
    # spring-back per lobe from the Rev A energy model, scaled (prop. to t^3 * delta^2 / L^3 * width ~ D^2 ... ~ s^2)
    F_sb = 5.94 * (D / 100.0) ** 2
    T_grip = 2 * 2 * 2 * MU * max(clamp - F_sb, 0) * g["r_c"] / 1000     # 2 discs x 2 lobes x 2 faces
    # shear wrinkling limit at the hub (coarse, k~5), two discs
    b = g["r0"] - g["r_h"]
    tau_cr = 5 * math.pi**2 * E / (12 * (1 - NU**2)) * (g["t"] / b) ** 2
    T_wrinkle = 2 * tau_cr * 2 * math.pi * g["r_h"] ** 2 * g["t"] / 1000
    # follower dynamics: rim height swings delta, twice per wave rev; allow 1.5x for the non-sinusoidal profile
    fm = follow_mass_g if follow_mass_g else 1.0 + 0.6 * m_stack
    fN = follow_N if follow_N else 0.5 * (D / 100.0) ** 2 + 0.3
    a = fN / (fm * 1e-3)
    f_wave = math.sqrt(a / (g["delta"] * 1e-3 / 2)) / (4 * math.pi) / 1.5
    rpm_follow = f_wave * 60 / ratio
    # n = 2 bending mode of the clamped-centre disc (lambda^2 ~ 5, rough): the fold IS this mode shape
    Dp = E * g["t"] ** 3 / (12 * (1 - NU**2))
    f_n2 = 5.0 / (2 * math.pi * g["r0"] ** 2) * math.sqrt(Dp / (RHO * g["t"]))
    rpm_res = (f_n2 / 2) * 60 / ratio              # wave speed at which a point is bent at f_n2
    # pusher force needed at the grip torque (4 pushers share it; output turns adv/ratio per push)
    T_use = min(T_grip, T_wrinkle)
    F_push = T_use * 1000 * (adv / ratio) / g_lock / 4 + F_sb
    rpm = min(rpm_follow, 0.5 * rpm_res)
    pushes_hz = n_st / 2 * (rpm * ratio / 60) * 2 / n_st * (n_st / 2) / (n_st / 2)  # per station: 2 per wave rev
    pushes_hz = 2 * rpm * ratio / 60
    return dict(D=D, stack=stack, amp=amp, t=g["t"], delta=g["delta"], sigma=g["sigma"], stroke_um=stroke * 1e3,
                g_lock_um=g_lock * 1e3, land_deg=math.degrees(phi_L), adv_deg=math.degrees(adv), stations=n_st,
                piezos=2 * n_st * 2, clamp_N=clamp, F_push=F_push, push_ok=F_push < blocking * (1 - 0.5),
                T_grip=T_grip, T_wrinkle=T_wrinkle, T=T_use, rpm_follow=rpm_follow, f_n2=f_n2, rpm_res=rpm_res,
                rpm=rpm, station_hz=pushes_hz, P_out=T_use * rpm * 2 * math.pi / 60)


if __name__ == "__main__":
    cols = ["D", "stack", "amp", "t", "delta", "stroke_um", "adv_deg", "stations", "piezos", "clamp_N",
            "T_grip", "T_wrinkle", "T", "rpm_follow", "f_n2", "rpm", "station_hz", "P_out", "push_ok"]
    print(" ".join(f"{c:>9s}" for c in cols))
    for D, st, amps in ((40, "3x3x9", (5, 10, 20)), (60, "5x5x18", (5, 10, 20)), (100, "5x5x18", (5, 10, 20)),
                        (100, "7x7x36", (5, 10)), (150, "7x7x36", (5, 10))):
        for amp in amps:
            r = design(D, st, amp)
            print(" ".join(f"{r[c]:>9.3g}" if isinstance(r[c], (int, float)) and not isinstance(r[c], bool)
                           else f"{str(r[c]):>9s}" for c in cols))
