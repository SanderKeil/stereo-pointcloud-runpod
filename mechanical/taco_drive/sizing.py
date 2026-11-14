"""Sizing calculations for the taco-fold friction wave drive (Rev A, roller wave generator).

Model (single-curvature 'taco' bend, from the concept session):
  - Flat spring-steel disc, clamped flat on a hub of radius r_h at height 0.
  - Two diametrically opposed rollers press the rim down by delta onto a flat anvil ring.
  - The sheet bends about one axis only: z = f(x). Flat hub strip |x| < r_h, an S-curve over
    span L to x_f, then a flat land x > x_f lying on the anvil.
  - Between two flat (clamped-guided) ends the elastic S-curve is the cubic
        f(s) = delta * (3 s^2/L^2 - 2 s^3/L^3)
    giving arc-length take-up  dx = 0.6 delta^2 / L   and peak curvature 6 delta / L^2.
  - Rolling contact with no slip at the anvil: output turns  dx / r_c  per radian of the wave,
    same direction as the wave.  Ratio  i = r_c / dx.
"""
import math
import numpy as np

P = dict(
    r0=50.0,        # disc outer radius, mm
    r_h=12.0,       # hub clamp radius, mm (Ø24 clamp)
    x_f=46.0,       # start of flat land (anvil ring inner radius ~45.5), mm
    r_c=48.0,       # roller contact radius, mm
    i_target=140.0, # reduction ratio
    E=193e3,        # 1.4310 (301) spring-hard, MPa
    nu=0.3,
    sigma_lim=500.0,  # assumed fully-reversed bending endurance for polished 1.4310 C1700 foil, MPa (VERIFY)
    mu=0.30,        # sheet-on-anvil friction (steel on hardened steel, dry/clean ~0.15-0.3; friction lining 0.35+)
    F_roller=60.0,  # roller spring force, N
    T_target=1.0,   # output torque target, N*m
    n_out=20.0,     # output speed, rpm
    eta=0.6,        # assumed efficiency (bending hysteresis + roller spin + bearings)
    m_roller=0.006, # roller + axle mass, kg
)


def calc(t, p=P):
    r0, r_h, x_f, r_c = p["r0"], p["r_h"], p["x_f"], p["r_c"]
    L = x_f - r_h
    dx = r_c / p["i_target"]
    delta = math.sqrt(dx * L / 0.6)
    sigma = 3 * p["E"] * t * delta / L**2

    # spring-back force per roller from bending energy (strip-wise cylindrical bending)
    D = p["E"] * t**3 / (12 * (1 - p["nu"] ** 2))
    s = np.linspace(0, L, 2001)
    x = r_h + s
    w = 2 * np.sqrt(np.clip(r0**2 - x**2, 0, None))
    kappa = delta * (6 / L**2 - 12 * s / L**3)
    U_side = np.trapezoid(0.5 * D * kappa**2 * w, s)      # N*mm
    F_sb = 2 * U_side / delta                             # N, per roller (U ~ delta^2)

    N = p["F_roller"] - F_sb                              # normal force at anvil per contact
    T_slip = 2 * p["mu"] * N * r_c / 1000                 # N*m, two contacts, one friction face each

    # sensitivities
    di_ddelta = -2 * p["i_target"] / delta                # i ~ 1/delta^2
    # shear stress at hub edge from output torque, and rough annular shear-buckling estimate
    tau_hub = p["T_target"] * 1e3 / (2 * math.pi * r_h**2 * t)
    b = r0 - r_h
    tau_cr = 5.0 * p["E"] / (12 * (1 - p["nu"] ** 2)) * (t / b) ** 2 * math.pi**2 / 1.0  # k~5, coarse

    n_in = p["n_out"] * p["i_target"]
    w_in = n_in * 2 * math.pi / 60
    F_cf = p["m_roller"] * w_in**2 * r_c / 1000
    cycles_per_hour = 2 * n_in * 60                        # two lobes pass every point per input rev
    T_in = p["T_target"] / (p["i_target"] * p["eta"]) * 1000  # mN*m

    # Hertz line contact roller (R=8 mm, crowned, effective length ~3 mm) on steel backed by steel
    R, Lc, Es = 8.0, 3.0, 200e3 / (2 * (1 - 0.3**2))
    p_hz = math.sqrt(p["F_roller"] / Lc * Es / (math.pi * R))

    return dict(t=t, L=L, dx=dx, delta=delta, sigma=sigma, SF_fatigue=p["sigma_lim"] / sigma,
                F_springback=F_sb, N_anvil=N, T_slip=T_slip, di_per_0p01mm=abs(di_ddelta) * 0.01,
                tau_hub=tau_hub, tau_buckle_est=tau_cr, n_in=n_in, F_centrifugal=F_cf,
                cycles_1e7_hours=1e7 / cycles_per_hour, T_in_mNm=T_in, p_hertz=p_hz)


if __name__ == "__main__":
    for t in (0.10, 0.15, 0.20, 0.30):
        r = calc(t)
        print(f"\n--- t = {t:.2f} mm ---")
        for k, v in r.items():
            print(f"  {k:18s} {v:10.3f}")
