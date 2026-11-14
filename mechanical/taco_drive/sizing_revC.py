"""Rev C sizing: Ø20 piezo walking rotary actuator with flexure-amplified push legs.

Four calipers grip a rigid rotor. Phase A (0/180 deg) and phase B (90/270 deg) alternate:
clamp -> push -> unclamp -> return. Clamps are normally closed by a flexure preload and opened by a
direct (unamplified) piezo stack, so the actuator holds with power off. Push stacks stand along the
shaft axis and drive the caliper tangentially through a flexure bell-crank with ratio k.

Stack figures are typical of multilayer stacks (~0.1% strain at full voltage, ~30 MPa blocking stress);
replace with datasheet values once parts are chosen.
"""
import numpy as np

P = dict(
    r_c=5.2,            # mean clamp radius on the rotor, mm (jaws span r 4.4..6.0)
    mu=0.30,
    preload=100.0,      # clamp preload per caliper, N (must stay below clamp-stack blocking force)
    clamp_block=150.0,  # 2x3x5 mm clamp stack, N
    clamp_stroke=4.0,   # um
    push_s0=15.0,       # 3x3x18 mm push stack free stroke, um
    push_F0=290.0,      # blocking force, N
    push_k_stack=20.0,  # stack stiffness, N/um
    k_nom=2.1,          # bell-crank ratio (output arm 4.0 / input arm 1.9 mm)
    eta=0.75,           # flexure efficiency on the ratio
    f=1000.0,           # step rate per phase pair, Hz
    n_push=2,           # legs pushing at any time
)


def leg(F, k, p=P):
    """Step size (um) of one push leg under tangential load F (N) at amplification k."""
    if k <= 1.0:
        s_free, F_blk = p["push_s0"], p["push_F0"]
    else:
        s_free, F_blk = p["eta"] * k * p["push_s0"], p["eta"] * p["push_F0"] / k
    return max(0.0, s_free * (1 - F / F_blk)), s_free, F_blk


def summary(p=P):
    T_grip = 2 * 2 * p["mu"] * p["preload"] * p["r_c"] / 1000          # 2 calipers x 2 faces
    rows = []
    for T in np.round(np.linspace(0, T_grip, 7), 3):
        F = T / (p["n_push"] * p["r_c"] / 1000)
        out = []
        for k in (1.0, p["k_nom"], 3.0):
            s, _, _ = leg(F, k)
            rpm = s * 1e-3 * p["f"] / p["r_c"] * 60 / (2 * np.pi)
            out.append((s, rpm))
        rows.append((T, F, out))
    k_out = p["push_k_stack"] / p["k_nom"] ** 2 * p["eta"]              # N/um at the caliper
    wind = (T_grip / (p["n_push"] * p["r_c"] / 1000)) / k_out           # um at rim, holding through 2 legs
    return dict(T_grip=T_grip, rows=rows, k_leg_out=k_out, windup_um=wind,
                windup_deg=np.degrees(wind * 1e-3 / p["r_c"]),
                clamp_margin=p["clamp_block"] / p["preload"])


if __name__ == "__main__":
    s = summary()
    print(f"grip torque {s['T_grip']:.2f} N*m, clamp stack margin {s['clamp_margin']:.2f}")
    print(f"amplified leg stiffness {s['k_leg_out']:.1f} N/um -> wind-up {s['windup_um']:.1f} um ({s['windup_deg']:.2f} deg) at full torque on 2 legs")
    print(" T(Nm)  F/leg   direct(um,rpm)    k=2.1(um,rpm)    k=3(um,rpm)")
    for T, F, out in s["rows"]:
        print(f" {T:5.2f} {F:6.1f}  " + "  ".join(f"{a:5.1f} {b:5.1f}rpm" for a, b in out))
