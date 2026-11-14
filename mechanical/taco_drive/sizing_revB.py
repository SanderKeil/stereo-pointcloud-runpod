"""Rev B sizing: back-to-back taco discs, magnetically actuated sprung plungers.

Layout (section through one station, z up):

      cage ── spring ── plunger/armature pad  (top)
                         ~~~~ disc 1 ~~~~         (pressed DOWN onto the top pole faces)
      ===== stator: post a | coil | post b =====  (soft-iron posts run through the stator)
                         ~~~~ disc 2 ~~~~         (pressed UP onto the bottom pole faces)
      cage ── spring ── plunger/armature pad  (bottom)

Flux loop per station (one coil): post a up -> top pad (a->b) -> post b down -> bottom pad (b->a) -> post a.
Both pads carry the SAME flux, so the two clamp forces are equal and opposite and close inside the stator.
Both discs sit on one output shaft with hubs at +/-delta, so sheet spring-back cancels in the shaft too.

Plungers are applied by the magnet and only lightly sprung toward the sheet so they ride on it.
A station that is about to clamp then sits ~0.2 mm above its pole, not 4.4 mm, so the magnet only
needs to close a small gap.
"""
import math

import sizing

MU0 = 4e-7 * math.pi

P = dict(
    t=0.15,                 # 17-7PH (1.4568) CH900, ferromagnetic, mm
    pole_w_tan=7.0,         # pole face, tangential, mm
    pole_w_rad=4.0,         # pole face, radial (r 46..50 = flat land under the clamp), mm
    g_closed=0.03,          # residual air gap per face when clamped (finish + flatness), mm
    g_open=0.20,            # sheet height above pole at the next station when the fold sits on its neighbour, mm
    iron_path=90.0,         # mean iron path per loop, mm
    mu_r_iron=800.0,        # nitrided soft iron, operating point
    B_target=1.10,          # T in the pole face when clamped
    coil_window=(4.0, 14.0),  # build x height, mm
    fill=0.55,
    wire_d=0.40,            # mm
    turn_len=52.0,          # mean turn length, mm
    mu=0.30,                # friction, sheet on pole and on pad
    r_c=48.0,               # clamp radius, mm
    V_drive=24.0,
    n_out=10.0,             # rpm design point
    follower_N=0.3,         # plunger follower spring, N
    plunger_mass=0.004,     # kg
)


def calc(p=P):
    a = sizing.calc(p["t"])
    A_face = p["pole_w_tan"] * p["pole_w_rad"] * 1e-6           # m^2, one face
    faces = 4                                                     # 2 per pad, 2 pads in series
    R_iron = p["iron_path"] * 1e-3 / (MU0 * p["mu_r_iron"] * A_face)

    def ni_for(B, g):
        flux = B * A_face
        return flux * (faces * g * 1e-3 / (MU0 * A_face) + R_iron)

    def force_per_pad(NI, g):
        flux = NI / (faces * g * 1e-3 / (MU0 * A_face) + R_iron)
        B = flux / A_face
        return 2 * B**2 * A_face / (2 * MU0), B                  # two faces per pad

    NI_hold = ni_for(p["B_target"], p["g_closed"])
    F_hold, B_hold = force_per_pad(NI_hold, p["g_closed"])
    F_pull_hold, _ = force_per_pad(NI_hold, p["g_open"])
    F_pull_boost, _ = force_per_pad(2 * NI_hold, p["g_open"])

    N_pad = F_hold - a["F_springback"]                            # net normal on the sheet
    T_disc_one_pair = 2 * 2 * p["mu"] * N_pad * p["r_c"] / 1000   # 2 pads, 2 friction faces each
    T_total = 2 * T_disc_one_pair                                 # two discs on one shaft

    w_area = p["coil_window"][0] * p["coil_window"][1] * p["fill"]
    a_wire = math.pi * p["wire_d"] ** 2 / 4
    turns = int(w_area / a_wire)
    I_hold = NI_hold / turns
    R_coil = 0.0172 * turns * p["turn_len"] * 1e-3 / a_wire        # ohm (Cu, 20 C)
    P_station = I_hold**2 * R_coil
    L_coil = turns**2 / (faces * p["g_closed"] * 1e-3 / (MU0 * A_face) + R_iron)
    t_rise = L_coil * 2 * I_hold / p["V_drive"]                    # to boost current

    wave_hz = p["n_out"] * sizing.P["i_target"] / 60
    steps_per_s = wave_hz * 12
    acc = (2 * math.pi * wave_hz) ** 2 * a["delta"] / 2 * 1e-3      # plunger follow, m/s^2
    acc_avail = p["follower_N"] / p["plunger_mass"]
    # hand-off: B lands while A still holds -> fold axis sits ~7 deg past A; then A releases.
    # The axis never moves while both are pinned, so no forced scrub (see notes).
    return dict(delta=a["delta"], sigma=a["sigma"], F_springback=a["F_springback"],
                NI_hold=NI_hold, B_hold=B_hold, F_hold_per_pad=F_hold, F_pull_at_0p2=F_pull_hold,
                F_pull_boost_at_0p2=F_pull_boost, N_pad=N_pad, T_one_pair_per_disc=T_disc_one_pair,
                T_total_one_pair=T_total, turns=turns, I_hold=I_hold, R_coil=R_coil,
                P_station_W=P_station, P_typ_W=3 * P_station, L_mH=L_coil * 1e3, t_rise_ms=t_rise * 1e3,
                wave_hz=wave_hz, steps_per_s=steps_per_s, follow_acc_needed=acc, follow_acc_avail=acc_avail)


if __name__ == "__main__":
    for k, v in calc().items():
        print(f"  {k:22s} {v:10.3f}")
