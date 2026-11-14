"""Parametric CAD for the taco-fold friction wave drive, Rev A (roller wave generator).

Z datum: hub clamp face = underside of the disc = z 0. The disc rim is pressed down by DELTA onto
the anvil ring. Output leaves at the bottom, motor sits on top. All dimensions in mm.

Run:  python3 cad.py      -> out/taco_drive_revA.step, out/stl/*.stl, out/parts.json
"""
import json
import math
import os

import cadquery as cq

import sizing

T_SHEET = 0.15
SZ = sizing.calc(T_SHEET)
DELTA = round(SZ["delta"], 2)          # 4.41 nominal, trimmed with the shim stack

R0 = sizing.P["r0"]                    # disc radius 50
R_HUB = sizing.P["r_h"]                # hub clamp radius 12
R_C = 47.5                             # roller centre radius (roller face spans r 45..50)

# ---- vertical stack -------------------------------------------------------------
Z_ANVIL_TOP = -DELTA
ANVIL_H = 4.0
SHIM_H = 0.5
Z_BASE_TOP = Z_ANVIL_TOP - ANVIL_H - SHIM_H
BASE_H = 13.0
Z_BASE_BOT = Z_BASE_TOP - BASE_H
ROLLER_R = 9.0
ROLLER_W = 5.0
Z_AXLE = Z_ANVIL_TOP + T_SHEET + ROLLER_R          # roller axle height
ARM_H = 6.0
R_PIVOT = 20.0
R_SPRING = 40.0
Z_PLATE_BOT = 14.0
PLATE_H = 4.0
Z_COVER_BOT = 24.0
COVER_H = 6.0
HOUSING_OD, HOUSING_ID = 124.0, 112.0

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")


def ring(od, id_, h, z0):
    return cq.Workplane("XY").workplane(offset=z0).circle(od / 2).circle(id_ / 2).extrude(h)


def disc_flat():
    d = cq.Workplane("XY").circle(R0).extrude(T_SHEET)
    d = d.faces(">Z").workplane().hole(8.0)
    d = d.faces(">Z").workplane().polarArray(8.5, 45, 360, 4).hole(2.7)
    return d


def output_hub():
    # flange Ø24 x 4 (z -4..0) + shaft Ø10 down to z -34, 4x M2.5 tapped (drawn as Ø2.05) PCD 17
    flange = cq.Workplane("XY").workplane(offset=-4).circle(R_HUB).extrude(4)
    flange = flange.faces(">Z").edges().fillet(2.0)        # relief where the sheet leaves the clamp
    pilot = cq.Workplane("XY").circle(3.95).extrude(T_SHEET + 0.8)
    shaft = cq.Workplane("XY").workplane(offset=-34).circle(5).extrude(30)
    hub = flange.union(pilot).union(shaft)
    hub = hub.cut(cq.Workplane("XY").workplane(offset=-34).center(4.2, 0).rect(2, 12).extrude(14))  # D-flat
    holes = cq.Workplane("XY").workplane(offset=-4).polarArray(8.5, 45, 360, 4).circle(1.03).extrude(5)
    return hub.cut(holes)


def clamp_washer():
    w = cq.Workplane("XY").workplane(offset=T_SHEET).circle(11).circle(4).extrude(2.5)
    holes = cq.Workplane("XY").workplane(offset=T_SHEET).polarArray(8.5, 45, 360, 4).circle(1.4).extrude(3)
    return w.cut(holes)


def screws_hub():
    s = None
    for k in range(4):
        a = math.radians(45 + 90 * k)
        head = cq.Workplane("XY").workplane(offset=T_SHEET + 2.5).center(8.5 * math.cos(a), 8.5 * math.sin(a)).circle(2.25).extrude(1.5)
        s = head if s is None else s.union(head)
    return s


def anvil_ring():
    return ring(106, 91, ANVIL_H, Z_ANVIL_TOP - ANVIL_H).faces(">Z").edges().chamfer(0.3)


def shim_ring():
    return ring(106, 91, SHIM_H, Z_BASE_TOP)


def base():
    b = cq.Workplane("XY").workplane(offset=Z_BASE_BOT).circle(HOUSING_OD / 2).extrude(BASE_H)
    # pocket for anvil/shim locating: anvil sits on the flat top; spigot ring locates it radially
    spigot = ring(110, 106.2, 2.0, Z_BASE_TOP)
    b = b.union(spigot)
    # bearing bores 2x 6800 (10x19x5), through bore Ø14
    b = b.cut(cq.Workplane("XY").workplane(offset=Z_BASE_BOT).circle(7).extrude(BASE_H))
    b = b.cut(cq.Workplane("XY").workplane(offset=Z_BASE_TOP - 5).circle(9.5).extrude(5))
    b = b.cut(cq.Workplane("XY").workplane(offset=Z_BASE_BOT).circle(9.5).extrude(5))
    # 6x M3 clearance for housing ring, PCD 118
    b = b.cut(cq.Workplane("XY").workplane(offset=Z_BASE_BOT).polarArray(59, 0, 360, 6).circle(1.7).extrude(BASE_H + 2))
    # lightening / cable pocket on the underside
    b = b.cut(ring(100, 30, 6, Z_BASE_BOT))
    return b


def housing_ring():
    h = Z_COVER_BOT - Z_BASE_TOP
    r = ring(HOUSING_OD, HOUSING_ID, h, Z_BASE_TOP)
    r = r.cut(cq.Workplane("XY").workplane(offset=Z_BASE_TOP).polarArray(59, 0, 360, 6).circle(1.25).extrude(h))
    return r


def top_cover():
    c = cq.Workplane("XY").workplane(offset=Z_COVER_BOT).circle(HOUSING_OD / 2).extrude(COVER_H)
    boss = cq.Workplane("XY").workplane(offset=Z_COVER_BOT - 5).circle(14).extrude(5)
    c = c.union(boss)
    # 2x 688 (8x16x5) bearing bores, Ø11 between
    c = c.cut(cq.Workplane("XY").workplane(offset=Z_COVER_BOT - 5).circle(8).extrude(5))
    c = c.cut(cq.Workplane("XY").workplane(offset=Z_COVER_BOT + COVER_H - 5).circle(8).extrude(5))
    c = c.cut(cq.Workplane("XY").workplane(offset=Z_COVER_BOT).circle(5.5).extrude(COVER_H))
    c = c.cut(cq.Workplane("XY").workplane(offset=Z_COVER_BOT).polarArray(59, 0, 360, 6).circle(1.7).extrude(COVER_H))
    # inspection windows
    for k in range(4):
        a = 45 + 90 * k
        win = (cq.Workplane("XY").workplane(offset=Z_COVER_BOT)
               .transformed(rotate=(0, 0, a)).center(38, 0).slot2D(26, 12).extrude(COVER_H))
        c = c.cut(win)
    # motor standoff holes (NEMA14: 26 mm square, M3)
    c = c.cut(cq.Workplane("XY").workplane(offset=Z_COVER_BOT).rect(26, 26, forConstruction=True).vertices().circle(1.7).extrude(COVER_H))
    return c


def carrier_body():
    """Hub + shaft + spring plate with two pivot yokes. Arms point along +X and -X."""
    z_hub0 = 6.0
    shaft = cq.Workplane("XY").workplane(offset=z_hub0).circle(4).extrude(Z_COVER_BOT + COVER_H + 8 - z_hub0)
    hub = cq.Workplane("XY").workplane(offset=z_hub0).circle(9).extrude(Z_PLATE_BOT + PLATE_H - z_hub0)
    plate = (cq.Workplane("XY").workplane(offset=Z_PLATE_BOT).rect(2 * (R_SPRING + 7), 16).extrude(PLATE_H)
             .edges("|Z").fillet(6))
    body = shaft.union(hub).union(plate)
    for sgn in (1, -1):
        # yoke cheeks either side of the arm, hanging down to the pivot
        for side in (1, -1):
            cheek = (cq.Workplane("XY").workplane(offset=Z_AXLE - 4)
                     .center(sgn * R_PIVOT, side * 6.5).rect(10, 3).extrude(Z_PLATE_BOT - (Z_AXLE - 4)))
            body = body.union(cheek)
        # pivot pin hole (tangential axis = Y)
        pin = (cq.Workplane("XZ").workplane(offset=-10).center(sgn * R_PIVOT, Z_AXLE).circle(1.55).extrude(20))
        body = body.cut(pin)
        # M5 preload screw hole above spring
        body = body.cut(cq.Workplane("XY").workplane(offset=Z_PLATE_BOT).center(sgn * R_SPRING, 0).circle(2.1).extrude(PLATE_H))
    return body


def swing_arm(sgn):
    """Radial arm, pivot at R_PIVOT, spring seat at R_SPRING, stub axle carrying the roller at R_C."""
    x0, x1 = R_PIVOT - 4, R_C - ROLLER_W / 2 - 0.5
    arm = (cq.Workplane("XY").workplane(offset=Z_AXLE - ARM_H / 2)
           .center(sgn * (x0 + x1) / 2, 0).rect(x1 - x0, 9.6).extrude(ARM_H))
    arm = arm.cut(cq.Workplane("XZ").workplane(offset=-6).center(sgn * R_PIVOT, Z_AXLE).circle(1.5).extrude(12))
    seat = cq.Workplane("XY").workplane(offset=Z_AXLE + ARM_H / 2 - 1).center(sgn * R_SPRING, 0).circle(5.2).extrude(1)
    arm = arm.cut(seat)
    axle = (cq.Workplane("YZ").workplane(offset=sgn * x1 if sgn > 0 else -x1)
            .center(0, Z_AXLE).circle(2.5).extrude(sgn * (ROLLER_W + 1.5)))
    return arm.union(axle)


def roller(sgn):
    """685 bearing + hardened crowned sleeve, OD 18 x 5, axis radial (X)."""
    x0 = sgn * R_C - ROLLER_W / 2
    outer = cq.Solid.makeCylinder(ROLLER_R, ROLLER_W, cq.Vector(x0, 0, Z_AXLE), cq.Vector(1, 0, 0))
    inner = cq.Solid.makeCylinder(2.5, ROLLER_W, cq.Vector(x0, 0, Z_AXLE), cq.Vector(1, 0, 0))
    rol = cq.Workplane("XY").add(outer).edges().fillet(1.5)     # crown approximated by edge radii
    return rol.cut(cq.Workplane("XY").add(inner))


def spring(sgn):
    z0, z1 = Z_AXLE + ARM_H / 2 - 1, Z_PLATE_BOT
    return (cq.Workplane("XY").workplane(offset=z0).center(sgn * R_SPRING, 0)
            .circle(5).circle(3.6).extrude(z1 - z0))


def bearing(od, id_, w, z0):
    return ring(od, id_, w, z0)


def motor():
    z0 = Z_COVER_BOT + COVER_H + 14
    flange = cq.Workplane("XY").workplane(offset=z0).rect(35, 35).extrude(28).edges("|Z").fillet(3)
    boss = cq.Workplane("XY").workplane(offset=z0 - 2).circle(11).extrude(2)
    stand = None
    for x in (13, -13):
        for y in (13, -13):
            s = cq.Workplane("XY").workplane(offset=Z_COVER_BOT + COVER_H).center(x, y).circle(2.5).extrude(14)
            stand = s if stand is None else stand.union(s)
    return flange.union(boss), stand


def coupling():
    z0 = Z_COVER_BOT + COVER_H + 1
    return cq.Workplane("XY").workplane(offset=z0).circle(7).circle(4).extrude(10)


PARTS = {}


def add(name, shape, group, color, material, qty=1):
    PARTS[name] = dict(shape=shape, group=group, color=color, material=material, qty=qty)


def build():
    add("output_hub", output_hub(), "output", "#9aa4ad", "7075-T6 or 17-4PH, clamp face lapped")
    add("clamp_washer", clamp_washer(), "output", "#b8c0c8", "Steel, hardened, 2.5 thk")
    add("hub_screws", screws_hub(), "output", "#3a3f44", "M2.5x6 ISO 7380 low head, 12.9", 4)
    add("anvil_ring", anvil_ring(), "fixed", "#c9a24a", "1.2379 / 52100 hardened 58+ HRC, top face ground Ra0.4, flat 0.01")
    add("shim_ring", shim_ring(), "fixed", "#d97a3a", "Shim stock stack, 0.5 nominal (0.05/0.1/0.2/0.5) - sets delta")
    add("base", base(), "fixed", "#6b7680", "6061-T6")
    add("housing_ring", housing_ring(), "fixed", "#8592a0", "6061-T6", 1)
    add("top_cover", top_cover(), "fixed", "#8592a0", "6061-T6")
    add("bearing_out_upper", bearing(19, 10, 5, Z_BASE_TOP - 5), "fixed", "#444", "6800-2RS 10x19x5")
    add("bearing_out_lower", bearing(19, 10, 5, Z_BASE_BOT), "fixed", "#444", "6800-2RS 10x19x5")
    add("bearing_in_lower", bearing(16, 8, 5, Z_COVER_BOT - 5), "fixed", "#444", "688-2RS 8x16x5")
    add("bearing_in_upper", bearing(16, 8, 5, Z_COVER_BOT + COVER_H - 5), "fixed", "#444", "688-2RS 8x16x5")
    add("carrier", carrier_body(), "carrier", "#4a7fb5", "7075-T6 (balanced, opposed arms)")
    for sgn, tag in ((1, "a"), (-1, "b")):
        add(f"swing_arm_{tag}", swing_arm(sgn), "carrier", "#5d95cc", "Steel 1.2312 or 17-4PH, stub axle Ø5 integral")
        add(f"roller_{tag}", roller(sgn), "carrier", "#e0e4e8", "685-2RS + crowned sleeve OD18x5, 100Cr6 60 HRC, crown R~30")
        add(f"spring_{tag}", spring(sgn), "carrier", "#c0392b", "Compression spring OD10, ~30 N/mm, preload ~80 N (M5 set screw)")
    m, st = motor()
    add("motor", m, "fixed", "#2c2f33", "NEMA14 stepper or 28 mm BLDC, >=20 mN*m")
    add("motor_standoffs", st, "fixed", "#8592a0", "M3 standoffs x4, 14 long")
    add("coupling", coupling(), "carrier", "#a0a0a0", "Jaw/bellows coupling 5-8 mm")
    add("disc_flat", disc_flat(), "output", "#d0d6dc", f"1.4310 (301) spring-hard C1700, t={T_SHEET}, photo-etched, edges deburred")


def export():
    os.makedirs(os.path.join(OUT, "stl"), exist_ok=True)
    asm = cq.Assembly(name="taco_drive_revA")
    meta = {}
    for name, p in PARTS.items():
        asm.add(p["shape"], name=name, color=cq.Color(p["color"]))
        if name != "disc_flat":
            cq.exporters.export(p["shape"], os.path.join(OUT, "stl", f"{name}.stl"), tolerance=0.02, angularTolerance=0.15)
        meta[name] = {k: v for k, v in p.items() if k != "shape"}
    asm.save(os.path.join(OUT, "taco_drive_revA.step"))
    for name in ("disc_flat", "output_hub", "anvil_ring", "carrier", "swing_arm_a", "base", "top_cover", "housing_ring", "clamp_washer"):
        cq.exporters.export(PARTS[name]["shape"], os.path.join(OUT, f"{name}.step"))
    params = dict(T_SHEET=T_SHEET, DELTA=DELTA, R0=R0, R_HUB=R_HUB, R_C=R_C, Z_AXLE=Z_AXLE,
                  Z_ANVIL_TOP=Z_ANVIL_TOP, ROLLER_R=ROLLER_R, L=SZ["L"], RATIO=sizing.P["i_target"],
                  sizing={k: float(v) for k, v in SZ.items()})
    json.dump(dict(parts=meta, params=params), open(os.path.join(OUT, "parts.json"), "w"), indent=1)
    print(json.dumps(params, indent=1))


if __name__ == "__main__":
    build()
    export()
