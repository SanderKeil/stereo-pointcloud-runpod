"""Parametric CAD, Rev B: back-to-back taco discs with magnetically actuated sprung plungers.

z = 0 is the stator mid-plane. Disc 1 sits above and is pressed DOWN onto the top pole faces; disc 2
mirrors it below. Both hubs are on one shaft. 12 stations at 30 deg; each station has two soft-iron
posts through the stator, one coil (around post a), and one plunger on each side.

Run: python3 cad_revB.py -> out_revB/taco_drive_revB.step, out_revB/stl/*.stl, out_revB/assembly_meshes.json
"""
import base64
import json
import math
import os

import cadquery as cq

import sizing
import sizing_revB as SB

T = SB.P["t"]
DELTA = round(sizing.calc(T)["delta"], 2)   # 4.41
R0, R_HUB, R_C = 50.0, 12.0, 48.0
N_ST = 12

H = 16.0                      # stator thickness
PROUD = 0.3                   # pole faces stand proud of the stator faces
ZP = H / 2 + PROUD            # pole-face plane (top); bottom is -ZP
Z_HUB = ZP + DELTA            # disc 1 underside at the hub
POST_T, POST_R = SB.P["pole_w_tan"], SB.P["pole_w_rad"]
COIL_B = SB.P["coil_window"][0]
COIL_H = SB.P["coil_window"][1]
POST_GAP = 0.5
# post a carries the coil; post b sits just outside the coil build. Pair is centred on the station line.
_pitch = POST_T + COIL_B + POST_GAP
A_OFF, B_OFF = -_pitch / 2, _pitch / 2
PAD_L = (B_OFF - A_OFF) + POST_T
PAD_H = 6.0
CAGE_Z0 = 24.0
CAGE_H = 4.0
OD = 124.0

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out_revB")


def ring(od, id_, h, z0):
    return cq.Workplane("XY").workplane(offset=z0).circle(od / 2).circle(id_ / 2).extrude(h)


def box(cx, cy, cz, dx, dy, dz):
    return cq.Workplane("XY").box(dx, dy, dz).translate((cx, cy, cz))


def at_station(shape, k):
    return shape.rotate((0, 0, 0), (0, 0, 1), 360.0 * k / N_ST)


# station-local frame: x = radial, y = tangential
def posts():
    s = None
    for off in (A_OFF, B_OFF):
        p = box(R_C, off, 0, POST_R, POST_T, 2 * ZP)
        s = p if s is None else s.union(p)
    return s


def coil():
    outer = box(R_C, A_OFF, 0, POST_R + 2 * COIL_B, POST_T + 2 * COIL_B, COIL_H)
    inner = box(R_C, A_OFF, 0, POST_R + 0.4, POST_T + 0.4, COIL_H)
    return outer.cut(inner)


def plunger(side=1):
    z_pad0 = ZP + T                                     # clamped position; the viewer moves it
    pad = box(R_C, 0, z_pad0 + PAD_H / 2, POST_R, PAD_L, PAD_H).edges("|X").fillet(0.8)
    stem_len = CAGE_Z0 + CAGE_H + 3 - (z_pad0 + PAD_H)
    stem = cq.Workplane("XY").workplane(offset=z_pad0 + PAD_H).center(R_C, 0).circle(1.5).extrude(stem_len)
    p = pad.union(stem)
    return p if side > 0 else p.mirror("XY")


def follower_spring(side=1):
    z0 = ZP + T + PAD_H
    s = cq.Workplane("XY").workplane(offset=z0).center(R_C, 0).circle(2.6).circle(2.0).extrude(CAGE_Z0 - z0)
    return s if side > 0 else s.mirror("XY")


def stator():
    s = cq.Workplane("XY").workplane(offset=-H / 2).circle(OD / 2).extrude(H)
    s = s.cut(cq.Workplane("XY").workplane(offset=-H / 2).circle(7).extrude(H))
    for z0 in (H / 2 - 5, -H / 2):
        s = s.cut(cq.Workplane("XY").workplane(offset=z0).circle(9.5).extrude(5))
    y_lo = A_OFF - POST_T / 2 - COIL_B          # coil outer edge
    y_hi = B_OFF + POST_T / 2                   # post b outer edge
    pocket = box(R_C, (y_lo + y_hi) / 2, 0, POST_R + 2 * COIL_B + 0.6, (y_hi - y_lo) + 0.6, H)
    slit = box(R_C + 12, (y_lo + y_hi) / 2, 0, 16, 0.8, H)   # pocket to OD: breaks the eddy-current loop
    lead = box(R_C + 9, A_OFF, 0, 10, 2.5, 2.5)
    for k in range(N_ST):
        s = s.cut(at_station(pocket, k)).cut(at_station(slit, k)).cut(at_station(lead, k))
    s = s.cut(cq.Workplane("XY").workplane(offset=-H / 2).polarArray(59, 15, 360, 6).circle(1.7).extrude(H))
    return s


def potting():
    """Epoxy/G10 filler that locks posts and coil into the pocket (drawn as the pocket minus iron/coil)."""
    return None


def hub(side=1):
    fl = cq.Workplane("XY").workplane(offset=Z_HUB - 4).circle(R_HUB).extrude(4)
    fl = fl.faces(">Z").edges().fillet(2.0)
    fl = fl.cut(cq.Workplane("XY").workplane(offset=Z_HUB - 4).circle(5).extrude(4))
    fl = fl.cut(cq.Workplane("XY").workplane(offset=Z_HUB - 4).polarArray(8.5, 45, 360, 4).circle(1.03).extrude(4))
    return fl if side > 0 else fl.mirror("XY")


def hub_shim(side=1):
    s = ring(16, 10, 0.5, Z_HUB - 4.5)
    return s if side > 0 else s.mirror("XY")


def clamp_washer(side=1):
    w = cq.Workplane("XY").workplane(offset=Z_HUB + T).circle(11).circle(5).extrude(2.5)
    w = w.cut(cq.Workplane("XY").workplane(offset=Z_HUB + T).polarArray(8.5, 45, 360, 4).circle(1.4).extrude(2.5))
    return w if side > 0 else w.mirror("XY")


def shaft():
    sh = cq.Workplane("XY").workplane(offset=-42).circle(5).extrude(42 + CAGE_Z0 + CAGE_H + 4)
    sh = sh.cut(cq.Workplane("XY").workplane(offset=-42).center(4.2, 0).rect(2, 12).extrude(14))   # D-flat output
    return sh


def encoder_magnet():
    return cq.Workplane("XY").workplane(offset=CAGE_Z0 + CAGE_H + 4).circle(3).extrude(2.5)


def wall(side=1):
    w = ring(OD, OD - 12, CAGE_Z0 - H / 2, H / 2)
    w = w.cut(cq.Workplane("XY").workplane(offset=H / 2).polarArray(59, 15, 360, 6).circle(1.25).extrude(CAGE_Z0 - H / 2))
    return w if side > 0 else w.mirror("XY")


def cage(side=1):
    c = cq.Workplane("XY").workplane(offset=CAGE_Z0).circle(OD / 2).extrude(CAGE_H)
    c = c.cut(cq.Workplane("XY").workplane(offset=CAGE_Z0).circle(6).extrude(CAGE_H))
    c = c.cut(cq.Workplane("XY").workplane(offset=CAGE_Z0).polarArray(R_C, 0, 360, N_ST).circle(2.5).extrude(CAGE_H))  # bronze bush Ø5/Ø3
    c = c.cut(cq.Workplane("XY").workplane(offset=CAGE_Z0).polarArray(59, 15, 360, 6).circle(1.7).extrude(CAGE_H))
    for k in range(4):
        c = c.cut(cq.Workplane("XY").workplane(offset=CAGE_Z0).transformed(rotate=(0, 0, 45 + 90 * k))
                  .center(28, 0).slot2D(22, 12).extrude(CAGE_H))
    return c if side > 0 else c.mirror("XY")


def bearing(z0):
    return ring(19, 10, 5, z0)


def disc_flat(side=1):
    d = cq.Workplane("XY").workplane(offset=Z_HUB).circle(R0).circle(5.1).extrude(T)
    d = d.cut(cq.Workplane("XY").workplane(offset=Z_HUB).polarArray(8.5, 45, 360, 4).circle(1.35).extrude(T))
    return d if side > 0 else d.mirror("XY")


PARTS = {}


def add(name, shape, group, color, material, qty=1):
    PARTS[name] = dict(shape=shape, group=group, color=color, material=material, qty=qty)


def build():
    add("stator", stator(), "fixed", "#8592a0", "6061-T6, slit at each station (no shorted turn); posts and coils potted in")
    add("posts", posts(), "station", "#4b5560", "Soft iron 1.1003 (or SMC), pole faces nitrocarburised, 2 per station", 24)
    add("coil", coil(), "station", "#b8742a", f"~{SB.calc()['turns']} turns Ø0.40 enamelled Cu on post a", 12)
    add("plunger", plunger(1), "plunger", "#5d95cc", "Armature pad 1.1003 nitrided + Ø3 stem, 4 g", 24)
    add("follower_spring", follower_spring(1), "plunger", "#c0392b", "Compression spring OD5.2, ~0.3 N preload", 24)
    for side, tag in ((1, "top"), (-1, "bot")):
        add(f"hub_{tag}", hub(side), "output", "#9aa4ad", "17-4PH H900, clamp face lapped, R2 relief")
        add(f"hub_shim_{tag}", hub_shim(side), "output", "#d97a3a", "Shim washers: set delta per side (ratio match)")
        add(f"clamp_washer_{tag}", clamp_washer(side), "output", "#b8c0c8", "Hardened steel 2.5")
        add(f"wall_{tag}", wall(side), "housing", "#8592a0", "6061-T6 spacer ring")
        add(f"cage_{tag}", cage(side), "housing", "#6b7680", "6061-T6 cover + 12 bronze guide bushes")
        add(f"disc_{tag}", disc_flat(side), "disc", "#d0d6dc", "17-7PH (1.4568) CH900, 0.15, photo-etched")
    add("shaft", shaft(), "output", "#9aa4ad", "Ø10 output shaft, D-flat bottom, encoder end top")
    add("encoder_magnet", encoder_magnet(), "output", "#7a3fb0", "Diametric magnet Ø6 for AS5047P on the top cover")
    add("bearing_top", bearing(H / 2 - 5), "fixed", "#555b62", "6800-2RS")
    add("bearing_bot", bearing(-H / 2), "fixed", "#555b62", "6800-2RS")


def export():
    os.makedirs(os.path.join(OUT, "stl"), exist_ok=True)
    asm = cq.Assembly(name="taco_drive_revB")
    meshes = {}
    for name, p in PARTS.items():
        if p["group"] in ("station", "plunger"):
            sides = (1, -1) if p["group"] == "plunger" else (1,)
            for k in range(N_ST):
                for s in sides:
                    shp = p["shape"] if s > 0 else p["shape"].mirror("XY")
                    asm.add(at_station(shp, k), name=f"{name}_{'t' if s > 0 else 'b'}{k}", color=cq.Color(p["color"]))
        else:
            asm.add(p["shape"], name=name, color=cq.Color(p["color"]))
        path = os.path.join(OUT, "stl", f"{name}.stl")
        cq.exporters.export(p["shape"], path, tolerance=0.02, angularTolerance=0.2)
        meshes[name] = base64.b64encode(open(path, "rb").read()).decode()
    asm.save(os.path.join(OUT, "taco_drive_revB.step"))
    for name in ("stator", "posts", "coil", "plunger", "hub_top", "cage_top", "wall_top", "shaft", "disc_top"):
        cq.exporters.export(PARTS[name]["shape"], os.path.join(OUT, f"{name}.step"))
    json.dump(meshes, open(os.path.join(OUT, "assembly_meshes.json"), "w"))
    params = dict(T=T, DELTA=DELTA, ZP=ZP, Z_HUB=Z_HUB, PAD_L=PAD_L, A_OFF=A_OFF, B_OFF=B_OFF, H=H)
    json.dump(dict(params=params, parts={k: {kk: vv for kk, vv in v.items() if kk != "shape"} for k, v in PARTS.items()}),
              open(os.path.join(OUT, "parts.json"), "w"), indent=1)
    print(params)


if __name__ == "__main__":
    build()
    export()
