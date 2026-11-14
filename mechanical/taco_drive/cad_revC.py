"""Parametric CAD, Rev C: Ø20 piezo walking rotary actuator with flexure bell-crank push amplifiers.

Frame: z along the shaft, rotor mid-plane z = 0. Each of the 4 stations is built in a local frame
(X radial, Y tangential, Z axial) and rotated to 0/90/180/270 deg. Phase A = stations 0 and 2, phase B = 1 and 3.

Per station:
  carriage   lower jaw/base + preload leaf + clamp stack (2x3x5) + yoke with upper jaw.
             The leaf preloads the upper jaw onto the rotor (normally closed); the clamp stack lifts the
             yoke to open it.
  bell-crank pivot on a ground post; the push stack (3x3x18, axial) lifts the short input arm and the
             long output arm drives the carriage tangentially (ratio ~2.1).
Run: python3 cad_revC.py -> out_revC/*.step, out_revC/assembly_meshes.json
"""
import base64
import json
import os

import cadquery as cq

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out_revC")

OD, WALL = 20.0, 0.5
R_ROTOR, T_ROTOR = 6.0, 2.0
JAW_IN, JAW_OUT = 4.4, 9.0
GAP = 0.05                     # drawn clearance jaw/rotor (in service: contact)

# z levels
Z_BASE_BOT, Z_BASE_TOP = -28.0, -25.0
Z_STACK_TOP = -7.0
Z_PIVOT = -7.0
Z_CAR_BOT = -2.5
Z_JAW_LO = -T_ROTOR / 2 - GAP
Z_JAW_HI = T_ROTOR / 2 + GAP
CLAMP_L = 5.0
Z_YOKE0 = Z_JAW_LO + CLAMP_L
Z_YOKE1 = Z_YOKE0 + 1.0
Z_TOP0, Z_TOP1 = 6.0, 8.5

# bell-crank geometry (local Y)
PIV_Y = -1.9                   # pivot just outside the stack edge (stack spans Y -1.5..1.5)
ARM_IN = 0.0 - PIV_Y           # 1.9 mm to the stack centre
ARM_OUT = Z_CAR_BOT - 0.5 - Z_PIVOT   # 4.0 mm up to the carriage
K_NOM = ARM_OUT / ARM_IN


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0).translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def cyl(r, z0, z1, r_in=0.0):
    w = cq.Workplane("XY").workplane(offset=z0).circle(r)
    if r_in:
        w = w.circle(r_in)
    return w.extrude(z1 - z0)


# ---------------------------------------------------------------- station parts (local frame)
def push_stack():
    return box(5.5, 8.5, -1.5, 1.5, Z_BASE_TOP + 0.3, Z_STACK_TOP - 0.3)


def stack_caps():
    return box(5.5, 8.5, -1.5, 1.5, Z_STACK_TOP - 0.3, Z_STACK_TOP).union(
        box(5.5, 8.5, -1.5, 1.5, Z_BASE_TOP, Z_BASE_TOP + 0.3))


def ground_post():
    return box(5.5, 8.5, -4.0, -2.4, Z_BASE_TOP, Z_PIVOT - 0.6)


def bell_crank():
    hinge = box(5.5, 8.5, PIV_Y - 0.5, PIV_Y - 0.1, Z_PIVOT - 0.6, Z_PIVOT)          # thin neck = pivot
    arm_in = box(5.5, 8.5, PIV_Y - 0.6, 1.5, Z_PIVOT, Z_PIVOT + 1.0)                   # over the stack top
    arm_out = box(5.5, 8.5, PIV_Y - 0.6, PIV_Y + 0.6, Z_PIVOT, Z_CAR_BOT)            # up to the carriage
    return hinge.union(arm_in).union(arm_out)


def carriage_base():
    """Lower jaw + base block; the bell-crank output arm attaches under it."""
    base = box(JAW_IN, JAW_OUT, -2.5, 1.5, Z_CAR_BOT, Z_JAW_LO)
    return base


def preload_leaf():
    return box(6.6, JAW_OUT, -2.5, -2.1, Z_JAW_LO, Z_YOKE0)


def clamp_stack():
    return box(6.6, 8.6, -1.5, 1.5, Z_JAW_LO, Z_YOKE0)


def yoke():
    y = box(JAW_IN, JAW_OUT, -2.5, 1.5, Z_YOKE0, Z_YOKE1)
    jaw = box(JAW_IN, R_ROTOR + 0.1, -1.5, 1.5, Z_JAW_HI, Z_YOKE0)
    return y.union(jaw)


# ---------------------------------------------------------------- global parts
def rotor():
    r = cyl(R_ROTOR, -T_ROTOR / 2, T_ROTOR / 2).union(cyl(2.5, T_ROTOR / 2, 3.0))
    return r.cut(cyl(1.5, -2, 4))


def shaft():
    s = cyl(1.5, Z_BASE_BOT, Z_TOP1 + 5)
    return s.cut(box(1.0, 2.0, -2, 2, Z_TOP1 + 1, Z_TOP1 + 5))       # D-flat on the output end


def base_plate():
    b = cyl(OD / 2, Z_BASE_BOT, Z_BASE_TOP)
    b = b.cut(cyl(3.5, Z_BASE_BOT, Z_BASE_BOT + 3))                  # 683 bearing pocket
    b = b.cut(cyl(1.8, Z_BASE_BOT, Z_BASE_TOP))
    return b


def housing():
    return cyl(OD / 2, Z_BASE_TOP, Z_TOP0, OD / 2 - WALL)


def top_cap():
    c = cyl(OD / 2, Z_TOP0, Z_TOP1)
    c = c.cut(cyl(3.5, Z_TOP1 - 3, Z_TOP1)).cut(cyl(1.8, Z_TOP0, Z_TOP1))
    return c


def bearing(z0):
    return cyl(3.5, z0, z0 + 3, 1.5)


STATION = {
    "push_stack": (push_stack, "push", "#2f3a46", "3x3x18 multilayer stack, ~15 um / ~290 N"),
    "stack_caps": (stack_caps, "push", "#a8b2bc", "Hardened end caps, spherical seat on top (drawn flat)"),
    "ground_post": (ground_post, "fixed", "#7e8a96", "Part of the monolithic stator (wire EDM)"),
    "bell_crank": (bell_crank, "crank", "#4a7fb5", f"Flexure bell-crank, ratio {K_NOM:.1f} (monolithic with stator)"),
    "carriage_base": (carriage_base, "carriage", "#5d95cc", "Lower jaw, ceramic or carbide pad on rotor face"),
    "preload_leaf": (preload_leaf, "carriage", "#c0392b", "Preload flexure, ~100 N clamp"),
    "clamp_stack": (clamp_stack, "carriage", "#2f3a46", "2x3x5 multilayer stack, ~4 um / ~150 N"),
    "yoke": (yoke, "yoke", "#5d95cc", "Yoke + upper jaw"),
}
GLOBAL = {
    "rotor": (rotor, "output", "#c9a24a", "Hardened, lapped both faces, Ø12 x 2"),
    "shaft": (shaft, "output", "#9aa4ad", "Ø3, D-flat output"),
    "base_plate": (base_plate, "fixed", "#7e8a96", "Stator base (monolithic with posts)"),
    "housing": (housing, "housing", "#8592a0", "Ø20 tube, 0.5 wall"),
    "top_cap": (top_cap, "housing", "#8592a0", "Bearing cap"),
    "bearing_top": (lambda: bearing(Z_TOP1 - 3), "housing", "#555b62", "683ZZ 3x7x3"),
    "bearing_bot": (lambda: bearing(Z_BASE_BOT), "fixed", "#555b62", "683ZZ 3x7x3"),
}


def rot(shape, k):
    return shape.rotate((0, 0, 0), (0, 0, 1), 90.0 * k)


def build():
    parts = {}
    for name, (fn, grp, col, mat) in {**STATION, **GLOBAL}.items():
        parts[name] = dict(shape=fn(), group=grp, color=col, material=mat, station=name in STATION)
    return parts


def export(parts):
    os.makedirs(OUT, exist_ok=True)
    asm = cq.Assembly(name="piezo_walker_revC")
    meshes = {}
    for name, p in parts.items():
        if p["station"]:
            for k in range(4):
                asm.add(rot(p["shape"], k), name=f"{name}_{k}", color=cq.Color(p["color"]))
        else:
            asm.add(p["shape"], name=name, color=cq.Color(p["color"]))
        path = os.path.join(OUT, f"{name}.stl")
        cq.exporters.export(p["shape"], path, tolerance=0.005, angularTolerance=0.15)
        meshes[name] = base64.b64encode(open(path, "rb").read()).decode()
        os.remove(path)
    asm.save(os.path.join(OUT, "piezo_walker_revC.step"))
    json.dump(meshes, open(os.path.join(OUT, "assembly_meshes.json"), "w"))
    meta = dict(K_NOM=K_NOM, PIV_Y=PIV_Y, Z_PIVOT=Z_PIVOT, Z_YOKE0=Z_YOKE0, Z_STACK_TOP=Z_STACK_TOP,
                Z_BASE_TOP=Z_BASE_TOP, parts={k: {kk: vv for kk, vv in v.items() if kk != "shape"} for k, v in parts.items()})
    json.dump(meta, open(os.path.join(OUT, "parts.json"), "w"), indent=1)
    print({k: v for k, v in meta.items() if k != "parts"})


if __name__ == "__main__":
    export(build())
