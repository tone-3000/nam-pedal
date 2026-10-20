"""
Off-board wiring as Blender bevelled curves, in the world frame of the finished (face-up) pedal.
14 wires: input jack T/TN/S, output jack T/S, DC +/-, footswitch x2, rotary T/1/2/3/4.

Board pads come from nam-pedal-diy.kicad_pcb; KiCad (kx, ky) -> world (119.62 - kx, 106.5 - ky) (fit on the LED holes).
Board: main body x -61..-2, y -12..45.6; LED tongue x -2..44, y -8..6.5; underside (component side) at z 24.9.
Hardware (enclosure.py, face-up): jacks on the +Y wall at z 19 (out x 15.5, DC 35.5, in 55.5); audio tabs: tip at
x-6.9 / sleeve at x+6.9, both (y 46.2, z 24.1); ground tab (x, 46.7, 9.3). DC lugs (35.5 +-2.5, 40.2, 21).
Footswitch lugs: columns x -55/-49.4/-43.8, rows y -44.4/-38.8/-33.2, z 12..17.5 (plates normal to Y).
Rotary outer pins r 10.8 about (20.13, -27.67), tips at z 23.9.
"""
import math
import bpy
from mathutils import Vector

Z_PAD = 24.9
BARE = 3.0                        # tinned length at each end, mm
R_INS, R_BARE = 0.75, 0.42

def pad(kx, ky):
    return (119.62 - kx, 106.5 - ky, Z_PAD)

X_OUT, X_DC, X_IN = 15.5, 35.5, 55.5
Y_TAB, Z_TAB, Y_GND, Z_GND = 46.2, 24.1, 46.7, 9.9
Z_HI = 23.3                       # lane just under the board edge
RX, RY = 20.13, -27.67
def rpin(a):
    return (RX + 10.8 * math.cos(math.radians(a)), RY + 10.8 * math.sin(math.radians(a)), 23.9)

def audio_hi(p0, x_exit, y_lane, x_tab, side):
    """tip (side -1) / tip-normal (side +1): along the board underside, off the edge, +X under the lane, up onto the
    outer face of the tilted tab (tabs lean +-50 deg about Y; normal = (side*0.766, 0, 0.643))."""
    x0, y0, _ = p0
    ox, oz = side * 0.46, 0.39
    return [p0, (x0, y0, Z_HI), (x_exit, y0, Z_HI), (x_exit, y_lane, Z_HI), (x_tab + ox, y_lane, Z_HI),
            (x_tab + ox, Y_TAB - 5.0, Z_TAB + oz), (x_tab + ox, Y_TAB + 0.6, Z_TAB + oz)]

def audio_gnd(p0, x_exit, z_lane, x_jack):
    """sleeve/ground: off the edge, drop to a low lane, run +X, in under the jack to the ground tab."""
    x0, y0, _ = p0
    return [p0, (x0, y0, Z_HI), (x_exit, y0, Z_HI), (x_exit + 3.0, y0, z_lane), (x_jack, y0, z_lane),
            (x_jack, Y_GND - 6.0, Z_GND + 1.0), (x_jack, Y_GND + 0.6, Z_GND)]

WHITE, YELLOW, BLACK, RED, GREEN = (0.85, 0.85, 0.80), (0.90, 0.72, 0.05), (0.02, 0.02, 0.02), (0.75, 0.03, 0.02), (0.05, 0.42, 0.12)
WIRES = [
    # input jack J2 (3T): pads T (y 24.95), TN (27.95), S (30.95) at x -4
    dict(name="in_T",  color=WHITE,  pts=audio_hi(pad(123.65, 81.55), 1.8, 36.0, X_IN - 6.9, -1)),
    dict(name="in_TN", color=YELLOW, pts=audio_hi(pad(123.65, 78.55), 3.6, 34.5, X_IN + 6.9, +1)),
    dict(name="in_S",  color=BLACK,  pts=audio_gnd(pad(123.65, 75.55), 6.0, 12.4, X_IN)),
    # output jack J3 (2T): pads T (y 11.05), S (14.05)
    dict(name="out_T", color=WHITE,  pts=audio_hi(pad(123.65, 95.45), 0.0, 37.5, X_OUT - 6.9, -1)),
    dict(name="out_S", color=BLACK,  pts=audio_gnd(pad(123.65, 92.45), 6.0, 10.2, X_OUT)),
    # DC jack J8: pads at (-34.8, 43.6) and (-37.8, 43.6); run high along the jack wall, down to the two upper lugs
    dict(name="dc_p", color=RED,   pts=[pad(154.4, 62.9), (-34.78, 43.6, 28.0), (-34.78, 47.0, 28.0), (X_DC - 2.5, 47.0, 28.0), (X_DC - 2.5, 41.6, 22.6), (X_DC - 2.5, 39.8, 21.6)]),
    dict(name="dc_n", color=BLACK, pts=[pad(157.4, 62.9), (-37.78, 43.6, 29.6), (-37.78, 48.6, 29.6), (X_DC + 2.5, 48.6, 29.6), (X_DC + 2.5, 41.6, 22.6), (X_DC + 2.5, 39.8, 21.6)]),
    # footswitch SW2: pads (-33.9, -9.95) and (-30.9, -9.95) -> one pole (column x -43.8), rows y -33.2 (throw) and -38.8 (common)
    dict(name="fs_a", color=GREEN, pts=[pad(153.5, 116.45), (-33.88, -9.95, Z_HI), (-33.88, -14.0, Z_HI), (-33.88, -17.0, 14.0), (-43.8, -23.0, 14.0), (-43.8, -32.4, 14.0)]),
    dict(name="fs_b", color=GREEN, pts=[pad(150.5, 116.45), (-30.88, -9.95, Z_HI), (-30.88, -14.0, Z_HI), (-30.88, -17.0, 16.5), (-39.0, -21.0, 16.5), (-39.0, -36.0, 16.5), (-43.0, -38.3, 16.2)]),
]
# rotary SW1 pads, as silkscreened on the arm: 4, 3, 2, 1 (kx 114.4 .. 105.4) and T (kx 102.4, net GND).
# T (the pole common) goes to an inner-ring pin (r 6.2, 30 deg); 1..4 go to four adjacent outer-ring pins of that pole
# (165 .. 75 deg). Targets increase in x with the pads, so no wire crosses another; each runs on its own level.
ROT_COLORS = {"T": (0.45, 0.45, 0.47), 1: (0.95, 0.40, 0.02), 2: YELLOW, 3: GREEN, 4: (0.05, 0.20, 0.70)}
def rpin_in(a):
    return (RX + 6.2 * math.cos(math.radians(a)), RY + 6.2 * math.sin(math.radians(a)), 23.9)
for k, (kx, a, zk) in zip((4, 3, 2, 1), zip((114.4, 111.4, 108.4, 105.4), (165, 135, 105, 75), (22.9, 21.4, 19.9, 18.4))):
    p0 = pad(kx, 112.95); px, py, pz = rpin(a)
    WIRES.append(dict(name=f"rot_{k}", color=ROT_COLORS[k],
                      pts=[p0, (p0[0], p0[1], zk), (p0[0], -12.0, zk), (px, py + 4.0, zk), (px, py + 0.4, pz - 0.6)]))
p0 = pad(102.4, 112.95); px, py, pz = rpin_in(30)
WIRES.append(dict(name="rot_0", color=ROT_COLORS["T"],
                  pts=[p0, (p0[0], p0[1], 16.9), (p0[0], -12.0, 16.9), (px, -16.0, 16.9), (px, py + 3.0, 16.9), (px, py + 0.4, pz - 0.6)]))


def _shorten(pts, d_start, d_end):
    P = [Vector(p) for p in pts]
    a, b = P[0], P[1]
    P[0] = a + (b - a).normalized() * d_start
    y, z = P[-2], P[-1]
    P[-1] = z - (z - y).normalized() * d_end
    return P


def _curve(name, points, radius, mat, smooth=True):
    cu = bpy.data.curves.new(name, 'CURVE'); cu.dimensions = '3D'
    cu.bevel_depth = radius; cu.bevel_resolution = 5; cu.resolution_u = 12
    cu.use_fill_caps = True
    cu.bevel_factor_mapping_start = cu.bevel_factor_mapping_end = 'SPLINE'
    sp = cu.splines.new('NURBS' if smooth else 'POLY')
    sp.points.add(len(points) - 1)
    for i, p in enumerate(points):
        sp.points[i].co = (p[0], p[1], p[2], 1.0)
    if smooth:
        sp.order_u = 3; sp.use_endpoint_u = True
    ob = bpy.data.objects.new(name, cu)
    bpy.context.scene.collection.objects.link(ob)
    cu.materials.append(mat)
    return ob


def build_wires(mat_fn, tinned):
    """mat_fn(name, color) -> insulation material. Returns [{name, ins, ends:[a, b]}] per wire."""
    out = []
    for w in WIRES:
        mat = mat_fn("wire_" + w["name"], w["color"])
        P = [Vector(p) for p in w["pts"]]
        ins_pts = _shorten(w["pts"], BARE, BARE)
        ins = _curve("wire_" + w["name"], ins_pts, R_INS, mat)
        d0 = (P[1] - P[0]).normalized(); d1 = (P[-1] - P[-2]).normalized()
        a = _curve("bare_" + w["name"] + "_a", [P[0], ins_pts[0] + d0 * 0.4], R_BARE, tinned, smooth=False)
        b = _curve("bare_" + w["name"] + "_b", [ins_pts[-1] - d1 * 0.4, P[-1]], R_BARE, tinned, smooth=False)
        out.append(dict(name=w["name"], ins=ins, ends=[a, b]))
    return out
