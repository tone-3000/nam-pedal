"""
TONE3000 DIY Pedal enclosure + hardware, parametric (CadQuery).
Units mm. World: X right, Y toward the jack wall, Z up. Z=0 = underside of back plate.
Face coordinates (Xf, Yf) are the designer's drill template: 141 x 117, origin top-left, Y down.
"""
import math, sys, json
import cadquery as cq
from cadquery import Location, Vector

OUT = sys.argv[1] if len(sys.argv) > 1 else "enclosure.glb"

# ---------------- dimensions ----------------
FACE_W, FACE_H = 141.0, 117.0          # top face (template)
BASE_W, BASE_H = 145.0, 121.0          # open end / back plate
LID_T = 2.0
TUB_H = 37.5
Z_FACE = LID_T + TUB_H                 # 39.5
WALL = 2.0
FACE_T = 2.0
Z_FACE_IN = Z_FACE - FACE_T            # 37.5
Z_PCB = 26.5                           # control-side surface of the PCB

def W(xf, yf):
    return (xf - FACE_W / 2, FACE_H / 2 - yf)

POTS = [(20.01, 19.41), (37.73, 19.41), (55.27, 19.41), (20.01, 46.13), (37.73, 46.13), (55.27, 46.13)]
PWR_LED = (31.69, 68.41)
ARC_LEDS = [(69.15, 64.21), (82.82, 56.62), (98.46, 56.66), (111.92, 64.22)]
ROTARY = (90.63, 86.17)
FOOTSW = (21.11, 97.29)
# jacks: in the wall under the T3K wordmark (face Yf = 0 wall), clear of the PCB (which ends at Xf ~ 68 on that side)
JACK_OUT_X, JACK_DC_X, JACK_IN_X = 86.0, 106.0, 126.0
JACK_Z = 19.0
# wall at JACK_Z (tub tapers 2 mm over its height)
WALL_Y_OUT = BASE_H / 2 - (BASE_H - FACE_H) / 2 * (JACK_Z - LID_T) / TUB_H
WALL_Y_IN = WALL_Y_OUT - WALL
DRAFT_DEG = math.degrees(math.atan2((BASE_H - FACE_H) / 2, TUB_H))   # side-wall draft (~3 deg)


# ---------------- helpers ----------------
def rrect_sketch(w, h, r):
    return cq.Sketch().rect(w, h).vertices().fillet(r)

def loft_shell(w0, h0, r0, z0, w1, h1, r1, z1):
    return (cq.Workplane("XY")
            .placeSketch(rrect_sketch(w0, h0, r0).moved(Location(Vector(0, 0, z0))),
                         rrect_sketch(w1, h1, r1).moved(Location(Vector(0, 0, z1))))
            .loft())

def cyl(r, h, x, y, z, axis=(0, 0, 1)):
    return cq.Workplane("XY").add(cq.Solid.makeCylinder(r, h, Vector(x, y, z), Vector(*axis)))

def cone(r1, r2, h, x, y, z, axis=(0, 0, 1)):
    return cq.Workplane("XY").add(cq.Solid.makeCone(r1, r2, h, Vector(x, y, z), Vector(*axis)))

def hex_nut(af, h, x, y, z, hole_r, axis="z"):
    if axis == "z":
        s = cq.Workplane("XY").polygon(6, af / math.cos(math.pi / 6)).extrude(h).translate((x, y, z))
        return s.cut(cyl(hole_r, h + 1, x, y, z - 0.5))
    # along +Y, starting at y
    s = cq.Workplane("XZ").polygon(6, af / math.cos(math.pi / 6)).extrude(-h).translate((x, y, z))
    return s.cut(cyl(hole_r, h + 1, x, y - 0.5, z, axis=(0, 1, 0)))

def box(sx, sy, sz, x, y, z):
    """box centred at x,y, bottom at z"""
    return cq.Workplane("XY").box(sx, sy, sz, centered=(True, True, False)).translate((x, y, z))

def boxc(sx, sy, sz, x, y, z):
    """box centred at x,y,z"""
    return cq.Workplane("XY").box(sx, sy, sz).translate((x, y, z))

def U(a, b):
    return b if a is None else a.union(b)

# ---------------- tub ----------------
tub = loft_shell(BASE_W, BASE_H, 4.0, LID_T, FACE_W, FACE_H, 3.0, Z_FACE)
tub = tub.faces(">Z").edges().fillet(1.2)
inner = loft_shell(BASE_W - 2 * WALL, BASE_H - 2 * WALL, 2.5, LID_T - 1,
                   FACE_W - 2 * WALL, FACE_H - 2 * WALL, 1.5, Z_FACE_IN)
tub = tub.cut(inner)
BOSS = [(sx * 65.0, sy * 53.0) for sx in (-1, 1) for sy in (-1, 1)]
for (bx, by) in BOSS:
    tub = tub.union(cyl(3.5, 12, bx, by, LID_T)).cut(cyl(1.3, 14, bx, by, LID_T - 1))

def face_hole(solid, xf, yf, d):
    x, y = W(xf, yf)
    return solid.cut(cyl(d / 2, FACE_T + 4, x, y, Z_FACE_IN - 2))

for p in POTS:
    tub = face_hole(tub, *p, 7.2)
tub = face_hole(tub, *PWR_LED, 3.2)
for p in ARC_LEDS:
    tub = face_hole(tub, *p, 4.8)
tub = face_hole(tub, *ROTARY, 9.6)
tub = face_hole(tub, *FOOTSW, 12.2)

def wall_hole(solid, xf, z, d):
    x = xf - FACE_W / 2
    return solid.cut(cyl(d / 2, 12, x, WALL_Y_IN - 4, z, axis=(0, 1, 0)))

tub = wall_hole(tub, JACK_OUT_X, JACK_Z, 10.0)
tub = wall_hole(tub, JACK_IN_X, JACK_Z, 10.0)
tub = wall_hole(tub, JACK_DC_X, JACK_Z, 10.5)

# ---------------- back plate ----------------
lid = cq.Workplane("XY").placeSketch(rrect_sketch(BASE_W, BASE_H, 4.0)).extrude(LID_T)
lid = lid.faces("<Z").edges().fillet(0.8)
for (bx, by) in BOSS:
    lid = lid.cut(cyl(1.6, LID_T + 1, bx, by, -0.5)).cut(cone(3.0, 1.6, 1.4, bx, by, 0))
screws = None
for (bx, by) in BOSS:
    s = cone(2.9, 1.5, 1.4, bx, by, 0.02)
    s = s.cut(box(0.5, 4.5, 1.0, bx, by, 0.0)).cut(box(4.5, 0.5, 1.0, bx, by, 0.0))
    screws = U(screws, s)

# ---------------- pots: nut, shaft, Davies 1900H knob ----------------
# 16 mm Alpha pot: bushing M7 x 7 mm above the flange; shaft 6 mm D, 15 mm from flange
# flange sits on the inside of the face (Z_FACE_IN); shaft top at Z_FACE_IN + 15
pot_nuts = None; shafts = None; knobs = None; knob_marks = None
for (xf, yf) in POTS:
    x, y = W(xf, yf)
    nut = U(cyl(5.2, 0.5, x, y, Z_FACE), hex_nut(10.0, 2.3, x, y, Z_FACE + 0.5, 3.6))
    pot_nuts = U(pot_nuts, nut)
    sh = cyl(3.0, Z_FACE_IN + 15.0 - Z_FACE, x, y, Z_FACE)
    sh = sh.cut(box(6.2, 1.6, 8.0, x, y + 3.0 + 0.8 - 0.1, Z_FACE_IN + 15.0 - 7.0))   # D flat toward +Y
    shafts = U(shafts, sh)
    # Davies 1900H clone (Tayda A-4559): Ø12.7 x 15.9 mm, fluted (8 scallops), flat top, white line on top
    z0 = Z_FACE + 3.0                                  # sits just above the nut
    KH = 15.9
    k = cone(6.35, 6.1, KH, 0, 0, z0)
    for i in range(8):
        a8 = 2 * math.pi * i / 8 + math.pi / 8
        k = k.cut(cyl(1.9, KH + 2, 7.6 * math.cos(a8), 7.6 * math.sin(a8), z0 - 1))
    k = k.faces(">Z").edges().fillet(0.6)
    k = k.cut(cyl(3.1, 12.0, 0, 0, z0 - 0.5))          # bore
    KNOB_ROT = 0.0     # renders show the knobs at noon; pointer side is +Y (toward the jack wall) before rotation
    k = k.rotate((0, 0, 0), (0, 0, 1), KNOB_ROT).translate((x, y, 0))
    knobs = U(knobs, k)
    mk = boxc(0.7, 4.6, 0.4, 0, 3.2, z0 + KH - 0.05)
    mk = mk.rotate((0, 0, 0), (0, 0, 1), KNOB_ROT).translate((x, y, 0))
    knob_marks = U(knob_marks, mk)

# ---------------- rotary switch: Alpha SR2511F 3P4T (Tayda A-1639) ----------------
rx, ry = W(*ROTARY)
# M9 bushing through the face, hex nut, 6 mm knurled shaft 17 mm long
rot_bush = cyl(4.75, 9.0, rx, ry, Z_FACE_IN - 0.5)
rot_nut = U(cyl(6.0, 0.5, rx, ry, Z_FACE), hex_nut(14.0, 2.4, rx, ry, Z_FACE + 0.5, 4.8))
rot_shaft = cyl(3.0, 11.0, rx, ry, Z_FACE + 1.0)   # 6 mm shaft, ends inside the knob
# stamped steel frame: round plate with raised boss under the face, four legs down to the wafer
PL_Z = Z_FACE_IN - 1.0
rot_frame = cyl(11.0, 1.0, rx, ry, PL_Z)
rot_frame = rot_frame.union(cyl(7.0, 2.5, rx, ry, PL_Z - 2.5))
WZ = Z_FACE_IN - 9.6                                   # wafer bottom
for a in (45, 135, 225, 315):
    pa = math.radians(a)
    lx, ly = rx + 10.0 * math.cos(pa), ry + 10.0 * math.sin(pa)
    leg = (cq.Workplane("XY").rect(3.0, 1.0).extrude(PL_Z - (WZ + 1.6)).translate((0, 0, WZ + 1.6))
           .rotate((0, 0, 0), (0, 0, 1), a + 90).translate((lx, ly, 0)))
    rot_frame = rot_frame.union(leg)
    rot_frame = rot_frame.union(cyl(1.0, 0.4, lx, ly, WZ - 0.4))           # rivet head under the wafer
# orange phenolic PCB wafer, Ø26 with four small flats, 1.6 thick
wafer = cq.Workplane("XY").circle(13.0).extrude(1.6).translate((rx, ry, WZ))
wafer = wafer.intersect(cq.Workplane("XY").placeSketch(rrect_sketch(25.0, 25.0, 9.0)).extrude(3).translate((rx, ry, WZ - 0.5)))
wafer = wafer.cut(cyl(2.0, 3, rx, ry, WZ - 0.5))
rot_hub = cyl(4.0, 1.0, rx, ry, WZ - 1.0).cut(box(6.0, 0.8, 1.2, rx, ry, WZ - 1.1))
# PC pins: two rings of 12 flat tabs (outer r 10.8, inner r 6.2), 4 mm down from the wafer
rot_lugs = None
for rr, off in ((10.8, 15), (6.2, 0)):
    for i in range(12):
        a = math.radians(360 * i / 12 + off)
        px, py = rx + rr * math.cos(a), ry + rr * math.sin(a)
        rot_lugs = U(rot_lugs, boxc(1.5, 0.5, 4.0, px, py, WZ - 2.0).rotate((px, py, 0), (px, py, 1), math.degrees(a)))
rot_body = rot_frame.union(rot_hub)
# EQP-1A knob (Tayda A-7053 datasheet): 38 long x 13 wide x 16 tall bar, flat top, 45 deg slope down to a
# 4.8 mm nose; shaft centred 19 mm from the back; Ø21 x 3.6 round hub flush with the bottom; white inlay on the slope
tx, ty = W(*ARC_LEDS[1])
ang = math.degrees(math.atan2(ty - ry, tx - rx))
kz = Z_FACE + 3.4
KL, KW, KH2, NOSE, SLOPE_X0 = 38.0, 13.0, 16.0, 4.8, 7.7
rk = cq.Workplane("XY").placeSketch(rrect_sketch(KL, KW, 2.5)).extrude(KH2)
prof = (cq.Workplane("XZ").polyline([(-KL / 2, 0), (KL / 2, 0), (KL / 2, NOSE), (SLOPE_X0, KH2), (-KL / 2, KH2)]).close()
        .extrude(KW, both=True))
rk = rk.intersect(prof)
try:
    rk = rk.edges().chamfer(0.5)
except Exception:
    pass
hub = cyl(10.5, 3.6, 0, 0, 0)
try:
    hub = hub.faces(">Z").edges().fillet(1.4)
except Exception:
    pass
rk = rk.union(hub)
rk = rk.cut(cyl(3.2, 12.0, 0, 0, -0.5))
rot_knob = rk.rotate((0, 0, 0), (0, 0, 1), ang).translate((rx, ry, kz))
# white inlay line down the full slope, continuing 1 mm onto the flat top
P0, P1 = (SLOPE_X0, KH2), (KL / 2, NOSE)
dxs, dzs = P1[0] - P0[0], P1[1] - P0[1]
L = math.hypot(dxs, dzs); dxs, dzs = dxs / L, dzs / L
nx, nz = -dzs, dxs                       # outward normal of the slope (up/forward)
def _off(p, t, u):                       # point p moved t along the slope and u along the normal
    return (p[0] + dxs * t + nx * u, p[1] + dzs * t + nz * u)
slope_line = (cq.Workplane("XZ").polyline([_off(P0, -0.3, 0.06), _off(P1, 0.3, 0.06), _off(P1, 0.3, -0.4), _off(P0, -0.3, -0.4)]).close()
              .extrude(0.5, both=True))
flat_line = boxc(1.3, 1.0, 0.46, SLOPE_X0 - 0.55, 0, KH2 - 0.17)
rot_mark = (slope_line.union(flat_line).rotate((0, 0, 0), (0, 0, 1), ang).translate((rx, ry, kz)))

# ---------------- footswitch (3PDT) ----------------
fx, fy = W(*FOOTSW)
fs_body = box(18.0, 18.0, 17.0, fx, fy, Z_FACE_IN - 17.0 - 3.0)
fs_body = fs_body.union(cyl(9.0, 3.0, fx, fy, Z_FACE_IN - 3.0))      # round collar under the bushing
fs_lugs = None
for i in (-1, 0, 1):
    for j in (-1, 0, 1):
        fs_lugs = U(fs_lugs, box(2.4, 0.8, 5.5, fx + i * 5.6, fy + j * 5.6, Z_FACE_IN - 20.0 - 5.5))
fs_bushing = cyl(6.0, FACE_T + 9.0, fx, fy, Z_FACE_IN - 1.0)
fs_washer = cyl(8.5, 1.0, fx, fy, Z_FACE).cut(cyl(6.1, 2, fx, fy, Z_FACE - 0.1))
fs_nut = hex_nut(14.0, 3.0, fx, fy, Z_FACE + 1.0, 6.0)
fs_cap = cyl(5.0, 8.0, fx, fy, Z_FACE + 4.0).faces(">Z").edges().fillet(1.6)

# ---------------- LEDs (3 mm flat-top) + bezels ----------------
# LED stands 10 mm off the board on its legs: flange at Z_PCB+10, flat top 4.3 above the flange
bezels = None; leds = None; leds_off = None; led_legs = None
def add_led(xf, yf, bezel, lit=True, legs_axis="y"):
    global bezels, leds, leds_off, led_legs
    x, y = W(xf, yf)
    if bezel:
        b = U(cyl(3.4, 1.0, x, y, Z_FACE), cyl(2.3, 1.6, x, y, Z_FACE + 1.0))
        b = b.union(cyl(2.35, FACE_T + 2.0, x, y, Z_FACE_IN - 2.0))
        b = b.cut(cyl(1.6, 8, x, y, Z_FACE_IN - 3))
        bezels = U(bezels, b)
    zf = Z_PCB + 10.0
    body = cyl(1.9, 1.0, x, y, zf)
    body = body.union(cyl(1.5, 4.3, x, y, zf + 1.0).faces(">Z").edges().fillet(0.5))
    if lit:
        leds = U(leds, body)
    else:
        leds_off = U(leds_off, body)
    for d in (-1.27, 1.27):
        lx, ly = (x + d, y) if legs_axis == "x" else (x, y + d)
        led_legs = U(led_legs, cyl(0.25, 10.0, lx, ly, Z_PCB))

for i, p in enumerate(ARC_LEDS):
    add_led(*p, True, lit=(i == 1))
add_led(*PWR_LED, False, lit=True, legs_axis="x")

# ---------------- jacks ----------------
jack_nuts = None; jack_bush = None; jack_frames = None; jack_ins = None; jack_lugs = None; dc_body = None; dc_ins = None

def audio_jack(xf, z):
    """1/4" stereo open-frame chassis socket, 4 terminals (Tayda A-5057): phenolic disc stack, 3/8" bushing,
    two long spring prongs (sleeve, tip) and two short lugs, all rising from the disc into the enclosure."""
    global jack_nuts, jack_bush, jack_frames, jack_ins, jack_lugs
    x = xf - FACE_W / 2
    yi = WALL_Y_IN
    T = lambda sol: sol.rotate((x, WALL_Y_OUT, z), (x + 1, WALL_Y_OUT, z), DRAFT_DEG)   # follow the wall draft
    bush = cyl(4.75, 8.0 + WALL, x, yi - 1.0, z, axis=(0, 1, 0)).cut(cyl(3.4, 14, x, yi - 2, z, axis=(0, 1, 0)))
    jack_bush = U(jack_bush, T(bush))
    jack_nuts = U(jack_nuts, T(hex_nut(14.0, 2.6, x, WALL_Y_OUT, z, 4.8, axis="y")))
    # disc stack: three phenolic layers Ø20, with a thin steel plate between
    ins = cyl(10.0, 1.6, x, yi - 1.6, z, axis=(0, 1, 0)).union(cyl(10.0, 1.6, x, yi - 3.6, z, axis=(0, 1, 0))).union(cyl(9.5, 1.6, x, yi - 5.6, z, axis=(0, 1, 0)))
    ins = ins.cut(cyl(3.6, 10, x, yi - 7, z, axis=(0, 1, 0)))
    jack_ins = U(jack_ins, T(ins))
    plates = cyl(10.2, 0.4, x, yi - 2.0, z, axis=(0, 1, 0)).union(cyl(10.2, 0.4, x, yi - 4.0, z, axis=(0, 1, 0)))
    f = plates
    # two spring prongs run along the axis into the enclosure, bent in toward the axis at the end:
    # sleeve (long, +X) and tip (short, -X). Ground comes from the bushing/frame itself.
    y0 = yi - 6.4
    for (dx, wid, ln) in ((1, 5.5, 24.0), (-1, 4.5, 16.0)):
        strip = boxc(0.5, ln, wid, x + dx * 8.2, y0 - ln / 2, z)
        bend = boxc(4.5, 0.5, wid, x + dx * 6.2, y0 - ln - 0.25, z)
        tipc = boxc(1.2, 1.2, wid, x + dx * 4.2, y0 - ln + 0.9, z)
        f = f.union(strip).union(bend).union(tipc)
    # three solder tabs with eyelets: ground at -Z (on the frame), tip and sleeve at the upper left/right
    for a in (-90, 40, 140):
        ra = math.radians(a)
        tab = boxc(4.0, 7.0, 0.5, x, y0 - 3.0, z + 9.0).cut(cyl(0.8, 2, x, y0 - 5.0, z + 9.0 - 1))
        tab = tab.rotate((x, 0, z), (x, 1, z), a - 90)
        if a == -90:
            f = f.union(tab)
        else:
            jack_lugs = U(jack_lugs, T(tab))
    jack_frames = U(jack_frames, T(f))

def dc_jack(xf, z):
    """DC-021 2.1 mm enclosed-frame panel jack (Tayda A-868): black threaded round body Ø12 x 14, chrome nut,
    phenolic back plate with 3 solder lugs."""
    global jack_nuts, jack_bush, jack_lugs, dc_body, dc_ins
    x = xf - FACE_W / 2
    yi = WALL_Y_IN
    T = lambda sol: sol.rotate((x, WALL_Y_OUT, z), (x + 1, WALL_Y_OUT, z), DRAFT_DEG)
    body = cyl(6.0, 19.5, x, yi - 12.0, z, axis=(0, 1, 0))                # threaded barrel, ~5.5 mm proud of the nut
    body = body.cut(cyl(3.0, 12, x, WALL_Y_OUT - 4.0, z, axis=(0, 1, 0)))
    pin = cyl(1.0, 10.0, x, WALL_Y_OUT - 4.0, z, axis=(0, 1, 0))
    dc_body = U(dc_body, T(body.union(pin)))
    jack_nuts = U(jack_nuts, T(hex_nut(14.0, 1.8, x, WALL_Y_OUT, z, 6.05, axis="y")))
    dc_ins = U(dc_ins, T(boxc(10.0, 1.6, 8.0, x, yi - 12.8, z)))
    for dz, dx, ln in ((3.0, -2.5, 5.0), (3.0, 2.5, 5.0), (-3.0, 0.0, 6.0)):
        jack_lugs = U(jack_lugs, T(boxc(2.0, ln, 0.4, x + dx, yi - 13.6 - ln / 2, z + dz)
                      .cut(cyl(0.6, 2, x + dx, yi - 13.6 - ln + 1.2, z + dz - 1))))

audio_jack(JACK_OUT_X, JACK_Z)
audio_jack(JACK_IN_X, JACK_Z)
dc_jack(JACK_DC_X, JACK_Z)

# ---------------- assembly / export ----------------
C = lambda r, g, b: cq.Color(r, g, b)
alu = C(0.80, 0.81, 0.83); chrome = C(0.85, 0.86, 0.88); black = C(0.03, 0.03, 0.03)
red = C(0.9, 0.05, 0.05); white = C(0.95, 0.95, 0.92); dark = C(0.12, 0.12, 0.13); blue = C(0.1, 0.25, 0.6)
phenolic = C(0.62, 0.32, 0.10); tan = C(0.72, 0.52, 0.28)

asm = cq.Assembly(name="pedal")
parts = [
    ("tub", tub, alu), ("lid", lid, alu), ("lid_screws", screws, chrome),
    ("pot_nuts", pot_nuts, chrome), ("pot_shafts", shafts, chrome), ("knobs", knobs, black), ("knob_marks", knob_marks, white),
    ("rotary_body", rot_body, chrome), ("rotary_mark", rot_mark, white), ("rotary_wafer", wafer, phenolic), ("rotary_lugs", rot_lugs, chrome),
    ("rotary_bush", rot_bush, chrome), ("rotary_nut", rot_nut, chrome), ("rotary_shaft", rot_shaft, chrome), ("rotary_knob", rot_knob, black),
    ("fs_body", fs_body, blue), ("fs_lugs", fs_lugs, chrome), ("fs_bushing", fs_bushing, chrome),
    ("fs_washer", fs_washer, white), ("fs_nut", fs_nut, chrome), ("fs_cap", fs_cap, chrome),
    ("bezels", bezels, chrome), ("leds", leds, red), ("leds_off", leds_off, C(0.45, 0.03, 0.02)), ("led_legs", led_legs, chrome),
    ("jack_bush", jack_bush, chrome), ("jack_nuts", jack_nuts, chrome), ("jack_frames", jack_frames, chrome),
    ("jack_ins", jack_ins, tan), ("jack_lugs", jack_lugs, chrome), ("dc_body", dc_body, black), ("dc_ins", dc_ins, tan),
]
for name, solid, color in parts:
    asm.add(solid, name=name, color=color)
asm.export(OUT, tolerance=0.02, angularTolerance=0.06)
json.dump({"jacks": {"out": JACK_OUT_X, "dc": JACK_DC_X, "in": JACK_IN_X, "z": JACK_Z, "wall_y_out": WALL_Y_OUT},
           "rotary_angle": ang, "z_face": Z_FACE, "z_face_in": Z_FACE_IN},
          open(OUT.replace(".glb", ".json"), "w"), indent=1)
print("exported", OUT)
