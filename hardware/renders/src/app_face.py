"""
Blender: render the companion app's preset-selector sprites from the pedal model.

The app draws the face flat (black print, bare-aluminium band and foot) and drops these on top
(app/src/components/Pedal.tsx):
  knob.webp       rotary knob, orthographic top-down, square tile centred on the axis (rotated in CSS)
  bezel-off.webp  one chrome bezel + LED, lens dark;  camera tilted so the chrome reads silvery
  bezel-on.webp   same, lens lit
  face.json       LED band, rotary centre and LED positions in face mm, and the sprite tile sizes

usage: blender -b -P app_face.py -- [--out ../../../app/src/assets/pedal] [--ppm 12] [--samples 64]
Builds the scene via render.py (--no-render), like anim.py.
"""
import sys, os, json, math, argparse
import bpy
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
ap = argparse.ArgumentParser()
ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "app", "src", "assets", "pedal"))
ap.add_argument("--ppm", type=float, default=12.0)   # render pixels per mm
ap.add_argument("--samples", type=int, default=64)
ap.add_argument("--tilt", type=float, default=20.0)  # bezel camera, degrees from vertical, looking up the face
ap.add_argument("--bezel-ppm", type=float, default=40.0)  # the bezel tile is small; render it denser
A = ap.parse_args(argv)
os.makedirs(A.out, exist_ok=True)

ENC = os.path.dirname(os.path.abspath(__file__))
sys.argv = [sys.argv[0], "--", "--no-render", "--mode", "final", "--samples", str(A.samples)]
exec(compile(open(f"{ENC}/render.py").read(), "render.py", "exec"))
# in scope: scene, E, pcb_objs, pcb_rotary, pcb_led_lit, pcb_led_off, cam, cam_data, CLOSED, show, mat, HW,
#           L (lights), ramp (studio backdrop colour ramp)

# ---- face geometry (mm, template coordinates: origin top-left of the printed face, y down) ----
FACE_W, FACE_H = 141.0, 117.0
ARC_LEDS = [(69.15, 64.21), (82.82, 56.62), (98.46, 56.66), (111.92, 64.22)]
ROTARY = (90.63, 86.17)
Z_FACE = 39.5
W = lambda xf, yf: (xf - FACE_W / 2, FACE_H / 2 - yf)
enc_json = json.load(open(os.path.join(HW, "enclosure", "cad", "enclosure.json")))
KNOB_MODEL_DEG = 90.0 - enc_json["rotary_angle"]   # model knob angle, degrees clockwise from 12 o'clock
# LED band measured from enclosure/face-artwork.png (20 px/mm): a half annulus, flat ends on the rotary centre line
BAND = {"outer": 35.28, "inner": 26.28}

KNOB = ["rotary_knob", "rotary_mark"]
KNOB_R = 30.0    # mm half-width of the knob tile
BEZEL_R = 6.0    # mm half-width of the bezel tile (bezel flange r 3.4)

M_LED_APP_OFF = mat("led_app_off", (0.30, 0.02, 0.015), 0.0, 0.25)
# kept just bright enough to read as lit: AgX washes strong red emission out to salmon
M_LED_APP_ON = mat("led_app_on", (0.95, 0.02, 0.01), 0.0, 0.25, emission=(1.0, 0.02, 0.0), strength=0.7)

# ---- scene: closed pedal, LEDs from the board, board's duplicate rotary hidden ----
show(CLOSED, pcb=True, floor=False)
for o in pcb_rotary:
    o.hide_render = o.hide_viewport = True
led_lenses = pcb_led_lit + pcb_led_off

def set_leds(m):
    for o in led_lenses:
        o.data.materials.clear(); o.data.materials.append(m)

def only(names):
    for n in E:
        E[n].hide_render = E[n].hide_viewport = n not in names
    for o in pcb_objs:
        o.hide_render = o.hide_viewport = True

cam_data.type = 'ORTHO'
scene.render.film_transparent = True
scene.render.image_settings.file_format = 'WEBP'   # ~10x smaller than PNG, keeps alpha
scene.render.image_settings.color_mode = 'RGBA'
scene.render.image_settings.quality = 90

def frame(target, half, tilt_deg=0.0, ppm=A.ppm):
    """Ortho camera on a square tile of +-half mm about target, tilted toward -Y (the pedal's foot)."""
    t = math.radians(tilt_deg)
    d = 400.0
    cam.location = (target[0], target[1] - d * math.sin(t), target[2] + d * math.cos(t))
    cam.rotation_euler = (Vector(target) - Vector(cam.location)).to_track_quat('-Z', 'Y').to_euler()
    cam_data.ortho_scale = 2 * half
    scene.render.resolution_x = scene.render.resolution_y = round(2 * half * ppm)

def render(name):
    scene.render.filepath = os.path.join(A.out, f"{name}.webp")
    bpy.ops.render.render(write_still=True)
    print("rendered", scene.render.filepath)

# 1. knob alone, straight down, centred on the rotary axis (the app rotates it about the tile centre
#    and adds the drop shadow in CSS so it keeps its direction as the knob turns)
only(KNOB)
frame((*W(*ROTARY), Z_FACE), KNOB_R)
render("knob")

# 2. one bezel + LED.  The face stays in the scene as a holdout: the chrome still reflects the dark
#    print and the LED still lights the flange, but the tile is transparent around the part.
only(["tub", "bezels"])
for o in led_lenses:
    o.hide_render = o.hide_viewport = False
E["tub"].is_holdout = True
# Real bezels have softened edges: a small bevel turns the flat rings' hard edges into bright
# highlight rings, which is what reads as "chrome" from (nearly) above.  Slightly rougher chrome
# blurs the studio into soft gradients instead of flat mirror patches.
bz = E["bezels"]
# Under an orthographic camera every point of a flat ring mirrors one sky direction, so the
# flat rings come out a single dark tone; the chrome read comes from the bevels and walls
# catching the studio boxes as white streaks against it.
bev = bz.modifiers.new("bevel", 'BEVEL'); bev.width = 0.35; bev.segments = 6; bev.limit_method = 'ANGLE'
bz.data.materials.clear(); bz.data.materials.append(mat("chrome_app", (0.97, 0.97, 0.98), 1.0, 0.14))
ramp.color_ramp.elements[0].color = (0.02, 0.02, 0.025, 1); ramp.color_ramp.elements[0].position = 0.45
ramp.color_ramp.elements[1].color = (1.0, 1.0, 1.0, 1); ramp.color_ramp.elements[1].position = 0.75
for l in L:
    l.data.energy *= {"key": 1.3, "spec": 1.0, "fill": 1.3, "rim": 0.5}[l.name]
frame((*W(*ARC_LEDS[1]), Z_FACE), BEZEL_R, A.tilt, A.bezel_ppm)
set_leds(M_LED_APP_OFF)
render("bezel-off")
set_leds(M_LED_APP_ON)
render("bezel-on")

# ---- geometry for the app (mm, template coordinates) ----
rx, ry = ROTARY
angles = [round(math.degrees(math.atan2(x - rx, ry - y)), 2) for x, y in ARC_LEDS]
geom = {
    "band": BAND,
    "rotary": {"center": list(ROTARY), "modelAngle": round(KNOB_MODEL_DEG, 2)},
    "leds": [{"center": list(p), "angle": a} for p, a in zip(ARC_LEDS, angles)],
    "tiles": {"knob": 2 * KNOB_R, "bezel": 2 * BEZEL_R},
}
path = os.path.join(A.out, "face.json")
json.dump(geom, open(path, "w"), indent=1)
print("wrote", path)
