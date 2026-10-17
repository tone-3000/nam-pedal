"""
Blender: assemble enclosure + PCB, apply materials, render shots / export GLB.
usage: Blender -b -P render.py -- --shots final_iso,step6_top [--mode check|final] [--res 1600] [--samples 96] [--glb out.glb]

Inputs (defaults relative to hardware/):
  --enc  enclosure/cad/enclosure.glb   build with: python enclosure/cad/enclosure.py enclosure/cad/enclosure.glb
  --pcb  3d/board-full.glb             export from KiCad: kicad-cli pcb export glb --subst-models --include-tracks
                                       --include-pads --include-silkscreen --include-soldermask -o 3d/board-full.glb pcb/nam-pedal-diy.kicad_pcb
Textures in ./textures are produced by make_textures.py from the enclosure artwork.
"""
import bpy, sys, math, os, argparse
from mathutils import Vector, Euler

# ENC = this directory (hardware/renders/src); HW = hardware/.  anim.py exec()s this file and pre-sets ENC.
ENC = globals().get("ENC") or os.path.dirname(os.path.abspath(__file__))
HW = os.path.abspath(os.path.join(ENC, "..", ".."))

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
ap = argparse.ArgumentParser()
ap.add_argument("--mode", default="final")
ap.add_argument("--shots", default="final_iso")
ap.add_argument("--out", default=os.path.join(HW, "renders", "out"))
ap.add_argument("--enc", default=os.path.join(HW, "enclosure", "cad", "enclosure.glb"))
ap.add_argument("--pcb", default=os.path.join(HW, "3d", "board-full.glb"))
ap.add_argument("--res", type=int, default=1600)
ap.add_argument("--samples", type=int, default=96)
ap.add_argument("--glb", default="")          # export the current shot's visible set as GLB (first shot only)
ap.add_argument("--save-blend", default="")
ap.add_argument("--no-render", action="store_true")   # build the scene only (used by anim.py)
args = ap.parse_args(argv)
os.makedirs(args.out, exist_ok=True)

# ---------- scene reset ----------
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene

def import_glb(path, name):
    before = set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=path)
    new = [o for o in bpy.data.objects if o not in before]
    roots = [o for o in new if o.parent is None or o.parent not in new]
    root = bpy.data.objects.new(name, None)
    scene.collection.objects.link(root)
    for r in roots:
        r.parent = root
    return root, new

def smooth(o, angle=35):
    for p in o.data.polygons:
        p.use_smooth = True
    with bpy.context.temp_override(object=o, active_object=o, selected_editable_objects=[o]):
        try:
            bpy.ops.object.shade_smooth_by_angle(angle=math.radians(angle))
        except Exception:
            pass

enc_root, enc_objs = import_glb(args.enc, "ENC")
E = {o.name.split(".")[0]: o for o in enc_objs if o.type == 'MESH'}
for o in E.values():
    smooth(o)

pcb_root, pcb_objs = import_glb(args.pcb, "PCB")
pcb_root.scale = (1000, 1000, 1000)
pcb_root.rotation_euler = Euler((0, math.pi, 0), 'XYZ')
pcb_root.location = (119.6, 105.24, 26.5)
bpy.context.view_layer.update()

# the KiCad board carries a rotary-switch model on the arm; the real switch bolts to the face (enclosure part)
ROT_C = Vector((20.13, -27.67))
pcb_rotary = []
for o in pcb_objs:
    if o.type != 'MESH' or not o.data.vertices:
        continue
    c = sum((o.matrix_world @ v.co for v in o.data.vertices), Vector()) / len(o.data.vertices)
    if (Vector((c.x, c.y)) - ROT_C).length < 13.5 and 20 < c.z < 46:
        pcb_rotary.append(o)
print("pcb rotary objs:", len(pcb_rotary))
# the board's own LED models: make the lit ones emissive (preset 2 + PWR)
LED_C = [Vector(v) for v in [(-1.35, -5.71), (12.32, 1.88), (27.96, 1.84), (41.42, -5.72), (-38.81, -9.91)]]
LIT = {1, 4}
pcb_led_lit, pcb_led_off = [], []
for o in pcb_objs:
    if o.type != 'MESH' or not o.data.vertices:
        continue
    c = sum((o.matrix_world @ v.co for v in o.data.vertices), Vector()) / len(o.data.vertices)
    if c.z < 34:
        continue
    for i, l in enumerate(LED_C):
        if (Vector((c.x, c.y)) - l).length < 2.5:
            m = o.active_material
            if m and m.node_tree and m.node_tree.nodes.get("Principled BSDF") and m.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value[0] > 0.5:
                (pcb_led_lit if i in LIT else pcb_led_off).append(o)
print("pcb led lenses:", len(pcb_led_lit), len(pcb_led_off))

# ---------- materials ----------
def mat(name, color, metallic=0.0, rough=0.5, emission=None, strength=0.0, alpha=1.0, specular=None):
    m = bpy.data.materials.new(name); m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*color, 1)
    b.inputs["Metallic"].default_value = metallic
    b.inputs["Roughness"].default_value = rough
    if emission:
        b.inputs["Emission Color"].default_value = (*emission, 1)
        b.inputs["Emission Strength"].default_value = strength
    if alpha < 1:
        b.inputs["Alpha"].default_value = alpha
        m.surface_render_method = 'BLENDED' if hasattr(m, "surface_render_method") else None
    return m

M_ALU = mat("aluminum", (0.76, 0.77, 0.79), 1.0, 0.40)
M_CHROME = mat("chrome", (0.90, 0.91, 0.92), 1.0, 0.12)
M_STEEL = mat("steel", (0.72, 0.73, 0.74), 1.0, 0.30)
M_BLACK = mat("black_plastic", (0.015, 0.015, 0.016), 0.0, 0.32)
M_DARK = mat("dark_plastic", (0.06, 0.06, 0.065), 0.0, 0.5)
M_WHITE = mat("white_plastic", (0.85, 0.85, 0.82), 0.0, 0.45)
M_BLUE = mat("blue_plastic", (0.05, 0.15, 0.45), 0.0, 0.45)
M_PHENOLIC = mat("phenolic", (0.55, 0.25, 0.07), 0.0, 0.55)
M_TAN = mat("tan_phenolic", (0.60, 0.42, 0.22), 0.0, 0.6)
M_LED_OFF = mat("led_red_off", (0.45, 0.03, 0.02), 0.0, 0.25)
M_LED = mat("led_red", (0.6, 0.02, 0.01), 0.0, 0.25, emission=(1.0, 0.03, 0.01), strength=2.5)

# printed materials (textured): top face + front wall wordmark
def print_mat(name, base_png, mr_png):
    m = bpy.data.materials.new(name); m.use_nodes = True
    nt = m.node_tree; b = nt.nodes["Principled BSDF"]
    t_base = nt.nodes.new("ShaderNodeTexImage"); t_base.image = bpy.data.images.load(base_png)
    t_mr = nt.nodes.new("ShaderNodeTexImage"); t_mr.image = bpy.data.images.load(mr_png)
    t_mr.image.colorspace_settings.name = 'Non-Color'
    t_base.extension = t_mr.extension = 'EXTEND'
    sep = nt.nodes.new("ShaderNodeSeparateColor")
    nt.links.new(t_base.outputs["Color"], b.inputs["Base Color"])
    nt.links.new(t_mr.outputs["Color"], sep.inputs["Color"])
    nt.links.new(sep.outputs["Green"], b.inputs["Roughness"])
    nt.links.new(sep.outputs["Blue"], b.inputs["Metallic"])
    return m

M_FACE = print_mat("face_print", f"{ENC}/textures/face_base.png", f"{ENC}/textures/face_mr.png")
M_SIDE = print_mat("side_print", f"{ENC}/textures/side_base.png", f"{ENC}/textures/side_mr.png")
# wordmark placement on the front wall (Y = -58.5 side): 135 x 32 mm, centred on the wall
SIDE_W, SIDE_H = 135.0, 32.0
SIDE_X0 = -SIDE_W / 2
SIDE_Z0 = 2.0 + (37.5 - SIDE_H) / 2

ASSIGN = {
    "tub": M_ALU, "lid": M_ALU, "lid_screws": M_STEEL,
    "pot_nuts": M_STEEL, "pot_shafts": M_STEEL, "knobs": M_BLACK, "knob_marks": M_WHITE,
    "rotary_body": M_STEEL, "rotary_wafer": M_PHENOLIC, "rotary_lugs": M_STEEL, "rotary_bush": M_STEEL,
    "rotary_nut": M_STEEL, "rotary_shaft": M_STEEL, "rotary_knob": M_BLACK, "rotary_mark": M_WHITE,
    "fs_body": M_BLUE, "fs_lugs": M_STEEL, "fs_bushing": M_STEEL, "fs_washer": M_WHITE, "fs_nut": M_STEEL, "fs_cap": M_CHROME,
    "bezels": M_CHROME, "leds": M_LED, "leds_off": M_LED_OFF, "led_legs": M_STEEL,
    "jack_bush": M_STEEL, "jack_nuts": M_STEEL, "jack_frames": M_STEEL, "jack_ins": M_TAN, "jack_lugs": M_STEEL, "dc_body": M_BLACK, "dc_ins": M_TAN,
}
for o in pcb_led_lit:
    o.data.materials.clear(); o.data.materials.append(M_LED)
for o in pcb_led_off:
    o.data.materials.clear(); o.data.materials.append(M_LED_OFF)

for n, m in ASSIGN.items():
    o = E[n]; o.data.materials.clear(); o.data.materials.append(m)

# tub: UV + second material slot for the printed face
tub = E["tub"]; me = tub.data
me.materials.append(M_FACE)   # slot 1
me.materials.append(M_SIDE)   # slot 2
uv = me.uv_layers.new(name="UVMap")
mw = tub.matrix_world
for poly in me.polygons:
    c = mw @ me.vertices[poly.vertices[0]].co
    n = (mw.to_3x3() @ poly.normal).normalized()
    if n.z > 0.97 and c.z > 39.3:
        poly.material_index = 1
    elif n.y < -0.9 and c.y < -55:
        poly.material_index = 2
        for li in poly.loop_indices:
            p = mw @ me.vertices[me.loops[li].vertex_index].co
            uv.data[li].uv = ((p.x - SIDE_X0) / SIDE_W, (p.z - SIDE_Z0) / SIDE_H)
        continue
    for li in poly.loop_indices:
        p = mw @ me.vertices[me.loops[li].vertex_index].co
        uv.data[li].uv = ((p.x + 70.5) / 141.0, (p.y + 58.5) / 117.0)

# ---------- world / lights ----------
world = bpy.data.worlds.new("W"); scene.world = world; world.use_nodes = True
wn = world.node_tree; bg = wn.nodes["Background"]
# soft studio gradient: brighter above
sky = wn.nodes.new("ShaderNodeTexGradient"); sky.gradient_type = 'LINEAR'
tc = wn.nodes.new("ShaderNodeTexCoord"); mp = wn.nodes.new("ShaderNodeMapping")
mp.inputs["Rotation"].default_value = (0, math.radians(-90), 0)
mp.inputs["Location"].default_value = (0.5, 0, 0)
ramp = wn.nodes.new("ShaderNodeValToRGB")
ramp.color_ramp.elements[0].color = (0.06, 0.06, 0.065, 1); ramp.color_ramp.elements[0].position = 0.35
ramp.color_ramp.elements[1].color = (0.45, 0.46, 0.5, 1); ramp.color_ramp.elements[1].position = 0.9
wn.links.new(tc.outputs["Generated"], mp.inputs["Vector"]); wn.links.new(mp.outputs["Vector"], sky.inputs["Vector"])
wn.links.new(sky.outputs["Fac"], ramp.inputs["Fac"]); wn.links.new(ramp.outputs["Color"], bg.inputs["Color"])
bg.inputs["Strength"].default_value = 1.0

def area(name, loc, size, energy, target=(0, 0, 20), shape='SQUARE', size_y=None):
    l = bpy.data.lights.new(name, 'AREA'); l.energy = energy; l.size = size; l.shape = shape
    if size_y: l.size_y = size_y
    o = bpy.data.objects.new(name, l); scene.collection.objects.link(o)
    o.location = loc
    o.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat('-Z', 'Y').to_euler()
    return o

L = [
    area("key", (-200, -140, 330), 240, 3.0e6),
    area("fill", (280, -160, 180), 360, 1.0e6),
    area("rim", (40, 320, 240), 120, 2.2e6, shape='RECTANGLE', size_y=420),
    area("spec", (90, 60, 420), 60, 1.8e6, shape='RECTANGLE', size_y=500),   # long strip for brushed highlights
]
L_UNDER = area("under", (-80, -60, -300), 300, 3.2e6, target=(0, 0, 10))
L_UNDER2 = area("under2", (200, 120, -260), 260, 1.6e6, target=(0, 0, 10))

# shadow catcher
sc_me = bpy.data.meshes.new("floor"); sc = bpy.data.objects.new("floor", sc_me); scene.collection.objects.link(sc)
sc_me.from_pydata([(-900, -900, 0), (900, -900, 0), (900, 900, 0), (-900, 900, 0)], [], [(0, 1, 2, 3)])
sc.is_shadow_catcher = True
sc.data.materials.append(mat("floor", (0.3, 0.3, 0.3), 0, 0.8))

# ---------- camera ----------
cam_data = bpy.data.cameras.new("cam"); cam_data.lens = 70; cam_data.sensor_width = 36
cam_data.clip_start = 1.0; cam_data.clip_end = 10000.0
cam = bpy.data.objects.new("cam", cam_data); scene.collection.objects.link(cam); scene.camera = cam

def aim(loc, target, lens=70):
    cam.location = loc; cam_data.lens = lens
    cam.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat('-Z', 'Y').to_euler()

# ---------- visibility sets ----------
ALL_ENC = list(ASSIGN.keys())
ENC_LEDS = ["leds", "leds_off", "led_legs"]                 # enclosure-side LEDs (used when the board is not shown)
FACE_HW = ["bezels", "rotary_body", "rotary_wafer", "rotary_lugs", "rotary_bush", "rotary_nut", "rotary_shaft",
           "fs_body", "fs_lugs", "fs_bushing", "fs_washer", "fs_nut", "fs_cap"]
JACKS = ["jack_bush", "jack_nuts", "jack_frames", "jack_ins", "jack_lugs", "dc_body", "dc_ins"]
KNOBS = ["knobs", "knob_marks", "rotary_knob", "rotary_mark"]
LID = ["lid", "lid_screws"]
POTS = ["pot_nuts", "pot_shafts"]

# off-board wiring (curves in world coords; hidden unless a shot asks for them)
sys.path.insert(0, ENC)
import importlib, wires as _wires
importlib.reload(_wires)
M_TIN = mat("tinned", (0.78, 0.78, 0.76), 1.0, 0.38)
WIRES = _wires.build_wires(lambda n, c: mat(n, c, 0.0, 0.55), M_TIN)
WIRE_OBJS = [w["ins"] for w in WIRES] + [e for w in WIRES for e in w["ends"]]

def show(enc_parts, pcb=True, pcb_rot=False, floor=True, wires=False):
    for n, o in E.items():
        o.hide_render = o.hide_viewport = n not in enc_parts
    for o in pcb_objs:
        o.hide_render = o.hide_viewport = not pcb
    for o in WIRE_OBJS:
        o.hide_render = o.hide_viewport = not wires
    sc.hide_render = sc.hide_viewport = not floor

CLOSED = [n for n in ALL_ENC if n not in ENC_LEDS]          # closed pedal: board inside carries the LEDs
SHOTS = {
    # closed, finished pedal (board inside; its LEDs poke through the bezels)
    "final_iso":  dict(enc=CLOSED, cam=((-250, -310, 290), (0, 0, 22)), lens=75),
    "final_iso2": dict(enc=CLOSED, cam=((300, -270, 250), (0, 0, 22)), lens=75),
    "final_top":  dict(enc=CLOSED, cam=((0, -0.01, 560), (0, 0, 20)), lens=75),
    "final_jacks": dict(enc=CLOSED, cam=((120, 330, 220), (0, 30, 25)), lens=75),
    "hero":       dict(enc=CLOSED, cam=((-230, -330, 260), (0, 0, 22)), lens=80),
    # step 5: PCB nut-mounted to the inside of the face, bezels in; viewed from the open back
    "step5_back": dict(enc=["tub", "pot_nuts", "bezels"], floor=False, cam=((-220, -300, -300), (0, 5, 20)), lens=75),
    # step 6: hardware mounted, board nut-mounted, no knobs yet
    "step6_top":  dict(enc=["tub"] + POTS + FACE_HW + JACKS, cam=((-250, -310, 290), (0, 0, 22)), lens=75),
    "step6_back": dict(enc=["tub"] + POTS + FACE_HW + JACKS, floor=False, cam=((-220, -300, -300), (0, 5, 20)), lens=75),
    # step 7: off-board wiring done, seen from the open back (+ details)
    "step7_back": dict(enc=["tub"] + POTS + FACE_HW + JACKS, floor=False, wires=True, cam=((-220, -300, -300), (0, 5, 20)), lens=75),
    "step7_back2": dict(enc=["tub"] + POTS + FACE_HW + JACKS, floor=False, wires=True, cam=((200, -230, -330), (0, 8, 20)), lens=75),
    "step7_jacks": dict(enc=["tub"] + POTS + FACE_HW + JACKS, floor=False, wires=True, cam=((-70, -60, -75), (25, 38, 18)), lens=55),
    "step7_dc": dict(enc=["tub"] + POTS + FACE_HW + JACKS, floor=False, wires=True, cam=((20, -20, 0), (35.5, 44, 22)), lens=40),
    "step7_ctrl": dict(enc=["tub"] + POTS + FACE_HW + JACKS, floor=False, wires=True, cam=((40, 70, -90), (-15, -25, 18)), lens=55),
    # step 8: everything in, back open
    "step8_back": dict(enc=["tub"] + POTS + FACE_HW + JACKS, floor=False, cam=((-220, -300, -300), (0, 5, 20)), lens=75),
    "step8_back2": dict(enc=["tub"] + POTS + FACE_HW + JACKS, floor=False, cam=((260, -240, -320), (0, 5, 20)), lens=75),
    # detail checks
    "knob_detail": dict(enc=CLOSED, cam=((-110, -60, 110), (-33, 25, 45)), lens=90),
    "rotary_detail": dict(enc=["tub"] + POTS + FACE_HW + JACKS, floor=False, cam=((60, -120, -60), (20, -28, 25)), lens=80),
    "jack_detail": dict(enc=["tub"] + POTS + FACE_HW + JACKS, floor=False, cam=((10, -30, -40), (35, 45, 19)), lens=60),
    "led_detail": dict(enc=["tub"] + POTS + FACE_HW + JACKS, floor=False, cam=((60, -90, -30), (15, -3, 32)), lens=90),
    "rknob_detail": dict(enc=CLOSED, cam=((80, -90, 95), (22, -22, 48)), lens=85),
    "rknob_front": dict(enc=CLOSED, cam=((-10, 70, 95), (16, -16, 50)), lens=90),
    "rknob_side": dict(enc=CLOSED, cam=((-70, -45, 62), (20, -22, 50)), lens=90),
    "rotary_under": dict(enc=["tub"] + POTS + FACE_HW + JACKS, floor=False, cam=((50, -75, -35), (20, -28, 28)), lens=90),
    "jack_side": dict(enc=["tub"] + POTS + FACE_HW + JACKS, floor=False, cam=((2, -12, 12), (35, 50, 19)), lens=45),
    # full GLB
    "glb_full":   dict(enc=CLOSED, floor=False, cam=((-250, -310, 290), (0, 0, 22)), lens=75),
}

# ---------- render settings ----------
scene.render.resolution_x = args.res
scene.render.resolution_y = int(args.res * 0.75)
scene.render.image_settings.file_format = 'PNG'
scene.render.image_settings.color_mode = 'RGBA'
scene.render.film_transparent = True
if args.mode == "check":
    scene.render.engine = 'BLENDER_WORKBENCH'
    sh = scene.display.shading
    sh.light = 'STUDIO'; sh.color_type = 'TEXTURE'; sh.show_shadows = True; sh.show_cavity = True
    scene.render.film_transparent = False
else:
    scene.render.engine = 'CYCLES'
    scene.cycles.samples = args.samples
    scene.cycles.use_denoising = True
    scene.cycles.device = 'CPU'
    scene.view_settings.view_transform = 'AgX'
    scene.view_settings.look = 'AgX - Medium High Contrast'
    scene.view_settings.exposure = -0.8

first = True
for s in ([] if args.no_render else args.shots.split(",")):
    S = SHOTS[s]
    show(S["enc"], pcb=S.get("pcb", True), pcb_rot=False, floor=S.get("floor", True), wires=S.get("wires", False))
    aim(*S["cam"], lens=S.get("lens", 70))
    if args.glb and first:
        bpy.ops.object.select_all(action='DESELECT')
        for o in bpy.data.objects:
            if o.type == 'MESH' and not o.hide_render:
                o.select_set(True)
        for o in pcb_objs + enc_objs:
            if not o.hide_render and o.type == 'MESH':
                o.select_set(True)
        bpy.ops.export_scene.gltf(filepath=args.glb, use_selection=True, export_format='GLB',
                                  export_apply=True, export_yup=True, export_image_format='AUTO')
        print("exported glb", args.glb)
    first = False
    scene.render.filepath = os.path.join(args.out, f"{s}.png")
    bpy.ops.render.render(write_still=True)
    print("rendered", scene.render.filepath)

if args.save_blend:
    bpy.ops.wm.save_as_mainfile(filepath=args.save_blend)
