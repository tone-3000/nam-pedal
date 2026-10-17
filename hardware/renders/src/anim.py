"""
Build animation: components fly onto the board, board flips, controls seat, tub descends, hardware and
knobs seat, back plate closes, LEDs light, hero orbit.

usage: Blender -b -P anim.py -- [--out DIR] [--res 1920] [--samples 48] [--start 1] [--end 1095] [--frames 520,600]
encode: ffmpeg -framerate 30 -i out/anim/f_%04d.png -vf "fade=t=in:st=0:d=0.5,fade=t=out:st=35.8:d=0.7,format=yuv420p" -c:v libx264 -preset slow -crf 22 -movflags +faststart -an build-animation-1080p.mp4
Builds the scene via render.py (--no-render) and keyframes on top of it.
"""
import sys, os, math, random, argparse
import bpy
from mathutils import Vector, Matrix, Euler

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
ap = argparse.ArgumentParser()
ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "out", "anim"))
ap.add_argument("--res", type=int, default=1920)
ap.add_argument("--samples", type=int, default=64)
ap.add_argument("--step", type=int, default=1)
ap.add_argument("--start", type=int, default=1)
ap.add_argument("--end", type=int, default=1095)
ap.add_argument("--engine", default="eevee")
ap.add_argument("--save-blend", default="")
ap.add_argument("--frames", default="")   # comma list: render only these frames (check mode)
A = ap.parse_args(argv)
os.makedirs(A.out, exist_ok=True)

# ---- build the still scene (materials, lights, camera, sets) ----
ENC = os.path.dirname(os.path.abspath(__file__))
sys.argv = [sys.argv[0], "--", "--no-render", "--mode", "final", "--res", str(A.res)]
exec(compile(open(f"{ENC}/render.py").read(), "render.py", "exec"))
# now in scope: scene, E, pcb_root, pcb_objs, pcb_rotary, pcb_led_lit, pcb_led_off, cam, cam_data, sc, M_LED, CLOSED, ENC_LEDS, show, aim

FPS = 30
scene.render.fps = FPS
scene.frame_start, scene.frame_end = A.start, A.end
scene.frame_step = A.step
random.seed(7)

show(CLOSED, pcb=True, floor=False)
for o in pcb_rotary:
    o.hide_render = o.hide_viewport = True

# ---------------- helpers ----------------
def fcurves_of(id_data):
    ad = id_data.animation_data
    if not ad or not ad.action:
        return []
    act = ad.action
    if hasattr(act, "fcurves") and len(act.fcurves):
        return list(act.fcurves)
    out = []
    for layer in act.layers:
        for strip in layer.strips:
            for cb in strip.channelbags:
                out += list(cb.fcurves)
    return out

def key(obj, path, frame, value=None, interp="BEZIER", easing="AUTO"):
    if value is not None:
        setattr(obj, path, value)
    obj.keyframe_insert(data_path=path, frame=frame)
    for fc in fcurves_of(obj):
        if fc.data_path == path:
            for kp in fc.keyframe_points:
                if abs(kp.co.x - frame) < 0.5:
                    kp.interpolation = interp
                    kp.easing = easing

def key_hide(obj, frame, hidden):
    for p in ("hide_render", "hide_viewport"):
        setattr(obj, p, hidden)
        obj.keyframe_insert(data_path=p, frame=frame)

def world_bbox_center(objs):
    pts = []
    for o in objs:
        if o.type == 'MESH' and o.data.vertices:
            vs = o.data.vertices
            step = max(1, len(vs) // 400)
            pts += [o.matrix_world @ vs[i].co for i in range(0, len(vs), step)]
    if not pts:
        return None
    mn = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
    mx = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
    return (mn + mx) / 2, mx - mn

def subtree(o):
    out = [o]
    for c in o.children:
        out += subtree(c)
    return out

def reorigin(o, c):
    """Move an enclosure object's origin to world point c (mesh is in world coords, object at identity)."""
    o.data.transform(Matrix.Translation(-c))
    o.location = c

bpy.context.view_layer.update()

# ---------------- rigs ----------------
# ARIG: the whole assembly (tub, hardware, board). Flipped face-down for the build, turned face-up for the reveal.
AC = Vector((0, 0, 20))
ARIG = bpy.data.objects.new("ARIG", None); scene.collection.objects.link(ARIG); ARIG.location = AC
bpy.context.view_layer.update()
enc_root.parent = ARIG; enc_root.matrix_parent_inverse = Matrix.Translation(-AC)

pcb_root_node = [o for o in pcb_root.children][0]           # glTF "Node_0"
board_layers = [o for o in pcb_root_node.children if o.type == 'MESH']
units = [u for u in pcb_root_node.children if u not in board_layers]
bc, bd = world_bbox_center(board_layers)
print("board centre", bc, "dims", bd)

# RIG: the board. Child of ARIG; flips about its own centre for the control side; descends into the tub.
RIG = bpy.data.objects.new("RIG", None); scene.collection.objects.link(RIG)
RIG.parent = ARIG; RIG.matrix_parent_inverse = Matrix.Identity(4)
RIG.location = bc - AC
bpy.context.view_layer.update()
pcb_root.parent = RIG
pcb_root.matrix_parent_inverse = Matrix.Translation(-bc)
bpy.context.view_layer.update()

comp_units, ctrl_units, daisy_units = [], [], []
for u in units:
    objs = [o for o in subtree(u) if o.type == 'MESH' and o not in pcb_rotary]
    if not objs:
        continue
    c, d = world_bbox_center(objs)
    rel = c.z - bc.z
    if rel > 1.0:
        ctrl_units.append((u, c, d))              # pots, LEDs (control side, toward the face)
    elif rel < -7.0:
        daisy_units.append((u, c, d))             # Daisy module + its pins
    else:
        comp_units.append((u, c, d))              # through-hole parts on the component side
print("units: comp", len(comp_units), "ctrl", len(ctrl_units), "daisy", len(daisy_units))

bpy.context.view_layer.update()
Minv = pcb_root_node.matrix_world.inverted().to_3x3()      # board-space directions (final pose)
def local_dir(v):
    return Minv @ Vector(v)

EASE = dict(interp="CUBIC", easing="EASE_OUT")               # decelerate into the seat, never overshoot

def fly_in(unit, f0, f1, board_from):
    for o in subtree(unit):
        if o in pcb_rotary:
            continue
        key_hide(o, 1, True); key_hide(o, f0, False)
    base = unit.location.copy()
    key(unit, "location", f0, base + local_dir(board_from), **EASE)
    key(unit, "location", f1, base)

# ---------------- timeline (30 fps) ----------------
F_COMP0, F_COMP1 = 30, 200        # through-hole parts onto the component side (board component-side up)
F_DAISY0, F_DAISY1 = 185, 225     # Daisy onto its headers
F_FLIP0, F_FLIP1 = 235, 285       # board flips: control side up
F_CTRL0, F_CTRL1 = 290, 360       # pots + LEDs in from the control side
F_FLIP2a, F_FLIP2b = 370, 420     # board flips back: pots down
F_DESC0, F_DESC1 = 430, 500       # board lowered into the face-down tub
F_HWB0, F_HWB1 = 505, 545         # rotary + footswitch bodies down through the open back
F_JK0, F_JK1 = 548, 588           # jacks slide out to the wall from inside
F_JKN0, F_JKN1 = 575, 605         # jack nuts on from outside
F_FLIPA0, F_FLIPA1 = 612, 657     # pedal turns face up
F_NUT0, F_NUT1 = 662, 697         # pot nuts straight down the shafts
F_HWN0, F_HWN1 = 690, 725         # rotary nut, footswitch washer + nut, LED bezels down
F_FLIPB0, F_FLIPB1 = 732, 777     # face down again
F_WIRE0, F_WIRE1 = 782, 856       # off-board wiring, pad to lug
F_LID0, F_LID1 = 860, 900         # back plate down onto the open back
F_SCR0, F_SCR1 = 900, 932         # screws down
F_FLIPC0, F_FLIPC1 = 940, 985     # face up for the reveal
F_KNOB0, F_KNOB1 = 992, 1047      # knobs straight down the shafts
F_LED = 1062                      # LEDs on
F_END = 1095

# assembly orientation: face-down for the build, face-up for the nuts and the reveal
ARIG.rotation_mode = 'XYZ'
for f, r in ((1, math.pi), (F_FLIPA0, math.pi), (F_FLIPA1, 0.0), (F_FLIPB0, 0.0), (F_FLIPB1, math.pi),
             (F_FLIPC0, math.pi), (F_FLIPC1, 0.0)):
    key(ARIG, "rotation_euler", f, (r, 0, 0))

# board: floats above the tub (local -Z is world-up while ARIG is flipped), flips twice, then descends
RIG.rotation_mode = 'XYZ'
HOVER = Vector((0, 0, -120))
rb = (bc - AC)
key(RIG, "location", 1, rb + HOVER)
key(RIG, "location", F_DESC0, rb + HOVER)
key(RIG, "location", F_DESC1, rb)
key(RIG, "rotation_euler", 1, (0, 0, 0))
key(RIG, "rotation_euler", F_FLIP0, (0, 0, 0))
key(RIG, "rotation_euler", F_FLIP1, (math.pi, 0, 0))
key(RIG, "rotation_euler", F_FLIP2a, (math.pi, 0, 0))
key(RIG, "rotation_euler", F_FLIP2b, (0, 0, 0))

comp_dir = (0, 0, -1)            # board space: away from the component side
ctrl_dir = (0, 0, 1)             # board space: away from the control side

comp_sorted = sorted(comp_units, key=lambda t: (t[2].z, t[2].x * t[2].y))
n = len(comp_sorted)
for i, (u, c, d) in enumerate(comp_sorted):
    f0 = F_COMP0 + int((F_COMP1 - 30 - F_COMP0) * i / max(1, n - 1))
    jitter = Vector((random.uniform(-5, 5), random.uniform(-5, 5), 0))
    fly_in(u, f0, f0 + 26, Vector(comp_dir) * 70 + jitter)
for (u, c, d) in daisy_units:
    fly_in(u, F_DAISY0, F_DAISY1, Vector(comp_dir) * 60)
ctrl_sorted = sorted(ctrl_units, key=lambda t: (t[2].z, t[1].x))
m = len(ctrl_sorted)
for i, (u, c, d) in enumerate(ctrl_sorted):
    f0 = F_CTRL0 + int((F_CTRL1 - 28 - F_CTRL0) * i / max(1, m - 1))
    fly_in(u, f0, f0 + 28, Vector(ctrl_dir) * 60)

# ---------------- enclosure parts ----------------
# Offsets are given in ARIG-local space (the pedal's own frame: +Z toward the face, +Y toward the jack wall).
# The glTF import leaves every enclosure mesh with an R_x(90 deg) basis, so a location offset keyed on the object
# has to be expressed in that rotated basis: basis_off = (parent @ parent_inverse)^-1 @ ARIG_rot @ local_off.
bpy.context.view_layer.update()
def to_basis(o):
    M = (o.parent.matrix_world @ o.matrix_parent_inverse).to_3x3()
    return M.inverted() @ ARIG.matrix_world.to_3x3()

def drop(name, f0, f1, local_off):
    o = E[name]
    key_hide(o, 1, True); key_hide(o, f0, False)
    base = o.location.copy()
    key(o, "location", f0, base + to_basis(o) @ Vector(local_off), **EASE)
    key(o, "location", f1, base)

E["tub"].hide_render = E["tub"].hide_viewport = False          # tub is there from the start
# the KiCad pot models carry their own shafts; the enclosure's duplicate shafts stay hidden in the clip
E["pot_shafts"].hide_render = E["pot_shafts"].hide_viewport = True

# face-down: bodies drop through the open back (local -Z = world up), jacks slide out to the wall, nuts on from outside
for i, nm in enumerate(("rotary_bush", "rotary_shaft", "rotary_body", "rotary_wafer", "rotary_lugs")):
    drop(nm, F_HWB0, F_HWB1 - 6, (0, 0, -80))
for nm in ("fs_body", "fs_lugs", "fs_bushing", "fs_cap"):
    drop(nm, F_HWB0 + 8, F_HWB1, (0, 0, -80))
for nm in ("jack_bush", "jack_frames", "jack_ins", "jack_lugs", "dc_body", "dc_ins"):
    drop(nm, F_JK0, F_JK1, (0, -50, 0))
drop("jack_nuts", F_JKN0, F_JKN1, (0, 40, 0))
# face-up: everything threaded onto a shaft comes straight down it
drop("pot_nuts", F_NUT0, F_NUT1, (0, 0, 45))
drop("rotary_nut", F_HWN0, F_HWN1 - 8, (0, 0, 45))
drop("fs_washer", F_HWN0 + 4, F_HWN1 - 6, (0, 0, 45))
drop("fs_nut", F_HWN0 + 10, F_HWN1, (0, 0, 55))
drop("bezels", F_HWN0 + 6, F_HWN1, (0, 0, 40))
# face-down: back plate and screws
drop("lid", F_LID0, F_LID1, (0, 0, -140))
drop("lid_screws", F_SCR0, F_SCR1, (0, 0, -40))
# face-up: knobs straight down the shafts
drop("knobs", F_KNOB0, F_KNOB1, (0, 0, 55))
drop("knob_marks", F_KNOB0, F_KNOB1, (0, 0, 55))
# EQP knob: drops down the rotary shaft while turning onto its pointer position, via a handle on the shaft axis
rc = Vector((20.13, -27.67, 0.0))
RK = bpy.data.objects.new("RK", None); scene.collection.objects.link(RK)
RK.parent = ARIG; RK.matrix_parent_inverse = Matrix.Identity(4); RK.location = rc - AC
bpy.context.view_layer.update()
for o in (E["rotary_knob"], E["rotary_mark"]):
    mpi = RK.matrix_world.inverted() @ o.parent.matrix_world @ o.matrix_parent_inverse
    o.parent = RK; o.matrix_parent_inverse = mpi
    key_hide(o, 1, True); key_hide(o, F_KNOB0 + 10, False)
RK.rotation_mode = 'XYZ'
key(RK, "location", F_KNOB0 + 10, rc - AC + Vector((0, 0, 55)), **EASE)
key(RK, "location", F_KNOB1 + 10, rc - AC)
key(RK, "rotation_euler", F_KNOB0 + 10, (0, 0, math.radians(-35)), **EASE)
key(RK, "rotation_euler", F_KNOB1 + 10, (0, 0, 0))

# ---------------- wiring: each wire grows from its board pad to its lug ----------------
for o in WIRE_OBJS:
    o.parent = ARIG; o.matrix_parent_inverse = Matrix.Translation(-AC)
order = ["out_T", "out_S", "in_T", "in_TN", "in_S", "dc_p", "dc_n", "fs_a", "fs_b", "rot_0", "rot_1", "rot_2", "rot_3", "rot_4"]
wmap = {w["name"]: w for w in WIRES}
GROW = 24
for i, nm in enumerate(order):
    w = wmap[nm]; f0 = F_WIRE0 + 4 * i; f1 = f0 + GROW
    ins, (ba, bb) = w["ins"], w["ends"]
    for o in (ins, ba, bb):
        key_hide(o, 1, True)
    key_hide(ba, f0, False); key_hide(ins, f0, False); key_hide(bb, f1 - 1, False)
    cu = ins.data
    cu.bevel_factor_end = 0.0; cu.keyframe_insert("bevel_factor_end", frame=f0)
    cu.bevel_factor_end = 1.0; cu.keyframe_insert("bevel_factor_end", frame=f1)
    for fc in fcurves_of(cu):
        for kp in fc.keyframe_points:
            kp.interpolation = 'SINE'; kp.easing = 'EASE_IN_OUT'

em = M_LED.node_tree.nodes["Principled BSDF"].inputs["Emission Strength"]
em.default_value = 0.0; em.keyframe_insert("default_value", frame=F_LED - 1)
em.default_value = 3.0; em.keyframe_insert("default_value", frame=F_LED + 10)

# ---------------- camera (world space) ----------------
TGT = bpy.data.objects.new("TGT", None); scene.collection.objects.link(TGT)
cam.constraints.clear()
tr = cam.constraints.new('TRACK_TO'); tr.target = TGT; tr.track_axis = 'TRACK_NEGATIVE_Z'; tr.up_axis = 'UP_Y'
cam.rotation_euler = (0, 0, 0)
def cam_key(f, loc, tgt, lens):
    key(cam, "location", f, Vector(loc)); key(TGT, "location", f, Vector(tgt))
    cam_data.lens = lens; cam_data.keyframe_insert("lens", frame=f)

# board hover position in world (ARIG flipped about X through AC): (x, -y, 40 - z) then +120 up
hb = Vector((bc.x, -bc.y, 40 - bc.z)) + Vector((0, 0, 120))
cam_key(1,        hb + Vector((-110, -225, 125)), hb, 60)
cam_key(F_COMP1,  hb + Vector((-40, -230, 140)), hb, 60)
cam_key(F_DAISY1, hb + Vector((-20, -220, 135)), hb, 62)
cam_key(F_FLIP1,  hb + Vector((-140, -230, 160)), hb, 62)
cam_key(F_CTRL1,  hb + Vector((-150, -240, 170)), hb, 62)
cam_key(F_FLIP2b, hb + Vector((-190, -280, 150)), hb - Vector((0, 0, 40)), 60)
cam_key(F_DESC1,  (-230, -320, 300), (0, 0, 40), 62)          # tub face-down, board settling into it
cam_key(F_HWB1,   (-220, -300, 290), (0, 0, 40), 62)
cam_key(F_JKN1,   (-150, -330, 190), (0, -25, 28), 66)        # jack wall faces the camera while face-down
cam_key(F_FLIPA1, (-250, -320, 270), (0, 0, 25), 70)
cam_key(F_HWN1,   (-200, -270, 250), (-5, -5, 35), 72)
cam_key(F_FLIPB1, (-190, -280, 290), (0, 0, 40), 64)
cam_key(F_WIRE1,  (-110, -170, 330), (-5, 0, 30), 62)
cam_key(F_LID1,   (-200, -280, 260), (0, 0, 40), 68)
cam_key(F_SCR1,   (-205, -285, 255), (0, 0, 40), 68)
cam_key(F_FLIPC1, (-250, -320, 270), (0, 0, 22), 72)
cam_key(F_KNOB1,  (-170, -250, 230), (0, -10, 40), 75)
cam_key(F_END,    (-230, -330, 260), (0, 0, 22), 80)
for ob in (cam, TGT, cam_data):
    for fc in fcurves_of(ob):
        for kp in fc.keyframe_points:
            kp.interpolation = 'BEZIER'; kp.easing = 'AUTO'; kp.handle_left_type = kp.handle_right_type = 'AUTO_CLAMPED'

# ---------------- render settings ----------------
scene.render.resolution_x = A.res
scene.render.resolution_y = int(A.res * 9 / 16)
scene.render.film_transparent = False
if A.engine == "cycles":
    scene.render.engine = 'CYCLES'; scene.cycles.samples = A.samples
else:
    eng = 'BLENDER_EEVEE_NEXT' if 'BLENDER_EEVEE_NEXT' in [i.identifier for i in scene.render.bl_rna.properties['engine'].enum_items] else 'BLENDER_EEVEE'
    scene.render.engine = eng
    scene.eevee.taa_render_samples = A.samples
    for attr, val in (("use_raytracing", True), ("use_shadows", True), ("shadow_ray_count", 2), ("shadow_step_count", 4)):
        if hasattr(scene.eevee, attr):
            setattr(scene.eevee, attr, val)
    for l in bpy.data.lights:
        l.use_shadow = True
        if hasattr(l, "shadow_soft_size"):
            pass
# world: camera sees near-black; reflections still see the studio gradient
wnt = scene.world.node_tree
bg_node = wnt.nodes["Background"]
out_node = [n for n in wnt.nodes if n.type == 'OUTPUT_WORLD'][0]
bg_cam = wnt.nodes.new("ShaderNodeBackground"); bg_cam.inputs["Color"].default_value = (0.004, 0.004, 0.005, 1)
lp = wnt.nodes.new("ShaderNodeLightPath"); mix = wnt.nodes.new("ShaderNodeMixShader")
wnt.links.new(lp.outputs["Is Camera Ray"], mix.inputs["Fac"])
wnt.links.new(bg_node.outputs["Background"], mix.inputs[1])
wnt.links.new(bg_cam.outputs["Background"], mix.inputs[2])
wnt.links.new(mix.outputs["Shader"], out_node.inputs["Surface"])
scene.view_settings.view_transform = 'AgX'
scene.view_settings.look = 'AgX - Medium High Contrast'
scene.view_settings.exposure = -0.6
scene.render.image_settings.file_format = 'PNG'
scene.render.image_settings.color_mode = 'RGB'
scene.render.filepath = os.path.join(A.out, "f_")

if A.save_blend:
    bpy.ops.wm.save_as_mainfile(filepath=A.save_blend)
print("engine", scene.render.engine, "frames", scene.frame_start, scene.frame_end, "step", scene.frame_step)
if A.frames:
    for f in [int(x) for x in A.frames.split(",")]:
        scene.frame_set(f)
        scene.render.filepath = os.path.join(A.out, f"f_{f:04d}.png")
        bpy.ops.render.render(write_still=True)
else:
    bpy.ops.render.render(animation=True)
print("done")
