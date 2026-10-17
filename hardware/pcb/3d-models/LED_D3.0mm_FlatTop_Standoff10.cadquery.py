"""
3 mm flat-top diffused LED standing 10 mm off the board on its legs, as mounted in this pedal
(legs left long so the LED seats in the enclosure bezel / face hole).
KiCad LED_D3.0mm convention: pad 1 (cathode) at x=0, pad 2 at x=2.54, body centred on x=1.27,
Z up = away from the board, legs through the board to z=-2.5.

Run from any CadQuery 2.x env:  python LED_D3.0mm_FlatTop_Standoff10.cadquery.py
"""
import cadquery as cq

STANDOFF = 10.0          # board surface -> underside of flange
FLANGE_D, FLANGE_T = 3.8, 1.0
BODY_D, BODY_H = 3.0, 4.3   # above flange, flat top
LEG_D = 0.5
CX = 1.27

flange = (cq.Workplane("XY").circle(FLANGE_D / 2).extrude(FLANGE_T)
          .translate((CX, 0, STANDOFF)))
# cathode flat on the flange (pad 1 side, -x)
flange = flange.cut(cq.Workplane("XY").box(2, 6, 3).translate((CX - FLANGE_D / 2 - 1 + 0.35, 0, STANDOFF + 0.5)))
body = (cq.Workplane("XY").circle(BODY_D / 2).extrude(BODY_H)
        .translate((CX, 0, STANDOFF + FLANGE_T)))
body = body.faces(">Z").edges().fillet(0.5)
lens = flange.union(body)

legs = None
for x in (0.0, 2.54):
    leg = cq.Workplane("XY").circle(LEG_D / 2).extrude(STANDOFF + 2.5).translate((x, 0, -2.5))
    legs = leg if legs is None else legs.union(leg)

asm = cq.Assembly(name="LED_D3.0mm_FlatTop_Standoff10")
asm.add(lens, name="lens", color=cq.Color(0.85, 0.08, 0.05))
asm.add(legs, name="legs", color=cq.Color(0.75, 0.75, 0.76))
asm.export("LED_D3.0mm_FlatTop_Standoff10.step")
print("ok")
