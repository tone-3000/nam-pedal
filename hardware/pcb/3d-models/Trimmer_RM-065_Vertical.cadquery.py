import cadquery as cq
# KiCad model space: X = footprint x, Y = -footprint y, Z up, mm. Origin = pin 1.
BX0,BX1 = -0.7, 5.7          # body x extents (matches F.Fab outline)
BY0,BY1 = -5.7, 1.1          # body y extents (footprint y -1.1..5.7, flipped)
H = 4.6                      # body height
CX,CY = 2.5, -2.3            # rotor centre
body = (cq.Workplane("XY").box(BX1-BX0, BY1-BY0, H, centered=(True,True,False))
        .translate(((BX0+BX1)/2,(BY0+BY1)/2,0))
        .edges("|Z").fillet(0.6).faces(">Z").edges().chamfer(0.4))
# shallow recess for the rotor
body = body.cut(cq.Workplane("XY").circle(2.75).extrude(1.0).translate((CX,CY,H-0.6)))
rotor = cq.Workplane("XY").circle(2.6).extrude(1.0).translate((CX,CY,H-0.6))
slot_a = cq.Workplane("XY").rect(4.2,0.7).extrude(0.6).translate((CX,CY,H-0.2))
slot_b = cq.Workplane("XY").rect(0.7,4.2).extrude(0.6).translate((CX,CY,H-0.2))
rotor = rotor.cut(slot_a).cut(slot_b)
pins = cq.Workplane("XY")
legs = None
for (x,y) in [(0,0),(2.5,-5),(5,0)]:
    leg = cq.Workplane("XY").circle(0.3).extrude(3.5).translate((x,y,-3.0))
    legs = leg if legs is None else legs.union(leg)
asm = cq.Assembly()
asm.add(body, name="body", color=cq.Color(0.16,0.42,0.80))
asm.add(rotor, name="rotor", color=cq.Color(0.93,0.91,0.84))
asm.add(legs, name="legs", color=cq.Color(0.78,0.78,0.80))
asm.export("Trimmer_RM-065_Vertical.step")
print("ok")
