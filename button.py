import cadquery as cq
import numpy as np

watchy_board_width = 33.8
shelf_height = 1.00
wiring_space = 0.6
air_space = 0.75
grill_thick = 1
grill_strut_width=1.5
grill_space_width=1.1
scd40_offset_adj = -0.5
bump_ysize=14.05
bump_xsize=14


# import Armadillonium model
case_parts = (
    cq.importers.importStep('Armadillonium_Model.step')
    .findSolid().Solids()
)
case_top = cq.Workplane(case_parts[1])
case_bottom = cq.Workplane(case_parts[2])
buttons = cq.Workplane(cq.Compound.makeCompound([
    case_parts[0], case_parts[3], case_parts[4], case_parts[5]
]))
button = cq.Workplane(case_parts[0]).tag("button")
# identify center of face where switch sits
button = button.faces("-X", tag="button").faces("<X").tag("bottom")
bottom_center_pt = button.val().Center()
# identify center of opposite face
button = button.faces("+X", tag="button").faces(">X").tag("top")
top_center_pt = button.val().Center()
# depth of button
button_depth = top_center_pt.x - bottom_center_pt.x;

cylinder_radius = 10

cyl = (cq.Workplane("XY")
    .workplane(centerOption='ProjectedOrigin', origin=bottom_center_pt)
    .center(-cylinder_radius + button_depth,0)
    .circle(cylinder_radius).extrude(10)
)

button = button.intersect(cyl).faces(">>X").fillet(0.75)

elephant_foot = 0.2
in_dist = 0.41 + elephant_foot
out_dist = 0.35 + elephant_foot

button = (button
    .faces(tag="bottom")
    .workplane(centerOption='CenterOfBoundBox')
    .faces(tag="bottom").wires().toPending()
    .extrude(out_dist)
    .faces("-X").faces("<X")
    .workplane(centerOption='CenterOfBoundBox')
    .tag("bottom_plane")
          # measured as 0.7m, switch throw is 0.25 +/- 0.1
    .rect(3,2).cutBlind(-in_dist)
)



#    .tag("button")
#    #.faces("+X").faces(">X").fillet(0.5)
#    .faces(">X").edges("not(|Y)").fillet(0.5)
#)

show_object(button, name="button")
