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
case_bottom = cq.Workplane(case_parts[2]).tag("bottom")
buttons = cq.Workplane(cq.Compound.makeCompound([
    case_parts[0], case_parts[3], case_parts[4], case_parts[5]
]))

# import SCD40 model
scd40 = cq.importers.importStep('Sensirion_CO2_Sensors_SCD4x_STEP_file.step')

# import PCB model
pcb = cq.importers.importStep('pcb.step')

# import Watchy model
watchy = cq.importers.importStep('Watchy_Battery.step')

# for some reason computing the center of the bound box of the
# actual case_bottom_plane doesn't work (offset oddly to the left)
# so we compute the center of the outermost face and then project
# that back when we make our *actual* work plane
outermost_center = (case_bottom
    .faces("-Z").faces("<Z")
    .workplane(centerOption='CenterOfBoundBox').val()
)
case_bottom = (case_bottom
    .faces("-Z").faces("<Z[-2]")
    .tag("bottom_faces")
    .workplane(centerOption='ProjectedOrigin', origin=outermost_center)
    .tag("bottom_plane")
    .rect(26, 40, forConstruction=True).vertices()
    .tag("bottom_screws")
)
# compute screw_boss_radius
screw_boss_radius = (
    # max screw
    case_bottom.vertices(">X and >Y", tag="bottom_screws")
    .val().Center().toPnt()
).Distance(
    # corner of topmost edge
    case_bottom.edges(">Y", tag="bottom_faces").vertices(">X and >Y")
    .val().Center().toPnt()
)
# tag the shelf bottom
case_bottom = (case_bottom
    .faces("+Z", tag="bottom").faces(">Z[-2]")
    .tag("bottom_shelf")
    .workplane(centerOption='ProjectedOrigin', origin=outermost_center)
    .tag("bottom_shelf_plane")
)
# compute scd40_offset (ie, center y position of scd40)
scd40_offset = -(outermost_center.y - scd40_offset_adj)
def do_scd40_translate(obj):
    return (obj
    .rotate([0,0,0], [1,0,0], 180)
    .translate(case_bottom.workplaneFromTagged("bottom_shelf_plane").val())
    .translate([0,scd40_offset,-shelf_height-wiring_space])
    )

scd40 = do_scd40_translate(scd40.translate([0,0,.8]))

simple_scd40 = (cq.Workplane("XY")
    .rect(10.1, 10.1).extrude(0.8)
    .faces("+Z").rect(8.5, 8.5)
    .workplane(offset=5.5).rect(7.8, 7.8)
    .loft(combine=True)
)
simple_scd40 = do_scd40_translate(simple_scd40)

#show_object(simple_scd40, name="simple-scd40")

# ok, here's the case extension
bottom_thick = (
    case_bottom.workplaneFromTagged("bottom_shelf_plane").val().z -
    case_bottom.workplaneFromTagged("bottom_plane").val().z
)
case_bottom = (case_bottom
    .workplaneFromTagged("bottom_shelf_plane")
    .move(0,scd40_offset)
    .rect(bump_xsize, bump_ysize, forConstruction=True)
    .vertices("<Y").tag("bottom_edge")
    .solids(tag="bottom")
)
pt1 = (case_bottom.vertices("<Y and >X", tag="bottom_screws")
       .val().Center())
pt2 = (case_bottom.vertices("<Y and >X", tag="bottom_edge")
       .val().Center())
def mk_bump(workplane, thickness):
    d = np.hypot(pt1.x - pt2.x, pt1.y - pt2.y)
    alpha = np.arctan2(pt1.x - pt2.x, pt1.y - pt2.y)
    beta = np.arcsin(screw_boss_radius / d)
    gamma = (np.pi/2) - (alpha + beta)
    d2 = np.sqrt(d*d - screw_boss_radius*screw_boss_radius)
    return (workplane
            .move(0, scd40_offset-(bump_ysize/2)).hLine(bump_xsize/2)
            .polarLine(d2, 90-np.degrees(alpha+beta))
            #.line(0.5,1) # a bit of a cheat, but also breaks the chamfer
            .hLineTo(0)
            .mirrorY()
            .extrude(thickness, combine=False)
    )
bottom_bump = (mk_bump(
    cq.Workplane("XY").copyWorkplane(
        case_bottom.workplaneFromTagged("bottom_shelf_plane")
    ).workplane(offset=-bottom_thick), bottom_thick
    )
    .faces("+Z").edges("(not <Y) and (not >Y)").chamfer(2)
    .faces("-Z").shell(-shelf_height)
)
pt3 = bottom_bump.faces(">Y").val().Center()
case_bottom = (case_bottom
    .union(bottom_bump)
    .workplaneFromTagged("bottom_shelf_plane")
    .workplane(offset=shelf_height, invert=True,
               centerOption='ProjectedOrigin', origin=pt3)
    .center(0,-10 + shelf_height)
    .rect(20,20).cutBlind(10)
)
# drill screw holes all the way through so they are not a blind tap
#case_bottom = case_bottom.vertices(tag="bottom_screws").hole(1.6)

# ok, now the same thing for the top of the case
case_top = (case_top
    .tag("top")
    .faces("<Z[-2]")
    .workplane(centerOption='ProjectedOrigin',origin=outermost_center)
    .tag("top_top_plane")
    .faces(">Z", tag="top")
    .workplane(centerOption='ProjectedOrigin',origin=outermost_center)
    .tag("top_bottom_plane")
    .solids(tag="top")
)
top_thick = (
    case_top.workplaneFromTagged("top_bottom_plane").val().z -
    case_top.workplaneFromTagged("top_top_plane").val().z
)
top_cut = (case_top
    .faces("+Z").faces("<<Z").tag("top_cut")
    .workplane()
    .faces(tag="top_cut").wires().toPending()
    .extrude(top_thick+10, combine=False)
).union(case_top
    .workplaneFromTagged("top_bottom_plane")
    .rect(100,100) # in theory we'd use a boundary box here
    .extrude(20, combine=False) # again, just "very tall"
)

top_bump = (mk_bump(
    cq.Workplane("XY").copyWorkplane(
        case_top.workplaneFromTagged("top_top_plane")
    ).workplane(invert=True), top_thick
    ).tag("top_bump")
    .faces("+Y").workplane()
    .faces("+Y", tag="top_bump").wires().toPending().extrude(2)
    .faces("-Z").edges("not >Y").chamfer(1.6)
    #.cut(top_cut) # done later on
)
case_top = (case_top
    .union(top_bump).tag("top_with_bump")
    .union(simple_scd40.faces("+Z").shell(air_space+grill_thick))
    .cut(simple_scd40).cut(simple_scd40.shell(air_space))
    .cut(top_cut)
)
# Grill
grill = (simple_scd40
    .faces("<<Z")
    .workplane(centerOption='CenterOfBoundBox')
    .tag("grill_plane")
)
if True:
    # Grill option 1
    grill = (grill
    .circle(7.5/2).circle(3.5/2)
    .extrude(air_space+grill_thick+1, combine=False)
    .workplaneFromTagged("grill_plane")
    .rect(grill_strut_width, 10).cutThruAll()
    .workplaneFromTagged("grill_plane")
    .rect(10, grill_strut_width).cutThruAll()
    )
elif False:
    # Grill option 2
    sp = (grill_strut_width + grill_space_width)/2
    grill = (grill
    .pushPoints([(-3*sp,0),(-sp,0),(sp,0),(3*sp,0)])
    .slot2D(7.5, grill_space_width, angle=90)
    .extrude(air_space+grill_thick+1, combine=False)
    )
else:
    # Grill option 3
    sp = (grill_strut_width + grill_space_width)/2
    grill = (grill
    .pushPoints([(0,-3*sp),(0,-sp),(0,sp),(0,3*sp)])
    .slot2D(7.5, grill_space_width, angle=0)
    .extrude(air_space+grill_thick+1, combine=False)
    )
case_top = case_top.cut(grill)

if True:
    show_object(case_top, name='top')
if True:
    show_object(case_bottom, name='bottom')
if True:
    show_object(buttons, name='buttons')
if True:
    show_object(scd40, name='scd40')
#show_object(pcb, name='pcb')

# remove a chunk from watchy
if True: # can disable this for faster refresh
    watchy = (watchy.transformed(offset=[watchy_board_width/2,0,0])
              .rect(15,8).cutThruAll()
    )
    show_object(watchy, name='watchy')

#debug(case_bottom.vertices(tag="bottom_screws").circle(screw_boss_radius))

# battery stats
battery_types = {
    # type: [zoffset, xoffset, diameter, length]
    '14500': [-4, -11, 14.2, 53 ], # AA 700-1000mAh
    '14430': [1, -11, 14.2, 43 ], # 2/3 AA? 400-600mAh
    '14250': [ 9, -11, 14.2, 25 ], # 1/2 AA 300mAh
    '10440': [2, -9, 10, 44 ], # AAA 250-350mAh
    '10280': [10, -9, 10, 28 ], # 2/3 AAA 200mAh
    '10180': [14, -9, 10, 18 ], # 1/3 AAA 90mAh
}
if False:
    b = '14500'
    battery_opt = (cq.Workplane("XZ")
    .workplane(invert=True, offset=battery_types[b][0])
    .center(battery_types[b][1],-2)
    .circle(battery_types[b][2]/2)
    .extrude(battery_types[b][3])
    )
    show_object(battery_opt, name=("battery " + b))
