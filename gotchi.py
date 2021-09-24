import cadquery as cq
import numpy as np
from math import atan2, degrees, radians, cos, sin, tan, sqrt
from watchy_sizes import *
from bat import make_battery_holder

watchy_board_width = 33.8

# watchy_pcb_bb: xlen=33.8 ylen=37.96
case_wall_clear = 0.7
case_wall_bot_clear = 0.5
case_wall_thick = 1.5

top_plate_thick = 1.5

# measurements from gotchi.svg
screw_separation_w = 41.2
screw_separation_h = 33.8
screw_lug_radius = 2.5
screw_inner_lug_radius = 2
faceplate_width = 43.3
faceplate_height = 36.1
faceplate_center_shift = 2.25
switch_plate_width = 26
switch_plate_depth = 5 # 4 on the model, but we need more room for switches
switch_plate_offset = 1
switch_plate_fillet = 0.75
switch_spacing = 8.5
switch_height = 2.5
switch_proud = 0.5 # probably switch depress is actually 0.3?
switch_pcb_thick = 0.6
switch_pcb_thick_clearance = 0.1 # pretty tight to support the switch
switch_pcb_width = 23.5  # PCB is 5x22.5 (but centered as if it was 4x)
switch_pcb_height = 4.5
switch_plate_thick = 1
# ensure we've got a decent wall at the front of the switch plate
switch_front_wall = switch_plate_thick
switch_cheat = (switch_plate_depth/2) - (switch_pcb_height)/2 + switch_front_wall
#switch_pcb_gap = (switch_pcb_thick + switch_height - switch_proud - switch_plate_thick)
extra_switch_pcb_gap = 1

# relative to screen center (x offset was 2.75)
pcb3_extra_width = 4
pcb3_position = (-0.75, -screw_separation_w/2 - switch_pcb_height/2)
pcb4_extra_width = 4
pcb4_position = (-0.75, screw_separation_w/2 - switch_pcb_height/2 - faceplate_center_shift)


screen_clearance = 0.5
pcb_support_radius = 2
m2_tap_diam = 1.6 # XXX
m2_clear_diam = 2 # XXX
m2_head_clear_diam = 2.5 # XXX
m2_head_thick = 1
mag_support_depth = 5
mag_angle2 = 35 # top angle; smaller is shallower
mag_angle1 = 20 # bottom angle; 0 is right angle
bottom_fillet = 1 # should be less than case_wall_thick
mag_depth_cheat = 0.85

switch_positions = [
    (-faceplate_height/2 - switch_plate_depth/2 + switch_cheat,
     i*switch_spacing,
     -screen_clearance - top_plate_thick + switch_height - switch_proud
    )
    for i in [-1,0,1]
]

# import SCD40 model
scd40 = cq.importers.importStep('Sensirion_CO2_Sensors_SCD4x_STEP_file.step')

# import Watchy model
watchy = cq.importers.importStep('Watchy.step').tag("watchy_untrimmed")

# trim off the case mounting straps
watchy = (watchy
    .faces("+Y", tag="watchy_untrimmed").faces(">Y[-2]")
    .tag("rightside")
    .workplane().split(keepBottom=True)
    .faces("-Y").faces("<Y[-2]")
    .tag("leftside")
    .workplane().split(keepBottom=True)
    .tag("watchy")
)

# compute an "all-in" bounding box (mostly for z depth information)
watchy_bb = watchy.solids().val().BoundingBox()

# find the center of the screen
watchy = (watchy
    .faces("-Z", tag="watchy").faces("<<Z[-3]")
    .tag("watchy_screen")
)
#debug(watchy, "watchy_screen_center")
watchy_screen_center = watchy.val().Center()

# find the top of the PCB
watchy = (watchy
    .faces("-Z", tag="watchy").faces("<<Z[-6]")
    .tag("watchy_pcb_top")
)
watchy_pcb_top = watchy.val().Center()

# find the bottom of the PCB
watchy = (watchy
    .faces("+Z", tag="watchy").faces("<<Z[-4]")
    .tag("watchy_pcb_bottom")
)
watchy_pcb_bb = watchy.val().BoundingBox()
watchy_pcb_bottom = watchy.val().Center()

# battery stats
def make_battery(type='10280', xOffset=0, yOffset=0, zOffset=0):
    battery_types = {
        # type: [diameter, length]
        '14500': [14.2, 53 ], # AA 700-1000mAh
        '14430': [14.2, 43 ], # 2/3 AA? 400-600mAh
        '14250': [14.2, 25 ], # 1/2 AA 300mAh
        '10440': [10, 44 ], # AAA 250-350mAh
        '10280': [10, 28 ], # 2/3 AAA 200mAh
        '10180': [10, 18 ], # 1/3 AAA 90mAh
    }
    return (
        cq.Workplane("YZ")
        .workplane(offset=xOffset)
        .center(yOffset,zOffset)
        .circle(battery_types[type][0]/2)
        .extrude(battery_types[type][1])
        .tag(type)
    )

# arguments to this function were computed originally, need to be altered
# for this design.
def make_lugs(p_outerLength=39, p_outerHeight=10, p_inset_depth=1, lugs_dia=5):
    # Strap Lugs
    # Originally from: github:giladaya/watchy-case-lugs (thanks!)
    p_strap_width = 22 + 0.5 # strap width inc. tolerance
    p_strap_dia = 4.0 # diameter of strap edge
    p_tbar_hole_r = 0.5 # Radius of t-bar pin

    tbar_hole_depth = 1.5
    lugs_th = 2.5
    lugs_width = p_strap_width + 2.0 * lugs_th
    #lugs_dia = 5.0
    lugs_length = p_outerHeight / 2.0 - 1.0

    lugs = (cq.Workplane("ZY")
        .workplane(
            origin=(0, -p_outerLength / 2.0, p_outerHeight - p_inset_depth), 
            offset=lugs_width / 2.0)
        .line(-lugs_length, -lugs_length)
        .tangentArcPoint((-lugs_dia, lugs_dia))
        .line(lugs_length, lugs_length)
        .close()
        .extrude(-lugs_th)
        .faces("<X")
        .edges()
        .fillet(lugs_th/2.0)
        .faces(">X")
        .workplane(
            origin=(0, -p_outerLength / 2.0 - p_outerHeight * 0.25 + 0.5, p_outerHeight * 0.25 - p_inset_depth + 0.5), 
            offset=(-tbar_hole_depth))
        .circle(p_tbar_hole_r)
        .cutBlind(tbar_hole_depth) 
        #.extrude(p_strap_width + tbar_hole_depth * 2.0)
        .mirror("ZY", union=True)
        .mirror("XZ", union=True)
        # rotate & translate giladaya's lugs to match our case orientation
        .rotate((0,0,0),(1,0,0), 180)
        .rotate((0,0,0),(0,0,1), 90)
    )
    lugs_translate = (watchy_screen_center.x, watchy_screen_center.y, 0)
    return lugs.translate(lugs_translate)

# magnetic connector
mag_translate = None # will be computed below
def make_mag(clearance=0, inner_clearance=None, extra=0):
    epsilon=.001 if clearance == 0 else 0
    if inner_clearance is None:
        inner_clearance = clearance
    prev = cq.Workplane("XY").workplane(offset=-clearance)
    l = []
    for diam,thick in [
            (7, 1 + clearance + inner_clearance),
            (5, 4-1-1-1.3 - inner_clearance + inner_clearance),
            (3.5, 1.3 - 2*inner_clearance),
            (5, 1 + clearance + inner_clearance + extra),
    ]:
        m = (prev
             .rect(14 + diam + 2*clearance, diam + 2*clearance)
             .extrude(thick, combine=False)
             .edges("|Z").fillet(diam/2 - epsilon)
        )
        prev = m.faces("+Z and >Z").workplane()
        l.append(m)
    m = l.pop(0)
    for m2 in l:
        m = m.union(m2)
    # zero is "the bottom edge" and "the outside face"
    return m.rotate((0,0,0), (1,0,0), -90).translate((0,0,-3.5))

# pushbutton switch
def make_sw(H=switch_height):
    sw = (cq.Workplane("XY").tag("base")
        .rect(3.35, 1).extrude(0.85)
        .workplaneFromTagged("base")
        .rect(2, 3.35).extrude(0.85)
        .faces(">Z").edges(">X or <X or >Y or <Y")
        .fillet((3.35-3.1)/2)
        .workplaneFromTagged("base")
        .pushPoints([(0,-1),(0,1)])
        .rect(3.95, 0.38).extrude(.15)
    )
    body_bottom = (sw
        .workplaneFromTagged("base")
        .rect(3.1, 3.1).extrude(0.85, combine=False)
        .edges("|Z").chamfer(0.2)
    )
    body_top = (sw
        .workplaneFromTagged("base").workplane(offset=0.8)
        .rect(3.1-0.4, 3.1-0.45).extrude(1.2 - 0.8, combine=False)
        .faces(">Z").edges()
        .fillet(0.35)
    )
    plunger = (sw
        .workplaneFromTagged("base").workplane(offset=0.85)
        .circle(1.8/2).extrude(H-0.85)
    )
    sw = sw.union(body_bottom).union(body_top).union(plunger)
    # rotate to match watchy orientation
    return (sw
            .rotate((0,0,0), (1,0,0), 180)
            .rotate((0,0,0), (0,0,1), 90)
    )

def make_sw_cutout(extra_depth=10, extra_length=2.5, H=switch_height):
    # pcb is 4x5, leave at least 0.5mm on all side (so 5x6); we're doing 4.75x7
    cheat=0.25 # trim the top clearance
    sw = (cq.Workplane("XY").tag("base")
          .workplane(offset=-extra_depth)
          .center(0,-cheat/2)
          .rect(4.5+extra_length,5-cheat).extrude(extra_depth + 1.2)
          .faces(">Z").workplane()
          .circle(2/2).extrude(H-1.2 + extra_depth)
    )
    # rotate to match watchy orientation
    return (sw
            .rotate((0,0,0), (1,0,0), 180)
            .rotate((0,0,0), (0,0,1), 90)
    )
    return sw

def sw4_translate(obj, perp_distance=(switch_proud - switch_height), par_distance=5.5):
    desired_wall_thick = 1
    desired_pcb_clearance = 0.25
    total_pcb_clear = 4 + 2*desired_pcb_clearance + 2*desired_wall_thick
    case2_top_thick = 2.56346 # hack hack hack

    perp_distance += screw_inner_lug_radius
    return (obj
        .rotate((0,0,0),(0,0,1), mag_angle2)
        .translate(watchy_screen_center +
                   cq.Vector(0,
                             -faceplate_center_shift,
                             -watchy_screen_center.z
                             + mag_translate.z - 3.5))
        .translate((screw_separation_h/2, -screw_separation_w/2, 0))
        .translate((cos(radians(-90 + mag_angle2))*perp_distance,
                    sin(radians(-90 + mag_angle2))*perp_distance, 0))
        .translate((cos(radians(-180 + mag_angle2))*par_distance,
                    sin(radians(-180 + mag_angle2))*par_distance, 0))
        ## "as high as it could go"
        .translate((0,0,(total_pcb_clear/2) - case2_top_thick))
        ## centered between the top options
        #.translate((0,0,(case2_thick + case2_top_thick)/2 - case2_top_thick))
        ## "as low as it could go"
        #.translate((0,0,case2_thick - (total_pcb_clear/2)))
    )

case1_depth = watchy_bb.zmax - watchy_pcb_top.z + case_wall_clear
def make_case1():
    return (cq.Workplane("XY")
    .workplane(offset=watchy_pcb_top.z,
               centerOption='ProjectedOrigin', origin=watchy_pcb_bb.center)
    .tag("case_bottom_top")
    .rect(watchy_pcb_bb.xlen + 2*case_wall_clear,
          watchy_pcb_bb.ylen + 2*case_wall_clear)
    .extrude(case1_depth, combine=False)
    .faces("<<Z").shell(case_wall_thick)
    .union(lugs)
#    .workplaneFromTagged("case_bottom_top")
#    .rect(watchy_pcb_bb.xlen + 2*case_wall_clear,
#          watchy_pcb_bb.ylen + 2*case_wall_clear)
#    .cutBlind(watchy_bb.zmax - watchy_pcb_top.z + case_wall_clear)
    )

def make_switch_plate(wp):
    return (cq.Workplane("XY").copyWorkplane(wp)
    .line(-faceplate_height/2, -switch_plate_width/2 - switch_plate_offset)
    .line(-switch_plate_depth, switch_plate_offset)
    .line(0, switch_plate_width)
    .line(switch_plate_depth, switch_plate_offset)
    .close()
    )

case2_thick = watchy_bb.zmax + case_wall_bot_clear + case_wall_thick - watchy_pcb_bottom.z
case2 = (cq.Workplane("XY")
    .workplane(offset=watchy_pcb_bottom.z,
               centerOption='ProjectedOrigin',
               origin=watchy_screen_center + cq.Vector(0,-faceplate_center_shift,0))
    .tag("case2_bottom_top")
    .rect(screw_separation_h + 2*screw_inner_lug_radius,
          screw_separation_w + 2*screw_inner_lug_radius)
    .extrude(case2_thick, combine=False)
    .faces("not(+Z or -Z)").edges("+Z or -Z")
    .fillet(screw_inner_lug_radius)
)

# extrude a bit for the magnetic connector
# have to do some math here to make edges tangent with screw_inner_lug
def make_mag_support(workplane):
    top_screw = cq.Vector(screw_separation_h/2, -screw_separation_w/2)
    bot_screw = top_screw + cq.Vector(-screw_separation_h, 0)
    top_point = top_screw + cq.Vector(
        cos(radians(-90 + mag_angle2))*screw_inner_lug_radius,
        sin(radians(-90 + mag_angle2))*screw_inner_lug_radius
    )
    bot_point = bot_screw + cq.Vector(
        cos(radians(-180 + mag_angle1))*screw_inner_lug_radius,
        sin(radians(-180 + mag_angle1))*screw_inner_lug_radius
    )
    target_y = -screw_separation_w/2 - screw_inner_lug_radius - mag_support_depth
    top_delta_y = (top_point.y - target_y) # positive
    top_delta_x = top_delta_y / tan(radians(mag_angle2))
    bot_delta_y = (bot_point.y - target_y) # positive
    bot_delta_x = bot_delta_y * tan(radians(mag_angle1))
    return (cq.Workplane("XY").copyWorkplane(workplane)
    .moveTo(top_screw.x, top_screw.y)
    .lineTo(top_point.x, top_point.y)
    .lineTo(top_point.x - top_delta_x, target_y)
    .lineTo(bot_point.x + bot_delta_x, target_y)
    .lineTo(bot_point.x, bot_point.y)
    .lineTo(bot_screw.x, bot_screw.y)
    .close()
    .extrude(case2_thick, combine=False)
    )

mag_support = (make_mag_support(case2.workplaneFromTagged("case2_bottom_top"))
    .faces("-Y and <<Y")
)
# find center of the Y face
#mag_support_bb = mag_support.val().BoundingBox()
mag_support_center = mag_support.val().Center()
mag_support= (mag_support
    .edges("+Z or -Z").fillet(2)
)
mag_translate = mag_support_center + cq.Vector(
    # "center of mag_support" is case2_thick; then move up to compensate for
    # bottom fillet, which is case_wall_thick
    #(0,0,(case2_thick/2) - bottom_fillet + mag_depth_cheat)
    (0,0,-(case2_thick/2) + 3.5)
)

case2 = (case2
    .union(mag_support)
)

# cutout2 for switch #4
sw4_cutout = sw4_translate(make_sw_cutout()
        .rotate((0,0,0),(0,0,1), 90)
        .rotate((0,0,0),(1,0,0),-90)
)
sw4_cutout_short = sw4_translate(
    make_sw_cutout(extra_depth=switch_pcb_thick + switch_pcb_thick_clearance)
    .union(make_sw_cutout(extra_length=-1))
        .rotate((0,0,0),(0,0,1), 90)
        .rotate((0,0,0),(1,0,0),-90)
)

# now add switch support
case2_switch = (make_switch_plate(
    case2
    .workplaneFromTagged("case2_bottom_top")
    .center(0,faceplate_center_shift))
    .extrude(case2_thick, combine=False)
    .faces("<X").edges("|Z").fillet(switch_plate_fillet)
)
case2 = (case2
    .union(case2_switch)
    .tag("case2_body_prelim")
    .faces("<Z").tag("case2_split_face")
)
case2_top_thick = (
    watchy_pcb_bottom.z - watchy_screen_center.z + screen_clearance
)
case2_top = (case2
    .workplaneFromTagged("case2_bottom_top")
    .faces(tag="case2_split_face").wires().toPending()
    .extrude(-case2_top_thick, combine=False)
)
case2_mag_shield = (cq.Workplane("XY")
    .copyWorkplane(case2.workplaneFromTagged("case2_bottom_top"))
    .workplane(centerOption='ProjectedOrigin', origin=mag_translate, invert=True)
    .center(0,-5)
    .rect(23,10).extrude(3.5, combine=False) # should be 4.5
    .faces("-Z").edges("|Y").fillet(3.25)
)
#debug(case2_mag_shield)
case2_top = (case2_top
    .union(case2_mag_shield)
    # cut out space for auxilliary PCBs
    .workplaneFromTagged("case2_bottom_top")
    .workplane(centerOption='ProjectedOrigin', origin=watchy_screen_center)
    .moveTo(pcb3_position[0], pcb3_position[1])
    .rect(switch_pcb_width + pcb3_extra_width, switch_pcb_height)
    .moveTo(pcb4_position[0], pcb4_position[1])
    .rect(switch_pcb_width + pcb4_extra_width, switch_pcb_height)
    .cutBlind(-(case2_top_thick - case_wall_thick))
    # cut out space for the main PCB
    .workplaneFromTagged("case2_bottom_top")
    .workplane(centerOption='ProjectedOrigin', origin=watchy_pcb_bb.center)
    .rect(watchy_pcb_bb.xlen + 2*case_wall_clear,
          watchy_pcb_bb.ylen + 2*case_wall_clear)
    .cutThruAll()
)
case2 = (case2
    .workplaneFromTagged("case2_bottom_top")
    .faces("+Z", tag="case2_body_prelim").edges().fillet(bottom_fillet)
    .cut(make_mag(extra=10).translate(mag_translate))
    .cut(make_mag(extra=10).translate(mag_translate).shell(0.25))
    .cut(sw4_cutout)
    # boss to support the switches
    .workplaneFromTagged("case2_bottom_top")
    .workplane(centerOption='ProjectedOrigin', origin=watchy_screen_center)
    .center(switch_positions[1][0], switch_positions[1][1])
    .rect(switch_pcb_height, switch_pcb_width)
         # shrink rectangle to provide a bit of clearance
         .offset2D(-0.25)
    .extrude((watchy_screen_center.z + switch_positions[1][2] + switch_pcb_thick + switch_pcb_thick_clearance)
             - watchy_pcb_bottom.z)
    # cut out space for auxilliary PCBs
    .workplaneFromTagged("case2_bottom_top")
    .workplane(centerOption='ProjectedOrigin', origin=watchy_screen_center)
    .moveTo(pcb3_position[0], pcb3_position[1])
    .rect(switch_pcb_width + pcb3_extra_width, switch_pcb_height)
    .moveTo(pcb4_position[0], pcb4_position[1])
    .rect(switch_pcb_width + pcb4_extra_width, switch_pcb_height)
    .cutBlind(case2_thick - case_wall_thick)
    .tag("case2_body")
    # cut out space for the PCB
    .workplaneFromTagged("case2_bottom_top")
    .workplane(centerOption='ProjectedOrigin', origin=watchy_pcb_bb.center)
    .rect(watchy_pcb_bb.xlen + 2*case_wall_clear,
          watchy_pcb_bb.ylen + 2*case_wall_clear)
    .extrude(watchy_bb.zmax - watchy_pcb_bottom.z + case_wall_bot_clear,
             both=True, combine=False).tag("pcb_cutout")
    .edges("|Z", tag="pcb_cutout")
    .chamfer(case_wall_clear + 2.7).tag("pcb_cutout2")
)
case2 = case2.solids(tag="case2_body").cut(case2.solids(tag="pcb_cutout2"))
case2 = (case2
    .workplaneFromTagged("case2_bottom_top")
    .rect(screw_separation_h,
          screw_separation_w, forConstruction=True)
    .vertices().circle(m2_tap_diam/2).cutThruAll()
)
show_object(case2, name='case2')

def make_top_plate(case2_top):
    top_plate = (cq.Workplane("XY")
    # create workplane, centered on the top plate
    # (which is offset from the screen)
    .workplane(offset=-(watchy_screen_center.z - screen_clearance),
               centerOption='ProjectedOrigin',
               invert=True,
               origin=watchy_screen_center + cq.Vector(0,-faceplate_center_shift,0))
    .tag("top_plate")
    # extrude rectangular center body
    .rect(faceplate_height, faceplate_width)
    .extrude(top_plate_thick)
    .faces("<Z and -Z").tag("top_plate_top")
    # extrude the screw bosses
    .workplaneFromTagged("top_plate")
    .rect(screw_separation_h, screw_separation_w, forConstruction=True)
    .vertices().circle(screw_lug_radius)
    .extrude(top_plate_thick)
    )
    # extrude switch plate.
    top_plate = top_plate.union(make_switch_plate(
    top_plate
    .workplaneFromTagged("top_plate")
    .center(0,-faceplate_center_shift))
    .tag("switch_plate")
    .extrude(top_plate_thick)
    .faces("not(+Z or -Z)").edges("+Z or -Z")
    .fillet(switch_plate_fillet)
    )
    # final bits
    top_plate = (top_plate
    # cut out screen
    .workplaneFromTagged("top_plate")
    .center(0,-faceplate_center_shift)
    .rect(30,30).cutThruAll(taper=45) # taper is in degrees
    # add to other other part of top case
    .union(case2_top)
    # cut out mag connector
    .cut(make_mag(extra=4).translate(mag_translate))
    .cut(make_mag(extra=4).translate(mag_translate).shell(0.25))
    .cut(sw4_cutout_short)
    # countersink the screw holes
    .workplaneFromTagged("top_plate")
    .faces(tag="top_plate_top").workplane(centerOption='ProjectedOrigin')
    .rect(screw_separation_h, screw_separation_w, forConstruction=True)
    .vertices()
    .cboreHole(m2_clear_diam, m2_head_clear_diam, m2_head_thick)
    # holes for gotchi switches
    .workplaneFromTagged("top_plate")
    .faces(tag="top_plate_top").workplane(
        centerOption='ProjectedOrigin', origin=watchy_screen_center
    )
    .pushPoints(switch_positions)
    .circle(2/2).cutThruAll()
    # pcb clearance for gotchi switches
    .faces("+Z and >Z").workplane(
        centerOption='ProjectedOrigin', origin=watchy_screen_center,
        invert=True
    ).center(switch_positions[1][0] + extra_switch_pcb_gap/2, switch_positions[1][1])
    .rect(switch_pcb_height + extra_switch_pcb_gap, switch_pcb_width)
    .cutBlind(case2_top_thick + top_plate_thick - switch_plate_thick)
    )
    return top_plate

if True:
    show_object(make_lugs(
        # length
        (screw_separation_h/2 + screw_inner_lug_radius) +
        (faceplate_height/2 + switch_plate_depth),
        # height
        case2_thick,# + case2_top_thick + top_plate_thick,
        # inset depth
        0,
        # lug diameter
        3
    ).translate((
        # the switch plate makes our case not centered on watchy, so shift
        ((screw_separation_h/2 + screw_inner_lug_radius) -
         (faceplate_height/2 + switch_plate_depth))/2,
        0,
        # lugs zero is the bottom of the case, so make that match
        watchy_pcb_bottom.z + case2_thick
    )), name='lugs')

if False:
    show_object(watchy.solids(tag="watchy"), name='watchy')

if False:
    show_object(make_case1(), name='case')

if True:
    show_object(make_top_plate(case2_top), name='top_plate')
elif True:
    show_object(case2_top, name='case2_top')

if False:
    show_object(make_mag().translate(mag_translate), name='magnetic_latch')

if True:
    #show_object(make_battery('10280', xOffset=3, yOffset=53, zOffset=1), name='10280')
    battery_holder_radius = 6.55 + 0.6 # clip offset + pcb thickness
    battery_holder_length = 37 # pcb length
    battery_holder = (make_battery_holder()
        # rotate to match watchy orientation
        .rotate((0,0,0),(0,0,1),90)
        # put center at the rotation axis of the battery
        .translate((0,0,-battery_holder_radius))
    )
    # mock up some end caps (total length 37mm)
    # adjust radius to make battery_holder_radius the flat-to-flat distance
    cap_oct_radius = battery_holder_radius / cos(radians(45/2)) # about 7.7
    cap_thick = 6.5
    chamfer_amt = 0.5
    cap_wall = 1
    end_cap = (cq.Workplane("YZ")
        .workplane(offset=battery_holder_length/2 - cap_thick)
        .transformed(rotate=(0,0,360/16)).tag("bot")
        .polygon(8, cap_oct_radius * 2)
        .extrude(cap_thick + chamfer_amt)
        .faces("+X").edges().chamfer(chamfer_amt)
        .workplaneFromTagged("bot")
        .polygon(8, (cap_oct_radius - cap_wall) * 2)
        #.cutThruAll()
        .cutBlind(cap_thick + chamfer_amt - cap_wall)
    )
    end_cap = end_cap.union(end_cap.mirror("YZ"))
    # decorate it a bit
    end_cap = (end_cap.faces("+X and >X").wires().toPending())
    nominal_start_radius = (cap_oct_radius - chamfer_amt) * cos(radians(45/2))
    start_angle = 45
    cap_radius = nominal_start_radius / sin(radians(start_angle))
    angle_increment = 9
    last_z = cos(radians(start_angle))*cap_radius
    for angle in range(start_angle-angle_increment, angle_increment, -angle_increment):
        new_z = cos(radians(angle))*cap_radius
        new_r = sin(radians(angle))*cap_radius
        end_cap = end_cap.workplane(offset=new_z - last_z).circle(new_r)
        last_z = new_z
    end_cap = end_cap.loft(combine=True)
    end_cap = (end_cap
        # neck
        .faces("-X and <X").workplane()
        .circle(6.7/2).extrude(0.5)
        # knurl
        .faces("-X and <X").workplane()
    )
    knurl = (cq.Workplane("YZ").copyWorkplane(end_cap)
        .circle(9.3/2).extrude(2.8)
        .polygon(12, 9.3, forConstruction=True).vertices()
        .circle(0.5).cutThruAll()
    )
    end_cap = (end_cap
        .solids()
        .union(knurl)
        # second knurl (with diamonds)
        .faces("-X and <X").workplane()
    )
    knurl2 = (cq.Workplane("YZ").copyWorkplane(end_cap)
        .circle(6.8/2).extrude(0.8)
        .copyWorkplane(end_cap)
        .polygon(4, 6.8 - 1.4 + 2*.3, forConstruction=True).vertices()
        .polygon(4, 1.4).extrude(1.1)
    )
    end_cap = (end_cap.solids().union(knurl2))
    battery_holder = battery_holder.union(end_cap)

    base_pos = (case2.faces("+Y and >Y").val().Center()
        + cq.Vector(0,battery_holder_radius,-0.8)
    )
    if False: # option 1 ("upside down")
        pass
    elif True: # option 2 ("in")
        battery_holder = (battery_holder
            .rotate((0,0,0),(1,0,0),90)
        )
    elif False: # option 3 ("out")
        battery_holder = (battery_holder
            .rotate((0,0,0),(1,0,0),-90)
        )
    else: # option 4 ("right side up")
        battery_holder = (battery_holder
            .rotate((0,0,0),(1,0,0),180)
        )
    battery_holder = battery_holder.translate(base_pos)
    show_object(battery_holder, name='10280')

if False:
    cnt = 1
    for pos in switch_positions:
        show_object(make_sw().translate(watchy_screen_center).translate(pos),
                    name = "Switch_"+str(cnt))
        cnt += 1
    # pcb for sw1-3
    pcb1 = (cq.Workplane("XY").workplane(invert=True)
            .center(0,-0.5).rect(22.5, 5).extrude(switch_pcb_thick)
            # rotate to match watchy orientation
            .rotate((0,0,0), (1,0,0), 180)
            .rotate((0,0,0), (0,0,1), 90)
    ).translate(watchy_screen_center).translate(switch_positions[1])
    show_object(pcb1, name='Switch_PCB_1')

    sw4 = sw4_translate(make_sw()
        .rotate((0,0,0),(0,0,1), 90)
        .rotate((0,0,0),(1,0,0),-90),
        switch_proud - switch_height
    )
    show_object(sw4, name='Switch_4')
    # pcb for sw4
    pcb2 = (cq.Workplane("XY").workplane(invert=True)
            .rect(5,4).extrude(switch_pcb_thick)
            # rotate to match watchy orientation
            .rotate((0,0,0), (1,0,0), 180)
            .rotate((0,0,0), (0,0,1), 90)
    )
    pcb2 = sw4_translate(pcb2
        .rotate((0,0,0),(0,0,1), 90)
        .rotate((0,0,0),(1,0,0),-90)
    )
    show_object(pcb2, name='Switch_PCB_2')
    # pcb3!
    pcb3 = (cq.Workplane("XY")
        .workplane(
            # "2" here is the thickness of the speaker (if it's on the bottom)
            offset=watchy_pcb_bottom.z + (case2_thick - case_wall_thick - 2),
            centerOption='ProjectedOrigin',
            origin=watchy_screen_center)
            # "1" here is because the PCB is 5mm centered in a 4mm spot
        .center(pcb3_position[0], pcb3_position[1] + 1)
        .rect(22.5, 5).extrude(switch_pcb_thick)
    )
    show_object(pcb3, name='Speaker_PCB')
