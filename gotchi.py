import cadquery as cq
import numpy as np
from math import atan2, degrees, radians, cos, sin, tan
from watchy_sizes import *

watchy_board_width = 33.8

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
screen_clearance = 0.5
top_plate_thick = 1.5
pcb_support_radius = 2
m2_tap_diam = 1.6 # XXX
m2_clear_diam = 2 # XXX
m2_head_clear_diam = 2.5 # XXX
m2_head_thick = 1
mag_support_depth = 5
mag_angle2 = 35 # top angle
mag_angle1 = 20 # bottom angle; 0 is right angle
bottom_fillet = 1 # should be less than case_wall_thick
mag_depth_cheat = 0.85

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
#debug(watchy, name="watchy_screen")
watchy_screen_center = watchy.val().Center()

# find the top of the PCB
watchy = (watchy
    .faces("-Z", tag="watchy").faces("<<Z[-6]")
    .tag("watchy_pcb_top")
)
#debug(watchy, name="watchy_pcb_top")
watchy_pcb_top = watchy.val().Center()

watchy = (watchy
    .faces("+Z", tag="watchy").faces("<<Z[-4]")
    .tag("watchy_pcb_bottom")
)
watchy_pcb_bb = watchy.val().BoundingBox()
#debug(watchy, name="watchy_pcb_bottom")
watchy_pcb_bottom = watchy.val().Center()

# find a good "entire PCB face" by splitting the object in the middle of the PCB
#watchy_pcb = (watchy
#    .workplaneFromTagged("watchy_screen")
#    .workplane(offset=-1.6)
#    .split(keepBottom=True)
#    .faces("-Z").faces("<<Z")
#)
#debug(watchy_pcb)

show_object(watchy.solids(tag="watchy"), name='watchy')

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

# Strap Lugs
# Originally from: github:giladaya/watchy-case-lugs (thanks!)
p_strap_width = 22 + 0.5 # strap width inc. tolerance
p_strap_dia = 4.0 # diameter of strap edge
p_tbar_hole_r = 0.5 # Radius of t-bar pin

# hack hack hack (these were computed originally, need to be altered for us)
p_outerLength = 39
p_outerHeight = 10
p_inset_depth = 1

tbar_hole_depth = 1.5
lugs_th = 2.5
lugs_width = p_strap_width + 2.0 * lugs_th
lugs_dia = 5.0
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
)
# rotate & translate giladaya's lugs to match our case orientation
lugs = (lugs
    .rotateAboutCenter((1,0,0), 180)
    .rotateAboutCenter((0,0,1), 90)
    .translate(watchy_screen_center)
)
lugs = (lugs
    .translate((0,0,watchy_pcb_top.z - lugs.solids().val().BoundingBox().zmin))
)
#show_object(lugs, name='lugs')

# magnetic connector
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

# watchy_pcb_bb: xlen=33.8 ylen=37.96
case_wall_clear = 0.7
case_wall_thick = 1.5
case_depth = watchy_bb.zmax - watchy_pcb_top.z + case_wall_clear
case = (cq.Workplane("XY")
    .workplane(offset=watchy_pcb_top.z,
               centerOption='ProjectedOrigin', origin=watchy_pcb_bb.center)
    .tag("case_bottom_top")
    .rect(watchy_pcb_bb.xlen + 2*case_wall_clear,
          watchy_pcb_bb.ylen + 2*case_wall_clear)
    .extrude(case_depth, combine=False)
    .faces("<<Z").shell(case_wall_thick)
    .union(lugs)
#    .workplaneFromTagged("case_bottom_top")
#    .rect(watchy_pcb_bb.xlen + 2*case_wall_clear,
#          watchy_pcb_bb.ylen + 2*case_wall_clear)
#    .cutBlind(watchy_bb.zmax - watchy_pcb_top.z + case_wall_clear)
#    #.cut
)
#show_object(case, name='case')

def make_switch_plate(obj):
    return (obj
    .line(-faceplate_height/2, -switch_plate_width/2 - switch_plate_offset)
    .line(-switch_plate_depth, switch_plate_offset)
    .line(0, switch_plate_width)
    .line(switch_plate_depth, switch_plate_offset)
    .close()
    )

case2_thick = watchy_bb.zmax + case_wall_thick - watchy_pcb_bottom.z
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
def make_mag_support():
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
    return (case2
    .workplaneFromTagged("case2_bottom_top")
    .moveTo(top_screw.x, top_screw.y)
    .lineTo(top_point.x, top_point.y)
    .lineTo(top_point.x - top_delta_x, target_y)
    .lineTo(bot_point.x + bot_delta_x, target_y)
    .lineTo(bot_point.x, bot_point.y)
    .lineTo(bot_screw.x, bot_screw.y)
    .close()
    .extrude(case2_thick, combine=False)
    )
mag_support = (make_mag_support()
    .faces("-Y and <<Y")
)
#debug(mag_support)
# find center of the Y face
mag_support_bb = mag_support.val().BoundingBox()
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
show_object(make_mag().translate(mag_translate), name='magnetic_latch')
#debug(mag_support)
case2 = (case2
    .union(mag_support)
)
# now add switch support
case2_switch = (make_switch_plate(
    case2
    .workplaneFromTagged("case2_bottom_top")
    .center(0,faceplate_center_shift))
    .extrude(case2_thick, combine=False)
    .faces("<X").edges("|Z").fillet(switch_plate_fillet)
)
#debug(case2_switch)
#debug(case2.faces("<X").edges("|Z"));
case2 = (case2
    .union(case2_switch)
    .tag("case2_body_prelim")
    .faces("<Z").tag("case2_split_face")
)
case2_top = (case2
    .workplaneFromTagged("case2_bottom_top")
    .faces(tag="case2_split_face").wires().toPending()
    .extrude(-(watchy_pcb_bottom.z - watchy_screen_center.z + screen_clearance),
             combine=False)
)
case2_mag_shield = (case2
    .workplaneFromTagged("case2_bottom_top")
    .workplane(centerOption='ProjectedOrigin', origin=mag_translate)
    .center(0,-5)
    .rect(23,10).extrude(4, combine=False) # should be 4.5
    .faces("-Z").edges("|Y").fillet(3.5)
)
#debug(case2_mag_shield)
case2_top = (case2_top
    #.union(case2_mag_shield) # XXX seems to hang?
    #.union(make_mag(0.6).translate(mag_translate))
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
    .tag("case2_body")
    # cut out space for the PCB
    .workplaneFromTagged("case2_bottom_top")
    .workplane(centerOption='ProjectedOrigin', origin=watchy_pcb_bb.center)
    .rect(watchy_pcb_bb.xlen + 2*case_wall_clear,
          watchy_pcb_bb.ylen + 2*case_wall_clear)
    .extrude(watchy_bb.zmax - watchy_pcb_bottom.z + case_wall_clear,
             combine=False).tag("pcb_cutout")
    .edges("|Z", tag="pcb_cutout")
    .chamfer(case_wall_clear + 2.7).tag("pcb_cutout2")
)
case2 = case2.solids(tag="case2_body").cut(case2.solids(tag="pcb_cutout2"))
case2 = (case2
    .workplaneFromTagged("case2_bottom_top")
    .rect(screw_separation_h,
          screw_separation_w, forConstruction=True)
    .vertices().circle(m2_tap_diam/2).cutThruAll()
#    .rect(watchy_pcb_bb.xlen, watchy_pcb_bb.ylen, forConstruction=True)
#    .vertices().circle(pcb_support_radius).cut()
#    .solids(tag="case2_body").cut(
)
show_object(case2, name='case2')

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
    # fillet the edges between screw bosses and body
    # XXX we're going to do this below
    #.faces("-X or +X or -Y or +Y").edges("+Z").fillet(0.75)
)
    # extrude switch plate.  A hull would be the easy way to do this,
    # and loft works, but creates tiny faces
top_plate = (make_switch_plate(
    top_plate
    .workplaneFromTagged("top_plate")
    .center(0,-faceplate_center_shift))
    .tag("switch_plate")
    .extrude(top_plate_thick)
    .faces("not(+Z or -Z)").edges("+Z or -Z").fillet(switch_plate_fillet)
)
top_plate = (top_plate
    # cut out screen
    .workplaneFromTagged("top_plate")
    .center(0,-faceplate_center_shift)
    .rect(30,30).cutThruAll(taper=45) # taper is in degrees
    # add to other other part of top case
    .union(case2_top)
    # cut out mag connector
    .cut(make_mag(extra=5).translate(mag_translate))
    .cut(make_mag(extra=5).translate(mag_translate).shell(0.25))
    # countersink the screw holes
    .workplaneFromTagged("top_plate")
    .faces(tag="top_plate_top").workplane(centerOption='ProjectedOrigin')
    .rect(screw_separation_h, screw_separation_w, forConstruction=True)
    .vertices()
    .cboreHole(m2_clear_diam, m2_head_clear_diam, m2_head_thick)
)
show_object(top_plate, name='top_plate')

show_object(make_battery('10280', xOffset=3, yOffset=50, zOffset=1), name='10280')
