import cadquery as cq
import numpy as np
from watchy_sizes import *

watchy_board_width = 33.8

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
show_object(lugs, name='lugs')

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
show_object(case, name='case')

show_object(make_battery('10280', xOffset=3, yOffset=50, zOffset=1), name='10280')
