import cadquery as cq
from math import atan2, degrees, radians

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
#show_object(make_battery('10280'), name='10280')

def make_battery_clip():
    shell_thick = 0.2
    taper_hack = 4
    sphere_plane = cq.Workplane("XZ").workplane(offset=0).center(0,6.55)
    #debug(sphere)
    side_tab = (
        cq.Workplane("XY").workplane(offset=shell_thick*3/2)
        .center(0,9.47 - (6.35/2))
        .rect(7.52 - 2*shell_thick, 6.35)
        .extrude(12.66 - shell_thick*3/2)
        .faces("-Y and <Y")
        .workplane(centerOption='ProjectedOrigin', origin=(0,0,0), invert=True)
        .moveTo(0,-6.55)
        .circle(10/2).extrude(6.35)
        .faces("<Z").edges("|Y").fillet(0.5)
        .faces("(+Z and >Z) or (+Y and >Y) or (-Y and <Y)")
        .shell(shell_thick, kind='intersection')
    )
    end_tab = (
        cq.Workplane("XZ").workplane(offset=-shell_thick)
        .center(0, 12.66/2 + shell_thick/4)
        .rect(6.35, 12.66 - shell_thick/2).extrude(shell_thick)
        .union(sphere_plane.sphere(2.50/2))
        .cut(sphere_plane.sphere(2.50/2 - shell_thick))
        .cut(cq.Workplane("XZ").rect(100,100).extrude(100))
        # sphere is 6.55mm up, and wants to be 2.5mm/2 back
        .rotate((0,0,shell_thick/2),(1,0,shell_thick/2), degrees(atan2(2.5/2,6.55-(shell_thick/2))))
        .union(cq.Workplane("XY")
               .workplane(offset=shell_thick/2)
               .center(0, 9.47/2)
               .rect(6.35, 9.47)
               .extrude(shell_thick)
        )
    )
    tab_overlap = 0.75
    solder_tab = (
        cq.Workplane("XY")
        .moveTo(0, -3.61/2 + tab_overlap/2)
        .rect(2.34, 3.61 + tab_overlap)
        .extrude(shell_thick)
        .moveTo(0, -3.61/2)
        .circle(1.2/2)
        .cutThruAll()
        .moveTo(0, 9.47 + 3.96/2 - tab_overlap/2)
        .rect(5.08, 3.96 + tab_overlap)
        .extrude(shell_thick)
        .moveTo(0, 9.47 + 3.96/2)
        .circle(2.34/2)
        .cutThruAll()
    )
    clip = (side_tab
        .union(end_tab)
        .faces("<Z[-2]").edges("|X and <Y").fillet(0.5)
        .faces("<Z").edges("|X and <Y").fillet(0.5+shell_thick)
        .union(solder_tab)
    )
    return clip

#show_object(make_battery_clip(), name='clip')

def make_battery_holder(with_battery=True):
    clips = make_battery_clip()
    clips = (clips.union(
        clips.mirror(mirrorPlane="XZ", basePointVector=(0,14.08,0))
        )
        .translate((0,-14.08,0.6))
    )
    pcb = (cq.Workplane("XY")
        # PCB
        .rect(5.5, 37).extrude(0.6)
    )
    clips = clips.union(pcb)
    if with_battery:
        battery = (make_battery('10280')
            .translate((-14,0,6.55 + 0.6))
            .rotate((0,0,0),(0,0,1),90)
        )
        clips = clips.union(battery)
    return clips

#show_object(make_battery_holder(), name='battery-holder')
