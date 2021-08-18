function watchy_board_width() = 34; // XXX check this
function shelf_height() = 0.75; // how much space underneath
function wiring_space() = 1; // how much space we need for the i2c wires
function scd40_offset() = -1.25;

module centercube(xyz=[10,10,10],x=false,y=false,z=false) {
    translate([x==true?-xyz[0]/2:0,y==true?-xyz[1]/2:0,z==true?-xyz[2]/2:0]) cube(xyz);
}

module chamfercube(xyz=[10,10,10],x=false,y=false,z=false,chamfer=1, hack=0, only_cube=false, only_chamfer=false) {
    if (only_cube) {
        centercube(xyz, x=x, y=y, z=z);
    } else if (only_chamfer) {
        chamfer(xyz, x=x, y=y, z=z, chamfer=chamfer, hack=hack);
    } else {
        difference() {
            centercube(xyz, x=x, y=y, z=z);
            chamfer(xyz, x=x, y=y, z=z, chamfer=chamfer, hack=hack);
        }
    }
}
module chamfer(xyz=[10,10,10],x=false,y=false,z=false,chamfer=1,hack=0) {
    epsilon=1;
    translate([x==true?-xyz[0]/2:0,y==true?-xyz[1]/2:0,z==true?-xyz[2]/2:0])
    translate([xyz[0]/2, xyz[1]/2, xyz[2]/2]) {
        for (i=[1,-1]) for (j=[1,-1]) {
            // four edges in the x direction
            if (hack==0 || (hack == 1 && i==1 && j==-1))
            scale([1,i,j])
                    translate([0,-xyz[1]/2,xyz[2]/2])
                    rotate([45,0,0])
                    cube([xyz[0]+epsilon,max(xyz[1],xyz[2]),chamfer*sqrt(2)], center=true);
            // four edges in the y direction
            if (hack==0 || (hack==2 && i==1 && j == -1))
            scale([i,1,j])
                    translate([-xyz[0]/2,0,xyz[2]/2])
                    rotate([0,-45,0])
                    cube([max(xyz[0],xyz[2]), xyz[1]+epsilon, chamfer*sqrt(2)], center=true);
            // four edges in the z direction
            if (hack==0)
            scale([i,j,1])
                    translate([-xyz[0]/2,-xyz[1]/2,0])
                    rotate([0,0,45])
                    cube([chamfer*sqrt(2), max(xyz[0], xyz[1]), xyz[2]+epsilon], center=true);
        }
    }
    // xxx chamfer the corners
}

module scd40(with_wiring=false, simplify=false) {
    epsilon = .01;
    translate([watchy_board_width()/2,scd40_offset(),1.55-wiring_space()-shelf_height()]) {
        if (simplify) {
            centercube([10.1, 10.1, 0.8], x=true, y=true);
            translate([0,0,-5.5]) linear_extrude(height=5.5 + epsilon, scale=8.5/7.8)
            square([7.8,7.8],center=true);
        } else {
            rotate([180,0,0])
                import("Sensirion_CO2_Sensors_SCD4x_STEP_file.stl", convexity=5);
        }
        if (with_wiring) {
            overlap=.05;
            translate([0,0,0.8+wiring_space()/2-overlap])
            cube([10.1,10.1,wiring_space()+2*overlap], center=true);
        }
    }
}
module watchy() {
    color("brown")
    import("Watchy_Battery.stl");
}
module case_bottom() {
    color("grey")
    import("Armadillonium_Bottom.stl");
}
module case_top() {
    color("grey")
    import("Armadillonium_Top.stl", convexity=10);
}

module scd40_with_clearance(clearance=0.5) {
    minkowski() {
        scd40(with_wiring=true, simplify=true);
        translate([-clearance, -clearance, -clearance]) {
            cube([clearance*2, clearance*2, clearance]);
        }
    }
}

module extra_bump(case_bottom=0, chamfer=0, stub_height=0, only_cube=false, only_chamfer=false) {
    ysize=13;
    xsize=12;
    translate([watchy_board_width()/2, scd40_offset(), case_bottom-stub_height]) {
        chamfercube([xsize,ysize,stub_height], x=true, y=true, chamfer=chamfer, hack=1, only_cube=only_cube, only_chamfer=only_chamfer);
        for (i=[1,-1]) scale([i,1,1])
        translate([-xsize/2,-ysize/2]) rotate([0,0,45])
        chamfercube([max(xsize,ysize),max(xsize,ysize)-2/*fudge*/,stub_height], top=false, chamfer=chamfer, hack=2, only_cube=only_cube, only_chamfer=only_chamfer);
    }
}

module new_case_top() {
    case_bottom = 2.385;
    stub_height = 4.435;
    chamfer=1.6;
    color("grey")
    difference() {
        union() {
            case_top();
            difference() {
                extra_bump(case_bottom=case_bottom, chamfer=chamfer, stub_height=stub_height, only_cube=true);
                extra_bump(case_bottom=case_bottom, chamfer=chamfer, stub_height=stub_height, only_chamfer=true);
                translate([watchy_board_width()/2, -.1, -.47])
                     centercube([27,15,10], x=true);
            }
        }
        scd40_with_clearance();
    }
}

module everything(new_top=true) {
    scd40();
    watchy();
    case_bottom();
    if (new_top) {
        new_case_top();
    } else {
        case_top();
    }
}

everything(true);
//new_case_top();
//scd40();
