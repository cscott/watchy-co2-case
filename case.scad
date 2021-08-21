function watchy_board_width() = 33.8; // XXX check this
function shelf_height() = 1.00; // how much space underneath
function wiring_space() = 0.6; // how much space we need for the i2c wires
function scd40_vertical_clear() = 0.3; // includes space below pcb
function scd40_horizontal_clear() = 0.5;
function air_space() = 1; // how much air space we need above the device
function grill_thick() = 1.0; // minimum wire thickness on the grill
function scd40_offset() = -.5; // this is pretty much all clearance
function scd_translate() = [
    // for the "only redo the top case" version, the 5.9 here used to be 1.55
    watchy_board_width()/2, scd40_offset(), 5.9-wiring_space()-shelf_height()
];

function case_top_height() = 4.435;
function case_split_offset() = 2.385;
function case_bottom_height() = 4.315;
function chamfer_amount() = 1.6;
function epsilon() = .01;

module centercube(xyz=[10,10,10],x=false,y=false,z=false) {
    translate([x==true?-xyz[0]/2:0,y==true?-xyz[1]/2:0,z==true?-xyz[2]/2:0]) cube(xyz);
}

module chamfercube(xyz=[10,10,10],x=false,y=false,z=false,chamfer=chamfer_amount(), hack=0, only_cube=false, only_chamfer=false) {
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

module roundcube(xyz=[10,10,10],x=false,y=false,z=false,chamfer=chamfer_amount(),hack=0,$fn=48) {
    translate([x==false?xyz[0]/2:0,y==false?xyz[1]/2:0,z==false?xyz[2]/2:0]) {
        cube(xyz-[0,0,2*chamfer], center=true);
	cube(xyz-[(hack==1?0:2)*chamfer,(hack==2?0:2)*chamfer,0], center=true);
        for(i=[1,-1]) {
            if (hack==1) scale([1,i,1])
                translate([0,xyz[1]/2-chamfer,xyz[2]/2-chamfer])
                rotate([0,90,0])
                cylinder(r=chamfer,h=xyz[0]-(hack==1?0:2)*chamfer,center=true);
            if (hack==2) scale([i,1,1])
                translate([xyz[0]/2-chamfer,0,xyz[2]/2-chamfer])
                rotate([90,0,0])
                cylinder(r=chamfer,h=xyz[1]-(hack==2?0:2)*chamfer,center=true);
           }
    }
}

module roundcube2(xyz=[10,10,10],x=false,y=false,z=false,chamfer=chamfer_amount(), $fn=48) {
    c = chamfer/xyz[0];
    translate([x==false?xyz[0]/2:0,y==false?xyz[1]/2:0,z==false?xyz[2]/2:0])
        scale(xyz) {
        cube([1-2*c,1-2*c,1], center=true);
        cube([1-2*c,1,1-2*c], center=true);
        cube([1,1-2*c,1-2*c], center=true);
        for(i=[1,-1]) for(j=[1,-1]) {
                for(k=[1,-1]) translate([i*(.5-c),j*(.5-c),k*(.5-c)]) sphere(r=c);
                translate([i*(.5-c),j*(.5-c),0]) cylinder(r=c, h=1-2*c, center=true);
                translate([i*(.5-c),0,j*(.5-c)]) rotate([90,0,0]) cylinder(r=c, h=1-2*c, center=true);
                translate([0,i*(.5-c),j*(.5-c)]) rotate([0,90,0]) cylinder(r=c, h=1-2*c, center=true);
        }
    }
}

module chamfer(xyz=[10,10,10],x=false,y=false,z=false,chamfer=chamfer_amount(),hack=0) {
    epsilon=1;
    translate([x==true?-xyz[0]/2:0,y==true?-xyz[1]/2:0,z==true?-xyz[2]/2:0])
    translate([xyz[0]/2, xyz[1]/2, xyz[2]/2]) {
        for (i=[1,-1]) for (j=[1,-1]) {
            // four edges in the x direction
            if (hack==0 || (hack == 1 && i==1) || (hack == 3 && j==1))
            scale([1,i,j])
                    translate([0,-xyz[1]/2,xyz[2]/2])
                    rotate([45,0,0])
                    cube([xyz[0]+epsilon,max(xyz[1],xyz[2]),chamfer*sqrt(2)], center=true);
            // four edges in the y direction
            if (hack==0 || (hack==2 && i==1) || (hack==3 && j==1))
            scale([i,1,j])
                    translate([-xyz[0]/2,0,xyz[2]/2])
                    rotate([0,-45,0])
                    cube([max(xyz[0],xyz[2]), xyz[1]+epsilon, chamfer*sqrt(2)], center=true);
            // four edges in the z direction
            if (hack==0 || (hack==3 && j==1))
            scale([i,j,1])
                    translate([-xyz[0]/2,-xyz[1]/2,0])
                    rotate([0,0,45])
                    cube([chamfer*sqrt(2), max(xyz[0], xyz[1]), xyz[2]+epsilon], center=true);
        }
    }
    // xxx chamfer the corners
}

module splitplane(top=false, bottom=false) {
    translate([-15,-40,case_split_offset()]) scale([1,1,bottom?1:-1])
        cube([100,100,100]);
}

module scd40_pcb_outline() {
    width=10.3; length=17;
    translate([-width/2,-width/2])
    square([width,length]);
}

module scd40(with_wiring=false, simplify=false) {
    translate(scd_translate()) {
        if (simplify) {
            centercube([10.1, 10.1, 0.8], x=true, y=true);
            translate([0,0,-5.5]) linear_extrude(height=5.5 + epsilon(), scale=8.5/7.8)
            square([7.8,7.8],center=true);
        } else {
            rotate([180,0,0])
                import("Sensirion_CO2_Sensors_SCD4x_STEP_file.stl", convexity=5);
        }
        if (with_wiring) {
            translate([0,0,0.8+wiring_space()/2-epsilon()])
            cube([10.1,10.1,wiring_space()+2*epsilon()], center=true);
        } else {
            pcb_thick=wiring_space();
            translate([0,0,0.8])
            linear_extrude(height=pcb_thick) scd40_pcb_outline();
        }
    }
}

module scd40_pcb_space(clearance=0.75, yclearance=0.5) {
    big_height = 10;
    big_tab = 10;
    translate(scd_translate()) {
        translate([0,big_tab/2,0.8 - big_height])
        centercube([10.3 + 2*clearance, 10.3 + 2*yclearance + big_tab, big_height + wiring_space()], x=true, y=true);
    }
}

module scd40_vent(lid=false) {
    translate(scd_translate()) scale([1,1,-1]) translate([0,0,4.5 + (lid?epsilon():0)]) {
        scd40_vent1(lid=lid);
    }
    min_wall_thick = 1.0;
    cutoff = 5;
    if (lid) {
        difference() {
        minkowski() {
        translate(scd_translate()) {
            translate([0,0,-5.5]) linear_extrude(height=5.5, scale=8.5/7.8)
            square([7.8,7.8], center=true);
        }
        cube([
                 (scd40_horizontal_clear() + min_wall_thick)*2,
                 (scd40_horizontal_clear() + min_wall_thick)*2,
                 (scd40_vertical_clear() + min_wall_thick)*2
        ], center=true);
        }
        translate([0,0,-0.6])
        centercube([100,100,100],x=true,y=true);
        }
    }
}
module scd40_vent2(lid=false) {
    big_height = 10;
    big_diam = 7*1.5;
    strut=1.5;
    if (lid) {
        translate([0,0,.25])
        chamfercube([big_diam+2*strut,10,.75+epsilon()+air_space()+grill_thick()],x=true,y=true,chamfer=1.15,hack=3);
        // xxx round instead?
    } else difference() {
        centercube([big_diam,7,big_height],x=true,y=true);
        translate([0,0,1+air_space()]) for (i=[-3:3]) {
            *translate([0,2*i*strut,0])
            centercube([big_diam+1,strut,big_height], x=true, y=true);
            translate([2*i*strut,0])
            centercube([strut,big_diam+1,big_height], x=true, y=true);
        }
    }
}
module scd40_vent1(lid=false) {
    big_height = 10;
    strut=1.5;
    small_diam = 3.5;
    big_diam = 7.5;
    if (lid) {
        translate([0,0,.25])
        rounded_cylinder(d=big_diam+2*strut,h=0.75+epsilon()+air_space()+grill_thick(), fillet=.75, $fn=48);
    } else difference() {
        cylinder(d=big_diam, h=big_height, $fn=24);
        //centercube([big_diam,big_diam,big_height],x=true,y=true);
        translate([0,0,1+air_space()]) union() {
            cylinder(d=small_diam, h=big_height, $fn=24);
            centercube([big_diam+1,strut,big_height], x=true, y=true);
            centercube([strut,big_diam+1,big_height], x=true, y=true);
        }
    }
}

module watchy() {
    //color("brown")
    import("Watchy_Battery.stl");
}

module case_bottom() {
    //color("grey")
    //import("Armadillonium_Bottom-4.3mf", convexity=10);
    import("Armadillonium_Bottom-11.stl", convexity=10);
}

module case_top() {
    color("grey")
    import("Armadillonium_Top.stl", convexity=10);
}

module scd40_with_clearance() {
    h = scd40_horizontal_clear();
    v = scd40_vertical_clear();
    minkowski() {
        scd40(with_wiring=true, simplify=true);
        translate([-h, -h, -v]) {
            cube([h*2, h*2, v]);
        }
    }
}

module screw_holes(fill_in=false,$fn=24) {
    extra_diam = 1;
    extra_height = .005;
    d=1.6 + (fill_in ? extra_diam : 0);
    h=10.1 + 2*extra_height;
    translate([watchy_board_width()/2,3,0])
        for (i=[1,-1]) scale([i,1,1]) for (j=[0,1]) {
                translate([13,j*40,11.70+extra_height-h+(fill_in?0:-1)]) cylinder(d=d, h=h+(fill_in?0:2));
            }
}

module chamroundcube(xyz=[10,10,10],x=false,y=false,z=false,chamfer=chamfer_amount(),hack=0,only_cube=false, only_chamfer=false, round=false) {
    if (round) {
        roundcube(xyz=xyz,x=x,y=y,z=z,chamfer=chamfer,hack=hack);
    } else {
        chamfercube(xyz=xyz,x=x,y=y,z=z,chamfer=chamfer,hack=hack,only_cube=only_cube,only_chamfer=only_chamfer);
    }
}

module extra_bump(only_cube=false, only_chamfer=false, round=false, round2=false, round3=false, extra_chamfer=0) {
    ysize=14.05;
    xsize=14;
    angle=55.081;
    fudge=3.3;
    cheat=(round2||round3)?3:0;
    translate([watchy_board_width()/2, scd40_offset(), case_split_offset()-case_top_height()]) {
        if (!round3) difference() {
        chamroundcube([xsize,ysize,case_top_height()+case_bottom_height()], x=true, y=true, chamfer=chamfer_amount()+extra_chamfer, hack=1, only_cube=only_cube, only_chamfer=only_chamfer, round=round);
        if (round2) {
            translate([0,-ysize/2+chamfer_amount()+extra_chamfer,case_top_height()+case_bottom_height()-chamfer_amount()-extra_chamfer])
            rotate([0,90,0])
                cylinder(r=chamfer_amount()+extra_chamfer, h=xsize, center=true, $fn=24);
        }
        }
        for (i=[1,-1]) scale([i,1,1])
        translate([-xsize/2,-ysize/2]) rotate([0,0,angle])
        translate([0,-cheat,0])
        chamroundcube([max(xsize,ysize),max(xsize,ysize)+cheat-fudge,case_top_height()+case_bottom_height()], chamfer=chamfer_amount()+extra_chamfer, hack=2, only_cube=only_cube, only_chamfer=only_chamfer, round=round);
    }
}

module new_case_top() {
    color("grey")
    difference() {
        union() {
            case_top();
            scd40_vent(lid=true);
            difference() {
                // cheat the walls just enough so front is solid
                hollow(1.21) difference() {
                extra_bump(only_cube=true);
                extra_bump(only_chamfer=true);
                scd40_with_clearance();
                }
                translate([watchy_board_width()/2, -.1, -.47])
                     centercube([40,15,10], x=true);
                translate([watchy_board_width()/2, -.1, -.47])
                for (i=[1,-1]) scale([i,1,1])
                translate([13,0,0]) centercube([10, 15, 10], x=true, z=true);
                splitplane(bottom=true);
            }
        }
        scd40_with_clearance();
        scd40_vent();
    }
}

module erode(wall_size=1) {
    minkowski() {
        difference() {
            cube([100,100,100],center=true);
            children();
        }
        //sphere(r=wall_size, $fn=16);
        cube(2*wall_size, center=true);
    }
}

module hollow_space(wall_size=1) {
    difference() {
        children();
        erode(wall_size=wall_size) children();
    }
}

module hollow(wall_size=1) {
    intersection() {
        children();
        erode(wall_size=wall_size) children();
    }
}

module new_case_bottom() {
    blind_screw_holes=true;
    extra_chamfer=.3;
    difference() {
        union() {
            case_bottom();
            if (!blind_screw_holes) screw_holes(fill_in=true);
	    extra=.1;
            difference() {
                hollow(1+extra) difference() {
                extra_bump(only_cube=true);
                extra_bump(only_chamfer=true, extra_chamfer=extra_chamfer, round3=true);
                scd40_pcb_space(clearance=0.75-extra, yclearance=0.5-extra);
                }
                translate([watchy_board_width()/2, -.1, -2.47])
                     centercube([40,15,15], x=true);
                translate([watchy_board_width()/2, -.1, -2.47])
                for (i=[1,-1]) scale([i,1,1])
                translate([13,0,0]) centercube([10, 15, 15], x=true);
		translate([0,0,0*epsilon()/2])
                splitplane(top=true);
            }
        }
        scd40_pcb_space(clearance=0.75, yclearance=0.5);
        if (!blind_screw_holes) screw_holes();
        // extra hollowing out to save material costs
	wall=1.25;
	translate([watchy_board_width()/2,23,0])
        centercube([34-wall*2,36 - wall*2,6.7 - wall], x=true, y=true);
    }
}

module rounded_cylinder(d=10, h=10, fillet=1, $fn=48) {
cylinder(d=d, h=h-fillet);
cylinder(d=d-2*fillet,h=h);
translate([0,0,h-fillet])
rotate_extrude(convexity = 10)
translate([d/2 - fillet, 0, 0])
circle(r = fillet);
}

module everything(new_parts=true) {
    scd40();
    watchy();
    if (new_parts) {
        new_case_bottom();
    } else {
        case_bottom();
    }
    if (new_parts) {
        new_case_top();
    } else {
        case_top();
    }
}

module main(part="everything", with_scd40=false, with_watchy=false) {
    if (part=="everything") {
        everything(true);
    } else if (part=="top") {
        new_case_top();
    } else if (part=="bottom") {
        new_case_bottom();
    } else if (part=="pcb_outline") {
        scd40_pcb_outline();
    } else if (part=="debug_vent") {
        rotate([180,0,0]) translate(-scd_translate()) {
            scd40(with_air=true);
            //scd40_vent();
            #scd40_pcb_space();
        }
    }
    if (with_scd40) {
        scd40();
    }
    if (with_watchy) {
        watchy();
    }
}

//main("bottom", with_scd40=true);
//main("pcb_outline");
//main("bottom");
//main();
//scd40();

main("bottom", with_watchy=false, with_scd40=false);
*watchy();
