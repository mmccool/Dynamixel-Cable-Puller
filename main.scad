// Dynamixel Cable Puller
// Developed by: Michael McCool
// Copyright 2017 Intel Corporation
include <tols.scad>
include <smooth_model.scad>
//include <smooth_make.scad>
include <bolt_params.scad>
use <bolts.scad>
use <MCAD/involute_gears.scad>

sm = 10*sm_base;

// Robotis Dynamixel AX-12/18A servo
servo_h = 32;
servo_w = 32;
servo_l = 50;
servo_l2 = 36;
servo_h_xo = servo_w/2;
servo_h_yo = servo_h/2;
servo_h_r = 25/2;
servo_h_rr = 23/2;
servo_h_h = 5;
servo_b_h = 3;
servo_ww = 20;
servo_bw = 3;
servo_ll = 32+tol;

bracket_h = 5.5;
bracket_hole_spacing = 8;
bracket_hole_r = m2_hole_radius;
bracket_base_x = 4*bracket_hole_spacing + 2;
bracket_base_y = 3*bracket_hole_spacing + 1;
bracket_t = 3;
bracket_s = 1;
bracket_tt = 0.5;
bracket_cs = 1;
bracket_cr = bracket_hole_r + 1 + cut_t;
bracket_rr = (bracket_base_y - 2*bracket_hole_spacing)/2;
bracket_hh = bracket_h + 0.8;
bracket_tweak_x = 0.5;
bracket_tweak_y = 0.25;
bracket_hole_sm = sm;
bracket_o = servo_l2+bracket_t+4;

standoff_h = 70;
plate_t = 5;

module servo(a=0) {
  color([0.3,0.3,0.7,1.0])
    translate([0,0,-servo_h/2-servo_h_h])
      scale(10)
        rotate([0,0,a])
          import("External/servo.stl",convexity=5);
}

use_bracket_model = 1;
module bracket() {
  if (!use_bracket_model || printed) {
    //color([0.5,0.4,0.4,0.2]) import("External/F3.stl",convexity=5);
    difference() {
      // outer shell
      hull() {
        translate([-bracket_base_y/2,0,-bracket_base_x/2])
          cube([bracket_base_y,bracket_t,bracket_base_x]);
        intersection() {
          translate([-bracket_base_y/2,
                     bracket_t-cut_t,
                     -servo_w/2-bracket_t])
            cube([bracket_base_y,
                  bracket_h+bracket_tt,
                  servo_w+2*bracket_t]);
          union() {
            translate([bracket_hole_spacing,
                       bracket_h,
                       -servo_w/2-bracket_t])
              cylinder(r=bracket_rr,
                       h=servo_w+2*bracket_t,
                       $fn=bracket_sm);
            translate([-bracket_hole_spacing,
                       bracket_h,
                       -servo_w/2-bracket_t])
              cylinder(r=bracket_rr,
                       h=servo_w+2*bracket_t,
                       $fn=bracket_sm);
          }
        }
       }
       // servo mount surface
       translate([0,bracket_t-cut_t,0]) hull() {
        translate([-bracket_base_y/2-eps,
                   0,
                   -servo_w/2+bracket_s-cut_t])
          cube([bracket_base_y+2*eps,
                bracket_s,
                servo_w-2*bracket_s+2*cut_t]);
        translate([-bracket_base_y/2-bracket_t,
                   bracket_s,
                   -servo_w/2-cut_t])
          cube([bracket_base_y+2*bracket_t,
                bracket_h,
                servo_w+2*cut_t]);
       }
       // bolt holes
       translate([bracket_hole_spacing,bracket_h,-bracket_base_y])
          cylinder(r=bracket_hole_r,
                   h=2*bracket_base_y,
                   $fn=bracket_hole_sm);
       translate([-bracket_hole_spacing,bracket_h,-bracket_base_y])
          cylinder(r=bracket_hole_r,
                   h=2*bracket_base_y,
                   $fn=bracket_hole_sm);
       // countersinks for bolt heads
       translate([bracket_hole_spacing,
                  bracket_h,
                  servo_w/2+bracket_t-bracket_cs])
          cylinder(r=bracket_cr,
                   h=servo_w,
                   $fn=bracket_hole_sm);
       translate([-bracket_hole_spacing,
                  bracket_h,
                  servo_w/2+bracket_t-bracket_cs])
          cylinder(r=bracket_cr,
                   h=servo_w,
                   $fn=bracket_hole_sm);
       translate([bracket_hole_spacing,
                  bracket_h,
                  -3*servo_w/2-bracket_t+bracket_cs])
          cylinder(r=bracket_cr,
                   h=servo_w,
                   $fn=bracket_hole_sm);
       translate([-bracket_hole_spacing,
                  bracket_h,
                  -3*servo_w/2-bracket_t+bracket_cs])
          cylinder(r=bracket_cr,
                   h=servo_w,
                   $fn=bracket_hole_sm);
     }
  } else {
    import("External/F3.stl",convexity=5);
  }
}
module end_bracket(a=0) {
  color([0.5,0.3,0.3,0.5])
  rotate([0,0,a-90])
    translate([0,servo_l2+bracket_h+0.5,-servo_h/2-servo_h_h-0.25])
      rotate([0,0,180])
        bracket();
}

plate_x = 75;
plate_y = 50;
plate_oy = -10;
standoff_r = m4_nut_radius;
standoff_hr = m4_hole_radius;
standoff_oy = -25;
standoff_ox = 20;
standoff_hole_sm = 2*sm;
module plate() {
  color([0.6,0.6,0.3,0.5]) {
    translate([-standoff_ox,standoff_oy,0])
      cylinder(r=standoff_r,h=standoff_h,$fn=6);
    translate([ standoff_ox,standoff_oy,0])
      cylinder(r=standoff_r,h=standoff_h,$fn=6);
  }
  difference() {
    color([0.3,0.3,0.3,0.25]) union() {
      translate([-plate_x/2,plate_oy-plate_y,-plate_t]) 
        cube([plate_x,plate_y,plate_t]);
      translate([-plate_x/2,plate_oy-plate_y,standoff_h]) 
        cube([plate_x,plate_y,plate_t]);
    }
    translate([-standoff_ox,standoff_oy,-plate_t-1])
      cylinder(r=standoff_hr,h=standoff_h+2*plate_t+2,$fn=standoff_hole_sm);
    translate([ standoff_ox,standoff_oy,-plate_t-1])
      cylinder(r=standoff_hr,h=standoff_h+2*plate_t+2,$fn=standoff_hole_sm);
  }
}

cable_r = 1.2/2;
puller_r = 25*cable_r; // needs to be at least 10x cable diameter 
puller_t = 5;
puller_sm = 8*sm;
puller_ox = 0;
puller_oy = 0.5;
puller_oz = standoff_h;
puller_sr = 4/2;
puller_gr = 35/2;
puller_goy = puller_oy+puller_t;
puller_n = 31;

drive_t = 5;
drive_ox = 0;
drive_oy = drive_t;
drive_oz = bracket_o;
drive_sm = 8*sm;
drive_r = 20/2;
drive_n = 15;

pitch = 210;

module puller() {
  // puller cylinder
  translate([puller_ox,puller_oy,puller_oz]) 
    rotate([90,0,0]) 
       cylinder(r=puller_r,h=puller_t,$fn=puller_sm);
  // puller drive gear
  translate([puller_ox,puller_goy,puller_oz]) 
    rotate([90,0,0]) 
       // cylinder(r=puller_gr,h=puller_t,$fn=puller_sm);
      linear_extrude(puller_t) rotate(90) gear(
        number_of_teeth=puller_n,
        circular_pitch=pitch,
        pressure_angle=28,
        clearance = 0.2,
        gear_thickness=drive_t,
        involute_facets=0,
        bore_diameter=0,
        flat=true);
}

module drive() {
  // drive gear
  translate([drive_ox,drive_oy,drive_oz]) 
    rotate([90,0,0]) 
      // cylinder(r=drive_r,h=drive_t,$fn=drive_sm);
      linear_extrude(drive_t) rotate(90) gear(
        number_of_teeth=drive_n,
        circular_pitch=pitch,
        pressure_angle=28,
        clearance = 0.2,
        gear_thickness=drive_t,
        involute_facets=0,
        bore_diameter=0,
        flat=true);
}

module parts() {
  puller();
  drive();
}

module assembly() {
  translate([0,0,bracket_o]) rotate([0,90,90]) {
    servo();
    end_bracket();
  }
  parts();
  plate();
}

parts();
// assembly();
