// Dynamixel Cable Puller
// Developed by: Michael McCool
// Copyright 2017 Intel Corporation
include <tols.scad>
include <smooth_model.scad>
//include <smooth_make.scad>
include <bolt_params.scad>
use <bolts.scad>
use <r200.scad>
include <r200_params.scad>

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

module assembly() {
  translate([0,0,servo_l2+bracket_t+4]) rotate([0,90,90]) {
    servo();
    end_bracket();
  }
  plate();
}

assembly();
