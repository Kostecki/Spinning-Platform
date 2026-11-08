// Spinning Platform V2 — case
// Kitchen-scale stack: base → load cell (fixed end on a floor post) → floating deck
// (motor hangs below it, bearing on top) → platform.
//
// Origin = motor shaft / platform centre, on the table. +Y = north (USB/LED/pot wall), Z up.
// PCB coordinates convert as X = x_kicad - 150, Y = 150 - y_kicad.
//
// Render one part:  openscad -D 'part="base"' -o base.stl spinning-platform.scad
// part = "base" | "deck" | "riser" | "platform" | "fit_cell" | "inside" | "assembly"
//
// Assembly order:
//  1. load cell fixed end → base post (screws from under the floor)
//  2. riser → cell free end (countersunk screws from the top of the riser)
//  3. on the bench: motor → deck; bearing bottom plate → deck (turn the top plate 45° to
//     reach the corner holes); platform → bearing top plate through the access hole
//  4. drop the deck sub-assembly into the base, 2 screws through the deck into the riser
//     (outside the bearing footprint, so always reachable)
//  Pot: mount with its wire/lug side pointing WEST (towards the LED) — ~15 mm free there,
//  only ~4.5 mm towards the HX711 (east), ~6.5 down, ~5.5 up.

part = "assembly";
$fn = 64;
eps = 0.01;

// ---------------------------------------------------------------- measured / chosen
// Case
base_outer   = 110;   // square
base_r       = 4;     // corner radius
wall         = 2.5;
floor_t      = 2.0;

// Heights (Z from table)
pcb_bottom_z = 5.0;   // on 3 mm standoffs
pcb_t        = 1.6;
pcb_top_z    = pcb_bottom_z + pcb_t;      // 6.6
deck_t       = 3.0;
deck_top_z   = 34.0;                      // base rim is flush with this
deck_bot_z   = deck_top_z - deck_t;       // 31
deck_gap     = 2.0;                       // deck ↔ wall, all round: must never touch
bearing_h    = 8.0;                       // lazy-susan, 75 mm square
bearing_sq   = 75.0;
platform_d   = 112;                       // covers the bearing's 106 mm corner sweep
platform_t   = 4.0;

// Load cell (Soldered 5 kg bar) — TODO measure screw size / hole spacing on arrival
cell_l = 80; cell_w = 12.7; cell_h = 12.7;
cell_y = -14.35;                          // centre line (from PCB)
cell_x0 = -40;                            // west (fixed) end
riser_h    = 6.0;                         // riser block between cell free end and deck
cell_top_z = deck_bot_z - riser_h;        // 25
cell_bot_z = cell_top_z - cell_h;         // 12.3 (5.7 above the PCB)
cell_hole_d      = 5.5;                   // clearance for M5 (TODO: M4 → 4.5)
cell_hole_end    = 5.0;                   // first hole centre from the end (TODO)
cell_hole_pitch  = 15.0;                  // between the two holes at one end (TODO)
cell_mount_len   = 15.0;                  // support length at each end

// Motor 28BYJ-48: body Ø28, ears Ø4.2 holes 35 mm apart on the body centre line,
// shaft offset 8 mm from body centre. Body centre is north of the shaft.
motor_body_d = 28.2; motor_body_h = 19; motor_off = 8;
motor_ear_pitch = 35; motor_ear_hole = 4.4;
motor_boss_d = 9.5;                       // round boss around the shaft on the top face
shaft_d = 5.0; shaft_flat = 3.0; shaft_len = 9.5; // above the top face (flats ~6 mm)

// Bearing holes — TODO from the real part. Bottom plate → deck, top plate → platform,
// access hole in the bottom plate (and the deck) to reach the top-plate screws.
bearing_bottom_holes = [[-31.5,-31.5],[31.5,-31.5],[31.5,31.5],[-31.5,31.5]];  // TODO
bearing_top_holes    = [[-24,0],[24,0],[0,24],[0,-24]];                        // TODO
bearing_access       = [0,-28];  bearing_access_d = 8;                         // TODO
bearing_screw_pilot  = 2.5;     // M3 thread-forming into plastic

// Riser block: sits on the cell's free end, extends east past it; deck screws into the extension
riser_x1    = 48;               // east end (cell ends at x=40)
riser_w     = 16;               // Y width of the extension
deck_screws = [[44, -14.35 - 4.5],[44, -14.35 + 4.5]];  // M3, outside the 75 mm bearing

// PCB (from KiCad)
pcb_mount = [[-47.5,25],[48.5,24.5],[-47.5,-47.5],[47.5,-47.5]];
post_cut  = [[-39,-21.3],[-24.5,-7.5]];   // PCB cutout for the fixed-end post (X/Y min,max)
stop_xy   = [34,-14.35];                  // overload stop under the free end
stop_gap  = 0.5;                          // cell ↔ stop screw head

// Wall parts on the north wall (Y = +inner), centred on the visible wall height
wall_mid_z = deck_top_z/2;   // 17
usb_x = -38; usb_z = wall_mid_z;  usb_cut = [13,3.5]; usb_flange = [20.1,7]; usb_flange_t = 1.6;
usb_hole_d = 2.3; usb_hole_pitch = 16.1;
led_x = -16; led_z = wall_mid_z;  led_hole = 5.14; led_wall = 4.5;  led_boss_d = 12;
pot_x = 14;  pot_z = wall_mid_z;  pot_hole = 7.0;  pot_wall = 4.0;  pot_boss_d = 16;
pot_body_d = 17;  pot_depth = 11;   // behind the wall (diameter assumed — typical switched pot)
usb_depth = 12;
usb_nut_clear_d = 5.5;   // M2 nut (4 mm AF) + spinner room, inside the wall
// HX711 (Soldered 333005, 22×22) upright on two posts, holes 16 mm apart, 3 mm in from its sides
hx_x0 = 27; hx_bottom_z = 4; hx_standoff = 3.5; hx_post_d = 6.5; hx_screw = 2.6;

// ---------------------------------------------------------------- sanity checks (render fails if violated)
// Gap between two features along the north wall, given centres and diameters/widths
function wall_gap(x1, w1, x2, w2) = abs(x1 - x2) - (w1 + w2)/2;
usb_east_hole = usb_x + usb_hole_pitch/2;
usb_west_hole = usb_x - usb_hole_pitch/2;
assert(wall_gap(usb_east_hole, usb_nut_clear_d, led_x, led_boss_d) >= 1, "LED boss collides with the USB-C screw/nut");
assert(wall_gap(usb_x, usb_flange[0], led_x, led_boss_d) >= 1, "LED boss overlaps the USB-C flange area");
assert(wall_gap(led_x, led_boss_d, pot_x, pot_boss_d) >= 1, "LED and pot bosses collide");
assert(usb_west_hole - usb_nut_clear_d/2 > -inner/2 + 1, "USB-C screw too close to the west wall");
assert(pot_x + pot_boss_d/2 < hx_x0 - 1, "pot boss runs into the HX711 posts");
// behind the wall: the PCB starts at Y=37, so parts must end before it (strip is free of PCB)
pcb_north_y = 37;
assert(inner/2 - pot_depth > pcb_north_y + 1, "pot body reaches over the PCB");
assert(inner/2 - usb_depth > pcb_north_y + 1, "USB-C board reaches over the PCB");
assert(pot_z - pot_body_d/2 > floor_t + 1, "pot body hits the floor");
assert(pot_z + pot_body_d/2 < deck_bot_z - 1, "pot body hits the deck");

// ---------------------------------------------------------------- helpers
inner = base_outer - 2*wall;
module rsq(s, r, h) { linear_extrude(h) offset(r) square(s - 2*r, center=true); }
module cyl_at(xy, d, z0, h) { translate([xy[0], xy[1], z0]) cylinder(d=d, h=h); }

// ---------------------------------------------------------------- base
module base() {
  difference() {
    union() {
      difference() {
        rsq(base_outer, base_r, deck_top_z);
        translate([0,0,floor_t]) rsq(inner, max(base_r-wall,0.5), deck_top_z);
      }
      // PCB standoffs
      for (p = pcb_mount) cyl_at(p, 7, 0, pcb_bottom_z);
      // fixed-end post: solid block under the west end of the cell
      translate([cell_x0, cell_y - cell_w/2, 0]) cube([cell_mount_len, cell_w, cell_bot_z]);
      // overload-stop boss (an M3 screw threads in from the top; set the head 0.5 mm under the cell)
      cyl_at(stop_xy, 7.5, 0, cell_bot_z - stop_gap - 3 - 4);
      // inside thickening for the LED and pot so their holders sit flush
      north_boss(led_x, led_z, led_boss_d, led_wall);
      north_boss(pot_x, pot_z, pot_boss_d, pot_wall);
      // HX711 posts
      for (dx = [3, 19]) translate([hx_x0 + dx, inner/2 - hx_standoff, hx_bottom_z + 3])
        rotate([-90,0,0]) cylinder(d=hx_post_d, h=hx_standoff + eps);
    }
    // PCB standoff screw holes (M3 thread-forming)
    for (p = pcb_mount) cyl_at(p, 2.5, floor_t, 10);
    // fixed-end post: screws up into the cell, heads recessed under the floor
    for (i = [0,1]) translate([cell_x0 + cell_hole_end + i*cell_hole_pitch, cell_y, -eps]) {
      cylinder(d=cell_hole_d, h=cell_bot_z + 1);
      cylinder(d=cell_hole_d + 4.5, h=3);
    }
    // overload stop: M3 thread-forming hole
    cyl_at(stop_xy, 2.5, 2, 40);
    // USB-C panel socket: through cut + 1.6 mm outside pocket + 2 screw holes
    translate([usb_x, base_outer/2, usb_z]) rotate([90,0,0]) {
      translate([0,0,-1]) cube([usb_cut[0], usb_cut[1], 20], center=true);
      translate([0,0,-eps]) linear_extrude(usb_flange_t + eps) offset(0.15) square(usb_flange, center=true);
      for (s=[-1,1]) translate([s*usb_hole_pitch/2, 0, -1]) cylinder(d=usb_hole_d, h=10);
    }
    north_hole(led_x, led_z, led_hole);
    north_hole(pot_x, pot_z, pot_hole);
    // HX711 post screw holes
    for (dx = [3, 19]) translate([hx_x0 + dx, inner/2 - hx_standoff - 1, hx_bottom_z + 3])
      rotate([-90,0,0]) cylinder(d=hx_screw, h=hx_standoff + 2);
  }
}
module north_boss(x, z, d, t) {
  translate([x, inner/2 + eps, z]) rotate([90,0,0]) cylinder(d=d, h=t - wall + eps);
}
module north_hole(x, z, d) {
  translate([x, base_outer/2 + 1, z]) rotate([90,0,0]) cylinder(d=d, h=10);
}

// ---------------------------------------------------------------- floating deck
deck_sq = inner - 2*deck_gap;
module deck() {
  difference() {
    translate([0,0,deck_bot_z]) rsq(deck_sq, 3, deck_t);
    // deck → riser: M3 countersunk from the top, outside the bearing
    for (p = deck_screws) translate([p[0], p[1], 0]) {
      cylinder(d=3.4, h=50);
      translate([0,0,deck_top_z - 1.8]) cylinder(d1=3.4, d2=6.6, h=1.8 + eps);
    }
    // access hole lined up with the bearing's bottom-plate access hole
    translate([bearing_access[0], bearing_access[1], 0]) cylinder(d=bearing_access_d + 1, h=50);
    // motor: shaft boss + ears (M4, countersunk from the top)
    cylinder(d=motor_boss_d + 1.5, h=50);
    for (s=[-1,1]) translate([s*motor_ear_pitch/2, motor_off, 0]) {
      cylinder(d=motor_ear_hole, h=50);
      translate([0,0,deck_top_z - 2.2]) cylinder(d1=motor_ear_hole, d2=motor_ear_hole + 4.4, h=2.2 + eps);
    }
    // bearing bottom plate: M3 thread-forming pilots
    for (p = bearing_bottom_holes) translate([p[0], p[1], 0]) cylinder(d=bearing_screw_pilot, h=50);
  }
}

// ---------------------------------------------------------------- riser
module riser() {
  x0 = cell_x0 + cell_l - cell_mount_len;
  difference() {
    union() {
      translate([x0, cell_y - cell_w/2, cell_top_z]) cube([cell_mount_len, cell_w, riser_h]);
      translate([cell_x0 + cell_l - eps, cell_y - riser_w/2, cell_top_z]) cube([riser_x1 - cell_x0 - cell_l, riser_w, riser_h]);
    }
    // into the cell: countersunk from the riser top (covered by the deck afterwards)
    for (i = [0,1]) translate([cell_x0 + cell_l - cell_hole_end - i*cell_hole_pitch, cell_y, cell_top_z - 1]) {
      cylinder(d=cell_hole_d, h=riser_h + 2);
      translate([0,0,riser_h + 1 - 2.8]) cylinder(d1=cell_hole_d, d2=cell_hole_d + 5.6, h=2.8 + eps);
    }
    // deck screws: M3 thread-forming pilots
    for (p = deck_screws) translate([p[0], p[1], cell_top_z + 1]) cylinder(d=2.5, h=riser_h);
  }
}

// ---------------------------------------------------------------- platform + hub
platform_z = deck_top_z + bearing_h;
module platform() {
  difference() {
    union() {
      translate([0,0,platform_z]) cylinder(d=platform_d, h=platform_t, $fn=180);
      // hub reaching down through the bearing onto the motor shaft
      translate([0,0,deck_top_z + 1]) cylinder(d=12, h=platform_z - deck_top_z - 1 + eps);
    }
    // bearing top plate: M3 thread-forming pilots, blind (3.2 mm deep)
    for (p = bearing_top_holes) translate([p[0], p[1], platform_z - eps]) cylinder(d=bearing_screw_pilot, h=3.2);
    // D-shaft: Ø5 with 3 mm across the flats, small clearance
    translate([0,0,deck_top_z + 1 - eps]) intersection() {
      cylinder(d=shaft_d + 0.25, h=shaft_len + 2);
      translate([-(shaft_flat+0.25)/2, -5, 0]) cube([shaft_flat + 0.25, 10, shaft_len + 2]);
    }
  }
}

// ---------------------------------------------------------------- fit test: cell mount + post only
module fit_cell() {
  intersection() {
    base();
    translate([cell_x0 - 5, cell_y - 12, -1]) cube([cell_mount_len + 10, 24, cell_bot_z + 2]);
  }
}

// ---------------------------------------------------------------- reference parts (assembly view)
module ref_parts() {
  color("green", 0.6) translate([0,0,pcb_bottom_z]) linear_extrude(pcb_t) difference() {
    translate([-51.5,-51.5]) square([103, 88.5]);
    translate(post_cut[0]) square(post_cut[1] - post_cut[0]);
    translate(stop_xy) circle(d=9);
  }
  color("silver") translate([cell_x0, cell_y - cell_w/2, cell_bot_z]) cube([cell_l, cell_w, cell_h]);
  color("gold") translate([0, motor_off, deck_bot_z - motor_body_h]) cylinder(d=motor_body_d, h=motor_body_h);
  color("gray") translate([0, 0, deck_top_z]) linear_extrude(bearing_h) difference() {
    square(bearing_sq, center=true); circle(d=45);
  }
  color("dodgerblue") translate([-43, -48.5, pcb_top_z]) cube([83, 25, 23]);   // 21700 in holder
  color("black") translate([pot_x, inner/2 - pot_wall + wall, pot_z]) rotate([90,0,0]) cylinder(d=pot_body_d, h=pot_depth);
  color("darkslategray") translate([usb_x, inner/2, usb_z]) rotate([90,0,0]) translate([0,0,usb_depth/2]) cube([usb_flange[0], usb_flange[1], usb_depth], center=true);
}

// ---------------------------------------------------------------- select
if (part == "base") base();
else if (part == "deck") deck();
else if (part == "riser") riser();
else if (part == "platform") platform();
else if (part == "fit_cell") fit_cell();
else if (part == "inside") { color("white", 0.9) base(); color("red") riser(); ref_parts(); }
else {
  color("white", 0.9) base();
  color("orange", 0.9) deck();
  color("red") riser();
  color("purple", 0.8) platform();
  ref_parts();
}
