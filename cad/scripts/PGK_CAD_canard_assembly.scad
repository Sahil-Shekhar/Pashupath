// ============================================================
//  YIL PGK-155  — Canard Actuation Assembly
//  OpenSCAD Parametric Model
//  Document: YIL-PGK-155-CAD-001  Rev 1.0   2026-09-26
//  Units: mm   Tolerance: ±0.05 mm (CNC machined)
// ============================================================
//
//  ASSEMBLY BREAKDOWN (bottom-up, muzzle = +Z)
//  1. NATO thread stub         (aft)
//  2. S&A / aft body           110 mm OD cylinder
//  3. RDBA bearing housing     despins guidance section
//  4. Canard hub               rotates with guidance section
//     4a. 4× canard arm pivots (BLDC + HD)
//     4b. 4× NACA-0010 canard blades
//  5. Fuze forward body        GNSS + PCB + IMU bay
//
//  To render:   OpenSCAD → F6 (Render) → Export STL
//  To preview:  OpenSCAD → F5 (Preview)
// ============================================================

/* ── GLOBAL PARAMETERS ───────────────────────────────────── */
$fn = 80;                    // circle segments (quality)

// --- Shell / Fuze body ---
D_shell        = 155.0;     // mm — shell OD
D_fuze_OD      = 110.0;     // mm — PGK body OD (fits inside fuze pocket)
D_fuze_ID      = 98.0;      // mm — internal bore
T_wall         = 6.0;       // mm — wall thickness

// --- NATO thread stub (aft face) ---
D_thread_major = 57.15;     // mm — 2-inch 12-UN-2A (2.250" = 57.15 mm)
D_thread_minor = 55.32;     // mm — minor diameter
L_thread       = 22.0;      // mm — engagement length

// --- Aft body (battery + S&A + ordnance) ---
L_aft          = 62.0;      // mm — aft body length

// --- RDBA bearing housing ---
L_rdba         = 18.0;      // mm — housing height
D_bearing_OD   = 42.0;      // mm — SKF 71905 ACE OD
D_bearing_ID   = 25.0;      // mm — bearing bore
T_bearing_wall = 4.5;       // mm — housing wall

// --- Canard hub ---
D_hub_OD       = 94.0;      // mm — hub outer diameter
D_hub_ID       = 42.0;      // mm — fits over RDBA outer race
L_hub          = 30.0;      // mm — hub height

// --- BLDC motor housing (×4, at 0/90/180/270 deg) ---
D_motor_OD     = 32.0;      // mm — Maxon EC-i 30 OD
L_motor        = 38.0;      // mm — motor + HD stack height
R_motor_center = 44.0;      // mm — radial offset from assembly CL

// --- Harmonic drive drum ---
D_hd_OD        = 32.0;      // mm — matches motor OD
L_hd           = 12.0;      // mm — HD drum height

// --- Canard arm (NACA 0010 approximation, deployed) ---
Span           = 80.0;      // mm — half-span from hub edge
Chord_root     = 40.0;      // mm — root chord
Chord_tip      = 28.0;      // mm — tip chord (taper ratio 0.70)
T_canard       = 1.2;       // mm — sheet thickness (Ti-6Al-4V)
Sweep_deg      = 5.0;       // degrees — leading-edge sweep
R_canard_pivot = 47.0;      // mm — pivot radius from CL

// --- Fuze forward body (GNSS + PCB + IMU) ---
L_fwd          = 65.0;      // mm — forward body length

// --- Total assembly height ---
Z_thread_bot   = 0;
Z_aft_bot      = L_thread;
Z_rdba_bot     = Z_aft_bot + L_aft;
Z_hub_bot      = Z_rdba_bot + L_rdba;
Z_fwd_bot      = Z_hub_bot + L_hub;
Z_total        = Z_fwd_bot + L_fwd;  // ~197 mm total

/* ── NACA 0010 PROFILE (normalised, 12-point approximation) ─ */
// t/c = 0.10, points in (x, y) with x in [0..1]
function naca0010_y(x) =
    0.10 * (  0.2969*sqrt(x)
            - 0.1260*x
            - 0.3516*x*x
            + 0.2843*x*x*x
            - 0.1015*x*x*x*x );

// Precomputed table for polygon (16 stations)
N_stations = 16;
function x_station(i) = i / (N_stations - 1);

function naca_upper(c, i) =
    [ c * x_station(i),  c * naca0010_y(x_station(i)) ];
function naca_lower(c, i) =
    [ c * x_station(N_stations-1-i), -c * naca0010_y(x_station(N_stations-1-i)) ];

// Build closed profile polygon at chord c
function naca_profile(c) =
    [for (i=[0:N_stations-1]) naca_upper(c, i),
     for (i=[0:N_stations-1]) naca_lower(c, i)];

/* ── MODULES ─────────────────────────────────────────────── */

// 1) NATO 2-inch 12-UN-2A threaded stub (simplified as cylinder with knurl)
module nato_thread_stub() {
    color("gray") {
        cylinder(h=L_thread, d=D_thread_major);
        // Knurl ring representation
        for (a=[0:30:330])
            rotate([0,0,a])
            translate([D_thread_major/2 - 0.5, -0.5, 0])
            cube([1, 1, L_thread]);
    }
}

// 2) Aft body (battery bay + S&A + ordnance section)
module aft_body() {
    color("silver") difference() {
        // Outer shell
        cylinder(h=L_aft, d=D_fuze_OD);
        // Internal bore
        translate([0,0,-1]) cylinder(h=L_aft+2, d=D_fuze_ID);
        // Ventilation / inspection ports (×4)
        for (a=[45,135,225,315])
            rotate([0,0,a])
            translate([D_fuze_OD/2-3, 0, L_aft*0.55])
            rotate([0,90,0]) cylinder(h=8, d=6);
        // Thread engagement pocket at bottom
        translate([0,0,-1]) cylinder(h=L_thread+2, d=D_thread_major+0.2);
    }
}

// 3) RDBA (Rolling De-Spin Bearing Assembly) housing
module rdba_housing() {
    color("darkgray") difference() {
        // Housing outer body
        cylinder(h=L_rdba, d=D_fuze_OD);
        // Bore for bearing OD
        translate([0,0,-1])
            cylinder(h=L_rdba+2, d=D_bearing_OD + 1.0);
        // Central throughbore
        translate([0,0,-1])
            cylinder(h=L_rdba+2, d=D_bearing_ID - 1.0);
    }
    // Inner race representation (spins with shell body)
    color("lightblue") {
        difference() {
            cylinder(h=L_rdba, d=D_bearing_OD);
            translate([0,0,-1]) cylinder(h=L_rdba+2, d=D_bearing_ID);
        }
    }
    // Bearing balls (represented as torus slice)
    color("gold") translate([0,0,L_rdba/2])
        rotate_extrude(angle=360)
        translate([(D_bearing_OD + D_bearing_ID)/4, 0, 0])
        circle(d=2.5);
}

// 4a) Single BLDC motor + Harmonic Drive housing
module motor_hd_assy() {
    // BLDC motor body
    color("steelblue") cylinder(h=L_motor, d=D_motor_OD);
    // HD drum on top
    color("dimgray") translate([0,0,L_motor])
        cylinder(h=L_hd, d=D_hd_OD);
    // Output shaft stub
    color("silver") translate([0,0,L_motor+L_hd])
        cylinder(h=6, d=6);
}

// 4b) Single canard blade (NACA 0010, tapered, swept)
module canard_blade() {
    color("lightcyan", 0.9) {
        // Sweep offset at tip: tan(5°) × Span ≈ 7 mm
        tip_sweep = Span * tan(Sweep_deg * PI / 180);
        hull() {
            // Root cross-section (full chord, at pivot)
            translate([0, 0, 0])
                linear_extrude(height=T_canard)
                polygon(naca_profile(Chord_root));
            // Tip cross-section (tapered, swept)
            translate([tip_sweep, Span, 0])
                linear_extrude(height=T_canard)
                polygon(naca_profile(Chord_tip));
        }
    }
}

// 4) Canard hub with 4× motor+HD + 4× canard blades
module canard_hub() {
    // Hub ring body
    color("orange") difference() {
        cylinder(h=L_hub, d=D_hub_OD);
        translate([0,0,-1]) cylinder(h=L_hub+2, d=D_hub_ID);
        // 4× motor pockets at 0/90/180/270°
        for (a=[0,90,180,270])
            rotate([0,0,a])
            translate([R_motor_center, 0, (L_hub - L_motor - L_hd)/2])
            cylinder(h=L_motor+L_hd+5, d=D_motor_OD+0.5);
    }
    // 4× Motor + HD assemblies
    for (a=[0,90,180,270])
        rotate([0,0,a])
        translate([R_motor_center, 0, (L_hub - L_motor - L_hd)/2])
        motor_hd_assy();
    // 4× Canard blades (deployed, perpendicular to body axis)
    // Canards rotate 90° about their span axis into deployed position
    for (a=[0,90,180,270])
        rotate([0,0,a + 45])          // blades stagger 45° from motors
        translate([R_canard_pivot, 0, L_hub/2])
        rotate([0, -90, 90])          // rotate blade to be perpendicular to body
        translate([-Chord_root*0.25, 0, 0])  // align pivot at 25% chord (aero centre)
        canard_blade();
}

// 5) Forward body (GNSS bay + PCB + IMU)
module fwd_body() {
    color("dimgray") difference() {
        cylinder(h=L_fwd, d=D_fuze_OD);
        translate([0,0,-1]) cylinder(h=L_fwd+2, d=D_fuze_ID);
        // GPS antenna window (flat on top)
        translate([-15, D_fuze_OD/2 - 3, L_fwd*0.75])
            cube([30, 8, 20]);
        // 4× canard deployment slots (split lines)
        for (a=[45,135,225,315])
            rotate([0,0,a])
            translate([-2, D_fuze_OD/2-4, -1])
            cube([4, 8, L_fwd+2]);
    }
    // GPS antenna patch representation
    color("white", 0.8)
    translate([0, D_fuze_OD/2 - 1, L_fwd*0.75])
    rotate([90,0,0])
    linear_extrude(height=2)
    square([28, 18], center=true);
    // IMU shock-mount representation inside
    color("yellow", 0.5)
    translate([0, 0, L_fwd*0.4])
    cylinder(h=15, d=18);
}

/* ── FULL ASSEMBLY (exploded = false for assembled view) ─── */
exploded = false;
explode_gap = exploded ? 8 : 0;

// Render full assembly
translate([0, 0, Z_thread_bot])   nato_thread_stub();
translate([0, 0, Z_aft_bot + 0*explode_gap])   aft_body();
translate([0, 0, Z_rdba_bot + 1*explode_gap])   rdba_housing();
translate([0, 0, Z_hub_bot  + 2*explode_gap])   canard_hub();
translate([0, 0, Z_fwd_bot  + 3*explode_gap])   fwd_body();

/* ── CROSS-SECTION VIEW (uncomment to see internal detail) ──
difference() {
    union() {
        translate([0,0,Z_aft_bot])  aft_body();
        translate([0,0,Z_rdba_bot]) rdba_housing();
        translate([0,0,Z_hub_bot])  canard_hub();
        translate([0,0,Z_fwd_bot])  fwd_body();
    }
    translate([-200, 0, -10]) cube([200, 400, 300]);
}
*/

/* ── ANNOTATION (echo to console) ──────────────────────── */
echo("=== YIL PGK-155 Assembly Dimensions ===");
echo(str("Total height: ", Z_total, " mm"));
echo(str("Fuze OD: ", D_fuze_OD, " mm"));
echo(str("Thread: 2-inch 12-UN-2A, major dia=", D_thread_major, " mm"));
echo(str("Canard span (deployed): ", 2 * Span + D_fuze_OD, " mm (tip-to-tip)"));
echo(str("Canard chord (root): ", Chord_root, " mm"));
echo(str("BLDC motor pockets: ", R_motor_center, " mm from CL"));
echo("=========================================");
