import sys
sys.stdout.reconfigure(encoding="utf-8", errors="replace")
import os, math
import cadquery as cq
from cadquery import exporters

OUT = os.path.dirname(os.path.abspath(__file__))

# ─── PARAMETERS ───────────────────────────────────────────
P = dict(
    D_OD          = 110.0,   # fuze maximum outer diameter [mm]
    D_base        = 60.0,    # fuze base diameter (meets the shell) [mm]
    D_nose        = 35.0,    # fuze blunt nose diameter [mm]
    T_wall        = 6.0,     # wall thickness [mm]
    D_thread      = 57.15,   # NATO thread major dia [mm]
    L_thread      = 22.0,    # thread stub length [mm]
    L_aft         = 62.0,    # aft body length [mm]
    L_rdba        = 18.0,    # RDBA housing height [mm]
    D_brg_OD      = 42.0,    # SKF 71905 ACE OD [mm]
    D_brg_ID      = 25.0,    # bearing bore [mm]
    D_hub_OD      = 110.0,   # canard hub OD (matched to body) [mm]
    D_hub_ID      = 43.0,    # hub inner bore [mm]
    L_hub         = 30.0,    # hub height [mm]
    D_motor       = 32.5,    # motor pocket dia [mm]
    L_motor       = 50.0,    # motor+HD pocket depth [mm]
    R_motor       = 44.0,    # motor radial offset [mm]
    span          = 80.0,    # canard half-span [mm]
    chord_root    = 40.0,    # root chord [mm]
    chord_tip     = 28.0,    # tip chord [mm]
    sweep_deg     = 5.0,     # LE sweep angle [deg]
    R_pivot       = 47.0,    # canard pivot radius [mm]
    L_fwd         = 65.0,    # forward body length [mm]
)

# Z-stack positions
P["z_thread"] = 0.0
P["z_aft"]    = P["z_thread"] + P["L_thread"]
P["z_rdba"]   = P["z_aft"]   + P["L_aft"]
P["z_hub"]    = P["z_rdba"]  + P["L_rdba"]
P["z_fwd"]    = P["z_hub"]   + P["L_hub"]
P["z_total"]  = P["z_fwd"]   + P["L_fwd"]

# ─── NACA 0010 PROFILE ─────────────────────────────────────
def naca0010(chord, n=24):
    def yt(xc):
        return 0.10*(0.2969*xc**0.5 - 0.1260*xc
                     - 0.3516*xc**2 + 0.2843*xc**3 - 0.1015*xc**4)
    xs = [i/(n-1) for i in range(n)]
    upper = [(chord*x,  chord*yt(x)) for x in xs]
    lower = [(chord*x, -chord*yt(x)) for x in reversed(xs[1:-1])]
    return upper + lower

# ─── PART: NATO THREAD STUB ───────────────────────────────
def make_thread_stub(p):
    stub = (cq.Workplane("XY")
            .cylinder(p["L_thread"], p["D_thread"]/2))
    for dz in [5, 11, 17]:
        groove = (cq.Workplane("XY")
                  .workplane(offset=dz)
                  .cylinder(1.5, p["D_thread"]/2 + 0.3))
        stub = stub.union(groove)
    return stub

# ─── PART: AFT BODY (TAPERED BASE) ────────────────────────
def make_aft_body(p):
    # Loft outer body
    body = (cq.Workplane("XY")
            .circle(p["D_base"]/2)
            .workplane(offset=p["L_aft"])
            .circle(p["D_OD"]/2)
            .loft())
    # Loft inner bore
    bore = (cq.Workplane("XY")
            .circle((p["D_base"] - 2*p["T_wall"])/2)
            .workplane(offset=p["L_aft"])
            .circle((p["D_OD"] - 2*p["T_wall"])/2)
            .loft())
    body = body.cut(bore)
    
    # Thread pocket (bottom)
    tp = (cq.Workplane("XY")
          .cylinder(18.0, p["D_thread"]/2 + 0.5))
    body = body.cut(tp)
    return body

# ─── PART: RDBA HOUSING ───────────────────────────────────
def make_rdba(p):
    h = (cq.Workplane("XY")
         .cylinder(p["L_rdba"], p["D_OD"]/2))
    bp = (cq.Workplane("XY")
          .cylinder(p["L_rdba"], p["D_brg_OD"]/2 + 0.05))
    h = h.cut(bp)
    tb = (cq.Workplane("XY")
          .cylinder(p["L_rdba"], p["D_brg_ID"]/2 - 1))
    h = h.cut(tb)
    return h

# ─── PART: CANARD HUB ─────────────────────────────────────
def make_canard_hub(p):
    hub = (cq.Workplane("XY")
           .cylinder(p["L_hub"], p["D_hub_OD"]/2))
    bore = (cq.Workplane("XY")
            .cylinder(p["L_hub"], p["D_hub_ID"]/2))
    hub = hub.cut(bore)
    
    mz = (p["L_hub"] - p["L_motor"]) / 2
    for a in [0, 90, 180, 270]:
        r = math.radians(a)
        mx = p["R_motor"] * math.cos(r)
        my = p["R_motor"] * math.sin(r)
        pocket = (cq.Workplane("XY")
                  .workplane(offset=mz)
                  .center(mx, my)
                  .cylinder(p["L_motor"], p["D_motor"]/2))
        hub = hub.cut(pocket)
        
    for a in [45, 135, 225, 315]:
        r = math.radians(a)
        px = p["R_pivot"]*math.cos(r)
        py = p["R_pivot"]*math.sin(r)
        pin = (cq.Workplane("XY")
               .center(px, py)
               .cylinder(p["L_hub"], 3.5))
        hub = hub.cut(pin)
    return hub

# ─── PART: CANARD BLADE ───────────────────────────────────
def make_canard_blade(p):
    root_pts = naca0010(p["chord_root"])
    tip_pts  = naca0010(p["chord_tip"])
    sw       = p["span"] * math.tan(math.radians(p["sweep_deg"]))

    def make_wire(pts, z_off, x_off=0.0):
        verts = [cq.Vector(x + x_off, y, z_off) for (x,y) in pts]
        verts.append(verts[0])
        return cq.Wire.makePolygon(verts)

    r_wire = make_wire(root_pts, 0.0,  0.0)
    t_wire = make_wire(tip_pts,  p["span"], sw)

    solid = cq.Solid.makeLoft([r_wire, t_wire])
    return cq.Workplane("XY").add(solid)

# ─── PART: FORWARD BODY (OGIVE NOSE) ──────────────────────
def make_fwd_body(p):
    # Loft outer body
    body = (cq.Workplane("XY")
            .circle(p["D_OD"]/2)
            .workplane(offset=p["L_fwd"])
            .circle(p["D_nose"]/2)
            .loft())
    # Loft inner bore (stopping slightly short of the tip for wall thickness)
    bore = (cq.Workplane("XY")
            .circle((p["D_OD"] - 2*p["T_wall"])/2)
            .workplane(offset=p["L_fwd"] - p["T_wall"])
            .circle((p["D_nose"] - 2*p["T_wall"])/2)
            .loft())
    body = body.cut(bore)
    
    # GPS antenna recess on the flat nose tip
    gps_recess = (cq.Workplane("XY")
                  .workplane(offset=p["L_fwd"])
                  .rect(18, 18)
                  .extrude(-2))
    body = body.cut(gps_recess)

    # 4× canard deployment slots hugging the outside
    for a in [45, 135, 225, 315]:
        slot = (cq.Workplane("XY")
                .rect(40, 8)  # 8mm wide slot
                .extrude(p["L_fwd"])
                .translate((p["D_OD"]/2, 0, 0)) # push out to edge
                .rotate((0,0,0),(0,0,1), a))
        body = body.cut(slot)
    return body

# ─── BUILD ─────────────────────────────────────────────────
print("YIL PGK-155 — CadQuery CAD Builder  Rev 1.2")
print("=" * 50)

parts = {}
parts["thread"] = make_thread_stub(P)
parts["aft"]    = make_aft_body(P)
parts["rdba"]   = make_rdba(P)
parts["hub"]    = make_canard_hub(P)
parts["fwd"]    = make_fwd_body(P)

try:
    parts["blade"] = make_canard_blade(P)
except Exception as e:
    print(f"Fallback to extruded blade: {e}")
    rpts = naca0010(P["chord_root"])
    parts["blade"] = cq.Workplane("XY").polyline(rpts).close().extrude(P["span"])

# ─── ASSEMBLIES ────────────────────────────────────────────
z = P

def create_base_assembly():
    assy = cq.Assembly()
    assy.add(parts["thread"].translate((0,0,z["z_thread"])), name="thread", color=cq.Color(0.5,0.5,0.5))
    assy.add(parts["aft"].translate((0,0,z["z_aft"])),       name="aft",    color=cq.Color(0.8,0.8,0.8))
    assy.add(parts["rdba"].translate((0,0,z["z_rdba"])),     name="rdba",   color=cq.Color(0.3,0.3,0.3))
    assy.add(parts["hub"].translate((0,0,z["z_hub"])),       name="hub",    color=cq.Color(0.9,0.6,0.1))
    assy.add(parts["fwd"].translate((0,0,z["z_fwd"])),       name="fwd",    color=cq.Color(0.4,0.4,0.4))
    return assy

assy_deployed = create_base_assembly()
assy_stowed   = create_base_assembly()

# Add blades
for i, a_deg in enumerate([45, 135, 225, 315]):
    a_rad = math.radians(a_deg)
    
    # Base blade has root at X=0, span along +Z, chord along +X
    # Shift so aerodynamic center (X=10) is at the origin (pivot)
    base_blade = parts["blade"].translate((-10, 0, 0))

    # --- DEPLOYED: Rotate outwards ---
    # +90 around Y points the +Z span axis towards +X (radially outward)
    blade_dep = (
        base_blade
        .rotate((0,0,0),(0,1,0), 90)     # Deploy outward
        .rotate((0,0,0),(0,0,1), a_deg)  # Azimuth
        .translate((
            P["R_pivot"]*math.cos(a_rad),
            P["R_pivot"]*math.sin(a_rad),
            z["z_hub"] + P["L_hub"]/2
        ))
    )
    assy_deployed.add(blade_dep, name=f"canard_dep_{i+1}", color=cq.Color(0.7, 0.9, 1.0, 0.9))

    # --- STOWED: Lying longitudinally in slots ---
    # Span stays along +Z (points forward into the nose cone slots)
    blade_stow = (
        base_blade
        .rotate((0,0,0),(0,0,1), a_deg)  # Azimuth
        .translate((
            P["R_pivot"]*math.cos(a_rad),
            P["R_pivot"]*math.sin(a_rad),
            z["z_hub"] + P["L_hub"]/2
        ))
    )
    assy_stowed.add(blade_stow, name=f"canard_stow_{i+1}", color=cq.Color(0.7, 0.9, 1.0, 0.9))


# ─── EXPORT ────────────────────────────────────────────────
print("\n  Exporting files ...")

def export_step(obj, filename):
    path = os.path.join(OUT, filename)
    exporters.export(obj.toCompound() if isinstance(obj, cq.Assembly) else obj, path)
    print(f"  [OK] {filename}")

export_step(assy_deployed, "PGK_assembly_deployed.step")
export_step(assy_stowed,   "PGK_assembly_stowed.step")

print("\n  Complete! Generated Deployed and Stowed assemblies.")
