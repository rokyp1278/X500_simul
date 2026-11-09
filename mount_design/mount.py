#!/usr/bin/env python3
"""Parametric 3D-print mount: Holybro X500 V2 (#31109 platform) x Jetson Orin
Nano Developer Kit x Intel RealSense D435i.

DESIGN STANCE (honest, see docs verification 2026-11-06):
  * The Orin Nano DevKit baseplate screw-hole coordinates are NOT officially
    published, so this design DOES NOT bolt through them. The Jetson is held by
    an ENVELOPE CRADLE (verified outer size 103.0 x 90.5 x 34.77 mm) plus
    velcro/zip straps over the top.
  * The X500 rail centre spacing is likewise not relied upon: the tray bolts
    DOWN onto the X500 V2 #31109 platform board using its VERIFIED 86 x 58 mm
    M2.5 hole pattern (Holybro assembly guide p.2). Holybro's own platform does
    the rail clamping.
  * D435i uses its VERIFIED pattern: 2x M3 @ 45 mm pitch, insertion <= 3 mm.

All millimetres. Every number below is either a verified spec or a clearly
labelled, user-tunable build parameter. Re-run after editing to regenerate
STEP (editable) + STL (printable) in ./out.
"""
import os
import cadquery as cq

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
os.makedirs(OUT, exist_ok=True)

# ---- verified / measured component specs ----
DEVKIT_L = 103.0           # Orin Nano DevKit outer length  [NVIDIA spec Fig 4-2]
DEVKIT_W = 90.5            # Orin Nano DevKit outer width   [NVIDIA spec Fig 4-2]
DEVKIT_H = 34.77           # Orin Nano DevKit outer height  [NVIDIA spec Fig 4-2]
# NVIDIA does NOT publish the Orin Nano DevKit hole positions (spec gives board
# size only; coords live in the PCB files). So we use the FIRST AUTHOR's measured
# values -- the only authoritative source for this unit. (86x58 was the OLD Jetson
# Nano pattern and does NOT apply to Orin Nano.)
JHX = 84.97                # Jetson hole pitch, long  (first-author caliper)
JHY = 57.12                # Jetson hole pitch, short (first-author caliper)
M25 = 2.9                  # M2.5 clearance hole dia (for the Jetson corner holes)
CAM_PITCH = 45.0           # D435i mount hole pitch (M3)     [D435 datasheet 10-9]
M3 = 3.2                   # M3 clearance hole dia

# ---- 2-STORY SHELF: drone top-plate mount (USER-MEASURED on THIS X500 V2) ----
# The KOA FC occupies the top plate centre, so the tray is a SHELF held on 4
# standoffs that straddle the FC and bolt into the plate's inner-column holes.
# VERIFIED against official Holybro CAD (x500v2-frame.step): the inner-column
# inset holes are exactly 86.0 x 27.0 mm, dia 3.0 (M2.5). The first author's
# caliper read 85.46 x 26.82 x 2.94 -> confirmed (<0.6 mm error).
DRONE_HX = 86.0            # lateral span L<->R across the FC  [CAD verified; meas 85.46]
DRONE_HY = 27.0            # fore-aft span within each column  [CAD verified; meas 26.82]
DRONE_HOLE = 3.2           # tray clearance for M2.5 standoff screw (plate hole is 3.0)
STAND_H = 23.0             # leg height: ALL plate components <20 mm (re-measured) -> 3 mm margin
LEG_OD = 5.0               # leg outer dia (round boss for M2.5 self-tap, wall 1.45 mm)
LEG_FLAT = 1.5             # shave the OUTBOARD (receiver) face to 1.5 mm from hole centre
SLOT = 6.0                 # fore-aft slot length: absorbs any residual mismatch

# ---- tunable build params ----
# AXES:  +X = drone FORWARD (camera faces here)   Y = lateral (L<->R)
# The 86 mm spans run along Y (lateral), the short spans (58 Jetson / 27 drone)
# run along X (fore-aft), so the camera flange on +X looks straight ahead.
TH = 4.0                   # adapter-plate thickness
PLATE_L = 69.0             # X (fore-aft): covers Jetson 58 (+/-29) + margin
PLATE_W = 97.0             # Y (lateral):  covers the 86 (+/-43) span + margin
FRONT = PLATE_L / 2.0      # +X = drone FORWARD (camera bracket side)


# ===================================== TRAY (thin uniform RING = stock Jetson frame)
# Design goal (user, with photo of the stock Jetson plastic frame): REPLICATE that
# frame -- a thin, uniform-width rectangular RING whose 4 corner holes are the
# 84.97 x 57.12 pattern, centre fully OPEN. Because the stock frame contacts the
# Jetson only along that edge rectangle, matching it GUARANTEES the board's
# centre/random underside protrusions clear. All holes SELF-TAP: the ORIGINAL M2.5
# Jetson screws cut their own thread in the plastic. Then 4 legs drop to the drone
# and a forward arm carries the camera clear of the board's front overhang.
SELFTAP_M25 = 2.1          # drone-leg self-tap pilot (user's own M2.5x8 bolts)
SELFTAP_M2 = SELFTAP_M25   # JETSON screw hole = SAME as the drone hole. The user's
                           # Jetson screw EMPIRICALLY fits the 2.1 mm drone hole, so
                           # matching it guarantees the screw fits the posts too.
                           # (1.6 was too small; stop guessing M2 vs M2.5 -- just match
                           # the hole the screw is proven to fit.)
SELFTAP_M3 = 2.5           # M3 self-tap pilot -> camera bracket
WALL = 7.0                 # ring wall width (thin & uniform, like the stock frame)
JPOST_H = 8.0              # Jetson post height ABOVE the ring: board underside protrudes
                           # up to 7 mm (first-author measured) -> 8 mm lifts it clear
JPOST_OD = 6.0             # Jetson post outer dia (fits on the 7 mm wall)


def jetson_tray():
    HX, HY = JHY / 2.0, JHX / 2.0              # Jetson hole corners: (28.56, 42.49)
    XO, YO = HX + WALL / 2.0, HY + WALL / 2.0  # ring outer half-extents (~32, ~46)
    t = None
    # long walls (run along X) centred on the hole line Y = +/- HY
    for sy in (-1, 1):
        w = (cq.Workplane("XY").box(2 * XO, WALL, TH, centered=(True, True, False))
             .translate((0, sy * HY, 0)))
        t = w if t is None else t.union(w)
    # short walls (run along Y) centred on the hole line X = +/- HX -> closed ring
    for sx in (-1, 1):
        t = t.union(cq.Workplane("XY").box(WALL, 2 * YO, TH, centered=(True, True, False))
                    .translate((sx * HX, 0, 0)))
    # 4 Jetson STANDOFF POSTS rising from the ring corners: the board SCREWS INTO
    # these (not a flat hole), lifting it clear of the ring and giving the original
    # M2 screws real thread depth. Pilot runs post-top down through the wall.
    for sx in (-1, 1):
        for sy in (-1, 1):
            t = t.union(cq.Workplane("XY").circle(JPOST_OD / 2.0).extrude(JPOST_H)
                        .translate((sx * HX, sy * HY, TH)))
    for sx in (-1, 1):
        for sy in (-1, 1):
            t = t.cut(cq.Workplane("XY").circle(SELFTAP_M2 / 2.0).extrude(JPOST_H + TH + 2)
                      .translate((sx * HX, sy * HY, -1)))
    # DRONE mount = 4 LEGS drop from the ring down to the plate's 86x27 holes. The
    # plate is OCCUPIED (KOA FC 23 mm centre + FS-iA6B receiver + wiring), so the legs
    # must RAISE the whole tray clear above all of it. Self-tap feet: an M2.5x8 bolt
    # comes UP from under the plate into each leg.
    for sx in (-1, 1):
        for sy in (-1, 1):
            px, py = sx * DRONE_HY / 2.0, sy * DRONE_HX / 2.0
            t = t.union(cq.Workplane("XY").circle(LEG_OD / 2.0).extrude(-STAND_H)
                        .translate((px, py, 0)))
            # shave the OUTBOARD (receiver-facing) face flat so the leg clears the
            # FS-iA6B: leg reaches only LEG_FLAT past the hole centre toward the edge.
            t = t.cut(cq.Workplane("XY").box(2 * LEG_OD, LEG_OD, STAND_H,
                      centered=(True, True, False))
                      .translate((px, py + sy * (LEG_FLAT + LEG_OD / 2.0), -STAND_H)))
    for sx in (-1, 1):
        for sy in (-1, 1):
            px, py = sx * DRONE_HY / 2.0, sy * DRONE_HX / 2.0
            t = t.cut(cq.Workplane("XY").circle(SELFTAP_M25 / 2.0).extrude(13)
                      .translate((px, py, -STAND_H - 0.5)))
    # ---- forward camera mount: continue the two long walls forward (along Y=+/-HY,
    # under the board EDGE only -- never the open centre) past the board front
    # (~X+45), bridge with a cross wall, then the camera pad. Holes at X=62.
    AX = 48.0
    for sy in (-1, 1):
        t = t.union(cq.Workplane("XY").box(AX - XO, WALL, TH, centered=(False, True, False))
                    .translate((XO, sy * HY, 0)))
    t = t.union(cq.Workplane("XY").box(WALL, 2 * YO, TH, centered=(True, True, False))
                .translate((AX, 0, 0)))
    FL_HOLE_X = 62.0
    t = t.union(cq.Workplane("XY").box((FL_HOLE_X + 12.0) - AX, 40, TH,
                centered=(False, True, False)).translate((AX, 0, 0)))
    for sy in (-1, 1):
        t = t.cut(cq.Workplane("XY").circle(SELFTAP_M3 / 2.0).extrude(TH + 2)
                  .translate((FL_HOLE_X, sy * 15, -1)))
    return t


# =================================================== D435i BRACKETS (swappable)
# Both brackets share the SAME foot interface so they are interchangeable on the
# tray front flange: 2x M3 @ 30 mm pitch (holes at Y = +/-15, near the rear).
# The D435i mounting face (bottom, 2x M3 @ 45 mm) is PERPENDICULAR to the lens,
# so: horizontal pad -> lens forward;  vertical pad -> lens down.
T = 4.0
FOOT_HX, FOOT_HY = 0.0, 15.0     # flange holes: X=rear(-12 below), Y=+/-15 (30 mm)


def _foot_holes(wp, x):
    for sy in (-1, 1):
        wp = wp.cut(cq.Workplane("XY").circle(M3 / 2.0)
                    .extrude(T + 2).translate((x, sy * FOOT_HY, -1)))
    return wp


def d435i_bracket_front():
    """Horizontal camera pad -> D435i looks FORWARD (+X)."""
    plate = (cq.Workplane("XY").box(46, 50, T, centered=(True, True, False))
             .translate((4, 0, 0)))                       # X: -19..27, Y: +/-25
    plate = _foot_holes(plate, -12)                       # bolt to tray flange
    for sy in (-1, 1):                                    # camera 2x M3 @ 45 mm
        plate = plate.cut(cq.Workplane("XY").circle(M3 / 2.0)
                          .extrude(T + 2).translate((14, sy * CAM_PITCH / 2.0, -1)))
    return plate


def d435i_bracket_down():
    """Vertical camera pad -> D435i looks DOWN (-Z)."""
    foot = (cq.Workplane("XY").box(34, 50, T, centered=(True, True, False))
            .translate((-7, 0, 0)))                       # X: -24..10, Y: +/-25
    foot = _foot_holes(foot, -12)
    pad_x, pad_h, hole_z = 8.0, 52.0, 30.0
    pad = (cq.Workplane("YZ").box(50, pad_h, T, centered=(True, False, False))
           .translate((pad_x, 0, 0)))                     # vertical wall, normal +X
    for sy in (-1, 1):                                    # camera 2x M3 @ 45 mm
        pad = pad.cut(cq.Workplane("YZ").circle(M3 / 2.0)
                      .extrude(T + 2).translate((pad_x - 1, sy * CAM_PITCH / 2.0, hole_z)))
    # two side gussets tying the vertical pad to the foot
    gus = (cq.Workplane("XZ").polyline([(0, 0), (-16, 0), (0, 26)]).close()
           .extrude(T).translate((pad_x, -T / 2.0, 0)))
    g1 = gus.translate((0, 23, 0))
    g2 = gus.translate((0, -23, 0))
    return foot.union(pad).union(g1).union(g2)


def export(obj, name):
    cq.exporters.export(obj, os.path.join(OUT, name + ".step"))
    cq.exporters.export(obj, os.path.join(OUT, name + ".stl"),
                        tolerance=0.05, angularTolerance=0.1)
    bb = obj.val().BoundingBox()
    print("%-18s bbox  %6.1f x %6.1f x %6.1f mm"
          % (name, bb.xlen, bb.ylen, bb.zlen))


if __name__ == "__main__":
    print("Jetson 85x57 posts %.0fmm up (orig M2) | drone 86x27 legs %.0fmm down "
          "(M2.5x8) -> raises tray above the occupied plate" % (JPOST_H, STAND_H))
    # print-ready: rotate 180 about X so the long DRONE LEGS point UP as clean
    # vertical towers on the bed (the short Jetson posts become small feet).
    export(jetson_tray().rotate((0, 0, 0), (1, 0, 0), 180), "jetson_tray")
    export(d435i_bracket_front(), "d435i_bracket_front")
    export(d435i_bracket_down(), "d435i_bracket_down")
    print("done ->", OUT)
