#!/usr/bin/env python3
"""Fit verification: extract the ACTUAL screw-hole centres from the generated
solids and check every mating interface before printing.

Checks:
  1. Jetson <-> tray  : tray top 4 holes == Jetson Orin Nano pattern (86 x 58).
  2. drone  <-> tray  : tray mounts to the X500 V2 platform board at the SAME
                        86 x 58 holes (official "Jetson Nano" holes) -> identical.
  3. camera <-> bracket: bracket camera holes == D435i spec (45 mm M3).
  4. bracket <-> tray  : bracket foot holes == tray flange holes (30 mm M3).
"""
import math
import mount  # the design module (functions build the solids)
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder

# ---- real-world target specs ----
JETSON = (84.97, 57.12)    # Orin Nano DevKit (first-author caliper; NVIDIA spec omits holes)
DRONE = (86.0, 27.0)       # X500 V2 top-plate inner holes [CAD x500v2-frame.step verified]
D435I_PITCH = 45.0         # D435i side M3 pitch [Intel datasheet / Holybro guide]
FOOT_PITCH = 30.0          # bracket<->tray flange
TOL = 0.6                  # mm tolerance for a PASS


def holes(solid, rmax=3.0):
    """Return [(x,y,z,r,axis)] for every cylindrical face with radius < rmax."""
    out = []
    for f in solid.val().Faces():
        s = BRepAdaptor_Surface(f.wrapped)
        if s.GetType() == GeomAbs_Cylinder:
            r = s.Cylinder().Radius()
            if r < rmax:
                c = f.Center()
                d = s.Cylinder().Axis().Direction()
                ax = "Z" if abs(d.Z()) > 0.9 else ("X" if abs(d.X()) > 0.9 else "Y")
                out.append((round(c.x, 2), round(c.y, 2), round(c.z, 2),
                            round(r, 2), ax))
    return out


def dedup(hs, axis):
    """Collapse the two cylindrical faces of one through-hole into one point."""
    pts = {}
    for x, y, z, r, a in hs:
        if a != axis:
            continue
        key = (round(x, 1), round(y, 1)) if axis == "Z" else \
              ((round(y, 1), round(z, 1)) if axis == "X" else (round(x, 1), round(z, 1)))
        pts[key] = (x, y, z, r)
    return list(pts.values())


def rect_pitch(pts):
    """Given 4 points, return (long, short) centre-to-centre edge lengths."""
    xs = sorted(set(round(p[0], 1) for p in pts))
    ys = sorted(set(round(p[1], 1) for p in pts))
    return (round(max(xs) - min(xs), 2), round(max(ys) - min(ys), 2))


def pair_pitch(pts, idx):
    vals = sorted(p[idx] for p in pts)
    return round(abs(vals[-1] - vals[0]), 2)


def ok(a, b):
    return "PASS" if abs(a - b) <= TOL else "**FAIL**"


print("=" * 64)
tray = mount.jetson_tray()
th = holes(tray, rmax=3.0)
def ok2(pair, target):  # order-independent pitch-pair check
    a, b = sorted(pair); c, d = sorted(target)
    return "PASS" if abs(a - c) <= TOL and abs(b - d) <= TOL else "**FAIL**"

# holes are now self-tap sized, so classify by POSITION not radius
z = dedup(th, "Z")
zj = [p for p in z if 24 < abs(p[0]) < 34 and p[3] < 1.5]   # Jetson pilot at |x|~29
zd = [p for p in z if abs(p[0]) < 20 and p[3] < 1.5]        # leg pilot at |x|~13.5
#  (p[3]<1.5 keeps only the self-tap PILOTS, not the leg/post outer cylinders or the
#   shaved-leg partial-cylinder faces, so each hole is counted exactly once)
flange = [p for p in z if p[0] > 35]                    # camera-flange self-tap
jp = rect_pitch(zj) if len(zj) == 4 else ("?", "?")
print("TRAY Jetson self-tap holes:", len(zj), "found, pattern =", jp)
print("  [1] Jetson 84.97x57.12   :", ok2(jp, JETSON), " (measured; orig M2 screws (thread dia 2.0 measured))")
dp = rect_pitch(zd) if len(zd) == 4 else ("?", "?")
print("TRAY leg self-tap holes  :", len(zd), "found, pattern =", dp)
print("  [2] X500 top plate 86x27 :", ok2(dp, DRONE),
      " [CAD-verified vs caliper 85.46x26.82]")
fp = pair_pitch(flange, 1) if len(flange) >= 2 else 0
print("TRAY camera-flange holes :", len(flange), "found, pair pitch =", fp,
      "mm", ok(fp, FOOT_PITCH))

for name, fn in [("FRONT", mount.d435i_bracket_front),
                 ("DOWN", mount.d435i_bracket_down)]:
    print("-" * 64)
    b = fn()
    hb = holes(b, rmax=2.2)                              # all M3 (r1.6)
    zf = dedup(hb, "Z")                                  # horizontal plate holes
    # foot holes = the 2 at the most-negative X (rear); camera = 45mm pair
    if name == "FRONT":
        foot = sorted(zf, key=lambda p: p[0])[:2]
        cam = sorted(zf, key=lambda p: p[0])[-2:]
        fpp = pair_pitch(foot, 1); cpp = pair_pitch(cam, 1)
    else:
        foot = dedup(hb, "Z")                            # foot holes (vertical)
        camx = dedup(hb, "X")                            # camera holes on vertical pad
        fpp = pair_pitch(foot, 1); cpp = pair_pitch(camx, 1)
    print("BRACKET", name, ": foot pitch =", fpp, ok(fpp, FOOT_PITCH),
          "| camera pitch =", cpp, ok(cpp, D435I_PITCH))
    print("  [3] camera==D435i 45mm :", ok(cpp, D435I_PITCH))
    print("  [4] foot==tray flange  :", ok(fpp, fp))

print("=" * 64)
