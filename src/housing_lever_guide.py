"""Lever-side guide bearing — 608 between the ratchet and brake levers,
riding on the brake-pad track (lever-side flange's BOTTOM face at z=0).

Replaces the radial-support role of the bottom axle bearing once the
"open bottom axle" change exposes that end of the axle for cable exit.
The ratchet pivot was moved inboard from x=76 → x=70 to clear this
bearing's footprint.

Geometry mirrors housing_guide.py (the pancake-side guide bearing) but
on the lever side, and centered in y between the levers (no off-center
+Y wrap). Press-fit hole is on the OUTBOARD side (toward the front
spine) so the axle's grip region doesn't intersect the moved ratchet
pivot at x=70.

Layout:
  Bearing axis at (y=0, z=-11), axis along +X
  Bearing X range: x = 73.8..80.8 (BEARING_W = 7 mm)
  Bearing pocket cut: 24×24 box, x = 72.8..81.8, centered at axle
  Press hole: Ø(AXLE_PRINT_D+0.3) at x = 81.8..89.5 (8 mm grip into the
    plate, ending just shy of the front-spine inner face at x=90)
"""

import cadquery as cq

from .dimensions import AXLE_PRINT_D, BEARING_OD, BEARING_W, FLANGE_OD
from .housing import SPINE_X_INNER


# ── Bearing geometry ────────────────────────────────────────────────────────
LEVER_GUIDE_BEARING_Y_OFFSET = 0.0                 # centered between levers
LEVER_GUIDE_AXLE_Z           = -BEARING_OD / 2     # -11 — bearing OD touches flange bottom (z=0) from below

LEVER_GUIDE_BEARING_GAP_RAD_CLR = 1.0              # cut wall clearance (radial)
LEVER_GUIDE_AXLE_X_OUTER        = FLANGE_OD / 2 + 0.8   # 80.8 — bearing's outboard face (matches pancake side)

# Bearing X extent (axial along X)
_BEARING_X_OUTER = LEVER_GUIDE_AXLE_X_OUTER                # 80.8
_BEARING_X_INNER = _BEARING_X_OUTER - BEARING_W            # 73.8

# Bearing pocket cut: rectangular box, 1 mm clearance each side in x
_CUT_X_MIN = _BEARING_X_INNER - 1.0                        # 72.8
_CUT_X_MAX = _BEARING_X_OUTER + 1.0                        # 81.8
_GAP_R     = BEARING_OD / 2 + LEVER_GUIDE_BEARING_GAP_RAD_CLR   # 12

# Press hole: extends from cut's outboard face to ~0.5 mm shy of the spine
# inner face (so the axle's outer end doesn't poke through the spine).
LEVER_GUIDE_AXLE_PRESS_LEN = SPINE_X_INNER - _CUT_X_MAX - 0.5  # 7.7
LEVER_GUIDE_PRESS_CLR      = 0.3                           # diametral slip-fit (0.15 mm per side)
_press_hole_d = AXLE_PRINT_D + LEVER_GUIDE_PRESS_CLR       # 8.2


# ── Cuts ────────────────────────────────────────────────────────────────────
guide_bearing_cut = (
    cq.Workplane("XY")
    .box(_CUT_X_MAX - _CUT_X_MIN, _GAP_R * 2, _GAP_R * 2,
         centered=False)
    .translate((_CUT_X_MIN,
                LEVER_GUIDE_BEARING_Y_OFFSET - _GAP_R,
                LEVER_GUIDE_AXLE_Z - _GAP_R))
)

# Press hole: Ø8.2 cylinder along +X, starting at the cut's outboard face,
# ending 0.5 mm shy of the spine inner face. Slight overshoot at each end
# for clean booleans.
guide_press_cut = cq.Workplane("XY").add(cq.Solid.makeCylinder(
    _press_hole_d / 2, LEVER_GUIDE_AXLE_PRESS_LEN + 0.1,
    pnt=cq.Vector(_CUT_X_MAX - 0.05,
                  LEVER_GUIDE_BEARING_Y_OFFSET, LEVER_GUIDE_AXLE_Z),
    dir=cq.Vector(1, 0, 0),
))


# ── Boss / sleeve around the press hole ─────────────────────────────────────
# The press hole has 7.7 mm of grip in the 11 mm-thick plate, but it cuts
# fully through the plate (in z), so the axle's effective wall support is
# only the 11 mm strip of plate around the Ø8.2 hole. A cylindrical sleeve
# around the press hole — analogous to the pancake-side guide_sleeve —
# adds a Ø12 tube of material extending UP from the plate's inner face
# into the lever-housing gap, giving the axle press more bearing surface
# and resisting tilt under bearing load.
LEVER_GUIDE_SLEEVE_OD       = 12.0
LEVER_GUIDE_SLEEVE_LEN      = 6.0    # extends from the bearing pocket into the gap
LEVER_GUIDE_SLEEVE_SLIP_CLR = 0.3    # diametral slip-fit for axle (matches pancake-side)

# Sleeve sits OUTBOARD of the bearing pocket: from the pocket's +x face
# (= _CUT_X_MAX = 81.8) outward by SLEEVE_LEN (= 87.8). It surrounds the
# axle's press portion. (The press hole goes all the way to ~89.5 — the
# sleeve only covers the inboard 6 mm of that grip.)
_SLEEVE_X_MIN = _CUT_X_MAX                            # 81.8
_SLEEVE_X_MAX = _SLEEVE_X_MIN + LEVER_GUIDE_SLEEVE_LEN # 87.8

guide_sleeve = cq.Workplane("XY").add(cq.Solid.makeCylinder(
    LEVER_GUIDE_SLEEVE_OD / 2, LEVER_GUIDE_SLEEVE_LEN,
    pnt=cq.Vector(_SLEEVE_X_MIN, LEVER_GUIDE_BEARING_Y_OFFSET, LEVER_GUIDE_AXLE_Z),
    dir=cq.Vector(1, 0, 0),
)).cut(
    cq.Workplane("XY").add(cq.Solid.makeCylinder(
        (AXLE_PRINT_D + LEVER_GUIDE_SLEEVE_SLIP_CLR) / 2,
        LEVER_GUIDE_SLEEVE_LEN + 0.2,
        pnt=cq.Vector(_SLEEVE_X_MIN - 0.1,
                      LEVER_GUIDE_BEARING_Y_OFFSET, LEVER_GUIDE_AXLE_Z),
        dir=cq.Vector(1, 0, 0),
    ))
)


def apply_to_housing(housing: cq.Workplane) -> cq.Workplane:
    """Cut the lever-side guide bearing pocket + axle press hole into
    the housing, then union the sleeve so it isn't carved away by the
    bearing pocket cut."""
    return (
        housing
        .cut(guide_bearing_cut)
        .cut(guide_press_cut)
        .union(guide_sleeve)
    )
