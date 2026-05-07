"""Guide-bearing area of the housing — thickened plate + boss + sleeve + wrap.

The guide 608 bearing's outer race rolls on the pancake-side flange's
outer face, providing an axial stop on the side opposite the levers so
lever forces can't lift the spool off-true. This module defines the
geometric pieces (bearing pocket, press hole, anti-rotation recess,
sleeve, +Y wrap, gussets) and exposes apply_to_housing(housing) to
union/cut them onto the U-bracket built by housing.py.
"""

import math
import cadquery as cq

from .dimensions import AXLE_PRINT_D, BEARING_OD, BEARING_W, FLANGE_OD, SPOOL_H
from .housing import HOUSING_SPINE_T, HOUSING_W, PANCAKE_PLATE_Z_IN, PANCAKE_PLATE_Z_OUT


# ── Guide bearing: thickened plate + boss + separate axle part ───────────
# A 608 guide bearing whose outer race rolls on the pancake-side
# flange's outer face provides an axial stop on the side OPPOSITE the
# levers — so the lever forces can't lift the spool off true.
#
# Layout:
#   • Bearing axis: Y = GUIDE_BEARING_Y_OFFSET, Z = SPOOL_H + BEARING_OD/2.
#   • Bearing pocket cut: cylinder R = BEARING_OD/2 + 1 (= 12), spanning
#     X = 66..81 (15 mm). 1 mm radial clearance for the bearing OD;
#     1 mm axial clearance past the flange edge (X=80).
#   • Press hole: Ø(AXLE_D + PRESS_CLR) bored from the cut's inboard end
#     at X = 66 inward to X = 60 (6 mm of mount-hole grip).
#   • Half-circle anti-rotation recess: at X = 65..66, BOTTOM HALF only
#     (Z < axle Z), the press-hole region is widened to radius 7.15.
#     The matching half-disc tab on the axle keys against rotation;
#     because the recess is only on one half, the TOP HALF housing
#     material at X = 66 still forms a 1 mm lip that axially stops the
#     bearing.
#   • Boss above plate: 4-vertex loft from a wide base on the plate
#     (45° taper on -X and ±Y) up to a narrower top covering the
#     press-hole region. +X face is vertical (90°, against bearing
#     pocket). Y taper at 45° avoids print overhangs.
#   • Plate-thickening pad: column on the bot plate's outer side
#     (Z<PANCAKE_PLATE_Z_OUT) extending 4.5 mm deeper, mimicking the
#     main-axle column pattern but with 45° only on the -X side and
#     a vertical 90° on the +X side (against the bearing pocket).
#   • +Y wrap: a tab in the plate plane extending +Y past the housing
#     edge so the bearing pocket has 4 mm of plate material wrapping
#     it on the +Y side.
GUIDE_BEARING_Y_OFFSET    = 3.392    # shifted total -1.608 from 5.0 so the
                                      # -Y housing wall measures 4 mm at the
                                      # plate-bottom cross-section (where the
                                      # cylindrical cut is at its narrowest).
GUIDE_AXLE_Z             = SPOOL_H + BEARING_OD / 2         # bearing axle Z (above the spool)

GUIDE_BEARING_GAP_RAD_CLR = 1.0
GUIDE_BEARING_INBOARD_GAP = 1.0      # mount wall → bearing inboard face
GUIDE_BEARING_OUTBOARD_GAP = 7.2     # bearing outboard face → cut outboard wall
                                      # (= 7 mm install travel + 0.2 mm clearance)

GUIDE_AXLE_BEARING_LEN    = 8.0      # axle visible inside the cut
GUIDE_AXLE_PRESS_LEN      = 6.0      # axle press-fit shank inside mount hole
GUIDE_AXLE_TOTAL_LEN      = (GUIDE_AXLE_BEARING_LEN
                             + GUIDE_AXLE_PRESS_LEN)         # 14
GUIDE_PRESS_CLR           = 0.3      # diametral clearance (0.15 mm per side, slip fit)

# X coords along axle. The bearing's outboard face sits 0.8 mm past the
# flange edge (total +6.8 mm shift toward the housing mounting side).
# Cut extends inboard for the bearing+axle and outboard into the housing-
# mount-side material for installation travel.
_GUIDE_AXLE_X_OUTER       = FLANGE_OD / 2 + 0.8                                 # 80.8
_GUIDE_CUT_X_MIN          = (_GUIDE_AXLE_X_OUTER - BEARING_W
                             - GUIDE_BEARING_INBOARD_GAP)                       # 72
_GUIDE_CUT_X_MAX          = _GUIDE_AXLE_X_OUTER + GUIDE_BEARING_OUTBOARD_GAP    # 87.2
GUIDE_CUT_LEN             = _GUIDE_CUT_X_MAX - _GUIDE_CUT_X_MIN                 # 15.2
GUIDE_PRESS_X_MIN        = _GUIDE_CUT_X_MIN - GUIDE_AXLE_PRESS_LEN             # 66
_GUIDE_AXLE_X_MAX         = GUIDE_PRESS_X_MIN + GUIDE_AXLE_TOTAL_LEN           # 80

# Anti-rotation half-disc tab + recess. Recess (housing) is 1 mm deep into
# the housing on the bottom half. Tab (axle) is 2 mm long along X — 1 mm
# inside the recess and 1 mm protruding into the bearing pocket. The
# protruding 1 mm acts as a hard stop for the bearing's inner-race face
# on the bottom half, guaranteeing a 1 mm air gap between the bearing
# and the housing's mount-wall face on the rest of the inboard side.
GUIDE_KEY_RECESS_LEN      = 1.0      # axial depth of recess INTO housing
GUIDE_KEY_TAB_PROTRUDE    = 1.0      # axial protrusion of tab past mount wall
GUIDE_KEY_TAB_LEN         = (GUIDE_KEY_RECESS_LEN
                             + GUIDE_KEY_TAB_PROTRUDE)         # 2
GUIDE_KEY_TAB_R           = 6.0      # tab outer radius (axle Ø8 base + 2 mm)
GUIDE_KEY_CLR             = 0.20     # radial clearance, recess→tab (key exception)
_GUIDE_KEY_RECESS_X_MIN   = _GUIDE_CUT_X_MIN - GUIDE_KEY_RECESS_LEN           # 65
_GUIDE_KEY_RECESS_X_MAX   = _GUIDE_CUT_X_MIN                                  # 66
GUIDE_KEY_TAB_X_MIN      = _GUIDE_KEY_RECESS_X_MIN                           # 65
GUIDE_KEY_TAB_X_MAX      = (GUIDE_KEY_TAB_X_MIN
                             + GUIDE_KEY_TAB_LEN)                             # 67

# ── Bearing-clearance cut (rectangular box around the bearing) ──────────
# A square 24×24 cross-section (bearing OD 22 + 1 mm clearance on all
# four sides) extruded over the cut length. Straight walls print cleanly
# without the curved overhangs a cylindrical cut would leave.
_GUIDE_GAP_R              = BEARING_OD / 2 + GUIDE_BEARING_GAP_RAD_CLR        # 12 (half-side of the 24×24 box)
guide_bearing_cut = (
    cq.Workplane("XY")
    .box(GUIDE_CUT_LEN, _GUIDE_GAP_R * 2, _GUIDE_GAP_R * 2,
         centered=False)
    .translate((_GUIDE_CUT_X_MIN,
                GUIDE_BEARING_Y_OFFSET - _GUIDE_GAP_R,
                GUIDE_AXLE_Z - _GUIDE_GAP_R))
)

# ── Recess-plane rectangle cut (1 mm deep, in the X = 71.8..72.8 plane) ─
# Rectangle defined by two diagonal corners in YZ:
#   Corner A: (Y = +Y face of bearing cut, Z = -Z face of bearing cut)
#   Corner B: (Y = axle center, Z = bottom of axle hole circle - 2 mm)
_RECESS_RECT_Y_MIN = GUIDE_BEARING_Y_OFFSET                                   # 3.392
_RECESS_RECT_Y_MAX = GUIDE_BEARING_Y_OFFSET + _GUIDE_GAP_R                    # 15.392
_RECESS_RECT_Z_MIN = GUIDE_AXLE_Z - _GUIDE_GAP_R                             # -23
_RECESS_RECT_Z_MAX = (GUIDE_AXLE_Z
                      - (AXLE_PRINT_D + GUIDE_PRESS_CLR) / 2
                      - 2.0)                                                  # -17.05
_RECESS_PLANE_X_MIN = _GUIDE_KEY_RECESS_X_MIN                                 # 71.8
_RECESS_PLANE_X_MAX = _GUIDE_CUT_X_MIN                                        # 72.8

guide_recess_rect_cut = (
    cq.Workplane("XY")
    .box(_RECESS_PLANE_X_MAX - _RECESS_PLANE_X_MIN,
         _RECESS_RECT_Y_MAX - _RECESS_RECT_Y_MIN,
         _RECESS_RECT_Z_MAX - _RECESS_RECT_Z_MIN,
         centered=False)
    .translate((_RECESS_PLANE_X_MIN,
                _RECESS_RECT_Y_MIN,
                _RECESS_RECT_Z_MIN))
)

# ── Press hole (Ø8.1 along +X axis) ─────────────────────────────────────
_press_hole_d = AXLE_PRINT_D + GUIDE_PRESS_CLR
guide_press_cut = cq.Workplane("XY").add(cq.Solid.makeCylinder(
    _press_hole_d / 2, GUIDE_AXLE_PRESS_LEN + 0.1,
    pnt=cq.Vector(GUIDE_PRESS_X_MIN - 0.05,
                  GUIDE_BEARING_Y_OFFSET, GUIDE_AXLE_Z),
    dir=cq.Vector(1, 0, 0),
))

# ── Anti-rotation recess (bottom half of widened press-hole region) ─────
_key_recess_r = GUIDE_KEY_TAB_R + GUIDE_KEY_CLR    # 6.15
_key_full_cyl = cq.Workplane("XY").add(cq.Solid.makeCylinder(
    _key_recess_r, GUIDE_KEY_RECESS_LEN + 0.05,
    pnt=cq.Vector(_GUIDE_KEY_RECESS_X_MIN - 0.025,
                  GUIDE_BEARING_Y_OFFSET, GUIDE_AXLE_Z),
    dir=cq.Vector(1, 0, 0),
))
_key_clip_box = (
    cq.Workplane("XY")
    .box(GUIDE_KEY_RECESS_LEN + 1.0, 30.0, 30.0, centered=False)
    .translate((_GUIDE_KEY_RECESS_X_MIN - 0.5,
                GUIDE_BEARING_Y_OFFSET - 15.0,
                GUIDE_AXLE_Z))
)
guide_key_recess = _key_full_cyl.intersect(_key_clip_box)

# ── Printability cuts on the recess side of the boss ──────────────────
# Both cuts live on the SAME half as the half-moon recess (Z > axle Z)
# so the OPPOSITE half — the housing material that blocks the axle's
# tab from rotating out of the recess — stays intact.
#
# Top extension: annular cut on the recess side at the recess depth
# (X=_KEY_RECESS_X_MIN..X_MAX) from _key_recess_r out to _GUIDE_GAP_R.
# Removes the radial lip between the half-moon's outer radius (6.15)
# and the bearing-pocket cut radius (12), so the recess's outer edge
# transitions cleanly into the bearing pocket without a sharp ledge.
_top_extend_outer = cq.Workplane("XY").add(cq.Solid.makeCylinder(
    _GUIDE_GAP_R, GUIDE_KEY_RECESS_LEN + 0.05,
    pnt=cq.Vector(_GUIDE_KEY_RECESS_X_MIN - 0.025,
                  GUIDE_BEARING_Y_OFFSET, GUIDE_AXLE_Z),
    dir=cq.Vector(1, 0, 0),
))
_top_extend_inner = cq.Workplane("XY").add(cq.Solid.makeCylinder(
    _key_recess_r, GUIDE_KEY_RECESS_LEN + 0.2,
    pnt=cq.Vector(_GUIDE_KEY_RECESS_X_MIN - 0.1,
                  GUIDE_BEARING_Y_OFFSET, GUIDE_AXLE_Z),
    dir=cq.Vector(1, 0, 0),
))
_top_extend_annulus = _top_extend_outer.cut(_top_extend_inner)
# Apply on BOTH halves of the boss: the annular cut runs from the recess
# outer radius (6.15) out to the bearing-pocket radius (12). Anti-rotation
# is preserved because the sleeve material at radius 4.1..6 (on the half
# opposite the recess) sits inside the annulus and isn't touched.
guide_top_extend = _top_extend_annulus

# Bottom flat: thin slab cut just outside the recess curve's apex
# (radial extreme on the recess side), connecting the recess back wall
# at X=_KEY_RECESS_X_MIN out to the bearing-pocket wall via a short
# flat surface. Outside the tab's rotational path (radius 4..6), so it
# doesn't affect anti-rotation.
GUIDE_KEY_BOTTOM_FLAT_T = 1.0
_bottom_flat_z_top = GUIDE_AXLE_Z + _key_recess_r
_bottom_flat_z_bot = _bottom_flat_z_top + GUIDE_KEY_BOTTOM_FLAT_T
guide_bottom_flat = (
    cq.Workplane("XY")
    .box(GUIDE_KEY_RECESS_LEN, _key_recess_r * 2,
         GUIDE_KEY_BOTTOM_FLAT_T, centered=False)
    .translate((_GUIDE_KEY_RECESS_X_MIN,
                GUIDE_BEARING_Y_OFFSET - _key_recess_r,
                _bottom_flat_z_top))
)

# ── Half-annular bearing lip (1 mm radial) + 45° support ────────────────
# In the 1 mm radial gap between bearing OD (R=11) and the housing cut
# wall (R=12), restore material as a half-annulus on ONE half (the same
# half as the half-moon recess opens onto). The opposite half stays cut
# by guide_top_extend. A 45° triangular support anchors the lip's +Y
# chord-end to the plate's outer face — the housing material the bearing
# clearance cut didn't remove.
GUIDE_LIP_INNER_R = BEARING_OD / 2     # 11
GUIDE_LIP_OUTER_R = _GUIDE_GAP_R       # 12

_lip_outer = cq.Workplane("XY").add(cq.Solid.makeCylinder(
    GUIDE_LIP_OUTER_R, GUIDE_KEY_RECESS_LEN,
    pnt=cq.Vector(_GUIDE_KEY_RECESS_X_MIN,
                  GUIDE_BEARING_Y_OFFSET, GUIDE_AXLE_Z),
    dir=cq.Vector(1, 0, 0),
))
_lip_inner = cq.Workplane("XY").add(cq.Solid.makeCylinder(
    GUIDE_LIP_INNER_R, GUIDE_KEY_RECESS_LEN + 0.2,
    pnt=cq.Vector(_GUIDE_KEY_RECESS_X_MIN - 0.1,
                  GUIDE_BEARING_Y_OFFSET, GUIDE_AXLE_Z),
    dir=cq.Vector(1, 0, 0),
))
_lip_annulus = _lip_outer.cut(_lip_inner)
_lip_clip = (
    cq.Workplane("XY")
    .box(GUIDE_KEY_RECESS_LEN + 1.0, 30.0, 30.0, centered=False)
    .translate((_GUIDE_KEY_RECESS_X_MIN - 0.5,
                GUIDE_BEARING_Y_OFFSET - 15.0,
                GUIDE_AXLE_Z))
)
guide_bearing_lip = _lip_annulus.intersect(_lip_clip)

# 45° support — triangle in YZ extruded over the recess depth in X.
# Anchors the lip's +Y chord-end at (axle_Y + R_outer, axle_Z) to the
# plate's outer face (Z = PANCAKE_PLATE_Z_OUT), with the hypotenuse running
# at 45° (ΔY = ΔZ).
_LIP_SUPPORT_DZ = PANCAKE_PLATE_Z_OUT - GUIDE_AXLE_Z     # 6
guide_lip_support = (
    cq.Workplane("YZ")
    .polyline([
        (GUIDE_BEARING_Y_OFFSET + GUIDE_LIP_OUTER_R,
         GUIDE_AXLE_Z),
        (GUIDE_BEARING_Y_OFFSET + GUIDE_LIP_OUTER_R - _LIP_SUPPORT_DZ,
         PANCAKE_PLATE_Z_OUT),
        (GUIDE_BEARING_Y_OFFSET + GUIDE_LIP_OUTER_R,
         PANCAKE_PLATE_Z_OUT),
    ]).close()
    .extrude(GUIDE_KEY_RECESS_LEN)
)
_bb = guide_lip_support.val().BoundingBox()
guide_lip_support = guide_lip_support.translate(
    (_GUIDE_KEY_RECESS_X_MIN - _bb.xmin, 0, 0)
)



# ── Cylindrical sleeve around the axle (replaces the Z-axis boss + pad) ─
# A tube of OD 12 mm × 16 mm long, axis on the bearing axle, starting at
# the cut's inboard end (X = _GUIDE_CUT_X_MIN, midway through the half-
# circle anti-rotation tab). The bore is a slip-fit hole for the axle.
# Unioned AFTER the bearing-pocket cut so the sleeve isn't carved away.
GUIDE_SLEEVE_OD            = 12.0
GUIDE_SLEEVE_LEN           = 6.0     # was 8 mm; trimmed so the axle's
                                      # inboard end sits flush with the
                                      # sleeve's -X face (no extra room).
GUIDE_SLEEVE_SLIP_CLR      = 0.3     # diametral slip-fit clearance for axle (0.15 mm per side)
_GUIDE_SLEEVE_X_MAX        = _GUIDE_CUT_X_MIN                                # 66
_GUIDE_SLEEVE_X_MIN        = _GUIDE_SLEEVE_X_MAX - GUIDE_SLEEVE_LEN          # 50

guide_sleeve = cq.Workplane("XY").add(cq.Solid.makeCylinder(
    GUIDE_SLEEVE_OD / 2, GUIDE_SLEEVE_LEN,
    pnt=cq.Vector(_GUIDE_SLEEVE_X_MIN, GUIDE_BEARING_Y_OFFSET, GUIDE_AXLE_Z),
    dir=cq.Vector(1, 0, 0),
)).cut(
    cq.Workplane("XY").add(cq.Solid.makeCylinder(
        (AXLE_PRINT_D + GUIDE_SLEEVE_SLIP_CLR) / 2, GUIDE_SLEEVE_LEN + 0.2,
        pnt=cq.Vector(_GUIDE_SLEEVE_X_MIN - 0.1,
                      GUIDE_BEARING_Y_OFFSET, GUIDE_AXLE_Z),
        dir=cq.Vector(1, 0, 0),
    ))
)

# ── 45° printability fillet on the -Y side of the sleeve ───────────────
# At 45° around the cylinder from -Z (= bed-side of the sleeve in the
# printer), the surface tangent reaches the printer's 45° overhang limit.
# From that point we add a 45° wall sloping up to the plate, turning the
# previously unprintable -Y underhang into a self-supporting triangular
# fillet. In printer terms: the housing prints vertical, then 45° until
# it meets the cylinder, then continues around the cylinder.
_FILLET_R           = GUIDE_SLEEVE_OD / 2
_FILLET_45          = _FILLET_R * (2 ** 0.5) / 2                              # R*sin(45°) = 4.243
_FILLET_TANGENT_Y   = GUIDE_BEARING_Y_OFFSET - _FILLET_45                     # 0.071
_FILLET_TANGENT_Z   = GUIDE_AXLE_Z - _FILLET_45                              # 44.757
_FILLET_PLATE_Z     = PANCAKE_PLATE_Z_IN                                         # 53
# Slope dY/dZ = -1: 45° line from the tangent point hits the plate at...
_FILLET_FOOT_Y      = (_FILLET_TANGENT_Y
                       - (_FILLET_PLATE_Z - _FILLET_TANGENT_Z))               # -8.172
# Sleeve cross-section -Y edge at the plate top
_FILLET_SLEEVE_Y    = (GUIDE_BEARING_Y_OFFSET
                       - math.sqrt(_FILLET_R**2
                                   - (_FILLET_PLATE_Z - GUIDE_AXLE_Z)**2))   # -1.343

# Fillet matches the sleeve's full X range (X = _GUIDE_SLEEVE_X_MIN ..
# _GUIDE_SLEEVE_X_MAX), so the triangular support and the cylindrical
# boss share the same start and end X.
guide_sleeve_fillet = (
    cq.Workplane("YZ")
    .polyline([
        (_FILLET_TANGENT_Y, _FILLET_TANGENT_Z),
        (_FILLET_FOOT_Y,    _FILLET_PLATE_Z),
        (_FILLET_SLEEVE_Y,  _FILLET_PLATE_Z),
    ]).close()
    .extrude(GUIDE_SLEEVE_LEN)
)
_bb = guide_sleeve_fillet.val().BoundingBox()
guide_sleeve_fillet = guide_sleeve_fillet.translate(
    (_GUIDE_SLEEVE_X_MIN - _bb.xmin, 0, 0)
)

# ── +Y wrap (4 mm of plate material wrapping the bearing cut on +Y) ─────
GUIDE_WRAP_T              = 4.0      # ±X wrap thickness (=4 mm wall to spine
                                      # outer face / housing mount edge)
GUIDE_WRAP_T_Y            = 2.392    # +Y wrap thickness — sized so the wall
                                      # measures 4 mm at the plate-bottom
                                      # cross-section (where the cylindrical
                                      # cut is narrowest in Y).
_GUIDE_WRAP_Y_MIN         = HOUSING_W / 2                                     # 11
_GUIDE_WRAP_Y_MAX         = (GUIDE_BEARING_Y_OFFSET
                             + _GUIDE_GAP_R + GUIDE_WRAP_T_Y)                 # 17.784
_GUIDE_WRAP_X_MIN         = _GUIDE_CUT_X_MIN - GUIDE_WRAP_T                   # 68.8
_GUIDE_WRAP_X_MAX         = _GUIDE_CUT_X_MAX + GUIDE_WRAP_T                   # 92

guide_wrap = (
    cq.Workplane("XY")
    .box(_GUIDE_WRAP_X_MAX - _GUIDE_WRAP_X_MIN,
         _GUIDE_WRAP_Y_MAX - _GUIDE_WRAP_Y_MIN,
         PANCAKE_PLATE_Z_OUT - PANCAKE_PLATE_Z_IN,
         centered=False)
    .translate((_GUIDE_WRAP_X_MIN, _GUIDE_WRAP_Y_MIN, PANCAKE_PLATE_Z_IN))
)

# ── Triangular gussets bracing the +Y wrap to the housing main body ─────
# Right gusset (mounting-side, where the wrap overhangs past the spine):
# rotated 90° from the plate plane into the YZ plane so it extends into
# the housing interior instead of past the spine. 45° hypotenuse (Y span
# = Z span = wrap height).
# Left gusset (spool-center side): stays in the plate plane — no overhang
# issue there since the housing material continues inboard.
_GUSSET_45_H    = _GUIDE_WRAP_Y_MAX - _GUIDE_WRAP_Y_MIN     # 9.314
# Right gusset (in YZ, extruded along X) steps DOWN from the +Y wrap to the
# front spine, so its X depth follows the spine thickness. Left gusset (in
# XY, extruded along Z) sits in the plate's plane stepping UP from the
# plate to the wrap, so its Z depth follows the (now 11 mm) plate thickness.
_gusset_x_thick = HOUSING_SPINE_T                              # 2 — front-face thickness
_gusset_z_h     = PANCAKE_PLATE_Z_IN - PANCAKE_PLATE_Z_OUT     # -11 — plate thickness

guide_gusset_right = (
    cq.Workplane("YZ")
    .polyline([
        (_GUIDE_WRAP_Y_MAX, PANCAKE_PLATE_Z_IN),
        (_GUIDE_WRAP_Y_MIN, PANCAKE_PLATE_Z_IN),
        (_GUIDE_WRAP_Y_MIN, PANCAKE_PLATE_Z_IN - _GUSSET_45_H),
    ]).close()
    .extrude(_gusset_x_thick)
)
_bb = guide_gusset_right.val().BoundingBox()
guide_gusset_right = guide_gusset_right.translate(
    (_GUIDE_WRAP_X_MAX - _bb.xmax, 0, 0)
)

guide_gusset_left = (
    cq.Workplane("XY")
    .polyline([
        (_GUIDE_WRAP_X_MIN,                  _GUIDE_WRAP_Y_MAX),
        (_GUIDE_WRAP_X_MIN,                  _GUIDE_WRAP_Y_MIN),
        (_GUIDE_WRAP_X_MIN - _GUSSET_45_H,   _GUIDE_WRAP_Y_MIN),
    ]).close()
    .extrude(_gusset_z_h)
    .translate((0, 0, PANCAKE_PLATE_Z_OUT))
)



def apply_to_housing(housing: cq.Workplane) -> cq.Workplane:
    """Apply the guide-bearing geometry to a U-bracket housing. Cuts come
    first; the sleeve is unioned LAST so the bearing-pocket cut doesn't
    carve it away."""
    return (
        housing
        .union(guide_wrap)
        .union(guide_gusset_right)
        .union(guide_gusset_left)
        .cut(guide_press_cut)
        .cut(guide_bearing_cut)
        .union(guide_sleeve)
        .union(guide_sleeve_fillet)
        .cut(guide_recess_rect_cut)
    )
