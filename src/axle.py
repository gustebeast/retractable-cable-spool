"""Main spool axle: shaft + bearing-retention shoulders + cross-pin hole +
center mortise-and-tenon joint. The axle is printed in TWO halves that
glue together at the joint — the tape-spring assembly only has a 13.46 mm
opening, and the joint must pass through it, so we trade single-piece
removability for shape: tenon Ø AXLE_PRINT_D (= 7.9) descends from the
pancake-side (TOP) half; mortise (Ø JOINT_MORTISE_HOLE_D bore inside Ø
JOINT_MORTISE_OD = 11.45 wall) on the lever-side (BOTTOM) half. Joint
glued.

Print orientation:
  • BOTTOM half: print -z (axle bottom) on the bed, building toward +z.
    The cap at the bore's closed bottom becomes a "hole appears" event
    (the previous solid layer supports the new annular layer), so no
    overhang there. The lever lip's -z face is now the expanding
    overhang; a 45° cone fixes it. The bearing seats at the cone's
    bottom (where the diameter begins to grow from the shaft).
  • TOP half: print -z (tenon tip) on the bed, building toward +z. The
    pancake lip's -z face is the expanding overhang; a 45° cone fixes it.

Slit through the joint (for the spring leg to engage) — TODO next."""

import cadquery as cq

from .dimensions import (
    AXLE_CROSS_HOLE_D, AXLE_D, AXLE_EXTRA_LEVER, AXLE_H,
    AXLE_LIP_H, AXLE_LIP_OD, AXLE_PRINT_D,
    FIT_CLR, STRUCT_WALL,
    LEVER_CAP_SEAT_Z1,
    PANCAKE_CAP_SEAT_Z0, PANCAKE_CROSS_PIN_Z,
)
from .helpers import cone_solid, cyl


# ── Mortise-and-tenon center joint ─────────────────────────────────────────
JOINT_TENON_D        = AXLE_PRINT_D                          # 7.9 — male, on TOP half
JOINT_MORTISE_HOLE_D = AXLE_PRINT_D + 2 * FIT_CLR            # 8.2 — female bore (FIT_CLR per side)
JOINT_MORTISE_OD     = JOINT_MORTISE_HOLE_D + 2 * STRUCT_WALL  # 11.6 — outer wall
JOINT_H              = 18.0                                  # tenon length = bore depth — sized to fit
                                                             # the slit overlap exactly so the bottom
                                                             # slit doesn't extend past the bore into
                                                             # the cap.
JOINT_CAP_H          = STRUCT_WALL                           # 1.7 — closed-bottom cap of the
                                                             # mortise, joining the bore's
                                                             # outer ring to the shaft below.

# ── Spring-strip slit ──────────────────────────────────────────────────────
# Each half has its own slit (cut through Y) so the parts can be slid down
# onto the spring's metal strip from each end of the joint. The slits open at
# the piece's joint-side face (bottom half: +z = mortise opening; top half:
# -z = tenon tip). Where both slits coexist along Z, the strip is held by
# both halves — that's the engagement region.
SLIT_W            = 1.0           # X width of the slit
SLIT_OVERLAP_H    = 18.0          # Z height where both halves have the slit
# For symmetric slits each is (SLIT_OVERLAP_H + JOINT_H) / 2 tall, so each
# extends (SLIT_OVERLAP_H − JOINT_H) / 2 = 1.5 mm beyond the joint into
# its own half.
SLIT_EACH_H       = (SLIT_OVERLAP_H + JOINT_H) / 2  # 16.5


def _axle_cross_hole(z_center):
    """M2 clearance hole through the axle, axis along y, centered at z."""
    return cq.Workplane("XY").add(cq.Solid.makeCylinder(
        AXLE_CROSS_HOLE_D / 2, AXLE_D + 2,
        pnt=cq.Vector(0, -(AXLE_D / 2 + 1), z_center),
        dir=cq.Vector(0, 1, 0),
    ))


def _build_halves():
    # 45° cone height for transitioning a Ø AXLE_PRINT_D shaft to an
    # Ø AXLE_LIP_OD lip (or vice versa).
    cone_h = (AXLE_LIP_OD - AXLE_PRINT_D) / 2

    # Each bearing-lip feature group (cone + lip) bounds the cavity. The
    # cavity midpoint is between the TOP of the lever group and the BOTTOM
    # of the pancake group.
    pancake_lip_inner_z = PANCAKE_CAP_SEAT_Z0 - AXLE_LIP_H        # bottom face of pancake lip
    cavity_top_z        = pancake_lip_inner_z - cone_h            # bottom of pancake cone
    cavity_bot_z        = LEVER_CAP_SEAT_Z1 + cone_h + AXLE_LIP_H # top of lever lip (above cone)
    cavity_midpoint_z   = (cavity_top_z + cavity_bot_z) / 2

    # Joint Z layout. The whole mortise body (collar of height
    # JOINT_H + JOINT_CAP_H) is centered on the cavity midpoint, so the
    # shaft-segment lengths from the lever lip to the mortise and from the
    # mortise to the pancake lip are equal.
    mortise_body_h  = JOINT_H + JOINT_CAP_H
    mortise_top_z   = cavity_midpoint_z + mortise_body_h / 2
    bore_bottom_z   = mortise_top_z - JOINT_H
    collar_bottom_z = bore_bottom_z - JOINT_CAP_H

    axle_z_bot = -AXLE_EXTRA_LEVER
    axle_z_top = -AXLE_EXTRA_LEVER + AXLE_H

    # ── BOTTOM half (lever-side) — shaft + lever lip + mortise collar ──
    # Print -axle_z (axle bottom) on the bed → grows toward +axle_z. Lever
    # lip's -z face is the upward-expanding overhang in this direction →
    # 45° cone below the lip. Bearing seats at the cone's bottom
    # (= LEVER_CAP_SEAT_Z1), where Ø just begins to grow above shaft Ø.
    bottom = cyl(AXLE_PRINT_D, collar_bottom_z - axle_z_bot, z=axle_z_bot)
    # Cone BELOW lever lip: Ø AXLE_PRINT_D at z=LEVER_CAP_SEAT_Z1 (bearing
    # seat) → Ø AXLE_LIP_OD at z=LEVER_CAP_SEAT_Z1 + cone_h (lip bottom).
    bottom = bottom.union(cone_solid(
        d_bottom=AXLE_PRINT_D, d_top=AXLE_LIP_OD,
        h=cone_h, z_base=LEVER_CAP_SEAT_Z1,
    ))
    # Lip ABOVE the cone, full Ø AXLE_LIP_OD for AXLE_LIP_H.
    bottom = bottom.union(cyl(AXLE_LIP_OD, AXLE_LIP_H,
                              z=LEVER_CAP_SEAT_Z1 + cone_h))
    # Mortise collar (solid Ø JOINT_MORTISE_OD cylinder, JOINT_H + JOINT_CAP_H tall).
    bottom = bottom.union(cyl(JOINT_MORTISE_OD,
                              mortise_top_z - collar_bottom_z,
                              z=collar_bottom_z))
    # Bore (cylindrical pocket Ø JOINT_MORTISE_HOLE_D, JOINT_H deep, opens at top).
    bottom = bottom.cut(cyl(JOINT_MORTISE_HOLE_D, JOINT_H, z=bore_bottom_z))
    # Spring-strip slit — opens at +z (mortise top), extends SLIT_EACH_H
    # down. Cuts through the axle in Y.
    slit_y_overshoot = JOINT_MORTISE_OD + 10
    bottom = bottom.cut(
        cq.Workplane("XY")
        .workplane(offset=mortise_top_z - SLIT_EACH_H)
        .rect(SLIT_W, slit_y_overshoot)
        .extrude(SLIT_EACH_H)
    )

    # ── TOP half (pancake-side) — tenon (= shaft continuation) + pancake lip ──
    # Print -axle_z (tenon tip) on the bed → grows toward +axle_z. Pancake
    # lip's -z face is the upward-expanding overhang in this direction →
    # 45° cone.
    top = cyl(AXLE_PRINT_D, axle_z_top - bore_bottom_z, z=bore_bottom_z)
    # Cone below pancake lip: Ø AXLE_PRINT_D at z=pancake_lip_inner_z - cone_h
    # (shaft) → Ø AXLE_LIP_OD at z=pancake_lip_inner_z (lip bottom).
    top = top.union(cone_solid(
        d_bottom=AXLE_PRINT_D, d_top=AXLE_LIP_OD,
        h=cone_h, z_base=pancake_lip_inner_z - cone_h,
    ))
    top = top.union(cyl(AXLE_LIP_OD, AXLE_LIP_H, z=pancake_lip_inner_z))
    # Cross-pin hole — pancake-side only.
    top = top.cut(_axle_cross_hole(PANCAKE_CROSS_PIN_Z))
    # Spring-strip slit — opens at -z (tenon tip), extends SLIT_EACH_H up.
    # Cuts through the tenon in Y.
    top = top.cut(
        cq.Workplane("XY")
        .workplane(offset=bore_bottom_z)
        .rect(SLIT_W, slit_y_overshoot)
        .extrude(SLIT_EACH_H)
    )

    return bottom, top


axle_bottom, axle_top = _build_halves()
# Assembled view (for the assembly STEP).
axle = axle_bottom.union(axle_top)
