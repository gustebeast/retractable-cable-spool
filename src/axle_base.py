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
    overhang there. The mortise collar runs all the way down to the
    bottom bearing seat, so there's no separate lever lip: the collar's
    chamfered foot (Ø shaft → Ø mortise) IS the bottom bearing shoulder,
    and its 45° rise self-supports the collar's -z underside. The bearing
    seats at the chamfer's bottom (where the Ø begins to grow from the shaft).
  • TOP half: print -z (tenon tip) on the bed, building toward +z. The
    pancake lip's -z face is the expanding overhang; a 45° cone fixes it.

Slit through the joint (for the spring leg to engage) — TODO next."""

import cadquery as cq

from .dimensions import (
    AXLE_CROSS_HOLE_D, AXLE_D, AXLE_EXTRA_LEVER, AXLE_H, TRAY_JOURNAL_H,
    AXLE_LIP_H, AXLE_LIP_OD, AXLE_PRINT_D,
    FIT_CLR, STRUCT_WALL,
    LEVER_CAP_SEAT_Z1,
    PANCAKE_CAP_SEAT_Z0, PANCAKE_CROSS_PIN_Z,
)
from .helpers import cone_solid, cyl


# ── Mortise-and-tenon center joint ─────────────────────────────────────────
JOINT_TENON_D        = AXLE_PRINT_D                          # 7.9 — male, on TOP half
JOINT_MORTISE_HOLE_D = AXLE_PRINT_D + 2 * FIT_CLR            # 8.2 — female bore (FIT_CLR per side)
JOINT_MORTISE_OD     = JOINT_MORTISE_HOLE_D + 2 * STRUCT_WALL  # 11.4 — outer wall
JOINT_H              = 23.0                                  # bore depth = tenon engagement = slit
                                                             # length. Lengthened from 18 to grip more
                                                             # spring strip: the mortise top moved up
                                                             # (only MORTISE_TOP_GAP below the top
                                                             # bearing lip) and the collar now runs all
                                                             # the way down to the BOTTOM bearing seat,
                                                             # taking over the (removed) bottom bearing
                                                             # lip. Below the bore the collar stays
                                                             # solid (cap + bearing-shoulder foot).
MORTISE_TOP_GAP      = 4.0                                   # gap from the mortise top up to the
                                                             # top-axle bearing lip (was 5.7)

# ── Spring-strip slit ──────────────────────────────────────────────────────
# Each half has its own slit (cut through Y) so the parts can be slid down
# onto the spring's metal strip from each end of the joint. The slits open at
# the piece's joint-side face (bottom half: +z = mortise opening; top half:
# -z = tenon tip). Where both slits coexist along Z, the strip is held by
# both halves — that's the engagement region.
SLIT_W            = 1.0           # X width of the slit
SLIT_OVERLAP_H    = JOINT_H       # slit overlap == bore depth (the slit fills the joint exactly)
SLIT_EACH_H       = (SLIT_OVERLAP_H + JOINT_H) / 2  # = JOINT_H — each slit spans the whole joint


def _axle_cross_hole(z_center):
    """M2 clearance hole through the axle, axis along y, centered at z."""
    return cq.Workplane("XY").add(cq.Solid.makeCylinder(
        AXLE_CROSS_HOLE_D / 2, AXLE_D + 2,
        pnt=cq.Vector(0, -(AXLE_D / 2 + 1), z_center),
        dir=cq.Vector(0, 1, 0),
    ))


def _build_halves():
    # 45° cone height transitioning the Ø AXLE_PRINT_D shaft to the Ø AXLE_LIP_OD
    # TOP (pancake) bearing lip.
    cone_h = (AXLE_LIP_OD - AXLE_PRINT_D) / 2
    # The mortise collar's chamfered foot (Ø AXLE_PRINT_D → Ø JOINT_MORTISE_OD)
    # doubles as the BOTTOM bearing shoulder, so it gets its own cone height.
    mortise_cone_h = (JOINT_MORTISE_OD - AXLE_PRINT_D) / 2

    # Top bearing lip (pancake) — unchanged.
    pancake_lip_inner_z = PANCAKE_CAP_SEAT_Z0 - AXLE_LIP_H        # bottom face of pancake lip
    cavity_top_z        = pancake_lip_inner_z - cone_h            # bottom of pancake cone (top lip foot)

    # Joint Z layout. The mortise TOP sits MORTISE_TOP_GAP below the top bearing
    # lip; the bore runs JOINT_H deep below that (= slit length). The collar runs
    # FURTHER down to the bottom bearing seat (LEVER_CAP_SEAT_Z1) — its chamfered
    # foot replaces the old separate bottom bearing lip — so below the bore the
    # collar stays solid (cap + shoulder foot).
    mortise_top_z   = cavity_top_z - MORTISE_TOP_GAP             # 37
    bore_bottom_z   = mortise_top_z - JOINT_H                    # 14 (= tenon tip = slit bottom)
    collar_bottom_z = LEVER_CAP_SEAT_Z1 + mortise_cone_h         # 9.75 (chamfer foot on the seat)

    # Bottom now reaches down past the lever stub by TRAY_JOURNAL_H (the service-
    # loop tray journal). axle_z_top is unchanged (= 62) because AXLE_H grew by the
    # same TRAY_JOURNAL_H, so axle_z_bot + AXLE_H lands where it always did.
    axle_z_bot = -(AXLE_EXTRA_LEVER + TRAY_JOURNAL_H)
    axle_z_top = axle_z_bot + AXLE_H

    # ── BOTTOM half (lever-side) — shaft + mortise collar. NO separate bearing
    # lip: the collar's chamfered foot IS the bottom bearing shoulder. ──
    # Print -axle_z (axle bottom) on the bed → grows toward +axle_z.
    bottom = cyl(AXLE_PRINT_D, collar_bottom_z - axle_z_bot, z=axle_z_bot)   # shaft up to collar foot
    # Chamfer Ø AXLE_PRINT_D → Ø JOINT_MORTISE_OD bottoming out at the bearing
    # seat (LEVER_CAP_SEAT_Z1), so the bearing still seats there; its 45° rise
    # also makes the collar's -z underside self-supporting in the print direction.
    bottom = bottom.union(cone_solid(
        d_bottom=AXLE_PRINT_D, d_top=JOINT_MORTISE_OD,
        h=mortise_cone_h, z_base=collar_bottom_z - mortise_cone_h,
    ))
    # Mortise collar (solid Ø JOINT_MORTISE_OD) from the foot up to the mortise top.
    bottom = bottom.union(cyl(JOINT_MORTISE_OD,
                              mortise_top_z - collar_bottom_z,
                              z=collar_bottom_z))
    # Bore (Ø JOINT_MORTISE_HOLE_D, JOINT_H deep, opens at top). Below it the
    # collar stays solid down to collar_bottom_z (cap + bearing-shoulder foot).
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
