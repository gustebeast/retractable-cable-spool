"""Main spool body — drum, flanges, hub, cap pockets, pancake flange, spring slot.

Lever-dependent cuts (ratchet teeth, drum cable hole) are applied later
in `levers.py`.
"""

import cadquery as cq

from .dimensions import (
    BOOL_OVERSHOOT,
    CAP_H, CAP_STOP_ID, CAP_STOP_LIP_H,
    CAVITY_Z0, CAVITY_Z1,
    DRUM_BOTTOM_Z, DRUM_H, DRUM_ID, DRUM_OD, DRUM_TOP_Z,
    FLANGE_H,
    HUB_CAVITY_D, HUB_OD,
    LEVER_CAP_SEAT_Z0, LEVER_CAP_SEAT_Z1,
    LEVER_STOP_LIP_Z0, LEVER_STOP_LIP_Z1,
    PANCAKE_CAP_SEAT_Z0, PANCAKE_STOP_LIP_Z0,
    SPOKE_W, SPOOL_H,
)
from .helpers import (
    cyl, cone_solid,
    lever_flange_solid, pancake_flange_solid,
    spokes_solid,
)

# ────────────────────────────────────────────────────────────────────────────
# MAIN BODY — the full spool
#   Bottom flange, drum, top flange, 4 spokes, hub with bottom bearing pocket,
#   straight spring cavity, 1 mm-inset cap-stop lip, cap seat at top.
#   The cavity opens straight to the top face (no obstruction) — spring drops
#   in from above, then the bearing cap slides in after.
# ────────────────────────────────────────────────────────────────────────────

def _build_main_body():
    drum = cyl(DRUM_OD, DRUM_H, z=DRUM_BOTTOM_Z).cut(
           cyl(DRUM_ID, DRUM_H, z=DRUM_BOTTOM_Z))

    main_body = (
        cyl(HUB_OD, SPOOL_H, z=0)                       # hub cylinder (full height)
        .union(pancake_flange_solid(DRUM_TOP_Z))         # pancake-side flange (z=SPOOL_H-FLANGE_H..SPOOL_H)
        .union(drum)
        .union(lever_flange_solid(DRUM_BOTTOM_Z))        # lever-side flange (z=0..FLANGE_H) with ratchet
        .union(spokes_solid(0, SPOOL_H))                 # spokes with taper at lever side
    )

    # Interior z-map cuts — symmetric cap seats at both ends:
    #   SPOOL_H .. PANCAKE_CAP_SEAT_Z0     — top cap seat (HUB_CAVITY_D = 34 mm ID)
    #   PANCAKE_CAP_SEAT_Z0 .. PANCAKE_STOP_LIP_Z0 — top cap-stop lip (CAP_STOP_ID = 32 mm, 1 mm)
    #   PANCAKE_STOP_LIP_Z0 .. CAVITY_Z0   — straight spring cavity (34 mm ID, 20 mm tall)
    #   CAVITY_Z0 .. LEVER_STOP_LIP_Z0 — bottom cap-stop lip (CAP_STOP_ID → HUB_CAVITY_D)
    #   LEVER_CAP_SEAT_Z1 .. 0       — bottom cap seat (HUB_CAVITY_D = 34 mm ID)
    main_body = (
        main_body
        # Bottom cap seat (0..CAP_H)
        .cut(cyl(HUB_CAVITY_D, CAP_H, z=LEVER_CAP_SEAT_Z0))
        # Bottom cap-stop lip — inverted cone opening upward into the spring
        # cavity. At z=LEVER_STOP_LIP_Z0 the ID is CAP_STOP_ID (narrow, holds
        # cap from sliding up); at z=LEVER_STOP_LIP_Z1 it widens to HUB_CAVITY_D
        # (matches the spring cavity above).
        .cut(cone_solid(CAP_STOP_ID, HUB_CAVITY_D, CAP_STOP_LIP_H, LEVER_STOP_LIP_Z0))
        # Spring cavity (9..29)
        .cut(cyl(HUB_CAVITY_D,    CAVITY_Z1 - CAVITY_Z0,        z=CAVITY_Z0))
        # Top cap-stop lip — HUB_CAVITY_D at bottom (PANCAKE_STOP_LIP_Z0) narrowing
        # to CAP_STOP_ID at top (PANCAKE_CAP_SEAT_Z0). Self-supporting underside,
        # and the slope coincides with the spring slot's sloped roof.
        .cut(cone_solid(HUB_CAVITY_D, CAP_STOP_ID, CAP_STOP_LIP_H, PANCAKE_STOP_LIP_Z0))
        # Top cap seat (30..38)
        .cut(cyl(HUB_CAVITY_D,    SPOOL_H - PANCAKE_CAP_SEAT_Z0,        z=PANCAKE_CAP_SEAT_Z0))
    )
    main_body = main_body.cut(_spring_slot())
    return main_body


# (The OD-88 pancake-spool floor disc that used to sit at z=49..51 was
# removed; the source-cable wrap groove no longer has a spool-side floor.
# The cable now exits the drum interior upward through the open spool top
# and reaches the cone via a dedicated entry cut through the hub wall +
# bearing_cap_top, defined in caps.py.)

# Spring outer-end attachment slot — a blind radial notch in the hub
# wall sized to receive the spring's bent-over outer tab (12.7 mm z x
# 0.15 mm thick). Orientation: tall in z (matching the strip width),
# narrow in y (fits the strip thickness with tolerance), shallow in x.
# Blind outer face leaves ≥1.5 mm of hub-OD wall intact — no slits.
#
# Conical top: the slot's roof IS the cap-stop lip's 45° cone surface,
# extended. Both the lip underside (z in [PANCAKE_STOP_LIP_Z0, PANCAKE_STOP_LIP_Z0 +
# CAP_STOP_LIP_H]) and the slot roof (below) lie on the same cone
# r = HUB_CAVITY_D/2 + PANCAKE_STOP_LIP_Z0 − z, apex at (0, 0, PANCAKE_STOP_LIP_Z0 +
# HUB_CAVITY_D/2). Using the cone itself to trim the slot guarantees
# flush alignment at every y, not just y=0 (a flat-plane approximation
# would drift ~0.03 mm off the cone at y=±SPRING_SLOT_W/2).
SPRING_SLOT_H_RECT = 23.0   # rectangle height (z_bottom to outer-edge roof).
                            # Sized for the 21.8 mm Stanley tape blade with
                            # 1.2 mm of axial play.
SPRING_SLOT_W      =  2.0   # y-extent (perpendicular to blade flat) — fits
                            # 0.1 mm tape with bend clearance
SPRING_SLOT_DEPTH  =  4.0   # x-extent (radial). Cuts all the way through
                            # the 3 mm hub wall (with 0.5 mm overshoot
                            # each side) — tape blade threads through
                            # the slot from the cavity, gets bent on the
                            # outer face to lock in place.
SPRING_SLOT_ANGLE_DEG = 210.0  # angular position of the slot. 210° sits
                            # halfway between the 180° and 240° spokes
                            # so the slot doesn't intersect any spoke.

def _spring_slot():
    slot_x_outer = -(HUB_CAVITY_D / 2 + SPRING_SLOT_DEPTH - 0.5)   # outer x face of slot
    slot_x_inner = -(HUB_CAVITY_D / 2 - 0.5)                       # inner x face (cavity-side)
    # At outer x (r=|slot_x_outer|), the lever-side cap-stop-lip cone surface
    # extending UP into the cavity reaches r=|slot_x_outer| at this z; this
    # is the slot's lower bound at that x.
    slot_zb_at_outer = LEVER_STOP_LIP_Z1 + (SPRING_SLOT_DEPTH - 0.5)
    slot_ztop = slot_zb_at_outer + SPRING_SLOT_H_RECT             # slot's upper face
    lip_apex_z = LEVER_STOP_LIP_Z1 - HUB_CAVITY_D / 2              # cone apex (below spool body)

    # Rectangular prism that overshoots BELOW the apex in z; the cone
    # intersection below trims its bottom surface to the cone.
    slot_prism = (
        cq.Workplane("XY")
        .workplane(offset=lip_apex_z - 1.0)
        .center((slot_x_outer + slot_x_inner) / 2, 0)
        .box(
            abs(slot_x_inner - slot_x_outer),
            SPRING_SLOT_W,
            slot_ztop - (lip_apex_z - 1.0),
            centered=(True, True, False),
        )
    )

    # The lever-side cap-stop-lip cone, extended UPWARD from its apex —
    # shifted 0.02 mm DOWNWARD from the true lip apex so the slot's roof
    # surface is ever so slightly DEEPER than the actual lip cone (by
    # 0.02 mm / 20 µm at each z). Without this offset, the cut operation
    # produces two mathematically coincident cone surfaces (the lip's
    # topside and the slot's floor), which OCCT's boolean fails to merge
    # cleanly and marks the result invalid.
    lip_cone_offset = 0.02
    lip_cone_height = SPOOL_H - lip_apex_z + 1.0    # tall enough to cover the slot
    lip_cone_ext = cq.Workplane("XY").add(cq.Solid.makeCone(
        0.001, lip_cone_height + 0.001, lip_cone_height,
        pnt=cq.Vector(0, 0, lip_apex_z - lip_cone_offset),
        dir=cq.Vector(0, 0, 1),
    ))

    slot = slot_prism.intersect(lip_cone_ext)
    # Rotate from constructed position at 180° (-x) to SPRING_SLOT_ANGLE_DEG.
    return slot.rotate((0, 0, 0), (0, 0, 1), SPRING_SLOT_ANGLE_DEG - 180.0)


main_body = _build_main_body()
