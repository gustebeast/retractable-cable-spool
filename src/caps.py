"""Bearing caps (top/bottom) + anti-rotation key joining.

Caller must invoke ``apply_to_main_body(main_body)`` to add the matching
groove cuts to the main spool body — pure-function style, no side effects.

The pancake spool was removed in the source-cable-rework: the source cable
now exits the housing from the lever-side end of the axle, with no source-
side wrapping. ``bearing_cap_top`` is therefore just a plain disc + bearing
pocket + axle bore now (no cone funnel, no tongue cylinder).
"""

import cadquery as cq

from .dimensions import (
    AXLE_D,
    BEARING_BORE, BEARING_LIP_H, BEARING_LIP_ID, BEARING_W,
    CAP_H, CAP_OD,
    HUB_CAVITY_D,
    LEVER_CAP_SEAT_Z0, LEVER_CAP_SEAT_Z1,
    PANCAKE_CAP_SEAT_Z0,
    SPOOL_H,
    TOP_BEARING_BORE,
)
from .helpers import cyl, cone_solid, make_keys



# ────────────────────────────────────────────────────────────────────────────
# BEARING CAPS — two separate parts retaining the 608 bearings at each end.
#
# Both caps have the bearing pressed in from their INTERIOR (spring-cavity)
# face, so the 45° retention lip lives at the exterior face. They share the
# body geometry (CAP_OD × CAP_H, slip-fit into the hub's cap seat).
#
# bearing_cap_bottom (lever side, below the spool)
# ------------------------------------------------
# Body at assembly z=0..8, inside the hub's bottom cap seat. Interior
# face at z=8 (bearing enters here from the spring-cavity side), exterior
# face at z=0 (lip ends here; faces the lever-side housing plate at
# z=-HOUSING_GAP_LEVER).
#
# bearing_cap_top (pancake side, above the spool — opposite the levers)
# ---------------------------------------------------------------------
# Body at assembly z=PANCAKE_CAP_SEAT_Z0..SPOOL_H, inside the hub's top cap seat.
# Pure bearing retainer now — bearing pocket + axle clearance bore + the
# anti-rotation tongues on its OD that key into main_body's cap seat.
# Used to also carry the source-cable pancake (cone funnel + tongue
# cylinder + pancake_spool disc); those are gone now that the source
# cable exits the housing through the lever-side end of the axle.
# ────────────────────────────────────────────────────────────────────────────

CABLE_OD = 6.0    # source cable OD (USB-A-to-C)

_axle_bore_d = AXLE_D + 2.0   # 10 mm — 1 mm radial clearance between cap
                              # and axle through the funnel/clearance bore.

# ---- bearing_cap_bottom -----------------------------------------------------
# Post-flip assembly position: z = 0 .. CAP_H (=8). Exterior face at z=0
# (lip, narrow end, adjacent to lever-side housing plate). Interior face at
# z=CAP_H (pocket opening, adjacent to spring cavity at z=9..29).

def _build_bearing_cap_bottom():
    cap = (
        cyl(CAP_OD, CAP_H, z=0)
        # Lip z=0..1 — straight 90° shelf at ID=BEARING_LIP_ID (20). The
        # bearing's bottom outer-race rim (r=10..11) lands flat on the lip.
        .cut(cyl(BEARING_LIP_ID, BEARING_LIP_H, z=0))
        # Pocket z=1..CAP_H — bearing press-fits from the z=CAP_H face.
        .cut(cyl(BEARING_BORE, BEARING_W, z=BEARING_LIP_H))
    )
    # Anti-rotation tongue keys around the cap OD.
    return cap.union(
        make_keys(CAP_OD / 2, LEVER_CAP_SEAT_Z0, LEVER_CAP_SEAT_Z1)
    )

# ---- bearing_cap_top --------------------------------------------------------
# Plain disc with bearing pocket and axle clearance bore. Anti-rotation
# tongues on the OD key into main_body's top cap seat.
#
# Post-flip assembly position: z = PANCAKE_CAP_SEAT_Z0..SPOOL_H = 43..51.
#
# Print orientation: disc face on build plate, axle bore pointing up.

_top_cap_body_z0   = PANCAKE_CAP_SEAT_Z0      # 43 — body interior face (= main body cap seat)
_top_cap_body_z1   = _top_cap_body_z0 + CAP_H # 51 — body exterior face (= main body top face)

pancake_bearing_z0 = _top_cap_body_z0         # 43 — bearing pocket Z (kept name for viz.py compat)
_pancake_lip_z0    = pancake_bearing_z0 + BEARING_W  # 50 — top of bearing pocket (where the funnel starts)


def _build_bearing_cap_top():
    cap = (
        # Disc body — slip-fits into main_body's top cap seat.
        cyl(CAP_OD, CAP_H, z=_top_cap_body_z0)
        # Bearing pocket — TOP_BEARING_BORE is 0.1 mm looser than BEARING_BORE
        # so the top bearing drops in by hand.
        .cut(cyl(TOP_BEARING_BORE, BEARING_W, z=pancake_bearing_z0))
        # 45° funnel from pocket top to axle bore. Bearing outer race wedges
        # into the cone where it has narrowed to D=axle_bore.
        .cut(cone_solid(TOP_BEARING_BORE, _axle_bore_d,
                        (TOP_BEARING_BORE - _axle_bore_d) / 2,
                        _pancake_lip_z0))
        # Axle clearance bore from funnel top through the rest of the cap.
        .cut(cyl(_axle_bore_d,
                 _top_cap_body_z1 - (_pancake_lip_z0 + (TOP_BEARING_BORE - _axle_bore_d) / 2),
                 z=_pancake_lip_z0 + (TOP_BEARING_BORE - _axle_bore_d) / 2))
    )
    # Anti-rotation tongues on the cap OD → main_body's top cap-seat groove.
    cap = cap.union(make_keys(CAP_OD / 2, _top_cap_body_z0, _top_cap_body_z1))
    return cap


# ────────────────────────────────────────────────────────────────────────────
# Anti-rotation keys (tongue & groove) at each spoke angle:
#   bearing_cap_bottom (tongue) → main_body bottom cap seat (groove)
#   bearing_cap_top    (tongue) → main_body top cap seat    (groove)
# Tongue: 2 mm tangential × full-axial × 1 mm radial protrusion.
# Groove: 2.2 × full-axial × 1.2 mm — 0.2 mm per-side slack to drop in by hand.
# ────────────────────────────────────────────────────────────────────────────
bearing_cap_bottom = _build_bearing_cap_bottom()
bearing_cap_top    = _build_bearing_cap_top()


def apply_to_main_body(main_body: cq.Workplane) -> cq.Workplane:
    """Cut the bottom + top cap groove keys into the main spool body.
    Returns the modified body."""
    main_body = main_body.cut(
        make_keys(HUB_CAVITY_D / 2, LEVER_CAP_SEAT_Z0, LEVER_CAP_SEAT_Z1, groove=True)
    )
    main_body = main_body.cut(
        make_keys(HUB_CAVITY_D / 2, PANCAKE_CAP_SEAT_Z0, SPOOL_H, groove=True)
    )
    return main_body
