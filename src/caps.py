"""Lever-side bearing cap + anti-rotation key joining.

Caller must invoke ``apply_to_main_body(main_body)`` to add the matching
groove cut to the main spool body — pure-function style, no side effects.

Only the LEVER-side bearing cap is a separately-printed part now. The
PANCAKE-side bearing pocket is fused into the housing as a stem
extending from the pancake plate down into the spool's hub cavity (see
``_pancake_bearing_carrier`` in housing.py); the formerly-separate
``bearing_cap_top`` and its tongue/groove keying are gone.
"""

import cadquery as cq

from .dimensions import (
    BEARING_BORE, BEARING_LIP_H, BEARING_LIP_ID, BEARING_W,
    CAP_H, CAP_OD,
    HUB_CAVITY_D,
    LEVER_CAP_SEAT_Z0, LEVER_CAP_SEAT_Z1,
    STRUCT_WALL,
)
from .helpers import cyl, make_keys



# ────────────────────────────────────────────────────────────────────────────
# BEARING CAP — lever-side (below the spool).
#
# Bearing is pressed in from the INTERIOR (spring-cavity) face, with a
# 45° retention lip at the exterior face.
#
# Body at assembly z=0..CAP_H (=8), inside the hub's bottom cap seat.
# Interior face at z=CAP_H (bearing enters here from the spring-cavity
# side). Exterior face at z=0 (lip ends here; faces the lever-side
# housing plate at z=-HOUSING_GAP_LEVER).
# ────────────────────────────────────────────────────────────────────────────


# Lightened bearing cap. Instead of a solid Ø66.7 × 8 puck (the metal-
# heavy original), the cap is just:
#   - a CAP_BOSS_WALL-thick collar around the Ø22.3 press-fit pocket
#     (with the bearing-retention lip at its base),
#   - a CAP_RIM_WALL-thick outer rim — carries the anti-rotation tongues,
#     slip-fits the hub cavity, and seats the Ø64 Stanley cassette flange
#     on its top face (the flange lands at r≈32, inside the rim's
#     r=rim_id/2..CAP_OD/2 top annulus),
#   - CAP_RIB_COUNT radial spokes tying boss → rim, full cap height.
# No solid web — the cap is open on both faces between the spokes (the
# cassette's central bore is Ø13.4; its flange bears only on the rim top,
# the lever housing plate sits 2 mm clear of the bottom face).
CAP_BOSS_WALL = 3.0          # wall around the bearing press-fit pocket
CAP_RIM_WALL  = 3.0          # outer-wall thickness (tongues + cassette-flange seat)
CAP_RIB_W     = STRUCT_WALL  # radial-spoke width — pinned to STRUCT_WALL
CAP_RIB_COUNT = 6            # built as CAP_RIB_COUNT // 2 full-diameter boxes


def _build_bearing_cap_bottom():
    boss_od = BEARING_BORE + 2 * CAP_BOSS_WALL          # 28.3
    rim_id  = CAP_OD - 2 * CAP_RIM_WALL                 # 60.7

    boss = cyl(boss_od, CAP_H, z=0)
    rim  = cyl(CAP_OD, CAP_H, z=0).cut(cyl(rim_id, CAP_H, z=0))

    cap = boss.union(rim)
    # Radial spokes — full-diameter boxes through the centre; the middle
    # overlaps the boss and the ends overlap the rim (union, harmless).
    n_boxes = CAP_RIB_COUNT // 2
    for i in range(n_boxes):
        rib = (
            cq.Workplane("XY")
            .box(CAP_OD, CAP_RIB_W, CAP_H, centered=(True, True, False))
            .rotate((0, 0, 0), (0, 0, 1), i * 180.0 / n_boxes)
        )
        cap = cap.union(rib)

    cap = (
        cap
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


# ────────────────────────────────────────────────────────────────────────────
# Anti-rotation keys (tongue & groove) for the lever-side cap only:
#   bearing_cap_bottom (tongue) → main_body bottom cap seat (groove)
# Tongue: KEY_W tangential × full-axial × KEY_DEPTH radial protrusion.
# Groove oversized by FIT_CLR (0.15 mm) per side tangentially and at the
# radial tip — drops in by hand without wobble.
# ────────────────────────────────────────────────────────────────────────────
bearing_cap_bottom = _build_bearing_cap_bottom()


def apply_to_main_body(main_body: cq.Workplane) -> cq.Workplane:
    """Cut the bottom cap groove keys into the main spool body.
    Returns the modified body."""
    return main_body.cut(
        make_keys(HUB_CAVITY_D / 2, LEVER_CAP_SEAT_Z0, LEVER_CAP_SEAT_Z1, groove=True)
    )
