"""Pancake-side (TOP) bearing cap + anti-rotation key joining.

Caller must invoke ``apply_to_main_body(main_body)`` to add the matching
groove cut to the main spool body — pure-function style, no side effects.

BEARING CAP FLIPPED: the removable bearing cap is now on the PANCAKE
(top) side; the LEVER-side (bottom) bearing pocket is fused into the
hub (see spool.py). The cap drops into the hub's top cap seat
(PANCAKE_CAP_SEAT_Z0..SPOOL_H) and is keyed against rotation.
"""

import cadquery as cq

from .dimensions import (
    BEARING_BORE, BEARING_LIP_H, BEARING_LIP_ID, BEARING_W,
    CAP_H, CAP_OD,
    HUB_CAVITY_D,
    PANCAKE_CAP_SEAT_Z0, SPOOL_H,
    STRUCT_WALL,
)
from .helpers import cyl, make_keys



# ────────────────────────────────────────────────────────────────────────────
# BEARING CAP — pancake-side (above the spool), removable.
#
# Body at assembly z=PANCAKE_CAP_SEAT_Z0..SPOOL_H (= 43..51), inside the
# hub's TOP cap seat. Bearing presses in from the INTERIOR (spring-cavity)
# face at z=PANCAKE_CAP_SEAT_Z0 (the bottom of the cap); a 90° retention
# lip at the exterior (top, z=SPOOL_H) face catches the bearing. The axle
# exits the spool's top face through the lip ID.
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
CAP_BOSS_WALL = STRUCT_WALL  # wall around the bearing press-fit pocket
CAP_RIM_WALL  = STRUCT_WALL  # outer-wall thickness (tongues + cassette-flange seat)
CAP_RIB_W     = STRUCT_WALL  # radial-spoke width — pinned to STRUCT_WALL
CAP_RIB_COUNT = 6            # built as CAP_RIB_COUNT // 2 full-diameter boxes


def _build_bearing_cap_top():
    boss_od = BEARING_BORE + 2 * CAP_BOSS_WALL          # 28.3
    rim_id  = CAP_OD - 2 * CAP_RIM_WALL                 # 60.7
    z0      = PANCAKE_CAP_SEAT_Z0                        # 43 — interior (spring-cavity) face
    z1      = SPOOL_H                                    # 51 — exterior (top) face

    boss = cyl(boss_od, CAP_H, z=z0)
    rim  = cyl(CAP_OD, CAP_H, z=z0).cut(cyl(rim_id, CAP_H, z=z0))

    cap = boss.union(rim)
    # Radial spokes — full-diameter boxes through the centre; the middle
    # overlaps the boss and the ends overlap the rim (union, harmless).
    n_boxes = CAP_RIB_COUNT // 2
    for i in range(n_boxes):
        rib = (
            cq.Workplane("XY")
            .box(CAP_OD, CAP_RIB_W, CAP_H, centered=(True, True, False))
            .translate((0, 0, z0))
            .rotate((0, 0, 0), (0, 0, 1), i * 180.0 / n_boxes)
        )
        cap = cap.union(rib)

    cap = (
        cap
        # Lip at the TOP (z=SPOOL_H−BEARING_LIP_H..SPOOL_H) — straight 90°
        # shelf at ID=BEARING_LIP_ID (20). The bearing's top outer-race rim
        # lands flat on the lip; axle exits through the lip ID.
        .cut(cyl(BEARING_LIP_ID, BEARING_LIP_H, z=SPOOL_H - BEARING_LIP_H))
        # Pocket — bearing press-fits from the z=z0 (spring-cavity) face.
        .cut(cyl(BEARING_BORE, BEARING_W, z=z0))
    )
    # Anti-rotation tongue keys around the cap OD.
    return cap.union(make_keys(CAP_OD / 2, z0, z1))


# ────────────────────────────────────────────────────────────────────────────
# Anti-rotation keys (tongue & groove) for the pancake-side (top) cap:
#   bearing_cap_top (tongue) → main_body top cap seat (groove)
# ────────────────────────────────────────────────────────────────────────────
bearing_cap_top = _build_bearing_cap_top()


def apply_to_main_body(main_body: cq.Workplane) -> cq.Workplane:
    """Cut the top cap groove keys into the main spool body.
    Returns the modified body."""
    return main_body.cut(
        make_keys(HUB_CAVITY_D / 2, PANCAKE_CAP_SEAT_Z0, SPOOL_H, groove=True)
    )
