"""Removable bearing cap — built in the housing's PRE-FLIP (print) frame.

Everything here models the cap at the +Z seat (PANCAKE_CAP_SEAT_Z0..
SPOOL_H) of the pre-flip housing; spring_housing.py MIRRORS it at export,
so in the ASSEMBLY the cap seats at the BOTTOM (user's call: the housing
hangs from the axle's top-bearing lip, so the TOP pocket must be FUSED —
a top-side cap would carry that hanging load on a removable part; the
bottom cap just idles under its floating bearing, held by the frame).
The cap drops into the seat on a SMOOTH CONTACT RING — no keys, no
grooves, no z retention beyond the blind tenon stop (the frame blocks
removal once assembled). The cap's own print is the pre-flip pose,
exterior face down — exactly as modelled here.
"""

import cadquery as cq

from v2.dimensions import NOZZLE  # bead-multiple sizing unit
from .dimensions import (
    BEARING_BORE, BEARING_LIP_H, BEARING_LIP_ID, BEARING_W,
    CAP_H, CAP_OD,
    HUB_CAVITY_D, KEY_ANGLES,
    PANCAKE_CAP_SEAT_Z0, SPOOL_H,
    STRUCT_WALL,
)
from .helpers import cyl

# ── Cap-seat TONGUE & GROOVE (user redesign) ─────────────────────────────────
# Six radial TENONS on the housing's cap-seat wall (one per spoke angle,
# 1.6 × 1.6) mate MORTISES in the cap's rim, each backed by a local BOSS
# so ≥ 1.6 of material surrounds the pocket on every side. No single joint
# retains by shape — the RADIAL SET mutually locks rotation (basic
# tongue-and-groove suffices). Z RETENTION: tenon AND mortise stop
# CAP_KEY_STOP short of the top face — the blind mortise ROOF lands on the
# tenon TOP, replacing the old cap-stop lip's inward stop at ZERO radial
# cost (the frame blocks the outward direction once assembled). The
# anchor's gate/insert at 270° threads between the 240° and 300° tenons
# by construction.
CAP_KEY_W    = 2 * NOZZLE                    # tenon tangential width
CAP_KEY_D    = 2 * NOZZLE                    # tenon radial protrusion
CAP_KEY_CLR  = 0.15                   # per-side fit
CAP_KEY_STOP = 2 * NOZZLE                    # tenon/mortise stop short of the top —
                                      # the blind roof = the cap's depth stop
_SEAT_R      = HUB_CAVITY_D / 2.0     # 31.6 — the cap-seat wall radius
_KEY_TOP     = SPOOL_H - CAP_KEY_STOP # 49.4 — tenon top = mortise roof


def cap_seat_tenons():
    """UNION solid for the HOUSING: one axial 1.6 × 1.6 rib per spoke on
    the cap-seat wall, 45°-chamfered at its lower end (the open cavity is
    below it in the −Z→+Z print), stopping at _KEY_TOP — the flat tenon
    top is the cap's seating stop."""
    out = None
    for a in KEY_ANGLES:
        rib = (cq.Workplane("XZ")
               .polyline([(_SEAT_R + 0.2, PANCAKE_CAP_SEAT_Z0 - 0.2),
                          (_SEAT_R - CAP_KEY_D,
                           PANCAKE_CAP_SEAT_Z0 - 0.2 + (CAP_KEY_D + 0.2)),
                          (_SEAT_R - CAP_KEY_D, _KEY_TOP),
                          (_SEAT_R + 0.2, _KEY_TOP)])
               .close().extrude(CAP_KEY_W / 2.0, both=True)
               .rotate((0, 0, 0), (0, 0, 1), a))
        out = rib if out is None else out.union(rib)
    return out



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
    # Cap-seat MORTISES: a boss behind each notch keeps 1.6 of floor, the
    # boss flanks and the rim provide ≥ 1.6 side walls; the notch passes
    # the housing tenon with CAP_KEY_CLR per side and stops at the BLIND
    # ROOF (_KEY_TOP — 1.6 of material to the top face): the roof landing
    # on the tenon top is the cap's inward z-stop. Bosses union first,
    # then all notches cut (through the rim + into the boss face). In the
    # cap's print (exterior face down) the pockets are blind holes opening
    # upward — no bridges.
    r_floor = _SEAT_R - CAP_KEY_D - CAP_KEY_CLR         # 29.85 — mortise floor
    nw = CAP_KEY_W / 2.0 + CAP_KEY_CLR                  # notch half-width
    bw = nw + 2 * NOZZLE                                       # boss half-width
    for a in KEY_ANGLES:
        boss = (cq.Workplane("XY").workplane(offset=z0)
                .polyline([(r_floor - 2 * NOZZLE, -bw), (rim_id / 2.0 + 0.3, -bw),
                           (rim_id / 2.0 + 0.3, bw), (r_floor - 2 * NOZZLE, bw)])
                .close().extrude(CAP_H)
                .rotate((0, 0, 0), (0, 0, 1), a))
        cap = cap.union(boss)
    for a in KEY_ANGLES:
        notch = (cq.Workplane("XY").workplane(offset=z0 - 0.5)
                 .polyline([(r_floor, -nw), (CAP_OD / 2.0 + 0.5, -nw),
                            (CAP_OD / 2.0 + 0.5, nw), (r_floor, nw)])
                 .close().extrude((_KEY_TOP - z0) + 0.5)
                 .rotate((0, 0, 0), (0, 0, 1), a))
        cap = cap.cut(notch)
    return cap


bearing_cap_top = _build_bearing_cap_top()
