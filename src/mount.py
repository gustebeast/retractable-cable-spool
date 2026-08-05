"""MOUNT — desk/wall TWIST-LOCK DOUBLE RING (params block for the full
story).

ONE continuous flush part: an OUTER ring near the arm ends and an INNER
ring at half the diameter (user: one ring is flimsy), EACH carrying
four hanging arc tenons into its own channels in the arms, tied by
four mid-quadrant SPOKES, with four AXIS-HUGGING screw pads (use the
pair matching your mounting beam's axis — x or y). Screw the mount to
the wood first, offer the assembly up (all eight tenons pass through
the open quadrants), rotate ~11° to the stops. Rotation keeps both
rings on their own circles, so only the spokes and pads move over the
frame's plan — and they live in the 90°-wide quadrants.

Joint: cadkit's OCTAGON (stop-sign) ARC — the standard for two hosts
printing the same direction (+z→−z): the cavities CLOSE on their
one-bead roofs inside the arms with a ≥1.6 floor below (user's call —
the earlier THROUGH mushroom left the arms' undersides open and weak;
the spine shrank to MOUNT_SPINE_T to buy the octagon its room). The
octagon carries the 0.30 Z-RELIEF natively now (user's mechanism: the
tenon's post and the cavity's waist vertical both grow by the depth
gap, opening the flare z-sandwich — the faces that grab — by 0.30
while the arm's printed neck wall keeps its tier). SITED via cadkit's
ring↔arm CROSSING, the SAME arrangement as the frame_top↔frame_bottom
joints (user: shared code, different profile) evaluated per ring
radius (angles scale up at the inner ring). Spines are one stem wide;
over each arm's stop-wall zone they ride annular V-bottom grooves.

PRINTS +z→−z (top face on the bed): rings, spokes and pads on the bed;
the eight arc tenons rise library-native. Screw countersinks open at
the world underside — v2's validated screw geometry.
"""

import math

import cadquery as cq

from cadkit.joinery import PrintSpec, joint

from .params import (
    NOZZLE, FRAME_Z1, FRAME_RIB,
    MOUNT_TEN_W, MOUNT_TEN_ARC, MOUNT_SPINE_T, MOUNT_FLOOR_MIN,
    MOUNT_PLATE_T, MOUNT_MATE_Z,
    MOUNT_RING_R, MOUNT_RING2_R, MOUNT_SPINE_W,
    MOUNT_SPOKE_W, MOUNT_SPOKE_AZ, MOUNT_PAD_W, MOUNT_PAD_AZ,
    MOUNT_GRV_W, MOUNT_GRV_VERT, MOUNT_GRV_FLAT, MOUNT_GRV_DEPTH,
    TOP_JOINT_SEAT_CLR, TOP_ENTRY_OVER,
    WOOD_SCREW_SHAFT_D, WOOD_SCREW_HEAD_D, WOOD_SCREW_HEAD_H,
)

_SPEC = PrintSpec(nozzle=NOZZLE, material="PETG-GF", facing="up")
# cadkit's standard for two same-direction hosts: the OCTAGON — its
# cavity CLOSES on the one-bead roof inside the arm (user's call: the
# old THROUGH mushroom left the arm's underside open and weak). The GF
# specs give it the 0.30 z-relief natively (joint height includes it).
MOUNT_J = joint(MOUNT_TEN_W, None, tenon=_SPEC, mortise=_SPEC,
                install="-x")
assert (MOUNT_MATE_Z - (MOUNT_J.height + MOUNT_J.clearance)
        >= MOUNT_FLOOR_MIN - 1e-9), \
    "octagon cavity leaves under 1.6 of arm floor — shrink MOUNT_TEN_W"

_SITES = (0.0, 90.0, 180.0, 270.0)
_RINGS = (MOUNT_RING_R, MOUNT_RING2_R)

# ONE crossing per ring — cadkit's ring↔arm arrangement, shared verbatim
# with the frame arc joints (same seat/overshoot constants, same seating
# chirality): tenon anchored at the arm's ENTRY face spanning
# MOUNT_TEN_ARC (half the crossing — the stop-side half stays solid arm)
_CROSS = {R: MOUNT_J.crossing(R, FRAME_RIB, MOUNT_TEN_ARC,
                              seat=TOP_JOINT_SEAT_CLR, over=TOP_ENTRY_OVER)
          for R in _RINGS}

# the shared install rotation: the DEEPER offset of the two rings' entries
MOUNT_MATE_DEG = max(c.free for c in _CROSS.values()) + 1.0     # ≈ 11.1°


def _hang(w):
    """Library-native arc solid (bulb up, mating plane z=0) → hanging
    from the flush spine: z-mirror (plan angles preserved), mating plane
    to MOUNT_MATE_Z."""
    return w.mirror("XY").translate((0.0, 0.0, MOUNT_MATE_Z))


def _screw_cut(x, y):
    """One wood-screw cutter (world pose): Ø4 shaft through, countersink
    cone opening at the pad's UNDERSIDE — the head sits flush there,
    driven up into the wood. v2's validated geometry, re-posed."""
    z0 = FRAME_Z1 - MOUNT_PLATE_T
    slope = (WOOD_SCREW_HEAD_D - WOOD_SCREW_SHAFT_D) / 2.0 / WOOD_SCREW_HEAD_H
    over = 0.5
    shaft = cq.Solid.makeCylinder(
        WOOD_SCREW_SHAFT_D / 2.0, MOUNT_PLATE_T + 1.0,
        cq.Vector(x, y, z0 - 0.5), cq.Vector(0, 0, 1))
    cone = cq.Solid.makeCone(
        WOOD_SCREW_HEAD_D / 2.0 + over * slope,
        WOOD_SCREW_SHAFT_D / 2.0,
        WOOD_SCREW_HEAD_H + over,
        cq.Vector(x, y, z0 - over), cq.Vector(0, 0, 1))
    return cq.Workplane(obj=shaft).union(cq.Workplane(obj=cone))


def _annulus(r0, r1, z0, z1):
    return (cq.Workplane("XY").workplane(offset=z0)
            .circle(r1).circle(r0).extrude(z1 - z0))


def _build_mount():
    """Built with clean=False THROUGHOUT and NO heal: OCC's unify/clean
    stage corrupts this compound (drops all solids, or — via heal's
    unifier — fills a screw hole). The raw fuse chain is one valid
    solid; the screw-hole probes at the gate keep it honest."""
    hw = MOUNT_SPINE_W / 2.0
    m = None
    for R in _RINGS:
        ring = _annulus(R - hw, R + hw, MOUNT_MATE_Z, FRAME_Z1)
        for site in _SITES:
            # hanging octagon arc tenons, SEATED (the crossing pre-places
            # them: anchored at the arm's ENTRY face, spanning half the
            # crossing inboard — the stop-side half stays solid arm)
            ring = ring.union(
                _hang(_CROSS[R].tenon(root=2.0))
                .rotate((0, 0, 0), (0, 0, 1), site),
                clean=False)
        m = ring if m is None else m.union(ring, clean=False)
    # mid-quadrant SPOKES tying the rings (full plate depth — they hang
    # below the shallow spine level, over open quadrant air)
    for az in MOUNT_SPOKE_AZ:
        m = m.union(
            cq.Workplane("XY").workplane(offset=FRAME_Z1 - MOUNT_PLATE_T)
            .center((MOUNT_RING_R + MOUNT_RING2_R) / 2.0, 0.0)
            .rect(MOUNT_RING_R - MOUNT_RING2_R + MOUNT_SPINE_W,
                  MOUNT_SPOKE_W)
            .extrude(MOUNT_PLATE_T)
            .rotate((0, 0, 0), (0, 0, 1), az), clean=False)
    # AXIS-ALIGNED screw pads hugging the arms (use the pair on your
    # beam's axis); screws at the outer ring line
    for az in MOUNT_PAD_AZ:
        x = MOUNT_RING_R * math.cos(math.radians(az))
        y = MOUNT_RING_R * math.sin(math.radians(az))
        m = m.union(
            cq.Workplane("XY").workplane(offset=FRAME_Z1 - MOUNT_PLATE_T)
            .center(x, y).rect(MOUNT_PAD_W, MOUNT_PAD_W)
            .extrude(MOUNT_PLATE_T), clean=False)
    for az in MOUNT_PAD_AZ:
        x = MOUNT_RING_R * math.cos(math.radians(az))
        y = MOUNT_RING_R * math.sin(math.radians(az))
        m = m.cut(_screw_cut(x, y), clean=False)
    return m


def _v_groove_arc(R, a0, a1):
    """Annular V-bottom groove segment for a spine (profile in the r-z
    plane, revolved a0→a1): vertical walls MOUNT_GRV_VERT deep, then the
    45° V to the dulled flat — the groove's −z boundary is the roof of
    the slot in the frame's +z→−z print."""
    hw = MOUNT_GRV_W / 2.0
    hf = MOUNT_GRV_FLAT / 2.0
    z1 = FRAME_Z1 + 1.0
    zv = FRAME_Z1 - MOUNT_GRV_VERT
    zb = FRAME_Z1 - MOUNT_GRV_DEPTH
    return (cq.Workplane("XZ")
            .polyline([(R - hw, z1), (R + hw, z1), (R + hw, zv),
                       (R + hf, zb), (R - hf, zb), (R - hw, zv)])
            .close().revolve(a1 - a0, (0, 0), (0, 1))
            .rotate((0, 0, 0), (0, 0, 1), a0))


def mount_channel_cuts():
    """frame_top's cutters — per arm, per ring: the crossing's retained
    arc CAVITY (open past the arm's CCW face — the entry; its CW end
    wall, seat-shy of the seated tenon, is the stop — the frame arc
    joints' arrangement verbatim, same seating rotation) plus the
    spine's V-groove over the stop-wall zone so the rings pass at every
    install angle."""
    cuts = []
    for R in _RINGS:
        c = _CROSS[R]
        for site in _SITES:
            cuts.append(
                _hang(c.mortise(drop=MOUNT_SPINE_T + 0.5))
                .rotate((0, 0, 0), (0, 0, 1), site))
            cuts.append(_v_groove_arc(R, site - c.arm - c.over,
                                      site + c.arm - c.ten - c.seat + 0.1))
    return cuts


mount_ring = _build_mount()
