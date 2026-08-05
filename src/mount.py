"""MOUNT — desk/wall TWIST-LOCK DOUBLE RING (params block for the full
story).

ONE continuous flush part: an OUTER ring near the arm ends and an INNER
ring at half the diameter (user: one ring is flimsy), EACH carrying
four hanging arc tenons into its own channels in the arms, tied by
four mid-quadrant SPOKES, with four AXIS-HUGGING screw pads (use the
pair matching your mounting beam's axis — x or y). Screw the mount to
the wood first, offer the assembly up (all eight tenons pass through
the open quadrants), rotate ~17° to the stops. Rotation keeps both
rings on their own circles, so only the spokes and pads move over the
frame's plan — and they live in the 90°-wide quadrants.

Joint: the flat-top mushroom ARC (cadkit, THROUGH mortises — both
hosts print +z→−z; the cavities exit the arms' undersides). Angular
constants are the frame arc joints' own, evaluated per ring radius
(they scale up at the inner ring). Spines are one stem wide; over each
arm's stop-wall zone they ride annular V-bottom grooves.

PRINTS +z→−z (top face on the bed): rings, spokes and pads on the bed;
the eight arc tenons rise library-native. Screw countersinks open at
the world underside — v2's validated screw geometry.
"""

import math

import cadquery as cq

from cadkit.joinery import PrintSpec, joint

from .params import (
    NOZZLE, FRAME_Z1, FRAME_RIB,
    MOUNT_TEN_W, MOUNT_TEN_H, MOUNT_TEN_ARC, MOUNT_THRU_D,
    MOUNT_PLATE_T, MOUNT_MATE_Z,
    MOUNT_RING_R, MOUNT_RING2_R, MOUNT_SPINE_W,
    MOUNT_SPOKE_W, MOUNT_SPOKE_AZ, MOUNT_PAD_W, MOUNT_PAD_AZ,
    MOUNT_GRV_W, MOUNT_GRV_VERT, MOUNT_GRV_FLAT, MOUNT_GRV_DEPTH,
    TOP_JOINT_SEAT_CLR, TOP_ENTRY_OVER,
    WOOD_SCREW_SHAFT_D, WOOD_SCREW_HEAD_D, WOOD_SCREW_HEAD_H,
)

_SPEC = PrintSpec(nozzle=NOZZLE, material="PETG-GF", facing="up")
MOUNT_J = joint(MOUNT_TEN_W, None, tenon=_SPEC, mortise=_SPEC,
                install="-x", depth=MOUNT_THRU_D, through=True)

_SITES = (0.0, 90.0, 180.0, 270.0)
_RINGS = (MOUNT_RING_R, MOUNT_RING2_R)


def _angles(R):
    """(arm_half, seat, over, tenon_sweep) in degrees at radius R. The
    tenon ANCHORS AT THE ARM'S ENTRY FACE and spans MOUNT_TEN_ARC of
    arc inboard (user: the stop side of the crossing stays solid arm —
    the stop wall is wherever the cavity ends, about half the crossing
    in); seat/over are the frame arc joints' constants, arc-converted."""
    arm = math.degrees(math.asin((FRAME_RIB / 2.0) / R))
    seat = math.degrees(TOP_JOINT_SEAT_CLR / R)
    over = math.degrees(TOP_ENTRY_OVER / R)
    ten = math.degrees(MOUNT_TEN_ARC / R)
    assert ten <= 2.0 * arm - seat - math.degrees(2 * NOZZLE / R) + 1e-9, \
        f"mount tenon arc at r={R} leaves no stop-side arm material"
    return arm, seat, over, ten


# the shared install rotation: the DEEPER offset of the two rings' entries
MOUNT_MATE_DEG = max(_angles(R)[2] + _angles(R)[3]
                     for R in _RINGS) + 1.0                     # ≈ 11.1°


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
        arm, seat, over, ten = _angles(R)
        for site in _SITES:
            # hanging arc tenons, SEATED: anchored at the arm's ENTRY
            # face, spanning MOUNT_TEN_ARC inboard, grown to full cavity
            # depth (height=) — the joint fills its mortise exactly, and
            # the mortise is only as big as the joint
            ring = ring.union(
                _hang(MOUNT_J.tenon_arc(R, ten, root=2.0,
                                        height=MOUNT_TEN_H))
                .rotate((0, 0, 0), (0, 0, 1), site + arm - ten),
                clean=False)
        m = ring if m is None else m.union(ring, clean=False)
    # mid-quadrant SPOKES tying the rings (flush slabs over open air)
    for az in MOUNT_SPOKE_AZ:
        m = m.union(
            cq.Workplane("XY").workplane(offset=MOUNT_MATE_Z)
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
            cq.Workplane("XY").workplane(offset=MOUNT_MATE_Z)
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
    """frame_top's cutters — per arm, per ring: the retained arc CAVITY
    (open past the arm's CCW face — the entry; its CW end, TOP_STOP_WALL
    inside the arm's CW face, is the stop — the frame arc joints'
    arrangement verbatim, same seating rotation) plus the spine's
    V-groove over the stop-wall zone so the rings pass at every install
    angle."""
    cuts = []
    for R in _RINGS:
        arm, seat, over, ten = _angles(R)
        cav_sweep = ten + seat + over
        for site in _SITES:
            cuts.append(
                _hang(MOUNT_J.mortise_arc(R, cav_sweep,
                                          drop=MOUNT_PLATE_T + 0.5))
                .rotate((0, 0, 0), (0, 0, 1), site + arm - ten - seat))
            cuts.append(_v_groove_arc(R, site - arm - over,
                                      site + arm - ten - seat + 0.1))
    return cuts


mount_ring = _build_mount()
