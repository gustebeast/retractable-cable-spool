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
the spine shrank to MOUNT_SPINE_T to buy the octagon its room; the GF
z-relief mechanics are the library's — see the MOUNT_J comment).
SITED via cadkit's ring↔arm CROSSING, the SAME arrangement as the
frame_top↔frame_bottom joints (user: shared code, different profile)
evaluated per ring radius (angles scale up at the inner ring). Spines
are one stem wide; BETWEEN the cavity spans they ride annular
V-bottom grooves — inside a span the cavity slot IS the groove (see
mount_channel_cuts).

PRINTS +z→−z (top face on the bed): rings, spokes and pads on the bed;
the eight arc tenons rise library-native. Screw countersinks open at
the world underside — v2's validated screw geometry.
"""

import math

import cadquery as cq

from cadkit.joinery import PrintSpec, joint

from .params import (
    NOZZLE_D, FRAME_Z1, FRAME_RIB, FRAME_R_OUT,
    MOUNT_TEN_W, MOUNT_TEN_ARC, MOUNT_SPINE_T, MOUNT_FLOOR_MIN,
    MOUNT_PLATE_T, MOUNT_MATE_Z,
    MOUNT_RING_R, MOUNT_RING2_R, MOUNT_SPINE_W,
    MOUNT_SPOKE_W, MOUNT_SPOKE_AZ, MOUNT_PAD_W, MOUNT_PAD_AZ,
    MOUNT_PAD_AZ_OFF,
    MOUNT_GRV_W, MOUNT_GRV_VERT, MOUNT_GRV_FLAT, MOUNT_GRV_DEPTH,
    TOP_JOINT_SEAT_CLR, TOP_ENTRY_OVER,
    WOOD_SCREW_SHAFT_D, WOOD_SCREW_HEAD_D, WOOD_SCREW_HEAD_H,
)

_SPEC = PrintSpec(nozzle=NOZZLE_D, material="PETG-GF", facing="up")
# cadkit's standard for two same-direction hosts: the OCTAGON — its
# cavity CLOSES on the one-bead roof inside the arm (user's call: the
# old THROUGH mushroom left the arm's underside open and weak). The GF
# specs give it the 0.30 z-relief natively (joint height includes it).
# stem= (user #910): the tenon stem WIDENED to the ring bar's 2.4 so
# the hanging octagon matches the spine it grows from (width/2 gave a
# 2.0 stem with a 0.2 step per side). The library's host-matching
# override keeps the silhouette: the 45° shoulder gives 0.2 per side
# (0.8 — the one-nozzle floor) and the waist vertical takes it (1.8);
# height and swallow unchanged, so the arm-floor assert stands as-is.
MOUNT_J = joint(MOUNT_TEN_W, None, tenon=_SPEC, mortise=_SPEC,
                install="-x", stem=MOUNT_SPINE_W)
assert (MOUNT_MATE_Z - MOUNT_J.cavity_depth
        >= MOUNT_FLOOR_MIN - 1e-9), \
    "octagon cavity leaves under 1.6 of arm floor — shrink MOUNT_TEN_W"
# the load-bearing identity behind the groove-arc scheme (#911/#913):
# the cavities' neck slot and the spine groove are ONE width — a hair
# of divergence here recreates the float-identical-walls unify
# pathology the arcs exist to avoid, so check it, don't just state it
assert abs(MOUNT_GRV_W - (MOUNT_J.stem + 2.0 * MOUNT_J.clearance)) < 1e-9, \
    "spine groove no longer matches the cavities' neck slot"

_SITES = (0.0, 90.0, 180.0, 270.0)
_RINGS = (MOUNT_RING_R, MOUNT_RING2_R)

# ONE crossing per ring — cadkit's ring↔arm arrangement, shared verbatim
# with the frame arc joints (same seat/overshoot constants, same seating
# chirality): tenon anchored at the arm's ENTRY face spanning
# MOUNT_TEN_ARC (half the crossing — the stop-side half stays solid arm).
# hand="ccw" — the CHIRALITY AUDIT (user's worry: joints working
# undone): pulling the cable out torques the whole assembly CW about
# the axis (the cable's line of action is tangent on the drum's
# CW-driving side), so the hanging assembly must seat rotating CW
# relative to the fixed mount ≡ the mount TENONS seat CCW relative to
# frame_top. Pull-out then PRESSES the stops; uninstall = a deliberate
# CCW twist of the assembly, which no sustained load produces.
_CROSS = {R: MOUNT_J.crossing(R, FRAME_RIB, MOUNT_TEN_ARC,
                              seat=TOP_JOINT_SEAT_CLR, over=TOP_ENTRY_OVER,
                              hand="ccw")
          for R in _RINGS}

# the shared install rotation: the DEEPER offset of the two rings' entries
MOUNT_MATE_DEG = max(c.free for c in _CROSS.values()) + 1.0     # ≈ 11.1°


def _hang(w):
    """Library-native arc solid (bulb up, mating plane z=0) → hanging
    from the flush spine: z-mirror (plan angles preserved), mating plane
    to MOUNT_MATE_Z."""
    return w.mirror("XY").translate((0.0, 0.0, MOUNT_MATE_Z))


# the pads' plan positions — shared by the pad prisms AND their screw
# cuts (one table, so the screws can never silently miss the pads)
_PAD_XY = tuple(
    (MOUNT_RING_R * math.cos(math.radians(az)),
     MOUNT_RING_R * math.sin(math.radians(az)))
    for az in MOUNT_PAD_AZ)


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
    solid; the screw-hole probes at the gate keep it honest.
    Per-site solids are ONE prototype rotated (#932 efficiency review —
    the revolves are the cost, the rotates are free)."""
    hw = MOUNT_SPINE_W / 2.0
    m = None
    for R in _RINGS:
        ring = _annulus(R - hw, R + hw, MOUNT_MATE_Z, FRAME_Z1)
        # hanging octagon arc tenons, SEATED (the crossing pre-places
        # them: anchored at the arm's ENTRY face, spanning half the
        # crossing inboard — the stop-side half stays solid arm)
        ten = _hang(_CROSS[R].tenon(root=2.0))
        for site in _SITES:
            ring = ring.union(
                ten.rotate((0, 0, 0), (0, 0, 1), site), clean=False)
        m = ring if m is None else m.union(ring, clean=False)
    # mid-quadrant SPOKES tying the rings — the same 2.4 × 2.4 square as
    # the rings, riding the rings' own z-band (user: consistent section)
    for az in MOUNT_SPOKE_AZ:
        m = m.union(
            cq.Workplane("XY").workplane(offset=MOUNT_MATE_Z)
            .center((MOUNT_RING_R + MOUNT_RING2_R) / 2.0, 0.0)
            .rect(MOUNT_RING_R - MOUNT_RING2_R + MOUNT_SPINE_W,
                  MOUNT_SPOKE_W)
            .extrude(MOUNT_SPINE_T)
            .rotate((0, 0, 0), (0, 0, 1), az), clean=False)
    # AXIS-ALIGNED screw pads hugging the arms (use the pair on your
    # beam's axis); screws at the outer ring line. Pads and their screw
    # cuts share ONE placement table so they cannot drift apart.
    for x, y in _PAD_XY:
        m = m.union(
            cq.Workplane("XY").workplane(offset=FRAME_Z1 - MOUNT_PLATE_T)
            .center(x, y).rect(MOUNT_PAD_W, MOUNT_PAD_W)
            .extrude(MOUNT_PLATE_T), clean=False)
    # GUIDE TAILS (user, corrected #880: AXIS-ALIGNED, hugging the arm's
    # flank — not radial at the pad's offset azimuth): a 2.4 × 2.4 bar
    # running parallel to the wood beam from each pad's outboard end to
    # the top frame's outer radius, flush along the arm-flank plane the
    # pad seats against. Screw the bare mount to the beam and the tail
    # tips mark exactly how far the assembled frame will reach.
    pad_cx = MOUNT_RING_R * math.cos(math.radians(MOUNT_PAD_AZ_OFF))
    x0 = pad_cx + MOUNT_PAD_W / 2.0 - 0.5          # buried into the pad
    tail = (cq.Workplane("XY").workplane(offset=MOUNT_MATE_Z)
            .center((x0 + FRAME_R_OUT) / 2.0,
                    -(FRAME_RIB / 2.0 + MOUNT_SPINE_W / 2.0))
            .rect(FRAME_R_OUT - x0, MOUNT_SPINE_W)
            .extrude(MOUNT_SPINE_T))
    for axis_az in (0.0, 90.0, 180.0, 270.0):
        m = m.union(tail.rotate((0, 0, 0), (0, 0, 1), axis_az),
                    clean=False)
    for x, y in _PAD_XY:
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
    arc CAVITY (open past the arm's entry face; its far end wall,
    seat-shy of the seated tenon, is the stop) plus V-groove ARCS
    BETWEEN the cavity spans. The grooves used to run full circles;
    with the 2.4 stem (user #910) the cavities' neck slots are the
    SAME 2.7 as the groove — one straight wall, user #911 — and two
    cutters carving float-identical walls is OCC's pathological unify
    case (15+ min frame builds, py-spy-caught). So inside a cavity's
    span the SLOT is the groove (same width, deeper) and the groove
    arcs live only between spans: each runs from a cavity's STOP-WALL
    azimuth (an exact flat-on-flat abutment — benign; a 0.3° lap into
    the slot instead rode the walls coincident and left frame_top's
    outer shell OPEN) to the next cavity's air-side end (its `over`
    overshoot is already past the arm's entry face — open air)."""
    cuts = []
    for R in _RINGS:
        c = _CROSS[R]
        # one mortise + one groove-arc PROTOTYPE per ring, rotated to
        # the four sites (#932: the revolves are the cost)
        mor = _hang(c.mortise(drop=MOUNT_SPINE_T + 0.5))
        a_lo, a_hi = c.cavity_span     # what mortise() cuts, hand applied
        grv = _v_groove_arc(R, 0.0, (90.0 + a_lo) - a_hi)
        for site in _SITES:            # fixed 90° pitch
            cuts.append(mor.rotate((0, 0, 0), (0, 0, 1), site))
            cuts.append(grv.rotate((0, 0, 0), (0, 0, 1), site + a_hi))
    return cuts


mount_ring = _build_mount()
