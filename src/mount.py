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
    SPACER_H, SPACER_CONE_ENG, FIT_CLR,
    LOCK_X0, LOCK_X1, LOCK_HEAD_Y1,
    LOCK_SLOT_X0, LOCK_SLOT_X1, LOCK_SLOT_Y0, LOCK_SLOT_Y1,
    LOCK_SHIFT_X, LOCK_SHIFT_Y,
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
# hand="cw" — the #938 chirality FLIP (user, field failure): the #915
# audit seated against PULL-OUT torque (assembly CW, viewed +z), but a
# fast retract SLAMS the cable into the exit horn and that impulse
# torques the assembly the OTHER way — exactly the uninstall twist —
# and spun it off the mount. The slam is impulsive and dominates the
# slow, hand-damped pull, so the joint now seats against the SLAM:
# the hanging assembly seats rotating CCW relative to the fixed mount
# ≡ the mount TENONS seat CW relative to frame_top. Retract-slam
# torque PRESSES the stops; uninstall = a deliberate CW twist.
_CROSS = {R: MOUNT_J.crossing(R, FRAME_RIB, MOUNT_TEN_ARC,
                              seat=TOP_JOINT_SEAT_CLR, over=TOP_ENTRY_OVER,
                              hand="cw")
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


# the head-pocket cone's radial taper per mm of height — shared by the
# pad cutter, the spacer's female cut AND its male centring cone
_CSK_SLOPE = (WOOD_SCREW_HEAD_D - WOOD_SCREW_SHAFT_D) / 2.0 / WOOD_SCREW_HEAD_H


def _screw_cut(x, y, z_face, thru):
    """One wood-screw cutter: Ø4 shaft rising `thru` from the face,
    countersink cone opening AT z_face (the head seats flush there,
    driven up into the wood). v2's validated geometry — ONE profile
    for the pad and the stack spacer (user #936), so the head seats
    identically wherever it lands."""
    over = 0.5
    shaft = cq.Solid.makeCylinder(
        WOOD_SCREW_SHAFT_D / 2.0, thru + 1.0,
        cq.Vector(x, y, z_face - 0.5), cq.Vector(0, 0, 1))
    cone = cq.Solid.makeCone(
        WOOD_SCREW_HEAD_D / 2.0 + over * _CSK_SLOPE,
        WOOD_SCREW_SHAFT_D / 2.0,
        WOOD_SCREW_HEAD_H + over,
        cq.Vector(x, y, z_face - over), cq.Vector(0, 0, 1))
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
    # tails flush along the CCW arm flank now — the flank the pads seat
    # against since the #938 hand flip
    tail = (cq.Workplane("XY").workplane(offset=MOUNT_MATE_Z)
            .center((x0 + FRAME_R_OUT) / 2.0,
                    FRAME_RIB / 2.0 + MOUNT_SPINE_W / 2.0)
            .rect(FRAME_R_OUT - x0, MOUNT_SPINE_W)
            .extrude(MOUNT_SPINE_T))
    for axis_az in (0.0, 90.0, 180.0, 270.0):
        m = m.union(tail.rotate((0, 0, 0), (0, 0, 1), axis_az),
                    clean=False)
    # TPU-LOCK jib HEAD (user #939/#941, params LOCK block): the −y
    # beam's guide tail widens into a slotted head growing +x off the
    # beam's flank — the TOP storey of the three-part lock stack
    # (corbels on both frames below, frame.py). As-drawn geometry
    # TRANSLATED into place (user: move, don't rotate). NO −x weld
    # stub: past the flank plane is frame-arm material — the head
    # welds through its real overlap with the tail rib (and this
    # arm's screw pad), all mount material.
    m = m.union(
        cq.Workplane("XY").workplane(offset=MOUNT_MATE_Z)
        .polyline([(LOCK_X0, FRAME_RIB / 2.0),
                   (LOCK_X1, FRAME_RIB / 2.0),
                   (LOCK_X1, LOCK_HEAD_Y1),
                   (LOCK_X0, LOCK_HEAD_Y1)])
        .close().extrude(FRAME_Z1 - MOUNT_MATE_Z)
        .translate((LOCK_SHIFT_X, LOCK_SHIFT_Y, 0.0)), clean=False)
    m = m.cut(
        cq.Workplane("XY").workplane(offset=MOUNT_MATE_Z - 1.0)
        .polyline([(LOCK_SLOT_X0, LOCK_SLOT_Y0), (LOCK_SLOT_X1, LOCK_SLOT_Y0),
                   (LOCK_SLOT_X1, LOCK_SLOT_Y1), (LOCK_SLOT_X0, LOCK_SLOT_Y1)])
        .close().extrude((FRAME_Z1 - MOUNT_MATE_Z) + 2.0)
        .translate((LOCK_SHIFT_X, LOCK_SHIFT_Y, 0.0)), clean=False)
    for x, y in _PAD_XY:
        m = m.cut(_screw_cut(x, y, FRAME_Z1 - MOUNT_PLATE_T,
                             MOUNT_PLATE_T), clean=False)
    return m


def _build_screw_spacer():
    """SCREW STACK SPACER (user #936): the screws out-reach the desk, so
    this puck rides between the head and the pad, stealing SPACER_H
    (5.6) of drilled depth. Pad-footprint block seated FLAT on the
    pad's underside (the clamp path — the cones only centre); on top a
    truncated MALE cone (the pocket's cone at FIT_CLR radial shrink,
    SPACER_CONE_ENG of engagement) noses into the pad's existing
    countersink; underneath, the IDENTICAL female countersink + shaft
    bore (_screw_cut — one validated profile). Modeled seated at pad 0;
    print one per screw. Prints AS MODELED — female opening on the bed,
    both cone surfaces at ~57° from the bed, self-supporting."""
    x, y = _PAD_XY[0]
    z_top = FRAME_Z1 - MOUNT_PLATE_T                 # pad underside = 6.4
    z_bot = z_top - SPACER_H                         # head face = 0.8
    body = (cq.Workplane("XY").workplane(offset=z_bot)
            .center(x, y).rect(MOUNT_PAD_W, MOUNT_PAD_W)
            .extrude(SPACER_H))
    male = cq.Workplane(obj=cq.Solid.makeCone(
        WOOD_SCREW_HEAD_D / 2.0 - FIT_CLR,
        WOOD_SCREW_HEAD_D / 2.0 - FIT_CLR - SPACER_CONE_ENG * _CSK_SLOPE,
        SPACER_CONE_ENG,
        cq.Vector(x, y, z_top), cq.Vector(0, 0, 1)))
    return (body.union(male)
            .cut(_screw_cut(x, y, z_bot, SPACER_H + SPACER_CONE_ENG)))


screw_spacer = _build_screw_spacer()


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
