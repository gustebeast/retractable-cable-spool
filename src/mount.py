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

STIFFNESS (user #973 — the bare mount bent while its screws were
driven): every span that is not sitting in an arm's channel hangs a
WEB down to the screw pads' underside plane, so the ring arcs across
the quadrants, the mid-quadrant spokes and the pads are one deep
girder. The spans stop clear of the arms by the install sweep (see
_web_span); the quadrants are open air under the mount, which is what
makes the depth free.

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

from .helpers import chevron_plan

from .params import (
    NOZZLE_D, FRAME_Z1, FRAME_RIB, FRAME_R_OUT,
    MOUNT_TEN_W, MOUNT_SPINE_T, MOUNT_FLOOR_MIN,
    MOUNT_PLATE_T, MOUNT_MATE_Z,
    MOUNT_RING_R, MOUNT_RING2_R, MOUNT_RING3_R, MOUNT_SPINE_W,
    MOUNT_WEB_Z0, MOUNT_WEB_CLR_DEG,
    MOUNT_SPOKE_W, N_MOUNT_SPOKE, MOUNT_PAD_W, MOUNT_PAD_LY,
    ANCH_IR, ANCH_A_POS,
    MOUNT_PAD_SCREW_Y, MOUNT_SCREW_TILT_DEG, MOUNT_SCREW_BORE_D,
    MOUNT_SCREW_SEAT,
    MOUNT_PAD_AZ_OFF, MOUNT_TAIL_TIP_X,
    MOUNT_GRV_W, MOUNT_GRV_VERT, MOUNT_GRV_FLAT, MOUNT_GRV_DEPTH,
    TOP_ENTRY_OVER,
    WOOD_SCREW_SHAFT_D, WOOD_SCREW_HEAD_D, WOOD_SCREW_HEAD_H,
    MOUNT_PAD_H, MOUNT_PAD_INSET_MY, LOCK_HEAD_ZF, LOCK_ROTS,
    LOCK_X0, LOCK_X1, LOCK_MOUNT_Y1, LOCK_HEAD_ZB, LOCK_SLOT_XA,
    LOCK_WALL_Y1, LOCK_PAD_Z0, LOCK_SWEEP_CLR,
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
# rings that LOCK to the frame (arc tenons in all four arms) vs rings
# that only RIDE a plain V-groove through the arms (user #985: the inner
# ring's joinery carried next to nothing, and dropping it frees room for
# a middle ring). All of them are mount structure and carry deep webs.
_RINGS_JOINED = (MOUNT_RING_R,)
_RINGS_FREE = (MOUNT_RING2_R, MOUNT_RING3_R)
_RINGS = _RINGS_JOINED + _RINGS_FREE

# ONE crossing per ring — cadkit's ring↔arm arrangement, shared verbatim
# with the frame arc joints once — until #987 (user): the tenon anchored
# at the arm's entry face and spanned HALF the crossing, the other half
# left solid as the angular STOP. The screw pads land on the arm flanks
# and ARE the stop, so the joint now runs THROUGH the whole arm — twice
# the engagement. cadkit's ArcCrossing refuses a crossing with no stop
# wall (its 50% rule), so _ThroughCrossing composes the SAME profile
# from the library's public tenon_arc / mortise_arc instead.
# The seating chirality stands — the #938 chirality FLIP (user, field failure): the #915
# audit seated against PULL-OUT torque (assembly CW, viewed +z), but a
# fast retract SLAMS the cable into the exit horn and that impulse
# torques the assembly the OTHER way — exactly the uninstall twist —
# and spun it off the mount. The slam is impulsive and dominates the
# slow, hand-damped pull, so the joint now seats against the SLAM:
# the hanging assembly seats rotating CCW relative to the fixed mount
# ≡ the mount TENONS seat CW relative to frame_top. Retract-slam
# torque PRESSES the stops; uninstall = a deliberate CW twist.
class _ThroughCrossing:
    """A ring↔arm crossing whose joint runs THROUGH the arm (user #987):
    the seated tenon spans the arm face to face, the cavity is open at
    BOTH faces (plus `over` of entry overshoot on each), and nothing in
    the joint is a stop — the screw pads landing on the arm flanks are.
    Arm centred on plan angle 0, like cadkit's ArcCrossing; the profile
    is the library's (tenon_arc / mortise_arc), only the placement is
    ours. The geometry is symmetric about the arm, so the seating hand
    only decides which side the tenon enters from (the pads' side)."""

    def __init__(self, jnt, radius, arm_w, over):
        self.joint, self.radius = jnt, radius
        self.arm = math.degrees(math.asin((arm_w / 2.0) / radius))
        self.over = math.degrees(over / radius)
        self.ten = 2.0 * self.arm                 # face to face

    @property
    def free(self):
        """The whole tenon plus the overshoot: the install offset at which
        the tenon is fully out of the arm and clear to z-mate."""
        return self.ten + self.over

    def tenon(self, root=1.0):
        return (self.joint.tenon_arc(self.radius, self.ten, root=root)
                .rotate((0, 0, 0), (0, 0, 1), -self.arm))

    def mortise(self, drop=2.0):
        return (self.joint.mortise_arc(self.radius,
                                       self.ten + 2.0 * self.over, drop=drop)
                .rotate((0, 0, 0), (0, 0, 1), -self.arm - self.over))


_CROSS = {R: _ThroughCrossing(MOUNT_J, R, FRAME_RIB, TOP_ENTRY_OVER)
          for R in _RINGS_JOINED}

# the install rotation: the deepest entry offset of the JOINED rings'
# crossings, plus a degree. It was the inner ring's — an arm subtends
# twice the angle at half the radius — at ≈10.9°; with only the outer
# ring joined (#985) it roughly halves.
MOUNT_MATE_DEG = max(c.free for c in _CROSS.values()) + 1.0


def _web_span(R):
    """The azimuth band, within one quadrant, where a DEEP web may hang
    off a ring at radius `R` (params STIFFENING WEBS). Two different
    keep-outs, because the install sweep is one-way: the mount enters
    MOUNT_MATE_DEG CCW of its seat and rotates CW onto the stops, so
    material just CCW of an arm only ever swings AWAY from it (the
    screw pads live there and already land ON the flank), while
    material on an arm's CW side passes over that arm on the way in and
    must clear the whole sweep as well."""
    half = math.degrees(math.asin((FRAME_RIB / 2.0) / R))   # arm half-angle
    a0 = half + MOUNT_WEB_CLR_DEG
    a1 = 90.0 - (half + MOUNT_MATE_DEG + MOUNT_WEB_CLR_DEG)
    assert a1 - a0 > 10.0, \
        "no room left between the arms for a stiffening web at this radius"
    return a0, a1


# the +x arm's spring-anchor wall (ANCH_IR..ANCH_OR) rises through the
# frame at azimuths out to ANCH_A_POS, and a radial spoke crosses
# exactly those radii — so in THAT quadrant the spoke band starts past
# the wall (plus the bar's own half-angle there, plus the margin). The
# wall's −y side needs no rule: the previous quadrant's band ends long
# before it. (The RINGS are radially clear of the wall at every
# azimuth — this bounds the spokes only.)
_ANCH_BLOCK = (ANCH_A_POS
               + math.degrees(math.asin((MOUNT_SPOKE_W / 2.0) / ANCH_IR))
               + MOUNT_WEB_CLR_DEG)


def _spoke_layout():
    """Per quadrant: (site, spoke azimuths). N_MOUNT_SPOKE bars, the
    outermost pair pushed HARD AGAINST the arms at each end of the
    usable band (user #975 — over the arm is impossible, see the params
    note) and the rest spread evenly between. That outermost pair also
    BOUNDS this quadrant's deep ring webs (user #976), so a ring's
    tall-to-short step always lands on a spoke.

    The band is _web_span at the bar's INNERMOST radius — an arm
    subtends its widest angle there, so that end governs the whole bar
    — inset by the bar's half-angle, so its EDGE keeps the clearance,
    not its centreline."""
    r0 = MOUNT_RING2_R - MOUNT_SPINE_W / 2.0
    a0, a1 = _web_span(r0)
    half_bar = math.degrees(math.asin((MOUNT_SPOKE_W / 2.0) / r0))
    layout = []
    for site in _SITES:
        lo = (max(a0, _ANCH_BLOCK) if site == 0.0 else a0) + half_bar
        hi = a1 - half_bar
        assert hi > lo, "no room for a ring-tie spoke in this quadrant"
        n = N_MOUNT_SPOKE
        layout.append((site, tuple(
            site + lo + (hi - lo) * (0.5 if n == 1 else i / (n - 1.0))
            for i in range(n))))
    return tuple(layout)


_SPOKE_LAYOUT = _spoke_layout()
MOUNT_SPOKE_AZ = tuple(a for _site, az in _SPOKE_LAYOUT for a in az)


def _hang(w):
    """Library-native arc solid (bulb up, mating plane z=0) → hanging
    from the flush spine: z-mirror (plan angles preserved), mating plane
    to MOUNT_MATE_Z."""
    return w.mirror("XY").translate((0.0, 0.0, MOUNT_MATE_Z))


# PAD SITES — one PROTOTYPE (the +x arm's pad: hugging the arm's +y
# flank, growing +y, screw tilted toward −y = toward the arm) placed
# at the four arms by rotation, so the pad prism, its screw cut and
# its tilt can never drift apart. The −y arm's pad slides
# MOUNT_PAD_INSET_MY inboard along its beam (params #947 comment: its
# deep countersink must clear the TPU lock) — applied to the prototype
# along −x BEFORE the rotation.
_PAD_CX = MOUNT_RING_R * math.cos(math.radians(MOUNT_PAD_AZ_OFF))
_PAD_Y0 = FRAME_RIB / 2.0                 # the flank the pad seats on
# the radial band EVERY screw pad covers (the −y pad is inset, so it is
# the tighter one): a ring inside this band runs through pad material,
# so its deep span may end there — the step is buried in the pad
_PAD_R0 = _PAD_CX - MOUNT_PAD_W / 2.0
_PAD_R1 = _PAD_CX + MOUNT_PAD_W / 2.0 - MOUNT_PAD_INSET_MY
# (the #982 HAIRLINE FILL is retired — #1063: the head no longer stops
# short of the screw pad at all. It runs a LOCK_WELD INTO the pad
# (LOCK_MOUNT_Y1), and the slot stops at the pad's face, so the pad IS
# the slot's inboard wall and the TPU bears straight on the screw boss.)
assert -(LOCK_SHIFT_Y + LOCK_MOUNT_Y1) <= _PAD_R1 + 1e-9,     "the jib head no longer reaches into its screw pad — the weld is open"


# the arms that carry a TPU lock (the −y one, turned by each LOCK_ROTS):
# their screw pads take the inset that clears the lock stack
_LOCK_ARMS = tuple((270.0 + r) % 360.0 for r in LOCK_ROTS)


def _site_place(w, base_az):
    inset = MOUNT_PAD_INSET_MY if base_az in _LOCK_ARMS else 0.0
    return (w.translate((-inset, 0.0, 0.0))
            .rotate((0, 0, 0), (0, 0, 1), base_az))


def _pad_proto():
    return (cq.Workplane("XY").workplane(offset=FRAME_Z1 - MOUNT_PAD_H)
            .center(_PAD_CX, _PAD_Y0 + MOUNT_PAD_LY / 2.0)
            .rect(MOUNT_PAD_W, MOUNT_PAD_LY).extrude(MOUNT_PAD_H))


# the head-pocket cone's radial taper per mm of height
_CSK_SLOPE = (WOOD_SCREW_HEAD_D - WOOD_SCREW_SHAFT_D) / 2.0 / WOOD_SCREW_HEAD_H


def _screw_cut_proto():
    """The TILTED wood-screw cutter (user #959, params ANGLED SCREWS):
    axis entering the pad's underside MOUNT_PAD_SCREW_Y off the flank
    and leaning MOUNT_SCREW_TILT_DEG toward the arm as it climbs. A
    head-pass bore from the underside to the countersink base (recessed
    MOUNT_SCREW_SEAT so the tilted head disc sits inside the pad, its
    outboard rim flush with the underside), v2's validated cone from
    there, then the Ø4 shaft out through the wood face."""
    t = math.radians(MOUNT_SCREW_TILT_DEG)
    a = cq.Vector(0.0, -math.sin(t), math.cos(t))          # up and inward
    e = cq.Vector(_PAD_CX, _PAD_Y0 + MOUNT_PAD_SCREW_Y,
                  FRAME_Z1 - MOUNT_PAD_H)                  # underside entry
    over = 0.5
    # the cylinders start a full MOUNT_SCREW_SEAT (= r·tan tilt, the
    # axial run their tilted start cap needs to clear the underside
    # plane) + 1 BELOW the entry — a 1.0 overshoot alone left the
    # start cap slicing through the mouth on the outboard side (#959
    # probe: the head could not pass)
    back = MOUNT_SCREW_SEAT + 1.0
    shaft = cq.Solid.makeCylinder(
        WOOD_SCREW_SHAFT_D / 2.0, MOUNT_PAD_H / math.cos(t) + back + 1.0,
        e - a * back, a)
    bore = cq.Solid.makeCylinder(
        MOUNT_SCREW_BORE_D / 2.0, MOUNT_SCREW_SEAT + back, e - a * back, a)
    cone = cq.Solid.makeCone(
        WOOD_SCREW_HEAD_D / 2.0 + over * _CSK_SLOPE,
        WOOD_SCREW_SHAFT_D / 2.0,
        WOOD_SCREW_HEAD_H + over,
        e + a * (MOUNT_SCREW_SEAT - over), a)
    return (cq.Workplane(obj=shaft).union(cq.Workplane(obj=bore))
            .union(cq.Workplane(obj=cone)))


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
        if R in _CROSS:
            # JOINED rings only (#985): hanging octagon arc tenons,
            # SEATED (the crossing pre-places them: anchored at the
            # arm's ENTRY face, spanning half the crossing inboard —
            # the stop-side half stays solid arm)
            ten = _hang(_CROSS[R].tenon(root=2.0))
            for site in _SITES:
                ring = ring.union(
                    ten.rotate((0, 0, 0), (0, 0, 1), site), clean=False)
        m = ring if m is None else m.union(ring, clean=False)
    # SPOKES tying the rings — 2.4 wide like the rings, FULL DEPTH (user
    # #973), N_MOUNT_SPOKE per quadrant at the derived azimuths (#975):
    # each sits inside the band that clears the arms through the whole
    # install sweep, so no spoke ever passes over frame material
    for az in MOUNT_SPOKE_AZ:
        m = m.union(
            cq.Workplane("XY").workplane(offset=MOUNT_WEB_Z0)
            .center((MOUNT_RING_R + MOUNT_RING2_R) / 2.0, 0.0)
            .rect(MOUNT_RING_R - MOUNT_RING2_R + MOUNT_SPINE_W,
                  MOUNT_SPOKE_W)
            .extrude(FRAME_Z1 - MOUNT_WEB_Z0)
            .rotate((0, 0, 0), (0, 0, 1), az), clean=False)
    # DEEP RING WEBS (params STIFFENING WEBS): each ring drops to the
    # pads' plane across the open quadrant — one revolved prototype per
    # ring, rotated to the four quadrants. The band starts inside the
    # screw pads' own span, so pad and ring fuse into one deep girder.
    for R in _RINGS:
        for site, az in _SPOKE_LAYOUT:
            # the CW end stops at the quadrant's LAST spoke, so that
            # tall-to-short step lands ON a spoke (#976). The CCW end runs
            # deep right up to the arm (user #987): material on an arm's
            # CCW side only swings AWAY from it during the install, so it
            # never needs the shallow section — only the CW side, which
            # passes over the next arm on the way in, does
            a0 = _web_span(R)[0]
            a1 = az[-1] - site
            m = m.union(
                cq.Workplane("XZ")
                .polyline([(R - hw, MOUNT_WEB_Z0), (R + hw, MOUNT_WEB_Z0),
                           (R + hw, FRAME_Z1), (R - hw, FRAME_Z1)])
                .close().revolve(a1 - a0, (0, 0), (0, 1))
                .rotate((0, 0, 0), (0, 0, 1), site + a0), clean=False)
    # AXIS-ALIGNED screw pads hugging the arms (use the pair on your
    # beam's axis); one prototype rotated to the four arms, so the pads
    # and their (tilted) screw cuts cannot drift apart. DEEP pads (user
    # #947: the #936 stack spacer FUSED in — pads run the full
    # MOUNT_PAD_H, everything else keeps its section)
    pad = _pad_proto()
    for base_az in (0.0, 90.0, 180.0, 270.0):
        m = m.union(_site_place(pad, base_az), clean=False)
    # (the −y pad's wedge relief over the TPU lock corbel is RETIRED —
    # #971: that pad is inset clear of the lock stack instead, params)
    # GUIDE TAILS (user, corrected #880: AXIS-ALIGNED, hugging the arm's
    # flank — not radial at the pad's offset azimuth): a 2.4 × 2.4 bar
    # running parallel to the wood beam from each pad's outboard end to
    # MOUNT_TAIL_TIP_X (params: the frame's outer radius on the desk
    # build; the rotate-on SET-BACK past it in the couch version), flush
    # along the arm-flank plane the pad seats against. Screw the bare
    # mount to the beam and the tail tips mark where the assembled frame
    # reaches — or, on the couch, where the front beam must sit.
    # buried into EVERY pad — the −y one is inset MOUNT_PAD_INSET_MY, and
    # its tail is what welds the lock's jib head back to the ring
    x0 = _PAD_CX + MOUNT_PAD_W / 2.0 - MOUNT_PAD_INSET_MY - 0.5
    # tails flush along the CCW arm flank now — the flank the pads seat
    # against since the #938 hand flip
    tail = (cq.Workplane("XY").workplane(offset=MOUNT_MATE_Z)
            .center((x0 + MOUNT_TAIL_TIP_X) / 2.0,
                    FRAME_RIB / 2.0 + MOUNT_SPINE_W / 2.0)
            .rect(MOUNT_TAIL_TIP_X - x0, MOUNT_SPINE_W)
            .extrude(MOUNT_SPINE_T))
    for axis_az in (0.0, 90.0, 180.0, 270.0):
        m = m.union(tail.rotate((0, 0, 0), (0, 0, 1), axis_az),
                    clean=False)
    # TPU-LOCK jib HEAD (user #939/#941/#942, params LOCK block): the
    # −y beam's guide tail widens into a slotted head growing +x off
    # the beam's flank — the TOP storey of the three-part lock stack
    # (corbels on both frames below, frame.py). As-drawn geometry
    # TRANSLATED into place (user: move, don't rotate). TRAPEZOID
    # section (user #942 — cover more of the pin): flat top at the
    # frame face, underside riding the top corbel's 45 roof at
    # LOCK_SWEEP_CLR, meeting the tail flush at MOUNT_MATE_Z on the
    # flank; in the flipped print that underside is a print-TOP face.
    # NO −x weld stub: past the flank plane is frame-arm material —
    # the head welds through its real overlap with the tail rib (and
    # this arm's screw pad), all mount material.
    # (#983) the underside meets the flank at LOCK_HEAD_ZF — the mating
    # plane on the desk, the dropped interface in the couch build, where
    # the head grows by the whole frame_top pad. One head per LOCK_ROTS.
    head = (cq.Workplane("XZ")
            .polyline([(LOCK_X0, FRAME_Z1), (LOCK_X1, FRAME_Z1),
                       (LOCK_X1, LOCK_HEAD_ZB), (LOCK_X0, LOCK_HEAD_ZF)])
            .close().extrude(LOCK_MOUNT_Y1 - FRAME_RIB / 2.0)
            .translate((LOCK_SHIFT_X,
                        LOCK_MOUNT_Y1 + LOCK_SHIFT_Y, 0.0)))
    # the head only needs to reach PAST the frame's own block where the
    # screw pad is there to weld to — i.e. from the pad's underside up.
    # Below that it was hanging LOCK_MOUNT_Y1 − LOCK_WALL_Y1 proud of
    # both frame corbels for nothing (user #1068). Trimmed flush with
    # them; the trim's own face lands on the pad's underside plane, and
    # in this part's FLIPPED print it is a print-TOP face, not an
    # overhang.
    trim = (cq.Workplane("XY").workplane(offset=LOCK_HEAD_ZB - 2.0)
            .center((LOCK_X0 + LOCK_X1) / 2.0,
                    (LOCK_WALL_Y1 + LOCK_MOUNT_Y1 + 1.0) / 2.0)
            .rect(LOCK_X1 - LOCK_X0 + 4.0,
                  LOCK_MOUNT_Y1 + 1.0 - LOCK_WALL_Y1)
            .extrude(LOCK_PAD_Z0 - (LOCK_HEAD_ZB - 2.0))
            .translate((LOCK_SHIFT_X, LOCK_SHIFT_Y, 0.0)))
    head = head.cut(trim)
    for r in LOCK_ROTS:
        m = m.union(head.rotate((0, 0, 0), (0, 0, 1), r), clean=False)
    # NOTHING OF THE MOUNT BELOW THE LOCK'S 45 (user #1069): the head's
    # own underside rides the frame corbel's roof at LOCK_SWEEP_CLR, but
    # it is not the only mount part that reaches out here — the ring-tie
    # WEB bottoms at MOUNT_WEB_Z0, which is FRAME_Z1 − MOUNT_PAD_H, the
    # same depth as the screw pad, and its outer edge (MOUNT_RING_R +
    # MOUNT_SPINE_W/2) dropped a sliver past that 45 into the frame's
    # block. Tiny — 0.42 mm² of face — but it is inside the joint, so it
    # fouls the stack. So the envelope is CUT from the finished mount
    # rather than merely respected by the head: past the beam's flank,
    # below the head's underside line, the mount is empty by
    # construction, whatever grows there later.
    env_z0 = LOCK_HEAD_ZB - 12.0
    env = (cq.Workplane("XZ")
           .polyline([(LOCK_X0, LOCK_HEAD_ZF), (LOCK_X1, LOCK_HEAD_ZB),
                      (LOCK_X1, env_z0), (LOCK_X0, env_z0)])
           .close().extrude(LOCK_WALL_Y1 + LOCK_SWEEP_CLR - FRAME_RIB / 2.0)
           .translate((LOCK_SHIFT_X,
                       LOCK_WALL_Y1 + LOCK_SWEEP_CLR + LOCK_SHIFT_Y, 0.0)))
    for r in LOCK_ROTS:
        m = m.cut(env.rotate((0, 0, 0), (0, 0, 1), r), clean=False)
    # the slot is a CLOSED pocket in the head — the −x wall between the
    # flank and the slot is the lock itself (user #948: the mount's
    # uninstall motion here is +x, and that wall is what drives the
    # pin against the frame corbels' +x walls; the #942 open-slot
    # variant slid straight off the pin)
    slot = (cq.Workplane("XY").workplane(offset=LOCK_HEAD_ZB - 1.0)
            .polyline(chevron_plan(LOCK_SLOT_X0, LOCK_SLOT_X1, LOCK_SLOT_XA,
                                   LOCK_SLOT_Y0, LOCK_SLOT_Y1))
            .close().extrude((FRAME_Z1 - LOCK_HEAD_ZB) + 2.0)
            .translate((LOCK_SHIFT_X, LOCK_SHIFT_Y, 0.0)))
    for r in LOCK_ROTS:
        m = m.cut(slot.rotate((0, 0, 0), (0, 0, 1), r), clean=False)
    screw = _screw_cut_proto()
    for base_az in (0.0, 90.0, 180.0, 270.0):
        m = m.cut(_site_place(screw, base_az), clean=False)
    return m


# (the #936 screw_spacer part is RETIRED — user #947: fused into the
# deep pads above; same 5.6 of drilled depth stolen, zero loose parts)


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
    for R in _RINGS_JOINED:
        # (#987) a THROUGH cavity per arm, open at both faces: the ring's
        # spine rides its neck slot (the same 2.7 as a groove, asserted)
        # across the whole arm, and between the arms there is only air —
        # so the joined ring needs no groove arcs at all
        mor = _hang(_CROSS[R].mortise(drop=MOUNT_SPINE_T + 0.5))
        for site in _SITES:            # fixed 90° pitch
            cuts.append(mor.rotate((0, 0, 0), (0, 0, 1), site))
    # FREE rings (#985): a plain full-circle V-groove each — no cavity,
    # so no slot wall for it to run coincident with; it only bites where
    # frame_top has material, i.e. across the arms
    for R in _RINGS_FREE:
        cuts.append(_v_groove_arc(R, 0.0, 360.0))
    return cuts


mount_ring = _build_mount()
