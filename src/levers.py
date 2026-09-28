"""Ratchet + brake levers — v2's radial-engagement design, hardware-free,
ported to v3's FLIPPED band stack.

v2's architecture kept whole: XZ plate levers (LEVER_T=10, HANDLE_W=6) at
+X flanking the beam (ratchet +y, brake −y), Y-axis square-TPU torsion
pivots spanning the fork posts, pull = +X; the pawl edge is the exact
tooth-profile negative + catch bump; the brake pad is the CONTACT-frame
design (arm + pad built as one straight prism at the contact pose,
back-rotated to rest) with the L-flange + cadkit hook slide joint and the
knurl-negative face.

v3 differences (the bands swapped — brake band is the disk's BOTTOM band):
  * RATCHET pivot stays 10 above its (now upper) tooth band — unchanged
    behaviour: rest = pawl seated, pull swings it out;
  * BRAKE pivot goes 5.5 BELOW its (now lower) band: the pad must sit
    ABOVE its pivot for pull to press it INTO the band (an above-band
    pivot swings the pad away — caught in the port);
  * the pawl's one-bead running clearance moves to its BOTTOM (the brake
    band is below the teeth now) — A_PAWL_BAND_CLR sweeps mirrored;
  * the levers reach the bands through the FULL-HEIGHT bays; the ratchet's
    over-pull stop tab is STILL TO COME (the old wall sill left with the
    bays — RATCHET_STOP_DEG remains the asserted design limit).

The kinematic assertion suite (lever_kinematics.py) fires at import.
TPU parts (brake pad + 2 torsion pins) print in 95A — black in the viewer.
"""

import math

import cadquery as cq

from cadkit.joinery import joint

from .axle import ratchet_star_world, knurl_band_world
from .lever_kinematics import assert_kinematics, R_ROOT, PAWL_CLEAR_MM
from .params import (
    NOZZLE_D,
    RIM_OD, RATCHET_DEPTH, RATCHET_TEETH, RATCHET_PHASE_DEG,
    SEP_Z0, SEP_Z1, BRAKE_H, R_OUT_COIL, CABLE_D,
    LEVER_T, LEVER_HANDLE_W, LEVER_PIVOT_X, LEVER_TRAVEL_DEG,
    RATCHET_LEV_Y0, RATCHET_LEV_Y1, BRAKE_LEV_Y0, BRAKE_LEV_Y1,
    RATCHET_PIVOT_Z, BRAKE_PIVOT_Z, HANDLE_Z_BOT, LEVER_BEND_Z, HANDLE_X_BOT,
    LEVER_PIN_D, LEVER_PIN_LEVER_CLR, LEVER_BOSS_OD,
    PIN_KEY_W, PIN_KEY_H, PIN_KEY_L, PIN_KEY_WIDEN, PIN_KEY_SLOT_H,
    PIN_KEY_RATE, PIN_KEY_FLOAT, PIN_CLIP_SLOT_W, PIN_CLIP_RIB_W,
    PIN_SHANK_UNDER, PIN_CLIP_SLOT_FREE, PIN_CLIP_SQUEEZE_RUN,
    PIN_KEY_SEAT_UNDER, PIN_HUB_OVER, PIN_JOURNAL_UNDER,
    LEVER_T,
    BRAKE_RUBBER_T,
    PAD_JOINT_CLR, PAD_SPEC, BRAKE_PAD_TOP_MARGIN, BRAKE_PAD_BOT_MARGIN,
    PAD_FLANGE_H, PAD_FLANGE_T,
    LEVER_PIN_L, PIN_TIP_END_Y, PIN_TIP_CLEAR, PIN_TIP_NOSE,
    LEVER_Y_IN, BEAM_SIZE, POST_OUT_T,
    LEVER_COL_Y1, LEVER_PIN_BODY_L, BEAM_IR,
    TPU_E, CLIP_GAP, CLIP_T, CLIP_HUB_R, CLIP_BORE_CLR, CLIP_LEAF_T,
    CLIP_LEAF_X, CLIP_CONTACT_R, CLIP_TIP_PAST, CLIP_PRELOAD, CLIP_AX_CLR,
    POST_R, PIN_HEAD_R, PIN_HEAD_T, RETURN_TORQUE_MIN, RETURN_TORQUE_MAX,
    RATCHET_CLIP_ROT_DEG, BRAKE_CLIP_ROT_DEG, FLOOR_Z0,
    WALL_IR, WALL_OR, RATCHET_OPEN_CLR, LEVER_SIDE_CLR, STOP_LEVER_CLR,
    BEAM_IR, GUARD_LEVER_CLR, FLOOR_Z1,
)

R_RIM = RIM_OD / 2.0                              # 69.435 — band / tooth tips
_HW   = LEVER_HANDLE_W / 2.0

# Band z-geometry (world) — v3 stack: BRAKE band below, RATCHET band above.
BRAKE_BAND_Z0   = SEP_Z0                          # −44.0
BRAKE_BAND_Z1   = SEP_Z0 + BRAKE_H                # −39.2
RATCHET_BAND_Z0 = BRAKE_BAND_Z1                   # teeth sit DIRECTLY on the
RATCHET_BAND_Z1 = SEP_Z1                          # knurl ring (−34.4)

# Pawl block spans the tooth band; kinematics sample at its mid point.
# The one-bead running clearance is at the block's BOTTOM now — the brake
# band's full ring sits DIRECTLY UNDER the teeth (v2 had it above).
PAWL_BOT_CLR = 0.8
PAWL_Z_LO, PAWL_Z_HI = RATCHET_BAND_Z0 + PAWL_BOT_CLR, RATCHET_BAND_Z1
PAWL_Y_MID = (RATCHET_LEV_Y0 + RATCHET_LEV_Y1) / 2.0        # 12
PAWL_Z_MID = (PAWL_Z_LO + PAWL_Z_HI) / 2.0                  # −36.4


def _pivot_boss(pivot_x, pivot_z, y0, y1):
    """Ø LEVER_BOSS_OD boss along the pivot axis, spanning the plate — the
    6-wide handle is narrower than the axle pocket, the bulge carries it."""
    return (cq.Workplane("XZ").center(pivot_x, pivot_z)
            .circle(LEVER_BOSS_OD / 2)
            .extrude(-(y1 - y0)).translate((0, y0, 0)))


# The pin's DOME bottoms on the blind bore's floor, which leaves it
# sitting PIN_STOP_D further in than the drawn nominal (#1016). That is
# the assembled position now — a hard stop, not a feel — so every FIT on
# the body is cut PIN_FIT_SHIFT nearer the tip to compensate: at the
# stop each one lands exactly where it landed when the bare tip was
# pushed to the bore floor, which is the fit the user has printed and
# likes. Only the head, and the clip resting against it, end up further
# out than before — which is the whole point (the clip gets its gap).
PIN_STOP_D = PIN_TIP_CLEAR - PIN_TIP_NOSE            # 2.4 past nominal
PIN_FIT_SHIFT = PIN_TIP_NOSE                         # 1.6 nearer the tip
CLIP_GAP_ACT = CLIP_GAP - max(0.0, PIN_STOP_D - PIN_KEY_FLOAT - CLIP_AX_CLR)
assert CLIP_GAP_ACT >= 4 * NOZZLE_D / 5.0, \
    "the pin's dome lets the clip come too near the frame — lengthen it"

# where the lever's outboard face sits along the pin (tip at x = 0):
# the wedge seats with the lever's rib ending here
PIN_KEY_SEAT_X = (LEVER_Y_IN + LEVER_T) - PIN_TIP_END_Y - PIN_FIT_SHIFT


def _key_slot(x_end):
    """CUTTER for the pin's key SLOT (#1011; #1013 profile), on its −z
    side, running to `x_end`. INBOARD of the seat it tapers at the rib's
    own rate all the way to the tip: the lever's rib then has
    PIN_KEY_RATE of width clearance per mm still to go — slack for the
    whole push, closing over the last millimetre or two, and bearing
    over the rib's whole length at home. OUTBOARD of the seat it opens
    to PIN_CLIP_SLOT_FREE, where the TPU clip's rib merely slides, and
    closes to PIN_CLIP_SLOT_W only where that clip's hub comes to rest —
    so the clip goes on easy and grips at the end."""
    # the clip rests against the pin's HEAD, so its hub's inboard face
    # is one CLIP_T short of it — the squeeze closes right there (#1016:
    # before, this was reckoned off the column's face and the drawn gap,
    # which the dome has now changed)
    x_hub0 = LEVER_PIN_BODY_L - CLIP_T
    x_sq = x_hub0 - PIN_CLIP_SQUEEZE_RUN                # squeeze starts
    w_seat = PIN_KEY_W - PIN_KEY_SEAT_UNDER        # bites ≈2 mm early
    x_nose = -(PIN_TIP_NOSE + 0.5)                 # the slot grooves the
                                   # DOME too, or the lever's and the clip's
                                   # ribs could not pass it on the way on
    w = [(x_nose, w_seat + PIN_KEY_RATE * (PIN_KEY_SEAT_X - x_nose)),
         (PIN_KEY_SEAT_X, w_seat),
         (PIN_KEY_SEAT_X + 0.4, PIN_CLIP_SLOT_FREE),
         (x_sq, PIN_CLIP_SLOT_FREE),
         (x_sq + PIN_CLIP_SQUEEZE_RUN, PIN_CLIP_SLOT_W),
         (x_end, PIN_CLIP_SLOT_W)]
    pts = ([(x, y / 2.0) for x, y in w]
           + [(x, -y / 2.0) for x, y in reversed(w)])
    return (cq.Workplane("XY")
            .workplane(offset=-(LEVER_PIN_D / 2.0 - PIN_KEY_SLOT_H))
            .polyline(pts).close().extrude(-LEVER_PIN_D))


def _key_rib(y_face, y_sign, r_bore, widen=PIN_KEY_WIDEN, w=PIN_KEY_W,
             h=PIN_KEY_H, length=PIN_KEY_L):
    """The mating RIB, as a solid at the origin's bore: `w` wide at the
    face `y_face`, `w + widen` at `length` inboard of it (the wedge),
    standing `h` into a bore of radius `r_bore`, at the TOP (+z, where
    the pin's slot faces at rest)."""
    yi = y_face - y_sign * length
    pts = [(w / 2.0, y_face), (-w / 2.0, y_face),
           (-(w + widen) / 2.0, yi), ((w + widen) / 2.0, yi)]
    rib = (cq.Workplane("XY").workplane(offset=r_bore - h)
           .polyline(pts).close().extrude(h + 1.0))
    # its far end RAMPS out of the bore wall at 45° — square, that face
    # is a ceiling in the lever's along-y print (user #1011)
    ramp = (cq.Workplane("YZ")
            .polyline([(yi, r_bore), (yi + y_sign * h, r_bore - h),
                       (yi + y_sign * h, r_bore - h - 2.0),
                       (yi - y_sign * 2.0, r_bore - h - 2.0),
                       (yi - y_sign * 2.0, r_bore)])
            .close().extrude(w + widen + 4.0)
            .translate((-(w + widen + 4.0) / 2.0, 0, 0)))
    return rib.cut(ramp)


def _pivot_hole(pivot_x, pivot_z, y_outer, y_inner):
    """The KEYED pocket straight through the lever (#988): the pin's D
    grown by the snug fit, flat UP at rest — the pin turns WITH the
    lever and runs in round bores in the frame. Flat up (user #989): the
    frame bores are teardrops whose peaked crown never touches the pin
    anyway, so the flat hides under it and the pin keeps its full round
    where it actually bears (the ±41° flat stays inside the ±45° crown
    at rest). The lever prints along y, so the pocket is a vertical hole
    in its print: any shape is free."""
    lo = min(y_outer, y_inner)
    s = 1.0 if y_outer > y_inner else -1.0
    r = LEVER_PIN_D / 2.0 + LEVER_PIN_LEVER_CLR
    bore = (cq.Workplane("XZ").circle(r)
            .extrude(-(abs(y_inner - y_outer) + 1.0))
            .translate((pivot_x, lo - 0.5, pivot_z)))
    rib = _key_rib(y_outer, s, r).translate((pivot_x, 0.0, pivot_z))
    return bore.cut(rib)


def _handle(y0, y1, z_top):
    """The HANDLE plate below z_top (params BENT HANDLES, user #966): a
    LEVER_HANDLE_W-wide vertical bar on the pivot's x from z_top down
    to the shared knee LEVER_BEND_Z, a round knee, then a bar of the
    same width running to the grip disc at (HANDLE_X_BOT, HANDLE_Z_BOT)
    — 45° toward +x −z in the couch version, straight down on the desk
    build (params). For the brake z_top is above its
    pivot and the knee IS the pivot (the boss covers it); the ratchet's
    vertical stretch ends level with the brake's axle. In the plate's
    along-y print every face here is a vertical wall."""
    y_len = y1 - y0

    def _xz(wp):
        return wp.extrude(-y_len).translate((0, y0, 0))

    vert = _xz(cq.Workplane("XZ")
               .polyline([(LEVER_PIVOT_X - _HW, LEVER_BEND_Z),
                          (LEVER_PIVOT_X + _HW, LEVER_BEND_Z),
                          (LEVER_PIVOT_X + _HW, z_top),
                          (LEVER_PIVOT_X - _HW, z_top)]).close())
    knee = _xz(cq.Workplane("XZ").center(LEVER_PIVOT_X, LEVER_BEND_Z)
               .circle(_HW))
    # the knee→grip run: half-width _HW measured PERPENDICULAR to the
    # run (45° toward +x −z in the couch version; straight down on the
    # desk build, where HANDLE_X_BOT sits under the pivot — params)
    dx, dz = HANDLE_X_BOT - LEVER_PIVOT_X, HANDLE_Z_BOT - LEVER_BEND_Z
    L = math.hypot(dx, dz)
    nx, nz = -dz / L * _HW, dx / L * _HW          # unit normal × half-width
    diag = _xz(cq.Workplane("XZ")
               .polyline([(LEVER_PIVOT_X - nx, LEVER_BEND_Z - nz),
                          (LEVER_PIVOT_X + nx, LEVER_BEND_Z + nz),
                          (HANDLE_X_BOT + nx, HANDLE_Z_BOT + nz),
                          (HANDLE_X_BOT - nx, HANDLE_Z_BOT - nz)]).close())
    grip = _xz(cq.Workplane("XZ").center(HANDLE_X_BOT, HANDLE_Z_BOT)
               .circle(_HW))
    return vert.union(knee).union(diag).union(grip)


# ── Ratchet lever (v2 verbatim, v3 anchors) ──────────────────────────────────
def _build_ratchet_lever():
    """(1) horizontal pawl block minus the separator's tooth solid (exact
    negative) + the chord-tracking catch bump; (2) the handle through the
    pivot — vertical to the shared knee, then 45° out to the grip disc
    (_handle)."""
    y0, y1 = RATCHET_LEV_Y0, RATCHET_LEV_Y1
    y_len = y1 - y0

    h_x_hi = LEVER_PIVOT_X + _HW
    h_x_lo = 56.0                                # safely inside the valley radius
    horiz = (cq.Workplane("XY").workplane(offset=PAWL_Z_LO)
             .polyline([(h_x_lo, y0), (h_x_hi, y0), (h_x_hi, y1), (h_x_lo, y1)])
             .close().extrude(PAWL_Z_HI - PAWL_Z_LO))
    horiz = horiz.cut(ratchet_star_world())

    # catch-face bump (v2's parallelogram, chord-tracking back edge): one
    # catch face sits exactly at RATCHET_PHASE_DEG (the star is phased so)
    ts      = math.radians(RATCHET_PHASE_DEG)
    ts_next = ts + 2 * math.pi / RATCHET_TEETH
    r_root  = R_RIM - RATCHET_DEPTH
    overlap, depth = 0.5, 0.4
    valley   = (r_root * math.cos(ts), r_root * math.sin(ts))
    tip_next = (R_RIM * math.cos(ts_next), R_RIM * math.sin(ts_next))
    chord    = (tip_next[0] - valley[0], tip_next[1] - valley[1])
    clen     = math.hypot(*chord)
    cdir     = (chord[0] / clen, chord[1] / clen)
    A = ((R_RIM + overlap) * math.cos(ts), (R_RIM + overlap) * math.sin(ts))
    B = valley
    C = (B[0] + depth * cdir[0], B[1] + depth * cdir[1])
    D = (A[0] + depth * cdir[0], A[1] + depth * cdir[1])
    bump = (cq.Workplane("XY").workplane(offset=PAWL_Z_LO)
            .polyline([A, D, C, B]).close()
            .extrude(PAWL_Z_HI - PAWL_Z_LO))
    horiz = horiz.union(bump)

    # 45° UNDERSIDE BEVEL (v3, flipped-band consequence — assert-caught):
    # the brake band's full ring sits DIRECTLY BELOW the teeth at the SAME
    # radius, and the pawl's bottom-inner corner arcs ~2 down as it swings
    # out — unbeveled, it clips the band mid-release. Beveled, the
    # underside's low corner sits at the TIP radius and exits the band's
    # radius before crossing its top plane (the swept assert below samples
    # the beveled corners). The catch face keeps full height at the tips,
    # tapering toward the root; in the lever's along-Y print the bevel is
    # a vertical wall.
    _bev = (RATCHET_BAND_Z1 + 0.5) - PAWL_Z_LO
    horiz = horiz.cut(
        cq.Workplane("XZ")
        .polyline([(0.0, PAWL_Z_LO - 2.0), (R_RIM, PAWL_Z_LO - 2.0),
                   (R_RIM, PAWL_Z_LO),
                   (R_RIM - _bev, RATCHET_BAND_Z1 + 0.5),
                   (0.0, RATCHET_BAND_Z1 + 0.5)])
        .close().revolve(360.0, (0, 0), (0, 1)))

    # OVER-PULL WING (user stop rework #880, flipped to −y at #886: the
    # lever prints − to + y, so the nub TOPS the print on the inner
    # side): a chunk extending past the handle's y-band into the bay's
    # inner air strip — outside the handle's swept y, the only place
    # static frame material can catch the lever. Its flat bottom lands
    # on the beam-flank shelf at RATCHET_OPEN_DEG.
    horiz = horiz.union(
        cq.Workplane("XY").workplane(offset=PAWL_Z_LO)
        .polyline([(RWING_X0, RWING_Y_TIP), (RWING_X1, RWING_Y_TIP),
                   (RWING_X1, y0 + 1.0), (RWING_X0, y0 + 1.0)])
        .close().extrude(PAWL_Z_HI - PAWL_Z_LO))

    part = horiz.union(_handle(y0, y1, RATCHET_PIVOT_Z + _HW))
    part = part.union(_pivot_boss(LEVER_PIVOT_X, RATCHET_PIVOT_Z, y0, y1))
    # pin inserts from the +y (outer) side → pocket opens at y1
    part = part.cut(_pivot_hole(LEVER_PIVOT_X, RATCHET_PIVOT_Z, y1, y0))
    return part


# ── Brake lever + TPU pad — v2's CONTACT-frame design ────────────────────────
PAD_Y_MID = (BRAKE_LEV_Y0 + BRAKE_LEV_Y1) / 2.0   # −12


def _ratchet_pawl_clear_angle(target_r=R_RIM):
    """Pull angle at which the pawl just clears radius `target_r` (v2
    math; default = the tooth tips)."""
    x_pawl_rest = math.sqrt(R_ROOT ** 2 - PAWL_Y_MID ** 2)
    target_x    = math.sqrt(target_r ** 2 - PAWL_Y_MID ** 2)
    dx = x_pawl_rest - LEVER_PIVOT_X
    dz = PAWL_Z_MID - RATCHET_PIVOT_Z
    A, B = dx, -dz
    rhs = target_x - LEVER_PIVOT_X
    R = math.hypot(A, B)
    phi = math.degrees(math.atan2(B, A))
    ac = math.degrees(math.acos(max(-1.0, min(1.0, rhs / R))))
    return min(s for s in (phi - ac, phi + ac) if 0 < s < LEVER_TRAVEL_DEG)


RATCHET_PAWL_CLEAR_DEG  = _ratchet_pawl_clear_angle()
BRAKE_CONTACT_DELAY_DEG = 1.5                     # dead zone after pawl-clear
                                    # (3.0 until #1005: the one-bead-deeper
                                    # teeth need 13.8° to clear, and the
                                    # later contact tilted the pad's joint
                                    # into A_PAD_HANDLE_CLR)
BRAKE_CONTACT_DEG = RATCHET_PAWL_CLEAR_DEG + BRAKE_CONTACT_DELAY_DEG

# ── RATCHET OVER-PULL STOP (user: nothing stopped a hard pull — the
# lever could swing on and creep the TPU axle). REDESIGNED at #979
# after the print: the #880 scheme landed the pawl's WING on a sloped
# SHELF, and a landing face is the wrong shape here — at the stop the
# wing's motion runs ~96% ALONG that face, so a hard pull skids the
# wing off its end (probed on the build: first contact at 20.0°, the
# entire modeled travel, over ~5 mm² of sliding contact). The stop is a
# BUTTING face now: a RIB on the +X beam's flank, in the bay's inner
# air strip (the one place static frame material may live — the plate's
# own y-band is swept by the handle), whose −x end is the exact
# negative of the WING's leading face at the stop pose. Contact is pure
# compression, ~11 mm², normal to the motion. frame.py carves it by
# CUTTING a blank with the wing's own swept volume (ratchet_wing_swept
# below), so the face can never drift out of match, and the sweep
# starts at the axle-install lean — the install stays workable by
# construction. Geometry pinned HERE (the kinematics live here).
assert abs(RATCHET_OPEN_CLR - PAWL_CLEAR_MM) < 1e-9, \
    "the over-pull stop's rim clearance drifted off PAWL_CLEAR_MM"
RATCHET_OPEN_DEG = _ratchet_pawl_clear_angle(R_RIM + RATCHET_OPEN_CLR)
assert RATCHET_OPEN_DEG <= LEVER_TRAVEL_DEG + 1e-9, \
    "A_RSTOP_TRAVEL: the stop pose exceeds the modeled travel"
assert RATCHET_OPEN_DEG > RATCHET_PAWL_CLEAR_DEG + 1e-9, \
    "A_RSTOP_CLEAR: the stop catches before the pawl clears the teeth"

# THE CATCH ARCHITECTURE (#880 rework after the sweep audit + user's
# three corrections — ≥1.6 geometry, weld to the side wall, reach the
# lever): NOTHING static may live inside the lever's own y-band (7..17)
# at the catch zone — the handle sweeps all of it between rest and full
# pull (and, until the #988 pin pivot, the axle-install lean). So the PAWL
# grows a WING sideways past the handle's y, into the bay's outer air
# strip, and the fork COLUMN's inner face grows the SHELF it lands on:
# chunky, welded across its whole width, up at lever height. Contact =
# the wing's flat bottom on the shelf's matched sloped top, full-face,
# at RATCHET_OPEN_DEG. At rest (and through every working click) the
# wing sits x-INBOARD of the shelf; it swings out-and-down onto it.
# the nub rides the lever's −y (INNER) side (user #886: the lever
# prints − to + y — outer face on the bed — so the nub must TOP the
# print, not hang under the bed plane); the inner air strip (window
# edge 5.2 → lever face 7) is just as handle-free as the outer one,
# and the shelf gets a better root: the +X beam's flank
RWING_Y_TIP = LEVER_Y_IN - LEVER_SIDE_CLR        # 5.2 — tip FLUSH on the
                                                 # window edge / beam flank
                                                 # (user #934: the 0.5 air
                                                 # gap dropped — the wing
                                                 # runs all the way to the
                                                 # sidewall; sliding face
                                                 # contact, zero volume)
# inner edge at the separator-rim clearance limit (rim-derived so the
# wing tracks cable/capacity resizes); 4.0 of chunk outboard of it
RWING_Y_OUT = RATCHET_LEV_Y0 + 1.0               # 7.9 — the wing's far y
                                                 # (1.0 into the plate)
RWING_X0 = math.sqrt((R_RIM + 0.8) ** 2 - RWING_Y_TIP ** 2) + 0.05
RWING_X1 = RWING_X0 + 4.0
RSTOP_Y_TIP = LEVER_Y_IN - STOP_LEVER_CLR        # 6.7 — the rib's free face:
                                                 # STOP_LEVER_CLR off the
                                                 # lever's inner face, and it
                                                 # still overlaps the wing 1.5
RSTOP_WELD = NOZZLE_D                            # 0.8 — rib buried into
                                                 # the beam flank (one bead of
                                                 # real shared volume, the
                                                 # cadkit fusion rule)


def ratchet_wing_swept(n=32):
    """The WING's swept volume over the lever's whole life — rest (the
    pin pivot has no install lean since #988) to the over-pull stop. The
    frame's stop rib is a blank CUT BY THIS: what survives is rib, and
    the face it leaves is exactly the wing's leading face at
    RATCHET_OPEN_DEG (so the stop can never drift out of match, and
    nothing static ever stands in the install swing)."""
    box = (cq.Workplane("XY").workplane(offset=PAWL_Z_LO)
           .polyline([(RWING_X0, RWING_Y_TIP - RSTOP_WELD - 0.5),
                      (RWING_X1, RWING_Y_TIP - RSTOP_WELD - 0.5),
                      (RWING_X1, RWING_Y_OUT), (RWING_X0, RWING_Y_OUT)])
           .close().extrude(PAWL_Z_HI - PAWL_Z_LO))
    swept = None
    for i in range(n + 1):
        d = RATCHET_OPEN_DEG * i / n
        w = box.rotate((LEVER_PIVOT_X, 0, RATCHET_PIVOT_Z),
                       (LEVER_PIVOT_X, 1, RATCHET_PIVOT_Z), -d)
        swept = w if swept is None else swept.union(w, clean=False)
    return swept


assert RWING_X1 <= BEAM_IR + BEAM_SIZE - 0.5 + 1e-9, \
    "A_RSTOP_WELD: the rib blank starts past the beam flank's x extent"
assert math.hypot(RWING_X0, RWING_Y_TIP) >= R_RIM + 0.8 - 1e-9, \
    "A_RSTOP_RIM: the wing reaches within 0.8 of the separator's rim"

# ── Contact-frame geometry (v2's closed form) ────────────────────────────────
_AC_RAD     = math.radians(BRAKE_CONTACT_DEG)
_X_BAND_MID = math.sqrt(R_RIM ** 2 - PAD_Y_MID ** 2)      # band x at y −12
_PLAN_TAN   = abs(PAD_Y_MID) / _X_BAND_MID                # tangent-plane slope
_FACE_OFF   = BRAKE_RUBBER_T * math.sqrt(1.0 + _PLAN_TAN ** 2)
Z_TOP_C     = BRAKE_BAND_Z1 - BRAKE_PAD_TOP_MARGIN        # pad slab AT CONTACT
Z_BOT_C     = BRAKE_BAND_Z0 + BRAKE_PAD_BOT_MARGIN
BRAKE_PAD_H = Z_TOP_C - Z_BOT_C                           # 4.8 — full band
PAD_FLANGE_TOP = Z_TOP_C                                  # flange TOP-ALIGNED
PAD_FLANGE_BOT = Z_TOP_C - PAD_FLANGE_H                   # (v2's call)
_FL_OFF = PAD_FLANGE_T * math.sqrt(1.0 + _PLAN_TAN ** 2)


def _x_band_plane(y):
    """CONTACT pose: the band's tangent plane at the pad's mid azimuth."""
    return _X_BAND_MID + (y - PAD_Y_MID) * _PLAN_TAN


def _x_face(y):
    """CONTACT pose: the pad SLAB's back plane (= the flange's front)."""
    return _x_band_plane(y) + _FACE_OFF


def _x_face2(y):
    """CONTACT pose: the joint plane's anchor at the flange-bottom line."""
    return _x_face(y) + _FL_OFF


# joint-plane TILT (v2's rest-plumb interface — the whole pad↔arm
# interface leans forward by the contact angle about the flange-bottom
# line, so at REST it stands exactly plumb beside the handle)
_TILT_TAN = math.tan(_AC_RAD)


def _x_joint(y, z):
    return _x_face2(y) - (z - PAD_FLANGE_BOT) * _TILT_TAN


_JOINT_AX0 = (_x_face2(BRAKE_LEV_Y0), BRAKE_LEV_Y0, PAD_FLANGE_BOT)
_JOINT_AX1 = (_x_face2(BRAKE_LEV_Y1), BRAKE_LEV_Y1, PAD_FLANGE_BOT)


def _tilt_joint(wp):
    return wp.rotate(_JOINT_AX0, _JOINT_AX1, -BRAKE_CONTACT_DEG)


def _rest_xz(x, z):
    """Map a CONTACT-pose point to the REST pose (+contact about the pivot)."""
    dx, dz = x - LEVER_PIVOT_X, z - BRAKE_PIVOT_Z
    ca, sa = math.cos(_AC_RAD), math.sin(_AC_RAD)
    return (LEVER_PIVOT_X + dx * ca + dz * sa,
            BRAKE_PIVOT_Z - dx * sa + dz * ca)


def _to_rest(wp):
    return wp.rotate((LEVER_PIVOT_X, 0, BRAKE_PIVOT_Z),
                     (LEVER_PIVOT_X, 1, BRAKE_PIVOT_Z), BRAKE_CONTACT_DEG)


PAD_Z_HI = max(_rest_xz(_x_joint(y, Z_TOP_C), Z_TOP_C)[1]
               for y in (BRAKE_LEV_Y0, BRAKE_LEV_Y1))
# the brake side's swept CEILING (its fork-roof driver — frame.py): the
# highest any brake-side material reaches over the rest↔contact swing.
# Rest is the high extreme (the corners' radius never crosses the
# pivot's vertical inside the swing), so the candidates are the pad
# slab's and joint plane's top corners at REST, plus Z_TOP_C at contact.
BRAKE_SWEPT_TOP = max(
    Z_TOP_C, PAD_Z_HI,
    max(_rest_xz(_x_band_plane(_y), Z_TOP_C)[1]
        for _y in (BRAKE_LEV_Y0, BRAKE_LEV_Y1)),
)
# (A_PAD_RING is RETIRED — user's call: the band's gable crown over the
# bays is gone, the windows run open through the top rim, so nothing of
# the band sits over the pad's swing anymore. BRAKE_SWEPT_TOP stays
# exported — frame.py's arch roof is set from it.)

# ── BRAKE REST + END STOPS (user #988 — the brake had no hard stop at
# either end: at rest the TPU pad's corner squished into a corbel under
# the axle's pretwist, and at the far end only the pad's compression
# resisted). A WING on the lever's OUTER (−y) face, in the bay's outer
# air strip between the plate and the fork column (user #1002: the brake
# prints + to − y — its +y face on the bed — so the wing must TOP the
# print; on the inner face it hung under the bed plane. The ratchet's
# wing is the mirror case: it prints on its +y face, wing on −y), and it
# sweeps between two ribs that frame.py
# carves out of ONE blank with this sweep: the +x rib is where the wing
# lands at REST (the return spring holds it there), the −x rib where it
# lands at FULL PULL. The wing sits BWING_Z0 over the pivot to clear the
# frame's thrust ring and its teardrop crown (the ring shares this air
# strip, reaching ~7.0 over the pivot), and far enough out that both
# ribs keep ≥ 3 beads of length on the band strip + fork column they
# root on (asserted in frame.py). Pull moves this wing −x: it rides ABOVE the
# brake pivot, where pulling swings the pad into the band.
BRAKE_END_DEG = LEVER_TRAVEL_DEG             # the brake's full pull
BWING_Y_IN = BRAKE_LEV_Y0 + 1.0              # −16.25 — 1.0 into the plate
BWING_Y_TIP = BRAKE_LEV_Y0 - LEVER_SIDE_CLR  # −18.9 — tip FLUSH on the fork
                                             # column's inner face
BSTOP_Y_TIP = BRAKE_LEV_Y0 - STOP_LEVER_CLR  # −17.4 — the ribs' free face:
                                             # STOP_LEVER_CLR off the plate's
                                             # outer face, 1.5 over the wing
BSTOP_Y_ROOT = BWING_Y_TIP - RSTOP_WELD      # −19.7 — ribs buried a bead
                                             # into the column
BWING_X0 = LEVER_PIVOT_X - 2 * NOZZLE_D
BWING_X1 = BWING_X0 + 3 * NOZZLE_D
BWING_Z0 = BRAKE_PIVOT_Z + 10 * NOZZLE_D     # 8.0 over the pivot
BWING_Z1 = BWING_Z0 + 3 * NOZZLE_D


def _bwing_box(y_far):
    """The brake wing as a box at REST, from BWING_Y_IN (inside the
    plate) out to the signed `y_far`."""
    return (cq.Workplane("XY").workplane(offset=BWING_Z0)
            .polyline([(BWING_X0, BWING_Y_IN), (BWING_X1, BWING_Y_IN),
                       (BWING_X1, y_far), (BWING_X0, y_far)])
            .close().extrude(BWING_Z1 - BWING_Z0))


def brake_wing_at(d):
    """The brake wing's CUTTER box (reaching past the flank into the rib
    weld) at pull angle `d`."""
    return (_bwing_box(BSTOP_Y_ROOT - 0.5)
            .rotate((LEVER_PIVOT_X, 0, BRAKE_PIVOT_Z),
                    (LEVER_PIVOT_X, 1, BRAKE_PIVOT_Z), -d))


def brake_wing_swept(n=32):
    """The brake WING's swept volume, rest → full pull. frame.py cuts
    its stop blanks with this: the survivors are the rest and end ribs,
    each face the wing's own face at that pose."""
    swept = None
    for i in range(n + 1):
        w = brake_wing_at(BRAKE_END_DEG * i / n)
        swept = w if swept is None else swept.union(w, clean=False)
    return swept


# (v2's A_PAD_WINDOW is RETIRED — user #834: the bays run through the
# whole wall stack, so there is no cap over the swinging pad anymore; the
# nearest ceiling is frame_top at z 0, tens of mm clear. PAD_Z_HI stays
# as the reference number.)

# A_PAD_HANDLE_CLR / A_PAD_BOSS_CLR (v2): the plumb-at-rest joint plane
# must clear the handle face and the pivot boss — static gaps are the
# whole story (the plane contains the û slide direction).
_GUARD = 0.45
for _y in (BRAKE_LEV_Y0, BRAKE_LEV_Y1):
    for _z in (PAD_FLANGE_BOT, PAD_FLANGE_TOP):
        _xr, _zr = _rest_xz(_x_joint(_y, _z), _z)
        assert _xr <= LEVER_PIVOT_X - _HW - _GUARD + 1e-9, (
            f"A_PAD_HANDLE_CLR: joint-plane corner (y={_y}, z={_z:.1f}) "
            f"rests at x {_xr:.2f}, within {_GUARD} of the handle face at "
            f"{LEVER_PIVOT_X - _HW:.2f} — grow the pivot-out offset")
for _y in (BRAKE_LEV_Y0, BRAKE_LEV_Y1):
    _xr, _zr = _rest_xz(_x_face2(_y), PAD_FLANGE_BOT)
    _d = math.hypot(_xr - LEVER_PIVOT_X, _zr - BRAKE_PIVOT_Z)
    assert _d >= LEVER_BOSS_OD / 2.0 + 0.1 - 1e-9, (
        f"A_PAD_BOSS_CLR: flange bottom-back edge rests {_d:.2f} from the "
        f"pivot axis, within 0.1 of the boss {LEVER_BOSS_OD / 2.0:.2f}")


# ── Pad ↔ arm SLIDE JOINT (cadkit hook, v2's edge-bounded site) ──────────────
# Stop wall at the INNER end, mouth through the arm's OUTER face: braking
# drag (retraction sense carried from v2 — RE-CONFIRM when the v3 drum
# wind chirality is final) presses the tenon into the stop.
_B_DEG   = math.degrees(math.atan(_PLAN_TAN))     # face plan-bevel
_U_LEN   = (BRAKE_LEV_Y1 - BRAKE_LEV_Y0) * math.sqrt(1.0 + _PLAN_TAN ** 2)
_JT_STOP = 2 * NOZZLE_D
_PAD_J   = joint(PAD_FLANGE_H, None, tenon=PAD_SPEC, mortise=PAD_SPEC,
                 install="-z", bounded=True, clearance=PAD_JOINT_CLR)


def _pad_joint(tenon):
    """CONTACT-frame solid of the pad↔arm slide joint (v2 verbatim —
    library frame mapped onto û, then tilted onto the joint plane)."""
    L = _U_LEN - _JT_STOP + 1.0
    if tenon:
        j = _PAD_J.tenon(root=0.5, length=L)
    else:
        j = _PAD_J.mortise(drop=1.0, length=L)
    return _tilt_joint(
        j.rotate((0, 0, 0), (1, 0, 0), 90.0)
        .translate((0.0, _U_LEN - _JT_STOP, 0.0))
        .rotate((0, 0, 0), (0, 0, 1), -_B_DEG)
        .translate((_x_face2(BRAKE_LEV_Y0), BRAKE_LEV_Y0,
                    (PAD_FLANGE_BOT + PAD_FLANGE_TOP) / 2.0)))


def _joint_backspace():
    """Halfspace whose front face is the tilted joint plane — arm gets its
    end face by intersect, the pad flange its back by cut: identical
    plane, flush mate."""
    return _tilt_joint(
        cq.Workplane("XY").workplane(offset=-20.0)
        .polyline([(0.0, -30.0), (60.0, -30.0), (60.0, 30.0), (0.0, 30.0)])
        .close().extrude(60.0)
        .rotate((0, 0, 0), (0, 0, 1), -_B_DEG)
        .translate((_x_face2(BRAKE_LEV_Y0), BRAKE_LEV_Y0, PAD_FLANGE_BOT)))


def brake_pad_rect_corners(h=None):
    """(lever_face_corners, band_face_corners) of the TPU pad, REST pose."""
    z_bot = Z_TOP_C - h if h is not None else Z_BOT_C
    zf_bot = z_bot if h is not None else PAD_FLANGE_BOT
    lever, band = [], []
    for y, zs, zl in ((BRAKE_LEV_Y0, z_bot, zf_bot),
                      (BRAKE_LEV_Y1, z_bot, zf_bot),
                      (BRAKE_LEV_Y1, Z_TOP_C, Z_TOP_C),
                      (BRAKE_LEV_Y0, Z_TOP_C, Z_TOP_C)):
        xf, zf = _rest_xz(_x_joint(y, zl), zl)
        xb, zb = _rest_xz(math.sqrt(R_RIM ** 2 - y * y), zs)
        lever.append((xf, y, zf))
        band.append((xb, y, zb))
    return lever, band


def _build_brake_lever():
    """(1) the straight pad arm — contact-frame box trimmed to the tilted
    joint plane, back-rotated to rest; (2) the handle — vertical above the
    pivot, 45° out from the pivot to the grip disc (_handle)."""
    y0, y1 = BRAKE_LEV_Y0, BRAKE_LEV_Y1
    y_len = y1 - y0
    h_x_hi = LEVER_PIVOT_X + _HW
    horiz = (cq.Workplane("XY").workplane(offset=PAD_FLANGE_BOT)
             .polyline([(_x_joint(y0, PAD_FLANGE_TOP) - 0.5, y0),
                        (h_x_hi + 2.0, y0),
                        (h_x_hi + 2.0, y1),
                        (_x_joint(y1, PAD_FLANGE_TOP) - 0.5, y1)])
             .close().extrude(PAD_FLANGE_TOP - PAD_FLANGE_BOT))
    horiz = _to_rest(horiz.intersect(_joint_backspace()))
    horiz = horiz.cut(
        cq.Workplane("XY").workplane(offset=PAD_FLANGE_BOT - 3.0)
        .polyline([(h_x_hi, y0 - 1), (h_x_hi + 9.0, y0 - 1),
                   (h_x_hi + 9.0, y1 + 1), (h_x_hi, y1 + 1)])
        .close().extrude(PAD_FLANGE_H + 8.0))

    vert_top = _rest_xz(LEVER_PIVOT_X - _HW, Z_TOP_C)[1]
    part = horiz.union(_handle(y0, y1, vert_top))
    # the stop WING (#988; on the OUTER face since #1002), flush on the
    # fork column's plane
    part = part.union(_bwing_box(BWING_Y_TIP))
    part = part.union(_pivot_boss(LEVER_PIVOT_X, BRAKE_PIVOT_Z, y0, y1))
    # mortise channel cut AFTER every union (v2's refill lesson)
    part = part.cut(_to_rest(_pad_joint(tenon=False)))
    # pin inserts from the −y (outer) side → pocket opens at y0
    part = part.cut(_pivot_hole(LEVER_PIVOT_X, BRAKE_PIVOT_Z, y0, y1))
    return part


def _build_brake_pad(h=None):
    """The 95A TPU pad (contact frame): straight slab, band side cut
    CONCAVE by the KNURLED band itself (the texture's exact negative),
    L-flange carrying the hook rail, back-rotated to rest. Prints
    STANDING on its OUTER end face (v2's recipe)."""
    y0, y1 = BRAKE_LEV_Y0, BRAKE_LEV_Y1
    z_bot = Z_TOP_C - h if h is not None else Z_BOT_C
    slab = (cq.Workplane("XY").workplane(offset=z_bot)
            .polyline([(_x_band_plane(y0) - 0.5, y0), (_x_face(y0), y0),
                       (_x_face(y1), y1), (_x_band_plane(y1) - 0.5, y1)])
            .close().extrude(Z_TOP_C - z_bot))
    slab = slab.cut(knurl_band_world(z_bot - 1.0, Z_TOP_C + 1.0))
    if h is not None:
        return _to_rest(slab)
    slab = slab.union(
        cq.Workplane("XY").workplane(offset=PAD_FLANGE_BOT)
        .polyline([(_x_face(y0), y0), (_x_face2(y0), y0),
                   (_x_face2(y1), y1), (_x_face(y1), y1)])
        .close().extrude(PAD_FLANGE_H))
    slab = slab.cut(_joint_backspace())
    pad = _to_rest(slab)
    rail = _pad_joint(tenon=True)
    rail = rail.cut(
        cq.Workplane("XY").workplane(offset=PAD_FLANGE_BOT - 1.0)
        .center(_x_face2(y0) + 1.0, y0 - 5.0).rect(20.0, 10.0)
        .extrude(PAD_FLANGE_H + 2.0))
    return pad.union(_to_rest(rail))


def brake_pad_tall_demo(h):
    """DEMO ONLY (viewer poses): the pad at a forced height, top edge kept."""
    return _build_brake_pad(h)


# ── BAY WALL sill (user #977 — give the cable tray its wall back) ───────────
# The bay windows ran FULL HEIGHT because a hand had to reach in and
# grip a handle hanging straight down. The handles bend 45° outward
# below the knee now (params BENT HANDLES), so the wall can come back
# wherever no lever material can reach — and that wall matters: it is
# the cable TRAY's surface, and without it a wrap drops into the bay
# and jams. PER BAY (they differ a lot), the sill sits GUARD_LEVER_CLR
# under the LOWEST z that lever's material reaches inside the wall
# band's radial window, swept over the lever's whole life: rest through
# full pull (the #988 pin pivot dropped the axle-install lean, which was
# what held the RATCHET bay's sill ~15 lower). Rotation about y is
# y-independent in (x, z), so sweeping outline points is exact — as long
# as long EDGES are sampled densely (#988: the pawl block spans the band,
# and corner-only sampling missed it entirely once the lean was gone);
# the solids are probed against the result after each build.
# (The #887 chevrons are RETIRED with this: a wall beats a fence.)
def _handle_outline(z_top, n=24):
    """(x, z) samples around the handle bar: both edges of the vertical
    run and of the 45° run, plus the knee and grip discs."""
    pts = []
    for i in range(n + 1):
        z = LEVER_BEND_Z + (z_top - LEVER_BEND_Z) * i / n
        pts += [(LEVER_PIVOT_X - _HW, z), (LEVER_PIVOT_X + _HW, z)]
    dx, dz = HANDLE_X_BOT - LEVER_PIVOT_X, HANDLE_Z_BOT - LEVER_BEND_Z
    ln = math.hypot(dx, dz)
    nx, nz = (-dz / ln * _HW, dx / ln * _HW) if ln > 1e-9 else (_HW, 0.0)
    for i in range(n + 1):
        x = LEVER_PIVOT_X + dx * i / n
        z = LEVER_BEND_Z + dz * i / n
        pts += [(x - nx, z - nz), (x + nx, z + nz)]
    for cx, cz in ((LEVER_PIVOT_X, LEVER_BEND_Z),
                   (HANDLE_X_BOT, HANDLE_Z_BOT)):
        for k in range(n):
            a = 2.0 * math.pi * k / n
            pts.append((cx + _HW * math.cos(a), cz + _HW * math.sin(a)))
    return pts


def _bay_sill(pts, pivot_z):
    """Sill for a bay: GUARD_LEVER_CLR under the lowest point of `pts`
    that enters the wall band, over rest → full pull (no install lean
    since the #988 pin pivot)."""
    lo = 1e9
    for i in range(0, int(LEVER_TRAVEL_DEG * 4) + 1):
        a = math.radians(i / 4.0)                  # rest → full pull
        c, s = math.cos(a), math.sin(a)
        for x, z in pts:
            dx, dz = x - LEVER_PIVOT_X, z - pivot_z
            xr = LEVER_PIVOT_X + dx * c - dz * s
            if WALL_IR - 0.5 <= xr <= WALL_OR + 0.5:
                lo = min(lo, pivot_z + dx * s + dz * c)
    assert lo < 1e8, "A_BAY_SILL: no lever point crosses the wall band?"
    return lo - GUARD_LEVER_CLR


# the BRAKE's pad + joint corners (the lowest brake material there)
_g_pts = []
for _y in (BRAKE_LEV_Y0, BRAKE_LEV_Y1):
    for _zc in (Z_BOT_C, PAD_FLANGE_BOT, Z_TOP_C):
        _g_pts.append(_rest_xz(_x_band_plane(_y) - 0.5, _zc))
        _g_pts.append(_rest_xz(_x_face2(_y), _zc))
_g_pts.append(_rest_xz(LEVER_PIVOT_X + _HW + 2.0, PAD_FLANGE_BOT))
_g_pts.append(_rest_xz(LEVER_PIVOT_X - _HW, PAD_FLANGE_BOT))

def _edge(p0, p1, n=80):
    """Dense samples along a straight edge, (x, z) → (x, z)."""
    return [(p0[0] + (p1[0] - p0[0]) * i / n, p0[1] + (p1[1] - p0[1]) * i / n)
            for i in range(n + 1)]


_R_PTS = _handle_outline(RATCHET_PIVOT_Z + _HW)
for _z in (PAWL_Z_LO, PAWL_Z_HI):                  # the pawl block's long
    _R_PTS += _edge((56.0, _z), (LEVER_PIVOT_X + _HW, _z))   # edges
_B_PTS = _handle_outline(_rest_xz(LEVER_PIVOT_X - _HW, Z_TOP_C)[1]) + _g_pts
for _y in (BRAKE_LEV_Y0, BRAKE_LEV_Y1):            # the pad arm and pad
    for _z in (PAD_FLANGE_BOT, PAD_FLANGE_TOP):    # slab, contact frame →
        _B_PTS += [_rest_xz(x, z) for x, z in _edge(   # rest
            (_x_joint(_y, _z) - 0.5, _z), (LEVER_PIVOT_X + _HW + 2.0, _z))]
    for _z in (Z_BOT_C, Z_TOP_C):
        _B_PTS += [_rest_xz(x, z) for x, z in _edge(
            (_x_band_plane(_y) - 0.5, _z), (_x_face2(_y), _z), 20)]

BAY_WALL_Z1_RATCHET = _bay_sill(_R_PTS, RATCHET_PIVOT_Z)
BAY_WALL_Z1_BRAKE = _bay_sill(_B_PTS, BRAKE_PIVOT_Z)
# the restored wall is worth having at all only if it covers a real
# slice of the tray (the coil's own z band)
assert min(BAY_WALL_Z1_RATCHET, BAY_WALL_Z1_BRAKE) > FLOOR_Z1 + 5.0, \
    "A_BAY_WALL: a bay sill lands within 5 of the tray floor — no wall left"


# ── Kinematic assertions — fire at import ────────────────────────────────────
_PAD_BAND_CORNERS = brake_pad_rect_corners()[1]

KIN = assert_kinematics(
    ratchet_pivot_x=LEVER_PIVOT_X, ratchet_pivot_z=RATCHET_PIVOT_Z,
    ratchet_stop_deg=RATCHET_OPEN_DEG,     # the SHELF is the stop now
    pawl_y_mid=PAWL_Y_MID, pawl_z_mid=PAWL_Z_MID,
    brake_pivot_x=LEVER_PIVOT_X, brake_pivot_z=BRAKE_PIVOT_Z,
    brake_travel_deg=LEVER_TRAVEL_DEG,
    brake_contact_deg=BRAKE_CONTACT_DEG,
    brake_band_z0=BRAKE_BAND_Z0, brake_band_z1=BRAKE_BAND_Z1,
    # spool-chamber keep-out: the outermost design-capacity wrap's true
    # surface, above the disk top (v2's contact-only doctrine)
    chamber_r_min=R_OUT_COIL + CABLE_D / 2.0, chamber_z0=SEP_Z1,
    pad_band_corners=_PAD_BAND_CORNERS,
)

# A_PAWL_BAND_CLR — MIRRORED from v2 (the brake band's full ring now sits
# DIRECTLY UNDER the teeth): the pawl's BOTTOM corners must clear its TOP
# face over the in-service swing (rest → stop, + 1° of back-swing). The
# corners sampled are the BEVELED underside's: tip-radius corner at
# PAWL_Z_LO, root corner raised by the 45° bevel.
_pawl_band_clr = 1e9
for _y in (RATCHET_LEV_Y0, RATCHET_LEV_Y1):
    for _r, _zc in ((R_RIM, PAWL_Z_LO),
                    (R_ROOT, PAWL_Z_LO + (R_RIM - R_ROOT))):
        _x = math.sqrt(max(_r ** 2 - _y ** 2, 0.0))
        _dx, _dz = _x - LEVER_PIVOT_X, _zc - RATCHET_PIVOT_Z
        for _i in range(-int(math.ceil(RATCHET_OPEN_DEG)), 2):
            _t = math.radians(_i)
            _nx = LEVER_PIVOT_X + _dx * math.cos(_t) + _dz * math.sin(_t)
            _nz = RATCHET_PIVOT_Z - _dx * math.sin(_t) + _dz * math.cos(_t)
            if math.hypot(_nx, _y) <= R_RIM + 0.3:
                _pawl_band_clr = min(_pawl_band_clr, _nz - BRAKE_BAND_Z1)
assert _pawl_band_clr >= 0.3 - 1e-9, (
    f"A_PAWL_BAND_CLR: a swept pawl bottom corner comes within "
    f"{_pawl_band_clr:.2f} mm of the brake band's top — grow PAWL_BOT_CLR")


ratchet_lever = _build_ratchet_lever()
brake_lever = _build_brake_lever()
brake_pad_tpu = _build_brake_pad()


# ── RETURN CLIPS (user #1004, params RETURN CLIPS) ─────────────────────────
# In the lever's rotation plane (x, z) RELATIVE TO THE PIVOT. The layout
# is drawn for a leaf hanging straight DOWN (the ratchet's): pulling turns
# the pin by −d about +y, points below the pivot move +x, so the leaf sits
# on the post's −x side and pull bends it harder. Each lever's layout is
# that drawing rotated by CLIP_ROT[side] about the pivot — the brake's by
# 180° (user #1008: its leaf points UP, where the column has room for the
# long leaf; hanging down, its post ran into the frame's hand-access
# cutaway). A rotation, so preload, force and pull sense are unchanged.
CLIP_ROT = {+1: RATCHET_CLIP_ROT_DEG, -1: BRAKE_CLIP_ROT_DEG}
_LEAF_ROOT_Z = -math.sqrt(CLIP_HUB_R ** 2 - CLIP_LEAF_X ** 2)  # hub edge


def _back(p, deg):
    """(x, z) about the pivot, turned `deg` (positive = the bottom swings
    toward −x, opposite the pull)."""
    t = math.radians(deg)
    return (p[0] * math.cos(t) + p[1] * math.sin(t),
            -p[0] * math.sin(t) + p[1] * math.cos(t))


_POST_DOWN = (CLIP_LEAF_X + CLIP_LEAF_T / 2.0 + POST_R, -CLIP_CONTACT_R)


def clip_post_xz(y_sign):
    """The post's nose centre, (x, z) about the pivot, for one lever."""
    return _back(_POST_DOWN, CLIP_ROT[y_sign])


_LEAF_L = CLIP_CONTACT_R + _LEAF_ROOT_Z            # free length to contact
_LEAF_I = CLIP_T * CLIP_LEAF_T ** 3 / 12.0
_LEAF_K = 3.0 * TPU_E * _LEAF_I / _LEAF_L ** 3      # N/mm at the contact
CLIP_TRAVEL_DEG = LEVER_TRAVEL_DEG


def _clip_torque(d):
    """Return torque (N·mm) at pull `d`: the leaf's contact deflection is
    its preload plus the arc the pin swings that point through."""
    delta = CLIP_PRELOAD + math.radians(d) * CLIP_CONTACT_R
    return _LEAF_K * delta * CLIP_CONTACT_R, delta


def _leaf_strain(delta):
    return 1.5 * CLIP_LEAF_T * delta / _LEAF_L ** 2


CLIP_TORQUE = (_clip_torque(0.0)[0], _clip_torque(CLIP_TRAVEL_DEG)[0])
CLIP_STRAIN = (_leaf_strain(_clip_torque(0.0)[1]),
               _leaf_strain(_clip_torque(CLIP_TRAVEL_DEG)[1]))
assert CLIP_TORQUE[0] >= RETURN_TORQUE_MIN - 1e-9, \
    f"clip holds only {CLIP_TORQUE[0]:.1f} N·mm at rest"
assert CLIP_TORQUE[1] <= RETURN_TORQUE_MAX + 1e-9, \
    f"clip needs {CLIP_TORQUE[1]:.1f} N·mm at full pull — too stiff"
assert CLIP_STRAIN[0] <= 0.07 + 1e-9, \
    "the leaf's RESTING strain is high enough to creep"
# (the guard was 6% until #1014: the user wanted a few more degrees of
# preload, and preload IS resting strain — 5.6 of lean makes 6.9%. If a
# lever goes soft over months, this is the number that bought it, and
# the leaf has to grow longer or thinner to win the force back.)

_POST_OUT = CLIP_GAP + CLIP_T + CLIP_AX_CLR + NOZZLE_D  # post past the face
def _leaf_face_x(d, s):
    """In the DOWN drawing: the x of the leaf's post-side (+x) face at arc
    length `s` from its root, with the lever pulled `d`. The leaf is a
    cantilever clamped in the hub (which turns with the pin) and pushed
    back at the contact by the post: the printed-in preload lean plus the
    pin's turn, less the tip-load bending that returns the contact to the
    post."""
    th = math.radians(d)
    a0 = math.atan2(CLIP_PRELOAD, _LEAF_L)           # the printed lean
    delta = CLIP_PRELOAD + th * CLIP_CONTACT_R
    w = delta * s * s * (3.0 * _LEAF_L - s) / (2.0 * _LEAF_L ** 3)
    x_free = CLIP_LEAF_X + math.tan(a0) * s + th * (-_LEAF_ROOT_Z + s)
    return x_free - w + CLIP_LEAF_T / 2.0


# the UP-leaf (brake) post's BAR lies on the hub side of its nose, where
# the leaf's free span swings as it bends. In the down drawing the bar's
# near edge is the nose's 45° tangent at 10:30 — the leaf must keep 0.2
# off it the whole way up (on the 3:00→4:30 arc the leaf just ROLLS: its
# lean stays under 45°, so the contact shifts along the arc)
_CX = _POST_DOWN[0] - POST_R                       # the contact's x
for _i in range(11):
    for _j in range(1, 40):
        _sj = _LEAF_L * _j / 40.0
        _up = (_LEAF_ROOT_Z - _sj) + CLIP_CONTACT_R    # height over contact
        if _up >= POST_R / math.sqrt(2.0) + 0.3:
            assert (_leaf_face_x(CLIP_TRAVEL_DEG * _i / 10, _sj)
                    <= _POST_DOWN[0] - POST_R * math.sqrt(2.0) + _up
                    - 0.2 + 1e-9),                 "the bent leaf crosses the up-post's bar"

# the DOWN-leaf (ratchet) post's brace runs back to the band UNDER the
# leaf's tip, one bead below the post's block: as the leaf leans at full
# pull, its end face's post-side corner dips toward it
_BRACE_DROP = NOZZLE_D
for _i in range(11):
    _d = CLIP_TRAVEL_DEG * _i / 10
    _dx = (_leaf_face_x(_d, _LEAF_L) - _leaf_face_x(_d, _LEAF_L - 1e-3)) / 1e-3
    _nrm = math.hypot(_dx, 1.0)
    _ux, _uz = _dx / _nrm, -1.0 / _nrm                # the leaf past contact
    _ez = (_LEAF_ROOT_Z - _LEAF_L) + _uz * CLIP_TIP_PAST
    _low = _ez - abs(_ux) * CLIP_LEAF_T / 2.0         # its lowest corner
    assert _low >= -CLIP_CONTACT_R - POST_R - _BRACE_DROP + 0.8 - 1e-9, \
        "the leaf's tip dips within 0.8 of the ratchet post's brace"

# each post's 45° BAR lands on the wall band (#1010). Where it meets the
# band its underside must still sit above the bed, and clear of the 45°
# hand-access cutaway (frame._hand_wedge) that rises from (WALL_OR, the
# bed) — the cut only reaches x ≥ WALL_OR, well outboard of the band
_R_MID = (WALL_IR + WALL_OR) / 2.0
for _s, _pz in ((+1, RATCHET_PIVOT_Z), (-1, BRAKE_PIVOT_Z)):
    _cx, _cz = clip_post_xz(_s)
    _cx, _cz = LEVER_PIVOT_X + _cx, _pz + _cz
    _c_bot = ((_cz - _BRACE_DROP - _cx if CLIP_ROT[_s] == 0 else _cz - _cx)
              - (2.0 if CLIP_ROT[_s] == 0 else 1.0) * POST_R * math.sqrt(2.0))
    _xb = math.sqrt(_R_MID ** 2 - LEVER_COL_Y1 ** 2)   # the band, inboard y
    _zb = _xb + _c_bot                                 # the bar's underside
    assert _zb >= FLOOR_Z0 + max(0.0, _xb - WALL_OR) + 0.8 - 1e-9,         "a clip post's bar meets the band below the bed / in the cutaway"
    assert _cx - LEVER_PIVOT_X + POST_R <= (BEAM_IR + BEAM_SIZE)         - LEVER_PIVOT_X - 0.5,         "a clip post runs past the column's +x face"


def _build_lever_pin():
    """The PIVOT PIN (#988; single part again since #1004): one keyed
    Ø LEVER_PIN_D D-rod from the beam's blind-bore floor out through the
    lever, the fork column and the TPU return clip, to a round HEAD that
    caps the clip's hub — with the key SLOT (#1011) down its −z side.
    Symmetric, so the same part serves both levers. Prints standing on
    its head."""
    L = LEVER_PIN_BODY_L
    # the SHANK runs PIN_SHANK_UNDER under nominal and rises to full Ø
    # only over its last stretch — the bit the column's bore and the TPU
    # hub land on at the seat (user #1013). So the round fits are slack
    # everywhere the pin merely passes through, and the wedge key is the
    # only thing that grips at the end.
    x0 = PIN_KEY_SEAT_X
    x_full = LEVER_COL_Y1 - PIN_TIP_END_Y - PIN_FIT_SHIFT
    x_hub = x_full + PIN_KEY_FLOAT          # the fat bit stays out of the bore
    r_shank = LEVER_PIN_D / 2.0 - PIN_SHANK_UNDER / 2.0
    r_jrnl = LEVER_PIN_D / 2.0 - PIN_JOURNAL_UNDER / 2.0
    body = (cq.Workplane("YZ").circle(r_shank).extrude(x0)
            .union(cq.Workplane("YZ").workplane(offset=x0).circle(r_shank)
                   .workplane(offset=x_full - x0).circle(r_jrnl).loft())
            .union(cq.Workplane("YZ").workplane(offset=x_full).circle(r_jrnl)
                   .extrude(x_hub - x_full))
            .union(cq.Workplane("YZ").workplane(offset=x_hub)
                   .circle(LEVER_PIN_D / 2.0 + PIN_HUB_OVER / 2.0)
                   .extrude(L - x_hub)))
    head = (cq.Workplane("YZ").workplane(offset=L).circle(PIN_HEAD_R)
            .extrude(PIN_HEAD_T))
    # the DOME on the tip (#1016) — an ellipsoidal nose PIN_TIP_NOSE long
    # on the shank's own radius, so it meets the blind bore's floor at a
    # single point on the axis and that is the pin's depth stop
    n = PIN_TIP_NOSE
    r_nose = (r_shank ** 2 + n ** 2) / (2.0 * n)   # the cap's own radius:
    c_nose = r_nose - n                            # n tall on a r_shank base
    nose = (cq.Workplane("YZ").workplane(offset=c_nose).sphere(r_nose)
            .intersect(cq.Workplane("YZ").workplane(offset=-n)
                       .rect(4 * r_shank, 4 * r_shank).extrude(n)))
    return body.union(head).union(nose).cut(_key_slot(L + 0.5))


lever_pin = _build_lever_pin()                                # the print


def lever_pin_in_place(pivot_z, y_sign, pull_deg=0.0):
    """The pin as INSTALLED: tip on the beam's blind-bore floor, running
    out through the lever, the column and the clip to its head, D-flat UP
    at rest (under the bores' teardrop crowns) — and, keyed to its lever,
    turned with it by `pull_deg`."""
    pin = (lever_pin.rotate((0, 0, 0), (1, 0, 0), 180.0)   # flat to +z
           .rotate((0, 0, 0), (0, 0, 1), y_sign * 90.0)
           .translate((LEVER_PIVOT_X,
                       y_sign * (PIN_TIP_END_Y - PIN_STOP_D), pivot_z)))
    if pull_deg:
        pin = pin.rotate((LEVER_PIVOT_X, 0, pivot_z),
                         (LEVER_PIVOT_X, 1, pivot_z), -pull_deg)
    return pin


def _clip_profile(bent, y_sign):
    """One lever's clip in (x, z) about the pivot, extruded y 0..CLIP_T:
    the hub with the pin's D bore (flat UP, as the pin sits at rest — so
    the ratchet's and the brake's clips are different parts) and the
    leaf. `bent` = as installed (leaf straight, on the post); otherwise
    as PRINTED — the leaf leaned CLIP_PRELOAD further into the post about
    its root."""
    hub = cq.Workplane("XZ").circle(CLIP_HUB_R).extrude(-CLIP_T)
    z_tip = -(CLIP_CONTACT_R + CLIP_TIP_PAST)
    leaf = (cq.Workplane("XZ").center(CLIP_LEAF_X, z_tip / 2.0)
            .rect(CLIP_LEAF_T, -z_tip).extrude(-CLIP_T))
    if not bent:
        a = math.degrees(math.atan2(CLIP_PRELOAD, _LEAF_L))
        leaf = leaf.rotate((CLIP_LEAF_X, 0, _LEAF_ROOT_Z),
                           (CLIP_LEAF_X, 1, _LEAF_ROOT_Z), -a)
    clip = hub.union(leaf).rotate((0, 0, 0), (0, 1, 0), CLIP_ROT[y_sign])
    # the bore, keyed by the clip's OWN rib into the pin's slot — a touch
    # wider than the slot there, so the TPU is squeezed (#1011)
    rb = LEVER_PIN_D / 2.0 + CLIP_BORE_CLR
    bore = (cq.Workplane("XZ").circle(rb).extrude(-CLIP_T - 2.0)
            .translate((0, -1.0, 0)))
    rib = _key_rib(-1.0, -1.0, rb, widen=0.0, w=PIN_CLIP_RIB_W,
                   h=PIN_KEY_SLOT_H - 0.1, length=CLIP_T + 2.0)
    clip = clip.cut(bore.cut(rib))
    return clip if y_sign > 0 else clip.mirror("XZ")


ratchet_clip_tpu = _clip_profile(bent=False, y_sign=+1)      # the prints
brake_clip_tpu = _clip_profile(bent=False, y_sign=-1)


def return_clip_in_place(pivot_z, y_sign):
    """The clip as installed, AT REST (leaf bent onto its post): on the
    pin, resting against its HEAD — so what is left over the column's
    outer face is CLIP_GAP_ACT, the drawn gap less however far the dome
    lets the pin push the clip in (#1016)."""
    return _clip_profile(bent=True, y_sign=y_sign).translate(
        (LEVER_PIVOT_X, y_sign * (LEVER_COL_Y1 + CLIP_GAP_ACT), pivot_z))


def clip_post(pivot_z, y_sign):
    """The frame's POST for one lever's clip: a round nose the leaf bears
    on, carried by a 45° BAR running down and back (−x) into the frame's
    WALL BAND (user #1010 — the band is a full ring, so it is there under
    every y-slice of the bar: the bar grows out of it at 45° and needs no
    gusset off the column face, which is what carried the post until
    #1009). The bar's two edges are the nose's own 45° tangents, so nose
    and bar read as one capsule; it dies mid-band, never into the spool
    chamber, and the column buries its inboard bead.
      · UP leaf (brake): the tangents at 10:30 and 4:30 — the leaf rests
        on the nose's 3:00 and the bar leans away from it;
      · DOWN leaf (ratchet): the leaf comes down the nose's 9:00 side,
        so the bar hangs under a square BLOCK instead, _BRACE_DROP below
        it — clear of the leaf's tip as it leans (asserted above)."""
    cx0, cz0 = clip_post_xz(y_sign)
    cx, cz = LEVER_PIVOT_X + cx0, pivot_z + cz0
    y_face = y_sign * LEVER_COL_Y1
    y0, y1 = y_face - y_sign * NOZZLE_D, y_face + y_sign * _POST_OUT
    lo, hi = min(y0, y1), max(y0, y1)
    B = 60.0                       # far enough −x to reach the band at any y
    r_mid = (WALL_IR + WALL_OR) / 2.0

    def xz_prism(pts):
        return (cq.Workplane("XZ").polyline(pts).close()
                .extrude(-(hi - lo)).translate((0.0, lo, 0.0)))

    def strip(c_top):
        """The 45° bar: slope +1, POST_R × 2 wide square to itself, its
        TOP edge the line z = x + c_top."""
        c_bot = c_top - 2.0 * POST_R * math.sqrt(2.0)
        return xz_prism([(cx - B, cx - B + c_top), (cx + B, cx + B + c_top),
                         (cx + B, cx + B + c_bot), (cx - B, cx - B + c_bot)])

    def to_band(w):
        """Dies mid-band: everything inside stays out of the chamber."""
        return w.cut(cq.Workplane("XY").workplane(offset=-200.0)
                     .circle(r_mid).extrude(400.0))

    def keep_below(z_lim):
        return xz_prism([(cx - B, z_lim), (cx + B, z_lim),
                         (cx + B, z_lim + B), (cx - B, z_lim + B)])

    nose = cq.Workplane(obj=cq.Solid.makeCylinder(
        POST_R, hi - lo, cq.Vector(cx, lo, cz), cq.Vector(0, 1, 0)))
    if CLIP_ROT[y_sign] == 0:                      # DOWN leaf: nose + block
        block = xz_prism([(cx - POST_R, cz - POST_R), (cx + POST_R, cz - POST_R),
                          (cx + POST_R, cz), (cx - POST_R, cz)])
        bar = strip(cz - _BRACE_DROP - cx).cut(keep_below(cz - POST_R))
        bar = bar.cut(xz_prism([(cx + POST_R, cz - B), (cx + B, cz - B),
                                (cx + B, cz + B), (cx + POST_R, cz + B)]))
        return nose.union(block).union(to_band(bar))
    # UP leaf: the capsule — the nose sits BETWEEN its two tangents, so
    # the bar ends on the diameter square to it and the nose rounds it off
    bar = strip(cz - cx + POST_R * math.sqrt(2.0))
    bar = bar.cut(xz_prism([(cx - 3 * B, cz + 3 * B), (cx + 3 * B, cz - 3 * B),
                            (cx + 3 * B, cz + 3 * B)]))  # past the diameter
    return nose.union(to_band(bar))
