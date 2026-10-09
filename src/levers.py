"""The RATCHET lever — v2's radial-engagement design, hardware-free.

The BRAKE went at #1019 (user): lever, TPU pad, its return clip, its
fork, its stops, its bay and its knurled band on the separator. What it
bought — holding a paid-out length against the spring — the ratchet
already does, and it cost a second lever, four printed parts, the −y
half of the frame's fork and 4.8 of disk. The one place its removal did
NOT pay was height: see RIM_H in params, where the bottom 608's pocket,
not the bands, sets how thick the disk has to be.

v2's architecture kept whole: an XZ plate lever (LEVER_T, HANDLE_W) at
+X on the beam's +y flank, pull = +X; the pawl edge is the exact
tooth-profile negative + catch bump. The pivot sits 10 above the tooth
band: rest = pawl seated in the teeth, pull swings it out, and the
pawl's one-bead running clearance is at its BOTTOM (A_PAWL_BAND_CLR).
The lever reaches the band through its bay; its over-pull stop is a
butting rib the frame carves with the wing's own swept volume (#979).

The kinematic assertion suite (lever_kinematics.py) fires at import.
TPU parts (the return clip) print in 95A — black in the viewer.
"""

import math

import cadquery as cq

from cadkit.joinery import joint

from .axle import ratchet_star_world
from .lever_kinematics import assert_kinematics, R_ROOT, PAWL_CLEAR_MM
from .params import (
    NOZZLE_D,
    RIM_OD, RATCHET_DEPTH, RATCHET_TEETH, RATCHET_PHASE_DEG,
    SEP_Z0, SEP_Z1, R_OUT_COIL, CABLE_D,
    LEVER_T, LEVER_HANDLE_W, LEVER_PIVOT_X, LEVER_TRAVEL_DEG,
    RATCHET_LEV_Y0, RATCHET_LEV_Y1,
    RATCHET_PIVOT_Z, HANDLE_Z_BOT, LEVER_BEND_Z, HANDLE_X_BOT,
    LEVER_PIN_D, LEVER_PIN_LEVER_CLR, LEVER_BOSS_OD,
    PIN_KEY_W, PIN_KEY_H, PIN_KEY_L, PIN_KEY_WIDEN, PIN_KEY_SLOT_H,
    PIN_KEY_RATE, PIN_KEY_FLOAT, PIN_CLIP_SLOT_W, PIN_CLIP_RIB_W,
    PIN_SHANK_UNDER, PIN_CLIP_SLOT_FREE, PIN_CLIP_SQUEEZE_RUN,
    PIN_KEY_SEAT_UNDER, PIN_HUB_OVER, PIN_JOURNAL_UNDER,
    LEVER_T,
    LEVER_PIN_L, PIN_COL_Y0, PIN_TIP_Y,
    CLIP_FACE_Y, CLIP_SIDE,
    RATCHET_BAND_Z0,
    LEVER_Y_IN, BEAM_SIZE, POST_OUT_T,
    LEVER_COL_Y1, LEVER_PIN_BODY_L, BEAM_IR,
    TPU_E, CLIP_GAP, CLIP_T, CLIP_HUB_R, CLIP_BORE_CLR, CLIP_LEAF_T,
    CLIP_LEAF_X, CLIP_CONTACT_R, CLIP_TIP_PAST, CLIP_PRELOAD, CLIP_AX_CLR,
    CLIP_MODEL_OFFSET,
    POST_R, PIN_HEAD_R, PIN_HEAD_T, RETURN_TORQUE_MAX,
    RATCHET_CLIP_ROT_DEG, FLOOR_Z0,
    WALL_IR, WALL_OR, RATCHET_OPEN_CLR, LEVER_SIDE_CLR, STOP_LEVER_CLR,
    BEAM_IR, GUARD_LEVER_CLR, FLOOR_Z1,
)

R_RIM = RIM_OD / 2.0                              # 69.435 — band / tooth tips
_HW   = LEVER_HANDLE_W / 2.0

# Band z-geometry (world) — ONE band since #1019: the teeth, with a
# plain ring under them (the knurled brake band is gone).
RATCHET_BAND_Z1 = SEP_Z1                          # −34.4

# Pawl block spans the tooth band; kinematics sample at its mid point.
PAWL_BOT_CLR = 0.0                 # the pawl spans the WHOLE tooth band
                                   # (#1025). It used to stand a bead off
                                   # the band's bottom because the brake's
                                   # knurled ring sat directly under the
                                   # teeth at the same radius; with that
                                   # ring gone there is nothing below the
                                   # disk out here at all — the bearing is
                                   # at r ≤ 11, 2.4 further down — so the
                                   # clearance was buying nothing and
                                   # costing a fifth of the catch height
PAWL_Z_LO, PAWL_Z_HI = RATCHET_BAND_Z0 + PAWL_BOT_CLR, RATCHET_BAND_Z1
PAWL_Y_MID = (RATCHET_LEV_Y0 + RATCHET_LEV_Y1) / 2.0        # 12
PAWL_Z_MID = (PAWL_Z_LO + PAWL_Z_HI) / 2.0                  # −36.4


def _pivot_boss(pivot_x, pivot_z, y0, y1):
    """Ø LEVER_BOSS_OD boss along the pivot axis, spanning the plate — the
    6-wide handle is narrower than the axle pocket, the bulge carries it."""
    return (cq.Workplane("XZ").center(pivot_x, pivot_z)
            .circle(LEVER_BOSS_OD / 2)
            .extrude(-(y1 - y0)).translate((0, y0, 0)))


# The pin spans the beam (#1029, params): its x = 0 is the fork column's
# OUTER face — the bore runs THROUGH it since #1035 — and it grows the
# other way, across the beam and out the −y side to the clip. Depth is
# set by the WEDGE in the lever, and with no blind floor anywhere there
# is nothing else it could bottom on.

# where the lever's KEY face sits along the pin (tip at x = 0): the pin
# enters the lever at its −y face, so that is where the rib starts and
# where the wedge seats
PIN_KEY_SEAT_X = PIN_TIP_Y - LEVER_Y_IN


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
    # the clip's hub comes to rest CLIP_GAP off the beam's −y flank —
    # the squeeze closes right there
    x_hub0 = PIN_TIP_Y + CLIP_FACE_Y + CLIP_GAP
    x_sq = x_hub0 - PIN_CLIP_SQUEEZE_RUN                # squeeze starts
    w_seat = PIN_KEY_W - PIN_KEY_SEAT_UNDER        # bites ≈2 mm early
    x_nose = -0.5                                  # the slot runs out of
                                   # the pin's flat tip (#1035), so the
                                   # lever's and the clip's ribs can enter
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
    build (params). In the plate's along-y print every face here is a
    vertical wall."""
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

    # (the 45° UNDERSIDE BEVEL is RETIRED at #1025 with the brake band
    # it dodged: the pawl's bottom-inner corner arcs ~2 down as it swings
    # out, and unbeveled it used to clip that ring. Nothing is under the
    # teeth now, so the catch face runs full height at every radius —
    # the bevel was pure lost contact area, user-caught.)

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
    # the pin crosses the beam and enters from the −y side now (#1029),
    # so the wedge's rib starts at the plate's INNER face and runs outward
    part = part.cut(_pivot_hole(LEVER_PIVOT_X, RATCHET_PIVOT_Z, y0, y1))
    return part


# ── Ratchet pawl kinematics ─────────────────────────────────────────────────
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


def _edge(p0, p1, n=80):
    """Dense samples along a straight edge, (x, z) → (x, z)."""
    return [(p0[0] + (p1[0] - p0[0]) * i / n, p0[1] + (p1[1] - p0[1]) * i / n)
            for i in range(n + 1)]


_R_PTS = _handle_outline(RATCHET_PIVOT_Z + _HW)
for _z in (PAWL_Z_LO, PAWL_Z_HI):                  # the pawl block's long
    _R_PTS += _edge((56.0, _z), (LEVER_PIVOT_X + _HW, _z))   # edges
BAY_WALL_Z1_RATCHET = _bay_sill(_R_PTS, RATCHET_PIVOT_Z)
# (the BRAKE bay's sill went with the brake — #1019. Its pad and joint
# corners were the lowest lever material anywhere, so that bay's wall
# was the low one; the −y side is a plain wall now.)
# the restored wall is worth having at all only if it covers a real
# slice of the tray (the coil's own z band)
assert BAY_WALL_Z1_RATCHET > FLOOR_Z1 + 5.0, \
    "A_BAY_WALL: the bay sill lands within 5 of the tray floor — no wall left"


# ── Kinematic assertions — fire at import ────────────────────────────────────
KIN = assert_kinematics(
    ratchet_pivot_x=LEVER_PIVOT_X, ratchet_pivot_z=RATCHET_PIVOT_Z,
    ratchet_stop_deg=RATCHET_OPEN_DEG,     # the SHELF is the stop now
    pawl_y_mid=PAWL_Y_MID, pawl_z_mid=PAWL_Z_MID,
    # spool-chamber keep-out: the outermost design-capacity wrap's true
    # surface, above the disk top (v2's contact-only doctrine)
    chamber_r_min=R_OUT_COIL + CABLE_D / 2.0, chamber_z0=SEP_Z1,
)

# (A_PAWL_BAND_CLR is RETIRED at #1025. It asserted that the pawl's
# swept bottom corners cleared the brake band's top face; that band is
# gone, the pawl now spans the tooth band exactly, and below the disk at
# this radius there is open air for the corner to arc into.)

ratchet_lever = _build_ratchet_lever()


# ── RETURN CLIPS (user #1004, params RETURN CLIPS) ─────────────────────────
# In the lever's rotation plane (x, z) RELATIVE TO THE PIVOT. The layout
# is drawn for a leaf hanging straight DOWN (the ratchet's): pulling turns
# the pin by −d about +y, points below the pivot move +x, so the leaf sits
# on the post's −x side and pull bends it harder. Each lever's layout is
# that drawing rotated by CLIP_ROT[side] about the pivot. (The rotation
# existed for the brake's clip, which pointed its leaf UP; #1019 left
# only this one, at 0°. Kept as a rotation because it costs nothing and
# it is how the layout is anchored.)
CLIP_ROT = {CLIP_SIDE: RATCHET_CLIP_ROT_DEG}
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


# What the leaf ACTUALLY deflects, measured (params CLIP_MODEL_OFFSET):
# the drawn preload less however far the real part sits behind the model.
# Everything below is force, and force follows the real deflection — the
# GEOMETRY still uses the full CLIP_PRELOAD, because that is what prints.
CLIP_PRELOAD_REAL = max(0.0, CLIP_PRELOAD - CLIP_MODEL_OFFSET)


def _clip_torque(d):
    """Return torque (N·mm) at pull `d`: the leaf's REAL contact
    deflection plus the arc the pin swings that point through."""
    delta = CLIP_PRELOAD_REAL + math.radians(d) * CLIP_CONTACT_R
    return _LEAF_K * delta * CLIP_CONTACT_R, delta


def _leaf_strain(delta):
    return 1.5 * CLIP_LEAF_T * delta / _LEAF_L ** 2


CLIP_TORQUE = (_clip_torque(0.0)[0], _clip_torque(CLIP_TRAVEL_DEG)[0])
CLIP_STRAIN = (_leaf_strain(_clip_torque(0.0)[1]),
               _leaf_strain(_clip_torque(CLIP_TRAVEL_DEG)[1]))
# (there is no MINIMUM resting torque any more — user #1039. The clip is
# set to land exactly ON its post: contact, no gap, so no slack in the
# lever. That leaves nothing pressing the pawl into the teeth at rest,
# which is now a deliberate choice rather than a failure — each 0.8 of
# CLIP_PRELOAD above 3.2 would buy 26.4 N·mm of it back.)
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


# the UP-leaf post's BAR lay on the hub side of its nose, where
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
for _s, _pz in ((CLIP_SIDE, RATCHET_PIVOT_Z),):
    _cx, _cz = clip_post_xz(_s)
    _cx, _cz = LEVER_PIVOT_X + _cx, _pz + _cz
    _c_bot = (_cz - _BRACE_DROP - _cx) - 2.0 * POST_R * math.sqrt(2.0)
    assert _cx - LEVER_PIVOT_X + POST_R <= (BEAM_IR + BEAM_SIZE)         - LEVER_PIVOT_X - 0.5,         "a clip post runs past the column's +x face"


def _build_lever_pin():
    """The PIVOT PIN (#988; single part again since #1004): one keyed
    Ø LEVER_PIN_D D-rod from the beam's blind-bore floor out through the
    lever, the fork column and the TPU return clip, to a round HEAD that
    caps the clip's hub — with the key SLOT (#1011) down its −z side.
    Symmetric, so the same part serves both levers. Prints standing on
    its head."""
    L = LEVER_PIN_BODY_L
    # ONE journal diameter down the whole body (#1024 halved the shank's
    # undersize onto the journal's own number, so the old taper between
    # them evaluated flat anyway), then the HUB section — a touch over
    # nominal — where the TPU clip grips, outboard of every frame bore.
    x_hub = PIN_TIP_Y + CLIP_FACE_Y + CLIP_GAP      # the clip's inner face
    r_shank = LEVER_PIN_D / 2.0 - PIN_SHANK_UNDER / 2.0
    body = (cq.Workplane("YZ").circle(r_shank).extrude(x_hub)
            .union(cq.Workplane("YZ").workplane(offset=x_hub)
                   .circle(LEVER_PIN_D / 2.0 + PIN_HUB_OVER / 2.0)
                   .extrude(L - x_hub)))
    head = (cq.Workplane("YZ").workplane(offset=L).circle(PIN_HEAD_R)
            .extrude(PIN_HEAD_T))
    return body.union(head).cut(_key_slot(L + 0.5))


lever_pin = _build_lever_pin()                                # the print


def lever_pin_in_place(pivot_z, y_sign, pull_deg=0.0):
    """The pin as INSTALLED (#1029): its tip in the +y fork column's
    blind bore, running −y through the lever and the beam and out to the
    clip on the far side, slot UP at rest (under the bores' teardrop
    crowns) — and, keyed to its lever, turned with it by `pull_deg`."""
    pin = (lever_pin.rotate((0, 0, 0), (1, 0, 0), 180.0)   # slot to +z
           .rotate((0, 0, 0), (0, 0, 1), y_sign * 90.0)
           .translate((LEVER_PIVOT_X, PIN_TIP_Y, pivot_z)))
    if pull_deg:
        pin = pin.rotate((LEVER_PIVOT_X, 0, pivot_z),
                         (LEVER_PIVOT_X, 1, pivot_z), -pull_deg)
    return pin


def _clip_profile(y_sign):
    """The clip in (x, z) about the pivot, extruded y 0..CLIP_T: the hub
    with the pin's bore (flat UP, as the pin sits at rest) and the leaf,
    leaned CLIP_PRELOAD into the post about its root.

    ONE shape (user #1037). There used to be a second, "bent" variant —
    the leaf straight, as the post holds it once installed — and the
    assembly showed that one. But then the viewer was rendering a part
    that is not the part: the assembly may ORIENT geometry, never change
    it. The real clip goes in the assembly, so the preload shows there as
    what it physically is — the leaf passing through where the post's
    surface is, by exactly the interference the TPU takes up."""
    hub = cq.Workplane("XZ").circle(CLIP_HUB_R).extrude(-CLIP_T)
    z_tip = -(CLIP_CONTACT_R + CLIP_TIP_PAST)
    leaf = (cq.Workplane("XZ").center(CLIP_LEAF_X, z_tip / 2.0)
            .rect(CLIP_LEAF_T, -z_tip).extrude(-CLIP_T))
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


ratchet_clip_tpu = _clip_profile(CLIP_SIDE)                  # the print


def return_clip_in_place(pivot_z, y_sign):
    """The clip as installed, AT REST: the PRINTED part, oriented — on the
    pin, resting against its HEAD — so what is left over the column's
    outer face is CLIP_GAP — exact now, because the wedge seats the pin
    and the head carries the clip with it (#1029)."""
    return _clip_profile(y_sign).translate(
        (LEVER_PIVOT_X, y_sign * (CLIP_FACE_Y + CLIP_GAP), pivot_z))


def clip_post(pivot_z, y_sign):
    """The frame's POST for one lever's clip: a round nose the leaf bears
    on, carried by a 45° BAR running down and back (−x) into the frame's
    WALL BAND (user #1010 — the band is a full ring, so it is there under
    every y-slice of the bar: the bar grows out of it at 45° and needs no
    gusset off the column face, which is what carried the post until
    #1009). The bar's two edges are the nose's own 45° tangents, so nose
    and bar read as one capsule; it dies mid-band, never into the spool
    chamber, and the column buries its inboard bead.
    The leaf comes DOWN the nose's 9:00 side, so the bar hangs under a
    square BLOCK, _BRACE_DROP below it — clear of the leaf's tip as it
    leans (asserted above). (#1019 retired the UP-leaf variant with the
    brake's clip, which rested on the nose's 3:00 instead.)"""
    cx0, cz0 = clip_post_xz(y_sign)
    cx, cz = LEVER_PIVOT_X + cx0, pivot_z + cz0
    y_face = y_sign * CLIP_FACE_Y
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
    # the block runs DOWN to where the bar's top edge crosses its inboard
    # face, so the two merge and there is no ceiling between them (user
    # #1028). It used to stop at cz − POST_R, one _BRACE_DROP above the
    # bar, which left an 11 mm² flat facing straight down in the print —
    # the bar only reached the block's OUTBOARD half. _BRACE_DROP is the
    # leaf-tip clearance to the BAR and is unchanged; the block sits
    # inboard of where the tip swings.
    block = xz_prism([(cx - POST_R, cz - POST_R - _BRACE_DROP),
                      (cx + POST_R, cz - POST_R - _BRACE_DROP),
                      (cx + POST_R, cz), (cx - POST_R, cz)])
    bar = strip(cz - _BRACE_DROP - cx).cut(keep_below(cz - POST_R))
    # never below the BED, nor into the hand-access cutaway (#1027).
    # frame.py unions the posts AFTER its own _hand_wedge cut, so the post
    # has to respect that 45° itself: same shape, revolved — flat at the
    # bed inside WALL_OR, rising at 45° outside it.
    _F = 200.0
    under_bed = (cq.Workplane("XZ")
                 .polyline([(0.0, FLOOR_Z0), (WALL_OR, FLOOR_Z0),
                            (_F, FLOOR_Z0 + (_F - WALL_OR)),
                            (_F, FLOOR_Z0 - _F), (0.0, FLOOR_Z0 - _F)])
                 .close().revolve(360.0, (0, 0), (0, 1)))
    bar = bar.cut(under_bed)
    bar = bar.cut(xz_prism([(cx + POST_R, cz - B), (cx + B, cz - B),
                            (cx + B, cz + B), (cx + POST_R, cz + B)]))
    # the WHOLE post respects the bed, not just the bar (#1073). The
    # DESK build runs a 4 ft cable where the couch runs 7, so its drum
    # floor is 5.8 SHALLOWER while the lever pivot does not move — which
    # lifted the bed into the nose's bottom-outboard corner, 1.58 mm3 of
    # it. The leaf bears on the nose's 9:00 side, so this never touches
    # the contact; on the couch the post is already clear and it removes
    # nothing. A_POST_BED below still checks every vertex — the cut is
    # what makes it pass, so the assert now guards the CUT's coverage.
    return nose.union(block).union(to_band(bar)).cut(under_bed)


# A_POST_BED — on the BUILT solid, because the analytic version got this
# wrong once (#1027): it sampled the band chord at the post's INBOARD y,
# but the band is a cylinder and the post is _POST_OUT wide, so the
# outboard edge meets the band 6 further in and the 45° bar ran 1.94
# BELOW the bed there. Nothing may hang under the bed plane (it would
# print in mid-air), nor dip into the hand-access cutaway, which rises
# at 45° from (WALL_OR, the bed). Checked on every vertex of the solid,
# so no sampling choice can miss it again.
_post_worst = None
for _v in clip_post(RATCHET_PIVOT_Z, CLIP_SIDE).val().Vertices():
    _need = FLOOR_Z0 + max(0.0, math.hypot(_v.X, _v.Y) - WALL_OR)
    if _post_worst is None or _v.Z - _need < _post_worst[0]:
        _post_worst = (_v.Z - _need, _v.X, _v.Y, _v.Z)
assert _post_worst[0] >= -1e-6, (
    f"A_POST_BED: a clip-post corner sits {-_post_worst[0]:.2f} under the bed "
    f"or in the hand cutaway, at ({_post_worst[1]:.2f}, {_post_worst[2]:.2f}, "
    f"{_post_worst[3]:.2f})")
