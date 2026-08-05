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
from .lever_kinematics import assert_kinematics, R_ROOT
from .params import (
    NOZZLE,
    RIM_OD, RATCHET_DEPTH, RATCHET_TEETH, RATCHET_PHASE_DEG,
    SEP_Z0, SEP_Z1, BRAKE_H, R_OUT_COIL, CABLE_D,
    LEVER_T, LEVER_HANDLE_W, LEVER_PIVOT_X, LEVER_TRAVEL_DEG, RATCHET_STOP_DEG,
    RATCHET_LEV_Y0, RATCHET_LEV_Y1, BRAKE_LEV_Y0, BRAKE_LEV_Y1,
    RATCHET_PIVOT_Z, BRAKE_PIVOT_Z, HANDLE_Z_BOT,
    PIN_SQ_S, PIN_SQ_LEVER_CLR, LEVER_BOSS_OD,
    PIN_PRETWIST_DEG, PIN_KEY_BASE_DEG, BRAKE_RUBBER_T,
    PAD_JOINT_CLR, PAD_SPEC, BRAKE_PAD_TOP_MARGIN, BRAKE_PAD_BOT_MARGIN,
    PAD_FLANGE_H, PAD_FLANGE_T,
    LEVER_PIN_L, PIN_TIP_END_Y, LEVER_Y_IN, BEAM_SIZE, POST_OUT_T, PIN_GRIP_L,
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


def _pivot_hole(pivot_x, pivot_z, y_outer, y_inner):
    """SQUARE axle pocket straight THROUGH the lever (v2): prints along Y →
    a vertical hole, any clock free. Clocked BASE − PRETWIST: the frame
    keys the straight axle at 45° both ends, so seating the lever at rest
    twists the axle's middle by the pre-twist."""
    side = PIN_SQ_S + 2.0 * PIN_SQ_LEVER_CLR
    lo = min(y_outer, y_inner)
    return (cq.Workplane("XY").rect(side, side)
            .extrude(abs(y_inner - y_outer) + 1.0)
            .rotate((0, 0, 0), (1, 0, 0), -90)
            .translate((pivot_x, lo - 0.5, pivot_z))
            .rotate((pivot_x, 0, pivot_z), (pivot_x, 1, pivot_z),
                    PIN_KEY_BASE_DEG - PIN_PRETWIST_DEG))


# ── Ratchet lever (v2 verbatim, v3 anchors) ──────────────────────────────────
def _build_ratchet_lever():
    """(1) horizontal pawl block minus the separator's tooth solid (exact
    negative) + the chord-tracking catch bump; (2) vertical handle through
    the pivot; (3) grip-end disc."""
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

    vert_top = RATCHET_PIVOT_Z + _HW
    vert = (cq.Workplane("XZ")
            .polyline([(LEVER_PIVOT_X - _HW, HANDLE_Z_BOT),
                       (LEVER_PIVOT_X + _HW, HANDLE_Z_BOT),
                       (LEVER_PIVOT_X + _HW, vert_top),
                       (LEVER_PIVOT_X - _HW, vert_top)]).close()
            .extrude(-y_len).translate((0, y0, 0)))
    bottom = (cq.Workplane("XZ").center(LEVER_PIVOT_X, HANDLE_Z_BOT)
              .circle(_HW).extrude(-y_len).translate((0, y0, 0)))

    part = horiz.union(vert).union(bottom)
    part = part.union(_pivot_boss(LEVER_PIVOT_X, RATCHET_PIVOT_Z, y0, y1))
    # pin inserts from the +y (outer) side → pocket opens at y1
    part = part.cut(_pivot_hole(LEVER_PIVOT_X, RATCHET_PIVOT_Z, y1, y0))
    return part


# ── Brake lever + TPU pad — v2's CONTACT-frame design ────────────────────────
PAD_Y_MID = (BRAKE_LEV_Y0 + BRAKE_LEV_Y1) / 2.0   # −12


def _ratchet_pawl_clear_angle():
    """Pull angle at which the pawl just clears the tooth tips (v2 math)."""
    x_pawl_rest = math.sqrt(R_ROOT ** 2 - PAWL_Y_MID ** 2)
    target_x    = math.sqrt(R_RIM ** 2 - PAWL_Y_MID ** 2)
    dx = x_pawl_rest - LEVER_PIVOT_X
    dz = PAWL_Z_MID - RATCHET_PIVOT_Z
    A, B = dx, -dz
    rhs = target_x - LEVER_PIVOT_X
    R = math.hypot(A, B)
    phi = math.degrees(math.atan2(B, A))
    ac = math.degrees(math.acos(max(-1.0, min(1.0, rhs / R))))
    return min(s for s in (phi - ac, phi + ac) if 0 < s < LEVER_TRAVEL_DEG)


RATCHET_PAWL_CLEAR_DEG  = _ratchet_pawl_clear_angle()
BRAKE_CONTACT_DELAY_DEG = 3.0                     # dead zone after pawl-clear
BRAKE_CONTACT_DEG = RATCHET_PAWL_CLEAR_DEG + BRAKE_CONTACT_DELAY_DEG

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
_JT_STOP = 2 * NOZZLE
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
    joint plane, back-rotated to rest; (2) vertical handle; (3) grip disc."""
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
    vert = (cq.Workplane("XZ")
            .polyline([(LEVER_PIVOT_X - _HW, HANDLE_Z_BOT),
                       (LEVER_PIVOT_X + _HW, HANDLE_Z_BOT),
                       (LEVER_PIVOT_X + _HW, vert_top),
                       (LEVER_PIVOT_X - _HW, vert_top)]).close()
            .extrude(-y_len).translate((0, y0, 0)))
    bottom = (cq.Workplane("XZ").center(LEVER_PIVOT_X, HANDLE_Z_BOT)
              .circle(_HW).extrude(-y_len).translate((0, y0, 0)))

    part = horiz.union(vert).union(bottom)
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


# ── Kinematic assertions — fire at import ────────────────────────────────────
_PAD_BAND_CORNERS = brake_pad_rect_corners()[1]

KIN = assert_kinematics(
    ratchet_pivot_x=LEVER_PIVOT_X, ratchet_pivot_z=RATCHET_PIVOT_Z,
    ratchet_stop_deg=RATCHET_STOP_DEG,
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
        for _i in range(-16, 2):
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


def _build_axle_pin():
    """The SQUARE torsion axle as PRINTED (v2): one plain PIN_SQ_S prism,
    standing on end — the frame's diamond holes and the lever's clocked
    pocket do all the keying; the free spans across the side gaps are the
    torsion springs."""
    return (cq.Workplane("XY").rect(PIN_SQ_S, PIN_SQ_S)
            .extrude(LEVER_PIN_L))


lever_pin = _build_axle_pin()                                 # the print


def lever_pin_in_place(pivot_z, y_sign, pull_deg=0.0):
    """The axle placed as INSTALLED, twist-extruded to show the real
    working twist (v2 — a rigid prism can't be keyed 45° in the frame AND
    45−PRETWIST in the lever at once)."""
    k = 1.0 if y_sign > 0 else -1.0
    mid = PIN_KEY_BASE_DEG - PIN_PRETWIST_DEG - pull_deg
    base = PIN_KEY_BASE_DEG
    seg_in = BEAM_SIZE / 2.0 - PIN_TIP_END_Y
    gap = LEVER_Y_IN - BEAM_SIZE / 2.0
    seg_out = POST_OUT_T + PIN_GRIP_L

    def seg(z0, length, a0, twist):
        wp = cq.Workplane("XY").rect(PIN_SQ_S, PIN_SQ_S)
        s = wp.twistExtrude(length, k * twist) if twist else wp.extrude(length)
        return s.rotate((0, 0, 0), (0, 0, 1), k * a0).translate((0, 0, z0))

    z1 = seg_in
    z2 = z1 + gap
    z3 = z2 + LEVER_T
    z4 = z3 + gap
    p = (seg(0.0, seg_in, base, 0.0)
         .union(seg(z1, gap, base, mid - base))
         .union(seg(z2, LEVER_T, mid, 0.0))
         .union(seg(z3, gap, mid, base - mid))
         .union(seg(z4, seg_out, base, 0.0)))
    return (p.rotate((0, 0, 0), (1, 0, 0), -y_sign * 90)
            .translate((LEVER_PIVOT_X, y_sign * PIN_TIP_END_Y, pivot_z)))
