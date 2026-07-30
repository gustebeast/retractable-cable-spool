"""ratchet + brake levers — the original design's radial-engagement design, hardware-free.

Ported from src/levers.py: XZ plate levers (LEVER_T=10 thick, HANDLE_W=6) at
+X, Y-axis pivots, pull = +X; RATCHET pivot ABOVE its tooth band (rest =
ENGAGED, pull swings the pawl out), BRAKE pivot BELOW its band (rest = OFF,
pull swings the pad in); the pawl edge is the exact tooth-profile negative +
catch bump; the brake pad is the tilted-face + rectangular-TPU design solved
for handoff timing / mid-travel parallelism / pad-bottom landing.

differences:
  * the plates FLANK the +X beam (ratchet y 6..16, brake −16..−6) instead of
    riding a housing spine;
  * pivots are printed Ø4 PINS spanning fork posts on BOTH sides of each
    lever (frame.py grows the posts; the −Z→+Z print direction allows it);
  * NO hardware: return force = 95A TPU blocks compressed between each
    handle tail's +X face and a fixed abutment post (pull compresses,
    release re-seats; the ratchet's rest stop is the tooth valley itself,
    the brake's is a stop post under the wall);
  * the arms reach the separator rims through open-top WALL WINDOWS.

The original kinematic assertion suite runs at import (src/lever_kinematics.py).
TPU parts (brake pad + 2 spring blocks) print in 95A TPU — black in the
viewer per the cadkit colour convention.
"""

import math

import cadquery as cq

from src.helpers import cyl
from cadkit.joinery import hook_tenon, hook_mortise
from .separator import ratchet_star_world
from .lever_kinematics import assert_kinematics, R_ROOT
from .params import (
    FLOOR_OD, RATCHET_DEPTH, RATCHET_TEETH, RATCHET_PHASE_DEG,
    SEP_Z0, SEP_Z1, RATCHET_H, CONE_H, R_OUT_COIL, CABLE_D,
    LEVER_T, LEVER_HANDLE_W, LEVER_PIVOT_X, LEVER_TRAVEL_DEG, RATCHET_STOP_DEG,
    RATCHET_LEV_Y0, RATCHET_LEV_Y1, BRAKE_LEV_Y0, BRAKE_LEV_Y1,
    RATCHET_PIVOT_Z, BRAKE_PIVOT_Z, HANDLE_Z_BOT,
    PIN_SQ_S, PIN_SQ_LEVER_CLR, LEVER_BOSS_OD,
    PIN_PRETWIST_DEG, PIN_KEY_BASE_DEG, BRAKE_RUBBER_T,
    PAD_JOINT_CLR, BRAKE_PAD_TOP_MARGIN, BRAKE_PAD_BOT_MARGIN,
    PAD_FLANGE_H, PAD_FLANGE_T, WALL_SPLIT_Z,
)

R_RIM = FLOOR_OD / 2.0                            # 72.9 — band / tooth-tip radius
_HW   = LEVER_HANDLE_W / 2.0

# Band z-geometry (world): the separator seats at RIM_SEAT_Z, teeth first.
RATCHET_BAND_Z0 = SEP_Z0                          # 19.5
RATCHET_BAND_Z1 = SEP_Z0 + RATCHET_H              # 24.5
BRAKE_BAND_Z0   = RATCHET_BAND_Z1 + CONE_H        # 26.0 — brake band bottom
BRAKE_BAND_Z1   = SEP_Z1                          # 31.5

# Pawl block spans the tooth band; kinematics sample at its mid point.
PAWL_Z_LO, PAWL_Z_HI = RATCHET_BAND_Z0, RATCHET_BAND_Z1
PAWL_Y_MID = (RATCHET_LEV_Y0 + RATCHET_LEV_Y1) / 2.0        # 11
PAWL_Z_MID = (PAWL_Z_LO + PAWL_Z_HI) / 2.0                  # 22


def _pivot_boss(pivot_x, pivot_z, y0, y1):
    """Ø LEVER_BOSS_OD cylindrical boss along the pivot axis, spanning the
    lever plate's thickness — the handle is only 6 wide, narrower than the
    axle's pocket, so this local bulge carries the pivot."""
    return (cq.Workplane("XZ").center(pivot_x, pivot_z)
            .circle(LEVER_BOSS_OD / 2)
            .extrude(-(y1 - y0)).translate((0, y0, 0)))


def _pivot_hole(pivot_x, pivot_z, y_outer, y_inner):
    """SQUARE axle pocket straight THROUGH the lever (user redesign — one
    plain square prism, no sections): the lever prints along Y, so this is
    a vertical hole in its print and ANY clock costs nothing. Clocked to
    PIN_KEY_BASE_DEG − PIN_PRETWIST_DEG: the frame keys the straight axle
    at 45° (diamond) at both ends, so seating the lever at rest twists the
    axle's middle by the pre-twist — hold the lever pulled by that angle
    (~12°) to slide the axle in at install."""
    side = PIN_SQ_S + 2.0 * PIN_SQ_LEVER_CLR
    lo = min(y_outer, y_inner)
    return (cq.Workplane("XY").rect(side, side)
            .extrude(abs(y_inner - y_outer) + 1.0)
            .rotate((0, 0, 0), (1, 0, 0), -90)
            .translate((pivot_x, lo - 0.5, pivot_z))
            .rotate((pivot_x, 0, pivot_z), (pivot_x, 1, pivot_z),
                    PIN_KEY_BASE_DEG - PIN_PRETWIST_DEG))


# ── Ratchet lever ────────────────────────────────────────────────────────────
def _build_ratchet_lever():
    """the original design's three primitives: (1) horizontal pawl block minus the separator's
    tooth solid (inner edge = exact tooth negative) + the chord-tracking catch
    bump; (2) vertical handle through the pivot; (3) grip-end disc. Plus the
    TPU spring seat is just the arm tail's flat +X face (no socket)."""
    y0, y1 = RATCHET_LEV_Y0, RATCHET_LEV_Y1
    y_len = y1 - y0

    # 1. Pawl block — from inside the rim out to the handle's +X edge, minus
    #    the separator's solid tooth star (world position).
    h_x_hi = LEVER_PIVOT_X + _HW
    h_x_lo = 60.0                                # safely inside the valley radius
    horiz = (cq.Workplane("XY").workplane(offset=PAWL_Z_LO)
             .polyline([(h_x_lo, y0), (h_x_hi, y0), (h_x_hi, y1), (h_x_lo, y1)])
             .close().extrude(PAWL_Z_HI - PAWL_Z_LO))
    horiz = horiz.cut(ratchet_star_world())

    # Catch-face bump (the original design's parallelogram, chord-tracking back edge). One rim
    # catch face sits exactly at ts = RATCHET_PHASE_DEG (the separator is
    # phased for this); the bump rides it at the pawl's inner-y azimuth.
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

    # 2. Vertical handle through the pivot.
    vert_top = RATCHET_PIVOT_Z + _HW
    vert = (cq.Workplane("XZ")
            .polyline([(LEVER_PIVOT_X - _HW, HANDLE_Z_BOT),
                       (LEVER_PIVOT_X + _HW, HANDLE_Z_BOT),
                       (LEVER_PIVOT_X + _HW, vert_top),
                       (LEVER_PIVOT_X - _HW, vert_top)]).close()
            .extrude(-y_len).translate((0, y0, 0)))

    # 3. Grip-end disc.
    bottom = (cq.Workplane("XZ").center(LEVER_PIVOT_X, HANDLE_Z_BOT)
              .circle(_HW).extrude(-y_len).translate((0, y0, 0)))

    part = horiz.union(vert).union(bottom)
    part = part.union(_pivot_boss(LEVER_PIVOT_X, RATCHET_PIVOT_Z, y0, y1))
    # pin inserts from the +y (outer) side → pocket opens at y1
    part = part.cut(_pivot_hole(LEVER_PIVOT_X, RATCHET_PIVOT_Z, y1, y0))
    return part


# ── Brake lever + TPU pad — CONTACT-frame design (user's spec) ───────────────
# The brake is validated at FIRST CONTACT only, and the arm + pad are BUILT
# in that pose, where they read as ONE STRAIGHT FLAT PRISM: the arm is a
# plain box ending in a FLAT face, the TPU pad is the straight constant-
# thickness slab between that face and the band. The only non-orthogonal
# feature is the face's small PLAN bevel (~8.7°) keeping it TANGENT to the
# round band at the pad's mid azimuth — dead flush at contact (a face ⊥ the
# arm would gap 1.6 across the pad's width). Both are back-rotated about the
# pivot by the contact angle into the REST pose — the ARM angles instead of
# carrying an angled cut. Past contact is the COMPRESSION regime (TPU
# squish, reported not asserted). Guards: the BOT margin keeps the further
# full-pull drop ≥1 clear of the ratchet tooth box; a swept-corner assertion
# keeps the raised resting pad out of the spool chamber's cable keep-out.
PAD_Y_MID = (BRAKE_LEV_Y0 + BRAKE_LEV_Y1) / 2.0   # −11


def _ratchet_pawl_clear_angle():
    """Pull angle at which the pawl just clears the tooth tips (original math)."""
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
BRAKE_CONTACT_DELAY_DEG = 3.0                     # dead zone after pawl-clear (the original design)
BRAKE_CONTACT_DEG = RATCHET_PAWL_CLEAR_DEG + BRAKE_CONTACT_DELAY_DEG
# BRAKE_PAD_TOP_MARGIN / BOT_MARGIN / PAD_JOINT_CLR moved to params.py —
# the BAND BUDGET derives from them now (pad face = the hook's quality
# width; RIM_H grew to fit). Imported below with the band constants.


# ── Contact-frame geometry (closed form — replaces the tilt solver) ──────────
_AC_RAD     = math.radians(BRAKE_CONTACT_DEG)
_X_BAND_MID = math.sqrt(R_RIM ** 2 - PAD_Y_MID ** 2)      # 72.07 — band x at y −11
_PLAN_TAN   = abs(PAD_Y_MID) / _X_BAND_MID                # tangent-plane dx/dy (~0.153)
_FACE_OFF   = BRAKE_RUBBER_T * math.sqrt(1.0 + _PLAN_TAN ** 2)  # plane offset, in x
Z_TOP_C     = BRAKE_BAND_Z1 - BRAKE_PAD_TOP_MARGIN        # pad SLAB z-band AT CONTACT
Z_BOT_C     = BRAKE_BAND_Z0 + BRAKE_PAD_BOT_MARGIN
BRAKE_PAD_H = Z_TOP_C - Z_BOT_C                           # 3.1 — the band-contact slab
# L-SECTION pad (user's design): the slab above contacts the BAND; a taller
# back FLANGE carries the hook joint at its quality width (6.6). The flange
# hangs TOP-ALIGNED with the slab (opening downward — centered, its rest
# swing would poke past the brake window's split-plane cap) and nests into
# the arm's recessed end, so it sits BEHIND the old face radius — clear of
# the coil keep-out above and the cone/teeth below (both are r ≤ band).
PAD_FLANGE_TOP = Z_TOP_C
PAD_FLANGE_BOT = Z_TOP_C - PAD_FLANGE_H
_FL_OFF = PAD_FLANGE_T * math.sqrt(1.0 + _PLAN_TAN ** 2)  # plane offset, in x


def _x_band_plane(y):
    """CONTACT pose: the band's tangent plane at the pad's mid azimuth."""
    return _X_BAND_MID + (y - PAD_Y_MID) * _PLAN_TAN


def _x_face(y):
    """CONTACT pose: the pad SLAB's back plane — tangent plane + slab
    thickness (= the flange's FRONT plane)."""
    return _x_band_plane(y) + _FACE_OFF


def _x_face2(y):
    """CONTACT pose: the ARM's flat end face — recessed one flange depth
    behind the slab's back plane; the pad flange fills the gap and this is
    the JOINT face."""
    return _x_face(y) + _FL_OFF


def _rest_xz(x, z):
    """Map a CONTACT-pose point to the REST pose (+contact about the pivot)."""
    dx, dz = x - LEVER_PIVOT_X, z - BRAKE_PIVOT_Z
    ca, sa = math.cos(_AC_RAD), math.sin(_AC_RAD)
    return (LEVER_PIVOT_X + dx * ca + dz * sa,
            BRAKE_PIVOT_Z - dx * sa + dz * ca)


def _to_rest(wp):
    """Rotate a CONTACT-frame solid into the REST pose."""
    return wp.rotate((LEVER_PIVOT_X, 0, BRAKE_PIVOT_Z),
                     (LEVER_PIVOT_X, 1, BRAKE_PIVOT_Z), BRAKE_CONTACT_DEG)


# rest-frame pad extremes (reference) — the arm face is the outermost plane
PAD_Z_HI = _rest_xz(_x_face2(BRAKE_LEV_Y1), Z_TOP_C)[1]
PAD_Z_LO = _rest_xz(_x_face2(BRAKE_LEV_Y0), PAD_FLANGE_BOT)[1]

# A_PAD_WINDOW: REST is the swing's highest pose — the pad/arm top must
# stay under the brake window's cap (wall_top's underside at the split
# plane), which DROPPED when the rim shrank back to 12.4
assert PAD_Z_HI <= WALL_SPLIT_Z - 0.5 + 1e-9, (
    f"A_PAD_WINDOW: pad/arm rest top {PAD_Z_HI:.2f} is within 0.5 of the "
    f"brake window cap at the split plane {WALL_SPLIT_Z:.2f} — lower "
    f"PAD_FLANGE_TOP or grow BRAKE_PAD_TOP_MARGIN")


# ── Pad ↔ arm SLIDE JOINT (glue-free, user's design) ─────────────────────────
# A HOOKED TENON rail on the TPU pad's back slides into a mortise channel in
# the arm's flat face, along the face's in-plane horizontal direction û (Y
# tilted by the 8.7° plan bevel). Direction is picked by the BRAKING load:
# the brake acts while the spool RETRACTS — CW viewed from +Z (= the user's
# CCW seen from below) — and the band's drag on the pad at this azimuth is
# then −û (toward the arm's OUTER end). So the STOP WALL sits at the outer
# end and the channel opens through the arm's INNER face: braking presses
# the tenon INTO the stop — no glue. (The fork column 1 mm outside and the
# beam flank 1 mm inside also cage the pad once the lever is mounted.)
# CROSS-SECTION: cadkit's HOOK — the single-flank dovetail for EDGE-BOUNDED
# install-z sites (this arm was its first case; the geometry now lives in
# the library, and slide_joint(install='z', bounded=True) at this face's
# 4.15 width picks it — a symmetric dovetail can't fit printable shoulders
# between the free edges). Width = the FULL 4.15 face, edges landing on the
# arm/pad top and bottom faces; the library budgets one-nozzle walls, a
# one-nozzle lip in the notch (hooks −n pull-off), closed floor/roof (±z).
# PRINTS: the channel runs ≈ the lever's −Y→+Y build axis → every wall is
# near-vertical and the bed-side stop wall means NO bridge; the pad prints
# STANDING on its INNER end face (the rail is trimmed flush there, so it
# rises straight off the bed — overhang-free; the curved band face is a
# vertical surface, so no X-axis build needed).
# PAD_JOINT_CLR lives in params now (the band budget derives from it); the
# pad face BRAKE_PAD_H = the hook's QUALITY width → every joint segment on
# both halves prints at 1.6 (was 0.8-1.0, user-caught as tear-prone TPU)
_B_DEG   = math.degrees(math.atan(_PLAN_TAN))     # face plan-bevel (~8.7°)
_U_LEN   = (BRAKE_LEV_Y1 - BRAKE_LEV_Y0) * math.sqrt(1.0 + _PLAN_TAN ** 2)
_JT_STOP = 1.6                                    # stop wall at the outer end (along û)


def _pad_joint(tenon):
    """CONTACT-frame solid of the pad↔arm slide joint: tenon=True → the
    library hook rail (union into the TPU pad, root sunk 0.5); False → the
    mortise CUTTER (dilated, stop wall left at the outer end, mouth open
    through the arm's inner face). Library frame (profile in plan, prism
    along +Z, hook toward +Y) mapped: prism → û with the hook UP and the
    head into the arm — rotate to lay the prism on −Y, offset along +Y,
    then rotate the plan by −_B_DEG so y-offsets become û-offsets."""
    L = _U_LEN - _JT_STOP + (0.3 if tenon else 1.0)   # tenon: trimmed flush at
                                                      # the inner face; cutter: open
    if tenon:
        j = hook_tenon(PAD_FLANGE_H, L, clearance=PAD_JOINT_CLR, root=0.5)
    else:
        j = hook_mortise(PAD_FLANGE_H, L, clearance=PAD_JOINT_CLR, drop=1.0)
    return (j.rotate((0, 0, 0), (1, 0, 0), 90.0)  # prism → −Y, hook → +z (up)
            .translate((0.0, _JT_STOP + L, 0.0))  # spans y ∈ [_JT_STOP, stop+L]
            .rotate((0, 0, 0), (0, 0, 1), -_B_DEG)    # y-offsets → û, +X → n̂
            .translate((_x_face2(BRAKE_LEV_Y0), BRAKE_LEV_Y0,
                        (PAD_FLANGE_BOT + PAD_FLANGE_TOP) / 2.0)))


def brake_pad_rect_corners(h=None):
    """(lever_face_corners, band_face_corners) of the TPU pad, REST pose.
    `h` (DEMO only): a FORCED pad height, top edge kept — lets the viewer
    show an oversized pad without touching the assertion-gated geometry."""
    z_bot = Z_TOP_C - h if h is not None else Z_BOT_C
    zf_bot = z_bot if h is not None else PAD_FLANGE_BOT
    lever, band = [], []
    for y, zs, zl in ((BRAKE_LEV_Y0, z_bot, zf_bot),
                      (BRAKE_LEV_Y1, z_bot, zf_bot),
                      (BRAKE_LEV_Y1, Z_TOP_C, Z_TOP_C),
                      (BRAKE_LEV_Y0, Z_TOP_C, Z_TOP_C)):
        # lever side = the FLANGE's outer plane (the L's back leg)
        xf, zf = _rest_xz(_x_face2(y), zl)
        # band corners sit ON the band cylinder (the face is cut concave)
        xb, zb = _rest_xz(math.sqrt(R_RIM ** 2 - y * y), zs)
        lever.append((xf, y, zf))
        band.append((xb, y, zb))
    return lever, band


def _build_brake_lever():
    """Three primitives: (1) the STRAIGHT pad arm — a contact-frame box from
    the flat (plan-beveled) face back through the pivot, back-rotated to
    rest and trimmed flush with the handle's +X face; (2) vertical handle
    through the pivot; (3) grip-end disc."""
    y0, y1 = BRAKE_LEV_Y0, BRAKE_LEV_Y1
    y_len = y1 - y0
    h_x_hi = LEVER_PIVOT_X + _HW
    # arm end face = _x_face2 (recessed one flange depth — the pad's L
    # flange fills the gap) and the arm spans the FLANGE height, so the
    # mortise host face is the full 6.6 between free edges
    horiz = _to_rest(
        cq.Workplane("XY").workplane(offset=PAD_FLANGE_BOT)
        .polyline([(_x_face2(y0), y0), (h_x_hi + 2.0, y0),
                   (h_x_hi + 2.0, y1), (_x_face2(y1), y1)])
        .close().extrude(PAD_FLANGE_TOP - PAD_FLANGE_BOT))
    # the back-rotated box's far end sticks past the handle face — trim it
    # flush (the cut is buried in the handle join, invisible at contact)
    horiz = horiz.cut(
        cq.Workplane("XY").workplane(offset=PAD_FLANGE_BOT - 3.0)
        .polyline([(h_x_hi, y0 - 1), (h_x_hi + 9.0, y0 - 1),
                   (h_x_hi + 9.0, y1 + 1), (h_x_hi, y1 + 1)])
        .close().extrude(PAD_FLANGE_H + 8.0))

    vert_top = _rest_xz(LEVER_PIVOT_X - _HW, Z_TOP_C)[1]   # arm's rest top at the handle
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
    # pad-joint mortise channel (built in the contact frame, rotated to
    # rest, and cut from the WHOLE lever — AFTER every union: the L-flange
    # joint reaches low enough that the Ø12 pivot boss crosses the channel
    # band, and a boss unioned after this cut refills the cavity
    # (5.3 mm³ rail collision, caught by the scan)
    part = part.cut(_to_rest(_pad_joint(tenon=False)))
    # pin inserts from the −y (outer) side → pocket opens at y0
    part = part.cut(_pivot_hole(LEVER_PIVOT_X, BRAKE_PIVOT_Z, y0, y1))
    return part


def _build_brake_pad(h=None):
    """The 95A TPU pad (contact frame): a straight slab off the arm's flat
    face whose band side is CUT CONCAVE by the band cylinder itself — it
    mates the rim's curvature exactly at contact (no sagitta gap), then
    back-rotates to rest. NO GLUE: a hooked tenon rail on its back slides
    into the arm's mortise channel (see _pad_joint) — braking drag presses
    it into the stop. Prints STANDING on its inner (+y) end face."""
    y0, y1 = BRAKE_LEV_Y0, BRAKE_LEV_Y1
    z_bot = Z_TOP_C - h if h is not None else Z_BOT_C
    slab = (cq.Workplane("XY").workplane(offset=z_bot)
            .polyline([(_x_band_plane(y0) - 0.5, y0), (_x_face(y0), y0),
                       (_x_face(y1), y1), (_x_band_plane(y1) - 0.5, y1)])
            .close().extrude(Z_TOP_C - z_bot))
    # concave band face: in the contact pose the pad is world-aligned with
    # the band, so the actual band cylinder is the cutter
    slab = slab.cut(cyl(2.0 * R_RIM, (Z_TOP_C - z_bot) + 2.0, z=z_bot - 1.0))
    if h is not None:
        return _to_rest(slab)
    # the real pad: L-FLANGE + rail. The back flange — the joint-carrying
    # leg of the L: slab back plane → the recessed arm face, spanning the
    # flange height
    slab = slab.union(
        cq.Workplane("XY").workplane(offset=PAD_FLANGE_BOT)
        .polyline([(_x_face(y0), y0), (_x_face2(y0), y0),
                   (_x_face2(y1), y1), (_x_face(y1), y1)])
        .close().extrude(PAD_FLANGE_H))
    pad = _to_rest(slab)
    # the HANDLE prism and the Ø12 pivot boss live in the REST frame and
    # lean across the contact-frame face planes — the deeper, taller
    # L-flange now reaches them at its inner-top / inner-bottom corners
    # (0.23 mm³ of parasitic sliver, scan-caught). Shave the pad BODY clear
    # of both; the RAIL is exempt — it legitimately rides inside the
    # mortise cavity, which tunnels through the handle. Guard clearance
    # covers the ENTRY SLIDE too: û carries a sin(8.7°) ≈ 0.15 x-component,
    # so a corner shaved flush at seat drifts up to ~0.33 back toward the
    # handle plane during the first ~2 mm of travel (probe-caught).
    _G = 0.45
    gx0 = LEVER_PIVOT_X - _HW - _G
    pad = pad.cut(
        cq.Workplane("XY").workplane(offset=PAD_FLANGE_BOT - 10.0)
        .polyline([(gx0, y0 - 1.0), (gx0 + 30.0, y0 - 1.0),
                   (gx0 + 30.0, y1 + 1.0), (gx0, y1 + 1.0)])
        .close().extrude(40.0))
    pad = pad.cut(
        cq.Workplane("XZ").center(LEVER_PIVOT_X, BRAKE_PIVOT_Z)
        .circle(LEVER_BOSS_OD / 2.0 + _G)
        .extrude(-(y1 - y0 + 2.0)).translate((0, y0 - 1.0, 0)))
    rail = _pad_joint(tenon=True)
    # trim the rail's overshoot flush with the pad's inner end face —
    # the print's bed face (the û-prism end is 8.7° off the y-plane)
    rail = rail.cut(
        cq.Workplane("XY").workplane(offset=PAD_FLANGE_BOT - 1.0)
        .center(_x_face2(y1) + 1.0, y1 + 5.0).rect(20.0, 10.0)
        .extrude(PAD_FLANGE_H + 2.0))
    return pad.union(_to_rest(rail))


def brake_pad_tall_demo(h):
    """DEMO ONLY (viewer poses): the pad rebuilt at a forced height on the
    same mount plane, top edge kept — visualizes WHY the solver caps the
    height (the bottom strip sweeps below the band into the cone/teeth).
    The exported/printable pad stays the solved `brake_pad_tpu`."""
    return _build_brake_pad(h)


# ── Kinematic assertions — fire at import (contact-frame corners) ────────────
_PAD_BAND_CORNERS = brake_pad_rect_corners()[1]
# the arm and the pad flange both bottom out at PAD_FLANGE_BOT now — pass
# their bottom-front corners into the teeth-box sweep (they are RADIALLY
# outside the tooth annulus, which the radial-aware check credits)
_ARM_BOTTOM_CORNERS = []
_FLANGE_BOTTOM_CORNERS = []
for _y in (BRAKE_LEV_Y0, BRAKE_LEV_Y1):
    _xr, _zr = _rest_xz(_x_face2(_y), PAD_FLANGE_BOT)
    _ARM_BOTTOM_CORNERS.append((_xr, _y, _zr))
    _xr, _zr = _rest_xz(_x_face(_y), PAD_FLANGE_BOT)
    _FLANGE_BOTTOM_CORNERS.append((_xr, _y, _zr))

KIN = assert_kinematics(
    ratchet_pivot_x=LEVER_PIVOT_X, ratchet_pivot_z=RATCHET_PIVOT_Z,
    ratchet_stop_deg=RATCHET_STOP_DEG,
    pawl_y_mid=PAWL_Y_MID, pawl_z_mid=PAWL_Z_MID,
    brake_pivot_x=LEVER_PIVOT_X, brake_pivot_z=BRAKE_PIVOT_Z,
    brake_travel_deg=LEVER_TRAVEL_DEG,
    brake_contact_deg=BRAKE_CONTACT_DEG,
    brake_band_z0=BRAKE_BAND_Z0, brake_band_z1=BRAKE_BAND_Z1,
    # spool-chamber keep-out: outermost design wrap + 0.2 sweep margin,
    # starting at the separator top
    chamber_r_min=R_OUT_COIL + CABLE_D / 2.0 + 0.2, chamber_z0=SEP_Z1,
    pad_band_corners=_PAD_BAND_CORNERS,
    arm_bottom_corners=_ARM_BOTTOM_CORNERS,
    flange_bottom_corners=_FLANGE_BOTTOM_CORNERS,
    ratchet_band_z0=RATCHET_BAND_Z0, ratchet_band_z1=RATCHET_BAND_Z1,
)


ratchet_lever = _build_ratchet_lever()
brake_lever = _build_brake_lever()
brake_pad_tpu = _build_brake_pad()

# TPU TORSION-BAR pivot pin (95A, prints standing) — ONE stepped pin per
# lever, inserted from the OUTER post's face. Tip-first profile: round TIP
# (inner-post bearing) → LEVER hex (keys the boss pocket) → NECK (the torsion
# spring) → POST hex (keyed in the clocked outer-post keyway) → flange HEAD.
from .params import (  # noqa: E402
    LEVER_PIN_L, PIN_TIP_END_Y,
)


def _build_axle_pin():
    """The SQUARE torsion axle as PRINTED (user redesign — the stepped hex
    pin's corners printed too soft to grip): ONE plain PIN_SQ_S prism,
    standing on end — a square column, trivially printable. No steps, no
    flanges: the frame's diamond holes and the lever's clocked pocket do
    all the keying, the blind-bore floor registers the depth, and the free
    spans across the two side gaps are the torsion springs (working in
    parallel)."""
    return (cq.Workplane("XY").rect(PIN_SQ_S, PIN_SQ_S)
            .extrude(LEVER_PIN_L))


lever_pin = _build_axle_pin()                                 # the print


def lever_pin_in_place(pivot_z, y_sign, pull_deg=0.0):
    """The axle placed as INSTALLED, showing the REAL WORKING TWIST (the
    viewer solid is twist-extruded — a rigid prism can't be keyed 45° in
    the frame AND 45−PRETWIST in the lever at once): frame-keyed spans at
    45°, the lever span at the pocket's clock (−PRETWIST, − pull when the
    lever is posed pulled), twisting across the two free side gaps — the
    torsion springs, made visible."""
    from .params import LEVER_Y_IN, BEAM_SIZE, POST_OUT_T, PIN_GRIP_L
    k = 1.0 if y_sign > 0 else -1.0               # local CCW → world +Y clock sign
    mid = PIN_KEY_BASE_DEG - PIN_PRETWIST_DEG - pull_deg
    base = PIN_KEY_BASE_DEG
    seg_in = BEAM_SIZE / 2.0 - PIN_TIP_END_Y      # keyed in the beam flank
    gap = LEVER_Y_IN - BEAM_SIZE / 2.0            # the free torsion spans
    seg_out = POST_OUT_T + PIN_GRIP_L             # column + grip stub

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
    return (p.rotate((0, 0, 0), (1, 0, 0), -y_sign * 90)   # +Z → ±Y, inner end first
            .translate((LEVER_PIVOT_X, y_sign * PIN_TIP_END_Y, pivot_z)))
