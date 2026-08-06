"""FRAME — split frame_top / frame_bottom, v2's architecture ported to v3.

frame_top — plus spider carrying the 608 IN its own thickness (recessed
    onto a 45° FUNNEL cone seat — overhang-free replacement for the old
    bridge-ring lip, params block), the spring-strip TENSION-TRAP anchor
    wall under the +X arm (trapezoid window: the strip's T-head enters
    flat-on at the tall +y end, spring tension parks it in the −y bay,
    too short to escape — params block), and the arc-joint MORTISE
    channels in its underside. PRINTS +z→−z (top face on the bed — the
    anchor wall rises rooted on its rib band): the arc cavities open at
    the print TOP, so they close with plain floors — no bridges — and
    the funnel seat prints as a 45° narrowing taper.

frame_bottom — NO bottom plus anymore (user's call, #815): the WHOLE
    WALL BAND is FUSED in (bed → the lid-top plane — wall_top, the lock
    strips and the beam T channels are all retired, user's call #864),
    closed by a sparse SPOKED FLOOR (1.6 plate/spokes — the coil
    chamber's bottom). Four beams rise from the bed to the top plus's
    underside, carrying the arc TENONS on their tops. The v2 LEVER
    MOUNT simplified to match: the fork COLUMNS with v2's arch struts,
    gussets, thrust-boss rings and diamond pin bores (feet wedge-cut
    for hand access). Prints −Z→+Z (floor plate on the bed).

INSTALL: frame_top rotates straight onto frame_bottom — z-mate through
the open quadrants, rotate CW (viewed +z) to the angular stops. The
chirality is load-derived (user's audit): the parked spring loop
(strip anchor CW on frame_top, ratchet pawl CCW on frame_bottom),
pull-out and retraction brake drag all press frame_top CW = INTO the
stops; uninstall (CCW) is a torque no sustained load produces.
Nothing slides anymore.
"""

import math

import cadquery as cq

from cadkit.joinery import PrintSpec, joint
from cadkit.contact import contact_ring
from cadkit.holes import teardrop_hole

from .helpers import cone_solid, cyl, heal
from .levers import (BRAKE_SWEPT_TOP, BRAKE_STOP_ZC,
                     RSTOP_X0, RSTOP_X1, RSTOP_ZT0, RSTOP_SLOPE,
                     RSTOP_Y_ROOT, RSTOP_Y_TIP, RSTOP_ZBOT,
                     GUARD_X0, GUARD_PEAK_Z)
from .mount import mount_channel_cuts
from .wall import wall_band
from .params import (
    NOZZLE,
    FRAME_RIB, FRAME_R_OUT, FRAME_Z0, FRAME_Z1,
    ANCH_IR, ANCH_OR, ANCH_T, ANCH_Z0, ANCH_A_NEG, ANCH_A_POS,
    ANCH_WIN_W, ANCH_WIN_Y0, ANCH_TIP_Z, ANCH_WIN_Z1, ANCH_HOLD_Z0,
    BRG_BORE, BRG_LIP_ID, BRG_BOSS_OD,
    BRG_SEAT_CONE_Z0, BRG_SEAT_CONE_Z1,
    WALL_IR, WALL_OR, WALL_SPLIT_Z, WALL_Z1,
    BEAM_IR, BEAM_SIZE, BEAM_Z0, BEAM_Z1,
    FLOOR_Z0, FLOOR_Z1, FLOOR_SPOKE_N, FLOOR_SPOKE_W,
    FLOOR_HUB_R, FLOOR_RING_RC,
    SLEEVE_OD, SLEEVE_T, SLEEVE_FLARE, SLEEVE_Z1, SLEEVE_TOP_CH,
    STUB_D, STUB_BOSS_D, STUB_BOSS_Z1, STUB_Z1, STUB_FLARE, STUB_TIP_CH,
    JOINT_SPEC, JOINT_CLR, JOINT_BACK_CLR,
    FRAMEJ_W, FRAMEJ_R, FRAMEJ_TEN_ARC,
    TOP_JOINT_SEAT_CLR, TOP_ENTRY_OVER,
    LEVER_PIVOT_X, RATCHET_PIVOT_Z, BRAKE_PIVOT_Z,
    RATCHET_LEV_Y1, LEVER_SIDE_CLR, LEVER_BOSS_OD, BRAKE_LEV_Y1,
    BRAKE_STOP_TOP_T, GUARD_T,
    CABLE_EXIT_ANGLE_DEG, HORN_W, HORN_Z1,
    PIN_SQ_S, PIN_SQ_FRAME_CLR, PIN_KEY_BASE_DEG,
    POST_OUT_T, PIN_TIP_END_Y, BOSS_BORE_ID,
    MOUNT_RING_R, MOUNT_TEN_W,
)

# ── frame_top ↔ frame_bottom arc joints — cadkit's MUSHROOM arc, now
# TWO-SIDED (user: v2 halved this joint to one flare for the wall
# T-channel access; the T is retired, so the second flare returns — a
# stop sign with its top chopped off). The hosts print in OPPOSITE
# directions, so the flat top needs no roof geometry: frame_top's
# cavities open at its print top and close with plain floors, and the
# mushroom's z-faces run the depth-face clearance (0.30) natively.
# SITED via cadkit's ring↔arm CROSSING — ONE shared arrangement with the
# mount's rings (user: share the code; the two joints differ only in
# profile — flat-top here, full stop sign there): tenon flush at the
# beam's entry face spanning FRAMEJ_TEN_ARC = half the crossing (the
# 50% rule now applies here too, user's call), cavity + seat + entry
# overshoot, and the stop wall is simply the solid uncut half of the
# beam. hand="ccw" — the CHIRALITY AUDIT (user's worry: rotational
# joints working undone): parked with the ratchet holding, the spring's
# torque loop reacts CW on frame_top (strip anchor at +X pulled −Y) and
# CCW on frame_bottom (pawl holding the separator) — that relative
# twist crosses THIS joint, so frame_top must seat rotating CW (viewed
# +z) for the parked spring, pull-out and brake drag to all PRESS the
# stops. Uninstall = frame_top CCW, a torque no sustained load makes. ────────
_DOWN = PrintSpec(nozzle=NOZZLE, material="PETG-GF", facing="down")
FRAME_J = joint(FRAMEJ_W, None, tenon=JOINT_SPEC, mortise=_DOWN,
                install="-x")
FRAME_X = FRAME_J.crossing(FRAMEJ_R, BEAM_SIZE, FRAMEJ_TEN_ARC,
                           seat=TOP_JOINT_SEAT_CLR, over=TOP_ENTRY_OVER,
                           hand="ccw")

assert ((BEAM_IR + BEAM_SIZE) - (FRAMEJ_R + FRAMEJ_W / 2.0 + JOINT_CLR)
        >= 2 * NOZZLE - 1e-9), \
    "arc-joint cavity's outer wall under 1.6 at the flush arm end"
assert (FRAME_RIB - (FRAME_J.height + JOINT_BACK_CLR)
        >= 2 * NOZZLE - 1e-9), \
    "arc-joint cavity ceiling under 1.6 in the top plus"
# the mount ring's cavities sit radially INBOARD of the arc-joint
# cavities — the web between them must hold tier
assert ((FRAMEJ_R - FRAMEJ_W / 2.0 - JOINT_CLR)
        - (MOUNT_RING_R + MOUNT_TEN_W / 2.0 + JOINT_CLR)
        >= 2 * NOZZLE - 1e-9), \
    "mount ring cavities reach the frame arc-joint radius"

_R_OUT   = FRAME_R_OUT
_BEAM_RC = BEAM_IR + BEAM_SIZE / 2.0


def _plus(z0, z1, r_out=_R_OUT):
    """Plus-shaped spider: two crossed FRAME_RIB-wide bars spanning ±r_out."""
    h = z1 - z0
    bar_x = (cq.Workplane("XY").workplane(offset=z0)
             .rect(2 * r_out, FRAME_RIB).extrude(h))
    bar_y = (cq.Workplane("XY").workplane(offset=z0)
             .rect(FRAME_RIB, 2 * r_out).extrude(h))
    return bar_x.union(bar_y)


def _beam(angle_deg):
    return (cq.Workplane("XY").workplane(offset=BEAM_Z0)
            .center(_BEAM_RC, 0)
            .rect(BEAM_SIZE, BEAM_SIZE).extrude(BEAM_Z1 - BEAM_Z0)
            .rotate((0, 0, 0), (0, 0, 1), angle_deg))


def _arc_tenon(site_deg):
    """Arc tenon on a beam top (the cadkit crossing, root sunk 1.0): flush
    at the beam's CW entry face (hand="ccw"), spanning half the crossing
    inboard — the stop-side half stays solid beam."""
    return (FRAME_X.tenon(root=1.0)
            .rotate((0, 0, 0), (0, 0, 1), site_deg)
            .translate((0, 0, BEAM_Z1)))


def _arc_mortise(site_deg):
    """Matching arc channel in frame_top's underside (the crossing's
    cavity): its CCW end wall — TOP_JOINT_SEAT_CLR shy of the seated
    tenon — is the angular STOP; the CW end sweeps open past the arm's
    CW face (the entry — the tenon swings in CCW from the open quadrant
    = frame_top seats rotating CW over it). In the +z→−z print the
    cavities open at the print top and close with plain floors."""
    return (FRAME_X.mortise(drop=2.0)
            .rotate((0, 0, 0), (0, 0, 1), site_deg)
            .translate((0, 0, BEAM_Z1)))


def _box(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane("XY").workplane(offset=z0)
            .polyline([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
            .close().extrude(z1 - z0))


def _yz_prism(pts_yz, x0, x1):
    """Closed YZ-profile prism spanning [x0, x1]."""
    return (cq.Workplane("YZ").polyline(pts_yz).close()
            .extrude(x1 - x0).translate((x0, 0, 0)))


# ── Lever support (v2's fork geometry, SIMPLIFIED — user's call #815: no
# fans/climbs; with the floor fused, nothing needs to pass below, so the
# columns stand straight on the foot pad). Verticals + 45° ramps only →
# −Z→+Z support-free. ────────────────────────────────────────────────────────
_SB_Y0 = RATCHET_LEV_Y1 + LEVER_SIDE_CLR         # 18.8 — side-column inner face
_SB_Y1 = _SB_Y0 + POST_OUT_T                     # 24.0 — side-column outer face
# arch-turn start, PER SIDE (user: each fork sets its own roof — reusing
# the ratchet's on the brake wasted ~19 of fork): the band's turn sits
# _ARCH_CLR above that side's swept ceiling. RATCHET: the swing-invariant
# pivot boss (the material around the lever's torsion axle) is the
# lever's top. BRAKE: the pad/flange assembly rides ABOVE its own pivot
# axle — its swept top (levers.BRAKE_SWEPT_TOP) governs instead.
_ARCH_CLR   = 1.0                                    # v2's roof clearance
_ARCH_Z0_R = RATCHET_PIVOT_Z + LEVER_BOSS_OD / 2.0 + _ARCH_CLR   # ≈ −18.5
_ARCH_Z0_B = BRAKE_SWEPT_TOP + _ARCH_CLR                          # ≈ −37


def _arch(s, z_b0):
    """CONSTANT-SECTION bent side arm (v2): column → 45° diagonal band →
    horizontal run into the beam's side face; perpendicular thickness =
    POST_OUT_T through every bend."""
    t = POST_OUT_T
    y5 = BEAM_SIZE / 2.0
    half = (_SB_Y0 - y5) / 2.0
    z_apex = z_b0 + half
    off = t / math.sqrt(2.0)
    pts = [(_SB_Y0, z_b0),
           (_SB_Y0 - half, z_apex),
           (y5, z_apex),
           (y5, z_apex + t),
           (_SB_Y0 + 2.0 * off - half - t, z_apex + t),
           (_SB_Y1, z_b0 + 2.0 * off - t),
           (_SB_Y1, z_b0 - 4.5),
           (_SB_Y0, z_b0 - 4.5)]
    return _yz_prism([(s * y, z) for y, z in pts], BEAM_IR, _R_OUT)


def _side_column(s, col_top):
    """One fork column (s=±1): the lever's outer pivot post, standing on
    the foot pad from the bed to where its arch strut departs."""
    col = _box(BEAM_IR, _R_OUT, min(s * _SB_Y0, s * _SB_Y1),
               max(s * _SB_Y0, s * _SB_Y1), FLOOR_Z0, col_top)
    # bevel the column's top-outer corner 45°, continuing the arch's plane
    bevel = _yz_prism([(s * _SB_Y0, col_top),
                       (s * _SB_Y1, col_top - (_SB_Y1 - _SB_Y0)),
                       (s * _SB_Y1, col_top + 2.0),
                       (s * _SB_Y0, col_top + 2.0)],
                      BEAM_IR - 1.0, _R_OUT + 1.0)
    return col.cut(bevel)


def _lever_pad():
    """FOOT PAD under the lever area (floor-thick) — THREE STRIPS now
    (user's call #833): one under the +X beam, one under each fork
    column, tying them to the wall band and floor at the bed. The bands
    directly under the LEVERS stay OPEN so a hand reaches up to the
    handles from below."""
    p = None
    for y0, y1 in ((-BEAM_SIZE / 2.0, BEAM_SIZE / 2.0),
                   (_SB_Y0, _SB_Y1), (-_SB_Y1, -_SB_Y0)):
        b = _box(WALL_IR - 2.0, _R_OUT, y0, y1, FLOOR_Z0, FLOOR_Z1)
        p = b if p is None else p.union(b)
    return p


def _beam_filler(angle_deg):
    """CRESCENT FILLER at a beam (user's call): the fused band's curved
    outer face pulls up to 0.18 away from the flat beam face across the
    beam's width — fill beam-wide from inside the band out into the beam,
    over the band's FULL height (bed → the lid-top plane, now that the
    band is one piece), ID trimmed to the bore cylinder."""
    f = (_box(66.0, BEAM_IR + 1.0, -BEAM_SIZE / 2.0, BEAM_SIZE / 2.0,
              FLOOR_Z0, WALL_Z1)
         .cut(cyl(2.0 * WALL_IR, (WALL_Z1 - FLOOR_Z0) + 1.0,
                  z=FLOOR_Z0 - 0.5)))
    return f.rotate((0, 0, 0), (0, 0, 1), angle_deg)


def _bay_web(s):
    """WEB sealing the tall slot between the wall band's cut end and its
    fork column (user's call — the wall's outer face curves inward to
    ~r 71 while the column starts at BEAM_IR): a column-width prism,
    wall→column in x, pad→band-top in z, its inner face trimmed to the
    wall's BORE cylinder — it continues the wall surface (and the 1.6
    disk clearance) straight into the column. Its top face at the split
    also widens wall_top's seat at this azimuth."""
    w = _box(66.0, BEAM_IR + 1.0,
             min(s * _SB_Y0, s * _SB_Y1), max(s * _SB_Y0, s * _SB_Y1),
             FLOOR_Z0, WALL_SPLIT_Z)
    return w.cut(cyl(2.0 * WALL_IR, (WALL_SPLIT_Z - FLOOR_Z0) + 1.0,
                     z=FLOOR_Z0 - 0.5))


def _hand_wedge():
    """FOOT WEDGE across the WHOLE lever front — all three uprights (both
    fork columns AND the +X beam, user's call): everything below the 45°
    plane running −z−x → +x+z, hinged at the wall bottom's OD at the
    bed, out past the outer faces. A finger can now hook up under the
    grips from outside. The three uprights lose their bed feet and hang
    from the band weld / webs / arches; their 45° undersides are
    self-supporting in the upright print. The T-channel and the fused
    band's weld tenons live far above the diagonal at these radii.
    (The earlier windowed ports through the fork blocks are RETIRED —
    user: the wedge alone is the access story.)"""
    return (cq.Workplane("XZ")
            .polyline([(WALL_OR, FLOOR_Z0 - 1.0), (WALL_OR, FLOOR_Z0),
                       (_R_OUT + 1.0, FLOOR_Z0 + (_R_OUT + 1.0 - WALL_OR)),
                       (_R_OUT + 1.0, FLOOR_Z0 - 1.0)])
            .close().extrude(2.0 * (_SB_Y1 + 1.0))
            .translate((0.0, _SB_Y1 + 1.0, 0.0)))


def _brake_rest_stop():
    """BRAKE REST STOP (user, params block): a small corbel rooted in the
    wall-band strip at the brake bay window's edge, reaching
    BRAKE_STOP_TIP_OVER past the pad's inner end. Its 45° underside —
    rising toward −y, the printable corbel face in this part's upright
    print — IS the catch face: the pretwisted axle swings the assembled
    lever back until the PAD's top inner corner lands on it (~1.3° past
    plumb, asserted in levers.py). The bare lever clears it at every
    angle from full pull to the axle-insertion pose, the braking stroke
    dives away from it, and it stays >2 mm off the separator's rim
    (all probed)."""
    zc = BRAKE_STOP_ZC
    y_c = BRAKE_LEV_Y1                        # −7 — the contact plane
    # the 45° underside now meets a vertical ONE-BEAD end face instead
    # of running to a knife point (user #886) — which also shortens the
    # −y reach to TOP_T − 0.8 past the contact plane
    y_tip = y_c - (BRAKE_STOP_TOP_T - NOZZLE)
    y_root = -4.5                             # buried into the band strip
    z_top = zc + BRAKE_STOP_TOP_T
    return (cq.Workplane("YZ")
            .polyline([(y_root, zc + (y_c - y_root)),
                       (y_tip, z_top - NOZZLE),
                       (y_tip, z_top), (y_root, z_top)])
            .close().extrude(WALL_OR - WALL_IR)
            .translate((WALL_IR, 0.0, 0.0)))


def _ratchet_stop():
    """RATCHET OVER-PULL STOP shelf (user's three corrections at #880;
    numbers pinned in levers.py): the pawl's WING (the lever's chunk
    past the handle's y-band — nothing static may live INSIDE that
    band, the full-height handle sweeps it during the ±swing) lands
    FULL-FACE here at RATCHET_OPEN_DEG, where the pawl clears the
    teeth by RATCHET_OPEN_CLR. The wing rides the lever's −y INNER
    side since #886 (the lever prints − to + y, so the nub must TOP
    the print), and the shelf grows out of the +X BEAM's flank —
    welded across its whole width — chunky (≥1.6 everywhere), its top
    matched to the wing's open-pose bottom plane; the usual 45° corbel
    underside carries its reach."""
    top1 = RSTOP_ZT0 + RSTOP_SLOPE * (RSTOP_X1 - RSTOP_X0)
    prism = (cq.Workplane("XZ")
             .polyline([(RSTOP_X0, RSTOP_ZT0), (RSTOP_X1, top1),
                        (RSTOP_X1, RSTOP_ZBOT), (RSTOP_X0, RSTOP_ZBOT)])
             .close().extrude(RSTOP_Y_TIP - (RSTOP_Y_ROOT - 1.0)))
    # whichever way the XZ plane extruded, land the y-span at
    # [ROOT − 1 (buried into the beam's flank), TIP]
    prism = prism.translate(
        (0.0, RSTOP_Y_TIP - prism.val().BoundingBox().ymax, 0.0))
    corbel = (cq.Workplane("YZ")
              .polyline([(RSTOP_Y_ROOT, RSTOP_ZBOT),
                         (RSTOP_Y_ROOT + 12.0, RSTOP_ZBOT + 12.0),
                         (RSTOP_Y_ROOT + 12.0, RSTOP_ZBOT - 8.0),
                         (RSTOP_Y_ROOT, RSTOP_ZBOT - 8.0)])
              .close().extrude(40.0).translate((50.0, 0.0, 0.0)))
    return prism.cut(corbel)


def _cable_horn():
    """CABLE HORN pier (user #889, corrected #890: STRAIGHT ALONG +X,
    not radial): a solid prism from the exit window's midway point
    running x-parallel out to the part's +x edge (the fork blocks'
    plane), top FLUSH with the window's bottom so the exiting cable
    rides straight onto it. Its root is trimmed to the band's BORE —
    nothing intrudes into the coil chamber — welding through the
    band's solid below-sill ring. Prints with the frame off the bed:
    plain verticals. (Next: the channel side walls + the lock cap.)"""
    yc = ((WALL_IR + WALL_OR) / 2.0
          * math.sin(math.radians(CABLE_EXIT_ANGLE_DEG)))
    p = (cq.Workplane("XY").workplane(offset=FLOOR_Z0)
         .center((44.0 + _R_OUT) / 2.0, yc)
         .rect(_R_OUT - 44.0, HORN_W)
         .extrude(HORN_Z1 - FLOOR_Z0))
    return p.cut(cyl(2.0 * WALL_IR, (HORN_Z1 - FLOOR_Z0) + 1.0,
                     z=FLOOR_Z0 - 0.5))


def _cable_guard(s):
    """^ CABLE-GUARD chevron for one lever bay (s=+1 ratchet, −1 brake;
    heights pinned in levers.py, user #887): the full-height windows
    that give fingers access would also let a slack coil escape into
    the levers. A GUARD_T-square bar fences the window at the wall
    line — 45° legs off both side walls (the beam-side band strip and
    the bay web), peak on the bay's centreline, the V underneath left
    open for the fingers. The peak clears the brake side's swept
    envelope by GUARD_LEVER_CLR and BOTH bays share the height."""
    y0 = RSTOP_Y_ROOT                        # 5.2 — window's beam-side edge
    y1 = _SB_Y0                              # 18.8 — web face
    mid = (y0 + y1) / 2.0
    t = GUARD_T * math.sqrt(2.0)             # ⊥ bar = GUARD_T on the 45°
    pts = [(y0 - 1.0, GUARD_PEAK_Z - (mid - y0) - 1.0),
           (mid, GUARD_PEAK_Z),
           (y1 + 1.0, GUARD_PEAK_Z - (mid - y0) - 1.0),
           (y1 + 1.0, GUARD_PEAK_Z - (mid - y0) - 1.0 - t),
           (mid, GUARD_PEAK_Z - t),
           (y0 - 1.0, GUARD_PEAK_Z - (mid - y0) - 1.0 - t)]
    return (cq.Workplane("YZ")
            .polyline([(s * y, z) for y, z in pts]).close()
            .extrude(GUARD_T).translate((GUARD_X0, 0.0, 0.0)))


def _floor():
    """The SPOKED coil-chamber floor (user's design): 1.6 plate — centre
    hub disc, FLOOR_SPOKE_N radial spokes, two tie rings under the coil
    span — everything 1.6 wide, spokes overlapping 1.0 into the fused
    wall band. Sparse on purpose: just enough web that the resting coil
    can't dip through and tangle."""
    f = cyl(2.0 * FLOOR_HUB_R, FLOOR_Z1 - FLOOR_Z0, z=FLOOR_Z0)
    for rc in FLOOR_RING_RC:
        f = f.union(
            cyl(2.0 * (rc + FLOOR_SPOKE_W / 2.0), FLOOR_Z1 - FLOOR_Z0,
                z=FLOOR_Z0)
            .cut(cyl(2.0 * (rc - FLOOR_SPOKE_W / 2.0),
                     (FLOOR_Z1 - FLOOR_Z0) + 1.0, z=FLOOR_Z0 - 0.5)))
    for i in range(FLOOR_SPOKE_N):
        a = i * 360.0 / FLOOR_SPOKE_N
        # the four CARDINAL spokes run beam-wide (user's call): they sit
        # under the beam azimuths and carry the hub's load out to the
        # band/beam feet as continuations of the beams
        w = BEAM_SIZE if a % 90.0 == 0.0 else FLOOR_SPOKE_W
        f = f.union(
            _box(0.0, WALL_IR + 1.0, -w / 2.0, w / 2.0, FLOOR_Z0, FLOOR_Z1)
            .rotate((0, 0, 0), (0, 0, 1), a))
    # CENTRE GUIDE SLEEVE (user's call): the tight coil winds against it
    # — it can never be pulled under the cable's bend radius — with a 45°
    # flared root for strength (widens toward the bed: support-free; the
    # hub disc is sized to land it). Open-top tube, floor plate closed.
    f = f.union(cyl(SLEEVE_OD, SLEEVE_Z1 - FLOOR_Z0, z=FLOOR_Z0))
    f = f.union(cone_solid(d_bottom=SLEEVE_OD + 2.0 * SLEEVE_FLARE,
                           d_top=SLEEVE_OD, h=SLEEVE_FLARE, z_base=FLOOR_Z1))
    f = f.cut(cyl(SLEEVE_OD - 2.0 * SLEEVE_T,
                  (SLEEVE_Z1 - FLOOR_Z1) + 1.0, z=FLOOR_Z1))
    # 45° fairlead chamfer on the top rim (the feed drop crosses here at
    # the tight state — no snag edge); narrows upward: support-free
    ch_ring = (cyl(SLEEVE_OD + 2.0, SLEEVE_TOP_CH, z=SLEEVE_Z1 - SLEEVE_TOP_CH)
               .cut(cone_solid(d_bottom=SLEEVE_OD,
                               d_top=SLEEVE_OD - 2.0 * SLEEVE_TOP_CH,
                               h=SLEEVE_TOP_CH,
                               z_base=SLEEVE_Z1 - SLEEVE_TOP_CH)))
    f = f.cut(ch_ring)
    # STUB AXLE (user's redesign — replaces the 45° rest lip): the static
    # spindle for the separator's bottom 608 — shoulder boss up to the
    # inner race's seat plane (lands on the face annulus, like the top
    # axle's lip), then the proven Ø7.95 slip shaft through the bore,
    # 45° entry chamfer at the tip. 45° base flare for strength, inside
    # the guide sleeve's bore (asserted).
    f = f.union(cyl(STUB_BOSS_D, STUB_BOSS_Z1 - FLOOR_Z0, z=FLOOR_Z0))
    f = f.union(cone_solid(d_bottom=STUB_BOSS_D + 2.0 * STUB_FLARE,
                           d_top=STUB_BOSS_D, h=STUB_FLARE, z_base=FLOOR_Z1))
    f = f.union(cyl(STUB_D, STUB_Z1 - STUB_BOSS_Z1, z=STUB_BOSS_Z1))
    tip = (cyl(STUB_D + 2.0, STUB_TIP_CH, z=STUB_Z1 - STUB_TIP_CH)
           .cut(cone_solid(d_bottom=STUB_D,
                           d_top=STUB_D - 2.0 * STUB_TIP_CH,
                           h=STUB_TIP_CH, z_base=STUB_Z1 - STUB_TIP_CH)))
    f = f.cut(tip)
    return f


def _thrust_boss(y_face, grow, pz, clip_x=None):
    """The lever's ONLY side contact: cadkit's contact ring — TEARDROP-
    profiled stock now (cadkit e07f1f7, user's print call: the round-top
    ring's ID crown printed as a sagging bridge) — bored with cadkit's own
    teardrop_hole BEFORE unioning (v2's crown-sliver lesson), so the
    surviving ring is one rib wide everywhere, apex included, and both its
    ID and OD ceilings print at 45°. Optionally clipped off the wall
    ring's slide path."""
    boss = contact_ring(BOSS_BORE_ID, (LEVER_PIVOT_X, y_face, pz),
                        (0.0, grow, 0.0), nozzle=NOZZLE)
    boss = boss.cut(teardrop_hole(BOSS_BORE_ID, LEVER_SIDE_CLR + 2.0,
                                  (LEVER_PIVOT_X, y_face - grow * 1.0, pz),
                                  (0.0, grow, 0.0)))
    if clip_x is not None:
        boss = boss.cut(cyl(2.0 * clip_x, 24.0, z=pz - 12.0))
    return boss


def _lever_mount():
    """Fork columns + arch struts + per-pivot thrust bosses (v2's design,
    fans/climbs retired; the lever plates/pins land next round on these
    same constants). Each side runs the SAME construction off its own
    roof height (_ARCH_Z0_R / _ARCH_Z0_B — user's call)."""
    _rise = _SB_Y0 - BEAM_SIZE / 2.0
    _half = _rise / 2.0
    _y2 = BEAM_SIZE / 2.0 + _half + 1.0
    m = None
    for s, z0 in ((+1.0, _ARCH_Z0_R), (-1.0, _ARCH_Z0_B)):
        col_top = z0 + 2.0 * POST_OUT_T / math.sqrt(2.0) - POST_OUT_T
        p = _side_column(s, col_top)
        p = p.union(_arch(s, z0))
        # centre-arm gusset: the beam spreads 45° into the arch
        p = p.union(_yz_prism(
            [(s * BEAM_SIZE / 2.0, z0),
             (s * (BEAM_SIZE / 2.0 + _half), z0 + _half),
             (s * BEAM_SIZE / 2.0, z0 + _half)],
            BEAM_IR, _R_OUT))
        # the arm turns back down 45° into the beam: bevel keeping the band
        # POST_OUT_T thick perpendicular from the ^ apex (v2's yellow == blue)
        _k = ((z0 + _half) - (_SB_Y0 - _half) + POST_OUT_T * math.sqrt(2.0))
        p = p.cut(_yz_prism(
            [(s * BEAM_SIZE / 2.0, BEAM_SIZE / 2.0 + _k),
             (s * BEAM_SIZE / 2.0, BEAM_Z1 + 1.0),
             (s * _y2, BEAM_Z1 + 1.0),
             (s * _y2, _y2 + _k)],
            BEAM_IR - 1.0, _R_OUT + 1.0))
        m = p if m is None else m.union(p)
    # thrust bosses: beam flank grows OUT to the lever's inner face (wall-
    # cylinder guard on its −x side); the column face grows IN
    for s, pz in ((+1.0, RATCHET_PIVOT_Z), (-1.0, BRAKE_PIVOT_Z)):
        m = m.union(_thrust_boss(s * BEAM_SIZE / 2.0, s, pz, clip_x=WALL_OR))
        m = m.union(_thrust_boss(s * _SB_Y0, -s, pz))
    return m


def _pin_bores_side(pz, s):
    """SQUARE-axle bores for one lever (s=+1 ratchet, −1 brake) — v2:
    DIAMOND (square at 45°), tip-up in this part's sideways print, so no
    teardrop needed. Column: THROUGH keyway; beam flank: BLIND bore whose
    floor at PIN_TIP_END_Y is the axle's depth stop. Both run through
    their thrust rings + 0.5 (a cutter step inside a ring strands a
    crown sliver — v2 lesson)."""
    y1o = s * RATCHET_LEV_Y1
    F = y1o + s * (LEVER_SIDE_CLR + POST_OUT_T)
    side = PIN_SQ_S + 2.0 * PIN_SQ_FRAME_CLR

    def sq(ya, yb):
        lo, hi = min(ya, yb), max(ya, yb)
        return (cq.Workplane("XY").rect(side, side)
                .extrude(hi - lo)
                .rotate((0, 0, 0), (1, 0, 0), -90)
                .translate((LEVER_PIVOT_X, lo, pz))
                .rotate((LEVER_PIVOT_X, 0, pz), (LEVER_PIVOT_X, 1, pz),
                        PIN_KEY_BASE_DEG))

    out = sq(F + s * 0.5, y1o - s * 0.5)
    out = out.union(sq(s * PIN_TIP_END_Y,
                       s * (BEAM_SIZE / 2.0 + LEVER_SIDE_CLR + 0.5)))
    return out


def _lever_pin_bores():
    return (_pin_bores_side(RATCHET_PIVOT_Z, +1.0)
            .union(_pin_bores_side(BRAKE_PIVOT_Z, -1.0)))


def _build_frame_bottom():
    fb = wall_band                        # the ONE fused wall band, bed →
                                          # the lid-top plane (wall.py)
    fb = fb.union(_floor())
    fb = fb.union(_lever_pad())
    fb = fb.union(_bay_web(+1.0)).union(_bay_web(-1.0))
    for a in (0.0, 90.0, 180.0, 270.0):
        fb = fb.union(_beam(a)).union(_arc_tenon(a)).union(_beam_filler(a))
    fb = fb.union(_lever_mount())
    fb = fb.union(_brake_rest_stop())
    fb = fb.union(_ratchet_stop())
    # BRAKE bay only (probed): the ratchet's pin-install swing
    # (+PIN_PRETWIST, its high pivot leaning the full-height handle far
    # inboard) blankets that bay's wall line from z −38.7 to the floor —
    # no static fence can exist there; see the #887 report
    fb = fb.union(_cable_guard(-1.0))
    fb = fb.union(_cable_horn())
    fb = fb.cut(_hand_wedge())
    fb = fb.cut(_lever_pin_bores())
    return heal(fb)


def _anchor_wall():
    """Spring-strip TENSION-TRAP anchor (params block): a wall sector
    under the +X arm, ANCH_Z0 up THROUGH the frame (the over-frame band
    is the arm rib and the wall's bed-rooted print seat in the +z→−z
    print). ASYMMETRIC sector now (user #879): the loaded HOLD edge
    sits ON the arm's +y face — the pier taking the strip's −y pull is
    the arm's own footprint — and the sector runs from the arm's −y
    face out past the +y entry wall. The window keeps the print rule:
    FLAT +z edge, one 45° floor descending to the DULLED one-bead tip
    at the +y entry wall; the bay is SNUG on the 9 neck (user: a
    trustworthy seat — the old 12 bay wobbled)."""
    w = (cq.Workplane("XZ")
         .polyline([(ANCH_IR, ANCH_Z0), (ANCH_OR, ANCH_Z0),
                    (ANCH_OR, FRAME_Z1), (ANCH_IR, FRAME_Z1)])
         .close().revolve(ANCH_A_NEG + ANCH_A_POS, (0, 0), (0, 1))
         .rotate((0, 0, 0), (0, 0, 1), -ANCH_A_NEG))
    y0 = ANCH_WIN_Y0                             # HOLD edge (beam's +y face)
    y1 = ANCH_WIN_Y0 + ANCH_WIN_W                # entry wall
    # the cutter must start INBOARD of the wall's inner skin at the
    # window's FAR-y end — the revolved wall's chord pulls in with y
    # (x = √(r² − y²)), which stranded a membrane when the window moved
    # off-centre (user-caught, #880)
    x_in = math.sqrt(ANCH_IR ** 2 - y1 ** 2) - 1.0
    win = (cq.Workplane("YZ").workplane(offset=x_in)
           .polyline([(y0, ANCH_HOLD_Z0),        # bay bottom corner (loaded)
                      (y0, ANCH_WIN_Z1),         # bay wall up to the flat top
                      (y1, ANCH_WIN_Z1),         # flat top edge
                      (y1, ANCH_TIP_Z),          # +y entry wall down to...
                      (y1 - NOZZLE, ANCH_TIP_Z)])    # ...the dulled tip flat
           .close().extrude((ANCH_OR - x_in) + 1.0))   # (close = the 45° floor)
    return w.cut(win)


def _build_frame_top():
    ft = _plus(FRAME_Z0, FRAME_Z1)
    ft = ft.union(cyl(BRG_BOSS_OD, FRAME_Z1 - FRAME_Z0, z=FRAME_Z0))
    ft = ft.union(_anchor_wall())   # tension-trap strip anchor + arm rib
    for a in (0.0, 90.0, 180.0, 270.0):
        ft = ft.cut(_arc_mortise(a))
    for c in mount_channel_cuts():  # flush mount channels in the ±Y arms
        ft = ft.cut(c)
    # lip bore through, the 45° FUNNEL seat (flipped-print cone — the
    # outer race's chamfer lands face-on-face on it, params block), then
    # the pocket from the funnel rim to the frame top
    ft = ft.cut(cyl(BRG_LIP_ID, (FRAME_Z1 - FRAME_Z0) + 1.0, z=FRAME_Z0 - 0.5))
    ft = ft.cut(cone_solid(d_bottom=BRG_LIP_ID, d_top=BRG_BORE,
                           h=BRG_SEAT_CONE_Z1 - BRG_SEAT_CONE_Z0,
                           z_base=BRG_SEAT_CONE_Z0))
    ft = ft.cut(cyl(BRG_BORE, (FRAME_Z1 - BRG_SEAT_CONE_Z1) + 0.5,
                    z=BRG_SEAT_CONE_Z1))
    return heal(ft)


frame_bottom = _build_frame_bottom()
frame_top = _build_frame_top()
