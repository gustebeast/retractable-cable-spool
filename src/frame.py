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
and pull-out all press frame_top CW = INTO the stops; uninstall (CCW)
is a torque no sustained load produces.
Nothing slides anymore.
"""

import math

import cadquery as cq

from cadkit.joinery import PrintSpec, joint
from cadkit.contact import contact_ring
from cadkit.holes import teardrop_hole

from .helpers import (chevron_plan, chord_x, cone_solid, cyl,
                      drop_stray_shells, heal)
from .levers import clip_post
from .levers import (
                     RSTOP_Y_TIP, RSTOP_WELD, RWING_X1, RWING_Y_TIP,
                     PAWL_Z_LO, PAWL_Z_HI, ratchet_wing_swept,
                     RATCHET_OPEN_DEG)
from .mount import mount_channel_cuts
from .wall import wall_band
from .params import (
    NOZZLE_D,
    FRAME_RIB, FRAME_R_OUT, FRAME_Z0, FRAME_Z1, FRAME_TOP_Z1, COUCH_TOP_PAD,
    LOCK_X0, LOCK_X1, LOCK_BLK_D, LOCK_WELD, LOCK_WALL_Y1,
    LOCK_SLOT_X0, LOCK_SLOT_X1, LOCK_SLOT_Y0, LOCK_SLOT_Y1,
    LOCK_TOPB_Z1, LOCK_BOTB_H, LOCK_SLOT_XA, LOCK_PIN_CLR, LOCK_PIN_Z0,
    LOCK_PIN_L, LOCK_SHIFT_X, LOCK_SHIFT_Y, LOCK_ROTS,
    ANCH_IR, ANCH_OR, ANCH_Z0, ANCH_A_NEG, ANCH_A_POS,
    ANCH_WIN_W, ANCH_WIN_Y0, ANCH_TIP_Z, ANCH_WIN_Z1, ANCH_HOLD_Z0,
    BRG_BORE, BRG_LIP_ID, BRG_BOSS_OD,
    BRG_SEAT_CONE_Z0, BRG_SEAT_CONE_Z1,
    WALL_IR, WALL_OR, WALL_SPLIT_Z, WALL_Z1,
    BEAM_IR, BEAM_SIZE, BEAM_Z0, BEAM_Z1,
    FLOOR_Z0, FLOOR_Z1,
    FLOOR_HOLE_D, FLOOR_HOLE_WEB, FLOOR_HOLE_R0, FLOOR_HOLE_R1,
    FLOOR_RIB_W,
    SLEEVE_OD, SLEEVE_T, SLEEVE_FLARE, SLEEVE_Z1, SLEEVE_TOP_CH,
    BRG_BORE, BOT_BRG_Z0, BOT_BRG_Z1, BOT_LIP_ID, BOT_LIP_H,
    BOT_LIP_FUNNEL, BOT_PLUG_Z0,
    N_HUB_WEB, HUB_WEB_W, HUB_WEB_Z1,
    JOINT_SPEC, JOINT_CLR, JOINT_BACK_CLR,
    FRAMEJ_W, FRAMEJ_R, FRAMEJ_TEN_ARC,
    TOP_JOINT_SEAT_CLR, TOP_ENTRY_OVER,
    LEVER_PIVOT_X, RATCHET_PIVOT_Z,
    RATCHET_LEV_Y1, LEVER_SIDE_CLR, LEVER_BOSS_OD,
    HORN_W, HORN_Z1, HORN_CH_W, HORN_BLOCK_H, STRUCT_WALL,
    HORN_BLOCK_L, HORN_BELL_R, HORN_YC,
    HORNJ_W, HORNJ_Z_CLR, HORN_STOP_T, HORN_STOP_DROP,
    HORN_WALL_T, HORN_X1,
    LEVER_PIN_D, LEVER_PIN_FRAME_CLR, LEVER_COL_Y1,
    POST_OUT_T, CLIP_SIDE, BOSS_BORE_ID, BOSS_RING_W,
    BOSS_RING_CUT,
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
# +z) for the parked spring and pull-out to both PRESS the stops. Uninstall = frame_top CCW, a torque no sustained load makes. ────────
_DOWN = PrintSpec(nozzle=NOZZLE_D, material="PETG-GF", facing="down")
FRAME_J = joint(FRAMEJ_W, None, tenon=JOINT_SPEC, mortise=_DOWN,
                install="-x")
FRAME_X = FRAME_J.crossing(FRAMEJ_R, BEAM_SIZE, FRAMEJ_TEN_ARC,
                           seat=TOP_JOINT_SEAT_CLR, over=TOP_ENTRY_OVER,
                           hand="ccw")

assert ((BEAM_IR + BEAM_SIZE) - (FRAMEJ_R + FRAMEJ_W / 2.0 + JOINT_CLR)
        >= 2 * NOZZLE_D - 1e-9), \
    "arc-joint cavity's outer wall under 1.6 at the flush arm end"
assert FRAME_RIB - FRAME_J.cavity_depth >= 2 * NOZZLE_D - 1e-9, \
    "arc-joint cavity ceiling under 1.6 in the top plus"
# the mount ring's cavities sit radially INBOARD of the arc-joint
# cavities — the web between them must hold tier
assert ((FRAMEJ_R - FRAMEJ_W / 2.0 - JOINT_CLR)
        - (MOUNT_RING_R + MOUNT_TEN_W / 2.0 + JOINT_CLR)
        >= 2 * NOZZLE_D - 1e-9), \
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


# the crossing solids are built ONCE and rotated per site (#932
# efficiency review — the arc-wire revolves are the cost, the rotates
# are free)
_ARC_TEN = FRAME_X.tenon(root=1.0)
_ARC_MOR = FRAME_X.mortise(drop=2.0)


def _arc_tenon(site_deg):
    """Arc tenon on a beam top (the cadkit crossing, root sunk 1.0): flush
    at the beam's CW entry face (hand="ccw"), spanning half the crossing
    inboard — the stop-side half stays solid beam."""
    return (_ARC_TEN.rotate((0, 0, 0), (0, 0, 1), site_deg)
            .translate((0, 0, BEAM_Z1)))


def _arc_mortise(site_deg):
    """Matching arc channel in frame_top's underside (the crossing's
    cavity): its CCW end wall — TOP_JOINT_SEAT_CLR shy of the seated
    tenon — is the angular STOP; the CW end sweeps open past the arm's
    CW face (the entry — the tenon swings in CCW from the open quadrant
    = frame_top seats rotating CW over it). In the +z→−z print the
    cavities open at the print top and close with plain floors."""
    return (_ARC_MOR.rotate((0, 0, 0), (0, 0, 1), site_deg)
            .translate((0, 0, BEAM_Z1)))


def _box(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane("XY").workplane(offset=z0)
            .polyline([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
            .close().extrude(z1 - z0))


def _yz_prism(pts_yz, x0, x1):
    """Closed YZ-profile prism spanning [x0, x1]."""
    return (cq.Workplane("YZ").polyline(pts_yz).close()
            .extrude(x1 - x0).translate((x0, 0, 0)))


def _xz_prism(pts_xz, y0, y1):
    """Closed XZ-profile prism spanning [y0, y1] — the missing sibling
    of _yz_prism (#932: the idiom was open-coded at four sites)."""
    return (cq.Workplane("XZ").polyline(pts_xz).close()
            .extrude(y1 - y0).translate((0, y1, 0)))


def _trim_to_bore(w, z_top):
    """Cut everything inside the band's bore from FLOOR_Z0 up to z_top —
    the shared 'nothing intrudes into the coil chamber' trim (#932: was
    spelled verbatim at three sites)."""
    return w.cut(cyl(2.0 * WALL_IR, (z_top - FLOOR_Z0) + 1.0,
                     z=FLOOR_Z0 - 0.5), clean=False)


# ── Lever support (v2's fork geometry, SIMPLIFIED — user's call #815: no
# fans/climbs; with the floor fused, nothing needs to pass below, so the
# columns stand straight on the foot pad). Verticals + 45° ramps only →
# −Z→+Z support-free. ────────────────────────────────────────────────────────
_SB_Y0 = RATCHET_LEV_Y1 + LEVER_SIDE_CLR         # 19.0 — side-column inner face
_SB_Y1 = _SB_Y0 + POST_OUT_T                     # 24.5 — side-column outer face
assert abs(_SB_Y1 - LEVER_COL_Y1) < 1e-9, "fork column face ≠ params LEVER_COL_Y1"
# arch-turn start: the band's turn sits _ARCH_CLR above the lever's
# swept ceiling, which is the swing-invariant pivot boss (the material
# around the pin). ONE fork since #1019 — the −y side's whole fork went
# with the brake, and with it the per-side roof rule, which existed
# because that lever's pad topped out ~19 lower than this one's boss.
_ARCH_CLR   = 1.0                                    # v2's roof clearance
_ARCH_Z0_R = RATCHET_PIVOT_Z + LEVER_BOSS_OD / 2.0 + _ARCH_CLR   # ≈ −18.5


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
    """FOOT PAD under the lever area (floor-thick) — TWO STRIPS since
    #1019: one under the +X beam, one under the ratchet's fork column,
    tying them to the wall band and floor at the bed. (The third, under
    the brake's column, went with that column — it was a bare ledge
    tying nothing.) The band directly under the LEVER stays OPEN so a
    hand reaches up to the handle from below."""
    p = None
    for y0, y1 in ((-BEAM_SIZE / 2.0, BEAM_SIZE / 2.0),
                   (_SB_Y0, _SB_Y1)):
        b = _box(WALL_IR - 2.0, _R_OUT, y0, y1, FLOOR_Z0, FLOOR_Z1)
        p = b if p is None else p.union(b)
    return p


def _beam_filler(angle_deg):
    """CRESCENT FILLER at a beam (user's call): the fused band's curved
    outer face pulls up to 0.18 away from the flat beam face across the
    beam's width — fill beam-wide from inside the band out into the beam,
    over the band's FULL height (bed → the lid-top plane, now that the
    band is one piece), ID trimmed to the bore cylinder."""
    f = _trim_to_bore(_box(66.0, BEAM_IR + 1.0,
                           -BEAM_SIZE / 2.0, BEAM_SIZE / 2.0,
                           FLOOR_Z0, WALL_Z1), WALL_Z1)
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
    return _trim_to_bore(w, WALL_SPLIT_Z)


def _hand_wedge(y_half=None):
    """FOOT WEDGE across the WHOLE lever front — all three uprights (both
    fork columns AND the +X beam, user's call): everything below the 45°
    plane running −z−x → +x+z, hinged at the wall bottom's OD at the
    bed, out past the outer faces. A finger can now hook up under the
    grips from outside. The three uprights lose their bed feet and hang
    from the band weld / webs / arches; their 45° undersides are
    self-supporting in the upright print. The T-channel and the fused
    band's weld tenons live far above the diagonal at these radii.
    (The earlier windowed ports through the fork blocks are RETIRED —
    user: the wedge alone is the access story.) `y_half` widens it
    (the clip posts stand out past the columns and are trimmed by it)."""
    yh = _SB_Y1 + 1.0 if y_half is None else y_half
    return (cq.Workplane("XZ")
            .polyline([(WALL_OR, FLOOR_Z0 - 1.0), (WALL_OR, FLOOR_Z0),
                       (_R_OUT + 1.0, FLOOR_Z0 + (_R_OUT + 1.0 - WALL_OR)),
                       (_R_OUT + 1.0, FLOOR_Z0 - 1.0)])
            .close().extrude(2.0 * yh)
            .translate((0.0, yh, 0.0)))


def _swung(x, z, pivot_z, d):
    """(x, z) of a wing point at rest, swung `d` about the lever pivot
    (the same −d rotation about +y the levers.py sweeps use)."""
    v = (cq.Vertex.makeVertex(x, 0.0, z)
         .rotate(cq.Vector(LEVER_PIVOT_X, 0, pivot_z),
                 cq.Vector(LEVER_PIVOT_X, 1, pivot_z), -d))
    return v.X, v.Z


def _stop_rib(x0, x1, z0, z1, y_r, y_f, y_t):
    """A stop-rib BLANK in a bay's air strip (signed y): rooted from y_r
    (buried in the wall) through the wall face y_f, standing out to its
    free face y_t. Its underside is a 45° GUSSET down into that face
    (user #993: the flat bottom cantilevered 1.6 off the flank — an
    overhang in the upright print)."""
    out = abs(y_t - y_f)                             # 1.6 — the cantilever
    return _yz_prism([(y_r, z0 - out), (y_f, z0 - out), (y_t, z0),
                      (y_t, z1), (y_r, z1)], x0, x1)


def _ratchet_stop():
    """RATCHET OVER-PULL STOP rib (kinematics in levers.py, #979): a
    chunk rooted along the +X BEAM's flank, standing in the bay's
    INNER air strip — the only place static frame material may live,
    since the plate's own y-band is swept by the handle. Its −x end is
    the WING's leading face at RATCHET_OPEN_DEG, carved by CUTTING the
    blank with the wing's swept volume, so the wing BUTTS it in pure
    compression instead of landing on a slope it skidded off (#880's
    shelf). (#993, printability) The blank's top stops at the wing's
    leading-TOP corner at the stop pose — above it the old blank hung
    over the wing's swept path, a notch ceiling in the upright print —
    and its underside is a 45° gusset into the beam flank."""
    _, z_top = _swung(RWING_X1, PAWL_Z_HI, RATCHET_PIVOT_Z, RATCHET_OPEN_DEG)
    blank = _stop_rib(RWING_X1, BEAM_IR + BEAM_SIZE,
                      PAWL_Z_LO - 6.0, z_top, RWING_Y_TIP - RSTOP_WELD,
                      RWING_Y_TIP, RSTOP_Y_TIP)
    rib = blank.cut(ratchet_wing_swept())
    # the discrete sweep leaves ~0-volume step slivers at the capped top
    return cq.Workplane("XY").add(
        max(rib.solids().vals(), key=lambda v: v.Volume()))


def _cable_horn():
    """CABLE HORN (user #889, redesigned #903 — the ideal cable model
    first; SPLIT for printability/install at #905; the full pier
    RETIRED for material at #914 — the block rides its own footing on
    a 45° strut + band foot, see the A/B/C comment in the body; the
    cable now flies unsupported from the window sill to the tunnel
    mouth, which its tension is fine with). The TUNNEL BLOCK:
    a prism at the part's +x edge, top of the old deck line, shaped by
    cuts —
      · the TUNNEL: a round HORN_CH_W bore along x, its floor tangent
        to the pier top (the cable slides on at deck level, no step);
      · a MOUTH BELL at EACH end: a quarter-round trumpet revolved a
        FULL CIRCLE around the bore axis, tangent to the bore and to
        the face (they scoop into the footing below). The +x exit
        needs it — the free cable pulls in any direction with no edge
        to bite; the −x entry reuses it unchanged for simplicity
        (user: the spool-plane cable never technically needs the
        top/bottom quadrants there).
    Returns (bottom, cap) SPLIT at the bore's equator plane (user
    #905, printability + install): the bottom half — an open
    semicircular trough whose walls steepen to vertical at the plane
    — stays on frame_bottom and prints upright overhang-free; the
    upper half is the HORN CAP, its own part (flat top face on the
    bed, trough up — also overhang-free). The cable lays into the
    trough from above and the cap closes the circle.
    +x EXTENT (user #963): the block's +x face is params.HORN_X1 —
    the frame face on the desk build, the couch's front-beam FRONT
    face in the couch version (the exit runs out under the beam).
    A, B and C are written off that face, so they follow it: the
    strut's bed landing moves out with the block (past the frame's
    plan when extended — printable, it is on the bed). The brace D
    descends 45° from the sill onto the strut's inboard face; once
    the block is out of its 45° reach it lands on the bed foot's top
    instead (user #964: keep the strut), tying the band wall to the
    foot just inboard of the strut's base.
    ATTACHMENT (user #906, single rail #915, params HORNJ_*): ONE
    central cadkit MUSHROOM rail, the cap sliding on − to + y: two
    TENON segments rise off the trough strips' tops (up-printed: 45°
    flares, no bridge); the cap takes one through-CAVITY (down-printed
    host: the wide flat end is its first supported layers — no
    bridge), open at the +y face for entry, z-sandwich at the lid's
    print-proven 0.20. The STOP FLANGE on the cap's trailing −y end
    hangs past the equator and lands on the trough's −y face — it,
    not the cavity end wall (relieved the joint's back gap deeper),
    defines seating and blocks +y over-travel; it rides in free air
    (the exit window's band) for the whole slide."""
    yc = HORN_YC
    ch2 = HORN_CH_W / 2.0
    zc = HORN_Z1 + ch2             # tunnel axis: bore floor = the sill
    xb0 = HORN_X1 - HORN_BLOCK_L   # block x-span [xb0, HORN_X1]
    x_mid = (xb0 + HORN_X1) / 2.0
    y_lo, y_hi = yc - HORN_W / 2.0, yc + HORN_W / 2.0
    rim = HORN_BELL_R + STRUCT_WALL          # bell mouth + its kept wall
    diag = math.sqrt(2.0) * (ch2 + rim)      # the 45° bell-wall diagonal —
                                   # shared by A's depth and the chamfer
                                   # rule so the two BIND by construction

    def _blk(z0, z1, pad=0.0):     # the block footprint, from z0 to z1
        return _box(xb0 - pad, HORN_X1 + pad, y_lo - pad, y_hi + pad,
                    z0, z1)

    def _chord(R):                 # band chord at the support's far edge
        return chord_x(R, yc + POST_OUT_T / 2.0)

    # MATERIAL-SAVING support (user #914 — the full drum-to-edge pier is
    # RETIRED). Four prisms:
    #   A · the exit block's own FOOTING: the block footprint extended
    #       down to z_a0 — deep enough for all its geometry (the bells
    #       scoop HORN_BELL_R below the deck) plus 1.6 of material;
    #   B · a 45° STRUT falling inboard from A's underside to the print
    #       bed: HORIZONTAL width = HORN_BLOCK_L (user #915 — the whole
    #       exit section locked to the 10.4 beam budget), y-width
    #       POST_OUT_T (user: match the lever SIDE supports, 5.6 — A
    #       overhangs it ±2.8 in y, accepted for now); its top face
    #       carries A's x-span exactly and its underside corbels at 45°;
    #   C · a FOOT prism on the bed tying B into the band's below-sill
    #       ring (BEAM_SIZE tall — user #917 — by POST_OUT_T wide);
    #   D · a 45° BRACE (user #919: cable tension left on the block
    #       must not bend it backwards): HORN_BLOCK_L horizontal width
    #       (matching B, user #920) × POST_OUT_T in y, from the exit
    #       window's sill in the band, falling down-OUTBOARD at 45°
    #       into B's top face (sunk 0.5 — the cadkit volumetric-fusion
    #       rule), triangulating B's other diagonal.
    # B and C are bore-trimmed like the old pier root — nothing enters
    # the coil chamber; the band ring is the weld.
    # A's depth (user #925): deep enough for the bell scoops + 1.6 AND
    # for the 45° corner chamfers to narrow the bottom to the strut's
    # width while keeping the full 1.6 wall at the bell mouths — the
    # second term grows with the bore, so a fat exit cable deepens the
    # footing instead of leaving overhung ledges past the strut
    dep = max(rim, diag - POST_OUT_T / 2.0 - ch2)
    z_a0 = HORN_Z1 - dep
    p = _blk(z_a0, HORN_Z1)                                 # A
    d_bed = z_a0 - FLOOR_Z0
    y_s0, y_s1 = yc - POST_OUT_T / 2.0, yc + POST_OUT_T / 2.0
    strut = _xz_prism([(HORN_X1, z_a0),                     # B
                       (HORN_X1 - d_bed, FLOOR_Z0),
                       (HORN_X1 - d_bed - HORN_BLOCK_L, FLOOR_Z0),
                       (xb0, z_a0)], y_s0, y_s1)
    # C: from the band's bore chord at the strut's FAR-y edge (the #890
    # membrane lesson) to EXACTLY where B's 45° underside lands on the
    # bed (user #918: running past it left a little vertical end face
    # sticking out from under the slope; flush, the end face is buried
    # in the strut's foot wedge)
    x_c0 = _chord(WALL_IR) - 1.0
    foot = _box(x_c0, HORN_X1 - d_bed, y_s0, y_s1,          # C
                FLOOR_Z0, FLOOR_Z0 + BEAM_SIZE)
    # D: top edge at the sill, its outboard END flush with the band's
    # OUTER face at the far-y corner (user #921: it poked 4.98 past the
    # sill's edge there — the outer chord shrinks fast at these y's);
    # the brace welds into the band's outer wall lower down its 45° run
    x_d0 = _chord(WALL_OR) - HORN_BLOCK_L
    x_land = xb0 - d_bed          # where B's TOP plane lands on the bed
    sink = 0.5                    # into B — volumetric fusion
    x_m1 = (HORN_Z1 + x_d0 + x_land + sink - FLOOR_Z0) / 2.0
    x_m2 = x_m1 + HORN_BLOCK_L / 2.0
    z_c1 = FLOOR_Z0 + BEAM_SIZE   # the foot's top
    if HORN_Z1 - (x_m1 - x_d0) >= FLOOR_Z0 + STRUCT_WALL - 1e-9:
        # D lands on B's inboard 45° face, its toe ≥ a tier above the
        # bed (the desk build, and moderate extensions — #965)
        brace = _xz_prism([(x_d0, HORN_Z1),                 # D
                           (x_d0 + HORN_BLOCK_L, HORN_Z1),
                           (x_m2, HORN_Z1 - (x_m2 - x_d0 - HORN_BLOCK_L)),
                           (x_m1, HORN_Z1 - (x_m1 - x_d0))], y_s0, y_s1)
    else:
        # EXTENDED exit (user #964: the strut is out of D's 45° reach,
        # but the brace stays): the same 45° descent from the sill now
        # lands on the FOOT's top, sunk `sink` — tying the band wall to
        # the foot just inboard of the strut's base
        x_m1 = x_d0 + (HORN_Z1 - (z_c1 - sink))
        x_m2 = x_m1 + HORN_BLOCK_L
        assert x_m2 <= HORN_X1 - d_bed + 1e-9, \
            "extended-exit brace lands past the foot's end"
        brace = _xz_prism([(x_d0, HORN_Z1),                 # D
                           (x_d0 + HORN_BLOCK_L, HORN_Z1),
                           (x_m2, z_c1 - sink),
                           (x_m1, z_c1 - sink)], y_s0, y_s1)
    support = _trim_to_bore(
        strut.union(foot, clean=False).union(brace, clean=False), HORN_Z1)
    p = p.union(support, clean=False)
    # TUNNEL BLOCK on the footing, flush with the +x edge
    p = p.union(_blk(HORN_Z1, HORN_Z1 + HORN_BLOCK_H), clean=False)
    # BOTTOM-CORNER CHAMFERS (user #922/#925/#926): cut AFTER the block
    # union so the 45° also shaves the block's side walls wherever it
    # rises past the deck (order-bug caught by the user at the 10mm
    # demo: cutting before the block left overhung wall strips at deck
    # level). Two rules size the cut — ≥ 1.6 of wall left to the bell
    # mouths, and a remaining bottom face no narrower than the strut's
    # POST_OUT_T; A's derived depth (above) makes them BIND TOGETHER,
    # so the bottom face lands exactly on the strut at every bore size
    # with the bell rims intact. The cutters run 4.0 up-and-out past
    # the sides so the 45° keeps cutting anything higher up.
    d_wall = (zc - z_a0) + HORN_W / 2.0 - diag
    d_ch = min((HORN_W - POST_OUT_T) / 2.0, d_wall)
    assert d_ch > 0.0, "horn footing chamfer has no room"
    for s in (+1.0, -1.0):
        yk = yc + s * (HORN_W / 2.0 - d_ch)      # chamfer knee at z_a0
        p = p.cut(_yz_prism([(yk, z_a0),
                             (yk + s * (d_ch + 4.0), z_a0),
                             (yk + s * (d_ch + 4.0), z_a0 + d_ch + 4.0)],
                            xb0 - 1.0, HORN_X1 + 1.0), clean=False)
    # cut 1 — the TUNNEL (round; the mid-bore overhang is moot since the
    # #905 equator split — each half prints trough-up)
    p = p.cut(
        cq.Workplane("YZ").workplane(offset=xb0 - 1.0)
        .center(yc, zc).circle(ch2)
        .extrude(HORN_BLOCK_L + 2.0), clean=False)
    # cuts 2+3 — a MOUTH BELL at each end: quarter-round profile
    # (tangent to the bore wall, tangent to the face) revolved 360°
    # around the bore axis; s = which way the mouth opens
    r = HORN_BELL_R
    s45 = math.sqrt(0.5)
    for x_f, s in ((HORN_X1, +1.0), (xb0, -1.0)):
        bell = (cq.Workplane("XZ")
                .moveTo(x_f - s * r, ch2)
                .threePointArc(
                    (x_f - s * r * (1.0 - s45), ch2 + r * (1.0 - s45)),
                    (x_f, ch2 + r))
                .lineTo(x_f + s * 1.0, ch2 + r)
                .lineTo(x_f + s * 1.0, ch2)
                .close()
                .revolve(360.0, (0.0, 0.0), (1.0, 0.0))
                .translate((0.0, yc, zc)))
        p = p.cut(bell, clean=False)
    # SPLIT at the bore equator (z = zc): everything above becomes the
    # horn cap, its own printed part
    upper = _blk(zc, zc + HORN_BLOCK_H, pad=2.0)
    bottom = p.cut(upper, clean=False)
    cap = p.intersect(upper, clean=False)
    # STOP FLANGE on the cap's trailing −y end (extra material on the
    # CAP only — the bottom prism keeps its size, user), flush with the
    # cap's top face
    cap = cap.union(
        _box(xb0, HORN_X1, y_lo - HORN_STOP_T, y_lo,
             zc - HORN_STOP_DROP, HORN_Z1 + HORN_BLOCK_H), clean=False)
    # MUSHROOM rail (cadkit): local +X (the slide) → global −y — the
    # CAP travels +y over the fixed tenons; local Y → global +x
    hj = joint(HORNJ_W, HORN_WALL_T, tenon=JOINT_SPEC, mortise=_DOWN,
               install="+x", back_clearance=HORNJ_Z_CLR)
    # (z-sandwich tightened to the lid's print-proven 0.20 — user #912:
    # short joinery that must stay put)
    assert HORN_BLOCK_H - hj.cavity_depth >= STRUCT_WALL - 1e-9, \
        "horn cap roof over the rail cavity"
    assert hj.width / 2.0 + hj.clearance \
        <= HORN_BLOCK_L / 2.0 - HORN_BELL_R, \
        "horn rail window reaches the mouth bells"
    m_len = HORN_W + hj.back_clearance + 1.0
    for y_top in (y_lo + HORN_WALL_T, y_hi):
        bottom = bottom.union(
            hj.tenon(root=1.0)
            .rotate((0, 0, 0), (0, 0, 1), -90.0)
            .translate((x_mid, y_top, zc)), clean=False)
    cap = cap.cut(
        hj.mortise(drop=1.0, length=m_len)
        .rotate((0, 0, 0), (0, 0, 1), -90.0)
        .translate((x_mid, y_hi + 1.0, zc)), clean=False)
    return bottom, cap


def _floor():
    """The coil-chamber floor: a SOLID plate out to the fused wall band
    (user #977 — the old sparse spoke web let a wrap drop into its
    openings and JAM), perforated by a HEX GRID of viewing holes (user
    #980 — round holes tile far tighter than the first cut's arc
    slits). Every hole is FLOOR_HOLE_D — under the cable Ø until #994,
    now twice that for a better view (a wrap can sag into one but not
    kink; params.py asserts the bound), and they pack on a
    triangular lattice at the two-bead web — as many as the plate can
    carry. SOLID where it must be: the hub (it carries the stub axle
    and the sleeve), the rim at the band (the weld, and the coil's
    outermost wraps ride it) and four CARDINAL ribs under the beams
    (the hub's load path out, what the old beam-wide spokes did). One
    compound, ONE boolean, cut from the bare plate long before it meets
    the rest of the part."""
    f = cyl(2.0 * (WALL_IR + 1.0), FLOOR_Z1 - FLOOR_Z0, z=FLOOR_Z0)
    pitch = FLOOR_HOLE_D + FLOOR_HOLE_WEB
    row = pitch * math.sqrt(3.0) / 2.0          # triangular lattice
    r_hole = FLOOR_HOLE_D / 2.0
    keep_out = FLOOR_RIB_W / 2.0 + r_hole       # clear of the cardinal ribs
    n_row = int(FLOOR_HOLE_R1 / row) + 2
    n_col = int(FLOOR_HOLE_R1 / pitch) + 2
    holes = []
    for j in range(-n_row, n_row + 1):
        y = j * row
        x0 = (pitch / 2.0) if j % 2 else 0.0
        for i in range(-n_col, n_col + 1):
            x = x0 + i * pitch
            r = math.hypot(x, y)
            if not (FLOOR_HOLE_R0 + r_hole <= r <= FLOOR_HOLE_R1 - r_hole):
                continue
            if abs(x) < keep_out or abs(y) < keep_out:
                continue
            holes.append(cq.Solid.makeCylinder(
                r_hole, (FLOOR_Z1 - FLOOR_Z0) + 1.0,
                cq.Vector(x, y, FLOOR_Z0 - 0.5), cq.Vector(0, 0, 1)))
    f = f.cut(cq.Workplane(obj=cq.Compound.makeCompound(holes)), clean=False)
    # CENTRE GUIDE SLEEVE (user's call): the tight coil winds against it
    # — it can never be pulled under the cable's bend radius — with a 45°
    # flared root for strength (widens toward the bed: support-free; the
    # hub disc is sized to land it). Open-top tube, floor plate closed.
    f = f.union(cyl(SLEEVE_OD, SLEEVE_Z1 - FLOOR_Z0, z=FLOOR_Z0),
                clean=False)
    f = f.union(cone_solid(d_bottom=SLEEVE_OD + 2.0 * SLEEVE_FLARE,
                           d_top=SLEEVE_OD, h=SLEEVE_FLARE,
                           z_base=FLOOR_Z1), clean=False)
    f = f.cut(cyl(SLEEVE_OD - 2.0 * SLEEVE_T,
                  (SLEEVE_Z1 - FLOOR_Z1) + 1.0, z=FLOOR_Z1), clean=False)
    # 45° fairlead chamfer on the top rim (the feed drop crosses here at
    # the tight state — no snag edge); narrows upward: support-free
    ch_ring = (cyl(SLEEVE_OD + 2.0, SLEEVE_TOP_CH, z=SLEEVE_Z1 - SLEEVE_TOP_CH)
               .cut(cone_solid(d_bottom=SLEEVE_OD,
                               d_top=SLEEVE_OD - 2.0 * SLEEVE_TOP_CH,
                               h=SLEEVE_TOP_CH,
                               z_base=SLEEVE_Z1 - SLEEVE_TOP_CH)))
    f = f.cut(ch_ring, clean=False)
    # STUB AXLE (user's redesign — replaces the 45° rest lip): the static
    # spindle for the separator's bottom 608 — shoulder boss up to the
    # inner race's seat plane (lands on the face annulus, like the top
    # axle's lip), then the shaft at the 608's nominal bore, STRAIGHT to
    # its FLAT tip (user #945: the 0.8 entry chamfer retired — full-
    # cylinder grip on the inner race over the stub's whole reach; the
    # upright print never needed the chamfer). 45° base flare for
    # strength, inside the guide sleeve's bore (asserted).
    # BOTTOM-BEARING POCKET (user #1020, replacing the static stub): the
    # sleeve's top is PLUGGED solid and bored Ø BRG_BORE down to a
    # retention lip — the outer race drops in from above and rests on it,
    # the separator's nub turns in the bore. Under the lip the bore opens
    # back out at 45° into the sleeve's own bore, so the lip's underside
    # is a funnel and not a ceiling in the upright print.
    z_plug = BOT_PLUG_Z0
    f = f.union(cyl(SLEEVE_OD, BOT_BRG_Z1 - z_plug, z=z_plug), clean=False)
    f = f.cut(cyl(BRG_BORE, (BOT_BRG_Z1 - BOT_BRG_Z0) + 1.0, z=BOT_BRG_Z0),
              clean=False)
    f = f.cut(cyl(BOT_LIP_ID, BOT_LIP_H, z=BOT_BRG_Z0 - BOT_LIP_H),
              clean=False)
    f = f.cut(cone_solid(d_bottom=SLEEVE_OD - 2.0 * SLEEVE_T,
                         d_top=BOT_LIP_ID, h=BOT_LIP_FUNNEL,
                         z_base=BOT_PLUG_Z0),
              clean=False)
    # HUB WEBS (params, user #970): radial ribs boss -> sleeve wall,
    # floor plate -> sleeve top, buried 0.5 into each neighbour
    web = _box(0.0, SLEEVE_OD / 2.0 - SLEEVE_T + 0.5,
               -HUB_WEB_W / 2.0, HUB_WEB_W / 2.0,
               FLOOR_Z1 - 0.5, HUB_WEB_Z1)
    for i in range(N_HUB_WEB):
        f = f.union(web.rotate((0, 0, 0), (0, 0, 1), i * 360.0 / N_HUB_WEB),
                    clean=False)
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
                        (0.0, grow, 0.0), nozzle=NOZZLE_D)
    boss = boss.cut(teardrop_hole(BOSS_BORE_ID, LEVER_SIDE_CLR + 2.0,
                                  (LEVER_PIVOT_X, y_face - grow * 1.0, pz),
                                  (0.0, grow, 0.0)))
    if clip_x is not None:
        boss = boss.cut(cyl(2.0 * clip_x, 24.0, z=pz - 12.0))
    # the ring's proud face trimmed back (user #1014): at 0.05 of side
    # play the pair pinched the lever too hard for the TPU to
    # return it
    face = y_face + grow * (BOSS_RING_W - BOSS_RING_CUT)
    lo = min(face, face + grow * 20.0)
    return boss.cut(_box(LEVER_PIVOT_X - 20.0, LEVER_PIVOT_X + 20.0,
                         lo, lo + 20.0, pz - 20.0, pz + 20.0))


def _lever_mount():
    """Fork columns + arch struts + per-pivot thrust bosses (v2's design,
    fans/climbs retired; the lever plates/pins land next round on these
    same constants). Each side runs the SAME construction off its own
    roof height (_ARCH_Z0_R — the −y fork left with the brake, #1019)."""
    _rise = _SB_Y0 - BEAM_SIZE / 2.0
    _half = _rise / 2.0
    _y2 = BEAM_SIZE / 2.0 + _half + 1.0
    m = None
    for s, z0 in ((+1.0, _ARCH_Z0_R),):
        col_top = z0 + 2.0 * POST_OUT_T / math.sqrt(2.0) - POST_OUT_T
        p = _side_column(s, col_top)
        p = p.union(_arch(s, z0), clean=False)
        # centre-arm gusset: the beam spreads 45° into the arch
        p = p.union(_yz_prism(
            [(s * BEAM_SIZE / 2.0, z0),
             (s * (BEAM_SIZE / 2.0 + _half), z0 + _half),
             (s * BEAM_SIZE / 2.0, z0 + _half)],
            BEAM_IR, _R_OUT), clean=False)
        # the arm turns back down 45° into the beam: bevel keeping the band
        # POST_OUT_T thick perpendicular from the ^ apex (v2's yellow == blue)
        _k = ((z0 + _half) - (_SB_Y0 - _half) + POST_OUT_T * math.sqrt(2.0))
        p = p.cut(_yz_prism(
            [(s * BEAM_SIZE / 2.0, BEAM_SIZE / 2.0 + _k),
             (s * BEAM_SIZE / 2.0, BEAM_Z1 + 1.0),
             (s * _y2, BEAM_Z1 + 1.0),
             (s * _y2, _y2 + _k)],
            BEAM_IR - 1.0, _R_OUT + 1.0), clean=False)
        m = p if m is None else m.union(p, clean=False)
    # thrust bosses: beam flank grows OUT to the lever's inner face (wall-
    # cylinder guard on its −x side); the column face grows IN
    for s, pz in ((+1.0, RATCHET_PIVOT_Z),):
        m = m.union(_thrust_boss(s * BEAM_SIZE / 2.0, s, pz, clip_x=WALL_OR),
                    clean=False)
        m = m.union(_thrust_boss(s * _SB_Y0, -s, pz), clean=False)
    return m


def _clip_posts():
    """Each lever's TPU return-clip POST (#1004 — replaces the Lee
    spring's pocket and leg groove): levers.clip_post. Both stand well
    clear of the hand-access cutaway (asserted in levers.py) — the #1007
    trim by the cut left a 35° edge where its 45° met the gusset's
    (user)."""
    out = None
    for s, pz in ((CLIP_SIDE, RATCHET_PIVOT_Z),):
        p = clip_post(pz, s)
        out = p if out is None else out.union(p, clean=False)
    return out


def _pin_bores_side(pz, s):
    """ROUND running bores for the lever's pivot pin (#988; re-laid at
    #1029, when the pin grew to SPAN the beam). Two cuts:
      · the +y fork column, THROUGH (user #1035 — it was blind, and the
        pin only reached 1.6 into it). The pin now runs the column's full
        5.6 of wall, so it is carried at both ends of its span; nothing
        can bottom anywhere, which leaves the WEDGE as the only stop.
      · the BEAM, THROUGH both flanks and on out past the −y face, where
        the TPU clip now lives.
    Horizontal in this part's upright print, so cadkit's teardrop cutter
    peaks their crowns at 45°; the pin rides the round part. Both run
    through their thrust rings + 0.5 (a cutter step inside a ring strands
    a crown sliver)."""
    y1o = s * RATCHET_LEV_Y1
    d = LEVER_PIN_D + 2.0 * LEVER_PIN_FRAME_CLR

    def bore(ya, yb):
        lo, hi = min(ya, yb), max(ya, yb)
        h = teardrop_hole(d, hi - lo, (LEVER_PIVOT_X, lo, pz), (0.0, 1.0, 0.0))
        return h if isinstance(h, cq.Workplane) else cq.Workplane(obj=h)

    col = bore(s * (LEVER_COL_Y1 + 0.5), y1o - s * 0.5)    # column, through
    beam = bore(s * (BEAM_SIZE / 2.0 + LEVER_SIDE_CLR + 0.5),
                -s * (BEAM_SIZE / 2.0 + 1.0))              # beam, through
    return col.union(beam)


def _lever_pin_bores():
    return _pin_bores_side(RATCHET_PIVOT_Z, +1.0)   # one lever since #1019


def _lock_place(w):
    """As-drawn lock geometry → its world spot on the −y beam's +x
    flank (user #941: translated, never rotated)."""
    return w.translate((LOCK_SHIFT_X, LOCK_SHIFT_Y, 0.0))


def _lock_copies(w):
    """A placed lock solid at EVERY lock site (#983: the −y beam's, and
    its copy turned about z by each further LOCK_ROTS — a rotation,
    so the copy keeps the lock's handedness)."""
    return [w.rotate((0, 0, 0), (0, 0, 1), r) for r in LOCK_ROTS]


def _lock_corbel_top():
    """TPU-LOCK middle storey (user #939/#941, params LOCK block): a
    corbel off the −y beam's +x FLANK, growing +x far enough to wall
    the pin slot 1.6 on every side. In this part's FLIPPED print the
    model-TOP surface is the overhang, so it is the 45: peaked at the
    flank (the LOCK_WELD stub buried in the arm rises 45 too — a flat
    weld ledge would print as an overhang), falling toward +x. Its
    roof is the lock's interface with the mount head (#983: in the
    couch build it sits LOCK_HEAD_DROP lower, handing the pad's extra
    height to the head; the corbel itself keeps its desk shape)."""
    # COUCH version (#955): the LOCK_* z's are MOUNT-local (the head is
    # an unchanged part seating COUCH_TOP_PAD higher), so this storey is
    # drawn with its foot dropped by the pad and then lifted — its roof
    # rides under the raised head at the same LOCK_SWEEP_CLR, and the
    # extra height is solid corbel down to the frame underside
    zb = -COUCH_TOP_PAD
    w = _xz_prism(
        [(LOCK_X0 - LOCK_WELD, zb),
         (LOCK_X0 - LOCK_WELD, LOCK_TOPB_Z1 - LOCK_WELD),
         (LOCK_X0, LOCK_TOPB_Z1),
         (LOCK_X1, LOCK_TOPB_Z1 - LOCK_BLK_D),
         (LOCK_X1, zb)],
        FRAME_RIB / 2.0, LOCK_WALL_Y1)       # #1064/#1066: the section
                                   # runs STRUCT_WALL past the slot, so the
                                   # pin is walled on all four sides; the
                                   # screw pad is inset clear of it
    return _lock_place(w).translate((0.0, 0.0, COUCH_TOP_PAD))


def _lock_corbel_bot():
    """TPU-LOCK bottom storey: the flank mirror of _lock_corbel_top —
    this part prints UPRIGHT, so the 45 is the UNDERSIDE, rising
    toward +x (and rising into the weld stub buried in the beam)."""
    return _lock_place(_xz_prism(
        [(LOCK_X0 - LOCK_WELD, 0.0),
         (LOCK_X1, 0.0),
         (LOCK_X1, -(LOCK_BOTB_H - LOCK_BLK_D)),
         (LOCK_X0, -LOCK_BOTB_H),
         (LOCK_X0 - LOCK_WELD, -(LOCK_BOTB_H - LOCK_WELD))],
        FRAME_RIB / 2.0, LOCK_WALL_Y1))      # #1064: full inboard wall —
                                   # nothing of the mount reaches this far
                                   # down, so this one just widens


def _lock_prism(z0, z1, inset=0.0):
    """The shared pin-slot plan over [z0, z1] — ONE shape for all three
    storeys AND for the pin itself (user's recipe: size the prisms,
    then cut the TPU pin's shape through everything). `inset` makes it
    the PIN instead of the slot. The +x end is a chevron, which is what
    keeps the corbels' 45 fronts from bridging the slot (params)."""
    return (cq.Workplane("XY").workplane(offset=z0)
            .polyline(chevron_plan(LOCK_SLOT_X0, LOCK_SLOT_X1, LOCK_SLOT_XA,
                                   LOCK_SLOT_Y0, LOCK_SLOT_Y1, inset))
            .close().extrude(z1 - z0))


def _lock_slot_cut(z0, z1):
    """The pin slot, placed on the lock's beam."""
    return _lock_place(_lock_prism(z0, z1))


def _build_frame_bottom():
    # clean=False throughout, one heal at the end (#932 efficiency
    # review — the same fix _build_frame_top got at #910: cq's default
    # re-runs OCC unify over the whole part after EVERY boolean, and
    # this is the project's largest part)
    fb = wall_band                        # the ONE fused wall band, bed →
                                          # the lid-top plane (wall.py)
    fb = fb.union(_floor(), clean=False)
    fb = fb.union(_lever_pad(), clean=False)
    fb = fb.union(_bay_web(+1.0), clean=False)   # −y web went with the
                                   # brake (#1019): no bay window cuts
                                   # that side any more, so there is no
                                   # band end to seal and no column to
                                   # seal it to — it stood as a blade
    # the 4 sites' beams/tenons/fillers as ONE compound → one fuse
    # instead of 12 whole-part unions
    site_solids = []
    for a in (0.0, 90.0, 180.0, 270.0):
        for w in (_beam(a), _arc_tenon(a), _beam_filler(a)):
            site_solids.append(w.val())
    fb = fb.union(cq.Workplane(obj=cq.Compound.makeCompound(site_solids)),
                  clean=False)
    fb = fb.union(_lever_mount(), clean=False)
    fb = fb.union(_ratchet_stop(), clean=False)
    fb = fb.union(_HORN_BOTTOM, clean=False)
    for c in _lock_copies(_lock_corbel_bot()):
        fb = fb.union(c, clean=False)
    fb = fb.cut(_hand_wedge(), clean=False)
    fb = fb.cut(_lever_pin_bores(), clean=False)
    fb = fb.union(_clip_posts(), clean=False)
    for c in _lock_copies(_lock_slot_cut(-LOCK_BOTB_H - 1.0, 1.0)):
        fb = fb.cut(c, clean=False)
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
                    (ANCH_OR, FRAME_TOP_Z1), (ANCH_IR, FRAME_TOP_Z1)])
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
                      (y1 - NOZZLE_D, ANCH_TIP_Z)])    # ...the dulled tip flat
           .close().extrude((ANCH_OR - x_in) + 1.0))   # (close = the 45° floor)
    return w.cut(win)


def _build_frame_top():
    # +z end grows COUCH_TOP_PAD in the couch version (params block):
    # the arms, boss and anchor wall rise to FRAME_TOP_Z1; everything
    # cut from the TOP face (mount channels, lock slot, pocket) follows
    ft = _plus(FRAME_Z0, FRAME_TOP_Z1)
    ft = ft.union(cyl(BRG_BOSS_OD, FRAME_TOP_Z1 - FRAME_Z0, z=FRAME_Z0))
    ft = ft.union(_anchor_wall())   # tension-trap strip anchor + arm rib
    for c in _lock_copies(_lock_corbel_top()):
        ft = ft.union(c, clean=False)
    # clean=False through the cut loops: cq's default runs OCC's
    # unify-same-domain over the WHOLE part after EVERY cut — profiled
    # (#910, py-spy) as the frame build's runaway cost. heal(ft) at the
    # end is the one cleanup pass.
    for a in (0.0, 90.0, 180.0, 270.0):
        ft = ft.cut(_arc_mortise(a), clean=False)
    # mount channels: the v-grooves and the cavity slots SHARE a wall by
    # design (2.7 = 2.7 — one straight surface, user #911); the groove
    # arcs stop short of the cavity spans (mount_channel_cuts) so no two
    # cutters ever carve float-identical walls — OCC's pathological
    # unify case (#910, 15+ min builds).
    for c in mount_channel_cuts():  # flush mount channels in the ±Y arms
        ft = ft.cut(c.translate((0.0, 0.0, COUCH_TOP_PAD)), clean=False)
    for c in _lock_copies(_lock_slot_cut(-1.0, FRAME_TOP_Z1)):
        ft = ft.cut(c, clean=False)
    # lip bore through, the 45° FUNNEL seat (flipped-print cone — the
    # outer race's chamfer lands face-on-face on it, params block), then
    # the pocket from the funnel rim to the frame top
    ft = ft.cut(cyl(BRG_LIP_ID, (FRAME_TOP_Z1 - FRAME_Z0) + 1.0,
                    z=FRAME_Z0 - 0.5), clean=False)
    ft = ft.cut(cone_solid(d_bottom=BRG_LIP_ID, d_top=BRG_BORE,
                           h=BRG_SEAT_CONE_Z1 - BRG_SEAT_CONE_Z0,
                           z_base=BRG_SEAT_CONE_Z0), clean=False)
    ft = ft.cut(cyl(BRG_BORE, (FRAME_TOP_Z1 - BRG_SEAT_CONE_Z1) + 0.5,
                    z=BRG_SEAT_CONE_Z1), clean=False)
    # the groove-arc lap bands imprint orphan open-shell debris where
    # they die inside the cavity slots — rebuild from closed shells
    return drop_stray_shells(heal(ft))


# horn split once (bottom half fuses into frame_bottom; the cap is its
# own printed part, exported from build.py)
_HORN_BOTTOM, _HORN_CAP = _cable_horn()

frame_bottom = _build_frame_bottom()
frame_top = _build_frame_top()
horn_cap = heal(_HORN_CAP)

# the TPU frame-lock pin (user #939, params LOCK block), modeled in
# place at full insert: top flush with the frame face, clearance all
# around (#948 — the mount head WRAPS its −x side again, the wall
# that actually locks it; the #942 flush-on-the-beam-wall pin is
# reverted), tip LOCK_PIN_GRIP proud below the beam corbel for the
# pliers grab. Free in z — the desk caps +z once mounted; goes in
# from BELOW, after the mount rotation.
lock_pin_tpu = _lock_place(_lock_prism(LOCK_PIN_Z0, LOCK_PIN_Z0 + LOCK_PIN_L,
                                       LOCK_PIN_CLR))
# one printed PART, installed at every lock site (#983): the assembly
# and the overlap gate place these; the STEP export stays the one part
lock_pins_in_place = _lock_copies(lock_pin_tpu)
