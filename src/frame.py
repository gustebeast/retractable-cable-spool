"""FRAME — split frame_top / frame_bottom, v2's architecture ported to v3.

frame_top — plus spider carrying the 608 IN its own thickness (recessed on
    a quality-tier lip → flat assembly top), the spring-strip TWIST-LOCK
    anchor wall under the +X arm (its teardrop-hole + 45°-slot window
    passes the strip's T-head only twisted ~45° — params block), and the arc-joint MORTISE
    channels in its underside. PRINTS INVERTED (+Z→−Z, top face on the
    bed — the anchor wall rises rooted on its rib band): the arc cavities
    open at the print TOP, so they close with plain floors — no bridges
    at all (the mount lesson; v2 printed this part upright and needed the
    no-roof-bridge profile).

frame_bottom — NO bottom plus anymore (user's call, #815): the WALL's
    lower band is FUSED in as the beams' structural tie, closed by a
    sparse SPOKED FLOOR (1.6 plate/spokes — the coil chamber's bottom;
    just enough web that the cable can't dip through and tangle) — the
    whole stack sits ~11 shorter for it. Four beams rise from the bed to
    the top plus's underside, carrying the sliding wall pieces' T
    mortises (channel stops just under the fused band's top) and the arc
    TENONS on their tops. The v2 LEVER MOUNT simplified to match: with
    the floor fused, nothing has to drop past the +X arm anymore, so the
    fans/45°-climbs are gone — the fork COLUMNS stand straight on a foot
    PAD (floor-thick, wall→frame-extent under the lever bays), with v2's
    arch struts, gussets, thrust-boss rings and diamond pin bores
    unchanged. Prints −Z→+Z (floor plate on the bed).

INSTALL (v2's): wall pieces slide down the beam channels; then frame_top
rotates on: offset ~20° CCW, z-mate through the open quadrants, rotate CW
to the angular stops (retraction torque is CW → operation preloads the
joints; uninstall = CCW against load).
"""

import math

import cadquery as cq

from cadkit.joinery import joint
from cadkit.contact import contact_ring
from cadkit.holes import teardrop_hole

from .helpers import cone_solid, cyl, heal
from .lid_joint import arrowhead_solid, TOPJ_TOP, TOPJ_RD
from .wall import wall_bottom_band
from .params import (
    NOZZLE,
    FRAME_RIB, FRAME_R_OUT, FRAME_Z0, FRAME_Z1,
    ANCH_IR, ANCH_OR, ANCH_T, ANCH_Z0, ANCH_HALF_A,
    ANCH_ZC, ANCH_HOLE_D, ANCH_SLOT_L, ANCH_SLOT_W,
    BRG_BORE, BRG_POCKET_Z0, BRG_LIP_ID, BRG_BOSS_OD,
    WALL_IR, WALL_OR, WALL_SPLIT_Z, BEAM_IR, BEAM_SIZE, BEAM_Z0, BEAM_Z1,
    FLOOR_Z0, FLOOR_Z1, FLOOR_SPOKE_N, FLOOR_SPOKE_W,
    FLOOR_HUB_R, FLOOR_RING_RC,
    SLEEVE_OD, SLEEVE_T, SLEEVE_FLARE, SLEEVE_Z1, SLEEVE_TOP_CH,
    STUB_D, STUB_BOSS_D, STUB_BOSS_Z1, STUB_Z1, STUB_FLARE, STUB_TIP_CH,
    JOINT_SPEC, JOINT_WIDTH, JOINT_DEPTH, JOINT_CLR, JOINT_BACK_CLR,
    JOINT_SEAT_CLR, MORTISE_L,
    TOPJ_STEM, TOPJ_FLARE,
    TOP_JOINT_SEAT_CLR, TOP_ENTRY_OVER, TOP_STOP_WALL,
    LEVER_PIVOT_X, RATCHET_PIVOT_Z, BRAKE_PIVOT_Z,
    RATCHET_LEV_Y1, LEVER_SIDE_CLR, LEVER_BOSS_OD,
    PIN_SQ_S, PIN_SQ_FRAME_CLR, PIN_KEY_BASE_DEG,
    POST_OUT_T, PIN_TIP_END_Y, BOSS_BORE_ID,
)

# ── Wall ↔ beam T joint (cadkit — v2's print-proven box) ─────────────────────
_WALL_JOINT = joint(JOINT_WIDTH, MORTISE_L, tenon=JOINT_SPEC,
                    mortise=JOINT_SPEC, install="-z", depth=JOINT_DEPTH)

# ── frame_top ↔ frame_bottom arc joints (shared arrowhead profile at the
# beam-face radius — radially just outside the wall channels' keep-out) ──────
_TOPJ_R0 = BEAM_IR + _WALL_JOINT.height + JOINT_CLR

assert (BEAM_IR + BEAM_SIZE) - (_TOPJ_R0 + TOPJ_RD) >= 2 * NOZZLE - 1e-9, \
    "half-arrowhead cavity's outer wall under 1.6 at the flush arm end"
assert FRAME_RIB - (TOPJ_TOP + JOINT_BACK_CLR) >= 2 * NOZZLE - 1e-9, \
    "arc-joint cavity ceiling under 1.6 in the top plus"

_R_T = _TOPJ_R0 + (TOPJ_STEM + TOPJ_FLARE) / 2.0
_ARM_HALF_A = math.degrees(math.asin((BEAM_SIZE / 2.0) / _R_T))
_SEAT_A     = math.degrees(TOP_JOINT_SEAT_CLR / _R_T)
_STOP_A     = math.degrees(TOP_STOP_WALL / _R_T)
_OVER_A     = math.degrees(TOP_ENTRY_OVER / _R_T)
_TEN_HALF_A = _ARM_HALF_A - _STOP_A - _SEAT_A

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


def _wall_mortise(angle_deg):
    """T mortise channel in a beam's inner face: open at the beam top (the
    sliding wall pieces enter there) to a hard stop JOINT_SEAT_CLR below
    the fused band's top — wall_top's tenon seats there while its ring
    face lands on the band."""
    return (_WALL_JOINT.mortise(drop=2.0)
            .translate((WALL_OR, 0, WALL_SPLIT_Z - JOINT_SEAT_CLR))
            .rotate((0, 0, 0), (0, 0, 1), angle_deg))


def _arc_tenon(site_deg):
    """Arc tenon on a beam top, seated CENTERED on its arm, root sunk 1.0."""
    return (arrowhead_solid(True, 2.0 * _TEN_HALF_A, -1.0, _TOPJ_R0)
            .rotate((0, 0, 0), (0, 0, 1), site_deg - _TEN_HALF_A)
            .translate((0, 0, BEAM_Z1)))


def _arc_mortise(site_deg):
    """Matching arc channel in frame_top's underside: the CW end wall (the
    STOP) sits TOP_STOP_WALL inside the arm's CW face; the CCW end sweeps
    open past the arm's CCW face (the entry — the tenon rotates in CW from
    the open quadrant)."""
    sweep = (_TEN_HALF_A + _SEAT_A) + (_ARM_HALF_A + _OVER_A)
    return (arrowhead_solid(False, sweep, -2.0, _TOPJ_R0)
            .rotate((0, 0, 0), (0, 0, 1), site_deg - _TEN_HALF_A - _SEAT_A)
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
# arch-turn start: underside clears the swept lever boss by 1 (v2 hardcoded
# the swept value; the boss top is swing-invariant = pivot + boss/2)
_ARCH_Z0 = RATCHET_PIVOT_Z + LEVER_BOSS_OD / 2.0 + 1.0           # ≈ −18.5


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
    over the band's height, ID trimmed to the bore cylinder (curved on
    the ID, solid into the beam). The T channel's stop cut comes after
    and re-opens its 0.15 at the filler's top."""
    f = (_box(66.0, BEAM_IR + 1.0, -BEAM_SIZE / 2.0, BEAM_SIZE / 2.0,
              FLOOR_Z0, WALL_SPLIT_Z)
         .cut(cyl(2.0 * WALL_IR, (WALL_SPLIT_Z - FLOOR_Z0) + 1.0,
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
    same constants)."""
    _col_top = _ARCH_Z0 + 2.0 * POST_OUT_T / math.sqrt(2.0) - POST_OUT_T
    m = _side_column(+1.0, _col_top)
    m = m.union(_side_column(-1.0, _col_top))
    m = m.union(_arch(+1.0, _ARCH_Z0))
    m = m.union(_arch(-1.0, _ARCH_Z0))
    # centre-arm gussets: the beam spreads 45° into each arch
    _rise = _SB_Y0 - BEAM_SIZE / 2.0
    for s in (+1.0, -1.0):
        m = m.union(_yz_prism(
            [(s * BEAM_SIZE / 2.0, _ARCH_Z0),
             (s * (BEAM_SIZE / 2.0 + _rise / 2.0), _ARCH_Z0 + _rise / 2.0),
             (s * BEAM_SIZE / 2.0, _ARCH_Z0 + _rise / 2.0)],
            BEAM_IR, _R_OUT))
    # the arm turns back down 45° into the beam: bevel keeping the band
    # POST_OUT_T thick perpendicular from the ^ apex (v2's yellow == blue)
    _half = _rise / 2.0
    _k = ((_ARCH_Z0 + _half) - (_SB_Y0 - _half) + POST_OUT_T * math.sqrt(2.0))
    _y2 = BEAM_SIZE / 2.0 + _half + 1.0
    for s in (+1.0, -1.0):
        m = m.cut(_yz_prism(
            [(s * BEAM_SIZE / 2.0, BEAM_SIZE / 2.0 + _k),
             (s * BEAM_SIZE / 2.0, BEAM_Z1 + 1.0),
             (s * _y2, BEAM_Z1 + 1.0),
             (s * _y2, _y2 + _k)],
            BEAM_IR - 1.0, _R_OUT + 1.0))
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
    fb = wall_bottom_band                 # the fused lower wall (its tenon
                                          # segments weld it into the beams)
    fb = fb.union(_floor())
    fb = fb.union(_lever_pad())
    fb = fb.union(_bay_web(+1.0)).union(_bay_web(-1.0))
    for a in (0.0, 90.0, 180.0, 270.0):
        fb = fb.union(_beam(a)).union(_arc_tenon(a)).union(_beam_filler(a))
    fb = fb.union(_lever_mount())
    fb = fb.cut(_lever_pin_bores())
    for a in (0.0, 90.0, 180.0, 270.0):
        fb = fb.cut(_wall_mortise(a))     # channels for the SLIDING pieces
                                          # only — they stop at the band top
    return heal(fb)


def _anchor_wall():
    """Spring-strip TWIST-LOCK anchor (params block): a short wall sector
    centred under the +X arm, ANCH_Z0 up THROUGH the frame (the over-frame
    band is the arm rib and the wall's bed-rooted print seat — the gate
    wall's proven pattern). The pass/block window is OVERHANG-FREE in the
    inverted print (user's call): a cadkit teardrop neck hole (apex toward
    assembly −z = print-up) crossed by the 45° insertion slot, whose long
    sides and square end caps all lie at 45° to the layers."""
    w = (cq.Workplane("XZ")
         .polyline([(ANCH_IR, ANCH_Z0), (ANCH_OR, ANCH_Z0),
                    (ANCH_OR, FRAME_Z1), (ANCH_IR, FRAME_Z1)])
         .close().revolve(2.0 * ANCH_HALF_A, (0, 0), (0, 1))
         .rotate((0, 0, 0), (0, 0, 1), -ANCH_HALF_A))
    w = w.cut(teardrop_hole(ANCH_HOLE_D, ANCH_T + 2.0,
                            (ANCH_IR - 1.0, 0.0, ANCH_ZC),
                            (1.0, 0.0, 0.0), print_up=(0.0, 0.0, -1.0)))
    slot = (cq.Workplane("XY")
            .box(ANCH_T + 4.0, ANCH_SLOT_L, ANCH_SLOT_W)
            .rotate((0, 0, 0), (1, 0, 0), 45.0)
            .translate((ANCH_IR + ANCH_T / 2.0, 0.0, ANCH_ZC)))
    return w.cut(slot)


def _build_frame_top():
    ft = _plus(FRAME_Z0, FRAME_Z1)
    ft = ft.union(cyl(BRG_BOSS_OD, FRAME_Z1 - FRAME_Z0, z=FRAME_Z0))
    ft = ft.union(_anchor_wall())   # twist-lock strip anchor + arm rib
    for a in (0.0, 90.0, 180.0, 270.0):
        ft = ft.cut(_arc_mortise(a))
    # lip bore through, then the pocket from the frame top down to the lip
    ft = ft.cut(cyl(BRG_LIP_ID, (FRAME_Z1 - FRAME_Z0) + 1.0, z=FRAME_Z0 - 0.5))
    ft = ft.cut(cyl(BRG_BORE, (FRAME_Z1 - BRG_POCKET_Z0) + 0.5, z=BRG_POCKET_Z0))
    return heal(ft)


frame_bottom = _build_frame_bottom()
frame_top = _build_frame_top()
