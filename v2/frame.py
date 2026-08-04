"""FRAME — split into frame_bottom + frame_top, joined by octagon slide
joints that install horizontally (retention along Z, the load direction).

  frame_bottom — bottom plus-spider (below the spring housing, 3.2 mm clear) +
      the four vertical beams rising from it. The beams carry the WALL's
      dovetail mortises (install="z") and octagon TENONS on their top faces.
      Prints −Z→+Z (plus on the bed, beams up — tenons in the library's native
      orientation).
  frame_top — the top plus-spider only, with the matching octagon MORTISE
      channels in its underside. Prints −Z→+Z (flat plus on the bed — its
      future desk-mount hardware lands on a clean build face). Installs by
      sliding from −x toward +x over the beam tenons; each cavity's −x end
      wall is the hard stop.
"""

import math

import cadquery as cq

from v2.helpers import cyl, heal
from cadkit.joinery import PrintSpec, joint
from cadkit.contact import contact_ring
from .axle import rod_hole
from .mount import mount_channel_cuts
from .params import (
    WALL_ZB, WALL_OR, BEAM_IR, BEAM_SIZE, BEAM_Z0, BEAM_Z1,
    TOP_RIB_SIZE, TOP_RIB_Z0, TOP_RIB_Z1, BOT_RIB_Z0, BOT_RIB_Z1,
    JOINT_WIDTH, JOINT_DEPTH, JOINT_CLR, JOINT_SEAT_CLR, MORTISE_L,
    TOPJ_STEM, TOPJ_FLARE, TOPJ_NECK, TOPJ_TIP,
    JOINT_BACK_CLR, MOUNT_LOCK_D,
    TOP_JOINT_SEAT_CLR, TOP_ENTRY_OVER,
    TOP_WALL_CLEAR, TOP_STOP_WALL, AXLE_BORE_D, ROD_Z, ROD_Z_BOT,
    LEVER_PIVOT_X, RATCHET_PIVOT_Z, BRAKE_PIVOT_Z, LEVER_HANDLE_W,
    RATCHET_LEV_Y0, RATCHET_LEV_Y1, BRAKE_LEV_Y0, BRAKE_LEV_Y1,
    PIN_SQ_S, PIN_SQ_FRAME_CLR, PIN_KEY_BASE_DEG,
    POST_OUT_T, PIN_TIP_END_Y, LEVER_SIDE_CLR,
    BOSS_BORE_ID, CLIMB_X0,
    NOZZLE,
)

_UP = PrintSpec(nozzle=NOZZLE, material="PETG-GF", facing="up")
# clearances come from the LIBRARY's material policy now (user's call): the
# GF material doubles the T's depth-face gap — see params' JOINT_CLR block
# Wall ↔ beams: installs along Z (see wall.py — same width/depth/clr).
_WALL_JOINT = joint(JOINT_WIDTH, MORTISE_L, tenon=_UP, mortise=_UP,
                    install="-z", depth=JOINT_DEPTH)  # wall seats travelling −z
# frame_top ↔ frame_bottom: ROTATIONAL HALF-ARROWHEAD joints (manual —
# the cadkit octagon arc printed CONSIDERABLY too tight and pushed the
# top arms proud of the frame extent; user redesign, see params TOPJ_*).
# Profile in the r-z plane, revolved: FLAT inner face hard against the
# wall-joinery keep-out (the cavity's flat lands exactly ON it), stem,
# ONE outboard 45° flare (the retention lip — lift/hang cams the tenon
# INBOARD onto the flat, which blocks flat-on-flat, the cadkit hook
# lesson), then a 45° taper to a TOPJ_TIP dull tip AT the flat: the
# cavity has NO roof bridge in frame_top's upright print. Lateral faces
# run JOINT_CLR; the z-sandwich pairs run JOINT_BACK_CLR (2× — the T
# joint's print-validated GF depth rule, user's call).
_TOPJ_R0  = BEAM_IR + _WALL_JOINT.height + JOINT_CLR   # tenon FLAT face radius
_TOPJ_TOP = TOPJ_NECK + TOPJ_FLARE + (TOPJ_STEM + TOPJ_FLARE - TOPJ_TIP)
_TOPJ_RD  = (_TOPJ_TOP + TOPJ_TIP + TOPJ_STEM - TOPJ_NECK
             + 2.0 * JOINT_BACK_CLR) / 2.0        # cavity flare∩taper corner (rel)


def _topj_pts(tenon, z_base):
    """(r, z) profile points. tenon=True → nominal; False → the CAVITY:
    lateral faces dilated JOINT_CLR, z-sandwich faces JOINT_BACK_CLR."""
    s2, f, zn, tp = TOPJ_STEM, TOPJ_FLARE, TOPJ_NECK, TOPJ_TIP
    zt = _TOPJ_TOP
    if tenon:
        return [(_TOPJ_R0, z_base), (_TOPJ_R0 + s2, z_base),
                (_TOPJ_R0 + s2, zn), (_TOPJ_R0 + s2 + f, zn + f),
                (_TOPJ_R0 + tp, zt), (_TOPJ_R0, zt)]
    c, bc = JOINT_CLR, JOINT_BACK_CLR
    return [(_TOPJ_R0 - c, z_base), (_TOPJ_R0 + s2 + c, z_base),
            (_TOPJ_R0 + s2 + c, zn + c - bc),
            (_TOPJ_R0 + _TOPJ_RD, _TOPJ_RD - s2 + zn - bc),
            (_TOPJ_R0 + tp, zt + bc), (_TOPJ_R0 - c, zt + bc)]


def _topj_solid(tenon, sweep, z_base):
    return (cq.Workplane("XZ").polyline(_topj_pts(tenon, z_base)).close()
            .revolve(sweep, (0, 0), (0, 1)))


# every printed wall around the cavity keeps the 1.6 tier (user's rule),
# with the arms FLUSH at the frame extent (user's call — mounting surface)
assert (BEAM_IR + BEAM_SIZE) - (_TOPJ_R0 + _TOPJ_RD) >= 2 * NOZZLE - 1e-9, \
    "half-arrowhead cavity's outer wall under 1.6 at the flush arm end"
assert ((TOP_RIB_Z1 - MOUNT_LOCK_D)
        - (BEAM_Z1 + _TOPJ_TOP + JOINT_BACK_CLR)) >= 2 * NOZZLE - 1e-9, \
    "half-arrowhead cavity ceiling under 1.6 to the mount lock groove"

# Arc-joint angular geometry, DERIVED (reference radius = profile mid).
_R_T = _TOPJ_R0 + (TOPJ_STEM + TOPJ_FLARE) / 2.0
_ARM_HALF_A = math.degrees(math.asin((BEAM_SIZE / 2.0) / _R_T))   # arm face angle
_SEAT_A     = math.degrees(TOP_JOINT_SEAT_CLR / _R_T)
_STOP_A     = math.degrees(TOP_STOP_WALL / _R_T)
_OVER_A     = math.degrees(TOP_ENTRY_OVER / _R_T)
_TEN_HALF_A = _ARM_HALF_A - _STOP_A - _SEAT_A                     # tenon half-sweep

_R_OUT = BEAM_IR + BEAM_SIZE                     # plus-arm outer radius
_BEAM_RC = BEAM_IR + BEAM_SIZE / 2.0             # beam centre radius
def _plus(z0, z1, r_out=_R_OUT):
    """Plus-shaped spider: two crossed TOP_RIB_SIZE-wide bars spanning ±r_out."""
    h = z1 - z0
    bar_x = (cq.Workplane("XY").workplane(offset=z0)
             .rect(2 * r_out, TOP_RIB_SIZE).extrude(h))
    bar_y = (cq.Workplane("XY").workplane(offset=z0)
             .rect(TOP_RIB_SIZE, 2 * r_out).extrude(h))
    return bar_x.union(bar_y)


def _beam(angle_deg):
    return (cq.Workplane("XY").workplane(offset=BEAM_Z0)
            .center(_BEAM_RC, 0)
            .rect(BEAM_SIZE, BEAM_SIZE).extrude(BEAM_Z1 - BEAM_Z0)
            .rotate((0, 0, 0), (0, 0, 1), angle_deg))


def _wall_mortise(angle_deg):
    """Dovetail mortise channel in a beam's inner face (see wall.py): runs OPEN
    from the beam top (the wall enters there and slides down — the beams root
    in the bottom plus, so there's no bottom entry) to a hard stop
    JOINT_SEAT_CLR below the seated wall tenon (whose bottom now sits at the
    skirt bottom WALL_ZB); the wall rests on the stops."""
    return (_WALL_JOINT.mortise(drop=2.0)
            .translate((WALL_OR, 0, WALL_ZB - JOINT_SEAT_CLR))
            .rotate((0, 0, 0), (0, 0, 1), angle_deg))


def _arc_tenon(site_deg):
    """Arc tenon on a beam top, seated CENTERED on its arm: the
    half-arrowhead profile swept ±_TEN_HALF_A about the site angle,
    root sunk 1.0 into the beam top."""
    return (_topj_solid(True, 2.0 * _TEN_HALF_A, -1.0)
            .rotate((0, 0, 0), (0, 0, 1), site_deg - _TEN_HALF_A)
            .translate((0, 0, BEAM_Z1)))


def _arc_mortise(site_deg):
    """Matching arc channel in frame_top's underside: the CW end wall (the
    STOP, _SEAT_A past the seated tenon) sits _STOP_A inside the arm's CW
    face; the CCW end sweeps open past the arm's CCW face (the entry — the
    tenon rotates in CW from the open quadrant)."""
    sweep = (_TEN_HALF_A + _SEAT_A) + (_ARM_HALF_A + _OVER_A)
    return (_topj_solid(False, sweep, -2.0)
            .rotate((0, 0, 0), (0, 0, 1), site_deg - _TEN_HALF_A - _SEAT_A)
            .translate((0, 0, BEAM_Z1)))


def _box(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane("XY").workplane(offset=z0)
            .polyline([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
            .close().extrude(z1 - z0))


def _yz_prism(pts_yz, x0, x1):
    """Closed YZ-profile prism spanning [x0, x1] — for the ramped brackets."""
    return (cq.Workplane("YZ").polyline(pts_yz).close()
            .extrude(x1 - x0).translate((x0, 0, 0)))


# ── Lever support: the +X arm SPLITS INTO THREE climbing branches ────────────
# (User sketch.) No mounting pad — the two handles hang in the OPEN gaps
# between branches, fully grabbable from below. Each branch runs flat on the
# plate, then CLIMBS at 45° starting where its top face will clear the cable
# floor's LOWEST position (floor clamp bottomed on the hub base → floor
# underside at FLOOR_MIN_BOT_Z) by STATIC_MOVE_CLR both radially and in z:
# a 45° top line through (floor_edge_x + clr, floor_bot) ⇒ climb starts at
# floor_edge_x + clr − (floor_bot − BOT_RIB_Z1). At the frame's +X extent the
# branches rise PURE VERTICAL: the side columns ARE the outer pivot posts;
# the centre branch merges up into the +X beam.
_SB_Y0 = RATCHET_LEV_Y1 + LEVER_SIDE_CLR         # 17.0 — side-branch inner face
_SB_Y1 = _SB_Y0 + POST_OUT_T                     # 23.0 — side-branch outer face

# ONE shared climb start for all three branches. Its anchor CHANGED with the
# axial chamber (the old rule cleared the tray floor's lowest position; the
# tray is gone): the climb now starts just OUTSIDE the wall/cup band
# (CLIMB_X0, params) so the coil cup drops past the flat runs to the
# lowered plus — the runs pass entirely UNDER the chamber.
_XC = CLIMB_X0                                   # ≈ 78.0 — where the TOP face bends
# the UNDERSIDE bends t·(√2−1) later (constant-perpendicular-thickness band —
# see _climb_prism); the beam's foot trim follows this line
_XB = _XC + (BOT_RIB_Z1 - BOT_RIB_Z0) * (math.sqrt(2.0) - 1.0)   # ≈ 73.04
_X_SPLIT = _XC - (_SB_Y1 - BEAM_SIZE / 2.0)      # plan fan-out (45°) ends at the climb
_ARCH_TOP = BEAM_Z1                              # ceiling for arch struts — flush with the
                                                 # beam tops (frame_top's underside already
                                                 # slides on that plane; coincident is fine)
# Arch-turn start heights (column inner face): underside must clear the
# lever's SWEPT top by ≥1 mm at the lever's outer edge. Ratchet = Ø12 boss
# top 40.5 (swing-invariant) + 1 clr − 1 y-offset; brake = swept arm top
# 31.4 + 1 − 1.
_ARCH_Z0_RAT = 40.5
_ARCH_Z0_BRK = _ARCH_Z0_RAT                      # raised to the ratchet's height: the
                                                 # TALL brake pad arm (top 32.6, swept
                                                 # ~33) outgrew the old 31.5 underside


def _arch(s, z_b0):
    """CONSTANT-SECTION bent side arm (user's call: half the beam's width →
    POST_OUT_T × 10): the column continues as a 45° diagonal band, then
    ROTATES TO HORIZONTAL at the ^ apex height (where it meets the centre
    arm's gusset) and runs into the beam's side face. Perpendicular
    thickness = POST_OUT_T through every bend, so the material band above/
    outside the ^ opening is uniform. Bottom edge sinks into the column for
    a solid fuse."""
    t = POST_OUT_T
    y5 = BEAM_SIZE / 2.0
    half = (_SB_Y0 - y5) / 2.0                   # 6 — the 45° y-run to the apex
    z_apex = z_b0 + half                         # 46.5 — the ^ top
    off = t / math.sqrt(2.0)                     # 3.54 — perpendicular offset
    pts = [(_SB_Y0, z_b0),                       # arch start on the column inner face
           (_SB_Y0 - half, z_apex),              # the ^ apex (inner bend)
           (y5, z_apex),                         # beam face — underside of the run
           (y5, z_apex + t),                     # beam face — top of the run
           (_SB_Y0 + 2.0 * off - half - t, z_apex + t),   # outer top bend
           (_SB_Y1, z_b0 + 2.0 * off - t),       # outer face meets the diagonal
           (_SB_Y1, z_b0 - 4.5),                 # sink into the column body
           (_SB_Y0, z_b0 - 4.5)]
    return _yz_prism([(s * y, z) for y, z in pts], BEAM_IR, _R_OUT)


def _climb_prism(x_c, y_a, y_b):
    """45° climbing band in XZ from the flat run to the frame's +X extent,
    the arm's FULL thickness PERPENDICULAR to the slope (user's constant-
    section rule — the old parallelogram read 10/√2 ≈ 7.07 across the
    diagonal). The TOP face bends up at x_c (it is the derived floor-
    clearance plane — unchanged); the UNDERSIDE bends t·(√2−1) later so the
    two 45° faces sit a true t apart — the band grows DOWNWARD, so the
    floor keep-out and the fork top stay exactly as they were. Meets the
    flat run face-to-face at x_c (full-face coplanar fuse — verified to
    merge into one solid)."""
    t = BOT_RIB_Z1 - BOT_RIB_Z0
    x_b = x_c + t * (math.sqrt(2.0) - 1.0)       # underside bend (≈ x_c + 4.14)
    return (cq.Workplane("XZ")
            .polyline([(x_c, BOT_RIB_Z0), (x_b, BOT_RIB_Z0),
                       (_R_OUT, BOT_RIB_Z0 + (_R_OUT - x_b)),
                       (_R_OUT, BOT_RIB_Z1 + (_R_OUT - x_c)),
                       (x_c, BOT_RIB_Z1)])
            .close()
            .extrude(-abs(y_b - y_a)).translate((0, min(y_a, y_b), 0)))


def _side_branch(s, pz, col_top):
    """One side branch (s=±1): plan 45° fan off the arm + flat band (on the
    plate), the 45° climb, and the vertical COLUMN = the lever's outer pivot
    post (out to the frame's +X extent, POST_OUT_T wide), rising to where its
    arch strut departs."""
    y5 = s * BEAM_SIZE / 2.0
    # plan 45° fan, POST_OUT_T PERPENDICULAR width (user-caught: parallel
    # edges landing on the column's two faces at the same x gave only
    # POST_OUT_T/√2 ≈ 3.54): the outer edge is offset a true POST_OUT_T·√2
    # along the axes and mitres into the column's outer face just before the
    # climb start — a pentagon instead of the old too-thin quad
    _po = POST_OUT_T * math.sqrt(2.0)            # 7.07 — axis offset for 5 perp
    fan = (cq.Workplane("XY").workplane(offset=BOT_RIB_Z0)
           .polyline([(_X_SPLIT + POST_OUT_T - _po, y5),
                      (_XC - (_po - POST_OUT_T), s * _SB_Y1),
                      (_XC, s * _SB_Y1),
                      (_XC, s * _SB_Y0),
                      (_X_SPLIT + POST_OUT_T, y5)])
           .close().extrude(BOT_RIB_Z1 - BOT_RIB_Z0))
    climb = _climb_prism(_XC, s * _SB_Y0, s * _SB_Y1)
    col_z0 = BOT_RIB_Z0 + (_R_OUT - _XC) + 0.1   # rests fully on the climb
    col = _box(BEAM_IR, _R_OUT, min(s * _SB_Y0, s * _SB_Y1),
               max(s * _SB_Y0, s * _SB_Y1), col_z0, col_top)
    # bevel the column's top-outer corner 45°, continuing the arch strut's
    # top plane outward — no nub above the strut junction
    bevel = _yz_prism([(s * _SB_Y0, col_top),
                       (s * _SB_Y1, col_top - (_SB_Y1 - _SB_Y0)),
                       (s * _SB_Y1, col_top + 2.0),
                       (s * _SB_Y0, col_top + 2.0)],
                      BEAM_IR - 1.0, _R_OUT + 1.0)
    col = col.cut(bevel)
    return fan.union(climb).union(col)


def _thrust_boss(y_face, grow, pz, bore_id, clip_x=None):
    """The lever's ONLY side contact: cadkit's CONTACT RING (2·nozzle =
    1.6 wide AND proud — the library owns the rule) with its 45° teardrop
    tail fused on for the −Z→+Z print. Grows from `y_face` in the `grow`
    (±1) direction; 1.6 proud into the 1.8 side gap leaves 0.2 nominal
    float to the lever's face. `clip_x` flattens the boss's −x side
    (the beam-flank bosses must stay out of the wall ring's slide path)."""
    boss = contact_ring(bore_id, (LEVER_PIVOT_X, y_face, pz),
                        (0.0, grow, 0.0), nozzle=NOZZLE)
    # Bore the BOSS SOLID itself round before it unions to the frame: the
    # contact ring is born bored, but its fused teardrop TAIL is not, and
    # the diamond pin cutter alone leaves four crown slivers inside the
    # bore envelope — the TWISTED torsion pin's corners sweep exactly
    # there (0.85 mm³ at rest, probe-caught). Cutting the boss (not the
    # frame) keeps the host face pristine — no counterbore shelf (the
    # reason the old through-teardrop was removed). The cleared crown is
    # a ~1.6 bridge in print, same as the ring's own ceiling.
    bore = cq.Solid.makeCylinder(
        bore_id / 2.0, LEVER_SIDE_CLR + 2.0,
        cq.Vector(LEVER_PIVOT_X, y_face - grow * 1.0, pz),
        cq.Vector(0.0, grow, 0.0))
    boss = boss.cut(cq.Workplane(obj=bore))
    if clip_x is not None:
        # bound by the WALL's swept cylinder (+margin) rather than a flat
        # chop — the wall's solid band passes here during its slide-down, so
        # the boss hugs that surface as closely as legally possible
        boss = boss.cut(cyl(2.0 * clip_x, 24.0, z=pz - 12.0))
    return boss


def _lever_mount():
    """JUST the three branch structures (user's call — no extra prisms beside
    the levers), plus the per-pivot thrust bosses and the brake rest tab (the
    torsion preload holds the brake there; the ratchet's rest stop is its own
    pawl seated in the teeth). The pin tips bear in blind bores in the +X
    BEAM's flanks — no inner posts. Flats, verticals + 45° ramps only →
    −Z→+Z support-free."""
    # columns end where the bent arm's outer diagonal reaches their outer
    # face (the arm band takes over above; its base sinks in to fuse)
    _col_top = _ARCH_Z0_RAT + 2.0 * POST_OUT_T / math.sqrt(2.0) - POST_OUT_T
    m = _side_branch(+1.0, RATCHET_PIVOT_Z, _col_top)
    m = m.union(_side_branch(-1.0, BRAKE_PIVOT_Z, _col_top))
    # centre branch: the arm's flat run is trimmed past its climb start (see
    # _build_frame_bottom) and this diagonal carries it up into the beam.
    m = m.union(_climb_prism(_XC, -BEAM_SIZE / 2.0, BEAM_SIZE / 2.0))
    # overhead arch struts: columns rejoin the beam over the levers (both-side
    # support closed into a loop)
    m = m.union(_arch(+1.0, _ARCH_Z0_RAT))
    m = m.union(_arch(-1.0, _ARCH_Z0_BRK))
    # centre-arm GUSSETS (user's sketch): the beam SPREADS at 45° into each
    # arch, filling the V between its y-face and the arch's 45° underside —
    # triangle per side: vertical edge on the beam face, underside rising at
    # 45° off the face (self-supporting), top edge fused along the arch.
    # Widens the arch↔beam attachment from a thin band to a full joint.
    _rise = _SB_Y0 - BEAM_SIZE / 2.0             # 12 — full column↔beam y-gap
    for s, z0 in ((+1.0, _ARCH_Z0_RAT), (-1.0, _ARCH_Z0_BRK)):
        m = m.union(_yz_prism(
            [(s * BEAM_SIZE / 2.0, z0),
             (s * (BEAM_SIZE / 2.0 + _rise / 2.0), z0 + _rise / 2.0),
             (s * BEAM_SIZE / 2.0, z0 + _rise / 2.0)],
            BEAM_IR, _R_OUT))
    # the arm TURNS BACK DOWN 45° into the beam (user's sketch): bevel the
    # junction with a −45° descending roof placed so the material band stays
    # POST_OUT_T thick measured PERPENDICULAR from the ^ apex (yellow ==
    # blue). Cut plane: z = y + k, k = z_apex − y_apex + POST_OUT_T·√2.
    _half = (_SB_Y0 - BEAM_SIZE / 2.0) / 2.0
    _k = ((_ARCH_Z0_RAT + _half) - (_SB_Y0 - _half)
          + POST_OUT_T * math.sqrt(2.0))         # ≈ 42.57
    for s in (+1.0, -1.0):
        m = m.cut(_yz_prism(
            [(s * BEAM_SIZE / 2.0, BEAM_SIZE / 2.0 + _k),
             (s * BEAM_SIZE / 2.0, 60.0),
             (s * 12.0, 60.0),
             (s * 12.0, 12.0 + _k)],
            BEAM_IR - 1.0, _R_OUT + 1.0))
    # thrust bosses (identical rings, extremities flush with the arm edges):
    # beam flank grows OUT to the lever's inner face (wall-cylinder guard on
    # its −x side); the column face grows IN to the lever's outer face
    # both rings identical (the square axle needs the same envelope bore
    # on each side); built around that bore so the annulus survives full
    for s, pz in ((+1.0, RATCHET_PIVOT_Z), (-1.0, BRAKE_PIVOT_Z)):
        m = m.union(_thrust_boss(s * BEAM_SIZE / 2.0, s, pz,
                                 BOSS_BORE_ID, clip_x=WALL_OR))
        m = m.union(_thrust_boss(s * _SB_Y0, -s, pz, BOSS_BORE_ID))
    # (the BRAKE rest tab is GONE: it rooted on the climb's inner face, and
    # the climb moved outside the wall band with the axial chamber. The
    # brake's rest stop is now its own WINDOW SILL — the torsion preload
    # rotates the lever ~the 1.0 sill clearance until its lower edge rests
    # on wall_bottom's brake-window sill, pad still clear of the band.
    # FLAGGED for user review.)
    return m


def _arm_trim():
    """Cut the +X arm's FLAT run past the centre branch's climb start — the
    45° diagonal replaces it (the beams are unioned after this cut, restoring
    the full-height beam above)."""
    return _box(_XC, _R_OUT + 1.0, -BEAM_SIZE / 2.0 - 0.05,
                BEAM_SIZE / 2.0 + 0.05, BOT_RIB_Z0 - 0.05, BOT_RIB_Z1 + 0.05)


def _beam_bottom_trim():
    """Cut the +X beam (and the inner fork-post bases beside it) below the
    centre branch's 45° climb line — the climb IS the beam's foot, so nothing
    at +X reaches the build plate except the three branches' flat runs. The
    resulting 45° underside prints self-supporting."""
    def z_at(x):
        return BOT_RIB_Z0 + (x - _XB)
    x0, x1 = BEAM_IR, _R_OUT + 1.0
    return (cq.Workplane("XZ")
            .polyline([(x0, BOT_RIB_Z0 - 0.1), (x1, BOT_RIB_Z0 - 0.1),
                       (x1, z_at(x1)), (x0, z_at(x0))])
            .close()
            .extrude(-11.6).translate((0, -5.8, 0)))


def _pin_bores_side(pz, s):
    """SQUARE-axle bores for one lever (s=+1 ratchet, −1 brake) — user
    redesign: the stepped hex pin's corners printed too soft to grip. The
    axle is one plain square prism; the frame's holes are the square
    rotated 45° — DIAMOND, TIP-UP in this part's sideways print: 45°
    flanks self-support and the ceiling is a POINT, so no teardrop is
    needed. Column: THROUGH keyway (grip stub stands proud outside);
    beam flank: BLIND bore whose floor at PIN_TIP_END_Y is the axle's
    depth stop. Both run through their thrust rings to the rings' free
    faces + 0.5 (a cutter step inside a ring strands a crown sliver —
    learned on the old design). The pre-twist lives in the RELATIVE clock
    vs the lever's pocket (BASE − PRETWIST; it prints vertically,
    clock-free)."""
    y1o = s * RATCHET_LEV_Y1                     # outer lever face (±16.8)
    F = y1o + s * (LEVER_SIDE_CLR + POST_OUT_T)  # side-column outer face (±23.6)
    side = PIN_SQ_S + 2.0 * PIN_SQ_FRAME_CLR

    def sq(ya, yb):
        lo, hi = min(ya, yb), max(ya, yb)
        return (cq.Workplane("XY").rect(side, side)
                .extrude(hi - lo)
                .rotate((0, 0, 0), (1, 0, 0), -90)
                .translate((LEVER_PIVOT_X, lo, pz))
                .rotate((LEVER_PIVOT_X, 0, pz), (LEVER_PIVOT_X, 1, pz),
                        PIN_KEY_BASE_DEG))

    out = sq(F + s * 0.5, y1o - s * 0.5)         # column + its ring, mouth open
    out = out.union(sq(s * PIN_TIP_END_Y,        # beam blind bore + its ring
                       s * (BEAM_SIZE / 2.0 + LEVER_SIDE_CLR + 0.5)))
    return out


def _lever_pin_bores():
    return (_pin_bores_side(RATCHET_PIVOT_Z, +1.0)
            .union(_pin_bores_side(BRAKE_PIVOT_Z, -1.0)))


def _build_frame_bottom():
    fb = _plus(BOT_RIB_Z0, BOT_RIB_Z1)
    fb = fb.cut(_arm_trim())              # the centre climb replaces the flat run
    for a in (0.0, 90.0, 180.0, 270.0):
        fb = fb.union(_beam(a)).union(_arc_tenon(a))
    fb = fb.union(_lever_mount())
    fb = fb.cut(_beam_bottom_trim())      # the 45° climb is the +X beam's foot
    fb = fb.cut(_lever_pin_bores())
    for a in (0.0, 90.0, 180.0, 270.0):
        fb = fb.cut(_wall_mortise(a))
    # axle slip bore through the plus centre (the axle's bottom end lands here)
    # + the diagonal rod bore pinning it (mirrors the top crossing's)
    fb = fb.cut(cyl(AXLE_BORE_D, (BOT_RIB_Z1 - BOT_RIB_Z0) + 1.0, z=BOT_RIB_Z0 - 0.5))
    fb = fb.cut(rod_hole(ROD_Z_BOT))
    return heal(fb)


def _build_frame_top():
    ft = _plus(TOP_RIB_Z0, TOP_RIB_Z1)   # FLUSH at the frame extent again
    for a in (0.0, 90.0, 180.0, 270.0):
        ft = ft.cut(_arc_mortise(a))
    # axle slip bore through the crossing + the diagonal rod bore (the rod pins
    # the axle to this part — same cutter as the axle's own bore, so they align)
    ft = ft.cut(cyl(AXLE_BORE_D, (TOP_RIB_Z1 - TOP_RIB_Z0) + 1.0, z=TOP_RIB_Z0 - 0.5))
    ft = ft.cut(rod_hole(ROD_Z))
    # desk/wall-mount channels in the TOP face: a mortise + entry-pocket
    # pair on each arm, along X and along Y (see mount.py)
    for c in mount_channel_cuts():
        ft = ft.cut(c)
    return heal(ft)


frame_bottom = _build_frame_bottom()
frame_top = _build_frame_top()
