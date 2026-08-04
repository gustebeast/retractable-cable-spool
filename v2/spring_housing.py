"""spring chamber (the modular backbone).

COPIED from the working design (src.spool._build_main_body) — hub cylinder +
fused bearing pocket + spring cavity + cap seat + bearing-block lightening +
hub key ridges — MINUS the old cable rim / channel spokes / deck / routing
(those become separate sliding parts in the current design). The spring-strip
SCREW mount (M2 insert boss + driver access hole) is REPLACED by the
hardware-free KEYHOLE anchor (see the anchor block below) — the proven
center is otherwise reused verbatim so its fit is unchanged.

ORIENTATION: everything below is built in the PRE-FLIP (= PRINT) frame —
fused bearing at −Z, cap seat at +Z — and MIRRORED at export (see _flip):
in the ASSEMBLY the fused pocket is at +Z (it carries the housing's weight
hanging on the axle's top-bearing lip, user's call) and the removable cap
seats at −Z. The housing prints world-+Z→−Z (the print plate sees the
pre-flip frame's −Z→+Z, which every 45° relief was designed for).

Adds this design's RIM SEAT: a 45° cone the separator's bore chamfer lands
on plane-on-plane at RIM_SEAT_Z (assembly frame — added after the flip).
"""

import math

import cadquery as cq

from v2.dimensions import NOZZLE  # bead-multiple sizing unit

from v2.dimensions import (
    HUB_OD, HUB_CAVITY_D, SPOOL_H,
    BEARING_BORE, BEARING_LIP_H, BEARING_LIP_ID, BEARING_W,
    BOOL_OVERSHOOT, PANCAKE_CAP_SEAT_Z0, CAP_STOP_LIP_H,
    SPOKE_COUNT, STRUCT_WALL,
)
from v2.helpers import cyl, cone_solid, heal, make_keys
from v2.caps import bearing_cap_top, cap_seat_tenons  # proven cap + seat T&G
from .params import RIM_SEAT_Z, SEAT_SHOULDER_OD, SEAT_CONE_H

# ── Spring-chamber local constants (copied from v2.spool) ───────────────────
BOT_POCKET_TOP        = BEARING_LIP_H + BEARING_W         # 8 — top of the fused bottom pocket
BOT_BEARING_BOSS_WALL = STRUCT_WALL
BOT_BEARING_RIB_W     = STRUCT_WALL

# ── HARDWARE-FREE spring-strip anchor (user's GATE design — replaces the
# M2 insert + screw boss + Ø10 driver access hole) ───────────────────────────
# The constant-force spring's outer strip end (user-measured: 0.2 thick,
# 22-wide strip tapering to a 9×8 NECK, then a 7-long × 16-tall T-HEAD)
# anchors in a two-block GATE on the cavity wall + a slide-in INSERT. The
# hub wall stays a SOLID 1.6 everywhere; all housing-side geometry is
# added material, printable in the housing's (pre-flip, bottom-up) print.
#   · GATE BLOCKS — two blocks flanking a z-GAP that is > C (the neck
#     passes) but < G (the head cannot); each block < D wide tangentially,
#     so the 8-long neck spans it. INSTALL: lay the strip's end against
#     the wall, the neck dropping RADIALLY into the gap — head CW of the
#     blocks, taper CCW. Tension (CW pull) drags the strip until the A→C
#     TAPER widens past the gap and JAMS on the blocks' CCW faces: the
#     tangential anchor — no threading, no hardware.
#   · INSERT (spring_gate_insert, printed separately) — the strip could
#     still pop back out RADIALLY the way it was laid in (user-caught),
#     so a flat bar slides through matching WALL-OPEN channels in BOTH
#     blocks, passing over the neck: radial escape closed. In THIS
#     (pre-flip/print) frame it drops +z→−z through the open cap seat,
#     shoulder landing on block B's flat top; in the ASSEMBLY (mirrored)
#     it enters through the open BOTTOM seat and the installed CAP
#     retains it — the strip's inward pull presses it onto the channels'
#     inner lips either way, never toward its entry.
# PRINTABILITY: the blocks' pre-flip undersides get 45° INWARD-RISING
# reliefs starting at the strip-clearance radius (only a 0.8 one-bead
# cantilever band is left at the wall), so nothing overhangs and the
# gap's wall band keeps its full height for the neck. Channels run full-z
# through the blocks — vertical walls only.
# (x, y) survive the z-flip, so CW (final +Z view) = CW here = −y local.
STRIP_T      = 0.2                    # spring steel thickness
STRIP_W      = 22.0                   # A — full strip width
NECK_W       = 9.0                    # C — neck width (across = z)
NECK_L       = 8.0                    # D — neck length (along the strip)
HEAD_L       = 7.0                    # F — head length (along the strip)
HEAD_W       = 16.0                   # G — head width (across = z)
ANCHOR_ANGLE = 270.0                  # the old screw azimuth
_R_WALL      = HUB_CAVITY_D / 2.0     # 31.6 — cavity wall radius
# gate geometry (local frame: +X radial at the anchor, CW = −y)
GATE_GAP0, GATE_GAP1 = 25.2, 34.8     # the neck gate: 9.6 — > C, < G
GATE_BLOCK_W = 7.7                    # tangential block width — < D (8)
GATE_INS_T   = 0.8                    # insert bar radial thickness — UNDER
                                      # the 1.6 tier by design (user's call:
                                      # the bar prints lying FLAT, so this
                                      # is solid layers, not a thin vertical
                                      # wall; load is a few N of press).
                                      # 0.8 buys the strip a 0.7 gap
GATE_INS_W   = 4.0                    # insert bar tangential width (FLAT
                                      # prism — its inner corners must clear
                                      # the channel's converging radial
                                      # walls, hence a touch narrower)
GATE_CLR     = 0.3                    # insert ↔ channel tangential clearance
GATE_LIP_T   = 2 * NOZZLE                    # channel inner lip (quality tier)
GATE_CH_R0   = _R_WALL - 2 * NOZZLE          # 30.0 — the CHANNEL's full depth from
                                      # the wall is capped at 1.6 (user
                                      # measured the 1.9 opening; the cap is
                                      # the opening itself, not the bar)
GATE_INS_R1  = _R_WALL - 0.7          # 30.9 — insert outer face (the 0.2
                                      # strip rides between it and the wall
                                      # with 0.5 of working play)
GATE_INS_R0  = GATE_INS_R1 - GATE_INS_T           # 30.1 — 0.1 floor play
                                                  # over the channel
GATE_R0      = GATE_CH_R0 - GATE_LIP_T            # 28.4 — block inner face
GATE_A_Z0    = 17.0                   # block A (below the gap) bottom
GATE_B_Z1    = 41.4                   # block B (above the gap) top
GATE_BAR_Z0, GATE_BAR_Z1 = 15.4, 41.9             # insert bar z-span (seated)
_STRIP_ZC    = 30.0                   # seated strip centreline (spring 19..41)

assert NECK_W + 0.4 < (GATE_GAP1 - GATE_GAP0) < HEAD_W - 3.0, \
    "gate gap must pass the neck and block the head"
assert GATE_BLOCK_W <= NECK_L - 0.3 + 1e-9, \
    "gate blocks wider than the neck can span"
assert _R_WALL - GATE_CH_R0 <= 2 * NOZZLE + 1e-9, \
    "channel depth from the wall exceeds the 1.6 cap (user-measured length)"
assert GATE_INS_R0 - GATE_CH_R0 >= 0.1 - 1e-9, \
    "insert has no radial floor play in the channel"
SPRING_OD_MEASURED = 57.0             # the REAL spring (user-measured; the
                                      # design constant SPRING_OD=56 sized
                                      # the cavity and stays untouched)
assert (HUB_CAVITY_D - SPRING_OD_MEASURED) - (_R_WALL - GATE_R0) \
    >= 2.0 - 1e-9, \
    "gate intrusion leaves the measured coil under 2.0 diametral float " \
    "(the old cap-stop lip's -1.0 is GONE — the lip was removed)"
assert GATE_B_Z1 + 1.4 <= PANCAKE_CAP_SEAT_Z0 - 0.2 + 1e-9, \
    "insert shoulder reaches the cap's interior face"
assert GATE_B_Z1 <= (PANCAKE_CAP_SEAT_Z0 - CAP_STOP_LIP_H) - 0.5 - 1e-9, \
    "gate block B reaches the cap-seat cone"
assert (_STRIP_ZC - NECK_W / 2.0 >= GATE_GAP0 + 0.2 - 1e-9
        and _STRIP_ZC + NECK_W / 2.0 <= GATE_GAP1 - 0.2 + 1e-9), \
    "seated neck not centred in the gate gap"


def _bot_bearing_void():
    boss_od = BEARING_BORE + 2 * BOT_BEARING_BOSS_WALL
    z_lo    = 0.0 - BOOL_OVERSHOOT
    h       = BOT_POCKET_TOP - z_lo
    return cyl(HUB_CAVITY_D, h, z=z_lo).cut(cyl(boss_od, h, z=z_lo))


def _bot_bearing_ribs():
    boss_od = BEARING_BORE + 2 * BOT_BEARING_BOSS_WALL
    r_in    = boss_od / 2 - 0.5
    r_out   = HUB_OD / 2
    z_lo, z_hi = 0.0, BOT_POCKET_TOP
    out = None
    for i in range(SPOKE_COUNT):
        rib = (cq.Workplane("XZ")
               .polyline([(r_in, z_lo), (r_out, z_lo), (r_out, z_hi), (r_in, z_hi)])
               .close()
               .extrude(BOT_BEARING_RIB_W / 2, both=True)
               .rotate((0, 0, 0), (0, 0, 1), i * 360.0 / SPOKE_COUNT))
        out = rib if out is None else out.union(rib)
    return out


def _sector(r0, r1, a0, a1, z0, z1):
    """Annular sector (local frame): radii r0..r1 × arc a0..a1 (deg) ×
    z0..z1 — every radial face CONCENTRIC with the hub wall, every side
    face a radial plane. The whole gate is built from these (chordal
    boxes read short depths at their edges — a flat channel floor
    measured 1.52 where the caliper landed, user-caught — and their
    corners graze the curved wall)."""
    return (cq.Workplane("XZ")
            .polyline([(r0, z0), (r1, z0), (r1, z1), (r0, z1)])
            .close().revolve(a1 - a0, (0, 0), (0, 1))
            .rotate((0, 0, 0), (0, 0, 1), a0))


# arc half-angles: widths are ARC LENGTHS at the strip's mid-plane radius
_GATE_RREF = _R_WALL - 0.3
_A_BLK = math.degrees(GATE_BLOCK_W / _GATE_RREF) / 2.0
# channel half-angle: the FLAT bar's inner corners are its widest angular
# points — size the radial walls off them, plus the clearance
_A_CH  = math.degrees(math.atan((GATE_INS_W / 2.0 + GATE_CLR) / GATE_INS_R0))
_A_INS = math.degrees(GATE_INS_W / _GATE_RREF) / 2.0
_A_SH  = math.degrees((GATE_INS_W + 4 * NOZZLE) / _GATE_RREF) / 2.0
assert _A_SH < _A_BLK - 1e-9, "insert shoulder wider than the gate block"


def _gate_block(z0, z1):
    """ONE gate block (local frame): a revolved sector rooted on the hub
    wall, spanning z0..z1, with (a) a 45° INWARD-RISING relief on its
    pre-flip underside — full section only above z0 + its radial depth; a
    0.8 one-bead cantilever band is left at the wall so the gap keeps its
    full height where the strip rides — and (b) the wall-open INSERT
    CHANNEL running full z through it (concentric floor, radial side
    walls)."""
    # underside = ONE CONTINUOUS 45° from the wall face (user print
    # finding ×2: an earlier version left a 0.8 flat cantilever band at
    # the wall — the feet printed as flat overhangs). The 0.2 embed start
    # puts the chamfer's landing exactly ON the wall face at z0.
    b = (cq.Workplane("XZ")
         .polyline([(_R_WALL + 0.2, z0 - 0.2),
                    (GATE_R0, z0 - 0.2 + (_R_WALL + 0.2 - GATE_R0)),
                    (GATE_R0, z1), (_R_WALL + 0.2, z1)])
         .close().revolve(2.0 * _A_BLK, (0, 0), (0, 1))
         .rotate((0, 0, 0), (0, 0, 1), -_A_BLK))
    b = b.cut(_sector(GATE_CH_R0, _R_WALL + 0.4, -_A_CH, _A_CH,
                      z0 - 1.0, z1 + 1.0))
    # GABLE the lip's bottom over the channel span (user print finding:
    # the r-z relief alone leaves the lip starting as an unsupported
    # knife-edge BRIDGE across the channel — mushy). Two 45° INWARD
    # DIAGONALS rise from the channel's side walls, so every lip layer
    # roots on the flanking material. The chordal prism is INTERSECTED
    # with the channel's angular wedge — its flat sides otherwise poke
    # past the radial walls at the lip's smaller radii and shave a sliver
    # off the feet (user-caught artifact).
    hw = math.tan(math.radians(_A_CH)) * GATE_CH_R0
    lip0 = z0 + (_R_WALL - GATE_CH_R0)   # the lip's outer-edge start height
    gable = (cq.Workplane("YZ").workplane(offset=GATE_R0 - 0.5)
             .polyline([(-hw, z0 - 0.5), (hw, z0 - 0.5),
                        (hw, lip0), (0.0, lip0 + hw), (-hw, lip0)])
             .close().extrude((GATE_CH_R0 + 0.4) - (GATE_R0 - 0.5))
             .intersect(_sector(GATE_R0 - 0.6, GATE_CH_R0 + 0.5,
                                -_A_CH, _A_CH, z0 - 1.0, z1 + 1.0)))
    return b.cut(gable)


def _spring_gate():
    """Both gate blocks, rotated to ANCHOR_ANGLE."""
    g = _gate_block(GATE_A_Z0, GATE_GAP0)         # A — below the gap
    g = g.union(_gate_block(GATE_GAP1, GATE_B_Z1))  # B — above the gap
    return g.rotate((0, 0, 0), (0, 0, 1), ANCHOR_ANGLE)


def _gate_insert_local():
    """The slide-in INSERT (seated pose, local frame): a flat bar through
    both blocks' channels + the SHOULDER at its pre-flip-bottom end
    (final-frame TOP — it lands on the then-upper block A, so gravity
    keeps the insert seated; the user's +z→−z install)."""
    # FLAT PRISM (user print finding: an arc-section bar can't lie flat on
    # its side to print) — plain boxes, sized so every corner clears the
    # channel's radial walls, the concentric floor, and the curved hub
    # wall at its worst (corner) points.
    def _pbox(x0, x1, y0, y1, z0, z1):
        return (cq.Workplane("XY").workplane(offset=z0)
                .polyline([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
                .close().extrude(z1 - z0))

    bar = _pbox(GATE_INS_R0, GATE_INS_R1, -GATE_INS_W / 2.0,
                GATE_INS_W / 2.0, GATE_BAR_Z0, GATE_BAR_Z1)
    # shoulder at the TOP (the cap is at +Z now — the insert enters from
    # the open cap seat moving DOWN, gravity seats the shoulder onto block
    # B's flat top, and the installed cap then covers the entry)
    sh_y = GATE_INS_W / 2.0 + 2 * NOZZLE
    sh_x1 = math.sqrt((_R_WALL - 0.15) ** 2 - sh_y ** 2)
    sh = _pbox(GATE_INS_R0, sh_x1, -sh_y, sh_y,
               GATE_B_Z1, GATE_B_Z1 + 1.4)
    return bar.union(sh)


def _hub_key_bumps():
    return make_keys(HUB_OD / 2, 0.0, SPOOL_H)


def _rim_seat_cone():
    """The separator's 45° CONE SEAT — ASSEMBLY frame, added AFTER the flip
    (it must NOT mirror: the separator stays at RIM_SEAT_Z). An up-facing
    cone from SEAT_SHOULDER_OD at RIM_SEAT_Z shrinking back to the hub OD;
    the separator's matching bore chamfer lands on it PLANE-ON-PLANE —
    self-centering, its bottom face at exactly RIM_SEAT_Z. Replaces the
    old up-EXPANDING cone + flat ledge, which was support-free only in
    the old print direction: in the flipped print this cone's flank is
    the self-supporting 45° and the flat ledge (an overhang ring there)
    is gone. Bonus: the hub below RIM_SEAT_Z is now PLAIN — nothing for
    the cable floor to clear. cone_solid fills to the axis, so the cavity
    bore is cut back open."""
    cone = cone_solid(SEAT_SHOULDER_OD, HUB_OD, SEAT_CONE_H, RIM_SEAT_Z)
    bore = cyl(HUB_CAVITY_D, SEAT_CONE_H + 2.0, z=RIM_SEAT_Z - 1.0)
    return cone.cut(bore)


def _build_core():
    """Proven spring-chamber geometry, built in the ORIGINAL orientation
    (bottom-fused bearing, top removable-cap seat). Flipped below."""
    _BOT_POCKET_TOP = BEARING_LIP_H + BEARING_W                 # 8
    _CAVITY_TOP     = PANCAKE_CAP_SEAT_Z0 - CAP_STOP_LIP_H       # 42
    body = cyl(HUB_OD, SPOOL_H, z=0)
    body = (
        body
        .cut(cyl(BEARING_LIP_ID, BEARING_LIP_H, z=0))
        .cut(cyl(BEARING_BORE, BEARING_W, z=BEARING_LIP_H))
        .cut(cyl(HUB_CAVITY_D, _CAVITY_TOP - _BOT_POCKET_TOP, z=_BOT_POCKET_TOP))
        # cap-stop LIP REMOVED (user's call): the bore runs straight through
        # to the cap seat — a smooth contact ring, no z retention (the frame
        # blocks cap removal once assembled) — and the Ø57 spring gains the
        # lip's 1 mm of insertion corridor
        .cut(cyl(HUB_CAVITY_D, CAP_STOP_LIP_H, z=_CAVITY_TOP))
        .cut(cyl(HUB_CAVITY_D, SPOOL_H - PANCAKE_CAP_SEAT_Z0, z=PANCAKE_CAP_SEAT_Z0))
    )
    body = body.cut(_bot_bearing_void())
    body = body.union(_bot_bearing_ribs())
    body = body.union(_spring_gate())
    body = body.union(cap_seat_tenons())    # 6 radial T&G tenons at the seat
    body = body.union(_hub_key_bumps())
    return body


# FLIPPED AT EXPORT (user's call — the cap moved BACK to −Z): the housing
# HANGS from the axle via the axle's top lip → top bearing → its pocket.
# With the pocket FUSED at +Z that load runs into solid material; the old
# top-side cap could slide off its smooth seat under exactly that load.
# The proven core (and its whole gate/assert suite) stays in the PRE-FLIP
# frame — fused bearing at −Z, cap seat at +Z — which IS the print
# orientation: the housing now prints +Z→−Z (world), i.e. the pre-flip
# frame's −Z→+Z on the plate; every 45° relief was designed for it.
# Both pocket stacks are CAP_H deep, so the two bearings' ASSEMBLY
# positions are unchanged by the flip — the axle needed no changes.


def _flip(w):
    """PRINT frame → ASSEMBLY frame: mirror about the mid-plane."""
    return w.mirror("XY", (0.0, 0.0, SPOOL_H / 2.0))


def _fairlead_chamfer():
    """45° CUTTER ring easing the hub barrel's BOTTOM-OUTER corner (world
    z 0): at full extension the AXIAL loop's drop funnels inward BELOW the
    barrel to the tight coil, and the cable bears on this edge. Everything
    here CO-ROTATES (drop, barrel, coil top — no rubbing), so the edge just
    needs to stop being a knife: a 3*NOZZLE 45° fairlead. In the flipped
    print (world +Z→−Z) this is bed-side geometry — no overhang."""
    ch = 3 * NOZZLE
    ring = cyl(HUB_OD + 2.0, ch, z=0)
    keep = cone_solid(d_bottom=HUB_OD - 2 * ch, d_top=HUB_OD,
                      h=ch, z_base=0.0)
    return ring.cut(keep)


_core = _build_core()                # smooth cap seat — no key grooves
spring_housing = heal(_flip(_core).union(_rim_seat_cone())
                      .cut(_fairlead_chamfer()))

# The removable cap — at the −Z seat in the assembly (flipped with the
# housing; caps.py builds the pre-flip/print frame).
spring_housing_cap = heal(_flip(bearing_cap_top))

# The gate insert, modelled SEATED (the slicer lays the flat bar on its
# side for printing). Flipped with the housing: in the ASSEMBLY it enters
# through the OPEN BOTTOM cap seat, its shoulder stops against the lower
# block's underside, and the installed CAP retains it (gravity rests it
# on the cap's interior face; the strip's pull stays radial, pressing it
# onto the channel lips — never toward the entry).
spring_gate_insert = heal(
    _flip(_gate_insert_local().rotate((0, 0, 0), (0, 0, 1), ANCHOR_ANGLE)))




# ── PRINT-FIT COUPON: the spring-strip LOCK on a standing wall section ───────
# A curved slice of the hub wall (true 1.6 wall, true radius) carrying BOTH
# gate blocks, built in the LOCAL frame with its bottom on z=0 — it prints
# VERTICALLY exactly like the housing (same relief orientation), brim
# recommended (user's plan). Print with spring_gate_insert and test the
# real strip: lay the neck into the gap, feel the taper jam CW, slide the
# insert down, confirm the radial trap. Arc spans the taper-jam run CCW
# and the full head CW.
_CPN_A0, _CPN_A1 = -22.0, 23.0
_CPN_Z0, _CPN_Z1 = 13.0, 42.5         # gate + insert travel + shoulder room


def _build_gate_coupon():
    w = _sector(_R_WALL, _R_WALL + 2 * NOZZLE, _CPN_A0, _CPN_A1, _CPN_Z0, _CPN_Z1)
    w = w.union(_gate_block(GATE_A_Z0, GATE_GAP0))
    w = w.union(_gate_block(GATE_GAP1, GATE_B_Z1))
    return heal(w.translate((0, 0, -_CPN_Z0)))


spring_gate_coupon = _build_gate_coupon()
