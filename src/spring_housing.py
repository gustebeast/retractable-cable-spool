"""spring chamber (the modular backbone).

COPIED from the working design (src.spool._build_main_body) — hub cylinder +
bottom fused bearing pocket + spring cavity + top cap-stop lip + top cap seat +
bearing-block lightening + hub key ridges — MINUS the old cable rim / channel
spokes / deck / routing (those become separate sliding parts in the current design). The
spring-strip SCREW mount (M2 insert boss + driver access hole) is REPLACED by
the hardware-free KEYHOLE anchor (see the anchor block below) — the proven
center is otherwise reused verbatim so its fit is unchanged.

Adds this design's RIM SEAT: a 45° cone-supported shoulder on the hub OD that stops the
middle brake/ratchet rim at half height (equal chamber length above and below).
"""

import math

import cadquery as cq

from src.dimensions import (
    HUB_OD, HUB_CAVITY_D, SPOOL_H,
    BEARING_BORE, BEARING_LIP_H, BEARING_LIP_ID, BEARING_W,
    BOOL_OVERSHOOT, PANCAKE_CAP_SEAT_Z0, CAP_STOP_ID, CAP_STOP_LIP_H,
    SPOKE_COUNT, STRUCT_WALL,
)
from src.helpers import cyl, cone_solid, heal, make_keys
from src.caps import bearing_cap_top, apply_to_main_body  # proven cap + its seat-groove cut
from .params import RIM_SEAT_Z, SEAT_SHOULDER_OD, SEAT_CONE_H

# ── Spring-chamber local constants (copied from src.spool) ───────────────────
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
#     blocks, passing over the neck: radial escape closed. It installs
#     from +z to −z in the FINAL orientation (its shoulder lands on the
#     then-upper block), so GRAVITY holds it seated; the strip's inward
#     pull presses it onto the channels' inner lips — never toward its
#     entry.
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
GATE_INS_T   = 0.9                    # insert bar radial thickness — UNDER
                                      # the 1.6 tier by design (user's call:
                                      # the bar prints lying FLAT, so this
                                      # is solid layers, not a thin vertical
                                      # wall; load is a few N of press)
GATE_INS_W   = 4.2                    # insert bar tangential width
GATE_CLR     = 0.3                    # insert ↔ channel tangential clearance
GATE_LIP_T   = 1.6                    # channel inner lip (quality tier)
GATE_CH_R0   = _R_WALL - 1.6          # 30.0 — the CHANNEL's full depth from
                                      # the wall is capped at 1.6 (user
                                      # measured the 1.9 opening; the cap is
                                      # the opening itself, not the bar)
GATE_INS_R1  = _R_WALL - 0.6          # 31.0 — insert outer face (the 0.2
                                      # strip + play rides between it and
                                      # the wall)
GATE_INS_R0  = GATE_INS_R1 - GATE_INS_T           # 30.1 — 0.1 floor play
                                                  # over the channel
GATE_R0      = GATE_CH_R0 - GATE_LIP_T            # 28.4 — block inner face
GATE_A_Z0    = 17.0                   # block A (below the gap) bottom
GATE_B_Z1    = 41.4                   # block B (above the gap) top
GATE_BAR_Z0, GATE_BAR_Z1 = 15.4, 41.9             # insert bar z-span (seated)
_STRIP_ZC    = 30.0                   # seated strip centreline (spring 19..41)
# the strip's SEATED tangential shift: the taper jams where its width
# reaches the gap height at the blocks' CCW edge (0.05 visual margin)
_SS          = 3.1

assert NECK_W + 0.4 < (GATE_GAP1 - GATE_GAP0) < HEAD_W - 3.0, \
    "gate gap must pass the neck and block the head"
assert GATE_BLOCK_W <= NECK_L - 0.3 + 1e-9, \
    "gate blocks wider than the neck can span"
assert _R_WALL - GATE_CH_R0 <= 1.6 + 1e-9, \
    "channel depth from the wall exceeds the 1.6 cap (user-measured length)"
assert GATE_INS_R0 - GATE_CH_R0 >= 0.1 - 1e-9, \
    "insert has no radial floor play in the channel"
SPRING_OD_MEASURED = 57.0             # the REAL spring (user-measured; the
                                      # design constant SPRING_OD=56 sized
                                      # the cavity and stays untouched)
assert (HUB_CAVITY_D - SPRING_OD_MEASURED) - 1.0 - (_R_WALL - GATE_R0) \
    >= 2.0 - 1e-9, \
    "gate intrusion leaves the measured coil under 2.0 diametral float " \
    "(the −1.0 is the cap-stop lip, live during insertion)"
assert GATE_BAR_Z1 - GATE_BAR_Z0 <= GATE_GAP1 - BOT_POCKET_TOP + 1e-9, \
    "insert too long to enter the cavity axially (final-frame headroom)"
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
_A_CH  = math.degrees((GATE_INS_W + GATE_CLR) / _GATE_RREF) / 2.0
_A_INS = math.degrees(GATE_INS_W / _GATE_RREF) / 2.0
_A_SH  = math.degrees((GATE_INS_W + 3.2) / _GATE_RREF) / 2.0
assert _A_SH < _A_BLK - 1e-9, "insert shoulder wider than the gate block"


def _gate_block(z0, z1):
    """ONE gate block (local frame): a revolved sector rooted on the hub
    wall, spanning z0..z1, with (a) a 45° INWARD-RISING relief on its
    pre-flip underside — full section only above z0 + its radial depth; a
    0.8 one-bead cantilever band is left at the wall so the gap keeps its
    full height where the strip rides — and (b) the wall-open INSERT
    CHANNEL running full z through it (concentric floor, radial side
    walls)."""
    rise = (_R_WALL - 0.8) - GATE_R0
    b = (cq.Workplane("XZ")
         .polyline([(_R_WALL + 0.2, z0), (_R_WALL - 0.8, z0),
                    (GATE_R0, z0 + rise), (GATE_R0, z1),
                    (_R_WALL + 0.2, z1)])
         .close().revolve(2.0 * _A_BLK, (0, 0), (0, 1))
         .rotate((0, 0, 0), (0, 0, 1), -_A_BLK))
    return b.cut(_sector(GATE_CH_R0, _R_WALL + 0.4, -_A_CH, _A_CH,
                         z0 - 1.0, z1 + 1.0))


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
    bar = _sector(GATE_INS_R0, GATE_INS_R1, -_A_INS, _A_INS,
                  GATE_BAR_Z0, GATE_BAR_Z1)
    # the shoulder is a wider sector — concentric like everything else, so
    # its outer face can hug the wall uniformly (the old chordal box's
    # corners poked 0.07 into the curved wall, probe-caught)
    sh = _sector(GATE_INS_R0, _R_WALL - 0.15, -_A_SH, _A_SH,
                 GATE_BAR_Z0, GATE_A_Z0 - 0.3)
    return bar.union(sh)


def _hub_key_bumps():
    return make_keys(HUB_OD / 2, 0.0, SPOOL_H)


def _rim_seat_notch():
    """45° cone flaring from the hub OD up to a shoulder at RIM_SEAT_Z — its top
    face is the self-supporting ledge the middle rim slides down onto. cone_solid
    fills to the axis, so cut the spring cavity bore back open (else the cone
    plugs the cavity with a solid disk)."""
    cone = cone_solid(HUB_OD, SEAT_SHOULDER_OD, SEAT_CONE_H, RIM_SEAT_Z - SEAT_CONE_H)
    bore = cyl(HUB_CAVITY_D, SEAT_CONE_H + 2.0, z=RIM_SEAT_Z - SEAT_CONE_H - 1.0)
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
        .cut(cone_solid(HUB_CAVITY_D, CAP_STOP_ID, CAP_STOP_LIP_H, _CAVITY_TOP))
        .cut(cyl(HUB_CAVITY_D, SPOOL_H - PANCAKE_CAP_SEAT_Z0, z=PANCAKE_CAP_SEAT_Z0))
    )
    body = body.cut(_bot_bearing_void())
    body = body.union(_bot_bearing_ribs())
    body = body.union(_spring_gate())
    body = body.union(_hub_key_bumps())
    return body


def _flip_z(wp):
    """Mirror a part about z = SPOOL_H/2 (z → SPOOL_H − z): swaps the two ends
    top↔bottom. Applied to BOTH the housing core and its cap so their fit is
    preserved, just inverted."""
    return cq.Workplane(obj=wp.val().mirror("XY")).translate((0, 0, SPOOL_H))


# FLIP the proven core so the FUSED bearing is at +Z and the REMOVABLE cap seat
# is at −Z (per the install sequence: the housing now prints +Z→−Z, features
# added below the flip keep their as-designed print orientation). The rim seat
# notch is added AFTER the flip so it stays at mid-height (equal chamber above
# and below) with a cone that is self-supporting in the new +Z-down print.
_core = apply_to_main_body(_build_core())   # cut cap-seat key grooves (pre-flip)
_flipped = _flip_z(_core)
_flipped = _flipped.union(_rim_seat_notch())
spring_housing = heal(_flipped)

# The removable cap, flipped to the −Z seat with the same transform.
spring_housing_cap = heal(_flip_z(bearing_cap_top))

# The gate insert, modelled SEATED (same flip → correct assembly pose;
# the slicer lays the flat bar on its side for printing).
spring_gate_insert = heal(_flip_z(
    _gate_insert_local().rotate((0, 0, 0), (0, 0, 1), ANCHOR_ANGLE)))


# ── VIEWER MODEL: the spring strip's END, seated in the anchor ───────────────
# Not a printed part — a 0.2 steel ribbon (the user's measured outline)
# hugging the cavity wall: NECK in the gate gap (under the insert), taper
# JAMMED against the blocks' CCW faces, HEAD clear CW of the blocks
# (legacy screw hole and the E-notch included so it reads as the real
# part), and a stretch of full-width body wrapping away CW toward the
# coil (truncated — the coil itself isn't modelled). Every arc position
# carries the seated taper-jam shift _SS.
_SR0, _SR1 = _R_WALL - 0.45, _R_WALL - 0.25    # ribbon radial band (in the
                                               # strip gap wall ↔ insert)
_SR_MID    = (_SR0 + _SR1) / 2.0


def _strip_band(s0, s1, z0, z1):
    """Ribbon segment: arc-lengths s0..s1 (mm along the wall, CW negative,
    in the head-at-origin layout — the seated shift _SS is applied here)
    × z-band, revolved at the strip's radius."""
    d0, d1 = (math.degrees((s + _SS) / _SR_MID) for s in (s0, s1))
    return (cq.Workplane("XZ")
            .polyline([(_SR0, z0), (_SR1, z0), (_SR1, z1), (_SR0, z1)])
            .close().revolve(d1 - d0, (0, 0), (0, 1))
            .rotate((0, 0, 0), (0, 0, 1), d0))


def _strip_end_model_local():
    zc = _STRIP_ZC
    strip = _strip_band(-6.4, 1.6, zc - NECK_W / 2.0, zc + NECK_W / 2.0)
    strip = strip.union(_strip_band(1.6, 8.6, zc - HEAD_W / 2.0,
                                    zc + HEAD_W / 2.0))
    # A→C taper in 8 steps, each at the TRUE width of its NARROW (CCW) end
    # so the stepped model stays strictly INSIDE the smooth outline (a
    # midpoint-sampled step once overshot into the gate blocks' cantilever
    # bands, probe-caught)
    for i in range(8):
        s0 = -19.4 + i * 13.0 / 8.0
        half = 11.0 - (i + 1) * (11.0 - NECK_W / 2.0) / 8.0
        strip = strip.union(_strip_band(s0, s0 + 13.0 / 8.0 + 0.05,
                                        zc - half, zc + half))
    strip = strip.union(_strip_band(-52.0, -19.4, zc - STRIP_W / 2.0,
                                    zc + STRIP_W / 2.0))
    # E-notch at the head's lower-CW corner + rounded-end corner chamfers
    strip = strip.cut(_strip_band(1.4, 4.0, zc - HEAD_W / 2.0 - 0.5,
                                  zc - HEAD_W / 2.0 + 4.0))
    strip = strip.cut(_strip_band(7.2, 8.8, zc + HEAD_W / 2.0 - 1.6,
                                  zc + HEAD_W / 2.0 + 0.5))
    strip = strip.cut(_strip_band(7.2, 8.8, zc - HEAD_W / 2.0 - 0.5,
                                  zc - HEAD_W / 2.0 + 1.6))
    # the legacy screw hole in the head (radial punch)
    ah = (5.1 + _SS) / _SR_MID
    hole = cq.Solid.makeCylinder(
        1.75, 1.5,
        cq.Vector((_SR0 - 0.5) * math.cos(ah), (_SR0 - 0.5) * math.sin(ah), zc),
        cq.Vector(math.cos(ah), math.sin(ah), 0.0))
    return strip.cut(cq.Workplane(obj=hole))


spring_strip_end = heal(_flip_z(
    _strip_end_model_local().rotate((0, 0, 0), (0, 0, 1), ANCHOR_ANGLE)))


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
    w = _sector(_R_WALL, _R_WALL + 1.6, _CPN_A0, _CPN_A1, _CPN_Z0, _CPN_Z1)
    w = w.union(_gate_block(GATE_A_Z0, GATE_GAP0))
    w = w.union(_gate_block(GATE_GAP1, GATE_B_Z1))
    return heal(w.translate((0, 0, -_CPN_Z0)))


spring_gate_coupon = _build_gate_coupon()
