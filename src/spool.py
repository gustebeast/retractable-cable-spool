"""Main spool body — drum, flanges, hub, cap pockets, pancake flange, spring slot.

Lever-dependent cuts (ratchet teeth, drum cable hole) are applied later
in `levers.py`.
"""

import math

import cadquery as cq

from .dimensions import (
    AXLE_D,
    BEARING_BORE, BEARING_LIP_H, BEARING_LIP_ID,
    BEARING_W, BOOL_OVERSHOOT,
    CAP_H, CAP_STOP_ID, CAP_STOP_LIP_H,
    CAVITY_Z0, CAVITY_Z1,
    DRUM_BOTTOM_Z, DRUM_H, DRUM_ID, DRUM_OD, DRUM_TOP_Z, DRUM_WALL,
    FLANGE_H, FLANGE_INNER_ID, FLANGE_OD, FLANGE_LIP_T,
    HUB_CAVITY_D, HUB_OD,
    RATCHET_TEETH, RATCHET_DEPTH,
    LEVER_CAP_SEAT_Z0, LEVER_CAP_SEAT_Z1,
    LEVER_STOP_LIP_Z0, LEVER_STOP_LIP_Z1,
    PANCAKE_CAP_SEAT_Z0,
    SPOKE_COUNT, SPOKE_W, SPOOL_H,
    STRUCT_WALL,
    TOP_BEARING_BORE,
)
from .helpers import (
    cyl, cone_solid, heal, make_keys,
    lever_flange_solid, pancake_flange_solid,
    spokes_solid,
)

# No drum-skirt extension — the lever-flange chamfer apex now sits ABOVE
# DRUM_BOTTOM_Z (chamfer rises up into the drum region), so the chamfer
# body itself bridges into the drum and no downward extension is needed.
DRUM_SKIRT_H = 0.0


# ────────────────────────────────────────────────────────────────────────────
# MAIN BODY — the full spool
#   Bottom flange, drum, top flange, 4 spokes, hub with bottom bearing pocket,
#   straight spring cavity, 1 mm-inset cap-stop lip, cap seat at top.
#   The cavity opens straight to the top face (no obstruction) — spring drops
#   in from above, then the bearing cap slides in after.
# ────────────────────────────────────────────────────────────────────────────

def _build_main_body():
    # PANCAKE-SPOOL REWRITE (Part 1): the tall cable drum, its helical
    # groove, and the hub→drum spokes are GONE. The cable will spiral in a
    # single Z-plane in a channel formed later by separately-printed cable-
    # retention plates (Part 2). What remains here is the "inner spool
    # body": the full-height hub plus two rims that the guide wheels and
    # levers contact directly —
    #   - bottom (lever-side) 14 mm rim: ratchet teeth + brake/guide track,
    #     at the SAME radii as the old outer-spool flange (so the levers and
    #     -z guide wheel keep their positions), tied to the hub by a 45°
    #     ramp (also the eventual cable-channel floor slope).
    #   - top (pancake-side) 7 mm rim: +z guide-wheel surface, tied to the
    #     hub by a thin top deck.
    main_body = (
        cyl(HUB_OD, SPOOL_H, z=0)                       # hub cylinder (full height)
        .union(_lever_rim())                             # bottom 14 mm-tall cable rim
        .union(_channel_spokes())                        # radial spokes: sole hub↔rim tie
        .union(_hub_key_bumps())                         # key ridges for the sliding top rim
    )

    # Interior z-map cuts — BEARING CAP FLIPPED: the LEVER-side (bottom)
    # bearing is now FUSED into the hub; the PANCAKE-side (top) bearing
    # lives in a separately-printed removable cap (bearing_cap_top).
    #   z=0 .. _BOT_POCKET_TOP            — fused bottom bearing pocket
    #   _BOT_POCKET_TOP .. _CAVITY_TOP    — spring cavity (HUB_CAVITY_D)
    #   _CAVITY_TOP .. PANCAKE_CAP_SEAT_Z0 — top cap-stop lip cone
    #   PANCAKE_CAP_SEAT_Z0 .. SPOOL_H    — top cap seat (removable cap drops in)
    _BOT_POCKET_TOP = BEARING_LIP_H + BEARING_W                 # 8
    _CAVITY_TOP     = PANCAKE_CAP_SEAT_Z0 - CAP_STOP_LIP_H       # 42
    main_body = (
        main_body
        # Bottom FUSED bearing pocket: 90° retention lip (z=0..BEARING_LIP_H,
        # ID=BEARING_LIP_ID) + press-fit pocket (ID=BEARING_BORE). Bearing
        # presses in from the spring-cavity side (z=_BOT_POCKET_TOP); the
        # axle exits the bottom face through the lip ID.
        .cut(cyl(BEARING_LIP_ID, BEARING_LIP_H, z=0))
        .cut(cyl(BEARING_BORE, BEARING_W, z=BEARING_LIP_H))
        # Spring cavity.
        .cut(cyl(HUB_CAVITY_D, _CAVITY_TOP - _BOT_POCKET_TOP, z=_BOT_POCKET_TOP))
        # Top cap-stop lip — cone narrowing DOWN from HUB_CAVITY_D (at the cap
        # seat) to CAP_STOP_ID, so the removable top cap can't slide down into
        # the spring cavity.
        .cut(cone_solid(CAP_STOP_ID, HUB_CAVITY_D, CAP_STOP_LIP_H, _CAVITY_TOP))
        # Top cap seat — HUB_CAVITY_D bore the removable cap drops into.
        .cut(cyl(HUB_CAVITY_D, SPOOL_H - PANCAKE_CAP_SEAT_Z0, z=PANCAKE_CAP_SEAT_Z0))
    )
    # Lighten the fused bottom bearing block: hollow the annulus between the
    # bearing collar and the hub wall, leaving SPOKE_COUNT (6) radial ribs —
    # the same design the pancake bearing block used before the cap flip.
    main_body = main_body.cut(_bot_bearing_void())
    main_body = main_body.union(_bot_bearing_ribs())
    main_body = main_body.cut(_spring_slot())
    return main_body


# ── Inner-spool-body cable rim (pancake rewrite, v2) ────────────────────────
# The cable winds in the annular channel BETWEEN the hub OD and the rim's
# inner wall, in a single Z-plane band RIM_H tall, stacking radially (and a
# couple of layers axially). The ratchet teeth + brake track will live on
# the rim's OUTER cylindrical face (radial-X load, structurally sound across
# the cable air-gap) — that's the upcoming lever rework, not done here. The
# rim is a plain vertical wall (no 45° chamfer) so the whole body prints
# bottom-to-top without supports.
RIM_H      = 14.0      # rim height in Z (the lever-grip surface height)
# Rim INNER Ø sets the cable-channel outer wall. Sized so the hub→rim
# channel (RIM_H tall, ~82% packing) holds ~7 ft of 6 mm cable (6 ft + margin)
# — see capacity sweep. Cable winds INSIDE the rim so its OUTER face stays
# free for the ratchet/brake.
RIM_ID     = 108.0
RIM_WALL   = STRUCT_WALL + RATCHET_DEPTH   # 3.2 — sized so the wall behind the
                       # deepest ratchet-tooth valley (RIM_WALL − RATCHET_DEPTH)
                       # equals STRUCT_WALL (1.7 mm).
RIM_OD     = RIM_ID + 2 * RIM_WALL   # 114.4


def _cyl_ratchet_band(z_lo, z_hi):
    """Cylindrical (radial-engagement) ratchet on the rim's OUTER face over
    [z_lo, z_hi]. A 90°-rotated version of the old axial ratchet: the
    sawtooth now runs around the circumference with its catch face RADIAL,
    so the pawl engages from OUTSIDE in X/Y. Tooth tips sit at RIM_OD/2
    (flush with the brake band), valleys RATCHET_DEPTH deep."""
    r_tip  = RIM_OD / 2
    r_root = r_tip - RATCHET_DEPTH
    n      = RATCHET_TEETH
    pts = []
    for i in range(n):
        ts = 2 * math.pi * i / n
        # radial catch face (valley → tip at the same angle), then the
        # ramp is the straight chord from this tip to the next valley.
        pts.append((r_root * math.cos(ts), r_root * math.sin(ts)))
        pts.append((r_tip  * math.cos(ts), r_tip  * math.sin(ts)))
    star = (
        cq.Workplane("XY").workplane(offset=z_lo)
        .polyline(pts).close()
        .extrude(z_hi - z_lo)
    )
    # Carve the bore so it's an annular toothed ring (ID = RIM_ID).
    return star.cut(cyl(RIM_ID, z_hi - z_lo, z=z_lo))


CABLE_D = 6.0   # nominal cable diameter — also the max allowed inter-spoke gap


def _channel_spokes():
    """Radial spokes that are the SOLE connection between the inner spool
    (hub) and the outer cable rim — there's no floor disc. They also keep
    the wound cable from dropping out the open bottom: the count is set so
    the circumferential gap between adjacent spokes where they meet the rim
    (RIM_ID, the widest point) stays ≤ CABLE_D, so a cable strand bridges
    the gap instead of falling through. Full rim height (z=0..RIM_H)."""
    r_in   = HUB_OD / 2 - 0.5        # 0.5 mm overlap into the hub
    r_out  = RIM_ID / 2 + 0.5        # 0.5 mm overlap into the rim wall
    w      = STRUCT_WALL
    r_rim  = RIM_ID / 2
    n = math.ceil(2 * math.pi * r_rim / (CABLE_D + w))
    out = None
    for i in range(n):
        sp = (
            cq.Workplane("XY")
            .center((r_in + r_out) / 2, 0)
            .box(r_out - r_in, w, RIM_H, centered=(True, True, False))
            .rotate((0, 0, 0), (0, 0, 1), i * 360.0 / n)
        )
        out = sp if out is None else out.union(sp)
    return out


def _hub_key_bumps():
    """6 vertical key ridges on the hub OD at KEY_ANGLES, full hub height.
    They (a) key the sliding cable top-rim against rotation, and (b) sit
    radially aligned with the top bearing-cap key grooves on the INSIDE of
    the hub wall (z=43..51), adding material back across that thinned
    section. KEY_DEPTH (1 mm) tall, KEY_W (2 mm) wide."""
    return make_keys(HUB_OD / 2, 0.0, SPOOL_H)


def _lever_rim():
    """Bottom cable rim — the RIM_H-tall outer wall only (NO floor disc; the
    spokes carry the hub↔rim connection). Its 14 mm height splits into:
      - bottom 7 mm: cylindrical RATCHET teeth on the outer face, and
      - top 7 mm: SMOOTH brake band (RIM_ID..RIM_OD).
    Bands swapped (ratchet low, brake high) so the brake lever's pivot —
    which must sit BELOW its band — lands above the spool bottom instead of
    being dragged negative, keeping the levers/housing from hanging as far
    below the spool. Cable winds in the hub→RIM_ID channel on the spoke grid."""
    z_mid = RIM_H / 2                                    # 7 — ratchet/brake split
    ratchet = _cyl_ratchet_band(0.0, z_mid)             # teeth on the bottom 7 mm
    brake = (
        cyl(RIM_OD, RIM_H - z_mid, z=z_mid)
        .cut(cyl(RIM_ID, RIM_H - z_mid, z=z_mid))
    )
    return ratchet.union(brake)


# Top rim cap — re-added AFTER the helix cut so the helix's topmost turn
# can't erode the rim into a knife edge. 9 mm radial × 2 mm axial; sits
# at the very top of the spool, restoring a uniform 2 mm-thick deck.
TOP_RIM_CAP_H = STRUCT_WALL      # 1.7 — pinned to the spoke/wall thickness
TOP_RIM_CAP_W = 9.0


def _top_rim_cap():
    z_lo    = SPOOL_H - TOP_RIM_CAP_H
    inner_d = DRUM_OD - 2 * TOP_RIM_CAP_W
    return cyl(DRUM_OD, TOP_RIM_CAP_H, z=z_lo).cut(cyl(inner_d, TOP_RIM_CAP_H, z=z_lo))


# Bottom-bearing block lightening — the fused lever-side bearing pocket
# otherwise sits in a solid Ø-HUB_OD hub block over z=0..BOT_POCKET_TOP.
# Same collar + 6-rib design the pancake (top) bearing block used before
# the cap flip.
BOT_POCKET_TOP        = BEARING_LIP_H + BEARING_W   # 8 — top of the fused bottom pocket
BOT_BEARING_BOSS_WALL = 3.0          # collar wall kept around the Ø(BEARING_BORE) pocket
BOT_BEARING_RIB_W     = STRUCT_WALL  # radial-rib tangential width — pinned to STRUCT_WALL


def _bot_bearing_void():
    """Annular air pocket between the bottom bearing-pocket collar and the
    hub OD wall — extends the Ø(HUB_CAVITY_D) spring cavity DOWN through the
    bottom face. The 6 ribs alone tie collar → hub wall."""
    boss_od = BEARING_BORE + 2 * BOT_BEARING_BOSS_WALL
    z_lo    = 0.0 - BOOL_OVERSHOOT                     # through the spool's bottom face
    h       = BOT_POCKET_TOP - z_lo
    return cyl(HUB_CAVITY_D, h, z=z_lo).cut(cyl(boss_od, h, z=z_lo))


def _bot_bearing_ribs():
    """SPOKE_COUNT radial ribs spanning the bottom bearing-pocket collar out
    to the hub OD wall, at the spoke angles."""
    boss_od = BEARING_BORE + 2 * BOT_BEARING_BOSS_WALL
    # Span the bearing collar (inner) to the inner-spool outer wall (HUB_OD),
    # ending flush at the hub OD — no protrusion past it. The inner end
    # overlaps into the collar; the outer end is fully inside the hub wall
    # ring (HUB_CAVITY_D/2..HUB_OD/2) so the union is volumetric.
    r_in    = boss_od / 2 - 0.5            # overlap into the collar
    r_out   = HUB_OD / 2                   # flush with the inner-spool outer wall
    z_lo, z_hi = 0.0, BOT_POCKET_TOP
    out = None
    for i in range(SPOKE_COUNT):
        rib = (
            cq.Workplane("XZ")
            .polyline([(r_in, z_lo), (r_out, z_lo), (r_out, z_hi), (r_in, z_hi)])
            .close()
            .extrude(BOT_BEARING_RIB_W / 2, both=True)
            .rotate((0, 0, 0), (0, 0, 1), i * 360.0 / SPOKE_COUNT)
        )
        out = rib if out is None else out.union(rib)
    return out


# Helical cable groove — half-round channel cut into the drum exterior,
# 7 mm pitch (≈1 mm flat between adjacent turns). Track top at the top of
# the drum-wall stadium hole (z = SPOOL_H − 2); track bottom at
# z = DRUM_BOTTOM_Z. The cradle keeps a Ø(HELIX_GROOVE_D) radius (so the
# Ø5.5 cable nests in it) but only dips GROOVE_DEPTH — its centre is
# pushed outward by the difference. GROOVE_DEPTH = DRUM_WALL − STRUCT_WALL
# so the wall behind the cradle bottom equals STRUCT_WALL; the inner
# relief (below) thins the rest of the drum wall to match → one uniform
# wall thickness everywhere along every helix turn.
HELIX_GROOVE_D     = 6.0      # Ø of the half-round cable cradle (fits the Ø5.5 cable)
HELIX_PITCH        = 7.0
# Top of the helix sits STRUCT_WALL (1.7 mm) below the spool top, matching
# the top-rim-cap thickness and the cable-hole's new top z (levers.py).
# 1.7 mm of unbroken material above guarantees the guide-wheel rim has its
# full STRUCT_WALL of support, with no helix erosion reaching into the rim.
HELIX_Z_TOP        = SPOOL_H - STRUCT_WALL   # 49.3
HELIX_Z_BOT        = DRUM_BOTTOM_Z            # 7  — top of lever-flange slope
# The cable exits the drum at the +y edge of the -x (α=180°) spoke, then
# wraps the drum CLOCKWISE. The helix is rotated so its TOP terminates at
# that same azimuth — the cut doesn't extend CCW past the exit point, so
# none of the drum surface CCW of the spoke face is needlessly grooved
# (it'd never be used by the cable anyway).
_HELIX_START_DEG = math.degrees(math.atan2(
    SPOKE_W / 2,
    -math.sqrt((DRUM_OD / 2) ** 2 - (SPOKE_W / 2) ** 2),
))                                            # ≈ 179.37°
GROOVE_DEPTH       = DRUM_WALL - STRUCT_WALL                          # 2.8
_GROOVE_CIRCLE_R   = HELIX_GROOVE_D / 2                               # 3.0 — cradle radius
_GROOVE_CENTER_R   = DRUM_OD / 2 + (_GROOVE_CIRCLE_R - GROOVE_DEPTH)  # 77.7
# Half-axial extent of the V cut measured at the drum OD: how far above
# or below the helix-tangent z=z0 the cut breaks the outer surface.
# Slightly less than HELIX_PITCH/2 so adjacent turns leave a 0.1 mm flat
# (see _helical_v_groove_cut comment for why exactly 0.05 mm back-off).
HELIX_HALF_PITCH_AT_OD = HELIX_PITCH / 2 - 0.05                        # 3.45
# Lowest z reached by the V-groove cut at the drum OD: at the helix's
# bottom turn the cut extends HELIX_HALF_PITCH_AT_OD axially below the
# tangent point. Other modules use this (e.g. the back-axle's shaft
# length) to align with the bottom of the cable track.
HELIX_OD_BOTTOM_Z  = HELIX_Z_BOT - HELIX_HALF_PITCH_AT_OD              # 3.55


# ── PROOF OF CONCEPT (step 3) — swept-only zig-zag drum wall (no cuts) ────
# Both V-sides, widened to a full pitch so consecutive turns meet FLUSH —
# no flat band, no air gap: a continuous thin zig-zag wall. The swept
# profile is a ">"-shaped chevron — two STRUCT_WALL-thick parallelograms
# (one per V-side) joined at the apex edge, spanning ±HELIX_PITCH/2
# axially so turn k's upper peak edge coincides with turn k+1's lower
# peak edge. All straight profile edges → all ruled-surface (helicoid)
# faces, which is what survives the downstream cap/lever cuts (the round
# C-sweep, with pipe surfaces, collapsed main_body).
def _drum_chevron_poc():
    z0       = HELIX_Z_BOT
    R_out    = DRUM_OD / 2                                # 77.5 — OD / V opening / "peak"
    R_in_od  = DRUM_OD / 2 - STRUCT_WALL                  # 75.8 — wall inner face at the peak
    R_apex_o = DRUM_OD / 2 - GROOVE_DEPTH                 # 74.7 — V apex (cradle bottom), outer
    R_apex_i = DRUM_OD / 2 - GROOVE_DEPTH - STRUCT_WALL   # 73.0 — V apex, inner
    a        = HELIX_PITCH / 2                            # 3.5 — flush turns (a `heal()` below
                                                          # merges the otherwise-coincident peak
                                                          # edges that slicers flag as non-manifold)
    # outer chevron: side-A top → outer apex → side-B top;
    # inner chevron (STRUCT_WALL in): side-B inner → inner apex → side-A inner;
    # closed by the two radial "end cap" edges at the OD ends.
    prof = (
        cq.Workplane("XZ")
        .moveTo(R_out, z0 - a)
        .lineTo(R_apex_o, z0)
        .lineTo(R_out, z0 + a)
        .lineTo(R_in_od, z0 + a)
        .lineTo(R_apex_i, z0)
        .lineTo(R_in_od, z0 - a)
        .close()
    )
    helix_wire = cq.Wire.makeHelix(
        pitch=HELIX_PITCH, height=HELIX_Z_TOP - HELIX_Z_BOT, radius=DRUM_OD / 2,
        center=cq.Vector(0, 0, z0), dir=cq.Vector(0, 0, 1),
    )
    # heal() runs ShapeUpgrade_UnifySameDomain, which merges the coincident
    # peak faces that pitch-wide profile sweeps leave between consecutive
    # turns — cleans up the non-manifold seams slicers flag.
    return heal(prof.sweep(cq.Workplane(obj=helix_wire), isFrenet=True))


def _helix_sweep(helix_radius: float, circle_r: float):
    """Sweep a circle of radius `circle_r` (centred on the helix) along a
    helix of pitch HELIX_PITCH wound at `helix_radius`, over the cable-
    groove z-span. Frenet frame keeps the profile perpendicular to the
    tangent (the circle is symmetric, so the exact frame is moot). Used
    for both the cable cradle and the inner relief tube — same shape,
    different radius."""
    helix_wire = cq.Wire.makeHelix(
        pitch=HELIX_PITCH, height=HELIX_Z_TOP - HELIX_Z_BOT, radius=helix_radius,
        center=cq.Vector(0, 0, HELIX_Z_BOT), dir=cq.Vector(0, 0, 1),
    )
    profile = cq.Workplane("XZ").moveTo(helix_radius, HELIX_Z_BOT).circle(circle_r)
    return profile.sweep(cq.Workplane(obj=helix_wire), isFrenet=True)


def _helical_cable_groove():
    return _helix_sweep(_GROOVE_CENTER_R, _GROOVE_CIRCLE_R)


# V-notch helical cable groove — applied as a CUT to the thick-wall drum
# at the very END of build.py (after every cap/lever cut), so the
# fragile helical-swept geometry never has to survive another boolean.
# The profile is a triangle whose apex points radially INWARD: apex at
# the desired cradle bottom (r = DRUM_OD/2 − GROOVE_DEPTH = 74.7), with
# the two outer corners straddling the apex axially by ±HELIX_PITCH/2 at
# a radius past the OD (so the cut breaks the surface cleanly). Sweeping
# this triangle along a helix at the apex radius carves a full-pitch
# V-groove — adjacent turns meet flush at the OD, giving the "angled
# outer wall" the chevron POC had, without the chevron's fragility.
_V_APEX_R   = DRUM_OD / 2 - GROOVE_DEPTH         # 74.7 — radial depth of the cut
_V_OUTER_R  = DRUM_OD / 2 + BOOL_OVERSHOOT       # past the OD so the cut surfaces cleanly


def _helical_v_groove_cut():
    z0 = HELIX_Z_BOT
    # The V must open the FULL pitch *at the OD* (not at the overshoot
    # radius) so adjacent turns meet flush at the outer surface with zero
    # flat band between them. The triangle's apex is at (_V_APEX_R, z0);
    # we extend the profile's outer corners past the OD on the line whose
    # axial half-width is HELIX_PITCH/2 *at r = DRUM_OD/2*. That overshoot
    # ensures the cut breaks the OD surface cleanly even after the
    # downstream booleans nudge things by float-eps.
    # Back off 0.05 mm from "exactly flush" so adjacent turns' cuts leave
    # a 0.1 mm flat at the OD instead of (a) touching at a knife edge
    # (OCCT throws Standard_OutOfRange) or (b) overlapping (the single
    # swept solid self-intersects between turns and produces a degenerate
    # boolean). A 0.1 mm flat is well below print resolution (~0.4 mm
    # extrusion width), so it's effectively invisible in the print.
    slope_dz_dr = HELIX_HALF_PITCH_AT_OD / (DRUM_OD / 2 - _V_APEX_R)
    a  = slope_dz_dr * (_V_OUTER_R - _V_APEX_R)   # extrapolated half-width at the overshoot radius
    # Triangle profile in XZ: apex inward at (_V_APEX_R, z0); outer top
    # and outer bottom at the overshoot radius, straddling z0 by ±a.
    prof = (
        cq.Workplane("XZ")
        .moveTo(_V_APEX_R, z0)
        .lineTo(_V_OUTER_R, z0 + a)
        .lineTo(_V_OUTER_R, z0 - a)
        .close()
    )
    helix_wire = cq.Wire.makeHelix(
        pitch=HELIX_PITCH, height=HELIX_Z_TOP - HELIX_Z_BOT, radius=_V_APEX_R,
        center=cq.Vector(0, 0, z0), dir=cq.Vector(0, 0, 1),
    )
    swept = prof.sweep(cq.Workplane(obj=helix_wire), isFrenet=True)
    # makeHelix starts at angle 0° (at z=HELIX_Z_BOT) and accumulates
    # (height/pitch)·360° by the top. Rotate by (target − top_angle) so
    # the TOP of the helix — where the cable exits the drum — lands on
    # the spoke's +y face. (The bottom's azimuth is wherever it falls;
    # nothing depends on it.)
    top_angle_deg = ((HELIX_Z_TOP - HELIX_Z_BOT) / HELIX_PITCH) * 360.0
    return swept.rotate((0, 0, 0), (0, 0, 1), _HELIX_START_DEG - top_angle_deg)


# Groove-following inner relief — makes the drum wall a constant STRUCT_WALL.
# The wall behind the cradle bottom is already STRUCT_WALL (by GROOVE_DEPTH);
# everywhere else (cradle edges, bands) it's the full DRUM_WALL. We carve
# the inside down to the SAME cradle profile shifted radially inward by
# STRUCT_WALL — an inner cradle tube centred STRUCT_WALL below the outer
# cradle centre. The material to remove = that tube subtracted from the
# annular shell DRUM_ID/2 → (DRUM_OD/2 − STRUCT_WALL): in the bands the
# whole shell goes (inner surface → DRUM_OD/2 − STRUCT_WALL); under the
# cradle the tube fills the shell so nothing is removed (inner surface
# stays DRUM_ID/2); at the edges it transitions smoothly. Restricting to
# that shell (r ≥ DRUM_ID/2) keeps the cut off the hub/spokes; the spokes
# still land on the recessed wall at every cradle crossing along their
# height.
def _helical_groove_following_relief():
    z_lo = HELIX_Z_BOT - BOOL_OVERSHOOT
    h    = (HELIX_Z_TOP - HELIX_Z_BOT) + 2 * BOOL_OVERSHOOT
    shell = (
        cyl(2 * (DRUM_OD / 2 - STRUCT_WALL), h, z=z_lo)      # r ≤ DRUM_OD/2 − STRUCT_WALL
        .cut(cyl(DRUM_ID, h, z=z_lo))                        # minus r ≤ DRUM_ID/2 → annular shell
    )
    inner_tube = _helix_sweep(_GROOVE_CENTER_R - STRUCT_WALL, _GROOVE_CIRCLE_R)
    return shell.cut(inner_tube)
# NB: spoke buttresses (keeping a full-thickness wall sector at each spoke)
# were tried both ways — cutting buttress boxes out of the relief, and
# unioning solid wedges back afterward. Both make OCCT collapse the model
# (the helical relief sweep leaves geometry too fragile for further
# booleans). Dropped: the spokes still meet the drum at every cradle
# crossing along their height (the relief's inner surface dips back to
# DRUM_ID/2 there) with the same 0.5 mm overlap used elsewhere; the
# drum→flange→spoke→hub path stays fully continuous regardless.


# Bearing pocket bottom Z — exported for viz.py's dummy-bearing
# placement in the assembly STEP.
pancake_bearing_z0 = PANCAKE_CAP_SEAT_Z0


# (The OD-88 pancake-spool floor disc that used to sit at z=49..51 was
# removed; the source-cable wrap groove no longer has a spool-side floor.
# The cable now exits the drum interior upward through the open spool top
# and reaches the cone via a dedicated entry cut through the hub wall +
# bearing_cap_top, defined in caps.py.)

# Spring outer-end attachment slot — a blind radial notch in the hub
# wall sized to receive the spring's bent-over outer tab (12.7 mm z x
# 0.15 mm thick). Orientation: tall in z (matching the strip width),
# narrow in y (fits the strip thickness with tolerance), shallow in x.
# Blind outer face leaves ≥1.5 mm of hub-OD wall intact — no slits.
#
# Conical bottom: the slot's lower face is bounded by the LEVER cap-
# stop lip's 45° cone surface, extended UPWARD into the spring cavity.
# Using the cone itself to trim the slot guarantees flush alignment at
# every y, not just y=0 (a flat-plane approximation would drift
# ~0.03 mm off the cone at y=±SPRING_SLOT_W/2).
SPRING_SLOT_H_RECT = 23.0   # rectangle height (z_bottom to outer-edge roof).
                            # Sized for the 21.8 mm Stanley tape blade with
                            # 1.2 mm of axial play.
SPRING_SLOT_W      =  2.0   # y-extent (perpendicular to blade flat) — fits
                            # 0.1 mm tape with bend clearance
SPRING_SLOT_DEPTH  =  4.0   # x-extent (radial). Cuts all the way through
                            # the 3 mm hub wall (with 0.5 mm overshoot
                            # each side) — tape blade threads through
                            # the slot from the cavity, gets bent on the
                            # outer face to lock in place.
SPRING_SLOT_ANGLE_DEG = 270.0  # angular position of the slot. 270° sits
                            # halfway between the 240° and 300° spokes —
                            # one spoke-section CCW from 210° to clear the
                            # cable entry hole through the 180° spoke.

def _spring_slot():
    slot_x_outer = -(HUB_CAVITY_D / 2 + SPRING_SLOT_DEPTH - 0.5)   # outer x face of slot
    slot_x_inner = -(HUB_CAVITY_D / 2 - 0.5)                       # inner x face (cavity-side)
    # At outer x (r=|slot_x_outer|), the lever-side cap-stop-lip cone surface
    # extending UP into the cavity reaches r=|slot_x_outer| at this z; this
    # is the slot's lower bound at that x.
    slot_zb_at_outer = LEVER_STOP_LIP_Z1 + (SPRING_SLOT_DEPTH - 0.5)
    slot_ztop = slot_zb_at_outer + SPRING_SLOT_H_RECT             # slot's upper face
    lip_apex_z = LEVER_STOP_LIP_Z1 - HUB_CAVITY_D / 2              # cone apex (below spool body)

    # Rectangular prism that overshoots BELOW the apex in z; the cone
    # intersection below trims its bottom surface to the cone.
    slot_prism = (
        cq.Workplane("XY")
        .workplane(offset=lip_apex_z - 1.0)
        .center((slot_x_outer + slot_x_inner) / 2, 0)
        .box(
            abs(slot_x_inner - slot_x_outer),
            SPRING_SLOT_W,
            slot_ztop - (lip_apex_z - 1.0),
            centered=(True, True, False),
        )
    )

    # The lever-side cap-stop-lip cone, extended UPWARD from its apex —
    # shifted 0.02 mm DOWNWARD from the true lip apex so the slot's roof
    # surface is ever so slightly DEEPER than the actual lip cone (by
    # 0.02 mm / 20 µm at each z). Without this offset, the cut operation
    # produces two mathematically coincident cone surfaces (the lip's
    # topside and the slot's floor), which OCCT's boolean fails to merge
    # cleanly and marks the result invalid.
    lip_cone_offset = 0.02
    lip_cone_height = SPOOL_H - lip_apex_z + 1.0    # tall enough to cover the slot
    lip_cone_ext = cq.Workplane("XY").add(cq.Solid.makeCone(
        0.001, lip_cone_height + 0.001, lip_cone_height,
        pnt=cq.Vector(0, 0, lip_apex_z - lip_cone_offset),
        dir=cq.Vector(0, 0, 1),
    ))

    slot = slot_prism.intersect(lip_cone_ext)
    # Rotate from constructed position at 180° (-x) to SPRING_SLOT_ANGLE_DEG.
    return slot.rotate((0, 0, 0), (0, 0, 1), SPRING_SLOT_ANGLE_DEG - 180.0)


main_body = _build_main_body()
