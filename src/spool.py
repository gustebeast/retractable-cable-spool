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
    CABLE_RIM_AIR_GAP, TOP_RIM_H,
    CAP_H, CAP_STOP_ID, CAP_STOP_LIP_H,
    DRUM_BOTTOM_Z, DRUM_H, DRUM_ID, DRUM_OD, DRUM_TOP_Z, DRUM_WALL,
    FLANGE_H, FLANGE_INNER_ID, FLANGE_OD, FLANGE_LIP_T,
    HUB_CAVITY_D, HUB_OD,
    M2_INSERT_PILOT_D, M2_INSERT_DEPTH, M2_SHAFT_CLR_D,
    RATCHET_TEETH, RATCHET_DEPTH, RATCHET_TOOTH_OFFSET_DEG,
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
        # _hub_key_bumps() (top-rim tenons) is unioned LATER — after the
        # cable-entry tunnel cut — otherwise the diagonal tunnel clips one
        # of the tenons at the bottom of the hub.
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
        # Top cap-stop lip — the removable top cap can't slide down into the
        # spring cavity. The bore narrows UPWARD (HUB_CAVITY_D at the cavity, to
        # CAP_STOP_ID at the cap seat) so the lip's underside is a self-supporting
        # 45° cone (−z) and its top is a flat ledge (+z) the cap rests on — prints
        # support-free bottom-up. (Flipping these diameters would put the flat
        # overhang on −z, which is what we're avoiding.)
        .cut(cone_solid(HUB_CAVITY_D, CAP_STOP_ID, CAP_STOP_LIP_H, _CAVITY_TOP))
        # Top cap seat — HUB_CAVITY_D bore the removable cap drops into.
        .cut(cyl(HUB_CAVITY_D, SPOOL_H - PANCAKE_CAP_SEAT_Z0, z=PANCAKE_CAP_SEAT_Z0))
    )
    # Lighten the fused bottom bearing block: hollow the annulus between the
    # bearing collar and the hub wall, leaving SPOKE_COUNT (6) radial ribs —
    # the same design the pancake bearing block used before the cap flip.
    main_body = main_body.cut(_bot_bearing_void())
    main_body = main_body.union(_bot_bearing_ribs())
    # Spring-strip attachment: an M2 screw + heat-set insert clamps the drilled
    # tape against the hub. An elevated square boss hosts the insert (the 1.7 mm
    # wall is too thin); the radial bore takes the insert pocket + screw clearance.
    main_body = main_body.union(_spring_screw_boss())
    main_body = main_body.cut(_spring_screw_bore())
    # Driver access: Ø7.2 hole through the hub wall diametrally OPPOSITE
    # the spring screw, so a hex driver can reach the screw head (which
    # faces the cavity) from outside the spool.
    main_body = main_body.cut(_spring_screw_access_hole())
    # Build a walled channel through the spokes first, then bore the cable
    # tunnel through it. Without the shell the tunnel just chops the spokes
    # and leaves dangling stubs around the cable's path.
    main_body = main_body.union(_cable_entry_channel_shell())
    main_body = main_body.cut(_cable_entry_cut())
    # Top-rim tenons added AFTER the tunnel cut so they aren't clipped by it.
    main_body = main_body.union(_hub_key_bumps())
    return main_body


# ── Inner-spool-body cable rim (pancake rewrite, v2) ────────────────────────
# The cable winds in the annular channel BETWEEN the hub OD and the rim's
# inner wall, in a single Z-plane band RIM_H tall, stacking radially (and a
# couple of layers axially). The ratchet teeth + brake track will live on
# the rim's OUTER cylindrical face (radial-X load, structurally sound across
# the cable air-gap) — that's the upcoming lever rework, not done here. The
# rim is a plain vertical wall (no 45° chamfer) so the whole body prints
# bottom-to-top without supports.
# The rim splits into a short RATCHET teeth band at the bottom and a tall
# SMOOTH brake band filling the rest. RIM_H is derived from the two band
# heights so bumping either one walks the entire stack above the rim
# (cable groove, retainer, cable top rim, hub spring-mount hole) up in Z.
RATCHET_BAND_H = 6.0                  # ratchet teeth z-height (3→6: doubled
                                      # pawl contact area — teeth were skipping)
# 45° conical transition between the ratchet rim and the (wider) brake rim.
# At 45°, the cone's axial height equals its radial step = RATCHET_DEPTH.
CONE_H         = RATCHET_DEPTH        # 1.7
# The +Z extent of the ratchet_pin_chunk (housing.py). The chunk's cut
# plane is z − y ≤ RATCHET_CHUNK_CUT_C, so its top point is at y = +HOUSING_W/2
# where z reaches RATCHET_CHUNK_CUT_C + HOUSING_W/2. Mirrored here as a
# literal (with the same derivation) so the brake-rim height can be sized
# off it without spool.py needing to import from housing.py (which would
# be circular). Update both together if any of the inputs change.
#   = (RATCHET_PIVOT_Z + LEVER_PIVOT_BOSS_OD/2)        # boss top z       = 19.1
#   + 0.5                                              # boss inside-y offset
#   + RATCHET_CHUNK_RING_CLR · √2                      # 0.4·√2 = 0.566
#   = 20.166
# (RATCHET_PIVOT_Z = 16.1 — derived in housing.py from the 6 mm guide
# wheel: BRAKE_RIM_Z_LO 7.5 + wheel 6.0 + face clr 1.0 + 0.4 solid +
# M2 bore half 1.2.)
RATCHET_CHUNK_Z_MAX = 20.166
# Manual clearance above the ratchet_pin_chunk's top — added to anything
# that needs to sit ABOVE the chunk (the cable retainer's bottom ring, the
# housing's front-thicken top) so the chunk has a small breathing margin.
RATCHET_CHUNK_CLEARANCE = 0.8
# Brake-rim STRAIGHT section z-height — just the cylindrical portion above
# the cone, NOT including the cone itself. Sized directly from what must
# fit on the band:
#   BRAKE_PAD_TARGET_H  — brake-pad contact height
#   + 1.0               — clearance between the pad's top and the retainer
#   + STRUCT_WALL       — the retainer's bottom ring (RETAIN_RING_T) wraps
#                         the band's top
# (Previously sized so the retainer's ring cleared the ratchet chunk's top
# — that tied the band height to the whole lever-pivot stack and made the
# band ~3.5 mm taller than the brake needs. The chunk split / retainer /
# ratchet-axle interactions are being redesigned separately, so the chunk
# clearance constraint is dropped here.)
BRAKE_PAD_TARGET_H = 9.0
BRAKE_BAND_H   = BRAKE_PAD_TARGET_H + 1.0 + STRUCT_WALL          # 11.6
# Z of the brake rim's STRAIGHT section bottom — top of the cone, also where
# the brake rim's cylindrical wall meets the cone's expanding face. Stationary
# things on the spool's side that need to align with the brake rim's bottom
# (e.g. the guide wheel) key off this.
BRAKE_RIM_Z_LO = RATCHET_BAND_H + CONE_H              # 4.7
RIM_H      = RATCHET_BAND_H + CONE_H + BRAKE_BAND_H   # rim total height (15.766)
# RIM_OD is the brake-band OD that the brake lever, the cable top rim, and
# the cable retainer all key off. Bumped 114.4→117.8 so the brake band's
# OUTER matches the ratchet TOOTH TIP extent (RIM_OD/2 = 58.9 = old
# r_tip) — no more overhang of the teeth past the brake band's wall.
# A 45° cone (RATCHET_DEPTH tall) below the brake band transitions the
# wall outward from the ratchet valley radius up to this new brake outer.
# Wall thickness stays at STRUCT_WALL (1.7) throughout.
RIM_OD     = 117.8                    # new brake-band outer (= old r_tip)
RIM_WALL   = STRUCT_WALL              # 1.7 — brake-band wall thickness
# RIM_ID is the cable-channel inner edge at the NARROWEST point of the rim
# (the ratchet level — kept at the old value so the ratchet bore, the cable
# channel spokes, and the cable_rim spoke count are unchanged). The BRAKE
# band's inner is (RIM_OD - 2*RIM_WALL) = 114.4 — larger, because the brake
# band grew outward but the wall stayed 1.7 mm. The cone smoothly steps
# the bore from RIM_ID up to that larger inner over RATCHET_DEPTH in z.
RIM_ID     = 111.0                    # cable-channel inner at ratchet level

# Minimum radial gap between the spool's OD and any STATIONARY feature
# (housing walls/thicken, cable retainer's bottom ring, etc.) — covers spool
# wobble under load. Used by cable_retainer.RETAIN_RADIAL_GAP and by
# housing.PIVOT_X (the front-thicken's inner face is held at this gap).
SPOOL_HORIZONTAL_CLEARANCE = 2.5


def _cyl_ratchet_band(z_lo, z_hi):
    """Cylindrical (radial-engagement) ratchet on the rim's OUTER face over
    [z_lo, z_hi]. A 90°-rotated version of the old axial ratchet: the
    sawtooth runs around the circumference with its catch face RADIAL, so
    the pawl engages from OUTSIDE in X/Y. Tip radius = RIM_OD/2 (flush
    with the brake-band outer above the cone); teeth grow INWARD from
    there by RATCHET_DEPTH to the valleys at r = RIM_OD/2 − RATCHET_DEPTH."""
    r_tip  = RIM_OD / 2
    r_root = r_tip - RATCHET_DEPTH
    n      = RATCHET_TEETH
    offset = math.radians(RATCHET_TOOTH_OFFSET_DEG)
    pts = []
    for i in range(n):
        ts = offset + 2 * math.pi * i / n
        # radial catch face (tip → valley at the same angle), then the
        # ramp is the straight chord from this valley out to the next tip.
        # (Swapped from valley-first to tip-first to flip the engagement
        # direction — catch now blocks the opposite rotation.)
        pts.append((r_tip  * math.cos(ts), r_tip  * math.sin(ts)))
        pts.append((r_root * math.cos(ts), r_root * math.sin(ts)))
    star = (
        cq.Workplane("XY").workplane(offset=z_lo)
        .polyline(pts).close()
        .extrude(z_hi - z_lo)
    )
    # Carve the bore so it's an annular toothed ring (ID = RIM_ID).
    return star.cut(cyl(RIM_ID, z_hi - z_lo, z=z_lo))


CABLE_D = 6.0   # nominal cable diameter — also the max allowed inter-spoke gap

CABLE_ENTRY_CUT_W         = 13.0   # square cross-section of the cable-entry cut
CABLE_ENTRY_CUT_ANGLE_DEG = 45.0   # tilt from horizontal: cable enters at this angle
                                   # so it exits the cut near the angle it sits at
                                   # in the (flat) channel.


# Cut's axis position — the box's INNER X face (at axis − CABLE_ENTRY_CUT_W/2)
# is at constant X = HUB_OD/2, so at the EXIT (top of cut, Y=0) the tunnel is
# tangent to the hub OD: the cable comes out right at the inner spool wall.
# The shell shares this axis so STRUCT_WALL extends symmetrically on every
# side of the cut (no lopsidedness in either X direction).
_CABLE_ENTRY_AXIS_X = HUB_OD / 2 + CABLE_ENTRY_CUT_W / 2


def _cable_entry_prism(w):
    """The diagonal cable-entry tunnel as a rectangular prism, centred on
    the cut's axis (_CABLE_ENTRY_AXIS_X), parameterised by cross-section
    side `w`. Used twice — once at CABLE_ENTRY_CUT_W to carve the tunnel
    itself, and once at CABLE_ENTRY_CUT_W + 2·STRUCT_WALL as a
    STRUCT_WALL-thick channel shell that caps the spokes the tunnel cuts
    through (so the cable runs in a clean walled channel instead of past
    a row of dangling spoke stubs)."""
    angle_rad = math.radians(CABLE_ENTRY_CUT_ANGLE_DEG)
    sin_a = math.sin(angle_rad)
    # z_start needs to drop just past the z=0 cap so the tilted box's
    # cross-section at z=0 is limited by its LOCAL width (perpendicular to
    # axis), not by the diagonal edge running through the slice. ~1 mm of
    # extra extension below the original -5 is enough.
    z_start = -6.0
    z_end = RIM_H + 5.0
    L = (z_end - z_start) / sin_a
    y_exit  = 0.0
    y_entry = y_exit + (z_end - z_start)            # Δy = -Δz at 45° (axis along -Y +Z)
    center_y = (y_entry + y_exit) / 2
    center_z = (z_start + z_end) / 2
    return (
        cq.Workplane("XY")
        .box(w, w, L, centered=True)
        # Rotate +45° around X so local +Z (length) → world -Y +Z (CW-winding tangent).
        .rotate((0, 0, 0), (1, 0, 0), CABLE_ENTRY_CUT_ANGLE_DEG)
        .translate((_CABLE_ENTRY_AXIS_X, center_y, center_z))
    )


# Spoke region: tunnel + shell are capped to this z range so they don't run
# above/below the spokes they need to cross. The prism itself extends past
# both ends (entry/exit clearance); the cap clips that to z = 0..16.47.
_SPOKE_REGION_Z_HI = 16.47


def _spoke_region_cap():
    """Z-bounded box (z = 0..16.47, full XY span) intersected with the
    cable-entry tunnel/shell to clip them to the spoke region."""
    big = 1000.0
    return (cq.Workplane("XY")
            .box(big, big, _SPOKE_REGION_Z_HI, centered=(True, True, False)))


def _cable_entry_cut():
    """Inner cable-entry tunnel — the actual cable clearance hole, clipped
    to the spoke region."""
    return _cable_entry_prism(CABLE_ENTRY_CUT_W).intersect(_spoke_region_cap())


def _cable_entry_channel_shell():
    """STRUCT_WALL-thick rectangular shell around the cable-entry tunnel.
    Unioned into the spool BEFORE the tunnel is cut so the spokes the
    tunnel crosses get rejoined by a solid border — no dangling stubs.
    Clipped to the spoke region so the shell doesn't extend above/below."""
    return (_cable_entry_prism(CABLE_ENTRY_CUT_W + 2 * STRUCT_WALL)
            .intersect(_spoke_region_cap()))


def _channel_spokes():
    """Two-level (48→24→12) branching radial spokes (sole hub↔rim
    connection — no floor disc). Each spoke is built as a TRUE radial
    bar at its own angular position (not a Cartesian-offset chord), so
    the top profile at z=RIM_H matches the original 48-spoke pattern
    exactly. Tilted segments use twist-extrude to rotate the cross-
    section angularly with Z.

    Spoke OUTER follows the rim's INNER wall profile (stays 0.5 mm INTO
    the wall) wherever the rim profile applies — for the prongs and
    mid-trunks (entirely in the brake band) that's r_out_hi; for the
    ground trunk (spans the cone and ratchet band) it's a polygon
    following the cone.

    Topology, per branch (12 branches × 4 prongs = 48 cable contacts):
      prongs A,B,C,D at original spoke angles (4j..4j+3) × spoke_pitch
      pair-merge: A+B → mid-trunk L at (4j+0.5)×spoke_pitch
                  C+D → mid-trunk R at (4j+2.5)×spoke_pitch
      final merge: L+R → ground trunk G at (4j+1.5)×spoke_pitch

    Tilt geometry: each merge is 45° at the OUTER radius (r_out_hi),
    which is the steepest point of each tilted spoke. At inner radii
    the tilt is shallower (~26° at the hub), well under the 45°
    print-overhang limit. Merge heights:
       merge_h1 = r_out_hi · spoke_pitch / 2 ≈ 3.78 mm  (48→24)
       merge_h2 = r_out_hi · spoke_pitch     ≈ 7.57 mm  (24→12)
    Ground trunks span z ∈ [0, RIM_H − merge_h1 − merge_h2]."""
    r_in          = HUB_OD / 2 - 0.5
    brake_inner   = (RIM_OD - 2 * RIM_WALL) / 2
    z_ratchet_top = RATCHET_BAND_H
    z_brake_lo    = z_ratchet_top + CONE_H
    r_out_lo      = RIM_ID / 2 + 0.5
    r_out_hi      = brake_inner + 0.5
    w             = STRUCT_WALL

    n_top = math.ceil(2 * math.pi * brake_inner / (CABLE_D + w))
    n_top = 4 * ((n_top + 3) // 4)                    # multiple of 4
    n_branches = n_top // 4
    spoke_pitch = 2 * math.pi / n_top

    # 45° at the outer radius (where tilt is steepest).
    merge_h1 = r_out_hi * spoke_pitch / 2             # 48→24
    merge_h2 = r_out_hi * spoke_pitch                 # 24→12
    z_split1_bot = RIM_H - merge_h1                   # ~12.0
    z_split2_bot = z_split1_bot - merge_h2            # ~4.4

    def twisted_segment(theta_top_rad, theta_bot_rad, z_top, z_bot, r_out):
        """A radial bar (r_in → r_out, width w tangential) whose cross
        section is at angle theta_bot at z_bot and angle theta_top at
        z_top. Implemented as a twistExtrude of a flat rectangle along
        +Z, then translated to z_bot and rotated to theta_bot in world."""
        delta_z = z_top - z_bot
        delta_theta_deg = math.degrees(theta_top_rad - theta_bot_rad)
        profile = (cq.Workplane("XY")
                   .moveTo(r_in, -w / 2)
                   .rect(r_out - r_in, w, centered=False))
        seg = profile.twistExtrude(delta_z, delta_theta_deg)
        return (seg.translate((0, 0, z_bot))
                   .rotate((0, 0, 0), (0, 0, 1), math.degrees(theta_bot_rad)))

    def ground_trunk(theta_rad):
        """Vertical radial spoke at angle theta following the rim
        profile (cone + ratchet band) from z=0 to z_split2_bot."""
        pts = [(r_in, 0.0), (r_out_lo, 0.0)]
        if z_split2_bot >= z_ratchet_top:
            pts.append((r_out_lo, z_ratchet_top))
            if z_split2_bot >= z_brake_lo:
                pts.append((r_out_hi, z_brake_lo))
                pts.append((r_out_hi, z_split2_bot))
            else:
                t = (z_split2_bot - z_ratchet_top) / (z_brake_lo - z_ratchet_top)
                r_at_top = r_out_lo + t * (r_out_hi - r_out_lo)
                pts.append((r_at_top, z_split2_bot))
        else:
            pts.append((r_out_lo, z_split2_bot))
        pts.append((r_in, z_split2_bot))
        return (cq.Workplane("XZ").polyline(pts).close()
                .extrude(w / 2, both=True)
                .rotate((0, 0, 0), (0, 0, 1), math.degrees(theta_rad)))

    out = None
    for j in range(n_branches):
        theta_ground = (4 * j + 1.5) * spoke_pitch
        theta_mid_L  = (4 * j + 0.5) * spoke_pitch
        theta_mid_R  = (4 * j + 2.5) * spoke_pitch

        # 4 prongs: each tilts from its own spoke angle to its mid-trunk angle.
        for k in range(4):
            theta_prong = (4 * j + k) * spoke_pitch
            theta_mid = theta_mid_L if k < 2 else theta_mid_R
            prong = twisted_segment(theta_prong, theta_mid,
                                    RIM_H, z_split1_bot, r_out_hi)
            out = prong if out is None else out.union(prong)

        # 2 mid-trunks: each tilts from its mid-trunk angle to ground trunk.
        for theta_mid in (theta_mid_L, theta_mid_R):
            mid = twisted_segment(theta_mid, theta_ground,
                                  z_split1_bot, z_split2_bot, r_out_hi)
            out = out.union(mid)

        # 1 ground trunk: vertical, rim-following.
        out = out.union(ground_trunk(theta_ground))

    return out


def compare_spoke_top_profile():
    """Verify the spoke top profile matches the original 48-spoke design.
    Prints angular position of each prong's inner and outer endpoints
    compared to a pure-radial 48-spoke reference. With the twist-extrude
    implementation in _channel_spokes() each prong is a TRUE radial bar,
    so the angles should match exactly (within float precision)."""
    brake_inner = (RIM_OD - 2 * RIM_WALL) / 2
    r_out_hi    = brake_inner + 0.5
    r_in_local  = HUB_OD / 2 - 0.5
    n_top = math.ceil(2 * math.pi * brake_inner / (CABLE_D + STRUCT_WALL))
    n_top = 4 * ((n_top + 3) // 4)
    spoke_pitch = 2 * math.pi / n_top

    print(f"\n  n_top = {n_top} prongs, spoke_pitch = {math.degrees(spoke_pitch):.3f}°")
    print(f"  r_in = {r_in_local:.2f}, r_out_hi = {r_out_hi:.2f}")
    print(f"  {'k':>3} {'orig (°)':>10} {'new outer (°)':>14} {'Δouter (°)':>11} "
          f"{'new inner (°)':>14} {'Δinner (°)':>11}")
    print(f"  {'-'*3} {'-'*10} {'-'*14} {'-'*11} {'-'*14} {'-'*11}")

    # All 4 prongs in branch j=0 (other branches are rotated copies — same Δ).
    for k in range(4):
        theta_prong = k * spoke_pitch
        orig_deg = math.degrees(theta_prong)
        # New: true radial bar at theta_prong (both endpoints at same angle).
        new_outer_deg = math.degrees(math.atan2(
            r_out_hi * math.sin(theta_prong), r_out_hi * math.cos(theta_prong)))
        new_inner_deg = math.degrees(math.atan2(
            r_in_local * math.sin(theta_prong), r_in_local * math.cos(theta_prong)))
        print(f"  {k:>3} {orig_deg:>10.3f} {new_outer_deg:>14.3f} "
              f"{new_outer_deg - orig_deg:>+11.3e} "
              f"{new_inner_deg:>14.3f} {new_inner_deg - orig_deg:>+11.3e}")


def _hub_key_bumps():
    """6 vertical key ridges on the hub OD at KEY_ANGLES, full hub height.
    They (a) key the sliding cable top-rim against rotation, and (b) sit
    radially aligned with the top bearing-cap key grooves on the INSIDE of
    the hub wall (z=43..51), adding material back across that thinned
    section. KEY_DEPTH (1 mm) tall, KEY_W (2 mm) wide."""
    return make_keys(HUB_OD / 2, 0.0, SPOOL_H)


def _lever_rim():
    """Bottom cable rim — the RIM_H-tall outer wall only (NO floor disc; the
    spokes carry the hub↔rim connection). Three concentric stacked sections:
      - z=[0, RATCHET_BAND_H=3]: ratchet teeth (tip at RIM_OD/2, valley at
        RIM_OD/2 − RATCHET_DEPTH, bore at RIM_ID).
      - z=[RATCHET_BAND_H, RATCHET_BAND_H + CONE_H=4.7]: 45° cone
        transition. Both inner and outer surfaces slope outward by
        RATCHET_DEPTH (=1.7) over the same axial distance, so the wall
        thickness stays at STRUCT_WALL throughout.
      - z=[RATCHET_BAND_H + CONE_H, RIM_H]: BRAKE rim — smooth cylinder of
        height BRAKE_BAND_H, outer at RIM_OD/2, inner at RIM_OD/2 −
        STRUCT_WALL (= 57.2). This is "the brake rim".
    Bands swapped (ratchet low, brake high) so the brake lever's pivot —
    which must sit BELOW its band — lands above the spool bottom instead of
    being dragged negative, keeping the levers/housing from hanging as far
    below the spool. Cable winds in the hub→rim channel on the spoke grid."""
    z_ratchet_top = RATCHET_BAND_H                              # 3
    z_brake_lo    = BRAKE_RIM_Z_LO                              # 4.7
    brake_id      = RIM_OD - 2 * RIM_WALL                       # 114.4

    ratchet = _cyl_ratchet_band(0.0, z_ratchet_top)

    # 45° transition cone: outer goes from (RIM_OD − 2·RATCHET_DEPTH) at the
    # ratchet's top up to RIM_OD at the brake-rim bottom; inner does the
    # same shift (RIM_ID → brake_id), maintaining a STRUCT_WALL wall.
    cone_outer = cone_solid(RIM_OD - 2 * RATCHET_DEPTH, RIM_OD,
                            CONE_H, z_ratchet_top)
    cone_inner = cone_solid(RIM_ID, brake_id,
                            CONE_H, z_ratchet_top)
    cone = cone_outer.cut(cone_inner)

    # Brake rim — straight cylinder above the cone (BRAKE_BAND_H tall).
    brake = (
        cyl(RIM_OD, BRAKE_BAND_H, z=z_brake_lo)
        .cut(cyl(brake_id, BRAKE_BAND_H, z=z_brake_lo))
    )
    return ratchet.union(cone).union(brake)


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
BOT_BEARING_BOSS_WALL = STRUCT_WALL  # collar wall kept around the Ø(BEARING_BORE) pocket
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

# Spring-strip attachment: M2 screw + heat-set insert.
# The clock-spring's outer tape is drilled (2 mm hole) and clamped against the
# hub by an M2 x 8 low-profile socket-head screw (McMaster 92855A839); the screw
# head sits on the INSIDE (cavity side) and threads into a McMaster 94459A110
# M2 heat-set insert (2.5 mm long) pressed into the hub wall. The 1.7 mm
# structural wall is too thin for the 3.5 mm insert pocket (2.5 mm + 1 mm
# margin), so an elevated square boss (45 deg chamfer all round, for print support
# + a clean blend) is added to the cavity wall to host it. The insert sits FLUSH
# with the boss pad; the screw may protrude past the hub OD (acceptable - it's
# clear of the cable channel, which is lower in z).
SPRING_SCREW_ANGLE = 270.0
# Cable top-rim sits CABLE_RIM_AIR_GAP above the bottom rim's top (= RIM_H); its
# lid body is TOP_RIM_H tall. (build.py places it at CABLE_TOP_RIM_BASE_Z.)
CABLE_TOP_RIM_BASE_Z = RIM_H + CABLE_RIM_AIR_GAP        # lid bottom face
_CABLE_TOP_RIM_TOP_Z = CABLE_TOP_RIM_BASE_Z + TOP_RIM_H  # lid top face
# Put the screw hole so its lower edge (the Ø M2_SHAFT_CLR_D shank hole that exits
# the OD) is FLUSH with the lid's top — so the protruding screw clears the lid at
# its maximum spacing. Tracks the air gap automatically: bump the gap → both the
# lid and this hole move up together.
SPRING_SCREW_Z     = _CABLE_TOP_RIM_TOP_Z + M2_SHAFT_CLR_D / 2
SCREW_BOSS_H       = 2.0    # radial height the boss protrudes inward from wall A
                            # (boss + 1.7 mm wall = 3.7 mm > the M2_INSERT_DEPTH pocket)
SCREW_BOSS_SIDE    = 8.0    # square pad side at the boss face (the strip seats here)
# Insert pilot / depth and screw clearance reuse the shared M2 heat-set-insert
# constants (M2_INSERT_PILOT_D = 3.3, M2_INSERT_DEPTH = 3.5, M2_SHAFT_CLR_D = 2.4),
# which are already sized for the McMaster 94459A110 insert — see dimensions.py.


def _spring_screw_boss():
    """Elevated square boss on the cavity wall (45 deg chamfer all round) that
    hosts the heat-set insert - it adds the radial depth the 1.7 mm wall lacks.
    Built with its pad facing the cavity, then spun to SPRING_SCREW_ANGLE."""
    r_wall = HUB_CAVITY_D / 2                       # 27 - cavity inner face
    r_pad  = r_wall - SCREW_BOSS_H                  # boss pad face (innermost)
    base   = SCREW_BOSS_SIDE + 2 * SCREW_BOSS_H     # 45 deg: base grows by the height each side
    boss = (cq.Workplane("YZ").workplane(offset=r_pad)
            .rect(SCREW_BOSS_SIDE, SCREW_BOSS_SIDE)
            .workplane(offset=SCREW_BOSS_H)
            .rect(base, base)
            .loft())
    # Extend the base 0.3 mm INTO the wall so the union merges volumetrically with
    # the curved hub rather than touching it on a tangent plane.
    boss = boss.union(cq.Workplane("YZ").workplane(offset=r_wall)
                      .rect(base, base).extrude(0.3))
    return (boss.translate((0, 0, SPRING_SCREW_Z))
            .rotate((0, 0, 0), (0, 0, 1), SPRING_SCREW_ANGLE))


def _spring_screw_bore():
    """Radial bore through the boss + wall: an insert pocket (M2_INSERT_PILOT_D
    dia x M2_INSERT_DEPTH) from the pad face, then an M2_SHAFT_CLR_D clearance
    hole the rest of the way out past the OD (so the screw shank can protrude)."""
    r_pad = HUB_CAVITY_D / 2 - SCREW_BOSS_H
    x_clr = r_pad + M2_INSERT_DEPTH
    clr_len = (HUB_OD / 2 + 5.0) - x_clr            # run out well past the OD
    pocket = cq.Solid.makeCylinder(M2_INSERT_PILOT_D / 2, M2_INSERT_DEPTH,
                                   cq.Vector(r_pad, 0, SPRING_SCREW_Z), cq.Vector(1, 0, 0))
    clr = cq.Solid.makeCylinder(M2_SHAFT_CLR_D / 2, clr_len,
                                cq.Vector(x_clr, 0, SPRING_SCREW_Z), cq.Vector(1, 0, 0))
    bore = pocket.fuse(clr).rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), SPRING_SCREW_ANGLE)
    return cq.Workplane(obj=bore)


SPRING_SCREW_ACCESS_D = 6.0   # driver-access hole Ø through the opposite hub wall


def _spring_screw_access_hole():
    """Ø SPRING_SCREW_ACCESS_D radial hole through the hub wall diametrally
    OPPOSITE the spring screw (same Z, SPRING_SCREW_ANGLE + 180°), coaxial
    with the screw, so a hex driver can pass through the spool wall and
    across the cavity to reach the screw head on the far side."""
    r0 = HUB_CAVITY_D / 2 - 1.0                     # start just inside the cavity wall
    length = (HUB_OD / 2 + 5.0) - r0                # run out well past the OD (and key ridges)
    hole = cq.Solid.makeCylinder(SPRING_SCREW_ACCESS_D / 2, length,
                                 cq.Vector(r0, 0, SPRING_SCREW_Z), cq.Vector(1, 0, 0))
    hole = hole.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1),
                       SPRING_SCREW_ANGLE + 180.0)
    return cq.Workplane(obj=hole)


main_body = _build_main_body()
