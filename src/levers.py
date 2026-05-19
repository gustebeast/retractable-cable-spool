"""Ratchet and brake levers + their pivot/pawl/pad helpers.

Module-load computes ``RATCHET_TOOTH_OFFSET_DEG`` from lever geometry. The
caller must invoke ``apply_to_main_body(main_body)`` to apply the lever-
dependent cuts (ratchet teeth, drum cable hole) — pure-function style.
"""

import math
import cadquery as cq

from .dimensions import (
    CAVITY_Z0,
    DRUM_ID, DRUM_OD,
    FLANGE_H, FLANGE_INNER_ID, FLANGE_OD, FLANGE_RIM_MID_R,
    HUB_CAVITY_D, HUB_OD,
    M2_SHAFT_CLR_D,
    RATCHET_DEPTH, RATCHET_TEETH,
    SPOKE_W, SPOOL_H, STRUCT_WALL,
)
from .helpers import cyl, cone_solid, heal, ratchet_cutter
from .housing import (
    BRAKE_BOSS_EXT_ALPHA_HI, BRAKE_BOSS_EXT_ALPHA_LO,
    BRAKE_INNER_TRAVEL_DEG, BRAKE_LEVER_BOSS_EXTENSION,
    BRAKE_LEVER_LEG_ALPHA_DEG, BRAKE_PIVOT_X,
    HOUSING_W,
    LEVER_PIVOT_BOSS_OD, LEVER_PIVOT_Z, LEVER_RIM_H,
    LEVER_STOP_PIN_HOLE_Y_FROM_GAP_END,
    RATCHET_BOSS_EXT_ALPHA_HI, RATCHET_BOSS_EXT_ALPHA_LO,
    RATCHET_LEVER_BOSS_EXTENSION, RATCHET_LEVER_LEG_ALPHA_DEG,
    RATCHET_OUTER_TRAVEL_DEG, RATCHET_PIVOT_X,
    SPINE_X_INNER, SPRING_BODY_LEN,
    STOP_HOUSING_PIN_ALPHA_BRAKE_DEG, STOP_HOUSING_PIN_ALPHA_RATCHET_DEG,
    STOP_LEVER_PIN_ALPHA_BRAKE_DEG, STOP_LEVER_PIN_ALPHA_RATCHET_DEG,
    STOP_PIN_D, STOP_PIN_H, STOP_PIN_R,
    STOP_REST_PIN_ALPHA_BRAKE_DEG, STOP_REST_PIN_ALPHA_RATCHET_DEG,
    pivot_boss_sector, spring_leg_hole_dir_alpha_deg,
    stop_pin_solid, stop_pin_hole,
)


# ────────────────────────────────────────────────────────────────────────────
# LEVERS — geometric layout for matched ROM
#
# Both arms are HORIZONTAL at rest (90° from pull direction). User pushes
# the bottom face of each lever toward the spool (in +Z direction, since
# the levers sit below the spool body), which lifts the handle end and
# actuates the contact on the other end.
#
# RATCHET (class-1): pivot at x=76, pawl catch edge at (x≈66.4, z=36.5),
#   handle at (x=115, z=50.5). Pivot BETWEEN pawl and handle, so they
#   move in opposite z directions — pulling the handle down lifts the
#   pawl up off the teeth. Horizontal arms:
#     pawl arm = 76 − 66.4 = 9.6 mm,  handle arm = 115 − 76 = 39 mm
#     throw ratio (handle/pawl) = 39/9.6 = 4.06
#
# BRAKE (class-2): pivot at x=55, pad midpoint at (x=74.47, z=38), handle
#   at (x=115, z=50.5). Pad and handle BOTH on +x of pivot, same motion —
#   pulling the handle down pushes the pad down onto the surface.
#   Horizontal arms:
#     pad arm = 74.47 − 55 = 19.47 mm,  handle arm = 115 − 55 = 60 mm
#     throw ratio (handle/pad) = 60/19.47 = 3.08
#
# Matched ROM (small-angle, horizontal-arm approximation):
#   Per mm of handle drop: pawl rises 9.6/39 = 0.246 mm,
#                          pad descends 19.47/60 = 0.3245 mm.
#   The ratchet pivot x=76 is tuned so P_b/P_r = 1.32 — matching the
#   2 mm:1.5 mm ratio of (pad rest lift : pawl tooth depth), which is
#   what makes transition simultaneous.
#   - At rest:    pawl seated in valley (z=36.5, engaged); pad lifted
#                 1.98 mm above surface (pad_rest_z = 39.98, disengaged).
#   - Transition: at handle drop ≈ 6.09 mm, pawl has risen 1.5 mm to reach
#                 tooth tip (z=38) AND pad has descended 1.98 mm to touch
#                 surface (z=38). Disconnect and start-of-brake coincide.
#   - Max pull:   handle drop LEVER_HANDLE_TRAVEL_MAX ≈ 8.12 mm → pawl
#                 0.5 mm above tip (headroom), pad 0.66 mm compressed
#                 into the rubber. A handle stop on the housing will
#                 enforce this position (next subtask).
#
# Note: contacts are drawn at their REST positions. Ratchet pawl sits in
# the tooth valley (rest = engaged for the ratchet). Brake pad sits
# PAD_REST_Z_BOT − SPOOL_H = 1.98 mm above the brake surface (rest =
# disengaged for the brake). At max pull, the brake arm rotates the pad
# down to (and slightly past) the surface; the pad tilts ~8° with the
# arm over that travel, which is within the rubber's compliance range.
# ────────────────────────────────────────────────────────────────────────────

# Kinematic constants for reference by subsequent subtasks (stop geometry,
# spring return). Small-angle horizontal-arm approximation.
PAWL_REST_Z_BOT        = SPOOL_H - RATCHET_DEPTH        # pawl at rest (engaged)
PAWL_HEADROOM          = 0.5                            # lift beyond tooth tip at max-pull
PAD_REST_LIFT          = 1.98                           # pad sits this far above the
                                                        # brake surface at rest (disengaged)
PAD_REST_Z_BOT         = SPOOL_H + PAD_REST_LIFT        # pad's bottom face at rest
LEVER_HANDLE_TRAVEL_MAX = 8.12                          # max handle z-travel from rest

# Thickness of the rubber pad adhered to the bottom of the printed brake
# pad block (McMaster 86495K36 Buna-N, 70A, 3 mm, cut to the pad
# footprint and bonded with your own adhesive). The printed pad stops
# BRAKE_RUBBER_T above PAD_REST_Z_BOT so that (printed material) +
# (rubber) = the full rest height — the geometric-layout math stays
# valid regardless of this value, so adjust freely if the actual
# measured thickness differs from spec.
BRAKE_RUBBER_T         = 3.3

LEVER_W              = 11.0    # grip width, perpendicular to pull (in y)
LEVER_T              =  2.0    # thickness in z (swing plane direction)
LEVER_PIVOT_HOLE_D   = M2_SHAFT_CLR_D    # M2 shaft clearance (shared)
LEVER_Z_CENTER       = LEVER_PIVOT_Z                 # 53
LEVER_Z_BOT          = LEVER_Z_CENTER - LEVER_T / 2  # 52 (lever body bottom)
LEVER_Z_TOP          = LEVER_Z_CENTER + LEVER_T / 2  # 54 (lever body top)

# Each lever sits on its side of the housing with a LEVER_RIM_H gap (= 3 mm)
# between the lever body's inner face and the housing column outer face.
# The lever's pivot boss extends across that gap to bridge to the housing,
# with a counterbore on the housing-facing end seating the spring coil.
# 11 mm of lever grip then extends outward (LEVER_W).
LEVER_INNER_Y       = HOUSING_W / 2 + LEVER_RIM_H   # 12

RATCHET_LEVER_Y0     = LEVER_INNER_Y                # 12 (+y side, inner)
RATCHET_LEVER_Y1     = RATCHET_LEVER_Y0 + LEVER_W    # 23
BRAKE_LEVER_Y1       = -LEVER_INNER_Y               # -12 (-y side, inner)
BRAKE_LEVER_Y0       = BRAKE_LEVER_Y1 - LEVER_W      # -23

# Lever x bounds — these define the extent of each lever's flat body.
#   RATCHET (class-1): body from pawl-side edge through pivot to handle.
#     Pawl-side edge derived from the pawl footprint's innermost corner
#     (where the +y-far corner of the pawl meets r=FLANGE_INNER_ID/2),
#     extended by RATCHET_LEVER_PAWL_OVERHANG to capture the pawl edge.
#   BRAKE (class-2): body from pivot to handle. The pad is within this
#     span, so the body alone encloses the pad region.
#   Both handles extend LEVER_GRIP_OVERHANG past the housing spine for
#     a consistent grip distance regardless of housing/spool diameter.
LEVER_GRIP_OVERHANG          = 25.0     # mm past spine for the user grip
RATCHET_LEVER_PAWL_OVERHANG  = 2.0      # mm of lever body past pawl inner corner
_RATCHET_PAWL_X_INNER        = math.sqrt(
    (FLANGE_INNER_ID / 2) ** 2 - RATCHET_LEVER_Y1 ** 2
)
RATCHET_LEVER_X_PAWL_SIDE = _RATCHET_PAWL_X_INNER - RATCHET_LEVER_PAWL_OVERHANG
RATCHET_LEVER_X_HANDLE    = SPINE_X_INNER + LEVER_GRIP_OVERHANG
BRAKE_LEVER_X_HANDLE      = SPINE_X_INNER + LEVER_GRIP_OVERHANG

def _lever_body(x0, x1, y0, y1):
    """Flat lever tongue: length x 11 mm (y) x 2 mm (z)."""
    return (
        cq.Workplane("XY").workplane(offset=LEVER_Z_BOT)
        .center((x0 + x1) / 2, (y0 + y1) / 2)
        .box(x1 - x0, y1 - y0, LEVER_T, centered=(True, True, False))
    )

def _lever_pivot_hole(pivot_x, y_start, y_end):
    """M2 clearance hole along the pivot axis (y), spanning [y_start, y_end]
    with a small overlap on each end for clean boolean cuts."""
    return (
        cq.Workplane("XY")
        .circle(LEVER_PIVOT_HOLE_D / 2)
        .extrude((y_end - y_start) + 1)
        .rotate((0, 0, 0), (1, 0, 0), -90)
        .translate((pivot_x, y_start - 0.5, LEVER_PIVOT_Z))
    )

# Pivot boss — cylindrical reinforcement coaxial with the pivot, axis along
# y. Replaces the lever's 2 mm body thickness in z at the pivot region with
# a 6 mm OD cylinder so the bolt clearance hole has real wall thickness in z.
# The boss also extends LEVER_RIM_H past the lever body's inner face to
# bridge the gap to the housing column outer face (replacing the old thin-
# walled rim).
#
# LEVER_PIVOT_BOSS_OD lives in housing.py now so the housing-side
# matching sector on the ratchet side can use the same OD.
# BRAKE_LEVER_BOSS_EXTENSION + RATCHET_LEVER_BOSS_EXTENSION live in
# housing.py — both sides split the gap-buffer with their housing-
# facing counterparts (a true housing boss on the ratchet, a slab on
# the brake rest pin on the brake).

# Boss-extension angular clearance + sector α bounds live in housing.py
# now (so the housing can build its own matching sector on the ratchet
# side without a circular import). LEVER_BOSS_EXT_PIN_CLR,
# RATCHET_BOSS_EXT_ALPHA_LO/HI, and BRAKE_BOSS_EXT_ALPHA_LO/HI are
# imported from there. The sector-cylinder helper is also there as
# pivot_boss_sector.

def _lever_pivot_boss(pivot_x, y0, y1, *, od=LEVER_PIVOT_BOSS_OD):
    """Cylindrical boss centered on the pivot, axis along +y, spanning [y0, y1].
    Used for the in-lever portion that thickens the lever plate at the
    pivot. The portion extending into the lever-housing gap is built
    separately by pivot_boss_sector (sectored to clear the stop pins).

    Pass od=RATCHET_PIVOT_BOSS_OD for the ratchet side (slimmer wall)."""
    return (
        cq.Workplane("XY")
        .circle(od / 2)
        .extrude(y1 - y0)
        .rotate((0, 0, 0), (1, 0, 0), -90)
        .translate((pivot_x, y0, LEVER_PIVOT_Z))
    )

# Ratchet pawl contact — same "project the ring onto the lever" approach
# as the brake pad: the pawl footprint is the intersection of the toothed
# ring (r = FLANGE_INNER_ID/2..DRUM_ID/2 - PAWL_BRAKE_GAP) with the
# lever's y-strip, so the +y and -y side walls are flush with the lever
# edges. The bottom face is then carved to match the tooth ring's own
# sawtooth top surface, so the pawl nests into every valley it covers
# (providing the catch face at each tooth drop) and skims every tip it
# covers.
PAWL_BRAKE_GAP = 1.5    # radial gap between pawl outer edge and brake
                        # surface inner edge. Prevents pawl from dragging
                        # on the brake ring if there's any side play in
                        # the pivot / lever bearing.

# ── Dynamic ratchet tooth phase ──────────────────────────────────────────────
# Center a tooth boundary (HIGH→LOW step) at the angular midpoint of the
# ratchet pawl's footprint. The footprint is the intersection of the
# pawl's y-strip (inner lever edge to inner edge + LEVER_W) with the
# toothed ring (r_in=FLANGE_INNER_ID/2, r_out=DRUM_ID/2 - PAWL_BRAKE_GAP).
# Its angular midpoint is at the y-midline of the lever, r-midline of
# the ring. Computing the offset this way means any future move of the
# ratchet lever in y auto-updates the tooth phase.
_y_pawl_mid = HOUSING_W / 2 + LEVER_RIM_H + LEVER_W / 2
_r_pawl_mid = (FLANGE_INNER_ID / 2 + (FLANGE_RIM_MID_R - PAWL_BRAKE_GAP)) / 2
_theta_pawl_mid_deg = math.degrees(
    math.asin(_y_pawl_mid / _r_pawl_mid)
)
_tooth_pitch_deg = 360.0 / RATCHET_TEETH
# Offset = the phase shift that places a tooth boundary (which, with
# offset=0, sits at i * pitch) right at _theta_pawl_mid_deg.
RATCHET_TOOTH_OFFSET_DEG = _theta_pawl_mid_deg % _tooth_pitch_deg

# Drum-wall cable transit — stadium-shaped slot through the drum wall
# at angle 180°, aligned alongside the +y face of the 180° spoke (the
# same spoke as the spool-disk transit hole). The cable descends along
# the spoke face after dropping through the spool disk; it exits the
# drum interior through this slot to wrap the drum OD. The spoke face
# at y=+SPOKE_W/2 sits on one of the stadium's flat edges, so the
# spoke serves as one wall of the cable channel through the drum.
# Long axis 12 mm runs along Z (axial, parallel to spoke); short axis
# 7 mm gives ~1 mm clearance around the 6 mm cable.
CABLE_HOLE_AZIMUTH_DEG = 180.0
# Top of hole sits STRUCT_WALL (1.7 mm) below the spool top — matches the
# top-rim-cap thickness so the rim above the hole has a clean STRUCT_WALL
# of support material, no more, no less.
CABLE_HOLE_Z_CENTER    =  SPOOL_H - STRUCT_WALL - 13.0 / 2
CABLE_HOLE_LONG_AXIS   =  13.0                       # along Z
CABLE_HOLE_SHORT_AXIS  =   7.0                       # along Y


def _build_cable_hole():
    """Stadium-shaped slot through the drum wall at CABLE_HOLE_AZIMUTH_DEG,
    aligned alongside the +y face of the spoke at that angle. Cable
    descends along the spoke face, exits through this slot to wrap the
    drum OD."""
    alpha = math.radians(CABLE_HOLE_AZIMUTH_DEG)
    cos_a = math.cos(alpha)
    # Start the slot well outside the drum OD and run it through past the
    # inner face, so the radial cut cleanly opens both faces.
    # Inner reach extended by FLANGE_H so the slot cuts cleanly through
    # the pancake flange's inward-overhanging rim material that sits
    # above the drum at the top of the slot.
    r_start = DRUM_ID / 2 - 2.0 - FLANGE_H
    r_end   = DRUM_OD / 2 + 2.0
    # At angle 180° the bore axis is -x; the workplane sits at the OUTER
    # face of the bore and extrudes along its +x normal toward the drum.
    return (
        cq.Workplane("YZ")
        .workplane(offset=cos_a * r_end)
        .center(SPOKE_W / 2 + CABLE_HOLE_SHORT_AXIS / 2,
                CABLE_HOLE_Z_CENTER)
        .slot2D(CABLE_HOLE_LONG_AXIS, CABLE_HOLE_SHORT_AXIS, angle=90)
        .extrude(r_end - r_start)
    )


# Cable ENTRY hole — stadium slot THROUGH the α=180° spoke, axis along
# Y. At low Z near the hub OD; cable threads from the spool cavity into
# the +Y chamber here. Long axis along Z, short axis along X.
ENTRY_HOLE_Z_BOTTOM   = 2.0
ENTRY_HOLE_LONG_AXIS  = 13.0
ENTRY_HOLE_SHORT_AXIS = CABLE_HOLE_SHORT_AXIS     # 7 mm


def _build_entry_hole():
    """Bottom entry slot through the α=180° spoke at low Z near the hub."""
    x_center = -(HUB_OD / 2 + ENTRY_HOLE_SHORT_AXIS / 2)
    z_center = ENTRY_HOLE_Z_BOTTOM + ENTRY_HOLE_LONG_AXIS / 2
    y_extent = SPOKE_W + 2.0
    return (
        cq.Workplane("XZ")
        .workplane(offset=-y_extent / 2)
        .center(x_center, z_center)
        .slot2D(ENTRY_HOLE_LONG_AXIS, ENTRY_HOLE_SHORT_AXIS, angle=90)
        .extrude(y_extent)
    )


# Top merged stadium — combines the former top-spoke pass-through and
# the drum-wall cable hole into a single 3D cut. The slot's long axis
# lies in the XY plane, angled TOP_MERGED_ANGLE_FROM_WALL_DEG away from
# the spool's outer-wall tangent (toward the spoke's radial direction)
# at α=180°. A small angle keeps the cable's path nearly parallel to
# the outer wall — a gentler transition onto the helix groove than a
# steep (more radial) exit. Slot extrudes straight up in Z across the
# same Z range the drum-wall hole used to cover; its length is solved
# so the outer cap apex clears the helix groove where the cable exits.
#
# Inner cap apex x is at the old drum-wall hole's inner end (-64). Its
# y is SOLVED (see _top_merged_inner_apex_y) so the slot's far edge,
# where it crosses the spoke's +Y face, lands flush on the drum's inner
# cavity wall — no leftover lip of spoke material between the spoke hole
# and the drum wall.
TOP_MERGED_ANGLE_FROM_WALL_DEG = 15.0
TOP_MERGED_INNER_APEX_X        = -(DRUM_ID / 2 - 9.0)   # -64


def _top_merged_inner_apex_y():
    """Inner cap apex y, solved so the slot's far edge at the spoke's
    +Y face just reaches r=DRUM_ID/2 (the drum's inner cavity wall)."""
    th       = math.radians(TOP_MERGED_ANGLE_FROM_WALL_DEG)
    half_w   = CABLE_HOLE_SHORT_AXIS / 2
    face_y   = SPOKE_W / 2
    x_target = -math.sqrt((DRUM_ID / 2) ** 2 - face_y ** 2)
    return ((x_target - TOP_MERGED_INNER_APEX_X + half_w * math.cos(th)) / math.tan(th)
            + face_y + half_w * math.sin(th))


def _build_top_merged_slot():
    """Merged top-spoke + drum-wall stadium slot, angled in the XY plane."""
    angle_rad = math.radians(TOP_MERGED_ANGLE_FROM_WALL_DEG)
    # Direction: mostly +Y (outer-wall tangent), with a -X (radial out)
    # component sized by the angle-from-wall. Sends the cable CW around
    # the spool onto the helix.
    dir_x = -math.sin(angle_rad)
    dir_y = +math.cos(angle_rad)
    ix = TOP_MERGED_INNER_APEX_X
    iy = _top_merged_inner_apex_y()
    # Solve for L so the outer cap apex sits well past the helix groove's
    # outermost cut extent (DRUM_OD/2 + helix-groove radius), with margin,
    # so the slot fully clears the groove where the cable exits:
    #   (ix + L·dir_x)² + (iy + L·dir_y)² = target_r²
    target_r = DRUM_OD / 2 + 6.0
    b = 2 * (ix * dir_x + iy * dir_y)
    c = ix * ix + iy * iy - target_r * target_r
    L = (-b + math.sqrt(b * b - 4 * c)) / 2
    ox = ix + L * dir_x
    oy = iy + L * dir_y
    cx = (ix + ox) / 2
    cy = (iy + oy) / 2
    angle_deg = math.degrees(math.atan2(dir_y, dir_x))
    z_lo = CABLE_HOLE_Z_CENTER - CABLE_HOLE_LONG_AXIS / 2
    z_hi = CABLE_HOLE_Z_CENTER + CABLE_HOLE_LONG_AXIS / 2
    return (
        cq.Workplane("XY").workplane(offset=z_lo)
        .center(cx, cy)
        .slot2D(L, CABLE_HOLE_SHORT_AXIS, angle=angle_deg)
        .extrude(z_hi - z_lo)
    )


def apply_to_main_body(main_body: cq.Workplane) -> cq.Workplane:
    """Apply the lever-dependent cuts to the main spool body: ratchet
    teeth (phase aligned with the ratchet pawl footprint), the drum-
    wall cable-transit slot, and the cable-entry slot through the
    hub wall. Returns the modified body."""
    main_body = main_body.cut(
        ratchet_cutter(RATCHET_TEETH,
                       r_in=FLANGE_INNER_ID / 2,
                       r_out=FLANGE_RIM_MID_R,
                       z_top=RATCHET_DEPTH,
                       depth=RATCHET_DEPTH,
                       theta_offset_deg=RATCHET_TOOTH_OFFSET_DEG)
    )
    # Heal between the ratchet cut and the transit-slot cut — the slot
    # passes through the spool's groove-relieved drum wall, whose geometry
    # is valid but fragile, and the cut otherwise silently no-ops.
    main_body = heal(main_body)
    main_body = main_body.cut(_build_top_merged_slot())
    main_body = main_body.cut(_build_entry_hole())
    return main_body

def _ratchet_pawl_contact(y_near, y_far):
    """Pawl built as (lever-aligned annular-strip prism extending from
    valley floor up to the lever body bottom) MINUS (tooth material in
    that region). Result: sawtooth underside that fits the ring exactly,
    with vertical side walls flush with the lever y-edges."""
    import math
    r_in  = FLANGE_INNER_ID / 2
    r_out = FLANGE_RIM_MID_R - PAWL_BRAKE_GAP
    z_valley = RATCHET_DEPTH
    z_tip    = 0
    # Footprint corners — intersections of y = const with the two ring radii
    x_in_near  = math.sqrt(r_in**2  - y_near**2)
    x_out_near = math.sqrt(r_out**2 - y_near**2)
    x_in_far   = math.sqrt(r_in**2  - y_far**2)
    x_out_far  = math.sqrt(r_out**2 - y_far**2)
    y_mid = (y_near + y_far) / 2
    x_in_mid  = math.sqrt(r_in**2  - y_mid**2)
    x_out_mid = math.sqrt(r_out**2 - y_mid**2)

    prism = (
        cq.Workplane("XY").workplane(offset=z_valley)
        .moveTo(x_in_far, y_far)
        .lineTo(x_out_far, y_far)
        .threePointArc((x_out_mid, y_mid), (x_out_near, y_near))
        .lineTo(x_in_near, y_near)
        .threePointArc((x_in_mid, y_mid), (x_in_far, y_far))
        .close()
        .extrude(LEVER_Z_TOP - z_valley)
    )
    # Tooth material = full annulus minus the ratchet cutter (the air
    # above each ramp). Spans z=z_tip..z_valley.
    tooth_material = (
        cyl(2 * r_out, z_valley - z_tip, z=z_tip)
        .cut(cyl(2 * r_in, z_valley - z_tip, z=z_tip))
        .cut(ratchet_cutter(RATCHET_TEETH,
                            r_in=r_in, r_out=r_out,
                            z_top=z_valley, depth=RATCHET_DEPTH,
                            theta_offset_deg=RATCHET_TOOTH_OFFSET_DEG))
    )
    return prism.cut(tooth_material)

def _build_ratchet_lever():
    return (
        _lever_body(RATCHET_LEVER_X_PAWL_SIDE, RATCHET_LEVER_X_HANDLE,
                    RATCHET_LEVER_Y0, RATCHET_LEVER_Y1)
        .union(_ratchet_pawl_contact(RATCHET_LEVER_Y0, RATCHET_LEVER_Y1))
        # Pivot boss extension into the gap — FULL DISC around the pivot.
        # Sector trim no longer needed: with STOP_PIN_R=6 the pin inner
        # edges sit at r=4 while the boss outer is at r=3, giving 1 mm
        # clearance to every stop pin at every rotation. Unioned BEFORE
        # the cylinder boss because OCCT's boolean produces an invalid
        # face if the cylinder boss is unioned in first (only on the +X-
        # side ratchet — the -X-side brake is fine in either order).
        .union(pivot_boss_sector(RATCHET_PIVOT_X,
                                  RATCHET_LEVER_Y0 - RATCHET_LEVER_BOSS_EXTENSION,
                                  RATCHET_LEVER_Y0 + 0.5,
                                  0.0, 360.0))
        # Pivot boss inside the lever body.
        .union(_lever_pivot_boss(RATCHET_PIVOT_X,
                                 RATCHET_LEVER_Y0,
                                 RATCHET_LEVER_Y1))
        # Lever stop pin + teardrop spring-leg through-hole.
        .union(stop_pin_solid(RATCHET_PIVOT_X, STOP_LEVER_PIN_ALPHA_RATCHET_DEG,
                               LEVER_INNER_Y - STOP_PIN_H,
                               RATCHET_LEVER_Y1))
        .cut(stop_pin_hole(RATCHET_PIVOT_X, STOP_LEVER_PIN_ALPHA_RATCHET_DEG,
                            hole_y=+((LEVER_INNER_Y - STOP_PIN_H)
                                     + LEVER_STOP_PIN_HOLE_Y_FROM_GAP_END),
                            teardrop=False,
                            hole_dir_alpha_deg=spring_leg_hole_dir_alpha_deg(
                                RATCHET_LEVER_LEG_ALPHA_DEG)))
        .cut(_lever_pivot_hole(RATCHET_PIVOT_X,
                               RATCHET_LEVER_Y0 - RATCHET_LEVER_BOSS_EXTENSION,
                               RATCHET_LEVER_Y1))
    )

# Brake pad contact — the portion of the brake annulus (r = 73..80) that
# lies directly under the lever's y-footprint. Think of it as the lever's
# projection dropped onto the brake ring:
#   - y sides are straight chords at the lever edges (y=-12 and y=-23).
#     The y=-12 edge is 1 mm from the housing face at y=-11 (set by the
#     lever rim gap), which is the clearance the user asked for.
#   - r sides are true arcs at the inner (r=73) and outer (r=80) ring radii,
#     so the pad exactly fills the lever-aligned strip of the brake surface.
#   - Bottom face sits at z = SPOOL_H (flat on the brake ring).
#   - Top face at LEVER_Z_BOT (arm connection in the next subtask).

def _brake_pad_contact(y_near, y_far):
    """Pad carved from the brake annulus by the strip y ∈ [y_far, y_near]
    in the +x half-plane. Straight edges at constant y (parallel to the
    housing); arc edges at the inner/outer ring radii.

    Pre-angled for the brake's engagement rotation: the pad is built at
    its ENGAGED (design-intent) position — rubber sitting flat on the
    ring, printed bottom at SPOOL_H + BRAKE_RUBBER_T — then rotated by
    -BRAKE_INNER_TRAVEL_DEG around the brake pivot's Y axis to get the
    REST (printed) pose. That pre-tilt cancels the engagement rotation:
    when the user activates the brake, the lever rotates +12° and the
    pad ends up flat on the ring AT THE DESIGN POSITION, instead of
    tilted away with the +x edge digging in.

    The pad solid extends from the engaged bottom up well past
    LEVER_Z_BOT so the rotated solid still overlaps the lever body for a
    clean boolean union (the rotated top face is no longer at
    LEVER_Z_BOT — it's tilted)."""
    import math
    r_in  = FLANGE_RIM_MID_R
    r_out = FLANGE_OD / 2
    # 4 corners — intersections of y = const with each ring
    x_in_near  = math.sqrt(r_in**2  - y_near**2)
    x_out_near = math.sqrt(r_out**2 - y_near**2)
    x_in_far   = math.sqrt(r_in**2  - y_far**2)
    x_out_far  = math.sqrt(r_out**2 - y_far**2)
    # Arc midpoints at the y halfway between the two chords
    y_mid = (y_near + y_far) / 2
    x_in_mid  = math.sqrt(r_in**2  - y_mid**2)
    x_out_mid = math.sqrt(r_out**2 - y_mid**2)
    # ENGAGED-pose pad: built at the pose where the brake is fully engaged
    # — class-2 lever rises to engage, rubber's TOP touches the lever-side
    # flange BOTTOM (brake track at z=0). Then rotated +BRAKE_INNER_TRAVEL_DEG
    # around the brake pivot to get the REST pose.
    #
    # Tuned so that at REST (post-rotation), the pad's CORNER closest to
    # the track has rubber-top-to-track clearance = PAD_REST_LIFT — i.e.,
    # the brake just-touches at the same lever throw the ratchet just-
    # clears (matched ROM). The +12° rotation tilts the inboard / -X end
    # of the pad UP, so the spool-facing peak is at the pad's inner arc
    # at the y with largest magnitude. That x value is the relevant
    # x_rel for the matched-ROM constraint.
    BRAKE_TRACK_Z = 0.0
    _pad_inner_rest_z = BRAKE_TRACK_Z - PAD_REST_LIFT - BRAKE_RUBBER_T
    _y_max_abs        = max(abs(y_near), abs(y_far))
    _x_rel_corner     = math.sqrt(r_in ** 2 - _y_max_abs ** 2) - BRAKE_PIVOT_X
    _theta            = math.radians(BRAKE_INNER_TRAVEL_DEG)
    _z_rel = (_pad_inner_rest_z - LEVER_PIVOT_Z + _x_rel_corner * math.sin(_theta)) / math.cos(_theta)
    pad_bot_engaged_z = LEVER_PIVOT_Z + _z_rel
    # Extend the top past LEVER_Z_TOP so the rotated solid still
    # interpenetrates the lever body (its inner face is at LEVER_Z_TOP and
    # the rotated pad top is tilted; we need the union to absorb the
    # excess cleanly).
    pad_top_engaged_z = LEVER_Z_TOP - 5.0
    pad_solid = (
        cq.Workplane("XY").workplane(offset=pad_bot_engaged_z)
        .moveTo(x_in_far, y_far)
        .lineTo(x_out_far, y_far)
        .threePointArc((x_out_mid, y_mid), (x_out_near, y_near))
        .lineTo(x_in_near, y_near)
        .threePointArc((x_in_mid, y_mid), (x_in_far, y_far))
        .close()
        .extrude(pad_top_engaged_z - pad_bot_engaged_z)
    )
    # Rotate -BRAKE_INNER_TRAVEL_DEG around the brake pivot's Y axis to
    # get the REST pose (the print pose). Engagement applies the
    # opposite rotation, returning the pad to the engaged design pose.
    pad_solid = pad_solid.rotate(
        (BRAKE_PIVOT_X, 0, LEVER_PIVOT_Z),
        (BRAKE_PIVOT_X, 1, LEVER_PIVOT_Z),
        BRAKE_INNER_TRAVEL_DEG,
    )
    # The rotation tilts the pad's top face (originally at LEVER_Z_BOT +
    # 5 mm so the boolean union with the lever body would still
    # interpenetrate) up significantly on the -x side — wasting a lot of
    # printed material above where the lever body actually sits. Clip
    # everything above LEVER_Z_BOT: the lever body fills above that
    # plane, so the pad only needs to occupy the gap between lever and
    # spool.
    clip_top = (
        cq.Workplane("XY")
        .workplane(offset=LEVER_Z_TOP)
        .center(0, 0)
        .rect(400, 400)   # cover the full pad XY footprint
        .extrude(-50)
    )
    return pad_solid.cut(clip_top)

def _build_brake_lever():
    return (
        _lever_body(BRAKE_PIVOT_X, BRAKE_LEVER_X_HANDLE,
                    BRAKE_LEVER_Y0, BRAKE_LEVER_Y1)
        .union(_brake_pad_contact(BRAKE_LEVER_Y1, BRAKE_LEVER_Y0))
        # Pivot boss + FULL-DISC extension — see ratchet for rationale.
        .union(_lever_pivot_boss(BRAKE_PIVOT_X,
                                 BRAKE_LEVER_Y0,
                                 BRAKE_LEVER_Y1))
        .union(pivot_boss_sector(BRAKE_PIVOT_X,
                                  BRAKE_LEVER_Y1 - 0.5,    # 0.01 mm overlap
                                  BRAKE_LEVER_Y1 + BRAKE_LEVER_BOSS_EXTENSION,
                                  0.0, 360.0))
        # Lever stop pin + teardrop spring-leg through-hole.
        .union(stop_pin_solid(BRAKE_PIVOT_X, STOP_LEVER_PIN_ALPHA_BRAKE_DEG,
                               BRAKE_LEVER_Y0,
                               -LEVER_INNER_Y + STOP_PIN_H))
        .cut(stop_pin_hole(BRAKE_PIVOT_X, STOP_LEVER_PIN_ALPHA_BRAKE_DEG,
                            hole_y=-((LEVER_INNER_Y - STOP_PIN_H)
                                     + LEVER_STOP_PIN_HOLE_Y_FROM_GAP_END),
                            teardrop=False,
                            hole_dir_alpha_deg=spring_leg_hole_dir_alpha_deg(
                                BRAKE_LEVER_LEG_ALPHA_DEG)))
        .cut(_lever_pivot_hole(BRAKE_PIVOT_X,
                               BRAKE_LEVER_Y0,
                               BRAKE_LEVER_Y1 + BRAKE_LEVER_BOSS_EXTENSION))
    )


ratchet_lever = _build_ratchet_lever()
brake_lever   = _build_brake_lever()
