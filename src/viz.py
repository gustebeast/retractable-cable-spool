"""Assembly-only visualization parts — dummy 608 bearings + dummy torsion springs.

Not printed; included in the assembly STEP only for visual sanity-checking.
"""

import math
import cadquery as cq

from .dimensions import (
    AXLE_D,
    BEARING_LIP_H, BEARING_OD, BEARING_W,
    SPOOL_H,
)
from .helpers import cyl
from .housing import (
    BACK_AXLE_BEARING_Z0, BACK_AXLE_X, BACK_AXLE_Y,
    BRAKE_HOUSING_BOSS_EXTENSION,
    BRAKE_HOUSING_LEG_ALPHA_DEG, BRAKE_LEVER_LEG_ALPHA_DEG,
    BRAKE_PIN_HOLE_BASE_CLR, BRAKE_PIVOT_X, HOUSING_W, LEVER_PIVOT_Z, LEVER_RIM_H, LEVER_STOP_PIN_HOLE_Y_FROM_GAP_END,
    RATCHET_HOUSING_BOSS_EXTENSION,
    RATCHET_HOUSING_LEG_ALPHA_DEG, RATCHET_LEVER_LEG_ALPHA_DEG,
    RATCHET_PIVOT_X,
    SPRING_BODY_LEN, SPRING_COIL_MAJOR_R, SPRING_LEG_PIN_OFFSET_DEG, SPRING_WIRE_R,
    STOP_HOUSING_PIN_ALPHA_BRAKE_DEG, STOP_HOUSING_PIN_ALPHA_RATCHET_DEG,
    STOP_LEVER_PIN_ALPHA_BRAKE_DEG, STOP_LEVER_PIN_ALPHA_RATCHET_DEG,
    STOP_PIN_D, STOP_PIN_H, STOP_PIN_HOLE_D, STOP_PIN_HOLE_OFFSET, STOP_PIN_R,
)
from .levers import LEVER_INNER_Y
from .spool import pancake_bearing_z0


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║ ASSEMBLY-ONLY VISUALIZATION PARTS                                       ║
# ║ The parts below are NOT printed — they exist only so the assembly.step  ║
# ║ shows the purchased components (608 bearings, torsion springs) in their ║
# ║ assembled positions for visual sanity-checking and clearance review.    ║
# ╚══════════════════════════════════════════════════════════════════════════╝

# ────────────────────────────────────────────────────────────────────────────
# TORSION SPRING (dummy / visualization only — not printed)
#
# Modeled as a SINGLE solid: a helix coil swept along a parametric curve,
# unioned with two bent legs (axial then radial segment each). Single
# solid means Onshape (and other STEP viewers) can hide/show the entire
# spring as one part.
#
# Geometry:
#   - Coil: helix between (α_housing, y_coil_a) and (α_lever, y_coil_b),
#     with SPRING_VIZ_TURNS full revolutions plus the (α_lever − α_housing)
#     angular offset. Coil mean radius SPRING_COIL_MAJOR_R, wire radius
#     SPRING_WIRE_R.
#   - Each leg: starts at the coil's axial end (one of y_coil_a, y_coil_b)
#     at the angular position where the helix terminates. Bends axially
#     by Δy to reach the corresponding stop pin's through-hole Y, then
#     extends radially outward by SPRING_LEG_LEN to pass through the pin.
#
# Per the housing's α convention (z = pivot_z − R sin α), the radial
# direction outward from the coil axis at angle α is (cos α, 0, −sin α).
# The hole through each housing stop pin (stop_pin_hole) uses the same
# vector, so the spring leg lines up with the hole.
# ────────────────────────────────────────────────────────────────────────────

# SPRING_BODY_LEN lives in housing.py (shared with the lever's
# pivot-boss-extension sizing).
# Coil viz approximation: instead of stacked tori (which fuse into a
# fragile Compound that strict STEP validators reject), the coil is
# rendered as a single annular cylinder — outer R = mean + wire_R,
# inner R = mean − wire_R, axial extent = SPRING_BODY_LEN. Loses the
# helix detail but produces a clean, simply-connected solid that fuses
# robustly with the legs.
#
# SPRING_COIL_MAJOR_R, SPRING_LEG_PIN_OFFSET_DEG, and the per-pin
# SPRING leg α constants live in housing.py so the housing/lever
# stop-pin through-holes can use them to align with the leg direction.

# Leg length: long enough for the tangent leg to reach the pin's center
# (= sqrt(R_pin² − R_coil²) ≈ 4.09 mm) and extend the pin's radius past
# (= STOP_PIN_D/2 = 2 mm), plus 1 mm visible overshoot.
SPRING_LEG_LEN      = (math.sqrt(STOP_PIN_R ** 2 - SPRING_COIL_MAJOR_R ** 2)
                       + STOP_PIN_D / 2 + 1.0)

# The two springs in the prototype are DIFFERENT TYPES with different
# natural rest angles:
#   - Brake spring (~4.25 turns): "90° type" — natural rest 90° apart;
#     installed legs ~65° apart on the coil. Visually a narrow "V" —
#     legs leave the coil from neighbouring α and their tangent
#     directions are ~65° apart.
#   - Ratchet spring (~4.75 turns or LH 4.25): "270° type" — natural
#     rest 270° apart; installed legs ~195° apart on the coil. Visually
#     the legs are diametrically opposite tangents — they exit the coil
#     on OPPOSITE sides and their tangent lines are nearly collinear, so
#     they read as one straight line passing through the spring (the
#     "parallel-legs" appearance the user described).
#
# These leg α-offsets aren't free parameters — they're determined by
# the housing/lever pin α positions and the tangent-line constraint
# (each leg's tangent must pass through its pin). We pick which side of
# each pin the tangent comes off (±SPRING_LEG_PIN_OFFSET_DEG) to land
# on the desired offset family.

def _torsion_spring(pivot_x, y_coil_a, y_coil_b, *,
                    alpha_housing_leg_deg, alpha_lever_leg_deg,
                    handedness="RH",
                    housing_leg_sign=+1, lever_leg_sign=+1):
    """Torsion spring viz: stacked-ring coil + two TANGENTIAL legs.

    Each leg exits its coil end at the wire centerline (at the specified
    α on that end's ring) and extends in the helix's tangent direction —
    parallel to the wire's path at the exit, NOT radial. The housing-end
    leg extends in the helix's −tangent direction (the wire came from
    there before entering the coil); the lever-end leg extends in the
    +tangent direction (the wire continues there after exiting the coil).

    For a 4.25-turn helix, the natural angular offset between the two
    coil ends is 4.25·360° mod 360° = 90°, so alpha_lever_leg_deg should
    differ from alpha_housing_leg_deg by ±90° (sign determined by
    handedness and the direction of travel from y_coil_a to y_coil_b).

    Tangent direction at angle α (housing α convention `z = pivot_z −
    R sin α`): the helix has `(R cos α, y, −R sin α)` so its t-tangent
    is `(−R sin α, dy/dt, −R cos α)`. The XZ part is `(−sin α, −cos α)`,
    multiplied by ±1 for handedness and ±1 for which end of the wire
    we're at.
    """
    pivot_z = LEVER_PIVOT_Z
    parts = []

    # ── Coil: single annular cylinder ──────────────────────────────────
    # Axis along Y, centered between y_coil_a and y_coil_b. The wall is
    # SLIGHTLY THICKER than the wire radius so a tangent-leg cylinder
    # (radius SPRING_WIRE_R) sits FULLY INSIDE the annulus's radial
    # extent at the junction — strictly transverse boolean intersection,
    # no tangent surfaces (which choke strict STEP validators).
    COIL_HALF_WIDTH = SPRING_WIRE_R + 0.15
    coil_span = abs(y_coil_b - y_coil_a)
    y_min = min(y_coil_a, y_coil_b)
    coil_outer = cq.Solid.makeCylinder(
        SPRING_COIL_MAJOR_R + COIL_HALF_WIDTH, coil_span,
        pnt=cq.Vector(pivot_x, y_min, pivot_z),
        dir=cq.Vector(0, 1, 0),
    )
    coil_inner = cq.Solid.makeCylinder(
        SPRING_COIL_MAJOR_R - COIL_HALF_WIDTH, coil_span + 0.4,
        pnt=cq.Vector(pivot_x, y_min - 0.2, pivot_z),
        dir=cq.Vector(0, 1, 0),
    )
    parts.append(coil_outer.cut(coil_inner))

    # End-ring Y values for leg attachment (legs come off the axial
    # extremes of the coil).
    sign = 1 if y_coil_b > y_coil_a else -1
    ring_ys = [y_coil_a + sign * SPRING_WIRE_R,
               y_coil_b - sign * SPRING_WIRE_R]

    # ── Two tangential legs ────────────────────────────────────────────
    # Each leg comes off its coil end in the tangent direction at its α.
    # Per-leg sign (+1 = +tangent, −1 = -tangent) lets us model the two
    # standard torsion-spring families: the 90°-natural type has the two
    # legs leaving the coil in OPPOSITE tangent senses (one +, one −);
    # the 270°-natural type has them leaving in the SAME tangent sense.
    # A small sphere at each junction breaks the leg/annulus tangency
    # so the boolean fuse exports as a strict-validator-friendly Solid.
    h_sign = +1 if handedness == "RH" else -1
    OVERLAP = 0.1
    for ring_y, alpha_deg, leg_sign in (
        (ring_ys[0],  alpha_housing_leg_deg, housing_leg_sign),
        (ring_ys[-1], alpha_lever_leg_deg,   lever_leg_sign),
    ):
        a = math.radians(alpha_deg)
        cos_a = math.cos(a)
        sin_a = math.sin(a)
        x_a = pivot_x + SPRING_COIL_MAJOR_R * cos_a
        z_a = pivot_z - SPRING_COIL_MAJOR_R * sin_a
        # Sphere at the coil exit point — sits half inside the coil
        # annulus body, half along the leg's axis. Ensures every face in
        # the resulting fuse meets transversely, never tangentially.
        parts.append(cq.Solid.makeSphere(
            SPRING_WIRE_R + 0.1,
            pnt=cq.Vector(x_a, ring_y, z_a),
        ))
        # Tangent direction (XZ-only); handedness and per-leg sign flip.
        tan_x = -sin_a * h_sign * leg_sign
        tan_z = -cos_a * h_sign * leg_sign
        start = cq.Vector(
            x_a - OVERLAP * tan_x,
            ring_y,
            z_a - OVERLAP * tan_z,
        )
        parts.append(cq.Solid.makeCylinder(
            SPRING_WIRE_R, SPRING_LEG_LEN + OVERLAP,
            pnt=start,
            dir=cq.Vector(tan_x, 0, tan_z),
        ))

    # Fuse iteratively.
    fused = parts[0]
    for p in parts[1:]:
        fused = fused.fuse(p)
    return cq.Workplane("XY").add(fused)


# ── Spring placement ─────────────────────────────────────────────────────
# Each spring's coil sits in the lever-housing gap. Its housing-side
# axial end is flush with the housing's column outer face (y=±HOUSING_W/2);
# the lever-side end is SPRING_BODY_LEN further outboard.
#
# Leg α placement: each leg's α-on-coil is the corresponding pin's α
# ± SPRING_LEG_PIN_OFFSET_DEG (=65.38°) so the leg's tangent line
# passes through the pin's center. Which sign we pick selects which
# spring TYPE the viz represents (90° vs 270° natural rest).
#
# Handedness: ratchet RH and brake LH so the tangent leg directions
# come out mirror-symmetric across the housing's centerline.

# Each pin admits TWO valid leg-attachment α on the coil — the points
# where a tangent line to the coil passes through the pin center
# (α_pin ± SPRING_LEG_PIN_OFFSET_DEG, diametrically opposite sides of
# the coil with opposite tangent directions). The per-spring side pick
# (housing.py: {RATCHET,BRAKE}_{HOUSING,LEVER}_LEG_SIDE) chooses which.
# That choice both shifts the attachment α AND flips the leg's exit
# direction, which is what visually distinguishes the spring "type":
#
#   Ratchet ("270" type, near-collinear-line look): sides (h, l) =
#     (−1, +1). Leg attachments nearly diametrically opposite on the
#     coil; tangent directions nearly antiparallel — legs read as one
#     straight line through the spring.
#
#   Brake (mirror of the ratchet): sides (h, l) = (−1, +1). Same
#     topology as the ratchet, mirrored across the brake's local Y
#     axis (= leg-tangent signs flipped, see below).

# Per-leg tangent-direction sign. The "standard" 90-degree-type
# convention is housing leg = +tangent, lever leg = −tangent (the
# wire's natural exit sense at each axial end of an RH helix wound
# from −y to +y). The RATCHET coil runs that way (y_coil_a < y_coil_b
# on the +y side of the housing), so it uses the standard signs.
#
# The BRAKE coil is built on the −y side and runs the OTHER way
# (y_coil_a > y_coil_b), so the helix's natural exit direction at
# each axial end is inverted relative to the ratchet — both leg signs
# flip. This is the equivalent of flipping the spring about its
# Y-axis and then its X-axis, but built that way from the start
# rather than post-hoc rotated, so we don't reintroduce the Z-flip
# regression we hit before.
_RATCHET_HOUSING_LEG_SIGN = +1
_RATCHET_LEVER_LEG_SIGN   = -1
# Brake's mirror-along-Y: side picks flipped, AND leg-tangent signs
# flipped, so each leg still extends from its coil exit TOWARD its
# pin (rather than away from it). Net result: brake's leg topology
# is the mirror image of the ratchet's, exactly.
_BRAKE_HOUSING_LEG_SIGN   = +1
_BRAKE_LEVER_LEG_SIGN     = -1

ratchet_spring = _torsion_spring(
    RATCHET_PIVOT_X,
    # Ratchet spring is offset by the housing-side boss-extension stub
    # so it sits at the gap midpoint (1 mm boss on each side of the
    # 2.5 mm spring within the 4.5 mm gap).
    y_coil_a=+HOUSING_W / 2 + RATCHET_HOUSING_BOSS_EXTENSION,
    y_coil_b=+HOUSING_W / 2 + RATCHET_HOUSING_BOSS_EXTENSION + SPRING_BODY_LEN,
    alpha_housing_leg_deg=RATCHET_HOUSING_LEG_ALPHA_DEG,
    alpha_lever_leg_deg=RATCHET_LEVER_LEG_ALPHA_DEG,
    handedness="RH",
    housing_leg_sign=_RATCHET_HOUSING_LEG_SIGN,
    lever_leg_sign=_RATCHET_LEVER_LEG_SIGN,
)
brake_spring = _torsion_spring(
    BRAKE_PIVOT_X,
    # Brake spring is offset by the housing-side slab (printed onto
    # the brake rest pin) so it sits at the gap midpoint, mirroring
    # the ratchet's split.
    y_coil_a=-HOUSING_W / 2 - BRAKE_HOUSING_BOSS_EXTENSION,
    y_coil_b=-HOUSING_W / 2 - BRAKE_HOUSING_BOSS_EXTENSION - SPRING_BODY_LEN,
    alpha_housing_leg_deg=BRAKE_HOUSING_LEG_ALPHA_DEG,
    alpha_lever_leg_deg=BRAKE_LEVER_LEG_ALPHA_DEG,
    handedness="RH",
    housing_leg_sign=_BRAKE_HOUSING_LEG_SIGN,
    lever_leg_sign=_BRAKE_LEVER_LEG_SIGN,
)

# ── Dummy 608 bearings (visualization-only) ───────────────────────────
# Solid annulus, OD=BEARING_OD, ID=AXLE_D, height=BEARING_W. Bearings
# are purchased parts (not printed), so this is just for assembly preview.
def _bearing_dummy(z_base):
    return (
        cyl(BEARING_OD, BEARING_W, z=z_base)
        .cut(cyl(AXLE_D, BEARING_W, z=z_base))
    )

# Bottom bearing: in bearing_cap_bottom's pocket, sits on the lip.
bearing_bottom = _bearing_dummy(BEARING_LIP_H)
# Top bearing: in bearing_cap_top's pocket. Bottom face at z=pancake_bearing_z0
# (= cap interior face); top face wedges into the cone funnel where it has
# narrowed to Ø22.
bearing_top    = _bearing_dummy(pancake_bearing_z0)
# Back-axle bearing: in the pancake-plate -X extension's pocket, holding
# the cable-retention guide axle.
back_axle_bearing = _bearing_dummy(BACK_AXLE_BEARING_Z0).translate((BACK_AXLE_X, BACK_AXLE_Y, 0))


# ── Brake pad rubber (dummy / visualization only) ─────────────────────
# 3.3 mm-thick rubber slab bonded to the printed brake pad's spool-facing
# contact face. Built at the engaged pose then rotated to rest pose, same
# kinematics as the printed pad — so it always sits flush on the pad.
def _brake_pad_rubber():
    from .dimensions import FLANGE_OD, FLANGE_RIM_MID_R
    from .housing import BRAKE_INNER_TRAVEL_DEG, BRAKE_PIVOT_X
    from .levers import (
        BRAKE_LEVER_Y0, BRAKE_LEVER_Y1,
        BRAKE_RUBBER_T, LEVER_Z_TOP, PAD_REST_LIFT,
    )

    y_near = BRAKE_LEVER_Y1
    y_far  = BRAKE_LEVER_Y0
    r_in   = FLANGE_RIM_MID_R
    r_out  = FLANGE_OD / 2

    x_in_near  = math.sqrt(r_in ** 2  - y_near ** 2)
    x_out_near = math.sqrt(r_out ** 2 - y_near ** 2)
    x_in_far   = math.sqrt(r_in ** 2  - y_far ** 2)
    x_out_far  = math.sqrt(r_out ** 2 - y_far ** 2)
    y_mid = (y_near + y_far) / 2
    x_in_mid  = math.sqrt(r_in ** 2  - y_mid ** 2)
    x_out_mid = math.sqrt(r_out ** 2 - y_mid ** 2)

    # Inverse-kinematic z of the printed pad's contact face at engaged
    # (matches _brake_pad_contact's pad_bot_engaged_z exactly).
    BRAKE_TRACK_Z = 0.0
    _pad_inner_rest_z = BRAKE_TRACK_Z - PAD_REST_LIFT - BRAKE_RUBBER_T
    _y_max_abs        = max(abs(y_near), abs(y_far))
    _x_rel_corner     = math.sqrt(r_in ** 2 - _y_max_abs ** 2) - BRAKE_PIVOT_X
    _theta            = math.radians(BRAKE_INNER_TRAVEL_DEG)
    _z_rel = (_pad_inner_rest_z - LEVER_PIVOT_Z + _x_rel_corner * math.sin(_theta)) / math.cos(_theta)
    rubber_bot_engaged_z = LEVER_PIVOT_Z + _z_rel
    rubber_top_engaged_z = rubber_bot_engaged_z + BRAKE_RUBBER_T

    rubber_solid = (
        cq.Workplane("XY").workplane(offset=rubber_bot_engaged_z)
        .moveTo(x_in_far, y_far)
        .lineTo(x_out_far, y_far)
        .threePointArc((x_out_mid, y_mid), (x_out_near, y_near))
        .lineTo(x_in_near, y_near)
        .threePointArc((x_in_mid, y_mid), (x_in_far, y_far))
        .close()
        .extrude(rubber_top_engaged_z - rubber_bot_engaged_z)
    )

    rubber_solid = rubber_solid.rotate(
        (BRAKE_PIVOT_X, 0, LEVER_PIVOT_Z),
        (BRAKE_PIVOT_X, 1, LEVER_PIVOT_Z),
        BRAKE_INNER_TRAVEL_DEG,
    )

    return rubber_solid


brake_pad_rubber = _brake_pad_rubber()
