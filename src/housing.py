"""Housing — L-bracket holding the spool and carrying both levers.

PANCAKE / 90°-LEVER REWRITE
===========================
The spool's brake + ratchet bands now live on the OUTER cylindrical face
of the bottom rim (RIM_OD = 114.4 → r = 57.2), engaged RADIALLY. The
levers were rotated 90° to match: instead of sitting under the spool and
swinging in a vertical plane to push axial contacts, each lever now hangs
DOWN the +X side of the housing on a Y-axis M2 pivot screw and swings in
the X-Z plane to push a radial contact against the rim.

Geometry forced by "both handles hang down and are pulled toward +X":
  - RATCHET pivot sits ABOVE the teeth band (z = 7..14). Pulling the
    handle (+X, below the pivot) swings the pawl — which is BELOW the
    pivot — radially OUTWARD, disengaging it. Rest = engaged (spring
    holds the pawl in).
  - BRAKE pivot sits BELOW the brake band (z = 0..7). Pulling the handle
    swings the pad — which is ABOVE the pivot — radially INWARD, engaging
    it. Rest = disengaged (spring holds the pad off).

Because the levers mount on the +X face (not a separate -Z plate), the
housing is now a basic L: a vertical FRONT-HOUSING block on +X (spanning
the full lever Z range) + a horizontal PANCAKE plate across the spool top
that carries the axle bearing/cross-pin, with a plain -X extension prism
left over from the (removed) cable-guide axle.

The front-housing X position is re-anchored to the brake rim (RIM_OD/2)
plus a clearance/leverage offset — NOT the stale FLANGE_OD the old drum
spool used.
"""

import math
import cadquery as cq

from .dimensions import (
    AXLE_PRINT_D,
    BOOL_OVERSHOOT,
    M2_HEAD_RECESS_D, M2_HEAD_RECESS_H,
    M2_INSERT_DEPTH, M2_INSERT_PILOT_D, M2_SHAFT_CLR_D,
    PANCAKE_CROSS_PIN_Z, PLATE_T,
    HOUSING_GAP_PANCAKE,
    SPOOL_H,
)
from .helpers import cyl, cone_solid
from .spool import RIM_OD, RIM_H

# ────────────────────────────────────────────────────────────────────────────
# OVERALL HOUSING ENVELOPE
# ────────────────────────────────────────────────────────────────────────────

HOUSING_W       = 22.0    # Y width of the housing (matches bearing OD region)
HOUSING_SPINE_T = 10.0    # X thickness of the vertical front-housing block —
                          # solid mounting material for the wood-screw cover.
HOUSING_HOLE_CLR = 0.25   # axle hole slip fit (radial)

# Front-housing inner face re-anchored to the brake rim. The spool's
# largest rotating feature is now the lever rim (RIM_OD = 114.4, r = 57.2);
# the block sits FRONT_HOUSING_CLR past it. The clearance doubles as the
# lever leverage knob: the pivot (block mid-X) ends up FRONT_HOUSING_CLR +
# HOUSING_SPINE_T/2 outboard of the rim, setting the contact-arm length.
FRONT_HOUSING_CLR = 5.0                                 # = old SPOOL_CLEARANCE
SPINE_X_INNER = RIM_OD / 2 + FRONT_HOUSING_CLR          # 62.2
SPINE_X_OUTER = SPINE_X_INNER + HOUSING_SPINE_T         # 75.2

# Pancake plate (horizontal arm of the L) sits above the spool; thickness
# grows OUTWARD (away from the spool) so the bearing gap is unchanged.
PANCAKE_PLATE_Z_IN  = SPOOL_H + HOUSING_GAP_PANCAKE     # 53 — spool-facing face
PANCAKE_PLATE_Z_OUT = PANCAKE_PLATE_Z_IN + PLATE_T      # 64 — outer (wood-mount) face

# Pancake plate X extent: from a tail just past the axle's -X edge out to
# the front-housing inner face. The plain -X extension prism (left from the
# removed guide axle) continues from the tail to BACK_EXT_X_TAIL.
PANCAKE_HOUSING_X_TAIL = -6.0       # 2 mm past the axle hole's -x edge
BACK_EXT_X_TAIL        = -65.3      # -X extension tip (temporary, per design)

# ────────────────────────────────────────────────────────────────────────────
# LEVER PIVOTS  — both on a common X (through the front-housing block), at
# different Z (ratchet high above the teeth, brake low below the band). The
# M2 pivot screw runs along Y through the block; the lever bodies sit on the
# block's outer Y faces (ratchet +Y, brake -Y) and hang downward — nothing
# of the lever passes through the block.
# ────────────────────────────────────────────────────────────────────────────
PIVOT_X         = (SPINE_X_INNER + SPINE_X_OUTER) / 2   # block mid-X
RATCHET_PIVOT_X = PIVOT_X
BRAKE_PIVOT_X   = PIVOT_X
# Pivot heights are set by the kinematic assertions (lever_kinematics.py):
#   - ratchet high enough above the teeth that a 16° pull retracts the pawl
#     clear of the tooth tips by ≥1 mm,
#   - brake as HIGH as the pad-overlap assertion allows (to minimise how far
#     the levers hang below the spool), but still below the band so the pull
#     swings the pad inward.
# Set to the assertion FLOORS at LEVER_TRAVEL_DEG: ratchet as LOW as it can be
# while still clearing the teeth by 1 mm at full pull, brake as HIGH as it can
# be while still compressing the pad 0.5 mm — both chosen to minimise how far
# the levers hang below the spool (-Z extent).
RATCHET_PIVOT_Z = 17.0                                  # above teeth top (14)
BRAKE_PIVOT_Z   = -3.5                                  # below brake band bottom (0)

# Front-housing block Z extent: from below the brake pivot (room for the
# screw + wall) up to the pancake plate's outer face.
FRONT_Z_BOT = BRAKE_PIVOT_Z - 7.0                       # -12
FRONT_Z_TOP = PANCAKE_PLATE_Z_OUT                       # 64

# Lever-side gap + spring sizing (shared with levers.py and viz.py).
LEVER_RIM_H     = 4.5     # Y gap between each lever's inner face and the
                          # block's outer Y face. The torsion-spring coil
                          # lives in this gap around the pivot screw.
SPRING_BODY_LEN = 2.5     # axial (Y) extent of the coil
LEVER_PIVOT_BOSS_OD = 6.0 # reinforcement boss OD coaxial with each pivot

LEVER_SCREW_CLR_D      = M2_SHAFT_CLR_D
LEVER_INSERT_PILOT_D   = M2_INSERT_PILOT_D
LEVER_INSERT_DEPTH     = M2_INSERT_DEPTH
LEVER_INSERT_CHAMFER_H = (LEVER_INSERT_PILOT_D - LEVER_SCREW_CLR_D) / 2
SCREW_HEAD_RECESS_D    = M2_HEAD_RECESS_D
SCREW_HEAD_RECESS_H    = M2_HEAD_RECESS_H


# ────────────────────────────────────────────────────────────────────────────
# STOP PINS + TORSION-SPRING ANCHORS
#
# Each lever gets: a LEVER pin (rotates with the lever) and two HOUSING pins
# — a REST pin (catches the lever pin at the rest pose) and a SPRING pin (at
# the far end of the pull travel, carrying the torsion-spring leg and acting
# as the travel limit). All pins are plain cylinders printed in place (the
# L-housing's print orientation differs from the old U, so the separately-
# glued brake pin is no longer needed); each carries a diametral through-hole
# for a spring leg.
#
# α convention (shared with the lever pose transform in build.py):
#     x = pivot_x + R·cos α ,  z = pivot_z − R·sin α
# so increasing α rotates CW about +Y. Pulling a handle (+X, below pivot)
# DECREASES the lever pin's α; the spring restores it (increasing α) until
# the lever pin re-seats on the REST pin.
# ────────────────────────────────────────────────────────────────────────────
STOP_PIN_D      = 4.0     # pin OD (0.75 mm walls around the 2 mm leg hole)
STOP_PIN_HOLE_D = 2.0     # spring-leg through-hole Ø (loose; prints reliably)
STOP_PIN_H      = 3.5     # Y projection of each pin into the lever gap
STOP_PIN_R      = 6.0     # radial offset of the pins from the pivot
SPRING_WIRE_R   = 0.25

# Both levers seat their LEVER pin straight down from the pivot (α = 90°),
# in the spring gap (so it never collides with the contact arm or handle
# body, which sit further out in Y).
STOP_LEVER_PIN_ALPHA_DEG = 90.0

# Angular separation at which two STOP_PIN_D pins on a circle of STOP_PIN_R
# just touch (chord = STOP_PIN_D).
_STOP_CONTACT_SEP_DEG = math.degrees(2 * math.asin(STOP_PIN_D / (2 * STOP_PIN_R)))

# Pull travel for each lever (handle pulled +X until the lever pin meets the
# spring pin). Both rotate the same real direction (α decreasing). EQUAL for
# the two levers so they start and end at the same angles (asserted in
# lever_kinematics.py: A_ANGLES_MATCH).
LEVER_TRAVEL_DEG         = 18.0   # > old 12° — more travel lets the pivots sit
                                  # closer to the bands, reducing -Z hang
RATCHET_OUTER_TRAVEL_DEG = LEVER_TRAVEL_DEG   # disengage travel
BRAKE_INNER_TRAVEL_DEG   = LEVER_TRAVEL_DEG   # engage travel

# Print warp pre-compensation: printed levers warp a few degrees toward
# their motion; pre-rotate the rest pose back by this so they settle true.
LEVER_REST_PRECOMP_DEG = 5.0

# Per-lever pin α positions. Lever pin at α=90; spring pin TRAVEL+SEP below
# (lower α), rest pin SEP+precomp above (higher α).
RATCHET_LEVER_PIN_ALPHA  = STOP_LEVER_PIN_ALPHA_DEG
RATCHET_SPRING_PIN_ALPHA = (STOP_LEVER_PIN_ALPHA_DEG
                            - RATCHET_OUTER_TRAVEL_DEG - _STOP_CONTACT_SEP_DEG)
RATCHET_REST_PIN_ALPHA   = (STOP_LEVER_PIN_ALPHA_DEG
                            + _STOP_CONTACT_SEP_DEG + LEVER_REST_PRECOMP_DEG)
BRAKE_LEVER_PIN_ALPHA    = STOP_LEVER_PIN_ALPHA_DEG
BRAKE_SPRING_PIN_ALPHA   = (STOP_LEVER_PIN_ALPHA_DEG
                            - BRAKE_INNER_TRAVEL_DEG - _STOP_CONTACT_SEP_DEG)
BRAKE_REST_PIN_ALPHA     = (STOP_LEVER_PIN_ALPHA_DEG
                            + _STOP_CONTACT_SEP_DEG + LEVER_REST_PRECOMP_DEG)

# Spring-leg tangent geometry: a leg tangent to the coil's mean circle
# (radius SPRING_COIL_MAJOR_R) reaching a pin at STOP_PIN_R touches the coil
# at coil-α = pin-α ± SPRING_LEG_PIN_OFFSET_DEG.
SPRING_COIL_MAJOR_R = 1.875
SPRING_LEG_PIN_OFFSET_DEG = math.degrees(
    math.acos(SPRING_COIL_MAJOR_R / STOP_PIN_R))


def stop_pin_solid(pivot_x, pivot_z, alpha_deg, y_from, y_to):
    """Plain cylindrical stop pin, axis +Y, OD STOP_PIN_D, spanning
    [y_from, y_to]. Center at pivot + STOP_PIN_R·(cos α, 0, −sin α)."""
    a = math.radians(alpha_deg)
    x = pivot_x + STOP_PIN_R * math.cos(a)
    z = pivot_z - STOP_PIN_R * math.sin(a)
    return (
        cq.Workplane("XY").circle(STOP_PIN_D / 2)
        .extrude(y_to - y_from)
        .rotate((0, 0, 0), (1, 0, 0), -90)
        .translate((x, y_from, z))
    )


def stop_pin_hole(pivot_x, pivot_z, alpha_deg, hole_y,
                  hole_dir_alpha_deg=None, length=None):
    """Diametral spring-leg through-hole cutter for a stop pin. Runs along
    (cos β, 0, −sin β) where β defaults to the pin's radial α (pass
    hole_dir_alpha_deg to align with the spring leg's tangent instead)."""
    a = math.radians(alpha_deg)
    x = pivot_x + STOP_PIN_R * math.cos(a)
    z = pivot_z - STOP_PIN_R * math.sin(a)
    if hole_dir_alpha_deg is None:
        hole_dir_alpha_deg = alpha_deg
    b = math.radians(hole_dir_alpha_deg)
    dx, dz = math.cos(b), -math.sin(b)
    L = STOP_PIN_D + 0.4 if length is None else length
    start = cq.Vector(x - (STOP_PIN_D / 2 + 0.2) * dx,
                      hole_y,
                      z - (STOP_PIN_D / 2 + 0.2) * dz)
    return cq.Workplane("XY").add(cq.Solid.makeCylinder(
        STOP_PIN_HOLE_D / 2, L, pnt=start, dir=cq.Vector(dx, 0, dz)))


def spring_leg_hole_dir_alpha_deg(leg_alpha_deg):
    """Hole-direction β for a spring leg attached at coil-α = leg_alpha_deg:
    its tangent direction is (−sin α, 0, −cos α) = (cos β, 0, −sin β) with
    β = α + 90°."""
    return leg_alpha_deg + 90.0


def pivot_boss_sector(pivot_x, pivot_z, y0, y1, *, od=LEVER_PIVOT_BOSS_OD):
    """Full-disc cylindrical boss coaxial with a pivot, axis +Y, spanning
    [y0, y1]. (The radial clearance from the Ø6 boss to the stop pins at
    r=6, inner edge r=4, is 1 mm, so no sector trimming is needed.)"""
    r = od / 2
    return (
        cq.Workplane("XZ").circle(r).extrude(-(y1 - y0))
        .translate((pivot_x, y0, pivot_z))
    )


# ────────────────────────────────────────────────────────────────────────────
# M2 PIVOT-SCREW CUTS (lever pivots)
# ────────────────────────────────────────────────────────────────────────────
def _pivot_clearance(x, z):
    """M2 shaft clearance through the full block width (axis Y)."""
    return (
        cq.Workplane("XY").circle(LEVER_SCREW_CLR_D / 2)
        .extrude(HOUSING_W + 2)
        .rotate((0, 0, 0), (1, 0, 0), -90)
        .translate((x, -HOUSING_W / 2 - 1, z))
    )


def _pivot_insert_pilot(x, z, insert_face_sign):
    """M2 heat-set insert pilot drilled from one Y face, with a 45° cone
    lead-in to the shaft clearance so the step prints self-supporting."""
    pilot = (
        cq.Workplane("XY").circle(LEVER_INSERT_PILOT_D / 2)
        .extrude(LEVER_INSERT_DEPTH)
        .union(cone_solid(LEVER_INSERT_PILOT_D, LEVER_SCREW_CLR_D,
                          LEVER_INSERT_CHAMFER_H, LEVER_INSERT_DEPTH))
    )
    if insert_face_sign > 0:
        return pilot.rotate((0, 0, 0), (1, 0, 0), +90).translate((x, HOUSING_W / 2, z))
    return pilot.rotate((0, 0, 0), (1, 0, 0), -90).translate((x, -HOUSING_W / 2, z))


# ────────────────────────────────────────────────────────────────────────────
# AXLE CROSS-PIN CUTS (pancake side) — also reused by mount_bracket.py
# ────────────────────────────────────────────────────────────────────────────
def _axle_pin_clearance(z_center, x_center=0.0):
    """M2 through-hole at (x, z), axis Y, full housing width."""
    return (
        cq.Workplane("XY").circle(LEVER_SCREW_CLR_D / 2)
        .extrude(HOUSING_W + 2)
        .rotate((0, 0, 0), (1, 0, 0), -90)
        .translate((x_center, -HOUSING_W / 2 - 1, z_center))
    )


def _axle_pin_insert_pilot(z_center, insert_face_sign, recess_depth=0.0,
                           x_center=0.0):
    pilot = (
        cq.Workplane("XY").circle(LEVER_INSERT_PILOT_D / 2)
        .extrude(LEVER_INSERT_DEPTH)
        .union(cone_solid(LEVER_INSERT_PILOT_D, LEVER_SCREW_CLR_D,
                          LEVER_INSERT_CHAMFER_H, LEVER_INSERT_DEPTH))
    )
    if insert_face_sign > 0:
        return pilot.rotate((0, 0, 0), (1, 0, 0), +90) \
                    .translate((x_center, HOUSING_W / 2 - recess_depth, z_center))
    return pilot.rotate((0, 0, 0), (1, 0, 0), -90) \
                .translate((x_center, -HOUSING_W / 2 + recess_depth, z_center))


def _screw_head_counterbore(x, z, entry_face_sign):
    """M2 cap-head counterbore on a Y face. entry_face_sign +1 = +Y, -1 = -Y."""
    cb = cq.Workplane("XY").circle(SCREW_HEAD_RECESS_D / 2).extrude(SCREW_HEAD_RECESS_H)
    if entry_face_sign > 0:
        return cb.rotate((0, 0, 0), (1, 0, 0), -90) \
                 .translate((x, HOUSING_W / 2 - SCREW_HEAD_RECESS_H, z))
    return cb.rotate((0, 0, 0), (1, 0, 0), +90) \
             .translate((x, -HOUSING_W / 2 + SCREW_HEAD_RECESS_H, z))


# ────────────────────────────────────────────────────────────────────────────
# HOUSING BODY (L-bracket)
# ────────────────────────────────────────────────────────────────────────────
def _front_block():
    """Vertical front-housing block on +X — the lever-mount + wood-mount
    wall of the L."""
    return (
        cq.Workplane("XY").workplane(offset=FRONT_Z_BOT)
        .center((SPINE_X_INNER + SPINE_X_OUTER) / 2, 0)
        .box(HOUSING_SPINE_T, HOUSING_W, FRONT_Z_TOP - FRONT_Z_BOT,
             centered=(True, True, False))
    )


def _pancake_plate():
    """Horizontal top plate (over the spool) + the plain -X extension prism,
    as one slab. The axle hole is cut later."""
    return (
        cq.Workplane("XY").workplane(offset=PANCAKE_PLATE_Z_IN)
        .center((BACK_EXT_X_TAIL + SPINE_X_INNER) / 2, 0)
        .box(SPINE_X_INNER - BACK_EXT_X_TAIL, HOUSING_W, PLATE_T,
             centered=(True, True, False))
    )


def _axle_round_hole():
    return cyl(AXLE_PRINT_D + 2 * HOUSING_HOLE_CLR, PLATE_T + 1,
               z=PANCAKE_PLATE_Z_IN - 0.5)


def _build_lever_mounts(h):
    """Add both levers' pivot-insert/clearance cuts and their housing stop
    pins (+ spring-leg holes). Ratchet on +Y, brake on -Y."""
    # ── Ratchet (+Y side): screw enters from -Y, insert in +Y face ──
    h = (h.cut(_pivot_clearance(RATCHET_PIVOT_X, RATCHET_PIVOT_Z))
          .cut(_pivot_insert_pilot(RATCHET_PIVOT_X, RATCHET_PIVOT_Z, insert_face_sign=+1)))
    # ── Brake (-Y side): screw enters from +Y, insert in -Y face ──
    h = (h.cut(_pivot_clearance(BRAKE_PIVOT_X, BRAKE_PIVOT_Z))
          .cut(_pivot_insert_pilot(BRAKE_PIVOT_X, BRAKE_PIVOT_Z, insert_face_sign=-1)))

    # Housing stop pins project from the block face into the gap toward the
    # lever (+Y for ratchet, -Y for brake).
    y_face_pos = HOUSING_W / 2
    # Ratchet rest + spring pins (+Y).
    for alpha, is_spring in ((RATCHET_REST_PIN_ALPHA, False),
                             (RATCHET_SPRING_PIN_ALPHA, True)):
        h = h.union(stop_pin_solid(RATCHET_PIVOT_X, RATCHET_PIVOT_Z, alpha,
                                   y_face_pos - 0.5, y_face_pos + STOP_PIN_H))
        if is_spring:
            leg_a = alpha + (-1) * SPRING_LEG_PIN_OFFSET_DEG
            h = h.cut(stop_pin_hole(RATCHET_PIVOT_X, RATCHET_PIVOT_Z, alpha,
                                    hole_y=y_face_pos + STOP_PIN_H - STOP_PIN_HOLE_D,
                                    hole_dir_alpha_deg=spring_leg_hole_dir_alpha_deg(leg_a)))
    # Brake rest + spring pins (-Y).
    for alpha, is_spring in ((BRAKE_REST_PIN_ALPHA, False),
                             (BRAKE_SPRING_PIN_ALPHA, True)):
        h = h.union(stop_pin_solid(BRAKE_PIVOT_X, BRAKE_PIVOT_Z, alpha,
                                   -y_face_pos - STOP_PIN_H, -y_face_pos + 0.5))
        if is_spring:
            leg_a = alpha + (-1) * SPRING_LEG_PIN_OFFSET_DEG
            h = h.cut(stop_pin_hole(BRAKE_PIVOT_X, BRAKE_PIVOT_Z, alpha,
                                    hole_y=-y_face_pos - STOP_PIN_H + STOP_PIN_HOLE_D,
                                    hole_dir_alpha_deg=spring_leg_hole_dir_alpha_deg(leg_a)))
    return h


def _build_housing_skeleton():
    h = (_front_block()
         .union(_pancake_plate())
         .cut(_axle_round_hole()))
    h = _build_lever_mounts(h)
    # Axle cross-pin (pancake side): insert on +Y, head counterbore on -Y.
    h = (h.cut(_axle_pin_clearance(PANCAKE_CROSS_PIN_Z))
          .cut(_axle_pin_insert_pilot(PANCAKE_CROSS_PIN_Z, insert_face_sign=+1))
          .cut(_screw_head_counterbore(0, PANCAKE_CROSS_PIN_Z, entry_face_sign=-1)))
    return h


def _build_housing():
    h = _build_housing_skeleton()
    from . import mount_bracket
    h = mount_bracket.cut_from_housing(h)
    return h


housing = _build_housing()
