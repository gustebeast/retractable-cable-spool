"""Housing — U-bracket holding the spool and carrying both levers.

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
housing is a U: a vertical FRONT-HOUSING block on +X (carrying both lever
pivots + stop pins) and a mirrored BACK-HOUSING block on -X (which will host
the guide wheel that resists the brake lever's -X push), joined across the
spool top by a horizontal PANCAKE plate that carries the axle bearing/cross-
pin. Both blocks use the same spool gap (mirror in X).

The front-housing X position is re-anchored to the brake rim (RIM_OD/2)
plus a clearance/leverage offset — NOT the stale FLANGE_OD the old drum
spool used.
"""

import math
import cadquery as cq

from .dimensions import (
    AXLE_PRINT_D,
    BOOL_OVERSHOOT,
    FIT_CLR,
    M2_HEAD_RECESS_D, M2_HEAD_RECESS_H,
    M2_INSERT_DEPTH, M2_INSERT_PILOT_D, M2_SHAFT_CLR_D,
    PANCAKE_CROSS_PIN_Z, PLATE_T,
    HOUSING_GAP_PANCAKE,
    SPOOL_H, STRUCT_WALL,
)
from .helpers import cyl, cone_solid
from .spool import RIM_OD, RIM_H, RATCHET_BAND_H
from .cable_retainer import (
    RETAIN_Z_LO, RETAIN_Z_HI,
    RETAIN_R_IN, RETAIN_R_OUT, RETAIN_N_BARS,
    RETAIN_WINDOW_W, RETAIN_WINDOW_H,
    RETAIN_WINDOW_Z_LO, RETAIN_WINDOW_Z_HI,
)

# ────────────────────────────────────────────────────────────────────────────
# OVERALL HOUSING ENVELOPE
# ────────────────────────────────────────────────────────────────────────────

HOUSING_W       = 22.0    # Y width of the housing (matches bearing OD region)
HOUSING_SPINE_T = 10.0    # X thickness of the vertical front-housing block —
                          # solid mounting material for the wood-screw cover.
HOUSING_HOLE_CLR = 0.25   # axle hole slip fit (radial)

# ── Stop-pin geometry (defined early so the front-housing +X face can be
#    positioned to swallow the pins — see SPINE_X_OUTER below) ──────────────
STOP_PIN_D = 4.0          # pin OD (0.75 mm walls around the 2 mm leg hole)
STOP_PIN_R = 6.0          # radial offset of the pins from the pivot
STOP_LEVER_PIN_ALPHA_DEG = 90.0   # lever pin straight down from the pivot
LEVER_TRAVEL_DEG = 20.0   # equal pull travel for both levers (A_ANGLES_MATCH).
                          # Bumped 18→20 to keep ratchet pawl clearance margin after
                          # the teeth grew outward (valley/tip +RATCHET_DEPTH).
_STOP_CONTACT_SEP_DEG = math.degrees(2 * math.asin(STOP_PIN_D / (2 * STOP_PIN_R)))

# Lever pivot X is fixed by the contact arm to the rim (10 mm), DECOUPLED from
# the block's mid-plane — so the block can slide outward to clear the stop
# pins without dragging the pivot (and the pins) out with it.
PIVOT_X         = RIM_OD / 2 + 10.0                     # 67.2
RATCHET_PIVOT_X = PIVOT_X
BRAKE_PIVOT_X   = PIVOT_X

# The +X-most fixed stop pin is the SPRING pin (lowest α). Its outer edge must
# sit inside the block's +X (wood-mount) face so the housing mounts flush on a
# slab. Place the +X face MOUNT_PIN_CLR beyond that edge; the inner face — and
# hence the spool gap — then follows from the fixed HOUSING_SPINE_T. (This is
# why the spool gap grew: the block slid out to recess the pins, leaving the
# pivot/pins where they were. The -X-most pin now pokes past the INNER face
# into the spool gap; it clears the rim and gets a small support boss.)
_SPRING_PIN_ALPHA_DEG = (STOP_LEVER_PIN_ALPHA_DEG - LEVER_TRAVEL_DEG
                         - _STOP_CONTACT_SEP_DEG)
_PIN_MAX_X    = (PIVOT_X + STOP_PIN_R * math.cos(math.radians(_SPRING_PIN_ALPHA_DEG))
                 + STOP_PIN_D / 2)
MOUNT_PIN_CLR = 0.5                                     # pins recessed this far behind the +X face
SPINE_X_OUTER = _PIN_MAX_X + MOUNT_PIN_CLR              # ≈ 74.73
SPINE_X_INNER = SPINE_X_OUTER - HOUSING_SPINE_T         # ≈ 64.73
FRONT_HOUSING_CLR = SPINE_X_INNER - RIM_OD / 2          # derived spool gap (≈ 7.5)

# Pancake plate (horizontal arm of the L) sits above the spool; thickness
# grows OUTWARD (away from the spool) so the bearing gap is unchanged.
PANCAKE_PLATE_Z_IN  = SPOOL_H + HOUSING_GAP_PANCAKE     # 53 — spool-facing face
PANCAKE_PLATE_Z_OUT = PANCAKE_PLATE_Z_IN + PLATE_T      # 64 — outer (wood-mount) face

# U-HOUSING: a BACK-housing block on -X mirrors the front block across x=0,
# reusing the same spool-housing gap. It will host the guide wheel that resists
# the brake lever's -X push on the rim. The pancake plate now spans the full U
# — from the back block's inner face (-X) to the front block's inner face (+X)
# — so each vertical block extends outward from where the pancake meets it.
BACK_SPINE_X_INNER = -SPINE_X_INNER      # -64.73 — back block spool-facing face
BACK_SPINE_X_OUTER = -SPINE_X_OUTER      # -74.73 — back block outer (wood-mount) face
BACK_HOUSING_CLR   = FRONT_HOUSING_CLR   # same spool gap as the front (mirror in X)

# ────────────────────────────────────────────────────────────────────────────
# LEVER PIVOTS  — both on a common X (through the front-housing block), at
# different Z (ratchet high above the teeth, brake low below the band). The
# M2 pivot screw runs along Y through the block; the lever bodies sit on the
# block's outer Y faces (ratchet +Y, brake -Y) and hang downward — nothing
# of the lever passes through the block.
# ────────────────────────────────────────────────────────────────────────────
# (PIVOT_X / RATCHET_PIVOT_X / BRAKE_PIVOT_X are defined in the envelope
# section above — decoupled from the block faces.)
# Pivot heights are set to the kinematic-assertion FLOORS (lever_kinematics.py)
# at LEVER_TRAVEL_DEG. With the bands swapped (ratchet teeth on the BOTTOM
# z=0..7, brake band on TOP z=7..14):
#   - ratchet pivot just above the teeth (pull swings the pawl radially out),
#   - brake pivot just below the brake band — which now lands ABOVE the spool
#     bottom, so neither pivot is dragged negative.
# Both at their floors to minimise how far the levers hang below the spool.
RATCHET_PIVOT_Z = 10.0                                  # above teeth top (7)
BRAKE_PIVOT_Z   = 2.0                                   # lowered 3.5→2.0 so the brake pad,
                          # now on the LOWER band (clear of the cable retainer), keeps a long
                          # enough arm above the pivot to preserve brake overlap.

# Front-housing block Z extent: from below the brake pivot (room for the
# screw + wall) up to the pancake plate's outer face.
FRONT_Z_BOT = min(RATCHET_PIVOT_Z, BRAKE_PIVOT_Z) - 7.0  # -3.5
FRONT_Z_TOP = PANCAKE_PLATE_Z_OUT                       # 64

# ────────────────────────────────────────────────────────────────────────────
# GUIDE WHEEL  (-X side, in the back block)
#
# The brake lever (on +X) pushes the rim toward -X when it engages; the axle
# alone can't react that, so a Ø GUIDE_WHEEL_OD wheel bears on the rim's OUTER
# face at the -X point, reacting the load straight back. Its axle is ALONG Z
# (parallel to the spool axis) so it ROLLS — not skids — as the rim's surface
# sweeps tangentially (±Y) past the contact point while the spool spins.
#
# Width (its Z extent) is maxed out between the two spool features that bracket
# the brake band: the ratchet teeth below (top at RATCHET_BAND_H) and the cable
# retainer's bottom ring above (RETAIN_Z_LO), keeping GUIDE_WHEEL_Z_CLR clear of
# each. The wheel sits in a +X-open pocket cut into the back block, with a
# STRUCT_WALL-thick wall behind it (-X) and below it (-Z); the leftover space
# between that back wall and the wheel is the air gap.
GUIDE_WHEEL_OD     = 14.0
GUIDE_WHEEL_Z_CLR  = 1.0                                  # gap to ratchet rim / cable retainer
GUIDE_WHEEL_Z_LO   = RATCHET_BAND_H + GUIDE_WHEEL_Z_CLR   # 4.0 — 1 mm above the ratchet teeth
GUIDE_WHEEL_Z_HI   = RETAIN_Z_LO - GUIDE_WHEEL_Z_CLR      # 11.3 — 1 mm below the retainer ring
GUIDE_WHEEL_W      = GUIDE_WHEEL_Z_HI - GUIDE_WHEEL_Z_LO  # 7.3 — axial (Z) width
GUIDE_WHEEL_CX     = -(RIM_OD / 2 + GUIDE_WHEEL_OD / 2)   # -64.2 — center; surface tangent to rim
GUIDE_WHEEL_BORE_D = 2.6                                  # spinning clearance for a future M2 axle

# Pocket: open toward +X (so the wheel reaches the rim), walled STRUCT_WALL
# thick on -X (behind) and -Z (below). The back leg's bottom is the underside
# of that 1.7 mm floor; everything between the floor and the wheel is air.
GUIDE_POCKET_WALL    = STRUCT_WALL                            # 1.7 — back + bottom walls
GUIDE_POCKET_X_BACK  = BACK_SPINE_X_OUTER + GUIDE_POCKET_WALL # -73.03 — pocket back face
GUIDE_WHEEL_FACE_CLR = 1.0                                    # housing clearance off the wheel faces
GUIDE_POCKET_Z_LO    = GUIDE_WHEEL_Z_LO - GUIDE_WHEEL_FACE_CLR # 3.0 — pocket floor (top of bottom wall)
GUIDE_POCKET_Z_HI    = GUIDE_WHEEL_Z_HI + GUIDE_WHEEL_FACE_CLR # 12.3 — pocket ceiling
GUIDE_POCKET_Y_HALF  = HOUSING_W / 2 - GUIDE_POCKET_WALL    # 9.3 — leaves a 1.7 mm wall on each ±Y side
# Back leg bottoms at the underside of the 1.7 mm floor (the rest of its old
# full-height leg below the wheel is dropped — it served no purpose).
BACK_Z_BOT = GUIDE_POCKET_Z_LO - GUIDE_POCKET_WALL          # 1.3

# The wall box reaches +X past the wheel's axle hole — which is coaxial with the
# wheel (along Z) — leaving GUIDE_AXLE_HOLE_CLR of material beyond the hole's +X
# edge. That solid band is where the wheel's mounting screw lands + the
# positioning material to come (detailed next). Sized off the wheel bore (the
# axle-hole Ø); revisit if the finalized screw hole is larger. (At -61.9 it
# stays 0.5 mm clear of the cable retainer's OD at x=-61.4.)
GUIDE_AXLE_HOLE_CLR = 1.0
GUIDE_BOX_X_FRONT   = GUIDE_WHEEL_CX + GUIDE_WHEEL_BORE_D / 2 + GUIDE_AXLE_HOLE_CLR  # -61.9

# ── Guide-wheel axle: an M2 × GUIDE_AXLE_SCREW_LEN socket-head screw along Z ──
# The screw self-taps a tight Ø GUIDE_AXLE_SHAFT_D hole in the floor and in a
# +Z boss; the wheel spins on the smooth shank between them (its Ø2.6 bore
# clears the screw). The head is NOT recessed — it sits proud on the OUTSIDE of
# the -Z (floor) wall, which is too thin for a counterbore. The +Z boss is grown
# tall enough to bury the screw tip + 1 mm so it doesn't poke out; to save
# material it's only GUIDE_AXLE_BOSS_WALL of material around the hole, joined
# back into the leg by a 45° face on -Y (self-supporting in the -Y→+Y print).
GUIDE_AXLE_SHAFT_D    = M2_SHAFT_CLR_D - 0.2     # 2.2 — tight thread fit (matches the old guide wheels)
GUIDE_AXLE_SCREW_LEN  = 20.0                     # M2 × 20 socket-head screw
GUIDE_AXLE_HEAD_Z     = BACK_Z_BOT               # 1.3 — head seats on the floor's outer (-Z) face
GUIDE_AXLE_HOLE_TOP_Z = GUIDE_AXLE_HEAD_Z + GUIDE_AXLE_SCREW_LEN + 1.0  # 22.3 — screw tip + 1 mm clearance
GUIDE_AXLE_BOSS_WALL  = 1.0                      # material around the screw hole in the +Z boss

# ── Low-friction wheel-retaining rings ───────────────────────────────────────
# Thin annular pads on the floor + ceiling that pinch the wheel's flat faces at
# a small radius, holding it axially with minimal contact (low spin friction) —
# same idea as the old guide-wheel hub bosses. Each is a GUIDE_RING_WIDTH-wide
# ring (ID = the wheel's Ø2.6 bore) standing GUIDE_RING_H proud of its housing
# face to meet the wheel; a 45° skirt on its bottom (-Y) quarter makes the outer
# overhang self-supporting for the -Y→+Y print (the rest of the ring self-
# supports past its own 45° points, so only the bottom needs propping).
GUIDE_RING_ID    = GUIDE_WHEEL_BORE_D                     # 2.6 — matches the wheel's axle bore
GUIDE_RING_WIDTH = 1.0                                    # radial width of the contact pad
GUIDE_RING_OD    = GUIDE_RING_ID + 2 * GUIDE_RING_WIDTH   # 4.6
GUIDE_RING_H     = GUIDE_WHEEL_FACE_CLR                   # 1.0 — spans the face gap to touch the wheel

_GAP = (GUIDE_WHEEL_CX - GUIDE_WHEEL_OD / 2) - GUIDE_POCKET_X_BACK  # wheel back ↔ back-wall inner face
assert _GAP > 0, f"guide-wheel air gap is negative ({_GAP:.2f} mm) — wheel hits the back wall"

# Lever-side gap + spring sizing (shared with levers.py and viz.py).
LEVER_RIM_H     = 4.5     # Y gap between each lever's inner face and the
                          # block's outer Y face. The torsion-spring coil
                          # lives in this gap around the pivot screw.
SPRING_BODY_LEN = 2.5     # axial (Y) extent of the coil
LEVER_PIVOT_BOSS_OD = 6.0 # reinforcement boss OD coaxial with each pivot
# Spring positioning ring: a short boss around each pivot hole on the block's
# lever-facing Y face. It bears on the coil's housing-side end, holding the
# spring centred in the gap. Paired 1:1 with the lever's own pivot-boss
# extension, the two split the (LEVER_RIM_H − SPRING_BODY_LEN) = 2 mm buffer.
HOUSING_BOSS_EXT = (LEVER_RIM_H - SPRING_BODY_LEN) / 2   # 1.0

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
STOP_PIN_HOLE_D = 1.5     # spring-leg through-hole Ø
STOP_PIN_H      = 3.5     # Y projection of each pin into the lever gap
SPRING_WIRE_R   = 0.25
# (STOP_PIN_D, STOP_PIN_R, STOP_LEVER_PIN_ALPHA_DEG, LEVER_TRAVEL_DEG, and
# _STOP_CONTACT_SEP_DEG are defined early in the envelope section so the
# block's +X face can be placed to clear the pins.)

# Both levers rotate the same real direction (α decreasing) by LEVER_TRAVEL_DEG,
# so they start and end at the same angles (asserted: A_ANGLES_MATCH).
RATCHET_OUTER_TRAVEL_DEG = LEVER_TRAVEL_DEG   # disengage travel
BRAKE_INNER_TRAVEL_DEG   = LEVER_TRAVEL_DEG   # engage travel

# Print warp pre-compensation: pre-rotate the rest pose back by this so the
# printed lever settles true. Set to 0 for now — the old 5° was tuned for the
# old (axial) levers' print orientation; the radial levers print flat on their
# Y face, a different warp profile, so leave it at 0 until measured. (Also
# keeps the rendered rest pose showing the pawl truly meshed.)
LEVER_REST_PRECOMP_DEG = 0.0

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


def _pivot_insert_pilot(x, z, insert_face_sign, boss_ext=0.0):
    """M2 heat-set insert pilot drilled from one Y face. The pilot→clearance
    step gets a 45° cone lead-in ONLY when the pilot opens on the -Y (build-
    plate) face — there the step faces down and would otherwise be an overhang
    for the -Y→+Y print. A +Y (top) pilot's step faces up (supported), so it's
    left flat. boss_ext shifts the pilot mouth OUTWARD (so the Ø3.3 bore
    carries through a spring-positioning ring protruding from the face)."""
    pilot = (cq.Workplane("XY").circle(LEVER_INSERT_PILOT_D / 2)
             .extrude(LEVER_INSERT_DEPTH))
    if insert_face_sign < 0:
        pilot = pilot.union(cone_solid(LEVER_INSERT_PILOT_D, LEVER_SCREW_CLR_D,
                                       LEVER_INSERT_CHAMFER_H, LEVER_INSERT_DEPTH))
    if insert_face_sign > 0:
        return pilot.rotate((0, 0, 0), (1, 0, 0), +90).translate((x, HOUSING_W / 2 + boss_ext, z))
    return pilot.rotate((0, 0, 0), (1, 0, 0), -90).translate((x, -HOUSING_W / 2 - boss_ext, z))


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
    pilot = (cq.Workplane("XY").circle(LEVER_INSERT_PILOT_D / 2)
             .extrude(LEVER_INSERT_DEPTH))
    if insert_face_sign < 0:   # cone only for -Y (build-plate) inserts
        pilot = pilot.union(cone_solid(LEVER_INSERT_PILOT_D, LEVER_SCREW_CLR_D,
                                       LEVER_INSERT_CHAMFER_H, LEVER_INSERT_DEPTH))
    if insert_face_sign > 0:
        return pilot.rotate((0, 0, 0), (1, 0, 0), +90) \
                    .translate((x_center, HOUSING_W / 2 - recess_depth, z_center))
    return pilot.rotate((0, 0, 0), (1, 0, 0), -90) \
                .translate((x_center, -HOUSING_W / 2 + recess_depth, z_center))


def _screw_head_counterbore(x, z, entry_face_sign):
    """M2 cap-head counterbore on a Y face. entry_face_sign +1 = +Y, -1 = -Y.
    A -Y (build-plate) counterbore gets a 45° cone at its inner step (Ø head →
    Ø shaft) so the recess floor isn't a flat downward overhang for the
    -Y→+Y print; the cap head still seats in the cylindrical part above it.
    A +Y counterbore's floor faces up (supported), so it stays flat."""
    cb = cq.Workplane("XY").circle(SCREW_HEAD_RECESS_D / 2).extrude(SCREW_HEAD_RECESS_H)
    if entry_face_sign > 0:
        return cb.rotate((0, 0, 0), (1, 0, 0), -90) \
                 .translate((x, HOUSING_W / 2 - SCREW_HEAD_RECESS_H, z))
    cone_h = (SCREW_HEAD_RECESS_D - LEVER_SCREW_CLR_D) / 2
    cb = cb.union(cone_solid(SCREW_HEAD_RECESS_D, LEVER_SCREW_CLR_D,
                             cone_h, SCREW_HEAD_RECESS_H))
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


# The stop pins sit -X of the block's inner face, so as separate cylinders
# they'd be thin features cantilevered off the wall (poor print support).
# Instead, thicken the inner face inward to the stop pins' -X extent over the
# lowest FRONT_THICKEN_H of the block, embedding the pins in solid wall.
FRONT_THICKEN_H = 16.0


def _front_thicken():
    x_in = (PIVOT_X + STOP_PIN_R * math.cos(math.radians(RATCHET_REST_PIN_ALPHA))
            - STOP_PIN_D / 2)                       # stop-pin -X extent (≈ 61.43)
    return (
        cq.Workplane("XY").workplane(offset=FRONT_Z_BOT)
        .center((x_in + SPINE_X_INNER) / 2, 0)
        .box(SPINE_X_INNER - x_in, HOUSING_W, FRONT_THICKEN_H,
             centered=(True, True, False))
    )


def _pancake_plate():
    """Horizontal top plate (the crossbar of the U), spanning between the two
    vertical blocks' inner faces. The axle hole is cut later."""
    return (
        cq.Workplane("XY").workplane(offset=PANCAKE_PLATE_Z_IN)
        .center((BACK_SPINE_X_INNER + SPINE_X_INNER) / 2, 0)
        .box(SPINE_X_INNER - BACK_SPINE_X_INNER, HOUSING_W, PLATE_T,
             centered=(True, True, False))
    )


def _back_block():
    """Vertical BACK-housing block on -X — the X-mirror of the front block,
    minus its lever/stop-pin features. Reaches up to the pancake's outer face
    (completing the U) and down to BACK_Z_BOT, the underside of the guide-wheel
    pocket's bottom wall. The +X-open wheel pocket is carved by
    _guide_wheel_pocket_cut(); the wall box that wraps the axle hole is bumped
    out from this leg's inner face by _guide_box_frame()."""
    return (
        cq.Workplane("XY").workplane(offset=BACK_Z_BOT)
        .center((BACK_SPINE_X_INNER + BACK_SPINE_X_OUTER) / 2, 0)
        .box(HOUSING_SPINE_T, HOUSING_W, FRONT_Z_TOP - BACK_Z_BOT,
             centered=(True, True, False))
    )


def _guide_box_frame():
    """New material bumped out in +X from the back leg's inner face — the box
    that wraps the wheel's axle hole. Built as a solid prism (inner face →
    GUIDE_BOX_X_FRONT, up to a STRUCT_WALL-thick roof above the pocket); the
    shared wheel-cavity cut then hollows its center, leaving four
    STRUCT_WALL-thick walls (floor, ceiling, ±Y) around the protruding wheel,
    open toward +X (the rim) and -X (into the back leg). Like _front_thicken(),
    it thickens the housing locally rather than moving the primary wall."""
    z_top = GUIDE_POCKET_Z_HI + GUIDE_POCKET_WALL          # 14.0 — 1.7 mm roof over the cavity
    return (
        cq.Workplane("XY").workplane(offset=BACK_Z_BOT)
        .center((BACK_SPINE_X_INNER + GUIDE_BOX_X_FRONT) / 2, 0)
        .box(GUIDE_BOX_X_FRONT - BACK_SPINE_X_INNER, HOUSING_W, z_top - BACK_Z_BOT,
             centered=(True, True, False))
    )


def _guide_wheel_pocket_cut():
    """The wheel cavity carved into the back block: a box open toward +X,
    leaving GUIDE_POCKET_WALL behind (-X) and below (-Z) and the ±Y walls.
    Spans +X clear past the box's front face (into the spool gap) so the wheel
    protrudes to the rim; the cut beyond the box is just air. Bounded in Y/Z to
    the wheel envelope, so the floor, ceiling, and ±Y walls keep their +X
    extension (the band that wraps the axle hole)."""
    x_lo = GUIDE_POCKET_X_BACK
    x_hi = GUIDE_WHEEL_CX + GUIDE_WHEEL_OD / 2 + BOOL_OVERSHOOT   # open past the wheel's +X extent
    return (
        cq.Workplane("XY").workplane(offset=GUIDE_POCKET_Z_LO)
        .center((x_lo + x_hi) / 2, 0)
        .box(x_hi - x_lo, 2 * GUIDE_POCKET_Y_HALF, GUIDE_POCKET_Z_HI - GUIDE_POCKET_Z_LO,
             centered=(True, True, False))
    )


def _guide_wheel():
    """The printed guide wheel — Ø GUIDE_WHEEL_OD × GUIDE_WHEEL_W disc, axis Z,
    centred at (GUIDE_WHEEL_CX, 0), with a center bore for a future M2 axle.
    Built at its assembled position (tangent to the rim at the -X point)."""
    body = cyl(GUIDE_WHEEL_OD, GUIDE_WHEEL_W, z=GUIDE_WHEEL_Z_LO)
    bore = cyl(GUIDE_WHEEL_BORE_D, GUIDE_WHEEL_W + 2 * BOOL_OVERSHOOT,
               z=GUIDE_WHEEL_Z_LO - BOOL_OVERSHOOT)
    return body.cut(bore).translate((GUIDE_WHEEL_CX, 0, 0))


guide_wheel = _guide_wheel()


def _guide_axle_boss():
    """+Z material around the axle screw, tall enough to bury the M2 tip + 1 mm
    (top at GUIDE_AXLE_HOLE_TOP_Z). Cross-section (extruded in Z): GUIDE_AXLE_BOSS_WALL
    of material around the hole on +X/+Y, merged into the back leg on -X, and a
    45° face on -Y (tangent to the clearance circle) that joins back into the
    leg — self-supporting for the -Y→+Y print. Sits atop the frame roof."""
    cx, cy = GUIDE_WHEEL_CX, 0.0
    rc = GUIDE_AXLE_SHAFT_D / 2 + GUIDE_AXLE_BOSS_WALL      # clearance radius (1 mm wall)
    x_front = cx + rc
    y_top   = cy + rc
    x_back  = BACK_SPINE_X_INNER - 0.5                      # buried in the leg for a clean union
    c = cy - cx - rc * math.sqrt(2)                         # 45° line tangent to the circle's -Y
    return (
        cq.Workplane("XY").workplane(offset=GUIDE_POCKET_Z_HI)
        .polyline([(x_front, y_top),
                   (x_front, x_front + c),                  # +X face down to the chamfer
                   (x_back,  x_back + c),                   # 45° -Y chamfer running into the leg
                   (x_back,  y_top)])                       # -X face (buried in the leg)
        .close()
        .extrude(GUIDE_AXLE_HOLE_TOP_Z - GUIDE_POCKET_Z_HI)
    )


def _guide_axle_screw_hole():
    """Ø GUIDE_AXLE_SHAFT_D tight-fit bore along Z for the M2 axle screw: from
    the floor's outer face (head seat) up through the floor and the +Z boss,
    ending 1 mm past the screw tip. Where it crosses the open wheel cavity it
    cuts nothing; the wheel (a separate part) carries its own Ø2.6 shank bore."""
    return cyl(GUIDE_AXLE_SHAFT_D, GUIDE_AXLE_HOLE_TOP_Z - GUIDE_AXLE_HEAD_Z,
               z=GUIDE_AXLE_HEAD_Z).translate((GUIDE_WHEEL_CX, 0, 0))


def _guide_ring(z_base, z_contact):
    """One retaining ring + its bottom-quarter 45° support skirt. z_base is the
    housing face it grows from (floor or ceiling); z_contact is the wheel face
    it meets. The contact face stays a clean ring — the skirt sits OUTSIDE the
    ring OD, BELOW the contact face, and only on the -Y bottom quarter (where the
    outer overhang actually needs propping for the -Y→+Y print)."""
    OS = BOOL_OVERSHOOT
    sgn = 1 if z_contact > z_base else -1                  # +1 floor ring (+Z), -1 ceiling ring (-Z)
    z_wall = z_base - sgn * OS                             # base buried OS into the wall for a clean union
    lo, hi = min(z_wall, z_contact), max(z_wall, z_contact)
    h = hi - lo
    ring = (cyl(GUIDE_RING_OD, h, z=lo)
            .cut(cyl(GUIDE_RING_ID, h + 2 * OS, z=lo - OS)))
    # 45° skirt: a cone hugging the ring's outer wall, ring OD at the contact end
    # widening (rise = run) to the wall end, core removed so it tapers to nothing
    # at the contact face and overlaps the ring for a clean union.
    big = GUIDE_RING_OD + 2 * (GUIDE_RING_H + OS)          # skirt OD at the (extended) wall end
    if sgn > 0:
        cone = cone_solid(big, GUIDE_RING_OD, h, lo)       # wide at the floor, narrow at the contact
    else:
        cone = cone_solid(GUIDE_RING_OD, big, h, lo)       # narrow at the contact, wide at the ceiling
    skirt = cone.cut(cyl(GUIDE_RING_OD - 0.2, h + 2 * OS, z=lo - OS))
    # Trim to the X-span of the ring's 45° points (x = ±OD/2·cos45) with clean
    # Y-aligned (constant-X) cut faces — material beyond those points serves no
    # purpose — and to the bottom half (y ≤ 0), dropping the top of the flare.
    x_half = GUIDE_RING_OD / 2 * math.cos(math.radians(45))    # 1.626
    keep = (cq.Workplane("XY").workplane(offset=lo - OS)
            .center(0, -10.0)
            .box(2 * x_half, 20.0, h + 2 * OS, centered=(True, True, False)))
    skirt = skirt.intersect(keep)
    return ring.union(skirt).translate((GUIDE_WHEEL_CX, 0, 0))


def _guide_retainer_rings():
    """Both retaining rings — one up from the floor to the wheel's bottom face,
    one down from the ceiling to its top face — pinching the wheel axially."""
    return (_guide_ring(GUIDE_POCKET_Z_LO, GUIDE_WHEEL_Z_LO)
            .union(_guide_ring(GUIDE_POCKET_Z_HI, GUIDE_WHEEL_Z_HI)))


# ── Cable-retainer mounts: square warts on the housing's ±X inner faces ─────
# Plug the retainer's ±X-aligned windows to lock it onto the housing
# (cable_retainer.py uses an even bar count + half-pitch offset so windows sit
# exactly on the ±X axes). Each wart bridges the spool gap and fills the
# window's wall depth, flush with the retainer's inner face; tangential / axial
# size is the window minus the standard FIT_CLR per side.
RETAIN_WART_W  = RETAIN_WINDOW_W - 2 * FIT_CLR
RETAIN_WART_H  = RETAIN_WINDOW_H - 2 * FIT_CLR
RETAIN_WART_Z0 = (RETAIN_WINDOW_Z_LO + RETAIN_WINDOW_Z_HI - RETAIN_WART_H) / 2

# Chunk: a printable base that bumps the housing's inner face out to just shy of
# the retainer's OD (closest perpendicular approach is SPINE_X_INNER − R_OUT, of
# which we keep FIT_CLR as the running clearance). Full housing width in Y and
# the window's axial extent in Z — flat, axis-aligned faces all round, so it
# adds no overhangs to the -Y→+Y print. The 3 retainer warts then grow inward
# from this chunk's inner face into the retainer windows.
RETAIN_CHUNK_X_INNER = RETAIN_R_OUT              # 61.4 — chunk inner face flush with retainer OD


def _retainer_chunk(housing_inner_x):
    """Solid chunk protruding from the housing's inner face at housing_inner_x
    (signed) into the spool gap, stopping at the retainer's OD. Z spans the
    tenon range (GUIDE_POCKET_Z_HI .. RETAIN_Z_HI) plus a STRUCT_WALL cap
    above z=RETAIN_Z_HI so the mortise's upper wedges (chunk material above
    the tenon's 45° flare) tie back into the housing instead of floating. The
    +X side also gets a STRUCT_WALL cap *below* z=GUIDE_POCKET_Z_HI to support
    the lower wedge; the -X side doesn't need it because the guide-axle boss
    + pocket frame already provide material in that region."""
    sign = 1 if housing_inner_x > 0 else -1
    x_inner = sign * RETAIN_CHUNK_X_INNER
    x_outer = housing_inner_x + sign * BOOL_OVERSHOOT
    z_lo = GUIDE_POCKET_Z_HI - (STRUCT_WALL if sign > 0 else 0.0)
    z_hi = RETAIN_Z_HI + STRUCT_WALL
    return (
        cq.Workplane("XY")
        .workplane(offset=z_lo)
        .center((x_inner + x_outer) / 2, 0)
        .box(abs(x_outer - x_inner), HOUSING_W, z_hi - z_lo,
             centered=(True, True, False))
    )


def _retainer_chunks():
    return (_retainer_chunk(SPINE_X_INNER)
            .union(_retainer_chunk(BACK_SPINE_X_INNER)))


# ── Dovetail mortise: matching cavity in the chunk that receives the tenon ──
def _offset_polygon_ccw(pts, distance):
    """Offset a CCW polygon outward by `distance` perpendicular to each edge.
    New vertices = intersections of adjacent offset edges."""
    n = len(pts)
    lines = []
    for i in range(n):
        p1, p2 = pts[i], pts[(i + 1) % n]
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        L = math.hypot(dx, dy)
        nx, ny = dy / L, -dx / L           # CCW outward normal
        lines.append(((p1[0] + distance * nx, p1[1] + distance * ny),
                      (p2[0] + distance * nx, p2[1] + distance * ny)))
    out = []
    for i in range(n):
        (x1, y1), (x2, y2) = lines[(i - 1) % n]
        (x3, y3), (x4, y4) = lines[i]
        d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(d) < 1e-9:
            out.append((x2, y2))
            continue
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / d
        out.append((x1 + t * (x2 - x1), y1 + t * (y2 - y1)))
    return out


def _retainer_mortise_plus():
    """+X-side dovetail mortise cutter — matches the tenon (cable_retainer.py:
    _retainer_tenon): wider tip on the housing side (+X), narrower neck on
    the retainer side, with the 45° / 30° flares chamfering the neck corners.
    Slot is the tenon polygon offset outward by FIT_CLR perpendicular to each
    edge. Extruded in Y from y = -HOUSING_W/2 + STRUCT_WALL (1.7 mm stop wall
    at -Y) to y = +HOUSING_W/2 + BOOL_OVERSHOOT (open at +Y for slide entry)."""
    x_neck = RETAIN_R_OUT
    x_tip = SPINE_X_INNER
    top_flare_x = STRUCT_WALL                                    # 1.7 (45°)
    bot_flare_x = STRUCT_WALL / math.tan(math.radians(30))       # 2.945 (30° from horizontal — shallow)
    z_top = RETAIN_Z_HI
    z_bot = GUIDE_POCKET_Z_HI
    z_top_flare = z_top - STRUCT_WALL
    z_bot_flare = z_bot + STRUCT_WALL
    # Tenon hexagon, CCW from neck-bottom-flare-top:
    tenon = [
        (x_neck,               z_bot_flare),
        (x_neck + bot_flare_x, z_bot),
        (x_tip,                z_bot),
        (x_tip,                z_top),
        (x_tip - top_flare_x,  z_top),
        (x_neck,               z_top_flare),
    ]
    slot = _offset_polygon_ccw(tenon, FIT_CLR)
    OS = BOOL_OVERSHOOT
    y_lo = -HOUSING_W / 2 + STRUCT_WALL                # -9.3 (face of stop wall)
    y_hi = +HOUSING_W / 2 + OS                          # +11+OS (open at +Y)
    return (cq.Workplane("XZ").workplane(offset=y_lo)
            .polyline(slot).close()
            .extrude(y_hi - y_lo))


def _retainer_mortises():
    plus = _retainer_mortise_plus()
    # Mirror in X (across YZ plane) so the -X mortise keeps the same Y range
    # as +X (stop wall at -Y on both sides). A Z-rotation would also flip Y,
    # putting the -X stop wall on +Y — asymmetric assembly direction.
    return plus.union(plus.mirror("YZ"))


def _retainer_warts():
    """3 mount warts per side (6 total). Each is a PARALLELOGRAM in its local
    X-Y plane — both top and bottom faces are 45° slants going the same way
    (spool side shifted by -chamfer_sign·L in Y_local). So:
      • The bottom (world -Y) face is self-supporting for the -Y→+Y print.
      • The top (world +Y) face is up-facing.
      • The matching parallelogram slots in the retainer's slabs (cable_retainer.py)
        let the retainer slide -Y onto the warts, flexing slightly in X."""
    pitch_deg = 360.0 / RETAIN_N_BARS                          # 7.2° for N=50
    W = RETAIN_WART_W
    H = RETAIN_WART_H
    out = None
    for base_deg in (0.0, 180.0):                              # +X side, then -X side
        # Local face that rotates to world -Y: -Y_local on +X side (base=0);
        # +Y_local on -X side (base=180° flips it).
        chamfer_sign = -1 if base_deg == 0.0 else +1
        for off_deg in (-pitch_deg, 0.0, pitch_deg):
            theta_deg = base_deg + off_deg
            r_outer = (RETAIN_CHUNK_X_INNER
                       / abs(math.cos(math.radians(theta_deg)))
                       + BOOL_OVERSHOOT)
            L = r_outer - RETAIN_R_IN                          # wart length along its radial
            # Both spool corners shifted by -chamfer_sign·L in Y_local → parallelogram.
            y_shift = -chamfer_sign * L
            pts = [
                (RETAIN_R_IN, -chamfer_sign * W / 2 + y_shift),  # spool, un-chamfered-aligned
                (RETAIN_R_IN,  chamfer_sign * W / 2 + y_shift),  # spool, chamfered-aligned
                (r_outer,      chamfer_sign * W / 2),            # chunk, chamfered corner
                (r_outer,     -chamfer_sign * W / 2),            # chunk, un-chamfered
            ]
            wart = (cq.Workplane("XY")
                    .workplane(offset=RETAIN_WART_Z0)
                    .polyline(pts).close()
                    .extrude(H)
                    .rotate((0, 0, 0), (0, 0, 1), theta_deg))
            out = wart if out is None else out.union(wart)
    return out


def _axle_round_hole():
    return cyl(AXLE_PRINT_D + 2 * HOUSING_HOLE_CLR, PLATE_T + 1,
               z=PANCAKE_PLATE_Z_IN - 0.5)


def _build_lever_mounts(h):
    """Add both levers' pivot-insert/clearance cuts and their housing stop
    pins (+ spring-leg holes). Ratchet on +Y, brake on -Y."""
    # Spring-positioning rings: a HOUSING_BOSS_EXT-tall boss around each pivot
    # on the lever-facing Y face, unioned BEFORE the pivot bores so the screw-
    # clearance + insert pilot carry through them. The coil bears on the ring.
    yf = HOUSING_W / 2
    h = h.union(pivot_boss_sector(RATCHET_PIVOT_X, RATCHET_PIVOT_Z,
                                  yf - 0.5, yf + HOUSING_BOSS_EXT))
    h = h.union(pivot_boss_sector(BRAKE_PIVOT_X, BRAKE_PIVOT_Z,
                                  -yf - HOUSING_BOSS_EXT, -yf + 0.5))
    # ── Ratchet (+Y side): screw enters from -Y, insert in +Y face ──
    h = (h.cut(_pivot_clearance(RATCHET_PIVOT_X, RATCHET_PIVOT_Z))
          .cut(_pivot_insert_pilot(RATCHET_PIVOT_X, RATCHET_PIVOT_Z,
                                   insert_face_sign=+1, boss_ext=HOUSING_BOSS_EXT)))
    # ── Brake (-Y side): screw enters from +Y, insert in -Y face ──
    h = (h.cut(_pivot_clearance(BRAKE_PIVOT_X, BRAKE_PIVOT_Z))
          .cut(_pivot_insert_pilot(BRAKE_PIVOT_X, BRAKE_PIVOT_Z,
                                   insert_face_sign=-1, boss_ext=HOUSING_BOSS_EXT)))

    # Housing stop pins project into the gap toward the lever (+Y for ratchet,
    # -Y for brake). The SPRING pin is a short stub from its near face into the
    # gap (it's fully over the block in X, so well-rooted). The REST pin sits
    # mostly -X of the block's inner face, so it would be only lightly rooted —
    # instead it runs the FULL housing width (flush with the far face, across
    # to the near face) and then STOP_PIN_H out, anchoring it along the whole
    # block (its own "boss") and giving the lever-side stub to catch the lever
    # pin.
    y_face = HOUSING_W / 2
    # Ratchet (+Y side).
    for alpha, is_spring in ((RATCHET_REST_PIN_ALPHA, False),
                             (RATCHET_SPRING_PIN_ALPHA, True)):
        y_from, y_to = ((y_face - 0.5, y_face + STOP_PIN_H) if is_spring
                        else (-y_face, y_face + STOP_PIN_H))
        h = h.union(stop_pin_solid(RATCHET_PIVOT_X, RATCHET_PIVOT_Z, alpha, y_from, y_to))
        if is_spring:
            leg_a = alpha + (-1) * SPRING_LEG_PIN_OFFSET_DEG
            h = h.cut(stop_pin_hole(RATCHET_PIVOT_X, RATCHET_PIVOT_Z, alpha,
                                    hole_y=y_face + STOP_PIN_H - STOP_PIN_HOLE_D,
                                    hole_dir_alpha_deg=spring_leg_hole_dir_alpha_deg(leg_a)))
    # Brake (-Y side).
    for alpha, is_spring in ((BRAKE_REST_PIN_ALPHA, False),
                             (BRAKE_SPRING_PIN_ALPHA, True)):
        y_from, y_to = ((-y_face - STOP_PIN_H, -y_face + 0.5) if is_spring
                        else (-y_face - STOP_PIN_H, y_face))
        h = h.union(stop_pin_solid(BRAKE_PIVOT_X, BRAKE_PIVOT_Z, alpha, y_from, y_to))
        if is_spring:
            leg_a = alpha + (-1) * SPRING_LEG_PIN_OFFSET_DEG
            h = h.cut(stop_pin_hole(BRAKE_PIVOT_X, BRAKE_PIVOT_Z, alpha,
                                    hole_y=-y_face - STOP_PIN_H + STOP_PIN_HOLE_D,
                                    hole_dir_alpha_deg=spring_leg_hole_dir_alpha_deg(leg_a)))
    return h


def _build_housing_skeleton():
    h = (_front_block()
         .union(_back_block())
         .union(_guide_box_frame())
         .union(_guide_axle_boss())
         .union(_pancake_plate())
         .union(_front_thicken())
         .union(_retainer_chunks())
         .cut(_retainer_mortises())
         .cut(_axle_round_hole())
         .cut(_guide_wheel_pocket_cut())
         .union(_guide_retainer_rings())
         .cut(_guide_axle_screw_hole()))
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


# ── 45° glue-on chunk for the -Y brake mount ────────────────────────────────
# The brake lever's mounting features (pivot insert, spring-positioning ring,
# both stop pins) sit on the -Y face and protrude in -Y, so they'd stop the
# housing sitting flat on its -Y face for a support-free print. Instead, slice
# the -Y/lower corner off along ONE 45° plane (z + y = C, in the Y-Z section)
# and print that chunk SEPARATELY, resting on its cut face. Both pieces then
# print without supports — each cut face is a 45° self-supporting surface —
# and the chunk glues on afterward. (The full-width rest pins are split by the
# cut: each keeps its functional stub + anchoring on the correct side and
# rejoins at the glue line; the brake pivot screw also clamps the joint.)
BRAKE_CHUNK_CUT_C = -3.0          # cut plane z + y = C; chunk is the z+y <= C side
                                  # (raised +1 mm to put more material in the chunk)
_BRAKE_CHUNK_X_LO = 60.0          # cutter X span (covers the front block + -X rest pin)
_BRAKE_CHUNK_X_HI = 80.0


def _brake_chunk_cutter():
    """Half-space z + y <= BRAKE_CHUNK_CUT_C (45° plane in Y-Z), bounded in X
    to the brake-mount region. housing ∩ it = the chunk; housing − it = main."""
    C = BRAKE_CHUNK_CUT_C
    big = 80.0
    return (cq.Workplane("YZ").workplane(offset=_BRAKE_CHUNK_X_LO)
            .polyline([(-big, C + big), (-big, -big), (C + big, -big)]).close()
            .extrude(_BRAKE_CHUNK_X_HI - _BRAKE_CHUNK_X_LO))


# ── Registration V-rails on the glue joint ──────────────────────────────────
# Two thin 45° V-rails run across the joint face (along its +Y..-Y direction)
# to lock the X position when gluing the chunk on. The HOUSING gets the V
# ridge, the chunk gets a matching (oversized) groove. They flank the central
# pivot-screw hole — one on the -X side (over the embedded rest pin), one on
# the +X side (in the gap past the pivot screw) — so neither hits the bore.
REG_RAIL_SIDE         = 2.0        # V flank length (mm)
REG_RAIL_GROOVE_EXTRA = 0.3        # groove V is this much larger (glue clearance)
# Rails centred in the two solid spans of the joint face, flanking the pivot
# bore void at x∈[66.0, 68.4]: -X span [61.43, 66.0], +X span [68.4, 74.73].
# A 2 mm V's groove (~3.25 mm wide) fits each with margin.
REG_RAIL_X            = (63.7, 71.5)   # -X / +X rail positions
_REG_RAIL_L           = 40.0       # raw length; trimmed to the joint by ∩ housing
_REG_RAIL_OVERLAP     = 0.3        # base slab buried in the main side for a clean union
REG_RAIL_END_GAP      = 0.4        # material left between the cut and the close
                                   # (-Y) face at the rail's +Z end
_REG_RAIL_YC          = -5.0
_REG_RAIL_ZC          = BRAKE_CHUNK_CUT_C - _REG_RAIL_YC   # (yc, zc) lies on z+y=C
_REG_GROOVE_H         = (REG_RAIL_SIDE + REG_RAIL_GROOVE_EXTRA) / math.sqrt(2)
# Clip the +Z (high-z, -Y) end so the cut bevel faces UP when the chunk prints
# on its -Z face (support-free), and so the DEEPEST cut — the groove V apex,
# h_g·cos45 toward -Y — clears the -Y close face by REG_RAIL_END_GAP. The -Z
# (low-z, +Y) end is left unclipped: the rail runs fully down to the block
# bottom, its cut opening on the -Z face (the build plate). Local Y maps to
# world y via y = yc + localY·cos45, so the world apex-Y target maps to:
_REG_RAIL_LY_LO       = ((-HOUSING_W / 2 + REG_RAIL_END_GAP - _REG_RAIL_YC) * math.sqrt(2)
                         + _REG_GROOVE_H)


def _reg_rail(x_rail, side):
    """One V-rail prism centred on the joint plane (z + y = C) at X=x_rail,
    running along the joint's in-plane Y direction. House-shaped section: a V
    of `side`-long 45° flanks (apex toward the chunk) on a thin base slab that
    buries into the main side for a clean boolean. The +Z (high-z) end is
    clipped (in the rail's local frame, so the cut is a clean 45° perpendicular
    to the main cut face, facing up for a support-free print) leaving
    REG_RAIL_END_GAP to the -Y close face; the -Z end runs full to the plate."""
    h = side / math.sqrt(2)                     # half-base = V depth (45° flanks)
    yc = _REG_RAIL_YC
    zc = _REG_RAIL_ZC
    clip = (cq.Workplane("XY").box(1000, 1000, 1000, centered=True)
            .translate((0, _REG_RAIL_LY_LO - 500, 0)))   # local -Y before the clip
    return (cq.Workplane("XZ")
            .polyline([(-h, _REG_RAIL_OVERLAP), (h, _REG_RAIL_OVERLAP),
                       (h, 0.0), (0.0, -h), (-h, 0.0)]).close()
            .extrude(_REG_RAIL_L / 2, both=True)
            .cut(clip)
            .rotate((0, 0, 0), (1, 0, 0), -45)
            .translate((x_rail, yc, zc)))


def _reg_rails(side):
    return _reg_rail(REG_RAIL_X[0], side).union(_reg_rail(REG_RAIL_X[1], side))


_housing_full = _build_housing()
_brake_cutter = _brake_chunk_cutter()
_main  = _housing_full.cut(_brake_cutter)
_chunk = _housing_full.intersect(_brake_cutter)
# Ridge on the housing (bounded to the solid), groove in the chunk (oversized).
housing = _main.union(_reg_rails(REG_RAIL_SIDE).intersect(_housing_full))
brake_pin_chunk = _chunk.cut(_reg_rails(REG_RAIL_SIDE + REG_RAIL_GROOVE_EXTRA))
