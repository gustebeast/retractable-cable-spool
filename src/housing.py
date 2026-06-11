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
    FIT_CLR, CROSS_PRINT_CLR, CHUNK_RAIL_CLR,
    M2_HEAD_RECESS_D, M2_HEAD_RECESS_H,
    M2_INSERT_DEPTH, M2_INSERT_PILOT_D, M2_SHAFT_CLR_D,
    PANCAKE_CROSS_PIN_Z, PLATE_T,
    HOUSING_GAP_PANCAKE,
    SPOOL_H, STRUCT_WALL,
)
from .helpers import cyl, cone_solid
from .spool import (RIM_OD, RIM_H, RATCHET_BAND_H, BRAKE_RIM_Z_LO,
                    SPOOL_HORIZONTAL_CLEARANCE,
                    RATCHET_CHUNK_Z_MAX, RATCHET_CHUNK_CLEARANCE)
from .cable_retainer import (
    RETAIN_Z_LO, RETAIN_Z_HI,
    RETAIN_R_IN, RETAIN_R_OUT, RETAIN_N_BARS,
    RETAIN_WINDOW_W, RETAIN_WINDOW_H,
    RETAIN_WINDOW_Z_LO, RETAIN_WINDOW_Z_HI,
    RETAIN_TENON_Z_BOT, RETAIN_TENON_Z_TOP, RETAIN_TENON_DEPTH,
    retainer_tenon_polygon,
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

# Print warp pre-compensation: pre-rotate the rest pose back by this so the
# printed lever settles true (0 for the radial-lever generation; see the
# longer note where it's used).
LEVER_REST_PRECOMP_DEG = 0.0

# Lever pivot X is derived so the FRONT_THICKEN feature's inner (spool-
# facing) face — which sits at the REST stop pin's -X extent in order to
# embed the pin in solid wall — lands SPOOL_HORIZONTAL_CLEARANCE beyond the
# rim's OD. The rest pin sits at α = STOP_LEVER + _STOP_CONTACT_SEP (+ rest-
# precomp). Its -X edge is at PIVOT_X + STOP_PIN_R·cos(α) − STOP_PIN_D/2.
# Setting that = RIM_OD/2 + SPOOL_HORIZONTAL_CLEARANCE and solving:
_REST_PIN_ALPHA_DEG = (STOP_LEVER_PIN_ALPHA_DEG + _STOP_CONTACT_SEP_DEG
                      + LEVER_REST_PRECOMP_DEG)
PIVOT_X = (RIM_OD / 2 + SPOOL_HORIZONTAL_CLEARANCE
           - STOP_PIN_R * math.cos(math.radians(_REST_PIN_ALPHA_DEG))
           + STOP_PIN_D / 2)                            # ≈ 67.17
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
# Ratchet pivot Z is driven by the 6 mm guide wheel: the wheel spans
# BRAKE_RIM_Z_LO (7.5) .. 13.5, the pocket ceiling sits GUIDE_WHEEL_FACE_CLR
# above the wheel (14.5), and the pivot's M2 bore needs 0.4 mm of solid
# housing below its -Z extent:
#   RATCHET_PIVOT_Z = 7.5 + 6.0 + 1.0 + 0.4 + M2_SHAFT_CLR_D/2 = 16.1
# (GUIDE_POCKET_Z_HI_PLUS below re-derives the wheel from this pivot, so
# the two stay consistent. Mirror RATCHET_CHUNK_Z_MAX in spool.py if this
# changes.)
RATCHET_PIVOT_Z = 16.1
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
# Width (its Z extent) spans the brake rim's straight section: the wheel's
# bottom aligns with BRAKE_RIM_Z_LO (top of the rim's cone — wheel rolls
# only on the cylindrical brake band, never on the cone or teeth) and its
# top sits GUIDE_TO_RETAINER_CLR + GUIDE_WHEEL_FACE_CLR below the retainer
# ring. The wheel sits in a +X-open pocket cut into the back block, with a
# STRUCT_WALL-thick wall behind it (-X) and below it (-Z).
GUIDE_WHEEL_OD     = 14.0
GUIDE_WHEEL_CX     = -(RIM_OD / 2 + GUIDE_WHEEL_OD / 2)   # -64.2 — center; surface tangent to rim
GUIDE_WHEEL_BORE_D = 2.6                                  # spinning clearance for a future M2 axle

# Pocket: open toward +X (so the wheel reaches the rim), walled STRUCT_WALL
# thick on -X (behind) and -Z (below). The back leg's bottom is the underside
# of that 1.7 mm floor; everything between the floor and the wheel is air.
GUIDE_POCKET_WALL    = STRUCT_WALL                            # 1.7 — back + bottom walls
# Target gap between the wheel's -X extent and the pocket back face.
# Drives the pocket back position; the back-spine wall is then thickened
# OUTWARD in -X (via _back_wall_thicken) so it keeps its STRUCT_WALL thickness
# behind the new pocket back.
GUIDE_WHEEL_BACK_CLR = 1.0
GUIDE_POCKET_X_BACK  = (GUIDE_WHEEL_CX - GUIDE_WHEEL_OD / 2
                        - GUIDE_WHEEL_BACK_CLR)               # -73.9 — pocket back face
# Boss thickness — extends BACK_SPINE_X_OUTER outward in -X enough to keep
# GUIDE_POCKET_WALL of wall between the new pocket back and the outer face.
# (= how much the pocket back shifted from the natural BACK_SPINE_X_OUTER +
# GUIDE_POCKET_WALL position.)
GUIDE_BACK_WALL_EXTRA_T = max(0.0,
    (BACK_SPINE_X_OUTER + GUIDE_POCKET_WALL) - GUIDE_POCKET_X_BACK)
GUIDE_WHEEL_FACE_CLR = 1.0                                    # housing clearance off the wheel faces
GUIDE_WHEEL_AXIAL_CLR = 0.5                                   # extra Z slack per face on the wheel hub
                                                              # (hub_h shrinks by 2× this so the wheel
                                                              # doesn't bind axially against the pocket)
# Wheel's Z range keyed off the brake rim's straight section:
#   - TOP:  pocket top sits GUIDE_TO_RETAINER_CLR (= 0.8) below the retainer's
#           bottom ring, leaving room for the mortise/tenon material above.
#   - BOT:  wheel's bottom aligns with the brake rim's straight section bottom
#           (BRAKE_RIM_Z_LO = 4.7) — the wheel rolls on the cylindrical brake
#           band only, never on the cone or the ratchet teeth.
# GUIDE_TO_RETAINER_CLR now = STRUCT_WALL so that pocket top + STRUCT_WALL
# (the chunk top wall) lands EXACTLY on the retainer's -Z face. This makes
# CHUNK_Z_TOP = RETAIN_Z_LO: the brake/ratchet chunks no longer overshoot
# the retainer and stop being clipped by the retainer's slab → chunks are
# now fully independent from the retainer.
GUIDE_TO_RETAINER_CLR = STRUCT_WALL
# Structural top: where the housing material above the wheel rises to —
# tied to the retainer's -Z face. Used by features ABOVE the wheel cavity
# (guide-axle boss, back chamfer extrusion, retainer thickener, chunk top).
GUIDE_POCKET_Z_HI    = RETAIN_Z_LO - GUIDE_TO_RETAINER_CLR        # 13.266
# Wheel-cavity ceiling: where the actual wheel pocket cut stops. Driven by
# the +X side's ratchet-pivot M2 bore at RATCHET_PIVOT_Z=10 (Ø 2.4), which
# sweeps z = 8.8..11.2 — the cavity ceiling has to sit at least 0.4 mm
# BELOW the bore's -Z extent to leave that much solid housing material
# between the bore and the cavity. The wheel is sized to fit this tighter
# ceiling, and BOTH the -X and +X pocket cuts use it so the wheel rides
# snug on both sides (no vertical rattle on -X).
GUIDE_POCKET_Z_HI_PLUS = (RATCHET_PIVOT_Z - M2_SHAFT_CLR_D / 2) - 0.4  # 8.4
GUIDE_WHEEL_Z_HI     = GUIDE_POCKET_Z_HI_PLUS - GUIDE_WHEEL_FACE_CLR   # 7.4
GUIDE_WHEEL_Z_LO     = BRAKE_RIM_Z_LO                            # 4.7 — flush with brake-rim bottom
GUIDE_WHEEL_W        = GUIDE_WHEEL_Z_HI - GUIDE_WHEEL_Z_LO       # 2.7 — axial (Z) width
GUIDE_POCKET_Z_LO    = GUIDE_WHEEL_Z_LO - GUIDE_WHEEL_FACE_CLR   # 3.7 — pocket floor
GUIDE_POCKET_Y_HALF  = HOUSING_W / 2 - GUIDE_POCKET_WALL    # 9.3 — leaves a 1.7 mm wall on each ±Y side
# +X (lever-side) pocket Y-half: tighter — wheel OD + 2 mm total (1 mm of
# wheel-to-wall clearance per side). Lets the wider ±Y walls absorb the
# +X lever-pivot insert/screw without the pocket cutting into them.
GUIDE_POCKET_Y_HALF_PLUS = GUIDE_WHEEL_OD / 2 + 1.0          # 8.0

# 45° corner chamfer at the pocket's back / +Y corner: adds material to the
# corner so the +Y "ceiling" transitions to the -X wall at 45° instead of a
# sharp 90° interior corner. Sized so the chamfer face stays
# GUIDE_CHAMFER_WHEEL_CLR clear of the wheel's outer surface.
#   Chamfer line in XY: from (Xb, Yh − L) on the -X wall to (Xb + L, Yh) on
#   the +Y wall. Slope +1, equation x − y = Xb − Yh + L. Solving for tangent
#   to circle of radius (wheel_r + clr) centered at (Wx, 0):
#       L = Wx − Xb + Yh − (wheel_r + clr)·√2
GUIDE_CHAMFER_WHEEL_CLR = 1.0
GUIDE_CHAMFER_LEG = (
    GUIDE_WHEEL_CX - GUIDE_POCKET_X_BACK + GUIDE_POCKET_Y_HALF
    - (GUIDE_WHEEL_OD / 2 + GUIDE_CHAMFER_WHEEL_CLR) * math.sqrt(2)
)
assert GUIDE_CHAMFER_LEG > 0, (
    f"guide-pocket chamfer leg is non-positive ({GUIDE_CHAMFER_LEG:.2f} mm) — "
    "wheel sits too close to the corner; shrink GUIDE_CHAMFER_WHEEL_CLR.")
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
SPRING_BODY_LEN = 2.5     # axial (Y) extent of the coil
SPRING_SPACER_T = 0.4     # axial (Y) extent of each spring-positioning boss
                          # (one on the housing's lever-facing face, one on
                          # the lever's housing-facing face). Was 1.0; shaved
                          # to 0.4 so the lever sits closer to the housing,
                          # giving the M2 pivot screw more thread engagement
                          # on each side of the gap.
LEVER_RIM_H     = SPRING_BODY_LEN + 2 * SPRING_SPACER_T  # 3.3 — Y gap between
                          # each lever's inner face and the block's outer Y
                          # face. The torsion-spring coil lives in this gap
                          # around the pivot screw.
LEVER_PIVOT_BOSS_OD = 6.0 # reinforcement boss OD coaxial with each pivot
# Spring positioning ring on the housing's lever-facing Y face. Paired 1:1
# with the lever's own pivot-boss extension on the other side of the spring.
HOUSING_BOSS_EXT = SPRING_SPACER_T   # 0.4

LEVER_SCREW_CLR_D      = GUIDE_AXLE_SHAFT_D   # 2.2 — same tight thread-fit as the guide-wheel axle holes (M2 self-taps the housing side; the lever's pivot_hole is the 2.4 mm clearance bore)
# No heat-set insert anymore — the M2 cap-screw self-taps the housing's
# 2.2 mm bore directly. LEVER_INSERT_PILOT_D matches LEVER_SCREW_CLR_D so
# the "pilot" pocket is effectively a continuation of the bore (no step,
# no cone). The constant + the LEVER_INSERT_DEPTH / CHAMFER_H values are
# kept (rather than ripping out _pivot_insert_pilot) so the call sites
# don't need to change.
LEVER_INSERT_PILOT_D   = LEVER_SCREW_CLR_D
LEVER_INSERT_DEPTH     = M2_INSERT_DEPTH
LEVER_INSERT_CHAMFER_H = (LEVER_INSERT_PILOT_D - LEVER_SCREW_CLR_D) / 2  # 0
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
PIN_TIP_CLEARANCE = 1.0   # Y gap left between each pin's tip and the opposing
                          # body face (lever or housing). Held constant when
                          # the lever-housing gap changes — STOP_PIN_H is
                          # derived to keep this clearance fixed.
STOP_PIN_H      = LEVER_RIM_H - PIN_TIP_CLEARANCE   # 2.3 (was 3.5; pin's
                          # tip-side stays put, body side shortens to track
                          # the smaller LEVER_RIM_H).
SPRING_WIRE_R   = 0.25
# (STOP_PIN_D, STOP_PIN_R, STOP_LEVER_PIN_ALPHA_DEG, LEVER_TRAVEL_DEG, and
# _STOP_CONTACT_SEP_DEG are defined early in the envelope section so the
# block's +X face can be placed to clear the pins.)

# Both levers rotate the same real direction (α decreasing) by LEVER_TRAVEL_DEG,
# so they start and end at the same angles (asserted: A_ANGLES_MATCH).
RATCHET_OUTER_TRAVEL_DEG = LEVER_TRAVEL_DEG   # disengage travel
BRAKE_INNER_TRAVEL_DEG   = LEVER_TRAVEL_DEG   # engage travel

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
LEVER_SCREW_BORE_LEN = 21.0   # 20 mm M2 screw + 1 mm clearance past the tip


def _pivot_clearance(x, z, head_sign=None):
    """M2 shaft clearance bore — LEVER_SCREW_BORE_LEN mm along Y, starting
    at the screw head position (outer face of the lever-pivot spacer) and
    extending toward the housing's interior. head_sign = +1 for ratchet
    (head on +Y, bore extends -Y); −1 for brake (head on -Y, bore extends
    +Y). The head end sits OUTSIDE the housing at world Y ≈ ±28 (past the
    lever + spacer); only the ~4 mm of bore that lands inside the housing
    actually removes material. Previous version cut the full 24 mm housing
    width — punching a useless exit hole through the far +Y/-Y face."""
    from .levers import LEVER_INNER_Y, LEVER_T
    if head_sign is None:
        # Legacy callers: cut both sides like the original full-width bore.
        return (
            cq.Workplane("XY").circle(LEVER_SCREW_CLR_D / 2)
            .extrude(HOUSING_W + 2)
            .rotate((0, 0, 0), (1, 0, 0), -90)
            .translate((x, -HOUSING_W / 2 - 1, z))
        )
    # Head's underside (= shaft start) sits LEVER_HEAD_RECESS_H below the
    # lever's outer face — the head is recessed INTO the lever, not riding
    # on top of a spacer washer.
    from .levers import LEVER_HEAD_RECESS_H
    y_head = head_sign * (LEVER_INNER_Y + LEVER_T - LEVER_HEAD_RECESS_H)
    return (
        cq.Workplane("XY").circle(LEVER_SCREW_CLR_D / 2)
        .extrude(LEVER_SCREW_BORE_LEN)
        .rotate((0, 0, 0), (1, 0, 0), head_sign * 90)
        .translate((x, y_head, z))
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
    # Pilot→clearance step cone is only needed when the pilot is wider
    # than the clearance bore (heat-set insert design). With pilot D ==
    # clearance D the cone is degenerate (zero radial step), so skip it.
    if insert_face_sign < 0 and LEVER_INSERT_CHAMFER_H > 0:
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
    # Cone lead-in only when the pilot is wider than the clearance bore.
    # With pilot D == clearance D (current setup, no heat-set insert) the
    # cone is degenerate, so skip it.
    if insert_face_sign < 0 and LEVER_INSERT_CHAMFER_H > 0:
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
# Height is sized so the thickening's TOP z reaches the ratchet_pin_chunk's
# top PLUS the chunk clearance — so the chunk's full Z range is backed by the
# thickened wall (embedding both brake and ratchet stop pins in solid wall)
# with the same breathing room above the chunk that BRAKE_BAND_H uses.
FRONT_THICKEN_H = (RATCHET_CHUNK_Z_MAX + RATCHET_CHUNK_CLEARANCE
                   - FRONT_Z_BOT)                       # ≈ 19.87


def _retainer_thicken():
    """Thicken both ±X inner faces of the housing across the retainer's Z range,
    bumping inward to x = ±RETAIN_R_OUT (= 63.1, the retainer's flat OD where
    the tenon would emerge). Z extends from RETAIN_Z_LO up through
    RETAIN_Z_HI + STRUCT_WALL — the extra 1.7 mm cap above the retainer
    gives material for the (future) mortise's top wall."""
    z_lo = RETAIN_Z_LO
    z_hi = RETAIN_Z_HI + STRUCT_WALL
    h    = z_hi - z_lo

    def _side(housing_inner_x):
        sign    = 1 if housing_inner_x > 0 else -1
        x_inner = sign * RETAIN_R_OUT
        x_outer = housing_inner_x + sign * BOOL_OVERSHOOT
        return (cq.Workplane("XY").workplane(offset=z_lo)
                .center((x_inner + x_outer) / 2, 0)
                .box(abs(x_outer - x_inner), HOUSING_W, h,
                     centered=(True, True, False)))

    return _side(SPINE_X_INNER).union(_side(BACK_SPINE_X_INNER))


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


def _guide_pocket_back_y_chamfer(y_half=GUIDE_POCKET_Y_HALF):
    """45° corner chamfer in XY at the guide-wheel pocket's back-/+Y corner.
    A triangular prism added back AFTER the pocket cut, extruded across the
    pocket's Z range. The chamfer face is tangent to a circle of radius
    (wheel_r + GUIDE_CHAMFER_WHEEL_CLR) at the wheel axis, so the wheel
    keeps GUIDE_CHAMFER_WHEEL_CLR clearance from the chamfer surface."""
    # Chamfer leg recomputed from y_half so the chamfer fits a tighter
    # pocket (+X side uses GUIDE_POCKET_Y_HALF_PLUS = 8.0).
    L = (GUIDE_WHEEL_CX - GUIDE_POCKET_X_BACK + y_half
         - (GUIDE_WHEEL_OD / 2 + GUIDE_CHAMFER_WHEEL_CLR) * math.sqrt(2))
    z_lo = GUIDE_POCKET_Z_LO
    z_hi = GUIDE_POCKET_Z_HI
    return (
        cq.Workplane("XY").workplane(offset=z_lo)
        .moveTo(GUIDE_POCKET_X_BACK,        y_half)
        .lineTo(GUIDE_POCKET_X_BACK,        y_half - L)
        .lineTo(GUIDE_POCKET_X_BACK + L,    y_half)
        .close()
        .extrude(z_hi - z_lo)
    )


def _back_wall_thicken():
    """Adds GUIDE_BACK_WALL_EXTRA_T mm of material in -X to the back-spine
    wall, only across the guide-wheel pocket's Z range. The wall behind the
    wheel grows from STRUCT_WALL (1.7) to STRUCT_WALL + extra (= 2.7),
    extending OUTWARD into -X — the wheel and pocket-back-face stay put."""
    z_lo = BACK_Z_BOT                                                 # 2.0
    # Stop at the wheel cavity's top + a STRUCT_WALL roof above it (rather
    # than running all the way up to the retainer). The thickening exists
    # only to back up the wheel cavity, so it has nothing to do above that.
    z_hi = GUIDE_POCKET_Z_HI_PLUS + GUIDE_POCKET_WALL                  # 10.0
    return (
        cq.Workplane("XY").workplane(offset=z_lo)
        .center(BACK_SPINE_X_OUTER - GUIDE_BACK_WALL_EXTRA_T / 2, 0)
        .box(GUIDE_BACK_WALL_EXTRA_T, HOUSING_W, z_hi - z_lo,
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


def _guide_wheel_pocket_cut(y_half=GUIDE_POCKET_Y_HALF, z_hi=GUIDE_POCKET_Z_HI_PLUS):
    """The wheel cavity carved into the back block: a box open toward +X,
    leaving GUIDE_POCKET_WALL behind (-X) and below (-Z) and the ±Y walls.
    Spans +X clear past the box's front face (into the spool gap) so the wheel
    protrudes to the rim; the cut beyond the box is just air. Bounded in Y/Z to
    the wheel envelope, so the floor, ceiling, and ±Y walls keep their +X
    extension (the band that wraps the axle hole). y_half lets the +X side
    use a tighter pocket (GUIDE_POCKET_Y_HALF_PLUS = 8.0) than the -X side.
    z_hi defaults to GUIDE_POCKET_Z_HI_PLUS so the cavity is snug to the
    wheel on both sides (no vertical rattle); structural features ABOVE the
    cavity (boss, chamfer, etc.) still use GUIDE_POCKET_Z_HI."""
    x_lo = GUIDE_POCKET_X_BACK
    x_hi = GUIDE_WHEEL_CX + GUIDE_WHEEL_OD / 2 + BOOL_OVERSHOOT   # open past the wheel's +X extent
    return (
        cq.Workplane("XY").workplane(offset=GUIDE_POCKET_Z_LO)
        .center((x_lo + x_hi) / 2, 0)
        .box(x_hi - x_lo, 2 * y_half, z_hi - GUIDE_POCKET_Z_LO,
             centered=(True, True, False))
    )


GUIDE_WHEEL_SPOKE_COUNT = 3


def _guide_wheel():
    """Lightweight spoked wheel: outer contact ring + inner hub ring joined
    by GUIDE_WHEEL_SPOKE_COUNT radial spokes.

    The hub extends past the +Z face only (replacing the housing-side ring
    on that side). The -Z hub extension was REMOVED — the housing now
    carries a matching puck on the pocket floor instead. Reason: the -Z
    hub extension was the wheel's print-bed face, and the outer ring's
    -Z face sitting above it forced an FDM overhang that needed supports.
    With the wheel's -Z face now flat (outer ring + hub bottom both at
    GUIDE_WHEEL_Z_LO), the wheel prints support-free on its -Z face."""
    # Outer contact ring: full GUIDE_WHEEL_W axial extent.
    outer_id = GUIDE_WHEEL_OD - 2 * STRUCT_WALL
    outer = (cyl(GUIDE_WHEEL_OD, GUIDE_WHEEL_W, z=GUIDE_WHEEL_Z_LO)
             .cut(cyl(outer_id, GUIDE_WHEEL_W + 2 * BOOL_OVERSHOOT,
                      z=GUIDE_WHEEL_Z_LO - BOOL_OVERSHOOT)))

    # Hub ring: -Z face flush with the outer ring's -Z face (flat print
    # base, no overhang); +Z face extends (GUIDE_WHEEL_FACE_CLR −
    # GUIDE_WHEEL_AXIAL_CLR) past the outer ring's +Z face so it nominally
    # rides face-to-face against the pocket ceiling (with AXIAL_CLR of
    # slack). The matching -Z bearing is the housing-side spacer puck.
    hub_od = GUIDE_WHEEL_BORE_D + 2 * STRUCT_WALL
    hub_z_lo = GUIDE_WHEEL_Z_LO
    hub_h    = GUIDE_WHEEL_W + (GUIDE_WHEEL_FACE_CLR - GUIDE_WHEEL_AXIAL_CLR)
    hub = (cyl(hub_od, hub_h, z=hub_z_lo)
           .cut(cyl(GUIDE_WHEEL_BORE_D, hub_h + 2 * BOOL_OVERSHOOT,
                    z=hub_z_lo - BOOL_OVERSHOOT)))

    # Spokes — full GUIDE_WHEEL_W axial extent, STRUCT_WALL thick (tangential),
    # span from inside the outer ring's ID to inside the hub's OD (small
    # overlap on each end for a clean boolean union).
    r_in  = hub_od / 2 - BOOL_OVERSHOOT
    r_out = outer_id / 2 + BOOL_OVERSHOOT
    wheel = outer.union(hub)
    for i in range(GUIDE_WHEEL_SPOKE_COUNT):
        ang = i * 360.0 / GUIDE_WHEEL_SPOKE_COUNT
        spoke = (cq.Workplane("XY").workplane(offset=GUIDE_WHEEL_Z_LO)
                 .center((r_in + r_out) / 2, 0)
                 .box(r_out - r_in, STRUCT_WALL, GUIDE_WHEEL_W,
                      centered=(True, True, False))
                 .rotate((0, 0, 0), (0, 0, 1), ang))
        wheel = wheel.union(spoke)
    return wheel.translate((GUIDE_WHEEL_CX, 0, 0))


guide_wheel = _guide_wheel()
# Mirrored copy of the same wheel for the new +X (lever-leg) guide pocket.
guide_wheel_plus = _guide_wheel().mirror("YZ")


# ── Lever pivot ─────────────────────────────────────────────────────────────
# No external spacer/riser — the M2 cap-screw head seats in a Ø4.1 ×
# LEVER_HEAD_RECESS_H counterbore drilled into the lever's outer Y face
# (see levers.py). The shaft after the recess is sized so the screw is
# split evenly between the lever and the housing.


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


GUIDE_AXLE_HOLE_LEN = GUIDE_AXLE_SCREW_LEN + 1.0   # 21 mm: screw shank + 1 mm tip clr


def _guide_axle_spacer():
    """Thin puck on the pocket floor under the wheel's hub. Replaces the
    -Z hub extension that used to live on the wheel (which forced an
    overhang on the wheel's print-bed face). Same OD as the wheel's hub;
    its center bore is cut later by the existing _guide_axle_screw_hole
    pass. Height = GUIDE_WHEEL_FACE_CLR − GUIDE_WHEEL_AXIAL_CLR so the
    wheel still rides with AXIAL_CLR of slack against the puck top.

    On the +X side this puck sits inside the brake/ratchet chunks'
    footprint — the chunk-split cutters slice it along the diagonal so
    each chunk carries its share."""
    h  = GUIDE_WHEEL_FACE_CLR - GUIDE_WHEEL_AXIAL_CLR
    od = GUIDE_WHEEL_BORE_D + 2 * STRUCT_WALL
    # Pre-drilled axle bore (same Ø as the housing's screw bore) — the
    # spacer is unioned AFTER the housing's screw-hole cut, so it has to
    # carry its own bore.
    puck = (cyl(od, h, z=GUIDE_POCKET_Z_LO)
            .cut(cyl(GUIDE_AXLE_SHAFT_D, h + 2 * BOOL_OVERSHOOT,
                     z=GUIDE_POCKET_Z_LO - BOOL_OVERSHOOT)))
    return puck.translate((GUIDE_WHEEL_CX, 0, 0))


def _guide_axle_screw_hole(z_head=GUIDE_AXLE_HEAD_Z):
    """Ø GUIDE_AXLE_SHAFT_D tight-fit bore along Z for the M2 axle screw:
    GUIDE_AXLE_HOLE_LEN (= screw length + 1 mm tip clearance) starting at
    z_head (head seat / -Z entry). Where it crosses the open wheel cavity
    it cuts nothing; the wheel (a separate part) carries its own Ø2.6 bore.

    z_head defaults to the -X side's value (= BACK_Z_BOT, on the back-leg
    floor). The +X (lever-side) mirror passes z_head = CHUNK_Z_BOT so the
    head seats at the brake chunk's -Z extent — same 21 mm bore, just
    starting from a lower z."""
    return cyl(GUIDE_AXLE_SHAFT_D, GUIDE_AXLE_HOLE_LEN,
               z=z_head).translate((GUIDE_WHEEL_CX, 0, 0))


def _guide_axle_head_recess():
    """Ø M2_HEAD_RECESS_D head-access bore for the +X (lever-side) guide
    axle: tunnels from the chunk's -Z face (CHUNK_Z_BOT) up to the head
    seat at GUIDE_AXLE_HEAD_Z, so the screw head sits at the same Z as
    the -X side's head (which seats directly on the back-leg floor) and
    the 21 mm shaft bore reaches correspondingly deeper. Mirror with _mx
    when cutting."""
    h = GUIDE_AXLE_HEAD_Z - CHUNK_Z_BOT + BOOL_OVERSHOOT
    return (cyl(M2_HEAD_RECESS_D, h, z=CHUNK_Z_BOT - BOOL_OVERSHOOT)
            .translate((GUIDE_WHEEL_CX, 0, 0)))


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


def _retainer_mortise_plus(z_bot):
    """+X-side dovetail mortise cutter — matches the tenon (cable_retainer.py:
    _retainer_tenon): wider tip on the housing side (+X), narrower neck on
    the retainer side, with the 45° / 30° flares chamfering the neck corners.
    z_bot sets the cavity's -Z extent — both ±X mortises now use the same
    z_bot = RETAIN_TENON_Z_BOT (= RETAIN_Z_LO), since the brake-band height
    now lifts the retainer's bottom above the ratchet chunk.
    Slot is the tenon polygon offset outward by FIT_CLR perpendicular to each
    edge. Extruded in Y from y = -HOUSING_W/2 + STRUCT_WALL (1.7 mm stop wall
    at -Y) to y = +HOUSING_W/2 + BOOL_OVERSHOOT (open at +Y for slide entry).

    x_tip uses x_neck + RETAIN_TENON_DEPTH (NOT SPINE_X_INNER) so the slot
    matches the tenon's actual depth — otherwise, if SPINE_X_INNER is closer
    to the spool than the tenon's flare endpoint, the polygon becomes
    self-intersecting and the boolean cut emits invalid geometry (faces with
    bad topology) that OCCT accepts but stricter importers reject on import. The
    1-2 mm of mortise extending past SPINE_X_INNER eats into the spine
    block, but the spine is HOUSING_SPINE_T (= 10) thick so there's still
    plenty of wall material left after the cut."""
    x_neck = RETAIN_R_OUT
    x_tip = x_neck + RETAIN_TENON_DEPTH
    # Shared polygon — guarantees mortise tracks tenon shape (straight top,
    # 45° bottom flare). Offset outward by FIT_CLR for slide fit.
    tenon = retainer_tenon_polygon(x_neck, x_tip, z_bot=z_bot,
                                    z_top=RETAIN_TENON_Z_TOP)
    # Cross-print joint (retainer prints +Z-down, housing prints -Y-down) —
    # use the wider CROSS_PRINT_CLR for slide fit, not the tight FIT_CLR.
    slot = _offset_polygon_ccw(tenon, CROSS_PRINT_CLR)
    OS = BOOL_OVERSHOOT
    y_lo = -HOUSING_W / 2 + STRUCT_WALL                # -9.3 (+Y face of stop wall)
    y_hi = +HOUSING_W / 2 + OS                          # +11+OS (open at +Y)
    # See cable_retainer.py: "XZ" workplane normal is -Y, so negate offset and
    # extrude length to actually span y ∈ [y_lo, y_hi]. This leaves the stop
    # wall on the -Y side of the chunk (at the print bed, no overhang).
    return (cq.Workplane("XZ").workplane(offset=-y_lo)
            .polyline(slot).close()
            .extrude(-(y_hi - y_lo)))


def _retainer_mortises():
    # Both sides use the same z_bot = RETAIN_TENON_Z_BOT (= RETAIN_Z_LO). The
    # -X side is mirrored across YZ so its stop wall (-Y) lines up with +X's;
    # a Z-rotation would flip Y too.
    plus  = _retainer_mortise_plus(RETAIN_TENON_Z_BOT)
    minus = _retainer_mortise_plus(RETAIN_TENON_Z_BOT).mirror("YZ")
    return plus.union(minus)


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
    # ── Ratchet (+Y side): head on +Y outer (lever side), insert in +Y face ──
    h = (h.cut(_pivot_clearance(RATCHET_PIVOT_X, RATCHET_PIVOT_Z, head_sign=+1))
          .cut(_pivot_insert_pilot(RATCHET_PIVOT_X, RATCHET_PIVOT_Z,
                                   insert_face_sign=+1, boss_ext=HOUSING_BOSS_EXT)))
    # ── Brake (-Y side): head on -Y outer (lever side), insert in -Y face ──
    h = (h.cut(_pivot_clearance(BRAKE_PIVOT_X, BRAKE_PIVOT_Z, head_sign=-1))
          .cut(_pivot_insert_pilot(BRAKE_PIVOT_X, BRAKE_PIVOT_Z,
                                   insert_face_sign=-1, boss_ext=HOUSING_BOSS_EXT)))

    # Housing stop pins project into the gap toward the lever (+Y for ratchet,
    # -Y for brake). Both SPRING and REST pins now use the same shallow
    # STRUCT_WALL-deep anchor in the housing wall — the old REST-pin design
    # ran the pin through the FULL housing width to gain anchorage when the
    # pin sat mostly -X of the block's inner face, but that through-pin now
    # collides with the new +X guide-wheel pocket. STRUCT_WALL (1.7 mm) of
    # overlap is enough boolean-fuse depth for a printed pin to merge into
    # its housing wall without breaking strength — same overlap depth the
    # spring pin already used (was 0.5 mm; bumped here for consistency).
    y_face = HOUSING_W / 2
    pin_anchor = STRUCT_WALL                                   # overlap depth into housing
    # Ratchet (+Y side): pin runs from inside the +Y wall outward into the
    # +Y lever gap. NO REST PIN on this side — the ratchet rim's teeth catch
    # the pawl at rest, and the rest pin was kicking the pawl just clear of
    # a clean tooth engagement.
    for alpha, is_spring in ((RATCHET_SPRING_PIN_ALPHA, True),):
        y_from, y_to = (y_face - pin_anchor, y_face + STOP_PIN_H)
        h = h.union(stop_pin_solid(RATCHET_PIVOT_X, RATCHET_PIVOT_Z, alpha, y_from, y_to))
        if is_spring:
            leg_a = alpha + (-1) * SPRING_LEG_PIN_OFFSET_DEG
            # Spring-leg hole at the halfway point of the EXPOSED pin (y_face
            # to y_face + STOP_PIN_H), so re-tuning STOP_PIN_H keeps the hole
            # automatically centred without re-deriving the hole position.
            h = h.cut(stop_pin_hole(RATCHET_PIVOT_X, RATCHET_PIVOT_Z, alpha,
                                    hole_y=y_face + STOP_PIN_H / 2,
                                    hole_dir_alpha_deg=spring_leg_hole_dir_alpha_deg(leg_a)))
    # Brake (-Y side): mirrored.
    for alpha, is_spring in ((BRAKE_REST_PIN_ALPHA, False),
                             (BRAKE_SPRING_PIN_ALPHA, True)):
        y_from, y_to = (-y_face - STOP_PIN_H, -y_face + pin_anchor)
        h = h.union(stop_pin_solid(BRAKE_PIVOT_X, BRAKE_PIVOT_Z, alpha, y_from, y_to))
        if is_spring:
            leg_a = alpha + (-1) * SPRING_LEG_PIN_OFFSET_DEG
            h = h.cut(stop_pin_hole(BRAKE_PIVOT_X, BRAKE_PIVOT_Z, alpha,
                                    hole_y=-y_face - STOP_PIN_H / 2,
                                    hole_dir_alpha_deg=spring_leg_hole_dir_alpha_deg(leg_a)))
    return h


def _build_housing_skeleton():
    # Mirror every guide-wheel feature across YZ so a second wheel sits in
    # the +X (lever) leg. The X-direction asymmetry of the originals (cut
    # toward +X to reach the rim) flips correctly to "cut toward -X" on the
    # mirror, putting both wheels tangent to the rim on opposite sides.
    def _mx(wp): return wp.mirror("YZ")
    h = (_front_block()
         .union(_back_block())
         .union(_back_wall_thicken())
         .union(_guide_box_frame())
         .union(_mx(_guide_box_frame()))
         .union(_guide_axle_boss())
         .union(_mx(_guide_axle_boss()))
         .union(_pancake_plate())
         .union(_front_thicken())
         .union(_retainer_thicken())
         .cut(_retainer_mortises())
         .cut(_axle_round_hole())
         # Screw-hole cut applied BEFORE the pocket cut — OCCT chokes on the
         # cylinder when the pocket cut has already exposed the cylinder's
         # +X edge, which sits within OCCT-tolerance of BACK_SPINE_X_INNER.
         .cut(_guide_axle_screw_hole())
         # +X (lever-side) mirror: shaft bore starts at the same z_head as
         # the -X side (GUIDE_AXLE_HEAD_Z = 1.9), so the head Z matches on
         # both sides and the 21 mm bore reaches deeper (through the chunk
         # roof and into the retainer's tenon — the retainer gets a matching
         # clearance hole in build.py). A Ø M2_HEAD_RECESS_D access bore
         # tunnels from the chunk's -Z face up to the head seat.
         .cut(_mx(_guide_axle_screw_hole()))
         .cut(_mx(_guide_axle_head_recess()))
         .cut(_guide_wheel_pocket_cut())
         .cut(_mx(_guide_wheel_pocket_cut(y_half=GUIDE_POCKET_Y_HALF_PLUS,
                                          z_hi=GUIDE_POCKET_Z_HI_PLUS)))
         # -Z bearing puck under each wheel hub. Unioned AFTER the pocket
         # cut so it survives. The +X puck sits inside the brake/ratchet
         # chunks' footprint and gets sliced by the chunk-split diagonal —
         # each chunk ends up carrying its share of the puck.
         .union(_guide_axle_spacer())
         .union(_mx(_guide_axle_spacer()))
         .union(_guide_pocket_back_y_chamfer())
         .union(_mx(_guide_pocket_back_y_chamfer(y_half=GUIDE_POCKET_Y_HALF_PLUS)))
         )
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
# SHARED 45° diagonal split for both chunks. Both the brake (green, -Y/-Z
# half) and ratchet (red, +Y/+Z half) glue-on chunks now sit on the +X (lever)
# leg of the housing covering the full pivot + guide-wheel volume, and split
# that volume along ONE diagonal plane z + y = CHUNK_DIAGONAL_C:
#   • brake:  z + y ≤ C  → -Y/-Z half (lower-left when viewed from +X)
#   • ratchet: z + y > C → +Y/+Z half (upper-right)
# Both chunks are bounded above at CHUNK_Z_TOP = guide-wheel-pocket top +
# STRUCT_WALL, so the material above belongs to housing-main (which carries
# the retainer mortise STRUCT_WALL above the chunk top — see cable_retainer.
# RETAIN_TENON_Z_BOT).
CHUNK_Z_BOT        = -5.0                                   # current chunk bottom
CHUNK_Z_TOP        = GUIDE_POCKET_Z_HI + STRUCT_WALL        # 15.77 — +STRUCT_WALL above wheel cut
# Diagonal shifted off centroid so the brake (green) chunk gets STRUCT_WALL
# of flat top where it meets the housing-main at Z = CHUNK_Z_TOP. This gives
# the brake chunk a small horizontal landing onto the housing's -Z face so
# the brake can engage the housing's rail tenon and pick up support from it
# (without that overlap, brake's +Z face was only 0.6 mm wide). At Z_TOP the
# brake covers Y ∈ [-HOUSING_W/2, -HOUSING_W/2 + STRUCT_WALL]; at lower Z it
# fans out and covers most of the chunk Y/Z volume.
CHUNK_DIAGONAL_C   = CHUNK_Z_TOP - HOUSING_W / 2 + STRUCT_WALL   # ≈ 6.47
BRAKE_CHUNK_RING_CLR = 0.4                                  # (kept for downstream rail-math compat)
BRAKE_CHUNK_CUT_C    = CHUNK_DIAGONAL_C                     # legacy alias; brake = z+y ≤ C
_BRAKE_CHUNK_X_LO = 60.0          # cutter X span (covers the front block + -X rest pin)
_BRAKE_CHUNK_X_HI = 80.0


def _brake_chunk_cutter():
    """Half-space z + y ≤ CHUNK_DIAGONAL_C AND z ≤ CHUNK_Z_TOP, bounded in X
    to the brake-mount region. housing ∩ it = the brake chunk."""
    C  = CHUNK_DIAGONAL_C
    Zt = CHUNK_Z_TOP
    big = 80.0
    # CCW in (y, z): the diagonal hits z=Zt at y = C-Zt and y=+big at z=C-big.
    pts = [
        (-big, -big),
        (+big, -big),
        (+big, C - big),        # diagonal at +y edge (far below Zt)
        (C - Zt, Zt),           # diagonal at top edge
        (-big, Zt),
    ]
    return (cq.Workplane("YZ").workplane(offset=_BRAKE_CHUNK_X_LO)
            .polyline(pts).close()
            .extrude(_BRAKE_CHUNK_X_HI - _BRAKE_CHUNK_X_LO))


# ── Ratchet glue-on chunk (+Y/+Z half of the shared chunk region) ───────────
# Mirror of the brake chunk across the shared z+y=C diagonal. Together with
# brake_pin_chunk, covers the full +X-leg chunk region below CHUNK_Z_TOP.
RATCHET_CHUNK_RING_CLR = 0.4
RATCHET_CHUNK_CUT_C    = CHUNK_DIAGONAL_C        # SAME plane as brake now

_RATCHET_CHUNK_X_LO = _BRAKE_CHUNK_X_LO          # 60 — same X span as brake
_RATCHET_CHUNK_X_HI = _BRAKE_CHUNK_X_HI          # 80


def _ratchet_chunk_cutter():
    """Half-space z + y > CHUNK_DIAGONAL_C AND z ≤ CHUNK_Z_TOP, bounded in X.
    The +Y/+Z half opposite the brake chunk along the shared diagonal."""
    C  = CHUNK_DIAGONAL_C
    Zt = CHUNK_Z_TOP
    big = 80.0
    # CCW in (y, z) — the +Y/+Z triangle above the diagonal and below Zt.
    pts = [
        (+big, C - big),        # bottom-right (on diagonal, very low)
        (+big, Zt),             # top-right
        (C - Zt, Zt),           # top, on diagonal
    ]
    return (cq.Workplane("YZ").workplane(offset=_RATCHET_CHUNK_X_LO)
            .polyline(pts).close()
            .extrude(_RATCHET_CHUNK_X_HI - _RATCHET_CHUNK_X_LO))


# ── Registration V-rails on the glue joint ──────────────────────────────────
# Two arrowhead rails run across the joint face (along its +Y..-Y direction)
# to lock the X position when gluing the chunk on. The HOUSING gets the
# arrowhead ridge, the chunk gets a matching (oversized) groove. They flank
# the central pivot-screw hole — one on the -X side (over the embedded rest
# pin), one on the +X side (in the gap past the pivot screw) — so neither
# hits the bore.
#
# Cross-section (in the rail's local XZ, before rotation):
#   • Flat bottom of width REG_ARROW_W_BOT at z=0 (the joint plane).
#   • "Hips" of width REG_ARROW_W_HIP at z=-REG_ARROW_H_HIP, wider than the
#     bottom so the rail locks the chunk against pulling perpendicular to the
#     joint plane (dovetail-like).
#   • Pointed apex at z=-REG_ARROW_H_APEX (chunk side).
# h_apex > h_hips so the apex-to-hips face is self-supporting after the rail's
# 45° rotation into the housing print frame.
REG_ARROW_W_BOT   = STRUCT_WALL              # 1.7 mm — flat bottom width
REG_ARROW_W_HIP   = REG_ARROW_W_BOT + 0.8    # 2.5 mm — hips 0.8 mm wider than bottom
REG_ARROW_H_HIP   = 1.0                       # depth of hips below the flat bottom
REG_ARROW_H_APEX  = 2.0                       # depth of apex below the flat bottom
# Rails centred in the two solid spans of the joint face, flanking the pivot
# bore void at x∈[66.0, 68.4]: -X span [61.43, 66.0], +X span [68.4, 74.73].
REG_RAIL_X            = (63.7, 71.5)   # -X / +X rail positions
_REG_RAIL_L           = 40.0       # raw length; trimmed to the joint by ∩ housing
REG_RAIL_END_GAP      = 0.4        # material left between the cut and the close
                                   # (-Z) face at the rail's -Z end
_REG_RAIL_YC          = -5.0
_REG_RAIL_ZC          = BRAKE_CHUNK_CUT_C - _REG_RAIL_YC   # (yc, zc) lies on z+y=C


def _arrowhead_polygon_ccw(w_bot, w_hip, h_hip, h_apex):
    """Arrowhead cross-section polygon in CCW order (in local XZ plane).
    Flat bottom at z=0, hips at z=-h_hip, apex at z=-h_apex (chunk side)."""
    return [
        (-w_bot / 2, 0),
        (-w_hip / 2, -h_hip),
        (0, -h_apex),
        (w_hip / 2, -h_hip),
        (w_bot / 2, 0),
    ]


_ARROW_RAIL_POLY   = _arrowhead_polygon_ccw(
    REG_ARROW_W_BOT, REG_ARROW_W_HIP, REG_ARROW_H_HIP, REG_ARROW_H_APEX,
)
# Groove = rail polygon expanded outward by FIT_CLR perpendicular to each edge.
# Gives a uniform running clearance everywhere around the rail's cross-section.
_ARROW_GROOVE_POLY = _offset_polygon_ccw(_ARROW_RAIL_POLY, FIT_CLR)
# Clip the brake rail at the brake-ratchet boundary so it doesn't extend
# into the ratchet chunk's region. The portion at y > boundary_y would be
# floating in air during the housing's print: that region is removed from
# main (it becomes the ratchet chunk) and the pure-V rail doesn't connect
# to anything above the brake joint plane at those Y. The clip face is
# perpendicular to the rail's local Y, which puts it 45° in world — facing
# UP on the +Y/+Z side, supported from below — printable. The boundary is
# where the two cut planes intersect: y = (C_brake − C_ratchet)/2. Local
# LY at that world y: LY = (y − yc)/c = (y − yc)·√2.
# (This is a tighter trim than the old "apex END_GAP above FRONT_Z_BOT"
# constraint — the resulting stop wall is ~3.6 mm instead of 2.2 mm.)
# With brake/ratchet sharing one diagonal, the brake rail runs along the
# full length of the joint (no longer clipped at a brake-ratchet boundary).
# Big value → effectively unbounded; the .intersect(_housing_full) trim
# later confines it to actual housing material.
_REG_RAIL_LY_HI       = 1000.0


def _reg_rail(x_rail, polygon):
    """One arrowhead rail prism centred on the joint plane (z + y = C) at
    X=x_rail, running along the joint's in-plane Y direction. The cross-
    section `polygon` (in local XZ; apex toward chunk side, flat bottom on
    the joint plane) is extruded along local Y and rotated to align with the
    brake joint plane. Pass _ARROW_RAIL_POLY for the rail solid or
    _ARROW_GROOVE_POLY for the chunk-side groove cutter.
    The rail material is entirely on the chunk side (the flat bottom sits
    exactly on the joint plane) so the rail doesn't intrude into adjacent
    regions; the kissing surface at the joint plane is handled by OCCT's
    union as an internal face.
    The -Z (low-z) end is clipped at the brake-ratchet boundary so the rail
    doesn't extend into the ratchet chunk's region (where it would print
    floating in air). The cut face is 45° in world, facing UP when the
    housing prints -Y down — support-free. The +Z end runs full to the
    housing's -Y face (entry for the chunk's groove)."""
    yc = _REG_RAIL_YC
    zc = _REG_RAIL_ZC
    clip = (cq.Workplane("XY").box(1000, 1000, 1000, centered=True)
            .translate((0, _REG_RAIL_LY_HI + 500, 0)))   # local +Y past the clip
    return (cq.Workplane("XZ")
            .polyline(polygon).close()
            .extrude(_REG_RAIL_L / 2, both=True)
            .cut(clip)
            .rotate((0, 0, 0), (1, 0, 0), -45)
            .translate((x_rail, yc, zc)))


def _reg_rails(polygon):
    return (_reg_rail(REG_RAIL_X[0], polygon)
            .union(_reg_rail(REG_RAIL_X[1], polygon)))


# ── Registration V-rails on the RATCHET joint plane ─────────────────────────
# Mirror of the brake rails but on the ratchet joint plane (z − y = C_ratchet).
# Open end at world +Y/+Z (housing's +Y face); stop at world −Y/−Z (toward the
# brake-ratchet boundary). The chunk slides on in −Y/−Z direction along the
# joint plane, so it enters from +Y and stops at −Y per the requested
# orientation.
#
# Printability note: the stop face's outward normal is in world −Y/−Z, which
# has a downward (−Y) component in the housing's −Y-down print orientation.
# That's a 45° overhang at the stop end — borderline self-supporting; supports
# may be needed here. The brake's stop avoids this because it sits at +Y/−Z
# (up-facing in print).
_RATCHET_REG_RAIL_YC = 3.5                                              # mid-ish along chunk y-range
_RATCHET_REG_RAIL_ZC = _RATCHET_REG_RAIL_YC + RATCHET_CHUNK_CUT_C       # on z = y + C_ratchet
# LY_LO is set so the cut axis sits a target STOP_T perpendicular distance
# from the brake cut line — that distance IS the stop wall thickness in local
# Y (because the ratchet rail's local Y direction is perpendicular to the
# brake cut line, since the two cut planes are at 90° to each other).
#
# We match the brake side's stop wall (set by clipping the brake rail at the
# brake-ratchet boundary — its stop wall is the local-Y distance from the
# boundary to the brake chunk's apex point at (C_brake − FRONT_Z_BOT, FRONT_Z_BOT)).
# Both joints end up with the same ~3.6 mm stop wall thickness.
#
# Apex y + z = (yc + zc) + 2·LY·c (c = cos45 = √2/2). Setting cut-axis
# perpendicular distance to brake line = STOP_T:
#   (yc + zc) + LY_LO·√2 = C_brake + STOP_T·√2
#   LY_LO = (C_brake + STOP_T·√2 − (yc + zc)) / √2
_BRAKE_STOP_WALL = ((BRAKE_CHUNK_CUT_C + RATCHET_CHUNK_CUT_C - 2 * FRONT_Z_BOT)
                    / math.sqrt(2))                                     # ≈ 3.628 mm
_RATCHET_REG_RAIL_STOP_T = _BRAKE_STOP_WALL
_RATCHET_REG_RAIL_LY_LO = (
    (BRAKE_CHUNK_CUT_C + _RATCHET_REG_RAIL_STOP_T * math.sqrt(2)
     - _RATCHET_REG_RAIL_YC - _RATCHET_REG_RAIL_ZC) / math.sqrt(2)
)


def _ratchet_reg_rail(x_rail, polygon):
    """One arrowhead rail on the ratchet joint plane (z − y = C). Same cross-
    section as the brake rail (apex toward chunk, flat bottom on the joint
    plane), but rotated +45° about X so the rail length runs along world
    +Y/+Z, and clipped at the local −Y end so the cut/stop face is at world
    −Y/−Z (toward the brake chunk). Open end at world +Y/+Z (the housing's
    +Y face). Pass _ARROW_RAIL_POLY for the rail solid or _ARROW_GROOVE_POLY
    for the chunk-side groove cutter."""
    yc = _RATCHET_REG_RAIL_YC
    zc = _RATCHET_REG_RAIL_ZC
    clip = (cq.Workplane("XY").box(1000, 1000, 1000, centered=True)
            .translate((0, _RATCHET_REG_RAIL_LY_LO - 500, 0)))   # local Y ≤ LY_LO
    return (cq.Workplane("XZ")
            .polyline(polygon).close()
            .extrude(_REG_RAIL_L / 2, both=True)
            .cut(clip)
            .rotate((0, 0, 0), (1, 0, 0), +45)
            .translate((x_rail, yc, zc)))


def _ratchet_reg_rails(polygon):
    return (_ratchet_reg_rail(REG_RAIL_X[0], polygon)
            .union(_ratchet_reg_rail(REG_RAIL_X[1], polygon)))


_housing_full   = _build_housing()
_brake_cutter   = _brake_chunk_cutter()
_ratchet_cutter = _ratchet_chunk_cutter()
_main           = _housing_full.cut(_brake_cutter).cut(_ratchet_cutter)
_brake_chunk    = _housing_full.intersect(_brake_cutter)
# Brake takes precedence in the overlap region: subtract brake_cutter so the
# two chunks share a clean 45° boundary along the brake cut line instead of
# competing for material.
_ratchet_chunk  = _housing_full.intersect(_ratchet_cutter).cut(_brake_cutter)

# Both joint planes get V-rail ridges on the housing + oversized V-grooves in
# the chunks. The brake rail extends a hair past the joint plane into
# housing-main material — in the brake-ratchet overlap region (y ∈ [-4, -1.434])
# housing-main IS the ratchet chunk's region, so the brake rail also clips into
# the ratchet chunk; cut the brake groove in both chunks for clearance. The
# ratchet rail stays on its own side (cut face is well inside the ratchet
# chunk region with END_GAP clearance from the brake-ratchet boundary), so it
# only needs to be cut from the ratchet chunk.
# ── Ratchet-to-housing joint: Y-aligned dovetail on the housing's -Z face ──
# The ratchet chunk meets housing-main at Z = CHUNK_Z_TOP. The housing
# carries a Y-aligned dovetail tenon hanging DOWN from its -Z face into the
# chunk's territory; the chunk has a matching FIT_CLR-oversized groove on
# its +Z face. Slide-in is along Y from the housing's +Y face. Cross-section
# in (X, Z) is a 45° dovetail trapezoid: narrow at the base (Z = 0, at the
# housing's -Z face) and wider at the tip (Z = -H_TENON, deep in chunk) so
# the chunk locks against -Z pull-off.
RAIL_TENON_W_BASE  = STRUCT_WALL                          # narrow base at the joint plane (ratchet side)
RAIL_TENON_W_MAX   = RAIL_TENON_W_BASE + 2 * STRUCT_WALL  # widest point (the captive "diamond" middle)
RAIL_TENON_W_TIP   = STRUCT_WALL                          # narrow flat tip
# Brake side uses doubled widths (neck/tip = 2 * STRUCT_WALL, max scaled to
# preserve 45° flares and overall depth). The wider stem gives more contact
# area on the chunk's diagonal face; ratchet keeps the narrow widths because
# the retainer's mortise/tenon already occupies the X real-estate it would
# need to grow into.
_BRAKE_RAIL_W_BASE = 2 * STRUCT_WALL                          # 3.2
_BRAKE_RAIL_W_MAX  = _BRAKE_RAIL_W_BASE + 2 * STRUCT_WALL     # 6.4
_BRAKE_RAIL_W_TIP  = 2 * STRUCT_WALL                          # 3.2
# Octagonal cross-section: vertical neck → 45° flare-out to the wide middle
# → 45° flare-in to the flat tip. The wide middle is what locks the joint
# perpendicular to the joint plane; the neck makes the joint-plane corner a
# clean 90° (prints accurately on both pieces) and the flat tip avoids a
# fragile single-point apex at the joint's deepest end.
RAIL_TENON_NECK_H      = STRUCT_WALL                                       # 1.6 — vertical neck
RAIL_TENON_FLARE_OUT_H = (RAIL_TENON_W_MAX - RAIL_TENON_W_BASE) / 2        # 1.6 — flare out (45°)
RAIL_TENON_FLARE_IN_H  = (RAIL_TENON_W_MAX - RAIL_TENON_W_TIP) / 2         # 1.6 — flare in (45°)
RAIL_TENON_H = RAIL_TENON_NECK_H + RAIL_TENON_FLARE_OUT_H + RAIL_TENON_FLARE_IN_H   # 4.8 — total depth
_RAIL_X_CENTER    = (_BRAKE_CHUNK_X_LO + _BRAKE_CHUNK_X_HI) / 2

# Brake (green) side: housing-side tenon hanging DOWN into the brake chunk
# at z ≤ CHUNK_Z_TOP. Y extent spans the FULL brake-chunk wall thickness
# (= 3 mm from y = -HOUSING_W/2 to y = -HOUSING_W/2 + 3), so the mortise
# carves all the way through the wall instead of using only the lower half
# of it. At z = CHUNK_Z_TOP, the chunk only covers y ∈ [-11, -9.4] — the
# 1.4 mm of rail at y > -9.4 protrudes into the ratchet chunk's territory
# instead; the ratchet chunk takes a matching cut of the brake-rail groove
# (see ratchet_pin_chunk below). The brake doesn't need its own slide stop —
# the ratchet provides the depth reference: the brake slides in until its
# diagonal face seats against the ratchet's.
_BRAKE_RAIL_Y_LO = -HOUSING_W / 2 - BOOL_OVERSHOOT                # flush with housing's -Y face
_BRAKE_RAIL_Y_HI = -HOUSING_W / 2 + 3.2                           # spans the full chunk wall + 0.2 mm to absorb the leftover sliver where the chunk's diagonal meets the mortise edge

# Ratchet (red) side: chunk-side tenon sticking UP from the chunk's top face
# at z = CHUNK_Z_TOP into a housing-side groove going UP into housing material
# at z > CHUNK_Z_TOP.
#
# Chunk slides -Y to insert (enters the housing from the +Y face). Groove's
# CLOSED end at -Y is the hard stop — the tenon's -Y leading edge seats
# against it. Groove's +Y end extends past the housing's +Y face (open) so
# the chunk can slide in from outside.
#
# Tenon and groove have DIFFERENT Y ranges:
#   - Tenon stays inside the ratchet chunk's top territory (y > C − CHUNK_Z_TOP)
#     and ends inside the chunk on +Y (no need to reach the +Y face).
#   - Groove extends past the +Y face on its open side; both close at the
#     same -Y stop so the tenon's -Y face seats cleanly against the groove wall.
# Tenon's -Y end (and groove's closed-end / hard stop) sits STRUCT_WALL past
# the brake rail's +Y end so the ratchet chunk keeps 1.6 mm of solid material
# between its brake-rail groove cut and the base of its own tenon.
_RATCHET_TENON_Y_LO  = _BRAKE_RAIL_Y_HI + STRUCT_WALL         # tenon/groove -Y stop (= -6.4)
_RATCHET_TENON_Y_HI  = +HOUSING_W / 2                         # tenon's +Y end flush with the chunk's +Y face
_RATCHET_GROOVE_Y_HI = +HOUSING_W / 2 + BOOL_OVERSHOOT        # groove +Y open end (past the +Y face)


def _rail_tenon_polygon(w_base, w_max, w_tip,
                        neck_h=RAIL_TENON_NECK_H,
                        flare_out_h=RAIL_TENON_FLARE_OUT_H,
                        flare_in_h=RAIL_TENON_FLARE_IN_H,
                        z_clearance=0.0,
                        points_up=False):
    """8-vertex (or 10-vertex when z_clearance>0) polygon (X, Z) for the
    chunk-joint rail/mortise. Vertical neck of length neck_h at width
    w_base, 45° flare-out to width w_max, 45° flare-in to a flat tip of
    width w_tip.

    z_clearance > 0 adds a vertical extension of length z_clearance at the
    flat tip (width unchanged), giving Z slack so the groove can be deeper
    than its tenon — the tenon's tip doesn't bottom out. Pass z_clearance
    only to the GROOVE polygon (the tenon stays at its nominal depth, and
    the locking widest-point geometry stays identical on both sides).

    points_up=False (default): base at z=0, tip at the most negative z.
    Used for the BRAKE rail (housing tenon hanging down into the chunk).
    points_up=True: base at z=0, tip at the most positive z. Used for the
    RATCHET rail (chunk tenon sticking up into the housing's mortise)."""
    s = +1 if points_up else -1
    z_neck   = neck_h
    z_widest = neck_h + flare_out_h
    z_tip    = z_widest + flare_in_h
    pts = [
        (+w_base / 2, 0),                       # base-right
        (-w_base / 2, 0),                       # base-left
        (-w_base / 2, s * z_neck),              # neck-bot-left
        (-w_max  / 2, s * z_widest),            # widest-left
        (-w_tip  / 2, s * z_tip),               # tip-left (top corner of any tip extension)
    ]
    if z_clearance > 0:
        pts.append((-w_tip / 2, s * (z_tip + z_clearance)))  # extended tip, -X corner
        pts.append((+w_tip / 2, s * (z_tip + z_clearance)))  # extended tip, +X corner
    pts.extend([
        (+w_tip  / 2, s * z_tip),               # tip-right (top corner of any tip extension)
        (+w_max  / 2, s * z_widest),            # widest-right
        (+w_base / 2, s * z_neck),              # neck-bot-right
    ])
    return pts


def _rail_prism(poly, y_lo, y_hi, z_base):
    """Extrude `poly` (defined in XZ) along +Y from y_lo to y_hi, with its
    z=0 line landing on z=z_base."""
    return (cq.Workplane("XZ")
            .polyline(poly).close()
            .extrude(y_hi - y_lo)
            .translate((_RAIL_X_CENTER, y_hi, z_base)))


def _brake_rail_solid_and_groove():
    """Brake side: housing-side tenon (the SOLID) + matching chunk-side
    groove (the GROOVE = solid + CROSS_PRINT_CLR all around — wider than
    FIT_CLR because the rail prints on the housing's -Y face while the
    chunk's groove prints on a different face)."""
    solid_poly = _rail_tenon_polygon(_BRAKE_RAIL_W_BASE, _BRAKE_RAIL_W_MAX, _BRAKE_RAIL_W_TIP)
    groove_poly = _rail_tenon_polygon(_BRAKE_RAIL_W_BASE + 2 * CHUNK_RAIL_CLR,
                                       _BRAKE_RAIL_W_MAX  + 2 * CHUNK_RAIL_CLR,
                                       _BRAKE_RAIL_W_TIP  + 2 * CHUNK_RAIL_CLR,
                                       z_clearance=CHUNK_RAIL_CLR)
    solid  = _rail_prism(solid_poly,  _BRAKE_RAIL_Y_LO, _BRAKE_RAIL_Y_HI, CHUNK_Z_TOP)
    groove = _rail_prism(groove_poly, _BRAKE_RAIL_Y_LO, _BRAKE_RAIL_Y_HI, CHUNK_Z_TOP)
    return solid, groove


def _ratchet_rail_solid_and_groove():
    """Ratchet side: chunk-side tenon (the SOLID, attached to the chunk) +
    matching housing-side groove going UP from the joint plane. Tenon and
    groove share the +Y stop but the groove extends further -Y, past the
    housing's -Y face, so the chunk can slide in from outside."""
    solid_poly = _rail_tenon_polygon(RAIL_TENON_W_BASE, RAIL_TENON_W_MAX, RAIL_TENON_W_TIP,
                                      points_up=True)
    # Cross-print joint (chunk prints +Y-down, housing prints -Y-down).
    groove_poly = _rail_tenon_polygon(RAIL_TENON_W_BASE + 2 * CHUNK_RAIL_CLR,
                                       RAIL_TENON_W_MAX  + 2 * CHUNK_RAIL_CLR,
                                       RAIL_TENON_W_TIP  + 2 * CHUNK_RAIL_CLR,
                                       z_clearance=CHUNK_RAIL_CLR,
                                       points_up=True)
    solid  = _rail_prism(solid_poly,  _RATCHET_TENON_Y_LO, _RATCHET_TENON_Y_HI,  CHUNK_Z_TOP)
    groove = _rail_prism(groove_poly, _RATCHET_TENON_Y_LO, _RATCHET_GROOVE_Y_HI, CHUNK_Z_TOP)
    return solid, groove


_brake_rail_solid,   _brake_rail_groove   = _brake_rail_solid_and_groove()
_ratchet_rail_solid, _ratchet_rail_groove = _ratchet_rail_solid_and_groove()


# ── Floating tenon: hexagonal hourglass spanning the brake/ratchet diagonal ─
# Both chunks print face-down on their diagonal joint face, so neither can
# carry a printed tenon. Instead each chunk has a half-mortise on its
# diagonal face, and a separately-printed "floating tenon" (this part) sits
# in both mortises to lock the chunks together perpendicular to the joint.
#
# Slide direction: along the in-plane diagonal direction ξ = (-Y + Z)/√2 —
# the tenon enters from the chunks' (+Y, -Z) corner where the diagonal joint
# first meets the chunk volume, and slides "up the joint" toward (-Y, +Z).
# Cross-section perpendicular to ξ is in (local-X, η) where η = (Y + Z)/√2
# is the diagonal-joint normal. The hourglass varies along η: wide at
# η = ±end_h/2 (deep into each chunk), narrow at η = 0 (on the diagonal
# joint line itself). 45° transitions (end_x − waist_x = end_h).
#
# Length is set so the far end of the tenon stops STRUCT_WALL clear of the
# +X guide-wheel pocket along Z (the pocket starts at GUIDE_POCKET_Z_LO).
FLOAT_TENON_BULGE_X       = 6.0          # X-extent at the wide bulge (max width)
FLOAT_TENON_WAIST_X       = 2.0          # X-extent through the straight waist
FLOAT_TENON_WAIST_H       = STRUCT_WALL  # η-thickness of the STRAIGHT waist section
FLOAT_TENON_APEX_X        = STRUCT_WALL  # X-extent of the apex flat at top/bottom
# Both tapers (waist→bulge and bulge→apex) at exactly 45°. Each derives
# its η-extent from the X-extent change (Δη = ΔX). Every interior angle in
# the polygon is now a multiple of 45° — easier to print accurately, and
# 135° at the apex flat corners / 45° at the bulge vertex.
FLOAT_TENON_BULGE_H = FLOAT_TENON_WAIST_H + (FLOAT_TENON_BULGE_X - FLOAT_TENON_WAIST_X)
FLOAT_TENON_APEX_H  = FLOAT_TENON_BULGE_H + (FLOAT_TENON_BULGE_X - FLOAT_TENON_APEX_X)
FLOAT_TENON_X_CENTER = (_BRAKE_CHUNK_X_LO + _BRAKE_CHUNK_X_HI) / 2  # 70 — chunk X midpoint

# Entry face placed at the chunks' +Y/-Z exterior corner — the most "+y, -z"
# point the diagonal joint reaches inside the chunk region. With the new C,
# the diagonal crosses the +Y face (Y = HOUSING_W/2) at Z = C − HOUSING_W/2,
# so the entry sits right at that face. The +η corner of the tenon at this
# entry face sticks slightly past the housing's +Y face — accepted, since
# the floating tenon is a separate part and the overshoot is just exposed
# material when assembled.
# Entry placed at the chunk's +Y -Z exterior corner — off-diagonal by
# (C - entry_y - entry_z)/√2, sitting just below the diagonal in brake
# territory. This lets the tenon's -Z tip reach all the way down to the
# chunk floor (Z = CHUNK_Z_BOT) instead of stopping at the diagonal's
# entry-on-+Y-face point.
_FLOAT_ENTRY_Y = HOUSING_W / 2
_FLOAT_ENTRY_Z = CHUNK_Z_BOT

# Length: stop ≈ 1 mm short of the +X guide-wheel pocket at the +η corner
# (the corner deepest into ratchet — its world Z is the largest along the
# tenon and so binds first against the pocket floor). Larger clearance
# values would shorten the tenon further; STRUCT_WALL clearance pulls the
# length down significantly so we use 1 mm here.
_FLOAT_POCKET_CLEARANCE = 1.0
FLOAT_TENON_LENGTH = (
    (GUIDE_POCKET_Z_LO - _FLOAT_POCKET_CLEARANCE - _FLOAT_ENTRY_Z
     - FLOAT_TENON_APEX_H / (2 * math.sqrt(2)))
    * math.sqrt(2)
)


def _float_tenon_polygon(bulge_x, waist_x, bulge_h, waist_h, apex_x, apex_h):
    """CCW 12-vertex polygon in (local-X, η). Cross-section perpendicular
    to ξ. Each side (top and bottom) tapers: apex flat at η = ±apex_h/2
    (X = apex_x) → wide bulge at η = ±bulge_h/2 (X = bulge_x) → straight
    waist of η-thickness waist_h centred on η=0 (X = waist_x). 45° taper
    from bulge to apex if apex_h − bulge_h == bulge_x − apex_x; the
    waist↔bulge taper has its own (steeper) slope set by the
    waist_h / bulge_h ratio."""
    return [
        (+bulge_x / 2, -bulge_h / 2),    # V1: bottom-right bulge
        (+waist_x / 2, -waist_h / 2),    # V2: right of waist bottom
        (+waist_x / 2, +waist_h / 2),    # V3: right of waist top
        (+bulge_x / 2, +bulge_h / 2),    # V4: top-right bulge
        (+apex_x  / 2, +apex_h  / 2),    # V5: top-right of apex flat
        (-apex_x  / 2, +apex_h  / 2),    # V6: top-left of apex flat
        (-bulge_x / 2, +bulge_h / 2),    # V7: top-left bulge
        (-waist_x / 2, +waist_h / 2),    # V8: left of waist top
        (-waist_x / 2, -waist_h / 2),    # V9: left of waist bottom
        (-bulge_x / 2, -bulge_h / 2),    # V10: bottom-left bulge
        (-apex_x  / 2, -apex_h  / 2),    # V11: bottom-left of apex flat
        (+apex_x  / 2, -apex_h  / 2),    # V12: bottom-right of apex flat
    ]


def _float_tenon_solid_from_poly(poly_pts, length, pre_translate_z=0.0):
    """Build hourglass in local frame (XY-plane = cross-section, +Z = ξ
    slide direction), translate to set X centre, rotate +45° about world X
    so local +Z maps to world ξ = (-Y+Z)/√2, then translate so the entry
    plane (local Z = 0) intersects the diagonal at the chosen entry point.
    pre_translate_z extends the prism backward along local ξ — used to
    extend the mortise past the tenon's entry face."""
    return (cq.Workplane("XY")
            .polyline(poly_pts).close()
            .extrude(length)
            .translate((FLOAT_TENON_X_CENTER, 0, pre_translate_z))
            .rotate((0, 0, 0), (1, 0, 0), 45)
            .translate((0, _FLOAT_ENTRY_Y, _FLOAT_ENTRY_Z)))


_float_tenon_pts     = _float_tenon_polygon(FLOAT_TENON_BULGE_X,
                                             FLOAT_TENON_WAIST_X,
                                             FLOAT_TENON_BULGE_H,
                                             FLOAT_TENON_WAIST_H,
                                             FLOAT_TENON_APEX_X,
                                             FLOAT_TENON_APEX_H)
_float_mortise_pts   = _offset_polygon_ccw(_float_tenon_pts, FIT_CLR)
floating_tenon       = _float_tenon_solid_from_poly(_float_tenon_pts,
                                                    FLOAT_TENON_LENGTH)
# Trim the +Y overshoot so the tenon's +Y face sits flush with the chunk's
# +Y wall (Y = HOUSING_W/2 = 11). The polygon's +η corner naturally pokes a
# bit past +Y when rotated 45° around X, and with LEVER_RIM_H shrunk to
# 3.3 mm that overshoot now collides with the ratchet lever.
_FLOAT_TENON_PY_TRIM = (cq.Workplane("XY")
                       .box(2000, 1000, 2000, centered=(True, False, True))
                       .translate((0, HOUSING_W / 2, 0)))
floating_tenon       = floating_tenon.cut(_FLOAT_TENON_PY_TRIM)
_float_tenon_mortise = _float_tenon_solid_from_poly(_float_mortise_pts,
                                                    FLOAT_TENON_LENGTH + 2 * FIT_CLR,
                                                    pre_translate_z=-FIT_CLR)


# ── Assemble main housing + chunks ─────────────────────────────────────────
# The old 45°-diagonal brake reg-rails are gone: with the shared z+y=C
# diagonal, that joint is now between the brake and ratchet chunks (not
# between brake and housing-main), so the chunk-chunk interlock is the
# floating tenon. Brake-to-housing-main is implicit (brake's tiny +Z
# slice y ∈ [-11, -10.38] is too narrow for a useful rail; the chunk
# stays in place via the floating tenon + glue).
# Brake side: housing carries the tenon (UNION the brake rail solid back
# into the main, trimmed to housing boundary), brake chunk gets a matching
# groove. Ratchet side (swapped): the chunk carries the tenon (UNION the
# ratchet rail solid into the ratchet chunk), housing gets the matching
# groove (CUT from the main).
housing           = (_main
                     .union(_brake_rail_solid.intersect(_housing_full))
                     .cut(_ratchet_rail_groove))
brake_pin_chunk   = (_brake_chunk
                     .cut(_brake_rail_groove)
                     .cut(_float_tenon_mortise))
# Ratchet's tenon protrudes ABOVE CHUNK_Z_TOP into housing-territory; just
# union the full solid into the chunk's STEP (it's already X-bounded to
# the chunk's footprint by the rail polygon's centering). Also cut the
# brake-rail groove from this chunk: the brake rail spans more Y than
# the brake chunk reaches at z=CHUNK_Z_TOP, so its +Y end pokes 1.4 mm
# into the ratchet chunk's territory and needs a matching cavity here.
ratchet_pin_chunk = (_ratchet_chunk
                     .union(_ratchet_rail_solid)
                     .cut(_brake_rail_groove)
                     .cut(_float_tenon_mortise))
