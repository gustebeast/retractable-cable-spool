"""Fixed cable-retention cage — keeps the wound cable from bulging out of
its channel (radially, in X/Y) when it goes slack.

Unlike the spool's own rims, this part must NOT rotate with the spool: the
cable enters the channel at a point fixed relative to the housing, so the
retainer is housing-attached and stationary while the spool turns inside it.

This module models ONLY the functional portion — a stationary cage that
SURROUNDS the spool rims at a slightly larger diameter (a ~1 mm radial air
gap all round, so it never touches the rotating spool): two thin rings joined
by vertical bars, leaving sub-cable air gaps (< CABLE_D) so a slack coil can't
escape. The ring-gap lines up with the cable air gap between the bottom rim and
the cable top rim, where the cable passes through (+x/+y). Attachment to the
housing comes later.
"""

import math

import cadquery as cq

from .spool import RIM_OD, RIM_H, CABLE_D, SPOOL_HORIZONTAL_CLEARANCE
from .dimensions import (STRUCT_WALL, FIT_CLR, BOOL_OVERSHOOT, HUB_OD,
                         GUIDE_WHEEL_OD, GUIDE_WHEEL_RIM_PRELOAD)
from .helpers import cyl


# The cage now sits OUTSIDE the spool rims (larger diameter) and surrounds them
# with a small radial air gap, so it never touches the rotating spool. Its
# ring-gap lines up with the cable air gap between the bottom (brake) rim and the
# cable top rim — the cable passes through there (radially, at +x/+y).
RETAIN_RADIAL_GAP = 1.0                    # radial air gap from the rims' OD (no contact)
RETAIN_R_IN    = RIM_OD / 2 + RETAIN_RADIAL_GAP   # just outside the rims' OD
RETAIN_WALL    = STRUCT_WALL               # housing-interface wall (slab flat / tenon neck)
RETAIN_R_OUT   = RETAIN_R_IN + RETAIN_WALL  # 61.5 — slab flat + tenon neck radius
                                            # (housing-facing; the mortise/tenon and the
                                            # housing's retainer-thicken key off this — keep)
# Cage rigidity: the rings and bars are DOUBLE wall thickness radially,
# growing OUTWARD (inward would eat the spool air gap). In the ±X bands
# where the housing sits (|y| ≤ slab half-width) the cage is trimmed back
# flat to RETAIN_R_OUT so the slab/tenon/housing interface is unchanged.
RETAIN_CAGE_WALL  = 2 * STRUCT_WALL                  # 3.2
RETAIN_CAGE_R_OUT = RETAIN_R_IN + RETAIN_CAGE_WALL   # 63.1 — cage outer (away from ±X)

# Lever-side (+X) housing face: the housing's FRONT thicken holds its inner
# face at the spool clearance (RIM_OD/2 + SPOOL_HORIZONTAL_CLEARANCE = 61.4)
# — 0.1 closer to the spool than RETAIN_R_OUT (61.5), which the -X side
# uses. The retainer's +X flat, its +X tenon neck, and the housing's +X
# mortise/thicken all key off THIS value so the joint sits flush.
RETAIN_LEVER_FACE_X = RIM_OD / 2 + SPOOL_HORIZONTAL_CLEARANCE   # 61.4

RETAIN_RING_T  = STRUCT_WALL                # top & bottom rim axial thickness (1.7 mm)
RETAIN_BAR_W   = STRUCT_WALL                # tangential width of each vertical spoke (1.7 mm)

# Vertical placement & height — both pinned by hand (see the two rules below):
#  1. BOTTOM ON THE BRAKE RIM TOP. The cage used to dip a full bottom ring
#     1.6 mm DOWN into the spinning brake rim (z 17.5→19.1) — friction with no
#     function, since the wound cable is held in radially by the rim itself down
#     there. The retainer now sits ENTIRELY above the rim: its bottom edge lands
#     ON RIM_H, so the cage never wraps the rotating band. Only the cable-entry
#     arc drops its window back down to RIM_H so the cable still feeds in at the
#     winding height (see the wedge cut in _build_cable_retainer).
#  2. FIXED 10 mm TALL. Hardcoded (was CABLE_RIM_AIR_GAP + 2 rings = 15.2 mm).
#     This is the NORMAL-section height; the cable-entry arc's opening is one
#     ring-thickness taller because its floor drops that much lower. 10 mm also
#     lands the top ring exactly on the cable top rim base (RIM_H + 10).
RETAIN_Z_LO    = RIM_H                              # bottom edge sits ON the brake rim top
RETAIN_H       = 10.0                               # normal-section height (hardcoded)
RETAIN_Z_HI    = RETAIN_Z_LO + RETAIN_H             # 29.1

# Bar count: smallest EVEN N satisfying the cable-escape constraint (the window
# arc at the inner face must stay < CABLE_D). Even so the pattern can be offset
# by a half pitch to put windows exactly on the ±X axis — those align with the
# housing's retainer mount warts (housing.py), which plug them to lock the
# retainer in place.
_RETAIN_N_MIN  = math.ceil(2 * math.pi * RETAIN_R_IN / (CABLE_D + RETAIN_BAR_W))
RETAIN_N_BARS  = _RETAIN_N_MIN if _RETAIN_N_MIN % 2 == 0 else _RETAIN_N_MIN + 1
RETAIN_BAR_OFFSET_DEG = 360.0 / (2 * RETAIN_N_BARS)   # half-pitch shift → windows on ±X

# Window dimensions (the rectangular hole between two adjacent bars + the two
# rings). Exposed so housing.py can size its mount warts to fit. Tangential
# width is taken at the INNER face — that's the binding constraint for a
# straight rectangular wart (the outer face is ~0.2 mm wider), and matches the
# perpendicular spoke-to-spoke distance the wart must clear.
RETAIN_WINDOW_W    = 2 * math.pi * RETAIN_R_IN / RETAIN_N_BARS - RETAIN_BAR_W
RETAIN_WINDOW_Z_LO = RETAIN_Z_LO + RETAIN_RING_T                                   # 20.7 — top of bottom ring
RETAIN_WINDOW_Z_HI = RETAIN_Z_HI - RETAIN_RING_T                                   # 27.5 — bottom of top ring
RETAIN_WINDOW_H    = RETAIN_WINDOW_Z_HI - RETAIN_WINDOW_Z_LO                       # normal window height


# Tree-taper: each bar stays its nominal RETAIN_BAR_W wide from the top ring
# down to z = RETAIN_WINDOW_Z_LO + RETAIN_TAPER_H, then flares outward
# tangentially at 45° to RETAIN_BAR_TIP_W at z = RETAIN_WINDOW_Z_LO (top of the
# bottom ring). The retainer is printed top-ring-on-bed, so this flare runs in
# the print direction; adjacent bar tips end ~RETAIN_BRIDGE_W apart at the
# bottom-ring level, so the first layer of the bottom ring bridges short
# straight spans (~2 mm) instead of the raw window arc (~5.8 mm at R_IN).
RETAIN_TAPER_H   = 1.9                                   # axial height of the 45° flare
RETAIN_BAR_TIP_W = RETAIN_BAR_W + 2 * RETAIN_TAPER_H     # = 5.5 mm at the bottom-ring top
RETAIN_BRIDGE_W  = (2 * math.pi * RETAIN_R_IN / RETAIN_N_BARS) - RETAIN_BAR_TIP_W  # ~2 mm


# Cable exit channel (+X/+Y quadrant). The cable leaves the spool tangent to
# the +X direction at a Y that varies with how much cable is wound: as the
# spool unwinds from full to empty, the cable feed-off point sweeps from the
# outer winding edge (RIM_OD/2, just inside the cable rim) down to the inner
# spool surface (HUB_OD/2). To clear the cable's full physical extent through
# the cage, every bar whose center sits in the matching angular arc at R_IN
# is omitted. The two rings stay continuous — the cable's z lives strictly
# between them — so the cage stays structurally sound; this region will need
# slicer supports to print, but that's a one-part cost.
RETAIN_CABLE_EXIT_Y_LO_DEG = math.degrees(math.asin((HUB_OD / 2) / RETAIN_R_IN))   # ≈ 25.8°
RETAIN_CABLE_EXIT_Y_HI_DEG = math.degrees(math.asin((RIM_OD / 2) / RETAIN_R_IN))   # ≈ 80.0°


# Dropped-floor angular extent — the cable-entry floor (built at the very end of
# _build_cable_retainer) runs from the opening out to the first PRESENT bar past
# each edge, then across that bar's FULL width to its far outer corner, so the
# strut supports the floor's end (it prints as a low ledge needing support) and
# fuses to it. Bars sit at theta_i = RETAIN_BAR_OFFSET_DEG + i·pitch; those
# inside [LO, HI] are omitted, so the bordering struts are the nearest present
# bars just outside that range.
_FLOOR_PITCH      = 360.0 / RETAIN_N_BARS
_FLOOR_STRUT_HALF = math.degrees(math.atan((RETAIN_BAR_TIP_W / 2)      # ½ the flared
                                           / RETAIN_CAGE_R_OUT))        # tip's angular span
_FLOOR_STRUT_HI   = RETAIN_BAR_OFFSET_DEG + (
    math.floor((RETAIN_CABLE_EXIT_Y_HI_DEG - RETAIN_BAR_OFFSET_DEG) / _FLOOR_PITCH)
    + 1) * _FLOOR_PITCH                                                 # first bar above HI
_FLOOR_STRUT_LO   = RETAIN_BAR_OFFSET_DEG + (
    math.ceil((RETAIN_CABLE_EXIT_Y_LO_DEG - RETAIN_BAR_OFFSET_DEG) / _FLOOR_PITCH)
    - 1) * _FLOOR_PITCH                                                 # first bar below LO
RETAIN_FLOOR_LO_DEG = _FLOOR_STRUT_LO - _FLOOR_STRUT_HALF               # far corner, low side
RETAIN_FLOOR_HI_DEG = _FLOOR_STRUT_HI + _FLOOR_STRUT_HALF               # far corner, high side

# Cable opening (the bottom-ring removal) is bounded by those SAME struts' INNER
# faces, so the entry sits cleanly BETWEEN the two struts. Keying it to the raw
# asin sweep [LO, HI] instead left a ~1° sliver of bottom ring protruding past
# the lower strut into the opening (the sweep edges don't land on a bar).
RETAIN_OPENING_LO_DEG = _FLOOR_STRUT_LO + _FLOOR_STRUT_HALF             # lower strut inner edge
RETAIN_OPENING_HI_DEG = _FLOOR_STRUT_HI - _FLOOR_STRUT_HALF             # upper strut inner edge


# Housing-mount slab: fills the cage in the ±X regions that overlap the 22 mm
# housing face. The cage stays as-is everywhere; in the overlap region the slab
# adds material that:
#   • fills the inter-bar gaps from top to bottom (no axial windows there),
#   • thickens the wall outward so the outer face is FLAT at x = ±RETAIN_R_OUT
#     (tangent to the cage at y=0, where the wall stays the original RETAIN_WALL
#      thick — that's the closest point to the housing — and grows to ~2.7 mm
#      thick at y = ±HOUSING_W/2 to keep the outer face flat).
RETAIN_SLAB_HALF_W = 11.0   # = HOUSING_W/2 in housing.py


def _retainer_slab():
    """Slab built as a single box minus the cage's inner cylinder. The box
    spans both ±X sides at once (x ∈ [-R_OUT, +R_OUT]), bounded tangentially to
    ±RETAIN_SLAB_HALF_W and axially to the full retainer Z; the inner-cylinder
    cut removes the middle, leaving two symmetric slabs — one at +X, one at
    -X. Each slab has a flat outer face at x = ±R_OUT and a curved inner face
    that hugs the cage cylinder, so the slab's wall thickness is RETAIN_WALL
    at y=0 and grows toward ±y."""
    box = (cq.Workplane("XY").workplane(offset=RETAIN_Z_LO)
           .box(2 * RETAIN_R_OUT, 2 * RETAIN_SLAB_HALF_W, RETAIN_H,
                centered=(True, True, False)))
    inside = cyl(2 * RETAIN_R_IN, RETAIN_H + 2 * BOOL_OVERSHOOT,
                 z=RETAIN_Z_LO - BOOL_OVERSHOOT)
    return box.cut(inside)


# Dovetail tenon — slides into the matching mortise in the housing chunk.
# DEPTH is set so the tenon TIP lands exactly on the guide-wheel axle screw
# axis: the screw (along Z) then sits HALF in the tenon and half in the
# housing, pinning the Y-slide the dovetail alone can't resist — so the
# screw double-duties as both wheel axle AND retainer-joint pin, no glue.
#   screw axis radius = RIM_OD/2 + GUIDE_WHEEL_OD/2 − GUIDE_WHEEL_RIM_PRELOAD
#   tenon tip radius  = RETAIN_R_OUT + DEPTH = RIM_OD/2 + RADIAL_GAP + WALL + DEPTH
# setting them equal (both scale with RIM_OD/2, so DEPTH is diameter-invariant):
RETAIN_TENON_DEPTH = ((GUIDE_WHEEL_OD / 2 - GUIDE_WHEEL_RIM_PRELOAD)
                      - RETAIN_RADIAL_GAP - RETAIN_WALL)    # ≈ 4.4
# Tenon z_bot sits STRUCT_WALL above the retainer's own -Z face (= the
# housing chunk top, now aligned with RETAIN_Z_LO via the new
# GUIDE_TO_RETAINER_CLR = STRUCT_WALL). That STRUCT_WALL gap is the housing
# material between the chunk's top face and the retainer's mortise floor.
# Defined as the source of truth here; housing.py imports this same value
# for its mortise z_bot.
RETAIN_TENON_Z_BOT    = RETAIN_Z_LO + STRUCT_WALL
RETAIN_TENON_Z_TOP    = RETAIN_Z_HI
RETAIN_TENON_STOP_W   = STRUCT_WALL         # wall in the housing at -Y end of mortise
RETAIN_TENON_BOT_FLARE = STRUCT_WALL        # 45° bottom flare (Δx = Δz = STRUCT_WALL)


def retainer_tenon_polygon(x_neck, x_tip, z_bot=RETAIN_TENON_Z_BOT,
                            z_top=RETAIN_TENON_Z_TOP):
    """Shared 6-vertex CCW polygon (in X, Z) for the retainer's
    tongue-and-groove tenon AND the housing's matching groove (offset by
    FIT_CLR for fit). Defined here so both sides MUST stay in sync.

    All-square corners (no sloped faces) — the earlier 45° dovetail flare
    printed bloated and required dremel work. A STRUCT_WALL × STRUCT_WALL
    notch at the neck-bottom corner creates a 90° step: the housing's
    groove leaves a matching block of material at the same position, and
    that block sits in the tongue's notch. Pulling the retainer in -X
    jams the step against the housing block — the joint is locked along
    X without any sloped print faces.

    Print orientation: retainer prints +Z face down. The notch is at
    z_bot (last few layers), and the step's "ceiling" face is supported
    by the layers printed before it."""
    step_x = STRUCT_WALL
    step_z = STRUCT_WALL
    return [
        (x_neck,          z_top),            # V1: neck-top
        (x_neck,          z_bot + step_z),   # V2: notch outer side
        (x_neck + step_x, z_bot + step_z),   # V3: notch inner corner (the step)
        (x_neck + step_x, z_bot),            # V4: notch outer bottom
        (x_tip,           z_bot),            # V5: tip-bottom
        (x_tip,           z_top),            # V6: tip-top
    ]


def _retainer_tenon():
    """Dovetail tenons on both ±X sides of the slab. Uses the shared
    retainer_tenon_polygon (above). Both share z_bot = RETAIN_TENON_Z_BOT.
    The necks start at each side's actual housing face — RETAIN_R_OUT on
    -X, RETAIN_LEVER_FACE_X (0.1 closer to the spool) on +X — while the
    tips share the same absolute depth, so the +X tenon is 0.1 longer and
    meets its slab face with no gap."""
    def _at(x_neck):
        x_tip = RETAIN_R_OUT + RETAIN_TENON_DEPTH
        pts = retainer_tenon_polygon(x_neck, x_tip, z_bot=RETAIN_TENON_Z_BOT,
                                      z_top=RETAIN_TENON_Z_TOP)
        # No FIT_CLR on -Y: tenon seats flush against the stop wall.
        y_lo = -RETAIN_SLAB_HALF_W + RETAIN_TENON_STOP_W
        y_hi = +RETAIN_SLAB_HALF_W
        return (cq.Workplane("XZ").workplane(offset=-y_lo)
                .polyline(pts).close()
                .extrude(-(y_hi - y_lo)))

    tenon_plus  = _at(RETAIN_LEVER_FACE_X)
    # Mirror in X (across YZ plane) keeps the -Y stop wall on the same side
    # for both tenons (a 180° Z-rotation would flip Y too).
    tenon_minus = _at(RETAIN_R_OUT).mirror("YZ")
    return tenon_plus.union(tenon_minus)


def _build_cable_retainer():
    # Top & bottom rings tie the bars together (double-thick, see
    # RETAIN_CAGE_WALL above).
    def _ring(z_lo):
        return (cyl(2 * RETAIN_CAGE_R_OUT, RETAIN_RING_T, z=z_lo)
                .cut(cyl(2 * RETAIN_R_IN, RETAIN_RING_T, z=z_lo)))
    cage = _ring(RETAIN_Z_LO).union(_ring(RETAIN_Z_HI - RETAIN_RING_T))

    # Cable-entry drop: over the +X/+Y exit arc the cable feeds out at the
    # winding height, so the cage floor there must sit FLUSH WITH THE SPOOL
    # FLOOR (its top on RIM_H) — one ring-thickness LOWER than the normal bottom
    # ring, whose bottom sits on RIM_H to clear the spinning brake rim. So over
    # the exit arc we (a) clear the normal bottom ring and (b) drop in a
    # replacement floor whose TOP lands on RIM_H. The dropped floor spans a few
    # degrees WIDER than the opening so it underlaps the normal ring on both
    # sides (sharing a z=RIM_H face, not a bare edge) and stays fused to the cage.
    def _arc_wedge(lo_deg, hi_deg, z_lo, h):
        lo, hi = math.radians(lo_deg), math.radians(hi_deg)
        mid = (lo + hi) / 2
        Rw  = RETAIN_CAGE_R_OUT + 1.0                      # past the cage's outer face
        return (cq.Workplane("XY").workplane(offset=z_lo)
                .moveTo(0, 0)
                .lineTo(Rw * math.cos(lo), Rw * math.sin(lo))
                .threePointArc((Rw * math.cos(mid), Rw * math.sin(mid)),
                               (Rw * math.cos(hi),  Rw * math.sin(hi)))
                .close()
                .extrude(h))

    # (a) clear the normal bottom ring over the opening, edge-aligned to the two
    # bounding struts (RETAIN_OPENING_LO/HI_DEG) so no ring sliver protrudes into
    # the cable entry. The replacement dropped floor (b) is added at the very END
    # of the build — it meets the normal ring on a coincident z=RIM_H face, which
    # the bar/slab/tenon fuses below would silently drop if it were added now, so
    # it goes in last (see end of fn).
    cage = cage.cut(_arc_wedge(RETAIN_OPENING_LO_DEG, RETAIN_OPENING_HI_DEG,
                               RETAIN_Z_LO - BOOL_OVERSHOOT,
                               RETAIN_RING_T + 2 * BOOL_OVERSHOOT))

    # Tree-tapered bars (see RETAIN_TAPER_H above): straight RETAIN_BAR_W wide
    # from the top ring down to z_flare_top, then 45° flare to RETAIN_BAR_TIP_W
    # at the top of the bottom ring. CCW polygon in (Y, Z) — workplane YZ has
    # xDir=+Y, yDir=+Z — extruded radially in +X by RETAIN_WALL.
    y_top       = RETAIN_BAR_W / 2
    y_bot       = RETAIN_BAR_TIP_W / 2
    z_flare_top = RETAIN_WINDOW_Z_LO + RETAIN_TAPER_H   # start of flare going -Z (22.6)
    z_flare_bot = RETAIN_WINDOW_Z_LO                     # end of flare = top of bottom ring (20.7)
    bar_pts = [
        (-y_top, RETAIN_Z_HI),       # neck top-left (inside top ring)
        (-y_top, z_flare_top),       # neck bottom-left (start of flare)
        (-y_bot, z_flare_bot),       # tip top-left  (end of 45° flare)
        (-y_bot, RETAIN_Z_LO),       # tip bottom-left (inside bottom ring)
        ( y_bot, RETAIN_Z_LO),       # tip bottom-right
        ( y_bot, z_flare_bot),       # tip top-right
        ( y_top, z_flare_top),       # neck bottom-right
        ( y_top, RETAIN_Z_HI),       # neck top-right
    ]
    for i in range(RETAIN_N_BARS):
        theta = RETAIN_BAR_OFFSET_DEG + i * 360.0 / RETAIN_N_BARS
        # Skip bars in the cable exit channel (+X/+Y quadrant only).
        if RETAIN_CABLE_EXIT_Y_LO_DEG <= theta <= RETAIN_CABLE_EXIT_Y_HI_DEG:
            continue
        bar = (cq.Workplane("YZ").workplane(offset=RETAIN_R_IN)
               .polyline(bar_pts).close()
               .extrude(RETAIN_CAGE_WALL)
               .rotate((0, 0, 0), (0, 0, 1), theta))
        cage = cage.union(bar)

    # Trim the retainer FLAT on both ±X sides with full YZ-plane cuts. The
    # retainer slides into the housing along Y, so its whole profile (at
    # EVERY y, not just the slab band) must fit through the housing
    # channel. The two sides have DIFFERENT channel extents (derived from
    # different housing constants — keep both in sync with housing.py):
    #   -X (non-lever): the back-side mortise/thicken keys off RETAIN_R_OUT
    #       (= RIM_OD/2 + RETAIN_RADIAL_GAP + RETAIN_WALL = 61.5)
    #   +X (lever): the front-thicken inner face is held at the spool
    #       clearance instead (= RIM_OD/2 + SPOOL_HORIZONTAL_CLEARANCE =
    #       61.4) — 0.1 tighter, so the +X cut must also shave the SLAB
    #       (applied AFTER the slab union; tenons are added last, so they
    #       stay intact either way).
    flat_x_minus = -RETAIN_R_OUT                                   # -61.5
    flat_x_plus  = RETAIN_LEVER_FACE_X                             # +61.4
    def _flat_trim(x_plane, sign):
        return (cq.Workplane("XY")
                .workplane(offset=RETAIN_Z_LO - BOOL_OVERSHOOT)
                .center(x_plane + sign * 50.0, 0)
                .box(100.0, 2 * (RETAIN_CAGE_R_OUT + 5.0),
                     RETAIN_H + 2 * BOOL_OVERSHOOT,
                     centered=(True, True, False)))
    cage = cage.cut(_flat_trim(flat_x_minus, -1))

    # Housing-mount slab (then the +X flat shaves cage AND slab together).
    cage = cage.union(_retainer_slab())
    cage = cage.cut(_flat_trim(flat_x_plus, +1))

    # Dovetail tenons last — they protrude past both flats by design.
    cage = cage.union(_retainer_tenon())

    # (b) Dropped cable-entry floor — added LAST (see note at cut (a)). A ring
    # segment whose TOP sits flush with the spool floor (RIM_H), one ring-
    # thickness below the normal bottom ring. It runs from the opening out to the
    # far corner of the first present bar on each side (RETAIN_FLOOR_LO/HI_DEG),
    # so each end lands on a strut that supports and fuses it. Goes in after
    # every other fuse so the thin coincident-face join isn't discarded.
    dropped = _ring(RIM_H - RETAIN_RING_T).intersect(
        _arc_wedge(RETAIN_FLOOR_LO_DEG, RETAIN_FLOOR_HI_DEG,
                   RIM_H - RETAIN_RING_T - BOOL_OVERSHOOT,
                   RETAIN_RING_T + 2 * BOOL_OVERSHOOT))
    cage = cage.union(dropped)
    return cage


cable_retainer = _build_cable_retainer()
