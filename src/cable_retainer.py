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

from .spool import RIM_OD, RIM_H, CABLE_D
from .dimensions import CABLE_RIM_AIR_GAP, STRUCT_WALL, FIT_CLR, BOOL_OVERSHOOT, HUB_OD
from .helpers import cyl


# The cage now sits OUTSIDE the spool rims (larger diameter) and surrounds them
# with a small radial air gap, so it never touches the rotating spool. Its
# ring-gap lines up with the cable air gap between the bottom (brake) rim and the
# cable top rim — the cable passes through there (radially, at +x/+y).
RETAIN_RADIAL_GAP = 1.0                    # radial air gap from the rims' OD (no contact)
RETAIN_R_IN    = RIM_OD / 2 + RETAIN_RADIAL_GAP   # just outside the rims' OD
RETAIN_WALL    = STRUCT_WALL               # radial thickness of the cage (1.7 mm)
RETAIN_R_OUT   = RETAIN_R_IN + RETAIN_WALL

RETAIN_RING_T  = STRUCT_WALL                # top & bottom rim axial thickness (1.7 mm)
RETAIN_BAR_W   = STRUCT_WALL                # tangential width of each vertical spoke (1.7 mm)

# Ring-gap == the cable air gap (brake rim top → cable top rim bottom), so a
# coil can't escape there except where it's meant to pass. Tracks CABLE_RIM_AIR_GAP.
_GAP_LO        = RIM_H                              # brake rim top  = gap bottom
_GAP_HI        = RIM_H + CABLE_RIM_AIR_GAP          # cable top rim bottom = gap top
RETAIN_Z_LO    = _GAP_LO - RETAIN_RING_T            # bottom ring overlaps the brake rim
RETAIN_Z_HI    = _GAP_HI + RETAIN_RING_T            # top ring overlaps the cable top rim
RETAIN_H       = RETAIN_Z_HI - RETAIN_Z_LO          # gap between rings == CABLE_RIM_AIR_GAP

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
RETAIN_WINDOW_Z_LO = _GAP_LO                                                       # top of bottom rim
RETAIN_WINDOW_Z_HI = _GAP_HI                                                       # bottom of top rim
RETAIN_WINDOW_H    = RETAIN_WINDOW_Z_HI - RETAIN_WINDOW_Z_LO                       # = CABLE_RIM_AIR_GAP


# Tree-taper: each bar stays its nominal RETAIN_BAR_W wide from the top ring
# down to z = _GAP_LO + RETAIN_TAPER_H, then flares outward tangentially at
# 45° to RETAIN_BAR_TIP_W at z = _GAP_LO (top of the bottom ring). The retainer
# is printed top-ring-on-bed, so this flare runs in the print direction;
# adjacent bar tips end ~RETAIN_BRIDGE_W apart at the bottom-ring level, so the
# first layer of the bottom ring bridges short straight spans (~2 mm) instead
# of the raw window arc (~5.8 mm at R_IN).
RETAIN_TAPER_H   = 1.9                                   # axial height of the 45° flare
RETAIN_BAR_TIP_W = RETAIN_BAR_W + 2 * RETAIN_TAPER_H     # = 5.5 mm at z = _GAP_LO
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
RETAIN_CABLE_EXIT_Y_LO_DEG = math.degrees(math.asin((HUB_OD / 2) / RETAIN_R_IN))   # ≈ 28.7°
RETAIN_CABLE_EXIT_Y_HI_DEG = math.degrees(math.asin((RIM_OD / 2) / RETAIN_R_IN))   # ≈ 73.3°


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
# X depth must match SPINE_X_INNER - RETAIN_R_OUT in housing.py (the chunk
# depth). Z bottom must match GUIDE_POCKET_Z_HI in housing.py (top of the
# guide-wheel pocket; going below would clash with the wheel).
RETAIN_TENON_DEPTH    = 3.44                # X depth, must match housing.py chunk depth
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
    retainer_tenon_polygon (above). The -X (guide-wheel side) and +X
    (lever side) tenons now share the same z_bot (= RETAIN_TENON_Z_BOT);
    the old lever-side lowering was for a previous chunk layout that
    didn't span the full chunk volume."""
    def _at(z_bot):
        x_neck = RETAIN_R_OUT
        x_tip  = x_neck + RETAIN_TENON_DEPTH
        pts = retainer_tenon_polygon(x_neck, x_tip, z_bot=z_bot,
                                      z_top=RETAIN_TENON_Z_TOP)
        # No FIT_CLR on -Y: tenon seats flush against the stop wall.
        y_lo = -RETAIN_SLAB_HALF_W + RETAIN_TENON_STOP_W
        y_hi = +RETAIN_SLAB_HALF_W
        return (cq.Workplane("XZ").workplane(offset=-y_lo)
                .polyline(pts).close()
                .extrude(-(y_hi - y_lo)))

    tenon_plus  = _at(RETAIN_TENON_Z_BOT)
    # Mirror in X (across YZ plane) keeps the -Y stop wall on the same side
    # for both tenons (a 180° Z-rotation would flip Y too).
    tenon_minus = _at(RETAIN_TENON_Z_BOT).mirror("YZ")
    return tenon_plus.union(tenon_minus)


def _build_cable_retainer():
    # Top & bottom rings tie the bars together.
    def _ring(z_lo):
        return (cyl(2 * RETAIN_R_OUT, RETAIN_RING_T, z=z_lo)
                .cut(cyl(2 * RETAIN_R_IN, RETAIN_RING_T, z=z_lo)))
    cage = _ring(RETAIN_Z_LO).union(_ring(RETAIN_Z_HI - RETAIN_RING_T))

    # Tree-tapered bars (see RETAIN_TAPER_H above): straight RETAIN_BAR_W wide
    # from the top ring down to z_flare_top, then 45° flare to RETAIN_BAR_TIP_W
    # at the top of the bottom ring. CCW polygon in (Y, Z) — workplane YZ has
    # xDir=+Y, yDir=+Z — extruded radially in +X by RETAIN_WALL.
    y_top       = RETAIN_BAR_W / 2
    y_bot       = RETAIN_BAR_TIP_W / 2
    z_flare_top = _GAP_LO + RETAIN_TAPER_H        # start of flare going -Z (15.9)
    z_flare_bot = _GAP_LO                          # end of flare = top of bottom ring (14)
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
               .extrude(RETAIN_WALL)
               .rotate((0, 0, 0), (0, 0, 1), theta))
        cage = cage.union(bar)

    # Housing-mount slab + dovetail tenons.
    cage = cage.union(_retainer_slab())
    cage = cage.union(_retainer_tenon())
    return cage


cable_retainer = _build_cable_retainer()
