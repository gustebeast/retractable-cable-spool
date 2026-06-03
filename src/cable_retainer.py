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
from .dimensions import CABLE_RIM_AIR_GAP, STRUCT_WALL, FIT_CLR, BOOL_OVERSHOOT
from .helpers import cyl


# The cage now sits OUTSIDE the spool rims (larger diameter) and surrounds them
# with a small radial air gap, so it never touches the rotating spool. Its
# ring-gap lines up with the cable air gap between the bottom (brake) rim and the
# cable top rim — the cable passes through there (radially, at +x/+y).
RETAIN_RADIAL_GAP = 2.5                    # radial air gap from the rims' OD (no contact)
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
RETAIN_TENON_Z_BOT    = 12.3                # = GUIDE_POCKET_Z_HI in housing.py
RETAIN_TENON_STOP_W   = STRUCT_WALL         # 1.7 mm wall in the housing at -Y end of mortise


def _retainer_tenon():
    """Hexagonal dovetail tenons on both ±X sides of the slab. Cross-section
    in X-Z: wider TIP on the housing side (high X, full Z extent) and
    narrower NECK on the retainer side (low X). Flares are on the LEFT
    (neck) corners:
      • Upper-left: 45° flare chamfering the neck-top corner (Δx = Δz = 1.7mm).
        Top flat fills the rest of the X span at z = z_top (the print bed).
      • Lower-left: 30°-from-horizontal flare chamfering the neck-bottom
        corner (Δz = 1.7mm, Δx = 1.7/tan30° ≈ 2.95mm). Bottom flat takes the
        remaining ~0.5mm at z = z_bot.
    Locks against -X pull (tenon retreating from the mortise) — the wider tip
    can't pass through the narrower mortise opening at the retainer side.
    Extruded in Y from -9.15 to +11."""
    x_neck = RETAIN_R_OUT                                       # 61.4
    x_tip  = x_neck + RETAIN_TENON_DEPTH                        # 64.84
    top_flare_x = STRUCT_WALL                                    # 1.7 (45°: Δx=Δz)
    bot_flare_x = STRUCT_WALL / math.tan(math.radians(30))       # 2.945 (30° from horizontal — shallow)
    z_top = RETAIN_Z_HI                                          # 23.7
    z_bot = RETAIN_TENON_Z_BOT                                   # 12.3
    z_top_flare = z_top - STRUCT_WALL                            # 22.0
    z_bot_flare = z_bot + STRUCT_WALL                            # 14.0
    # Hexagon CCW from neck-bottom-flare-top:
    pts = [
        (x_neck,               z_bot_flare),    # V1: top of 30° lower flare at neck
        (x_neck + bot_flare_x, z_bot),          # V2: end of 30° flare / start of bottom flat
        (x_tip,                z_bot),          # V3: tip-bottom (wider)
        (x_tip,                z_top),          # V4: tip-top
        (x_tip - top_flare_x,  z_top),          # V5: end of top flat / start of 45° flare
        (x_neck,               z_top_flare),    # V6: bottom of 45° upper flare at neck
    ]
    # No FIT_CLR here: tenon -Y face seats flush against the stop wall (running
    # clearance is taken on the other faces by the mortise offset). +0.15 mm
    # of tenon material vs. a clearance gap means the seated position is
    # positively defined by the wall, not by sloppy slide depth.
    y_lo = -RETAIN_SLAB_HALF_W + RETAIN_TENON_STOP_W              # -9.3
    y_hi = +RETAIN_SLAB_HALF_W                                     # +11
    tenon_plus = (cq.Workplane("XZ").workplane(offset=y_lo)
                  .polyline(pts).close()
                  .extrude(y_hi - y_lo))
    # Mirror in X (across YZ plane) so the -X tenon keeps the same Y orientation
    # (stop wall at -Y on both sides). A 180° Z-rotation would flip Y too,
    # putting the -X stop wall on +Y — not what we want.
    tenon_minus = tenon_plus.mirror("YZ")
    return tenon_plus.union(tenon_minus)


def _build_cable_retainer():
    # Top & bottom rings tie the bars together.
    def _ring(z_lo):
        return (cyl(2 * RETAIN_R_OUT, RETAIN_RING_T, z=z_lo)
                .cut(cyl(2 * RETAIN_R_IN, RETAIN_RING_T, z=z_lo)))
    cage = _ring(RETAIN_Z_LO).union(_ring(RETAIN_Z_HI - RETAIN_RING_T))

    # Vertical bars (plain rectangles).
    r_mid = (RETAIN_R_IN + RETAIN_R_OUT) / 2
    for i in range(RETAIN_N_BARS):
        bar = (cq.Workplane("XY").workplane(offset=RETAIN_Z_LO)
               .center(r_mid, 0)
               .box(RETAIN_WALL, RETAIN_BAR_W, RETAIN_H, centered=(True, True, False))
               .rotate((0, 0, 0), (0, 0, 1),
                       RETAIN_BAR_OFFSET_DEG + i * 360.0 / RETAIN_N_BARS))
        cage = cage.union(bar)

    # Housing-mount slab + dovetail tenons.
    cage = cage.union(_retainer_slab())
    cage = cage.union(_retainer_tenon())
    return cage


cable_retainer = _build_cable_retainer()
