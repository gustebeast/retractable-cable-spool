"""Printed guide wheel + sandwich-mount factory.

Replaces the 608 guide bearings (one above the spool flange, one below)
with custom-printed wheels riding on M2 cap screws. Each wheel is
captured between two flat rectangular sandwich slabs that are PRINTED
AS PART OF THE HOUSING. The M2 passes along X through both slab regions
and the wheel hub. No heat-set insert — the M2 threads into a tight-fit
Ø GUIDE_AXLE_SHAFT_D (2.2 mm) hole in printed PA6-GF on the far side
of the wheel.

Stepped M2 hole layout along X (single bore through both slabs + wheel):
  [head access + pocket] Ø MOUNT_HEAD_HOLE_D from the housing's +X
                         outer face inward to the head step
  [tight-fit shaft]      Ø GUIDE_AXLE_SHAFT_D from the head step
                         through outboard slab, outboard hub, wheel
                         bore, inboard hub, and GUIDE_AXLE_FAR_SCREW_DEPTH
                         + GUIDE_AXLE_FAR_HOLE_EXTRA into the inboard
                         slab — total 3 mm of enclosed axle space
                         past the wheel, of which the M2 screw uses
                         the first 2 mm (1 mm hub + 1 mm slab).

Slab dimensions (the rectangular footprint that defines the M2-hole
neighborhood):
  X thickness  : MOUNT_T  = 4 mm  (along wheel axis)
  Y width      : MOUNT_W  = 6 mm
  Z length     : MOUNT_L  = 9.5 mm  (housing surface → 1 mm past hole)

Wheel friction reduction:
  Each slab inner face carries a Ø HUB_OD × HUB_H raised boss centered
  on the M2 hole — printed as part of the housing. The boss is the only
  feature that contacts the wheel face during spin, concentrating axial-
  thrust contact at a small radius (r_eff ~1.7 mm vs ~3.2 mm for the
  full-face contact). Wheel itself stays a plain disc — prints flat-
  side-down without any small features on the build plate. Recommend a
  dab of silicone grease on assembly for further reduction.
"""

import cadquery as cq

from .dimensions import (
    M2_HEAD_RECESS_D,
)

# ── Wheel ───────────────────────────────────────────────────────────────────
WHEEL_OD              = 9.0    # printed OD (rolling-face envelope)
WHEEL_PAINTED_OD      = 10.0   # nominal OD after rubber paint coat
WHEEL_W               = 6.0    # rolling-face width along X (printed)
WHEEL_BORE_D          = 2.6    # generous spinning clearance on M2 (Ø2.0)

# Friction-reducing hub boss — built INTO the housing on each slab inner
# face (not on the wheel). The wheel stays a plain disc, printable flat-
# side-down without any features on the build plate.
HUB_OD                = 4.0    # boss OD (friction contact ring)
HUB_H                 = 1.0    # axial protrusion from slab inner face

# Axial clearance per side between hub tip face and wheel face — lets the
# wheel spin without binding.
HUB_AXIAL_CLR         = 0.2

# ── Sandwich slab (merged with housing) ─────────────────────────────────────
MOUNT_T               = 4.0    # thickness along X
MOUNT_W               = 6.0    # width along Y
MOUNT_L               = 9.5    # length along Z
MOUNT_OUTER_T         = 2.0    # outer 2 mm: head/insert pocket
MOUNT_INNER_T         = 2.0    # inner 2 mm: Ø M2_SHAFT_CLR_D shaft clearance
assert MOUNT_OUTER_T + MOUNT_INNER_T == MOUNT_T

MOUNT_HOUSING_END_DIST = 6.5   # housing-end of slab → M2 hole center along Z
                               # (= 2 mm air gap + 4.5 mm wheel printed radius)
MOUNT_FAR_END_DIST    = 3.0    # M2 hole center → far end of slab along Z
                               # (= 2 mm hole radius + 1 mm extra)
assert MOUNT_HOUSING_END_DIST + MOUNT_FAR_END_DIST == MOUNT_L

MOUNT_HEAD_HOLE_D     = M2_HEAD_RECESS_D     # 4.1 — head clearance

# ── Guide-wheel M2 axle (no heat-set insert) ────────────────────────────────
# The M2 screw threads directly into a tight-fit Ø2.2 hole in printed
# PA6-GF on the far side of the wheel — exception to the M2_SHAFT_CLR_D
# (=2.3) used elsewhere in the project, which assumes either a heat-set
# insert or a sliding shaft.
GUIDE_AXLE_SHAFT_D         = 2.2     # tight-fit thread hole (NOT M2_SHAFT_CLR_D)
GUIDE_AXLE_FAR_SCREW_DEPTH = 1.0     # screw tip lands this far past inboard slab inner face
GUIDE_AXLE_FAR_HOLE_EXTRA  = 1.0     # hole extends this far PAST the tip for clearance
GUIDE_AXLE_SCREW_LEN       = 20.0    # M2 cap screw, head-to-tip length (M2×20)

# Derived: distance the head pocket's +X face is shifted past the outboard
# slab's outer face. Picked so the M2 screw (length GUIDE_AXLE_SCREW_LEN)
# bottoms out at the head step (Ø MOUNT_HEAD_HOLE_D → Ø GUIDE_AXLE_SHAFT_D
# transition) with its tip GUIDE_AXLE_FAR_SCREW_DEPTH past the inboard
# slab inner face. axle_access_channel reads this constant.
#
# Algebra (all x_center terms cancel):
#   head_step_x = (inboard_inner_x - GUIDE_AXLE_FAR_SCREW_DEPTH) + GUIDE_AXLE_SCREW_LEN
#   head_pocket_outer_x = head_step_x + MOUNT_OUTER_T
#   EXTRA_HEAD_RECESS = head_pocket_outer_x - outboard_outer_x
#                     = GUIDE_AXLE_SCREW_LEN - GUIDE_AXLE_FAR_SCREW_DEPTH
#                       - (WHEEL_W + 2·HUB_H + 2·HUB_AXIAL_CLR)
#                       - MOUNT_T + MOUNT_OUTER_T
#                     = 20 - 1 - 8.4 - 4 + 2 = 8.6 mm
EXTRA_HEAD_RECESS = (
    GUIDE_AXLE_SCREW_LEN
    - GUIDE_AXLE_FAR_SCREW_DEPTH
    - (WHEEL_W + 2 * HUB_H + 2 * HUB_AXIAL_CLR)
    - MOUNT_T
    + MOUNT_OUTER_T
)


# ── Wheel pocket sizing ─────────────────────────────────────────────────────
# Pocket only carves the WHEEL region — slab X regions remain solid plate.
WHEEL_POCKET_X_CLR    = HUB_AXIAL_CLR        # 0.2 mm clearance at each hub face
WHEEL_POCKET_Y_CLR    = 1.5                  # Y clearance to painted OD per side
WHEEL_POCKET_Z_CLR    = 0.5                  # Z clearance to painted OD per side


# ────────────────────────────────────────────────────────────────────────────
# Builders
# ────────────────────────────────────────────────────────────────────────────

def wheel_solid(x_center: float, y_center: float, z_center: float) -> cq.Workplane:
    """Plain printed guide wheel — Ø WHEEL_OD × WHEEL_W disc with bore.
    No raised features on the X faces; prints flat-side-down."""
    body = cq.Workplane("XY").add(cq.Solid.makeCylinder(
        WHEEL_OD / 2, WHEEL_W,
        pnt=cq.Vector(x_center - WHEEL_W / 2, y_center, z_center),
        dir=cq.Vector(1, 0, 0),
    ))
    bore = cq.Workplane("XY").add(cq.Solid.makeCylinder(
        WHEEL_BORE_D / 2, WHEEL_W + 0.4,
        pnt=cq.Vector(x_center - WHEEL_W / 2 - 0.2, y_center, z_center),
        dir=cq.Vector(1, 0, 0),
    ))
    return body.cut(bore)


# Geometry helper — computes per-side slab inner-face X positions given the
# wheel center. The slab inner face is offset from the wheel face by the
# hub protrusion + axial clearance.
def slab_inner_faces(x_center: float) -> tuple[float, float]:
    """Return (inboard, outboard) slab inner-face X coords."""
    half = WHEEL_W / 2 + HUB_H + HUB_AXIAL_CLR
    return x_center - half, x_center + half


def slab_fingers(x_center: float, y_center: float, z_center: float, *,
                 housing_z_sign: int) -> cq.Workplane:
    """Two rectangular slab fingers (4 × 6 × 9.5 mm each) protruding from
    the housing into the wheel-pocket region — the M2 axle support
    structure. Unioned into the housing so they print as part of it.

    On the pancake side (housing_z_sign=+1) the fingers fully overlap
    existing plate material (since the plate's Z range covers the slab
    Z range), so the union has no visible effect — the M2 hole simply
    drills through the existing plate. On the lever side
    (housing_z_sign=-1) the fingers stick UP from the plate top into
    the gap area — this is where they're structurally necessary.

    Each finger is rooted in the plate by MOUNT_HOUSING_END_DIST -
    HOUSING_GAP_LEVER mm of overlap on the lever side; on the pancake
    side the entire finger overlaps the plate.
    """
    inboard_inner_x, outboard_inner_x = slab_inner_faces(x_center)
    inboard_outer_x  = inboard_inner_x  - MOUNT_T
    outboard_outer_x = outboard_inner_x + MOUNT_T

    if housing_z_sign > 0:
        z_min = z_center - MOUNT_FAR_END_DIST          # 53 on pancake
        z_max = z_center + MOUNT_HOUSING_END_DIST      # 62.5 on pancake
    else:
        z_min = z_center - MOUNT_HOUSING_END_DIST      # -11.5 on lever
        z_max = z_center + MOUNT_FAR_END_DIST          # -2 on lever

    def _finger(x_min):
        return (
            cq.Workplane("XY")
            .box(MOUNT_T, MOUNT_W, z_max - z_min, centered=False)
            .translate((x_min, y_center - MOUNT_W / 2, z_min))
        )
    return _finger(inboard_outer_x).union(_finger(outboard_inner_x))


def slab_hubs(x_center: float, y_center: float, z_center: float) -> cq.Workplane:
    """Two small Ø HUB_OD × HUB_H bosses extending from each slab inner face
    toward the wheel. Unioned into the housing so they print as part of it.
    The bosses are bored through by the M2 stepped hole afterward."""
    inboard_inner_x, outboard_inner_x = slab_inner_faces(x_center)
    # Inboard hub: protrudes in +X (toward wheel) from x = inboard_inner_x.
    inboard_hub = cq.Workplane("XY").add(cq.Solid.makeCylinder(
        HUB_OD / 2, HUB_H,
        pnt=cq.Vector(inboard_inner_x, y_center, z_center),
        dir=cq.Vector(1, 0, 0),
    ))
    # Outboard hub: protrudes in -X from x = outboard_inner_x.
    outboard_hub = cq.Workplane("XY").add(cq.Solid.makeCylinder(
        HUB_OD / 2, HUB_H,
        pnt=cq.Vector(outboard_inner_x - HUB_H, y_center, z_center),
        dir=cq.Vector(1, 0, 0),
    ))
    return inboard_hub.union(outboard_hub)


def wheel_pocket_cut(x_center: float, y_center: float, z_center: float, *,
                     housing_z_sign: int, extra_z: float = 0.0) -> cq.Workplane:
    """Pocket carved into the housing for the wheel only. X extent matches
    wheel + hub envelope + axial clearance; YZ extent matches painted OD
    + clearance, but stretched in the housing-end Z direction up to the
    slab housing-end so the slab's housing-end face is the pocket roof.

    extra_z extends the pocket's outer-Z extent by that many mm (in the
    housing_z_sign direction). Use this to extend the pocket through the
    mount-bracket's +Y chamfer wedge (so the bracket's small 45° chamfer
    is cut away where it would otherwise pass over the wheel pocket)."""
    inboard_x, outboard_x = slab_inner_faces(x_center)
    half_y = WHEEL_PAINTED_OD / 2 + WHEEL_POCKET_Y_CLR
    half_z = WHEEL_PAINTED_OD / 2 + WHEEL_POCKET_Z_CLR
    if housing_z_sign > 0:
        # Pancake: housing above. Pocket stretches up to the slab housing-end.
        z_min = z_center - half_z
        z_max = z_center + MOUNT_HOUSING_END_DIST + extra_z
    else:
        z_min = z_center - MOUNT_HOUSING_END_DIST - extra_z
        z_max = z_center + half_z
    return (
        cq.Workplane("XY")
        .box(outboard_x - inboard_x,
             half_y * 2,
             z_max - z_min,
             centered=False)
        .translate((inboard_x, y_center - half_y, z_min))
    )


def axle_access_channel(y_center: float, z_center: float,
                        x_start: float, x_end: float) -> cq.Workplane:
    """Ø MOUNT_HEAD_HOLE_D bore from (x_start + EXTRA_HEAD_RECESS) to
    x_end along +X — carries the M2 head body + driver from the housing's
    +X face down to where the head bottoms out. The first
    EXTRA_HEAD_RECESS mm just outboard of the slab is INTENTIONALLY left
    solid (apart from the Ø2.3 shaft hole bored by m2_hole_cut); that
    solid step is where the screw head seats. Callers pass the outboard
    slab's outer face as x_start — the EXTRA_HEAD_RECESS offset is
    applied internally so callers don't change.

    Must be cut LAST in the housing assembly so subsequent unions don't
    fill it back in."""
    eps = 0.05
    x_start_eff = x_start + EXTRA_HEAD_RECESS
    return cq.Workplane("XY").add(cq.Solid.makeCylinder(
        MOUNT_HEAD_HOLE_D / 2,
        (x_end - x_start_eff) + 2 * eps,
        pnt=cq.Vector(x_start_eff - eps, y_center, z_center),
        dir=cq.Vector(1, 0, 0),
    ))


def m2_hole_cut(x_center: float, y_center: float, z_center: float) -> cq.Workplane:
    """Stepped M2 axle hole through both slab regions, head at the +X
    (outboard) end. Returns the full cutter.

    Two diameters:
      • Ø MOUNT_HEAD_HOLE_D (4.1 mm) head pocket from the head step
        outward — joined by axle_access_channel into a single Ø4.1 bore
        out to the housing's +X face.
      • Ø GUIDE_AXLE_SHAFT_D (2.2 mm) tight-fit shaft from the head step
        through outboard slab + hub + wheel + hub + GUIDE_AXLE_FAR_SCREW_DEPTH
        of inboard slab, plus GUIDE_AXLE_FAR_HOLE_EXTRA past the screw
        tip for clearance.

    No heat-set insert — the screw threads engage the printed PA6-GF
    Ø2.2 hole directly on the far side."""
    inboard_inner_x, outboard_inner_x = slab_inner_faces(x_center)

    tip_x          = inboard_inner_x - GUIDE_AXLE_FAR_SCREW_DEPTH
    hole_far_end_x = tip_x - GUIDE_AXLE_FAR_HOLE_EXTRA
    head_step_x    = tip_x + GUIDE_AXLE_SCREW_LEN
    head_pocket_outer_x = head_step_x + MOUNT_OUTER_T

    eps = 0.05  # boolean overshoot

    def _x_cyl(d, x_min, x_max):
        length = (x_max - x_min) + 2 * eps
        return cq.Workplane("XY").add(cq.Solid.makeCylinder(
            d / 2, length,
            pnt=cq.Vector(x_min - eps, y_center, z_center),
            dir=cq.Vector(1, 0, 0),
        ))

    shaft = _x_cyl(GUIDE_AXLE_SHAFT_D, hole_far_end_x, head_step_x)
    head_pocket = _x_cyl(MOUNT_HEAD_HOLE_D, head_step_x, head_pocket_outer_x)
    return shaft.union(head_pocket)
