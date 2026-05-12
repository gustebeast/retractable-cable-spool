"""Printed guide wheel + sandwich-mount factory.

Replaces the 608 guide bearings (one above the spool flange, one below)
with custom-printed wheels riding on M2 cap screws. Each wheel is
captured between two flat rectangular sandwich slabs that are PRINTED
AS PART OF THE HOUSING. The M2 passes along X through both slab regions
and the wheel hub.

Per-side housing cut layout (merged-with-housing):

  Wheel pocket: rectangular box covering the wheel envelope only,
    cut into the plate so the wheel can spin in place. Slabs sit at
    its ±X edges (made of plate material).

  Stepped M2 hole along X (shared between both slabs + wheel pocket):
    [head pocket]   Ø MOUNT_HEAD_HOLE_D, depth MOUNT_OUTER_T (= 2 mm)
                    on the inboard slab's outer face
    [shaft clear]   Ø M2_SHAFT_CLR_D through the rest of inboard slab,
                    inner MOUNT_INNER_T mm
    (wheel pocket — the wheel's own hub bore Ø WHEEL_BORE_D handles
     the M2 in this region)
    [shaft clear]   Ø M2_SHAFT_CLR_D through outboard slab inner 2 mm
    [insert pocket] Ø MOUNT_INSERT_HOLE_D, depth MOUNT_OUTER_T on the
                    outboard slab's outer face

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
    M2_INSERT_PILOT_D,
    M2_SHAFT_CLR_D,
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

MOUNT_HEAD_HOLE_D     = M2_HEAD_RECESS_D     # 4.0 — head clearance
MOUNT_INSERT_HOLE_D   = M2_INSERT_PILOT_D    # 3.3 — heat-set insert pilot

# Tip-clearance blind-bore depth past the insert (so an M2 screw slightly
# longer than the stack doesn't bottom out on housing material).
TIP_CLEARANCE_DEPTH   = 2.0


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
                     housing_z_sign: int) -> cq.Workplane:
    """Pocket carved into the housing for the wheel only. X extent matches
    wheel + hub envelope + axial clearance; YZ extent matches painted OD
    + clearance, but stretched in the housing-end Z direction up to the
    slab housing-end so the slab's housing-end face is the pocket roof."""
    inboard_x, outboard_x = slab_inner_faces(x_center)
    half_y = WHEEL_PAINTED_OD / 2 + WHEEL_POCKET_Y_CLR
    half_z = WHEEL_PAINTED_OD / 2 + WHEEL_POCKET_Z_CLR
    if housing_z_sign > 0:
        # Pancake: housing above. Pocket stretches up to the slab housing-end.
        z_min = z_center - half_z
        z_max = z_center + MOUNT_HOUSING_END_DIST
    else:
        z_min = z_center - MOUNT_HOUSING_END_DIST
        z_max = z_center + half_z
    return (
        cq.Workplane("XY")
        .box(outboard_x - inboard_x,
             half_y * 2,
             z_max - z_min,
             centered=False)
        .translate((inboard_x, y_center - half_y, z_min))
    )


def m2_hole_cut(x_center: float, y_center: float, z_center: float,
                *, head_side: str = "inboard") -> cq.Workplane:
    """Stepped M2 hole through both slab regions, centered at (y, z).
    Returns the full cutter (head pocket + shaft + insert + tip clearance).

    head_side: "inboard" (head at the -X end) or "outboard" (head at +X end).
    """
    inboard_inner_x, outboard_inner_x = slab_inner_faces(x_center)
    inboard_outer_x  = inboard_inner_x  - MOUNT_T   # -X edge of inboard slab
    outboard_outer_x = outboard_inner_x + MOUNT_T   # +X edge of outboard slab

    if head_side == "inboard":
        head_x_min, head_x_max = inboard_outer_x, inboard_outer_x + MOUNT_OUTER_T
        insert_x_min, insert_x_max = outboard_outer_x - MOUNT_OUTER_T, outboard_outer_x
        tip_blind_x_min = outboard_outer_x
        tip_blind_x_max = outboard_outer_x + TIP_CLEARANCE_DEPTH
    else:
        head_x_min, head_x_max = outboard_outer_x - MOUNT_OUTER_T, outboard_outer_x
        insert_x_min, insert_x_max = inboard_outer_x, inboard_outer_x + MOUNT_OUTER_T
        tip_blind_x_min = inboard_outer_x - TIP_CLEARANCE_DEPTH
        tip_blind_x_max = inboard_outer_x

    eps = 0.05  # boolean overshoot

    def _x_cyl(d, x_min, x_max):
        length = (x_max - x_min) + 2 * eps
        return cq.Workplane("XY").add(cq.Solid.makeCylinder(
            d / 2, length,
            pnt=cq.Vector(x_min - eps, y_center, z_center),
            dir=cq.Vector(1, 0, 0),
        ))

    # Full-length Ø2.3 shaft clearance from inboard outer to outboard outer
    # (cuts through everything; head/insert pockets widen it locally).
    shaft = _x_cyl(M2_SHAFT_CLR_D, inboard_outer_x, outboard_outer_x)
    head_pocket = _x_cyl(MOUNT_HEAD_HOLE_D, head_x_min, head_x_max)
    insert_pocket = _x_cyl(MOUNT_INSERT_HOLE_D, insert_x_min, insert_x_max)
    tip_blind = _x_cyl(M2_SHAFT_CLR_D, tip_blind_x_min, tip_blind_x_max)
    return shaft.union(head_pocket).union(insert_pocket).union(tip_blind)
