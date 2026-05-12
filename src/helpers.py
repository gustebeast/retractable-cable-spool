"""Geometric helper functions used by build.py.

Pure functions — no module-level state. All dimensions are passed in as
arguments OR pulled from src.dimensions.
"""

from __future__ import annotations

import math
import cadquery as cq

from .dimensions import (
    FLANGE_H, FLANGE_OD, FLANGE_ID, FLANGE_LIP_T,
    FLANGE_INNER_ID, FLANGE_INNER_EXT, FLANGE_INNER_LIP_H,
    DRUM_OD, DRUM_ID,
    HUB_OD,
    SPOKE_COUNT, SPOKE_W,
    STRUCT_WALL,
    KEY_W, KEY_DEPTH, KEY_CLR, KEY_ANGLES,
    BOOL_OVERSHOOT,
)


def cyl(d: float, h: float, z: float = 0.0) -> cq.Workplane:
    """Solid cylinder, diameter d, height h, base at z."""
    return cq.Workplane("XY").workplane(offset=z).circle(d / 2).extrude(h)


def cone_solid(d_bottom: float, d_top: float, h: float, z_base: float) -> cq.Workplane:
    """Solid truncated cone: d_bottom at z_base, d_top at z_base+h (filled from axis)."""
    return (
        cq.Workplane("XY").workplane(offset=z_base)
        .circle(d_bottom / 2)
        .workplane(offset=h)
        .circle(d_top / 2)
        .loft()
    )


def lever_flange_solid(z_drum_face: float) -> cq.Workplane:
    """Lever-side flange — flat-bottom annular rim plus a single inward
    45° chamfer connecting the rim's inner edge up to the drum's inner
    cavity wall. z_drum_face is the upper face of the flange.

    Geometry (z_face_lo = z_drum_face − FLANGE_H = 0 in current build):
      - Rim: OD=DRUM_OD, ID=FLANGE_INNER_ID, FLANGE_LIP_T thick, at
        z=z_face_lo..z_face_lo+FLANGE_LIP_T. Inner half carries ratchet
        teeth, outer half is the smooth brake/wheel surface.
      - Chamfer: 45° conical face from (FLANGE_INNER_ID/2, rim_top_z) to
        (DRUM_ID/2, rim_top_z+chamfer_h), where chamfer_h equals
        (DRUM_ID−FLANGE_INNER_ID)/2 (rise=run).
      - Wall: cylindrical shell at r=DRUM_ID/2..DRUM_OD/2 from the
        chamfer's apex up to z_drum_face — bridges to the drum body
        above. spool.py drives DRUM_SKIRT_H from chamfer_h so the drum
        cylinder extends down to meet the chamfer apex seamlessly."""
    z_face_lo     = z_drum_face - FLANGE_H            # bottom of flange
    rim_top_z     = z_face_lo + FLANGE_LIP_T          # top of flat rim
    chamfer_h     = (DRUM_ID - FLANGE_INNER_ID) / 2   # 2.5 — 45° rise=run
    chamfer_top_z = rim_top_z + chamfer_h             # apex meets drum cavity wall

    rim = (
        cyl(DRUM_OD, FLANGE_LIP_T, z=z_face_lo)
        .cut(cyl(FLANGE_INNER_ID, FLANGE_LIP_T, z=z_face_lo))
    )

    # Chamfer body: full disk over the chamfer's axial range with a cone
    # carved out on the inside. Cone widens going up (FLANGE_INNER_ID at
    # rim_top_z → DRUM_ID at chamfer_top_z); cutting the cone leaves the
    # 45° conical face bounding the cavity.
    chamfer_body       = cyl(DRUM_OD, chamfer_h, z=rim_top_z)
    chamfer_inner_cone = cone_solid(FLANGE_INNER_ID, DRUM_ID, chamfer_h, rim_top_z)
    chamfer = chamfer_body.cut(chamfer_inner_cone)

    # Wall section bridging chamfer apex up to the drum's bottom face.
    wall_h = z_drum_face - chamfer_top_z
    if wall_h > 0:
        wall = (
            cyl(DRUM_OD, wall_h, z=chamfer_top_z)
            .cut(cyl(DRUM_ID, wall_h, z=chamfer_top_z))
        )
        return rim.union(chamfer).union(wall)
    return rim.union(chamfer)


def pancake_flange_solid(z_drum_face: float) -> cq.Workplane:
    """Pancake-side flange — z_drum_face is the lower (drum-facing) face.
    Body extends FLANGE_H above z_drum_face; outer wall flush with the
    drum (OD = DRUM_OD). Inside, a 45° taper rises from r=DRUM_ID/2 at
    z_drum_face to the vertical-lip radius, then continues straight up
    to the top face. Top flat annulus matches the lever-side rim width
    (FLANGE_INNER_EXT)."""
    pancake_rim_w  = FLANGE_INNER_EXT / 2                     # 7 — matches one lever rim band
    slope_h        = pancake_rim_w - (DRUM_OD - DRUM_ID) / 2  # 2.5 with current dims
    inner_top_d    = DRUM_ID - 2 * slope_h                    # vertical-lip ID
    z_face_hi      = z_drum_face + FLANGE_H                   # body top
    z_slope_hi     = z_drum_face + slope_h                    # top of slope

    base_ring = (
        cyl(DRUM_OD, FLANGE_H, z=z_drum_face)
        .cut(cyl(inner_top_d, FLANGE_H, z=z_drum_face))
    )

    # Inner overhang: 45° cone narrowing going up (DRUM_ID at z_drum_face,
    # inner_top_d at z_slope_hi). Air to cut is INSIDE the cone — intersect.
    inner_annulus = (
        cyl(DRUM_ID, slope_h, z=z_drum_face)
        .cut(cyl(inner_top_d, slope_h, z=z_drum_face))
    )
    inner_cone = cone_solid(DRUM_ID, inner_top_d, slope_h, z_drum_face)
    inner_overhang = inner_annulus.intersect(inner_cone)

    return base_ring.cut(inner_overhang)


def ratchet_cutter(
    num_teeth: int,
    r_in: float,
    r_out: float,
    z_top: float,
    depth: float,
    theta_offset_deg: float = 0.0,
) -> cq.Workplane:
    """Sawtooth ratchet cutter with a true helicoidal ramp.

    Each tooth's MATERIAL is built as a loft between two radial rectangular
    wires: a near-degenerate rectangle at θ_start (height = eps, hugging
    z_top, so the ramp begins thin at the top) and a full rectangle at
    θ_end (z_bottom → z_top, full depth). The loft's ruled surface has
    constant angular slope at every radius — no triangular artifacts at
    tooth boundaries the way a flat-plane ramp would produce. The ramp's
    BOTTOM edge drops from z_top down to z_bottom as θ increases, so each
    tooth presents its steep engagement face at θ_start.

    Wires are built slightly oversized radially (±r_buf) so the chord
    approximation of each radial edge stays outside the target annulus.
    The union of tooth materials is intersected with a true annular groove
    for clean circular inner/outer boundaries; the cutter is then
    (groove − teeth) = the air to remove above each ramp."""
    pitch_rad = 2 * math.pi / num_teeth
    offset_rad = math.radians(theta_offset_deg)
    z_bottom  = z_top - depth
    r_buf     = 1.0
    r_in_b    = r_in - r_buf
    r_out_b   = r_out + r_buf
    eps       = 0.05      # minimum vertical thickness of the degenerate start wire

    def _tooth_material(i):
        th_s = offset_rad + i * pitch_rad
        th_e = offset_rad + (i + 1) * pitch_rad
        cs, ss = math.cos(th_s), math.sin(th_s)
        ce, se = math.cos(th_e), math.sin(th_e)

        pts_s = [
            cq.Vector(r_in_b  * cs, r_in_b  * ss, z_top - eps),
            cq.Vector(r_out_b * cs, r_out_b * ss, z_top - eps),
            cq.Vector(r_out_b * cs, r_out_b * ss, z_top),
            cq.Vector(r_in_b  * cs, r_in_b  * ss, z_top),
        ]
        pts_e = [
            cq.Vector(r_in_b  * ce, r_in_b  * se, z_bottom),
            cq.Vector(r_out_b * ce, r_out_b * se, z_bottom),
            cq.Vector(r_out_b * ce, r_out_b * se, z_top),
            cq.Vector(r_in_b  * ce, r_in_b  * se, z_top),
        ]
        wire_s = cq.Wire.makePolygon(pts_s, close=True)
        wire_e = cq.Wire.makePolygon(pts_e, close=True)
        # ruled=True forces each side face to a simple ruled surface
        # (bilinear patch between corresponding edge points) instead of
        # the smooth BSpline that the default loft produces. Ruled
        # surfaces are topologically cleaner and avoid the 1 bad face
        # that OCCT's BSpline loft generates near the degenerate
        # start-wire corner (z_bottom..z_bottom+eps edge collapses to
        # near-zero length at one end).
        return cq.Solid.makeLoft([wire_s, wire_e], ruled=True)

    teeth = _tooth_material(0)
    for i in range(1, num_teeth):
        teeth = teeth.fuse(_tooth_material(i))

    # Full annular groove (what would be cut if there were no teeth)
    groove = (
        cyl(2 * r_out, depth, z=z_bottom)
        .cut(cyl(2 * r_in, depth, z=z_bottom))
    )

    # Wrap teeth (Solid) in a Workplane to interoperate with groove (Workplane)
    teeth_wp = cq.Workplane("XY").add(teeth)

    # Clip teeth to the annulus: clean circular inner/outer boundaries
    teeth_clipped = teeth_wp.intersect(groove)

    # Cutter = groove − teeth_clipped = the air above each ramp
    return groove.cut(teeth_clipped)


def spokes_solid(z_base: float, z_top: float) -> cq.Workplane:
    """Vertical rib walls connecting hub OD → drum ID over [z_base, z_top].
    Outer-top corner is chamfered at 45° along the same line as the flange
    inner slope (from (r=r_taper, z=z_top) to (r=DRUM_ID/2, z=z_top−taper_h)
    where taper_h = FLANGE_INNER_EXT + FLANGE_INNER_LIP_H). Two reasons:
      1. Keeps the spoke top well below every tooth valley (z_top−9 = 29 is
         7.5 mm below the z=36.5 valley floor), so the spoke can never
         interfere with the pawl even when it drops fully into a valley.
      2. The 45° taper matches the flange inner slope visually, so the
         spoke appears to terminate along the same line that carves the
         flange inner cavity — no square-corner notch at the junction."""
    # Inner/outer radii overlap the hub OD and drum ID by 0.5 mm so the
    # spoke-to-hub and spoke-to-drum unions are volumetric, not tangent.
    # A perfectly tangent boolean (r_in = HUB_OD/2 exactly) gives OCCT a
    # knife-edge contact line that occasionally produces a degenerate
    # face on one of the six spokes — visible as a tapered / triangulated
    # spoke that doesn't span the full hub height.
    _spoke_overlap = 0.5
    r_in         = HUB_OD / 2 - _spoke_overlap
    # Reach the cable-cradle bottom radius (= DRUM_ID/2 + STRUCT_WALL). That's
    # as far out as the spoke can go without poking past the drum's outer
    # surface (which dips to exactly there at each cradle bottom). The drum
    # wall's groove-following inner relief pulls its inner surface ~3 mm
    # clear of DRUM_ID/2 between cradle turns, so a spoke ending at DRUM_ID/2
    # would float free there; reaching the cradle bottom lets the spoke land
    # flush on the full-thickness wall at every cradle crossing.
    r_out        = DRUM_ID / 2 + STRUCT_WALL
    taper_h      = FLANGE_INNER_EXT + FLANGE_INNER_LIP_H
    z_taper_end  = z_base + taper_h                        # taper at the lever-side (z_base) end
    r_taper_start = r_out - taper_h
    out = None
    for i in range(SPOKE_COUNT):
        s = (
            cq.Workplane("XZ")
            .polyline([
                (r_in,          z_base),
                (r_taper_start, z_base),
                (r_out,         z_taper_end),
                (r_out,         z_top),
                (r_in,          z_top),
            ])
            .close()
            .extrude(SPOKE_W / 2, both=True)
            .rotate((0, 0, 0), (0, 0, 1), i * 360.0 / SPOKE_COUNT)
        )
        out = s if out is None else out.union(s)
    return out


def make_keys(
    cyl_r: float,
    z_low: float,
    z_high: float,
    *,
    groove: bool = False,
    chamfer_high: bool = False,
) -> cq.Workplane:
    """Build SPOKE_COUNT boxes (one at each spoke angle), each protruding
    radially outward from a cylinder at radius cyl_r. Tongue and groove
    both span the full [z_low, z_high] axial range; groove is oversized
    by KEY_CLR on tangential and radial dimensions so the tongue drops
    in by hand.

    chamfer_high=True adds a 45° chamfer at the z_high end of the tongue
    so it tapers from full radial protrusion down to the cylinder
    surface over the top KEY_DEPTH of axial travel. Used on tongues
    that print upside-down (i.e. z_high is the bottom in print
    orientation) so the protrusion is self-supporting at the start."""
    overshoot = BOOL_OVERSHOOT  # extend inward into host material for clean union/cut
    if groove:
        depth = KEY_DEPTH + KEY_CLR
        width = KEY_W + KEY_CLR
    else:
        depth = KEY_DEPTH
        width = KEY_W
    h_axial = z_high - z_low
    res = None
    for ang in KEY_ANGLES:
        if chamfer_high:
            # 5-sided profile in XZ: rectangle + 45° corner cut at top.
            inner_x = cyl_r - overshoot
            outer_x = cyl_r + depth
            chamfer_h = depth   # 45°: rise = run = depth
            b = (
                cq.Workplane("XZ")
                .moveTo(inner_x, z_low)
                .lineTo(outer_x, z_low)
                .lineTo(outer_x, z_high - chamfer_h)
                .lineTo(inner_x, z_high)
                .close()
                .extrude(width / 2, both=True)
                .rotate((0, 0, 0), (0, 0, 1), ang)
            )
        else:
            b = (
                cq.Workplane("XY")
                .workplane(offset=z_low)
                .center(cyl_r + (depth - overshoot) / 2, 0)
                .box(depth + overshoot, width, h_axial, centered=(True, True, False))
                .rotate((0, 0, 0), (0, 0, 1), ang)
            )
        res = b if res is None else res.union(b)
    return res


def heal(wp: cq.Workplane) -> cq.Workplane:
    """Run OCCT's ShapeFix + ShapeUpgrade on a Workplane's underlying
    shape to clean up minor face/edge tolerance issues and merge same-
    domain adjacent faces before STEP export. cadquery pipelines of
    many boolean ops can accumulate tiny invalidities (around lofted
    BSpline faces, near-tangent fuses, etc.) that pass cadquery's
    shallow validator but get flagged by strict STEP importers like
    Onshape. The unify pass also collapses fuse-of-disjoint Compounds
    down to plain Solids."""
    from OCP.ShapeFix import ShapeFix_Shape  # type: ignore[import]
    from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain  # type: ignore[import]
    from OCP.TopAbs import TopAbs_SOLID, TopAbs_COMPOUND
    shape = wp.val().wrapped
    fixer = ShapeFix_Shape(shape)
    fixer.SetPrecision(1e-4)
    fixer.SetMaxTolerance(1e-3)
    fixer.Perform()
    fixed = fixer.Shape()
    # Merge adjacent faces sharing the same underlying surface, and
    # adjacent edges sharing the same underlying curve. Also collapses
    # a Compound containing one connected solid into a plain Solid.
    unifier = ShapeUpgrade_UnifySameDomain(fixed, True, True, True)
    unifier.Build()
    unified = unifier.Shape()
    shape_type = unified.ShapeType()
    if shape_type == TopAbs_COMPOUND:
        wrapped = cq.Compound(unified)
    else:
        wrapped = cq.Solid(unified)
    return cq.Workplane("XY").add(wrapped)
