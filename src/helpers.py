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
    """Lever-side sloped flange — carries the ratchet teeth surfaces.
    z_drum_face is the upper face of the flange (drum-facing). Body
    extends FLANGE_H below z_drum_face. Inner overhang extends ABOVE
    z_drum_face into the drum region:
         - 45° outer drum-side: (r=DRUM_OD/2, z_drum_face) →
           (r=FLANGE_OD/2, z_drum_face−slope_h_outer), with the outer lip
           FLANGE_LIP_T tall on the lower (away-from-drum) side.
         - 45° inner upper: (r=FLANGE_INNER_ID/2, z_face_lo+FLANGE_INNER_LIP_H)
           → (r=DRUM_ID/2, z_drum_face+FLANGE_INNER_LIP_H). The slope
           bottoms out FLANGE_INNER_LIP_H above the flange's lower face,
           leaving a vertical inner lip; rise=run=slope_h_inner means the
           top of the slope extends FLANGE_INNER_LIP_H above z_drum_face,
           overlapping the drum.
    Built as (base ring over full extended z range) − (outer overhang)
    − (inner overhang)."""
    slope_h_outer = FLANGE_H - FLANGE_LIP_T              # 5
    slope_h_inner = (DRUM_ID - FLANGE_INNER_ID) / 2      # 7
    z_face_lo          = z_drum_face - FLANGE_H              # bottom of body
    z_inner_slope_lo   = z_face_lo + FLANGE_INNER_LIP_H      # 2 mm above bottom face
    z_inner_slope_hi   = z_drum_face + FLANGE_INNER_LIP_H    # 2 mm above drum face
    z_flange_top       = max(z_drum_face, z_inner_slope_hi)
    flange_h_actual    = z_flange_top - z_face_lo

    base_ring = (
        cyl(FLANGE_OD, flange_h_actual, z=z_face_lo)
        .cut(cyl(FLANGE_INNER_ID, flange_h_actual, z=z_face_lo))
    )

    # Outer overhang on the drum-facing side: spans z_drum_face−slope_h_outer..z_drum_face
    outer_annulus = (
        cyl(FLANGE_OD, slope_h_outer, z=z_drum_face - slope_h_outer)
        .cut(cyl(DRUM_OD, slope_h_outer, z=z_drum_face - slope_h_outer))
    )
    outer_cone = cone_solid(FLANGE_OD, DRUM_OD, slope_h_outer, z_drum_face - slope_h_outer)
    outer_overhang = outer_annulus.cut(outer_cone)

    # Inner overhang: at z_inner_slope_lo..z_inner_slope_hi. Cone widens
    # with increasing z, so the air we cut is INSIDE the cone — intersect.
    inner_annulus = (
        cyl(DRUM_ID, slope_h_inner, z=z_inner_slope_lo)
        .cut(cyl(FLANGE_INNER_ID, slope_h_inner, z=z_inner_slope_lo))
    )
    inner_cone = cone_solid(FLANGE_INNER_ID, DRUM_ID, slope_h_inner, z_inner_slope_lo)
    inner_overhang = inner_annulus.intersect(inner_cone)

    result = base_ring.cut(outer_overhang).cut(inner_overhang)

    # If the flange extends above z_drum_face into drum region, trim the
    # outer skirt outside DRUM_OD/2 there so the flange doesn't intrude
    # past the drum wall.
    if z_flange_top > z_drum_face:
        skirt_cut = (
            cyl(FLANGE_OD, z_flange_top - z_drum_face, z=z_drum_face)
            .cut(cyl(DRUM_OD, z_flange_top - z_drum_face, z=z_drum_face))
        )
        result = result.cut(skirt_cut)

    return result


def pancake_flange_solid(z_drum_face: float) -> cq.Workplane:
    """Pancake-side sloped flange ring. z_drum_face is the lower face
    (drum-facing). Body extends FLANGE_H above. Slope on the drum-facing
    side; lip on the upper (away-from-drum) end."""
    slope_h = FLANGE_H - FLANGE_LIP_T
    base_ring = cyl(FLANGE_OD, FLANGE_H, z=z_drum_face).cut(
                cyl(FLANGE_ID, FLANGE_H, z=z_drum_face))
    slope_annulus = (
        cyl(FLANGE_OD, slope_h, z=z_drum_face)
        .cut(cyl(DRUM_OD, slope_h, z=z_drum_face))
    )
    # Cone widens going UP from the drum face (DRUM_OD at bottom, FLANGE_OD at top).
    support_cone = cone_solid(DRUM_OD, FLANGE_OD, slope_h, z_drum_face)
    overhang = slope_annulus.cut(support_cone)
    return base_ring.cut(overhang)


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
    r_out        = DRUM_ID / 2 + _spoke_overlap
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
    """Run OCCT's ShapeFix on a Workplane's underlying solid to clean up
    any minor face/edge tolerance issues before STEP export. cadquery
    pipelines of many boolean ops can accumulate tiny invalidities
    (particularly around lofted/BSpline faces) that pass cadquery's
    shallow validator but get flagged by strict STEP importers."""
    from OCP.ShapeFix import ShapeFix_Shape  # type: ignore[import]
    shape = wp.val().wrapped
    fixer = ShapeFix_Shape(shape)
    fixer.SetPrecision(1e-4)
    fixer.SetMaxTolerance(1e-3)
    fixer.Perform()
    # Wrap the fixed TopoDS back into a cadquery Workplane
    new_solid = cq.Solid(fixer.Shape())
    return cq.Workplane("XY").add(new_solid)
