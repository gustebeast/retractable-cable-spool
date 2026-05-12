"""Main spool axle: shaft + spring-engagement bosses + bearing-retention hats + cross-pin holes."""

import math
import cadquery as cq

from .dimensions import (
    AXLE_CROSS_HOLE_D, AXLE_D, AXLE_EXTRA_LEVER, AXLE_H, AXLE_PRINT_D,
    FIN_Z1,
    LEVER_CAP_SEAT_Z1,
    M2_HEAD_RECESS_D, M2_HEAD_RECESS_H,
    M2_INSERT_DEPTH, M2_INSERT_PILOT_D, M2_SHAFT_CLR_D,
    PANCAKE_CAP_SEAT_Z0, PANCAKE_CROSS_PIN_Z,
    SPOOL_H,
)
from .helpers import cyl, cone_solid


# ────────────────────────────────────────────────────────────────────────────
# AXLE  (stationary; spool rotates around it on bearings)
#   8 mm shaft full-length, with a SINGLE fin protruding radially on
#   one side at z = mid-cavity. Fin profile: 45° taper at the bottom
#   (axle → full radial extent), cylindrical above, flat top at FIN_Z1.
#   A 0.8 mm radial slot splits the fin: INNER wall flush with the
#   axle surface at the fin's azimuthal center; OUTER prong is 1.5 mm
#   thick. Slot opens at top, closes onto a solid base for stiffness.
#   At the slot's TOP, BOTH walls flare to form a V-funnel that guides
#   the spring leg down into the slot during install — inner wall
#   flares inward (into the axle envelope, but the slot is cut from
#   fin material only, so the axle stays whole) and outer wall flares
#   OUTWARD into the outer prong material.
# ────────────────────────────────────────────────────────────────────────────

# ── Spring engagement: dual-boss M2 cap-screw + heat-set insert ──────────
# Replaces the previous cone-widened body. The single short body could
# only support the screw at one end, leaving the protruding shaft
# cantilevered against the spring's pull. This split moves to TWO bosses
# 17 mm apart axially — one houses the screw head (counterbore at the
# bottom face), the other a heat-set insert (pocket open at the top
# face). The 17 mm of bare M2 shaft between them is supported on both
# ends and forms the spring leg's engagement post.
#
# Cross-section (XY): each boss is a stadium / convex-hull profile —
# axle Ø8 fused via tangent walls to a Ø7.5 boss centered at +X offset
# SPRING_SCREW_OFFSET. Smooth tangent walls instead of a necked figure-8
# give a much beefier connection in the loading direction.
#
# Print orientation: vertical (axle along Z). Each boss has a 45°
# underside chamfer lofted from the axle Ø8 circle up to the full
# lozenge profile, so the +X overhang is self-supporting. The boss top
# faces are inward steps (lozenge → axle Ø8) and need no chamfer.
SPRING_BOSS_OD            = 7.5     # boss OD = head pocket (4.0) + 2 × wall (1.75)
SPRING_SCREW_OFFSET       = 7.0     # radial offset of screw axis from axle centerline.
                                    # Min: head_R + axle_R = 1.85 + 4 = 5.85 (else head
                                    # clips the axle); +1 mm for inner wall thickness.
SPRING_HEAD_POCKET_D      = M2_HEAD_RECESS_D       # 4.0 mm — shared with housing recesses
SPRING_INSERT_POCKET_D    = M2_INSERT_PILOT_D      # 3.3 mm — heat-set insert hole
SPRING_SHAFT_CLEARANCE_D  = M2_SHAFT_CLR_D         # 2.5 mm — same M2 clearance everywhere
SPRING_HEAD_POCKET_DEPTH  = M2_HEAD_RECESS_H + 0.5 # 2.5 mm: M2 head (2 mm) + 0.5 mm clearance
SPRING_INSERT_POCKET_DEPTH = M2_INSERT_DEPTH + 0.5 # 4.0 mm: insert (3.5 mm) + 0.5 mm clearance
SPRING_SHAFT_GAP          = 17.0    # head bearing-face → insert entry-face. The
                                    # screw's full unsupported shaft length (per
                                    # the screw the user has on hand).
SPRING_WALL_ABOVE_HEAD    = 1.0     # boss material above the counterbore step
SPRING_WALL_BELOW_INSERT  = 1.0     # boss material below the insert pocket
_SPRING_CHAMFER_H         = 2.5     # underside chamfer height. Limited by the
                                    # bottom cap (z=8): chamfer can't extend into
                                    # the bearing region. Steeper-than-45° at the
                                    # +X tip — slicer will add minimal supports.

def _spring_lozenge_pts():
    """Compute the 4 anchor points of the convex-hull lozenge profile
    (axle Ø8 + boss Ø_BOSS at offset)."""
    ax_r = AXLE_PRINT_D / 2
    bo_r = SPRING_BOSS_OD / 2
    d = SPRING_SCREW_OFFSET
    sin_a = (ax_r - bo_r) / d
    cos_a = math.sqrt(1 - sin_a * sin_a)
    p_axle = (ax_r * sin_a, ax_r * cos_a)            # tangent point on axle (upper)
    p_boss = (d + bo_r * sin_a, bo_r * cos_a)        # tangent point on boss (upper)
    return ax_r, bo_r, d, p_axle, p_boss

def _spring_lozenge_wp():
    """Workplane on XY with the closed lozenge profile as a pending wire."""
    ax_r, bo_r, d, p_axle, p_boss = _spring_lozenge_pts()
    return (
        cq.Workplane("XY")
        .moveTo(p_axle[0], p_axle[1])
        .threePointArc((-ax_r, 0), (p_axle[0], -p_axle[1]))
        .lineTo(p_boss[0], -p_boss[1])
        .threePointArc((d + bo_r, 0), (p_boss[0], p_boss[1]))
        .close()
    )

def _spring_boss(z_bot, z_top):
    """Lozenge boss spanning between z_top and z_bot. z_bot is the
    larger-Z "underside" face that gets the printability chamfer
    (z_top < z_bot). Chamfer is lofted from axle Ø8 (at z_bot + chamfer)
    up to the full lozenge profile (at z_bot)."""
    ax_r, _, d, p_axle, p_boss = _spring_lozenge_pts()
    body = _spring_lozenge_wp().extrude(z_top - z_bot).translate((0, 0, z_bot))
    chamfer = (
        cq.Workplane("XY")
        .workplane(offset=z_bot + _SPRING_CHAMFER_H)
        .circle(ax_r)
        .workplane(offset=-_SPRING_CHAMFER_H)
        .moveTo(p_axle[0], p_axle[1])
        .threePointArc((-ax_r, 0), (p_axle[0], -p_axle[1]))
        .lineTo(p_boss[0], -p_boss[1])
        .threePointArc((d + ax_r, 0), (p_boss[0], p_boss[1]))  # midpoint just needs to be on the right arc
        .close()
        .loft(combine=True)
    )
    return body.union(chamfer)

# Bearing retention shoulders — 45° "hat" widenings at each bearing's
# inner-cavity face. Bearings slide on from the outer ends of the axle
# and stop at these shoulders, fixing their axial position. Both halves
# of each hat are 45° (rise = run = (BEARING_AXLE_LIP_OD − AXLE_D) / 2),
# so they're self-supporting in vertical print orientation: the lower
# half widens going up; the upper half narrows going up (each upper
# layer sits inside the lower layer's footprint). The axle returns to
# AXLE_D on either side of each hat so the fin and other features
# (which rely on AXLE_D) keep their correct geometry.
BEARING_AXLE_LIP_OD = 10.0   # 1 mm radial protrusion to match the cap
                             # lips (which protrude ~1 mm inward from the
                             # bearing pocket bore)


def _axle_cross_hole(z_center):
    """M2 clearance hole through the axle, axis along y, centered at z."""
    return cq.Workplane("XY").add(cq.Solid.makeCylinder(
        AXLE_CROSS_HOLE_D / 2, AXLE_D + 2,
        pnt=cq.Vector(0, -(AXLE_D / 2 + 1), z_center),
        dir=cq.Vector(0, 1, 0),
    ))


def _build_axle():
    # Centered against the spring-housing bearing lips. The spring side is
    # at the pancake side (larger Z, where bearing_cap_top sits). Suffix _HI
    # / _LO indicates the boss face's Z magnitude (HI = larger Z, LO = smaller
    # Z). The two bosses are stacked along Z with the head boss at higher Z
    # and the insert boss at lower Z; the M2 screw enters the head boss from
    # the HI face and threads into the insert at _SPRING_INSERT_OPENING.
    _SPRING_HEAD_HI         = SPOOL_H - 15.0                                  # head boss face nearest the cavity
    _SPRING_HEAD_STEP       = _SPRING_HEAD_HI - SPRING_HEAD_POCKET_DEPTH      # head bearing-face step
    _SPRING_HEAD_LO         = _SPRING_HEAD_STEP - SPRING_WALL_ABOVE_HEAD      # head boss face away from cavity
    _SPRING_INSERT_OPENING  = _SPRING_HEAD_STEP - SPRING_SHAFT_GAP            # insert pocket entry (top of insert)
    _SPRING_INSERT_BOSS_HI  = _SPRING_INSERT_OPENING + SPRING_WALL_BELOW_INSERT   # insert boss face nearer to head boss
    _SPRING_INSERT_BOSS_LO  = _SPRING_INSERT_OPENING - SPRING_INSERT_POCKET_DEPTH # insert boss face where heat-set installs from

    axle_shaft = cyl(AXLE_PRINT_D, AXLE_H, z=-AXLE_EXTRA_LEVER)

    # Build the two bosses + cut the screw features
    screw_x = SPRING_SCREW_OFFSET
    spring_body = _spring_boss(_SPRING_HEAD_HI, _SPRING_HEAD_LO).union(
        _spring_boss(_SPRING_INSERT_BOSS_HI, _SPRING_INSERT_BOSS_LO)
    )

    # Head counterbore — opens at the head boss "bottom" face (the larger-Z
    # face, cavity-facing). Cut from z_step up to z_bot+chamfer so the
    # chamfer's filled-in centre is also bored out (Allen-key access from
    # that side).
    spring_body = spring_body.cut(
        cyl(SPRING_HEAD_POCKET_D,
            (_SPRING_HEAD_HI + _SPRING_CHAMFER_H) - _SPRING_HEAD_STEP,
            z=_SPRING_HEAD_STEP).translate((screw_x, 0, 0))
    )
    # 45° internal chamfer at the counterbore step (Ø4.5 → Ø2.4 over 1.05 mm)
    # replaces the right-angle ceiling so the step prints self-supporting.
    step_chamfer_h = (SPRING_HEAD_POCKET_D - SPRING_SHAFT_CLEARANCE_D) / 2
    spring_body = spring_body.cut(
        cone_solid(SPRING_SHAFT_CLEARANCE_D, SPRING_HEAD_POCKET_D,
                   step_chamfer_h, _SPRING_HEAD_STEP - step_chamfer_h).translate((screw_x, 0, 0))
    )
    # Shaft clearance through the head boss past the chamfered step.
    spring_body = spring_body.cut(
        cyl(SPRING_SHAFT_CLEARANCE_D,
            (_SPRING_HEAD_STEP - step_chamfer_h) - _SPRING_HEAD_LO,
            z=_SPRING_HEAD_LO).translate((screw_x, 0, 0))
    )
    # Shaft clearance through the insert boss past the insert pocket
    # (the wall_below_insert thickness, plus the chamfer region beyond).
    spring_body = spring_body.cut(
        cyl(SPRING_SHAFT_CLEARANCE_D,
            (_SPRING_INSERT_BOSS_HI + _SPRING_CHAMFER_H) - _SPRING_INSERT_OPENING,
            z=_SPRING_INSERT_OPENING).translate((screw_x, 0, 0))
    )
    # Insert pocket — opens at the insert boss "top" face (the smaller-Z
    # face, away from the cavity). Heat-set the insert from that face.
    spring_body = spring_body.cut(
        cyl(SPRING_INSERT_POCKET_D, SPRING_INSERT_POCKET_DEPTH,
            z=_SPRING_INSERT_OPENING - SPRING_INSERT_POCKET_DEPTH).translate((screw_x, 0, 0))
    )

    axle = axle_shaft.union(spring_body)

    bearing_lip_h = (BEARING_AXLE_LIP_OD - AXLE_PRINT_D) / 2   # half-hat height (45° rise = run)
    # Pancake-side hat: peaks at PANCAKE_CAP_SEAT_Z0 - lip_h, just below the
    # pancake-side bearing pocket which sits at z=PANCAKE_CAP_SEAT_Z0..SPOOL_H.
    pancake_hat_peak_z = PANCAKE_CAP_SEAT_Z0 - bearing_lip_h
    # Lever-side hat: peaks at LEVER_CAP_SEAT_Z1 + lip_h, just above the
    # lever-side bearing pocket which sits at z=0..CAP_H.
    lever_hat_peak_z   = LEVER_CAP_SEAT_Z1 + bearing_lip_h

    axle = (
        axle
        # Lever-side hat — widening up to the peak, then narrowing
        .union(cone_solid(AXLE_PRINT_D, BEARING_AXLE_LIP_OD,
                          bearing_lip_h, lever_hat_peak_z - bearing_lip_h))
        .union(cone_solid(BEARING_AXLE_LIP_OD, AXLE_PRINT_D,
                          bearing_lip_h, lever_hat_peak_z))
        # Pancake-side hat — widening up to the peak, then narrowing
        .union(cone_solid(AXLE_PRINT_D, BEARING_AXLE_LIP_OD,
                          bearing_lip_h, pancake_hat_peak_z - bearing_lip_h))
        .union(cone_solid(BEARING_AXLE_LIP_OD, AXLE_PRINT_D,
                          bearing_lip_h, pancake_hat_peak_z))
    )

    # Cross-pin hole — pancake-side only. The lever-side plate was
    # trimmed inboard to LEVER_HOUSING_X_TAIL with no material at the
    # axle's X, so there's no lever-side cross-pin to drill for. The
    # pancake cross-pin alone locks the axle against rotation + axial
    # slide.
    axle = axle.cut(_axle_cross_hole(PANCAKE_CROSS_PIN_Z))
    return axle


axle = _build_axle()
