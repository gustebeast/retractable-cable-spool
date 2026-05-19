"""Brake housing pins — separately printed, glue-installed into the housing.

Two square-keyed pins with one rounded corner (the only contact surface seen
by the lever pin). The spring pin (``brake_housing_pin``) has a diametral
spring-leg through-hole; the rest pin (``brake_housing_rest_pin``) carries a
1 mm gap-side slab acting as a half-boss for the brake spring (the brake
side of the housing must stay flat for printability, so the boss-equivalent
material rides on the rest pin instead).

The matching keyed holes in the housing are cut by ``housing.py`` itself
using ``_brake_hole_local`` + ``brake_pin_place`` defined there.
"""

import math

import cadquery as cq

from .dimensions import M2_INSERT_PILOT_D
from .housing import (
    BRAKE_BOSS_EXT_ALPHA_HI, BRAKE_BOSS_EXT_ALPHA_LO,
    BRAKE_HOUSING_BOSS_EXTENSION, BRAKE_HOUSING_LEG_ALPHA_DEG,
    BRAKE_PIVOT_X, HOUSING_W,
    LEVER_PIVOT_BOSS_OD, LEVER_PIVOT_Z,
    STOP_PIN_D, STOP_PIN_H, STOP_PIN_HOLE_D, STOP_PIN_R,
    BRAKE_PIN_INSERT_DEPTH,
    STOP_REST_PIN_ALPHA_BRAKE_DEG,
    brake_pin_xz_profile, brake_pin_place,
    pivot_boss_sector, spring_leg_hole_dir_alpha_deg,
)


def _brake_pin_local(rounded_z_sign, with_spring_hole):
    """Pin built in a LOCAL frame for readability:
      +y_local = pin axis, pointing from the exposed tip INTO the housing.
      +x_local = radial-outward (spring leg through-hole, when present,
                  passes along ±x).
      +z_local = tangential.
    Cross-section is a 2 r × 2 r square with one corner (on the -x side,
    facing the lever pin) rounded to a quarter-circle of radius r. Pin is
    extruded along ±y_local from the local origin to L_exp + L_ins.
    rounded_z_sign selects which of the two -x corners is rounded; pass
    +1 for spring pin (faces lever pin via +z half), -1 for rest pin
    (faces lever pin via -z half).

    Caller transforms this LOCAL frame to WORLD via brake_pin_place."""
    r     = STOP_PIN_D / 2
    L_exp = STOP_PIN_H
    L_ins = BRAKE_PIN_INSERT_DEPTH

    pin = (
        brake_pin_xz_profile(r, rounded_z_sign)
        # Workplane "XZ" extrudes along -Y (verified empirically — the
        # workplane's normal direction in cadquery's "XZ" string preset
        # is -Y, not +Y as one might naively expect). Pass a negative
        # length so the solid grows along +Y instead, putting local y=0
        # at the exposed tip and local y=L_exp+L_ins at the buried end
        # (which matches the place-transform's y_tip translation).
        .extrude(-(L_exp + L_ins))
    )

    if with_spring_hole:
        # Diametral spring-leg hole, along ±x_local. Cross-section is a
        # 2r × 2r square + 45° peak (matches the lever-stop-pin teardrop
        # convention). Geometry in the YZ plane:
        #
        #   +y_local: pin axis, away from the lever-facing tip.
        #
        # The square's back edge sits at local y = SPRING_HOLE_Y_OFFSET,
        # leaving a SPRING_HOLE_Y_OFFSET-mm cap of solid material at the
        # pin tip (the lever-facing end) so the spring leg sits in an
        # enclosed pocket. The peak points in +y (into the pin body,
        # hidden inside the housing-buried portion of the pin). +y in
        # print orientation maps to +z (peak points UP), so the
        # horizontal hole self-supports without bridges.
        SPRING_HOLE_Y_OFFSET = 1.0
        hole_r = STOP_PIN_HOLE_D / 2
        rect_y_len = STOP_PIN_HOLE_D    # 2 mm — square YZ section
        peak_h = hole_r                 # 1 mm — 45° peak
        through_len = 2 * r + 0.4
        y0 = SPRING_HOLE_Y_OFFSET
        y1 = y0 + rect_y_len
        y_peak = y1 + peak_h
        profile = (
            cq.Workplane("YZ")
            .moveTo(y0, -hole_r)
            .lineTo(y0, +hole_r)
            .lineTo(y1, +hole_r)
            .lineTo(y_peak, 0)
            .lineTo(y1, -hole_r)
            .close()
        )
        through = profile.extrude(through_len).translate((-r - 0.2, 0, 0))
        pin = pin.cut(through)

    return pin


# ── Half-boss slab on the brake rest pin ─────────────────────────────────
# The brake side of the housing prints face-down on the bed and must
# stay flat — we can't add a housing-side boss extension into the gap
# the way we do on the ratchet side. Instead we ride a 1 mm slab on
# the rest pin (which is separately printed and glued in) that fills
# the same role: axially constrains the brake spring's housing-end
# face so the spring sits at the gap midpoint rather than bottoming
# on the housing column.
#
# In XZ, the slab is an annular sector around the brake pivot. Its
# −X half MIRRORS the brake LEVER's boss-extension sector exactly
# (same outer Ø, same α range BRAKE_BOSS_EXT_ALPHA_LO/HI) so when
# you load the housing + brake_housing_rest_pin + brake_lever in the
# same scene, the lever boss and the slab's −X half occupy the same
# X-Z footprint at different Y. The remaining sweep from the lever-
# boss-LO bound back to the rest pin's own α is the connecting "neck"
# that makes the slab continuous with the rest pin so they print as
# one part. Inner edge tracks the M2 insert pilot Ø3.3 so the heat-
# set insert / M2 screw clear cleanly.
REST_PIN_SLAB_INNER_D       = M2_INSERT_PILOT_D       # 3.3 mm — matches insert pilot
REST_PIN_SLAB_OUTER_D       = LEVER_PIVOT_BOSS_OD     # 6 mm — matches pivot boss / lever boss

# Trim applied to the slab's far end (the lever-boss-aligned end,
# furthest from the rest pin). Brings the total slab sweep to a clean
# 180° instead of running 5.605° past it. The lever boss' own α range
# happens to extend that little bit further than 180° from the rest
# pin's α; we don't need that overhang on the housing-side slab.
REST_PIN_SLAB_FAR_END_TRIM_DEG = 5.605

# Small rectangular prism that fills the wedge gap between the slab's
# outer cylinder (R = LEVER_PIVOT_BOSS_OD/2 = 3) and the pin's sharp
# −z_local inner corner. The slab's outer arc passes inside the
# corner (the corner sits at R ≈ 3.21 from pivot, just past R=3), so
# without this prism there's a small triangular sliver between them.
# Built in pin-local so it transforms with the pin via brake_pin_place.
REST_PIN_NOOK_FILL_DX = 0.3    # x_local extent past the pin's left edge
REST_PIN_NOOK_FILL_DZ = 0.35   # z_local extent from the sharp corner


def _rest_pin_nook_fill_local():
    """Pin-local rectangular prism filling the slab/pin wedge gap.
    See REST_PIN_NOOK_FILL_* constants above. Sized to land flush with
    the slab on the housing-Y face (y_local = STOP_PIN_H) and span the
    full slab Y thickness; in XZ it's just past the pin's left edge in
    the −x_local direction and stops short of the slab/pin intersection
    point so it nests into the wedge cleanly."""
    r  = STOP_PIN_D / 2
    return (
        cq.Workplane("XY")
        .box(REST_PIN_NOOK_FILL_DX,
             BRAKE_HOUSING_BOSS_EXTENSION,
             REST_PIN_NOOK_FILL_DZ)
        .translate((-r - REST_PIN_NOOK_FILL_DX / 2,
                    STOP_PIN_H - BRAKE_HOUSING_BOSS_EXTENSION / 2,
                    -r + REST_PIN_NOOK_FILL_DZ / 2))
    )


def _rest_pin_slab():
    """Full-disc spring-retention slab unioned into the brake rest pin.

    Mirrors the ratchet HOUSING-side boss extension (a full Ø
    LEVER_PIVOT_BOSS_OD disc around the pivot), but anchored to the
    rest pin via a rectangular connector bar instead of a boss on the
    housing column — the brake half of the housing must stay flat for
    printability, so the spring-retention disc rides on the separately-
    printed rest pin.

    Two pieces unioned:
      1. Full annular Ø REST_PIN_SLAB_OUTER_D disc around the brake
         pivot, with Ø REST_PIN_SLAB_INNER_D inner clearance for the
         M2 + heat-set insert.
      2. Rectangular connector bar from disc edge out to the rest pin
         along the rest pin's radial direction (α =
         STOP_REST_PIN_ALPHA_BRAKE_DEG). Length overlaps 1 mm into the
         pin column; width matches STOP_PIN_D so it's flush with the
         pin tangentially.
    """
    y_in_face_world  = -HOUSING_W / 2                                       # housing column outer face (-Y)
    y_out_face_world = -HOUSING_W / 2 - BRAKE_HOUSING_BOSS_EXTENSION        # slab face into the gap

    # Full disc.
    disc = pivot_boss_sector(
        BRAKE_PIVOT_X,
        y_out_face_world, y_in_face_world,
        alpha_lo_deg=0.0, alpha_hi_deg=360.0,
        od=REST_PIN_SLAB_OUTER_D,
    )

    # Connector bar — TRAPEZOIDAL in the bar's local XZ plane. The +Z
    # (lever-facing) edge slopes the FULL length from disc to pin:
    #   • -Z edge: straight from disc to pin at local Z = -STOP_PIN_D/2
    #     (= -2), flush with the pin's -Z tangent.
    #   • +Z edge: slopes from local Z = +STOP_PIN_D/2 (= +2) at the disc
    #     end down to local Z = 0 at the pin end. The disc-end +Z corner
    #     sits at radial distance √(2.2² + 2²) ≈ 2.98 mm from the pivot,
    #     just inside the disc edge (radius 3).
    # In the print frame (build plate normal = +X -Z, print direction =
    # -X +Z), the resulting +Z face has a normal at 107° from the print
    # direction → only ~17° of overhang from vertical, well inside the
    # 45° self-supporting limit. The slope spans the entire bar length
    # so the visual cut runs from the circle to the pin, not buried
    # inside the pin column.
    # Overshoots into BOTH the disc and the pin so the boolean union
    # produces solid material (no kissing surfaces).
    r_disc        = REST_PIN_SLAB_OUTER_D / 2
    bar_overshoot = 0.8
    bar_inner_r   = r_disc - bar_overshoot
    bar_outer_r   = STOP_PIN_R + bar_overshoot
    bar_length    = bar_outer_r - bar_inner_r
    bar_thickness = BRAKE_HOUSING_BOSS_EXTENSION
    bar_center_radial = (bar_inner_r + bar_outer_r) / 2
    z_full        = STOP_PIN_D / 2          # +2 mm — full pin half-width (disc end)
    z_narrow      = 0.0                      # narrow side at pin (on centerline)

    alpha_rad = math.radians(STOP_REST_PIN_ALPHA_BRAKE_DEG)
    bar_world_x = BRAKE_PIVOT_X + bar_center_radial * math.cos(alpha_rad)
    bar_world_z = LEVER_PIVOT_Z - bar_center_radial * math.sin(alpha_rad)

    trap_pts = [
        (-bar_length / 2, -z_full),    # disc-end bottom
        (+bar_length / 2, -z_full),    # pin-end bottom
        (+bar_length / 2,  z_narrow),  # pin-end top (narrow side at pin)
        (-bar_length / 2,  z_full),    # disc-end top (full side at disc)
    ]

    bar = (
        cq.Workplane("XZ")
        .polyline(trap_pts)
        .close()
        .extrude(-bar_thickness)
        # Rotate around +Y so the local +X (radial) axis aligns with the
        # housing's α direction (right-hand rule: positive α rotates +X
        # toward -Z, matching x = R cos α, z = -R sin α). Local +Z
        # (tangential) then maps to world (+sin α, 0, +cos α) = the
        # lever-facing direction.
        .rotate((0, 0, 0), (0, 1, 0), STOP_REST_PIN_ALPHA_BRAKE_DEG)
        .translate((bar_world_x, y_out_face_world, bar_world_z))
    )

    # Cut the inner clearance hole through the disc so the M2 screw
    # passes straight through and the heat-set insert (already pressed
    # into the column at this same X-Z) doesn't get blocked.
    inner_hole = (
        cq.Workplane("XY")
        .circle(REST_PIN_SLAB_INNER_D / 2)
        .extrude(BRAKE_HOUSING_BOSS_EXTENSION + 0.4)
        .rotate((0, 0, 0), (1, 0, 0), -90)
        .translate((BRAKE_PIVOT_X,
                    y_out_face_world - 0.2,
                    LEVER_PIVOT_Z))
    )
    return disc.union(bar).cut(inner_hole)


# Spring pin: rounded corner faces the lever pin via the +z_local half
# (lever-pin direction in housing-pin local frame is (-x, +z) for the
# spring pin's geometry; see derivation in the pin redesign comment).
# Rotated so the diametral spring-leg through-hole aligns with the
# spring leg's actual tangent direction at this pin (rather than with
# the pin's radial direction, which differs by SPRING_LEG_PIN_OFFSET_DEG).
brake_housing_pin = brake_pin_place(
    _brake_pin_local(rounded_z_sign=-1, with_spring_hole=True),
    rotation_alpha=spring_leg_hole_dir_alpha_deg(BRAKE_HOUSING_LEG_ALPHA_DEG))
# Rest pin: catches the brake lever pin at its design rest position so
# the spring's restoring force can't over-rotate it. Lever-pin direction
# in this pin's local frame is (-x, -z), so the rounded corner is on the
# -z side. No spring hole. The half-boss slab is unioned on after
# brake_pin_place (slab is in world frame); the small wedge nook-fill
# is unioned in pin-local frame so it transforms with the pin.
brake_housing_rest_pin = (
    brake_pin_place(
        _brake_pin_local(rounded_z_sign=+1, with_spring_hole=False)
            .union(_rest_pin_nook_fill_local()),
        alpha=STOP_REST_PIN_ALPHA_BRAKE_DEG,
    )
    .union(_rest_pin_slab())
)
