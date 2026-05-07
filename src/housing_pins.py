"""Brake housing pins — separately printed, glue-installed into the housing.

Two square-keyed pins with one rounded corner (the only contact surface seen
by the lever pin). The spring pin (``brake_housing_pin``) has a diametral
spring-leg through-hole; the rest pin (``brake_housing_rest_pin``) doesn't.

The matching keyed holes in the housing are cut by ``housing.py`` itself
using ``_brake_hole_local`` + ``brake_pin_place`` defined there.
"""

import cadquery as cq

from .housing import (
    STOP_PIN_D, STOP_PIN_H, STOP_PIN_HOLE_D,
    BRAKE_PIN_INSERT_DEPTH,
    STOP_REST_PIN_ALPHA_BRAKE_DEG,
    brake_pin_xz_profile, brake_pin_place,
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


# Spring pin: rounded corner faces the lever pin via the +z_local half
# (lever-pin direction in housing-pin local frame is (-x, +z) for the
# spring pin's geometry; see derivation in the pin redesign comment).
brake_housing_pin = brake_pin_place(
    _brake_pin_local(rounded_z_sign=-1, with_spring_hole=True))
# Rest pin: catches the brake lever pin at its design rest position so
# the spring's restoring force can't over-rotate it. Lever-pin direction
# in this pin's local frame is (-x, -z), so the rounded corner is on the
# -z side. No spring hole.
brake_housing_rest_pin = brake_pin_place(
    _brake_pin_local(rounded_z_sign=+1, with_spring_hole=False),
    alpha=STOP_REST_PIN_ALPHA_BRAKE_DEG)
