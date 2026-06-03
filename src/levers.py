"""Ratchet and brake levers — RADIAL engagement, 90°-rotated layout.

Each lever is a flat plate in the X-Z plane (thickness LEVER_T in Y) riding
on a Y-axis M2 pivot screw through the front-housing block. It hangs DOWN
the +X side of the housing; the user reaches under and pulls the handle
toward +X. See housing.py's header for the forced engagement directions:

  - RATCHET pivot is ABOVE the teeth band (z=7..14). Rest = ENGAGED: the
    pawl sits radially in against the rim teeth (r≈55.7). Pulling the
    handle swings the pawl outward to clear the teeth.
  - BRAKE pivot is BELOW the brake band (z=0..7). Rest = DISENGAGED: the
    pad sits lifted off the smooth band (r=57.2). Pulling the handle swings
    the pad inward onto the band.

The rim already carries both bands (spool.py), so this module no longer
cuts teeth into the body — apply_to_main_body is a no-op pass-through.
"""

import math
import cadquery as cq

from .spool import RIM_OD, RIM_H, RATCHET_BAND_H, _cyl_ratchet_band
from .lever_kinematics import assert_kinematics, R_ROOT
from .housing import (
    HOUSING_W, LEVER_RIM_H, LEVER_PIVOT_BOSS_OD,
    RATCHET_PIVOT_X, RATCHET_PIVOT_Z,
    BRAKE_PIVOT_X, BRAKE_PIVOT_Z,
    RATCHET_OUTER_TRAVEL_DEG, BRAKE_INNER_TRAVEL_DEG,
    LEVER_REST_PRECOMP_DEG,
    M2_SHAFT_CLR_D,
    STOP_PIN_R, STOP_PIN_H, STOP_PIN_HOLE_D,
    RATCHET_LEVER_PIN_ALPHA, BRAKE_LEVER_PIN_ALPHA,
    SPRING_LEG_PIN_OFFSET_DEG, spring_leg_hole_dir_alpha_deg,
    stop_pin_solid, stop_pin_hole, pivot_boss_sector,
)

# ── Plate placement in Y ────────────────────────────────────────────────────
LEVER_T       = 4.0                              # plate thickness (Y)
LEVER_INNER_Y = HOUSING_W / 2 + LEVER_RIM_H      # 15.5 — inner face, past the gap
RATCHET_Y0, RATCHET_Y1 = LEVER_INNER_Y, LEVER_INNER_Y + LEVER_T   #  15.5 .. 19.5
BRAKE_Y1,  BRAKE_Y0    = -LEVER_INNER_Y, -LEVER_INNER_Y - LEVER_T # -15.5 .. -19.5

ARM_W        = 8.0        # width of the lever arms in the swing (X-Z) plane
GRIP_W       = 15.0       # width of the rounded handle paddle (X-Z) at the grab end
HANDLE_Z_BOT = -19.0      # common grab height for both handles. Raised again
                          # (was -26) thanks to the ratchet/brake band swap,
                          # which lifts the brake pivot above the spool bottom.
PAWL_MESH_CLR_DEG = 0.6   # tangential backlash: the rim-tooth cutter is also
                          # applied ±this so the pawl's meshing faces aren't a
                          # zero-clearance press fit (lets it click/seat)

R_RIM        = RIM_OD / 2                        # 57.2 — brake-band / tooth-tip radius

# Rubber pad bonded to the brake pad's rim-facing face (viz + clearance).
BRAKE_RUBBER_T = 3.3


def _bar(x0, z0, x1, z1, w, y0, y1):
    """A flat bar of width w (in the X-Z plane) between (x0,z0) and (x1,z1),
    extruded in Y over [y0, y1]."""
    dx, dz = x1 - x0, z1 - z0
    L = math.hypot(dx, dz)
    nx, nz = -dz / L, dx / L
    pts = [(x0 + nx * w / 2, z0 + nz * w / 2),
           (x1 + nx * w / 2, z1 + nz * w / 2),
           (x1 - nx * w / 2, z1 - nz * w / 2),
           (x0 - nx * w / 2, z0 - nz * w / 2)]
    return (cq.Workplane("XZ").polyline(pts).close()
            .extrude(-(y1 - y0)).translate((0, y0, 0)))


def _arc_strip(r_in, r_out, y_near, y_far, z_lo, z_hi):
    """An annular strip between radii r_in/r_out, clipped to the lever's
    y-range [y_near, y_far], extruded in Z over [z_lo, z_hi]. The inner/outer
    faces follow the rim arc so the contact conforms to the cylinder at the
    lever's azimuth (proper engagement, not a flat chord). Requires
    r_in > |y|."""
    x_in_near  = math.sqrt(r_in ** 2  - y_near ** 2)
    x_out_near = math.sqrt(r_out ** 2 - y_near ** 2)
    x_in_far   = math.sqrt(r_in ** 2  - y_far ** 2)
    x_out_far  = math.sqrt(r_out ** 2 - y_far ** 2)
    y_mid = (y_near + y_far) / 2
    x_in_mid  = math.sqrt(r_in ** 2  - y_mid ** 2)
    x_out_mid = math.sqrt(r_out ** 2 - y_mid ** 2)
    return (cq.Workplane("XY").workplane(offset=z_lo)
            .moveTo(x_in_far, y_far).lineTo(x_out_far, y_far)
            .threePointArc((x_out_mid, y_mid), (x_out_near, y_near))
            .lineTo(x_in_near, y_near)
            .threePointArc((x_in_mid, y_mid), (x_in_far, y_far))
            .close()
            .extrude(z_hi - z_lo))


def _handle_paddle(pivot_x, pivot_z, y0, y1):
    """Handle: tapers from ARM_W at the pivot to GRIP_W at the grab end, with a
    rounded (semicircular) bottom — a comfortable pull-tab. Flat extrusion in
    Y, so it prints clean lying on its side."""
    hw_top = ARM_W / 2
    hw_bot = GRIP_W / 2
    z_arc = HANDLE_Z_BOT + hw_bot          # center of the bottom semicircle
    return (cq.Workplane("XZ")
            .moveTo(pivot_x - hw_top, pivot_z)
            .lineTo(pivot_x - hw_bot, z_arc)
            .threePointArc((pivot_x, HANDLE_Z_BOT), (pivot_x + hw_bot, z_arc))
            .lineTo(pivot_x + hw_top, pivot_z)
            .close()
            .extrude(-(y1 - y0)).translate((0, y0, 0)))


def _mesh_pawl(r_in, r_out, y_near, y_far, z_lo, z_hi):
    """Ratchet pawl: an arc block whose rim-facing region is carved by the
    actual rim teeth (± PAWL_MESH_CLR_DEG of backlash) so it meshes — its
    inner face becomes the negative sawtooth that seats onto the rim teeth.
    Because the rim teeth are an asymmetric sawtooth (radial catch face + ramp),
    the meshed pawl LOCKS the unwind direction and CLICKS over the other."""
    block = _arc_strip(r_in, r_out, y_near, y_far, z_lo, z_hi)
    teeth = _cyl_ratchet_band(z_lo - 0.2, z_hi + 0.2)
    cutter = teeth
    for d in (PAWL_MESH_CLR_DEG, -PAWL_MESH_CLR_DEG):
        cutter = cutter.union(teeth.rotate((0, 0, 0), (0, 0, 1), d))
    return block.cut(cutter)


def _pivot_hole(pivot_x, pivot_z, y0, y1):
    """M2 shaft clearance along the pivot axis (Y), with overshoot."""
    return (cq.Workplane("XY").circle(M2_SHAFT_CLR_D / 2)
            .extrude((y1 - y0) + 1)
            .rotate((0, 0, 0), (1, 0, 0), -90)
            .translate((pivot_x, y0 - 0.5, pivot_z)))


def _lever_stop_pin_and_hole(part, pivot_x, pivot_z, lever_alpha, y_inner,
                             y_body_end, side):
    """Add the lever's own stop pin (projecting from the lever's inner face
    into the gap toward the housing) and cut its spring-leg through-hole.
    `side` = +1 for the +Y lever (ratchet), -1 for the -Y lever (brake)."""
    if side > 0:
        y_from, y_to = y_inner - STOP_PIN_H, y_body_end          # 12 .. 19.5
        hole_y = y_inner - STOP_PIN_H + STOP_PIN_HOLE_D
    else:
        y_from, y_to = y_body_end, -(abs(y_inner) - STOP_PIN_H)   # -19.5 .. -12
        hole_y = -(abs(y_inner) - STOP_PIN_H) - STOP_PIN_HOLE_D
    leg_a = lever_alpha + (+1) * SPRING_LEG_PIN_OFFSET_DEG
    part = part.union(stop_pin_solid(pivot_x, pivot_z, lever_alpha, y_from, y_to))
    part = part.cut(stop_pin_hole(pivot_x, pivot_z, lever_alpha, hole_y,
                                  hole_dir_alpha_deg=spring_leg_hole_dir_alpha_deg(leg_a)))
    return part


# ── Ratchet lever ────────────────────────────────────────────────────────────
# Pawl is an arc strip spanning the teeth band (the short BOTTOM band,
# z=0..RATCHET_BAND_H); its inner face meshes with the rim teeth.
PAWL_Z_LO, PAWL_Z_HI = 0.0, RATCHET_BAND_H       # 0 .. 3
PAWL_Y_MID = (RATCHET_Y0 + RATCHET_Y1) / 2       # 17.5
PAWL_Z_MID = (PAWL_Z_LO + PAWL_Z_HI) / 2         # 1.5
_PAWL_R_IN  = R_ROOT                             # 55.7 — seats in the valley
_PAWL_R_OUT = R_RIM + 3.0                         # 60.2 — back of the pawl
_RATCHET_CONTACT_X = (_PAWL_R_IN + _PAWL_R_OUT) / 2   # arm meets the pawl


def _build_ratchet_lever():
    arm    = _bar(RATCHET_PIVOT_X, RATCHET_PIVOT_Z,
                  _RATCHET_CONTACT_X, PAWL_Z_MID, ARM_W,
                  RATCHET_Y0, RATCHET_Y1)
    handle = _handle_paddle(RATCHET_PIVOT_X, RATCHET_PIVOT_Z,
                            RATCHET_Y0, RATCHET_Y1)
    pawl   = _mesh_pawl(_PAWL_R_IN, _PAWL_R_OUT, RATCHET_Y0, RATCHET_Y1,
                        PAWL_Z_LO, PAWL_Z_HI)
    boss   = pivot_boss_sector(RATCHET_PIVOT_X, RATCHET_PIVOT_Z,
                               RATCHET_Y0 - (LEVER_RIM_H - 2.5) / 2, RATCHET_Y1)
    part = arm.union(handle).union(pawl).union(boss)
    part = _lever_stop_pin_and_hole(part, RATCHET_PIVOT_X, RATCHET_PIVOT_Z,
                                    RATCHET_LEVER_PIN_ALPHA, LEVER_INNER_Y,
                                    RATCHET_Y1, side=+1)
    part = part.cut(_pivot_hole(RATCHET_PIVOT_X, RATCHET_PIVOT_Z,
                                RATCHET_Y0 - LEVER_RIM_H, RATCHET_Y1))
    return part


# ── Brake lever ──────────────────────────────────────────────────────────────
# Pad is an arc strip on the upper part of the smooth brake band (the band now
# spans z=RATCHET_BAND_H..RIM_H, but the pad stays high — z=RIM_H/2..RIM_H — so
# its lever arm above the brake pivot stays long enough for A_BRAKE_OVERLAP).
# The BRAKE_RUBBER_T
# rubber slab (viz) bonds to the printed pad's rim-facing face; at the rest
# (disengaged) pose that rubber face sits PAD_REST_GAP off the band (r=57.2),
# so the PRINTED pad body starts BRAKE_RUBBER_T further out. Both faces are
# arcs (conform to the band). Pulling the handle swings the pad in.
PAD_REST_GAP   = 1.0                           # rubber-to-band gap at rest (A_BRAKE_REST)
RUBBER_FACE_R  = R_RIM + PAD_REST_GAP          # 58.2 — rubber rim-facing arc at rest
_PAD_R_IN      = RUBBER_FACE_R + BRAKE_RUBBER_T    # 61.5 — printed pad inner arc
_PAD_R_OUT     = _PAD_R_IN + 3.0               # 64.5 — back of the printed pad
BRAKE_PAD_H = 4.0                              # short contact (was 7) — rubber still grips;
                                               # keeps the pad clear of the cable retainer
PAD_Z_LO, PAD_Z_HI = RIM_H / 2, RIM_H / 2 + BRAKE_PAD_H   # 7 .. 11 — lower band, ≥1 mm
                                               # below the retainer (its bottom ring is at z≈12.3)
PAD_Y_MID = (BRAKE_Y0 + BRAKE_Y1) / 2          # -17.5
PAD_Z_MID = (PAD_Z_LO + PAD_Z_HI) / 2          # 10.5
_BRAKE_CONTACT_X = (_PAD_R_IN + _PAD_R_OUT) / 2


def _build_brake_lever():
    arm    = _bar(BRAKE_PIVOT_X, BRAKE_PIVOT_Z,
                  _BRAKE_CONTACT_X, PAD_Z_MID, ARM_W,
                  BRAKE_Y0, BRAKE_Y1)
    handle = _handle_paddle(BRAKE_PIVOT_X, BRAKE_PIVOT_Z,
                            BRAKE_Y0, BRAKE_Y1)
    pad    = _arc_strip(_PAD_R_IN, _PAD_R_OUT, BRAKE_Y0, BRAKE_Y1,
                        PAD_Z_LO, PAD_Z_HI)
    boss   = pivot_boss_sector(BRAKE_PIVOT_X, BRAKE_PIVOT_Z,
                               BRAKE_Y0, BRAKE_Y1 + (LEVER_RIM_H - 2.5) / 2)
    part = arm.union(handle).union(pad).union(boss)
    part = _lever_stop_pin_and_hole(part, BRAKE_PIVOT_X, BRAKE_PIVOT_Z,
                                    BRAKE_LEVER_PIN_ALPHA, LEVER_INNER_Y,
                                    BRAKE_Y0, side=-1)
    part = part.cut(_pivot_hole(BRAKE_PIVOT_X, BRAKE_PIVOT_Z,
                                BRAKE_Y0, BRAKE_Y1 + LEVER_RIM_H))
    return part


def _apply_rest_precomp(part, pivot_x, pivot_z):
    """Pre-rotate the printed lever by +LEVER_REST_PRECOMP_DEG about its
    pivot (the print-warp pre-compensation), applied as a final step."""
    return part.rotate((pivot_x, 0, pivot_z), (pivot_x, 1, pivot_z),
                       LEVER_REST_PRECOMP_DEG)


def apply_to_main_body(main_body: cq.Workplane) -> cq.Workplane:
    """No lever-dependent cuts — the rim already carries both bands."""
    return main_body


# ── Kinematic assertions (radial) — fire at import ──────────────────────────
# Verifies the geometry meets the design constraints: the two levers travel
# through equal angles (A_ANGLES_MATCH), the pawl fully clears the teeth at
# full pull (A_PAWL_CLEAR), and the brake pad is clear at rest / compressed at
# full pull (A_BRAKE_REST / A_BRAKE_OVERLAP).
_KIN = assert_kinematics(
    ratchet_pivot_x=RATCHET_PIVOT_X, ratchet_pivot_z=RATCHET_PIVOT_Z,
    ratchet_travel_deg=RATCHET_OUTER_TRAVEL_DEG,
    pawl_y_mid=PAWL_Y_MID, pawl_z_mid=PAWL_Z_MID,
    brake_pivot_x=BRAKE_PIVOT_X, brake_pivot_z=BRAKE_PIVOT_Z,
    brake_travel_deg=BRAKE_INNER_TRAVEL_DEG,
    pad_y_mid=PAD_Y_MID, pad_z_mid=PAD_Z_MID, pad_rest_gap=PAD_REST_GAP,
)


ratchet_lever = _apply_rest_precomp(_build_ratchet_lever(),
                                    RATCHET_PIVOT_X, RATCHET_PIVOT_Z)
brake_lever   = _apply_rest_precomp(_build_brake_lever(),
                                    BRAKE_PIVOT_X, BRAKE_PIVOT_Z)
