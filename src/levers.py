"""Ratchet and brake levers — RADIAL engagement, 90°-rotated layout.

Each lever is a flat plate in the X-Z plane (thickness LEVER_T in Y) riding
on a Y-axis M2 pivot screw through the front-housing block. It hangs DOWN
the +X side of the housing; the user reaches under and pulls the handle
toward +X. See housing.py's header for the forced engagement directions:

  - RATCHET pivot sits ABOVE the teeth band (RATCHET_PIVOT_Z=16.1, teeth at
    z=0..RATCHET_BAND_H=6). Rest = ENGAGED: the pawl's inner edge meshes
    against the rim teeth at r=R_ROOT=57.4. Pulling the handle swings the
    pawl outward to clear the teeth.
  - BRAKE pivot sits BELOW the brake band (BRAKE_PIVOT_Z=2, band at
    z=BRAKE_RIM_Z_LO=7.5..RIM_H). Rest = DISENGAGED: the pad sits lifted
    off the smooth band (r=58.9). Pulling the handle swings the pad inward
    onto the band.

The rim already carries both bands (spool.py), so this module no longer
cuts teeth into the body — apply_to_main_body is a no-op pass-through.
"""

import math
import cadquery as cq

from .dimensions import STRUCT_WALL, M2_HEAD_RECESS_D, FIT_CLR, RATCHET_DEPTH, RATCHET_TEETH
from .spool import RIM_OD, RIM_ID, RIM_H, RATCHET_BAND_H, _cyl_ratchet_band
from .helpers import cyl
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
LEVER_T       = 10.0                             # plate thickness (Y) — grown
                                                 # 4→6→10. Wider Y footprint
                                                 # gives more pad surface area
                                                 # (and a longer contact line
                                                 # with the band as the pad
                                                 # sweeps through engagement).
LEVER_HANDLE_W = 6.0                             # vertical handle's X width
                                                 # (and the horizontal arm's
                                                 # +X overshoot past the
                                                 # pivot). Kept at 6 while
                                                 # LEVER_T grew, so the grip
                                                 # stays comfortable in the
                                                 # finger and the +X tail of
                                                 # the horizontal arm doesn't
                                                 # eat into the housing wall.
LEVER_INNER_Y = HOUSING_W / 2 + LEVER_RIM_H      # 14.3 — inner face, past the gap
RATCHET_Y0, RATCHET_Y1 = LEVER_INNER_Y, LEVER_INNER_Y + LEVER_T   #  14.3 .. 24.3
BRAKE_Y1,  BRAKE_Y0    = -LEVER_INNER_Y, -LEVER_INNER_Y - LEVER_T # -14.3 .. -24.3

# M2 head recess in the lever's OUTER face — Ø M2_HEAD_RECESS_D (= 4.1 mm)
# bored LEVER_HEAD_RECESS_H deep. The depth is set so that the M2x20 screw
# shaft is equally split between the lever and the housing:
#
#   shaft_in_lever = LEVER_T - LEVER_HEAD_RECESS_H    (left after the recess)
#   air_gap        = LEVER_INNER_Y - HOUSING_W/2      (= LEVER_RIM_H = 3.3)
#   shaft_in_housing = 20 - shaft_in_lever - air_gap
#
# Setting shaft_in_lever == shaft_in_housing and solving for the depth:
#   LEVER_HEAD_RECESS_H = LEVER_T - (20 - air_gap) / 2
#                       = 10 - 16.7/2 = 1.65
M2_SCREW_LEN          = 20.0
_LEVER_AIR_GAP        = LEVER_INNER_Y - HOUSING_W / 2             # 3.3
_SCREW_IN_LEVER       = (M2_SCREW_LEN - _LEVER_AIR_GAP) / 2       # 8.35
LEVER_HEAD_RECESS_H   = LEVER_T - _SCREW_IN_LEVER                 # 1.65

HANDLE_Z_BOT = -19.0      # common grab height for both handles. Raised again
                          # (was -26) thanks to the ratchet/brake band swap,
                          # which lifts the brake pivot above the spool bottom.

R_RIM        = RIM_OD / 2                        # 58.9 — brake-band / tooth-tip radius

# Rubber pad bonded to the brake pad's rim-facing face (viz + clearance).
BRAKE_RUBBER_T = 3.3


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


def _pivot_hole(pivot_x, pivot_z, y0, y1):
    """M2 shaft CLEARANCE hole along the pivot axis (Y), with overshoot.
    Uses M2_SHAFT_CLR_D = 2.4 mm — oversized so the lever pivots freely on
    the M2 cap-screw. The screw's tight thread-fit lives in the HOUSING's
    pivot bore (LEVER_SCREW_CLR_D = 2.2 mm); the lever just rides on it."""
    return (cq.Workplane("XY").circle(M2_SHAFT_CLR_D / 2)
            .extrude((y1 - y0) + 1)
            .rotate((0, 0, 0), (1, 0, 0), -90)
            .translate((pivot_x, y0 - 0.5, pivot_z)))


def _pivot_head_recess(pivot_x, pivot_z, side):
    """Ø M2_HEAD_RECESS_D × LEVER_HEAD_RECESS_H counterbore in the lever's
    OUTER face for the M2 cap-screw head. side = +1 for the +Y (ratchet)
    lever, -1 for the -Y (brake) lever. Sized so the shaft after the recess
    matches the shaft seated in the housing (LEVER_HEAD_RECESS_H derives
    from that equal-split constraint).

    The recess axis points INWARD (toward the spool centerline): for side
    = +1 the cylinder grows in −Y from the +Y outer face; for side = −1
    it grows in +Y from the −Y outer face. After rotation the cylinder
    base sits at the outer face plus a tiny overshoot, and its other end
    lands LEVER_HEAD_RECESS_H deep inside the lever."""
    overshoot = 0.5
    y_outer   = side * (LEVER_INNER_Y + LEVER_T)
    # extrude along +Z, then rotate so +Z maps to −side·Y (i.e. INTO the lever):
    #   side = +1 → rotate +90 around X (+Z → −Y)
    #   side = −1 → rotate −90 around X (+Z → +Y)
    return (cq.Workplane("XY").circle(M2_HEAD_RECESS_D / 2)
            .extrude(LEVER_HEAD_RECESS_H + overshoot)
            .rotate((0, 0, 0), (1, 0, 0), side * 90)
            .translate((pivot_x, y_outer + side * overshoot, pivot_z)))


def _lever_stop_pin_and_hole(part, pivot_x, pivot_z, lever_alpha, y_inner,
                             y_body_end, side):
    """Add the lever's own stop pin (projecting from the lever's inner face
    into the gap toward the housing) and cut its spring-leg through-hole.
    `side` = +1 for the +Y lever (ratchet), -1 for the -Y lever (brake)."""
    # Spring-leg hole at the halfway point of the EXPOSED pin (the part in
    # the gap, from y_inner - STOP_PIN_H to y_inner) — re-tuning STOP_PIN_H
    # auto-centres the hole.
    if side > 0:
        y_from, y_to = y_inner - STOP_PIN_H, y_body_end          # 12 .. 19.5
        hole_y = y_inner - STOP_PIN_H / 2
    else:
        y_from, y_to = y_body_end, -(abs(y_inner) - STOP_PIN_H)   # -19.5 .. -12
        hole_y = -(abs(y_inner) - STOP_PIN_H / 2)
    leg_a = lever_alpha + (+1) * SPRING_LEG_PIN_OFFSET_DEG
    part = part.union(stop_pin_solid(pivot_x, pivot_z, lever_alpha, y_from, y_to))
    part = part.cut(stop_pin_hole(pivot_x, pivot_z, lever_alpha, hole_y,
                                  hole_dir_alpha_deg=spring_leg_hole_dir_alpha_deg(leg_a)))
    return part


# ── Ratchet lever ────────────────────────────────────────────────────────────
# The horizontal block at z=0..RATCHET_BAND_H is a rectangle from the spool
# axis out to the vertical handle; subtracting the FULL rim solid leaves a
# mating inner edge that's the exact negative of the rim's tooth profile.
PAWL_Z_LO, PAWL_Z_HI = 0.0, RATCHET_BAND_H       # 0 .. 3
PAWL_Y_MID = (RATCHET_Y0 + RATCHET_Y1) / 2       # 18.5 — kinematics sample point
PAWL_Z_MID = (PAWL_Z_LO + PAWL_Z_HI) / 2         # 1.5 — kinematics sample point


def _build_ratchet_lever():
    """Ratchet lever in three primitives:
      1. Horizontal block at PAWL_Z range (3 mm tall in Z): a generous
         rectangle from the spool axis out to the vertical handle's right
         edge, minus the full rim solid — the inner edge becomes the exact
         negative of the rim teeth (perfect mesh at rest).
      2. Vertical handle centered on the pivot's X (LEVER_HANDLE_W wide in X, LEVER_T thick in Y), spanning
         from HANDLE_Z_BOT up past the pivot to RATCHET_PIVOT_Z + hw.
      3. Disc at the bottom of (2) to round off the grip end."""
    hw    = LEVER_HANDLE_W / 2
    y_len = RATCHET_Y1 - RATCHET_Y0

    # 1. Horizontal block. h_x_lo at the spool axis (=0) so the rectangle
    #    sits INSIDE the rim's outer profile at every y in [Y0, Y1] — the
    #    rim subtraction is then the sole determinant of the inner edge.
    #    (A larger h_x_lo such as RIM_ID/2 would sit outside the rim at
    #    y values where the outer profile dips inward of the rectangle
    #    bound, leaving a visible gap between the rim and the lever.)
    h_x_hi = RATCHET_PIVOT_X + hw
    h_x_lo = 0.0
    horiz_block = (cq.Workplane("XY").workplane(offset=PAWL_Z_LO)
                   .moveTo(h_x_lo, RATCHET_Y0).lineTo(h_x_hi, RATCHET_Y0)
                   .lineTo(h_x_hi, RATCHET_Y1).lineTo(h_x_lo, RATCHET_Y1)
                   .close()
                   .extrude(PAWL_Z_HI - PAWL_Z_LO))
    # Full rim solid at the tooth band = toothed annular ring + the central
    # cylinder it sits on (so the cut removes everything inside the rim's
    # outer profile, not just the tooth band itself).
    teeth_ring = _cyl_ratchet_band(PAWL_Z_LO - 0.2, PAWL_Z_HI + 0.2)
    core = cyl(RIM_ID, (PAWL_Z_HI - PAWL_Z_LO) + 0.4, z=PAWL_Z_LO - 0.2)
    horiz = horiz_block.cut(teeth_ring.union(core))

    # Catch-face bump on the lever's inner-Y face. Built as a PARALLELOGRAM
    # with one edge along the rim's catch face (radial at ts) and the
    # opposite parallel edge offset along the rim's CHORD direction
    # (valley_i → tip_{i+1}). The chord-direction back edge tracks the
    # actual rim outer at the angles just CCW of the catch face — a
    # straight axis-perpendicular back edge instead would diverge from
    # the chord and leave a triangular sliver between the bump and the
    # rim's outer boundary.
    #
    # Vertices (CCW: A → D → C → B):
    #   A = outer-catch: on the catch face at radius R_RIM + outer_overlap
    #       (overlapped into the lever's existing material so the union
    #       closes solidly there).
    #   B = inner-catch: at radius r_root on the catch face = the rim's
    #       valley_i vertex itself — shares this point with the rim's
    #       polygon exactly.
    #   C = B + chord_dir × bump_depth, on the rim's chord at distance
    #       bump_depth — so edge BC is a segment of the rim's chord.
    #   D = A + chord_dir × bump_depth, the matching offset at the outer
    #       end.
    _BUMP_TS            = math.asin(RATCHET_Y0 / (R_RIM + FIT_CLR))
    _BUMP_TS_NEXT       = _BUMP_TS + 2 * math.pi / RATCHET_TEETH
    _BUMP_OUTER_OVERLAP = 0.5
    _BUMP_DEPTH         = 0.4
    _R_ROOT             = R_RIM - RATCHET_DEPTH
    # Chord direction = (tip_{i+1} − valley_i) normalised.
    _VALLEY = (_R_ROOT * math.cos(_BUMP_TS), _R_ROOT * math.sin(_BUMP_TS))
    _TIP_NEXT = (R_RIM * math.cos(_BUMP_TS_NEXT), R_RIM * math.sin(_BUMP_TS_NEXT))
    _CHORD_VEC = (_TIP_NEXT[0] - _VALLEY[0], _TIP_NEXT[1] - _VALLEY[1])
    _CHORD_LEN = math.hypot(*_CHORD_VEC)
    _CHORD_DIR = (_CHORD_VEC[0] / _CHORD_LEN, _CHORD_VEC[1] / _CHORD_LEN)
    _A = ((R_RIM + _BUMP_OUTER_OVERLAP) * math.cos(_BUMP_TS),
          (R_RIM + _BUMP_OUTER_OVERLAP) * math.sin(_BUMP_TS))
    _B = _VALLEY
    _C = (_B[0] + _BUMP_DEPTH * _CHORD_DIR[0],
          _B[1] + _BUMP_DEPTH * _CHORD_DIR[1])
    _D = (_A[0] + _BUMP_DEPTH * _CHORD_DIR[0],
          _A[1] + _BUMP_DEPTH * _CHORD_DIR[1])
    bump = (cq.Workplane("XY").workplane(offset=PAWL_Z_LO)
            .polyline([_A, _D, _C, _B]).close()
            .extrude(PAWL_Z_HI - PAWL_Z_LO))
    horiz = horiz.union(bump)

    # 2. Vertical handle centered on the pivot's X, from HANDLE_Z_BOT up to
    #    RATCHET_PIVOT_Z + hw (so the pivot sits inside it).
    vert_top = RATCHET_PIVOT_Z + hw
    vert = (cq.Workplane("XZ")
            .polyline([(RATCHET_PIVOT_X - hw, HANDLE_Z_BOT),
                       (RATCHET_PIVOT_X + hw, HANDLE_Z_BOT),
                       (RATCHET_PIVOT_X + hw, vert_top),
                       (RATCHET_PIVOT_X - hw, vert_top)]).close()
            .extrude(-y_len).translate((0, RATCHET_Y0, 0)))

    # 3. Round off the bottom of the vertical handle.
    bottom = (cq.Workplane("XZ")
              .center(RATCHET_PIVOT_X, HANDLE_Z_BOT)
              .circle(hw)
              .extrude(-y_len).translate((0, RATCHET_Y0, 0)))

    boss = pivot_boss_sector(RATCHET_PIVOT_X, RATCHET_PIVOT_Z,
                             RATCHET_Y0 - (LEVER_RIM_H - 2.5) / 2, RATCHET_Y1)

    part = horiz.union(vert).union(bottom).union(boss)
    part = _lever_stop_pin_and_hole(part, RATCHET_PIVOT_X, RATCHET_PIVOT_Z,
                                    RATCHET_LEVER_PIN_ALPHA, LEVER_INNER_Y,
                                    RATCHET_Y1, side=+1)
    part = part.cut(_pivot_hole(RATCHET_PIVOT_X, RATCHET_PIVOT_Z,
                                RATCHET_Y0 - LEVER_RIM_H, RATCHET_Y1))
    part = part.cut(_pivot_head_recess(RATCHET_PIVOT_X, RATCHET_PIVOT_Z, side=+1))
    return part


# ── Brake lever ──────────────────────────────────────────────────────────────
# Pad is an arc strip on the upper part of the smooth brake band (the band now
# spans z=RATCHET_BAND_H..RIM_H, but the pad stays high — z=RIM_H/2..RIM_H — so
# its lever arm above the brake pivot stays long enough for A_BRAKE_OVERLAP).
# The BRAKE_RUBBER_T
# rubber slab (viz) bonds to the printed pad's rim-facing face; at the rest
# (disengaged) pose that rubber face sits PAD_REST_GAP off the band (r=58.9),
# so the PRINTED pad body starts BRAKE_RUBBER_T further out. Both faces are
# arcs (conform to the band). Pulling the handle swings the pad in.
# Pad top margin to the cable retainer's bottom ring (at RIM_H − STRUCT_WALL):
# 1.0 mm of true clearance + 0.8 mm for the rubber band-face's upward drift.
# (The rubber slab is extruded along the tilted mount-plane NORMAL, whose +Z
# component lifts the band-side top corner BRAKE_RUBBER_T·W_z ≈ 0.8 mm above
# PAD_Z_HI — without this allowance the actual corner gap was only 0.2 mm.)
PAD_Z_HI = RIM_H - STRUCT_WALL - 1.8
PAD_Y_MID = (BRAKE_Y0 + BRAKE_Y1) / 2

# ── Ratchet-brake handoff: brake first contact = ratchet pawl clears tips ──
# Clean handoff means the pawl is just clearing the tooth tips at the same
# angle the brake-pad face center first touches the band. Before this angle,
# only the ratchet is active; after it, only the brake.
def _ratchet_pawl_clear_angle():
    x_pawl_rest = math.sqrt(R_ROOT ** 2 - PAWL_Y_MID ** 2)
    target_x    = math.sqrt(R_RIM ** 2  - PAWL_Y_MID ** 2)
    dx          = x_pawl_rest - RATCHET_PIVOT_X
    dz          = PAWL_Z_MID  - RATCHET_PIVOT_Z
    A   = dx
    B   = -dz
    rhs = target_x - RATCHET_PIVOT_X
    R   = math.hypot(A, B)
    phi = math.degrees(math.atan2(B, A))
    ac  = math.degrees(math.acos(max(-1.0, min(1.0, rhs / R))))
    return min(s for s in (phi - ac, phi + ac)
               if 0 < s < RATCHET_OUTER_TRAVEL_DEG)

RATCHET_PAWL_CLEAR_DEG = _ratchet_pawl_clear_angle()

# Extra pull angle between the pawl clearing the teeth and the brake pad
# first touching the band. 0 = the original "clean handoff" (pad touches the
# instant the pawl clears). Raising it buys rest-gap margin between the pad
# and the band (the pad starts further out) at the cost of a small dead zone
# in the pull where neither ratchet nor brake is active.
BRAKE_CONTACT_DELAY_DEG = 3.0


# ── Joint solver: BRAKE_ARM_X_LO + BRAKE_PAD_H + pre-tilt θ ────────────────
#
# Constraints (with brake-rim cylinder at r=R_RIM):
#
#   1. FIRST CONTACT TIMING — the band-side face center reaches r=R_RIM at
#      α_first = RATCHET_PAWL_CLEAR_DEG (clean handoff). Sets BRAKE_ARM_X_LO,
#      which depends on z_fc = PAD_Z_HI − h/2 (and hence on h).
#
#   2. MID-CONTACT PARALLELISM — at α_mid = (α_first + α_max)/2, the face is
#      parallel to the band tangent at PAD_Y_MID. Sets the pre-tilt θ = α_mid.
#
#   3. PAD-BOTTOM LANDING — at α_mid, the band-side BOTTOM corner at y=BRAKE_Y0
#      (the pad's lowest-z point after rotation) lands at z = BRAKE_RIM_Z_LO,
#      the brake rim's straight-section bottom. Sets BRAKE_PAD_H.
#
# PAD_REST_GAP (the radial gap at the face center at rest) is DERIVED — it
# falls out of constraint 1. Delaying first contact pushes the pad further
# from the band at rest; that's the intended consequence.
def _solve_brake_pad_geom():
    from .spool import BRAKE_RIM_Z_LO
    px, pz       = BRAKE_PIVOT_X, BRAKE_PIVOT_Z
    alpha_max    = BRAKE_INNER_TRAVEL_DEG
    x_band_y_mid = math.sqrt(R_RIM ** 2 - PAD_Y_MID ** 2)
    y_half       = (BRAKE_Y1 - BRAKE_Y0) / 2
    alpha_first  = RATCHET_PAWL_CLEAR_DEG + BRAKE_CONTACT_DELAY_DEG
    alpha_mid    = (alpha_first + alpha_max) / 2
    a_rad        = math.radians(alpha_mid)
    af_rad       = math.radians(alpha_first)

    def brake_arm_x_lo_for_h(h):
        """BRAKE_ARM_X_LO such that face center reaches band at alpha_first."""
        dz = (PAD_Z_HI - h / 2) - pz
        # px + (x_fc_band - px)*cos(af) - dz*sin(af) = x_band_y_mid
        x_fc_band = px + (x_band_y_mid - px + dz * math.sin(af_rad)) \
                       / math.cos(af_rad)
        return x_fc_band + BRAKE_RUBBER_T

    def pad_bottom_z_at_mid(h):
        a = brake_arm_x_lo_for_h(h)
        tilt_dx_x = (h / 2) * math.tan(a_rad)
        tilt_dx_y = y_half * abs(PAD_Y_MID) / (math.cos(a_rad) * x_band_y_mid)
        x_corner = a - tilt_dx_x - tilt_dx_y - BRAKE_RUBBER_T
        z_corner = PAD_Z_HI - h
        new_z = (pz + (x_corner - px) * math.sin(a_rad)
                    + (z_corner - pz) * math.cos(a_rad))
        return new_z, a

    lo, hi = 0.5, 10.0
    for _ in range(50):
        mid = (lo + hi) / 2
        nz, _ = pad_bottom_z_at_mid(mid)
        if nz > BRAKE_RIM_Z_LO:
            lo = mid
        else:
            hi = mid
        if hi - lo < 0.001:
            break
    h_final = (lo + hi) / 2
    _, a_final = pad_bottom_z_at_mid(h_final)
    return a_final, h_final, alpha_mid

BRAKE_ARM_X_LO, BRAKE_PAD_H, BRAKE_PAD_MOUNT_TILT_DEG = _solve_brake_pad_geom()
PAD_Z_LO = PAD_Z_HI - BRAKE_PAD_H
RUBBER_PAD_Z_LO = PAD_Z_LO
# Derived rest gap (whatever the handoff timing produced).
PAD_REST_GAP = (math.hypot(BRAKE_ARM_X_LO - BRAKE_RUBBER_T, PAD_Y_MID)
                - R_RIM)

# ── Rubber pad: true RECTANGLE on the mount plane (square strip cuts) ───────
# The mount face is a doubly-tilted plane (X-tilt = BRAKE_PAD_MOUNT_TILT_DEG
# about Y for mid-travel verticality; Y-tilt slope _C for band tangency at
# PAD_Y_MID). The lever arm's face region is a PARALLELOGRAM on that plane:
# its edges follow world z=const / y=const planes, which meet at ~85°
# in-plane — so a pad matching it needs angled cuts. The physical pad is cut
# from BRAKE_RUBBER_T strip stock with SQUARE cuts (one for length, one for
# width), so the modeled pad is instead the largest axis-aligned RECTANGLE
# inscribed in that parallelogram:
#   U — in-plane unit vector along the z=const edges (horizontal in world)
#   V — in-plane unit vector perpendicular to U
#   L = base − side·sin(skew)   (length along U, the strip-length cut)
#   H = side · cos(skew)        (width along V, the strip-width cut)
# Inscribed ⊂ parallelogram → the existing arm face fully backs the pad; the
# arm needs no redesign. The pad's end faces lean ~5° from vertical in world
# (they're square ON THE STRIP), and its top/bottom edges stay horizontal.
_C = abs(PAD_Y_MID) / (math.cos(math.radians(BRAKE_PAD_MOUNT_TILT_DEG))
                       * math.sqrt(R_RIM ** 2 - PAD_Y_MID ** 2))   # face dx per unit y
_B = math.tan(math.radians(BRAKE_PAD_MOUNT_TILT_DEG))              # face dx per unit z
_NU = math.sqrt(1 + _C * _C)
_NE = math.sqrt(1 + _B * _B)
_U  = (_C / _NU, 1 / _NU, 0.0)            # in-plane, along z=const edges
_E  = (_B / _NE, 0.0, 1 / _NE)            # in-plane, along y=const edges
_SINPHI = _B * _C / (_NU * _NE)           # in-plane skew between _E and _V
_COSPHI = math.sqrt(1 - _SINPHI ** 2)
_V  = tuple((e - _SINPHI * u) / _COSPHI for e, u in zip(_E, _U))
# Outward normal of the mount face, pointing toward the band (-X side).
_W = (_U[1] * _V[2] - _U[2] * _V[1],
      _U[2] * _V[0] - _U[0] * _V[2],
      _U[0] * _V[1] - _U[1] * _V[0])
if _W[0] > 0:
    _W = (-_W[0], -_W[1], -_W[2])

_PAD_PARA_BASE = (BRAKE_Y1 - BRAKE_Y0) * _NU       # parallelogram base length (along U)
_PAD_PARA_SIDE = BRAKE_PAD_H * _NE                 # parallelogram side length (along E)
BRAKE_PAD_RECT_L = _PAD_PARA_BASE - _PAD_PARA_SIDE * _SINPHI   # strip-length cut
BRAKE_PAD_RECT_H = _PAD_PARA_SIDE * _COSPHI                    # strip-width cut

_PAD_FACE_CENTER = (BRAKE_ARM_X_LO, PAD_Y_MID, (PAD_Z_LO + PAD_Z_HI) / 2)


def brake_pad_rect_corners():
    """(lever_face_corners, band_face_corners) of the rectangular rubber
    pad — each a list of 4 (x, y, z) tuples in consistent winding order.
    Lever-face rectangle is centred on the mount face; band-face is the
    same rectangle offset BRAKE_RUBBER_T along the outward normal."""
    cx, cy, cz = _PAD_FACE_CENTER
    lever, band = [], []
    for su, sv in ((-1, -1), (+1, -1), (+1, +1), (-1, +1)):
        px = cx + su * BRAKE_PAD_RECT_L / 2 * _U[0] + sv * BRAKE_PAD_RECT_H / 2 * _V[0]
        py = cy + su * BRAKE_PAD_RECT_L / 2 * _U[1] + sv * BRAKE_PAD_RECT_H / 2 * _V[1]
        pz = cz + su * BRAKE_PAD_RECT_L / 2 * _U[2] + sv * BRAKE_PAD_RECT_H / 2 * _V[2]
        lever.append((px, py, pz))
        band.append((px + BRAKE_RUBBER_T * _W[0],
                     py + BRAKE_RUBBER_T * _W[1],
                     pz + BRAKE_RUBBER_T * _W[2]))
    return lever, band


def _build_brake_lever():
    """Brake lever in three primitives. The contact face (left side of the
    horizontal arm) is a straight vertical plane at x=BRAKE_ARM_X_LO; the
    brake pad sits flush against it with its own straight right side.
      1. Horizontal arm at PAD_Z range, x ∈ [BRAKE_ARM_X_LO, BRAKE_PIVOT_X+hw].
      2. Vertical handle centered on the pivot's X (LEVER_HANDLE_W wide in X, LEVER_T thick in Y), from
         HANDLE_Z_BOT up through the pivot to PAD_Z_HI.
      3. Disc at the bottom of (2) to round off the grip end."""
    hw    = LEVER_HANDLE_W / 2
    y_len = BRAKE_Y1 - BRAKE_Y0

    # 1. Horizontal arm. Left face (the pad-mount surface) is pre-tilted
    #    in BOTH the XZ plane (about Y) and the XY plane (about Z) so the
    #    face matches the band's tangent at PAD_Y_MID, mid-travel.
    #      X-tilt: leans OUTWARD as z increases (top at larger x).
    #      Y-tilt: leans OUTWARD as y increases (BRAKE_Y1 corner at larger x).
    #    After rotating CW by BRAKE_PAD_MOUNT_TILT_DEG, the X-tilt is undone
    #    (face vertical in XZ); the Y-tilt remains, matching the band's
    #    dx/dy slope at PAD_Y_MID for ideal flat contact at midpoint.
    h_x_lo = BRAKE_ARM_X_LO
    h_x_hi = BRAKE_PIVOT_X + hw
    tilt_dx_x = (BRAKE_PAD_H / 2) * math.tan(math.radians(BRAKE_PAD_MOUNT_TILT_DEG))
    # Y-tilt: face's dx/dy at rest matches the band's tangent slope (rotated
    # back by the X-tilt angle), so post-rotation dx/dy = |y_mid| / x_band.
    x_band_at_pad_y_mid = math.sqrt(R_RIM ** 2 - PAD_Y_MID ** 2)
    dx_dy_at_rest = (abs(PAD_Y_MID) /
                     (math.cos(math.radians(BRAKE_PAD_MOUNT_TILT_DEG))
                      * x_band_at_pad_y_mid))
    tilt_dx_y = (y_len / 2) * dx_dy_at_rest
    # Build by lofting two XZ parallelograms — one at y=BRAKE_Y0, one at
    # y=BRAKE_Y1 — with the left-face X shifted by ±tilt_dx_y.
    # cq.Workplane("XZ") has normal -Y, so workplane(offset=d) moves -dY.
    poly_y0 = [
        (h_x_lo - tilt_dx_x - tilt_dx_y, PAD_Z_LO),
        (h_x_hi,                          PAD_Z_LO),
        (h_x_hi,                          PAD_Z_HI),
        (h_x_lo + tilt_dx_x - tilt_dx_y, PAD_Z_HI),
    ]
    poly_y1 = [
        (h_x_lo - tilt_dx_x + tilt_dx_y, PAD_Z_LO),
        (h_x_hi,                          PAD_Z_LO),
        (h_x_hi,                          PAD_Z_HI),
        (h_x_lo + tilt_dx_x + tilt_dx_y, PAD_Z_HI),
    ]
    horiz = (cq.Workplane("XZ").workplane(offset=-BRAKE_Y0)
             .polyline(poly_y0).close()
             .workplane(offset=BRAKE_Y0 - BRAKE_Y1)
             .polyline(poly_y1).close()
             .loft(ruled=True, combine=True))

    # 2. Vertical handle centered on the pivot's X, from HANDLE_Z_BOT up to
    #    PAD_Z_HI (overlapping the horizontal arm at the top).
    vert = (cq.Workplane("XZ")
            .polyline([(BRAKE_PIVOT_X - hw, HANDLE_Z_BOT),
                       (BRAKE_PIVOT_X + hw, HANDLE_Z_BOT),
                       (BRAKE_PIVOT_X + hw, PAD_Z_HI),
                       (BRAKE_PIVOT_X - hw, PAD_Z_HI)]).close()
            .extrude(-y_len).translate((0, BRAKE_Y0, 0)))

    # 3. Round off the bottom of the vertical handle.
    bottom = (cq.Workplane("XZ")
              .center(BRAKE_PIVOT_X, HANDLE_Z_BOT)
              .circle(hw)
              .extrude(-y_len).translate((0, BRAKE_Y0, 0)))

    boss = pivot_boss_sector(BRAKE_PIVOT_X, BRAKE_PIVOT_Z,
                             BRAKE_Y0, BRAKE_Y1 + (LEVER_RIM_H - 2.5) / 2)

    part = horiz.union(vert).union(bottom).union(boss)
    part = _lever_stop_pin_and_hole(part, BRAKE_PIVOT_X, BRAKE_PIVOT_Z,
                                    BRAKE_LEVER_PIN_ALPHA, LEVER_INNER_Y,
                                    BRAKE_Y0, side=-1)
    part = part.cut(_pivot_hole(BRAKE_PIVOT_X, BRAKE_PIVOT_Z,
                                BRAKE_Y0, BRAKE_Y1 + LEVER_RIM_H))
    part = part.cut(_pivot_head_recess(BRAKE_PIVOT_X, BRAKE_PIVOT_Z, side=-1))
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
# Brake test point: pad's max-compression corner = (BRAKE_Y0, PAD_Z_HI).
# BRAKE_Y0 has the largest |y| in the pad's Y range; PAD_Z_HI is the top of the
# pad, furthest from the brake pivot (z=2) in +Z. That corner swings furthest
# at full pull and engages the band first.
# Build the actual pad/arm corner positions (with X- and Y-tilts) so the
# kinematic assertions run against real geometry, not a band-conforming hypo.
_TDX_X = (BRAKE_PAD_H / 2) * math.tan(math.radians(BRAKE_PAD_MOUNT_TILT_DEG))
_TDX_Y = ((BRAKE_Y1 - BRAKE_Y0) / 2) * abs(PAD_Y_MID) / (
    math.cos(math.radians(BRAKE_PAD_MOUNT_TILT_DEG))
    * math.sqrt(R_RIM ** 2 - PAD_Y_MID ** 2))

# Lever-arm left-face x at each (y_sign, z_sign) — sign +1 at top/at-Y1.
def _lever_left_x(y_sign, z_sign):
    return BRAKE_ARM_X_LO + z_sign * _TDX_X + y_sign * _TDX_Y

# Band-side rubber-pad corners — the RECTANGULAR pad's actual band face.
_PAD_BAND_CORNERS = brake_pad_rect_corners()[1]
# Lever arm's bottom-inner edge corners (at z=PAD_Z_LO, on the lever face)
_ARM_BOTTOM_CORNERS = [
    (_lever_left_x(-1, -1), BRAKE_Y0, PAD_Z_LO),
    (_lever_left_x(+1, -1), BRAKE_Y1, PAD_Z_LO),
]

_KIN = assert_kinematics(
    ratchet_pivot_x=RATCHET_PIVOT_X, ratchet_pivot_z=RATCHET_PIVOT_Z,
    ratchet_travel_deg=RATCHET_OUTER_TRAVEL_DEG,
    pawl_y_mid=PAWL_Y_MID, pawl_z_mid=PAWL_Z_MID,
    brake_pivot_x=BRAKE_PIVOT_X, brake_pivot_z=BRAKE_PIVOT_Z,
    brake_travel_deg=BRAKE_INNER_TRAVEL_DEG,
    pad_band_corners=_PAD_BAND_CORNERS,
    arm_bottom_corners=_ARM_BOTTOM_CORNERS,
    ratchet_band_h=RATCHET_BAND_H,
)


ratchet_lever = _apply_rest_precomp(_build_ratchet_lever(),
                                    RATCHET_PIVOT_X, RATCHET_PIVOT_Z)
brake_lever   = _apply_rest_precomp(_build_brake_lever(),
                                    BRAKE_PIVOT_X, BRAKE_PIVOT_Z)
