"""Radial-engagement lever kinematics + load-time assertions.

Replaces the old AXIAL A1–A5 suite. The levers were rotated 90°, so the
contacts now engage the rim's OUTER cylindrical face and "throw" is a RADIAL
displacement produced by rotating the contact point about the (Y-axis) pivot
by the pull travel.

Assertions (fire at import via assert_kinematics, called from levers.py):

  A_ANGLES_MATCH  : both levers travel through the same angle, so they start
                    (rest) and end (full pull) at the same orientation.
  A_PAWL_CLEAR    : at full pull, the ratchet pawl's rim-contact point has
                    swung radially OUT past the tooth tips by ≥ PAWL_CLEAR_MM.
  A_BRAKE_REST    : at rest, the brake rubber face sits ≥ BRAKE_REST_MM off
                    the band (disengaged).
  A_BRAKE_OVERLAP : at full pull, the brake rubber face has swung radially IN
                    to ≥ BRAKE_OVERLAP_MM past the band surface (compressed).
  A_BRAKE_RATCHET_CLR : at full pull, the brake assembly's closest corners
                    (lever arm bottom-inner edge and rubber pad band-side
                    bottom) each keep ≥ BRAKE_RATCHET_CLR_MM 3-D distance
                    from the ratchet tooth box (worst-case tooth alignment).

The pawl contact is sampled at the lever's mid-azimuth and the band's mid-Z
— representative of where the pawl tooth meshes.

The brake contact is sampled at the pad's MAX-COMPRESSION corner: the y with
the largest |y| (furthest from the spool axis) and the z furthest from the
brake pivot in +Z (the pad's top edge for a pivot below the pad). That corner
swings the most at full pull, so it engages the band first and most deeply.
Testing at the corner instead of the mid-point lets the pad grow downward (in
-Z) without artificially failing A_BRAKE_OVERLAP — what matters is that
SOME part of the pad engages, and the corner is the first part to do so.
"""

import math

from .spool import RIM_OD
from .dimensions import RATCHET_DEPTH

# Brake-band outer radius and ratchet tooth-tip radius are now EQUAL (the brake
# band was bumped 1.7→RATCHET_DEPTH so it matches the tip extent — no overhang).
# The teeth grow INWARD from the brake-band outer to a valley at R_BAND −
# RATCHET_DEPTH; a 45° cone (in spool._lever_rim) bridges between the two.
R_BAND        = RIM_OD / 2                  # 58.9 — brake-band surface (= ratchet tooth tip)
R_RATCHET_TIP = R_BAND                      # 58.9 — tooth tip flush with brake band's outer
                                            #        (no overhang; the cone transitions the
                                            #        wall between them)
R_ROOT        = R_BAND - RATCHET_DEPTH      # 57.4 — pawl seats on the valley floor

PAWL_CLEAR_MM           = 1.0
BRAKE_REST_MM           = 1.0
BRAKE_OVERLAP_MM        = 0.5
BRAKE_RATCHET_CLR_MM    = 1.0    # min 3-D clearance, brake pad ↔ nearest tooth
                                 # (worst-case alignment), at full engagement
_EPS = 1e-6


def _rotate_xz_about(x, z, px, pz, theta_deg):
    """Rotate (x, z) about (px, pz) by theta_deg about +Y.
    (Pull = -travel; matches build.py's assembly transform.)"""
    t = math.radians(theta_deg)
    dx, dz = x - px, z - pz
    return (px + dx * math.cos(t) + dz * math.sin(t),
            pz - dx * math.sin(t) + dz * math.cos(t))


def _contact_radius_after_pull(pivot_x, pivot_z, r_rest, y, z, travel_deg):
    """Radius (in XY) of a rim point at radius r_rest, at the given y and z,
    after the lever is pulled (rotated -travel about +Y at the pivot)."""
    x_rest = math.sqrt(max(0.0, r_rest ** 2 - y ** 2))
    x2, _z2 = _rotate_xz_about(x_rest, z, pivot_x, pivot_z, -travel_deg)
    return math.hypot(x2, y)


def assert_kinematics(*, ratchet_pivot_x, ratchet_pivot_z, ratchet_travel_deg,
                      pawl_y_mid, pawl_z_mid,
                      brake_pivot_x, brake_pivot_z, brake_travel_deg,
                      pad_band_corners,
                      arm_bottom_corners,
                      ratchet_band_h):
    """Brake assertions evaluate the ACTUAL pad geometry — the band-side
    face of the (flat, tilted) rubber pad has 4 explicit corners passed in
    as `pad_band_corners` = [(x, y, z), ...], and the lever-arm's bottom-
    inner edge has 2 corners as `arm_bottom_corners`. We rotate each by the
    travel and inspect every one — no band-conforming hypothetical."""
    # A_ANGLES_MATCH ----------------------------------------------------------
    assert abs(ratchet_travel_deg - brake_travel_deg) < _EPS, (
        f"A_ANGLES_MATCH: lever travels differ "
        f"({ratchet_travel_deg}° vs {brake_travel_deg}°) — the levers would "
        f"not start/end at the same angles.")

    # A_PAWL_CLEAR ------------------------------------------------------------
    # Pawl seats in the valley (r=R_ROOT) at rest; after the disengage pull it
    # must clear the tooth tips (r=R_RIM) by ≥ PAWL_CLEAR_MM.
    r_pawl = _contact_radius_after_pull(ratchet_pivot_x, ratchet_pivot_z,
                                        R_ROOT, pawl_y_mid, pawl_z_mid,
                                        ratchet_travel_deg)
    need = R_RATCHET_TIP + PAWL_CLEAR_MM
    assert r_pawl >= need - _EPS, (
        f"A_PAWL_CLEAR: pawl reaches r={r_pawl:.2f} mm at full pull, "
        f"need ≥ {need:.2f} (tip {R_RATCHET_TIP:.2f} + {PAWL_CLEAR_MM}). "
        f"Raise RATCHET_PIVOT_Z or increase travel.")

    # ── Brake-pad rest gaps (real geometry) ──────────────────────────────────
    # At rest each pad band-side corner sits at some radius from the spool
    # axis. The MIN gap (across all 4 corners) is the worst-case rest
    # clearance. The DESIGN gap (at the face center) is set elsewhere; here
    # we just verify no corner is already inside the band.
    rest_gaps = []
    for x, y, _z in pad_band_corners:
        rest_gaps.append(math.hypot(x, y) - R_BAND)
    min_rest_gap = min(rest_gaps)
    assert min_rest_gap > -_EPS, (
        f"A_BRAKE_REST: a pad band-side corner is INSIDE the band at rest "
        f"(min gap {min_rest_gap:.3f} mm). Move BRAKE_ARM_X_LO outward.")

    # ── Brake overlap at full pull (real geometry) ───────────────────────────
    # Rotate every pad band-side corner by -brake_travel_deg about the brake
    # pivot, compute the corner's new radius, and check the DEEPEST corner
    # makes ≥ BRAKE_OVERLAP_MM of compression into the band.
    overlaps = []
    for x, y, z in pad_band_corners:
        nx, _nz = _rotate_xz_about(x, z,
                                   brake_pivot_x, brake_pivot_z,
                                   -brake_travel_deg)
        overlaps.append((R_BAND - math.hypot(nx, y), x, y, z))
    overlaps.sort(reverse=True)               # deepest first
    max_overlap, mx, my, mz = overlaps[0]
    assert max_overlap >= BRAKE_OVERLAP_MM - _EPS, (
        f"A_BRAKE_OVERLAP: deepest pad corner overlaps band by "
        f"{max_overlap:.2f} mm at full pull (corner y={my:.2f}, z={mz:.2f}), "
        f"need ≥ {BRAKE_OVERLAP_MM}. Lower BRAKE_PIVOT_Z, increase travel, "
        f"or shrink PAD_REST_GAP.")

    # ── A_BRAKE_RATCHET_CLR (real geometry) ──────────────────────────────────
    # At full engagement, the brake assembly's bottom edges sweep close to
    # the ratchet teeth. We check the bottom-z 2 corners of pad_band_corners
    # plus the 2 arm_bottom_corners. Each must keep ≥ BRAKE_RATCHET_CLR_MM
    # 3-D distance from the tooth box {r ∈ [R_BAND, R_RATCHET_TIP],
    # z ∈ [0, ratchet_band_h]}.
    pad_z_sorted = sorted(pad_band_corners, key=lambda c: c[2])
    bottom_pad_corners = [c for c in pad_band_corners
                          if abs(c[2] - pad_z_sorted[0][2]) < _EPS]

    def _box_dist(x_rest, y, z_rest):
        nx, nz = _rotate_xz_about(x_rest, z_rest,
                                  brake_pivot_x, brake_pivot_z,
                                  -brake_travel_deg)
        r = math.hypot(nx, y)
        r_gap = max(0.0, R_BAND - r, r - R_RATCHET_TIP)
        z_gap = max(0.0, -nz, nz - ratchet_band_h)
        return math.hypot(r_gap, z_gap), nx, nz, r

    worst = None
    for label, corners in (
        ("arm bottom-inner edge", arm_bottom_corners),
        ("pad band-side bottom",  bottom_pad_corners),
    ):
        for x, y, z in corners:
            d, nx, nz, r = _box_dist(x, y, z)
            if worst is None or d < worst[0]:
                worst = (d, nx, nz, r, label, y)

    worst_d, nx, nz, r, label, y_check = worst
    if worst_d < BRAKE_RATCHET_CLR_MM - _EPS:
        import sys
        print(
            f"WARN  A_BRAKE_RATCHET_CLR: {label} (y={y_check:.2f}) ends at "
            f"(x={nx:.2f}, z={nz:.2f}) → r={r:.2f}; 3-D distance to ratchet "
            f"tooth box = {worst_d:.2f} mm, want ≥ {BRAKE_RATCHET_CLR_MM}.",
            file=sys.stderr, flush=True)

    r_pad = R_BAND - max_overlap
    return {
        "pawl_full_pull_r": r_pawl,
        "pawl_clearance_mm": r_pawl - R_RATCHET_TIP,
        "pad_min_rest_gap_mm": min_rest_gap,
        "pad_max_overlap_mm": max_overlap,
        "pad_full_pull_r": r_pad,
        "brake_ratchet_clr_mm": worst_d,
        "brake_ratchet_clr_label": label,
    }
