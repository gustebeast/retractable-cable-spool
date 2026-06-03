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

The contact points are taken on the rim cylinder at the lever's mid-azimuth
(its y-midline) and the band's mid-Z — representative of where each contact
actually bears.
"""

import math

from .spool import RIM_OD
from .dimensions import RATCHET_DEPTH

# The ratchet teeth now PROTRUDE past the brake band: the valley floor is flush
# with the band (so it supports it), and the tips stick out RATCHET_DEPTH. So the
# brake band and the pawl-tip radii are no longer the same value.
R_BAND        = RIM_OD / 2                  # 57.2 — brake-band surface = ratchet valley floor
R_RATCHET_TIP = R_BAND + RATCHET_DEPTH      # 58.7 — ratchet tooth tip (protrudes past the band)
R_ROOT        = R_BAND                      # 57.2 — pawl seats on the valley floor (flush
                                            #         with the band, fully supporting it)

PAWL_CLEAR_MM    = 1.0
BRAKE_REST_MM    = 1.0
BRAKE_OVERLAP_MM = 0.5
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
                      pad_y_mid, pad_z_mid, pad_rest_gap):
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

    # A_BRAKE_REST ------------------------------------------------------------
    assert pad_rest_gap >= BRAKE_REST_MM - _EPS, (
        f"A_BRAKE_REST: brake rest gap {pad_rest_gap:.2f} mm < {BRAKE_REST_MM}.")

    # A_BRAKE_OVERLAP ---------------------------------------------------------
    # Rubber face is at R_RIM + pad_rest_gap at rest; after the engage pull it
    # must reach ≤ R_RIM − BRAKE_OVERLAP_MM (compressed into the band).
    r_pad = _contact_radius_after_pull(brake_pivot_x, brake_pivot_z,
                                       R_BAND + pad_rest_gap, pad_y_mid, pad_z_mid,
                                       brake_travel_deg)
    limit = R_BAND - BRAKE_OVERLAP_MM
    assert r_pad <= limit + _EPS, (
        f"A_BRAKE_OVERLAP: pad reaches r={r_pad:.2f} mm at full pull, "
        f"need ≤ {limit:.2f} (band {R_BAND:.2f} − {BRAKE_OVERLAP_MM}). "
        f"Lower BRAKE_PIVOT_Z or increase travel.")

    return {
        "pawl_full_pull_r": r_pawl,
        "pawl_clearance_mm": r_pawl - R_RATCHET_TIP,
        "pad_full_pull_r": r_pad,
        "pad_overlap_mm": R_BAND - r_pad,
    }
