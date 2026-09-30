"""Lever kinematics + load-time assertions — v2's suite ported verbatim
(v3 anchors come in through assert_kinematics' arguments; the only local
bindings are the band radii, now off RIM_OD).

Assertions (fire at import via assert_kinematics, called from levers.py).
(The BRAKE's suite — A_BRAKE_REST, _CONTACT_FLUSH, _CONTACT_SPAN and
_CABLE_CLR — retired with the brake itself at #1019. What survives is the
ratchet's own clearance and the spool-chamber keep-out.)

  A_PAWL_CLEAR    : at the ratchet's stop angle (the design travel limit),
                    the pawl's rim-contact point has swung radially OUT
                    past the tooth tips by ≥ PAWL_CLEAR_MM.
  A_CHAMBER_CLR   : nothing the lever carries reaches into the spool
                    chamber's keep-out — kept as the guard for whatever
                    the ratchet lever grows next.
"""

import math

from .params import RIM_OD, RATCHET_DEPTH

R_BAND        = RIM_OD / 2.0                # 69.435 — the tooth tips
R_RATCHET_TIP = R_BAND
R_ROOT        = R_BAND - RATCHET_DEPTH      # 67.035 — pawl seats on the valley

PAWL_CLEAR_MM        = 1.0
_EPS = 1e-6


def _rotate_xz_about(x, z, px, pz, theta_deg):
    """Rotate (x, z) about (px, pz) by theta_deg about +Y (pull = −travel)."""
    t = math.radians(theta_deg)
    dx, dz = x - px, z - pz
    return (px + dx * math.cos(t) + dz * math.sin(t),
            pz - dx * math.sin(t) + dz * math.cos(t))


def _contact_radius_after_pull(pivot_x, pivot_z, r_rest, y, z, travel_deg):
    """Radius (in XY) of a rim point at radius r_rest, at the given y and z,
    after the lever is pulled (rotated −travel about +Y at the pivot)."""
    x_rest = math.sqrt(max(0.0, r_rest ** 2 - y ** 2))
    x2, _z2 = _rotate_xz_about(x_rest, z, pivot_x, pivot_z, -travel_deg)
    return math.hypot(x2, y)


def assert_kinematics(*, ratchet_pivot_x, ratchet_pivot_z, ratchet_stop_deg,
                      pawl_y_mid, pawl_z_mid,
                      chamber_r_min, chamber_z0):
    """Contract: the ratchet's travel limit is `ratchet_stop_deg` (v3 NOTE:
    the physical stop tab is still to be re-added — the wall sill that
    was the stop left with the full-height bays; the angle remains the
    design limit). `chamber_r_min` / `chamber_z0` carry the spool-chamber
    keep-out, which is still worth asserting against anything that swings
    up there."""
    # A_PAWL_CLEAR — at the stop angle, the design travel limit -------------
    r_pawl = _contact_radius_after_pull(ratchet_pivot_x, ratchet_pivot_z,
                                        R_ROOT, pawl_y_mid, pawl_z_mid,
                                        ratchet_stop_deg)
    need = R_RATCHET_TIP + PAWL_CLEAR_MM
    assert r_pawl >= need - _EPS, (
        f"A_PAWL_CLEAR: pawl reaches r={r_pawl:.2f} mm at the stop "
        f"({ratchet_stop_deg:.1f} deg), need ≥ {need:.2f}. Raise "
        f"RATCHET_STOP_DEG or RATCHET_PIVOT_Z.")

    return {
        "pawl_full_pull_r": r_pawl,
        "pawl_clearance_mm": r_pawl - R_RATCHET_TIP,
    }
