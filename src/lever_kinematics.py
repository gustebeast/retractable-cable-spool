"""Lever kinematics + load-time assertions — v2's suite ported verbatim
(v3 anchors come in through assert_kinematics' arguments; the only local
bindings are the band radii, now off RIM_OD).

Assertions (fire at import via assert_kinematics, called from levers.py).
The brake is validated at its CONTACT POSE (user's spec) — the pull past
contact is the TPU-compression regime and carries no geometric assertions
(full-pull compression is REPORTED, not asserted):

  A_PAWL_CLEAR    : at the ratchet's stop angle (the design travel limit),
                    the pawl's rim-contact point has swung radially OUT
                    past the tooth tips by ≥ PAWL_CLEAR_MM.
  A_BRAKE_REST    : at rest, no brake-pad band-side corner is inside the band.
  A_BRAKE_CONTACT_FLUSH : at the contact angle, every band-side pad corner
                    sits ON the band — gap in [0, FLUSH_TOL] (the positive
                    slack is the flat-face-vs-cylinder sagitta; parallelism
                    itself holds by construction, pre-tilt = contact angle).
  A_BRAKE_CONTACT_SPAN  : at the contact angle, all band-side corners land
                    inside the band's z-extent.
  A_BRAKE_CABLE_CLR : swept over the whole pull, any band corner above
                    `chamber_z0` (the spool chamber) keeps its radius ≥
                    `chamber_r_min` (outermost wrap) — the resting pad
                    must never clip a full coil.
"""

import math

from .params import RIM_OD, RATCHET_DEPTH

R_BAND        = RIM_OD / 2.0                # 69.435 — brake band = tooth tips
R_RATCHET_TIP = R_BAND
R_ROOT        = R_BAND - RATCHET_DEPTH      # 67.035 — pawl seats on the valley

PAWL_CLEAR_MM        = 1.0
BRAKE_REST_MM        = 1.0
BRAKE_FLUSH_TOL_MM   = 0.40      # ≥ flat-face sagitta across the pad width
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
                      brake_pivot_x, brake_pivot_z, brake_travel_deg,
                      brake_contact_deg, brake_band_z0, brake_band_z1,
                      chamber_r_min, chamber_z0,
                      pad_band_corners):
    """Contract: the ratchet's travel limit is `ratchet_stop_deg` (v3 NOTE:
    the physical stop tab is still to be re-added — the wall sill that
    was the stop left with the full-height bays; the angle remains the
    design limit), and the BRAKE validates at `brake_contact_deg` (flush +
    in-band) plus the swept spool-chamber keep-out — see header."""
    # A_PAWL_CLEAR — at the stop angle, the design travel limit -------------
    r_pawl = _contact_radius_after_pull(ratchet_pivot_x, ratchet_pivot_z,
                                        R_ROOT, pawl_y_mid, pawl_z_mid,
                                        ratchet_stop_deg)
    need = R_RATCHET_TIP + PAWL_CLEAR_MM
    assert r_pawl >= need - _EPS, (
        f"A_PAWL_CLEAR: pawl reaches r={r_pawl:.2f} mm at the stop "
        f"({ratchet_stop_deg:.1f} deg), need ≥ {need:.2f}. Raise "
        f"RATCHET_STOP_DEG or RATCHET_PIVOT_Z.")

    # A_BRAKE_REST ------------------------------------------------------------
    rest_gaps = [math.hypot(x, y) - R_BAND for x, y, _z in pad_band_corners]
    min_rest_gap = min(rest_gaps)
    assert min_rest_gap > -_EPS, (
        f"A_BRAKE_REST: a pad band-side corner is INSIDE the band at rest "
        f"(min gap {min_rest_gap:.3f} mm) — the contact-frame back-rotation "
        f"leaves too little rest standoff; raise BRAKE_CONTACT_DEG margins.")

    # A_BRAKE_CONTACT_FLUSH + A_BRAKE_CONTACT_SPAN ---------------------------
    contact_gaps, contact_zs = [], []
    for x, y, z in pad_band_corners:
        nx, nz = _rotate_xz_about(x, z, brake_pivot_x, brake_pivot_z,
                                  -brake_contact_deg)
        contact_gaps.append(math.hypot(nx, y) - R_BAND)
        contact_zs.append(nz)
    assert (min(contact_gaps) > -0.05 - _EPS
            and max(contact_gaps) < BRAKE_FLUSH_TOL_MM + _EPS), (
        f"A_BRAKE_CONTACT_FLUSH: at contact ({brake_contact_deg:.2f} deg) "
        f"corner gaps span [{min(contact_gaps):.3f}, {max(contact_gaps):.3f}]"
        f" mm, need [0, {BRAKE_FLUSH_TOL_MM}]. The face is not flush.")
    assert (min(contact_zs) >= brake_band_z0 - _EPS
            and max(contact_zs) <= brake_band_z1 + _EPS), (
        f"A_BRAKE_CONTACT_SPAN: at contact the pad spans z "
        f"[{min(contact_zs):.2f}, {max(contact_zs):.2f}], outside the band "
        f"[{brake_band_z0}, {brake_band_z1}].")

    # A_BRAKE_CABLE_CLR — swept spool-chamber keep-out ------------------------
    cable_clr = None
    for x, y, z in pad_band_corners:
        for i in range(0, int(brake_travel_deg * 4) + 1):
            t = min(i / 4.0, brake_travel_deg)
            nx, nz = _rotate_xz_about(x, z, brake_pivot_x, brake_pivot_z, -t)
            if nz > chamber_z0 + _EPS:
                c = math.hypot(nx, y) - chamber_r_min
                if cable_clr is None or c < cable_clr:
                    cable_clr = c
    if cable_clr is not None:
        assert cable_clr >= -_EPS, (
            f"A_BRAKE_CABLE_CLR: a swept pad corner dips {-cable_clr:.2f} mm "
            f"inside the spool-chamber keep-out (r < {chamber_r_min:.2f} "
            f"above z {chamber_z0}). Lower the pad top or add top margin.")

    # full-pull compression — REPORTED only (contact-pose contract)
    max_overlap = max(
        R_BAND - math.hypot(
            _rotate_xz_about(x, z, brake_pivot_x, brake_pivot_z,
                             -brake_travel_deg)[0], y)
        for x, y, z in pad_band_corners)

    return {
        "pawl_full_pull_r": r_pawl,
        "pawl_clearance_mm": r_pawl - R_RATCHET_TIP,
        "pad_min_rest_gap_mm": min_rest_gap,
        "contact_flush_gap_mm": max(contact_gaps),
        "pad_max_overlap_mm": max_overlap,
        "cable_clearance_mm": cable_clr,
    }
