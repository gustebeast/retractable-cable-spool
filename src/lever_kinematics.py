"""Lever kinematics — codifies the design assertions that govern when
the ratchet pawl clears the teeth and when the brake pad contacts the
spool, at both rest and full-pull poses.

A_PAWL_REST: at rest (θ_pull = 0), the ratchet pawl top sits flush with
    the tooth valley floor (z = RATCHET_DEPTH) — the pawl is fully
    engaged with the ratchet teeth. Automatic from construction (the
    pawl prism is built with its top at RATCHET_DEPTH).

A2 (pawl clearance at full pull): at full engagement, the highest point
    of the pawl top face anywhere in the toothed swept-volume sits
    ≥ A2_CLEARANCE_MM below the tooth tip. The check considers spool
    rotation by sweeping ALL angular positions of the teeth (so the
    comparison is against the cylinder r ∈ [r_in, r_out], not against
    any specific tooth orientation).

A3 (brake rest clearance): at rest (θ_pull = 0), the highest point of the
    brake pad rubber-top sits ≥ A3_CLEARANCE_MM below the brake track
    (z = BRAKE_TRACK_Z). No drag on the spool when the lever is released.

A4 (brake full-pull overlap): at full pull (θ_pull = TRAVEL_DEG), the
    rubber-top first-contact corner has risen ≥ A4_OVERLAP_MM above the
    brake track — i.e., the rubber is being compressed that much into
    the spool, ensuring the brake actually engages.

A5 (split-angle, symmetric contact tilt): the brake pad is parallel to
    the brake track at the MIDPOINT of the brake's contact range
    [θ_brake_contact, TRAVEL_DEG]. This makes the pad's tilt
    symmetric (equal magnitude, opposite sign) at first contact and at
    full pull, maximizing the rubber's contact area throughout the
    contact range. Verified by construction (pad pose solved by the
    iterative `pad_parallel_prerot_symmetric_contact` solver) but
    asserted here as a guard against future restructuring.

A1 (matched ROM — RELAXED): formerly a hard constraint that the pawl's
    tooth-tip clearance angle and the pad's contact angle coincide. Now
    relaxed: A2/A3/A4 individually enforce the rest and full-pull poses,
    but the precise theta at which each lever transitions is no longer
    constrained to match. θ_match is still computed for informational
    use (see THETA_MATCH_DEG in levers.py).

Coordinate convention
─────────────────────
Build / assembly rotation around the +Y axis at the lever pivot uses
right-hand-rule sign: positive θ around +Y maps (+X → -Z, +Z → +X). The
assembly applies θ_assembly = -θ_pull, where θ_pull ∈ [0°, TRAVEL_DEG]
is the user's "amount of pull" from rest. So:
    R_y(θ_assembly) ∘ R_y(prebuilt_offset)
where the brake pad's prebuilt_offset = +BRAKE_INNER_TRAVEL_DEG takes
the design pose to the rest pose at the printer.
"""

import math

from .dimensions import RATCHET_DEPTH, FLANGE_RIM_MID_R
from .housing import (
    RATCHET_PIVOT_X, BRAKE_PIVOT_X, LEVER_PIVOT_Z,
    RATCHET_OUTER_TRAVEL_DEG, BRAKE_INNER_TRAVEL_DEG,
)


BRAKE_TRACK_Z   = 0.0           # bottom face of the brake annulus
TOOTH_TIP_Z     = 0.0           # tip face of the ratchet teeth (lowest tooth z)
A2_CLEARANCE_MM = 1.0           # required pawl clearance at full engagement
A3_CLEARANCE_MM = 1.0           # required brake-pad rest clearance below track
A4_OVERLAP_MM   = 0.5           # required brake-pad overlap above track at full pull.
                                # 3.3 mm Buna-N 70A rubber compresses elastically
                                # to ~10-25% strain; 0.5 mm ≈ 15% strain — plenty
                                # of contact pressure for a hand-spool brake while
                                # staying well inside the elastic range and not
                                # acting as a hard end-stop near full pull.


def _rotate_xz_around_pivot(x, z, pivot_x, pivot_z, theta_deg):
    """Rotate (x, z) around (pivot_x, pivot_z) by theta degrees about +Y.
    Right-hand rule (+Y out of the XZ plane): +θ takes +X → -Z."""
    th = math.radians(theta_deg)
    dx = x - pivot_x
    dz = z - pivot_z
    new_dx = dx * math.cos(th) + dz * math.sin(th)
    new_dz = -dx * math.sin(th) + dz * math.cos(th)
    return pivot_x + new_dx, pivot_z + new_dz


# ── Pawl side ────────────────────────────────────────────────────────────────

def _pawl_top_worst_corner_x(pawl_y_near, pawl_r_out):
    """The pawl-top corner whose z stays HIGHEST under the assembly rotation
    -θ_pull around the ratchet pivot — i.e. the corner with the largest
    x in REST frame (= smallest |dx| when the pivot is outboard of the
    pawl). The annular pawl footprint's maximum x lies at the outer
    radius and smallest |y|, i.e. (sqrt(r_out² − y_near²), y_near)."""
    return math.sqrt(pawl_r_out ** 2 - pawl_y_near ** 2)


def pawl_top_max_z(theta_pull_deg, pawl_y_near, pawl_r_out):
    """Max z of any point on the pawl top face (z=RATCHET_DEPTH at rest)
    after the ratchet lever has been pulled by θ_pull. Assembly applies
    -θ_pull around the ratchet pivot."""
    x_worst = _pawl_top_worst_corner_x(pawl_y_near, pawl_r_out)
    _, z = _rotate_xz_around_pivot(
        x_worst, RATCHET_DEPTH,
        RATCHET_PIVOT_X, LEVER_PIVOT_Z,
        -theta_pull_deg,
    )
    return z


def theta_pawl_just_clears(pawl_y_near, pawl_r_out,
                           tip_z=TOOTH_TIP_Z, max_theta_deg=45.0):
    """Smallest θ_pull at which pawl_top_max_z drops to tip_z (z=0).
    Bisection over [0, max_theta_deg]. Returns None if the pawl never
    clears within the given range (geometry is broken)."""
    z0 = pawl_top_max_z(0.0, pawl_y_near, pawl_r_out)
    z_max = pawl_top_max_z(max_theta_deg, pawl_y_near, pawl_r_out)
    if z0 <= tip_z:
        return 0.0  # already clear at rest (shouldn't happen for a real ratchet)
    if z_max > tip_z:
        return None  # never clears within max_theta_deg
    lo, hi = 0.0, max_theta_deg
    for _ in range(80):
        mid = (lo + hi) / 2
        if pawl_top_max_z(mid, pawl_y_near, pawl_r_out) <= tip_z:
            hi = mid
        else:
            lo = mid
    return hi


# ── Pad side ─────────────────────────────────────────────────────────────────

def _pad_first_contact_corner_x_y(pad_y_near, pad_y_far):
    """The pad rubber-top corner that FIRST contacts the brake track as the
    lever engages. With the pad pre-rotated +BRAKE_INNER_TRAVEL_DEG to
    print pose, the inboard-far (large |y|, inner radius) corner is the
    highest at every θ_pull < TRAVEL_DEG, since the outer-radius corners
    descended more under the print-pose pre-tilt. Returns (x_rest_at_
    design_pose, y)."""
    y_with_larger_abs = pad_y_far if abs(pad_y_far) >= abs(pad_y_near) else pad_y_near
    r_in = FLANGE_RIM_MID_R
    x = math.sqrt(r_in ** 2 - y_with_larger_abs ** 2)
    return x, y_with_larger_abs


def pad_design_bot_z(pad_y_near, pad_y_far, rubber_t, prerot_deg,
                     rest_clearance=A3_CLEARANCE_MM):
    """Solve for the pad-solid spool-facing-face z (= rubber-bottom z, in
    DESIGN/parallel pose) such that the first-contact corner of the
    rubber-top sits EXACTLY `rest_clearance` mm below the brake track at
    θ_pull = 0 (rest pose).

    Pad construction: built at design pose (a flat slab at z =
    pad_design_bot_z, parallel to the brake track), then pre-rotated by
    +prerot_deg around the brake pivot to produce the print/rest pose.
    At θ_pull, the pad sits at angle (prerot_deg − θ_pull) above design
    pose; at θ_pull = prerot_deg the pad is flat/parallel.
    """
    x_c, _ = _pad_first_contact_corner_x_y(pad_y_near, pad_y_far)
    phi = math.radians(prerot_deg)  # pad rotation from design pose at rest
    dx_design = x_c - BRAKE_PIVOT_X
    # Constraint at θ_pull = 0 (rest):
    #   z_rest = LEVER_PIVOT_Z + (-dx*sin(phi) + dz*cos(phi))
    #          = BRAKE_TRACK_Z - rest_clearance
    target_z_rest = BRAKE_TRACK_Z - rest_clearance
    dz_design = (target_z_rest - LEVER_PIVOT_Z + dx_design * math.sin(phi)) / math.cos(phi)
    rubber_top_design_z = LEVER_PIVOT_Z + dz_design
    return rubber_top_design_z - rubber_t


def pad_rubber_top_z_at(theta_pull_deg, x_corner, rubber_top_design_z,
                        prerot_deg):
    """Rubber-top z at a given corner (x in design pose) at θ_pull, with
    pad pre-rotated by +prerot_deg from design to reach print/rest pose."""
    phi_deg = prerot_deg - theta_pull_deg
    _, z = _rotate_xz_around_pivot(
        x_corner, rubber_top_design_z,
        BRAKE_PIVOT_X, LEVER_PIVOT_Z,
        phi_deg,
    )
    return z


def pad_parallel_prerot_symmetric_contact(pad_y_near, pad_y_far, rubber_t,
                                          *, max_iter=20, tol=1e-6):
    """Iteratively solve for the prerot_deg that puts the pad parallel
    to the brake track at the MIDPOINT of the contact range [θ_bc,
    TRAVEL] — so the pad tilts by equal magnitudes (and opposite signs)
    at first contact and at full pull. Maximizes contact area inside the
    contact range vs. choosing the midpoint of [0, TRAVEL], which over-
    tilts the pad at full pull (since rest pose isn't in the contact
    range)."""
    travel = RATCHET_OUTER_TRAVEL_DEG
    prerot = travel / 2.0    # initial guess: midpoint of [0, TRAVEL]
    for _ in range(max_iter):
        pad_bot = pad_design_bot_z(pad_y_near, pad_y_far, rubber_t, prerot)
        theta_bc = theta_brake_first_contact(
            pad_y_near, pad_y_far, rubber_t, prerot, pad_bot,
        )
        if theta_bc is None:
            raise ValueError(
                "Brake pad never contacts the track within the search "
                "range. Geometry is broken — check BRAKE_PIVOT_X."
            )
        new_prerot = (theta_bc + travel) / 2.0
        if abs(new_prerot - prerot) < tol:
            return new_prerot
        prerot = new_prerot
    return prerot


def theta_brake_first_contact(pad_y_near, pad_y_far, rubber_t,
                              prerot_deg, pad_bot_design_z,
                              max_theta_deg=45.0):
    """Smallest θ_pull at which the brake pad's rubber-top first-contact
    corner reaches z=BRAKE_TRACK_Z (just touches the brake track).
    Bisection over [0, max_theta_deg]. Returns None if never contacts."""
    x_c, _ = _pad_first_contact_corner_x_y(pad_y_near, pad_y_far)
    rubber_top_design_z = pad_bot_design_z + rubber_t
    z0 = pad_rubber_top_z_at(0.0, x_c, rubber_top_design_z, prerot_deg)
    z_max = pad_rubber_top_z_at(max_theta_deg, x_c, rubber_top_design_z, prerot_deg)
    if z0 >= BRAKE_TRACK_Z:
        return 0.0  # already in contact at rest (shouldn't happen)
    if z_max < BRAKE_TRACK_Z:
        return None
    lo, hi = 0.0, max_theta_deg
    for _ in range(80):
        mid = (lo + hi) / 2
        if pad_rubber_top_z_at(mid, x_c, rubber_top_design_z, prerot_deg) >= BRAKE_TRACK_Z:
            hi = mid
        else:
            lo = mid
    return hi


# ── Module-load assertions ───────────────────────────────────────────────────

def assert_kinematics(*, pawl_y_near, pawl_y_far, pawl_r_out,
                      pad_y_near, pad_y_far, rubber_t,
                      pad_bot_design_z, prerot_deg):
    """Asserts A_PAWL_REST, A2, A3, A4. Called from levers.py with the
    geometric inputs that define the pawl and pad footprints."""

    # A_PAWL_REST: pawl top sits flush with the tooth valley floor at rest.
    # This is automatic (pawl is built with its top at z=RATCHET_DEPTH and no
    # rotation is applied at rest), but verify in case the geometry is ever
    # restructured to compute the rest-pose pawl-top dynamically.
    z_pawl_rest = pawl_top_max_z(0.0, pawl_y_near, pawl_r_out)
    assert abs(z_pawl_rest - RATCHET_DEPTH) < 1e-6, (
        f"A_PAWL_REST FAILED: pawl top at rest is z={z_pawl_rest:+.6f}; "
        f"expected {RATCHET_DEPTH:+.6f} (flush with the tooth valley floor)."
    )

    # A2: pawl clears teeth-swept-volume by ≥ A2_CLEARANCE_MM at full pull.
    z_pawl_full = pawl_top_max_z(RATCHET_OUTER_TRAVEL_DEG,
                                  pawl_y_near, pawl_r_out)
    assert z_pawl_full <= TOOTH_TIP_Z - A2_CLEARANCE_MM, (
        f"A2 FAILED: pawl top max-z at full engagement is "
        f"{z_pawl_full:+.3f} mm; need ≤ {TOOTH_TIP_Z - A2_CLEARANCE_MM:+.3f} "
        f"({A2_CLEARANCE_MM} mm clearance below tooth tip at z={TOOTH_TIP_Z}). "
        f"Move RATCHET_PIVOT_X outboard (currently {RATCHET_PIVOT_X})."
    )

    x_c, _ = _pad_first_contact_corner_x_y(pad_y_near, pad_y_far)
    rubber_top_design_z = pad_bot_design_z + rubber_t

    # A3: rubber-top first-contact corner sits ≥ A3_CLEARANCE_MM below the
    # brake track at rest (no drag on the spool when the lever is released).
    z_rubber_rest = pad_rubber_top_z_at(0.0, x_c, rubber_top_design_z,
                                        prerot_deg=prerot_deg)
    assert z_rubber_rest <= BRAKE_TRACK_Z - A3_CLEARANCE_MM, (
        f"A3 FAILED: rubber-top max-z at rest is {z_rubber_rest:+.3f} mm; "
        f"need ≤ {BRAKE_TRACK_Z - A3_CLEARANCE_MM:+.3f} "
        f"({A3_CLEARANCE_MM} mm clearance below the brake track at "
        f"z={BRAKE_TRACK_Z}). Move BRAKE_PIVOT_X inboard (currently "
        f"{BRAKE_PIVOT_X})."
    )

    # A4: rubber-top first-contact corner sits ≥ A4_OVERLAP_MM above the
    # brake track at full pull (brake is actively engaged, rubber compressed
    # into the spool to provide braking force).
    z_rubber_full = pad_rubber_top_z_at(RATCHET_OUTER_TRAVEL_DEG, x_c,
                                        rubber_top_design_z,
                                        prerot_deg=prerot_deg)
    assert z_rubber_full >= BRAKE_TRACK_Z + A4_OVERLAP_MM, (
        f"A4 FAILED: rubber-top max-z at full pull is {z_rubber_full:+.3f} "
        f"mm; need ≥ {BRAKE_TRACK_Z + A4_OVERLAP_MM:+.3f} "
        f"({A4_OVERLAP_MM} mm overlap into the brake track at "
        f"z={BRAKE_TRACK_Z}). Move BRAKE_PIVOT_X inboard (currently "
        f"{BRAKE_PIVOT_X}) to lengthen the pad arm."
    )

    # A5: pad is parallel to track at the midpoint of the brake contact
    # range — symmetric tilt at first contact and at full pull. The
    # iterative pad_parallel_prerot_symmetric_contact solver should
    # already enforce this, but a direct check makes the relationship
    # visible at the call site and catches drift if anything changes.
    theta_bc = theta_brake_first_contact(
        pad_y_near, pad_y_far, rubber_t,
        prerot_deg=prerot_deg,
        pad_bot_design_z=pad_bot_design_z,
    )
    assert theta_bc is not None, (
        "A5 prereq FAILED: brake pad never contacts the track within the "
        "search range — A4 should have caught this first."
    )
    contact_midpoint_deg = (theta_bc + RATCHET_OUTER_TRAVEL_DEG) / 2.0
    assert abs(prerot_deg - contact_midpoint_deg) < 1e-3, (
        f"A5 FAILED: pad parallel pose at θ_pull={prerot_deg:.4f}° but "
        f"the contact-range midpoint is {contact_midpoint_deg:.4f}° "
        f"(θ_brake_contact={theta_bc:.4f}°, TRAVEL="
        f"{RATCHET_OUTER_TRAVEL_DEG}°). Tilt at first contact and at "
        f"full pull are asymmetric, reducing the pad's contact area at "
        f"one of the endpoints."
    )
