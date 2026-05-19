"""Pancake-side guide wheel — printed Ø9 wheel + sandwich slabs merged
into the housing. Replaces the 608 guide bearing.

The wheel rolls on the spool's pancake-side flange's TOP face (painted
to ~Ø10). The two sandwich slabs are now part of the housing print:
the housing has a wheel pocket cut into the pancake plate, with the
plate material at the wheel pocket's ±X edges acting as the slabs. A
two-step M2 hole runs through both slab regions and the wheel pocket
along X — Ø MOUNT_HEAD_HOLE_D (4.1) head pocket on the +X (outboard)
side, Ø GUIDE_AXLE_SHAFT_D (2.2) tight-fit shaft hole the rest of the
way. No heat-set insert — the M2 threads engage the printed PA6-GF Ø2.2
hole directly on the far side of the wheel.

Printed parts exposed by this module: ``guide_wheel`` only. The slabs
are no longer separate.
"""

import cadquery as cq

from .dimensions import SPOOL_H, DRUM_OD, DRUM_ID, FLANGE_INNER_ID, STRUCT_WALL
from . import wheel as _wheel


# ── Wheel placement ─────────────────────────────────────────────────────────
# Centered on the pancake flange's flat top rim. Rim outer = DRUM_OD/2
# (flush with drum); rim inner is set by the 45° taper rising from
# r=DRUM_ID/2, sized so the rim's radial width matches the lever-side
# inner toothed rim (DRUM_ID−FLANGE_INNER_ID)/2.
_PANCAKE_RIM_W       = (DRUM_ID - FLANGE_INNER_ID) / 2
_PANCAKE_RIM_INNER_R = DRUM_OD / 2 - _PANCAKE_RIM_W
WHEEL_X_CENTER       = (_PANCAKE_RIM_INNER_R + DRUM_OD / 2) / 2
WHEEL_Y_CENTER       = 0.0
WHEEL_Z_CENTER       = SPOOL_H + _wheel.WHEEL_PAINTED_OD / 2   # 56


# ── Printed part ────────────────────────────────────────────────────────────
guide_wheel = _wheel.wheel_solid(WHEEL_X_CENTER, WHEEL_Y_CENTER, WHEEL_Z_CENTER)


# ── Housing cuts ────────────────────────────────────────────────────────────
_pocket = _wheel.wheel_pocket_cut(
    WHEEL_X_CENTER, WHEEL_Y_CENTER, WHEEL_Z_CENTER,
    housing_z_sign=+1,
    # Extend the pocket through the mount_bracket's +Y chamfer wedge
    # (a STRUCT_WALL-thick strip on the housing's top outer face) so the
    # bracket's small 45° chamfer is removed where it crosses the wheel
    # pocket. The wheel pocket then opens cleanly to the housing top.
    extra_z=STRUCT_WALL,
)
# Slab fingers skipped — they were originally needed when the housing
# was a thin 2 mm shell, but the housing is now 10 mm thick, so the
# fingers are entirely buried in plate material. They're geometric
# no-ops. (Earlier attempts to drop them tripped OCCT's "Courbes non
# jointives" on downstream cuts; that's been worked around with
# clean=False on the axle_access_channel + mount_bracket cuts and a
# heal() fallback in helpers.py.)
_hubs = _wheel.slab_hubs(WHEEL_X_CENTER, WHEEL_Y_CENTER, WHEEL_Z_CENTER)
_m2_hole = _wheel.m2_hole_cut(
    WHEEL_X_CENTER, WHEEL_Y_CENTER, WHEEL_Z_CENTER,
)


def apply_to_housing(housing: cq.Workplane) -> cq.Workplane:
    # Order: pocket cut, then union hubs (so they survive the pocket
    # carve), then drill the M2 stepped hole through them. The access
    # channel through the front of the housing is NOT cut here — it
    # must be cut last by _build_housing(), after every other union
    # (otherwise later bosses re-fill it). See axle_access_channel().
    return (housing.cut(_pocket)
                   .union(_hubs)
                   .cut(_m2_hole))


def axle_access_channel() -> cq.Workplane:
    """Ø MOUNT_HEAD_HOLE_D bore from the outboard slab's +X face to the
    housing's +X face — for installing the M2 axle screw from outside.
    Cut last in _build_housing()."""
    from .housing import SPINE_X_OUTER
    _, outboard_inner_x = _wheel.slab_inner_faces(WHEEL_X_CENTER)
    outboard_outer_x    = outboard_inner_x + _wheel.MOUNT_T
    return _wheel.axle_access_channel(WHEEL_Y_CENTER, WHEEL_Z_CENTER,
                                       outboard_outer_x, SPINE_X_OUTER)
