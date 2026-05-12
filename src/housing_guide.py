"""Pancake-side guide wheel — printed Ø9 wheel + sandwich slabs merged
into the housing. Replaces the 608 guide bearing.

The wheel rolls on the spool's pancake-side flange's TOP face (painted
to ~Ø10). The two sandwich slabs are now part of the housing print:
the housing has a wheel pocket cut into the pancake plate, with the
plate material at the wheel pocket's ±X edges acting as the slabs. A
stepped M2 hole runs through both slab regions and the wheel pocket
along X — Ø4 head pocket on the inboard side, Ø3.3 insert on the
outboard side, Ø2.3 shaft clearance everywhere in between.

Printed parts exposed by this module: ``guide_wheel`` only. The slabs
are no longer separate.
"""

import cadquery as cq

from .dimensions import SPOOL_H, DRUM_OD, DRUM_ID, FLANGE_INNER_ID
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
)
_fingers = _wheel.slab_fingers(
    WHEEL_X_CENTER, WHEEL_Y_CENTER, WHEEL_Z_CENTER,
    housing_z_sign=+1,
)
_hubs = _wheel.slab_hubs(WHEEL_X_CENTER, WHEEL_Y_CENTER, WHEEL_Z_CENTER)
_m2_hole = _wheel.m2_hole_cut(
    WHEEL_X_CENTER, WHEEL_Y_CENTER, WHEEL_Z_CENTER,
    head_side="inboard",
)


def apply_to_housing(housing: cq.Workplane) -> cq.Workplane:
    # Order: pocket cut, then union slab fingers + hubs (so they survive
    # the pocket carve), then drill the M2 stepped hole through everything.
    return (housing.cut(_pocket)
                   .union(_fingers)
                   .union(_hubs)
                   .cut(_m2_hole))
