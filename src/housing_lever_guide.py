"""Lever-side guide wheel — printed Ø9 wheel + sandwich slabs merged
into the housing. Replaces the 608 guide bearing on the lever side.

Mirrors housing_guide.py for the lever side.
"""

import cadquery as cq

from .dimensions import DRUM_OD, FLANGE_RIM_MID_R
from . import wheel as _wheel


# Centered on the brake/wheel band of the lever-side rim (smooth outer
# half of the rim annulus, from FLANGE_RIM_MID_R out to DRUM_OD/2).
WHEEL_X_CENTER       = (FLANGE_RIM_MID_R + DRUM_OD / 2) / 2
WHEEL_Y_CENTER       = 0.0
WHEEL_Z_CENTER       = -_wheel.WHEEL_PAINTED_OD / 2   # -5


lever_guide_wheel = _wheel.wheel_solid(WHEEL_X_CENTER, WHEEL_Y_CENTER, WHEEL_Z_CENTER)


_pocket = _wheel.wheel_pocket_cut(
    WHEEL_X_CENTER, WHEEL_Y_CENTER, WHEEL_Z_CENTER,
    housing_z_sign=-1,
)
_fingers = _wheel.slab_fingers(
    WHEEL_X_CENTER, WHEEL_Y_CENTER, WHEEL_Z_CENTER,
    housing_z_sign=-1,
)
_hubs = _wheel.slab_hubs(WHEEL_X_CENTER, WHEEL_Y_CENTER, WHEEL_Z_CENTER)
_m2_hole = _wheel.m2_hole_cut(
    WHEEL_X_CENTER, WHEEL_Y_CENTER, WHEEL_Z_CENTER,
    head_side="inboard",
)


def apply_to_housing(housing: cq.Workplane) -> cq.Workplane:
    return (housing.cut(_pocket)
                   .union(_fingers)
                   .union(_hubs)
                   .cut(_m2_hole))
