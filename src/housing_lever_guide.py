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


# extra_z=-0.5 shrinks the pocket's deep (housing-end) extent by 0.5 mm.
# The shared pocket depth is sized for the pancake side's slab housing-end,
# but the lever side skips the slab fingers, so the default pocket reaches
# z=-11.5 — only 0.5 mm above the plate's outer face (z=-12). Pulling it
# back to z=-11.0 leaves 1 mm of housing wall while still clearing the
# wheel (whose bottom sits at z=-10).
_pocket = _wheel.wheel_pocket_cut(
    WHEEL_X_CENTER, WHEEL_Y_CENTER, WHEEL_Z_CENTER,
    housing_z_sign=-1,
    extra_z=-0.5,
)
# Slab FINGERS skipped (cosmetic clean-up — see housing_guide.py).
# HUBS kept — they're the Ø4 friction-reducing axial spacers that
# contact the wheel faces.
_hubs = _wheel.slab_hubs(WHEEL_X_CENTER, WHEEL_Y_CENTER, WHEEL_Z_CENTER)
_m2_hole = _wheel.m2_hole_cut(
    WHEEL_X_CENTER, WHEEL_Y_CENTER, WHEEL_Z_CENTER,
)


def apply_to_housing(housing: cq.Workplane) -> cq.Workplane:
    # See housing_guide.py for why the access channel is NOT cut here.
    return (housing.cut(_pocket)
                   .union(_hubs)
                   .cut(_m2_hole))


def axle_access_channel() -> cq.Workplane:
    """Ø MOUNT_HEAD_HOLE_D bore from outboard slab to housing +X face."""
    from .housing import SPINE_X_OUTER
    _, outboard_inner_x = _wheel.slab_inner_faces(WHEEL_X_CENTER)
    outboard_outer_x    = outboard_inner_x + _wheel.MOUNT_T
    return _wheel.axle_access_channel(WHEEL_Y_CENTER, WHEEL_Z_CENTER,
                                       outboard_outer_x, SPINE_X_OUTER)
