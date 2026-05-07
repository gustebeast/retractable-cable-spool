"""Standalone lever-side guide-bearing axle (plain Ø7.9 cylinder).

Press-fits from the OUTBOARD (spine) side of the lever-side guide bearing
pocket — no anti-rotation tab needed because the press hole is hidden
inside the housing. The axle's inboard end sits inside the bearing
pocket; the outboard end sits ~0.5 mm shy of the spine inner face.
"""

import cadquery as cq

from .dimensions import AXLE_PRINT_D, BEARING_W
from .housing_lever_guide import (
    LEVER_GUIDE_AXLE_PRESS_LEN,
    LEVER_GUIDE_AXLE_X_OUTER,
    LEVER_GUIDE_AXLE_Z,
    LEVER_GUIDE_BEARING_Y_OFFSET,
)


def _build_lever_guide_axle():
    """Plain Ø7.9 cylinder; length = BEARING_W (bearing region) +
    LEVER_GUIDE_AXLE_PRESS_LEN (press grip into housing). Inboard end
    sits at LEVER_GUIDE_AXLE_X_OUTER - BEARING_W (bearing's inboard
    face)."""
    total_len = BEARING_W + LEVER_GUIDE_AXLE_PRESS_LEN
    bearing_x_inner = LEVER_GUIDE_AXLE_X_OUTER - BEARING_W
    return cq.Workplane("XY").add(cq.Solid.makeCylinder(
        AXLE_PRINT_D / 2, total_len,
        pnt=cq.Vector(bearing_x_inner,
                      LEVER_GUIDE_BEARING_Y_OFFSET, LEVER_GUIDE_AXLE_Z),
        dir=cq.Vector(1, 0, 0),
    ))


lever_guide_axle_part = _build_lever_guide_axle()
