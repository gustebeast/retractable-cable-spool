"""Standalone guide-bearing axle (Ø7.7 cylinder + half-disc anti-rotation tab)."""

import cadquery as cq

from .dimensions import AXLE_PRINT_D
from .housing_guide import (
    GUIDE_AXLE_TOTAL_LEN, GUIDE_KEY_TAB_LEN, GUIDE_KEY_TAB_R, GUIDE_PRESS_X_MIN, GUIDE_KEY_TAB_X_MIN, GUIDE_AXLE_Z, GUIDE_BEARING_Y_OFFSET,
)


def _build_guide_axle():
    # Ø7.9 cylinder spanning the press-fit + bearing region.
    axle = cq.Workplane("XY").add(cq.Solid.makeCylinder(
        AXLE_PRINT_D / 2, GUIDE_AXLE_TOTAL_LEN,
        pnt=cq.Vector(GUIDE_PRESS_X_MIN, GUIDE_BEARING_Y_OFFSET, GUIDE_AXLE_Z),
        dir=cq.Vector(1, 0, 0),
    ))
    # Half-cone tab (printability): big end at the housing side tapers
    # down to the axle radius at the bearing side. Acts as a sliding
    # bearing stop — bearing inner edge rides the cone until cone radius
    # matches the axle, identical to the cone stops on the main spool axle.
    tab_full_cyl = cq.Workplane("XY").add(cq.Solid.makeCone(
        GUIDE_KEY_TAB_R, AXLE_PRINT_D / 2, GUIDE_KEY_TAB_LEN,
        pnt=cq.Vector(GUIDE_KEY_TAB_X_MIN, GUIDE_BEARING_Y_OFFSET, GUIDE_AXLE_Z),
        dir=cq.Vector(1, 0, 0),
    ))
    tab_clip_box = (
        cq.Workplane("XY")
        .box(GUIDE_KEY_TAB_LEN + 1.0, 30.0, 30.0, centered=False)
        .translate((GUIDE_KEY_TAB_X_MIN - 0.5,
                    GUIDE_BEARING_Y_OFFSET - 15.0,
                    GUIDE_AXLE_Z))
    )
    guide_tab = tab_full_cyl.intersect(tab_clip_box)
    return axle.union(guide_tab)


guide_axle_part = _build_guide_axle()
