"""Assembly-only visualization parts — dummy 608 bearings, dummy torsion
springs, and the brake-pad rubber slab.

Not printed; included in the assembly STEP only for visual sanity-checking.
The springs are rendered as a simple coil annulus + two radial legs reaching
each lever's stop pins (exact wire path isn't load-bearing for the model).
"""

import math
import cadquery as cq

from .dimensions import (
    AXLE_D,
    BEARING_LIP_H, BEARING_OD, BEARING_W,
)
from .helpers import cyl
from .housing import (
    HOUSING_W,
    RATCHET_PIVOT_X, RATCHET_PIVOT_Z,
    BRAKE_PIVOT_X, BRAKE_PIVOT_Z,
    RATCHET_LEVER_PIN_ALPHA, RATCHET_SPRING_PIN_ALPHA,
    BRAKE_LEVER_PIN_ALPHA, BRAKE_SPRING_PIN_ALPHA,
    SPRING_BODY_LEN, SPRING_COIL_MAJOR_R, SPRING_WIRE_R,
    STOP_PIN_R,
)
from .spool import pancake_bearing_z0


# ── Torsion spring (viz only) ────────────────────────────────────────────────
def _torsion_spring(pivot_x, pivot_z, y_a, y_b, pin_alphas):
    """Coil annulus (axis Y) in the lever gap + one radial leg reaching from
    the coil edge to each stop pin (at its α). Fused into one solid so STEP
    viewers can hide it as a single part."""
    parts = []
    span = abs(y_b - y_a)
    y_min = min(y_a, y_b)
    half = SPRING_WIRE_R + 0.15
    outer = cq.Solid.makeCylinder(SPRING_COIL_MAJOR_R + half, span,
                                  pnt=cq.Vector(pivot_x, y_min, pivot_z),
                                  dir=cq.Vector(0, 1, 0))
    inner = cq.Solid.makeCylinder(SPRING_COIL_MAJOR_R - half, span + 0.4,
                                  pnt=cq.Vector(pivot_x, y_min - 0.2, pivot_z),
                                  dir=cq.Vector(0, 1, 0))
    parts.append(outer.cut(inner))

    y_mid = (y_a + y_b) / 2
    for alpha in pin_alphas:
        a = math.radians(alpha)
        cx = pivot_x + SPRING_COIL_MAJOR_R * math.cos(a)
        cz = pivot_z - SPRING_COIL_MAJOR_R * math.sin(a)
        px = pivot_x + STOP_PIN_R * math.cos(a)
        pz = pivot_z - STOP_PIN_R * math.sin(a)
        dx, dz = px - cx, pz - cz
        L = math.hypot(dx, dz)
        parts.append(cq.Solid.makeSphere(SPRING_WIRE_R + 0.1,
                                         pnt=cq.Vector(cx, y_mid, cz)))
        parts.append(cq.Solid.makeCylinder(
            SPRING_WIRE_R, L + 0.5, pnt=cq.Vector(cx, y_mid, cz),
            dir=cq.Vector(dx / L, 0, dz / L)))

    fused = parts[0]
    for p in parts[1:]:
        fused = fused.fuse(p)
    return cq.Workplane("XY").add(fused)


# Coil sits centered in the 4.5 mm lever gap, between the block face
# (±HOUSING_W/2) and the lever inner face. Legs reach the lever pin and the
# spring pin of each lever.
_GAP_PAD = (4.5 - SPRING_BODY_LEN) / 2          # ~1 mm each side of the coil
ratchet_spring = _torsion_spring(
    RATCHET_PIVOT_X, RATCHET_PIVOT_Z,
    y_a=HOUSING_W / 2 + _GAP_PAD,
    y_b=HOUSING_W / 2 + _GAP_PAD + SPRING_BODY_LEN,
    pin_alphas=(RATCHET_SPRING_PIN_ALPHA, RATCHET_LEVER_PIN_ALPHA),
)
brake_spring = _torsion_spring(
    BRAKE_PIVOT_X, BRAKE_PIVOT_Z,
    y_a=-HOUSING_W / 2 - _GAP_PAD,
    y_b=-HOUSING_W / 2 - _GAP_PAD - SPRING_BODY_LEN,
    pin_alphas=(BRAKE_SPRING_PIN_ALPHA, BRAKE_LEVER_PIN_ALPHA),
)


# ── Dummy 608 bearings (visualization-only) ───────────────────────────
def _bearing_dummy(z_base):
    return (cyl(BEARING_OD, BEARING_W, z=z_base)
            .cut(cyl(AXLE_D, BEARING_W, z=z_base)))

# Bottom bearing: in the hub's fused bottom pocket, sits on the lip.
bearing_bottom = _bearing_dummy(BEARING_LIP_H)
# Top bearing: in the removable bearing_cap_top's pocket.
bearing_top    = _bearing_dummy(pancake_bearing_z0)


# ── Brake pad rubber (viz only) ───────────────────────────────────────
# Thin slab bonded to the printed brake pad's rim-facing face, occupying the
# gap between that face and the band. Built at the lever's rest pose and
# pre-rotated by the same warp pre-comp as the printed lever.
def _brake_pad_rubber():
    from .housing import LEVER_REST_PRECOMP_DEG
    from .levers import (BRAKE_Y0, BRAKE_Y1, RUBBER_FACE_R, _PAD_R_IN,
                         PAD_Z_LO, PAD_Z_HI, _arc_strip)
    # Arc strip between the rubber's rim-facing arc and the printed pad's
    # inner arc, conforming to the band (matches the pad's curvature).
    slab = _arc_strip(RUBBER_FACE_R, _PAD_R_IN, BRAKE_Y0, BRAKE_Y1,
                      PAD_Z_LO, PAD_Z_HI)
    return slab.rotate((BRAKE_PIVOT_X, 0, BRAKE_PIVOT_Z),
                       (BRAKE_PIVOT_X, 1, BRAKE_PIVOT_Z),
                       LEVER_REST_PRECOMP_DEG)


brake_pad_rubber = _brake_pad_rubber()
