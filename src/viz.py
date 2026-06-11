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
    SPRING_BODY_LEN, SPRING_COIL_MAJOR_R, SPRING_WIRE_R, SPRING_SPACER_T,
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


# Coil sits centered in the LEVER_RIM_H-wide gap between the block face
# (±HOUSING_W/2) and the lever inner face, riding on the SPRING_SPACER_T
# bosses on either side. Legs reach the lever pin and the spring pin of
# each lever.
_GAP_PAD = SPRING_SPACER_T                       # housing-side spacer thickness
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
# Simple slab — a parallelogram in XZ, extruded across the lever's Y range.
# Matches the brake lever's tilted pad-mount surface so the pad sits flush
# against it. At the midpoint of brake travel the slab rotates to vertical,
# giving ideal flat contact with the band there. Built at the lever's rest
# pose and pre-rotated by the same warp pre-comp as the printed lever.
def _brake_pad_rubber():
    import math
    import cadquery as cq
    from .housing import LEVER_REST_PRECOMP_DEG
    from .levers import (BRAKE_Y0, BRAKE_Y1, BRAKE_ARM_X_LO, BRAKE_RUBBER_T,
                         RUBBER_PAD_Z_LO, PAD_Z_HI, PAD_Y_MID, R_RIM,
                         BRAKE_PAD_MOUNT_TILT_DEG)
    # X- and Y-tilts (mirroring the brake-lever horizontal arm).
    half_h    = (PAD_Z_HI - RUBBER_PAD_Z_LO) / 2
    tilt_dx_x = half_h * math.tan(math.radians(BRAKE_PAD_MOUNT_TILT_DEG))
    x_band    = math.sqrt(R_RIM ** 2 - PAD_Y_MID ** 2)
    dx_dy_at_rest = abs(PAD_Y_MID) / (
        math.cos(math.radians(BRAKE_PAD_MOUNT_TILT_DEG)) * x_band)
    y_len     = BRAKE_Y1 - BRAKE_Y0
    tilt_dx_y = (y_len / 2) * dx_dy_at_rest

    # XZ parallelograms at y=BRAKE_Y0 and y=BRAKE_Y1, lofted.
    # Lever-side face follows the lever's tilted mount; band-side is offset
    # BRAKE_RUBBER_T in -X.
    def _poly(sign_y):
        # sign_y: -1 for BRAKE_Y0, +1 for BRAKE_Y1
        offy = sign_y * tilt_dx_y
        x_lev_bot = BRAKE_ARM_X_LO - tilt_dx_x + offy
        x_lev_top = BRAKE_ARM_X_LO + tilt_dx_x + offy
        x_rim_bot = x_lev_bot - BRAKE_RUBBER_T
        x_rim_top = x_lev_top - BRAKE_RUBBER_T
        return [
            (x_lev_bot, RUBBER_PAD_Z_LO),
            (x_rim_bot, RUBBER_PAD_Z_LO),
            (x_rim_top, PAD_Z_HI),
            (x_lev_top, PAD_Z_HI),
        ]

    slab = (cq.Workplane("XZ").workplane(offset=-BRAKE_Y0)
            .polyline(_poly(-1)).close()
            .workplane(offset=BRAKE_Y0 - BRAKE_Y1)
            .polyline(_poly(+1)).close()
            .loft(ruled=True, combine=True))
    return slab.rotate((BRAKE_PIVOT_X, 0, BRAKE_PIVOT_Z),
                       (BRAKE_PIVOT_X, 1, BRAKE_PIVOT_Z),
                       LEVER_REST_PRECOMP_DEG)


brake_pad_rubber = _brake_pad_rubber()
