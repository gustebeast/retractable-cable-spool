"""Assembly-only visualization parts — dummy 608 bearings + dummy torsion springs.

Not printed; included in the assembly STEP only for visual sanity-checking.
"""

import math
import cadquery as cq

from .dimensions import (
    AXLE_D,
    BEARING_LIP_H, BEARING_OD, BEARING_W,
    SPOOL_H,
)
from .helpers import cyl
from .housing import (
    BRAKE_PIN_HOLE_BASE_CLR, BRAKE_PIVOT_X, HOUSING_W, LEVER_PIVOT_Z, LEVER_RIM_H, LEVER_STOP_PIN_HOLE_Y_FROM_GAP_END, RATCHET_PIVOT_X, SPRING_WIRE_R, STOP_HOUSING_PIN_ALPHA_BRAKE_DEG, STOP_HOUSING_PIN_ALPHA_RATCHET_DEG, STOP_LEVER_PIN_ALPHA_BRAKE_DEG, STOP_LEVER_PIN_ALPHA_RATCHET_DEG, STOP_PIN_D, STOP_PIN_H, STOP_PIN_HOLE_D, STOP_PIN_HOLE_OFFSET, STOP_PIN_R,
)
from .housing_guide import (
    GUIDE_BEARING_Y_OFFSET, GUIDE_AXLE_Z, GUIDE_KEY_TAB_X_MAX,
)
from .housing_lever_guide import (
    LEVER_GUIDE_AXLE_X_OUTER, LEVER_GUIDE_AXLE_Z, LEVER_GUIDE_BEARING_Y_OFFSET,
)
from .levers import LEVER_INNER_Y
from .caps import pancake_bearing_z0


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║ ASSEMBLY-ONLY VISUALIZATION PARTS                                       ║
# ║ The parts below are NOT printed — they exist only so the assembly.step  ║
# ║ shows the purchased components (608 bearings, torsion springs) in their ║
# ║ assembled positions for visual sanity-checking and clearance review.    ║
# ╚══════════════════════════════════════════════════════════════════════════╝

# ────────────────────────────────────────────────────────────────────────────
# TORSION SPRING (dummy / visualization only — not printed)
# Two springs, one per pivot. Coil approximated as a stack of single-wire
# tori spanning SPRING_BODY_LEN axially (= LEVER_RIM_H, the lever-housing
# gap that the coil itself sets). Each leg is a straight radial wire at
# the same angular position as its stop pin — the leg passes straight
# through the pin's diametral through-hole (STOP_PIN_HOLE_D). Inside the
# hole, the leg wire pushes tangentially against the hole walls as the
# spring tries to twist; that transmits torque into the pin, which is
# anchored to the lever (or housing). The pin's solid outer body still
# provides the mechanical stop via cylinder-to-cylinder tangent contact
# with the opposing pin. Shown in rest state.
# ────────────────────────────────────────────────────────────────────────────

SPRING_COIL_MAJOR_R = 1.875   # mean coil radius (ID 3, OD 4.5 per Lee
                              # Spring LTM*050E — rod 3 mm, coil OD 4.5 mm,
                              # wire 0.51 mm → mean r = (1.5 + 2.25)/2)
SPRING_BODY_LEN     = 2.5           # measured axial extent of the coiled
                                    # section. Shorter than LEVER_RIM_H —
                                    # the lever's pivot boss is extended
                                    # by LEVER_BOSS_EXTENSION toward the
                                    # housing to fill the rest of the gap.
                                    # In assembled position the spring
                                    # sits against the HOUSING face; the
                                    # boss extension sits against the
                                    # lever face.
SPRING_VIZ_TURNS    = 5             # number of single-wire tori stacked
                                    # along the coil axis to suggest a
                                    # multi-turn helix (visual only — the
                                    # real spring has 4.25 turns).
SPRING_LEG_LEN      = STOP_PIN_R + STOP_PIN_D / 2 - SPRING_COIL_MAJOR_R + 1.0
                              # start at coil centerline, reach 1 mm past
                              # the pin's outer edge so the exit side is
                              # visible

def _torsion_spring(pivot_x, y_center,
                    lever_leg_alpha_deg, lever_leg_y,
                    housing_leg_alpha_deg, housing_leg_y):
    """Dummy torsion spring at the given pivot, y centered between the
    lever inner face and the housing column outer face. Coil axis along
    +y. Coiled section rendered as SPRING_VIZ_TURNS evenly-spaced single-
    wire tori; the OUTER wire surfaces of the end tori align with the
    gap boundaries (so the rendered envelope = SPRING_BODY_LEN exactly,
    no clipping). Each leg is a straight radial cylinder at the y of its
    mating stop pin's through-hole — the lever-side leg at lever_leg_y,
    the housing-side leg at housing_leg_y. Returned as a Compound;
    visual reference only."""
    parts = []
    pivot_z = LEVER_PIVOT_Z
    # Inset by SPRING_WIRE_R so the outermost torus's outer surface lands
    # exactly on each face, instead of its center sitting on the face
    # (which would clip half a wire diameter into the lever and housing).
    usable_len = SPRING_BODY_LEN - 2 * SPRING_WIRE_R
    if SPRING_VIZ_TURNS == 1:
        y_offsets = [0.0]
    else:
        step = usable_len / (SPRING_VIZ_TURNS - 1)
        y_offsets = [
            -usable_len / 2 + i * step for i in range(SPRING_VIZ_TURNS)
        ]
    for dy in y_offsets:
        parts.append(cq.Solid.makeTorus(
            SPRING_COIL_MAJOR_R, SPRING_WIRE_R,
            pnt=cq.Vector(pivot_x, y_center + dy, pivot_z),
            dir=cq.Vector(0, 1, 0),
        ))
    for alpha_deg, leg_y in (
        (lever_leg_alpha_deg, lever_leg_y),
        (housing_leg_alpha_deg, housing_leg_y),
    ):
        alpha_rad = math.radians(alpha_deg)
        cos_a = math.cos(alpha_rad)
        sin_a = math.sin(alpha_rad)
        start = cq.Vector(
            pivot_x + SPRING_COIL_MAJOR_R * cos_a,
            leg_y,
            pivot_z - SPRING_COIL_MAJOR_R * sin_a,
        )
        leg = cq.Solid.makeCylinder(
            SPRING_WIRE_R, SPRING_LEG_LEN,
            pnt=start,
            dir=cq.Vector(cos_a, 0, -sin_a),
        )
        parts.append(leg)
    return cq.Compound.makeCompound(parts)

# Spring y centered against the HOUSING face (boss extension takes up
# the lever-side LEVER_BOSS_EXTENSION mm of the gap; spring fills the
# remaining SPRING_BODY_LEN mm against the housing). Each leg is at the
# y of its mating pin's through-hole — offset STOP_PIN_HOLE_OFFSET in
# from the pin's own base toward the gap center. Legs share α with
# their pins.
_SPRING_Y_GAP_CENTER = HOUSING_W / 2 + SPRING_BODY_LEN / 2
_LEVER_LEG_Y_POS         = (LEVER_INNER_Y - STOP_PIN_H
                             + LEVER_STOP_PIN_HOLE_Y_FROM_GAP_END)  # +y side
_HOUSING_LEG_Y_POS       = HOUSING_W / 2 + STOP_PIN_HOLE_OFFSET     # +y side
# Brake housing pin uses a deeper offset (BRAKE_PIN_HOLE_BASE_CLR) because
# of the lip at its blind-hole mouth, so its dummy leg shifts inboard too.
_BRAKE_HOUSING_LEG_Y_POS = HOUSING_W / 2 + BRAKE_PIN_HOLE_BASE_CLR  # +y magnitude

ratchet_spring = _torsion_spring(
    RATCHET_PIVOT_X,
    +_SPRING_Y_GAP_CENTER,
    STOP_LEVER_PIN_ALPHA_RATCHET_DEG,    +_LEVER_LEG_Y_POS,
    STOP_HOUSING_PIN_ALPHA_RATCHET_DEG,  +_HOUSING_LEG_Y_POS,
)
brake_spring = _torsion_spring(
    BRAKE_PIVOT_X,
    -_SPRING_Y_GAP_CENTER,
    STOP_LEVER_PIN_ALPHA_BRAKE_DEG,    -_LEVER_LEG_Y_POS,
    STOP_HOUSING_PIN_ALPHA_BRAKE_DEG,  -_BRAKE_HOUSING_LEG_Y_POS,
)

# ── Dummy 608 bearings (visualization-only) ───────────────────────────
# Solid annulus, OD=BEARING_OD, ID=AXLE_D, height=BEARING_W. Bearings
# are purchased parts (not printed), so this is just for assembly preview.
def _bearing_dummy(z_base):
    return (
        cyl(BEARING_OD, BEARING_W, z=z_base)
        .cut(cyl(AXLE_D, BEARING_W, z=z_base))
    )

# Bottom bearing: in bearing_cap_bottom's pocket, sits on the lip.
bearing_bottom = _bearing_dummy(BEARING_LIP_H)
# Top bearing: in bearing_cap_top's pocket. Bottom face at z=pancake_bearing_z0
# (= cap interior face); top face wedges into the cone funnel where it has
# narrowed to Ø22.
bearing_top    = _bearing_dummy(pancake_bearing_z0)
# Guide bearing: rolls on the pancake-side flange's outer face. Axis
# along world X at the GUIDE_BEARING_Y_OFFSET, Z = SPOOL_H + BEARING_OD/2
# so the bearing's lower edge sits at SPOOL_H (= the pancake-side flange's
# top face).
def _build_bearing_guide():
    # Bearing is pushed onto the axle until its inner-race bottom-half face
    # bottoms out against the protruding tab end, so bearing X starts at
    # the tab's outer X face — guaranteeing 1 mm air gap to the mount wall.
    x_min = GUIDE_KEY_TAB_X_MAX
    z = GUIDE_AXLE_Z
    return (
        cq.Workplane("XY").add(cq.Solid.makeCylinder(
            BEARING_OD / 2, BEARING_W,
            pnt=cq.Vector(x_min, GUIDE_BEARING_Y_OFFSET, z),
            dir=cq.Vector(1, 0, 0),
        ))
    ).cut(
        cq.Workplane("XY").add(cq.Solid.makeCylinder(
            AXLE_D / 2, BEARING_W,
            pnt=cq.Vector(x_min, GUIDE_BEARING_Y_OFFSET, z),
            dir=cq.Vector(1, 0, 0),
        ))
    )


bearing_guide = _build_bearing_guide()


# Lever-side guide bearing — same 608 dummy, rolling on the lever-side
# flange's BOTTOM face at z=0. Bearing's outboard face flush with the
# axle's outer end (x = LEVER_GUIDE_AXLE_X_OUTER); bearing extents
# inboard by BEARING_W.
def _build_bearing_lever_guide():
    x_min = LEVER_GUIDE_AXLE_X_OUTER - BEARING_W
    z = LEVER_GUIDE_AXLE_Z
    return (
        cq.Workplane("XY").add(cq.Solid.makeCylinder(
            BEARING_OD / 2, BEARING_W,
            pnt=cq.Vector(x_min, LEVER_GUIDE_BEARING_Y_OFFSET, z),
            dir=cq.Vector(1, 0, 0),
        ))
    ).cut(
        cq.Workplane("XY").add(cq.Solid.makeCylinder(
            AXLE_D / 2, BEARING_W,
            pnt=cq.Vector(x_min, LEVER_GUIDE_BEARING_Y_OFFSET, z),
            dir=cq.Vector(1, 0, 0),
        ))
    )


bearing_lever_guide = _build_bearing_lever_guide()

