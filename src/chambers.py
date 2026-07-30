"""cable-chamber caps + height clamp.

  cable_ceiling / cable_floor — plain 10 mm cylinders that slide on the hub, one
      each side of the separator, key-slotted so they turn with it. They cap the
      wound cable; no self-clamp. The cable itself stops each cap moving toward
      the separator; the height_clamp locks the other direction.
  height_clamp — a 4 mm C-clamp (split collar + tangential M2 pinch) that grips
      the hub at a set height and butts against a cap's outer face to lock it.
      Used twice (above the ceiling, below the floor).
"""

import math

import cadquery as cq

from src.helpers import cyl, heal, make_keys
from src.dimensions import HUB_OD
from cadkit.fasteners import (
    m2_head_bore_cutter, m2_anchor_cutter, M2_ANCHOR_MIN_WALL,
)
from .params import (
    RIM_ID, CAP_H, CAP_OD, CLAMP_H, CLAMP_OD, CLAMP_SLIT_W,
    CLAMP_HW_ANGLE_DEG, SEAT_SHOULDER_OD, SEAT_CONE_H,
    CAP_SPOKE_N, CAP_SPOKE_W, CAP_HUB_COLLAR_R,
    CAP_MID_R0, CAP_MID_R1, CAP_RIM_R0,
)


def _arc_slot(r0, r1, a_lo, a_hi, h):
    """One through-slot cutter: an annular sector r0..r1 whose radial sides
    are offset so the SPOKES between slots keep constant CAP_SPOKE_W width
    (the angular inset asin((W/2)/r) shrinks with radius)."""
    def edge(a_deg, r):
        d = math.degrees(math.asin((CAP_SPOKE_W / 2.0) / r))
        return a_deg + d if a_deg == a_lo else a_deg - d

    pts = []
    n = 6
    for i in range(n + 1):                       # inner arc, a_lo→a_hi
        a = math.radians(edge(a_lo, r0) + (edge(a_hi, r0) - edge(a_lo, r0)) * i / n)
        pts.append((r0 * math.cos(a), r0 * math.sin(a)))
    for i in range(n + 1):                       # outer arc, back a_hi→a_lo
        a = math.radians(edge(a_hi, r1) + (edge(a_lo, r1) - edge(a_hi, r1)) * i / n)
        pts.append((r1 * math.cos(a), r1 * math.sin(a)))
    return (cq.Workplane("XY").workplane(offset=-0.5)
            .polyline(pts).close().extrude(h + 1.0))


def _cap(h):
    """A cable-chamber cap: annular cylinder on the hub, key-slotted for
    rotation lock, with two rows of ARC-SLOT windows (see the wound cable at
    install) between CAP_SPOKE_N continuous radial spokes, a solid mid band,
    and a solid outer rim (params block explains the sizing)."""
    c = cyl(CAP_OD, h, z=0).cut(cyl(RIM_ID, h + 1.0, z=-0.5))
    c = c.cut(make_keys(HUB_OD / 2, -0.5, h + 0.5, groove=True))
    pitch = 360.0 / CAP_SPOKE_N
    for r0, r1 in ((CAP_HUB_COLLAR_R, CAP_MID_R0), (CAP_MID_R1, CAP_RIM_R0)):
        for i in range(CAP_SPOKE_N):
            c = c.cut(_arc_slot(r0, r1, i * pitch, (i + 1) * pitch, h))
    return heal(c)


def _build_height_clamp():
    h = CLAMP_H
    r_out = CLAMP_OD / 2
    # split collar sliding on the hub (bore = RIM_ID; the pinch closes it to grip)
    collar = cyl(CLAMP_OD, h, z=0).cut(cyl(RIM_ID, h + 2.0, z=-1.0))
    collar = collar.cut(make_keys(HUB_OD / 2, -0.5, h + 0.5, groove=True))
    # pinch boss beside the slit (holds the M2 that pulls the C closed)
    lug_y = CLAMP_SLIT_W / 2 + M2_ANCHOR_MIN_WALL
    boss = (cq.Workplane("XY").center(r_out + 1.0, 0)
            .box(6.0, CLAMP_SLIT_W + 2 * M2_ANCHOR_MIN_WALL, h,
                 centered=(True, True, False)))
    # C-slit from the bore out through the boss (full height) → the collar is a "C"
    slit = (cq.Workplane("XY").workplane(offset=-0.5)
            .box(r_out + 7.0, CLAMP_SLIT_W, h + 1.0, centered=(False, True, False)))
    cz, cx = h / 2, r_out + 2.5
    head = m2_head_bore_cutter((cx, lug_y, cz), (0, -1, 0),
                               clr_len=M2_ANCHOR_MIN_WALL + 0.25, overshoot=0.25)
    anch = m2_anchor_cutter((cx, -lug_y, cz), (0, 1, 0),
                            depth=M2_ANCHOR_MIN_WALL, overshoot=0.5)
    # clock the whole pinch assembly HALFWAY between two key slots (the keys
    # are on 60° pitch) so the slit never runs through a key groove
    rot = lambda w: w.rotate((0, 0, 0), (0, 0, 1), CLAMP_HW_ANGLE_DEG)  # noqa: E731
    clamp = collar.union(rot(boss))
    clamp = clamp.cut(rot(slit)).cut(rot(head)).cut(rot(anch))
    return heal(clamp)


def _build_floor():
    """cable_floor sits just under the separator, whose seat notch flares to
    SEAT_SHOULDER_OD right above it — so recess the floor's TOP-inner to clear
    the notch. The floor's outer part still reaches the separator to cap the
    cable; only the inner lip (over the notch) is relieved."""
    relief_h = SEAT_CONE_H + 0.5
    relief = cyl(SEAT_SHOULDER_OD + 1.0, relief_h + 0.5, z=CAP_H - relief_h)
    return heal(_cap(CAP_H).cut(relief))


cable_ceiling = _cap(CAP_H)
cable_floor   = _build_floor()
height_clamp  = _build_height_clamp()
