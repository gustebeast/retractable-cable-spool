"""Cable stop — a small C-clamp pinched onto the cable so it can't slip
back through the hole in the wood mounting plate.

Same split-ring pinch-clamp design as the cable top rim's height lock
(cable_rim.py), scaled way down: a C-shaped ring with one full radial slit,
and a tangential M2 screw + heat-set insert across the slit in a local
boss. Tightening the screw closes the slit, shrinking the bore onto the
cable. Closing a slit of width w shrinks both diameters by w/pi, so the
printed ring is oversized by exactly that: at full clamp the bore lands on
CABLE_STOP_ID_CLAMPED and the round body on CABLE_STOP_OD_CLAMPED.

Print flat on its end face — the screw bore is horizontal in print
orientation, so the insert pilot's lip step needs no taper (same argument
as the top rim's clamp).
"""

import math

import cadquery as cq

from .dimensions import (
    STRUCT_WALL,
    M2_HEAD_RECESS_D,
)
from .helpers import cyl
# Shared M2 hole geometry (cadkit/fasteners.py). `.dimensions` already put the
# cadkit/ folder on sys.path, so this flat import resolves.
from cadkit.fasteners import M2_ANCHOR_MIN_WALL, cut_m2_anchor, cut_m2_head_bore


CABLE_STOP_ID_CLAMPED = 10.0   # closed bore Ø — passes the Ø8 cable, blocks the Ø12 head
CABLE_STOP_OD_CLAMPED = 15.0   # round body Ø when closed (bears on the plate, blocks the head)
# Height: whatever the M2 mounting system needs — the tallest hole feature
# (the Ø M2_HEAD_RECESS_D head recess) plus STRUCT_WALL of material above
# and below it. The Ø3.3 insert pilot is smaller, so it gets more than a
# wall automatically.
CABLE_STOP_H          = M2_HEAD_RECESS_D + 2 * STRUCT_WALL    # 7.3
CABLE_STOP_SLIT_W     = 1.5    # open slit width (closing it = full clamp)

# Closing the slit removes CABLE_STOP_SLIT_W of circumference -> both
# diameters shrink by SLIT_W/pi. Print oversized by that so the CLAMPED
# dimensions hit the targets above.
_D_SHRINK     = CABLE_STOP_SLIT_W / math.pi          # ~0.48
CABLE_STOP_ID = CABLE_STOP_ID_CLAMPED + _D_SHRINK    # printed bore Ø
CABLE_STOP_OD = CABLE_STOP_OD_CLAMPED + _D_SHRINK    # printed body Ø

# Far lug is a standard M2 anchor (self-tap now, insert later): the Ø3.3 × 3.5
# pocket opens at the lug's OUTER face, with M2_MIN_BITE of Ø2.2 self-tap running
# from there to the slit. (In practice this clamp always gets the insert — the
# pinch pressure needs a real thread — but the geometry stays the standard
# anchor rather than a special case.)
CABLE_STOP_LUG_W = M2_ANCHOR_MIN_WALL   # 5.5 per side = 3.5 pocket + 2.0 bite

# Bore edge treatment is derived from the ring wall thickness (see
# _build_cable_stop): the TOP gets a full roundover (radius = wall) so no
# flat top face remains; the BOTTOM gets a 45° taper over the INNER half of
# the wall, leaving the OUTER half flat for reliable bed contact (a roundover
# there would overhang the bed, and a single thin contact ring prints badly).

# Screw-boss sizing: STRUCT_WALL of material on ALL transverse sides of the
# largest hole (the Ø M2_HEAD_RECESS_D head recess). The boss can't extend
# inward past the bore tangent (it would clip the cable bore near the slit),
# so the screw axis sits outboard: bore tangent + wall + recess radius.
_BOSS_X_IN          = (CABLE_STOP_ID_CLAMPED + CABLE_STOP_SLIT_W / math.pi) / 2
CABLE_STOP_SCREW_CX = _BOSS_X_IN + STRUCT_WALL + M2_HEAD_RECESS_D / 2   # ≈ 6.39
_BOSS_X_OUT         = CABLE_STOP_SCREW_CX + M2_HEAD_RECESS_D / 2 + STRUCT_WALL

# ── Wood-mount ear (on -X, opposite the C-clamp slit/screw) ──────────────────
# A flat tab extending WOOD_TAB_LEN past the OD with ONE countersunk wood
# screw (same spec as the housing bracket: Ø4 shaft, Ø9.3 countersunk head),
# driven +Z → -Z into a wood surface beneath the stop. Mirrors mount_bracket's
# wood-screw dims (kept local to avoid importing the heavy bracket build).
WOOD_SCREW_SHAFT_D = 4.0    # straight shaft Ø through to the wood
WOOD_SCREW_HEAD_D  = 9.3    # countersunk-head Ø at the top (+Z) face
WOOD_SCREW_HEAD_H  = 4.0    # countersink cone depth
WOOD_SCREW_BORE_GAP = 13.0  # gap from the bore's near (-X) edge to the screw
                            # hole's near edge — i.e. clear material between the
                            # cable hole and the wood screw. The tab extends as
                            # far as needed to seat the screw head past this.
WOOD_TAB_W         = WOOD_SCREW_HEAD_D + 2 * STRUCT_WALL   # Y width (wall around the head)


def _build_cable_stop():
    h      = CABLE_STOP_H
    half_y = CABLE_STOP_SLIT_W / 2
    lug_y  = half_y + CABLE_STOP_LUG_W                 # ±Y lug outer faces
    r_in   = CABLE_STOP_ID / 2
    r_out  = CABLE_STOP_OD / 2
    cx     = CABLE_STOP_SCREW_CX                       # screw axis radius
    cz     = h / 2                                     # screw axis height

    ring = cyl(CABLE_STOP_OD, h).cut(cyl(CABLE_STOP_ID, h))

    # Boss: flat ±Y lug faces around the slit for the screw head / insert
    # to seat on (the ring's own surface is curved). X span gives the head
    # recess STRUCT_WALL on its inner and outer sides; the boss protrudes
    # past the round OD — fine, the stop only needs the ROUND body to
    # out-size the plate hole.
    boss = (cq.Workplane("XY")
            .center((_BOSS_X_IN + _BOSS_X_OUT) / 2, 0)
            .box(_BOSS_X_OUT - _BOSS_X_IN, 2 * lug_y, h,
                 centered=(True, True, False)))
    part = ring.union(boss)

    # Shave the ring flush with the lug faces over the boss's X span: the
    # round wall otherwise bulges past |y| = lug_y near the bore, so the
    # head recess / insert pilot would punch through curved material with
    # no flat seat (and no flat face to press the insert against).
    def _lug_flat(sign_y):
        return (cq.Workplane("XY").workplane(offset=-1.0)
                .center((_BOSS_X_IN + _BOSS_X_OUT) / 2,
                        sign_y * (lug_y + r_out / 2))
                .box(_BOSS_X_OUT - _BOSS_X_IN, r_out, h + 2.0,
                     centered=(True, True, False)))
    part = part.cut(_lug_flat(+1)).cut(_lug_flat(-1))

    # Slit: radial slot from the bore out through the +X wall (the -X side
    # stays intact as the living hinge). Starting the box at x=0 is fine —
    # the bore is already air.
    slit = (cq.Workplane("XY").workplane(offset=-1.0)
            .box(_BOSS_X_OUT + 4.0, CABLE_STOP_SLIT_W, h + 2.0,
                 centered=(False, True, False)))
    part = part.cut(slit)

    # Bore edge treatment, sized off the ring wall thickness.
    wall  = r_out - r_in
    OS    = 0.3                                          # boolean overshoot
    cos45 = math.cos(math.radians(45))

    # TOP — full roundover (radius = wall): the bore edge rounds all the way
    # out to the OD, leaving NO flat top face. Revolved corner-minus-quarter-
    # disc. (The near-horizontal top of the roundover is on the print's top
    # side, so its overhang is acceptable.)
    R   = wall
    z_t = h - R                                          # arc tangency on the bore wall
    arc_mid = (r_in + R - R * cos45, z_t + R * cos45)
    top_roundover = (cq.Workplane("XZ")
                     .moveTo(r_in - OS, z_t)
                     .lineTo(r_in - OS, h + OS)
                     .lineTo(r_in + R,  h + OS)
                     .lineTo(r_in + R,  h)
                     .threePointArc(arc_mid, (r_in, z_t))
                     .close()
                     .revolve(360, (0, 0), (0, 1)))
    part = part.cut(top_roundover)

    # BOTTOM — 45° taper over the INNER half of the wall (cable lead-in),
    # leaving the OUTER half flat for reliable first-layer bed contact. A
    # roundover here would overhang the bed; a single thin contact ring
    # prints unreliably, hence the 50/50 flat/taper split.
    c = wall / 2
    bot_taper = (cq.Workplane("XZ")
                 .moveTo(r_in - OS, -OS)
                 .lineTo(r_in - OS, c)
                 .lineTo(r_in,      c)                  # taper top, on the bore wall
                 .lineTo(r_in + c,  0.0)                # 45° down to the bottom face
                 .lineTo(r_in + c,  -OS)
                 .close()
                 .revolve(360, (0, 0), (0, 1)))
    part = part.cut(bot_taper)

    # ── M2 screw cuts LAST (nothing may re-cut across them) ──────────────
    # +Y head lug: head recess at the outer face, shaft clearance through to the
    # slit, so the screw spins free and can PULL the lugs together.
    # -Y far lug: the standard M2 anchor, mouthed at the -Y OUTER face (where the
    # insert melts in); its Ø2.2 self-tap runs inward to the slit.
    part = cut_m2_head_bore(part, (cx, lug_y, cz), (0, -1, 0),
                            clr_len=CABLE_STOP_LUG_W + 0.25, overshoot=0.25)
    part = cut_m2_anchor(part, (cx, -lug_y, cz), (0, 1, 0),
                         depth=CABLE_STOP_LUG_W, overshoot=0.5)

    # ── Wood-mount ear on -X (opposite the +X clamp slit) ────────────────
    # Only WOOD_SCREW_HEAD_H tall (the ear needs no more than the countersink
    # cone depth — no reason to match the full part height). Its bottom sits
    # flush with the ring bottom (z=0), so the whole part rests flat on the
    # wood. The screw's NEAR edge sits WOOD_SCREW_BORE_GAP from the bore's
    # near edge; the ear extends out far enough to seat the head + a wall.
    ear_h     = WOOD_SCREW_HEAD_H                        # ear thickness in Z
    ws_x      = -(r_in + WOOD_SCREW_BORE_GAP + WOOD_SCREW_SHAFT_D / 2)
    x_tab_in  = -(r_out - 1.0)                           # overlap 1 mm into the ring
    x_tab_out = ws_x - (WOOD_SCREW_HEAD_D / 2 + STRUCT_WALL)
    tab = (cq.Workplane("XY")
           .center((x_tab_in + x_tab_out) / 2, 0)
           .box(x_tab_in - x_tab_out, WOOD_TAB_W, ear_h, centered=(True, True, False)))
    part = part.union(tab)

    # Wood screw cut LAST: a Ø9.3 → Ø4 countersink cone spanning the full ear
    # (head flush at the ear top z=ear_h, Ø4 shaft exits at z=0 into the wood
    # below). Ear thickness = cone depth, so no separate straight shaft needed.
    OS = 0.3
    _slope  = (WOOD_SCREW_HEAD_D - WOOD_SCREW_SHAFT_D) / WOOD_SCREW_HEAD_H
    _wide_d = WOOD_SCREW_HEAD_D + OS * _slope            # Ø at the top face = HEAD_D
    cone = cq.Workplane().add(cq.Solid.makeCone(
        _wide_d / 2, WOOD_SCREW_SHAFT_D / 2, WOOD_SCREW_HEAD_H + 2 * OS,
        pnt=cq.Vector(ws_x, 0, ear_h + OS), dir=cq.Vector(0, 0, -1)))
    return part.cut(cone)


cable_stop = _build_cable_stop()
