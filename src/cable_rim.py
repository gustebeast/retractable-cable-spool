"""Cable-retention TOP rim — a separately-printed lid that slides down the
spool body and holds the wound cable in its channel.

Mirrors the bottom rim's spoke grid (same count, STRUCT_WALL wide) but only
TOP_RIM_H tall, with the spokes offset half a pitch so they split the
bottom rim's gaps (the cable is sandwiched between two grids whose gaps
never align, so it can't escape). An inner collar wraps the hub and slides
along it; 6 key slots run on the hub's key bumps (same lock as the top
bearing cap) to stop the rim rotating.

For now nothing fixes its height — it slides freely up/down the hub. The
assembly places it CABLE_GAP mm above the bottom rim (see build.py).
"""

import math

import cadquery as cq

from .dimensions import (
    HUB_OD, FIT_CLR, STRUCT_WALL, TOP_RIM_H,
    KEY_W, KEY_DEPTH, BOOL_OVERSHOOT,
    M2_SHAFT_CLR_D, M2_INSERT_PILOT_D, M2_INSERT_DEPTH,
    M2_HEAD_RECESS_D, M2_HEAD_RECESS_H,
)
from .spool import RIM_ID, RIM_OD, CABLE_D
from .helpers import cyl, make_keys


COLLAR_WALL = 2.0                                   # radial wall of the hub-wrapping collar

_COLLAR_ID  = HUB_OD + 2 * FIT_CLR                  # slip fit over the hub OD
_COLLAR_OD  = _COLLAR_ID + 2 * COLLAR_WALL
_OUTER_OD   = RIM_OD                                # flush with the bottom (brake) rim OD —
                                                    # it stacks ABOVE the brake rim (cable air
                                                    # gap between), not nested inside it
_OUTER_ID   = _OUTER_OD - 2 * STRUCT_WALL

# Same spoke count as the bottom rim (gap ≤ CABLE_D at the rim), offset by
# half a pitch so the top grid's spokes sit over the bottom grid's gaps.
# (Sized against the BRAKE-band inner — the widest part of the bottom rim's
# cable channel — matching spool._channel_spokes.)
_N_SPOKES     = math.ceil(2 * math.pi * ((RIM_OD - 2 * STRUCT_WALL) / 2)
                          / (CABLE_D + STRUCT_WALL))
_SPOKE_OFFSET = 180.0 / _N_SPOKES                   # half-pitch

# ── Split-collar pinch clamp (height lock) ──────────────────────────────────
# A single full-radial slit turns the whole rim into a "C". A tangential M3
# screw across the slit, in a local boss, pulls the gap closed — contracting
# the collar onto the hub to lock the height anywhere (continuous). Sized for
# PETG (ductile, grippy) per the material discussion. The hub key bumps still
# carry anti-rotation; the clamp only needs to resist the cable lifting the lid.
CLAMP_SLIT_W      = 1.5     # open gap (closing it shrinks the collar ID ~0.48 mm)
CLAMP_SLIT_ANGLE  = 8.0     # a top-rim spoke gap, clear of the 6 key angles
CLAMP_BOSS_H      = 12.0    # boss rises ABOVE the 7 mm rim so the screw hole sits
                            # entirely above the rim's top surface (accessible from
                            # the side). The collar still clamps over the 7 mm height.
CLAMP_BOSS_R0     = _COLLAR_ID / 2          # boss inner radius (collar bore)
CLAMP_BOSS_R1     = 44.0                     # boss outer radius (< RIM_ID/2 = 54, clears the wall)
CLAMP_INSERT_LIP_T = 1.0    # lip length the insert seats against (its hole is the
                            # Ø M2_SHAFT_CLR_D the screw passes through)
# Lug must fit the heat-set insert (M2_INSERT_DEPTH) PLUS the seating lip.
CLAMP_LUG_W       = M2_INSERT_DEPTH + CLAMP_INSERT_LIP_T   # 4.5


def _clamp_features():
    """Return (boss_solid, cut_solid) for the pinch clamp at θ=0, ready to be
    rotated to CLAMP_SLIT_ANGLE. boss is unioned in; cut is the slit + the M2
    screw bore, drilled with the project's standard specs:
      - +Y (head) lug: Ø M2_HEAD_RECESS_D × M2_HEAD_RECESS_H head recess at the
        outer face, then a Ø M2_SHAFT_CLR_D shaft clearance through to the slit.
      - -Y (insert) lug: Ø M2_INSERT_PILOT_D heat-set pilot, M2_INSERT_DEPTH
        deep from the -Y OUTER face (accessible side), seating against a 1 mm
        Ø M2_SHAFT_CLR_D lip on the slit side. No conical chamfer — the screw
        bore is horizontal in the print orientation, so the pilot→lip step
        needs no self-supporting taper."""
    half_y      = CLAMP_SLIT_W / 2
    cx          = (CLAMP_BOSS_R0 + CLAMP_BOSS_R1) / 2     # screw axis radius
    lug_outer_y = half_y + CLAMP_LUG_W                     # ±Y lug outer faces
    # Screw axis sits entirely ABOVE the rim's top surface (TOP_RIM_H) so the
    # whole bore clears the rim and is reachable. (Stays tangential — that's
    # the only axis that closes the slit.)
    cz = TOP_RIM_H + M2_INSERT_PILOT_D / 2 + 0.5          # bore bottom 0.5 mm above the rim top

    boss = (
        cq.Workplane("XY")
        .center((CLAMP_BOSS_R0 + CLAMP_BOSS_R1) / 2, 0)
        .box(CLAMP_BOSS_R1 - CLAMP_BOSS_R0,
             CLAMP_SLIT_W + 2 * CLAMP_LUG_W,
             CLAMP_BOSS_H, centered=(True, True, False))
    )
    # Slit: thin radial slot through the whole rim (and the boss) at θ=0.
    slit = (
        cq.Workplane("XY").workplane(offset=-1.0)
        .box(_OUTER_OD / 2 + 4.0, CLAMP_SLIT_W, CLAMP_BOSS_H + 2.0,
             centered=(False, True, False))
    )

    def _cyl_y(d, h, y0, ydir):
        return cq.Workplane().add(cq.Solid.makeCylinder(
            d / 2, h, pnt=cq.Vector(cx, y0, cz), dir=cq.Vector(0, ydir, 0)))

    # +Y head lug: head recess at the outer face + shaft clearance to the slit.
    head_recess = _cyl_y(M2_HEAD_RECESS_D, M2_HEAD_RECESS_H + 0.25, lug_outer_y + 0.25, -1)
    head_clr    = _cyl_y(M2_SHAFT_CLR_D, CLAMP_LUG_W + 0.5, lug_outer_y + 0.25, -1)
    # -Y insert lug: heat-set pilot drilled from the -Y OUTER face (the
    # accessible side). The insert seats against a CLAMP_INSERT_LIP_T (1 mm)
    # long lip on the slit side, whose bore is the Ø M2_SHAFT_CLR_D the screw
    # passes through to pull the insert against it.
    pilot_inner_y = -lug_outer_y + M2_INSERT_DEPTH         # pilot's inner (slit-facing) end
    pilot   = _cyl_y(M2_INSERT_PILOT_D, M2_INSERT_DEPTH + 0.5, -lug_outer_y - 0.5, +1)
    lip_clr = _cyl_y(M2_SHAFT_CLR_D, (pilot_inner_y * -1) + half_y + 1.0, pilot_inner_y, +1)

    cut = slit.union(head_recess).union(head_clr).union(pilot).union(lip_clr)
    return (boss.rotate((0, 0, 0), (0, 0, 1), CLAMP_SLIT_ANGLE),
            cut.rotate((0, 0, 0), (0, 0, 1), CLAMP_SLIT_ANGLE))


def _build_cable_top_rim():
    z0 = 0.0
    collar = cyl(_COLLAR_OD, TOP_RIM_H, z=z0).cut(cyl(_COLLAR_ID, TOP_RIM_H, z=z0))
    outer  = cyl(_OUTER_OD,  TOP_RIM_H, z=z0).cut(cyl(_OUTER_ID,  TOP_RIM_H, z=z0))
    part = collar.union(outer)

    r_in  = _COLLAR_OD / 2 - 0.5      # overlap into the collar
    r_out = _OUTER_ID / 2 + 0.5       # overlap into the outer ring
    for i in range(_N_SPOKES):
        ang = i * 360.0 / _N_SPOKES + _SPOKE_OFFSET
        sp = (
            cq.Workplane("XY")
            .center((r_in + r_out) / 2, 0)
            .box(r_out - r_in, STRUCT_WALL, TOP_RIM_H, centered=(True, True, False))
            .rotate((0, 0, 0), (0, 0, 1), ang)
        )
        part = part.union(sp)

    # Split-collar clamp: union the boss FIRST so the key-groove cut below
    # carves through it too. The boss sits at θ = CLAMP_SLIT_ANGLE (= 8°)
    # and spans ~±10° tangentially at its inner face, so the hub key bump
    # at θ=0° falls inside the boss's angular footprint — without a groove
    # through the boss, that bump collides with the boss's -X face when
    # the cable_top_rim slides down the hub.
    boss, cut = _clamp_features()
    part = part.union(boss)

    # Key slots in the collar AND the clamp boss for the hub bumps (groove
    # = oversized by FIT_CLR). Cut height extended from TOP_RIM_H up to
    # CLAMP_BOSS_H so the groove runs through the full boss height — the
    # hub bumps extend the full spool height, so they enter the boss above
    # the rim top too.
    part = part.cut(make_keys(HUB_OD / 2, z0, z0 + CLAMP_BOSS_H, groove=True))

    # Sliver kill: because the boss is rotated by CLAMP_SLIT_ANGLE = 8° but
    # the θ=0° key groove is axis-aligned, the boss's -Y face at the inner
    # radius (where the bump enters the boss) lands ~0.04 mm beyond the
    # groove's -Y edge — leaving a hair-thin vertical sliver of boss
    # material between the groove and the boss face. Carve an extra slot
    # adjacent to the θ=0° groove, widened toward -Y, to absorb it.
    _SLIVER_KILL_W = 0.5
    sliver_kill = (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .center(HUB_OD / 2 + KEY_DEPTH / 2,
                -(KEY_W / 2 + FIT_CLR + _SLIVER_KILL_W / 2))
        .box(KEY_DEPTH + 2 * BOOL_OVERSHOOT, _SLIVER_KILL_W,
             CLAMP_BOSS_H, centered=(True, True, False))
    )
    part = part.cut(sliver_kill)

    # Slit + M2 screw holes for the pinch clamp.
    part = part.cut(cut)
    return part


cable_top_rim = _build_cable_top_rim()
