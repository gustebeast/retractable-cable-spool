"""MOUNT — desk/wall mount: bracket (tenon bar) ↔ frame_top (mortises).

The BRACKET is a full-frame-length PLUS: the tenon bar (two of the original design's wood
screws, two HANGING octagon tenons — cadkit.joinery octagon family, flipped
bulb-down) crossed by a JOINERY-FREE arm (no tenons, no screws) that rides
the perpendicular ribs' top faces, sandwiched flat between the wood and the
frame for lateral support. It screws flat to the mounting surface (desk
underside, shelf, wall…). frame_top carries the
matching channel pattern along BOTH its X and Y bars — mount the bracket in
either orientation to pick the install axis. Install:

  1. offer the assembly up: the tenons drop DOWN through the open ENTRY
     POCKETS (octagon_mortise(pocket=True), flipped → open at the top face);
  2. slide the assembly +x (or +y) ~21.5 mm: the tenons run into the
     retained MORTISE segments and butt the stops. The hang load sits on
     the mortise neck lips → tenon shoulders; the cable exit's +45° azimuth
     means working pulls push the frame INTO the stops;
  3. slide the floating TPU LOCK TENON into the cross arm's end mortise
     (half in the arm's underside, half in the rib's top face — aligned
     when seated): it spans the joint plane and shear-blocks the
     back-slide. Its axis is ⊥ the slide axis, so no working load can push
     it out; a plain rectangular bar, overhang-free in any print
     orientation, with a proud pull tail for removal.

FLIPPED-OCTAGON print story (why upside down beats the native pose here):
  · frame_top (−Z→+Z, unchanged): the cavity is a neck slot, 45° lip
    undersides, vertical bulb walls, 45° closing tapers, and a FLAT FLOOR —
    the library's one-nozzle bridge roof becomes that floor; nothing
    bridges at all.
  · bracket prints FLIPPED (+Z→−Z, bar flat on the bed, tenons standing) —
    the library-native tenon orientation, support-free.
Both halves use the stock octagon generators rotated 180° about the X axis
— a rigid transform, so the print-tested fit clearances carry over.
"""

import cadquery as cq

from src.helpers import heal
from .params import (
    TOP_RIB_Z1, MOUNT_TEN_W, MOUNT_TEN_L, MOUNT_CLR,
    MOUNT_MORT_L, MOUNT_POCKET_L,
    MOUNT_MORT_A_X0, MOUNT_POCK_A_X0, MOUNT_MORT_B_X0, MOUNT_POCK_B_X0,
    MOUNT_TEN_A_X0, MOUNT_TEN_B_X0,
    MOUNT_PLATE_T, MOUNT_PLATE_W, MOUNT_PLATE_X0, MOUNT_PLATE_X1,
    MOUNT_CROSS_HALF, MOUNT_SCREW_X, MOUNT_R_OUT,
    MOUNT_LOCK_W, MOUNT_LOCK_D, MOUNT_LOCK_L, MOUNT_LOCK_Y0, MOUNT_LOCK_TAIL,
    WOOD_SCREW_SHAFT_D, WOOD_SCREW_HEAD_D, WOOD_SCREW_HEAD_H,
)
from cadkit.joinery import octagon_tenon, octagon_mortise

_NOZ = 0.8


def _flip(w):
    """Native octagon pose → hanging pose: 180° about the X axis (x spans
    are preserved; every profile here is y-symmetric)."""
    return w.rotate((0, 0, 0), (1, 0, 0), 180.0)


def mount_channel_cuts():
    """The four channel cutters (X pattern) plus their 90° copies (Y
    pattern), in world position — frame.py subtracts these from frame_top.
    Mortise segments retain; pockets are the z-entry, open at the rib's top
    face. Each mortise+pocket pair shares a boundary plane, cutting one
    continuous channel."""
    cuts = []
    for x0, ln, pocket in ((MOUNT_MORT_A_X0, MOUNT_MORT_L, False),
                           (MOUNT_POCK_A_X0, MOUNT_POCKET_L, True),
                           (MOUNT_MORT_B_X0, MOUNT_MORT_L, False),
                           (MOUNT_POCK_B_X0, MOUNT_POCKET_L, True)):
        c = (_flip(octagon_mortise(MOUNT_TEN_W, ln, _NOZ, MOUNT_CLR,
                                   drop=2.0, pocket=pocket))
             .translate((x0, 0.0, TOP_RIB_Z1)))
        cuts.append(c)
        cuts.append(c.rotate((0, 0, 0), (0, 0, 1), 90.0))
    # LOCK grooves: the rib-side half of the lock mortise — a MOUNT_LOCK_D-
    # deep slot in the top face at every arm's outboard tip (all four, so
    # both bracket orientations find one under a cross-arm end), open at
    # the arm's end face for axial key entry
    g = (cq.Workplane("XY")
         .box(MOUNT_LOCK_W, MOUNT_LOCK_L + 1.0, MOUNT_LOCK_D + 0.5,
              centered=(True, False, False))
         .translate((0.0, MOUNT_LOCK_Y0, TOP_RIB_Z1 - MOUNT_LOCK_D)))
    for a in (0.0, 90.0, 180.0, 270.0):
        cuts.append(g.rotate((0, 0, 0), (0, 0, 1), a))
    return cuts


def _screw_cut(x):
    """One wood-screw cutter in PRINT POSE (bar z 0..T, tenons up): Ø4
    shaft through, countersink cone opening at the print-top face — the
    world UNDERSIDE, where the head must sit flush (the frame's rib top
    slides directly beneath it). Cone extrapolated past the face for a
    clean boolean break, the original design-style."""
    t = MOUNT_PLATE_T
    slope = (WOOD_SCREW_HEAD_D - WOOD_SCREW_SHAFT_D) / 2.0 / WOOD_SCREW_HEAD_H
    over = 0.5
    shaft = cq.Solid.makeCylinder(
        WOOD_SCREW_SHAFT_D / 2.0, t + 1.0,
        cq.Vector(x, 0.0, -0.5), cq.Vector(0, 0, 1))
    cone = cq.Solid.makeCone(
        WOOD_SCREW_SHAFT_D / 2.0,
        WOOD_SCREW_HEAD_D / 2.0 + over * slope,
        WOOD_SCREW_HEAD_H + over,
        cq.Vector(x, 0.0, t - WOOD_SCREW_HEAD_H), cq.Vector(0, 0, 1))
    return cq.Workplane(obj=shaft).union(cq.Workplane(obj=cone))


def _build_bracket():
    """Built in PRINT POSE (bar on the bed z 0..T, native tenons standing
    on top, roots sunk 1 mm), then flipped to world: tenons hanging, bar
    underside coplanar with the frame's top face. Print exactly as the
    flip implies: +Z→−Z, support-free."""
    t = MOUNT_PLATE_T
    bar = (cq.Workplane("XY")
           .box(MOUNT_PLATE_X1 - MOUNT_PLATE_X0, MOUNT_PLATE_W, t,
                centered=(False, True, False))
           .translate((MOUNT_PLATE_X0, 0.0, 0.0)))
    # PLUS: the joinery-free cross arm (lateral support — rides the other
    # ribs' top faces; no tenons, no screw holes on this axis)
    bar = bar.union(
        cq.Workplane("XY")
        .box(MOUNT_PLATE_W, 2.0 * MOUNT_CROSS_HALF, t,
             centered=(True, True, False)))
    # lock grooves in the cross arm's world-UNDERSIDE (= print-top face,
    # z=t here), BOTH ends — keeps the part y-symmetric, so the build→world
    # flip can't move the lock to the wrong end
    for sy in (+1.0, -1.0):
        y0 = sy * MOUNT_LOCK_Y0
        bar = bar.cut(
            cq.Workplane("XY")
            .box(MOUNT_LOCK_W, MOUNT_LOCK_L + 1.0, MOUNT_LOCK_D + 0.5,
                 centered=(True, False, False))
            .translate((0.0, min(y0, y0 + sy * (MOUNT_LOCK_L + 1.0)),
                        t - MOUNT_LOCK_D)))
    for x0 in (MOUNT_TEN_A_X0, MOUNT_TEN_B_X0):
        bar = bar.union(
            octagon_tenon(MOUNT_TEN_W, MOUNT_TEN_L, _NOZ, MOUNT_CLR, root=1.0)
            .translate((x0, 0.0, t)))
    for sx in (-MOUNT_SCREW_X, MOUNT_SCREW_X):
        bar = bar.cut(_screw_cut(sx))
    return heal(_flip(bar).translate((0, 0, TOP_RIB_Z1 + t)))


def _build_lock():
    """The floating TPU LOCK TENON (local: joint plane at z 0, sliding
    axis = y): a plain rectangular bar — overhang-free in ANY print
    orientation — that spans the bracket↔frame joint plane half-and-half
    (MOUNT_LOCK_D each side), sized line-on-line to its grooves (95A TPU +
    print fattening = press fit). The engaged length fills the groove; the
    tail runs proud of the arm end as the removal grip."""
    return heal(
        cq.Workplane("XY")
        .box(MOUNT_LOCK_W, MOUNT_LOCK_L + MOUNT_LOCK_TAIL, 2.0 * MOUNT_LOCK_D,
             centered=(True, False, True)))


def lock_in_place():
    """The lock seated in the +Y cross-arm end mortise (world pose)."""
    return mount_lock_tpu.translate((0.0, MOUNT_LOCK_Y0, TOP_RIB_Z1))


mount_bracket = _build_bracket()
mount_lock_tpu = _build_lock()
