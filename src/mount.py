"""MOUNT — desk/wall brackets, v3's FLUSH redesign (params block for the
full story). ONE printed bracket, used twice: seated on the +Y arm as
rotated 90°, on the −Y arm as 270°. Assembled, the bracket tops are
LEVEL with the frame's top face — the stack stays FRAME_RIB thick
(user's call: vertical space is precious; v2's bar added its 4.0 on
top).

Each bracket, inboard → out (local frame, arm along +X):
  · the mushroom TENON (cadkit joint, THROUGH mortise — both hosts
    print +z→−z) hanging under a full-depth SPINE section;
  · the shallow RAIL — one stem wide, riding the frame's 45°-V-bottom
    groove across the cap + arc-joint zone (under hang load the groove
    mouth at the arm tip becomes the near support for the pad, so the
    rail's bending span collapses to the pad gap);
  · the SCREW PAD in free air outboard of the arm tip, carrying v2's
    validated wood-screw countersink (verbatim geometry: Ø4 shaft,
    Ø9.3 × 4.0 cone opening at the world underside — the head stays
    driver-accessible from below with the assembly attached).

PRINT +z→−z (top face on the bed): pad, spine and rail all sit ON the
bed; the mushroom tenon rises library-native (stem → 45° flare → flat
top as a plain last layer). Built in PRINT POSE and flipped, v2's
pattern — _screw_cut carries over verbatim.
"""

import cadquery as cq

from cadkit.joinery import PrintSpec, joint

from .helpers import heal
from .params import (
    NOZZLE, FRAME_Z1,
    MOUNT_TEN_W, MOUNT_TEN_L, MOUNT_THRU_D, MOUNT_PLATE_T,
    MOUNT_T0, MOUNT_TEN_R1, MOUNT_RAIL_X0, MOUNT_RAIL_T,
    MOUNT_SPINE_W, MOUNT_PAD_X0, MOUNT_PAD_X1, MOUNT_PAD_W, MOUNT_SCREW_X,
    MOUNT_YP_MORT, MOUNT_YP_POCK, MOUNT_YM_MORT, MOUNT_YM_POCK,
    MOUNT_GRV_W, MOUNT_GRV_VERT, MOUNT_GRV_FLAT, MOUNT_GRV_DEPTH,
    FRAME_R_OUT,
    WOOD_SCREW_SHAFT_D, WOOD_SCREW_HEAD_D, WOOD_SCREW_HEAD_H,
)

_SPEC = PrintSpec(nozzle=NOZZLE, material="PETG-GF", facing="up")
# install "-x": on the +Y arm the tenon travels inboard (−r) to its stop;
# the −Y arm runs the same slide mirrored (the profile is x-uniform, so
# the sign is bookkeeping — both arms seat on ONE frame +y slide)
MOUNT_J = joint(MOUNT_TEN_W, MOUNT_TEN_L, tenon=_SPEC, mortise=_SPEC,
                install="-x", depth=MOUNT_THRU_D, through=True)


def _flip(w):
    """Print pose → world: 180° about X, then top face to FRAME_Z1."""
    return (w.rotate((0, 0, 0), (1, 0, 0), 180.0)
            .translate((0, 0, FRAME_Z1)))


def _pbox(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane("XY").workplane(offset=z0)
            .polyline([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
            .close().extrude(z1 - z0))


def _screw_cut(x):
    """One wood-screw cutter in PRINT POSE (plate z 0..T): Ø4 shaft
    through, countersink cone opening at the print-top face — the world
    UNDERSIDE, where the head must sit flush. Cone extrapolated past the
    face for a clean boolean break. v2's validated geometry, verbatim."""
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
    """Built in PRINT POSE (top faces on the bed z=0), then flipped."""
    t = MOUNT_PLATE_T
    hw = MOUNT_SPINE_W / 2.0
    b = _pbox(MOUNT_T0, MOUNT_RAIL_X0, -hw, hw, 0.0, t)          # deep spine
    b = b.union(_pbox(MOUNT_RAIL_X0, MOUNT_PAD_X0 + 1.0,
                      -hw, hw, 0.0, MOUNT_RAIL_T))               # shallow rail
    b = b.union(_pbox(MOUNT_PAD_X0, MOUNT_PAD_X1,
                      -MOUNT_PAD_W / 2.0, MOUNT_PAD_W / 2.0,
                      0.0, t))                                   # screw pad
    # the tenon: root 2.0 reaches through the spine AND past the rail's
    # underside at the 0.5 the tenon runs beyond the spine step
    b = b.union(MOUNT_J.tenon(root=2.0).translate((MOUNT_T0, 0.0, t)))
    b = b.cut(_screw_cut(MOUNT_SCREW_X))
    return heal(_flip(b))


def _v_groove(x0, x1):
    """Rail passage across the arm's cap + arc-joint zone: vertical walls
    MOUNT_GRV_VERT deep, then the 45° V closing on the dulled
    MOUNT_GRV_FLAT — the groove's −z boundary is the roof of the slot in
    the frame's +z→−z print. The rail (flat-bottomed) rides the vertical
    section; the V below stays empty."""
    hw = MOUNT_GRV_W / 2.0
    hf = MOUNT_GRV_FLAT / 2.0
    z1 = FRAME_Z1 + 1.0
    zv = FRAME_Z1 - MOUNT_GRV_VERT
    zb = FRAME_Z1 - MOUNT_GRV_DEPTH
    return (cq.Workplane("YZ").workplane(offset=x0)
            .polyline([(-hw, z1), (hw, z1), (hw, zv),
                       (hf, zb), (-hf, zb), (-hw, zv)])
            .close().extrude(x1 - x0))


def mount_channel_cuts():
    """The frame_top cutters for BOTH Y arms (world pose): per arm one
    retained MORTISE cavity + one entry POCKET (both THROUGH the arm —
    their far ends exit the underside over the drum chamber; the spine
    slot above the mating plane opens through the top face), plus the
    rail V-GROOVE running out the arm's tip face."""
    cuts = []
    for ang, segs, grv in (
            (90.0, ((MOUNT_YP_MORT, False), (MOUNT_YP_POCK, True)),
             (MOUNT_YP_POCK[1], FRAME_R_OUT + 1.0)),
            (270.0, ((MOUNT_YM_MORT, False), (MOUNT_YM_POCK, True)),
             (MOUNT_YM_MORT[1], FRAME_R_OUT + 1.0))):
        for (x0, x1), pocket in segs:
            c = (MOUNT_J.mortise(drop=MOUNT_PLATE_T + 0.5, pocket=pocket,
                                 length=x1 - x0)
                 .rotate((0, 0, 0), (1, 0, 0), 180.0)
                 .translate((x0, 0.0, FRAME_Z1 - MOUNT_PLATE_T)))
            cuts.append(c.rotate((0, 0, 0), (0, 0, 1), ang))
        cuts.append(_v_groove(*grv).rotate((0, 0, 0), (0, 0, 1), ang))
    return cuts


mount_bracket = _build_bracket()
