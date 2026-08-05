"""MOUNT — desk/wall TWIST-LOCK RING (params block for the full story).

ONE continuous flush ring riding arc channels near the arms' ends: screw
it to the wood first (both screws on quadrant pads INSIDE the frame's
footprint), then offer the assembly up — the four hanging arc TENONS
pass through the open quadrants — and rotate to the angular stops. The
angular constants and the seating sense are the frame_top↔frame_bottom
arc joints' own (one muscle memory for both installs); rotation keeps
every mount member on its own circle, so nothing sweeps the centre.

Joint: the flat-top mushroom ARC (cadkit, THROUGH mortises — both hosts
print +z→−z; the cavities exit the arms' undersides over the open
bays). The ring's SPINE is exactly one stem wide, so the cavities' stem
slots host it with zero ledge; over each arm's stop-wall zone it rides
an annular V-bottom groove instead.

PRINTS +z→−z (top face on the bed): ring, pads and spine sit ON the
bed; the four arc tenons rise library-native (stem → 45° flare → flat
top as a plain last layer). The screw countersinks open at the world
underside — v2's validated screw geometry, re-posed for a world-frame
build.
"""

import math

import cadquery as cq

from cadkit.joinery import PrintSpec, joint

from .helpers import heal
from .params import (
    NOZZLE, FRAME_Z1, FRAME_RIB, FRAME_R_OUT,
    MOUNT_TEN_W, MOUNT_THRU_D, MOUNT_PLATE_T, MOUNT_MATE_Z,
    MOUNT_RING_R, MOUNT_SPINE_W, MOUNT_PAD_W, MOUNT_PAD_AZ,
    MOUNT_GRV_W, MOUNT_GRV_VERT, MOUNT_GRV_FLAT, MOUNT_GRV_DEPTH,
    TOP_JOINT_SEAT_CLR, TOP_STOP_WALL, TOP_ENTRY_OVER,
    WOOD_SCREW_SHAFT_D, WOOD_SCREW_HEAD_D, WOOD_SCREW_HEAD_H,
)

_SPEC = PrintSpec(nozzle=NOZZLE, material="PETG-GF", facing="up")
MOUNT_J = joint(MOUNT_TEN_W, None, tenon=_SPEC, mortise=_SPEC,
                install="-x", depth=MOUNT_THRU_D, through=True)

# angular constants at the ring radius (the frame arc joints' pattern)
_R = MOUNT_RING_R
_ARM_HALF_A = math.degrees(math.asin((FRAME_RIB / 2.0) / _R))    # ≈ 4.19°
_SEAT_A     = math.degrees(TOP_JOINT_SEAT_CLR / _R)
_STOP_A     = math.degrees(TOP_STOP_WALL / _R)
_OVER_A     = math.degrees(TOP_ENTRY_OVER / _R)
_TEN_HALF_A = _ARM_HALF_A - _STOP_A - _SEAT_A                    # ≈ 2.77°
assert _TEN_HALF_A >= math.degrees(2 * NOZZLE / _R) - 1e-9, \
    "mount arc tenon thinner than a quality-tier arc at the ring radius"

_SITES = (0.0, 90.0, 180.0, 270.0)


def _hang(w):
    """Library-native arc solid (bulb up, mating plane z=0) → hanging
    from the flush spine: z-mirror (plan angles preserved), mating plane
    to MOUNT_MATE_Z."""
    return w.mirror("XY").translate((0.0, 0.0, MOUNT_MATE_Z))


def _screw_cut(az_deg):
    """One wood-screw cutter at the ring radius, azimuth az (world pose):
    Ø4 shaft through, countersink cone opening at the pad's UNDERSIDE —
    the head sits flush there, driven up into the wood. v2's validated
    geometry, re-posed."""
    x = _R * math.cos(math.radians(az_deg))
    y = _R * math.sin(math.radians(az_deg))
    z0 = FRAME_Z1 - MOUNT_PLATE_T
    slope = (WOOD_SCREW_HEAD_D - WOOD_SCREW_SHAFT_D) / 2.0 / WOOD_SCREW_HEAD_H
    over = 0.5
    shaft = cq.Solid.makeCylinder(
        WOOD_SCREW_SHAFT_D / 2.0, MOUNT_PLATE_T + 1.0,
        cq.Vector(x, y, z0 - 0.5), cq.Vector(0, 0, 1))
    cone = cq.Solid.makeCone(
        WOOD_SCREW_HEAD_D / 2.0 + over * slope,
        WOOD_SCREW_SHAFT_D / 2.0,
        WOOD_SCREW_HEAD_H + over,
        cq.Vector(x, y, z0 - over), cq.Vector(0, 0, 1))
    return cq.Workplane(obj=shaft).union(cq.Workplane(obj=cone))


def _annulus(r0, r1, z0, z1):
    return (cq.Workplane("XY").workplane(offset=z0)
            .circle(r1).circle(r0).extrude(z1 - z0))


def _build_ring():
    hw = MOUNT_SPINE_W / 2.0
    ring = _annulus(_R - hw, _R + hw, MOUNT_MATE_Z, FRAME_Z1)
    # the four hanging arc tenons, SEATED pose (centred on the arms)
    for site in _SITES:
        ring = ring.union(
            _hang(MOUNT_J.tenon_arc(_R, 2.0 * _TEN_HALF_A, root=2.0))
            .rotate((0, 0, 0), (0, 0, 1), site - _TEN_HALF_A))
    # quadrant screw pads (edges aligned radially), screws at the ring line
    for az in MOUNT_PAD_AZ:
        x = _R * math.cos(math.radians(az))
        y = _R * math.sin(math.radians(az))
        pad = (cq.Workplane("XY").workplane(offset=MOUNT_MATE_Z)
               .rect(MOUNT_PAD_W, MOUNT_PAD_W)
               .extrude(MOUNT_PLATE_T)
               .rotate((0, 0, 0), (0, 0, 1), az)
               .translate((x, y, 0.0)))
        ring = ring.union(pad).cut(_screw_cut(az))
    return heal(ring)


def _v_groove_arc(a0, a1):
    """Annular V-bottom groove segment for the spine (profile in the r-z
    plane, revolved a0→a1): vertical walls MOUNT_GRV_VERT deep, then the
    45° V to the dulled flat — the groove's −z boundary is the roof of
    the slot in the frame's +z→−z print."""
    hw = MOUNT_GRV_W / 2.0
    hf = MOUNT_GRV_FLAT / 2.0
    z1 = FRAME_Z1 + 1.0
    zv = FRAME_Z1 - MOUNT_GRV_VERT
    zb = FRAME_Z1 - MOUNT_GRV_DEPTH
    return (cq.Workplane("XZ")
            .polyline([(_R - hw, z1), (_R + hw, z1), (_R + hw, zv),
                       (_R + hf, zb), (_R - hf, zb), (_R - hw, zv)])
            .close().revolve(a1 - a0, (0, 0), (0, 1))
            .rotate((0, 0, 0), (0, 0, 1), a0))


def mount_channel_cuts():
    """frame_top's cutters, one pair per arm: the retained arc CAVITY
    (open past the arm's CCW face — the entry; its CW end, TOP_STOP_WALL
    inside the arm's CW face, is the stop — the frame arc joints'
    arrangement verbatim, same seating rotation) plus the spine's
    V-groove over the stop-wall zone so the ring passes at every install
    angle."""
    cuts = []
    cav_sweep = (_TEN_HALF_A + _SEAT_A) + (_ARM_HALF_A + _OVER_A)
    for site in _SITES:
        cav = (_hang(MOUNT_J.mortise_arc(_R, cav_sweep,
                                         drop=MOUNT_PLATE_T + 0.5))
               .rotate((0, 0, 0), (0, 0, 1), site - _TEN_HALF_A - _SEAT_A))
        cuts.append(cav)
        cuts.append(_v_groove_arc(site - _ARM_HALF_A - _OVER_A,
                                  site - _TEN_HALF_A - _SEAT_A + 0.1))
    return cuts


mount_ring = _build_ring()
