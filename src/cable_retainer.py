"""Fixed cable-retention cage — keeps the wound cable from bulging out of
its channel (radially, in X/Y) when it goes slack.

Unlike the spool's own rims, this part must NOT rotate with the spool: the
cable enters the channel at a point fixed relative to the housing, so the
retainer is housing-attached and stationary while the spool turns inside it.

This module models ONLY the functional portion — a stationary cage at the
channel's outer boundary: two thin rings joined by vertical bars, leaving
sub-cable air gaps (< CABLE_D) so a slack coil can't escape between them.
It floats at its working position (coaxial with the spool, in the cable-
channel Z-band) with ~1 mm axial clearance from each rotating spool rim.
Attachment to the housing comes later.

Geometry knobs (tune freely — the position is a best read of the current
channel, z = RIM_H .. RIM_H+CABLE_D, outer boundary ≈ RIM_ID/2):
"""

import math

import cadquery as cq

from .spool import RIM_ID, RIM_H, CABLE_D
from .helpers import cyl


RETAIN_RIM_CLR = 1.0                       # axial clearance from each spool rim
RETAIN_R_IN    = RIM_ID / 2                # inner face ≈ the cable's outer boundary
RETAIN_WALL    = 2.0                       # radial thickness of the cage
RETAIN_R_OUT   = RETAIN_R_IN + RETAIN_WALL

_CHANNEL_LO    = RIM_H                      # bottom rim top (cable-channel floor)
_CHANNEL_HI    = RIM_H + CABLE_D            # top rim bottom (cable-channel ceiling)
RETAIN_Z_LO    = _CHANNEL_LO + RETAIN_RIM_CLR
RETAIN_Z_HI    = _CHANNEL_HI - RETAIN_RIM_CLR
RETAIN_H       = RETAIN_Z_HI - RETAIN_Z_LO

RETAIN_RING_T  = 1.0                        # solid ring at the top & bottom of the cage
RETAIN_BAR_W   = 2.0                        # tangential width of each vertical bar
# Bar count so the air gap at the inner face stays below the cable diameter.
RETAIN_N_BARS  = math.ceil(2 * math.pi * RETAIN_R_IN / (CABLE_D + RETAIN_BAR_W))


def _build_cable_retainer():
    # Top & bottom rings tie the bars together.
    def _ring(z_lo):
        return (cyl(2 * RETAIN_R_OUT, RETAIN_RING_T, z=z_lo)
                .cut(cyl(2 * RETAIN_R_IN, RETAIN_RING_T, z=z_lo)))
    cage = _ring(RETAIN_Z_LO).union(_ring(RETAIN_Z_HI - RETAIN_RING_T))

    # Vertical bars spanning the full cage height between the rings.
    r_mid = (RETAIN_R_IN + RETAIN_R_OUT) / 2
    for i in range(RETAIN_N_BARS):
        bar = (
            cq.Workplane("XY").workplane(offset=RETAIN_Z_LO)
            .center(r_mid, 0)
            .box(RETAIN_WALL, RETAIN_BAR_W, RETAIN_H, centered=(True, True, False))
            .rotate((0, 0, 0), (0, 0, 1), i * 360.0 / RETAIN_N_BARS)
        )
        cage = cage.union(bar)
    return cage


cable_retainer = _build_cable_retainer()
