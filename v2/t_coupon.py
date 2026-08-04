"""T-JOINT PRINT-FIT COUPON — the PRODUCTION wall↔beam joint at full
assembly lengths (user's call: this validates the real thing, not the rig):
slide the tenon part down the mortise part exactly as the wall installs.

  · test_t_mortise — a BEAM segment: the real 10×10 section carrying the
    real mortise channel at its production z-extent (hard stop at
    WALL_Z0 − JOINT_SEAT_CLR, running open past the beam top — the full
    MORTISE_L ≈ 41.55 slide), with a 3.2 solid floor under the stop.
    Prints −Z→+Z standing, exactly like the beam.

  · test_t_tenon — a WALL arc (the real WALL_T ring band, WALL_H tall,
    ±12° about the +X azimuth) carrying ONE full-height tenon. The
    production tenon is this same column split across the two stacked wall
    halves; the coupon keeps it whole so the seated engagement length (and
    its GF friction) matches the assembled stack. A 1.6-tall inward foot
    flange stabilises the standing print; it points AWAY from the beam
    side, so it stays clear through the whole slide and at seat.

Both parts print −Z→+Z. Fit target (cadkit policy, PETG-GF both halves):
lateral 0.15, depth faces 0.3 — the joint should slide freely and stop
hard on the channel floor.
"""

import cadquery as cq

from v2.dimensions import NOZZLE  # bead-multiple sizing unit

from v2.helpers import heal
from .frame import _wall_mortise
from .wall import _tenon
from .params import (
    BEAM_IR, BEAM_SIZE, BEAM_Z1, WALL_IR, WALL_OR, WALL_H, WALL_Z0,
    JOINT_SEAT_CLR,
)

_SW     = 12.0                              # tenon-part arc half-sweep
_Z_BASE = WALL_Z0 - JOINT_SEAT_CLR - 4 * NOZZLE    # 8.65 — 3.2 solid floor under the stop
_FOOT_W = 6.0                               # inward foot flange, radial


def _build_mortise():
    """The real beam section over the channel's production z-range, real
    mortise cut (frame.py's own cutter at the +X site)."""
    b = (cq.Workplane("XY").workplane(offset=_Z_BASE)
         .center(BEAM_IR + BEAM_SIZE / 2.0, 0.0)
         .rect(BEAM_SIZE, BEAM_SIZE)
         .extrude(BEAM_Z1 - _Z_BASE))
    return heal(b.cut(_wall_mortise(0.0)))


def _arc_band(r0, r1, z0, z1):
    return (cq.Workplane("XZ")
            .polyline([(r0, z0), (r1, z0), (r1, z1), (r0, z1)])
            .close()
            .revolve(2.0 * _SW, (0, 0), (0, 1))
            .rotate((0, 0, 0), (0, 0, 1), -_SW))


def _build_tenon():
    """A production wall arc with the real full-height tenon (wall.py's own
    builder at the +X site) + the inward print foot."""
    arc = _arc_band(WALL_IR, WALL_OR, WALL_Z0, WALL_Z0 + WALL_H)
    foot = _arc_band(WALL_IR - _FOOT_W, WALL_IR, WALL_Z0, WALL_Z0 + 2 * NOZZLE)
    return heal(arc.union(foot).union(_tenon(0.0)))


t_test_mortise = _build_mortise()
t_test_tenon   = _build_tenon()
