"""WALL — the containment ring around the separator/lid (WALL_SEP_CLR off
their OD, user's 1.6 / 2.4 thick). ONE FUSED BAND now, bed → the LID'S
TOP (user's call: wall_top, the four lock strips and the beam T channels
are all RETIRED — frame.py fuses this band into frame_bottom and nothing
slides). Everything that used to close in wall_top's inverted print
closes with 45° geometry in the band's upright print instead:

  · lever bay windows — FULL-HEIGHT slots now, bed → THROUGH the band's
    top rim (user's call: the 45° gable crown between the levers and
    the lid was an overhang — remove it; the band's arcs are carried by
    the bay webs / fork columns instead of a closed ring);
  · cable EXIT port — NO ceiling either (user's call: the sawtooth roof
    was an unnecessary overhang — the port cuts open through the top
    rim over its ~30° span);
  · entry port — its house gable, as always (a hole mid-band still
    needs a printable roof).

The former derived sill (the over-pull stop) is still gone;
RATCHET_WIN_Z0 stays in params as the lever's future stop-tab reference.
"""

import math

import cadquery as cq

from .helpers import chord_x, cyl
from .params import (
    WALL_IR, WALL_OR, WALL_Z1, WALL_ZB,
    LEVER_WIN_Y0, LEVER_WIN_Y1, FLOOR_Z0,
    CABLE_EXIT_Y_LO, CABLE_EXIT_Y_HI, CABLE_EXIT_Z0,
    ENTRY_PORT_W, ENTRY_PORT_SILL, ENTRY_PORT_AZ_DEG,
)


def _ring(id_, od, z, h):
    return cyl(od, h, z=z).cut(cyl(id_, h + 1.0, z=z - 0.5))


def _lever_window(y0, y1, z0):
    """Bay window through the wall at the +X lever azimuth: a full-width,
    FULL-HEIGHT slot, open through the band's top rim (user's call: the
    old 45° gable crown between the levers and the lid was an overhang
    — gone; hand access and the lever swing own the whole bay now)."""
    return (cq.Workplane("YZ")
            .polyline([(y0, z0), (y1, z0),
                       (y1, WALL_Z1 + 1.0), (y0, WALL_Z1 + 1.0)])
            .close().extrude((WALL_OR + 3.0) - (WALL_IR - 3.0))
            .translate((WALL_IR - 3.0, 0, 0)))


def _cable_exit():
    """The cable exit WINDOW — a straight Y-SLAB cut through the band's
    +x side (user #893: the window is the Y-BAND the cable can take up
    between the coil and the horn's guide mouth — swept in params over
    the wind range — cut flat between those two y planes), from the
    spool chamber's floor OPEN THROUGH THE BAND'S TOP RIM (no roof, no
    overhang; the old tangential 30° arc is retired)."""
    # the cutter's x-start tracks the band's chord at the window's far-y
    # edge (the #890 membrane lesson: a fixed start strands a sliver
    # where the ring pulls inboard of it)
    x0 = chord_x(WALL_IR, CABLE_EXIT_Y_HI) - 1.0
    return (cq.Workplane("XY").workplane(offset=CABLE_EXIT_Z0)
            .polyline([(x0, CABLE_EXIT_Y_LO),
                       (WALL_OR + 3.0, CABLE_EXIT_Y_LO),
                       (WALL_OR + 3.0, CABLE_EXIT_Y_HI),
                       (x0, CABLE_EXIT_Y_HI)])
            .close().extrude((WALL_Z1 + 1.0) - CABLE_EXIT_Z0))


def _entry_port():
    """HOUSE-shaped cable ENTRY (v2's size: 13 sq + 45° gable — the gable
    prints support-free in the upright band): the SOURCE cable enters the
    coil chamber here, at its bottom edge, as close to −x as the 180°
    beam allows (azimuth derived in params)."""
    w = ENTRY_PORT_W
    s = ENTRY_PORT_SILL
    pts = [(-w / 2.0, s), (w / 2.0, s), (w / 2.0, s + w),
           (0.0, s + 1.5 * w), (-w / 2.0, s + w)]
    return (cq.Workplane("YZ").polyline(pts).close()
            .extrude((WALL_OR + 3.0) - 60.0)
            .translate((60.0, 0.0, 0.0))
            .rotate((0, 0, 0), (0, 0, 1), ENTRY_PORT_AZ_DEG))


# (the 45° REST LIP is GONE — user's redesign: the separator now rides a
# second 608 on the frame floor's stub axle; see frame.py/axle.py.
# The diamond PERFORATION of the band is gone too — user #932: it sat
# behind a build-speed flag through the whole v3 iteration and never
# got used; the band ships solid.)


def _build_wall_full():
    # the one band: bed → the lid's top plane
    w = _ring(2 * WALL_IR, 2 * WALL_OR, WALL_ZB, WALL_Z1 - WALL_ZB)
    # lever bays: FULL-HEIGHT slots, bed → through the top rim (user's
    # calls: no band −z of the levers — hand access wins; windows cut
    # from below the band's bottom face, the #847 lesson; and no crown
    # ABOVE them either — the gable ring between the levers and the lid
    # was an overhang, removed)
    w = w.cut(_lever_window(LEVER_WIN_Y0, LEVER_WIN_Y1, FLOOR_Z0 - 0.5))
    w = w.cut(_lever_window(-LEVER_WIN_Y1, -LEVER_WIN_Y0, FLOOR_Z0 - 0.5))
    w = w.cut(_cable_exit())
    w = w.cut(_entry_port())
    # NO heal here: the band's only consumer is frame_bottom, which runs
    # its own boolean chain over it and heals once at the end — a full
    # ShapeFix/unify pass on the bare band is thrown-away work (#932
    # efficiency review)
    return w


# ONE band — fused into frame_bottom (wall_top, the lock strips and the
# beam T channels are RETIRED, user's call; nothing slides anymore)
wall_band = _build_wall_full()
