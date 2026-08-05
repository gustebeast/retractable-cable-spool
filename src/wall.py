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

from .helpers import cyl, heal
from .params import (
    WALL_IR, WALL_OR, WALL_Z1, WALL_ZB,
    LEVER_WIN_Y0, LEVER_WIN_Y1, FLOOR_Z0, FLOOR_Z1,
    CABLE_EXIT_ANGLE_DEG, CABLE_EXIT_SPAN_DEG, CABLE_EXIT_Z0,
    ENTRY_PORT_W, ENTRY_PORT_SILL, ENTRY_PORT_AZ_DEG,
    PERF_D, PERF_WEB, CH_TOP_Z,
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
    """The cable exit PORT — an arc wedge at (+x,+y) from the spool
    chamber's floor OPEN THROUGH THE BAND'S TOP RIM (user's call: the
    old sawtooth ceiling was an unnecessary overhang — nothing needs the
    crown closed over this span, so the port has no roof at all and
    prints trivially). The span covers exit tangencies from the bare
    drum wall to design capacity (v2's math, params)."""
    a0 = math.radians(CABLE_EXIT_ANGLE_DEG - CABLE_EXIT_SPAN_DEG / 2.0)
    a1 = math.radians(CABLE_EXIT_ANGLE_DEG + CABLE_EXIT_SPAN_DEG / 2.0)
    n = 12
    r_arc = (WALL_OR + 3.0) / math.cos((a1 - a0) / n / 2.0)
    pts = [(0.0, 0.0)]
    pts += [(r_arc * math.cos(a0 + (a1 - a0) * i / n),
             r_arc * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]
    return (cq.Workplane("XY").workplane(offset=CABLE_EXIT_Z0)
            .polyline(pts).close()
            .extrude((WALL_Z1 + 1.0) - CABLE_EXIT_Z0))


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
# second 608 on the frame floor's stub axle; see frame.py/axle.py.)

# REVERTABLE build-speed flag (user's call): the ~150-diamond cut costs
# real build/scan time — keep False while iterating, flip TRUE before any
# real print/export of frame_bottom (the perforation is part of the
# shipped design).
WALL_PERF = False


def _perforation():
    """Diamond perforation cutter (see params PERF_*): checker-tiled 45°
    diamonds through the COIL-CHAMBER band only (user trimmed the disk-
    band row, #828), solid PERF_WEB margins at floor and ceiling, the
    entry port's sector skipped. Beam sectors kept solid WIDE — no hole
    adjacent to a beam (user's screenshot): ±27° at +x, ±12.5° at the
    other three. Built as ONE compound so the band takes a single cut."""
    d2 = PERF_D / 2.0
    r_mid = (WALL_IR + WALL_OR) / 2.0
    n_az = int((2.0 * math.pi * r_mid) // (PERF_D + PERF_WEB))   # 51
    pitch = 360.0 / n_az
    row_dz = (PERF_D + PERF_WEB) / 2.0             # checker stagger
    proto = (cq.Workplane("YZ")
             .polyline([(0.0, -d2), (d2, 0.0), (0.0, d2), (-d2, 0.0)])
             .close().extrude(16.0).translate((60.0, 0.0, 0.0)))
    keep = [(0.0, 27.0), (90.0, 12.5), (180.0, 12.5), (270.0, 12.5)]

    def rows(z0, z1):
        out, z = [], z0 + d2
        while z + d2 <= z1 + 1e-9:
            out.append(z)
            z += row_dz
        return out

    zones = (
        # coil chamber only — the loose coil rests/slides here: sag-safe
        # size, entry-port sector solid
        (FLOOR_Z1 + PERF_WEB, CH_TOP_Z - PERF_WEB,
         keep + [(ENTRY_PORT_AZ_DEG, 9.0)]),
    )
    solids = []
    for z0, z1, ko in zones:
        for ri, zc in enumerate(rows(z0, z1)):
            off = pitch / 2.0 if ri % 2 else 0.0
            for k in range(n_az):
                az = (k * pitch + off) % 360.0
                if any(min(abs(az - a), 360.0 - abs(az - a)) < w
                       for a, w in ko):
                    continue
                solids.append(
                    proto.translate((0.0, 0.0, zc))
                    .rotate((0, 0, 0), (0, 0, 1), az).val())
    return cq.Workplane("XY").add(cq.Compound.makeCompound(solids))


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
    if WALL_PERF:                  # revertable — see the flag above
        w = w.cut(_perforation())
    return heal(w)


# ONE band — fused into frame_bottom (wall_top, the lock strips and the
# beam T channels are RETIRED, user's call; nothing slides anymore)
wall_band = _build_wall_full()
