"""WALL — the containment ring around the separator/lid (WALL_SEP_CLR off
their OD, user's 1.6 / 2.4 thick). The BOTTOM band is no longer a part: it
exports as a solid that frame.py FUSES into frame_bottom (user's call — it
replaces the bottom plus as the beams' structural tie). The lever bays
open from the COIL CHAMBER's ceiling (CH_TOP_Z — below it the band runs a
CLOSED RING, the bottom rim + coil containment, user's call) up through
the split and the wall_top sector — the former derived sill (the
over-pull stop) is still gone; RATCHET_WIN_Z0 stays in params as the
lever's future stop-tab reference. Above the split, v2's sliding pieces
remain:

  wall_top  — prints +Z→−Z (INVERTED). The upper part of the ratchet
      window and the exit port — their world-CEILINGS are bed-side on its
      plate, so every opening closes with NO overhang, NO supports. Ends
      flat at WALL_Z1 = the highest opening + 3.2 (user's call; that flat
      top is its bed).
  wall_lock — ONE printed part, used FOUR times (replaces v2's collar,
      user's call): a flat 1.6 prism, beam-wide, carrying a T tenon —
      it slides down a beam channel ABOVE the seated wall_top and fills
      it to the frame_top underside plane, z-locking the stack. Prints
      standing (the T's plan profile as vertical walls, like the wall).

The split plane (WALL_SPLIT_Z) passes through the CABLE EXIT PORT at
(+x,+y): floor in the fused band, ceiling in wall_top. Every piece carries
its segment of the 4 T tenons (the fused band's double as its weld into
the beams), so all align in the same channels.

INSTALL: wall_top slides down the beam channels onto the fused band's top
face, then the four lock strips drop into the channels above it; frame_top
caps the channels (the strips end at its underside plane — the seated
stack can rise only JOINT_SEAT_CLR).
"""

import math

import cadquery as cq

from cadkit.joinery import joint

from .helpers import cyl, heal
from .params import (
    NOZZLE,
    WALL_IR, WALL_OR, WALL_SPLIT_Z, WALL_Z1, WALL_ZB,
    BEAM_IR, BEAM_SIZE, BEAM_Z1,
    JOINT_SPEC, JOINT_WIDTH, JOINT_DEPTH,
    LEVER_WIN_Y0, LEVER_WIN_Y1, FLOOR_Z1,
    CABLE_EXIT_ANGLE_DEG, CABLE_EXIT_SPAN_DEG, CABLE_EXIT_Z0, CABLE_EXIT_Z1,
    ENTRY_PORT_W, ENTRY_PORT_SILL, ENTRY_PORT_AZ_DEG,
    PERF_D, PERF_WEB, CH_TOP_Z,
)

_TEN_L = WALL_Z1 - WALL_ZB                        # band bottom → wall_top's top
_JOINT = joint(JOINT_WIDTH, _TEN_L, tenon=JOINT_SPEC, mortise=JOINT_SPEC,
               install="-z", depth=JOINT_DEPTH)   # wall slides DOWN to its stop


def _ring(id_, od, z, h):
    return cyl(od, h, z=z).cut(cyl(id_, h + 1.0, z=z - 0.5))


def _tenon(angle_deg):
    """T tenon (prism along Z = the install axis), head radially outward,
    root sunk into the wall's outer face; runs the FULL extended height —
    seated, the tenon tops sit JOINT_SEAT_CLR under frame_top (the
    stack's z-lock, v2)."""
    return (_JOINT.tenon(root=1.0, length=_TEN_L)
            .translate((WALL_OR, 0, WALL_ZB))
            .rotate((0, 0, 0), (0, 0, 1), angle_deg))


def _lever_window(y0, y1, z0, z1):
    """Window box through the wall at the +X lever azimuth. Windows flank
    the +X wall tenon, so the joinery survives."""
    return (cq.Workplane("XY").workplane(offset=z0)
            .polyline([(WALL_IR - 3.0, y0), (WALL_OR + 3.0, y0),
                       (WALL_OR + 3.0, y1), (WALL_IR - 3.0, y1)])
            .close().extrude(z1 - z0))


def _cable_exit():
    """The cable exit PORT — an arc wedge at (+x,+y) spanning the spool
    chamber's z-band; the span covers exit tangencies from the bare drum
    wall to design capacity (v2's math, params). The split plane passes
    through it: floor prints in the bottom half, ceiling in the inverted
    top half."""
    a0 = math.radians(CABLE_EXIT_ANGLE_DEG - CABLE_EXIT_SPAN_DEG / 2.0)
    a1 = math.radians(CABLE_EXIT_ANGLE_DEG + CABLE_EXIT_SPAN_DEG / 2.0)
    n = 12
    r_arc = (WALL_OR + 3.0) / math.cos((a1 - a0) / n / 2.0)
    pts = [(0.0, 0.0)]
    pts += [(r_arc * math.cos(a0 + (a1 - a0) * i / n),
             r_arc * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]
    return (cq.Workplane("XY").workplane(offset=CABLE_EXIT_Z0)
            .polyline(pts).close().extrude(CABLE_EXIT_Z1 - CABLE_EXIT_Z0))


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
    # ring + tenons both end at WALL_Z1 — the channel above is the lock
    # strips' territory
    w = _ring(2 * WALL_IR, 2 * WALL_OR, WALL_ZB, WALL_Z1 - WALL_ZB)
    for a in (0.0, 90.0, 180.0, 270.0):
        w = w.union(_tenon(a))
    # lever bays: from the COIL CHAMBER's ceiling up through the split
    # (user's call — the old floor-up cut severed the band's bottom rim
    # AND opened the chamber at both bays, where the loose coil could
    # bulge out; below CH_TOP_Z the band now runs a CLOSED RING again.
    # Every lever working part crosses the wall ABOVE it: the pawl band
    # bottom at −38.4, the brake contact frame above the −44.4 band —
    # posed-lever probes at the gate)…
    w = w.cut(_lever_window(LEVER_WIN_Y0, LEVER_WIN_Y1,
                            CH_TOP_Z, WALL_SPLIT_Z + 0.5))
    w = w.cut(_lever_window(-LEVER_WIN_Y1, -LEVER_WIN_Y0,
                            CH_TOP_Z, WALL_SPLIT_Z + 0.5))
    # …and the whole +x SECTOR of wall_top above them (user #834: the top
    # band over the bays deleted; the beam-strip between the bays would be
    # an island, so it goes too — wall_top is a C-ring on THREE tenons and
    # the +X channel carries no lock strip). The cutter reaches past the
    # +X tenon's HEAD (deeper than the window boxes — a WALL_OR+3 cutter
    # truncated it and stranded a floating sliver, probe-caught).
    w = w.cut(cq.Workplane("XY").workplane(offset=WALL_SPLIT_Z)
              .polyline([(WALL_IR - 3.0, -LEVER_WIN_Y1),
                         (WALL_OR + 8.0, -LEVER_WIN_Y1),
                         (WALL_OR + 8.0, LEVER_WIN_Y1),
                         (WALL_IR - 3.0, LEVER_WIN_Y1)])
              .close().extrude((WALL_Z1 + 5.0) - WALL_SPLIT_Z))
    w = w.cut(_cable_exit())
    w = w.cut(_entry_port())
    if WALL_PERF:                  # revertable — see the flag above
        w = w.cut(_perforation())
    return w


def _split(w):
    """Bottom band (→ fused into frame_bottom) + the sliding wall_top."""
    big = 400.0

    def band(z0, z1):
        return w.intersect(cq.Workplane("XY").workplane(offset=z0)
                           .rect(big, big).extrude(z1 - z0))

    return (band(WALL_ZB - 5.0, WALL_SPLIT_Z),
            heal(band(WALL_SPLIT_Z, WALL_Z1 + 5.0)))


wall_bottom_band, wall_top = _split(_build_wall_full())


# ── wall_lock — ONE part, printed 3× (90/180/270; replaces v2's collar,
# user's call — the +X channel lost its wall_top tenon with the open
# sector, so it carries no lock) ─────────────────────────────────────────────
_LOCK_T = 2 * NOZZLE                              # 1.6 — plate thickness
_LOCK_H = BEAM_Z1 - WALL_Z1                       # 20.2 — wall_top top → the
                                                  # frame_top underside plane
_LOCK_JOINT = joint(JOINT_WIDTH, _LOCK_H, tenon=JOINT_SPEC,
                    mortise=JOINT_SPEC, install="-z", depth=JOINT_DEPTH)


def _build_wall_lock():
    """Flat beam-wide prism + T tenon (modelled at the +X site; the
    assembly rotates four copies): drops down a beam channel onto
    wall_top's flat top and fills it to frame_top's underside — the
    stack's z-lock. Prints STANDING like the wall pieces (plan profile =
    vertical walls, no overhangs)."""
    plate = (cq.Workplane("XY").workplane(offset=WALL_Z1)
             .polyline([(BEAM_IR - _LOCK_T, -BEAM_SIZE / 2.0),
                        (BEAM_IR, -BEAM_SIZE / 2.0),
                        (BEAM_IR, BEAM_SIZE / 2.0),
                        (BEAM_IR - _LOCK_T, BEAM_SIZE / 2.0)])
             .close().extrude(_LOCK_H))
    ten = (_LOCK_JOINT.tenon(root=1.0, length=_LOCK_H)
           .translate((WALL_OR, 0, WALL_Z1)))
    return heal(plate.union(ten))


wall_lock = _build_wall_lock()
