"""WALL — the spool/tray containment ring, sliding into the frame along Z
on cadkit dovetail joints (slide_joint install="z") — now TWO STACKED HALVES:

  wall_bottom — prints −Z→+Z. Carries the brake window as an OPEN-TOP notch
      (capped by the top half's underside) and the lower part of the ratchet
      window + cable exit port.
  wall_top    — prints +Z→−Z (INVERTED in the slicer). Carries the upper part
      of the ratchet window and the exit port — their world-CEILINGS are
      bed-side geometry on its build plate, so every opening closes with NO
      overhang and NO supports. Its top band (above the ratchet window's
      41.5 ceiling) is a solid UNBROKEN ring.

The split plane (WALL_SPLIT_Z) passes through the CABLE EXIT PORT at the
(+x,+y) azimuth: the port's floor lives in the bottom half, its ceiling in the
top half — stacked, they form the closed hole. Each half carries its segment
of the 4 dovetail tenons, so both align in the same beam channels.

INSTALL: bottom half slides down the beam channels to the stops; top half
slides down and stacks on it; frame_top later caps the channels.
"""

import math

import cadquery as cq

from src.helpers import cyl, heal
from cadkit.joinery import PrintSpec, slide_joint
from .params import (
    WALL_IR, WALL_OR, WALL_H, WALL_Z0, WALL_Z1, WALL_SPLIT_Z,
    JOINT_WIDTH, JOINT_DEPTH, JOINT_CLR,
    LEVER_WIN_Y0, LEVER_WIN_Y1, RATCHET_WIN_Z0, BRAKE_WIN_Z0, RATCHET_WIN_TOP,
    CABLE_EXIT_ANGLE_DEG, CABLE_EXIT_SPAN_DEG, CABLE_EXIT_Z0, CABLE_EXIT_Z1,
    SOURCE_PORT_W, SOURCE_PORT_ANGLE_DEG, SOURCE_PORT_Z0,
    SOURCE_PORT_RECT_TOP, SOURCE_PORT_APEX_Z,
)

_UP = PrintSpec(nozzle=0.8, material="PETG-GF", facing="up")
_JOINT = slide_joint(JOINT_WIDTH, WALL_H, tenon=_UP, mortise=_UP,
                     install="z", depth=JOINT_DEPTH)


def _ring(id_, od, z, h):
    return cyl(od, h, z=z).cut(cyl(id_, h + 1.0, z=z - 0.5))


def _tenon(angle_deg):
    """Dovetail tenon (prism along Z = the install axis), head pointing radially
    outward, root sunk into the wall's outer face. Runs the full wall height
    (split later along with the ring)."""
    return (_JOINT.tenon(root=1.0)
            .translate((WALL_OR, 0, WALL_Z0))       # mating plane = wall outer face
            .rotate((0, 0, 0), (0, 0, 1), angle_deg))


def _lever_window(y0, y1, z0, z1):
    """Window box through the wall at the +X lever azimuth, z0..z1. Windows
    flank the +X wall tenon (y ±3.5), so the joinery survives."""
    return (cq.Workplane("XY").workplane(offset=z0)
            .polyline([(WALL_IR - 3.0, y0), (WALL_OR + 3.0, y0),
                       (WALL_OR + 3.0, y1), (WALL_IR - 3.0, y1)])
            .close().extrude(z1 - z0))


def _cable_exit():
    """The cable exit PORT — an ARC WEDGE in the (+x,+y) quadrant spanning
    CABLE_EXIT_Z0..Z1 (the spool chamber's z-band). The cable leaves tangent
    to the coil's current outer wrap, so its wall-crossing point sweeps
    ~asin(r_wrap / WALL_IR) as the spool winds/unwinds (the original design's cable_retainer
    math); CABLE_EXIT_SPAN_DEG covers tangencies from the bare hub out to
    design capacity, centered on CABLE_EXIT_ANGLE_DEG. The wall SPLIT plane
    passes through it: floor prints in the bottom half, ceiling in the
    (inverted) top half. Cutter = origin-fan polygon with chord-overshot arc
    (only the wall ring is cut, so the fan's interior is harmless)."""
    a0 = math.radians(CABLE_EXIT_ANGLE_DEG - CABLE_EXIT_SPAN_DEG / 2)
    a1 = math.radians(CABLE_EXIT_ANGLE_DEG + CABLE_EXIT_SPAN_DEG / 2)
    n = 12
    r_arc = (WALL_OR + 3.0) / math.cos((a1 - a0) / n / 2)
    pts = [(0.0, 0.0)]
    pts += [(r_arc * math.cos(a0 + (a1 - a0) * i / n),
             r_arc * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]
    return (cq.Workplane("XY").workplane(offset=CABLE_EXIT_Z0)
            .polyline(pts).close().extrude(CABLE_EXIT_Z1 - CABLE_EXIT_Z0))


def _source_port():
    """HOUSE-shaped through hole (square sides + 45° gable roof) near −X:
    the stationary source lead leaves the tray chamber here, offset off y=0
    so the cable clears the −X beam's flank. Entirely below the split plane
    (apex 0.5 under it), so it lives in wall_bottom — where the gable ceiling
    prints support-free −Z→+Z. Built at +X as a radial prism, then rotated to
    SOURCE_PORT_ANGLE_DEG."""
    w = SOURCE_PORT_W
    return (cq.Workplane("YZ").workplane(offset=WALL_IR - 3.0)
            .polyline([(-w / 2, SOURCE_PORT_Z0), (w / 2, SOURCE_PORT_Z0),
                       (w / 2, SOURCE_PORT_RECT_TOP), (0.0, SOURCE_PORT_APEX_Z),
                       (-w / 2, SOURCE_PORT_RECT_TOP)])
            .close().extrude((WALL_OR + 3.0) - (WALL_IR - 3.0))
            .rotate((0, 0, 0), (0, 0, 1), SOURCE_PORT_ANGLE_DEG))


def _build_wall_full():
    w = _ring(2 * WALL_IR, 2 * WALL_OR, WALL_Z0, WALL_H)
    for a in (0.0, 90.0, 180.0, 270.0):
        w = w.union(_tenon(a))
    # ratchet window: CLOSED at RATCHET_WIN_TOP (swept boss + clearance) — the
    # ceiling lives in the top half, whose inverted print handles it; the
    # unbroken WALL_TOP_BAND ring runs above it
    w = w.cut(_lever_window(LEVER_WIN_Y0, LEVER_WIN_Y1,
                            RATCHET_WIN_Z0, RATCHET_WIN_TOP))
    # brake window: open-top notch in the bottom half, capped by the top
    # half's flat underside at the split plane
    w = w.cut(_lever_window(-LEVER_WIN_Y1, -LEVER_WIN_Y0,
                            BRAKE_WIN_Z0, WALL_SPLIT_Z))
    w = w.cut(_cable_exit())
    # house-shaped source-cable port (tray chamber → outside, near −X)
    w = w.cut(_source_port())
    return w


def _split(w):
    big = 400.0
    bot = w.intersect(cq.Workplane("XY").workplane(offset=WALL_Z0 - 5.0)
                      .rect(big, big).extrude(WALL_SPLIT_Z - (WALL_Z0 - 5.0)))
    top = w.intersect(cq.Workplane("XY").workplane(offset=WALL_SPLIT_Z)
                      .rect(big, big).extrude((WALL_Z1 + 5.0) - WALL_SPLIT_Z))
    return heal(bot), heal(top)


wall_bottom, wall_top = _split(_build_wall_full())
