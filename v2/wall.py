"""WALL — the spool/tray containment ring, sliding into the frame along Z
on cadkit dovetail joints (slide_joint install="z") — THREE STACKED PIECES:

  wall_bottom — prints −Z→+Z. Bottom SKIRT ring (rigidity), the brake
      window as an OPEN-TOP notch (capped by the top piece's underside)
      and the lower part of the ratchet window + cable exit port. (The
      tray-band LOCK BAND and source PORT left with the flat tray design.)
  wall_top    — prints +Z→−Z (INVERTED in the slicer). Carries the upper part
      of the ratchet window and the exit port — their world-CEILINGS are
      bed-side geometry on its build plate, so every opening closes with NO
      overhang and NO supports. Ends FLAT at WALL_COL_SPLIT (its bed) — the
      SOLID rim above the windows splits EVENLY with the collar (user).
  wall_collar — prints −Z→+Z. Its half of the solid rim
      (WALL_COL_SPLIT..WALL_RING_TOP) with the four tenon POSTS rising bare
      to the frame_top underside (user: no rim in the last 3.2 — only an
      upright print can produce ring-less posts, hence the third piece).

The lower split plane (WALL_SPLIT_Z) passes through the CABLE EXIT PORT at
the (+x,+y) azimuth: the port's floor lives in the bottom piece, its ceiling
in the top — stacked, they form the closed hole. Every piece carries its
segment of the 4 dovetail tenons, so all align in the same beam channels.

INSTALL: the pieces slide down the beam channels in order (bottom → top →
collar); frame_top later caps the channels — and LOCKS the stack: the
collar's tenon posts end at frame_top's underside plane, so the seated
stack can rise only JOINT_SEAT_CLR (user: it used to be free to slide up).
"""

import math

import cadquery as cq

from v2.dimensions import NOZZLE  # bead-multiple sizing unit

from v2.helpers import cyl, heal
from cadkit.joinery import PrintSpec, joint
from .params import (
    WALL_IR, WALL_OR, WALL_Z0, WALL_SPLIT_Z, WALL_COL_SPLIT,
    WALL_ZB, WALL_RING_TOP, WALL_TEN_TOP,
    JOINT_WIDTH, JOINT_DEPTH, JOINT_CLR,
    LEVER_WIN_Y0, LEVER_WIN_Y1, RATCHET_WIN_Z0, BRAKE_WIN_Z0, RATCHET_WIN_TOP,
    CABLE_EXIT_ANGLE_DEG, CABLE_EXIT_SPAN_DEG, CABLE_EXIT_Z0, CABLE_EXIT_Z1,
)

_UP = PrintSpec(nozzle=NOZZLE, material="PETG-GF", facing="up")
_TEN_L = WALL_TEN_TOP - WALL_ZB                   # skirt bottom → frame_top underside
_JOINT = joint(JOINT_WIDTH, _TEN_L, tenon=_UP, mortise=_UP,
               install="-z", depth=JOINT_DEPTH)   # wall slides DOWN to its stop


def _ring(id_, od, z, h):
    return cyl(od, h, z=z).cut(cyl(id_, h + 1.0, z=z - 0.5))


def _tenon(angle_deg):
    """Dovetail tenon (prism along Z = the install axis), head pointing radially
    outward, root sunk into the wall's outer face. Runs the ring's FULL
    extended height — skirt bottom (its flat underside is the SEAT on the
    channel stop, and wall_bottom's bed) ALL THE WAY to the frame_top
    underside (WALL_TEN_TOP): seated, the tenon tops sit JOINT_SEAT_CLR
    under frame_top — the z-lock (user: the wall had no upward retention)."""
    return (_JOINT.tenon(root=1.0, length=_TEN_L)
            .translate((WALL_OR, 0, WALL_ZB))       # mating plane = wall outer face
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


def _build_wall_full():
    # ring to WALL_RING_TOP only — the last 3.2 up to the frame_top
    # underside is BARE TENON POSTS (user: no rim up there)
    w = _ring(2 * WALL_IR, 2 * WALL_OR, WALL_ZB, WALL_RING_TOP - WALL_ZB)
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
    # (the source PORT and the tray cable LOCK BAND are gone with the flat
    # tray design — the axial coil chamber will bring its own port when it
    # lands; see params' service-loop note)
    return w


def _split(w):
    """THREE stacked pieces (see module docstring): bottom (upright) /
    top (INVERTED — its flat top face at WALL_Z1 is its bed) / collar
    (upright: plain ring + the bare tenon posts — the posts have no ring
    beside them, which no inverted print could produce)."""
    big = 400.0

    def band(z0, z1):
        return w.intersect(cq.Workplane("XY").workplane(offset=z0)
                           .rect(big, big).extrude(z1 - z0))

    return (heal(band(WALL_ZB - 5.0, WALL_SPLIT_Z)),
            heal(band(WALL_SPLIT_Z, WALL_COL_SPLIT)),
            heal(band(WALL_COL_SPLIT, WALL_TEN_TOP + 5.0)))


wall_bottom, wall_top, wall_collar = _split(_build_wall_full())
