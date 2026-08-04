"""AXLE — the proven original two-half axle (center mortise+tenon glue joint,
spring-strip slit, bearing shoulders), used in its AS-DESIGNED orientation
(the housing flip is gone — the cap is at +Z again, user's call) and
EXTENDED to this design's frame:

  * the BOTTOM half's chamfered collar foot seats the fused −Z bearing;
    the TOP half's lip retains the removable cap's bearing;
  * the bottom end extends down flush with the bottom plus underside (and
    carries the pre-wind hex drive), the top end extends flush with the
    top plus top — both plus centres carry AXLE_BORE_D slip bores (see
    frame.py).

FIXING HARDWARE: the original design's M2 cross-pin is replaced by a printed
PLASTIC ROD (axle_pin) through a horizontal plan-45° bore (+xy → −xy) through
the frame_top crossing AND the axle at the crossing mid-height — inserted from
the open quadrant between the plus arms. The spring's constant radial load on
the axle friction-locks the rod; no thread needed. The rod is an OCTAGON
(0.8 flats top/bottom/sides + maximal 45° diagonals), not round: it prints
LYING on a flat with every face printable — a tall skinny standing cylinder
prints badly — and the 180°-symmetric bore is self-supporting in EVERY
consumer regardless of print direction (frames and axle_bottom print −Z→+Z,
axle_top prints FLIPPED; user's calls).
"""

import math

import cadquery as cq

from v2.axle_base import axle_bottom as _base_bottom, axle_top as _base_top
from v2.dimensions import AXLE_PRINT_D
from v2.helpers import cone_solid, cyl, heal
from .params import (
    BOT_RIB_Z0, TOP_RIB_Z1, ROD_D, ROD_TIP_W, ROD_CLR, ROD_L, ROD_PRINT_H,
    ROD_Z, ROD_Z_BOT,
    DRIVE_SQ_S, DRIVE_L, DRIVE_Z0,
    WIND_TOOL_CLR, WIND_TOOL_L, WIND_TOOL_W, WIND_TOOL_T,
    WIND_TOOL_BOSS_OD, WIND_TOOL_BOSS_H,
    AXLE_RING_H, AXLE_RING_OD, AXLE_RING_BITE,
)


def _oct_pts(c):
    """The rod's OCTAGON cross-section — ROD_TIP_W dull flats at top, bottom
    and both sides, joined by maximal 45° diagonals — centred on the axis,
    grown by `c` PERPENDICULAR to every face, so the bore keeps a true c gap
    all around (a flat moves by c; each 45° diagonal's corner terms pick up
    c(√2−1))."""
    w = ROD_D / 2.0 + c                            # flat faces (all four)
    f = ROD_TIP_W / 2.0 + c * (math.sqrt(2.0) - 1.0)   # flat half-widths
    return [(w, -f), (w, f), (f, w), (-f, w),
            (-w, f), (-w, -f), (-f, -w), (f, -w)]


def _oct_prism(c, length):
    """The octagon profile extruded `length` along the plan-45° rod axis
    (+xy → −xy), centred on the axle axis at z=0."""
    return (cq.Workplane("XZ").polyline(_oct_pts(c)).close()
            .extrude(length)                       # XZ extrudes toward −y
            .translate((0.0, length / 2.0, 0.0))   # centre the span
            .rotate((0, 0, 0), (0, 0, 1), -45.0))  # +y → the +xy diagonal


def rod_hole(z):
    """A diagonal rod BORE: horizontal, plan-45° (+xy → −xy), through the axle
    at height `z` (ROD_Z at the top crossing, ROD_Z_BOT at the bottom one).
    frame.py cuts the SAME bores through the plus crossings so the holes line
    up by construction. Octagonal, ROD_CLR clear of the rod on every face;
    180°-symmetric, so the ceiling (45° diagonals + a ~1 mm flat bridge) is
    self-supporting whichever way the host prints — frames and axle_bottom
    build −Z→+Z, axle_top builds FLIPPED."""
    return _oct_prism(ROD_CLR, 60.0).translate((0.0, 0.0, z))


def _build_bottom():
    """BOTTOM half = the original collar/mortise half AS DESIGNED (its
    chamfered collar foot seats the fused −Z bearing), trimmed of the old
    design's deep tray journal and extended through the bottom plus as the
    PRE-WIND SQUARE DRIVE: a male square inscribed in the shaft profile
    (bearing still slides over it) sticking DRIVE_L below the frame —
    turned with the printed winding_tool to tension the spring after
    assembly, before the bottom rod pin locks the axle. Square, not hex
    (user: the hex's six short facets printed ROUND at this scale — four
    real flats grip). Prints upright (−z end on the bed): a vertical
    prism."""
    b = _base_bottom
    # the modular frame's bottom plus sits far BELOW the axle base's foot
    # now (the axial chamber dropped it): extend the shaft down to the plus
    # underside (1 mm overlap up into the base, whose foot is at z −17) —
    # the coil cup's collar steadies this span mid-way
    b = b.union(cyl(AXLE_PRINT_D, -16.0 - BOT_RIB_Z0, z=BOT_RIB_Z0))
    b = b.union(cq.Workplane("XY").workplane(offset=DRIVE_Z0)
                .rect(DRIVE_SQ_S, DRIVE_SQ_S).extrude(DRIVE_L + 1.0))  # 1 mm overlap
    # 45° junction cone drive → shaft: the round shaft's underside overhangs
    # the square's FLATS where it starts (only the corners touch the shaft
    # Ø). A cone from the square's inscribed circle (Ø = its side) up to the
    # shaft Ø, topping out at the shaft start, covers exactly those
    # crescents self-supportingly; it never exceeds the shaft Ø, so the
    # cap's bearing still slides over it at install.
    _ch = (AXLE_PRINT_D - DRIVE_SQ_S) / 2.0       # 45° rise ≈ 1.17
    b = b.union(cone_solid(d_bottom=DRIVE_SQ_S, d_top=AXLE_PRINT_D,
                           h=_ch, z_base=BOT_RIB_Z0 - _ch))
    b = b.cut(rod_hole(ROD_Z_BOT))
    return heal(b)


def _build_top():
    """TOP half = the original tenon/lip half (its lip cone seats the top
    bearing that carries the hanging housing), extended flush with the top
    plus top and bored for the diagonal rod. Prints FLIPPED — top end on
    the bed (a full Ø AXLE_PRINT_D disc, vs the slit-split tenon tip;
    user's call): the lip geometry in axle_base is mirrored to suit, and
    the flipped direction also un-bridges the slit's closed end."""
    t = _base_top                                             # spans 14..62
    t = t.union(cyl(AXLE_PRINT_D, TOP_RIB_Z1 - 61.0, z=61.0))  # 1 mm overlap
    t = t.cut(cyl(AXLE_PRINT_D + 4.0, 10.0, z=TOP_RIB_Z1))    # trim flush 64.2
    t = t.cut(rod_hole(ROD_Z))
    return heal(t)


def _build_pin():
    """The plastic rod: the c=0 octagon prism, exported PRINT-READY the way
    it actually prints (user's orientation): ROLLED 45° onto one of its
    LONG diagonal faces, so the print height is the across-diagonals span
    ROD_PRINT_H — a whole number of 0.8 steps (the octagon is sized from
    that grid; off-grid heights printed out-of-square)."""
    p = (cq.Workplane("XZ").polyline(_oct_pts(0.0)).close()
         .extrude(ROD_L)                            # spans y −ROD_L..0
         .translate((0.0, ROD_L / 2.0, 0.0))        # centre, axis on z=0
         .rotate((0, 0, 0), (0, 1, 0), 45.0)        # roll onto a long face
         .translate((0.0, 0.0, ROD_PRINT_H / 2.0)))  # bed at z=0
    return heal(p)


def _build_winding_tool():
    """The printed WINDING TOOL: two flat handle arms and a centre BOSS whose
    female SQUARE socket runs the FULL drive length for maximum grip — slip
    it over the square drive and turn by the handle ends. Prints flat
    −Z→+Z: arms are a thin plate, the boss rises above them (vertical
    walls, overhang-free); the socket is a vertical through-hole. The arm
    plate is a STADIUM (slot) profile — full-radius rounded ends for a
    comfortable grip."""
    tool = (cq.Workplane("XY")
            .slot2D(WIND_TOOL_L, WIND_TOOL_W).extrude(WIND_TOOL_T)
            .union(cyl(WIND_TOOL_BOSS_OD, WIND_TOOL_BOSS_H, z=0)))
    socket = (cq.Workplane("XY").workplane(offset=-0.5)
              .rect(DRIVE_SQ_S + WIND_TOOL_CLR, DRIVE_SQ_S + WIND_TOOL_CLR)
              .extrude(WIND_TOOL_BOSS_H + 1.0))
    return heal(tool.cut(socket))


def _build_cap_ring(bite):
    """CAP-RETENTION TPU GRIP RING: a solid 95A washer gripping the bottom
    axle shaft directly under the spring housing cap — the cap has 3.2 of
    room to creep −z before the bottom plus catches it; the ring pins it
    (user's call). Same philosophy as the hub grip rings: light bite
    (AXLE_RING_BITE) + layer-corrugation interlock, continuously
    z-adjustable, hardware-free; TPU stretches over the square drive's
    corners on the way up. bite=0 → the SEATED pose model (line-on-line on
    the shaft for the scan gate). Prints flat, local z 0..AXLE_RING_H."""
    return heal(cyl(AXLE_RING_OD, AXLE_RING_H, z=0)
                .cut(cyl(AXLE_PRINT_D - bite, AXLE_RING_H + 1.0, z=-0.5)))


def cap_ring_seated(cap_bot_z):
    """Pose model: bite=0, top face flush under the cap's bottom face."""
    return _build_cap_ring(0.0).translate((0.0, 0.0, cap_bot_z - AXLE_RING_H))


axle_bottom = _build_bottom()
axle_top = _build_top()
axle_pin = _build_pin()
winding_tool = _build_winding_tool()
cap_grip_ring_tpu = _build_cap_ring(AXLE_RING_BITE)   # the PRINT (free bore)


def pin_in_place(z=ROD_Z):
    """A rod placed as installed: along the plan-45° diagonal, centred on the
    axle at height `z` (for the assembly view — one at each crossing),
    un-rolled from the print pose so the flats sit cardinal like the bores."""
    return (axle_pin
            .translate((0.0, 0.0, -ROD_PRINT_H / 2.0))        # axis onto z=0
            .rotate((0, 0, 0), (0, 1, 0), -45.0)              # un-roll off the long face
            .rotate((0, 0, 0), (0, 0, 1), -45.0)              # +y → +xy diagonal
            .translate((0.0, 0.0, z)))
