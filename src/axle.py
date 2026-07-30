"""AXLE — the proven original two-half axle (center mortise+tenon glue joint,
spring-strip slit, bearing shoulders), MIRRORED in z to match the flipped
spring housing and EXTENDED to this design's frame:

  * the halves swap roles under the mirror — the original design's collar/mortise half becomes
    this design's TOP half (its chamfered collar foot now seats the fused +Z bearing),
    the original design's tenon/lip half becomes this design's BOTTOM half (its lip now retains the
    removable cap's bearing from above);
  * the bottom end extends down flush with the bottom plus underside, the top
    end trims flush with the top plus top — both plus centres carry AXLE_BORE_D
    slip bores (see frame.py).

FIXING HARDWARE: the original design's M2 cross-pin is replaced by a printed PLASTIC ROD
(axle_pin) through a horizontal plan-45° bore (+xy → −xy) through the
frame_top crossing AND the axle at the crossing mid-height — inserted from
the open quadrant between the plus arms. The spring's constant radial load on
the axle friction-locks the rod; no thread needed.
"""

import math

import cadquery as cq

from src.axle_base import axle_bottom as _base_bottom, axle_top as _base_top
from src.dimensions import SPOOL_H, AXLE_PRINT_D
from src.helpers import cyl, heal
from cadkit.holes import teardrop_hole
from .params import (
    BOT_RIB_Z0, TOP_RIB_Z1, ROD_D, ROD_HOLE_D, ROD_L, ROD_Z, ROD_Z_BOT,
    HEX_DRIVE_AC, HEX_DRIVE_L, HEX_DRIVE_Z0,
    WIND_TOOL_CLR, WIND_TOOL_L, WIND_TOOL_W, WIND_TOOL_T,
    WIND_TOOL_BOSS_OD, WIND_TOOL_BOSS_H,
)

_S45 = 1.0 / math.sqrt(2.0)


def _flip_z(wp):
    """Mirror about z = SPOOL_H/2 — the same flip applied to the spring
    housing, so every axle shoulder lands on its (flipped) bearing."""
    return cq.Workplane(obj=wp.val().mirror("XY")).translate((0, 0, SPOOL_H))


def rod_hole(z, up=1.0):
    """A diagonal rod BORE: horizontal, plan-45° (+xy → −xy), through the axle
    at height `z` (ROD_Z at the top crossing, ROD_Z_BOT at the bottom one).
    frame.py cuts the SAME bores through the plus crossings so the holes line
    up by construction. Cut as a TEARDROP (cadkit.holes — a sideways round
    bore's ceiling sags); `up` is the cut part's PRINT direction in world z:
    +1 for the frames (print −Z→+Z), −1 for the axle halves (print FLIPPED)
    — each part's ceiling peak points its own print-up, while the round
    lower halves (where the rod actually bears) still line up exactly."""
    L = 60.0
    return teardrop_hole(ROD_HOLE_D, L,
                         (L / 2.0 * _S45, L / 2.0 * _S45, z),
                         (-_S45, -_S45, 0.0), (0.0, 0.0, up))


def _build_bottom():
    """BOTTOM half = original top half mirrored (tenon up, retention lip at the
    cap bearing), extended down through the bottom plus. Mirrored span was
    z −6..37; the plain-shaft stub grows to BOT_RIB_Z0, then continues as the
    PRE-WIND HEX DRIVE: a male hex inscribed in the shaft profile (bearing
    still slides over it) sticking HEX_DRIVE_L below the frame — turned with
    a 7 mm bit or the printed winding_tool to tension the spring after
    assembly, before the bottom rod pin locks the axle. In the flipped print
    the hex points up: a vertical prism, overhang-free."""
    b = _flip_z(_base_top)
    b = b.union(cyl(AXLE_PRINT_D, -5.0 - BOT_RIB_Z0, z=BOT_RIB_Z0))  # 1 mm overlap
    b = b.union(cq.Workplane("XY").workplane(offset=HEX_DRIVE_Z0)
                .polygon(6, HEX_DRIVE_AC).extrude(HEX_DRIVE_L + 1.0))  # 1 mm overlap
    b = b.cut(rod_hole(ROD_Z_BOT, up=-1.0))       # axle prints flipped
    return heal(b)


def _build_top():
    """TOP half = original bottom half mirrored (mortise collar down, its foot
    seating the fused bearing), trimmed flush with the top plus top and bored
    for the diagonal rod."""
    t = _flip_z(_base_bottom)                                 # spans 14..68
    t = t.cut(cyl(AXLE_PRINT_D + 4.0, 10.0, z=TOP_RIB_Z1))    # trim above 64.2
    t = t.cut(rod_hole(ROD_Z, up=-1.0))           # axle prints flipped
    return heal(t)


def _build_pin():
    """The plastic rod, modelled as a plain Ø ROD_D × ROD_L cylinder along Z
    (prints standing); build.py places it along the +xy→−xy diagonal."""
    return cyl(ROD_D, ROD_L, z=0)


def _build_winding_tool():
    """The printed WINDING TOOL for users without a 7 mm bit: two flat handle
    arms and a centre BOSS whose female hex socket runs the FULL hex-drive
    length for maximum grip — slip it over the hex drive and turn by the
    handle ends. Prints flat −Z→+Z: arms are a thin plate, the boss rises
    above them (vertical walls, overhang-free); the socket is a vertical
    through-hole. The arm plate is a STADIUM (slot) profile — full-radius
    rounded ends for a comfortable grip."""
    tool = (cq.Workplane("XY")
            .slot2D(WIND_TOOL_L, WIND_TOOL_W).extrude(WIND_TOOL_T)
            .union(cyl(WIND_TOOL_BOSS_OD, WIND_TOOL_BOSS_H, z=0)))
    socket = (cq.Workplane("XY").workplane(offset=-0.5)
              .polygon(6, HEX_DRIVE_AC + WIND_TOOL_CLR)
              .extrude(WIND_TOOL_BOSS_H + 1.0))
    return heal(tool.cut(socket))


axle_bottom = _build_bottom()
axle_top = _build_top()
axle_pin = _build_pin()
winding_tool = _build_winding_tool()


def pin_in_place(z=ROD_Z):
    """A rod placed as installed: along the plan-45° diagonal, centred on the
    axle at height `z` (for the assembly view — one at each crossing)."""
    return (axle_pin
            .translate((0, 0, -ROD_L / 2.0))                  # centre on origin
            .rotate((0, 0, 0), (0, 1, 0), 90)                 # along +X
            .rotate((0, 0, 0), (0, 0, 1), 45)                 # along +xy diagonal
            .translate((0, 0, z)))
