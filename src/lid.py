"""LID — the spool chamber's ceiling, locked onto the drum wall's top rim
by the rotational bayonet (lid_joint.py): four hanging half-arrowhead arc
tenons drop through the wall-top entry pockets in z, then the lid rotates
CW to seat them in the retained cavities.

BODY: v2 cable_ceiling's pattern — an annulus (bore = the drum/wall bore,
rim at the separator OD) with graded solid rings tied by radial spokes,
arc-slot windows between (see the cable at install, save material; every
web a bead multiple):
  · inner band bore→DRUM_OR — sits exactly over the wall, carries the
    tenon ring and its roots;
  · LID_MID_W tie band centered mid-span;
  · 6-bead solid outer rim to the OD (unbacked, v2's grade).

PRINTS INVERTED (top face on the bed): the hanging tenons then stand in
v2's library-native orientation — flare rising 45°, taper to the dull
tip, support-free.

INSTALL: thread the lid up from below EARLY (its Ø86.7 bore passes the
gate wall and insert, but NOT the Ø99.5 drum wall — and the Ø145.8 rim
can't pass the frame from above): bearing → axle_top → LID parked around
the gate wall → spring → strip → gate insert → axle_separator rises from
below (glue joint engages, wall top stops just under the parked lid) →
lower the lid, tenons through the pockets, rotate CW to the stops.
"""

import math

import cadquery as cq

from .helpers import cyl, heal
from .lid_joint import lid_tenon
from .params import (
    LID_OD, LID_T, LID_Z0, LID_Z1, DRUM_IR,
    LID_SPOKE_N, LID_SPOKE_W, LID_RING_IN_R1, LID_RIM_R0,
    LID_MID_W, LID_MID_RC, LIDJ_SITES,
)


def _win_slot(r0, r1, a_lo, a_hi):
    """One annular-sector window cutter, radial sides inset so the spokes
    between slots keep constant LID_SPOKE_W width (v2's cap style)."""
    def edge(a_deg, r):
        d = math.degrees(math.asin((LID_SPOKE_W / 2.0) / r))
        return a_deg + d if a_deg == a_lo else a_deg - d

    pts = []
    n = 6
    for i in range(n + 1):                       # inner arc, a_lo→a_hi
        a = math.radians(edge(a_lo, r0) + (edge(a_hi, r0) - edge(a_lo, r0)) * i / n)
        pts.append((r0 * math.cos(a), r0 * math.sin(a)))
    for i in range(n + 1):                       # outer arc, back a_hi→a_lo
        a = math.radians(edge(a_hi, r1) + (edge(a_lo, r1) - edge(a_hi, r1)) * i / n)
        pts.append((r1 * math.cos(a), r1 * math.sin(a)))
    return (cq.Workplane("XY").workplane(offset=LID_Z0 - 0.5)
            .polyline(pts).close().extrude(LID_T + 1.0))


def _build_lid():
    body = (cyl(LID_OD, LID_T, z=LID_Z0)
            .cut(cyl(2.0 * DRUM_IR, LID_T + 1.0, z=LID_Z0 - 0.5)))
    # two window rows between the three solid rings
    pitch = 360.0 / LID_SPOKE_N
    rows = ((LID_RING_IN_R1, LID_MID_RC - LID_MID_W / 2.0),
            (LID_MID_RC + LID_MID_W / 2.0, LID_RIM_R0))
    for r0, r1 in rows:
        for i in range(LID_SPOKE_N):
            body = body.cut(_win_slot(r0, r1, i * pitch, (i + 1) * pitch))
    # the four hanging bayonet tenons (modelled SEATED)
    for i in range(LIDJ_SITES):
        body = body.union(lid_tenon(i * 360.0 / LIDJ_SITES))
    return heal(body)


lid = _build_lid()
