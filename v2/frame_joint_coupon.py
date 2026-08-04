"""FRAME-JOINT print-fit coupon (user's design): the four frame_top ↔
frame_bottom ROTATIONAL arc joints, extracted with just enough structure
to prove all four seat SIMULTANEOUSLY — the real acceptance test is the
phase/concentricity stack-up across sites, which single-joint coupons
can't see, so the four joints must be held mutually RIGID:

  test_frame_joint_bottom — the four beam-top PILLARS (true beam section,
      true radius) carrying the REAL arc tenons, tied by a 1.6-thick
      PLUS plate at their base (low material, high in-plane rigidity —
      in-plane is what fixes the sites' relative phase).
  test_frame_joint_top — FULL-HEIGHT blocks at the four arm ends, each
      carrying its REAL arc mortise channel (same cutter as frame_top,
      stop + entry overshoot included), dropping down to the same
      1.6-thick plus between them. The X-AXIS bar is full height along
      its whole length and carries the MOUNT joinery on that one axis
      (user's spec): the real mortise+pocket channel pairs in its top
      face + the two X-tip lock grooves — same cutters as frame_top.
  test_mount_strip — the mount bracket reduced to its tenon BAR (no
      cross arm, no lock grooves): two hanging octagon tenons + screw
      holes, sliding into the coupon's X channels exactly like the real
      bracket into frame_top.

All pieces print flat (−Z→+Z; the strip inverted like the real bracket).
TEST 1 (frame joints): set the top on the bottom rotated ~20° CCW, z-mate
through the open quadrants, rotate CW — all four tenons must seat against
the stops together. TEST 2 (mount): drop the strip's tenons through the
top piece's entry pockets and slide +x to the stops. Exported/posed
SEATED.
"""

import cadquery as cq

from v2.dimensions import NOZZLE  # bead-multiple sizing unit

from v2.helpers import heal
from .mount import mount_channel_cuts, test_mount_strip  # noqa: F401  (strip re-exported via build)
from .frame import _arc_tenon, _arc_mortise, _BEAM_RC, _R_OUT, _TOPJ_R0
from .params import (
    BEAM_SIZE, BEAM_Z1, TOP_RIB_SIZE, TOP_RIB_Z1,
)

_SITES   = (0.0, 90.0, 180.0, 270.0)
_PILLAR_H = 3 * NOZZLE                       # material under the tenon-root plane
                                      # (user's trim: the 1.6 plus + 0.8 —
                                      # the tenon root sinks 1.0, landing
                                      # 0.2 into the plus band; the two
                                      # pieces' pluses end up 0.8 apart)
_PLUS_T   = 2 * NOZZLE                       # tie-plate thickness (user's spec)
_BLK_R0   = _TOPJ_R0 - 3 * NOZZLE            # block inner face — cavity flat + wall
_Z0       = BEAM_Z1 - _PILLAR_H       # coupon base plane (world)


def _plus_plate(w, z0, r_out=_R_OUT):
    """1.6-thick plus: two crossed w-wide bars spanning the full ±r_out."""
    bx = (cq.Workplane("XY").workplane(offset=z0)
          .rect(2.0 * r_out, w).extrude(_PLUS_T))
    by = (cq.Workplane("XY").workplane(offset=z0)
          .rect(w, 2.0 * r_out).extrude(_PLUS_T))
    return bx.union(by)


def _build_bottom():
    p = _plus_plate(BEAM_SIZE, _Z0)
    for a in _SITES:
        pillar = (cq.Workplane("XY").workplane(offset=_Z0)
                  .center(_BEAM_RC, 0.0).rect(BEAM_SIZE, BEAM_SIZE)
                  .extrude(_PILLAR_H)
                  .rotate((0, 0, 0), (0, 0, 1), a))
        p = p.union(pillar).union(_arc_tenon(a))
    return heal(p.translate((0, 0, -_Z0)))


def _build_top():
    """Y-axis: mortise blocks at the ends dropping to the plus. X-axis: a
    FULL-HEIGHT bar end to end — it hosts the MOUNT channels in its top
    face (they sit at mid-arm, where a 1.6 plus has no material), and a
    full bar is exactly the real arm anyway. Solids unioned first, THEN
    every cutter — arc mortises, mount channel pairs, X-tip lock grooves
    — so each mouth opens exactly as it does in the real frame_top."""
    t = _plus_plate(TOP_RIB_SIZE, BEAM_Z1)
    t = t.union(
        cq.Workplane("XY").workplane(offset=BEAM_Z1)
        .rect(2.0 * _R_OUT, TOP_RIB_SIZE)
        .extrude(TOP_RIB_Z1 - BEAM_Z1))
    for a in (90.0, 270.0):
        blk = (cq.Workplane("XY").workplane(offset=BEAM_Z1)
               .center((_BLK_R0 + _R_OUT) / 2.0, 0.0)
               .rect(_R_OUT - _BLK_R0, TOP_RIB_SIZE)
               .extrude(TOP_RIB_Z1 - BEAM_Z1)
               .rotate((0, 0, 0), (0, 0, 1), a))
        t = t.union(blk)
    for a in _SITES:
        t = t.cut(_arc_mortise(a))
    # MOUNT joinery, X axis only (base groove sits at +Y → 90/270 put the
    # two grooves at the ∓X tips)
    for c in mount_channel_cuts(channel_angles=(0.0,),
                                groove_angles=(90.0, 270.0)):
        t = t.cut(c)
    return heal(t.translate((0, 0, -_Z0)))


test_frame_joint_bottom = _build_bottom()
test_frame_joint_top    = _build_top()
