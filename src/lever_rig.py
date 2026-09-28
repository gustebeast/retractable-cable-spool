"""LEVER TEST RIG (user #999) — print-validate the lever system without
printing the whole spool: the pivot pins, the TPU return clips on their
frame posts (and so their holding force — print a couple of
CLIP_LEAF_T to tune it), the ratchet and brake stop ribs, the pawl's mesh with the teeth and the brake
pad's reach to the band. v2 had the same idea (git: src/lever_rig.py,
deleted at 6c6f6f6); this is the v3 version.

ONE printed part, `test_lever_rig.step`, in frame_bottom's own upright
print (same bed, same orientation — that is the point of the test):

  · frame_bottom CROPPED to the two lever bays: the real part intersected
    with a box, never re-modelled (cadkit coupon rule — tune the frame
    and the rig follows). The box spans both fork columns in y, reaches
    in to the rim sector in x, and stops RIG_ROOF_T above the underside
    of the ratchet arch's run into the beam — so the ratchet's fork
    column stays tied to the beam across the bay (user #1001: cropped at
    the column's own top, the two stood free).
    The floor inside the crop is re-filled SOLID: its viewing holes
    would sit under the rim pedestal as bridges.

  · a SECTOR of the rim stand-in — the axle separator's REAL knurled
    brake band and phased ratchet teeth (axle.py's own builders), ±
    RIG_SWEEP_DEG about +x, standing on a pedestal wall down to the
    floor. It is STATIC: the rig tests the levers' feel, return, stops,
    pawl drop and pad contact, not ratcheting or braking a turning drum.
    One consequence, trimmed on purpose: the pawl's positive catch face
    rubs its tooth ~0.4 as it lifts (the real drum gives way by that
    much toward pay-out; a fixed sector can't), so the rig's teeth are
    cut by the pawl's swept path — the real separator is untouched.

Assemble with the real printed parts: ratchet_lever, brake_lever,
brake_pad_tpu, two lever_pin, ratchet_clip_tpu and brake_clip_tpu.
"""

import math

import cadquery as cq

from .axle import _knurl_band, _ratchet_star
from .frame import frame_bottom, _box, _ARCH_Z0_R, _SB_Y0
from .helpers import cyl, heal
from .levers import R_RIM, ratchet_lever, RATCHET_OPEN_DEG, _POST_OUT
from .params import (
    NOZZLE_D, FLOOR_Z0, FLOOR_Z1, WALL_IR, BRAKE_H, KNURL_DEPTH,
    RATCHET_DEPTH, SEP_Z0, SEP_Z1, LEVER_COL_Y1, RATCHET_PIVOT_Z,
    BEAM_IR, BEAM_SIZE, LEVER_PIVOT_X,
)

RIG_SWEEP_DEG = 18.0             # rim sector half-angle: both levers' pawl
                                 # and pad sit within ±14.5° (|y| ≤ 18.8)
RIG_WEB_T = 3 * NOZZLE_D         # 2.4 — the sector's wall behind the teeth
RIG_X0 = 60.0                    # crop's inner edge: inside the sector's
                                 # inner radius at its ±RIG_SWEEP ends
RIG_Y = LEVER_COL_Y1 + _POST_OUT + 1.0   # crop's half-width: past the
                                 # fork columns AND the clip posts on them
RIG_ROOF_T = 2 * NOZZLE_D        # 1.6 — arch run kept over the ratchet bay

_R_IN = R_RIM - RATCHET_DEPTH - RIG_WEB_T      # sector's inner face


def _crop_top():
    """RIG_ROOF_T over the ratchet arch's apex: the arch climbs 45° off
    the column's inner face, then runs level into the beam's side at
    this height (frame._arch) — the material that joins the two."""
    z_apex = _ARCH_Z0_R + (_SB_Y0 - BEAM_SIZE / 2.0) / 2.0
    return z_apex + RIG_ROOF_T


def _frame_crop():
    z1 = _crop_top()
    crop = frame_bottom.intersect(
        _box(RIG_X0, BEAM_IR + BEAM_SIZE + 1.0, -RIG_Y, RIG_Y,
             FLOOR_Z0 - 1.0, z1))
    # the floor plate SOLID inside the wall band (its viewing holes would
    # be bridges under the rim pedestal)
    floor = cyl(2.0 * (WALL_IR + 0.5), FLOOR_Z1 - FLOOR_Z0, z=FLOOR_Z0)
    floor = floor.intersect(_box(RIG_X0, WALL_IR + 1.0, -RIG_Y, RIG_Y,
                                 FLOOR_Z0 - 1.0, FLOOR_Z1 + 1.0))
    return crop.union(floor)


def _sector():
    """A ±RIG_SWEEP_DEG wedge about +x, z-unbounded enough for the rig."""
    r = 2.0 * (R_RIM + 10.0)
    a = math.radians(RIG_SWEEP_DEG)
    return (cq.Workplane("XY").workplane(offset=FLOOR_Z0 - 2.0)
            .polyline([(0.0, 0.0), (r * math.cos(a), r * math.sin(a)),
                       (r, 0.0), (r * math.cos(a), -r * math.sin(a))])
            .close().extrude((SEP_Z1 - FLOOR_Z0) + 4.0))


def _rim_sector():
    """The separator's real rim — knurled brake band on the bottom, the
    phased teeth growing on it (the separator's own stacking) — cut to an
    annulus RIG_WEB_T deep behind the tooth roots, on a pedestal wall
    down into the floor. The pedestal stays inside the knurl's valley
    radius, so the band's lobes overhang it by only KNURL_DEPTH (0.4)."""
    band = (_knurl_band(SEP_Z0, SEP_Z0 + BRAKE_H)
            .union(_ratchet_star(SEP_Z0 + BRAKE_H, SEP_Z1)))
    band = band.cut(cyl(2.0 * _R_IN, (SEP_Z1 - SEP_Z0) + 2.0, z=SEP_Z0 - 1.0))
    ped = (cyl(2.0 * (R_RIM - KNURL_DEPTH), SEP_Z0 - FLOOR_Z1 + 0.5,
               z=FLOOR_Z1 - 0.5)
           .cut(cyl(2.0 * _R_IN, SEP_Z0 - FLOOR_Z1 + 2.0, z=FLOOR_Z1 - 1.0)))
    return band.union(ped).intersect(_sector()).cut(_pawl_sweep())


def _pawl_sweep(n=24):
    """The ratchet lever's pawl region swept rest → over-pull stop (see
    the module docstring): only the part near the teeth, so the union
    stays cheap."""
    pawl = ratchet_lever.intersect(
        _box(R_RIM - RATCHET_DEPTH - 2.0, R_RIM + 3.0, 0.0, 2.0 * R_RIM,
             SEP_Z0 - 1.0, SEP_Z1 + 1.0))
    out = None
    for i in range(n + 1):
        w = pawl.rotate((LEVER_PIVOT_X, 0, RATCHET_PIVOT_Z),
                        (LEVER_PIVOT_X, 1, RATCHET_PIVOT_Z),
                        -RATCHET_OPEN_DEG * i / n)
        out = w if out is None else out.union(w, clean=False)
    return out


def _build_lever_rig():
    return heal(_frame_crop().union(_rim_sector()))


lever_rig = _build_lever_rig()
