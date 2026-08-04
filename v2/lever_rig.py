"""TPU-LEVER TEST RIG — print-validate the design's biggest unknown: the
TPU square torsion axles (pre-twist seating preload, pull stiffness) and
the TPU brake pad's grip, on the REAL lever geometry without printing the
whole spool.

TWO printed parts (user's call — the fused rig blocked the ratchet
pre-tension workflow: rotate the pawl INTO the spool area, pull it back,
THEN add the spool):

  · test_lever_rig — the STAND: frame_bottom's +X lever mount, REBUILT
    from frame.py's building blocks (not cropped from the finished part —
    a boolean whose cutter contains the plus centre's intersecting
    axle/rod bores silently empties in OCCT): fan + climbs, columns, arch
    loop, the diamond axle keyways + blind bores, thrust rings, brake rest
    tab, the beam with its REAL wall dovetail channel, and the sill arc
    (the ratchet's 15° over-pull stop). Prints −Z→+Z, bed = the flat runs.

  · test_lever_wall — the SPOOL STAND-IN, installed AFTER
    pre-tensioning: the separator's toothed profile (real phased teeth,
    ridges to the bed → overhang-free standing print), 45° cone, true
    brake band, ±14° about +X, plus a BRIDGE to the beam carrying the
    REAL wall tenon — it slides DOWN the beam's dovetail channel exactly
    like the production wall (free print-validation of that joint). Its
    underside over the stand's centre climb follows the climb's 45° top
    plane + 0.2, so the vertical slide-in never touches the stand; prints
    −Z→+Z standing on the tooth arc + bridge foot.

Assemble with the REAL printed parts: ratchet_lever + brake_lever, two
square TPU axles (lever_pin), brake_pad_tpu.
"""

import math

import cadquery as cq

from v2.dimensions import NOZZLE  # bead-multiple sizing unit

from v2.helpers import heal
from cadkit.joinery import PrintSpec, joint
from .separator import _ratchet_star, brake_band_ring
from .frame import (
    _lever_mount, _beam, _arm_trim, _beam_bottom_trim, _lever_pin_bores,
    _box, _XC, _SB_Y0, _X_SPLIT,
)
from .params import (
    RIM_SEAT_Z, RATCHET_H, BRAKE_H, RIM_OD, RATCHET_DEPTH,
    BOT_RIB_Z0, BOT_RIB_Z1, BEAM_SIZE, BEAM_Z1, WALL_IR, WALL_OR,
    RATCHET_WIN_Z0, WALL_Z0, JOINT_WIDTH, JOINT_DEPTH, JOINT_CLR,
    JOINT_BACK_CLR, POST_OUT_T,
)

_R_TIP  = RIM_OD / 2.0                # 72.9 — brake band = tooth tip radius
_R_ROOT = _R_TIP - RATCHET_DEPTH      # tooth root = the seated pawl's rest
_T      = 2 * NOZZLE                         # stand-in wall thickness
_BED    = BOT_RIB_Z0                  # both parts print on the flat runs' plane
# The stand's −x edge: the FAN TIPS' x (where the plan-45° fans' outer edges
# reach the arm's y ±5 flank). The real frame's arm run CONNECTS the fans to
# the centre stub along this whole span; a shorter stub left the fan tips as
# ~14 mm free cantilevers off the columns (user-caught: "include the
# connector so it stays solid").
_X_BACK = _X_SPLIT + POST_OUT_T - POST_OUT_T * math.sqrt(2.0)   # ≈ 48.2
_SWEEP  = 14.0                        # wall half-sweep: covers both levers'
                                      # contact spans, clears the side fans
_Z_TOP  = RIM_SEAT_Z + RATCHET_H + BRAKE_H            # band top (cone gone)


def _rim_wall():
    """The smooth part of the separator stand-in: root-radius band through
    the ratchet zone, 45° cone, true brake band — carried down to the bed."""
    # vertical STEP root→tip at the band boundary (the cone is gone). TWO
    # clean bands — a single stepped polygon self-touches along the z1
    # line (its outward and return edges overlap → non-manifold,
    # scan-caught). The bands only meet through the TOOTH RIDGES the star
    # unions in (their tops land exactly at z1); in this part's standing
    # print the upper band's underside rides the valleys as short
    # ridge-to-ridge bridges (~3.7 spans) — fine.
    z1 = RIM_SEAT_Z + RATCHET_H
    def _band(r0, r1, za, zb):
        return (cq.Workplane("XZ")
                .polyline([(r0, za), (r1, za), (r1, zb), (r0, zb)])
                .close().revolve(2.0 * _SWEEP, (0, 0), (0, 1))
                .rotate((0, 0, 0), (0, 0, 1), -_SWEEP))
    # the brake band carries the same MICRO-KNURL as the real separator
    # (vertical ridge field on the OD — the rig's grip test must match
    # the surface the pad will actually see), masked to the rig's arc
    band = brake_band_ring(2.0 * (_R_TIP - _T), z1, _Z_TOP - z1)
    band = band.intersect(_band(_R_TIP - _T - 1.0, _R_TIP + 1.0,
                                z1 - 0.5, _Z_TOP + 0.5))
    return _band(_R_ROOT - _T, _R_ROOT, _BED, z1).union(band)


def _teeth():
    """REAL ratchet teeth (the separator's phased star), ridges run to the
    BED so the standing print never overhangs; reduced to the wall's
    annulus and the ±_SWEEP sector."""
    z1 = RIM_SEAT_Z + RATCHET_H
    star = (_ratchet_star(_BED - RIM_SEAT_Z, RATCHET_H)
            .translate((0, 0, RIM_SEAT_Z)))
    star = star.cut(cq.Workplane("XY").workplane(offset=_BED - 1.0)
                    .circle(_R_ROOT - _T).extrude((z1 - _BED) + 2.0))
    for a in (+_SWEEP, -_SWEEP):
        half = (cq.Workplane("XY").workplane(offset=_BED - 1.0)
                .polyline([(-200.0, 0.0), (200.0, 0.0),
                           (200.0, a * 15.0), (-200.0, a * 15.0)])
                .close().extrude((z1 - _BED) + 2.0)
                .rotate((0, 0, 0), (0, 0, 1), a))
        star = star.cut(half)
    return star


def _sill_block():
    """The ratchet over-pull stop: an ARC segment of the real wall ring's
    radial band whose top face is the window sill — the pawl block's bottom
    edge lands on it at RATCHET_STOP_DEG exactly as in the full assembly.
    Lives on the SLIDE-IN WALL part, not the stand (user-caught: fused to
    the stand it blocked the backward pre-tension swing — in the real
    assembly this stop is wall_bottom material, which installs AFTER the
    levers are tensioned). Sweeps 2°..13.5°: fuses into the bridge at its
    low end, covers the pawl's full y, stops short of the side climb band;
    its |y|<6 portion is corridor-cut like the rest of the wall part."""
    prof = [(WALL_IR, _BED), (WALL_OR, _BED),
            (WALL_OR, RATCHET_WIN_Z0), (WALL_IR, RATCHET_WIN_Z0)]
    return (cq.Workplane("XZ").polyline(prof).close()
            .revolve(12.5, (0, 0), (0, 1))
            .rotate((0, 0, 0), (0, 0, 1), 1.0))    # 1°..13.5°: overlaps the
                                                   # bridge (y ≤ 2.5) at its
                                                   # low end — volumetric fuse


def _corridor_cut():
    """Everything under the stand's centre-climb 45° top plane (+0.2), over
    the centre corridor |y| ≤ 5.2 (the climb band is |y| ≤ 5) — carves the
    WALL part's underside so its vertical slide-in (and seated pose) never
    touches the stand's climb band or flat-run stub. The 45° face this
    leaves IS the part's solid angled wall (user's call — a narrower
    bridge once left stepped orphan faces here)."""
    return (cq.Workplane("XZ")
            .polyline([(60.0, -14.0), (60.0, 60.0 - 72.1 + 0.2),
                       (90.0, 90.0 - 72.1 + 0.2), (90.0, -14.0)])
            .close().extrude(-10.4).translate((0, -5.2, 0)))


def _build_rig():
    """The STAND (see module docstring)."""
    fb = _box(_X_BACK, _XC + 1.0, -BEAM_SIZE / 2.0, BEAM_SIZE / 2.0,
              BOT_RIB_Z0, BOT_RIB_Z1)
    fb = fb.cut(_arm_trim())
    fb = fb.union(_beam(0.0))                    # the +X beam (no arc tenon)
    fb = fb.union(_lever_mount())
    fb = fb.cut(_beam_bottom_trim())
    fb = fb.cut(_lever_pin_bores())
    # the wall channel — RIG-SPECIFIC: it runs OPEN TO THE BED (the real
    # frame stops it at WALL_Z0 − seat; here the wall part's tenon carries
    # its 45°-grown print support below the tenon bottom, which must ride
    # down with it — the wall seats on its bed feet instead of the stop)
    up = PrintSpec(nozzle=NOZZLE, material="PETG-GF", facing="up")
    ch = (joint(JOINT_WIDTH, (BEAM_Z1 + 1.0) - (_BED - 1.0),
                      tenon=up, mortise=up,
                      install="-z", depth=JOINT_DEPTH)
          .mortise(drop=2.0).translate((WALL_OR, 0.0, _BED - 1.0)))
    fb = fb.cut(ch)
    return heal(fb)


def _build_rig_wall():
    """The slide-in SPOOL STAND-IN (see module docstring)."""
    w = _rim_wall().union(_teeth())
    # bridge to the beam: ONE solid box, y ±6.0 at full depth — from INSIDE
    # the tooth band (x 70.9, overlapping it like the floor tie) to the
    # beam face, so there is no see-through gap between the band and the
    # bridge at valley azimuths (user-caught: the band/bridge cut edges sit
    # 1.2 apart on the 45° plane, leaving slit windows). The corridor cut
    # (±5.2) tunnels through it; the side walls run to the BED. The teeth
    # at |y| < 6 get embedded — the pawl never reaches below y 6.8. The
    # walls' faces graze the beam-side thrust ring's tangent at x=WALL_OR:
    # line contact, zero measure, accepted for the rig.
    w = w.union(_box(70.9, WALL_OR, -6.0, 6.0, _BED, _Z_TOP))
    # the REAL wall tenon (same joint constants as wall.py), spanning the
    # wall's height from WALL_Z0 up — slides down the beam channel to the
    # stop, exactly like the production wall
    up = PrintSpec(nozzle=NOZZLE, material="PETG-GF", facing="up")
    ten = (joint(JOINT_WIDTH, _Z_TOP - WALL_Z0, tenon=up, mortise=up,
                       install="-z", depth=JOINT_DEPTH)
           .tenon(root=1.0).translate((WALL_OR, 0.0, WALL_Z0)))
    w = w.union(ten)
    # T-tenon PRINT SUPPORT (user's recipe — the tenon's flat bottom was an
    # overhang): a 45° X-ramp grows the STEM out of the bridge face, then
    # simultaneous 45° ramps grow the HEAD's width (±y flares, corner-cut
    # 45° in x so every corner grows supported) and its extra depth from
    # the stem, finishing exactly at the tenon's bottom. Every section
    # stays inside the T profile, so it rides down the rig's bed-open
    # channel at install.
    _d = joint(JOINT_WIDTH, 1.0, tenon=up, mortise=up, install="-z",
               depth=JOINT_DEPTH).dims
    nk, hd, du = _d["neck"], _d["head"], _d["depth_used"]
    lip = (du + JOINT_BACK_CLR) / 2.0            # tenon lip (= bar + back clr)
    F, z_t = WALL_OR, WALL_Z0
    z_m = z_t - (du - lip)                       # head growth starts
    z_s = z_m - lip                              # stem ramp roots on the face
    sup = (cq.Workplane("XZ")                    # stem: 45° out of the face
           .polyline([(F, z_s), (F + lip, z_m), (F + lip, z_t + 0.1),
                      (F, z_t + 0.1)]).close()
           .extrude(nk).translate((0, nk / 2.0, 0)))
    sup = sup.union(cq.Workplane("XZ")           # head extra depth: 45° on
               .polyline([(F + lip, z_m),        # from the stem's front
                          (F + du, z_t + 0.1), (F + lip, z_t + 0.1)])
               .close().extrude(nk).translate((0, nk / 2.0, 0)))
    corner = (cq.Workplane("XZ")                 # corner guard: nothing past
              .polyline([(F + lip, z_m),         # the 45° depth line
                         (F + du, z_t + 0.1), (F + du + 4.0, z_t + 0.1),
                         (F + du + 4.0, z_m - 2.0), (F + lip, z_m - 2.0)])
              .close().extrude(12.0).translate((0, 6.0, 0)))
    for s in (1.0, -1.0):                        # head width: ±y 45° flares —
        flare = (cq.Workplane("YZ")              # only in the HEAD band (the
                 .polyline([(s * nk / 2.0, z_m), # lip zone's cavity is just
                            (s * hd / 2.0, z_t + 0.1),   # the neck slot)
                            (s * nk / 2.0, z_t + 0.1)]).close()
                 .extrude(du - lip).translate((F + lip, 0, 0)))
        sup = sup.union(flare.cut(corner))
    w = w.union(sup)
    # the ratchet over-pull SILL rides this part (like the real wall_bottom)
    w = w.union(_sill_block())
    # FLOOR TIE (user's call): a 1.6-thick sector at the bed linking the
    # sill arc to the toothed wall across their 3.2 radial gap — print
    # stability for the sill's free-standing tail (also floors the tooth
    # valleys' bottom 1.6, well below the pawl's z band). The corridor cut
    # below trims its |y| ≤ 5.2 span with everything else.
    w = w.union(cq.Workplane("XZ")
                .polyline([(70.9, _BED), (76.5, _BED),
                           (76.5, _BED + 2 * NOZZLE), (70.9, _BED + 2 * NOZZLE)])
                .close().revolve(12.5, (0, 0), (0, 1))
                .rotate((0, 0, 0), (0, 0, 1), 1.0))
    # clear the stand's centre corridor (climb band + flat-run stub)
    w = w.cut(_corridor_cut())
    # …and the stand's plan-45° FAN edges (y = ±(x − (_XC − _SB_Y0))): the
    # 2.4 teeth pulled the arc's inner band inward, clipping the fan
    # corners at the ±14° ends — shave 0.2 clear (vertical faces; the
    # teeth sit outside these lines and stay intact)
    c0 = _XC - _SB_Y0 + 0.2
    for s in (1.0, -1.0):
        w = w.cut(cq.Workplane("XY").workplane(offset=_BED - 1.0)
                  .polyline([(60.0, s * (60.0 - c0)),
                             (78.0, s * (78.0 - c0)),
                             (60.0, s * (78.0 - c0))])
                  .close().extrude((_Z_TOP - _BED) + 2.0))
    return heal(w)


lever_test_rig = _build_rig()
lever_test_wall = _build_rig_wall()
