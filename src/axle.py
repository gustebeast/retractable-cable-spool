"""AXLE — the rotating spring arbor (the v3 architecture flip: in v2 the
axle was static and the housing spun; now the axle spins in the frame's
bearing and the spring body will be fixed to the frame).

Carried from v1/v2: the TWO-HALF glue joint (the joint collar can't pass
the spring's flange holes, so the axle splits and the halves meet inside
the spring). The strip slits are OPEN-ENDED slots (user's call — the
closed-window experiment is reversed: the spring's central strip is
FIXED and cannot thread a window, so the halves SLIDE ONTO the strip,
v2's design): the separator's slit runs open through its +z column top
(the mortise mouth is split), axle_top's runs open through its tenon
bottom (the tenon tip is split).

SEXES SWAPPED (user, #878: the spring installs UPSIDE DOWN in v3 — the
axle spins instead of being fixed, which flips the wind chirality — so
the Ø13.4 flange hole is on TOP now and the Ø8.7 on the bottom):

  * TOP half — the MORTISE half: retention lip at the very top (rests
    on the 608's inner race; tops out UNDER the frame face — flat
    assembly top), Ø7.95 shaft through the bearing, then a 45° cone to
    the Ø11.45 MORTISE COLLAR entering the spring through the top
    Ø13.4 hole, bore + open-bottom strip slit down to the datum plane.
    Prints FLIPPED (lip disc on the bed; the shaft→collar cone widens
    going up in print — 45°, support-free).
  * BOTTOM half = axle_separator — the TENON half, ONE printed part
    (user's call): Ø11.45 base column through the disk, 45° root cone
    topping at the SHOULDER (the fat section can't pass the Ø8.7
    bottom hole, so it stops TENON_SHOULDER_CLR under the spring),
    then the Ø7.95 TENON POST up through the bottom flange hole to the
    top datum plane, open-top strip slit. Fused into v2's SEPARATOR, a
    SOLID disk sitting SEP_SPRING_CLR under the spring.
    The rims are v2's unchanged — knurled brake band + phased ratchet
    star — but stacked brake-DOWN: the part prints UPRIGHT (disk on the
    bed, column rising), and with the full knurl ring bed-side the teeth
    grow on top of it exactly like v2's proven inverted print (a ring
    above tooth valleys would overhang 2.4). v2's 45° tangential cable
    pass-through kept (user's call) — 45° walls, support-free.
"""

import math

import cadquery as cq

from .helpers import cone_solid, cyl, drop_stray_shells, heal
from .lid_joint import sep_tenon
from .params import (
    AXLE_PRINT_D, AXLE_LIP_OD, AXLE_LIP_H, AXLE_Z_BOT, AXLE_Z_TOP,
    BRG_Z1,
    JOINT_H, JOINT_Z0, JOINT_Z1, JOINT_MORTISE_HOLE_D, JOINT_MORTISE_OD,
    TOP_CONE_Z0, TOP_CONE_Z1, SHOULDER_Z,
    SLIT_W, SLIT_ZLO, SLIT_ZHI,
    RIM_OD, RATCHET_DEPTH, RATCHET_TEETH, RATCHET_PHASE_DEG,
    BRAKE_H, RIM_H, KNURL_N, KNURL_DEPTH,
    SEP_PASS_W, SEP_PASS_ANGLE_DEG, SEP_PASS_R,
    SEP_Z0, SEP_Z1, AXLE_ROOT_CONE_H,
    DRUM_IR, DRUM_OR, DRUM_Z1, LIDJ_SITES,
    BRG_BORE, BOT_SEAT_CONE_Z0, BOT_CONE_CAP_Z, BOT_CONE_CAP_D,
)

_SLIT_Y_OVERSHOOT = JOINT_MORTISE_OD + 10.0


def _slit(z0, z1):
    """Spring-strip slit: a plain open-ended slot through the shaft in Y
    over z0..z1 — pass z0/z1 BEYOND the part so the joint-side end is
    fully open (the halves slide onto the fixed strip, v2's design).
    Each half's one CLOSED end lands as a print FLOOR in that part's
    own orientation (separator upright → its closed bottom end; top
    half flipped → its closed top end), so the old 45° roof peaks are
    no longer needed anywhere."""
    w = SLIT_W / 2.0
    return (cq.Workplane("XZ")
            .polyline([(-w, z0), (w, z0), (w, z1), (-w, z1)]).close()
            .extrude(_SLIT_Y_OVERSHOOT / 2.0, both=True))


def _build_top():
    """The MORTISE half now (sexes swapped — see the module docstring):
    lip → Ø7.95 shaft through the bearing → 45° cone (inside the top
    flange's Ø13.4 hole zone) → Ø11.45 collar down to the datum plane,
    carrying the female bore and the open-bottom strip slit."""
    t = cyl(AXLE_PRINT_D, AXLE_Z_TOP - TOP_CONE_Z1, z=TOP_CONE_Z1)
    t = t.union(cyl(AXLE_LIP_OD, AXLE_LIP_H, z=BRG_Z1))   # lip on the inner race
    t = t.union(cone_solid(d_bottom=JOINT_MORTISE_OD, d_top=AXLE_PRINT_D,
                           h=TOP_CONE_Z1 - TOP_CONE_Z0, z_base=TOP_CONE_Z0))
    t = t.union(cyl(JOINT_MORTISE_OD, TOP_CONE_Z0 - JOINT_Z0, z=JOINT_Z0))
    # female bore, opening DOWN at the collar bottom (= the datum plane
    # shared with the separator's slit floor); ceiling at JOINT_Z1
    t = t.cut(cyl(JOINT_MORTISE_HOLE_D, JOINT_H + 0.5, z=JOINT_Z0 - 0.5))
    # slit OPEN through the collar bottom (the axle slides DOWN onto the
    # fixed strip; the mortise mouth splits into two C-halves); the
    # closed top end sits 3.2 in from the spring's edge — a print floor
    # in this half's flipped print; the shaft is SOLID from there up
    # through the bearing
    t = t.cut(_slit(JOINT_Z0 - 1.0, SLIT_ZHI))
    return drop_stray_shells(heal(t))


def _ratchet_star(z_lo, z_hi):
    """v2's tooth band, verbatim: solid toothed cylinder (filled to the
    axis) over [z_lo, z_hi] — radial sawtooth on the OUTER face, tips at
    RIM_OD/2, valleys RATCHET_DEPTH inward, phased by RATCHET_PHASE_DEG."""
    r_tip = RIM_OD / 2.0
    r_root = r_tip - RATCHET_DEPTH
    phase = math.radians(RATCHET_PHASE_DEG)
    pts = []
    for i in range(RATCHET_TEETH):
        ts = phase + 2.0 * math.pi * i / RATCHET_TEETH
        pts.append((r_tip * math.cos(ts), r_tip * math.sin(ts)))
        pts.append((r_root * math.cos(ts), r_root * math.sin(ts)))
    return (cq.Workplane("XY").workplane(offset=z_lo)
            .polyline(pts).close().extrude(z_hi - z_lo))


def _knurl_band(z_lo, z_hi):
    """v2's knurled brake band, filled to the axis (the disk is solid now):
    a KNURL_N-lobe triangle wave, peaks AT the nominal RIM_OD/2 (flush-
    contact math unchanged), PHASE-LOCKED to the ratchet star — 5 lobes
    per tooth puts a peak under every tooth tip, so the teeth printing on
    top of this ring are supported at their tips (v2 print finding)."""
    r_pk = RIM_OD / 2.0
    r_vl = r_pk - KNURL_DEPTH
    ph = math.radians(RATCHET_PHASE_DEG)
    pts = []
    for i in range(KNURL_N):
        a0 = ph + 2.0 * math.pi * i / KNURL_N
        a1 = a0 + math.pi / KNURL_N
        pts.append((r_pk * math.cos(a0), r_pk * math.sin(a0)))
        pts.append((r_vl * math.cos(a1), r_vl * math.sin(a1)))
    return (cq.Workplane("XY").workplane(offset=z_lo)
            .polyline(pts).close().extrude(z_hi - z_lo))


def ratchet_star_world():
    """The tooth band as a WORLD-position solid (filled to the axis): the
    ratchet lever's pawl edge is cut as its exact negative (v2's pattern —
    shared geometry, never re-modelled)."""
    return _ratchet_star(SEP_Z0 + BRAKE_H - 0.2, SEP_Z1 + 0.2)


def knurl_band_world(z_lo, z_hi):
    """The knurled band solid (filled to the axis) over z_lo..z_hi: the
    brake pad's concave face is cut by it, so the TPU carries the
    texture's exact negative (v2's grip design)."""
    return _knurl_band(z_lo, z_hi)


def _cable_pass():
    """v2's 45° diagonal cable tunnel, ported: a SEP_PASS_W square prism
    tilted 45° TANGENTIALLY through the disk — the working cable
    (connector included) crosses from above the separator to below.
    Tilt sense follows the CW wind (v2's finding: a CCW climb kinks the
    wrap against the ratchet's freewheel sense); 45° walls print
    support-free in the upright print."""
    L = (RIM_H + 4.0) / math.sin(math.radians(45.0)) + 4.0
    return (cq.Workplane("XY")
            .box(SEP_PASS_W, SEP_PASS_W, L, centered=True)
            .rotate((0, 0, 0), (1, 0, 0), 45.0)    # +Z → (0,−1,+1)/√2 (CW climb)
            .translate((SEP_PASS_R, 0, SEP_Z0 + RIM_H / 2.0))
            .rotate((0, 0, 0), (0, 0, 1), SEP_PASS_ANGLE_DEG))


def _drum_wall():
    """The DRUM WALL — what the cable wraps: an annular wall rising from
    the separator disk to DRUM_Z1, wrapping the static gate wall with the
    user's 0.8 bore clearance. Its top rim hosts the lid's rotational
    bayonet (see lid_joint.py — the channel cutters are applied in the
    main build below)."""
    return (cyl(2.0 * DRUM_OR, DRUM_Z1 - SEP_Z1, z=SEP_Z1)
            .cut(cyl(2.0 * DRUM_IR, (DRUM_Z1 - SEP_Z1) + 1.0, z=SEP_Z1 - 0.5)))


def _build_axle_separator():
    """The TENON half now (sexes swapped — see the module docstring):
    fat base column through the disk, root cone topping at the
    SHOULDER, Ø7.95 tenon post up through the spring's Ø8.7 bottom
    hole to the top datum plane, open-top strip slit."""
    b = cyl(JOINT_MORTISE_OD, SHOULDER_Z - AXLE_Z_BOT, z=AXLE_Z_BOT)
    b = b.union(cyl(AXLE_PRINT_D, JOINT_Z1 - SHOULDER_Z,
                    z=SHOULDER_Z))                # the TENON POST — its tip
                                                  # at JOINT_Z1 = the top
                                                  # half's slit ceiling plane
    # 45° ROOT CONE stiffening the column↔disk junction (user's call):
    # flares at 45° from the column Ø down onto the disk top; its top IS
    # the column→tenon SHOULDER now (the fat section stops
    # TENON_SHOULDER_CLR under the spring — the Ø8.7 hole is above).
    # Widens TOWARD the bed in the upright print — self-supporting; the
    # shoulder's flat annulus faces UP: no overhang either.
    b = b.union(cone_solid(d_bottom=JOINT_MORTISE_OD + 2.0 * AXLE_ROOT_CONE_H,
                           d_top=JOINT_MORTISE_OD,
                           h=AXLE_ROOT_CONE_H, z_base=SEP_Z1))
    b = b.union(_knurl_band(SEP_Z0, SEP_Z0 + BRAKE_H))       # brake band on the bed
    b = b.union(_ratchet_star(SEP_Z0 + BRAKE_H, SEP_Z1))     # teeth grow on the ring
    b = b.union(_drum_wall())
    # slit OPEN through the +z tenon tip (the separator slides UP onto
    # the fixed strip — v2's design); the closed bottom end sits 3.2 in
    # from the spring's edge — 0.8 of shrinkage margin past the strip's
    # 4-inset lower end (user) — a print floor above the root cone
    b = b.cut(_slit(SLIT_ZLO, JOINT_Z1 + 1.0))
    for i in range(LIDJ_SITES):                # lid-bayonet tenons STANDING on
        b = b.union(sep_tenon(i * 360.0 / LIDJ_SITES))   # the wall top (user:
                                               # sides swapped — the upright
                                               # print grows them native)
    b = b.cut(_cable_pass())
    # BOTTOM-BEARING SEAT in the disk's underside (user's design): the
    # bearing sits FLUSH with the disk face; the 45° cone rising off the
    # pocket bore IS the seat (the race's factory OD-edge chamfer
    # face-mates it — v2's flipped-lip trick), and the cone CAPS FLAT at
    # the quality web under the disk top: a deliberate Ø19.4 ceiling
    # bridge — only empty space above the bearing, sag there breaks
    # nothing (user's call).
    b = b.cut(cyl(BRG_BORE, (BOT_SEAT_CONE_Z0 - SEP_Z0) + 0.5,
                  z=SEP_Z0 - 0.5))
    b = b.cut(cone_solid(d_bottom=BRG_BORE, d_top=BOT_CONE_CAP_D,
                         h=BOT_CONE_CAP_Z - BOT_SEAT_CONE_Z0,
                         z_base=BOT_SEAT_CONE_Z0))
    # a lid-bayonet tenon fuse strands an orphan internal face (an OPEN
    # second shell — Bambu flags it); strip it (see helpers)
    return drop_stray_shells(heal(b))


axle_top = _build_top()
axle_separator = _build_axle_separator()
