"""SEPARATOR — the part carrying the brake + ratchet rims.

A 12 mm cylinder that slides onto the spring_housing and seats on its cone notch
at half height. Split bottom-half RATCHET / top-half BRAKE with a 45° cone
chamfer between (ratchet → cone → brake), the same stack as the working lever
rim but as a standalone sliding part. Bore = RIM_ID (rides the hub OD) with KEY
SLOTS matching the spring_housing's key ridges, so it stays rotationally LOCKED
to the housing (turns with the drum). In the full design this becomes the spool
floor + tray ceiling + the levers' contact surface; here it is just the rim so
the wall diameter (3.2 mm outside it) can be pinned down.
"""

import math

import cadquery as cq

from src.helpers import cyl, cone_solid, heal, make_keys
from src.dimensions import HUB_OD
from .params import (
    RIM_ID, RIM_OD, RIM_H, RATCHET_DEPTH, RATCHET_TEETH,
    RATCHET_H, CONE_H, BRAKE_H, RIM_SEAT_Z, RATCHET_PHASE_DEG,
    SEP_PASS_W, SEP_PASS_ANGLE_DEG, SEP_PASS_WALL,
)


def _ratchet_star(z_lo, z_hi):
    """Solid toothed cylinder (filled to the axis) over [z_lo, z_hi]: radial
    sawtooth on the OUTER face, tips at RIM_OD/2, valleys RATCHET_DEPTH inward.
    Copied from the working ratchet band. Phased by RATCHET_PHASE_DEG so one
    catch face lands exactly at the ratchet pawl's inner-y azimuth (the lever's
    catch-bump math keys off the same constant)."""
    r_tip  = RIM_OD / 2
    r_root = r_tip - RATCHET_DEPTH
    n = RATCHET_TEETH
    phase = math.radians(RATCHET_PHASE_DEG)
    pts = []
    for i in range(n):
        ts = phase + 2 * math.pi * i / n
        pts.append((r_tip  * math.cos(ts), r_tip  * math.sin(ts)))
        pts.append((r_root * math.cos(ts), r_root * math.sin(ts)))
    return (cq.Workplane("XY").workplane(offset=z_lo)
            .polyline(pts).close().extrude(z_hi - z_lo))


def ratchet_star_world():
    """The tooth band as a WORLD-position solid (filled to the axis), for the
    ratchet lever's pawl cut — the exact negative of the seated separator's
    teeth (the separator itself is built at local z and lifted RIM_SEAT_Z in
    the assembly)."""
    return (_ratchet_star(-0.2, RATCHET_H + 0.2)
            .translate((0, 0, RIM_SEAT_Z)))


def _cable_pass(grow=0.0):
    """the original design's 45° diagonal cable tunnel, ported: a SEP_PASS_W square prism
    tilted 45° TANGENTIALLY through the separator, letting the working cable
    (connector included) cross from the spool chamber above to the tray
    chamber below. Tilt direction follows the CW winding (viewed from +Z):
    following the cable toward its inner end it wraps CW and dives, so the
    tunnel axis runs along the (0,+1,+1) line at its azimuth. The 45° ceiling
    prints support-free in the separator's flat print. Inner face leaves a
    SEP_PASS_WALL web over the bore; azimuth between two key slots.
    `grow` inflates the prism on all sides (used to mask the test windows so
    they never breach the tunnel's SEP_PASS_WALL webs)."""
    axis_r = RIM_ID / 2 + SEP_PASS_WALL + SEP_PASS_W / 2
    L = (RIM_H + 4.0) / math.sin(math.radians(45.0)) + 4.0
    w = SEP_PASS_W + 2 * grow
    return (cq.Workplane("XY")
            .box(w, w, L + 2 * grow, centered=True)
            .rotate((0, 0, 0), (1, 0, 0), -45.0)   # +Z → (0,+1,+1)/√2 (CW dive)
            .translate((axis_r, 0, RIM_H / 2.0))
            .rotate((0, 0, 0), (0, 0, 1), SEP_PASS_ANGLE_DEG))


# ── Slicer-tested and REJECTED: windowed separator ───────────────────────────
# Same arc-slot window treatment as the cable caps (chambers._arc_slot) but
# 12 spokes. Removed 60% of the solid volume yet only saved ~10 g in the
# slicer (the 12 mm interior was mostly 15% infill anyway, and each slot adds
# 12 mm-tall wall loops) — not worth opening the spool chamber into the tray.
# Kept OFF; flip only to re-measure.
SEP_TEST_WINDOWS = False
SEP_WIN_SPOKE_N  = 12
SEP_WIN_SPOKE_W  = 3.2                                  # web = 4 beads
SEP_WIN_R_IN     = RIM_ID / 2 + 1.6                     # hub-backed collar
SEP_WIN_R_OUT    = RIM_OD / 2 - RATCHET_DEPTH - 4.8     # solid ring behind teeth
SEP_WIN_MID_R    = (SEP_WIN_R_IN + SEP_WIN_R_OUT) / 2   # 3.2 mid ring center


def _win_slot(r0, r1, a_lo, a_hi):
    """One annular-sector window cutter, radial sides inset so the spokes
    between slots keep constant SEP_WIN_SPOKE_W width (cap-style)."""
    def edge(a_deg, r):
        d = math.degrees(math.asin((SEP_WIN_SPOKE_W / 2.0) / r))
        return a_deg + d if a_deg == a_lo else a_deg - d

    pts = []
    n = 6
    for i in range(n + 1):                       # inner arc, a_lo→a_hi
        a = math.radians(edge(a_lo, r0) + (edge(a_hi, r0) - edge(a_lo, r0)) * i / n)
        pts.append((r0 * math.cos(a), r0 * math.sin(a)))
    for i in range(n + 1):                       # outer arc, back a_hi→a_lo
        a = math.radians(edge(a_hi, r1) + (edge(a_lo, r1) - edge(a_hi, r1)) * i / n)
        pts.append((r1 * math.cos(a), r1 * math.sin(a)))
    return (cq.Workplane("XY").workplane(offset=-0.5)
            .polyline(pts).close().extrude(RIM_H + 1.0))


def _cut_test_windows(sep):
    """Cut the two window rows, masking every cutter with the wall-grown
    cable-pass prism so the tunnel keeps its full SEP_PASS_WALL webs."""
    mask = _cable_pass(grow=SEP_PASS_WALL)
    pitch = 360.0 / SEP_WIN_SPOKE_N
    for r0, r1 in ((SEP_WIN_R_IN, SEP_WIN_MID_R - 1.6),
                   (SEP_WIN_MID_R + 1.6, SEP_WIN_R_OUT)):
        for i in range(SEP_WIN_SPOKE_N):
            s = _win_slot(r0, r1, i * pitch, (i + 1) * pitch).cut(mask)
            if s.solids().size():
                sep = sep.cut(s)
    return sep


def _build_separator(windows=SEP_TEST_WINDOWS):
    # bottom: ratchet teeth
    sep = _ratchet_star(0.0, RATCHET_H)
    # 45° cone chamfer filling the tooth valleys up to the smooth brake OD
    sep = sep.union(cone_solid(RIM_OD - 2 * RATCHET_DEPTH, RIM_OD, CONE_H, RATCHET_H))
    # top: smooth brake band
    sep = sep.union(cyl(RIM_OD, BRAKE_H, z=RATCHET_H + CONE_H))
    # bore it out so it slides on the spring-chamber hub
    sep = sep.cut(cyl(RIM_ID, RIM_H + 1.0, z=-0.5))
    # KEY SLOTS in the bore matching the spring_housing's key ridges → rotationally
    # locked to the housing (grooves oversized by FIT_CLR, same as the cable rim).
    sep = sep.cut(make_keys(HUB_OD / 2, -0.5, RIM_H + 0.5, groove=True))
    # 45° cable pass-through (spool chamber → tray chamber)
    sep = sep.cut(_cable_pass())
    if windows:                       # TEMPORARY slicer test — see flag above
        sep = _cut_test_windows(sep)
    return heal(sep)


separator = _build_separator()
