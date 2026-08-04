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

from v2.dimensions import NOZZLE  # bead-multiple sizing unit
from v2.helpers import cone_solid, cyl, heal, make_keys
from v2.dimensions import HUB_OD
from .params import (
    RIM_ID, RIM_OD, RIM_H, RATCHET_DEPTH, RATCHET_TEETH,
    RATCHET_H, BRAKE_H, RIM_SEAT_Z, RATCHET_PHASE_DEG,
    SEP_PASS_W, SEP_PASS_ANGLE_DEG, SEP_PASS_WALL,
    SEAT_SHOULDER_OD,
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
    (connector included) cross between the tray chamber below and the spool
    chamber above. Tilt direction follows the winding (user's spec): the
    cable runs CW (viewed from +Z) from the SOURCE below to the CONTROLLER
    end at the exit port, so CLIMBING through the pass it advances CW —
    the tunnel axis runs along the (0,−1,+1) line at its azimuth (a CCW
    climb shipped first, user-caught: it kinked the wrap direction against
    the ratchet's freewheel sense). The 45° ceiling prints support-free in
    the separator's flat print. Inner face leaves a SEP_PASS_WALL web over
    the bore; azimuth between two key slots.
    `grow` inflates the prism on all sides (used to mask the test windows so
    they never breach the tunnel's SEP_PASS_WALL webs)."""
    axis_r = RIM_ID / 2 + SEP_PASS_WALL + SEP_PASS_W / 2
    L = (RIM_H + 4.0) / math.sin(math.radians(45.0)) + 4.0
    w = SEP_PASS_W + 2 * grow
    return (cq.Workplane("XY")
            .box(w, w, L + 2 * grow, centered=True)
            .rotate((0, 0, 0), (1, 0, 0), 45.0)    # +Z → (0,−1,+1)/√2 (CW climb)
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
SEP_WIN_SPOKE_W  = 4 * NOZZLE                                  # web = 4 beads
SEP_WIN_R_IN     = RIM_ID / 2 + 2 * NOZZLE                     # hub-backed collar
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
    for r0, r1 in ((SEP_WIN_R_IN, SEP_WIN_MID_R - 2 * NOZZLE),
                   (SEP_WIN_MID_R + 2 * NOZZLE, SEP_WIN_R_OUT)):
        for i in range(SEP_WIN_SPOKE_N):
            s = _win_slot(r0, r1, i * pitch, (i + 1) * pitch).cut(mask)
            if s.solids().size():
                sep = sep.cut(s)
    return sep


# ── Brake-band MICRO-KNURL (vertical "layer lines") ─────────────────────────
# A smooth band OD slices as LONG circumferential perimeter lines — glassy
# in exactly the direction the brake needs friction (user print finding).
# The user's bench test: TPU rubbed ACROSS layer lines grips decently — so
# the band's OD replicates that texture ROTATED 90°: a fine triangle-wave
# micro-serration (vertical ridges ⊥ the slip direction), at the smallest
# pitch/amplitude a 0.8 nozzle can actually draw. The slicer's perimeter
# must wiggle at every lobe (the long lines break into edge after edge)
# and the TPU pad bites the ridge field under squeeze. PEAKS sit at the
# nominal band radius, so the pad's flush-contact math is unchanged; the
# ridges print as clean vertical flutes in the inverted print.
KNURL_N     = 300                     # lobes — ~1.5 mm pitch at the OD (five
                                      # per ratchet tooth; the closest a 0.8
                                      # nozzle gets to layer-line pitch)
KNURL_DEPTH = 0.4                     # half a nozzle — a ridge field, not
                                      # flutes; tune by rig grip test


def brake_band_ring(inner_d, z0, h):
    """The knurled brake band: a KNURL_N-lobe triangle-wave ring from z0,
    h tall, bored to inner_d. Shared with the test rig's wall band so the
    rig's grip test matches the real part. PHASE-LOCKED to the ratchet
    star (KNURL_N = 5 lobes per tooth): a knurl PEAK sits under every
    TOOTH TIP, so in the inverted print the teeth growing on this band
    are supported at their peaks (a free-phased knurl left tips hanging
    up to 0.4 over valleys, user-caught; the residual ramp-over-valley
    deficit next to each tip is ~0.16 — under half a nozzle, the print
    rounds through it)."""
    pts = []
    r_pk = RIM_OD / 2.0
    r_vl = r_pk - KNURL_DEPTH
    ph = math.radians(RATCHET_PHASE_DEG)
    for i in range(KNURL_N):
        a0 = ph + 2.0 * math.pi * i / KNURL_N
        a1 = a0 + math.pi / KNURL_N
        pts.append((r_pk * math.cos(a0), r_pk * math.sin(a0)))
        pts.append((r_vl * math.cos(a1), r_vl * math.sin(a1)))
    ring = (cq.Workplane("XY").workplane(offset=z0)
            .polyline(pts).close().extrude(h))
    return ring.cut(cyl(inner_d, h + 1.0, z=z0 - 0.5))


def _build_separator(windows=SEP_TEST_WINDOWS):
    # bottom: ratchet teeth — the brake band sits DIRECTLY on them (the old
    # 45° cone is GONE, user's call): the part prints INVERTED (+Z→−Z,
    # brake band on the bed), so the teeth grow on top of the band's full
    # ring with nothing overhanging
    sep = _ratchet_star(0.0, RATCHET_H)
    # top: KNURLED brake band (see above)
    sep = sep.union(brake_band_ring(RIM_ID, RATCHET_H, BRAKE_H))
    # bore it out so it slides on the spring-chamber hub
    sep = sep.cut(cyl(RIM_ID, RIM_H + 1.0, z=-0.5))
    # 45° SEAT CHAMFER at the bore's bottom edge: the housing's cone seat
    # (spring_housing._rim_seat_cone — the flat ledge went with the cap
    # flip) lands PLANE-ON-PLANE in it — self-centering, and the bottom
    # face stays at exactly RIM_SEAT_Z. In this part's INVERTED print the
    # chamfer prints last, growing outward at 45° — self-supporting.
    _ch = (SEAT_SHOULDER_OD - RIM_ID) / 2.0
    sep = sep.cut(cone_solid(SEAT_SHOULDER_OD + 0.4, RIM_ID, _ch + 0.2,
                             z_base=-0.2))
    # KEY SLOTS in the bore matching the spring_housing's key ridges → rotationally
    # locked to the housing (grooves oversized by FIT_CLR, same as the cable rim).
    sep = sep.cut(make_keys(HUB_OD / 2, -0.5, RIM_H + 0.5, groove=True))
    # 45° cable pass-through (spool chamber → tray chamber)
    sep = sep.cut(_cable_pass())
    if windows:                       # TEMPORARY slicer test — see flag above
        sep = _cut_test_windows(sep)
    return heal(sep)


separator = _build_separator()


# ── HUB-FIT COUPON: the separator's housing interface, nothing else ──────────
# A 3.2-wall ring (user's spec) at the separator's full height carrying its
# EXACT spring-housing joinery: the RIM_ID bore, the six key grooves, and
# the 45° cone-seat chamfer — for print-fit testing against a real spring
# housing without printing the full separator. Print it INVERTED like the
# separator (+Z→−Z — chamfer up in print), so the mating surfaces carry
# the same texture. If the fit needs adjusting, the knobs live on THIS
# side (the housing stays fixed, user's plan): RIM_BORE_CLR (bore slide),
# FIT_CLR (groove tangential), and the seat is exact plane-on-plane by
# construction.
def _build_hub_fit_coupon():
    ring = (cyl(RIM_ID + 6.4, RIM_H, z=0.0)
            .cut(cyl(RIM_ID, RIM_H + 1.0, z=-0.5)))
    ring = ring.cut(make_keys(HUB_OD / 2, -0.5, RIM_H + 0.5, groove=True))
    _ch = (SEAT_SHOULDER_OD - RIM_ID) / 2.0
    ring = ring.cut(cone_solid(SEAT_SHOULDER_OD + 0.4, RIM_ID, _ch + 0.2,
                               z_base=-0.2))
    return heal(ring)


test_hub_fit = _build_hub_fit_coupon()


# ── KNURL GRIP COUPON (hand test) ────────────────────────────────────────────
# A palm-sized arc of the REAL brake band for rubbing the TPU pad against:
# same RIM_OD radius (the printed pad's concave face mates it exactly), same
# BRAKE_H height, and it prints the same way the separator does (annular
# face on the bed → the knurl ridges are vertical walls, so the surface
# texture off the nozzle matches the real part). Two sections share the
# arc: a KNURLED span and a SMOOTH span at the nominal radius — rub across
# both with the same squeeze to feel what the texture buys. KNURL_N /
# KNURL_DEPTH are the tuning knobs if the grip disappoints.
_CPN_KN_A = 25.0                      # knurled span (° — ~21 lobes)
_CPN_SM_A = 15.0                      # smooth comparison span (°)
_CPN_R_IN = RIM_OD / 2.0 - 4.0        # backing wall ≥ 3.6 behind the valleys


def _cpn_wedge(a0, a1):
    """Origin-fan wedge cutter over the coupon z-range (chord-overshot arc —
    only the ring band is intersected, the fan interior is harmless)."""
    n = 8
    r = RIM_OD
    pts = [(0.0, 0.0)]
    pts += [(r * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
             r * math.sin(math.radians(a0 + (a1 - a0) * i / n)))
            for i in range(n + 1)]
    return (cq.Workplane("XY").workplane(offset=-1.0)
            .polyline(pts).close().extrude(BRAKE_H + 2.0))


def _build_knurl_coupon():
    kn = (brake_band_ring(2.0 * _CPN_R_IN, 0.0, BRAKE_H)
          .intersect(_cpn_wedge(0.0, _CPN_KN_A)))
    sm = (cyl(RIM_OD, BRAKE_H, z=0.0)
          .cut(cyl(2.0 * _CPN_R_IN, BRAKE_H + 1.0, z=-0.5))
          .intersect(_cpn_wedge(-_CPN_SM_A, 0.0)))
    # finger tab at the arc's angular middle, pointing inward
    tab = (cq.Workplane("XY")
           .box(20.0, 14.0, BRAKE_H, centered=(False, True, False))
           .translate((_CPN_R_IN - 19.0, 0.0, 0.0))
           .rotate((0, 0, 0), (0, 0, 1), (_CPN_KN_A - _CPN_SM_A) / 2.0))
    return heal(kn.union(sm).union(tab))


test_brake_knurl = _build_knurl_coupon()
