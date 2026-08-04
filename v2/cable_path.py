"""CABLE PATH — the working cable posed FULLY RETRACTED (viewer only, not a
print), travelling SOURCE → CONTROLLER:

  1. ENTRY: straight tangent in through the coil cup's house port at the
     back (single fixed hole — design goal 1) onto the AXIAL LOOP's
     bottom wrap;
  2. AXIAL SERVICE LOOP, posed RETRACTED = LOOSE (see params' AXIAL
     block): bottom wraps near the cup floor at the loose diameter, a
     conical spiral climbing and shrinking to the pass radius, and the
     DROP — a gentle helix at the pass radius up to the separator pass's
     lower mouth. ~1.4 posed wraps ≈ LOOP_K_LOOSE;
  3. THE PASS: climbs the separator's 45° tilted tunnel dead on its
     centreline, smoothstep z-easing at both ends;
  4. SPOOL COIL: blends in to hug the hub, then exactly 4 ft (1219.2 mm)
     of retraction length as a flat spiral in the spool chamber;
  5. EXIT: tangent off the outer wrap, crossing the wall dead-centre in
     the exit port at +45° azimuth.

Doctrine: the WOUND cable runs CW (viewed from +Z) from SOURCE to
CONTROLLER — azimuth decreases monotonically along the ENTIRE posed path
(no back-bend: the axial loop absorbs rotation by breathing diameter, not
by reversing wind sense; the pass tunnel's tilt matches).

Retracted is the easy pose (user's call): everything lives inside; extended
would leave a bare hub and 4 ft of off-model cable.
"""

import math

import cadquery as cq

from v2.helpers import heal
from .params import (
    CABLE_D, CABLE_GAP, SEP_Z0, SEP_Z1, SEP_PASS_ANGLE_DEG,
    SEP_PASS_W, SEP_PASS_WALL, RIM_ID,
    CABLE_EXIT_ANGLE_DEG, WALL_IR, WALL_OR,
    COIL_R0, COIL_PITCH, N_MIGRATE,
    LOOP_D_LOOSE, CH_TOP_Z, CH_BOT_Z, CUP_PORT_AZ_DEG,
)

_R0     = COIL_R0                                  # 37.2 — spool coil inner wrap
_PITCH  = COIL_PITCH                               # 3.9 — wrap-to-wrap
_Z_TOP  = SEP_Z1 + CABLE_GAP / 2.0                 # 33.2 — spool-chamber mid-plane
_Z_STUB = SEP_Z0 - 2.1                             # 19.0 — pass lower-mouth plane
_R_PASS = RIM_ID / 2.0 + SEP_PASS_WALL + SEP_PASS_W / 2.0   # 41.55 — tunnel axis r

# ── the SPOOL coil + exit (top) ──────────────────────────────────────────────
_N      = N_MIGRATE                                # ≈ 4.26 turns of retraction
_R_END  = _R0 + _PITCH * _N                        # ≈ 53.8 — outer wrap
_D_TOP  = math.degrees(math.acos(_R_END / WALL_IR))
_PHI_END = CABLE_EXIT_ANGLE_DEG + _D_TOP           # wrap-leave azimuth (mod 360)
_PHI_SP0 = _PHI_END + 360.0 * _N                   # top spiral start (unwrapped)

# ── the PASS (separator tunnel centreline, unwrapped links) ──────────────────
_SLOPE  = math.degrees(1.0) / _R_PASS              # deg of azimuth per mm of z
_Z_MID  = (SEP_Z0 + SEP_Z1) / 2.0                  # 26.1 — tunnel centre plane
_PASS_B = SEP_PASS_ANGLE_DEG + (_Z_MID - _Z_STUB) * _SLOPE   # ≈ 39.8 (mod) mouth
_PASS_T = SEP_PASS_ANGLE_DEG - (_Z_TOP - _Z_MID) * _SLOPE    # ≈ 20.2 (mod) top
# unwrap: the pass top must sit ABOVE the top spiral start with room for the
# hub blend (~90°) + a hug arc
_K_TOP  = math.ceil((_PHI_SP0 + 120.0 - _PASS_T) / 360.0)
_PT_U   = _PASS_T + 360.0 * _K_TOP                 # pass top, unwrapped
_PB_U   = _PT_U + (_PASS_B - _PASS_T)              # pass bottom, unwrapped
_BLEND  = 90.0                                     # pass-exit → hub-radius blend arc

# ── the AXIAL LOOP, posed LOOSE (bottom) — az increasing toward the source ───
_R_BOT   = LOOP_D_LOOSE / 2.0                      # 72.7 — loose wrap radius
_Z_CONE0 = CH_TOP_Z - 2.0                          # −4.5 — cone top (drop hand-over)
_Z_BOT   = CH_BOT_Z + 2.7                          # −26.0 — bottom-wrap plane
_DROP_SW = 90.0                                    # the drop's helix sweep
_BOT_SW  = 160.0                                   # bottom wraps at the loose Ø
_D_ENT   = math.degrees(math.acos(_R_BOT / WALL_IR))   # ≈ 15.2 — entry tangent
# closure: entry crossing must land on the cup port azimuth (mod 360)
_CONE_SW = ((CUP_PORT_AZ_DEG - _D_ENT - _BOT_SW - _DROP_SW - _PASS_B) % 360.0)
if _CONE_SW < 240.0:
    _CONE_SW += 360.0                              # keep a real conical run
_T_U     = _PB_U + _DROP_SW + _CONE_SW + _BOT_SW   # entry touch az, unwrapped


def _pt(phi_deg, r, z):
    a = math.radians(phi_deg)
    return cq.Vector(r * math.cos(a), r * math.sin(a), z)


def _tangent_pts(touch_u, r_t, z):
    """Points along the straight EXIT tangent (touch → outside, CW travel:
    the outboard end sits at LOWER azimuth) out past the wall."""
    a0 = math.radians(touch_u % 360.0)
    tp = _pt(touch_u % 360.0, r_t, z)
    th = cq.Vector(-math.sin(a0), math.cos(a0), 0.0)  # toward +azimuth
    l_max = math.sqrt((WALL_OR + 4.0) ** 2 - r_t ** 2)
    return [tp - th * (l_max * i / 8.0) for i in range(1, 9)]


def _smooth(f):
    return f * f * (3.0 - 2.0 * f)


def _build():
    pts = []
    # 1 — source ENTRY: straight tangent from outside the cup port to the
    # bottom wrap's touch point (CW travel: the crossing sits ABOVE the
    # touch azimuth)
    a0 = math.radians(_T_U % 360.0)
    tp = _pt(_T_U % 360.0, _R_BOT, _Z_BOT)
    th = cq.Vector(-math.sin(a0), math.cos(a0), 0.0)   # toward +azimuth
    l_max = math.sqrt((WALL_OR + 4.0) ** 2 - _R_BOT ** 2)
    for i in range(8, 0, -1):
        pts.append(tp + th * (l_max * i / 8.0))
    # 2a — bottom wraps at the loose Ø, flat, just off the cup floor
    n_b = max(int(_BOT_SW / 7.5), 8)
    for i in range(0, n_b + 1):
        u = _T_U - _BOT_SW * i / n_b
        pts.append(_pt(u % 360.0, _R_BOT, _Z_BOT))
    # 2b — conical spiral: climbing + shrinking to the pass radius
    u_c0 = _T_U - _BOT_SW
    n_c = max(int(_CONE_SW / 7.5), 16)
    for i in range(1, n_c + 1):
        f = i / n_c
        u = u_c0 - _CONE_SW * f
        pts.append(_pt(u % 360.0,
                       _R_BOT + (_R_PASS - _R_BOT) * _smooth(f),
                       _Z_BOT + (_Z_CONE0 - _Z_BOT) * _smooth(f)))
    # 2c — the DROP: gentle helix at the pass radius up to the pass mouth
    for i in range(1, 13):
        f = i / 12.0
        u = _PB_U + _DROP_SW * (1.0 - f)
        pts.append(_pt(u % 360.0, _R_PASS,
                       _Z_CONE0 + (_Z_STUB - _Z_CONE0) * _smooth(f)))
    # 3 — the pass: tunnel centreline with smoothstep z (zero end slopes)
    n_p = 24
    for i in range(1, n_p + 1):
        f = i / n_p
        s = _smooth(f)
        u = _PB_U + (_PT_U - _PB_U) * f
        pts.append(_pt(u % 360.0, _R_PASS, _Z_STUB + (_Z_TOP - _Z_STUB) * s))
    # 4 — hub blend (pass radius → hub radius) + hug arc, spool plane
    for i in range(1, 13):
        u = _PT_U - _BLEND * i / 12.0
        r = _R_PASS + (_R0 - _R_PASS) * i / 12.0
        pts.append(_pt(u % 360.0, r, _Z_TOP))
    hug0 = _PT_U - _BLEND
    n_h = max(int((hug0 - _PHI_SP0) / 7.5), 4)
    for i in range(1, n_h + 1):
        u = hug0 + (_PHI_SP0 - hug0) * i / n_h
        pts.append(_pt(u % 360.0, _R0, _Z_TOP))
    # 5 — the 4 ft spool spiral, inner → outer
    n_s2 = max(int((_PHI_SP0 - _PHI_END) / 7.5), 16)
    for i in range(1, n_s2 + 1):
        u = _PHI_SP0 + (_PHI_END - _PHI_SP0) * i / n_s2
        pts.append(_pt(u % 360.0, _R0 + _PITCH * (_PHI_SP0 - u) / 360.0,
                       _Z_TOP))
    # 6 — exit tangent (touch → outside)
    pts += _tangent_pts(_PHI_END, _R_END, _Z_TOP)

    path = cq.Workplane("XY").newObject(
        [cq.Wire.assembleEdges([cq.Edge.makeSpline(pts)])])
    t0 = (pts[1] - pts[0]).normalized()
    prof = (cq.Workplane(cq.Plane(origin=pts[0],
                                  normal=(t0.x, t0.y, t0.z)))
            .circle(CABLE_D / 2.0))
    return heal(prof.sweep(path, isFrenet=True))


cable_path = _build()
