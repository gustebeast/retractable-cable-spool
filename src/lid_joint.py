"""LID ↔ DRUM-WALL rotational bayonet — shared joint code (both consumers
call THIS module; the coupon rule: never re-model the joint per part).

v2's HALF-ARROWHEAD arc profile carried verbatim (the manual design that
replaced the cadkit octagon arc after it bound in print): flat inner
cylindrical face at the drum bore, stem, ONE outboard 45° flare (the
retention lip — lifting cams the tenon onto the flat, which blocks
flat-on-flat), 45° taper to a dull tip at the flat. Lateral faces run
JOINT_CLR; the z-sandwich pairs run JOINT_BACK_CLR (cadkit's fiber-fill
policy — the T joint's print-validated GF depth rule).

MOUNT-FLIPPED (user's design): the LID hangs the tenons (prints INVERTED
→ they stand library-native, flare rising 45°); the WALL TOP carries per
site an entry POCKET (vertical-walled envelope — the tenon's whole
silhouette drops through) + the retained cavity arc CW of it. Seat by
rotating the lid CW (viewed +Z) until the tenons meet the cavities' CW
end walls (the stops). NOTE: back-rotation is only friction-blocked for
now — if the working torque direction lands CCW once the cable chirality
is final, add a small lock (v2 mount_lock style).
"""

import math

import cadquery as cq

from .params import (
    NOZZLE, JOINT_CLR, JOINT_BACK_CLR,
    TOPJ_STEM, TOPJ_FLARE, TOPJ_NECK, TOPJ_TIP,
    LIDJ_TEN_SWEEP, LIDJ_SEAT_CLR, LIDJ_ENTRY_OVER,
    DRUM_IR, DRUM_T, DRUM_Z1, LID_Z0, SEP_Z0,
)

_R0 = DRUM_IR                          # tenon flat face = the drum/lid bore
TOPJ_TOP = TOPJ_NECK + TOPJ_FLARE + (TOPJ_STEM + TOPJ_FLARE - TOPJ_TIP)   # 6.4
TOPJ_RD = (TOPJ_TOP + TOPJ_TIP + TOPJ_STEM - TOPJ_NECK
           + 2.0 * JOINT_BACK_CLR) / 2.0       # 4.3 — cavity flare∩taper corner
_TOPJ_TOP = TOPJ_TOP                   # (exported names; locals kept below)
_TOPJ_RD = TOPJ_RD

# every printed wall around the wall-top cavity keeps the 1.6 tier
assert DRUM_T - _TOPJ_RD >= 2 * NOZZLE - 1e-9, \
    "drum wall under 1.6 outboard of the lid-joint cavity — grow DRUM_T"
# the cavities may run deeper than the short wall — they continue into the
# separator DISK (one solid); the floor just can't approach the disk bottom
assert (DRUM_Z1 - SEP_Z0) - (_TOPJ_TOP + JOINT_BACK_CLR) >= 3 * NOZZLE - 1e-9, \
    "lid-joint cavities reach too close to the separator disk's bottom"

_R_REF  = _R0 + (TOPJ_STEM + TOPJ_FLARE) / 2.0     # profile mid radius
_SEAT_A = math.degrees(LIDJ_SEAT_CLR / _R_REF)
_OVER_A = math.degrees(LIDJ_ENTRY_OVER / _R_REF)


def _topj_pts(tenon, z_base, r0):
    """(r, z) profile points, v2 verbatim, at flat-face radius r0.
    tenon=True → nominal; False → the CAVITY: lateral faces dilated
    JOINT_CLR, z-sandwich faces JOINT_BACK_CLR."""
    s2, f, zn, tp = TOPJ_STEM, TOPJ_FLARE, TOPJ_NECK, TOPJ_TIP
    zt = _TOPJ_TOP
    if tenon:
        return [(r0, z_base), (r0 + s2, z_base),
                (r0 + s2, zn), (r0 + s2 + f, zn + f),
                (r0 + tp, zt), (r0, zt)]
    c, bc = JOINT_CLR, JOINT_BACK_CLR
    return [(r0 - c, z_base), (r0 + s2 + c, z_base),
            (r0 + s2 + c, zn + c - bc),
            (r0 + _TOPJ_RD, _TOPJ_RD - s2 + zn - bc),
            (r0 + tp, zt + bc), (r0 - c, zt + bc)]


def arrowhead_solid(tenon, sweep, z_base, r0):
    """The half-arrowhead profile revolved about Z — SHARED by the lid
    bayonet (at the drum bore) and the frame_top↔bottom arc joints (at
    the beam-face radius, see frame.py). Upright orientation (root at
    z_base, tip up); consumers mirror/translate as their print needs."""
    return (cq.Workplane("XZ").polyline(_topj_pts(tenon, z_base, r0)).close()
            .revolve(sweep, (0, 0), (0, 1)))


def _topj_solid(tenon, sweep, z_base):
    return arrowhead_solid(tenon, sweep, z_base, _R0)


def _envelope(sweep):
    """The entry-pocket cutter: the cavity's full radial extent as a
    vertical-walled slot, same depth — the tenon (flare included) passes
    straight through in z."""
    d = _TOPJ_TOP + JOINT_BACK_CLR
    return (cq.Workplane("XZ")
            .polyline([(_R0 - JOINT_CLR, -2.0), (_R0 + _TOPJ_RD, -2.0),
                       (_R0 + _TOPJ_RD, d), (_R0 - JOINT_CLR, d)])
            .close().revolve(sweep, (0, 0), (0, 1)))


def lid_tenon(site_deg):
    """One HANGING tenon for the lid: v2's upright arc tenon (root sunk
    1.0 into the host) z-mirrored to hang from the lid's underside,
    seated pose centered on site_deg."""
    return (_topj_solid(True, LIDJ_TEN_SWEEP, -1.0)
            .rotate((0, 0, 0), (0, 0, 1), site_deg - LIDJ_TEN_SWEEP / 2.0)
            .mirror("XY")
            .translate((0.0, 0.0, LID_Z0)))


def wall_channel_cut(site_deg):
    """The wall-top cutter for one site: the retained CAVITY arc (seated
    tenon span + the seat clearance at its CW end — that end wall IS the
    stop) plus the entry-pocket ENVELOPE CCW-adjacent. Built in v2's
    upright frame, then z-mirrored to open at the wall's top face."""
    half = LIDJ_TEN_SWEEP / 2.0
    cav = (_topj_solid(False, LIDJ_TEN_SWEEP + _SEAT_A, -2.0)
           .rotate((0, 0, 0), (0, 0, 1), site_deg - half - _SEAT_A))
    pkt = (_envelope(LIDJ_TEN_SWEEP + 2.0 * _OVER_A)
           .rotate((0, 0, 0), (0, 0, 1), site_deg + half))
    return cav.union(pkt).mirror("XY").translate((0.0, 0.0, DRUM_Z1))
