"""SHARED flat-top arc-joint profile + the SEPARATOR ↔ LID rotational
bayonet (both arc-joint consumers call THIS module; the coupon rule:
never re-model the joint per part).

PROFILE (user's call — replaces v2's half-arrowhead angled top, which
existed for hosts whose cavity had to close on an inclined roof): flat
inner cylindrical face, stem, ONE outboard 45° flare (the retention
lip — its 45° UNDERSIDE is the loaded face and the standing tenon's
only overhang, self-supporting), a TOPJ_TIP vertical tip face, then a
FLAT top. Every host prints it cleanly: frame_top's +z→−z print opens
its cavities at the print top (plain floors), and the lid's cavities
are THROUGH slots. Lateral faces run JOINT_CLR; the z-sandwich pairs
run JOINT_BACK_CLR (cadkit's fiber-fill policy).

SEP ↔ LID (user's design — sides SWAPPED vs the old mount-flip, so
this joint now mirrors frame_top↔frame_bottom): the SEPARATOR's drum
wall top carries STANDING tenons (its upright print grows them
library-native), and the LID carries per site a cavity arc + entry
pocket cut THROUGH its 4.8 plate (a closed ceiling would leave a 0.5
membrane over the tenon tip; the slots sit over the drum wall,
radially inside WRAP_R0, so no cable ever meets them; the lid prints
+z→−z and every slot wall is vertical or 45°). Drop the lid so the
tenons pass the entry pockets, then rotate it CW (viewed +Z) until the
tenons meet the cavities' stop walls. NOTE: back-rotation is only
friction-blocked for now — if the working torque direction lands CCW
once the cable chirality is final, add a small lock (v2 mount_lock
style).
"""

import math

import cadquery as cq

from .params import (
    NOZZLE, JOINT_CLR, JOINT_BACK_CLR,
    TOPJ_STEM, TOPJ_FLARE, TOPJ_NECK, TOPJ_TIP,
    LIDJ_TEN_SWEEP, LIDJ_SEAT_CLR, LIDJ_ENTRY_OVER,
    DRUM_IR, DRUM_T, DRUM_Z1, LID_Z0, LID_Z1, LID_T,
)

_R0 = DRUM_IR                          # tenon flat face = the drum/lid bore
TOPJ_TOP = TOPJ_NECK + TOPJ_FLARE + TOPJ_TIP     # 4.0 — tenon flat-top height
TOPJ_RD = TOPJ_STEM + TOPJ_FLARE + JOINT_CLR     # 4.15 — cavity radial reach
                                                 # outboard of the flat face

# every printed wall around the drum-wall tenons keeps the 1.6 tier
assert DRUM_T - TOPJ_RD >= 2 * NOZZLE - 1e-9, \
    "drum wall under 1.6 outboard of the lid-joint cavity — grow DRUM_T"
# the standing tenon must stay recessed below the lid's top face (its
# cavity is a THROUGH slot — nothing may poke out of the lid)
assert LID_Z1 - (DRUM_Z1 + TOPJ_TOP) >= NOZZLE - 1e-9, \
    "separator tenon tips proud of the lid's through slots"

_R_REF  = _R0 + (TOPJ_STEM + TOPJ_FLARE) / 2.0     # profile mid radius
_SEAT_A = math.degrees(LIDJ_SEAT_CLR / _R_REF)
_OVER_A = math.degrees(LIDJ_ENTRY_OVER / _R_REF)


def _flat_pts(tenon, z_base, r0, top=None):
    """(r, z) profile points at flat-face radius r0. tenon=True →
    nominal; False → the CAVITY: lateral faces dilated JOINT_CLR,
    z-sandwich faces JOINT_BACK_CLR. `top` overrides the profile's top
    (the lid uses it to run its cavities THROUGH the plate)."""
    s, f, zn, tp = TOPJ_STEM, TOPJ_FLARE, TOPJ_NECK, TOPJ_TIP
    if tenon:
        zt = TOPJ_TOP if top is None else top
        return [(r0, z_base), (r0 + s, z_base),
                (r0 + s, zn), (r0 + s + f, zn + f),
                (r0 + s + f, zt), (r0, zt)]
    c, bc = JOINT_CLR, JOINT_BACK_CLR
    zt = TOPJ_TOP + bc if top is None else top
    return [(r0 - c, z_base), (r0 + s + c, z_base),
            (r0 + s + c, zn - bc), (r0 + s + f + c, zn + f - bc),
            (r0 + s + f + c, zt), (r0 - c, zt)]


def flat_arc_solid(tenon, sweep, z_base, r0, top=None):
    """The flat-top profile revolved about Z — SHARED by the sep↔lid
    bayonet (at the drum bore) and the frame_top↔bottom arc joints (at
    the beam-face radius, see frame.py). Upright orientation (root at
    z_base, tip up); consumers rotate/translate as they need."""
    return (cq.Workplane("XZ")
            .polyline(_flat_pts(tenon, z_base, r0, top)).close()
            .revolve(sweep, (0, 0), (0, 1)))


def _pocket(sweep, top):
    """The entry-pocket cutter: the cavity's full radial extent as a
    vertical-walled slot — the tenon (flare included) passes straight
    through in z."""
    return (cq.Workplane("XZ")
            .polyline([(_R0 - JOINT_CLR, -2.0), (_R0 + TOPJ_RD, -2.0),
                       (_R0 + TOPJ_RD, top), (_R0 - JOINT_CLR, top)])
            .close().revolve(sweep, (0, 0), (0, 1)))


def sep_tenon(site_deg):
    """One STANDING tenon on the separator's drum wall top, seated pose
    centered on site_deg, root sunk 1.0 into the wall."""
    return (flat_arc_solid(True, LIDJ_TEN_SWEEP, -1.0, _R0)
            .rotate((0, 0, 0), (0, 0, 1), site_deg - LIDJ_TEN_SWEEP / 2.0)
            .translate((0.0, 0.0, DRUM_Z1)))


def lid_channel_cut(site_deg):
    """The LID cutter for one site (seated pose, THROUGH the plate):
    the retained cavity arc around the seated tenon + the seat
    clearance at its +azimuth end (the lid rotates CW to seat, so in
    the lid's frame the tenon travels CCW — that end wall IS the stop),
    plus the entry-pocket envelope CW-adjacent (where the pocket sits
    once seated: it swept CW away from the tenon during seating)."""
    half = LIDJ_TEN_SWEEP / 2.0
    through = LID_T + 2.0
    cav = (flat_arc_solid(False, LIDJ_TEN_SWEEP + _SEAT_A, -2.0, _R0,
                          top=through)
           .rotate((0, 0, 0), (0, 0, 1), site_deg - half))
    pkt = (_pocket(LIDJ_TEN_SWEEP + 2.0 * _OVER_A, through)
           .rotate((0, 0, 0), (0, 0, 1),
                   site_deg - half - LIDJ_TEN_SWEEP - 2.0 * _OVER_A))
    return cav.union(pkt).translate((0.0, 0.0, LID_Z0))
