"""Bearing caps (top/bottom) + pancake spool + anti-rotation key joining.

Caller must invoke ``apply_to_main_body(main_body)`` to add the matching
groove cuts to the main spool body — pure-function style, no side effects.
"""

import cadquery as cq

from .dimensions import (
    AXLE_D,
    BEARING_BORE, BEARING_LIP_H, BEARING_LIP_ID, BEARING_W,
    BOOL_OVERSHOOT,
    CAP_H, CAP_OD,
    HOUSING_GAP_LEVER, HOUSING_GAP_PANCAKE,
    HUB_CAVITY_D, HUB_OD,
    KEY_CLR,
    LEVER_CAP_SEAT_Z0, LEVER_CAP_SEAT_Z1,
    PANCAKE_CAP_SEAT_Z0,
    SPOKE_W,
    SPOOL_H,
    TOP_BEARING_BORE,
)
from .helpers import cyl, cone_solid, make_keys



# ────────────────────────────────────────────────────────────────────────────
# BEARING CAPS — two separate parts retaining the 608 bearings at each end.
#
# Both caps have the bearing pressed in from their INTERIOR (spring-cavity)
# face, so the 45° retention lip lives at the exterior face. They share the
# body geometry (CAP_OD × CAP_H, slip-fit into the hub's cap seat); the TOP
# cap additionally carries the source-cable pancake spool on its exterior
# side, living in the 12 mm pancake-side HOUSING gap.
#
# bearing_cap_bottom (lever side, below the spool)
# ------------------------------------------------
# Body at assembly z=0..8, inside the hub's bottom cap seat. Interior
# face at z=8 (bearing enters here from the spring-cavity side), exterior
# face at z=0 (lip ends here; faces the lever-side housing plate at
# z=-HOUSING_GAP_LEVER).
#
# bearing_cap_top (pancake side, above the spool — opposite the levers)
# ---------------------------------------------------------------------
# Body at assembly z=PANCAKE_CAP_SEAT_Z0..SPOOL_H, inside the hub's top cap seat.
# Pancake flanges + groove extend outward past SPOOL_H, with clearance to
# the pancake-side housing plate at z=SPOOL_H+HOUSING_GAP_PANCAKE.
#
# Pancake geometry (why these numbers):
#   Source cable accumulates ~4.2 wraps over 6 ft of extension (÷ main drum
#   circumference). Only ~12 mm of axial room exists between the hub top
#   and the housing plate — far less than 4.2 × 6 mm = 25 mm — so wraps
#   MUST stack radially. With the groove width tight to cable OD (6 mm +
#   0.5 clearance), the cable can't move axially; each new wrap is forced
#   to climb on top of the previous layer. Inner radius starts at
#   axle_r + 2 mm = 6 mm; 4–5 radial layers grow the stack to r ≈ 6 + 5×6
#   = 36 mm. Flange OD = 76 mm (r = 38) gives a 2 mm margin past a
#   fully-wound 5-layer stack.
#
# Printability: the outer pancake flange overhangs ~32 mm radially across
# the groove (unsupported bridge). This part needs print supports inside
# the groove (they pull out cleanly). The inner flange is supported by
# the body below it.
# ────────────────────────────────────────────────────────────────────────────

CABLE_OD                = 6.0    # source cable OD (USB-A-to-C)
PANCAKE_GROOVE_R_IN     = AXLE_D / 2 + 2.0                 #  6 mm — top of flare radius
                                                           # (smallest wrap radius)
PANCAKE_FLANGE_OD       = 88.0                             # 44 mm radius flange — supports
                                                           # the radial cable stack with margin
# No separate groove core. The cable wraps directly on the 45° flare
# surface, axially constrained between main_body's pancake_spool_flange
# (top face of main body) and the pancake_spool's upper flange.
# First wrap rests tangent to the cone at radius ~13.2 mm; subsequent
# wraps stack radially outward by 6 mm per layer.
PANCAKE_FLANGE_T_OUT    = 2.0                              # flange on cap (housing-plate side)
# Inner flange of the pancake is integrated into main_body (see
# pancake_spool_flange below) rather than being part of the cap. That
# keeps it on the spool's build-plate face during printing (no overhang)
# and shortens the cap by 2 mm — leaving 3.5 mm clearance from the cap's
# outer flange edge to the housing plate at z=SPOOL_H+HOUSING_GAP_PANCAKE.
PANCAKE_SPOOL_FLANGE_T  = 2.0
PANCAKE_SPOOL_FLANGE_ID = HUB_CAVITY_D                     # 34 — matches cap_seat ID
# Pancake on cap: groove (6.5) + outer flange (2) = 8.5 mm reaching from
# z=SPOOL_H to z=SPOOL_H+8.5; spool's pancake flange occupies z=SPOOL_H-2..SPOOL_H.

_axle_bore_d           = AXLE_D + 2.0                       # 10 mm — 1 mm radial clearance
                                                            # between cap and axle through the
                                                            # pancake section. Generous gap (the
                                                            # bearing — not this bore — fixes the
                                                            # cap concentric to the axle).

# ---- bearing_cap_bottom -----------------------------------------------------
# Post-flip assembly position: z = 0 .. CAP_H (=8). Exterior face at z=0
# (lip, narrow end, adjacent to lever-side housing plate). Interior face at
# z=CAP_H (pocket opening, adjacent to spring cavity at z=9..29).

def _build_bearing_cap_bottom():
    cap = (
        cyl(CAP_OD, CAP_H, z=0)
        # Lip z=0..1 — straight 90° shelf at ID=BEARING_LIP_ID (20). The
        # bearing's bottom outer-race rim (r=10..11) lands flat on the lip.
        .cut(cyl(BEARING_LIP_ID, BEARING_LIP_H, z=0))
        # Pocket z=1..CAP_H — bearing press-fits from the z=CAP_H face.
        .cut(cyl(BEARING_BORE, BEARING_W, z=BEARING_LIP_H))
    )
    # Anti-rotation tongue keys around the cap OD.
    return cap.union(
        make_keys(CAP_OD / 2, LEVER_CAP_SEAT_Z0, LEVER_CAP_SEAT_Z1)
    )

# ---- bearing_cap_top --------------------------------------------------------
# Carries the disc, bearing pocket, hourglass cones (cable wrap surface),
# and a short tongue cylinder for press-fitting the pancake_spool flange.
# The cones used to be part of pancake_spool, slip-fit through the cap,
# but that made the assembly fragile to install/remove without breaking.
# Fusing the cones into the cap leaves the pancake_spool as a flat disc.
#
# Post-flip assembly position: z = 43..51 (disc), z = 51..58.05
# (hourglass), z = 58.05..60.05 (24.1 mm tongue cylinder).
#
# Print orientation: disc face on build plate, hourglass + tongue
# cylinder pointing up. Outward 45° flare from the waist back to 26.1 mm
# is self-supporting on PA6-GF without supports.

# Wall thickness of the bearing-region cylinder around the bearing's
# outer race. 2 mm is comfortable for PA6-GF.
PANCAKE_BEARING_WALL_T    = 2.0
PANCAKE_BEARING_REGION_OD = BEARING_BORE + 2 * PANCAKE_BEARING_WALL_T   # 26.1 mm
# 45° hourglass cone — total height matches the original single-cone (so
# the housing-to-spool gap doesn't grow). Each half is HALF of the
# original flare height, which doubles the waist diameter from 12 mm to
# 19.05 mm. The disk-to-cone joint at the upper flange ends up at the
# full bearing-region OD (26.1 mm), and the waist (where the two halves
# meet) is thick at 19.05 mm — no more brittle thin neck.
PANCAKE_CONE_TOTAL_H      = (PANCAKE_BEARING_REGION_OD
                             - 2 * PANCAKE_GROOVE_R_IN) / 2   # 7.05 mm
# Add 1 mm per cone half (2 mm total spool-stack height) so the cable has
# more room to wind around the waist, AND so the housing's pancake side
# can grow by 2 mm to swallow the bearing-support boss flush.
PANCAKE_FLARE_EXTRA_H     = 1.0
PANCAKE_FLARE_H           = (PANCAKE_CONE_TOTAL_H / 2
                             + PANCAKE_FLARE_EXTRA_H)         # 4.525 mm per half
# Waist diameter (cable wrap surface) — halving the cone half-height
# halves the radial taper, so the waist is BEARING_REGION_OD − 2·HALF.
PANCAKE_WAIST_OD          = (PANCAKE_BEARING_REGION_OD
                             - 2 * PANCAKE_FLARE_H)           # 19.05 mm
# Press-fit tongue at the top of the cones: a short cylinder inset
# 2 mm radially from the cone-top OD. Pancake_spool slips down over it
# and bottoms out on the 2 mm-wide annular ledge between the cone top
# (26.1 OD) and the tongue (22.1 OD). Anti-rotation keys live on this OD.
PANCAKE_FIT_OD            = 22.1
PANCAKE_FIT_H             = 2.0
# Diametral slip-fit clearance for the pancake_spool's central hole —
# hole is PANCAKE_FIT_OD + PANCAKE_FIT_CLR, so the spool drops over the
# tongue with 0.1 mm diametral play. The keys lock rotation, so an
# interference fit on the cylinder isn't needed and just makes the spool
# hard to seat/remove without damage.
PANCAKE_FIT_CLR           = 0.3   # 0.15 mm per side

_top_cap_body_z0 = PANCAKE_CAP_SEAT_Z0          # 43 — body interior face (= main body cap seat)
_top_cap_body_z1 = _top_cap_body_z0 + CAP_H  # 51 — body exterior face (= main body top face)

pancake_bearing_z0 = _top_cap_body_z0                            # 43
_pancake_flare_z0   = _top_cap_body_z1                            # 51 — bottom of taper-in
_pancake_waist_z    = _pancake_flare_z0 + PANCAKE_FLARE_H         # 54.525 — cone waist (cable wrap)
_pancake_flare_z1   = _pancake_waist_z  + PANCAKE_FLARE_H         # 58.05 — top of taper-out
_pancake_lip_z0     = pancake_bearing_z0 + BEARING_W             # 50
_pancake_fit_z0     = _pancake_flare_z1                           # 58.05 — base of tongue cylinder
_pancake_fit_z1     = _pancake_fit_z0 + PANCAKE_FIT_H             # 60.05 — top of tongue
_pancake_flange_z0  = _pancake_fit_z0                             # 58.05 — bottom of pancake_spool flange
_pancake_flange_z1  = _pancake_flange_z0 + PANCAKE_FLANGE_T_OUT   # 60.05 — top of pancake_spool

# ────────────────────────────────────────────────────────────────────────────
# Cable transit cuts: provide a path for the source cable from the drum-
# interior side (where it transitions through the existing drum-wall slot)
# UP through the hub wall + cap, exiting on the cap top into the cone-base
# region for the helical pancake wrap. Both cuts are at azimuth 180° (the
# spoke that the existing drum-wall cable slot also hugs).
# ────────────────────────────────────────────────────────────────────────────
PATH_CUT_SHORT_AXIS         = 7.0   # tangential (Y) extent — matches existing CABLE_HOLE_SHORT_AXIS
PATH_CUT_HUB_PENETRATION    = 5.0   # mm of bearing_cap_top removed past CAP_OD/2
PATH_CUT_TANGENT_LONG       = 13.0  # tangential length of the cone-base relief
PATH_CUT_TANGENT_SHORT      = 5.0   # radial extent of the cone-base relief (kept small)
PATH_CUT_TANGENT_DEPTH      = 5.0   # how far the cone-base relief dips below cap top


def _hub_entry_cut():
    """Vertical stadium cut at azimuth 180° hugging the +y face of the
    spoke. Spans the cap height in z; runs from outside the hub
    (HUB_OD/2 + overshoot) radially inward through the hub wall and
    PATH_CUT_HUB_PENETRATION mm into the cap. Cuts both main_body and
    bearing_cap_top in their assembled positions."""
    z_min = PANCAKE_CAP_SEAT_Z0                         # 43 — flush with cap bottom (avoid stop-lip overlap)
    z_max = SPOOL_H + BOOL_OVERSHOOT                    # 51.5 — overshoot above cap top
    r_outer = HUB_OD / 2 + BOOL_OVERSHOOT               # 37   — just outside the hub
    r_inner = CAP_OD / 2 - PATH_CUT_HUB_PENETRATION     # 28.35 — 5 mm inside the cap
    return (
        cq.Workplane("YZ")
        .workplane(offset=-r_outer)                     # -X face at azimuth 180°
        .center(SPOKE_W / 2 + PATH_CUT_SHORT_AXIS / 2,  # hug +y face of spoke
                (z_min + z_max) / 2)
        .slot2D(z_max - z_min, PATH_CUT_SHORT_AXIS, angle=90)  # long along Z
        .extrude(r_outer - r_inner)                     # extrude radially in +X
    )


def _cone_base_relief_cut():
    """Stadium cut tangent to the cone-base circle at azimuth 180°. Long
    axis is parallel to the cone's bottom circle (= tangent direction at
    that point); short axis is radial. The cut dips PATH_CUT_TANGENT_DEPTH
    mm below the cap top and overshoots above, providing a notch where
    the cable enters the helical wrap channel tangentially."""
    cone_base_r = PANCAKE_BEARING_REGION_OD / 2         # 13.05
    # Stadium center placed so its inner edge sits tangent to the cone-base
    # circle, with the body of the cut in the cap material (away from axle).
    center_x = -(cone_base_r + PATH_CUT_TANGENT_SHORT / 2)
    z_min = SPOOL_H - PATH_CUT_TANGENT_DEPTH
    z_max = SPOOL_H + BOOL_OVERSHOOT
    return (
        cq.Workplane("XY")
        .workplane(offset=z_min)
        .center(center_x, 0)
        .slot2D(PATH_CUT_TANGENT_LONG, PATH_CUT_TANGENT_SHORT, angle=90)  # long along Y
        .extrude(z_max - z_min)
    )


def _build_bearing_cap_top():
    cap = (
        # Disc body — slip-fits into main_body's top cap seat.
        cyl(CAP_OD, CAP_H, z=_top_cap_body_z0)
        # Hourglass cones (cable wrap region) — fused into the cap.
        .union(cone_solid(PANCAKE_BEARING_REGION_OD, PANCAKE_WAIST_OD,
                          PANCAKE_FLARE_H, _pancake_flare_z0))
        .union(cone_solid(PANCAKE_WAIST_OD, PANCAKE_BEARING_REGION_OD,
                          PANCAKE_FLARE_H, _pancake_waist_z))
        # Tongue cylinder (24.1 OD × 2 mm) — pancake_spool flange presses on.
        .union(cyl(PANCAKE_FIT_OD, PANCAKE_FIT_H, z=_pancake_fit_z0))
        # Bearing pocket — TOP_BEARING_BORE is 0.1 mm looser than BEARING_BORE
        # so the top bearing drops in by hand.
        .cut(cyl(TOP_BEARING_BORE, BEARING_W, z=pancake_bearing_z0))
        # 45° funnel from pocket top to axle bore. Bearing outer race wedges
        # into the cone where it has narrowed to D=22.
        .cut(cone_solid(TOP_BEARING_BORE, _axle_bore_d,
                        (TOP_BEARING_BORE - _axle_bore_d) / 2,
                        _pancake_lip_z0))
        # Axle clearance bore from funnel top through the tongue cylinder.
        .cut(cyl(_axle_bore_d,
                 _pancake_fit_z1 - (_pancake_lip_z0 + (TOP_BEARING_BORE - _axle_bore_d) / 2),
                 z=_pancake_lip_z0 + (TOP_BEARING_BORE - _axle_bore_d) / 2))
    )
    # Anti-rotation tongues: cap OD into main body's top seat, AND the
    # tongue cylinder into pancake_spool's central hole.
    cap = cap.union(make_keys(CAP_OD / 2, _top_cap_body_z0, _top_cap_body_z1))
    cap = cap.union(make_keys(PANCAKE_FIT_OD / 2, _pancake_fit_z0, _pancake_fit_z1))
    # Cable transit cuts (see helpers above).
    cap = cap.cut(_hub_entry_cut())
    cap = cap.cut(_cone_base_relief_cut())
    return cap

# ---- pancake_spool ----------------------------------------------------------
# Flat 88 mm flange that press-fits down onto bearing_cap_top's 24.1 mm
# tongue cylinder. Forms the upper boundary of the source-cable groove;
# the cone surface on bearing_cap_top is the wrap surface and the
# main_body's pancake_spool_flange (z=51) is the lower boundary.
#
# Print orientation: flat — either face on build plate. Self-supporting.

def _build_pancake_spool():
    spool = (
        cyl(PANCAKE_FLANGE_OD, PANCAKE_FLANGE_T_OUT, z=_pancake_flange_z0)
        .cut(cyl(PANCAKE_FIT_OD + PANCAKE_FIT_CLR,
                 PANCAKE_FLANGE_T_OUT, z=_pancake_flange_z0))
    )
    return spool.cut(
        make_keys(PANCAKE_FIT_OD / 2, _pancake_flange_z0, _pancake_flange_z1, groove=True)
    )

# ────────────────────────────────────────────────────────────────────────────
# Anti-rotation keys (tongue & groove) at each spoke angle. Tongues run
# the full axial length of the press-fit interface, so the joint slides
# together axially with the tongue tracking the groove the whole way —
# more like wood tongue-and-groove than a stub mortise/tenon.
#
#   bearing_cap_bottom (tongue) → main_body bottom cap seat (groove)
#   bearing_cap_top    (tongue) → main_body top cap seat    (groove)
#   bearing_cap_top tongue cyl (tongue) → pancake_spool central hole (groove)
#
# Tongue: 2 mm tangential × full-axial × 1 mm radial protrusion.
# Groove: 2.2 × full-axial × 1.2 mm — KEY_CLR = 0.2 mm of slack matches the
# "too loose" fit we tested on the bearing pocket: flush enough not to
# wobble, loose enough to drop in by hand. We don't need friction here,
# just rotational lock.
# ────────────────────────────────────────────────────────────────────────────
bearing_cap_bottom = _build_bearing_cap_bottom()
bearing_cap_top    = _build_bearing_cap_top()
pancake_spool      = _build_pancake_spool()


def apply_to_main_body(main_body: cq.Workplane) -> cq.Workplane:
    """Cut the bottom + top cap groove keys into the main spool body, and
    the hub-entry slot that the source cable uses to enter the cap region.
    Returns the modified body."""
    main_body = main_body.cut(
        make_keys(HUB_CAVITY_D / 2, LEVER_CAP_SEAT_Z0, LEVER_CAP_SEAT_Z1, groove=True)
    )
    main_body = main_body.cut(
        make_keys(HUB_CAVITY_D / 2, PANCAKE_CAP_SEAT_Z0, SPOOL_H, groove=True)
    )
    # Source-cable entry — same stadium cut applied to bearing_cap_top
    # so the slot lines up across both parts in their assembled positions.
    main_body = main_body.cut(_hub_entry_cut())
    return main_body
