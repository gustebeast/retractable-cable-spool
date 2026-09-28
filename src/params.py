"""v3 — shared parameters (fresh start; v2 archived in v2/).

WHY v3: v2's flat cable-storage tray FAILED its physical wind test and the
redesign around an AXIAL cable storage chamber costs a lot of Z height — so
v3 rebuilds the stack looking for Z savings everywhere else.

ARCHITECTURE CHANGE vs v2: the 608 bearing moves INTO the top frame's own
thickness (v2 spent a separate spring-housing pocket on it); the AXLE is now
the part that ROTATES (it is the spring arbor), hanging on a top lip that
rests on the bearing's inner race; the SPRING BODY will be fixed to the
frame (tackled later). The spring is used UNMODIFIED — flanges on (v2 ran
it with the flange rims cut off).

First cut: frame_top (plus spider + bearing pocket), the two-half axle, and
the 608 bearing + spring as assembly-only dummies.

Sizing policy (user's rule): every printed width/thickness is a multiple of
the 0.8 nozzle bead or referenced off an existing object; hardware dims and
small fit clearances stay absolute.
"""

import math

from cadkit.joinery import PrintSpec, joint_clearances, joint_box_min
from cadkit.contact import contact_rib_size
from cadkit.printing import min_wall, snap

NOZZLE_D      = 0.8            # THE print-width unit (0.8 nozzle, v2's call)
STRUCT_WALL = min_wall(NOZZLE_D, beads=2)   # 1.6 — the two-bead quality tier
FIT_CLR     = 0.15           # per-side slip-fit clearance (v2's proven value)
# THE joint print policy — one spec, one clearance pair, defined at the
# top so every joint block below can derive from it (moved up at #932;
# the T-joint history comment lives at the old site)
JOINT_SPEC = PrintSpec(nozzle=NOZZLE_D, material="PETG-GF", facing="up")
JOINT_CLR, JOINT_BACK_CLR = joint_clearances(JOINT_SPEC, JOINT_SPEC)  # 0.15/0.30

# ── 608 bearing (purchased part — dummy in the assembly, no STEP) ────────────
BEARING_OD = 22.0            # outer race Ø
BEARING_ID = 8.0             # inner race bore
BEARING_W  = 7.0             # axial width
BEARING_CLR = 0.2            # diametral pocket clearance (0.1/side, v2-proven)
BEARING_ORACE_ID = 19.4      # outer-race face annulus ID (seal line, v2-measured)
BEARING_IRACE_OD = 12.1      # inner-race face annulus OD (v2-measured)
BEARING_INNER_CH = 0.3       # 608 ring-edge chamfer (spec r_s min ≈ 0.3, both
                             # rings) — v2's cone-seat trick face-mates it

# ── Spring (purchased part, UNMODIFIED — dummy in the assembly, no STEP) ─────
SPRING_FLANGE_OD = 78.3      # flange Ø, top and bottom
SPRING_FLANGE_T  = 2.4       # flange thickness, each
SPRING_BODY_OD   = 56.3      # coil body Ø between the flanges
SPRING_H         = 32.0      # TOTAL height, flanges included
# The spring installs UPSIDE DOWN in v3 (user 2026-08-05: the axle spins
# instead of being fixed, which flips the required wind chirality) — so
# the big Ø13.4 hole is on TOP now and the small Ø8.7 on the BOTTOM:
# the MORTISE collar therefore rides the TOP axle half (enters from
# above) and the separator carries the TENON post (the sexes swapped).
SPRING_BORE_TOP_D = 13.4     # top-flange hole — the mortise collar enters here
SPRING_BORE_BOT_D = 8.7      # bottom-flange hole (user-measured) — only the
                             # bare Ø7.95 tenon post passes; the collar CAN'T
SPRING_STRIP_T   = 0.2       # strip steel thickness (v2-measured)
SPRING_STRIP_W   = 22.0      # strip width (v2-measured)

# ── Cable (hardware) ─────────────────────────────────────────────────────────
CABLE_D        = 3.3         # working-cable Ø (user's measurement,
                             # 3.85 → 3.3 at #952; was 3.0 before #883)
CABLE_CAPACITY = 7 * 304.8   # 7 ft / 2133.6 of wrapped cable (user's call
                             # at #952; was 1 m since #884, 6 ft before)

assert CABLE_D - 3 * NOZZLE_D >= NOZZLE_D - 1e-9, \
    "floor viewing slits (FLOOR_SLIT_W) are wide enough to swallow the cable"

# ── Axle (rotating arbor) ────────────────────────────────────────────────────
AXLE_D       = 8.0                   # nominal — the 608's bore
AXLE_PRINT_D = AXLE_D                # 8.0 — printed AT the 608's nominal bore
                                     # (user: v2's 7.95 measured a touch loose
                                     # in both bearings — top shaft AND stub)
# (the retention LIP/head is RETIRED — user, post-swap: with the mortise
# collar at the axle's bottom, a head on top left NO installable
# direction through the bearing. The shaft is now plain: axle_top
# installs from BELOW — up through the spring and into the 608 — and
# the rotating stack's weight rides the BOTTOM bearing via the
# separator; the top shaft only locates radially.)

# ── Frame top — v2's plus spider, now hosting the bearing ────────────────────
FRAME_RIB   = 13 * NOZZLE_D    # 10.4 — plus-arm section (was v2's 10.0 even;
                             # user's call: onto the bead grid)
# (FRAME_R_OUT is DERIVED in the wall/beam block below: arm tips flush
# with the beams' outer faces, v2's pattern.)
FRAME_Z0 = 0.0               # frame underside — the v3 z datum
FRAME_Z1 = FRAME_Z0 + FRAME_RIB

# ── COUCH VERSION (user #954): the couch's front beam (COUCH_* block
# below) catches the ratchet lever at ~half its pull; the fix is to hang
# the whole assembly LOWER off the wood. frame_top alone grows a slab on
# its +z end — the mount channels, anchor wall, bearing pocket and lock
# slot follow the new top face; every other printed part is untouched
# (the mount ring simply seats COUCH_TOP_PAD higher in the assembly).
# 13 beads = 10.4 (the user's 10 snapped up onto the grid; the lever
# needs ≥ 7.0 to clear at 20°). False = the desk build, pad 0.
COUCH_VERSION = True
COUCH_TOP_PAD = 13 * NOZZLE_D if COUCH_VERSION else 0.0   # 10.4 / 0
FRAME_TOP_Z1  = FRAME_Z1 + COUCH_TOP_PAD   # frame_top's ACTUAL top face
                                           # (the wood / mount plane)

# Bearing pocket: drops in from the TOP onto a lip UNDER the outer race —
# the hanging load (axle + spring + future drum) pulls the bearing DOWN
# onto it. The lip is the quality tier, which RECESSES the bearing 1.8
# below the frame top so the axle's lip tops out UNDER the frame face:
# the assembly's top is FLAT (user's call — was lip 3.4 / bearing flush /
# axle lip 1.6 proud).
BRG_BORE      = BEARING_OD + BEARING_CLR       # 22.2 — pocket bore
BRG_LIP_H     = 2 * NOZZLE_D                     # 1.6 — retention lip height
BRG_POCKET_Z0 = FRAME_Z0 + BRG_LIP_H           # 1.6 — pocket floor (lip top)
BRG_Z1        = BRG_POCKET_Z0 + BEARING_W      # 8.6 — bearing top (recessed)
BRG_LIP_ID    = BRG_BORE - 2 * STRUCT_WALL     # 19.0 — retention lip bore (laps
                                               # the outer race; grazes the seal
                                               # annulus by 0.2 — v2-accepted: the
                                               # seal sits on the outer ring, no
                                               # relative motion at that contact)
BRG_BOSS_WALL = 4 * NOZZLE_D                     # 3.2 — hoop wall around the pocket
BRG_BOSS_OD   = BRG_BORE + 2 * BRG_BOSS_WALL   # 28.6 — central boss Ø

# The seat is a 45° FUNNEL cone (user: no overhangs — the old flat
# lip-top printed as the ~1.3-wide bridge ring; in the +z→−z print the
# funnel is a 45° narrowing taper, fully self-supporting, the
# bottom-608 lesson: the cone IS the seat): it flares from the lip
# bore up-and-out to the pocket bore, and the outer race's factory 0.3
# bottom-OD chamfer lands face-on-face on it, 0.1 deeper than nominal
# (the bottom seat's exact allowance — the dummy sits at nominal, the
# real bearing 0.1 low, which only GROWS the axle-lip↔frame-face
# margin).
BRG_SEAT_CONE_Z0 = (BRG_POCKET_Z0 - 0.1
                    - (BEARING_OD / 2.0 - BEARING_INNER_CH
                       - BRG_LIP_ID / 2.0))    # 0.3 — funnel base at the lip bore
BRG_SEAT_CONE_Z1 = (BRG_SEAT_CONE_Z0
                    + (BRG_BORE - BRG_LIP_ID) / 2.0)   # 1.9 — funnel rim at the
                                               # pocket bore (45° by construction)
assert BRG_SEAT_CONE_Z0 >= FRAME_Z0 + 0.2 - 1e-9, \
    "funnel seat undercuts the lip bore's root ring"

# ── Z stack below the frame ──────────────────────────────────────────────────
# The spring body will be FIXED to the frame later — modelled touching.
SPRING_Z1 = FRAME_Z0                   # top flange against the frame underside
SPRING_Z0 = SPRING_Z1 - SPRING_H       # −32

# ── Axle two-half glue joint + spring-strip slit (v1/v2-proven design:
# the joint's fat collar can only pass the spring's Ø13.4 flange hole,
# so the axle splits and each half slides onto the strip from its own
# end. SEXES SWAPPED at #878 — the flipped spring puts the Ø13.4 hole
# on TOP: the TOP half carries the MORTISE collar now, the separator
# carries the TENON post) ────────────────────────────────────────────────────
# The joint's Z DATUM pair (user's rule, converged over #874-877): the
# spring's strip physically starts 4 in from each spring edge; the
# slits' closed ends sit SLIT_INSET = 3.2 in (0.8 of shrinkage margin
# past the strip's ends); and each half's joint-side EXTENT lands
# exactly ON the other half's slit-end plane — the parts LINE UP, no
# slit window is ever exposed past the seated mating half:
#   · axle_top's collar bottom (mortise opening) = the separator's
#     slit floor (SLIT_ZLO)
#   · separator's tenon tip = axle_top's slit ceiling (SLIT_ZHI)
SLIT_INSET = 4 * NOZZLE_D            # 3.2 — slit-end inset from the spring edges
SLIT_ZLO   = SPRING_Z0 + SLIT_INSET   # −28.8 — sep slit floor = collar bottom
SLIT_ZHI   = SPRING_Z1 - SLIT_INSET   # −3.2 — top slit ceiling = tenon tip
STRIP_Z0   = SPRING_Z0 + 4.0       # −28 — strip physical lower end (reference)
STRIP_Z1   = SPRING_Z1 - 4.0       # −4  — strip physical upper end (reference)
JOINT_Z1   = SLIT_ZHI              # bore ceiling = tenon tip
JOINT_Z0   = SLIT_ZLO              # mortise opening = collar bottom
JOINT_H    = JOINT_Z1 - JOINT_Z0   # 25.6 — bore depth = engagement
assert SLIT_ZLO <= STRIP_Z0 and STRIP_Z1 <= SLIT_ZHI, \
    "slits no longer cover the strip's physical span"
JOINT_MORTISE_HOLE_D = AXLE_PRINT_D + 2 * FIT_CLR  # 8.25 — female bore
JOINT_MORTISE_OD     = JOINT_MORTISE_HOLE_D + 2 * STRUCT_WALL  # 11.45 — the
                                                   # TOP half's collar (and the
                                                   # separator's base column OD)
# the TOP half's shaft→collar 45° transition (print-safe both ways: in
# the flipped print the cone widens going up); it lives inside the top
# flange's Ø13.4 hole zone
TOP_CONE_Z1 = -0.5                 # cone top (shaft Ø) — under the frame face
TOP_CONE_Z0 = TOP_CONE_Z1 - (JOINT_MORTISE_OD - AXLE_PRINT_D) / 2.0  # −2.25 —
                                                   # full collar from here down
assert TOP_CONE_Z0 >= SPRING_Z1 - SPRING_FLANGE_T - 1e-9, \
    "collar cone reaches below the top flange's hole zone"
assert JOINT_Z1 <= TOP_CONE_Z0 - NOZZLE_D + 1e-9, \
    "no web between the bore ceiling and the collar cone"
# the separator's column→tenon SHOULDER: the fat column cannot pass the
# (now bottom) Ø8.7 hole, so it stops under the spring with a rotating gap
TENON_SHOULDER_CLR = 0.5
SHOULDER_Z = SPRING_Z0 - TENON_SHOULDER_CLR        # −32.5 — column top / post root
SLIT_W               = 2 * NOZZLE_D                  # spring-strip slit width
# (SLIT_END_CAP is RETIRED — user's call reversed: the spring's central
# strip is FIXED and cannot thread a closed window, so each half's slit
# is FULLY OPEN at its joint-side end and the halves SLIDE ONTO the
# strip — v2's design: separator open through its +z column top,
# axle_top open through its tenon bottom.)
assert SLIT_ZHI <= SPRING_Z1 - 1e-9, \
    "top-axle slit would poke out of the spring (shaft must stay solid " \
    "through the bearing above)"

# ── Separator (brake + ratchet rims), FUSED to the bottom axle ───────────────
# v2's standalone sliding rim, ported: now a SOLID disk (no bore, no key
# slots — it turns with the axle it's fused to), same two rims. Band ORDER
# flipped vs v2 (brake bottom / ratchet top): the fused part prints UPRIGHT
# (disk on the bed, column rising), so the bed-side band must be the full
# ring — exactly v2's proven print stack (v2 printed inverted, brake band
# on the bed, teeth growing on the knurl ring).
# (RIM_OD — the separator/lid OD — is DERIVED in the capacity block after
# the drum wall below: it must cap 6 ft of flat-wound cable.)
RATCHET_DEPTH = 4 * NOZZLE_D         # 3.2 — tooth tip→valley (user #1005:
                                   # a bead deeper than v2's print-proven
                                   # 2.4, for a firmer catch)
RATCHET_TEETH = 60
# (RATCHET_PHASE_DEG — the tooth/knurl phase — is DERIVED in the lever
# block below: one catch face lands at the pawl's y-midline, v2's rule.)
BRAKE_H       = 6 * NOZZLE_D         # 4.8 — knurled brake band height (was
                                   # v2's 5.0 even; user: onto the bead grid)
RATCHET_H     = 6 * NOZZLE_D         # 4.8 — tooth band height (ditto)
RIM_H         = BRAKE_H + RATCHET_H            # 10.0
KNURL_N       = 5 * RATCHET_TEETH  # 300 — knurl lobes, phase-locked 5 per
                                   # tooth so a peak supports every tip
KNURL_DEPTH   = 0.4                # half a nozzle (v2 print-proven ridge field)

# 45° cable PASS-THROUGH tunnel (v2's, kept by user's call): lets the
# working cable (connector included) cross from above the separator to
# below it. Tilted tangentially at 45°, so it prints support-free either
# way up; tilt sense follows the CW wind (v2's finding — a CCW climb
# kinked the wrap against the ratchet's freewheel sense).
SEP_PASS_W         = 13.0          # square tunnel section (connector passes)
SEP_PASS_ANGLE_DEG = 30.0          # azimuth (v2's spot; arbitrary now — the
                                   # key slots it dodged are gone)
# (SEP_PASS_R is derived in the drum block below — the mouth sits just
# outside the drum wall, where the innermost wrap dives.)

SEP_SPRING_CLR = 3 * NOZZLE_D        # 2.4 — separator↔spring gap (was v2's 3.2
                                   # static↔moving standard; user: overkill)
# (AXLE_ROOT_CONE_H is set right below — it needs SEP_Z1)
SEP_Z1 = SPRING_Z0 - SEP_SPRING_CLR    # −34.4 — disk top, right under the
                                       # (frame-fixed, static) spring
SEP_Z0 = SEP_Z1 - RIM_H                # −44.4 — disk bottom = the bed face
AXLE_ROOT_CONE_H = SHOULDER_Z - SEP_Z1 # 1.9 — 45° column→disk root cone: its
                                       # top IS the column→tenon shoulder now
                                       # (the fat column can't approach the
                                       # spring's bottom face anymore — the
                                       # Ø8.7 hole is there, sexes swapped)

AXLE_Z_BOT = SEP_Z0                    # the column runs flush through the disk
AXLE_Z_TOP = BRG_Z1 + 2 * NOZZLE_D       # 10.2 — shaft top, 1.6 proud of the
                                       # bearing, UNDER the frame face (the
                                       # flipped print's bed face)

# ── Spring-strip TENSION-TRAP anchor (user's v3 design — retires the ported
# v2 two-block gate + drop-in insert; zero extra parts) ──────────────────────
# A short wall hangs under the +X arm (running up THROUGH the frame as an
# arm rib tying it into the plate) with one WINDOW at the strip's z: the
# T-head pushes straight through the tall +y end FLAT-ON (no twist),
# then the spring's one-signed pull drags the strip to the −y bay, too
# short for the head to escape — the spring's own tension is the latch
# (user's insight: the pull direction never reverses, and the install's
# 1.5–2 pre-wraps mean it never goes slack in service).
# Strip tip, user-measured (same spring as v2; hardware = absolute dims):
# 0.2 thick; 22-wide body, 13-long taper to the 9-wide × 8-long neck, then
# the 7-long × 16-wide round T-head (4 root notch, unused here).
STRIP_BODY_W  = 22.0               # full strip width (across = z)
STRIP_TAPER_L = 13.0               # body → neck taper length
STRIP_NECK_W  = 9.0                # neck width (across = z)
STRIP_NECK_L  = 8.0                # neck length (along the strip)
STRIP_HEAD_W  = 16.0               # T-head width (across = z)
STRIP_HEAD_L  = 7.0                # T-head length (along the strip)

ANCH_FLANGE_CLR = 1.0              # wall face off the Ø78.3 flange edges
                                   # (was 1.8 — user gave 0.8 of it to the
                                   # wall; the OUTER face stays on the
                                   # proven 42.55, drum chain unmoved)
ANCH_IR = SPRING_FLANGE_OD / 2.0 + ANCH_FLANGE_CLR   # 40.15 — wall inner face
ANCH_T  = 3 * NOZZLE_D               # 2.4 — wall thickness (user: up from the
                                   # gate wall's 1.6)
ANCH_OR = ANCH_IR + ANCH_T                           # 42.55 — same as before

# WINDOW — the right-trapezoid TENSION TRAP (print-driven, user's rule
# for this +z→−z part: the opening's +z boundary must be a FLAT edge
# (it prints as a plain floor face) and the −z boundary a 45° — the
# roof of the hole as printed. So: flat top edge, ONE 45° floor
# descending from the −y bay corner to the +y entry, DULLED to a
# one-bead flat at its deepest point (no knife-edge void where the
# roof closes in print).
# CHIRALITY (user): pull-out spins the axle CW (viewed +z) and the
# spring TIGHTENS; strip winding onto the arbor CW means the strip runs
# CCW going outward, so it arrives at the +X anchor travelling toward
# +y — its tension drags the free end toward −Y. So: the tall ENTRY
# wall sits at +y (PASS — the head enters flat-on, no twist, hugging
# that wall), the floor climbs at 45° toward −y, and the HOLDING BAY
# at −y (HOLD — the 16 head cannot pass 12; escape means travelling
# back AGAINST the spring's pull). The width is FORCED at PASS − HOLD:
# the 45° floor must climb the full entry-to-bay height difference.
# The BAY is SNUG on the neck now (user #879: the old 12 bay was "too
# long to make a trustworthy seal" — bay height = the 9 neck), and the
# bay's z DERIVES from the strip's feasible float (a snug bay must sit
# where the neck can actually ride): bay centre ANCH_BAY_BIAS below the
# strip's highest feasible centreline. The head still threads the tall
# entry within the same float band (asserted).
# the strip's feasible z-band inside the spring (22 body between the
# flange inner faces):
_STRIP_C_LO = SPRING_Z0 + SPRING_FLANGE_T + STRIP_BODY_W / 2.0   # −18.6
_STRIP_C_HI = SPRING_Z1 - SPRING_FLANGE_T - STRIP_BODY_W / 2.0   # −13.4
ANCH_WIN_HOLD = STRIP_NECK_W       # 9.0 — bay height = the neck (user:
                                   # snug by design, a trustworthy seat)
ANCH_WIN_PASS = 23 * NOZZLE_D        # 18.4 — entry-wall height (head-passing)
ANCH_BAY_BIAS = 0.3                # bay centre bias below the float's top
ANCH_WIN_Z1  = (_STRIP_C_HI + ANCH_WIN_HOLD / 2.0
                - ANCH_BAY_BIAS)                     # −9.2 — flat top edge
ANCH_WIN_Z0  = ANCH_WIN_Z1 - ANCH_WIN_PASS           # −27.6 — the VIRTUAL
                                   # apex at the +y entry wall: the 45°
                                   # floor aims here, but the tip is
                                   # DULLED (user's call) —
ANCH_TIP_Z   = ANCH_WIN_Z0 + NOZZLE_D                  # −26.8 — a 0.8 flat
                                   # truncates the corner: one bead
                                   # bridges it in print
ANCH_HOLD_Z0 = ANCH_WIN_Z1 - ANCH_WIN_HOLD           # −18.2 — bay bottom
ANCH_WIN_W   = ANCH_WIN_PASS - ANCH_WIN_HOLD         # 9.4 — width: 45° floor
                                                     # bay-corner → virtual apex
                                                     # (wider with the 9 bay —
                                                     # the wall grows with it)
ANCH_Z0      = ANCH_TIP_Z - 4 * NOZZLE_D               # −30.0 — wall bottom
                                   # (3.2 sill under the dulled tip)
# the snug bay's centre must sit INSIDE the strip's float band
assert (_STRIP_C_LO + ANCH_BAY_BIAS - 1e-9
        <= ANCH_WIN_Z1 - ANCH_WIN_HOLD / 2.0
        <= _STRIP_C_HI - ANCH_BAY_BIAS + 1e-9), \
    "snug holding bay unreachable by the neck within the strip's float"
assert (min(_STRIP_C_HI, ANCH_WIN_Z1 - STRIP_HEAD_W / 2.0 - 0.2)
        - max(_STRIP_C_LO, ANCH_TIP_Z + STRIP_HEAD_W / 2.0 + 0.2)
        >= 0.5 - 1e-9), \
    "no feasible strip z threads the head through the entry"
# PLACEMENT (user #879: "the force is all applied at the short side —
# line that up with the beam"): the HOLD edge sits at the +X arm's +y
# face, so the loaded pier — the wall just −y of the bay, which takes
# the strip's whole −y pull — is backed by the arm's full 10.4
# footprint. The window runs +y from there; the wall sector is
# ASYMMETRIC: from the arm's −y face out past the entry wall + a
# 6-bead pier.
ANCH_WIN_Y0 = FRAME_RIB / 2.0      # 5.2 — HOLD edge, on the beam's +y face
ANCH_A_NEG = math.degrees(math.asin((FRAME_RIB / 2.0) / ANCH_IR))    # 7.4°
ANCH_A_POS = math.degrees(math.asin(
    (ANCH_WIN_Y0 + ANCH_WIN_W + 3 * NOZZLE_D) / ANCH_IR))              # ≈ 25.1°
                                   # (+y pier 3 beads — 6 put the wing at
                                   # 28.9° where a mount SPOKE clipped it
                                   # at the install-overshoot margin; the
                                   # LOADED pier is the beam-backed side)

# The pass/block mechanism IS these inequalities:
assert ANCH_WIN_Z1 - ANCH_TIP_Z >= STRIP_HEAD_W + 0.4 - 1e-9, \
    "head doesn't pass flat-on at the entry wall (tip flat included)"
assert (NOZZLE_D + (ANCH_WIN_Z1 - ANCH_TIP_Z) - (STRIP_HEAD_W + 0.4)
        >= 2 * NOZZLE_D - 1e-9), \
    "entry zone too narrow to thread the head by hand"
assert ANCH_WIN_HOLD <= STRIP_HEAD_W - 3.0 + 1e-9, \
    "head escapes the holding bay"
assert ANCH_WIN_HOLD >= STRIP_NECK_W - 1e-9, \
    "holding bay smaller than the neck (snug-by-design is the floor, user)"
assert STRIP_NECK_L >= ANCH_T + 1.0 - 1e-9, \
    "neck too short — the head can't fully clear the wall"
assert ANCH_WIN_Z1 <= SPRING_Z1 - SPRING_FLANGE_T - 0.5 + 1e-9, \
    "window reaches the spring's top flange"
assert ANCH_WIN_Z0 >= SPRING_Z0 + SPRING_FLANGE_T + 0.5 - 1e-9, \
    "window reaches the spring's bottom flange"
assert ANCH_Z0 - SEP_Z1 >= 3 * NOZZLE_D - 1e-9, \
    "anchor wall bottom reaches the separator disk"

# ── Drum wall + lid — the cable-wrap core and the spool chamber's ceiling ────
# The DRUM WALL rises from the separator disk, wrapping the anchor wall's
# radius with DRUM_ANCH_CLR (user's 0.8); the cable wraps IT. The LID seats on the
# wall's top rim via a ROTATIONAL bayonet (user's design; sides SWAPPED
# to the frame-joint arrangement): the WALL TOP carries STANDING
# flat-top arc tenons (the separator's upright print grows them
# library-native, flare underside at 45°), and the LID carries per site
# a THROUGH cavity slot + entry pocket (lid_joint.py — the 4.8 plate
# can't cap the tenon with a printable ceiling, and the slots sit over
# the drum wall, inboard of the cable). Drop the lid over the tenons,
# rotate CW to the end-wall stops.
DRUM_ANCH_CLR = NOZZLE_D             # 0.8 — drum bore off the anchor wall (user)
DRUM_IR = ANCH_OR + DRUM_ANCH_CLR              # 43.35 — wall bore = lid bore
DRUM_T  = 8 * NOZZLE_D               # 6.4 — X = 8: the smallest bead multiple
                                   # that swallows the joint cavity (4.3 with
                                   # clearances) and keeps ≥ 1.6 outboard of
                                   # it (asserted in lid_joint.py; 7 beads
                                   # would leave 1.3)
DRUM_OR = DRUM_IR + DRUM_T                     # 49.75

# ── Capacity → separator/lid OD (v2's one-knob pattern) ─────────────────────
# The cable winds ONE FLAT LAYER (the 4.0 chamber) spiralling out from the
# drum wall; CABLE_CAPACITY sets how far out, and the disk/lid must cap
# the coil. Archimedean turns solved from L = 2π·n·r0 + π·p·n·(n−1).
WRAP_R0    = DRUM_OR + CABLE_D / 2.0           # 51.25 — innermost wrap centre
COIL_PITCH = CABLE_D + 0.1                     # 3.1 — wrap-on-wrap pitch
_B = 2.0 * math.pi * WRAP_R0 - math.pi * COIL_PITCH
N_WRAPS = ((-_B + math.sqrt(_B ** 2 + 4.0 * math.pi * COIL_PITCH
                            * CABLE_CAPACITY))
           / (2.0 * math.pi * COIL_PITCH))     # ≈ 5.06 turns for 6 ft
R_OUT_COIL = WRAP_R0 + N_WRAPS * COIL_PITCH    # outermost wrap centre
# separator/lid OD: coil + 1.0 cover margin — FLOORED by the connector
# pass tunnel (at the 1 m capacity the coil-derived rim would shrink
# below what the Ø13 tunnel needs between the drum wall and the tooth
# band root; the disk keeps that minimum and simply carries spare coil
# room — the AXIAL stack still shrinks with capacity, the real win)
_RIM_COIL = 2.0 * (R_OUT_COIL + CABLE_D / 2.0 + 1.0)
# the tunnel's TRUE plan envelope (user-caught at #896: a 45°-tilted
# tube's plan corners swing wider than its mouth — the old mouth-width
# floor left only 1.0 to a tooth root): the corner edges reach
# hypot(R_outer_face, W·√2/2 + RIM_H/2); the tunnel sits FLUSH on the
# drum wall now (user), and ≥1.6 must survive to the tooth roots
_RIM_PASS = 2.0 * (math.hypot(DRUM_OR + SEP_PASS_W,
                              SEP_PASS_W * math.sqrt(2.0) / 2.0
                              + RIM_H / 2.0)
                   + RATCHET_DEPTH + 2 * NOZZLE_D)
RIM_OD = max(_RIM_COIL, _RIM_PASS)

LID_SEP_GAP = snap(CABLE_D, NOZZLE_D, "up")   # 4.0 — separator↔lid gap = the spool
                                   # chamber height: one flat cable layer,
                                   # the Ø ROUNDED UP to the next bead
                                   # multiple (user's rule at #885 — was a
                                   # hardcoded 4.0 that silently lost its
                                   # slack when the cable grew to 3.85;
                                   # derived, it tracks the cable)
LID_T   = 6 * NOZZLE_D               # 4.8 — user's stated minimum; it can stay
                                   # there because the cavities live in the
                                   # WALL, not the lid
LID_Z0  = SEP_Z1 + LID_SEP_GAP                 # −30.4 — lid underside
LID_Z1  = LID_Z0 + LID_T                       # −25.6 — lid top
# the anchored strip's T-head pokes past the wall to r ≈ 55, over the
# rotating lid's rim — its bottom edge must clear the lid's top face
# both SEATED (the bay floor sets the lowest ride) and mid-THREAD (the
# dulled tip sets the lowest transient dip):
assert ((max(_STRIP_C_LO, ANCH_HOLD_Z0 + STRIP_NECK_W / 2.0)
         - STRIP_HEAD_W / 2.0) - LID_Z1 >= 2 * NOZZLE_D - 1e-9), \
    "the seated strip's protruding head reaches the rotating lid"
# (the old threading-vs-lid assert is RETIRED — the snug 9 bay pushes
# the window's tip below the lid's top plane, and that's fine by
# ASSEMBLY ORDER: the strip threads the anchor on the BENCH, spring +
# frame_top alone, before frame_top ever meets the lid. Only the
# SEATED strip must clear the rotating lid — asserted above.)
DRUM_Z1 = LID_Z0                   # wall top — the seated lid rests on it;
                                   # the wall is a short 4-tall collar now,
                                   # and the joint cavities burrow on
                                   # through it into the disk's thickness
                                   # (one solid — backed by lid_joint's
                                   # depth assert)
LID_OD  = RIM_OD                   # lid rim tracks the separator

# the FLAT-TOP arc profile (v2's TOPJ_* dims kept; the arrowhead's
# angled top DROPPED — user's call at the frame_top flip: with both
# host plates printing their cavities as short flat ceilings or through
# slots, a flat tenon top prints fine on every side)
TOPJ_STEM  = 3 * NOZZLE_D            # 2.4 — stem width (radial, off the flat)
TOPJ_FLARE = 2 * NOZZLE_D            # 1.6 — the single outboard 45° flare
TOPJ_NECK  = 2 * NOZZLE_D            # 1.6 — neck height under the flare
TOPJ_TIP   = NOZZLE_D                # 0.8 — vertical tip face above the flare
                                   # (then the FLAT top)
LIDJ_SITES      = 4                # joint sites (90° pitch)
LIDJ_TEN_SWEEP_DEG  = 30.0             # tenon arc sweep (deg) — was 20 (user:
                                   # lid↔sep too loose; the drum rim has
                                   # free arc, so the engagement grew 1.5×)
LIDJ_SEAT_CLR   = JOINT_CLR        # seating clearance at each stop (arc
                                   # mm — tracks the material policy,
                                   # #932)
LIDJ_ENTRY_OVER = 1.0              # pocket overshoot past the tenon, each
LIDJ_Z_CLR      = 0.20             # z-sandwich clearance, THIS joint only
                                   # (user: drop the vertical slack a bit —
                                   # under the GF 0.30 depth-face policy;
                                   # print-check the seating twist for grab)
# ── the seating WEDGE (user #1017: the lid still sat a touch loose) ──────────
# The lid's cavity NARROWS radially over the last stretch of its arc, so
# the tenon is pinched between the lid's own bore wall and the cavity's
# outboard wall as the lid twists home — the pin's tapered key, in the
# one direction this joint can hold a ramp accurately. (A ramp in Z would
# be quantised by the layer height: the whole gap here is about one
# layer, so it would print as a detent, not a wedge. Radially the ramp is
# XY motion, which the printer places to a few hundredths.)
# Both walls close, so the tenon is centred, not shoved off its radius;
# the cavity's 45° face comes in with the outboard wall, which also draws
# the lid DOWN onto the drum rim and takes the flare's z slack with it.
LIDJ_CAM_W      = 0.40             # total narrowing, 0.20 a side: the 0.15
                                   # clearance plus 0.05 of squeeze
LIDJ_CAM_DEG    = 6.0              # closed over this much arc — 0.033/deg,
                                   # a 2.4° ramp at this radius, well under
                                   # the ~20° PETG friction angle, so it
                                   # holds the lid against back-rotation
                                   # (which was friction-only before)
LIDJ_CAM_RUN    = 3.0              # and the arc runs on this far PAST the
                                   # old stop wall, so the WEDGE stops the
                                   # lid, not a wall — the same lesson as
                                   # the pin's blind bore (#1014): a hard
                                   # stop short of the wedge means print
                                   # error lands on the FIT. ±3° here is
                                   # ±0.10 a side of print error absorbed
assert LIDJ_CAM_W / 2.0 > JOINT_CLR, "the lid's cam never reaches the tenon"
assert LIDJ_CAM_DEG + LIDJ_CAM_RUN < LIDJ_TEN_SWEEP_DEG, \
    "the lid's cam is longer than the tenon it wedges"
                                   # side (arc mm) — drop-in ease

# 45° pass tunnel: FLUSH on the drum wall (user #896 — the old one-bead
# web served nothing; inward is friendlier to the tooth-root keep-out)
SEP_PASS_R = DRUM_OR + SEP_PASS_W / 2.0

# lid body: v2 cable_ceiling's pattern — spokes + graded concentric rings
LID_SPOKE_N   = 24                 # spoke count (v2's call)
LID_SPOKE_W   = 4 * NOZZLE_D         # 3.2 — web width, 4 beads (v2)
LID_RING_IN_R1 = DRUM_OR           # inner solid band: bore → wall OR (covers
                                   # the tenon ring + its root, sits exactly
                                   # over the wall)
LID_RIM_R0    = LID_OD / 2.0 - 6 * NOZZLE_D      # 68.1 — solid outer rim (v2's
                                               # unbacked-rim width)
LID_MID_W     = 4 * NOZZLE_D         # 3.2 — mid tie band (v2)
LID_MID_RC    = (LID_RING_IN_R1 + LID_RIM_R0) / 2.0    # 58.925 — span middle

assert SEP_PASS_R - SEP_PASS_W / 2.0 - DRUM_OR >= -1e-9, \
    "pass tunnel undercuts the drum wall's root"
# the TRUE swept plan envelope of the tilted tube (its corner edges,
# not just its mouth) must keep the 1.6 tier to the tooth roots —
# user-caught: the mouth-only version measured 1.0 on the print model
assert (math.hypot(SEP_PASS_R + SEP_PASS_W / 2.0,
                   SEP_PASS_W * math.sqrt(2.0) / 2.0 + RIM_H / 2.0)
        <= RIM_OD / 2.0 - RATCHET_DEPTH - 2 * NOZZLE_D + 1e-9), \
    "pass tunnel's swept envelope reaches within 1.6 of the tooth roots"
assert DRUM_IR - SPRING_FLANGE_OD / 2.0 >= 2.0, \
    "drum bore too close to the spring flanges (install slide-past)"

# ── Containment WALL + frame BEAMS (v2's stack, v3-anchored) ─────────────────
WALL_SEP_CLR = 2 * NOZZLE_D          # 1.6 — wall bore off the separator/lid OD
                                   # (user's call)
WALL_IR = RIM_OD / 2.0 + WALL_SEP_CLR          # 71.03 — wall inner face
WALL_T  = 3 * NOZZLE_D               # 2.4 — ring thickness (user's call)
WALL_OR = WALL_IR + WALL_T                     # 73.43
BEAM_SIZE = FRAME_RIB              # beams match the plus arms (10.4 sq)
BEAM_IR   = WALL_OR                # beams sit just outside the wall ring (v2)
FRAME_R_OUT = BEAM_IR + BEAM_SIZE              # 83.83 — the frame extent,
                                   # DERIVED now (replaces the v2 87.7 carry)

# ── AXIAL service-loop chamber DEPTH (v2's floating torsion coil, re-derived
# for v3: NO AXLE below the separator — only the cable's bend radius floors
# the tight coil; the wall bore is the loose bound). Drum rotation changes
# the coil's diameter; the chamber must hold the worst-case wrap stack. ─────
CABLE_BEND_R_MIN = 15.0            # HARDWARE: safe bend radius (v2's value —
                                   # remeasure for the Ø3.0 cable, it can
                                   # only shrink the chamber)
LOOP_D_TIGHT = 2.0 * (CABLE_BEND_R_MIN + 1.0)  # 32 — extended (tight) centreline
LOOP_D_LOOSE = 2.0 * WALL_IR - CABLE_D - 1.4   # 137.66 — retracted: 0.7/side
                                               # off the wall bore
N_MIGRATE    = N_WRAPS             # drum turns over full payout (same spiral)
LOOP_K_TIGHT = N_MIGRATE * LOOP_D_LOOSE / (LOOP_D_LOOSE - LOOP_D_TIGHT)  # ≈ 6.59
AXIAL_STACK  = (LOOP_K_TIGHT + 1.0) * COIL_PITCH   # ≈ 23.5 — worst-case stack
CH_TOP_Z = SEP_Z0 - 3 * NOZZLE_D     # coil ceiling under the rotating disk
# chamber depth: the worst-case breathing stack — FLOORED by the entry
# port (at 1 m capacity the stack-derived chamber is too shallow for
# the Ø13 connector's house gable; the port is hardware-sized, so the
# chamber keeps its minimum and carries spare coil headroom)
CH_BOT_Z = CH_TOP_Z - max(AXIAL_STACK + 1.0,
                          0.5 + 1.5 * SEP_PASS_W + 2 * NOZZLE_D)
# SPOKED FLOOR (user's call, replacing BOTH v2's separate coil cup AND
# the bottom plus: the fused wall band + floor + beams ARE the bottom
# frame now — and the stack got ~11 shorter for it)
FLOOR_T   = 3 * NOZZLE_D             # 2.4 — floor plate thickness (user: 1.6
                                   # flexed along z under centre pressure —
                                   # ALL the bottom webbing went to 2.4)
FLOOR_Z1  = CH_BOT_Z               # ≈ −70.9 — floor top = the chamber floor
FLOOR_Z0  = FLOOR_Z1 - FLOOR_T     # ≈ −72.5 — the part's bed plane
# SOLID FLOOR (user #977 — the cable JAMMED in the old sparse spoke
# web's openings): the floor is a full plate out to the wall band now,
# relieved only by narrow VIEWING SLITS. Each slit is FLOOR_SLIT_W
# wide, NARROWER THAN THE CABLE, so no wrap can drop into one — you
# keep the look into the tray (the user's reason not to simply close
# it) and lose the traps. (FLOOR_SPOKE_N and the two tie rings are
# retired with the web; FLOOR_SPOKE_W lives on as the hub-web section.)
FLOOR_SPOKE_W = 3 * NOZZLE_D         # 2.4 — hub-web section (was the floor
                                   # spoke width, hence the name)
FLOOR_HUB_R   = 22 * NOZZLE_D        # 17.6 — centre disc tying the spokes AND
                                   # landing the sleeve's flared root (was
                                   # 9.6 before the sleeve arrived)
# HEX-GRID viewing holes (user #980: round holes tile far tighter than
# the first cut's arc slits — same cable rule, ~4x the open area): Ø
# FLOOR_HOLE_D (under the cable Ø until #994, below); packed on a triangular lattice at the
# two-bead web, which is as many as the plate can carry.
# (user #994: DOUBLED from 2.4 for a better look into the tray. That is
# now WIDER than the Ø3.3 cable, deliberately: a wrap lying across a
# 4.8 hole bridges it (its own ≥ 70 mm bend radius sags ~0.04 over that
# span), and 4.8 is too small a pocket to fold a cable into. Asserted
# ≤ 1.5 × CABLE_D so a cable resize can't quietly turn them into traps.)
FLOOR_HOLE_D   = 6 * NOZZLE_D        # 4.8 — hole Ø
FLOOR_HOLE_WEB = 2 * NOZZLE_D        # 1.6 — plate between holes (the tier)
assert FLOOR_HOLE_D <= 1.5 * CABLE_D + 1e-9, \
    "floor viewing holes are wide enough for a wrap to kink into"
FLOOR_HOLE_R0  = FLOOR_HUB_R       # 17.6 — hub stays SOLID inside this (it
                                   # carries the stub axle and the sleeve)
FLOOR_HOLE_R1  = WALL_IR - 3 * NOZZLE_D      # ≈72.7 — solid rim out to the
                                   # band: the weld, and the coil's own
                                   # outermost wraps, ride it
FLOOR_RIB_W    = BEAM_SIZE         # 10.4 — four CARDINAL ribs stay solid,
                                   # under the beams: the hub's load path
                                   # out to the band/beam feet (what the
                                   # old beam-wide floor spokes did)
BEAM_Z0 = FLOOR_Z0                 # beams stand on the bed with the floor
BEAM_Z1 = FRAME_Z0                 # beams end at the top plus underside

# CENTRE GUIDE SLEEVE (user's call): the tight coil winds against it, so
# it can never be yanked under the cable's bend radius, and it keeps the
# breathing helix round and centred (v2's cup sleeve, reborn on the floor).
SLEEVE_OD    = 33 * NOZZLE_D         # 26.4 — the tight coil's ID (LOOP_D_TIGHT
                                   # − CABLE_D = 28.15 at the Ø3.85 cable)
                                   # keeps 0.5+/side off it (v2's guard;
                                   # was 28.0 — slimmed at #883 with the
                                   # cable bump)
SLEEVE_T     = 2 * NOZZLE_D          # 1.6 — tube wall
SLEEVE_FLARE = 4 * NOZZLE_D          # 3.2 — 45° root flare (strength, user's
                                   # call; widens toward the bed — support-
                                   # free, and the hub disc lands it)
SLEEVE_Z1    = CH_TOP_Z            # sleeve top — 2.4 under the rotating disk
SLEEVE_TOP_CH = NOZZLE_D             # 0.8 — 45° chamfer easing the top rim:
                                   # the tight-state feed drop crosses the
                                   # sleeve's upper region on its way from
                                   # the pass tunnel to the coil top — a
                                   # sharp rim there is a snag (leaves a
                                   # one-bead flat on the 1.6 wall)

# HOUSE-shaped cable ENTRY port (v2's size — 13 wide, 45° gable roof,
# prints support-free in the upright band): the SOURCE cable enters the
# chamber here — the axial coil's stationary end — at its bottom edge.
# The user wants it at −x, but the 180° BEAM is exactly there and the
# 13-wide house would sever it — so the azimuth DERIVES to hug the
# beam's edge as close to −x as the geometry allows (≈ 191°, v2's spot
# for the same reason).
ENTRY_PORT_W    = SEP_PASS_W       # 13 — the connector passes (v2)
ENTRY_PORT_SILL = FLOOR_Z1 + 0.5   # just off the chamber floor (v2)
_A_BEAM_EDGE    = math.degrees(math.asin((BEAM_SIZE / 2.0) / WALL_IR))  # 4.2°
_A_ENTRY_HALF   = math.degrees(math.asin((ENTRY_PORT_W / 2.0) / WALL_IR))
ENTRY_PORT_AZ_DEG = 180.0 + _A_BEAM_EDGE + _A_ENTRY_HALF + 2.0   # ≈ 191.5

assert (LOOP_D_TIGHT - CABLE_D) - SLEEVE_OD >= 1.0 - 1e-9, \
    "tight coil pinches the guide sleeve"
assert FLOOR_HUB_R >= SLEEVE_OD / 2.0 + SLEEVE_FLARE - 1e-9, \
    "floor hub disc too small to land the sleeve's flared root"
assert ENTRY_PORT_SILL + 1.5 * ENTRY_PORT_W <= CH_TOP_Z - 2 * NOZZLE_D + 1e-9, \
    "entry port's gable peak reaches the chamber ceiling"

# (The diamond PERFORATION of the band is RETIRED — user #932: it sat
# behind wall.py's build-speed flag through the whole v3 iteration and
# never got used. The band ships solid.)

# ── BOTTOM BEARING (user's redesign — REPLACES the 45° rest lip): a second
# 608 seated in the separator disk's UNDERSIDE (outer race co-rotates with
# the disk) over a static STUB AXLE printed on the frame_bottom floor
# (inner race seats on its shoulder) — a real spindle: axial rest AND a
# second radial constraint, no rubbing plastic. Printability in the
# axle_separator's upright print: the pocket closes with a flat SEAT RING
# (the outer-race thrust seat — an accepted bridge ring), then a 45° CONE
# to a small apex bridge (user's cone) — no flat overhang.
BOT_BRG_Z0 = SEP_Z0                            # −44 — bearing FLUSH with the
                                               # disk's face (user's call —
                                               # the proud variant reverted)
BOT_BRG_Z1 = BOT_BRG_Z0 + BEARING_W            # −37 — bearing top face
# The CONE IS THE SEAT (user's call): v2's chamfer-mates-cone trick,
# outer-ring edition — the 45° cone rises straight off the pocket bore,
# and the race's factory OD-edge chamfer (BEARING_INNER_CH — same r_s
# spec both rings) face-mates it, landing the race top at BOT_BRG_Z1.
# The cone is then CAPPED FLAT wherever walls demand (user's call): the
# 1.6 web to the disk's TOP face caps it at Ø19.4 — a deliberate,
# accepted ceiling bridge: only empty space sits above the bearing, so
# print sag there breaks nothing.
BOT_SEAT_CONE_Z0 = (BOT_BRG_Z1 - BEARING_INNER_CH
                    - (BRG_BORE - BEARING_OD) / 2.0)   # −37.4 — cone base
                                                       # (Ø = the pocket bore)
BOT_CONE_CAP_Z = SEP_Z1 - 2 * NOZZLE_D           # −36.0 — flat cap: exactly the
                                               # quality web under the disk top
BOT_CONE_CAP_D = BRG_BORE - 2.0 * (BOT_CONE_CAP_Z - BOT_SEAT_CONE_Z0)  # 19.4
STUB_D       = AXLE_PRINT_D        # 8.0 — at the 608's nominal bore (user)
STUB_BOSS_D  = STUB_D + 4 * NOZZLE_D # 11.2 — inner-race seat shoulder (lands
                                   # on the face annulus, same as the top)
STUB_BOSS_Z1 = BOT_BRG_Z0          # boss shoulder top = inner race bottom
STUB_Z1      = BOT_BRG_Z1          # stub tip flush with the bearing top
STUB_FLARE   = 3 * NOZZLE_D          # 2.4 — 45° base flare (strength)
# HUB WEBS (user #970 — stiffen frame_bottom's centre): N_HUB_WEB radial
# ribs inside the guide sleeve's bore, tying the stub boss to the sleeve
# wall and the floor plate, at the BEAM azimuths so they continue the
# beam-wide floor spokes' load path. Vertical walls in the upright
# print; they top out with the sleeve, 2.4 under the rotating disk and
# well under the bottom 608 (which sits above the sleeve top).
N_HUB_WEB  = 4
HUB_WEB_W  = FLOOR_SPOKE_W           # 2.4 — spoke section
HUB_WEB_Z1 = SLEEVE_Z1               # flush with the sleeve top
# (STUB_TIP_CH is RETIRED — user #945: the 0.8 entry chamfer stole
# grip from the inner race; the stub runs full-Ø to a flat top at
# STUB_Z1, the same −37 tip height)

assert BOT_CONE_CAP_Z > BOT_SEAT_CONE_Z0 + 0.5, \
    "seat cone has no height before its cap — check the bearing z-chain"
assert SEP_Z1 - BOT_CONE_CAP_Z >= 2 * NOZZLE_D - 1e-9, \
    "under 1.6 of web between the seat cavity's cap and the disk's top face"
assert STUB_BOSS_D <= BEARING_IRACE_OD, \
    "stub shoulder must land fully on the inner race's face annulus"
assert STUB_BOSS_D / 2.0 + STUB_FLARE < (SLEEVE_OD - 2 * SLEEVE_T) / 2.0, \
    "stub base flare reaches the guide sleeve's bore"

# ── Separator DISC POCKET (user #880: save material) — the disc inside
# the lid-joinery ring is hollowed to a SEP_POCKET_FLOOR_T plate; a
# central BOSS keeps the bearing seat walled, and the axle's root cone
# now runs CONTINUOUSLY to the Ø8 post (the old one topped at the fat
# column's Ø with a flat ring — user: strength lost; the 45° must stop
# below the spring's Ø8.7 hole, so it completes at the shoulder plane) ───────
SEP_POCKET_R       = DRUM_IR       # 43.35 — pocket out to the drum bore
SEP_POCKET_FLOOR_T = 3 * NOZZLE_D    # 2.4 — plate left at the disc bottom (user)
SEP_BOSS_D = BOT_CONE_CAP_D + 2 * 3 * NOZZLE_D   # 24.2 — bearing-cavity walls
SEP_BOSS_FLARE = 3 * NOZZLE_D        # 2.4 — 45° skirt tying the boss into the
                                   # pocket floor (user; narrows upward in
                                   # the upright print — self-supporting)
assert SEP_BOSS_D / 2.0 + SEP_BOSS_FLARE < SEP_POCKET_R - 1.0, \
    "separator bearing boss (flare included) meets the pocket's outer wall"
assert LOOP_D_TIGHT / 2.0 >= CABLE_BEND_R_MIN - 1e-9, \
    "tight coil's centreline under the cable's bend radius"
assert 2.0 * WALL_IR - LOOP_D_LOOSE - CABLE_D >= 1.2 - 1e-9, \
    "loose coil pinches the wall bore"

# ── Wall clearances (the T-joint POLICY values live on — the wall↔beam T
# joints themselves are GONE, user's call: the wall is ONE fused band now,
# no sliding pieces, so JOINT_CLR/JOINT_BACK_CLR — now defined at the top
# of the file with the print policy — remain as the project's
# print-proven lateral / depth-face clearances for every other joint) ────────
WALL_ZB       = FLOOR_Z0           # wall band bottom — the band is FUSED
                                   # into frame_bottom, bed to lid-top
                                   # (user: wall_top, the 4 lock strips and
                                   # the beam T channels are all RETIRED)

# frame_top ↔ frame_bottom ROTATIONAL arc joints — cadkit's MUSHROOM
# arc, TWO-SIDED (user: v2 halved this joint to one flare for T-channel
# access; the T is retired, so the second flare returns — a stop sign
# with the top chopped off: the flat top prints fine because the two
# hosts print in OPPOSITE directions, so no roof geometry is needed).
# SITED via cadkit's ring↔arm CROSSING now, the same arrangement as the
# mount's rings (user: share the code, differ only in profile): tenon
# flush at the beam's CCW entry face spanning HALF the crossing — the
# 50% rule applies here too (user reversed the earlier revert) — so the
# stop-side half of every beam crossing stays solid; TOP_STOP_WALL is
# RETIRED (the stop wall is simply the uncut half).
TOP_JOINT_SEAT_CLR = JOINT_CLR     # seating clearance at each stop (arc
                                   # mm) — the crossing's own default,
                                   # NAMED for the pads-stop-first assert
                                   # (was a re-typed 0.15 that would have
                                   # stopped tracking the material policy
                                   # — #932 altitude review)
TOP_ENTRY_OVER     = 1.0           # channel overshoot past the entry (arc mm)
FRAMEJ_W = 6 * NOZZLE_D              # 4.8 — joint width (stem 2.4, cadkit)
FRAMEJ_R = snap((BEAM_IR + BEAM_SIZE)
                - (2 * NOZZLE_D + FRAMEJ_W / 2.0 + JOINT_CLR),
                NOZZLE_D, "down")
                                   # profile centreline radius on the beams:
                                   # the FARTHEST bead radius whose cavity
                                   # keeps a 1.6 outer wall at the flush arm
                                   # end (was a pinned 79.2 — DERIVED since
                                   # #884 so capacity resizes flow through;
                                   # frame.py still asserts the walls)
FRAMEJ_TEN_ARC = BEAM_SIZE / 2.0   # 5.2 — engaged tenon arc per site (the
                                   # 50% rule: the joint takes half the
                                   # crossing — the mount's joint ran the
                                   # same half until #987 made it through)

# ── MOUNT — desk/wall TWIST-LOCK RING (v3; v2's validated screw) ─────────────
# ONE CONTINUOUS ring (user: a one-piece mount can't be screwed down at
# the wrong spacing), FLUSH with the frame's top face when assembled
# (stack stays FRAME_RIB — vertical space is precious). The ring rides
# arc channels near the arms' ENDS (user: joinery outboard for
# structural integrity — the largest bead radius whose THROUGH cavities
# stay radially clear of the frame_top↔bottom arc joints further out,
# asserted in frame.py). WORKFLOW (user): (1) screw the ring to the
# wood — both screws sit on quadrant pads INSIDE the frame's footprint,
# over open air, fully driver-accessible because the assembly isn't
# there yet; (2) offer the assembly up — the ring's four arc TENONS
# pass through the OPEN QUADRANTS — and rotate it to the angular stops
# (the same motion and angular constants as the frame_top↔frame_bottom
# install: one muscle memory for both). Rotation is what makes the
# install sweep trivial: the ring maps onto its own circle, so nothing
# ever sweeps across the centre — the reason a slide-in bypass around
# the axle could not work. Remove: rotate back, drop away.
# Joint: cadkit's OCTAGON (stop-sign) ARC — the library's standard for
# two hosts printing the same direction (user's call at #864: the old
# THROUGH-mushroom avoided the stop sign by leaving the cavity open at
# the arm's underside, which weakened the arm; the octagon closes the
# cavity on its one-bead roof with a ≥1.6 floor left below — the spine
# shrank to MOUNT_SPINE_T to raise the mating plane and buy the room).
# The tenons hang from a flush SPINE ring exactly one stem wide (a
# wider slot floor would bridge in the frame's print); over each arm's
# stop-wall zone the spine rides an annular V-bottom groove.
WOOD_SCREW_SHAFT_D = 4.0           # v2's validated screw geometry, verbatim
WOOD_SCREW_HEAD_D  = 9.3           # countersunk-head pocket Ø at the face
WOOD_SCREW_HEAD_H  = 4.0           # cone depth
MOUNT_PLATE_T = 5 * NOZZLE_D         # 4.0 — pad/spine thickness: the MINIMUM
                                   # that swallows the head cone flush
                                   # (user's rule — v2's config)
assert MOUNT_PLATE_T >= WOOD_SCREW_HEAD_H - 1e-9, \
    "mount plate too thin to countersink the wood screw flush"

# DEEP SCREW PADS (user #947 — FUSES the #936 screw_spacer into the
# mount: the separate per-screw puck is retired, only the PADS deepen,
# the spine/rings/tails keep their sections): each pad runs the full
# MOUNT_PAD_H from the frame face, its countersink opening at the new
# underside — the same 5.6 of drilled depth stolen, zero loose parts.
MOUNT_PAD_H = 12 * NOZZLE_D        # 9.6 — pad z depth (the old 4.0 plate
                                   # + the 5.6 spacer, one piece)
assert MOUNT_PAD_H >= WOOD_SCREW_HEAD_H + 2 * NOZZLE_D - 1e-9, \
    "deep pad too thin to countersink the head with a tier of web"

MOUNT_TEN_W  = 5 * NOZZLE_D          # 4.0 — joint width: the largest whose
                                   # OCTAGON swallow fits above the 1.6
                                   # arm floor (asserted in mount.py); its
                                   # cavity leaves 59% of the arm intact
                                   # (user's ≥~50% rule, asserted). The
                                   # STEM is overridden to MOUNT_SPINE_W
                                   # (2.4, user #910 — match the ring
                                   # bar) via cadkit's stem= knob; width
                                   # 4.8 (stem 2.4 the parity way) was
                                   # infeasible: swallow 6.97 > the 6.4
                                   # of room over the arm floor
MOUNT_SPINE_T = 3 * NOZZLE_D         # 2.4 — spine ring depth (was 4.0: the
                                   # shallower spine raises the mating
                                   # plane, buying the octagon its room;
                                   # the screw PADS keep MOUNT_PLATE_T)
MOUNT_MATE_Z = FRAME_RIB - MOUNT_SPINE_T           # 8.0 — the joint's mating
                                                   # plane (spine underside)
MOUNT_FLOOR_MIN = 2 * NOZZLE_D       # 1.6 — solid arm required under the
                                   # closed cavities (user's call)
MOUNT_RING_R = snap(FRAMEJ_R - FRAMEJ_W / 2.0 - JOINT_CLR
                    - 2 * NOZZLE_D - MOUNT_TEN_W / 2.0 - JOINT_CLR,
                    NOZZLE_D, "down")
                                   # outer ring centreline: the farthest
                                   # bead radius whose cavities keep a 1.6
                                   # web to the frame arc joints (was a
                                   # pinned 71.2 — DERIVED since #884 so
                                   # capacity resizes flow through; the
                                   # keep-out stays asserted in frame.py)
MOUNT_RING2_R = min(snap(MOUNT_RING_R / 2.0, NOZZLE_D),
                    snap(ANCH_IR - MOUNT_TEN_W / 2.0 - JOINT_CLR
                         - 2 * NOZZLE_D, NOZZLE_D, "down"))
                                   # INNER ring at half the diameter, on the
                                   # bead grid (derived since #884)
                                   # (user: one ring is flimsy). Its arc
                                   # joinery is RETIRED (user #985 — see
                                   # MOUNT_RING3_R); the cap that kept its
                                   # old cavities off the +x anchor rib
                                   # still places it (36.0)
MOUNT_RING3_R = snap((MOUNT_RING_R + MOUNT_RING2_R) / 2.0, NOZZLE_D)
                                   # MIDDLE ring (user #985). The inner
                                   # ring's tenons carried next to nothing
                                   # — the screws sit on the OUTER ring and
                                   # the frame_top it grips is thick — so
                                   # the inner ring and this new one only
                                   # RIDE plain V-grooves through the arms:
                                   # structure for the mount, no joinery.
                                   # Only the outer ring locks to the frame
                                   # now, and since the inner ring's wide
                                   # crossing was what set the install
                                   # rotation, that rotation roughly halves
                                   # (mount.py MOUNT_MATE_DEG)
MOUNT_SPINE_W = 3 * NOZZLE_D         # 2.4 — ring section (user: one square
                                   # 2.4 x 2.4 for rings/spokes/tails; the
                                   # tenon stem matches it via cadkit's
                                   # stem= override, and the V-grooves run
                                   # as ARCS between the cavity spans —
                                   # #913 — the slot IS the groove inside
                                   # a span)
# (MOUNT_TEN_ARC is RETIRED — user #987: the mount's joint runs THROUGH
# the whole arm now, face to face; the screw pads are the stop, so no
# half of the crossing needs to stay solid. mount.py _ThroughCrossing.)
MOUNT_SPOKE_W  = 3 * NOZZLE_D        # 2.4 — ring-tie spokes (user: ONE
                                   # consistent 2.4 × 2.4 square section
                                   # for rings, spokes and the screw-pad
                                   # guide tails; spokes ride the rings'
                                   # own z-band now, not the full plate)
N_MOUNT_SPOKE = 3                  # ring-tie spokes PER QUADRANT (user
                                   # #974, up from the single mid-quadrant
                                   # one): with both rings deep now
                                   # (STIFFENING WEBS) the spokes are what
                                   # makes the pair act as ONE truss rather
                                   # than two rings bending separately —
                                   # and the bare mount, alone on the wood
                                   # while its screws are driven, has no
                                   # other tie. NOTHING can tie the rings
                                   # ACROSS an arm (user #975 asked): the
                                   # mount's top face is flush against the
                                   # wood, so a radial bar there would have
                                   # to pass through the arm's top, and it
                                   # sweeps MOUNT_MATE_DEG along the arm
                                   # during the install — the pocket that
                                   # needs is wider than the arm. The arm
                                   # IS that tie once the assembly is on
                                   # (both rings land a tenon in it). So
                                   # the spokes crowd the arms instead;
                                   # azimuths derive in mount.py
                                   # (_spoke_layout), the outermost pair
                                   # sitting as close to each arm as the
                                   # sweep allows — and they BOUND that
                                   # quadrant's deep ring webs, so every
                                   # tall-to-short step in a rim lands ON
                                   # a spoke instead of floating mid-span
                                   # (user #976).
# STIFFENING WEBS (user #973 — the bare mount BENT while its screws were
# driven and the assembly seated crooked): the ring/spoke wireframe is
# only MOUNT_SPINE_T deep, i.e. a floppy plate in z. So every span that
# does NOT sit in a frame arm's channel hangs a WEB down to the screw
# pads' underside plane: those members go from 2.4 to MOUNT_PAD_H deep
# (~60x the bending stiffness), and because the pads already reach that
# plane the deep spans simply fuse into them — pad, ring and spoke
# become one deep girder instead of a pad hanging off a ribbon.
# Room: the quadrants are OPEN AIR under the mount in both variants
# (frame_top is a plus), and the web stops one bead above the frame's
# underside — the couch build's frame is COUCH_TOP_PAD taller, so its
# webs clear everything below by that much more. The angular keep-outs
# live in mount.py: they need the install sweep (MOUNT_MATE_DEG).
MOUNT_WEB_Z0 = FRAME_Z1 - MOUNT_PAD_H   # 0.8 — web bottom = the screw
                                        # pads' underside (ONE datum, so a
                                        # deeper pad deepens the webs with it)
MOUNT_WEB_CLR_DEG = 2.0            # angular margin a deep span keeps off an
                                   # arm, on top of the install sweep
assert MOUNT_WEB_Z0 - FRAME_Z0 >= NOZZLE_D - 1e-9, \
    "mount webs reach the frame's underside plane"

# ANGLED SCREWS (user #959): each screw drives MOUNT_SCREW_TILT_DEG
# INWARD — toward the arm its pad hugs — so its wood entry lands well
# inside a narrow beam (a vertical screw at the old pad centre sat
# ~4.3 off a 30 beam's edge). Tilted geometry: a head-pass BORE
# (Ø HEAD + 2·FIT) rides the tilted axis from the pad's underside up to
# the countersink seat, recessed MOUNT_SCREW_SEAT along the axis so the
# TILTED head disc sits fully inside the pad (its outboard rim lands
# flush with the underside); the Ø4 shaft then climbs to the wood face
# staying inside the pad's INBOARD wall — past that wall is the frame
# arm. EVERY boss wall is sized to STRUCT_WALL (1.6, two beads — user
# #971: the first tilted cut left one-bead walls). In a print slice the
# tilted bore and shaft read r/cos(tilt) wide ACROSS the arm, so: the
# screw entry moves out until the shaft keeps 1.6 to the flank at the
# wood face, the pad grows across the arm until the bore keeps 1.6 to
# the outboard face at the underside, and along the arm the pad keeps
# 1.6 either side of the bore.
MOUNT_SCREW_TILT_DEG = 30.0
MOUNT_SCREW_BORE_D = WOOD_SCREW_HEAD_D + 2 * FIT_CLR   # 9.6 — head-pass bore
MOUNT_SCREW_SEAT = (WOOD_SCREW_HEAD_D / 2.0
                    * math.tan(math.radians(MOUNT_SCREW_TILT_DEG)))
                                   # 2.68 — countersink base recess (axial)
_SCREW_T = math.tan(math.radians(MOUNT_SCREW_TILT_DEG))
_SCREW_C = math.cos(math.radians(MOUNT_SCREW_TILT_DEG))
MOUNT_PAD_W   = snap(MOUNT_SCREW_BORE_D + 2 * STRUCT_WALL, NOZZLE_D, "up")
                                   # 12.8 — pad ALONG the arm
MOUNT_PAD_SCREW_Y = snap(STRUCT_WALL + WOOD_SCREW_SHAFT_D / 2.0 / _SCREW_C
                         + MOUNT_PAD_H * _SCREW_T, NOZZLE_D, "up")
                                   # 9.6 — screw entry (at the underside)
                                   # off the arm flank
MOUNT_PAD_LY  = snap(MOUNT_PAD_SCREW_Y + MOUNT_SCREW_BORE_D / 2.0 / _SCREW_C
                     + STRUCT_WALL, NOZZLE_D, "up")
                                   # 16.8 — pad ACROSS the arm, flank outward
MOUNT_SCREW_WOOD_Y = (BEAM_SIZE / 2.0 + MOUNT_PAD_SCREW_Y
                      - MOUNT_PAD_H * _SCREW_T)     # ≈ 9.3 — where the shaft
                                   # enters the wood, off the arm's axis
assert (MOUNT_PAD_SCREW_Y - MOUNT_PAD_H * _SCREW_T
        - WOOD_SCREW_SHAFT_D / 2.0 / _SCREW_C >= STRUCT_WALL - 1e-9), \
    "tilted shaft leaves under 1.6 of inboard wall at the wood face"
assert (MOUNT_PAD_LY - (MOUNT_PAD_SCREW_Y + MOUNT_SCREW_BORE_D / 2.0 / _SCREW_C)
        >= STRUCT_WALL - 1e-9), \
    "tilted head bore leaves under 1.6 of outboard wall"
assert (MOUNT_PAD_W - MOUNT_SCREW_BORE_D) / 2.0 >= STRUCT_WALL - 1e-9, \
    "head bore leaves under 1.6 of wall along the arm"
assert ((MOUNT_SCREW_SEAT + WOOD_SCREW_HEAD_H) * _SCREW_C + 2 * NOZZLE_D
        <= MOUNT_PAD_H + 1e-9), \
    "tilted countersink cone reaches the wood face"
# screw pads HUG THE AXES (user: the screws must land on a wooden BEAM
# running along x OR y — use whichever pad pair matches the beam; the
# beam just has to be wide enough for both screws of the pair —
# the tilted screws enter the wood ~±9.3 off-axis, MOUNT_SCREW_WOOD_Y). The pads sit at ZERO gap: seated, each pad's inner
# edge lands FLAT ON its arm's flank — the pads ARE the seating stop
# (user's call). The arc joints keep TOP_JOINT_SEAT_CLR (0.15) at
# their stop ends, so the joinery never bottoms out first. Pads sit
# CCW (+) of the arms — the #938 chirality FLIP (user, field failure:
# a fast retract SLAMS the cable into the exit horn and the impulse
# torque — opposite the pull-out torque the #915 audit optimised for
# — spun the assembly off the mount): the mount's tenons seat
# rotating CW relative to the frame now (hand="cw", mount.py), so the
# pads trail on the CCW side and land on the arms' CCW flanks; the
# retract-slam torque presses them tighter, and the slow, hand-damped
# pull-out is what friction has to hold instead of the slam.
MOUNT_PAD_AZ_OFF = math.degrees(math.asin(
    (BEAM_SIZE / 2.0 + MOUNT_PAD_LY / 2.0) / MOUNT_RING_R))   # ≈ 9.6°
MOUNT_PAD_AZ = tuple(a + MOUNT_PAD_AZ_OFF for a in (0.0, 90.0, 180.0, 270.0))
# GUIDE-TAIL TIP (user #958): the tails run from each pad's outboard
# edge along the arm flank; their tips are the install reference — the
# rotate-on install needs the face set back
# MOUNT_COUCH_SETBACK (3.06 probed: frame_bottom's lever-mount corner
# sweeping on a ~91 radius at the mate angle) from the front beam — so
# the visible tail grows from the 6.37 to-the-face length to 12 beads
# (9.6), and its tip is where the front beam goes.
_PAD_EDGE_X = (MOUNT_RING_R * math.cos(math.radians(MOUNT_PAD_AZ_OFF))
               + MOUNT_PAD_W / 2.0)                # ≈81.5 — pad outboard edge
MOUNT_COUCH_SETBACK = 3.06         # probed clearance (#958), the floor
MOUNT_TAIL_TIP_X = FRAME_R_OUT + snap(MOUNT_COUCH_SETBACK, NOZZLE_D, "up")
                                   # face + 3.2 (4 beads), EVERY build (user
                                   # #962: only frame_top's height is couch-
                                   # gated) — anchored to the FACE, not the
                                   # pad edge, so pad changes can't erode
                                   # the set-back (#959: the wider pad did)
MOUNT_TAIL_VIS = MOUNT_TAIL_TIP_X - _PAD_EDGE_X    # ≈9.8 past the pad edge
assert MOUNT_TAIL_TIP_X - FRAME_R_OUT >= MOUNT_COUCH_SETBACK - 1e-9, \
    "guide tails end inside the rotate-on set-back"
MOUNT_PAD_INSET_MY = 3 * NOZZLE_D  # 2.4 — the −Y ARM's pad (and its screw)
                                   # slides inboard along its beam until
                                   # its outboard edge CLEARS the TPU lock's
                                   # top corbel by the sweep clearance
                                   # (#971: the two-bead boss left no room
                                   # for the old wedge relief over the
                                   # corbel — asserted in the LOCK block;
                                   # the flank edge — the seating stop — is
                                   # untouched, and 2.4 off the ring line is
                                   # nothing to a wood beam)
assert TOP_JOINT_SEAT_CLR > 0.0, \
    "the pads-stop-first scheme needs a positive joint seat clearance"
assert (BEAM_SIZE - (MOUNT_TEN_W + 2 * JOINT_CLR)) / 2.0 >= 2 * NOZZLE_D - 1e-9, \
    "mount cavity leaves the arm's side walls under the 1.6 tier"
assert (BEAM_SIZE - (MOUNT_TEN_W + 2 * JOINT_CLR)) / BEAM_SIZE >= 0.49, \
    "mount cavities eat more than ~half the arm's width (user's bound)"
assert (math.hypot(MOUNT_RING_R * math.cos(math.radians(MOUNT_PAD_AZ_OFF))
                   + MOUNT_PAD_W / 2.0, BEAM_SIZE / 2.0 + MOUNT_PAD_LY)
        <= FRAME_R_OUT - 1e-9), \
    "mount screw pads poke past the frame's plan silhouette"
assert (ANCH_IR - (MOUNT_RING2_R + MOUNT_SPINE_W / 2.0 + JOINT_CLR)
        >= 2 * NOZZLE_D - 1e-9), \
    "inner mount ring's groove reaches the +x anchor rib"
assert abs(MOUNT_RING_R * math.sin(math.radians(MOUNT_PAD_AZ_OFF))
           - MOUNT_PAD_LY / 2.0 - BEAM_SIZE / 2.0) < 1e-6, \
    "seated pads must land exactly on the arm flanks (they are the stop)"
# annular V-GROOVE for the spine over each arm's stop-wall zone (the
# groove's floor is the roof of the slot in the frame's +z→−z print —
# vertical walls, then a 45° V closing on a ONE-BEAD flat: 0.8 is the
# bridging limit, user's call — the earlier +2·clearance flat printed a
# 1.1 bridge; the V runs 0.15 deeper instead):
MOUNT_GRV_W    = MOUNT_SPINE_W + 2 * JOINT_CLR     # 2.7 — groove width: the
                                   # spine's snug seat. With the 2.4 stem
                                   # this EQUALS the cavities' neck slot —
                                   # ONE straight wall top to bottom
                                   # (user #911; the #910 interim 3.1
                                   # step is retired). The shared wall is
                                   # safe because the groove ARCS stop at
                                   # the cavity spans (mount_channel_cuts)
                                   # — no two cutters ever carve the same
                                   # wall; float-identical walls from two
                                   # cutters sent OCC unify into 15+ min
                                   # builds (py-spy-caught #910; identity
                                   # asserted in mount.py)
MOUNT_GRV_VERT = MOUNT_SPINE_T + JOINT_CLR         # 2.55 — vertical depth
MOUNT_GRV_FLAT = NOZZLE_D                            # 0.8 — dulled V flat
MOUNT_GRV_DEPTH = MOUNT_GRV_VERT + (MOUNT_GRV_W - MOUNT_GRV_FLAT) / 2.0  # 3.3

# ── TPU FRAME LOCK (user #939 — extra assurance the stack stays
# together): ONE vertical 95A pin at the +x arm's end, dropped through
# three stacked storeys that all protrude +x PAST THE FRAME FACE into
# open air — the mount jib tail widened into a slotted HEAD (top,
# z 8→10.4), a 45° CORBEL off frame_top's arm end face (middle,
# z 0→7.8, roof falling +x — the 45 for its flipped print), and a
# mirrored corbel off the beam's end face (bottom, underside RISING +x
# — the 45 for the upright print). One pin-shaped slot cuts through
# all three (the user's build recipe); with the pin in, neither the
# mount joint nor the frame_top↔frame_bottom joint can rotate. The
# pin is FREE in z (user): the slot is open through, the pin goes in
# from BELOW after mounting (the desk covers the top face), the desk
# itself caps +z, and the TPU snug fit holds it; its tip stays proud
# under the bottom 45 as the removal grip. PLACEMENT (user #941): on
# the −Y BEAM's +x FLANK, growing +x — NOT past the beam's −y end —
# and the geometry keeps its original +x-arm ORIENTATION, TRANSLATED
# there (user: move it, don't rotate it). The stack sits flush with
# the beam's outer end; that flank is the jib tail's flank (the CCW
# side since the #938 flip), so the mount storey welds straight into
# the tail via their overlap — the head carries NO −x weld stub (past
# the flank plane is frame-arm material). The −y arm's screw pad ends
# 0.58 shy of clearing the stack, so the top corbel wears a 45° NOTCH
# riding LOCK_SWEEP_CLR under the pad's seated + swept envelope (the
# pad only moves away from the stack during the install sweep).
# Dims below stay in the AS-DRAWN (+x-arm) frame; builders translate
# by (LOCK_SHIFT_X, LOCK_SHIFT_Y).
LOCK_PIN_SQ  = 5 * NOZZLE_D        # 4.0 — square pin section (95A TPU)
LOCK_PIN_CLR = 0.20                # per-side, all three hosts — 4.4 sq
                                   # slots (user #978: 4.2 then 4.3 both
                                   # printed too tight to insert)
LOCK_WELD    = NOZZLE_D            # 0.8 — corbel/head weld overlap into its
                                   # host (a real shared volume —
                                   # coincident-face unions are OCC's bad
                                   # day; one bead, it also surfaces as the
                                   # little flank stub)
LOCK_SLOT_W  = LOCK_PIN_SQ + 2 * LOCK_PIN_CLR      # 4.4 — slot y width
                                   # (clearance both sides along the beam)
LOCK_X0      = FRAME_R_OUT                         # as-drawn base plane
                                                   # (LOCK_SHIFT_X maps it
                                                   # onto the beam's flank)
# walls on BOTH x sides of the slot — the #942 open-at-the-flank slot
# is REVERTED (user #948, the chirality realization: the mount's
# uninstall motion at the −y beam is tangential ≡ +x, so a head slot
# open at −x simply slides OFF the pin in exactly the escape
# direction; the head's −x wall is what makes the lock a lock — the
# escaping mount pushes the pin +x through that wall, and the pin
# braces on the frame corbels' +x walls). The pin rides 1.6 off the
# beam wall again, clearance all around.
LOCK_SLOT_X0 = LOCK_X0 + STRUCT_WALL
LOCK_SLOT_X1 = LOCK_SLOT_X0 + LOCK_SLOT_W          # 4.4 sq slot
LOCK_BLK_D   = LOCK_SLOT_W + 2 * STRUCT_WALL       # 7.4 — protrusion depth:
                                   # the slot + 1.6 walls BOTH sides
                                   # (the −x one is the LOCK)
LOCK_X1      = LOCK_X0 + LOCK_BLK_D
LOCK_SLOT_Y0 = FRAME_RIB / 2.0 + STRUCT_WALL       # 6.8 — slot y band sits
LOCK_SLOT_Y1 = LOCK_SLOT_Y0 + LOCK_SLOT_W          # 11.0   inside the head
LOCK_HEAD_Y1 = LOCK_SLOT_Y1 + STRUCT_WALL          # 12.6 — head/corbel +y face
LOCK_SWEEP_CLR = 0.2               # corbel roof under the jib head's swept
                                   # underside (the mount install rotation)
# COUCH HEAD DROP (user #983 — the spool twisted off its mount, and the
# mount's head gripped the pin over only a few mm): the couch build's
# frame_top carries COUCH_TOP_PAD of extra height, so the interface
# between the corbel's 45 ROOF and the mount head's 45 UNDERSIDE moves
# DOWN into it — to the height where the two storeys grip EQUAL lengths
# of pin at the slot's centreline (user #984: split it evenly; handing
# the whole pad to the mount left frame_top ~6 mm of grip against the
# head's ~19). Along the pin at the slot centre, frame_top grips from its
# underside up to its roof and the head from its underside up to the
# frame face; solving for equal spans gives the drop below, snapped to a
# bead. The corbel stays a 45 roof (no overhang in its flipped print) and
# the 45-on-45 still only OPENS during the install sweep. Desk build:
# there is no spare height, so the drop clamps to 0 and nothing moves.
_LOCK_SLOT_C = (LOCK_SLOT_X0 + LOCK_SLOT_X1) / 2.0 - LOCK_X0   # slot centre,
                                   # off the flank
_LOCK_TOPB_DESK = MOUNT_MATE_Z - LOCK_SWEEP_CLR   # the undropped roof
LOCK_HEAD_DROP = max(0.0, snap(
    ((_LOCK_TOPB_DESK + COUCH_TOP_PAD - _LOCK_SLOT_C)              # frame grip
     - (FRAME_Z1 - (_LOCK_TOPB_DESK + LOCK_SWEEP_CLR - _LOCK_SLOT_C)))  # head
    / 2.0, NOZZLE_D))              # 4.0 couch / 0 desk
LOCK_TOPB_Z1 = _LOCK_TOPB_DESK - LOCK_HEAD_DROP    # top corbel roof at the
                                   # flank, MOUNT-local (7.8 desk / 3.8 couch)
LOCK_HEAD_ZF = LOCK_TOPB_Z1 + LOCK_SWEEP_CLR      # the head's underside at
                                   # the flank (MOUNT_MATE_Z on the desk)
_LOCK_TOPB_FOOT = FRAME_Z0 - COUCH_TOP_PAD         # the top corbel's foot:
                                   # frame_top's underside, mount-local
LOCK_BOTB_H  = 13 * NOZZLE_D       # 10.4 — beam corbel depth at the face
# the MOUNT storey is a TRAPEZOID (user #942: cover more pin, better
# grip): flat top at the frame face, underside FOLLOWING the top
# corbel's 45 roof at LOCK_SWEEP_CLR — its slot walls grip the pin
# from 10.4 down to ~2.3 instead of stopping at the 8.0 mating plane.
# Printable for free: in the mount's flipped print that underside is
# a print-TOP face. At the flank it lands exactly on MOUNT_MATE_Z
# (= roof peak + clearance), so the head still meets the tail rib
# flush; the 45-on-45 sweep only OPENS during install (tangential
# motion at the −y beam is ±x, and the head enters displaced +x).
LOCK_HEAD_ZB = LOCK_TOPB_Z1 - LOCK_BLK_D + LOCK_SWEEP_CLR   # 0.6 — head
                                                   # underside at its +x face
LOCK_JOINT_Z0 = -(LOCK_BOTB_H - STRUCT_WALL)       # −8.8 — the BOTTOM of
                                   # the lock joint: the beam corbel's
                                   # underside at the slot's −x wall, the
                                   # lowest slot wall that grips the pin
LOCK_PIN_GRIP = 8 * NOZZLE_D       # 6.4 — pin stub proud below the joint
                                   # bottom: the pliers grab (user #942).
                                   # A CONSTANT bead count past the joint
                                   # (user #986), the nearest bead to a 36
                                   # couch pin: (36 − (FRAME_TOP_Z1 −
                                   # LOCK_JOINT_Z0)) / NOZZLE_D = 8.0
LOCK_PIN_Z0  = LOCK_JOINT_Z0 - LOCK_PIN_GRIP       # −15.2 — pin bottom
LOCK_PIN_L   = FRAME_TOP_Z1 - LOCK_PIN_Z0          # 36.0 couch / 25.6 desk — top
                                                   # flush with frame_top's
                                                   # ACTUAL face at full insert
                                                   # (follows COUCH_TOP_PAD:
                                                   # the pin must still reach
                                                   # the mount head, #955)
assert (LOCK_TOPB_Z1 - _LOCK_TOPB_FOOT - (LOCK_SLOT_X1 - LOCK_X0)
        >= 2 * NOZZLE_D - 1e-9), \
    "top corbel's 45 roof leaves under 1.6 of pin engagement at the slot"
assert LOCK_BOTB_H - (LOCK_SLOT_X1 - LOCK_X0) >= 2 * NOZZLE_D - 1e-9, \
    "beam corbel's 45 underside leaves under 1.6 of slot wall depth"
assert LOCK_TOPB_Z1 - _LOCK_TOPB_FOOT - LOCK_BLK_D >= 0.2 - 1e-9, \
    "top corbel's 45 runs out before its far face"
# TWO LOCKS (user #983 — one lock let the spool twist off its mount): the
# whole stack as placed on the −y beam, plus a copy turned 180° about z
# onto the +y beam. A ROTATION, not a mirror: the copy keeps the lock's
# handedness, so its arm-side slot wall drives the pin against the same
# uninstall twist. The +y arm's screw pad takes the same inset as the −y
# one (mount.py), for the same reason.
LOCK_ROTS = (0.0, 180.0)
# world placement: as-drawn base plane (LOCK_X0) → the −y beam's +x
# flank; as-drawn flank edge (FRAME_RIB/2) → the beam's outer end
LOCK_SHIFT_X = FRAME_RIB / 2.0 - LOCK_X0
LOCK_SHIFT_Y = -FRAME_R_OUT - FRAME_RIB / 2.0
# the −y arm's screw pad: its outboard edge along the arm (the #947
# one-bead inset applied — this pad slid inboard for the deep-pad
# countersink)
_PAD_OUT_R   = (MOUNT_RING_R * math.cos(math.radians(MOUNT_PAD_AZ_OFF))
                + MOUNT_PAD_W / 2.0 - MOUNT_PAD_INSET_MY)   # ≈75.9
# (the top corbel's PAD NOTCH is RETIRED — #983: since the #971 inset
# the pad ends ≥ LOCK_SWEEP_CLR short of the corbel (asserted below), so
# the notch had been cutting nothing, and the couch head drop would
# have folded its outline)
# the lock arms' DEEP pads vs the lock's top corbel (#971): each pad
# ends ≥ the sweep clearance INBOARD of the corbel's face, so it never
# overhangs the lock stack — the mount's old wedge relief and the
# corbel's pad notch are both retired on the strength of this. The pad
# only moves AWAY from the stack during the install sweep.
assert (-(LOCK_SHIFT_Y + LOCK_HEAD_Y1) - _PAD_OUT_R
        >= LOCK_SWEEP_CLR - 1e-9), \
    "the −y arm's screw pad reaches into the lock corbel's sweep clearance"
assert LOCK_HEAD_Y1 - FRAME_RIB / 2.0 <= BEAM_SIZE + 1e-9, \
    "lock stack overruns the beam's footprint along the arm"
# the frame arc joint at this beam: its tenon spans the ENTRY half of
# the crossing and its cavity stop ends TOP_JOINT_SEAT_CLR past the
# beam centre — the corbels' weld face must stay clear of that (the
# stack rides the joint's SOLID half; frame cuts also run after the
# corbel unions, so an overlap could not close the channel anyway)
assert (FRAME_RIB / 2.0 - LOCK_WELD
        >= FRAMEJ_TEN_ARC - BEAM_SIZE / 2.0 + TOP_JOINT_SEAT_CLR
        + NOZZLE_D - 1e-9), \
    "lock corbel weld reaches the frame arc joint's cavity stop"

# ── COUCH beams (the install site's wood — REFERENCE DIMENSIONS) ─────────────
# A 30 sq TOP beam running along X (the mount screws to it), and a FRONT
# beam running along Y that the assembly's +x face meets (levers + cable
# exit). User-measured. The brown DUMMIES that drew them are RETIRED
# (user #981 — they had served their fit-check); the numbers stay
# because real geometry derives from them: the cable exit centres under
# the front beam (HORN_X1) and the tilted mount screws must land inside
# the top beam's width (asserted below).
COUCH_TOP_BEAM_W  = 30.0           # top beam section (y and z), runs along x
COUCH_FRONT_T     = 38.0           # front beam thickness (x)
COUCH_FRONT_H     = 77.0           # front beam height (z)
# the X-axis pads' screws must land on the 30 top beam with the shaft
# fully inside its width (the tilt is what buys this, #959)
assert (MOUNT_SCREW_WOOD_Y + WOOD_SCREW_SHAFT_D / 2.0
        <= COUCH_TOP_BEAM_W / 2.0 - 1e-9), \
    "mount screws enter the wood outside the couch top beam's width"
# CABLE-EXIT +x EXTENT (user #963/#965, couch-gated): the tunnel
# block's +x face. Desk: the frame's outer radius (flush with the frame
# face). Couch: the 10.4 tunnel block CENTRED under the FRONT BEAM —
# the frame sits MOUNT_COUCH_SETBACK behind the beam (the guide-tail
# tips), the beam is COUCH_FRONT_T thick, and the exit runs out under
# it to its mid-thickness. frame.py's horn support (footing, 45° strut,
# bed foot, brace) renders itself from this one number — see _cable_horn.
HORN_X1 = ((MOUNT_TAIL_TIP_X + (COUCH_FRONT_T + BEAM_SIZE) / 2.0)
           if COUCH_VERSION else FRAME_R_OUT)  # ≈115.3 couch / 87.9 desk
                                   # (BEAM_SIZE = HORN_BLOCK_L, the tunnel
                                   # block's x length — defined below)
assert HORN_X1 >= FRAME_R_OUT - 1e-9, "cable exit pulled inside the frame face"

# ── Levers (v2's design "worked well" — constants re-anchored to the v3
# bands; the lever PARTS + kinematics suite land next round, these drive
# the frame's mount + the wall windows now) ──────────────────────────────────
LEVER_T        = 13 * NOZZLE_D     # 10.4 — plate thickness (Y): v2's
                                   # ergonomic 10.0 snapped UP onto the
                                   # bead grid (#933 audit; = BEAM_SIZE)
LEVER_HANDLE_W = 6.0               # handle X width (v2) — CANNOT snap:
                                   # up (6.4) eats the A_PAD_HANDLE_CLR
                                   # guard and the pivot can't move out
                                   # (thrust-ring packaging assert has
                                   # 0.18 of room); a tuned knob pinned
                                   # by that assert pair (bead audit
                                   # #933 — exempted in check_beads)
# RIGID PRINTED PIVOT (user #988). The TPU torsion axle was the pivot
# AND the return spring in one: its softness was the levers' side
# wobble, its low torque the pawl skipping, its creep the vague ends of
# travel. A round PETG pin now. It turns in round bores through the
# fork column and the beam flank, and is KEYED to its lever by a D-flat
# so it turns WITH the lever — which is what lets the return spring (the
# v1 Lee torsion spring, the only hardware) ride the pin coaxially and
# take its moving leg off the pin itself. Printed lying on the D-flat:
# the layers run along the pin and the flat is the bed face.
LEVER_PIN_D      = 8 * NOZZLE_D      # 6.4 — pin Ø (bearing + keyed body)
LEVER_PIN_FLAT   = NOZZLE_D          # 0.8 — (RETIRED at #1011 with the
                                     # D-key; kept only for the trail)
LEVER_PIN_FRAME_CLR = 0.15           # per-side running fit in the frame bores
LEVER_PIN_LEVER_CLR = 0.1            # per-side ROUND fit in the lever at
                                     # the SEAT. Install ease is the PIN's
                                     # job since #1013 (it tapers, so every
                                     # fit opens up the further it is from
                                     # home); this only sets how true the
                                     # lever runs once seated
                                     # (user #1003: 0.05 was too tight to
                                     # assemble — the D-flat keys it, and the
                                     # spring's undercuts hold the pin in)
PIN_TIP_END_Y  = 2 * NOZZLE_D        # 1.6 — the pin's TIP plane |y|: 3.6
                                     # into the beam, for real support (user
                                     # #1014: at 4.4 it reached only 0.8 in)
PIN_TIP_CLEAR  = 5 * NOZZLE_D        # 4.0 — and the blind bore runs this far
                                     # PAST the tip, so the wedge key is
                                     # always what stops the pin. At 0.8 of
                                     # bore the tip bottomed out first and
                                     # the wedge never bit — the pin went in
                                     # far too easily (user #1014)
PIN_TIP_NOSE   = 2 * NOZZLE_D        # 1.6 — and the pin's tip carries a DOME
                                     # this long, so it bottoms on that bore's
                                     # floor PIN_TIP_NOSE short of the full
                                     # PIN_TIP_CLEAR run (user #1016). The
                                     # taper alone never said when to STOP —
                                     # it only got gradually stiffer, so the
                                     # push ran on until the clip crushed
                                     # against the frame. Now the stop is a
                                     # hard one you can feel, and what is left
                                     # of the run (PIN_TIP_CLEAR - this, less
                                     # the head float) is the clip's gap.
                                     # A DOME, not a flat: it touches the bore
                                     # floor at one point on the axis, so the
                                     # stop does not care whether that floor
                                     # printed square. The pin stands on its
                                     # head to print, so the dome is the
                                     # topmost thing on it — free to make.
assert PIN_TIP_NOSE < PIN_TIP_CLEAR, "the dome would not fit in the blind bore"
                                   # (the pin's depth stop): ONE BEAD deep
                                   # into the beam's flank
# (PIN_PRETWIST_DEG, the diamond keyways and the TPU-squish brake rest
# corbel are RETIRED with the torsion axle — #988: there is no install
# lean any more (the round pin just slides in with the lever at rest),
# and both levers now rest and end on HARD printed stops: the ratchet on
# its teeth and its butting rib, the brake on the two ribs its wing
# sweeps between, levers.py.)
# (the #887 CABLE-GUARD chevrons are RETIRED — user #977: with the
# handles bent out of the way the bay WALL comes back instead, and a
# wall closes the bay better than a fence across it. GUARD_LEVER_CLR
# lives on as that wall's clearance under the swept lever.)
GUARD_LEVER_CLR = 3 * NOZZLE_D       # 2.4 — bay wall / lever sweep clearance

# ── CABLE HORN (user #889) — the grab station at the exit window: a
# TUNNEL BLOCK at the frame's +x edge, its top half the separate
# horn_cap part, carried on a strutted FOOTING (A footing / B 45° strut
# / C bed foot / D sill brace — frame.py's _cable_horn). Through the
# block: a round TUNNEL for the cable and a FULL-CIRCLE bell at EACH
# mouth (kept identical for simplicity, user: the +x exit needs the
# full circle — the free cable pulls in any direction with no edge to
# bite — and the −x entry just reuses it, though the spool-plane cable
# never technically needs the top/bottom quadrants there). The design
# history — solid pier #889, ideal-model tunnel #903, equator split
# #905, pier retired for the strut set #914-#921 — lives in git.
HORN_CH_W  = snap(1.5 * CABLE_D,
                  NOZZLE_D, "up")    # 6.4 — tunnel bore Ø: 1.5 × cable Ø,
                                   # rounded UP to the bead grid (user).
                                   # (The #923–#927 fat-cable demo — an
                                   # exit-only OD override at 4.8 then
                                   # 10.0 — proved the derived footing
                                   # depth + chamfer rules track any
                                   # bore; override retired at #928)
HORN_WALL_T = 4 * NOZZLE_D           # 3.2 — meat beside the tunnel (grew
                                   # with the #946 bell: the side flats
                                   # stay one bead, WALL_T = BELL_R + 0.8)
HORN_W      = HORN_CH_W + 2 * HORN_WALL_T      # 12.8 — block width
HORN_ROOF_T = 4 * NOZZLE_D           # 3.2 — meat above the tunnel.
                                   # Sized so the cap keeps ≥ 1.6 over
                                   # the rail cavities (user #908), with
                                   # the cavity swallow at 4.66 now that
                                   # the mushroom's z-relief rides the
                                   # tenon post (#909's tier fix): roof
                                   # left 1.74
HORN_BLOCK_H = HORN_CH_W + HORN_ROOF_T         # 9.6 — block height above
                                   # the sill (named at #932 — it was
                                   # spelled out at four frame.py sites)
HORN_BLOCK_L = BEAM_SIZE           # 10.4 — tunnel block length along x,
                                   # flush with the +x frame edge (user
                                   # #915: LOCKED to the beam/axle-
                                   # support size so the 45° strut's top
                                   # face carries it exactly — no
                                   # overhang, no exposed ledge): two
                                   # bells + straight middle
HORN_BELL_R = 3 * NOZZLE_D           # 2.4 — mouth bell, BOTH ends: a
                                   # quarter-round trumpet revolved
                                   # around the tunnel axis (tangent to
                                   # the bore, tangent to the face);
                                   # leaves NOZZLE_D dull flats at the
                                   # block's side faces. GENTLER at
                                   # #946 (user: softer in/out taper —
                                   # was 1.6): the LARGEST bead multiple
                                   # the cap rail allows — the rail
                                   # room assert in frame.py caps it at
                                   # 3.05 (HORN_BLOCK_L/2 − HORNJ half
                                   # slot); the footing depth and the
                                   # corner-chamfer twins re-derive
# cap ATTACHMENT (user #906, single rail #915): ONE central cadkit
# MUSHROOM rail (hosts print in opposite z directions), install sliding
# − to + y. Two TENON segments rise off the trough strips' tops (2.4
# engagement each); the CAP carries one through-CAVITY across its full
# width — open at the +y face (entry), run past the far face so the
# STOP FLANGE, not the cavity end wall, defines seating. The flange:
# extra material on the CAP ONLY (user: don't grow the bottom prism),
# at its trailing −y end, hanging past the equator to catch the
# trough's −y face and stop +y over-travel. Backing out −y has no
# shape retention yet (cadkit doctrine: stop one way, preload the
# other).
HORNJ_W      = 5 * NOZZLE_D          # 4.0 — joint width room (across x);
                                   # mushroom swallow 4.46 at the 0.20
                                   # sandwich, roof left 1.94
HORNJ_Z_CLR  = 0.20                # rail z-sandwich clearance, THIS joint
                                   # only (user #912: same treatment as
                                   # the lid's LIDJ_Z_CLR — short joinery
                                   # that must STAY PUT runs tighter than
                                   # the GF 0.30 policy; 0.20 is the
                                   # lid's print-proven value)
HORN_STOP_T    = 3 * NOZZLE_D        # 2.4 — stop flange thickness (y)
HORN_STOP_DROP = 3 * NOZZLE_D        # 2.4 — flange catch depth below the
                                   # equator plane (contact patch on the
                                   # trough's −y face)
# (HORN_YC — the horn's centreline — is defined AFTER the exit window:
# the WINDOW derives purely from the cable/coil geometry, and the horn
# then sits at its midway point — user's design order, #899)
# (HORN_Z1 = the window bottom — set beside CABLE_EXIT_Z0 below)
POST_OUT_T = snap(BEAM_SIZE / 2.0,
                  NOZZLE_D, "up")    # 5.6 — side-column / bent-arm section:
                                   # the half-beam ROUNDED UP to the bead
                                   # grid (user #914: BEAM_SIZE/2 = 5.2
                                   # was a half-bead width)
BOSS_RING_W    = contact_rib_size(NOZZLE_D)      # 1.6 — thrust ring width & proud
BOSS_RING_CUT  = 0.2               # trimmed off each ring's proud face
                                   # (user #1014: at 0.05 of side play the
                                   # rings pinched the brake lever hard
                                   # enough that the TPU could not return
                                   # it; 0.25 lets it swing)
BOSS_BORE_ID   = LEVER_PIN_D + 2 * LEVER_PIN_FRAME_CLR   # 6.7 — the pin's
                                   # running bore (thrust rings + frame)
LEVER_SIDE_CLR = BOSS_RING_W + 0.05   # 1.65 — ring + 0.05 air each side
STOP_LEVER_CLR = 0.15              # the stop ribs' free faces, off the
                                   # lever's PLATE (user #1016: only the
                                   # lever's stop wing should ever touch
                                   # them; at 0.05 the plate itself grazed)
                                   # of the lever plates (user #988 — the
                                   # side wobble: 0.1 → 0.05, the columns
                                   # pull in; was 0.2 before #944)
LEVER_Y_IN     = BEAM_SIZE / 2.0 + LEVER_SIDE_CLR  # 6.9 — inner plate face
RATCHET_LEV_Y0 = LEVER_Y_IN                        # ratchet plate on +Y side
RATCHET_LEV_Y1 = LEVER_Y_IN + LEVER_T              # 17.3
BRAKE_LEV_Y1   = -LEVER_Y_IN                       # brake plate on −Y side
BRAKE_LEV_Y0   = -LEVER_Y_IN - LEVER_T             # −17.3
LEVER_PIVOT_X  = (BEAM_IR + BOSS_BORE_ID / 2.0 + BOSS_RING_W
                  + (BEAM_SIZE - BOSS_BORE_ID - 2 * BOSS_RING_W))   # 82.93 —
                                   # v2's derivation, pushed to the beam's
                                   # limit at #1005 (outer ring flush with
                                   # the +x face; was + half a bead):
                                   # (rings flush at the beam's inner edge)
                                   # + an OUTBOARD offset: v3's larger
                                   # brake contact angle leans the pad's
                                   # rest-plumb joint plane closer to the
                                   # handle, and the offset restores the
                                   # 0.45 guard (A_PAD_HANDLE_CLR); the
                                   # ring/diamond packaging asserts below
                                   # still pass
LEVER_BOSS_OD  = (LEVER_PIN_D + 2 * LEVER_PIN_LEVER_CLR
                  + 2 * STRUCT_WALL)               # 9.9 — lever pivot boss
# Pivots re-anchored to the v3 band stack (FLIPPED vs v2 — brake band on
# the BOTTOM now, print-driven): ratchet pivot 10 above its band's top
# (v2's rule, rest = pawl seated in the teeth, pull swings it out). The
# BRAKE pivot goes 5.5 BELOW its band (v2's relative spot): the pad must
# sit ABOVE its pivot or pulling would swing it AWAY from the band (an
# above-band pivot shipped first — kinematically backwards). The pivot
# hardware lives OUTSIDE the wall and the lever bays are full-height, so
# the low pivot breaches nothing.
RATCHET_PIVOT_Z = SEP_Z1 + 10.0                    # −24.4
BRAKE_PIVOT_Z   = SEP_Z0 - 5.5                     # −49.5 — below the band
LEVER_TRAVEL_DEG = 20.0            # equal pull travel, both levers (v2)
RATCHET_OPEN_CLR = 1.0               # the OVER-PULL STOP pose (user;
                                     # cut from 1.6 at #1018: the lever swung
                                     # 6° past the point the pawl is clear,
                                     # opening the gap to the brake's handle
                                     # for nothing. It can go no tighter —
                                     # this IS lever_kinematics' pawl-to-tooth
                                     # target PAWL_CLEAR_MM, which params
                                     # cannot import back — levers.py asserts
                                     # the two stay equal, A_PAWL_CLEAR):
                                   # the pull angle where the pawl clears
                                   # the tooth tips by this much is where
                                   # the frame's RIB catches the pawl's wing
                                   # (RATCHET_OPEN_DEG derives in levers.py,
                                   # rib in frame.py). Cut from 2.4 at #979:
                                   # 2.4 put the stop at 20.0°, the whole
                                   # modeled travel and ~2x the 10.6° the
                                   # pawl needs to clear the teeth — all of
                                   # that over-travel was creeping the TPU
                                   # axle. 1.6 stops it at 17.0° and still
                                   # doubles the 1.0 the kinematics demand.
HANDLE_Z_BOT     = FLOOR_Z0 + LEVER_HANDLE_W / 2.0   # ≈ −75.1 — grip-disc
                                   # centre placed so the handles' round
                                   # ends land FLUSH with the assembly's
                                   # −z extent (user's call); pulling only
                                   # raises them
# BENT HANDLES (user #966 — easier to reach; COUCH-gated, #967): below
# the pivots the handles no longer drop straight to the grip. Each runs vertically
# from its pivot down to LEVER_BEND_Z — the BRAKE pivot's own height,
# so the brake's bend is AT its axle and the ratchet's vertical stretch
# ends level with it — then 45° toward +x −z to the unchanged
# HANDLE_Z_BOT. Both 45° runs start from the same (x, z), so both grip
# discs land at the same spot, HANDLE_X_BOT.
LEVER_BEND_Z = BRAKE_PIVOT_Z                     # −49.5 — the shared knee
HANDLE_X_BOT = (LEVER_PIVOT_X + (LEVER_BEND_Z - HANDLE_Z_BOT)
                if COUCH_VERSION else LEVER_PIVOT_X)   # ≈108.4 couch —
                                   # grip-disc centre x (45°: the run's
                                   # drop equals its reach); DESK: under
                                   # the pivot = the straight handle
RATCHET_PHASE_DEG = math.degrees(math.asin(
    ((RATCHET_LEV_Y0 + RATCHET_LEV_Y1) / 2.0) / (RIM_OD / 2.0)))   # ≈ 9.95 —
                                   # one catch face lands at the pawl's
                                   # y-midline (v2's rule; the knurl is
                                   # phase-locked to the same value)
# Brake pad (95A TPU) + its cadkit hook slide joint (v2's print-proven set)
BRAKE_RUBBER_T       = 3.3         # pad slab thickness (v1/v2)
PAD_JOINT_CLR        = 0.1         # TPU in PETG — snug (v2)
BRAKE_PAD_TOP_MARGIN = 0.0         # ZERO margins (v2's full-band pad —
BRAKE_PAD_BOT_MARGIN = 0.0         # contact-only doctrine)
PAD_SPEC      = PrintSpec(nozzle=NOZZLE_D, facing="up")
PAD_FLANGE_H  = joint_box_min(PAD_SPEC, PAD_SPEC, install="-z", bounded=True,
                              quality=True, clearance=PAD_JOINT_CLR)[0]
                                   # 6.6 — the edge-bounded site's quality
                                   # width = the pad flange / arm end face
PAD_FLANGE_T  = 2 * NOZZLE_D         # flange depth (the arm face recesses by it)
LEVER_COL_Y1 = RATCHET_LEV_Y1 + LEVER_SIDE_CLR + POST_OUT_T   # 24.5 — the
                                   # fork column's OUTER face, |y|
# RETURN CLIPS (user #1004 — TPU again, as a LEAF). The Lee springs
# (#988–#1003) are retired: their seats and hooks needed features a
# 0.8 nozzle in PETG-GF can't make. Each lever's pivot pin is the plain
# keyed D-rod; outside the fork column it carries a printed 95A TPU
# RETURN CLIP — a hub keyed onto the pin's D, and a LEAF hanging down the
# column's outer face to a round POST on the frame. The leaf is printed
# CLIP_PRELOAD past the post, so at rest it presses the lever home (the
# pawl into the teeth, the brake wing onto its rest rib); pulling turns
# the pin and bends the leaf harder against the post. BENDING, not the
# old TPU torsion axle's twist (too little force, and it crept) — force
# goes as the leaf's thickness cubed, so CLIP_LEAF_T is the tuning knob,
# and the long leaf keeps the resting strain low. The pin's round head
# retains the clip. No purchased parts at all now. Force model in
# levers.py (straight-beam estimate; TPU_E is nominal — print-tune on
# the lever test rig).
# PIVOT KEY (user #1011 — the D-flat's 0.15 fit was 4.1° of rotational
# slop in the lever and 2.7° in the clip: ~7° of dead travel before the
# TPU leaf even began to bend, and the fit was already at the limit of
# what could be pushed together). A WEDGE key instead: the pin carries a
# SLOT, the lever's pocket and the TPU clip carry 1.6-wide RIBS (two
# beads — printed at size, not fitted). The slot runs wide for most of
# the pin, so the rib slides in free, then narrows to PIN_KEY_W over
# PIN_KEY_L: the lever's rib wedges there, both its sides bearing, and
# the play goes to nil. A taper turns a FIT problem into a DEPTH one —
# print error moves where the pin seats (±PIN_KEY_FLOAT), not how tight
# it is — so the clip is located off the column's FACE, not off the
# pin's head, and the head keeps a loose gap.
PIN_KEY_W       = 2 * NOZZLE_D     # 1.6 — the rib, and the slot at the seat
PIN_KEY_H       = 2 * NOZZLE_D     # 1.6 — the rib's height
PIN_KEY_L       = LEVER_T          # 10.4 — the wedge's run: the FULL lever
                                   # (user #1012 — it bears evenly and the
                                   # rib runs out through the inner face;
                                   # the shallower wedge floats a touch more,
                                   # which only moves the head's slack gap)
PIN_KEY_WIDEN   = NOZZLE_D         # 0.8 — the lever rib's extra width at its
                                   # inboard end: the wedge angle (≈3.6°/side)
PIN_KEY_SLOT_H  = PIN_KEY_H + 0.2  # 1.8 — slot depth (the rib never bottoms)
# the slot's taper MATCHES the rib's, so at the seat they bear over the
# rib's whole length — and it runs ALL THE WAY to the tip (user #1013: a
# constant free run the same width as the rib's wide end meant the rib
# scraped the slot over the whole push; tapered, the rib's clearance is
# proportional to how far the pin still has to go)
PIN_KEY_RATE    = PIN_KEY_WIDEN / PIN_KEY_L   # 0.077 — width per mm
PIN_SHANK_UNDER = 0.2              # the pin's SHANK — everything inboard of
                                   # the column's bore — runs this far under
                                   # nominal, rising to full Ø only over its
                                   # last stretch (user #1013: the round fit
                                   # must never be the tight part; the wedge
                                   # key is what grips at the end). It leaves
                                   # the lever 0.2/side on the round, the
                                   # blind bore 0.25, the column's 0.15 and
                                   # the TPU's hub 0.1 — all at the seat
PIN_KEY_FLOAT   = 2 * NOZZLE_D     # 1.6 — how far the seat may shift with
                                   # print error, each way (±0.1 of width
                                   # error over the wedge's ≈2.2°/side)
PIN_CLIP_SLOT_W = PIN_KEY_W - 0.5  # 1.1 — the slot outboard of the seat: the
                                   # TPU clip's rib is 0.25/side wider, so it
                                   # is squeezed home (user #1014: at 0.1 it
                                   # still slid on far too easily)
PIN_CLIP_RIB_W  = PIN_KEY_W        # 1.6 — two beads, like the lever's: it
                                   # slides down the wedge zone (same width,
                                   # TPU) and squeezes into the clip slot
PIN_CLIP_SLOT_FREE = PIN_CLIP_RIB_W + 0.1     # 1.7 — the slot where the TPU
                                   # clip merely SLIDES; it squeezes only in
                                   # the stretch its hub lands on (user #1013:
                                   # the squeeze ran the pin's whole length)
PIN_CLIP_SQUEEZE_RUN = 3 * NOZZLE_D    # 2.4 — how far the squeeze closes over
PIN_KEY_SEAT_UNDER = 0.15          # the slot is this much NARROWER than the
                                   # rib at the nominal seat, so the wedge
                                   # bites ≈2 mm early — guaranteed, whatever
                                   # the print does (user #1014)
PIN_HUB_OVER    = 0.1              # the pin's Ø under the TPU hub, OVER
                                   # nominal: 0.05/side on the printed bore
PIN_JOURNAL_UNDER = 0.1            # …and its Ø where the frame's bores hold
                                   # it, UNDER nominal — 0.2/side running fit
                                   # (user #1015: at 0.15 the printed pin had
                                   # noticeable drag turning in the rig; a
                                   # standing-printed 35 mm pin need only
                                   # lean a little to fight two bores 20 apart)

TPU_E           = 26.0             # N/mm² — 95A TPU's nominal modulus
                                   # (datasheet class; not a length)
CLIP_GAP        = 2 * NOZZLE_D     # 1.6 — clip's inner face off the column
                                   # (user #1016: at 0.1 the clip ended up
                                   # flush on the frame and rubbed it as the
                                   # lever swung. It is the pin's HEAD that
                                   # sets where the clip sits, so a longer
                                   # pin simply holds it off the face)
CLIP_T          = 14 * NOZZLE_D    # 11.2 — clip thickness (the leaf's width
                                   # in bending). WIDER since #1017 (user
                                   # wanted more return force): width buys
                                   # force in direct proportion and costs
                                   # NO extra strain — thickness would buy
                                   # it as the cube but raise the strain in
                                   # step, and strain is what creeps
CLIP_HUB_R      = 7 * NOZZLE_D     # 5.6 — clip hub radius round the pin
CLIP_BORE_CLR   = 0.1              # TPU round on the pin, per side (the
                                   # rib keys it; see PIN_CLIP_RIB_W)
CLIP_LEAF_T     = 4 * NOZZLE_D     # 3.2 — leaf thickness: THE force knob
CLIP_LEAF_X     = -2.5             # leaf centreline off the pivot, x: the
                                   # post lands inside the column's face
CLIP_CONTACT_R  = 31 * NOZZLE_D    # 24.8 — post contact off the pivot
                                   # (#1008: LONG — the up-pointing brake
                                   # leaf freed the length; a longer leaf
                                   # rests at lower strain and its force
                                   # climbs less over the pull. Was 17.6)
                                   # (20.0 until #1007 — raised 2.4, user:
                                   # the brake's post then grows out of the
                                   # wall above the hand-access cutaway,
                                   # its gusset trimmed by the cut's 45°)
CLIP_TIP_PAST   = 2 * NOZZLE_D     # 1.6 — leaf runs on past the contact
RATCHET_CLIP_ROT_DEG = 0.0         # each lever's clip layout, turned about
BRAKE_CLIP_ROT_DEG = 180.0         # its pivot off "leaf hanging down"
                                   # (user #1008: the brake's leaf points
                                   # UP — hanging down, its post ran into
                                   # the frame's hand-access cutaway; up,
                                   # the leaf can be as long as it wants)
CLIP_PRELOAD    = 7 * NOZZLE_D     # 5.6 — leaf printed this far into the
                                   # post, at the contact
CLIP_AX_CLR     = 0.2              # clip ↔ pin head, axial play
POST_R          = 3 * NOZZLE_D     # 2.4 — the post's round nose
PIN_HEAD_R      = CLIP_HUB_R       # 5.6 — pin head: caps the clip's hub
PIN_HEAD_T      = 3 * NOZZLE_D     # 2.4 — pin head thickness
LEVER_PIN_BODY_L = (LEVER_COL_Y1 + CLIP_GAP + CLIP_T + PIN_KEY_FLOAT
                    + CLIP_AX_CLR - PIN_TIP_END_Y)   # blind-bore floor →
                                   # the head's face (the float keeps the
                                   # head off the clip at the deepest seat)
LEVER_PIN_L = LEVER_PIN_BODY_L + PIN_HEAD_T
RETURN_TORQUE_MIN = 15.0           # N·mm at rest — at least the Lee spring's
RETURN_TORQUE_MAX = 120.0          # N·mm at full pull — ~3 N at the handle

# ── Wall openings (v2's derivations, v3 anchors) ─────────────────────────────
_WALL_TOP_BAND_RETIRED = 4 * NOZZLE_D   # (kept only for the comment trail —
                                   # wall_top itself is retired; the band
                                   # runs to LID_Z1 and the openings close
                                   # with 45° gables/teeth instead) (user's
                                   # call; was 1.6)
# cable EXIT window at (+x,+y) — RE-DERIVED for the HORN (user #893:
# v2's 30° arc assumed a free tangential fly-off; the horn REDIRECTS
# every exit through its guide mouth). Defined the user's way: the
# Y-BAND the cable can take up — sweep the wind range, take each
# state's tangent line to the mouth (tangency on the CW-driving side,
# the pinned chirality), collect the WALL-CROSSING y values on both
# wall faces, add cable margins, and cut the wall STRAIGHT between
# those two y planes:
# THE MODEL (user's drawing, #898 — the PARALLEL EXIT): the cable
# peels at the top of its wind circle, (0, r), and runs straight along
# +x at y = r through the horn's guide. One line per wind state, so
# the window is simply the band between the innermost and the
# outermost winds' tangent lines, ± the cable's half-width — the EXACT
# take-up band, no buffer (any buffer gets added explicitly later).
# The outer bound is PHYSICAL: the cable can wind to the start of the
# ratchet teeth (not just design capacity — the rim is tunnel-floored).
_R_WIND_MAX = RIM_OD / 2.0 - RATCHET_DEPTH
CABLE_EXIT_Y_LO = WRAP_R0 - CABLE_D / 2.0
CABLE_EXIT_Y_HI = _R_WIND_MAX + CABLE_D / 2.0
# (the taut-line alternative — cable aimed at the guide mouth rather
# than parallel — dips ~1.6 further −y at the empty-spool end; add it
# as explicit buffer later if the print shows the taut path matters)
HORN_YC = (CABLE_EXIT_Y_LO + CABLE_EXIT_Y_HI) / 2.0   # 58.8 — the horn rides
                                   # the WINDOW's midway point (the original
                                   # spec, restored: window first, horn
                                   # second — user's design order)
assert CABLE_EXIT_Y_LO > BEAM_SIZE / 2.0 + 2.0, \
    "exit window's low-y edge reaches the +X beam"
CABLE_EXIT_Z0 = SEP_Z1 - 0.5                       # −34.9 — port floor
HORN_Z1 = CABLE_EXIT_Z0            # cable-horn pier top = the window bottom
                                   # (user #889; HORN_* block above)
CABLE_EXIT_Z1 = LID_Z0 + 0.5                       # −29.9 — reference ceiling
                                   # (anchors WALL_SPLIT_Z below; the port
                                   # itself now cuts OPEN through the band's
                                   # top rim — user: the sawtooth roof was an
                                   # unnecessary overhang, leave the top open)
WALL_SPLIT_Z  = (CABLE_EXIT_Z0 + CABLE_EXIT_Z1) / 2.0   # −32.4 — stack plane
# Lever WINDOWS at +X (y edges = plate faces + LEVER_SIDE_CLR; the pivot
# boss never enters the wall's radial band at these numbers — v2's
# _BOSS_WALL_DZ evaluates 0 — so the far z edges keep 1.0 off the pivots).
LEVER_WIN_Y0 = LEVER_Y_IN - LEVER_SIDE_CLR         # 5.2
LEVER_WIN_Y1 = RATCHET_LEV_Y1 + LEVER_SIDE_CLR     # 18.8
# Ratchet OVER-PULL STOP = the window SILL (v2's design): derived so the
# pawl block's bottom edge lands on it at RATCHET_STOP_DEG.
RATCHET_STOP_DEG = 15.0
_STOP_S = math.sin(math.radians(RATCHET_STOP_DEG))
_STOP_C = math.cos(math.radians(RATCHET_STOP_DEG))
_X_STOP = math.sqrt(WALL_IR ** 2 - RATCHET_LEV_Y1 ** 2)        # 68.96
_DZ_BOT = SEP_Z0 + BRAKE_H - RATCHET_PIVOT_Z       # −14.8 — tooth-band bottom
                                                   # (the pawl block's rest
                                                   # bottom) off the pivot
_DX_STOP = (_X_STOP - LEVER_PIVOT_X + _DZ_BOT * _STOP_S) / _STOP_C
RATCHET_WIN_Z0  = (RATCHET_PIVOT_Z + _DX_STOP * _STOP_S
                   + _DZ_BOT * _STOP_C)            # ≈ −42.2 — the FORMER wall
                                                   # sill (= the derived over-
                                                   # pull stop face): the wall
                                                   # below the windows is GONE
                                                   # (user's call, #815) — this
                                                   # stays as the reference z
                                                   # for the lever's future
                                                   # stop tab
# wall verticals — ONE fused band, bed → the LID'S TOP (user's call: the
# whole wall is frame_bottom now; the bay windows AND the exit port run
# open through the top rim — only the entry port keeps a roof, wall.py)
WALL_Z1 = LID_Z1                                   # −25.6 — band top = lid top
# (the lever-bay gable crown is RETIRED — user's call: the ring between
# the levers and the lid was an overhang; the bay windows now run
# full-height through the band's top rim, wall.py)
# (the exit-port sawtooth ceiling is RETIRED — user's call: an
# unnecessary overhang; the port cuts open through the band's top rim)
assert (LEVER_PIVOT_X + BOSS_BORE_ID / 2.0 + BOSS_RING_W
        <= BEAM_IR + BEAM_SIZE + 1e-9), \
    "thrust rings poke past the frame's +x face — shrink LEVER_PIN_D"
assert ((BEAM_IR + BEAM_SIZE) - (LEVER_PIVOT_X + BOSS_BORE_ID / 2.0)
        >= 2 * NOZZLE_D - 1e-9), \
    "under 1.6 of beam between the pin bore and the frame's +x face"

# ── Consistency guards (fire at import) ──────────────────────────────────────
assert AXLE_PRINT_D <= BEARING_ID, \
    "printed axle must be undersized vs the metal bearing bore"
assert BEARING_ID + 1.0 < STUB_BOSS_D <= BEARING_IRACE_OD, \
    "stub boss shoulder must land fully on the inner race's face annulus"
assert BRG_LIP_ID < BEARING_OD - 1.0, \
    "frame lip must lap the outer race by a real margin"
assert BRG_LIP_H >= STRUCT_WALL, \
    "bearing lip thinner than the quality tier"
assert AXLE_Z_TOP <= FRAME_Z1, \
    "axle lip pokes above the frame top — the assembly top must stay flat"
# (sexes swapped with the upside-down spring: collar on TOP, tenon below)
assert (SPRING_BORE_TOP_D - JOINT_MORTISE_OD) / 2.0 >= 0.5, \
    "mortise collar under 0.5/side rotating clearance in the top flange hole"
assert (SPRING_BORE_BOT_D - AXLE_PRINT_D) / 2.0 >= 0.3, \
    "tenon post under 0.3/side rotating clearance in the bottom flange hole"
assert JOINT_Z0 >= SPRING_Z0 + SPRING_FLANGE_T - 1e-9, \
    "collar bottom (mortise opening) reaches below the bottom flange interior"
assert SHOULDER_Z <= SPRING_Z0 - 1e-9, \
    "separator's column shoulder touches the spring's bottom face"
assert AXLE_ROOT_CONE_H <= SEP_SPRING_CLR, \
    "axle root cone pokes past the spring's bottom face"
# strip-zone coverage: the separator's slit floor (shrinkage margin
# included) must stay above the axle root cone
assert SLIT_ZLO >= SEP_Z1 + AXLE_ROOT_CONE_H - 1e-9, \
    "separator slit floor reaches down into the axle root cone"
assert STRIP_Z0 < STRIP_Z1, "spring strip zone inverted"
