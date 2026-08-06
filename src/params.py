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

NOZZLE      = 0.8            # THE print-width unit (0.8 nozzle, v2's call)
STRUCT_WALL = 2 * NOZZLE     # 1.6 — the two-bead quality tier
FIT_CLR     = 0.15           # per-side slip-fit clearance (v2's proven value)

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
CABLE_D        = 3.85        # working-cable Ø (user's measurement,
                             # bumped from 3.0 at #883)
CABLE_CAPACITY = 1000.0      # 1 m of wrapped cable (user's call at #884;
                             # was 6 ft / 1828.8)

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
FRAME_RIB   = 13 * NOZZLE    # 10.4 — plus-arm section (was v2's 10.0 even;
                             # user's call: onto the bead grid)
# (FRAME_R_OUT is DERIVED in the wall/beam block below: arm tips flush
# with the beams' outer faces, v2's pattern.)
FRAME_Z0 = 0.0               # frame underside — the v3 z datum
FRAME_Z1 = FRAME_Z0 + FRAME_RIB

# Bearing pocket: drops in from the TOP onto a lip UNDER the outer race —
# the hanging load (axle + spring + future drum) pulls the bearing DOWN
# onto it. The lip is the quality tier, which RECESSES the bearing 1.8
# below the frame top so the axle's lip tops out UNDER the frame face:
# the assembly's top is FLAT (user's call — was lip 3.4 / bearing flush /
# axle lip 1.6 proud).
BRG_BORE      = BEARING_OD + BEARING_CLR       # 22.2 — pocket bore
BRG_LIP_H     = 2 * NOZZLE                     # 1.6 — retention lip height
BRG_POCKET_Z0 = FRAME_Z0 + BRG_LIP_H           # 1.6 — pocket floor (lip top)
BRG_Z1        = BRG_POCKET_Z0 + BEARING_W      # 8.6 — bearing top (recessed)
BRG_LIP_ID    = BRG_BORE - 2 * STRUCT_WALL     # 19.0 — retention lip bore (laps
                                               # the outer race; grazes the seal
                                               # annulus by 0.2 — v2-accepted: the
                                               # seal sits on the outer ring, no
                                               # relative motion at that contact)
BRG_BOSS_WALL = 4 * NOZZLE                     # 3.2 — hoop wall around the pocket
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
SLIT_INSET = 4 * NOZZLE            # 3.2 — slit-end inset from the spring edges
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
assert JOINT_Z1 <= TOP_CONE_Z0 - NOZZLE + 1e-9, \
    "no web between the bore ceiling and the collar cone"
# the separator's column→tenon SHOULDER: the fat column cannot pass the
# (now bottom) Ø8.7 hole, so it stops under the spring with a rotating gap
TENON_SHOULDER_CLR = 0.5
SHOULDER_Z = SPRING_Z0 - TENON_SHOULDER_CLR        # −32.5 — column top / post root
SLIT_W               = 2 * NOZZLE                  # spring-strip slit width
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
RATCHET_DEPTH = 3 * NOZZLE         # 2.4 — tooth tip→valley (v2 print-proven)
RATCHET_TEETH = 60
# (RATCHET_PHASE_DEG — the tooth/knurl phase — is DERIVED in the lever
# block below: one catch face lands at the pawl's y-midline, v2's rule.)
BRAKE_H       = 6 * NOZZLE         # 4.8 — knurled brake band height (was
                                   # v2's 5.0 even; user: onto the bead grid)
RATCHET_H     = 6 * NOZZLE         # 4.8 — tooth band height (ditto)
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

SEP_SPRING_CLR = 3 * NOZZLE        # 2.4 — separator↔spring gap (was v2's 3.2
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
AXLE_Z_TOP = BRG_Z1 + 2 * NOZZLE       # 10.2 — shaft top, 1.6 proud of the
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
ANCH_T  = 3 * NOZZLE               # 2.4 — wall thickness (user: up from the
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
ANCH_WIN_PASS = 23 * NOZZLE        # 18.4 — entry-wall height (head-passing)
ANCH_BAY_BIAS = 0.3                # bay centre bias below the float's top
ANCH_WIN_Z1  = (_STRIP_C_HI + ANCH_WIN_HOLD / 2.0
                - ANCH_BAY_BIAS)                     # −9.2 — flat top edge
ANCH_WIN_Z0  = ANCH_WIN_Z1 - ANCH_WIN_PASS           # −27.6 — the VIRTUAL
                                   # apex at the +y entry wall: the 45°
                                   # floor aims here, but the tip is
                                   # DULLED (user's call) —
ANCH_TIP_Z   = ANCH_WIN_Z0 + NOZZLE                  # −26.8 — a 0.8 flat
                                   # truncates the corner: one bead
                                   # bridges it in print
ANCH_HOLD_Z0 = ANCH_WIN_Z1 - ANCH_WIN_HOLD           # −18.2 — bay bottom
ANCH_WIN_W   = ANCH_WIN_PASS - ANCH_WIN_HOLD         # 9.4 — width: 45° floor
                                                     # bay-corner → virtual apex
                                                     # (wider with the 9 bay —
                                                     # the wall grows with it)
ANCH_Z0      = ANCH_TIP_Z - 4 * NOZZLE               # −30.0 — wall bottom
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
    (ANCH_WIN_Y0 + ANCH_WIN_W + 3 * NOZZLE) / ANCH_IR))              # ≈ 25.1°
                                   # (+y pier 3 beads — 6 put the wing at
                                   # 28.9° where a mount SPOKE clipped it
                                   # at the install-overshoot margin; the
                                   # LOADED pier is the beam-backed side)

# The pass/block mechanism IS these inequalities:
assert ANCH_WIN_Z1 - ANCH_TIP_Z >= STRIP_HEAD_W + 0.4 - 1e-9, \
    "head doesn't pass flat-on at the entry wall (tip flat included)"
assert (NOZZLE + (ANCH_WIN_Z1 - ANCH_TIP_Z) - (STRIP_HEAD_W + 0.4)
        >= 2 * NOZZLE - 1e-9), \
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
assert ANCH_Z0 - SEP_Z1 >= 3 * NOZZLE - 1e-9, \
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
DRUM_ANCH_CLR = NOZZLE             # 0.8 — drum bore off the anchor wall (user)
DRUM_IR = ANCH_OR + DRUM_ANCH_CLR              # 43.35 — wall bore = lid bore
DRUM_T  = 8 * NOZZLE               # 6.4 — X = 8: the smallest bead multiple
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
                   + RATCHET_DEPTH + 2 * NOZZLE)
RIM_OD = max(_RIM_COIL, _RIM_PASS)

LID_SEP_GAP = (math.ceil(CABLE_D / NOZZLE - 1e-9)
               * NOZZLE)           # 4.0 — separator↔lid gap = the spool
                                   # chamber height: one flat cable layer,
                                   # the Ø ROUNDED UP to the next bead
                                   # multiple (user's rule at #885 — was a
                                   # hardcoded 4.0 that silently lost its
                                   # slack when the cable grew to 3.85;
                                   # derived, it tracks the cable)
LID_T   = 6 * NOZZLE               # 4.8 — user's stated minimum; it can stay
                                   # there because the cavities live in the
                                   # WALL, not the lid
LID_Z0  = SEP_Z1 + LID_SEP_GAP                 # −30.4 — lid underside
LID_Z1  = LID_Z0 + LID_T                       # −25.6 — lid top
# the anchored strip's T-head pokes past the wall to r ≈ 55, over the
# rotating lid's rim — its bottom edge must clear the lid's top face
# both SEATED (the bay floor sets the lowest ride) and mid-THREAD (the
# dulled tip sets the lowest transient dip):
assert ((max(_STRIP_C_LO, ANCH_HOLD_Z0 + STRIP_NECK_W / 2.0)
         - STRIP_HEAD_W / 2.0) - LID_Z1 >= 2 * NOZZLE - 1e-9), \
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
TOPJ_STEM  = 3 * NOZZLE            # 2.4 — stem width (radial, off the flat)
TOPJ_FLARE = 2 * NOZZLE            # 1.6 — the single outboard 45° flare
TOPJ_NECK  = 2 * NOZZLE            # 1.6 — neck height under the flare
TOPJ_TIP   = NOZZLE                # 0.8 — vertical tip face above the flare
                                   # (then the FLAT top)
LIDJ_SITES      = 4                # joint sites (90° pitch)
LIDJ_TEN_SWEEP  = 30.0             # tenon arc sweep (deg) — was 20 (user:
                                   # lid↔sep too loose; the drum rim has
                                   # free arc, so the engagement grew 1.5×)
LIDJ_SEAT_CLR   = 0.15             # seating clearance at each stop (arc mm)
LIDJ_ENTRY_OVER = 1.0              # pocket overshoot past the tenon, each
LIDJ_Z_CLR      = 0.20             # z-sandwich clearance, THIS joint only
                                   # (user: drop the vertical slack a bit —
                                   # under the GF 0.30 depth-face policy;
                                   # print-check the seating twist for grab)
                                   # side (arc mm) — drop-in ease

# 45° pass tunnel: FLUSH on the drum wall (user #896 — the old one-bead
# web served nothing; inward is friendlier to the tooth-root keep-out)
SEP_PASS_R = DRUM_OR + SEP_PASS_W / 2.0

# lid body: v2 cable_ceiling's pattern — spokes + graded concentric rings
LID_SPOKE_N   = 24                 # spoke count (v2's call)
LID_SPOKE_W   = 4 * NOZZLE         # 3.2 — web width, 4 beads (v2)
LID_RING_IN_R1 = DRUM_OR           # inner solid band: bore → wall OR (covers
                                   # the tenon ring + its root, sits exactly
                                   # over the wall)
LID_RIM_R0    = LID_OD / 2.0 - 6 * NOZZLE      # 68.1 — solid outer rim (v2's
                                               # unbacked-rim width)
LID_MID_W     = 4 * NOZZLE         # 3.2 — mid tie band (v2)
LID_MID_RC    = (LID_RING_IN_R1 + LID_RIM_R0) / 2.0    # 58.925 — span middle

assert SEP_PASS_R - SEP_PASS_W / 2.0 - DRUM_OR >= -1e-9, \
    "pass tunnel undercuts the drum wall's root"
# the TRUE swept plan envelope of the tilted tube (its corner edges,
# not just its mouth) must keep the 1.6 tier to the tooth roots —
# user-caught: the mouth-only version measured 1.0 on the print model
assert (math.hypot(SEP_PASS_R + SEP_PASS_W / 2.0,
                   SEP_PASS_W * math.sqrt(2.0) / 2.0 + RIM_H / 2.0)
        <= RIM_OD / 2.0 - RATCHET_DEPTH - 2 * NOZZLE + 1e-9), \
    "pass tunnel's swept envelope reaches within 1.6 of the tooth roots"
assert DRUM_IR - SPRING_FLANGE_OD / 2.0 >= 2.0, \
    "drum bore too close to the spring flanges (install slide-past)"

# ── Containment WALL + frame BEAMS (v2's stack, v3-anchored) ─────────────────
WALL_SEP_CLR = 2 * NOZZLE          # 1.6 — wall bore off the separator/lid OD
                                   # (user's call)
WALL_IR = RIM_OD / 2.0 + WALL_SEP_CLR          # 71.03 — wall inner face
WALL_T  = 3 * NOZZLE               # 2.4 — ring thickness (user's call)
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
CH_TOP_Z = SEP_Z0 - 3 * NOZZLE     # coil ceiling under the rotating disk
# chamber depth: the worst-case breathing stack — FLOORED by the entry
# port (at 1 m capacity the stack-derived chamber is too shallow for
# the Ø13 connector's house gable; the port is hardware-sized, so the
# chamber keeps its minimum and carries spare coil headroom)
CH_BOT_Z = CH_TOP_Z - max(AXIAL_STACK + 1.0,
                          0.5 + 1.5 * SEP_PASS_W + 2 * NOZZLE)
# SPOKED FLOOR (user's call, replacing BOTH v2's separate coil cup AND
# the bottom plus: the fused wall band + floor + beams ARE the bottom
# frame now — and the stack got ~11 shorter for it)
FLOOR_T   = 3 * NOZZLE             # 2.4 — floor plate thickness (user: 1.6
                                   # flexed along z under centre pressure —
                                   # ALL the bottom webbing went to 2.4)
FLOOR_Z1  = CH_BOT_Z               # ≈ −70.9 — floor top = the chamber floor
FLOOR_Z0  = FLOOR_Z1 - FLOOR_T     # ≈ −72.5 — the part's bed plane
FLOOR_SPOKE_N = 16                 # doubled from 8 (user's call): the hub
                                   # carries the whole rotating stack now
                                   # (stub → bottom 608 → separator), so
                                   # the floor is weight-bearing, not just
                                   # anti-tangle webbing
FLOOR_SPOKE_W = 3 * NOZZLE         # 2.4 — spoke width (user: stiffen the
                                   # bottom webbing, with the plate)
FLOOR_HUB_R   = 22 * NOZZLE        # 17.6 — centre disc tying the spokes AND
                                   # landing the sleeve's flared root (was
                                   # 9.6 before the sleeve arrived)
FLOOR_RING_RC = (33.6, 56.0)       # two spoke-width tie rings under the
                                   # coil span (42 / 70 beads)
BEAM_Z0 = FLOOR_Z0                 # beams stand on the bed with the floor
BEAM_Z1 = FRAME_Z0                 # beams end at the top plus underside

# CENTRE GUIDE SLEEVE (user's call): the tight coil winds against it, so
# it can never be yanked under the cable's bend radius, and it keeps the
# breathing helix round and centred (v2's cup sleeve, reborn on the floor).
SLEEVE_OD    = 33 * NOZZLE         # 26.4 — the tight coil's ID (LOOP_D_TIGHT
                                   # − CABLE_D = 28.15 at the Ø3.85 cable)
                                   # keeps 0.5+/side off it (v2's guard;
                                   # was 28.0 — slimmed at #883 with the
                                   # cable bump)
SLEEVE_T     = 2 * NOZZLE          # 1.6 — tube wall
SLEEVE_FLARE = 4 * NOZZLE          # 3.2 — 45° root flare (strength, user's
                                   # call; widens toward the bed — support-
                                   # free, and the hub disc lands it)
SLEEVE_Z1    = CH_TOP_Z            # sleeve top — 2.4 under the rotating disk
SLEEVE_TOP_CH = NOZZLE             # 0.8 — 45° chamfer easing the top rim:
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
assert ENTRY_PORT_SILL + 1.5 * ENTRY_PORT_W <= CH_TOP_Z - 2 * NOZZLE + 1e-9, \
    "entry port's gable peak reaches the chamber ceiling"

# Diamond PERFORATION of the fused wall band (user's call — material/time
# saving): squares rotated 45° print support-free at any size, and present
# only 45° ramp edges to the circumferentially-sliding coil (the knurl
# lesson inverted). Width is sag-safe: a Ø3 cable at its R15 bend spans a
# 6.4 hole dipping w²/8R ≈ 0.34 — under the loose coil's 0.7 standoff, so
# a resting wrap can't hook a far edge. Perforated: the coil-chamber band
# + the disk band. SOLID: the spool-chamber band (the working coil loads
# that wall outward), the tenon/bay/entry-port sectors, floor/ceiling
# margins.
PERF_D   = 8 * NOZZLE              # 6.4 — diamond size across both diagonals
PERF_WEB = 3 * NOZZLE              # 2.4 — min web between holes (= wall tier)

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
BOT_CONE_CAP_Z = SEP_Z1 - 2 * NOZZLE           # −36.0 — flat cap: exactly the
                                               # quality web under the disk top
BOT_CONE_CAP_D = BRG_BORE - 2.0 * (BOT_CONE_CAP_Z - BOT_SEAT_CONE_Z0)  # 19.4
STUB_D       = AXLE_PRINT_D        # 8.0 — at the 608's nominal bore (user)
STUB_BOSS_D  = STUB_D + 4 * NOZZLE # 11.2 — inner-race seat shoulder (lands
                                   # on the face annulus, same as the top)
STUB_BOSS_Z1 = BOT_BRG_Z0          # boss shoulder top = inner race bottom
STUB_Z1      = BOT_BRG_Z1          # stub tip flush with the bearing top
STUB_FLARE   = 3 * NOZZLE          # 2.4 — 45° base flare (strength)
STUB_TIP_CH  = NOZZLE              # 0.8 — 45° entry chamfer at the tip

assert BOT_CONE_CAP_Z > BOT_SEAT_CONE_Z0 + 0.5, \
    "seat cone has no height before its cap — check the bearing z-chain"
assert SEP_Z1 - BOT_CONE_CAP_Z >= 2 * NOZZLE - 1e-9, \
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
SEP_POCKET_FLOOR_T = 3 * NOZZLE    # 2.4 — plate left at the disc bottom (user)
SEP_BOSS_D = BOT_CONE_CAP_D + 2 * 3 * NOZZLE   # 24.2 — bearing-cavity walls
SEP_BOSS_FLARE = 3 * NOZZLE        # 2.4 — 45° skirt tying the boss into the
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
# no sliding pieces, so JOINT_CLR/JOINT_BACK_CLR remain as the project's
# print-proven lateral / depth-face clearances for every other joint) ────────
JOINT_SPEC = PrintSpec(nozzle=NOZZLE, material="PETG-GF", facing="up")
JOINT_CLR, JOINT_BACK_CLR = joint_clearances(JOINT_SPEC, JOINT_SPEC)  # 0.15/0.30
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
TOP_JOINT_SEAT_CLR = 0.15          # seating clearance at each stop (arc mm)
TOP_ENTRY_OVER     = 1.0           # channel overshoot past the entry (arc mm)
FRAMEJ_W = 6 * NOZZLE              # 4.8 — joint width (stem 2.4, cadkit)
FRAMEJ_R = (math.floor(((BEAM_IR + BEAM_SIZE)
                        - (2 * NOZZLE + FRAMEJ_W / 2.0 + JOINT_CLR))
                       / NOZZLE) * NOZZLE)
                                   # profile centreline radius on the beams:
                                   # the FARTHEST bead radius whose cavity
                                   # keeps a 1.6 outer wall at the flush arm
                                   # end (was a pinned 79.2 — DERIVED since
                                   # #884 so capacity resizes flow through;
                                   # frame.py still asserts the walls)
FRAMEJ_TEN_ARC = BEAM_SIZE / 2.0   # 5.2 — engaged tenon arc per site (the
                                   # 50% rule: the joint takes half the
                                   # crossing, matching MOUNT_TEN_ARC)

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
MOUNT_PLATE_T = 5 * NOZZLE         # 4.0 — pad/spine thickness: the MINIMUM
                                   # that swallows the head cone flush
                                   # (user's rule — v2's config)
assert MOUNT_PLATE_T >= WOOD_SCREW_HEAD_H - 1e-9, \
    "mount plate too thin to countersink the wood screw flush"

MOUNT_TEN_W  = 5 * NOZZLE          # 4.0 — joint width: the largest whose
                                   # OCTAGON swallow fits above the 1.6
                                   # arm floor (asserted in mount.py); its
                                   # cavity leaves 59% of the arm intact
                                   # (user's ≥~50% rule, asserted)
MOUNT_SPINE_T = 3 * NOZZLE         # 2.4 — spine ring depth (was 4.0: the
                                   # shallower spine raises the mating
                                   # plane, buying the octagon its room;
                                   # the screw PADS keep MOUNT_PLATE_T)
MOUNT_MATE_Z = FRAME_RIB - MOUNT_SPINE_T           # 8.0 — the joint's mating
                                                   # plane (spine underside)
MOUNT_FLOOR_MIN = 2 * NOZZLE       # 1.6 — solid arm required under the
                                   # closed cavities (user's call)
MOUNT_RING_R = (math.floor((FRAMEJ_R - FRAMEJ_W / 2.0 - JOINT_CLR
                            - 2 * NOZZLE - MOUNT_TEN_W / 2.0 - JOINT_CLR)
                           / NOZZLE) * NOZZLE)
                                   # outer ring centreline: the farthest
                                   # bead radius whose cavities keep a 1.6
                                   # web to the frame arc joints (was a
                                   # pinned 71.2 — DERIVED since #884 so
                                   # capacity resizes flow through; the
                                   # keep-out stays asserted in frame.py)
MOUNT_RING2_R = (round(MOUNT_RING_R / 2.0 / NOZZLE)
                 * NOZZLE)         # INNER ring at half the diameter, on the
                                   # bead grid (derived since #884)
                                   # (user: one ring is flimsy) — its own
                                   # arc joinery in all four arms; its
                                   # cavities stop exactly one 1.6 web
                                   # inboard of the +x anchor rib (assert)
MOUNT_SPINE_W = 3 * NOZZLE         # 2.4 — ring section (user: one square
                                   # 2.4 x 2.4 for rings/spokes/tails; the
                                   # spine no longer threads the cavities'
                                   # 2.3 stem slots — the V-grooves run
                                   # FULL CIRCLE now, carrying the rings
                                   # over every crossing, cavities incl.)
MOUNT_TEN_ARC = BEAM_SIZE / 2.0    # 5.2 — engaged tenon ARC LENGTH per site
                                   # (user: trim the joint from the STOP
                                   # side — the tenon anchors at the arm's
                                   # ENTRY face and spans half the crossing,
                                   # leaving the other half solid arm;
                                   # mirrors the ~50% width rule)
MOUNT_SPOKE_W  = 3 * NOZZLE        # 2.4 — ring-tie spokes (user: ONE
                                   # consistent 2.4 × 2.4 square section
                                   # for rings, spokes and the screw-pad
                                   # guide tails; spokes ride the rings'
                                   # own z-band now, not the full plate)
MOUNT_SPOKE_AZ = (45.0, 135.0, 225.0, 315.0)   # mid-quadrant: the spokes
                                   # stay over open air at EVERY install
                                   # angle (rotation keeps the rings on
                                   # their own circles; only the spokes
                                   # and pads move over the plan, and the
                                   # quadrants are 90° wide vs ~17° travel)
MOUNT_PAD_W   = WOOD_SCREW_HEAD_D + 2 * NOZZLE     # 10.9 — square screw pads
# screw pads HUG THE AXES (user: the screws must land on a wooden BEAM
# running along x OR y — use whichever pad pair matches the beam; the
# beam just has to be wide enough for both screws of the pair, ~±10.7
# off-axis here). The pads sit at ZERO gap: seated, each pad's inner
# edge lands FLAT ON its arm's flank — the pads ARE the seating stop
# (user's call). The arc joints keep TOP_JOINT_SEAT_CLR (0.15) at
# their stop ends, so the joinery never bottoms out first. Pads sit
# CW (−) of the arms now — the hand="ccw" chirality flip: the mount's
# tenons seat rotating CCW relative to the frame, so the pads trail on
# the CW side and land on the arms' CW flanks; pull-out torque (the
# assembly CW ≡ the mount CCW relative to it) presses them tighter.
MOUNT_PAD_AZ_OFF = math.degrees(math.asin(
    (BEAM_SIZE / 2.0 + MOUNT_PAD_W / 2.0) / MOUNT_RING_R))   # ≈ 8.6°
MOUNT_PAD_AZ = tuple(a - MOUNT_PAD_AZ_OFF for a in (0.0, 90.0, 180.0, 270.0))
assert TOP_JOINT_SEAT_CLR > 0.0, \
    "the pads-stop-first scheme needs a positive joint seat clearance"
assert (BEAM_SIZE - (MOUNT_TEN_W + 2 * JOINT_CLR)) / 2.0 >= 2 * NOZZLE - 1e-9, \
    "mount cavity leaves the arm's side walls under the 1.6 tier"
assert (BEAM_SIZE - (MOUNT_TEN_W + 2 * JOINT_CLR)) / BEAM_SIZE >= 0.49, \
    "mount cavities eat more than ~half the arm's width (user's bound)"
assert (MOUNT_RING_R + MOUNT_PAD_W * math.sqrt(2.0) / 2.0
        <= FRAME_R_OUT - 1e-9), \
    "mount screw pads poke past the frame's plan silhouette"
assert (ANCH_IR - (MOUNT_RING2_R + MOUNT_TEN_W / 2.0 + JOINT_CLR)
        >= 2 * NOZZLE - 1e-9), \
    "inner mount ring's cavities reach the +x anchor rib"
assert abs(MOUNT_RING_R * math.sin(math.radians(MOUNT_PAD_AZ_OFF))
           - MOUNT_PAD_W / 2.0 - BEAM_SIZE / 2.0) < 1e-6, \
    "seated pads must land exactly on the arm flanks (they are the stop)"
# annular V-GROOVE for the spine over each arm's stop-wall zone (the
# groove's floor is the roof of the slot in the frame's +z→−z print —
# vertical walls, then a 45° V closing on a ONE-BEAD flat: 0.8 is the
# bridging limit, user's call — the earlier +2·clearance flat printed a
# 1.1 bridge; the V runs 0.15 deeper instead):
MOUNT_GRV_W    = MOUNT_SPINE_W + 2 * JOINT_CLR     # 2.3 — groove width
MOUNT_GRV_VERT = MOUNT_SPINE_T + JOINT_CLR         # 2.55 — vertical depth
MOUNT_GRV_FLAT = NOZZLE                            # 0.8 — dulled V flat
MOUNT_GRV_DEPTH = MOUNT_GRV_VERT + (MOUNT_GRV_W - MOUNT_GRV_FLAT) / 2.0  # 3.3

# ── Levers (v2's design "worked well" — constants re-anchored to the v3
# bands; the lever PARTS + kinematics suite land next round, these drive
# the frame's mount + the wall windows now) ──────────────────────────────────
LEVER_T        = 10.0              # plate thickness (Y) — ergonomic (v2)
LEVER_HANDLE_W = 6.0               # handle X width (v2)
PIN_SQ_S       = 4.4               # square TPU torsion-axle side (v2 print-
                                   # proven; largest side passing the frame
                                   # packaging asserts below)
PIN_SQ_FRAME_CLR = 0.1             # per-side, column keyway + beam blind bore
PIN_SQ_LEVER_CLR = 0.15            # per-side, the lever's through-pocket
PIN_KEY_BASE_DEG = 45.0            # frame keyway clock: DIAMOND tip-up in the
                                   # sideways print — self-supporting bores
PIN_TIP_END_Y  = 4.4               # blind-bore floor |y| (the axle depth stop)
PIN_GRIP_L     = 2.0               # axle proud of the column face (removal grip)
PIN_PRETWIST_DEG = 12.0            # install pre-twist = seating preload (v2)
# BRAKE REST STOP (user): a frame-side corbel the PAD — never the bare
# lever — rests on under the pin's pretwist. Install story: the bare
# lever must swing to +PIN_PRETWIST to take the axle (pocket↔frame-key
# alignment), so the frame may not block the lever anywhere; the PAD,
# slid on afterwards with the lever held plumb, is what the stop
# catches when the axle twists the lever back. The corbel's 45°
# underside (rising toward −y off the bay window's edge — the printable
# corbel face) IS the catch face: the pad's preload arc has no
# y-component, so it lands square on it. Height anchor derived in
# levers.py (BRAKE_STOP_ZC); solid in frame.py.
BRAKE_STOP_GAP      = -0.25        # vertical INTERFERENCE at the pad's top
                                   # inner corner at plumb (user: the 0.15
                                   # gap version never made contact on the
                                   # print — TPU corner rounding + slop ate
                                   # it; now the soft corner presses 0.25
                                   # into the 45° face at plumb, a designed
                                   # squish — the gate whitelists the pair)
# (BRAKE_STOP_TIP_OVER retired at #886 — the corbel's reach past the
# contact plane is TOP_T − NOZZLE now: the 45° underside ends on a
# vertical one-bead face instead of a knife point, user's call)
# CABLE-GUARD chevrons (user #887): a ^ of GUARD_T square section
# fences each lever bay window at the wall line — the full-height
# windows that let fingers in would also let a slack coil escape into
# the levers; the open V underneath keeps the finger room. The peak
# clears the BRAKE side's swept envelope by GUARD_LEVER_CLR and BOTH
# bays share the height (user); peak z derives in levers.py.
GUARD_T         = 2 * NOZZLE       # 1.6 — chevron bar section (user)
GUARD_LEVER_CLR = 3 * NOZZLE       # 2.4 — peak clearance under the brake
                                   # side's swept envelope (user)

# ── CABLE HORN (user #889) — the grab station off the bottom frame at
# the exit window. Step 1: a SOLID radial pier at the window's
# mid-azimuth, welded into the band below the sill, running out to the
# fork blocks' radial extent, top FLUSH with the window bottom (the
# exiting cable rides straight onto it). Step 2 REDESIGNED at #903
# (user: the IDEAL cable model first, printability/installation set
# aside — the open-top guide walls + roof-cap plan are DROPPED): a
# second prism sits ON the pier near its +x end, shaped by cuts —
# a round through-TUNNEL for the cable and a FULL-CIRCLE bell at EACH
# mouth (kept identical for simplicity, user: the +x exit needs the
# full circle — the free cable pulls in any direction with no edge to
# bite — and the −x entry just reuses it, though the spool-plane cable
# never technically needs the top/bottom quadrants there).
HORN_CH_W  = (math.ceil(1.5 * CABLE_D / NOZZLE - 1e-9)
              * NOZZLE)            # 6.4 — tunnel bore Ø: 1.5 × cable Ø,
                                   # rounded UP to the bead grid (user)
HORN_WALL_T = 3 * NOZZLE           # 2.4 — meat beside the tunnel
HORN_W      = HORN_CH_W + 2 * HORN_WALL_T      # 11.2 — pier/block width
HORN_ROOF_T = 2.8                  # meat above the tunnel; block height
                                   # = HORN_CH_W + HORN_ROOF_T. A z-size
                                   # (layer-quantized, not a bead):
                                   # sized so the cap keeps ≥ 1.6 over
                                   # the rail cavities (user #908 — the
                                   # mushroom swallow is 4.36, and 2.4
                                   # here left only 1.24)
HORN_BLOCK_L = 16 * NOZZLE         # 12.8 — tunnel block length along x,
                                   # flush with the +x frame edge: two
                                   # bells + straight middle
HORN_BELL_R = 2 * NOZZLE           # 1.6 — mouth bell, BOTH ends: a
                                   # quarter-round trumpet revolved
                                   # around the tunnel axis (tangent to
                                   # the bore, tangent to the face);
                                   # leaves NOZZLE dull flats at the
                                   # block's side faces
# cap ATTACHMENT (user #906): cadkit MUSHROOM rails (hosts print in
# opposite z directions), install sliding − to + y. TENONS rise off the
# trough strips' tops (2.4 engagement each, rooted in the pier meat);
# the CAP carries one through-CAVITY per rail across its full width —
# open at the +y face (entry), 0.3 into the stop flange at the far end
# so the FLANGE, not the cavity wall, defines seating. The flange: extra
# material on the CAP ONLY (user: don't grow the bottom prism), at its
# trailing −y end, hanging past the equator to catch the trough's −y
# face and stop +y over-travel. Backing out −y has no shape retention
# yet (cadkit doctrine: stop one way, preload the other).
HORNJ_W      = 4 * NOZZLE          # 3.2 — joint width room (across x);
                                   # mushroom height 4.06, cap swallow
                                   # 4.36 — roof left 1.24
HORNJ_X_OFF  = 2.8                 # TWO rails at ± this off the block's
                                   # x middle (pitch-locks the cap). A
                                   # POSITION, not a bead — the one slot
                                   # between the quality-tier wall floor
                                   # (cavity gap 2.1 ≥ 1.6) and the
                                   # bell-free window (edges 0.25 clear)
HORN_STOP_T    = 3 * NOZZLE        # 2.4 — stop flange thickness (y)
HORN_STOP_DROP = 3 * NOZZLE        # 2.4 — flange catch depth below the
                                   # equator plane (contact patch on the
                                   # trough's −y face)
# (HORN_YC — the horn's centreline — is defined AFTER the exit window:
# the WINDOW derives purely from the cable/coil geometry, and the horn
# then sits at its midway point — user's design order, #899)
# (HORN_Z1 = the window bottom — set beside CABLE_EXIT_Z0 below)
BRAKE_STOP_TOP_T    = 3 * NOZZLE   # 2.4 — meat above the contact plane
POST_OUT_T     = BEAM_SIZE / 2.0   # 5.2 — side-column / bent-arm section
BOSS_RING_W    = contact_rib_size(NOZZLE)      # 1.6 — thrust ring width & proud
BOSS_BORE_ID   = PIN_SQ_S * math.sqrt(2.0) + 0.4   # 6.62 — clears the square's
                                                   # rotating envelope
LEVER_SIDE_CLR = BOSS_RING_W + 0.2 # 1.8 — air each side of the lever plates
LEVER_Y_IN     = BEAM_SIZE / 2.0 + LEVER_SIDE_CLR  # 7.0 — inner plate face
RATCHET_LEV_Y0 = LEVER_Y_IN                        # ratchet plate on +Y side
RATCHET_LEV_Y1 = LEVER_Y_IN + LEVER_T              # 17.0
BRAKE_LEV_Y1   = -LEVER_Y_IN                       # brake plate on −Y side
BRAKE_LEV_Y0   = -LEVER_Y_IN - LEVER_T             # −17.0
LEVER_PIVOT_X  = (BEAM_IR + BOSS_BORE_ID / 2.0 + BOSS_RING_W
                  + NOZZLE / 2.0)                  # 78.74 — v2's derivation
                                   # (rings flush at the beam's inner edge)
                                   # + HALF A BEAD outboard: v3's larger
                                   # brake contact angle leans the pad's
                                   # rest-plumb joint plane closer to the
                                   # handle, and the offset restores the
                                   # 0.45 guard (A_PAD_HANDLE_CLR); the
                                   # ring/diamond packaging asserts below
                                   # still pass
LEVER_BOSS_OD  = ((PIN_SQ_S + 2.0 * PIN_SQ_LEVER_CLR) * math.sqrt(2.0)
                  + 2.0 * STRUCT_WALL)             # 9.85 — lever pivot boss
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
RATCHET_OPEN_CLR = 3 * NOZZLE      # 2.4 — the OVER-PULL STOP pose (user):
                                   # the pull angle where the pawl clears
                                   # the tooth tips by this much is where
                                   # the frame's shelf catches the pawl —
                                   # no more levering the axle by yanking
                                   # the ratchet (RATCHET_OPEN_DEG derives
                                   # in levers.py; shelf in frame.py)
HANDLE_Z_BOT     = FLOOR_Z0 + LEVER_HANDLE_W / 2.0   # ≈ −69.5 — grip-disc
                                   # centre placed so the handles' round
                                   # ends land FLUSH with the assembly's
                                   # −z extent (user's call); pulling only
                                   # raises them
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
PAD_SPEC      = PrintSpec(nozzle=NOZZLE, facing="up")
PAD_FLANGE_H  = joint_box_min(PAD_SPEC, PAD_SPEC, install="-z", bounded=True,
                              quality=True, clearance=PAD_JOINT_CLR)[0]
                                   # 6.6 — the edge-bounded site's quality
                                   # width = the pad flange / arm end face
PAD_FLANGE_T  = 2 * NOZZLE         # flange depth (the arm face recesses by it)
# TPU torsion-axle length: blind-bore floor → column outer face → grip stub
LEVER_PIN_L = (RATCHET_LEV_Y1 + LEVER_SIDE_CLR + POST_OUT_T
               + PIN_GRIP_L - PIN_TIP_END_Y)       # 21.6

# ── Wall openings (v2's derivations, v3 anchors) ─────────────────────────────
_WALL_TOP_BAND_RETIRED = 4 * NOZZLE   # (kept only for the comment trail —
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
    "thrust rings poke past the frame's +x face — shrink PIN_SQ_S"
assert ((BEAM_IR + BEAM_SIZE)
        - (LEVER_PIVOT_X
           + (PIN_SQ_S + 2.0 * PIN_SQ_FRAME_CLR) * math.sqrt(2.0) / 2.0)
        >= 2 * NOZZLE - 1e-9), \
    "under 1.6 of beam between the diamond bore and the frame's +x face"

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
