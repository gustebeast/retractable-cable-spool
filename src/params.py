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
SPRING_BORE_TOP_D = 8.7      # top-flange arbor hole (user-measured 2026-08-03) —
                             # only the bare shaft passes; the joint collar CAN'T
SPRING_BORE_BOT_D = 13.4     # bottom-flange hole — the mortise collar enters here
SPRING_STRIP_T   = 0.2       # strip steel thickness (v2-measured)
SPRING_STRIP_W   = 22.0      # strip width (v2-measured)

# ── Cable (hardware) ─────────────────────────────────────────────────────────
CABLE_D        = 3.0         # working-cable Ø (user's measurement)
CABLE_CAPACITY = 6.0 * 304.8 # 1828.8 — 6 ft of wrapped cable (user's call)

# ── Axle (rotating arbor) ────────────────────────────────────────────────────
AXLE_D       = 8.0                   # nominal — the 608's bore
AXLE_PRINT_D = AXLE_D - 0.05         # 7.95 — 0.025/side under the bore (v2-proven)
AXLE_LIP_W   = 2 * NOZZLE            # retention lip radial protrusion
AXLE_LIP_OD  = AXLE_PRINT_D + 2 * AXLE_LIP_W   # 11.15 — rests on the inner race
AXLE_LIP_H   = 2 * NOZZLE            # lip axial height (tops out under the
                                     # frame face — flat assembly top)

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
# the joint must pass through the spring's Ø13.4 flange holes, so the axle
# splits and each half slides onto the strip from its own end) ───────────────
MORTISE_TOP_GAP      = 5 * NOZZLE                  # 4.0 — collar top below the spring top
JOINT_H              = 23.0                        # bore depth = engagement = slit
                                                   # length (v1 print-proven)
JOINT_Z1             = SPRING_Z1 - MORTISE_TOP_GAP # −4 — mortise opening
JOINT_Z0             = JOINT_Z1 - JOINT_H          # −27 — bore bottom = tenon tip
JOINT_MORTISE_HOLE_D = AXLE_PRINT_D + 2 * FIT_CLR  # 8.25 — female bore
JOINT_MORTISE_OD     = JOINT_MORTISE_HOLE_D + 2 * STRUCT_WALL  # 11.45 — collar,
                                                   # AND the bottom half's one
                                                   # constant section end to end
                                                   # (user's call — no narrowing
                                                   # below the collar)
SLIT_W               = 2 * NOZZLE                  # spring-strip slit width
SLIT_END_CAP         = 7 * NOZZLE                  # 5.6 — solid band between each
                                                   # half's slit and its joint-side
                                                   # face (user's call): the slits
                                                   # are closed WINDOWS now, not
                                                   # open-ended slots — the strip's
                                                   # end tongue threads the window

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
AXLE_ROOT_CONE_H = 3 * NOZZLE      # 2.4 — 45° column→disk root cone height =
                                   # the FULL gap (user's call): the cone top
                                   # lands exactly on the spring's face plane,
                                   # where it is still only the column Ø —
                                   # inside the Ø13.4 flange hole, no contact
SEP_Z1 = SPRING_Z0 - SEP_SPRING_CLR    # −34.4 — disk top, right under the
                                       # (frame-fixed, static) spring
SEP_Z0 = SEP_Z1 - RIM_H                # −44.4 — disk bottom = the bed face

AXLE_Z_BOT = SEP_Z0                    # the column runs flush through the disk
AXLE_Z_TOP = BRG_Z1 + AXLE_LIP_H       # 10.2 — lip top, UNDER the frame face

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
# The ENTRY (the tall half) is centred on the strip line (user's call —
# the head threads at its natural z). The BAY therefore sits HIGH of
# the strip line, sharing the flat top edge: at rest the neck rides UP
# into it within the spring's own z-float (asserted below). Retention
# is by SIZE (16 vs 12), wherever the bites land.
_SPRING_ZC  = (SPRING_Z0 + SPRING_Z1) / 2.0          # −16 — strip centreline
ANCH_ZC     = _SPRING_ZC           # ENTRY wall centred on the strip line
ANCH_WIN_HOLD = 15 * NOZZLE        # 12.0 — bay height (head-blocking)
ANCH_WIN_PASS = 23 * NOZZLE        # 18.4 — entry-wall height (head-passing:
                                   # 1.2 around the head at its natural z)
ANCH_WIN_Z1  = ANCH_ZC + ANCH_WIN_PASS / 2.0         # −6.8 — flat top edge
ANCH_WIN_Z0  = ANCH_ZC - ANCH_WIN_PASS / 2.0         # −25.2 — the VIRTUAL
                                   # apex at the +y entry wall: the 45°
                                   # floor aims here, but the tip is
                                   # DULLED (user's call) —
ANCH_TIP_Z   = ANCH_WIN_Z0 + NOZZLE                  # −24.4 — a 0.8 flat
                                   # truncates the corner: one bead
                                   # bridges it in print
ANCH_HOLD_Z0 = ANCH_WIN_Z1 - ANCH_WIN_HOLD           # −18.8 — bay bottom (−y)
ANCH_WIN_W   = ANCH_WIN_PASS - ANCH_WIN_HOLD         # 6.4 — width: 45° floor
                                                     # bay-corner → virtual apex
ANCH_Z0      = ANCH_TIP_Z - 4 * NOZZLE               # −27.6 — wall bottom
                                   # (3.2 sill under the dulled tip)
# the strip's feasible z-band inside the spring (22 body between the
# flange inner faces) must reach far enough UP to seat the neck in the
# bay, and the head must thread the entry somewhere in the same band:
_STRIP_C_LO = SPRING_Z0 + SPRING_FLANGE_T + STRIP_BODY_W / 2.0   # −18.6
_STRIP_C_HI = SPRING_Z1 - SPRING_FLANGE_T - STRIP_BODY_W / 2.0   # −13.4
assert (min(_STRIP_C_HI, ANCH_WIN_Z1 - STRIP_NECK_W / 2.0)
        - max(_STRIP_C_LO, ANCH_HOLD_Z0 + STRIP_NECK_W / 2.0)
        >= 0.5 - 1e-9), \
    "no feasible strip z seats the neck in the holding bay"
assert (min(_STRIP_C_HI, ANCH_WIN_Z1 - STRIP_HEAD_W / 2.0 - 0.2)
        - max(_STRIP_C_LO, ANCH_TIP_Z + STRIP_HEAD_W / 2.0 + 0.2)
        >= 0.5 - 1e-9), \
    "no feasible strip z threads the head through the entry"
# SWEEP: centred on the +X arm — window + a 6-bead pier each side (the
# wall chord still overhangs the 10.4 arm a touch each side; in the
# +z→−z print the outboard blades root on the BED like the old gate
# wall's open span did).
ANCH_HALF_A = math.degrees(math.asin(
    (ANCH_WIN_W / 2.0 + 6 * NOZZLE) / ANCH_IR))      # ≈ 11.5°

# The pass/block mechanism IS these inequalities:
assert ANCH_WIN_Z1 - ANCH_TIP_Z >= STRIP_HEAD_W + 0.4 - 1e-9, \
    "head doesn't pass flat-on at the entry wall (tip flat included)"
assert (NOZZLE + (ANCH_WIN_Z1 - ANCH_TIP_Z) - (STRIP_HEAD_W + 0.4)
        >= 2 * NOZZLE - 1e-9), \
    "entry zone too narrow to thread the head by hand"
assert ANCH_WIN_HOLD <= STRIP_HEAD_W - 3.0 + 1e-9, \
    "head escapes the holding bay"
assert ANCH_WIN_HOLD >= STRIP_NECK_W + 0.4 - 1e-9, \
    "neck binds in the holding bay"
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
R_OUT_COIL = WRAP_R0 + N_WRAPS * COIL_PITCH    # ≈ 66.9 — outermost wrap centre
RIM_OD = 2.0 * (R_OUT_COIL + CABLE_D / 2.0 + 1.0)   # ≈ 138.9 — separator/lid
                                               # OD: coil + 1.0 cover margin
                                               # (replaces v2's 145.8 carry)

LID_SEP_GAP = 4.0                  # FIXED separator↔lid gap (user's call) —
                                   # the spool chamber height: one flat
                                   # layer of the Ø3.0 cable + 1.0 play
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
assert (ANCH_TIP_Z + 0.2) - LID_Z1 >= 0.5 - 1e-9, \
    "threading the strip head would scrape the rotating lid"
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
LIDJ_TEN_SWEEP  = 20.0             # tenon arc sweep (deg)
LIDJ_SEAT_CLR   = 0.15             # seating clearance at each stop (arc mm)
LIDJ_ENTRY_OVER = 1.0              # pocket overshoot past the tenon, each
                                   # side (arc mm) — drop-in ease

# 45° pass tunnel, RE-DERIVED for the drum (the separator block's comment
# promised this): mouth just outside the wall, one bead of web to its root
SEP_PASS_R = DRUM_OR + NOZZLE + SEP_PASS_W / 2.0    # 57.05

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

assert SEP_PASS_R - SEP_PASS_W / 2.0 - DRUM_OR >= NOZZLE - 1e-9, \
    "pass tunnel undercuts the drum wall's root"
assert SEP_PASS_R + SEP_PASS_W / 2.0 <= RIM_OD / 2.0 - RATCHET_DEPTH - 2 * NOZZLE + 1e-9, \
    "pass tunnel reaches the ratchet tooth band"
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
CH_TOP_Z = SEP_Z0 - 3 * NOZZLE     # −46.4 — coil ceiling under the rotating disk
CH_BOT_Z = CH_TOP_Z - AXIAL_STACK - 1.0        # ≈ −70.9 — chamber floor plane
# SPOKED FLOOR (user's call, replacing BOTH v2's separate coil cup AND
# the bottom plus: the fused wall band + floor + beams ARE the bottom
# frame now — and the stack got ~11 shorter for it)
FLOOR_T   = 2 * NOZZLE             # 1.6 — floor plate thickness (user)
FLOOR_Z1  = CH_BOT_Z               # ≈ −70.9 — floor top = the chamber floor
FLOOR_Z0  = FLOOR_Z1 - FLOOR_T     # ≈ −72.5 — the part's bed plane
FLOOR_SPOKE_N = 16                 # doubled from 8 (user's call): the hub
                                   # carries the whole rotating stack now
                                   # (stub → bottom 608 → separator), so
                                   # the floor is weight-bearing, not just
                                   # anti-tangle webbing
FLOOR_SPOKE_W = 2 * NOZZLE         # 1.6 — spoke width (user)
FLOOR_HUB_R   = 22 * NOZZLE        # 17.6 — centre disc tying the spokes AND
                                   # landing the sleeve's flared root (was
                                   # 9.6 before the sleeve arrived)
FLOOR_RING_RC = (33.6, 56.0)       # two 1.6-wide tie rings under the coil
                                   # span (42 / 70 beads)
BEAM_Z0 = FLOOR_Z0                 # beams stand on the bed with the floor
BEAM_Z1 = FRAME_Z0                 # beams end at the top plus underside

# CENTRE GUIDE SLEEVE (user's call): the tight coil winds against it, so
# it can never be yanked under the cable's bend radius, and it keeps the
# breathing helix round and centred (v2's cup sleeve, reborn on the floor).
SLEEVE_OD    = 35 * NOZZLE         # 28.0 — the tight coil's ID (LOOP_D_TIGHT
                                   # − CABLE_D = 29) keeps 0.5/side off it
                                   # (v2's guard clearance)
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
STUB_D       = AXLE_PRINT_D        # 7.95 — the proven 608 slip fit
STUB_BOSS_D  = AXLE_LIP_OD         # 11.15 — inner-race seat shoulder (lands
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
assert LOOP_D_TIGHT / 2.0 >= CABLE_BEND_R_MIN - 1e-9, \
    "tight coil's centreline under the cable's bend radius"
assert 2.0 * WALL_IR - LOOP_D_LOOSE - CABLE_D >= 1.2 - 1e-9, \
    "loose coil pinches the wall bore"

# ── Wall ↔ beam T joints (cadkit slide_joint, install="-z" — v2's numbers,
# all print-proven at the 0.8 nozzle in PETG-GF) ─────────────────────────────
JOINT_SPEC = PrintSpec(nozzle=NOZZLE, material="PETG-GF", facing="up")
JOINT_CLR, JOINT_BACK_CLR = joint_clearances(JOINT_SPEC, JOINT_SPEC)  # 0.15/0.30
JOINT_WIDTH = 6.5 * NOZZLE         # 5.2 — the smallest box whose tenon keeps
                                   # EVERY wall segment ≥ 1.6 (v2 print find)
JOINT_DEPTH = joint_box_min(JOINT_SPEC, JOINT_SPEC, install="-z",
                            quality=True)[1]     # 3.5
JOINT_SEAT_CLR = 0.15              # z seating clearance at the channel stops
WALL_ZB       = FLOOR_Z0           # wall band bottom — FUSED into frame_bottom
                                   # now (user's call; only wall_top + collar
                                   # still slide)
# (v2's wall_collar is GONE — user's call: above wall_top, four identical
# flat LOCK STRIPS (wall.py) fill the beam channels up to frame_top's
# underside and z-lock the stack. MORTISE_L is derived below — the channel
# stop sits at the fused band's top at WALL_SPLIT_Z.)

# frame_top ↔ frame_bottom ROTATIONAL arc joints (v2's half-arrowhead, the
# same TOPJ_* profile the lid bayonet uses — angular constants from v2)
TOP_JOINT_SEAT_CLR = 0.15          # seating clearance at each stop (arc mm)
TOP_ENTRY_OVER     = 1.0           # channel overshoot past the entry (arc mm)
TOP_STOP_WALL      = 2 * NOZZLE    # stop-wall thickness at each arm face (arc)

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
# Joint: the flat-top mushroom ARC (cadkit, THROUGH mortise — both
# hosts print +z→−z, and the cavities exit the arms' undersides over
# the open bays), hanging from a flush SPINE ring exactly one stem wide
# (a wider slot floor would bridge in the frame's print). Over each
# arm's stop-wall zone the spine rides an annular V-bottom groove.
WOOD_SCREW_SHAFT_D = 4.0           # v2's validated screw geometry, verbatim
WOOD_SCREW_HEAD_D  = 9.3           # countersunk-head pocket Ø at the face
WOOD_SCREW_HEAD_H  = 4.0           # cone depth
MOUNT_PLATE_T = 5 * NOZZLE         # 4.0 — pad/spine thickness: the MINIMUM
                                   # that swallows the head cone flush
                                   # (user's rule — v2's config)
assert MOUNT_PLATE_T >= WOOD_SCREW_HEAD_H - 1e-9, \
    "mount plate too thin to countersink the wood screw flush"

MOUNT_TEN_W  = 8 * NOZZLE          # 6.4 — joint width (v2's mount size)
MOUNT_MATE_Z = FRAME_RIB - MOUNT_PLATE_T           # 6.4 — the joint's mating
                                                   # plane (spine underside)
MOUNT_THRU_D = MOUNT_MATE_Z + 1.0                  # 7.4 — cavity run past the
                                                   # mating plane: out the arm's
                                                   # underside → THROUGH
MOUNT_RING_R  = 89 * NOZZLE        # 71.2 — ring centreline radius (near the
                                   # arm ends, user's call; radial keep-out
                                   # to the arc joints asserted in frame.py)
MOUNT_SPINE_W = MOUNT_TEN_W / 2.0  # 3.2 — the spine IS the mushroom stem:
                                   # it rides the cavities' stem slots with
                                   # zero ledge
MOUNT_PAD_W   = WOOD_SCREW_HEAD_D + 2 * NOZZLE     # 10.9 — square screw pads
MOUNT_PAD_AZ  = (135.0, 315.0)     # pad azimuths: open quadrants, away from
                                   # the +x anchor sector and the exit at 45°
assert (BEAM_SIZE - (MOUNT_TEN_W + 2 * JOINT_CLR)) / 2.0 >= 2 * NOZZLE - 1e-9, \
    "mount cavity leaves the arm's side walls under the 1.6 tier"
assert (MOUNT_RING_R + MOUNT_PAD_W * math.sqrt(2.0) / 2.0
        <= FRAME_R_OUT - 1e-9), \
    "mount screw pads poke past the frame's plan silhouette"
# annular V-GROOVE for the spine over each arm's stop-wall zone (the
# groove's floor is the roof of the slot in the frame's +z→−z print —
# vertical walls, then a 45° V closing on a one-bead(+clearance) flat):
MOUNT_GRV_W    = MOUNT_SPINE_W + 2 * JOINT_CLR     # 3.5 — groove width
MOUNT_GRV_VERT = MOUNT_PLATE_T + JOINT_CLR         # 4.15 — vertical depth
                                                   # (the spine runs full-T)
MOUNT_GRV_FLAT = NOZZLE + 2 * JOINT_CLR            # 1.1 — dulled V flat
MOUNT_GRV_DEPTH = MOUNT_GRV_VERT + (MOUNT_GRV_W - MOUNT_GRV_FLAT) / 2.0  # 5.35

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
WALL_TOP_BAND = 4 * NOZZLE         # 3.2 — solid ring above the HIGHEST
                                   # opening = wall_top's +z extent (user's
                                   # call; was 1.6)
# cable EXIT port at (+x,+y): the cable leaves tangent to its current
# wrap, so the port spans tangency angles from the bare drum wall out to
# design capacity (v2's cable_retainer math), centered on 45°.
CABLE_EXIT_ANGLE_DEG  = 45.0
CABLE_EXIT_R_LO       = DRUM_OR                    # innermost tangency (empty)
CABLE_EXIT_R_HI       = R_OUT_COIL                 # outermost (full)
CABLE_EXIT_MARGIN_DEG = math.degrees((CABLE_D / 2.0 + 1.0) / WALL_IR)   # ≈ 2.0
CABLE_EXIT_SPAN_DEG   = (math.degrees(math.asin(CABLE_EXIT_R_HI / WALL_IR))
                         - math.degrees(math.asin(CABLE_EXIT_R_LO / WALL_IR))
                         + 2.0 * CABLE_EXIT_MARGIN_DEG)                 # ≈ 30
CABLE_EXIT_Z0 = SEP_Z1 - 0.5                       # −34.9 — port floor
CABLE_EXIT_Z1 = LID_Z0 + 0.5                       # −29.9 — port ceiling
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
# Both lever bays run through the ENTIRE wall stack now (user #834 — the
# top band over the bays deleted; the +x sector of wall_top goes with it,
# else the beam-strip between the bays is an island). The wall's highest
# remaining opening is therefore the EXIT PORT, and the highest-hole+3.2
# rule shrinks wall_top accordingly.
# wall verticals
WALL_Z1 = CABLE_EXIT_Z1 + WALL_TOP_BAND            # −26.7 — wall_top's flat
                                                   # top: highest opening + 3.2
MORTISE_L = (BEAM_Z1 + 1.0) - (WALL_SPLIT_Z - JOINT_SEAT_CLR)   # channel: stop
                                                   # just under the fused
                                                   # band's top → past beam top
assert WALL_Z1 - CABLE_EXIT_Z1 >= WALL_TOP_BAND - 1e-9, \
    "cable exit port breaks the wall's top band"
assert PIN_TIP_END_Y - (JOINT_WIDTH / 2.0 + JOINT_CLR) >= 2 * NOZZLE - 1e-9, \
    "wall-joint cavity too wide: under 1.6 of beam left beside the pin bores"
assert (LEVER_PIVOT_X + BOSS_BORE_ID / 2.0 + BOSS_RING_W
        <= BEAM_IR + BEAM_SIZE + 1e-9), \
    "thrust rings poke past the frame's +x face — shrink PIN_SQ_S"
assert ((BEAM_IR + BEAM_SIZE)
        - (LEVER_PIVOT_X
           + (PIN_SQ_S + 2.0 * PIN_SQ_FRAME_CLR) * math.sqrt(2.0) / 2.0)
        >= 2 * NOZZLE - 1e-9), \
    "under 1.6 of beam between the diamond bore and the frame's +x face"

# ── Consistency guards (fire at import) ──────────────────────────────────────
assert AXLE_PRINT_D < BEARING_ID, \
    "printed axle must be undersized vs the metal bearing bore"
assert BEARING_ID + 1.0 < AXLE_LIP_OD <= BEARING_IRACE_OD, \
    "axle lip must land fully on the inner race's face annulus"
assert BRG_LIP_ID < BEARING_OD - 1.0, \
    "frame lip must lap the outer race by a real margin"
assert BRG_LIP_H >= STRUCT_WALL, \
    "bearing lip thinner than the quality tier"
assert AXLE_Z_TOP <= FRAME_Z1, \
    "axle lip pokes above the frame top — the assembly top must stay flat"
assert (SPRING_BORE_BOT_D - JOINT_MORTISE_OD) / 2.0 >= 0.5, \
    "mortise collar under 0.5/side rotating clearance in the bottom flange hole"
assert (SPRING_BORE_TOP_D - AXLE_PRINT_D) / 2.0 >= 0.3, \
    "shaft under 0.3/side rotating clearance in the top flange hole"
assert JOINT_Z1 <= SPRING_Z1 - SPRING_FLANGE_T, \
    "mortise opening pokes above the spring's top flange interior"
assert JOINT_Z0 >= SPRING_Z0 + SPRING_FLANGE_T - 1e-9, \
    "joint bore reaches below the spring's bottom flange interior"
assert AXLE_ROOT_CONE_H <= SEP_SPRING_CLR, \
    "axle root cone pokes past the spring's bottom face"
# the slit must cover the strip (assume the strip is centred between the
# flange interiors — measure and pin down when the spring body mount lands)
_STRIP_ZC = (SPRING_Z0 + SPRING_Z1) / 2.0
assert (JOINT_Z0 <= _STRIP_ZC - SPRING_STRIP_W / 2.0 + 1e-9
        and _STRIP_ZC + SPRING_STRIP_W / 2.0 <= JOINT_Z1 + 1.0), \
    "spring-strip span not covered by the joint slit"
