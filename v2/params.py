"""(from-scratch modular redesign) — shared parameters.

ARCHITECTURE (the reason for the rewrite): the working center assembly — the
spring chamber + bearing caps — is a proven, rigid backbone. Everything else
becomes a stack of parts that SLIDE onto that backbone, so each can be placed at
any Z rather than being forced to the print bed for support:

    spool ceiling   (rides the spring chamber)
    spool wall      (rides the housing)
    MIDDLE piece    (spring chamber) — spool floor + tray ceiling + the brake &
                    ratchet lever-contact rims
    tray wall       (rides the housing)
    tray floor      (rides the spring chamber)

This first cut builds only: the spring chamber (copied from src), the middle
brake/ratchet RIM that seats at mid-height, and a minimal 4-beam housing whose
connecting ring is both the spool and tray wall. The rest follows.

Center (spring-chamber) dims are REUSED from the working design (src.dimensions);
only the new outer parts get fresh numbers here.
"""

import math

from v2.dimensions import (HUB_OD, SPOOL_H, STRUCT_WALL, FIT_CLR,
                            AXLE_PRINT_D, NOZZLE, KEY_DEPTH)
from cadkit.contact import contact_rib_size
from cadkit.joinery import PrintSpec, joint_clearances, joint_box_min

# NOZZLE lives in src.dimensions now (re-exported here) — bead-multiple
# sizing policy: every printed width/thickness is k * NOZZLE; hardware,
# cable and fit-clearance numbers stay absolute (user's exemption)

# ── Cable ────────────────────────────────────────────────────────────────────
CABLE_D = 3.8                         # source-cable Ø (the larger of the two cables)

# ── Capacity → the ONE knob that sets every OD (copied from the original design service_loop) ──
# The tray must clear the reverse-wound source-cable coil out to R_OUT_COIL; the
# floor caps that coil just inside the chamber. FLOOR_OD derives from it, and the
# separator/brake OD, the cap ODs, and the wall all derive from FLOOR_OD — so
# changing R_OUT_COIL (capacity) re-sizes the whole part.
R_IN_COIL   = 15.0                              # inner coil radius (arbor / bend limit)
R_OUT_COIL  = 71.0                              # outer coil radius (original value)
CHAMBER_R   = R_OUT_COIL + CABLE_D / 2 + 1.0    # 73.9 — coil clearance radius (the original design)
FLOOR_OD    = 2 * (CHAMBER_R - 1.0)             # 145.8 — floor caps the coil (original platen OD)

# ── Middle brake/ratchet RIM — a cylinder sliding on the spring chamber ──────
# Split top-half BRAKE / bottom-half RATCHET, with a 45° cone between (ratchet →
# cone chamfer → brake), copied from the working lever rim. Slides down the hub
# OD and seats on the spring chamber's cone notch at mid-height.
# RIM_H is now DERIVED (was fixed 12): the BRAKE band sizes itself from the
# pad's hook joint — the joint drives the stack, not the other way round.
RIM_BORE_CLR  = FIT_CLR + 0.10        # slide fit over the hub OD
RIM_ID        = HUB_OD + 2 * RIM_BORE_CLR         # ~66.9 — bore (rides the hub)
RIM_OD        = FLOOR_OD               # separator/brake OD = floor OD (derived from capacity)
RATCHET_DEPTH = 3 * NOZZLE                   # tooth tip→valley (also the 45° cone radial
                                      # step): three full 0.8-nozzle perimeters
                                      # (user's call — deeper bite; was 1.6)
RATCHET_TEETH = 60
# (the old 45° ratchet→brake CONE is GONE — user's call: the separator
# prints INVERTED (+Z→−Z), so the teeth grow directly on the brake band's
# face with nothing to transition; the pawl gets an explicit top clearance
# instead, see levers.py)
# Brake pad ↔ arm joint (cadkit hook, single-flank — the joint face is
# edge-bounded) + the pad's z margins inside the band. The pad is an
# L-SECTION now (user's design): a band-contact SLAB sized by the band's
# margins, plus a taller back FLANGE carrying the hook joint at its QUALITY
# width — the joint no longer sizes the band (an 8.5 band once grew the rim
# to 15 and ate 3.0 of spool-chamber headroom, user-caught). The flange
# nests into a recessed, taller arm end: at that radius (behind the old arm
# face) it clears the coil keep-out above and the cone/teeth below.
PAD_JOINT_CLR        = 0.1            # TPU in PETG — snug
BRAKE_PAD_TOP_MARGIN = 0.0            # ZERO margins (user's call): the pad
BRAKE_PAD_BOT_MARGIN = 0.0            # covers the FULL brake band top to
                                      # bottom, parallel, at first contact.
                                      # The over-rotation checks are gone —
                                      # the brake validates at CONTACT only;
                                      # past contact is the TPU squeeze and
                                      # the pad meeting the teeth is the
                                      # natural travel limit
PAD_SPEC      = PrintSpec(nozzle=NOZZLE, facing="up")   # both halves print along
                                      # the joint's install axis (the pad on its
                                      # end, the lever along Y)
PAD_FLANGE_H  = joint_box_min(PAD_SPEC, PAD_SPEC, install="-z", bounded=True,
                              quality=True, clearance=PAD_JOINT_CLR)[0]
                                      # 6.6 — the flange face: the edge-bounded
                                      # site's QUALITY width (all-1.6 segments
                                      # on both halves)
PAD_FLANGE_T  = 2 * NOZZLE                   # flange depth — the arm face recesses by
                                      # this so the flange never protrudes past
                                      # the old face radius
BRAKE_H       = 5.0                   # top brake band height (user's call —
                                      # 5 / 5 / 2.4 bands)
RATCHET_H     = 5.0                   # bottom ratchet band height (back to the
                                      # original engagement height)
RIM_H         = RATCHET_H + BRAKE_H               # 10.0 — the cone's 2.4 comes
                                                  # back as spool-chamber
                                                  # headroom too
# 45° cable PASS-THROUGH tunnel (ported from the original design): lets
# the working cable cross from the spool chamber (above the separator) to the
# tray chamber (below), connector included. Tilted tangentially, so it prints
# support-free in the separator's flat print; placed between two key slots.
SEP_PASS_W         = 13.0             # square tunnel section (connector passes)
SEP_PASS_ANGLE_DEG = 30.0             # azimuth (mid-way between key slots at 0/60°)
SEP_PASS_WALL      = 2 * NOZZLE              # web left between the tunnel and the bore

CLAMP_HW_ANGLE_DEG = 30.0             # height-clamp pinch hardware azimuth: halfway
                                      # between two key slots (60° pitch), so the
                                      # C-slit doesn't run through a key groove

# ── Rim seat: a 45° cone-supported notch on the spring chamber ───────────────
# Holds the rim at HALF height (equal spring-chamber length above and below).
RIM_SEAT_Z    = (SPOOL_H - RIM_H) / 2             # 19.5 — rim bottom rests here
SEAT_SHOULDER_OD = RIM_ID + 3.0                   # >rim bore, so the rim stops on it
SEAT_CONE_H   = (SEAT_SHOULDER_OD - HUB_OD) / 2   # 45°: radial step == axial rise

# ── Minimal housing: 4 vertical beams + a connecting wall ring ───────────────
# Beams at ±X/±Y only (no levers/guide wheels yet); the ring they carry is BOTH
# the spool wall (upper 1.5·D) and the tray wall (lower 1.5·D), bracketing the
# 12 mm middle rim. Beam height = wall height for now (extended +Z later).
BEAM_SIZE     = 10.0                              # 10 × 10 mm square section
STATIC_MOVE_CLR = 4 * NOZZLE                             # the project's standard clearance between
                                                  # STATIC and MOVING parts
RIM_TO_WALL   = 4 * NOZZLE                               # radial rim gap at the ORIGINAL 1.6 wall —
                                                  # kept as the anchor that FREEZES WALL_OR
                                                  # (the frame interface must not move)
WALL_OR       = RIM_OD / 2 + RIM_TO_WALL + STRUCT_WALL   # 77.7 — outer radius = the joint
                                                  # mating plane (beams, tenons — frozen)
WALL_T        = 3 * NOZZLE                               # ring radial thickness — bumped +0.8
                                                  # INWARD (user: more hoop rigidity after
                                                  # the bottom cable escaped wall↔floor)
WALL_IR       = WALL_OR - WALL_T                  # 75.3 — inner face (rim gap now 2.4)
BEAM_IR       = WALL_OR                           # beams sit just outside the separate wall
                                                  # ring (wall OD = beam inner face; the
                                                  # embedded-wall variant was tried and
                                                  # reverted by user call)
WALL_Z0       = 12.0                              # wall WINDOW-BAND bottom (tray side) —
                                                  # reaches the cable floor's TOP at its
                                                  # lowest (clamp bottomed): FLOOR_MIN_BOT_Z
                                                  # + CAP_H (both defined below; asserted at
                                                  # EOF), so the tray stays walled at any
                                                  # clamp setting
WALL_SKIRT    = 4 * NOZZLE                               # ring + tenons extend BELOW WALL_Z0
                                                  # (user: rigidity): the bottom lands 3.2
                                                  # above the +X climb's top plane at the
                                                  # wall band (measured 6.4 gap, asserted
                                                  # against frame's climb math at EOF)
WALL_ZB       = WALL_Z0 - WALL_SKIRT              # 8.8 — ring/tenon bottom = seat reference
WALL_TOP_BAND = 2 * NOZZLE                               # unbroken top ring above every window
# WALL_H / WALL_Z1 are DERIVED in the wall-windows block below (they need the
# lever constants): wall top = ratchet-window ceiling + WALL_TOP_BAND.

# ── Wall ↔ frame joinery (cadkit slide_joint, install="z" → plain dovetail) ──
# The WALL is a separate part carrying 4 dovetail TENONS (it's thin) on its
# outer face; each beam's inner face hosts the matching MORTISE channel. Both
# parts print −Z→+Z and the joint INSTALLS along Z (the library's install="z"
# family): the plan profile prints as vertical walls, so the classic dovetail
# applies with no arrow/octagon print compromises.
# INSTALL: the wall slides DOWN from the BEAM TOPS (the beams now root in the
# bottom plus, so there is no bottom entry) — each channel runs open from the
# beam top down to a stop just BELOW the seated wall tenon; the wall rests on
# those stops. Sequence: wall → frame_bottom, then frame_bottom → frame_top
# (the top then caps the open channel ends).
# Joint size = AVAILABLE room (the library optimizes the profile inside it and
# may use less): width along the beam face, depth into the 10 mm beam (leave
# ≥ 4 mm backing behind the cavity end).
JOINT_WIDTH    = 6.5 * NOZZLE                              # available width on the beam face —
                                                  # the SMALLEST room whose tenon has
                                                  # EVERY wall segment ≥ 1.6 (user
                                                  # measured 1.5s at the old box): neck
                                                  # 1.92, shoulders 1.6, head 5.12. The
                                                  # pin bores move out with it (see
                                                  # PIN_TIP_END_Y) to keep ≥ 1.6 of beam
                                                  # beside the cavity (EOF-asserted)
# Clearances come from CADKIT'S POLICY (user's call — callsites stop
# hardcoding them): the material table picks the lateral base, and the
# GF-filled filament DOUBLES the T's depth-face gap (only the mortise back
# wall deepens — print finding: the GF wall joint slid fine laterally at
# 0.15 but bound in the depth sandwich until that face reached 0.3).
JOINT_SPEC     = PrintSpec(nozzle=NOZZLE, material="PETG-GF", facing="up")
JOINT_CLR, JOINT_BACK_CLR = joint_clearances(JOINT_SPEC, JOINT_SPEC)
                                                  # 0.15 lateral / 0.30 back wall
JOINT_DEPTH    = joint_box_min(JOINT_SPEC, JOINT_SPEC, install="-z",
                               quality=True)[1]   # 3.5 — a 1.6 head bar + a 1.6 MORTISE
                                                  # lip + the DEPTH-FACE gap (the tenon
                                                  # lip is pre-grown by JOINT_BACK_CLR in
                                                  # the library so both depth pairs run
                                                  # at 0.3 — GF print finding — while
                                                  # every wall keeps 1.6; the depth box
                                                  # absorbs it); neck lands 1.92; keeps
                                                  # the frame_top arc-joint ring (r_T
                                                  # tracks the swallow) inside the arms
JOINT_SEAT_CLR = 0.15                             # z seating clearance at the bottom stop
# (MORTISE_L is derived below the frame section — it runs to the beam tops.)

# ── AXIAL SERVICE LOOP — the free-floating torsion coil (replaces the tray) ──
# The source loop is a single helix in the annulus below the housing,
# anchored to the DRUM at the separator pass (top) and to the stationary
# coil_cup at its port (bottom), sized to touch NOTHING at either extreme:
# drum rotation changes the coil's DIAMETER (an end moment distributes
# uniformly along a contact-free coil — a torsion spring), so no cable is
# ever dragged through wrapped contact and the capstan grip failure (user's
# concern — near-separator wraps binding the hub) cannot start. Chirality:
# RETRACTED = LOOSE (relaxed storage) / EXTENDED = TIGHT, so the stored
# torque assists retraction.
# TOPOLOGY RULE that reshaped the frame: a rotating coil (or its rotating
# drop from the pass) can never cross a stationary full-radius plane — so
# the bottom plus now sits BELOW the whole chamber (the beams lengthen) and
# the stationary cup floor is the only full disc under the rotating parts.
RETRACT_LEN   = 6.0 * 304.8                       # 1828.8 — 6 ft of payout (user's
                                                  # capacity call; was 4 ft)
COIL_R0       = (HUB_OD / 2.0 + KEY_DEPTH         # 37.2 — spool coil inner wrap
                 + CABLE_D / 2.0 + 0.5)           # (clears the hub key tongues)
COIL_PITCH    = CABLE_D + 0.1                     # 3.9 — wrap-on-wrap pitch
N_MIGRATE     = ((-COIL_R0 + math.sqrt(
    COIL_R0 ** 2 + COIL_PITCH * RETRACT_LEN / math.pi))
    / COIL_PITCH)                                 # ≈ 5.96 — drum turns, full travel
assert COIL_R0 + COIL_PITCH * N_MIGRATE <= R_OUT_COIL + 1e-9, \
    "spool coil outgrows the spool chamber capacity radius"
CABLE_BEND_R_MIN = 15.0                           # HARDWARE: the 3.8 cable's safe
                                                  # bend radius (flat-loop killer)
LOOP_D_TIGHT  = 2.0 * (CABLE_BEND_R_MIN + 1.0)    # 32 — extended (tight) centreline
LOOP_D_LOOSE  = 2.0 * WALL_IR - CABLE_D - 1.4     # 145.4 — retracted: 0.7/side off
                                                  # the cup bore
LOOP_K_TIGHT  = N_MIGRATE * LOOP_D_LOOSE / (LOOP_D_LOOSE - LOOP_D_TIGHT)
                                                  # ≈ 7.64 wraps at full extension
LOOP_K_LOOSE  = LOOP_K_TIGHT - N_MIGRATE          # ≈ 1.68 wraps retracted
LOOP_LEN      = math.pi * LOOP_D_TIGHT * LOOP_K_TIGHT   # ≈ 768 — the loop's cable
AXIAL_STACK   = (LOOP_K_TIGHT + 1.0) * COIL_PITCH # ≈ 33.7 — worst-case wrap stack
CH_TOP_Z      = -2.5                              # coil ceiling: under the housing
                                                  # bottom (0) with room for the cap
                                                  # grip ring band
CH_BOT_Z      = CH_TOP_Z - AXIAL_STACK - 1.0      # ≈ −37.2 — chamber floor (cup
                                                  # floor top face)
CUP_FLOOR_T   = 3 * NOZZLE                        # cup floor plate
CUP_Z0        = CH_BOT_Z - CUP_FLOOR_T            # ≈ −39.6 — cup bottom = plus top
CUP_SLEEVE_OD = 34 * NOZZLE                       # 27.2 — stationary inner guide:
                                                  # tight coil ID keeps 0.5/side off it
CUP_SLEEVE_T  = 2 * NOZZLE
CUP_COLLAR_T  = 3 * NOZZLE                        # radial wall around the axle bore
CUP_COLLAR_H  = 4 * NOZZLE                        # boss above the floor plate
CUP_RIB_T     = 3 * NOZZLE                        # beam-flank torque ribs (plan sq.)
CUP_PORT_AZ_DEG = 200.0                           # source port azimuth (the "back",
                                                  # clear of the 180° beam by ~16°)
CUP_PORT_W    = SEP_PASS_W                        # 13 — the connector threads it
CUP_PORT_SILL = CH_BOT_Z + 0.5                    # house-port sill just off the floor
CLIMB_X0      = WALL_OR + 0.3                     # the +X trident's 45° climb now
                                                  # starts OUTSIDE the wall/cup band
                                                  # (its old floor-clearance anchor
                                                  # died with the tray)
assert LOOP_D_TIGHT - CUP_SLEEVE_OD - CABLE_D >= 0.8 - 1e-9, \
    "tight coil pinches the cup sleeve"
assert 2 * WALL_IR - LOOP_D_LOOSE - CABLE_D >= 1.2 - 1e-9, \
    "loose coil pinches the cup bore"
assert LOOP_D_TIGHT / 2.0 >= CABLE_BEND_R_MIN - 1e-9, \
    "tight coil's centreline under the cable's bend radius"

# ── Frame split: frame_bottom (bottom plus + beams) / frame_top (top plus) ───
# frame_bottom: a plus-shaped spider BELOW the spring housing (same 3.2 mm
# clearance as the top side) + the four vertical beams (carrying the wall
# mortises) rising from it, + octagon TENONS on the beam tops. frame_top: just
# the top plus spider with the matching mortise channels in its underside.
# BOTH parts print −Z→+Z: frame_top gets its mounting hardware on a flat build,
# frame_bottom prints beams-up (the old monolithic frame had to print +Z→−Z to
# put the top spider on the bed). NOTE: the bottom plus blocks the removable
# spring_housing_cap's −Z exit — install the cap before the frame.
HOUSING_TOP_CLR = 4 * NOZZLE                             # top-plus underside ↔ housing top
HOUSING_BOT_CLR = 4 * NOZZLE                             # bottom-plus top ↔ housing/cap bottom
TOP_RIB_SIZE    = 10.0                            # plus-arm section (10 × 10, matches beams)
TOP_RIB_Z0      = SPOOL_H + HOUSING_TOP_CLR       # 54.2 — top plus underside
TOP_RIB_Z1      = TOP_RIB_Z0 + TOP_RIB_SIZE       # 64.2 — top plus top
BOT_RIB_Z1      = CUP_Z0                          # ≈ −39.6 — bottom plus top: the
                                                  # plus dropped BELOW the axial
                                                  # chamber (topology rule above) and
                                                  # now CARRIES the coil_cup, whose
                                                  # floor rests directly on the arms
                                                  # (HOUSING_BOT_CLR retired — the
                                                  # chamber is the clearance)
BOT_RIB_Z0      = BOT_RIB_Z1 - TOP_RIB_SIZE       # −13.2 — bottom plus underside
BEAM_Z0         = BOT_RIB_Z0                      # beams fuse down through the bottom plus
BEAM_Z1         = TOP_RIB_Z0                      # beams end at the top plus underside
# Wall-mortise channel: open at the beam TOP (wall enters there, slides down),
# hard stop JOINT_SEAT_CLR below the seated tenon bottom (= the skirt bottom
# WALL_ZB — the tenons run the ring's FULL extended height).
MORTISE_L       = (BEAM_Z1 + 1.0) - (WALL_ZB - JOINT_SEAT_CLR)   # stop → past beam top

# ── frame_top ↔ frame_bottom joinery (ROTATIONAL octagon arc joints) ─────────
# The AXLE locks both frames coaxially — no straight slide direction exists;
# the one free motion is ROTATION about the axle. So the joints are the
# library's octagon-ARC variant (the octagon profile revolved about the Z axis
# at the site radius; both parts still print −Z→+Z, roof-bridge print rules
# unchanged).
# INSTALL (CW to seat): rotate frame_bottom CCW off its seat by
# ~TOP_ROT_INSTALL_DEG, mate ALONG Z (all four tenons rise through the open
# quadrants between the top's arms), then rotate CW until the tenons seat
# against ANGULAR stops at each arm's CW face. RETRACTION TORQUE IS CW (the
# spool tightens the spring), so operation PRELOADS the joints against their
# stops; uninstall = CCW, against the load — it can't shake free in service.
# Radially the tenons sit just outside the wall channels' keep-out (they'd
# otherwise block the wall sliding down); angular extents are DERIVED in
# frame.py: tenon half-angle = arm half-angle − stop − seat.
# HALF-ARROWHEAD profile (user redesign after the coupon print — the
# cadkit octagon arc bound CONSIDERABLY and pushed the top arms 0.65
# proud of the frame extent): the INNER side is ONE FLAT cylindrical
# face against the wall-joinery keep-out; retention is the single
# OUTBOARD 45° flare; the top is a 45° taper to a small dull tip AT the
# flat — no cavity roof bridge at all in frame_top's upright print.
# Clearances follow the T's print-validated GF policy (user's call):
# lateral faces at JOINT_CLR, the Z-SANDWICH pairs (flare↔lip, taper↔
# ceiling) at JOINT_BACK_CLR — fiber walls bind in the load sandwich.
TOPJ_STEM           = 3 * NOZZLE                         # stem width (radial, off the flat)
TOPJ_FLARE          = 2 * NOZZLE                         # single outboard 45° flare = the lip
TOPJ_NECK           = 2 * NOZZLE                         # neck height under the flare
TOPJ_TIP            = NOZZLE                         # dull tip at the flat wall
TOP_JOINT_SEAT_CLR  = 0.15                        # seating clearance at each stop (arc mm)
TOP_ENTRY_OVER      = 1.0                         # channel overshoot past the CCW entry
                                                  # faces (arc mm)
TOP_WALL_CLEAR      = 0.1                         # tenon radial gap off the wall channel —
                                                  # right at the wall-install limit
TOP_STOP_WALL       = 2 * NOZZLE                         # stop-wall thickness at each arm's CW
                                                  # face (arc mm)
TOP_ROT_INSTALL_DEG = 20.0                        # suggested CCW install offset (anything
                                                  # ≥ ~7° clears the arms on z-mate)

# ── Axle (original two-half axle, mirrored + extended) + diagonal rod pin ──────────
# The axle spans the frame: bottom end flush with the bottom plus underside,
# top end flush with the top plus top; both plus centres carry slip bores.
# Fixing hardware = a printed PLASTIC ROD through a horizontal plan-45° bore
# (+xy → −xy) through the frame_top crossing AND the axle — inserted from the
# open quadrant between arms; the spring's constant radial load on the axle
# friction-locks it (no thread). Replaces the original design's M2 cross-pin.
AXLE_BORE_D = AXLE_PRINT_D + 2 * FIT_CLR          # 8.25 — plus-centre slip bores
ROD_CLR     = 0.3                                 # bore clearance PER FACE, all around —
                                                  # loose on purpose: no fit job here, the
                                                  # spring's radial load friction-locks it
ROD_TIP_W   = NOZZLE                                 # the OCTAGON's four dull flats (top /
                                                  # bottom / sides, 1×nozzle like the
                                                  # half-arrowhead tips)
ROD_PRINT_H = 4 * NOZZLE                                 # the rod PRINTS rolled 45° onto one of
                                                  # its LONG diagonal faces (user's
                                                  # orientation); this is its height that
                                                  # way (= across-diagonals span), pinned
                                                  # to a whole number of 0.8 print steps —
                                                  # off-grid heights (3.5, then 2.83)
                                                  # printed out-of-square
ROD_D       = ROD_PRINT_H * math.sqrt(2.0) - ROD_TIP_W
                                                  # ≈ 3.73 — octagon width = height across
                                                  # flats, DERIVED from the print-height
                                                  # grid. OCTAGON, not round/house:
                                                  # 0.8 flats + 45° diagonals is
                                                  # 180°-symmetric, so ONE bore shape is
                                                  # self-supporting in every consumer —
                                                  # frames + axle_bottom print −Z→+Z,
                                                  # axle_top prints FLIPPED
ROD_L       = 20.0                                # rod length (crossing diagonal + grip ends)
ROD_Z       = TOP_RIB_Z0 + TOP_RIB_SIZE / 2.0     # 60.4 — top rod axis (top crossing mid)
ROD_Z_BOT   = BOT_RIB_Z0 + TOP_RIB_SIZE / 2.0     # −8.2 — bottom rod axis (bottom crossing mid)

# ── Pre-wind hex drive + printed winding tool ────────────────────────────────
# INSTALL UNTENSIONED, wind LAST: the bottom axle ends in a male hex sticking
# out below the frame; with the rod pins still out, turn it to pre-wind the
# spring, then lock the torque in with the bottom rod pin. The hex is
# INSCRIBED in the axle's shaft profile (across-corners = AXLE_PRINT_D), so
# the cap's 608 bearing still slides over it at install; that makes the
# across-flats ≈ 6.89 — a standard 7 mm female hex bit/socket drives it. For
# users without a bit: a printed flat BAR tool with a female hex socket
# through its middle (turn by the two handle ends).
DRIVE_SQ_S      = AXLE_PRINT_D / math.sqrt(2.0)   # 5.62 — SQUARE drive side, inscribed
                                                  # in the shaft Ø (bearing still slides
                                                  # over). Square, not hex (user: the
                                                  # hex's six short facets printed ROUND
                                                  # at 0.8-nozzle scale — four real flats
                                                  # give a solid gripping surface)
DRIVE_L         = 12 * NOZZLE                     # 9.6 — male drive length below the
                                                  # frame (shortened: with the frame
                                                  # deepened for the axial chamber,
                                                  # every mm below the plus is unit
                                                  # height; 8 of socket engagement
                                                  # remains for the pre-wind — flag)
DRIVE_Z0        = BOT_RIB_Z0 - DRIVE_L            # −28.2 — drive tip
WIND_TOOL_CLR   = 0.35                            # socket side clearance
WIND_TOOL_L     = 110.0                           # bar length (two ~50 mm handle arms)
WIND_TOOL_W     = 16.0                            # bar width (≥3.8 wall around the socket)
WIND_TOOL_T     = 5.0                             # handle-arm thickness (prints flat −Z→+Z)
WIND_TOOL_BOSS_OD = 16.0                          # centre boss around the socket
WIND_TOOL_BOSS_H  = DRIVE_L - 2 * NOZZLE                 # 13.4 — socket grips the drive BELOW
                                                  # the square→shaft junction cone (the
                                                  # cone's Ø exceeds the square socket —
                                                  # scan-caught at the old full length)

# ── Cap-retention TPU grip ring (bottom axle) ────────────────────────────────
# A small solid 95A washer gripping the axle shaft directly under the spring
# housing cap: the cap has HOUSING_BOT_CLR of room to creep −z before the
# bottom plus catches it — the ring pins it in place instead (user's call).
# Same station-keeping philosophy as the big hub grip rings: light bite +
# layer-corrugation interlock; continuously z-adjustable, hardware-free.
AXLE_RING_H     = 2 * NOZZLE                             # ring height (tier; fits the 3.2 gap
                                                  # with 1.6 spare above the bottom plus)
AXLE_RING_OD    = 16 * NOZZLE                            # grip OD (0.8-grid; ≈ the axle lip Ø)
AXLE_RING_BITE  = 0.4                             # diametral undersize on the shaft

# ── Cable-chamber caps — cable_ceiling (+Z) and cable_floor (−Z) ─────────────
# Plain 10 mm cylinders sliding on the hub, one each side of the separator, key-
# slotted so they turn with it. No self-clamp: the wound CABLE stops them moving
# toward the separator; a separate height_clamp locks the other direction.
SEP_Z0        = RIM_SEAT_Z                        # 19.5 — separator bottom
SEP_Z1        = RIM_SEAT_Z + RIM_H                # 31.5 — separator top
CAP_H         = 10 * NOZZLE                               # cap height
CAP_OD        = FLOOR_OD                           # ceiling + floor OD = floor OD (master)
CABLE_GAP     = CABLE_D + 0.4                     # 4.2 — separator↔cap gap: one cable layer + clr
# Cap WINDOWS (cable_ceiling + cable_floor): arc slots to SEE the wound
# cable at install (and save some material). Slicer-aware (0.8 nozzle, 2
# loops, 3×0.4 skins): FEW LARGE openings (each slot's wall loops are the
# cost; its removed floor/ceiling skin is the win), and every web is 3.2
# wide = exactly 4 perimeter beads → prints as solid wall, no infill core.
# the original design's lesson: spoke EDGES flexed and lost cable retention → heavy solid
# OUTER RIM at the OD, a MID band tying the spokes at half-span, spokes
# CONTINUOUS root→rim (straight radial beams), one on every key azimuth.
CAP_SPOKE_N      = 24                             # spoke count (user's call — tighter
                                                  # cable-pressure spacing; still lands a
                                                  # spoke on every 60°-pitch key azimuth)
CAP_SPOKE_W      = 4 * NOZZLE                            # web width = 4 beads at the 0.8 nozzle
# Ring widths — all 0.8-bead multiples, graded by how much backing each ring
# has (user's call): the INNER collar rides the spring-housing hub (fully
# backed → thinnest); the MID band ties the spokes at half-span; the OUTER
# rim is unbacked and carries the retention pressure (thickest).
CAP_RING_W_IN    = 2 * NOZZLE                            # 2 beads
CAP_RING_W_MID   = 4 * NOZZLE                            # 4 beads
CAP_RING_W_OUT   = 6 * NOZZLE                            # 6 beads
CAP_HUB_COLLAR_R = RIM_ID / 2.0 + CAP_RING_W_IN   # ≈ 35.05 — collar around the bore
CAP_MID_R0       = 53.0 - CAP_RING_W_MID / 2.0    # 51.4 — mid band centred on the
CAP_MID_R1       = 53.0 + CAP_RING_W_MID / 2.0    # 54.6   coil span's middle (~53)
CAP_RIM_R0       = CAP_OD / 2.0 - CAP_RING_W_OUT  # 68.1 — rim solid to the OD
CEIL_Z0       = SEP_Z1 + CABLE_GAP                # ceiling a cable-gap above the separator
# (the tray-side cable_floor, its FLOOR_Z0 station, the wall's cable LOCK
# BAND and the source PORT are GONE — the flat-wound tray service loop
# FAILED its physical wind test (2026-08-03, user) and the whole flat tray
# design is dropped. The source loop's replacement is the AXIAL floating
# torsion coil (see the service-loop exploration artifact): a ~35–40 mm
# chamber below the separator, to be designed — the pass tunnel is its
# feed and stays.)

# ── height_clamp — a 4 mm C-clamp pinching the hub to lock a cap in Z ────────
CLAMP_H       = 5 * NOZZLE
CLAMP_OD      = HUB_OD + 10 * NOZZLE                      # 74.4 — collar OD (room for the pinch boss)
CLAMP_SLIT_W  = 1.4                               # C-slit gap; the M2 screw closes it onto the hub
CEIL_CLAMP_Z0  = CEIL_Z0 + CAP_H                  # clamp ABOVE the ceiling (locks it down)

# ── Wall SPLIT + cable exit port ─────────────────────────────────────────────
# The wall prints as TWO stacked halves — bottom −Z→+Z, top +Z→−Z (inverted)
# — so every window/port CEILING is bed-side geometry in its own print: no
# overhangs, no supports. The split plane passes through the cable exit port
# (floor in the bottom half, ceiling in the top half); the brake window stays
# an open-top notch in the bottom half, capped by the top half's underside.
CABLE_EXIT_ANGLE_DEG = 45.0                       # port arc CENTER azimuth (+x,+y — the original design-style)
# The cable leaves the wall TANGENT to the coil's current outer wrap, so its
# wall-crossing point sweeps as the spool fills/empties: the angular offset
# from the cable's travel direction is asin(r_wrap / WALL_IR) (the original design's
# cable_retainer math). Spanning wraps from the bare hub out to design
# capacity (R_OUT_COIL) covers any 6 ft payout window no matter what total
# cable length ends up wound. Margin = cable radius + clearance at each end.
CABLE_EXIT_R_LO       = HUB_OD / 2                # innermost tangency (spool empty)
CABLE_EXIT_R_HI       = R_OUT_COIL                # outermost tangency (spool full)
CABLE_EXIT_MARGIN_DEG = math.degrees((CABLE_D / 2 + 1.0) / WALL_IR)      # ≈ 2.2°
CABLE_EXIT_SPAN_DEG   = (math.degrees(math.asin(CABLE_EXIT_R_HI / WALL_IR))
                         - math.degrees(math.asin(CABLE_EXIT_R_LO / WALL_IR))
                         + 2 * CABLE_EXIT_MARGIN_DEG)                    # ≈ 47.5°
CABLE_EXIT_Z0  = SEP_Z1 - 0.5                     # 31.0 — port floor
CABLE_EXIT_Z1  = SEP_Z1 + CABLE_GAP + 0.5         # 36.2 — port ceiling
WALL_SPLIT_Z   = SEP_Z1 + CABLE_GAP / 2.0         # 33.6 — stack plane (mid-port)

# (the SOURCE PORT is gone with the flat tray design — the axial coil
# chamber will carry its own port at its bottom edge when it lands)

# ── Levers — both-side supported, HARDWARE-FREE (TPU torsion-bar pivots) ─────
# the original design's plate architecture kept: each lever is an XZ plate (LEVER_T thick in Y)
# pivoting about a Y axis at +X; ratchet pivot ABOVE its band (rest=ENGAGED),
# brake pivot BELOW its band (rest=OFF); pull = +X. changes: the plates
# FLANK the +X beam (y ±5) instead of a housing spine; each pivot pin IS the
# spring — a 95A TPU TORSION BAR spanning two fork posts (support both sides):
# hex-keyed ends in the posts, a hex-keyed middle in the lever, round NECKED
# spans between that twist as the lever rotates. The post keyways are CLOCKED
# PIN_PRETWIST_DEG past rest, so installing the pin pre-twists it → real
# seating preload at rest (pawl held into the teeth; brake held on its rest
# stop) with nothing protruding at +X. The brake pad is TPU as before. The
# WALL gets two open-top windows at +X so the arms reach the separator rims
# (bottom ring left intact).
LEVER_T          = 10.0                           # plate thickness (Y) — ergonomic (the original design)
LEVER_HANDLE_W   = 6.0                            # handle X width (the original design)
# Thrust boss rings — sized by cadkit's contact-rib rule: TWO nozzles (the
# quality tier — a single bead sliced mushy, user print finding), both WIDE
# and PROUD (1.6 at the 0.8 nozzle). Both rings are IDENTICAL with the
# square axle: bore = BOSS_BORE_ID, clearing the square's rotating
# envelope (the old design needed a bigger column-side relief bore — the
# ring-around-its-local-bore lesson still applies if bores ever diverge).
# The LEVER AREA GREW to fit (user's call): the side
# gaps are ring-proud + 0.2 float, and the whole y-layout derives from them
# (the print-proven original spacing had 1.0 gaps; _CLR_D tracks the growth so
# the pin stack follows).
PIN_SQ_S         = 4.4                            # SQUARE axle side — sharp 90° corners
                                                  # grip where the old hex's 120° corners
                                                  # printed too soft (the axle block below
                                                  # has the full story). 4.4 is the
                                                  # LARGEST side whose diamond bore keeps
                                                  # 1.6 of beam to the frame's +x face AND
                                                  # whose rings stay inside the frame
                                                  # extent (both EOF-asserted; was 4.8,
                                                  # which poked the rings 0.4 proud and
                                                  # left 1.26 at the diamond — user-caught)
BOSS_BORE_ID     = PIN_SQ_S * math.sqrt(2.0) + 0.4    # ≈ 6.62 — ring bore clears the
                                                  # axle's rotating envelope + 0.4; the
                                                  # rings track the axle automatically
BOSS_RING_W      = contact_rib_size(NOZZLE)       # 1.6 — rib width AND proud
LEVER_SIDE_CLR   = BOSS_RING_W + 0.2              # 1.8 — air each side of the lever
                                                  # plates (ring + light-touch float)
_CLR_D           = LEVER_SIDE_CLR - 1.0           # 0.8 — layout growth vs original spacing
LEVER_Y_IN       = BEAM_SIZE / 2.0 + LEVER_SIDE_CLR   # 6.8 — inner plate face
RATCHET_LEV_Y0   = LEVER_Y_IN                     # 6 — ratchet plate on the +Y side
RATCHET_LEV_Y1   = LEVER_Y_IN + LEVER_T           # 16
BRAKE_LEV_Y1     = -LEVER_Y_IN                    # −6 — brake plate on the −Y side
BRAKE_LEV_Y0     = -LEVER_Y_IN - LEVER_T          # −16
# BRAKE PAD: TALL, not wide (user's calls — a widened arm was tried and
# reverted: the arm body would run through the fork column). The brake uses
# a CONTACT-POSE solver (levers.py): validated at FIRST CONTACT only, where
# the pre-tilted pad arrives PARALLEL to the band and FLUSH against it with
# its corners landing inside the band; the pull beyond is the TPU
# compression regime (reported, not asserted). Guards that remain: a bottom
# margin keeps the full-pull drop ≥1 off the ratchet teeth, and a swept
# assertion keeps the tall resting pad out of the spool chamber's coil
# keep-out. Yields pad H ≈ 3.8 (vs 2.4 under the original design's mid-travel constraints).
# pivot x — DERIVED: as close to the spool as possible with the thrust
# rings' extremities landing exactly ON their arm's inner edge (x = BEAM_IR;
# the substrate is the binding limit — and it keeps clear of the wall's
# swept surface). ≈ 82.9 with the square-axle rings.
LEVER_PIVOT_X    = BEAM_IR + BOSS_BORE_ID / 2.0 + BOSS_RING_W
RATCHET_PIVOT_Z  = SEP_Z0 + RATCHET_H + 10.0      # 34.5 — above the tooth band
BRAKE_PIVOT_Z    = SEP_Z0 + RATCHET_H - 5.5       # below the brake band (which
                                                  # now starts right at the
                                                  # tooth band's top)
LEVER_TRAVEL_DEG = 20.0                           # equal pull travel, both levers (the original design)
HANDLE_Z_BOT     = -13.0                          # grab height (≈ bottom plus underside)
# SQUARE torsion AXLE (95A TPU, user redesign — the stepped hex pin's small
# corners printed too soft to grip the frame or lever): ONE plain square
# PRISM per lever, constant section end to end, inserted axially from the
# OUTER post's face. The FRAME's holes are the square rotated 45° —
# DIAMOND, tip-up: the flanks are 45° (self-supporting) and the top is a
# point, so the sideways frame holes print clean with no teardrop. The
# LEVER's through-pocket is clocked 45° − PIN_PRETWIST_DEG: inserting the
# straight axle (keyed 45° at BOTH frame ends) with the lever held pulled
# by the pre-twist angle twists the middle — the torsion springs are the
# two short free spans across the side gaps, in parallel. Torque path: lever
# square → both gap spans twist → column keyway + beam blind bore.
# Stiffness now comes from section side (∝ s⁴) and the gap lengths —
# print-test on the rig and tune PIN_SQ_S / PIN_PRETWIST_DEG.
PIN_SQ_FRAME_CLR = 0.1                            # per-side, column keyway + beam bore
                                                  # (PIN_SQ_S lives up with the rings —
                                                  # the ring bore derives from it)
PIN_SQ_LEVER_CLR = 0.15                           # per-side, the lever's through-pocket
PIN_TIP_END_Y    = 4.4                            # |y| of the seated axle's inner end =
                                                  # the blind bore's floor (depth stop)
                                                  # — placed so ≥ 1.6 of beam separates
                                                  # the bore floors from the wall-joint
                                                  # cavity (±2.71; EOF-asserted)
PIN_GRIP_L       = 2.0                            # axle runs proud of the column face —
                                                  # the removal/inspection grip
PIN_PRETWIST_DEG = 12.0                           # keyway↔pocket RELATIVE clock →
                                                  # install pre-twist = seating preload
                                                  # (~37% sustained shear; a 3° creep-
                                                  # minimizing bias was tried and
                                                  # REVERTED — user's call, creep
                                                  # judged acceptable; drop this if a
                                                  # tensioned rig pin measures weak
                                                  # after a week)
PIN_KEY_BASE_DEG = 45.0                           # frame keyway base clock: square at
                                                  # 45° = DIAMOND TIP-UP in the frame's
                                                  # sideways print (printable); the
                                                  # lever pocket sits at BASE − PRETWIST
                                                  # (prints vertically, clock-free)
POST_OUT_T       = BEAM_SIZE / 2                            # side-arm section thickness — HALF the
                                                  # 10 mm centre arm (user's call): the
                                                  # column + 45° diagonal + horizontal top
                                                  # run keep this constant 5×10 section
# axle length: blind-bore floor → column outer face → grip stub.
LEVER_PIN_L      = (RATCHET_LEV_Y1 + LEVER_SIDE_CLR + POST_OUT_T
                    + PIN_GRIP_L - PIN_TIP_END_Y)                # 21.2
# pivot BOSS Ø on each lever (the 6 mm handle is narrower than the pocket)
# — DERIVED (was a fat hardcoded 12.0, then a 0.2/side thrust-ring seat
# margin held it at 10.22/1.79 walls — user-caught twice): the ONE real
# floor is the tier wall around the square pocket's CORNERS (its rotated
# diagonal is the widest crossing). The lever needn't out-size the frame's
# contact rib (user's call): the rib exists for MINIMAL thrust contact, so
# a boss ending inside the rib would just end the band early — and at this
# floor it doesn't even do that: ring OD 9.82 vs boss 9.85, still fully
# seated with 0.015 to spare. Corner wall = exactly 1.6.
LEVER_BOSS_OD    = ((PIN_SQ_S + 2.0 * PIN_SQ_LEVER_CLR) * math.sqrt(2.0)
                    + 2.0 * STRUCT_WALL)                   # ≈ 9.85
# Separator tooth PHASE: one catch face lands at the pawl's y-MIDLINE, so
# the tooth-profile cut leaves a 50/50 material split each side of the
# grabbed tooth (the levers' catch-bump math and the separator share this).
RATCHET_PHASE_DEG = math.degrees(math.asin(
    ((RATCHET_LEV_Y0 + RATCHET_LEV_Y1) / 2.0) / (FLOOR_OD / 2.0)))
BRAKE_RUBBER_T   = 3.3                            # TPU brake-pad thickness (the original design)
# Cable floor's LOWEST possible position (for the branch-climb clearance):
# the floor clamp bottomed on the hub base (z 0..CLAMP_H), floor flush on top.
FLOOR_MIN_BOT_Z  = CLAMP_H                        # 4.0 — floor underside at minimum
# Wall windows (open to the wall TOP → no print overhang; bottom ring intact).
# Margins are DERIVED, not blanket: the levers move only in XZ, so the y
# edges need just LEVER_SIDE_CLR; and each window's far z edge (brake SILL,
# ratchet CEILING) needs only to clear the pivot boss where it actually
# enters the wall's RADIAL band (worst at the y=LEVER_Y_IN edge, where the
# wall's outer face reaches deepest toward the boss circle) — the boss
# spins in place, so its swept envelope is itself and 1.0 static clearance
# suffices. The SLIM derived boss no longer reaches the band at all (the
# old Ø12 did) — the crossing term clamps to 0 and the z edges keep just
# the 1.0 about the pivot. The wall's height then follows the ratchet
# ceiling.
_BOSS_WALL_DZ    = math.sqrt(max(
    (LEVER_BOSS_OD / 2.0) ** 2
    - (math.sqrt(WALL_OR ** 2 - LEVER_Y_IN ** 2)
       - LEVER_PIVOT_X) ** 2, 0.0))               # 0 — boss half-height at the wall
                                                  # band's deepest crossing (no crossing)
LEVER_WIN_Y0     = LEVER_Y_IN - LEVER_SIDE_CLR    # 5 — inner y (1.5 clear of the ±3.5 tenon)
LEVER_WIN_Y1     = RATCHET_LEV_Y1 + LEVER_SIDE_CLR   # 17 — outer y
# Ratchet OVER-PULL STOP = the window SILL (user's design): the pawl block's
# bottom edge arcs down-and-out into the wall's radial band as the lever is
# pulled, so the sill's top face is the natural travel stop. The sill is
# DERIVED so the edge lands on it at RATCHET_STOP_DEG, where the pawl has
# ~2.0 radial clearance off the tooth tips (checked at the STOP angle in
# lever_kinematics). The old blanket sill (SEP_Z0 − 1 = 18.5) silently
# jammed the lever at ~7.4° — barely past pawl-clear — user-caught.
# First-contact model (conservative): the full bottom edge (z = SEP_Z0 at
# rest) meeting the wall's INNER face at the block's outer-y (largest y →
# smallest wall x → earliest contact); tooth scallops can only delay it.
RATCHET_STOP_DEG = 15.0                           # pawl↔tip clearance ≈ 2.0 at the stop
_STOP_S  = math.sin(math.radians(RATCHET_STOP_DEG))
_STOP_C  = math.cos(math.radians(RATCHET_STOP_DEG))
_X_STOP  = math.sqrt(WALL_IR ** 2 - RATCHET_LEV_Y1 ** 2)         # 74.39
_DZ_BOT  = SEP_Z0 - RATCHET_PIVOT_Z                              # −15
_DX_STOP = (_X_STOP - LEVER_PIVOT_X + _DZ_BOT * _STOP_S) / _STOP_C
RATCHET_WIN_Z0   = (RATCHET_PIVOT_Z + _DX_STOP * _STOP_S
                    + _DZ_BOT * _STOP_C)          # ≈ 17.16 — sill = the stop face
RATCHET_WIN_TOP  = RATCHET_PIVOT_Z + _BOSS_WALL_DZ + 1.0   # ≈ 40.24 — ratchet ceiling
BRAKE_WIN_Z0     = BRAKE_PIVOT_Z - _BOSS_WALL_DZ - 1.0     # ≈ 14.76 — brake sill
assert BRAKE_WIN_Z0 >= WALL_Z0 + WALL_TOP_BAND - 1e-9, \
    "brake window sill fell below the wall's minimum bottom band"
# The wall's top: an unbroken WALL_TOP_BAND ring above EVERY opening — the
# ratchet window AND the cable exit port (user-caught: deriving from the
# ratchet ceiling alone let the port, which tracks the spool chamber and
# ROSE with the 15 rim while the pivots dropped with the thinner ratchet
# band, break out through the wall top and sever the ring).
WALL_H  = (max(RATCHET_WIN_TOP, CABLE_EXIT_Z1)
           + WALL_TOP_BAND - WALL_Z0)                      # window band height (the ring
                                                           # itself now runs higher, below)
WALL_Z1 = WALL_Z0 + WALL_H                                 # top of the WINDOW band
assert WALL_Z1 - CABLE_EXIT_Z1 >= WALL_TOP_BAND - 1e-9, \
    "cable exit port breaks the wall's top band"
assert WALL_Z1 - RATCHET_WIN_TOP >= WALL_TOP_BAND - 1e-9, \
    "ratchet window breaks the wall's top band"

# ── Wall Z-EXTENSIONS (user, after the pre-wind test print) ──────────────────
# TOP — z retention: the wall used to end mid-air with nothing stopping it
# sliding back UP its channels. The ring now runs to 3.2 shy of the
# frame_top underside (BEAM_Z1) and the T tenons ALONE continue to that
# plane (user: no rim up there): once frame_top rotates on, the tenon tops
# sit JOINT_SEAT_CLR under it — the whole wall STACK is z-locked through
# face-on-face contact. Bare posts 3.2 below an inverted print's bed are
# unprintable (the ring top would print on air), so the wall splits into a
# THIRD piece: wall_collar — everything above the window band (WALL_Z1, a
# plain unbroken ring by the band asserts) — which prints UPRIGHT, where a
# flat-topped ring with four rising tenon posts has zero overhangs.
# Stack: wall_bottom (inverted? no — upright), wall_top (INVERTED, now
# ending flat at WALL_Z1), wall_collar (upright) — all sliding down the
# same beam channels in sequence.
WALL_RING_TOP = BEAM_Z1 - 4 * NOZZLE                     # 52.2 — ring top (collar included)
WALL_TEN_TOP  = BEAM_Z1                           # 55.4 — tenon-post tops
# wall_top ↔ collar split: the SOLID rim (window ceiling → ring top) splits
# EVENLY between the two top pieces (user's call; the collar's bare tenon
# posts above the ring don't count in the split).
WALL_COL_SPLIT = ((WALL_Z1 - WALL_TOP_BAND) + WALL_RING_TOP) / 2.0
assert WALL_COL_SPLIT >= WALL_Z1 - 1e-9, \
    "collar split fell into the window band"
assert WALL_RING_TOP >= WALL_COL_SPLIT + 2 * NOZZLE - 1e-9, \
    "wall collar ring shorter than one tier band"
# +X climb sanity (the old floor-clearance anchor died with the flat tray):
# the trident's 45° climb starts OUTSIDE the wall/cup band (CLIMB_X0), so
# the coil cup can drop past it to the lowered plus; the climb still needs
# a real run before the frame extent to lift the columns off the plate.
assert CLIMB_X0 >= WALL_OR + 0.3 - 1e-9, \
    "+X climb starts inside the wall/cup band"
assert (BEAM_IR + BEAM_SIZE) - CLIMB_X0 >= 4.0 - 1e-9, \
    "+X climb has no run before the frame extent"

# ── DESK/WALL MOUNT — bracket (tenons) ↔ frame_top (mortises) ────────────────
# The mount BRACKET is a flat bar that screws to the mounting surface with two
# of the original design's wood screws (Ø4 shaft, Ø9.3 countersunk head — they fit, reused) and
# carries a row of two HANGING octagon tenons: the cadkit octagon joint
# flipped upside down, which makes BOTH prints easier — the mortise's
# one-nozzle bridge roof becomes a plain cavity floor in frame_top's −Z→+Z
# print (no bridge at all: neck slot, 45° lip undersides, vertical bulb
# walls, 45° closing tapers, flat floor), and the bracket prints FLIPPED
# (+Z→−Z, bar on the bed) with library-native standing tenons.
# frame_top's plus carries the SAME channel pattern along X and along Y
# (mount the bracket in either orientation to pick the install axis). Each
# channel = a retained MORTISE segment + an adjacent open-top ENTRY POCKET:
# offer the assembly up (tenons drop through the pockets), slide it +x (or
# +y) ~21.5 mm until the tenons hit the mortise stops. The cable exit's +45°
# azimuth means working pulls push the frame INTO the stops; the FLOATING
# TPU LOCK TENON (below) positively blocks the back-slide.
WOOD_SCREW_SHAFT_D = 4.0                          # original mount screws, reused
WOOD_SCREW_HEAD_D  = 9.3                          # countersunk-head pocket Ø at the face
WOOD_SCREW_HEAD_H  = 4.0                          # cone depth
MOUNT_R_OUT    = BEAM_IR + BEAM_SIZE              # 87.7 — plus-arm outer radius (the
                                                  # frame's full extent; = frame._R_OUT)
MOUNT_TEN_W    = 8 * NOZZLE                              # octagon width — leaves 1.65 side walls
                                                  # in the 10 rib (≥ 2 beads, asserted)
MOUNT_TEN_L    = 20.0                             # engagement per tenon
MOUNT_CLR      = JOINT_CLR                        # 0.15 — long engagement (tested table)
MOUNT_MORT_L   = MOUNT_TEN_L + 0.5                # retained cavity length
MOUNT_POCKET_L = MOUNT_TEN_L + 2.0                # z-entry pocket (tenon + insert ease)
MOUNT_WALL     = 2 * NOZZLE                              # min wall at the channels' outboard cap
# Channel layout along the +X arm (the Y pattern is this rotated 90°).
# Bounds: outboard cap at BEAM_IR − MOUNT_WALL (the frame_top↔bottom
# ARC-joint ring lives OUTSIDE BEAM_IR, r ≈ 84, in the rib's underside and
# the mount cavities reach z ≈ 56.3 from the top — staying inboard keeps
# solid material between the cavity systems by construction); inboard cap
# clear of the plan-diagonal rod bore (inside |x| ≲ 6.5 within the ±3.35
# channel band). BOTH channels hug the outboard cap as hard as the shared
# slide direction allows (print finding: tenon A's old inboard seat
# [12.35, 32.35] let the bar's +x half lever away from the frame):
#   · channel B's MORTISE is at the cap → tenon B seats at the very edge;
#   · channel A's ENTRY POCKET must sit outboard of its mortise (both
#     tenons travel −x together — a rigid bracket slides one way), so the
#     POCKET hugs the cap and tenon A seats at [33.6, 53.6] — as far out
#     as the geometry admits.
# The screws follow: a Ø9.3 head can't share an x-spot with a hanging
# tenon, so each head lands over its channel's pocket VOID (asserted) —
# screw A centres on the relocated pocket A; screw B keeps its proven
# halfway spot over pocket B.
MOUNT_POCK_A_X0 = (BEAM_IR - MOUNT_WALL) - MOUNT_POCKET_L # 54.1 — pocket hugs the cap
MOUNT_MORT_A_X0 = MOUNT_POCK_A_X0 - MOUNT_MORT_L          # 33.6 — stop face
MOUNT_MORT_B_X0 = -(BEAM_IR - MOUNT_WALL)                 # −76.1 — stop face
MOUNT_POCK_B_X0 = MOUNT_MORT_B_X0 + MOUNT_MORT_L          # −55.6
MOUNT_SCREW_A_X = MOUNT_POCK_A_X0 + MOUNT_POCKET_L / 2.0  # 65.1 — head over pocket A
MOUNT_SCREW_B_X = -MOUNT_R_OUT / 2.0                      # −43.85 — head over pocket B
MOUNT_TEN_A_X0  = MOUNT_MORT_A_X0                 # tenons modelled SEATED (butting
MOUNT_TEN_B_X0  = MOUNT_MORT_B_X0                 #  both stops — equal cavity lengths)
MOUNT_PLATE_T   = 5 * NOZZLE                             # bar thickness, 5 × 0.8 — the bar is
                                                  # sandwiched flat (wood above, rib tops
                                                  # below), so stiffness comes free; the
                                                  # floor is the screw head: the Ø9.3
                                                  # countersink needs its FULL cone depth
                                                  # to seat flush on the underside
MOUNT_PLATE_W   = WOOD_SCREW_HEAD_D + 2 * 0.8     # 10.9 — bar width: the Ø9.3 screw-head
                                                  # pocket + exactly one 0.8 bead each side
                                                  # (user's floor; 10 would leave 0.35)
MOUNT_PLATE_X0  = -MOUNT_R_OUT                    # the bar runs the FULL frame length
MOUNT_PLATE_X1  = MOUNT_R_OUT                     # (user's call), flush with the rib ends
# PLUS shape (user's call): a joinery-free CROSS ARM through the centre,
# perpendicular to the tenon row — no tenons, no screw holes. It rides the
# OTHER pair of ribs, sandwiched flat between the wood and their top faces:
# a wide lever arm against rocking about the slide axis. Full length too —
# its ends host the LOCK (below) over the only channel-free rib real estate.
MOUNT_CROSS_HALF = MOUNT_R_OUT                    # 87.7 — cross-arm half-length
# LOCK — floating TPU tenon on the SECONDARY axis (user's design): matching
# half-depth mortises in the cross arm's UNDERSIDE (both ends — keeps the
# part symmetric) and in each rib's TOP face at its outboard tip (all four
# arms, so both bracket orientations find one), aligned when seated. The
# TPU key slides in axially from the open arm end and spans the joint plane
# half-and-half → shear-blocks the octagon back-slide. Its axis is ⊥ the
# slide axis, so no working load can push it out; line-on-line width + TPU
# print fattening make it a press fit. The key is a plain rectangular bar
# (+ a proud pull tail) — every print orientation is overhang-free. Sits
# outboard of the channels (they cap at 76.1) and 2 mm of rib above the
# r≈84 arc-joint cavities (groove bottom 62.2 vs their ceiling ≈ 59.4).
MOUNT_LOCK_W    = 8 * NOZZLE                             # key width (8 beads)
MOUNT_LOCK_D    = 2 * NOZZLE                             # groove depth EACH side of the joint
                                                  # (was 2.0 — thinned the roof over the
                                                  # frame-top arc mortise to 1.18, user-
                                                  # caught; 1.6 leaves 1.58 — the last
                                                  # 0.02 is the arc cutter's dilation)
                                                  # plane (key = 4.0 tall, half-and-half)
MOUNT_LOCK_L    = 12 * NOZZLE                             # engaged length
MOUNT_LOCK_Y0   = MOUNT_R_OUT - MOUNT_LOCK_L      # 78.1 — groove start (2.0 wall to the
                                                  # channels' 76.1 cap, asserted)
MOUNT_LOCK_TAIL = 3.0                             # pull tail proud of the arm end
assert (TOP_RIB_SIZE - MOUNT_TEN_W) / 2.0 - MOUNT_CLR >= 2 * NOZZLE - 1e-9, \
    "mount mortise side walls fell under 2 beads"
assert MOUNT_PLATE_T >= WOOD_SCREW_HEAD_H - 1e-9, \
    "mount bar thinner than the wood-screw countersink — head would sit proud"
# each screw head must face a pocket VOID (a head over a hanging tenon would
# have to drill through it)
for _sx, _p0 in ((MOUNT_SCREW_A_X, MOUNT_POCK_A_X0), (MOUNT_SCREW_B_X, MOUNT_POCK_B_X0)):
    assert (_p0 + 0.25 <= _sx - WOOD_SCREW_HEAD_D / 2.0
            and _sx + WOOD_SCREW_HEAD_D / 2.0 <= _p0 + MOUNT_POCKET_L - 0.25), \
        "a mount screw head is not fully over its entry-pocket void"
assert MOUNT_LOCK_Y0 - (BEAM_IR - MOUNT_WALL) >= 2 * NOZZLE - 1e-9, \
    "lock groove too close to the mount channels"
# Square-axle packaging (user's rules): the thrust rings stay INSIDE the
# frame's +x extent, and the diamond bores leave ≥ 1.6 of beam to that face.
assert (LEVER_PIVOT_X + BOSS_BORE_ID / 2.0 + BOSS_RING_W
        <= BEAM_IR + BEAM_SIZE + 1e-9), \
    "thrust rings poke past the frame's +x face — shrink PIN_SQ_S"
assert ((BEAM_IR + BEAM_SIZE)
        - (LEVER_PIVOT_X
           + (PIN_SQ_S + 2.0 * PIN_SQ_FRAME_CLR) * math.sqrt(2.0) / 2.0)
        >= 2 * NOZZLE - 1e-9), \
    "under 1.6 of beam between the diamond bore and the frame's +x face"

# ── Consistency guards (fire at import) ──────────────────────────────────────
# The wall's bottom is a literal (defined before the cap/clamp block); verify
# it still reaches the cable floor's top face at the floor's LOWEST position.
assert abs(WALL_Z0 - (FLOOR_MIN_BOT_Z + CAP_H)) < 1e-6, (
    f"WALL_Z0 drifted: wall bottom {WALL_Z0} != lowest floor top "
    f"{FLOOR_MIN_BOT_Z + CAP_H}")
# The +X beam's wall-joint cavity (head ≤ JOINT_WIDTH, dilated JOINT_CLR)
# must leave ≥ 1.6 of beam between its side wall and the blind pin-bore
# floors at y ±PIN_TIP_END_Y (user's rule — quality-tier wall).
assert PIN_TIP_END_Y - (JOINT_WIDTH / 2.0 + JOINT_CLR) >= 2 * NOZZLE - 1e-9, (
    "wall-joint cavity too wide: under 1.6 of beam left beside the pin bores")
