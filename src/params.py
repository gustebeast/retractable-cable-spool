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

from src.dimensions import HUB_OD, SPOOL_H, STRUCT_WALL, FIT_CLR, AXLE_PRINT_D
from cadkit.contact import contact_rib_size
from cadkit.joinery import PrintSpec, joint_clearances, hook_width_min

NOZZLE = 0.8                          # the project prints at a 0.8 nozzle (user's
                                      # call, 2026-07-22; was 0.4) — bead-multiple
                                      # sizing keys off this

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
RATCHET_DEPTH = 2.4                   # tooth tip→valley (also the 45° cone radial
                                      # step): three full 0.8-nozzle perimeters
                                      # (user's call — deeper bite; was 1.6)
RATCHET_TEETH = 60
CONE_H        = RATCHET_DEPTH         # 45° ratchet→brake transition (tracks depth)
# Brake pad ↔ arm joint (cadkit hook, single-flank — the joint face is
# edge-bounded) + the pad's z margins inside the band. The pad is an
# L-SECTION now (user's design): a band-contact SLAB sized by the band's
# margins, plus a taller back FLANGE carrying the hook joint at its QUALITY
# width — the joint no longer sizes the band (an 8.5 band once grew the rim
# to 15 and ate 3.0 of spool-chamber headroom, user-caught). The flange
# nests into a recessed, taller arm end: at that radius (behind the old arm
# face) it clears the coil keep-out above and the cone/teeth below.
PAD_JOINT_CLR        = 0.1            # TPU in PETG — snug
BRAKE_PAD_TOP_MARGIN = 0.8            # contact-pose z margins inside the band
                                      # (0.8 protects cable_clearance — the 2.4
                                      # teeth raised the contact angle)
BRAKE_PAD_BOT_MARGIN = 1.1            # keeps the full-pull drop ≥ 1 off the teeth
PAD_FLANGE_H  = hook_width_min(NOZZLE, PAD_JOINT_CLR, quality=True)
                                      # 6.6 — the flange face: all-1.6 hook
                                      # segments on both halves
PAD_FLANGE_T  = 1.6                   # flange depth — the arm face recesses by
                                      # this so the flange never protrudes past
                                      # the old face radius
BRAKE_H       = 5.0                   # top brake band height (user's call —
                                      # 5 / 5 / 2.4 bands)
RATCHET_H     = 5.0                   # bottom ratchet band height (back to the
                                      # original engagement height)
RIM_H         = RATCHET_H + CONE_H + BRAKE_H      # 12.4 — recovers 2.6 of the
                                                  # spool-chamber headroom
# 45° cable PASS-THROUGH tunnel (ported from the original design): lets
# the working cable cross from the spool chamber (above the separator) to the
# tray chamber (below), connector included. Tilted tangentially, so it prints
# support-free in the separator's flat print; placed between two key slots.
SEP_PASS_W         = 13.0             # square tunnel section (connector passes)
SEP_PASS_ANGLE_DEG = 30.0             # azimuth (mid-way between key slots at 0/60°)
SEP_PASS_WALL      = 1.6              # web left between the tunnel and the bore

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
STATIC_MOVE_CLR = 3.2                             # the project's standard clearance between
                                                  # STATIC and MOVING parts
RIM_TO_WALL   = 3.2                               # radial gap: wall inner face ↔ brake rim
WALL_IR       = RIM_OD / 2 + RIM_TO_WALL          # wall inner radius (keeps the 3.2 rim gap)
WALL_T        = STRUCT_WALL                       # 1.6 — wall ring radial thickness
WALL_OR       = WALL_IR + WALL_T                  # wall outer radius = the joint mating plane
BEAM_IR       = WALL_OR                           # beams sit just outside the separate wall
                                                  # ring (wall OD = beam inner face; the
                                                  # embedded-wall variant was tried and
                                                  # reverted by user call)
WALL_Z0       = 12.0                              # wall/beam bottom (tray side) — reaches
                                                  # the cable floor's TOP at its lowest
                                                  # (clamp bottomed): FLOOR_MIN_BOT_Z +
                                                  # CAP_H (both defined below; asserted at
                                                  # EOF), so the tray stays walled at any
                                                  # clamp setting
WALL_TOP_BAND = 1.6                               # unbroken top ring above every window
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
JOINT_WIDTH    = 5.2                              # available width on the beam face —
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
JOINT_DEPTH    = 2 * 1.6 + JOINT_BACK_CLR         # 3.5 — a 1.6 head bar + a 1.6 MORTISE
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

# ── Frame split: frame_bottom (bottom plus + beams) / frame_top (top plus) ───
# frame_bottom: a plus-shaped spider BELOW the spring housing (same 3.2 mm
# clearance as the top side) + the four vertical beams (carrying the wall
# mortises) rising from it, + octagon TENONS on the beam tops. frame_top: just
# the top plus spider with the matching mortise channels in its underside.
# BOTH parts print −Z→+Z: frame_top gets its mounting hardware on a flat build,
# frame_bottom prints beams-up (the old monolithic frame had to print +Z→−Z to
# put the top spider on the bed). NOTE: the bottom plus blocks the removable
# spring_housing_cap's −Z exit — install the cap before the frame.
HOUSING_TOP_CLR = 3.2                             # top-plus underside ↔ housing top
HOUSING_BOT_CLR = 3.2                             # bottom-plus top ↔ housing/cap bottom
TOP_RIB_SIZE    = 10.0                            # plus-arm section (10 × 10, matches beams)
TOP_RIB_Z0      = SPOOL_H + HOUSING_TOP_CLR       # 54.2 — top plus underside
TOP_RIB_Z1      = TOP_RIB_Z0 + TOP_RIB_SIZE       # 64.2 — top plus top
BOT_RIB_Z1      = -HOUSING_BOT_CLR                # −3.2 — bottom plus top
BOT_RIB_Z0      = BOT_RIB_Z1 - TOP_RIB_SIZE       # −13.2 — bottom plus underside
BEAM_Z0         = BOT_RIB_Z0                      # beams fuse down through the bottom plus
BEAM_Z1         = TOP_RIB_Z0                      # beams end at the top plus underside
# Wall-mortise channel: open at the beam TOP (wall enters there, slides down),
# hard stop JOINT_SEAT_CLR below the seated tenon bottom.
MORTISE_L       = (BEAM_Z1 + 1.0) - (WALL_Z0 - JOINT_SEAT_CLR)   # 41.55 — stop → past beam top

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
TOP_JOINT_W         = 5.0                         # octagon width (radial; leaves a 1.15 mm
                                                  # outer cavity wall in the arm)
TOP_JOINT_SEAT_CLR  = 0.15                        # seating clearance at each stop (arc mm)
TOP_ENTRY_OVER      = 1.0                         # channel overshoot past the CCW entry
                                                  # faces (arc mm)
TOP_WALL_CLEAR      = 0.1                         # tenon radial gap off the wall channel —
                                                  # right at the wall-install limit
TOP_STOP_WALL       = 1.2                         # stop-wall thickness at each arm's CW
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
ROD_D       = 4.0                                 # printed rod Ø
ROD_HOLE_D  = ROD_D + 0.3                         # rod bore (slip fit)
ROD_L       = 20.0                                # rod length (crossing diagonal + grip ends)
ROD_Z       = TOP_RIB_Z0 + TOP_RIB_SIZE / 2.0     # 59.2 — top rod axis (top crossing mid)
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
HEX_DRIVE_AC    = AXLE_PRINT_D                    # 7.95 — hex across-CORNERS
HEX_DRIVE_AF    = HEX_DRIVE_AC * math.cos(math.radians(30.0))   # ≈ 6.89 across-flats
HEX_DRIVE_L     = 15.0                            # male hex length below the frame
HEX_DRIVE_Z0    = BOT_RIB_Z0 - HEX_DRIVE_L        # −33.2 — hex tip
WIND_TOOL_CLR   = 0.35                            # socket clearance (across-corners)
WIND_TOOL_L     = 110.0                           # bar length (two ~50 mm handle arms)
WIND_TOOL_W     = 16.0                            # bar width (≥3.8 wall around the socket)
WIND_TOOL_T     = 5.0                             # handle-arm thickness (prints flat −Z→+Z)
WIND_TOOL_BOSS_OD = 16.0                          # centre boss around the socket
WIND_TOOL_BOSS_H  = HEX_DRIVE_L                   # 15 — socket grips the FULL hex length

# ── Cable-chamber caps — cable_ceiling (+Z) and cable_floor (−Z) ─────────────
# Plain 10 mm cylinders sliding on the hub, one each side of the separator, key-
# slotted so they turn with it. No self-clamp: the wound CABLE stops them moving
# toward the separator; a separate height_clamp locks the other direction.
SEP_Z0        = RIM_SEAT_Z                        # 19.5 — separator bottom
SEP_Z1        = RIM_SEAT_Z + RIM_H                # 31.5 — separator top
CAP_H         = 8.0                               # cap height
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
CAP_SPOKE_W      = 3.2                            # web width = 4 beads at the 0.8 nozzle
# Ring widths — all 0.8-bead multiples, graded by how much backing each ring
# has (user's call): the INNER collar rides the spring-housing hub (fully
# backed → thinnest); the MID band ties the spokes at half-span; the OUTER
# rim is unbacked and carries the retention pressure (thickest).
CAP_RING_W_IN    = 1.6                            # 2 beads
CAP_RING_W_MID   = 3.2                            # 4 beads
CAP_RING_W_OUT   = 4.8                            # 6 beads
CAP_HUB_COLLAR_R = RIM_ID / 2.0 + CAP_RING_W_IN   # ≈ 35.05 — collar around the bore
CAP_MID_R0       = 53.0 - CAP_RING_W_MID / 2.0    # 51.4 — mid band centred on the
CAP_MID_R1       = 53.0 + CAP_RING_W_MID / 2.0    # 54.6   coil span's middle (~53)
CAP_RIM_R0       = CAP_OD / 2.0 - CAP_RING_W_OUT  # 68.1 — rim solid to the OD
CEIL_Z0       = SEP_Z1 + CABLE_GAP                # ceiling a cable-gap above the separator
FLOOR_Z0      = SEP_Z0 - CABLE_GAP - CAP_H        # floor a cable-gap below the separator

# ── height_clamp — a 4 mm C-clamp pinching the hub to lock a cap in Z ────────
CLAMP_H       = 4.0
CLAMP_OD      = HUB_OD + 8.0                      # 74.4 — collar OD (room for the pinch boss)
CLAMP_SLIT_W  = 1.4                               # C-slit gap; the M2 screw closes it onto the hub
CEIL_CLAMP_Z0  = CEIL_Z0 + CAP_H                  # clamp ABOVE the ceiling (locks it down)
FLOOR_CLAMP_Z0 = FLOOR_Z0 - CLAMP_H              # clamp BELOW the floor (locks it up)

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

# ── Source-cable port: HOUSE-shaped hole in the bottom wall half ─────────────
# The stationary source lead leaves the TRAY chamber at −X, offset in −Y just
# far enough that the exiting cable clears the −X beam's flank. HOUSE profile
# (square sides + 45° gable roof) so the opening's ceiling prints support-free
# in wall_bottom's −Z→+Z print. Width matches the separator tunnel (the
# connector must thread through); sill 0.5 under the tray floor's top face.
SOURCE_PORT_W         = SEP_PASS_W                # 13 — connector passes
SOURCE_PORT_CLR       = 3.0                       # cable-path clearance off the beam flank
SOURCE_PORT_ANGLE_DEG = 180.0 + math.degrees(math.asin(
    (BEAM_SIZE / 2 + SOURCE_PORT_CLR + SOURCE_PORT_W / 2) / WALL_IR))    # ≈ 191° (−y side)
SOURCE_PORT_Z0        = WALL_Z0 + WALL_TOP_BAND   # 15.4 — sill leaves a 1.6 band below
                                                  # (same minimum as the top band); sits
                                                  # 0.1 proud of the floor top — negligible
SOURCE_PORT_APEX_Z    = WALL_SPLIT_Z - WALL_TOP_BAND   # 32.0 — gable peak leaves a 1.6
                                                  # band up to the split plane
SOURCE_PORT_RECT_TOP  = SOURCE_PORT_APEX_Z - SOURCE_PORT_W / 2  # 26.6 — 45° roof springs here

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
BRAKE_PIVOT_Z    = SEP_Z0 + RATCHET_H + CONE_H - 5.5   # 20.5 — below the brake band
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
POST_OUT_T       = 5.0                            # side-arm section thickness — HALF the
                                                  # 10 mm centre arm (user's call): the
                                                  # column + 45° diagonal + horizontal top
                                                  # run keep this constant 5×10 section
# axle length: blind-bore floor → column outer face → grip stub.
LEVER_PIN_L      = (RATCHET_LEV_Y1 + LEVER_SIDE_CLR + POST_OUT_T
                    + PIN_GRIP_L - PIN_TIP_END_Y)                # 21.2
LEVER_BOSS_OD    = 12.0                           # pivot BOSS Ø on each lever (the 6 mm
                                                  # handle is narrower than the pocket)
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
# ratchet CEILING) needs only to clear the Ø12 pivot boss where it actually
# enters the wall's RADIAL band (worst at the y=LEVER_Y_IN edge, where the
# wall's outer face reaches deepest into the boss circle) — the boss spins
# in place, so its swept envelope is itself and 1.0 static clearance
# suffices. The wall's own height then follows the ratchet ceiling.
_BOSS_WALL_DZ    = math.sqrt((LEVER_BOSS_OD / 2.0) ** 2
                             - (math.sqrt(WALL_OR ** 2 - LEVER_Y_IN ** 2)
                                - LEVER_PIVOT_X) ** 2)   # 4.74 — boss half-height at the
                                                  # wall band's deepest crossing
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
           + WALL_TOP_BAND - WALL_Z0)                      # ≈ 27.3
WALL_Z1 = WALL_Z0 + WALL_H                                 # ≈ 39.3 — wall top
assert WALL_Z1 - CABLE_EXIT_Z1 >= WALL_TOP_BAND - 1e-9, \
    "cable exit port breaks the wall's top band"
assert WALL_Z1 - RATCHET_WIN_TOP >= WALL_TOP_BAND - 1e-9, \
    "ratchet window breaks the wall's top band"

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
MOUNT_TEN_W    = 6.4                              # octagon width — leaves 1.65 side walls
                                                  # in the 10 rib (≥ 2 beads, asserted)
MOUNT_TEN_L    = 20.0                             # engagement per tenon
MOUNT_CLR      = JOINT_CLR                        # 0.15 — long engagement (tested table)
MOUNT_MORT_L   = MOUNT_TEN_L + 0.5                # retained cavity length
MOUNT_POCKET_L = MOUNT_TEN_L + 2.0                # z-entry pocket (tenon + insert ease)
MOUNT_WALL     = 1.6                              # min wall at the channels' outboard cap
MOUNT_SCREW_X  = MOUNT_R_OUT / 2.0                # 43.85 — screw centres HALFWAY between
                                                  # the axle and the frame edge (user's
                                                  # call), both on the tenon axis
# Channel layout along the +X arm (the Y pattern is this rotated 90°).
# Bounds: outboard cap at BEAM_IR − MOUNT_WALL (the frame_top↔bottom
# ARC-joint ring lives OUTSIDE BEAM_IR, r ≈ 84, in the rib's underside and
# the mount cavities reach z ≈ 56.3 from the top — staying inboard keeps
# solid material between the cavity systems by construction); inboard cap
# clear of the plan-diagonal rod bore (inside |x| ≲ 6.5 within the ±3.35
# channel band). The screws pin the rest: a Ø9.3 head can't share an x-spot
# with a hanging tenon, so pocket A is CENTRED ON screw A (the head faces
# the pocket's void; both screw heads land over pocket voids, asserted) —
# which places tenon A inboard [12.35, 32.35] — and channel B hugs the
# outboard cap, putting its pocket over screw B: tenon B [−76.1, −56.1].
# The pattern stays a TRANSLATION of itself (both tenons travel −x relative
# to the frame while seating — a rigid bracket slides one way).
MOUNT_POCK_A_X0 = MOUNT_SCREW_X - MOUNT_POCKET_L / 2.0    # 32.85
MOUNT_MORT_A_X0 = MOUNT_POCK_A_X0 - MOUNT_MORT_L          # 12.35 — stop face
MOUNT_MORT_B_X0 = -(BEAM_IR - MOUNT_WALL)                 # −76.1 — stop face
MOUNT_POCK_B_X0 = MOUNT_MORT_B_X0 + MOUNT_MORT_L          # −55.6
MOUNT_TEN_A_X0  = MOUNT_MORT_A_X0                 # tenons modelled SEATED (butting
MOUNT_TEN_B_X0  = MOUNT_MORT_B_X0                 #  both stops — equal cavity lengths)
MOUNT_PLATE_T   = 4.0                             # bar thickness, 5 × 0.8 — the bar is
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
MOUNT_LOCK_W    = 6.4                             # key width (8 beads)
MOUNT_LOCK_D    = 2.0                             # groove depth EACH side of the joint
                                                  # plane (key = 4.0 tall, half-and-half)
MOUNT_LOCK_L    = 9.6                             # engaged length
MOUNT_LOCK_Y0   = MOUNT_R_OUT - MOUNT_LOCK_L      # 78.1 — groove start (2.0 wall to the
                                                  # channels' 76.1 cap, asserted)
MOUNT_LOCK_TAIL = 3.0                             # pull tail proud of the arm end
assert (TOP_RIB_SIZE - MOUNT_TEN_W) / 2.0 - MOUNT_CLR >= 1.6 - 1e-9, \
    "mount mortise side walls fell under 2 beads"
assert MOUNT_PLATE_T >= WOOD_SCREW_HEAD_H - 1e-9, \
    "mount bar thinner than the wood-screw countersink — head would sit proud"
# each screw head must face a pocket VOID (a head over a hanging tenon would
# have to drill through it)
for _sx, _p0 in ((MOUNT_SCREW_X, MOUNT_POCK_A_X0), (-MOUNT_SCREW_X, MOUNT_POCK_B_X0)):
    assert (_p0 + 0.25 <= _sx - WOOD_SCREW_HEAD_D / 2.0
            and _sx + WOOD_SCREW_HEAD_D / 2.0 <= _p0 + MOUNT_POCKET_L - 0.25), \
        "a mount screw head is not fully over its entry-pocket void"
assert MOUNT_LOCK_Y0 - (BEAM_IR - MOUNT_WALL) >= 1.6 - 1e-9, \
    "lock groove too close to the mount channels"
# Square-axle packaging (user's rules): the thrust rings stay INSIDE the
# frame's +x extent, and the diamond bores leave ≥ 1.6 of beam to that face.
assert (LEVER_PIVOT_X + BOSS_BORE_ID / 2.0 + BOSS_RING_W
        <= BEAM_IR + BEAM_SIZE + 1e-9), \
    "thrust rings poke past the frame's +x face — shrink PIN_SQ_S"
assert ((BEAM_IR + BEAM_SIZE)
        - (LEVER_PIVOT_X
           + (PIN_SQ_S + 2.0 * PIN_SQ_FRAME_CLR) * math.sqrt(2.0) / 2.0)
        >= 1.6 - 1e-9), \
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
assert PIN_TIP_END_Y - (JOINT_WIDTH / 2.0 + JOINT_CLR) >= 1.6 - 1e-9, (
    "wall-joint cavity too wide: under 1.6 of beam left beside the pin bores")
