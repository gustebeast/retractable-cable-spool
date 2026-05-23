"""Top-level dimensions, material parameters, and fit clearances.

These are the constants every part draws from. Local part-specific
constants (e.g. lever-housing column widths, brake-pin geometry) live in
build.py next to the part they apply to.
"""

import math

# ── Spool body (drum + flanges + hub) ────────────────────────────────────────
DRUM_OD        = 155.0   # cable drum outer diameter (cable-wrap region).
                         # Sized to fit a 3 mm-deep helical cable groove
                         # (Ø6 mm × 7 mm pitch) with ≥1.5 mm wall behind it.
DRUM_WALL      =   4.5   # drum wall thickness → drum ID = 146 mm
DRUM_H         =  37.0   # drum axial extent. Sized for the cassette spring
                         # cavity to be ≥ 33 mm tall (clears the 31.7 mm
                         # Stanley 30-455 cassette assembly with axial slack).

FLANGE_OD      = DRUM_OD   # flange outer diameter — flush with drum (no
                           # outer overhang). Both rims (smooth brake/wheel
                           # outer, toothed ratchet inner) live on a single
                           # flat-bottom annulus connected to the drum body
                           # by one inward 45° chamfer.
FLANGE_H       =   7.0   # flange axial extent at inner edge (over the drum).
                         # = (FLANGE_OD−DRUM_OD)/2 + FLANGE_LIP_T → 45° underside.
FLANGE_LIP_T   =   3.2   # rim thickness: 1.7 mm of clean support material above
                         # the ratchet teeth (which cut RATCHET_DEPTH=1.5 mm up
                         # into the rim from below) before the 45° chamfer starts.

# Universal thin-structural-wall thickness. Used for the hub wall, the
# main spool spokes, the bearing-cap radial spokes, and the top-bearing
# block radial ribs — keeping them all on one number means a single
# stiffness/material knob and one slicer-friendly print width. 1.7 mm =
# ≈4 perimeters at a 0.4 mm nozzle (extrusion width ~0.42–0.45) so these
# features print as solid walls with no infill sliver. The bearing
# press-fit collars are NOT pinned to this — they keep their own thicker
# wall (3 mm) for hoop strength under the press-fit.
STRUCT_WALL    =   1.7

HUB_OD         =  70.4   # hub outer diameter = cavity ID 67 mm + 2×HUB_WALL.
                         # Cavity still clears the 64 mm cassette flange OD by
                         # 1.5 mm per side, with 0.5 mm clearance also at the
                         # cap-stop lip's narrowest point — CAP_STOP_ID
                         # = 65 mm. (Was 73.0 with a 3 mm wall.)
HUB_WALL       = STRUCT_WALL   # hub wall thickness in the spring-cavity region.
                         # Pinned to STRUCT_WALL — see its comment above.

# ── 608 bearing (purchased part) ─────────────────────────────────────────────
BEARING_OD     =  22.0   # 608 bearing outer race diameter
BEARING_W      =   7.0   # 608 bearing axial width
BEARING_CLR    =   0.3   # pocket bore clearance above BEARING_OD (0.15 mm per side)
BEARING_LIP_H  =   1.0   # retention lip axial height
BEARING_LIP_ID =  20.0   # retention lip ID (< BEARING_OD → stops the bearing)

# ── Axle ─────────────────────────────────────────────────────────────────────
AXLE_D         =   8.0   # nominal — 608 bearing bore (the metal bearing's ID)
AXLE_PRINT_D   =   AXLE_D - 0.1   # 7.9 — printed axle Ø, 0.05 mm per side under nominal

# Bearing-retention lip on a printed axle: a short Ø(AXLE_LIP_OD) × AXLE_LIP_H
# shoulder that sits flat against the bearing's INNER race rim. Mirrors the
# main-body pocket's retention lip on the OUTER race: both protrude by
# AXLE_LIP_W = 1 mm past their reference surface. Used by both the main spool
# axle and the back guide axle (same shaft Ø → same lip Ø).
AXLE_LIP_W     =   1.0
AXLE_LIP_OD    =   AXLE_PRINT_D + 2 * AXLE_LIP_W   # 9.7 mm (was 10 — 1.15 mm/side)
AXLE_LIP_H     =   1.0
                                   # printing. Side-printed cylinders grow
                                   # ~0.05 mm per side; this compensates so
                                   # the printed axle fits the bearing.

# ── Lever-housing geometry that sizes the axle extensions ────────────────────
PLATE_T             = 10.0   # housing top/bottom plate thickness.
# Axial clearance between each housing plate and the nearest spool feature.
# The pancake side is now MUCH tighter than the lever side: the printed
# guide wheel rolls on the spool's pancake-side flange top with its
# painted-rubber peak just shy of the plate's inner face, so the gap only
# needs to clear the wheel's bottom (= 0). The 2 mm value lets the plate's
# inner face sit 2 mm above the spool top — the wheel pokes UP into a
# pocket cut into the plate's inner face, leaving 1.5 mm of plate roof
# above the painted wheel as the keep-out from the wood mount.
HOUSING_GAP_LEVER   =  2.0   # lever side (below the spool, z < 0). Mirrors
                             # the pancake-side gap so the lever-side guide
                             # wheel pokes into a pocket cut in the plate
                             # the same way it does on the pancake side.
                             # With _LEVER_PIVOT_OFFSET_FROM_INNER=8.13,
                             # the lever pivot ends up at z=-10.13 (was
                             # -12.5) — the whole lever assembly moves up
                             # 2.37 mm, while the wheel and sandwich slab
                             # geometry stay anchored to the spool flange.
                             # Top of top stop-pin boss now at z=-2 (=
                             # plate top) — no boss material in the
                             # spool clearance gap.
HOUSING_GAP_PANCAKE = 2.5    # pancake side (above the spool, z > SPOOL_H).
                             # Trimmed 3.2 → 2.5 (-0.7 mm) so the wheel's
                             # Ø4.1 axle hole sits well below the plate's
                             # outer face — avoids the hairline section
                             # of material a smaller bump would leave
                             # behind.
                             # The original 3.2 mm gap was sized so the
                             # plate material above the pancake guide-wheel
                             # pocket grew from 0.5 to 1.7 mm, leaving room
                             # for the mount_bracket's 1.7 mm-deep
                             # connecting strip to cross over the wheel
                             # without
                             # leaving a knife-edge in the housing.
# Axle extends past the spool. On the PANCAKE side the axle still goes
# through the full housing plate (gap + plate thickness) so the cross-
# pin sits at plate center with PLATE_T/2 of full-diameter material on
# each side of the pin hole. On the LEVER side the housing plate has
# been trimmed inboard to LEVER_HOUSING_X_TAIL, leaving no plate at
# the axle's X position — there's no housing material to mount or
# pin against, so the axle just ends with a short stub past the
# bearing cap's outer face.
AXLE_EXTRA_LEVER    = 2.0                              # short stub past bearing-cap outer face
AXLE_EXTRA_PANCAKE  = HOUSING_GAP_PANCAKE + PLATE_T    # axle past spool, pancake end

# Axle-to-housing cross-pin Z positions: centered in each housing axle
# column. Shared by the axle (cross-hole through axle) and the housing
# (clearance + insert pilot through column). Defined further down once
# SPOOL_H is available.

# ── Cassette engagement (fin + slot in axle / hub) ───────────────────────────
# A SINGLE fin protrudes radially from the axle on one side. The fin is
# split by a thin radial slot — the slot's INNER wall sits flush with the
# axle surface at the fin's azimuthal center (no inner prong material at
# center; what little material exists away from center comes naturally
# from the axle's curve receding). The OUTER prong (the substantial side
# of the slot) is 1.8 mm thick. The slot opens at the TOP of the fin
# (top disconnected, prongs free) and closes well below it onto a solid
# base for stiffness. Spring leg drops in from above; curl above can't
# pass through the 0.5 mm gap.
#
# Top tapers on BOTH sides of the slot guide the leg into place: the
# outer prong's outer face tapers inward at the top (existing), and
# the slot's inner wall flares INTO the axle at the top to widen the
# slot's mouth on the inner side.
FIN_RADIAL_EXT       = 2.3    # tip at r = 4 + 2.3 = 6.3 mm → diameter
                              # 12.6 mm (0.4 mm radial clearance to the
                              # 13.4 mm cassette central hole)
FIN_AZIMUTHAL_W      = 4.0    # azimuthal (tangential) fin width
FIN_TAPER_H          = FIN_RADIAL_EXT   # 45° tapers at top AND bottom
                              # (rise = run); self-supporting print +
                              # lead-in chamfers
FIN_BOTTOM_SOLID_EXT = 5.0    # extra solid material below the slot,
                              # between slot bottom and bottom taper —
                              # bending stiffness for the prongs
HOOK_SLOT_W          = 0.8    # radial slot width — fits 0.1 mm spring
                              # leg with generous clearance. Slot inner
                              # wall is at x = AXLE_D/2 (axle surface at
                              # fin center); outer wall at x = AXLE_D/2
                              # + HOOK_SLOT_W.
HOOK_SLOT_AXIAL_H    = 15.0   # slot 15 mm tall (matches spring strip)
INNER_FLARE_H        = 1.0    # inner-wall flare at top of slot. Slot
                              # inner wall slopes inward at top — the
                              # flare's cut overshoots into the axle's
                              # x-range but the axle stays solid (the
                              # slot is cut from fin material only,
                              # before the fin is unioned with the
                              # axle). Net visible effect: a small
                              # inward chamfer on the slot mouth.
OUTER_FLARE_H        = 1.1    # outer-wall flare at top of slot. Slot
                              # outer wall slopes OUTWARD (into the
                              # outer prong) at top — together with the
                              # inner flare, this creates a V-funnel
                              # that guides the spring leg DOWN INTO
                              # the slot during install. Sized so the
                              # outer prong is 0.4 mm thick at the very
                              # top (FIN_RADIAL_EXT − HOOK_SLOT_W −
                              # OUTER_FLARE_H = 2.3 − 0.8 − 1.1 = 0.4).
FIN_TOTAL_H          = (FIN_TAPER_H + FIN_BOTTOM_SOLID_EXT
                        + HOOK_SLOT_AXIAL_H)   # 22.3 mm — no top taper
                              # on the fin's outer face; all the "lead-
                              # in" geometry lives in the slot flares.

SPOKE_COUNT    =   6     # radial ribs (hub ↔ drum ↔ both flange rings).
                         # 6 (up from 4) shortens the unsupported span under
                         # the widened top flange between spokes.
SPOKE_W        = STRUCT_WALL  # spoke tangential width — pinned to STRUCT_WALL
                              # so all thin structural walls in the spool
                              # share one knob. See STRUCT_WALL above.

# ── Anti-rotation keys (cap → main_body, pancake_spool → cap tongue) ─────────
# Same key system used wherever a cylindrical part needs to lock rotationally
# inside a host cylinder: SPOKE_COUNT radial tongues on the inner part, matching
# grooves on the host. Groove oversized by FIT_CLR per side tangentially;
# radial tip clearance is supplied by the surrounding seat fit (tongue at
# inner radius, groove at outer radius — already FIT_CLR apart).
KEY_W      = 2.0    # tangential width of tongue
KEY_DEPTH  = 1.0    # radial protrusion of tongue
KEY_ANGLES = [i * 360.0 / SPOKE_COUNT for i in range(SPOKE_COUNT)]
# Groove sized by the shared FIT_CLR (defined below): 2*FIT_CLR tangentially
# (bilateral) and FIT_CLR radially at the tip.

# ── Top-flange widening + ratchet teeth ──────────────────────────────────────
# Top flange's flat top now spans FLANGE_INNER_ID → FLANGE_OD instead of
# FLANGE_ID → FLANGE_OD. Inner half carries ratchet teeth (shorter load-arm
# from the pivot → lower MA → less handle travel to disengage); outer half
# is a smooth surface for a rubber brake pad (larger radius → slightly more
# braking torque per normal force).
FLANGE_INNER_EXT = 14.0  # radial width of the lever-side rim, measured
                         # inward from DRUM_OD/2. Rim is OD=DRUM_OD,
                         # ID=DRUM_OD−2*FLANGE_INNER_EXT, FLANGE_LIP_T thick.
                         # Inner half (7 mm) carries the ratchet teeth,
                         # outer half (7 mm) is the smooth brake/wheel surface.
                         # The 45° chamfer above the rim spans
                         # (DRUM_ID−FLANGE_INNER_ID)/2 axially before
                         # meeting the drum's inner cavity wall.
FLANGE_INNER_LIP_H = 2.0 # vertical inner lip height (analogous to the
                         # outer lip via FLANGE_LIP_T). Creates 2 mm of
                         # straight rectangular material below the tooth
                         # valleys, eliminating the knife-edge that would
                         # otherwise form where the tooth inner corners
                         # meet the 45° slope.

RATCHET_TEETH  = 30      # 12° pitch — smooth hand feel, reliable pawl drop
RATCHET_DEPTH  = 1.5     # tooth depth (mm) into the flange top surface

# Angular offset applied to the tooth pattern. Chosen so a tooth boundary
# (the HIGH→LOW step) lands at the angular midpoint of the pawl footprint,
# yielding a symmetric HIGH/LOW split across the pawl face. The offset is
# computed dynamically from the lever's y-range and the pawl footprint's
# r-range so that moving the ratchet lever automatically re-centers the
# boundary. Assigned in build.py once all lever + pawl constants are
# defined; see the `RATCHET_TOOTH_OFFSET_DEG = ...` block near
# PAWL_BRAKE_GAP.
RATCHET_TOOTH_OFFSET_DEG = 0.0    # placeholder — real value computed below

# ── Shared M2 cap-screw / heat-set-insert constants ──────────────────────────
# Used everywhere the spool consumes an M2 cap screw (lever pivots, axle
# cross-pins, axle spring engagement). Keep these as the SINGLE source of
# truth — every new feature that drills, recesses, or threads M2 should
# reference these instead of redefining its own copy.
M2_SHAFT_CLR_D     = 2.4     # 2 mm thread + 0.20 mm clearance per side (was 2.3 — too
                             # tight; the guide-wheel axle hole stays at its own
                             # GUIDE_AXLE_SHAFT_D = 2.2 mm tight-thread fit)
M2_HEAD_RECESS_D   = 4.1     # M2 cap head ≈ 3.8 mm OD + 0.15 mm clearance per side
M2_HEAD_RECESS_H   = 2.0     # cap-head counterbore depth
M2_INSERT_PILOT_D  = 3.3     # McMaster 94459A110 heat-set insert pilot
M2_INSERT_DEPTH    = 3.5     # 2.5 mm insert + 1 mm margin

AXLE_CROSS_HOLE_D = M2_SHAFT_CLR_D    # M2 clearance through the axle.

# Standard "extend past a face" margin for boolean cut/union operations.
# Applied to through-cuts (workplane offset by -BOOL_OVERSHOOT, extrude
# by total_depth + 2*BOOL_OVERSHOOT) and to features that need a clean
# break from the host face (e.g. clip boxes, recess overruns).
BOOL_OVERSHOOT     =   0.5

# ── Bearing cap → main body fit ──────────────────────────────────────────────
# The cap is a small cylinder that slips into the top of the main body
# cavity. A 1 mm-per-side inward lip just below the cap's seat stops it
# from falling through. Slip fit (not grippy) so it can be removed for
# service; in operation the top bearing's axial load is upward (bearing
# press-fit into cap), and gravity keeps the cap seated.
FIT_CLR        =   0.15  # shared per-side slip-fit clearance for hand-assembled
                         # cylindrical/rectangular fits (cap seat, anti-rotation
                         # keys, etc.). Single source of truth for fitted parts.
CAP_STOP_LIP_H =   1.0   # axial height of the cap-stop lip in main body.
                         # Must equal CAP_STOP_INSET so the lip's underside
                         # prints as a 45° self-supporting cone (rise = run).
                         # The same 45° line also forms the roof of the
                         # spring-tab slot in the hub wall, so the slot
                         # doesn't need its own chamfer.
CAP_STOP_INSET =   1.0   # radial inset of cap-stop lip (per side)

# ── Derived ──────────────────────────────────────────────────────────────────
DRUM_ID        = DRUM_OD - 2 * DRUM_WALL                   # 146 mm
FLANGE_ID      = DRUM_ID                                   # 146 mm (bottom flange ID)
FLANGE_INNER_ID = DRUM_OD - 2 * FLANGE_INNER_EXT           # 127 mm (lever rim ID; pancake top-rim ID derives from band width)
FLANGE_RIM_MID_R = (FLANGE_INNER_ID + DRUM_OD) / 4         # 70.5 — boundary between ratchet (inner) and brake (outer) bands
HUB_CAVITY_D   = HUB_OD - 2 * HUB_WALL                     # 34 mm  — spring cavity ID
SPOOL_H        = 2 * FLANGE_H + DRUM_H                     # overall spool height
AXLE_H         = SPOOL_H + AXLE_EXTRA_LEVER + AXLE_EXTRA_PANCAKE  # 64 mm
BEARING_BORE   = BEARING_OD + BEARING_CLR                  # 22.3 mm
TOP_BEARING_BORE = BEARING_BORE                            # standardized — same as bottom
                                                            # (the 0.15 mm/side standard is loose
                                                            # enough that the top cap drops in
                                                            # by hand even with the cone funnel
                                                            # + tongue features attached)

DRUM_BOTTOM_Z  = FLANGE_H                                  #  7 mm
DRUM_TOP_Z     = SPOOL_H - FLANGE_H                        # 31 mm

# Cap geometry
CAP_H          = BEARING_W + BEARING_LIP_H                 #  8 mm — lip (1) + pocket (7)
CAP_OD         = HUB_CAVITY_D - 2 * FIT_CLR                # 33.8 mm (slip fit in 34 mm cavity)
CAP_STOP_ID    = HUB_CAVITY_D - 2 * CAP_STOP_INSET         # 32 mm — cavity ID at the stop lip

# Main body cavity z-map (from top down):
#   z = SPOOL_H             (38)   top face of main body
#   z = SPOOL_H − CAP_H     (30)   cap bottom rests here (on top of the stop lip)
#   z = 30 − CAP_STOP_LIP_H (29.5) cavity resumes full 34 mm ID
#   z = 8                          bottom of spring cavity (= top of bottom lip)
#   z = 7                          bottom of bottom bearing lip
#   z = 0                          bottom of main body
PANCAKE_CAP_SEAT_Z0 = SPOOL_H - CAP_H                          # 43 — bearing-pocket region starts (was cap-seat top)
LEVER_CAP_SEAT_Z0   = 0                                        # bottom cap seat starts
LEVER_CAP_SEAT_Z1   = LEVER_CAP_SEAT_Z0 + CAP_H                # 8 — bottom cap seat ends
LEVER_STOP_LIP_Z0   = LEVER_CAP_SEAT_Z1                        # 8 — bottom cap-stop lip starts
LEVER_STOP_LIP_Z1   = LEVER_STOP_LIP_Z0 + CAP_STOP_LIP_H       # 9 — bottom cap-stop lip ends
CAVITY_Z0           = LEVER_STOP_LIP_Z1                        # 9  — spring cavity starts (above bottom stop lip)
CAVITY_Z1           = PANCAKE_CAP_SEAT_Z0                      # 43 — spring cavity ends, flush with bearing-pocket bottom
                                                               #      (the formerly-separate top cap-stop lip is gone now
                                                               #      that the bearing pocket is fused into the spool)

# Slot is positioned so its center aligns with cassette mid-z. Fin top
# = slot top (no top taper on the fin); fin extends downward by the
# bottom solid extension + the bottom 45° taper.
SLOT_Z_MID = (CAVITY_Z0 + CAVITY_Z1) / 2              # 25.5 — cavity midpoint
SLOT_Z_TOP = SLOT_Z_MID + HOOK_SLOT_AXIAL_H / 2       # 33   — top of slot
SLOT_Z_BOT = SLOT_Z_MID - HOOK_SLOT_AXIAL_H / 2       # 18   — bottom of slot
FIN_Z1     = SLOT_Z_TOP                               # 33   — top of fin = top of slot
FIN_Z0     = FIN_Z1 - FIN_TOTAL_H                     # 10.7 — bottom of fin
FIN_CYL_Z0 = FIN_Z0 + FIN_TAPER_H                     # 13   — top of bottom taper

# Cross-pin Z (absolute), centered in the pancake housing plate. The
# lever-side plate was trimmed inboard so there's no lever cross-pin.
PANCAKE_CROSS_PIN_Z = SPOOL_H + HOUSING_GAP_PANCAKE + PLATE_T / 2 # pancake-side (Z > SPOOL_H)


# ── Sanity checks (catch silent miscoordinations early) ──────────────────────
# These fire at module import. Each one represents an invariant that other
# code silently depends on; a violation usually means someone tightened a
# clearance past zero or shrunk one feature past another.
assert AXLE_PRINT_D < AXLE_D, \
    "printed axle must be undersized vs the metal bearing bore"
assert BEARING_BORE > BEARING_OD, \
    "bearing pocket must be larger than bearing OD (need clearance)"
assert TOP_BEARING_BORE >= BEARING_BORE, \
    "top-bearing pocket must be at least as loose as the standard pocket"
assert BEARING_LIP_ID < BEARING_OD, \
    "bearing retention lip must overlap the bearing's outer-race face"
assert CAP_OD < HUB_CAVITY_D, \
    "cap must fit inside hub cavity with clearance"
assert CAP_STOP_ID < HUB_CAVITY_D, \
    "cap-stop lip ID must be smaller than the cavity (else nothing stops the cap)"
assert CAP_STOP_ID > BEARING_OD, \
    "cap-stop lip ID must be larger than the bearing OD (else the bearing can't pass through)"
assert FLANGE_INNER_ID < DRUM_ID, \
    "flange inner ID must be smaller than drum ID (else there's no flange overhang)"
assert FLANGE_OD >= DRUM_OD, \
    "flange OD cannot be smaller than drum OD"
assert HUB_CAVITY_D > BEARING_BORE, \
    "hub cavity must be larger than the bearing pocket so the bearing fits through"
assert CAP_STOP_LIP_H == CAP_STOP_INSET, \
    "cap-stop lip must be 45° (rise = run) for self-supporting print"
assert SPOKE_COUNT >= 4, \
    "need at least 4 spokes for structural integrity"
assert FIT_CLR > 0, "fit clearance must be positive"
