"""
Retractable Cable Spool

Outputs:
  spool_main_body.step   — full spool: both flanges, drum (with ratchet
                           teeth + brake ring on the top face), spokes,
                           hub with bottom bearing pocket + straight
                           spring cavity + cap-retention lip + top cap
                           seat.
  bearing_cap.step       — small cylindrical insert. Slides into the top
                           of the main body cavity, stopped by the 1 mm
                           inset lip. Houses the top bearing + its own
                           retention lip.
  axle.step              — fixed centre shaft with integrated 23 mm
                           spring stub and 0.5 mm housing-seat ledge.
  lever_housing.step     — U-bracket keyed to the axle's D-flats; carries
                           both levers and all four stop pins (except the
                           brake-side housing pin, which is separate —
                           see below).
  ratchet_lever.step     — class-1 lever, pivot at x=76. Pawl end drops
                           into the ratchet teeth to hold the spool;
                           handle end is pushed down to release.
  brake_lever.step       — class-2 lever, pivot at x=55. Handle end is
                           pushed down to press the rubber brake pad
                           against the brake ring (drag-only, no catch).
  brake_housing_pin.step — separately-printed stop pin with a D-flat
                           key. Glued into the matching blind hole on
                           the brake side of the housing after printing
                           (avoids a support tower under the column in
                           the housing's upside-down print orientation).
  ratchet_spring.step    — DUMMY viz of the purchased torsion spring
                           (coil + radial legs passing through the
                           stop-pin through-holes). Not a printed part.
  brake_spring.step      — DUMMY viz, same as above, for the brake pivot.
  assembly.step          — all of the above positioned in their as-built
                           relationship, for dimensional verification
                           only (individual parts are still exported
                           separately).

Purchased parts (two-warehouse sourcing plan):

  ── Lee Spring (leespring.com) ────────────────────────────────────────
  1x Constant-force spring — Lee Spring LCF 250 06 050S
      0.7 lbf · 12.7 mm strip x 0.15 mm x 762 mm · 20.32 mm natural ID ·
      25,000-cycle life.
      https://www.leespring.com/product/constant-force-lcf25006050s-stainless-steel
  Torsion springs — one right-wound and one left-wound. The ratchet
      and brake levers need restoring torque in opposite angular
      directions (ratchet biases pins APART, brake biases pins
      TOGETHER), so they get opposite-handed coils. Both SKUs share
      identical mechanical specs: music wire 0.51 mm, coil OD 4.50 mm,
      rod 3.00 mm (= LEVER_RIM_OD), body length 2.49 mm, 4.25 total
      coils, leg length 19.99 mm (trim during install to reach the
      stop-pin through-holes), max torque 17.96 N·mm at 86° deflection,
      90° free position between legs. The 01-variant (86° deflection
      budget) is chosen over the stiffer 07-variant (45° budget):
      our rest-state preload deflects the coil ~54° (ratchet) and ~66°
      (brake), which the 07 can't handle without overstressing the
      wire. Each coil slips over the lever rim cylinder and sits
      centered in the 3 mm lever/housing gap; each leg passes through
      the 0.6 mm diametral through-hole in its stop pin.
  1x Torsion spring — ratchet pivot, RIGHT-wound. Lee Spring
      LTMR050E 01 M.
      https://www.leespring.com/product/torsion-spring-ltmr050e01m-music-wire
  1x Torsion spring — brake pivot, LEFT-wound. Lee Spring
      LTML050E 01 M.
      https://www.leespring.com/product/torsion-spring-ltml050e01m-music-wire

  ── McMaster-Carr (mcmaster.com) ──────────────────────────────────────
  2x 608 bearing — 8 mm bore, 22 mm OD, 7 mm wide. Standard skate/roller
      bearing; any ZZ or RS version works.
  4x McMaster 94459A110 — M2 x 0.4 mm heat-set brass threaded inserts
      for plastic. Inner bore 3.1 mm, toothed OD 3.6 mm, 2.5 mm
      installed length, 3.3 mm max pilot hole. Install with a
      soldering iron at ~260 °C (see Assembly step 7): any conical
      tip ≤2 mm at the point works — rest it in the insert's bore,
      press gently as the surrounding plastic softens, tip comes away
      clean when the insert is flush. Two go into the housing pivot
      columns (one in the -y face at x=76 for the ratchet pivot,
      one in the +y face at x=55 for the brake pivot); two go into
      the top and bottom ends of the axle.
      https://www.mcmaster.com/94459A110/
  2x McMaster 91292A013 — M2 x 0.4 mm x 20 mm socket-head cap screw,
      18-8 stainless. Lever pivots: thread into the housing pivot-
      column inserts and clamp each lever axially against its rim
      cylinder. Tighten snug; lever rotates freely.
      https://www.mcmaster.com/91292A013/
  2x McMaster 92855A839 — M2 x 0.4 mm x 8 mm low-profile socket-head
      cap screw, 18-8 stainless, hex drive. Housing-to-axle retention:
      one through each housing axle pad, threading into the axle-end
      inserts. Keeps the snap-fit housing from springing open.
      https://www.mcmaster.com/92855A839/
  1x McMaster 86495K36 — High-strength oil-resistant Buna-N rubber
      sheet, 12" x 24", 3 mm thick, 70A (Hard) durometer. Tensile
      1500 psi, Mil-R-3065/ASTM D2000. One sheet is WAY more than
      needed; cut a ~7 x 13 mm pad (bounded by arcs at r=73 and
      r=80) and bond to the underside of the printed brake-pad block
      with your choice of adhesive (contact cement or thin CA both
      work on Buna-N). 3 mm matches BRAKE_RUBBER_T; kinematics
      auto-adjust if you change it.
      https://www.mcmaster.com/86495K36/
  Super glue (cyanoacrylate) — a few drops to retain brake_housing_pin
      in its keyed hole. Any brand; McMaster also stocks it if you
      want it on the same PO.
  2x McMaster 90915A641 — 10-24 wood-screw-to-stud, 18-8 stainless,
      #10 screw end drives into the wood, 1" long. Mounting hardware
      that penetrates only the wood (no pass-through to the back
      face). Drive these into the mounting surface on the diagonal
      mount pattern; the threaded stud ends stick out to receive the
      housing.
      https://www.mcmaster.com/90915A641/
  2x McMaster 90389A112 — 10-24 flange nut, 316 stainless (flange OD
      12.7 mm, hex 3/8" across flats, 0.219" total height). Tightens
      on the +x face of the spine to clamp the housing against the
      mounting board. Flange spreads clamping force across the 2 mm
      PA6-GF spine so it won't crush.
      https://www.mcmaster.com/90389A112/

Material: PA6-GF  (Support Material: ASA only where unavoidable — the 45°
sloped flanges and 45° stub chamfer keep the spool self-supporting; the
housing's upside-down print orientation + separately-printed brake
housing pin + 45° cones on all blind-hole ends keep the housing self-
supporting too.)

Assembly:
  1. Press bottom bearing into main body — seats against the 20 mm
     retention lip behind the pocket.
  2. Drop the axle through the open top of the main body. Shaft threads
     through the bottom bearing; stub settles into the spring cavity.
  3. Install the constant-force spring. The coil's natural ID (20.32 mm)
     is smaller than the stub OD (23 mm), so it has to be stretched on:
     open the coil slightly, slip it over the stub, and let it snap down
     onto the stub surface. Radial clamp force from the stretch plus
     capstan friction across the wraps anchors the inner end — no
     mechanical attachment. Then bend the outer end of the strip at 90°
     into a short radial tab and drop it into the hub wall slot from
     above (the slot's open roof clears the bent section).
  4. Slide the bearing cap into the top of the main body cavity. Bottoms
     out on the 1 mm inset lip (slip fit — easily removable for service).
  5. Press top bearing into the cap — seats against the cap's own
     internal 20 mm retention lip.
  6. Heat-set M2 inserts into both ends of the axle (3.2 mm pilots,
     insert flush with the axle tip on each end).
  7. Heat-set M2 inserts into the lever-pivot faces of the housing
     (3.2 mm pilots): -y face at x=76 (ratchet pivot), +y face at
     x=55 (brake pivot).
  8. Install brake_housing_pin: put a drop of super glue in the keyed
     blind hole on the +y side of the brake pivot column. Orient the
     pin so its D-flat matches the hole's flat (this automatically
     points the pin's diametral through-hole radially outward, which
     is what the spring leg expects). Lightly tap the pin home until
     the cone bottoms out.
  9. Install the housing onto the assembled spool as a snap-fit clip:
       a. Orient so the axle's D-flats (-y side) match the housing's
          D-holes.
       b. Flex the housing's plate tails apart by ~15-20 mm (bending
          the plates at the plate-spine junctions; the 2 mm PA6-GF arm
          is elastic to at least 30 MPa peak stress at this deflection).
       c. Slide the housing in from the tail side so the axle enters
          between the plates; keep the tails spread until the spine
          sits at its final x position (10 mm past the flange OD).
       d. Release. The top plate drops onto the axle's 0.5 mm ledge
          shoulder; the bottom plate rides its own D-flat.
 10. Thread an M2 cap screw (~8 mm) into each axle end, through each
     housing axle pad. Retains the housing vertically and prevents the
     snap-fit clip from opening during service.
 11. For each lever (ratchet, brake):
       a. Slip the torsion spring coil over the lever's rim cylinder.
       b. Offer the lever up to its pivot, slipping one spring leg
          through the housing stop pin's diametral through-hole before
          the lever face meets the housing.
       c. Rotate the lever so the other spring leg threads through the
          lever stop pin's through-hole, pre-loading the spring.
       d. Thread an M2 cap screw (~20 mm) through the lever's pivot
          hole into the housing-side insert; tighten snugly. The lever
          should rotate freely; the rim cylinder bears against the
          housing face across the 1 mm air gap.
 12. Cut a ~7 x 13 mm piece from the Buna-N sheet (86495K36) to the
     brake contact footprint (bounded by arcs at r=73 and r=80) and
     bond it to the underside of the brake lever's pad block with
     contact cement or thin CA glue.

Build:  py -3.12 spool_flange.py
"""

import math
import cadquery as cq

# ── Parameters (mm) ─────────────────────────────────────────────────────────
DRUM_OD        = 150.0   # cable drum outer diameter
DRUM_WALL      =   2.0   # drum wall thickness → drum ID = 146 mm
DRUM_H         =  24.0   # drum axial extent (cable-winding region, between flange roots)

FLANGE_OD      = 160.0   # flange outer diameter
FLANGE_H       =   7.0   # flange axial extent at inner edge (over the drum).
                         # = (FLANGE_OD−DRUM_OD)/2 + FLANGE_LIP_T → 45° underside.
FLANGE_LIP_T   =   2.0   # flange thickness at outer edge (cable retention lip)

HUB_OD         =  40.0   # hub outer diameter
HUB_WALL       =   3.0   # hub wall thickness in the spring-cavity region

BEARING_OD     =  22.0   # 608 bearing outer race diameter
BEARING_W      =   7.0   # 608 bearing axial width
BEARING_CLR    =   0.1   # pocket bore clearance above BEARING_OD for press-fit
BEARING_LIP_H  =   1.0   # retention lip axial height
BEARING_LIP_ID =  20.0   # retention lip ID (< BEARING_OD → stops the bearing)

AXLE_D         =   8.0   # axle shaft diameter (= 608 bearing bore)

# Lever-housing geometry that sizes the axle extensions
PLATE_T             =  2.0   # housing top/bottom plate thickness
HOUSING_BEARING_GAP = 10.0   # axial clearance between each plate and the
                             # nearest bearing, so the spool's rotating hub face
                             # can't rub against the stationary housing
AXLE_EXTRA     = HOUSING_BEARING_GAP + PLATE_T   # 12 — axle past spool each end

SPRING_STUB_OD =  23.0   # Matches Lee Spring's recommended drum diameter
                         # for the LCF 250 06 050S (22.86 mm spec; rounded
                         # to 23 so it prints clean). Keeping the stub at
                         # spec preserves the rated 0.7 lbf force and
                         # 25,000-cycle life; running on an oversized
                         # drum increases both force and fatigue stress.
                         # 23 > 20.32 natural ID still gives ~1.13x stretch
                         # so the coil grips immediately on install.

SPOKE_COUNT    =   6     # radial ribs (hub ↔ drum ↔ both flange rings).
                         # 6 (up from 4) shortens the unsupported span under
                         # the widened top flange between spokes.
SPOKE_W        =   2.0   # spoke tangential width

# Top flange widening (inner 45° slope). The top flange's flat top now spans
# FLANGE_INNER_ID → FLANGE_OD instead of FLANGE_ID → FLANGE_OD. Inner half
# carries ratchet teeth (shorter load-arm from the pivot → lower MA → less
# handle travel to disengage); outer half is a smooth surface for a rubber
# brake pad (larger radius → slightly more braking torque per normal force).
FLANGE_INNER_EXT = 7.0   # radial extension inward. Full 7 mm inward
                         # extension → tooth ring is r=66..73 (7 mm wide).
                         # The inner 45° slope runs the full 7 mm (rise=run)
                         # for self-supporting print, but its top is now
                         # FLANGE_INNER_LIP_H below the flange top — so the
                         # slope bottom sits 2 mm below DRUM_TOP_Z. The
                         # resulting underside at r=75..80 overlaps the
                         # drum top and prints as a short supported bridge.
FLANGE_INNER_LIP_H = 2.0 # vertical inner lip height (analogous to the
                         # outer lip via FLANGE_LIP_T). Creates 2 mm of
                         # straight rectangular material below the tooth
                         # valleys, eliminating the knife-edge that would
                         # otherwise form where the tooth inner corners
                         # meet the 45° slope.

# Ratchet teeth (cut into outer half of the top flange's flat top)
RATCHET_TEETH  = 30      # 12° pitch — smooth hand feel, reliable pawl drop
RATCHET_DEPTH  = 1.5     # tooth depth (mm) into the flange top surface

# Angular offset applied to the tooth pattern. Chosen so a tooth boundary
# (the HIGH→LOW step) lands at the angular midpoint of the pawl footprint,
# yielding a symmetric HIGH/LOW split across the pawl face. The offset is
# computed dynamically from the lever's y-range and the pawl footprint's
# r-range so that moving the ratchet lever automatically re-centers the
# boundary. Assigned below, once all lever + pawl constants are defined;
# see the `RATCHET_TOOTH_OFFSET_DEG = ...` block near PAWL_BRAKE_GAP.
RATCHET_TOOTH_OFFSET_DEG = 0.0    # placeholder — real value computed below

# Axle D-flats on BOTH extensions, so the housing keys the axle rotationally
# at both ends. Flats are on the +y side at both ends (same orientation), so
# the two housing arm holes are identical.
AXLE_FLAT_DEPTH  = 1.0   # radial cut depth of the D-flat
AXLE_LEDGE_D     = 7.0   # top 2 mm of the axle is turned down to this OD, so the
                         # top plate drops over and bottoms out on the 0.5 mm
                         # shoulder. Picked to leave enough wall around the M3
                         # insert pilot in the reduced section (1.2 mm).
AXLE_LEDGE_H     = PLATE_T   # 2 — reduced section exactly fills the plate

# M2 heat-set insert in the axle top + M2 cap screw through the housing's
# axle pad retains the housing vertically. M2 (not M2.5 or M3) because the
# thin reduced section only has 1.9 mm of wall around a 3.2 mm pilot —
# larger pilots leave the wall too thin to heat-set reliably. M2's pull-out
# force (~300 N) massively exceeds any axial load the housing will see.
AXLE_INSERT_PILOT_D = 3.2    # Ruthex RX-M2x4 pilot hole
AXLE_INSERT_DEPTH   = 5.0    # 4 mm insert + 1 mm margin
AXLE_SCREW_CLR_D    = 2.3    # M2 clearance through the housing's axle pad

# Bearing-cap joint: the cap is a small cylinder that slips into the top of the
# main body cavity. A 1 mm-per-side inward lip just below the cap's seat stops
# it from falling through. Slip fit (not grippy) so it can be removed for
# service; in operation the top bearing's axial load is upward (bearing press-
# fit into cap), and gravity keeps the cap seated.
CAP_SEAT_CLR   =   0.1   # radial slip-fit clearance between cap OD and cavity ID
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
FLANGE_INNER_ID = FLANGE_ID - 2 * FLANGE_INNER_EXT         # 132 mm (top flange only — inner edge)
HUB_CAVITY_D   = HUB_OD - 2 * HUB_WALL                     # 34 mm  — spring cavity ID
SPOOL_H        = 2 * FLANGE_H + DRUM_H                     # 38 mm  — overall spool height
AXLE_H         = SPOOL_H + 2 * AXLE_EXTRA                  # 58 mm
BEARING_BORE   = BEARING_OD + BEARING_CLR                  # 22.1 mm

DRUM_BOTTOM_Z  = FLANGE_H                                  #  7 mm
DRUM_TOP_Z     = SPOOL_H - FLANGE_H                        # 31 mm

# Cap geometry
CAP_H          = BEARING_W + BEARING_LIP_H                 #  8 mm — lip (1) + pocket (7)
CAP_OD         = HUB_CAVITY_D - 2 * CAP_SEAT_CLR           # 33.8 mm (slip fit in 34 mm cavity)
CAP_STOP_ID    = HUB_CAVITY_D - 2 * CAP_STOP_INSET         # 32 mm — cavity ID at the stop lip

# Main body cavity z-map (from top down):
#   z = SPOOL_H             (38)   top face of main body
#   z = SPOOL_H − CAP_H     (30)   cap bottom rests here (on top of the stop lip)
#   z = 30 − CAP_STOP_LIP_H (29.5) cavity resumes full 34 mm ID
#   z = 8                          bottom of spring cavity (= top of bottom lip)
#   z = 7                          bottom of bottom bearing lip
#   z = 0                          bottom of main body
CAP_SEAT_Z0    = SPOOL_H - CAP_H                           # 30 — cap seat starts
STOP_LIP_Z0    = CAP_SEAT_Z0 - CAP_STOP_LIP_H              # 29 — stop lip starts
CAVITY_Z0      = BEARING_W + BEARING_LIP_H                 #  8 — spring cavity starts
CAVITY_Z1      = STOP_LIP_Z0                               # 29 — spring cavity ends

# Spring stub (on axle)
STUB_BASE_Z    = CAVITY_Z0                                 #  8 — chamfer begins here
STUB_CHAMFER_H = (SPRING_STUB_OD - AXLE_D) / 2             # 8.5 — 45° self-supporting
STUB_CYL_Z0    = STUB_BASE_Z + STUB_CHAMFER_H              # 15.5
STUB_TOP_Z     = CAVITY_Z1                                 # 29
SPRING_Z_MID   = (STUB_CYL_Z0 + STUB_TOP_Z) / 2            # 22.25

# ── Helpers ─────────────────────────────────────────────────────────────────

def cyl(d, h, z=0.0):
    """Solid cylinder, diameter d, height h, base at z."""
    return cq.Workplane("XY").workplane(offset=z).circle(d / 2).extrude(h)

def cone_solid(d_bottom, d_top, h, z_base):
    """Solid truncated cone: d_bottom at z_base, d_top at z_base+h (filled from axis)."""
    return (
        cq.Workplane("XY").workplane(offset=z_base)
        .circle(d_bottom / 2)
        .workplane(offset=h)
        .circle(d_top / 2)
        .loft()
    )

def top_flange_solid(z_base):
    """Top-side sloped flange — widened for the ratchet + brake surfaces:
         - flat top at z_base+FLANGE_H spans FLANGE_INNER_ID → FLANGE_OD
         - 45° outer underside: (r=DRUM_OD/2, z_base) → (r=FLANGE_OD/2,
           z_base+slope_h_outer), outer lip FLANGE_LIP_T tall above
         - 45° inner underside: (r=DRUM_ID/2, z_inner_slope_bot) →
           (r=FLANGE_INNER_ID/2, z_inner_slope_top). The slope tops out
           FLANGE_INNER_LIP_H below the flange top, leaving a vertical
           inner lip (analogous to the outer lip). Since rise=run=7 mm
           for 45°, the slope extends z_inner_slope_bot below z_base —
           this overlaps the drum by FLANGE_INNER_LIP_H.
    Built as (base ring over full extended z range) − (outer overhang)
    − (inner overhang)."""
    slope_h_outer = FLANGE_H - FLANGE_LIP_T              # 5
    slope_h_inner = (DRUM_ID - FLANGE_INNER_ID) / 2      # 7
    z_top              = z_base + FLANGE_H               # flange top
    z_inner_slope_top  = z_top - FLANGE_INNER_LIP_H      # 2 mm below top
    z_inner_slope_bot  = z_inner_slope_top - slope_h_inner
    z_flange_bot       = min(z_base, z_inner_slope_bot)
    flange_h_actual    = z_top - z_flange_bot

    base_ring = (
        cyl(FLANGE_OD, flange_h_actual, z=z_flange_bot)
        .cut(cyl(FLANGE_INNER_ID, flange_h_actual, z=z_flange_bot))
    )

    # Outer overhang: unchanged, spans z_base..z_base+slope_h_outer
    outer_annulus = (
        cyl(FLANGE_OD, slope_h_outer, z=z_base)
        .cut(cyl(DRUM_OD, slope_h_outer, z=z_base))
    )
    outer_cone = cone_solid(DRUM_OD, FLANGE_OD, slope_h_outer, z_base)
    outer_overhang = outer_annulus.cut(outer_cone)

    # Inner overhang: now at z_inner_slope_bot..z_inner_slope_top (shifted
    # down by FLANGE_INNER_LIP_H). Cone narrows with increasing z, so the
    # air we cut is INSIDE the cone — intersect, not cut.
    inner_annulus = (
        cyl(DRUM_ID, slope_h_inner, z=z_inner_slope_bot)
        .cut(cyl(FLANGE_INNER_ID, slope_h_inner, z=z_inner_slope_bot))
    )
    inner_cone = cone_solid(DRUM_ID, FLANGE_INNER_ID, slope_h_inner, z_inner_slope_bot)
    inner_overhang = inner_annulus.intersect(inner_cone)

    result = base_ring.cut(outer_overhang).cut(inner_overhang)

    # If the flange extends below z_base (drum top) to accommodate the
    # 2 mm inner lip + 45° slope, cut any material outside DRUM_OD/2 in
    # that extension. Otherwise the flange's outer annulus skirts past
    # the drum wall at r=75..80 in z=29..31, intruding into the cable-
    # wrap zone. The slope itself terminates at the drum inner wall
    # (r=73); this just trims the skirt to match.
    if z_flange_bot < z_base:
        skirt_cut = (
            cyl(FLANGE_OD, z_base - z_flange_bot, z=z_flange_bot)
            .cut(cyl(DRUM_OD, z_base - z_flange_bot, z=z_flange_bot))
        )
        result = result.cut(skirt_cut)

    return result

def bottom_flange_solid(z_top):
    """Bottom-side sloped flange ring (mirror of top):
         - flat bottom at z_top−FLANGE_H (on the build plate)
         - 45° topside: slope from (r=DRUM_OD/2, z_top) to (r=FLANGE_OD/2, z_top−slope_h)
         - outer lip (2 mm thick) at the bed from z_top−FLANGE_H to z_top−slope_h
    Built as (base ring) − (overhang wedge at the top)."""
    slope_h = FLANGE_H - FLANGE_LIP_T
    z_base  = z_top - FLANGE_H
    z_slope_lo = z_top - slope_h
    base_ring = cyl(FLANGE_OD, FLANGE_H, z=z_base).cut(
                cyl(FLANGE_ID, FLANGE_H, z=z_base))
    slope_annulus = (
        cyl(FLANGE_OD, slope_h, z=z_slope_lo)
        .cut(cyl(DRUM_OD, slope_h, z=z_slope_lo))
    )
    # Cone is inverted vs top flange: wide at bottom (FLANGE_OD), narrow at top (DRUM_OD)
    support_cone = cone_solid(FLANGE_OD, DRUM_OD, slope_h, z_slope_lo)
    overhang = slope_annulus.cut(support_cone)
    return base_ring.cut(overhang)

def ratchet_cutter(num_teeth, r_in, r_out, z_top, depth, theta_offset_deg=0.0):
    """Sawtooth ratchet cutter with a true helicoidal ramp.

    Each tooth's MATERIAL is built as a loft between two radial rectangular
    wires: a near-degenerate rectangle at θ_start (height = eps, so the
    ramp begins at z_bottom) and a full rectangle at θ_end (z_bottom → z_top,
    so the ramp has climbed all the way up). The loft's ruled surface has
    constant angular slope at every radius — no triangular artifacts at
    tooth boundaries the way a flat-plane ramp would produce.

    Wires are built slightly oversized radially (±r_buf) so the chord
    approximation of each radial edge stays outside the target annulus.
    The union of tooth materials is intersected with a true annular groove
    for clean circular inner/outer boundaries; the cutter is then
    (groove − teeth) = the air to remove above each ramp."""
    import math
    pitch_rad = 2 * math.pi / num_teeth
    offset_rad = math.radians(theta_offset_deg)
    z_bottom  = z_top - depth
    r_buf     = 1.0
    r_in_b    = r_in - r_buf
    r_out_b   = r_out + r_buf
    eps       = 0.05      # minimum vertical thickness of the degenerate start wire

    def _tooth_material(i):
        th_s = offset_rad + i * pitch_rad
        th_e = offset_rad + (i + 1) * pitch_rad
        cs, ss = math.cos(th_s), math.sin(th_s)
        ce, se = math.cos(th_e), math.sin(th_e)

        pts_s = [
            cq.Vector(r_in_b  * cs, r_in_b  * ss, z_bottom),
            cq.Vector(r_out_b * cs, r_out_b * ss, z_bottom),
            cq.Vector(r_out_b * cs, r_out_b * ss, z_bottom + eps),
            cq.Vector(r_in_b  * cs, r_in_b  * ss, z_bottom + eps),
        ]
        pts_e = [
            cq.Vector(r_in_b  * ce, r_in_b  * se, z_bottom),
            cq.Vector(r_out_b * ce, r_out_b * se, z_bottom),
            cq.Vector(r_out_b * ce, r_out_b * se, z_top),
            cq.Vector(r_in_b  * ce, r_in_b  * se, z_top),
        ]
        wire_s = cq.Wire.makePolygon(pts_s, close=True)
        wire_e = cq.Wire.makePolygon(pts_e, close=True)
        return cq.Solid.makeLoft([wire_s, wire_e])

    teeth = _tooth_material(0)
    for i in range(1, num_teeth):
        teeth = teeth.fuse(_tooth_material(i))

    # Full annular groove (what would be cut if there were no teeth)
    groove = (
        cyl(2 * r_out, depth, z=z_bottom)
        .cut(cyl(2 * r_in, depth, z=z_bottom))
    )

    # Wrap teeth (Solid) in a Workplane to interoperate with groove (Workplane)
    teeth_wp = cq.Workplane("XY").add(teeth)

    # Clip teeth to the annulus: clean circular inner/outer boundaries
    teeth_clipped = teeth_wp.intersect(groove)

    # Cutter = groove − teeth_clipped = the air above each ramp
    return groove.cut(teeth_clipped)

def spokes_solid(z_base, z_top):
    """Vertical rib walls connecting hub OD → drum ID over [z_base, z_top].
    Outer-top corner is chamfered at 45° along the same line as the flange
    inner slope (from (r=r_taper, z=z_top) to (r=DRUM_ID/2, z=z_top−taper_h)
    where taper_h = FLANGE_INNER_EXT + FLANGE_INNER_LIP_H). Two reasons:
      1. Keeps the spoke top well below every tooth valley (z_top−9 = 29 is
         7.5 mm below the z=36.5 valley floor), so the spoke can never
         interfere with the pawl even when it drops fully into a valley.
      2. The 45° taper matches the flange inner slope visually, so the
         spoke appears to terminate along the same line that carves the
         flange inner cavity — no square-corner notch at the junction."""
    r_in         = HUB_OD / 2
    r_out        = DRUM_ID / 2
    taper_h      = FLANGE_INNER_EXT + FLANGE_INNER_LIP_H   # 9
    z_taper_end  = z_top - taper_h                         # 29
    r_taper_start = r_out - taper_h                        # 64
    out = None
    for i in range(SPOKE_COUNT):
        s = (
            cq.Workplane("XZ")
            .polyline([
                (r_in,          z_base),
                (r_out,         z_base),
                (r_out,         z_taper_end),
                (r_taper_start, z_top),
                (r_in,          z_top),
            ])
            .close()
            .extrude(SPOKE_W / 2, both=True)
            .rotate((0, 0, 0), (0, 0, 1), i * 360.0 / SPOKE_COUNT)
        )
        out = s if out is None else out.union(s)
    return out

# ────────────────────────────────────────────────────────────────────────────
# MAIN BODY — the full spool
#   Bottom flange, drum, top flange, 4 spokes, hub with bottom bearing pocket,
#   straight spring cavity, 1 mm-inset cap-stop lip, cap seat at top.
#   The cavity opens straight to the top face (no obstruction) — spring drops
#   in from above, then the bearing cap slides in after.
# ────────────────────────────────────────────────────────────────────────────

drum = cyl(DRUM_OD, DRUM_H, z=DRUM_BOTTOM_Z).cut(
       cyl(DRUM_ID, DRUM_H, z=DRUM_BOTTOM_Z))

main_body = (
    cyl(HUB_OD, SPOOL_H, z=0)                       # hub cylinder (full height)
    .union(bottom_flange_solid(DRUM_BOTTOM_Z))       # sloped bottom flange
    .union(drum)
    .union(top_flange_solid(DRUM_TOP_Z))             # sloped top flange
    .union(spokes_solid(0, SPOOL_H))                 # full-height spokes
)

# Interior z-map cuts (top of hub down):
#   SPOOL_H .. CAP_SEAT_Z0   — cap seat (HUB_CAVITY_D = 34 mm ID)
#   CAP_SEAT_Z0 .. STOP_LIP_Z0 — cap-stop lip (CAP_STOP_ID = 32 mm ID, 1 mm tall)
#   STOP_LIP_Z0 .. CAVITY_Z0 — straight spring cavity (34 mm ID)
#   CAVITY_Z0 .. BEARING_W   — bottom bearing retention lip (20 mm ID, 1 mm tall)
#   BEARING_W .. 0           — bottom bearing pocket (22.1 mm ID)
main_body = (
    main_body
    .cut(cyl(BEARING_BORE,    BEARING_W,                    z=0))              # bottom pocket
    # Bottom bearing lip — 45° cone taper from BEARING_BORE (pocket ID)
    # up to BEARING_LIP_ID. The spool prints teeth-up (pre-flip
    # orientation), so going z=+BEARING_W → z=+BEARING_W+BEARING_LIP_H
    # the inner wall moves inward 1.05 mm over 1 mm rise — 45°
    # self-supporting overhang. The UPPER face of the lip (that the
    # bearing presses against) stays flat 90°, as needed for a good
    # bearing seat.
    .cut(cone_solid(BEARING_BORE, BEARING_LIP_ID, BEARING_LIP_H, BEARING_W))   # bottom lip
    .cut(cyl(HUB_CAVITY_D,    CAVITY_Z1 - CAVITY_Z0,        z=CAVITY_Z0))      # spring cavity
    # Cap-stop lip — 45° conical inward taper from HUB_CAVITY_D at the
    # bottom (STOP_LIP_Z0) to CAP_STOP_ID at the top (CAP_SEAT_Z0). Self-
    # supporting underside, and the slope coincides with the spring
    # slot's sloped roof (see below).
    .cut(cone_solid(HUB_CAVITY_D, CAP_STOP_ID, CAP_STOP_LIP_H, STOP_LIP_Z0))   # cap-stop lip
    .cut(cyl(HUB_CAVITY_D,    SPOOL_H - CAP_SEAT_Z0,        z=CAP_SEAT_Z0))    # cap seat
)

# Spring outer-end attachment slot — a blind radial notch in the hub
# wall sized to receive the spring's bent-over outer tab (12.7 mm z x
# 0.15 mm thick). Orientation: tall in z (matching the strip width),
# narrow in y (fits the strip thickness with tolerance), shallow in x.
# Blind outer face leaves ≥1.5 mm of hub-OD wall intact — no slits.
#
# Conical top: the slot's roof IS the cap-stop lip's 45° cone surface,
# extended. Both the lip underside (z in [STOP_LIP_Z0, STOP_LIP_Z0 +
# CAP_STOP_LIP_H]) and the slot roof (below) lie on the same cone
# r = HUB_CAVITY_D/2 + STOP_LIP_Z0 − z, apex at (0, 0, STOP_LIP_Z0 +
# HUB_CAVITY_D/2). Using the cone itself to trim the slot guarantees
# flush alignment at every y, not just y=0 (a flat-plane approximation
# would drift ~0.03 mm off the cone at y=±SPRING_SLOT_W/2).
SPRING_SLOT_H_RECT = 13.0   # rectangle height (z_bottom to outer-edge roof)
SPRING_SLOT_W      =  2.0   # y-extent
SPRING_SLOT_DEPTH  =  2.0   # x-extent (radial)

_slot_x_outer = -(HUB_CAVITY_D / 2 + SPRING_SLOT_DEPTH - 0.5)   # -18.5
_slot_x_inner = -(HUB_CAVITY_D / 2 - 0.5)                       # -16.5
_slot_ztop_out = STOP_LIP_Z0 - (SPRING_SLOT_DEPTH - 0.5)        # 27.5 (at y=0, x=outer)
_slot_zb = _slot_ztop_out - SPRING_SLOT_H_RECT                  # 14.5
_lip_apex_z = STOP_LIP_Z0 + HUB_CAVITY_D / 2                    # 46 — cone apex

# Rectangular prism that overshoots the apex in z; the cone intersection
# below trims its top surface to the cone.
_slot_prism = (
    cq.Workplane("XY")
    .workplane(offset=_slot_zb)
    .center((_slot_x_outer + _slot_x_inner) / 2, 0)
    .box(
        abs(_slot_x_inner - _slot_x_outer),
        SPRING_SLOT_W,
        _lip_apex_z - _slot_zb + 1.0,
        centered=(True, True, False),
    )
)

# The lip cone, extended downward from its apex at z=_lip_apex_z all the
# way to z=0. Its lateral surface r = _lip_apex_z − z is exactly the
# cap-stop lip's underside surface.
_lip_cone_ext = cq.Workplane("XY").add(cq.Solid.makeCone(
    _lip_apex_z, 0.0, _lip_apex_z,
    pnt=cq.Vector(0, 0, 0),
    dir=cq.Vector(0, 0, 1),
))

spring_slot = _slot_prism.intersect(_lip_cone_ext)
main_body = main_body.cut(spring_slot)

# (Ratchet teeth are cut later in the file, after RATCHET_TOOTH_OFFSET_DEG
# has been computed from the ratchet lever's y-range and the pawl
# footprint's r-range. Placing the cut after those definitions lets the
# tooth phase automatically track any lever repositioning.)

# ────────────────────────────────────────────────────────────────────────────
# BEARING CAP — small cylindrical insert
#   33.8 mm OD (slip fit into 34 mm cavity), 8 mm tall.
#   Bottom 1 mm is a 20 mm-ID retention lip (stops the bearing).
#   Top 7 mm is the 22.1 mm-ID bearing pocket (bearing presses in from above).
#   No spokes, no flange — it's just a small disc that retains the bearing.
# ────────────────────────────────────────────────────────────────────────────

bearing_cap = cyl(CAP_OD, CAP_H, z=0)
bearing_cap = (
    bearing_cap
    # Bottom lip → 45° cone taper from BEARING_BORE down to BEARING_LIP_ID
    # over BEARING_LIP_H. Replaces the horizontal pocket→lip step so the
    # transition self-supports when printed.
    .cut(cone_solid(BEARING_LIP_ID, BEARING_BORE, BEARING_LIP_H, 0))
    .cut(cyl(BEARING_BORE,   BEARING_W,     z=BEARING_LIP_H))      # bearing pocket (z=1..8)
)

# ────────────────────────────────────────────────────────────────────────────
# AXLE  (stationary; spool rotates around it on bearings)
#   8 mm shaft full-length, with integrated 23 mm spring stub between the
#   bearings. The 8→23 mm shoulder is a 45° cone so the axle prints vertically
#   without supports.
# ────────────────────────────────────────────────────────────────────────────

axle_shaft = cyl(AXLE_D, AXLE_H, z=-AXLE_EXTRA)

# Both the 45° chamfer AND the straight stub cylinder are hollow 2 mm walls.
# Outer surface (what the spring wraps against) is unchanged. Inside, four
# internal radial fins (two diametric bars at 45°/135°) connect the walls
# to the central shaft — fins are off the ±x axis so the spring tab hole
# doesn't pass through them. Fins taper naturally in the chamfer region
# (grow from 0 width at z≈10 to full 6.5 mm at the stub) because we
# intersect the fin bars with the axle's solid outer envelope.
STUB_WALL_T   = 2.0
STUB_SPOKE_T  = 2.0
_stub_z0      = STUB_CYL_Z0
_stub_h       = STUB_TOP_Z - STUB_CYL_Z0

stub_chamfer = (
    cone_solid(AXLE_D, SPRING_STUB_OD, STUB_CHAMFER_H, STUB_BASE_Z)
    .cut(cone_solid(
        AXLE_D - 2 * STUB_WALL_T,
        SPRING_STUB_OD - 2 * STUB_WALL_T,
        STUB_CHAMFER_H, STUB_BASE_Z))
)
stub_wall = (
    cyl(SPRING_STUB_OD, _stub_h, z=_stub_z0)
    .cut(cyl(SPRING_STUB_OD - 2 * STUB_WALL_T, _stub_h, z=_stub_z0))
)

# Solid outer envelope used to clip the fin bars so nothing sticks out past
# the chamfer's 45° cone surface.
_axle_outer_env = (
    cone_solid(AXLE_D, SPRING_STUB_OD, STUB_CHAMFER_H, STUB_BASE_Z)
    .union(cyl(SPRING_STUB_OD, _stub_h, z=_stub_z0))
)
_raw_fins = None
for _angle in (45, 135):
    _bar = (
        cq.Workplane("XY").workplane(offset=STUB_BASE_Z)
        .rect(SPRING_STUB_OD, STUB_SPOKE_T)
        .extrude(STUB_TOP_Z - STUB_BASE_Z)
        .rotate((0, 0, 0), (0, 0, 1), _angle)
    )
    _raw_fins = _bar if _raw_fins is None else _raw_fins.union(_bar)
stub_spokes = _raw_fins.intersect(_axle_outer_env)

axle = axle_shaft.union(stub_chamfer).union(stub_wall).union(stub_spokes)

# Spring inner-end is NOT mechanically anchored — the spring's natural ID
# (20.32 mm) is smaller than SPRING_STUB_OD (25 mm), so stretching it
# onto the stub creates radial clamping force that grips immediately,
# and capstan friction across the remaining wraps holds the inner end
# against the 0.7 lbf pull. If field testing shows slip, replace with a
# small radial retention pin and a matching hole in the spring's inner
# end tab.

# Axle ledges — both ends turn down to AXLE_LEDGE_D with a -y D-flat. The
# housing plates drop onto the 0.5 mm shoulder (axial register) and the
# D-flat keys them rotationally. M2 inserts in both ends + screws through
# the housing's axle pads lock everything vertically.
_top_ledge_z   = SPOOL_H + HOUSING_BEARING_GAP   # 48 — top reduced section starts here
_bot_ledge_z   = -AXLE_EXTRA                     # -12 — bottom reduced section starts here
_axle_tip      = SPOOL_H + AXLE_EXTRA            # 50 — top of axle
_flat_y        = AXLE_D / 2 - AXLE_FLAT_DEPTH    # 3 — shared flat plane
# 1) Reduce the OD at the top: cut the annulus from AXLE_D → AXLE_LEDGE_D
axle = axle.cut(
    cyl(AXLE_D, AXLE_LEDGE_H, z=_top_ledge_z)
    .cut(cyl(AXLE_LEDGE_D, AXLE_LEDGE_H, z=_top_ledge_z))
)
# 2) Mirror reduction at the bottom
axle = axle.cut(
    cyl(AXLE_D, AXLE_LEDGE_H, z=_bot_ledge_z)
    .cut(cyl(AXLE_LEDGE_D, AXLE_LEDGE_H, z=_bot_ledge_z))
)
# 3) D-flat at the top ledge — 0.5 mm deep on the 7 mm section. Flat on -y
# so that when the housing is printed lying on its -y face, the flat edge of
# each D-hole is at the BOTTOM of the hole (sits on solid layers below)
# instead of a hard overhang at the top.
axle = axle.cut(
    cq.Workplane("XY").workplane(offset=_top_ledge_z)
    .center(0, -(_flat_y + 1))
    .rect(AXLE_D + 2, 2)
    .extrude(AXLE_LEDGE_H)
)
# 4) D-flat at the bottom ledge — same -y flat plane
axle = axle.cut(
    cq.Workplane("XY").workplane(offset=_bot_ledge_z)
    .center(0, -(_flat_y + 1))
    .rect(AXLE_D + 2, 2)
    .extrude(AXLE_LEDGE_H)
)
# 4) Insert pilot holes at both ends
axle = axle.cut(
    cyl(AXLE_INSERT_PILOT_D, AXLE_INSERT_DEPTH, z=_axle_tip - AXLE_INSERT_DEPTH)
)
axle = axle.cut(
    cyl(AXLE_INSERT_PILOT_D, AXLE_INSERT_DEPTH, z=-AXLE_EXTRA)
)

# ────────────────────────────────────────────────────────────────────────────
# Export
# ────────────────────────────────────────────────────────────────────────────

# ────────────────────────────────────────────────────────────────────────────
# LEVER HOUSING — U-bracket keyed only at the top of the axle.
#
# Two 2 mm plates (top + bottom) connected by a 2 mm spine. The top plate's
# 6 mm D-hole drops over the axle's reduced top section and catches on the
# 1 mm shoulder — that single ledge keys the whole housing rotationally AND
# axially. The bottom plate has a plain 8 mm round slip-fit hole; the spine
# keeps it parallel and concentric once the top is seated. Each plate sits
# HOUSING_BEARING_GAP away from the nearest bearing.
#
# A local 10 mm-thick pad on the top plate supports the lever pivot bosses
# (9 mm OD won't fit on 2 mm). Pad extends OUTWARD (+z, away from the spool)
# from the plate so it doesn't eat into the bearing-gap clearance.
#
# XZ side view (y into page; y-width = 22 mm):
#
#                                                 pad (10 mm combined)
#                                      ╱─────●────●──────┐ ┌─┐
#                                 ╱ 45°      pivots      │ │ │
#    (−11, 50)  ═══════════════════   thin plate (2 mm)   │ │ │
#    (−11, 48)  ═══ top plate ════════════════════════════┤ │ │
#           top ledge (6 mm D at z=48..50, caught on shoulder)
#           bearing gap (10 mm)                             │ │
#           spool (z = 0..38)                               │ │ spine
#           bearing gap (10 mm)                             │ │ (2 mm)
#    (−11, −10) ═══ bot plate ════════════════════════════┤ │ │
#    (−11, −12) ═══════════════════════════════════════════ │ │
#                                                          │ │ │
#                                                          └─┘
#
# ────────────────────────────────────────────────────────────────────────────

HOUSING_W         =  22.0    # matches bearing OD
HOUSING_CLR       =  10.0    # radial clearance past flange OD
HOUSING_SPINE_T   =   2.0    # spine thickness (x direction)
HOUSING_HOLE_CLR  =   0.15   # axle hole slip fit

SPINE_X_INNER     = FLANGE_OD / 2 + HOUSING_CLR         # 90 — 10 mm past flange OD
SPINE_X_OUTER     = SPINE_X_INNER + HOUSING_SPINE_T     # 92
HOUSING_X_TAIL    = -HOUSING_W / 2                      # -11 (22 mm x 2 mm square tail)

# Top-plate pads — 9 mm OD pivot bosses can't sit on a 2 mm plate (pivot pad
# → 10 mm). And the axle attachment needs more material for the M2 insert
# screw to hold onto (axle pad → 20 mm). Both pads extend OUTWARD (+z, away
# from the spool) so neither eats into the bearing-gap clearance.
PIVOT_PAD_Z       =  5.0     # pivot-column z-height at each lever pivot
PIVOT_COL_HALF_W  =  5.0     # half-width in x of the pivot column. 5 mm
                             # either side of the pivot axis gives the
                             # stop pin (at r=3.5, α up to 24.5°) about
                             # 1 mm of column material past the pin's
                             # outer edge — no "over the edge" issue.

AXLE_PAD_Z        =  4.0     # axle-column z-height at the axle hole
AXLE_COL_HALF_W   =  6.0     # half-width in x of the axle column. 6 mm
                             # either side of the 7 mm D-hole leaves 2.5 mm
                             # wall thickness.

# Lever pivot x-positions — defined here (rather than below) so the plate
# profile can place a column at each one.
# Pulling direction: user pushes the bottom face of the lever toward the
# spool (post-flip +z, pre-flip −z), which drops the handle end in pre-flip
# coords. For that pull to DISENGAGE the ratchet and ENGAGE the brake:
#   RATCHET is class-1 (pivot BETWEEN pawl and handle; opposite motions).
#     Pivot at x=76, pawl catch edge at x≈66.4 (−x of pivot), handle at
#     x=115 (+x). Handle drops → pawl rises off teeth. Pivot position is
#     tuned so that by the moment the pawl clears the tooth tip, the
#     brake pad has descended exactly its rest-lift distance (2 mm).
#   BRAKE is class-2 (pad and handle BOTH on +x of pivot; same motions).
#     Pivot at x=55, pad midpoint at x≈74 (+x), handle at x=115 (+x).
#     Handle drops → pad descends onto the brake ring.
RATCHET_PIVOT_X   = 76.0
BRAKE_PIVOT_X     = 55.0

# Derived z-coordinates (SPOOL_H=38 top, 0 bottom of the spool body)
_TOP_PLATE_Z_IN   = SPOOL_H + HOUSING_BEARING_GAP                 # 48 — plate inner face
_TOP_PLATE_Z_OUT  = _TOP_PLATE_Z_IN + PLATE_T                     # 50 — plate outer face
_TOP_PIVOT_Z_OUT  = _TOP_PLATE_Z_IN + PIVOT_PAD_Z                 # 58 — pivot pad outer face
_TOP_AXLE_Z_OUT   = _TOP_PLATE_Z_IN + AXLE_PAD_Z                  # 52 — top axle pad outer face
_BOT_PLATE_Z_IN   = -HOUSING_BEARING_GAP                          # -10 — inner face
_BOT_PLATE_Z_OUT  = _BOT_PLATE_Z_IN - PLATE_T                     # -12 — outer face
_BOT_AXLE_Z_OUT   = -HOUSING_BEARING_GAP - AXLE_PAD_Z             # -14 — bottom axle pad outer face

# Each column rises vertically by (column_Z − PLATE_T). The side walls use a
# 45° slope over the same distance in x (aesthetic, not structural — purely
# vertical walls would also print fine).
_AXLE_STEP_H      = AXLE_PAD_Z  - PLATE_T   # 2 mm rise at axle column
_PIVOT_STEP_H     = PIVOT_PAD_Z - PLATE_T   # 3 mm rise at pivot columns

_AXLE_COL_X0      = -AXLE_COL_HALF_W
_AXLE_COL_X1      = +AXLE_COL_HALF_W
_AXLE_RAMP_X0     = _AXLE_COL_X0 - _AXLE_STEP_H
_AXLE_RAMP_X1     = _AXLE_COL_X1 + _AXLE_STEP_H

_BRAKE_COL_X0     = BRAKE_PIVOT_X   - PIVOT_COL_HALF_W
_BRAKE_COL_X1     = BRAKE_PIVOT_X   + PIVOT_COL_HALF_W
_BRAKE_RAMP_X0    = _BRAKE_COL_X0 - _PIVOT_STEP_H
_BRAKE_RAMP_X1    = _BRAKE_COL_X1 + _PIVOT_STEP_H

_RATCHET_COL_X0   = RATCHET_PIVOT_X - PIVOT_COL_HALF_W
_RATCHET_COL_X1   = RATCHET_PIVOT_X + PIVOT_COL_HALF_W
_RATCHET_RAMP_X0  = _RATCHET_COL_X0 - _PIVOT_STEP_H
_RATCHET_RAMP_X1  = _RATCHET_COL_X1 + _PIVOT_STEP_H

def _top_plate():
    """2 mm base plate with local columns at each hole: axle column rising
    AXLE_STEP_H above the plate, pivot columns rising PIVOT_STEP_H. Each
    column has 45° side ramps in x (aesthetic). Columns are walked in
    strictly decreasing x order (spine → brake pivot → ratchet pivot →
    axle → tail) so the profile polygon never crosses itself.

    If a column's +x ramp edge happens to land exactly at SPINE_X_INNER,
    skip the redundant plate-level segment between them to avoid a
    zero-length edge in the polygon."""
    wp = (
        cq.Workplane("XZ")
        .moveTo(HOUSING_X_TAIL,     _TOP_PLATE_Z_IN)
        .lineTo(SPINE_X_INNER,      _TOP_PLATE_Z_IN)
        .lineTo(SPINE_X_INNER,      _TOP_PLATE_Z_OUT)
    )
    # Walk left along plate outer face, traversing columns in +x → -x order.
    # Sort by rightmost x (ramp_x1) descending so the walk is always correct
    # regardless of which pivot (ratchet/brake) is at which x.
    columns = sorted([
        (_BRAKE_RAMP_X1,   _BRAKE_COL_X1,   _BRAKE_COL_X0,   _BRAKE_RAMP_X0,   _TOP_PIVOT_Z_OUT),
        (_RATCHET_RAMP_X1, _RATCHET_COL_X1, _RATCHET_COL_X0, _RATCHET_RAMP_X0, _TOP_PIVOT_Z_OUT),
        (_AXLE_RAMP_X1,    _AXLE_COL_X1,    _AXLE_COL_X0,    _AXLE_RAMP_X0,    _TOP_AXLE_Z_OUT),
    ], key=lambda c: c[0], reverse=True)
    prev_x1 = SPINE_X_INNER
    for ramp_x1, col_x1, col_x0, ramp_x0, col_top_z in columns:
        if ramp_x1 < prev_x1:
            wp = wp.lineTo(ramp_x1, _TOP_PLATE_Z_OUT)
        wp = (
            wp
            .lineTo(col_x1,  col_top_z)
            .lineTo(col_x0,  col_top_z)
            .lineTo(ramp_x0, _TOP_PLATE_Z_OUT)
        )
        prev_x1 = ramp_x0
    return (
        wp
        .lineTo(HOUSING_X_TAIL, _TOP_PLATE_Z_OUT)
        .close()
        .extrude(HOUSING_W / 2, both=True)
    )

def _bot_plate():
    """2 mm base plate with a local axle column (45° side ramps). No pivot
    columns — the levers live on the top plate only."""
    return (
        cq.Workplane("XZ")
        .moveTo(HOUSING_X_TAIL,     _BOT_PLATE_Z_IN)
        .lineTo(SPINE_X_INNER,      _BOT_PLATE_Z_IN)
        .lineTo(SPINE_X_INNER,      _BOT_PLATE_Z_OUT)
        .lineTo(_AXLE_RAMP_X1,      _BOT_PLATE_Z_OUT)
        .lineTo(_AXLE_COL_X1,       _BOT_AXLE_Z_OUT)
        .lineTo(_AXLE_COL_X0,       _BOT_AXLE_Z_OUT)
        .lineTo(_AXLE_RAMP_X0,      _BOT_PLATE_Z_OUT)
        .lineTo(HOUSING_X_TAIL,     _BOT_PLATE_Z_OUT)
        .close()
        .extrude(HOUSING_W / 2, both=True)
    )

def _axle_d_hole(z_base, height, axle_d):
    """D-shape hole sized for a given axle OD. Flat is on -y so that when the
    housing is printed lying on -y face, the flat edge is at the bottom of
    the hole (easy to print). Shared flat plane across all plates."""
    r = axle_d / 2 + HOUSING_HOLE_CLR
    flat_y = (AXLE_D / 2 - AXLE_FLAT_DEPTH) + HOUSING_HOLE_CLR       # 3.15
    full_cyl = cyl(r * 2, height, z=z_base)
    cap = (
        cq.Workplane("XY").workplane(offset=z_base)
        .center(0, -(flat_y + r + 1) / 2)
        .rect(r * 2 + 2, (r + 1) - flat_y)
        .extrude(height)
    )
    return full_cyl.cut(cap)

top_plate = (
    _top_plate()
    .cut(_axle_d_hole(_TOP_PLATE_Z_IN, PLATE_T, AXLE_LEDGE_D))
    # M2 clearance hole through the top axle pad
    .cut(cyl(AXLE_SCREW_CLR_D, AXLE_PAD_Z - PLATE_T, z=_TOP_PLATE_Z_OUT))
)
bot_plate = (
    _bot_plate()
    .cut(_axle_d_hole(_BOT_PLATE_Z_OUT, PLATE_T, AXLE_LEDGE_D))
    # M2 clearance hole through the bottom axle pad
    .cut(cyl(AXLE_SCREW_CLR_D, AXLE_PAD_Z - PLATE_T, z=_BOT_AXLE_Z_OUT))
)

spine = (
    cq.Workplane("XY").workplane(offset=_BOT_PLATE_Z_OUT)
    .center((SPINE_X_INNER + SPINE_X_OUTER) / 2, 0)
    .box(HOUSING_SPINE_T, HOUSING_W,
         _TOP_PLATE_Z_OUT - _BOT_PLATE_Z_OUT,
         centered=(True, True, False))
)

# Wall-mount stud pattern — two clearance holes drilled diagonally
# through the spine from the +x face (the face that points away from
# the spool, i.e. the face that sits against the mounting surface).
# Install method: 10-24 wood-screw studs (McMaster 90915A641) drive
# into the mounting board, studs protrude out, the housing slides on,
# and 10-24 flange nuts (McMaster 90389A112, flange OD 12.7 mm)
# tighten on the +x face to clamp the spine to the board. No hardware
# shows on the back of the board.
# Hole placement:
#   Lever-side hole (z = MOUNT_SCREW_LEVER_Z = 34 pre-flip): on the
#     -y half of the spine. 16 mm in from the lever-side edge of the
#     spine so the housing overhangs the board by that much.
#   Far-side hole (z = MOUNT_SCREW_FAR_Z = -4 pre-flip): on the +y
#     half of the spine. 8 mm in from the opposite end.
# Edge inset: 6.5 mm from the y side edges (|y| = 4.5). Constraint:
# the flange nut's 12.7 mm OD must land fully on the 22 mm wide spine,
# so stud center must be at |y| ≤ 22/2 − 12.7/2 = 4.65 mm. 4.5 mm
# gives 0.15 mm of margin. A 2x2 pattern is impossible on this spine
# width — two 12.7 mm flanges can't fit side-by-side at any y that
# also keeps them inside the 22 mm spine edges. The diagonal layout
# puts the two flanges 39 mm apart, well clear of each other.
MOUNT_SCREW_CLR_D    = 5.3                                    # #10 clearance
MOUNT_SCREW_EDGE_Y   = HOUSING_W / 2 - 6.5                    # 4.5 mm
# Levers are mounted on the pre-flip +z half (top plate); "far from
# levers" is therefore the pre-flip -z end of the spine.
MOUNT_SCREW_LEVER_Z  = _TOP_PLATE_Z_OUT - 16.0                # 34 (pre-flip)
MOUNT_SCREW_FAR_Z    = _BOT_PLATE_Z_OUT + 8.0                 # -4 (pre-flip)

def _mount_hole(y, z):
    """M2 clearance hole through the spine, axis along x. Enters from the
    +x face (mounting face); extends slightly past both faces for clean
    booleans."""
    x_start = SPINE_X_INNER - 0.5                             # 0.5 mm inside
    x_len   = HOUSING_SPINE_T + 1.0                           # through + 0.5 each side
    return cq.Workplane("XY").add(cq.Solid.makeCylinder(
        MOUNT_SCREW_CLR_D / 2, x_len,
        pnt=cq.Vector(x_start, y, z),
        dir=cq.Vector(1, 0, 0),
    ))

# Lever pivots — two separate M2 cap screws, each threading into an M2 heat-
# set insert embedded in the housing. (RATCHET_PIVOT_X and BRAKE_PIVOT_X are
# defined above, near the column parameters.) Each insert is on the OPPOSITE
# face of the housing from its lever, so the screw tip stays inside the
# insert and doesn't protrude past the housing.
LEVER_PIVOT_Z         = (_TOP_PLATE_Z_IN + _TOP_PIVOT_Z_OUT) / 2   # centered on the pivot column
LEVER_SCREW_CLR_D     = 2.3     # M2 (2 mm thread) with 0.3 mm clearance
LEVER_INSERT_PILOT_D  = 3.3     # McMaster 94459A110 heat-set insert — spec'd
                                # "For Maximum Hole Diameter 3.3 mm". The
                                # insert's inner bore is 3.1 mm and the
                                # full toothed OD is 3.6 mm; the teeth
                                # melt ~0.15 mm of plastic per side during
                                # the heat-set install.
LEVER_INSERT_DEPTH    = 3.5     # 2.5 mm insert (94459A110) + 1 mm margin
LEVER_INSERT_CHAMFER_H = (LEVER_INSERT_PILOT_D - LEVER_SCREW_CLR_D) / 2  # 0.45 — 45° chamfer
                                # at the inner end of the pilot so the
                                # pilot→clearance step prints as a taper
                                # instead of a horizontal overhang ledge.

def _pivot_clearance(x):
    """M2 through-hole (axis along y), diameter 2.3 mm, full housing width."""
    return (
        cq.Workplane("XY")
        .circle(LEVER_SCREW_CLR_D / 2)
        .extrude(HOUSING_W + 2)
        .rotate((0, 0, 0), (1, 0, 0), -90)
        .translate((x, -HOUSING_W / 2 - 1, LEVER_PIVOT_Z))
    )

def _pivot_insert_pilot(x, insert_face_sign):
    """M2 insert pilot hole, drilled from one face of the housing (+y or -y,
    per insert_face_sign = +1 or -1). Straight 3.2 mm pilot for 5 mm, then a
    45° cone taper from 3.2 → 2.3 at the inner end so the pilot→clearance
    step prints as a self-supporting slope instead of a horizontal ledge."""
    pilot = (
        cq.Workplane("XY")
        .circle(LEVER_INSERT_PILOT_D / 2)
        .extrude(LEVER_INSERT_DEPTH)
        .union(cone_solid(LEVER_INSERT_PILOT_D, LEVER_SCREW_CLR_D,
                          LEVER_INSERT_CHAMFER_H, LEVER_INSERT_DEPTH))
    )
    if insert_face_sign > 0:
        return pilot.rotate((0, 0, 0), (1, 0, 0), +90) \
                    .translate((x, HOUSING_W / 2, LEVER_PIVOT_Z))
    else:
        return pilot.rotate((0, 0, 0), (1, 0, 0), -90) \
                    .translate((x, -HOUSING_W / 2, LEVER_PIVOT_Z))

# Stop pins — double-duty: mechanical travel stops AND anchors for the
# torsion-spring legs. One pin on each lever's inner face (projects into
# the 1 mm rim gap toward the housing), one matching pin on each housing
# pivot-column face (projects back toward the lever). Lever pin rotates
# with the lever; collision with the fixed housing pin sets the limit in
# the "outer" direction for each lever:
#   - Ratchet: housing pin CW of lever pin; blocks CW rotation (max-pull
#     direction) at Δθ ≈ -12° (matched LEVER_HANDLE_TRAVEL_MAX drop).
#   - Brake:   housing pin CCW of lever pin; blocks CCW rotation (droop
#     direction) at Δθ = 0° (rest — lever hangs on the housing pin).
# Torsion-spring legs press against these same pins, so no extra anchors
# are needed. At r=2 mm, a 1 mm pin has D/r = 28.65° of angular width;
# the housing-pin angles below are chosen so pin outer edges meet at the
# intended stop angle.
STOP_PIN_D       = 1.5     # mm, pin OD. Beefier than minimum so a 0.6 mm
                           # diametral through-hole (for the spring leg)
                           # still leaves ~0.45 mm wall for stop-contact
                           # strength.
STOP_PIN_HOLE_D  = 0.6     # mm, through-hole in pin (fits 0.5 mm wire with
                           # 0.1 mm radial clearance). Hole is perpendicular
                           # to pin axis, along the radial-from-pivot
                           # direction, so the spring leg passes straight
                           # through the pin from inside.
STOP_PIN_H       = 2.0     # mm, y-extent. With a 3 mm rim gap, each pin
                           # extends 2 mm from its own face into the gap,
                           # leaving 1 mm of air between its tip and the
                           # opposing surface. Pin overlap region = 1 mm
                           # in the middle of the gap.
STOP_PIN_R       = 3.5     # mm, radial offset from pivot. Pin inner edge at
                           # r=2.75 gives 0.25 mm clearance from the coil
                           # outer at r=2.5. At r=3.5, D/r = 24.5° angular
                           # width (pin-to-pin outer-edge tangent angle).
SPRING_WIRE_R    = 0.25    # radius of the spring wire (0.5 mm wire)

# Lever pin angular positions — at the pin (and therefore leg) α:
#   Ratchet: α=180° (pawl side of pivot)
#   Brake:   α=0°   (pad/handle side of pivot)
STOP_LEVER_PIN_ALPHA_RATCHET_DEG   = 180.0
STOP_LEVER_PIN_ALPHA_BRAKE_DEG     =   0.0
# Housing pins — positioned so their outer edge meets the lever pin's
# outer edge at the intended stop angle (angular separation = D/r = 24.5°
# when the pins touch):
#   Ratchet: rest pins ~36.5° apart → touch after Δθ = −12° CW rotation
#            → housing pin at 180° − 36.5° = 143.5°.
#   Brake:   rest pins are already touching (stop engaged at Δθ=0°)
#            → housing pin at 0° + 24.5° = 24.5°.
STOP_HOUSING_PIN_ALPHA_RATCHET_DEG = 143.5
STOP_HOUSING_PIN_ALPHA_BRAKE_DEG   =  24.5

STOP_PIN_SUPPORT_D = 3.0    # OD of the raised support ridge coaxial with
                            # each housing stop pin. The ridge runs across
                            # the full column y width; its upper half
                            # projects above the column top (by ~1 mm for
                            # ratchet, ~0.45 mm for brake) as a visible
                            # arch that encases the pin where it would
                            # otherwise poke out of the flat column top.

# Brake housing stop pin — printed separately, press-fit into the housing.
# The housing prints upside-down (axle D-flats sit on the build plate),
# which puts the brake-side column face on the bottom of the print. An
# integrated pin there would need a support tower under the entire column
# face. Instead, the housing has a keyed blind hole; the pin is a tiny
# standalone part that gets glued in after printing.
#
# Geometry: the pin's insert shank has a D-flat so it drops in at exactly
# one rotational orientation (ensures the diametral through-hole for the
# spring leg ends up radial). The blind hole terminates in a 45° cone
# pointing into the column — during printing (housing upside-down), the
# cone narrows the hole to a point as it goes up, so every printed layer
# in the hole region is supported by the ring of housing material from
# the previous layer. The pin mirrors that cone so it bottoms out cleanly.
BRAKE_PIN_INSERT_DEPTH = 2.0     # mm, shank length buried in the housing
BRAKE_PIN_CONE_H       = STOP_PIN_D / 2   # 45° cone: h = r_nom = 0.75
BRAKE_PIN_HOLE_CLR     = 0.10    # diametral slip clearance (hole − pin)
BRAKE_PIN_FLAT_DEPTH   = 0.4     # D-flat cut depth (into pin's +radial side)

# Lever rim — raised cylindrical boss on each lever's inner face, spacing
# the lever off the housing by LEVER_RIM_H. Defined here (ahead of the
# other lever constants) so the housing construction can compute the
# spring-coil y plane (= gap midline) for the stop-pin through-holes.
LEVER_RIM_OD         = 3.0     # matches Lee Spring LTM*050E rod spec
                               # (3.00 mm). With coil OD 4.5 mm, the coil
                               # outer edge sits at r=2.25 — leaves 0.5 mm
                               # clearance to the stop pins' inner edge
                               # at r = STOP_PIN_R − STOP_PIN_D/2 = 2.75.
LEVER_RIM_H          = 3.0     # axial extension past lever body, = gap
                               # between lever and housing. The gap is
                               # split into three 1-mm zones:
                               #   [1 mm air] [1 mm stop-pin overlap]
                               #                               [1 mm air]
                               # Each stop pin is STOP_PIN_H=2 mm long,
                               # spanning the air gap on its own side plus
                               # the 1-mm overlap. At the stop angle the
                               # pin cylinders contact each other side-to-
                               # side over the 1 mm overlap. Each pin tip
                               # has 1 mm clearance to the opposite face
                               # (lever pin tip to housing face, and vice
                               # versa) — prevents tip-dragging friction.

def _stop_pin(pivot_x, alpha_deg, y_from, y_to, hole_y):
    """Cylindrical pin (axis along y, OD STOP_PIN_D) with a diametral
    through-hole (STOP_PIN_HOLE_D) in the radial direction for the
    torsion-spring leg. Pin center at (x, z) = pivot + STOP_PIN_R·(cos α,
    sin α); pin extends in y from y_from to y_to. `hole_y` is the y
    coordinate of the hole's axis — pass the spring-coil y (gap center)
    so lever and housing pin holes align on the same plane even though
    the pins are offset along y.

    The pin's outer cylindrical surface provides the mechanical stop
    (cylinder-to-cylinder tangent contact with the opposing pin at the
    stop angle); the through-hole holds the spring leg that transmits
    restoring torque to the pin."""
    alpha_rad = math.radians(alpha_deg)
    cos_a = math.cos(alpha_rad)
    sin_a = math.sin(alpha_rad)
    x = pivot_x + STOP_PIN_R * cos_a
    z = LEVER_PIVOT_Z + STOP_PIN_R * sin_a
    pin = (
        cq.Workplane("XY")
        .circle(STOP_PIN_D / 2)
        .extrude(y_to - y_from)
        .rotate((0, 0, 0), (1, 0, 0), -90)
        .translate((x, y_from, z))
    )
    # Diametral through-hole for the spring leg — radial direction,
    # centered at hole_y (which should be the coil plane for both
    # matching pins to share a common leg line). Extend the hole
    # slightly beyond each face of the pin for a clean boolean cut.
    hole_len = STOP_PIN_D + 0.4
    hole_start = cq.Vector(
        x - (STOP_PIN_D / 2 + 0.2) * cos_a,
        hole_y,
        z - (STOP_PIN_D / 2 + 0.2) * sin_a,
    )
    hole = cq.Solid.makeCylinder(
        STOP_PIN_HOLE_D / 2, hole_len,
        pnt=hole_start,
        dir=cq.Vector(cos_a, 0, sin_a),
    )
    return pin.cut(cq.Workplane("XY").add(hole))

def _stop_pin_support(pivot_x, alpha_deg):
    """Raised support ridge coaxial with a housing stop pin, diameter
    STOP_PIN_SUPPORT_D, spanning the full column y width. Most of the
    cylinder lives inside the column material (redundant, just unions
    with the column); the upper portion that projects above the column
    top face (z=_TOP_PIVOT_Z_OUT) appears as a visible arched ridge —
    cleaner than a bare pin poking through the flat top."""
    alpha_rad = math.radians(alpha_deg)
    x = pivot_x + STOP_PIN_R * math.cos(alpha_rad)
    z = LEVER_PIVOT_Z + STOP_PIN_R * math.sin(alpha_rad)
    return (
        cq.Workplane("XY")
        .circle(STOP_PIN_SUPPORT_D / 2)
        .extrude(HOUSING_W)
        .rotate((0, 0, 0), (1, 0, 0), -90)
        .translate((x, -HOUSING_W / 2, z))
    )

def _brake_pin_local():
    """Pin built in a LOCAL frame for readability:
      +y_local = pin axis, pointing from the exposed tip INTO the housing.
      +x_local = radial-outward (spring leg through-hole passes along ±x).
      +z_local = tangential.
    The pin is assembled in this frame, then rotated/translated into world
    position by the callers (pin-solid vs housing hole-cutter both share
    the transform, so the fit is defined once)."""
    r     = STOP_PIN_D / 2
    L_exp = STOP_PIN_H
    L_ins = BRAKE_PIN_INSERT_DEPTH
    L_cone = BRAKE_PIN_CONE_H

    # Straight body: exposed + insert shank, y_local = 0..L_exp+L_ins.
    # Using makeCylinder with an explicit +y direction (Workplane("XZ")'s
    # extrude normal is -y in cadquery, not +y, which would put the body
    # on the wrong side of the origin).
    body = cq.Solid.makeCylinder(
        r, L_exp + L_ins,
        pnt=cq.Vector(0, 0, 0),
        dir=cq.Vector(0, 1, 0),
    )
    # Cone tip: full OD starts slightly INSIDE the body (0.01 mm overlap)
    # so the union is a clean single solid — OCCT sometimes leaves tangent
    # surfaces as disjoint. Apex at y_local = L_exp+L_ins+L_cone.
    cone = cq.Solid.makeCone(
        r, 0.0, L_cone + 0.01,
        pnt=cq.Vector(0, L_exp + L_ins - 0.01, 0),
        dir=cq.Vector(0, 1, 0),
    )
    pin = cq.Workplane("XY").add(body).union(cq.Workplane("XY").add(cone))

    # D-flat on +x side, spanning the ENTIRE buried section (shank + cone).
    # If the flat stopped at the shank/cone junction, the cone would keep
    # its full r_nom radius on +x and collide with the hole's flat wall
    # during insertion — the cone would be unable to pass through the
    # keyed entrance. Extending the flat through the cone makes the cone
    # a "D-cone" whose +x max shrinks linearly from (r − FLAT_DEPTH) at
    # the base to 0 at the apex, matching the hole's D-profile all the
    # way down. y_local = L_exp..L_exp+L_ins+L_cone.
    flat_cutter = (
        cq.Workplane("XY")
        .box(2 * r, L_ins + L_cone, 2 * r, centered=(False, False, True))
        .translate((r - BRAKE_PIN_FLAT_DEPTH, L_exp, 0))
    )
    pin = pin.cut(flat_cutter)

    # Diametral through-hole (for the spring leg), along ±x_local, at
    # y_local = hole_y_local. hole_y_world = -(HOUSING_W/2 + LEVER_RIM_H/2)
    # = gap center; world pin tip is at y = -HOUSING_W/2 - STOP_PIN_H, so
    # hole_y_local = hole_y_world - pin_tip_world = STOP_PIN_H - LEVER_RIM_H/2.
    hole_y_local = STOP_PIN_H - LEVER_RIM_H / 2
    through = cq.Solid.makeCylinder(
        STOP_PIN_HOLE_D / 2, 2 * r + 0.4,
        pnt=cq.Vector(-r - 0.2, hole_y_local, 0),
        dir=cq.Vector(1, 0, 0),
    )
    return pin.cut(cq.Workplane("XY").add(through))

def _brake_hole_local():
    """Housing-hole cutter in the same LOCAL frame as _brake_pin_local().
    Oversized everywhere by BRAKE_PIN_HOLE_CLR/2 (per side) for a glue-
    retained slip fit. Spans only the portion BURIED in the housing
    (insert shank + cone tip); the exposed portion doesn't exist in the
    hole."""
    r_nom = STOP_PIN_D / 2
    r     = r_nom + BRAKE_PIN_HOLE_CLR / 2
    L_ins  = BRAKE_PIN_INSERT_DEPTH
    L_cone = BRAKE_PIN_CONE_H

    # Shank hole: y_local = L_exp - eps .. L_exp + L_ins (start slightly
    # before the face for a clean boolean against the housing surface).
    L_exp = STOP_PIN_H
    eps = 0.05
    shank = cq.Solid.makeCylinder(
        r, L_ins + eps,
        pnt=cq.Vector(0, L_exp - eps, 0),
        dir=cq.Vector(0, 1, 0),
    )
    cone = cq.Solid.makeCone(
        r, 0.0, L_cone + 0.01,
        pnt=cq.Vector(0, L_exp + L_ins - 0.01, 0),
        dir=cq.Vector(0, 1, 0),
    )
    hole = cq.Workplane("XY").add(shank).union(cq.Workplane("XY").add(cone))

    # Matching D-flat: leave housing material on the +x side of the hole.
    # The pin's flat sits at x = r_nom - FLAT_DEPTH; the hole's flat wall
    # sits at x = r_nom - FLAT_DEPTH + CLR/2 (slightly outboard, giving
    # clearance). Cut that material out of the hole-cutter so it won't be
    # removed from the housing. The flat spans shank + cone (mirroring
    # the pin's D-cone) so the whole insertion path is keyed.
    flat_keep = (
        cq.Workplane("XY")
        .box(2 * r + 1, L_ins + L_cone + 2 * eps, 2 * r + 1, centered=(False, False, True))
        .translate((r_nom - BRAKE_PIN_FLAT_DEPTH + BRAKE_PIN_HOLE_CLR / 2,
                    L_exp - eps, 0))
    )
    return hole.cut(flat_keep)

def _brake_pin_place(part):
    """Shared transform from LOCAL frame to WORLD for the brake pin / hole.
    Local +x (radial) → world (cos α, 0, sin α) via rotation around Y by
    -α. Local +y (pin axis, exposed→deep) maps to world +y (from y=-13
    at exposed tip toward y=-8.25 at cone apex)."""
    alpha = STOP_HOUSING_PIN_ALPHA_BRAKE_DEG
    alpha_rad = math.radians(alpha)
    x_pin = BRAKE_PIVOT_X + STOP_PIN_R * math.cos(alpha_rad)
    z_pin = LEVER_PIVOT_Z + STOP_PIN_R * math.sin(alpha_rad)
    y_tip = -HOUSING_W / 2 - STOP_PIN_H
    return (part
            .rotate((0, 0, 0), (0, 1, 0), -alpha)
            .translate((x_pin, y_tip, z_pin)))

brake_housing_pin = _brake_pin_place(_brake_pin_local())

lever_housing = (
    top_plate.union(bot_plate).union(spine)
    # Ratchet pivot: clearance through from +y (lever side), insert on -y
    .cut(_pivot_clearance(RATCHET_PIVOT_X))
    .cut(_pivot_insert_pilot(RATCHET_PIVOT_X, insert_face_sign=-1))
    # Brake pivot: clearance through from -y (lever side), insert on +y
    .cut(_pivot_clearance(BRAKE_PIVOT_X))
    .cut(_pivot_insert_pilot(BRAKE_PIVOT_X,  insert_face_sign=+1))
    # Housing stop pins — project into each lever's rim gap. Lever stop
    # pin rotates to meet these at the travel limit for that lever.
    .union(_stop_pin(RATCHET_PIVOT_X, STOP_HOUSING_PIN_ALPHA_RATCHET_DEG,
                     HOUSING_W / 2, HOUSING_W / 2 + STOP_PIN_H,
                     hole_y=+(HOUSING_W / 2 + LEVER_RIM_H / 2)))
    # Raised support ridge coaxial with each housing stop pin — covers
    # the pin where it projects above the column's flat top and extends
    # that projection across the full column length in y as a visible
    # arched rib.
    .union(_stop_pin_support(RATCHET_PIVOT_X, STOP_HOUSING_PIN_ALPHA_RATCHET_DEG))
    .union(_stop_pin_support(BRAKE_PIVOT_X,   STOP_HOUSING_PIN_ALPHA_BRAKE_DEG))
    # Brake-side housing stop pin is printed separately and glued in
    # after printing (see BRAKE_PIN_* constants above). Cut AFTER the
    # support-ridge union — otherwise the ridge (3 mm OD, coaxial with
    # the pin) would fill the just-cut hole back in with solid material.
    .cut(_brake_pin_place(_brake_hole_local()))
    # Two wall-mount stud holes through the spine, on a diagonal: one
    # on the lever-side end at -y, one on the far-side end at +y. A
    # 2x2 pattern on this 22 mm wide spine would collide the 12.7 mm
    # flanges (they can't fit side-by-side at any y that also keeps
    # them inside the spine edges). Diagonal spacing is 39 mm so the
    # flanges are well clear of each other.
    .cut(_mount_hole(-MOUNT_SCREW_EDGE_Y, MOUNT_SCREW_LEVER_Z))
    .cut(_mount_hole(+MOUNT_SCREW_EDGE_Y, MOUNT_SCREW_FAR_Z))
)

# ────────────────────────────────────────────────────────────────────────────
# LEVERS — geometric layout for matched ROM
#
# Both arms are HORIZONTAL at rest (90° from pull direction). User pushes
# the bottom face of each lever toward the spool (post-flip +z = pre-flip
# −z), which drops the handle end in pre-flip coords and actuates the
# contact on the other end.
#
# RATCHET (class-1): pivot at x=76, pawl catch edge at (x≈66.4, z=36.5),
#   handle at (x=115, z=50.5). Pivot BETWEEN pawl and handle, so they
#   move in opposite z directions — pulling the handle down lifts the
#   pawl up off the teeth. Horizontal arms:
#     pawl arm = 76 − 66.4 = 9.6 mm,  handle arm = 115 − 76 = 39 mm
#     throw ratio (handle/pawl) = 39/9.6 = 4.06
#
# BRAKE (class-2): pivot at x=55, pad midpoint at (x=74.47, z=38), handle
#   at (x=115, z=50.5). Pad and handle BOTH on +x of pivot, same motion —
#   pulling the handle down pushes the pad down onto the surface.
#   Horizontal arms:
#     pad arm = 74.47 − 55 = 19.47 mm,  handle arm = 115 − 55 = 60 mm
#     throw ratio (handle/pad) = 60/19.47 = 3.08
#
# Matched ROM (small-angle, horizontal-arm approximation):
#   Per mm of handle drop: pawl rises 9.6/39 = 0.246 mm,
#                          pad descends 19.47/60 = 0.3245 mm.
#   The ratchet pivot x=76 is tuned so P_b/P_r = 1.32 — matching the
#   2 mm:1.5 mm ratio of (pad rest lift : pawl tooth depth), which is
#   what makes transition simultaneous.
#   - At rest:    pawl seated in valley (z=36.5, engaged); pad lifted
#                 1.98 mm above surface (pad_rest_z = 39.98, disengaged).
#   - Transition: at handle drop ≈ 6.09 mm, pawl has risen 1.5 mm to reach
#                 tooth tip (z=38) AND pad has descended 1.98 mm to touch
#                 surface (z=38). Disconnect and start-of-brake coincide.
#   - Max pull:   handle drop LEVER_HANDLE_TRAVEL_MAX ≈ 8.12 mm → pawl
#                 0.5 mm above tip (headroom), pad 0.66 mm compressed
#                 into the rubber. A handle stop on the housing will
#                 enforce this position (next subtask).
#
# Note: contacts are drawn at their REST positions. Ratchet pawl sits in
# the tooth valley (rest = engaged for the ratchet). Brake pad sits
# PAD_REST_Z_BOT − SPOOL_H = 1.98 mm above the brake surface (rest =
# disengaged for the brake). At max pull, the brake arm rotates the pad
# down to (and slightly past) the surface; the pad tilts ~8° with the
# arm over that travel, which is within the rubber's compliance range.
# ────────────────────────────────────────────────────────────────────────────

# Kinematic constants for reference by subsequent subtasks (stop geometry,
# spring return). Small-angle horizontal-arm approximation.
PAWL_REST_Z_BOT        = SPOOL_H - RATCHET_DEPTH    # 36.5 — pawl at rest (engaged)
PAWL_HEADROOM          = 0.5                        # lift beyond tooth tip at max-pull
PAD_REST_Z_BOT         = 39.98                      # pad lifted at rest (disengaged)
LEVER_HANDLE_TRAVEL_MAX = 8.12                      # max handle z-travel from rest

# Thickness of the rubber pad adhered to the bottom of the printed brake
# pad block (McMaster 86495K36 Buna-N, 70A, 3 mm, cut to the pad
# footprint and bonded with your own adhesive). The printed pad stops
# BRAKE_RUBBER_T above PAD_REST_Z_BOT so that (printed material) +
# (rubber) = the full rest height — the geometric-layout math stays
# valid regardless of this value, so adjust freely if the actual
# measured thickness differs from spec.
BRAKE_RUBBER_T         = 3.0

LEVER_W              = 11.0    # grip width, perpendicular to pull (in y)
LEVER_T              =  2.0    # thickness in z (swing plane direction)
LEVER_PIVOT_HOLE_D   =  2.3    # M2 clearance (2 mm screw + 0.3 clearance)
LEVER_Z_CENTER       = LEVER_PIVOT_Z                 # 53
LEVER_Z_BOT          = LEVER_Z_CENTER - LEVER_T / 2  # 52 (lever body bottom)
LEVER_Z_TOP          = LEVER_Z_CENTER + LEVER_T / 2  # 54 (lever body top)

# Lever spacer rim — small cylindrical collar around the pivot hole on the
# lever's housing-facing face. Creates a tiny bearing surface against the
# housing so the lever's broad face doesn't rub as it rotates. Small OD =
# small contact area = less friction. (LEVER_RIM_OD / LEVER_RIM_H
# defined earlier, with the housing-level constants.)

# Each lever sits on its side of the housing with a 1 mm air gap (the rim
# spans this gap, contacting the housing face). 11 mm of lever grip then
# extends outward.
_LEVER_INNER_Y       = HOUSING_W / 2 + LEVER_RIM_H   # 12

RATCHET_LEVER_Y0     = _LEVER_INNER_Y                # 12 (+y side, inner)
RATCHET_LEVER_Y1     = RATCHET_LEVER_Y0 + LEVER_W    # 23
BRAKE_LEVER_Y1       = -_LEVER_INNER_Y               # -12 (-y side, inner)
BRAKE_LEVER_Y0       = BRAKE_LEVER_Y1 - LEVER_W      # -23

# Lever x bounds — these define the extent of each lever's flat body:
#   RATCHET (class-1): body from pawl-side edge x≈60 through pivot x=83
#     to handle x=115 (55 mm). Must extend past the pivot on the -x side
#     to hold the pawl — pawl footprint's innermost corner is at x≈62.
#   BRAKE (class-2): body from pivot x=55 to handle x=115 (60 mm). The
#     pad at x≈74 is within this span, so the body alone encloses the
#     pad region.
RATCHET_LEVER_X_PAWL_SIDE =  60.0
RATCHET_LEVER_X_HANDLE    = 115.0
BRAKE_LEVER_X_HANDLE      = 115.0

def _lever_body(x0, x1, y0, y1):
    """Flat lever tongue: length x 11 mm (y) x 2 mm (z)."""
    return (
        cq.Workplane("XY").workplane(offset=LEVER_Z_BOT)
        .center((x0 + x1) / 2, (y0 + y1) / 2)
        .box(x1 - x0, y1 - y0, LEVER_T, centered=(True, True, False))
    )

def _lever_rim(pivot_x, rim_side_y, direction):
    """Spacer rim on the housing-facing face. rim_side_y = lever's inner-face y.
    direction = +1 for +y lever (rim grows in −y toward housing) or −1 for −y
    lever (rim grows in +y toward housing). After rotation the cadquery
    extrude is always in +y, so we set start_y so that the extruded segment
    lies BETWEEN the lever face and the housing on each side."""
    if direction > 0:
        start_y = rim_side_y - LEVER_RIM_H        # extrude +y → rim_side_y
    else:
        start_y = rim_side_y                       # extrude +y → rim_side_y + RIM_H
    return (
        cq.Workplane("XY")
        .circle(LEVER_RIM_OD / 2)
        .extrude(LEVER_RIM_H)
        .rotate((0, 0, 0), (1, 0, 0), -90)
        .translate((pivot_x, start_y, LEVER_PIVOT_Z))
    )

def _lever_pivot_hole(pivot_x, y_start, y_end):
    """M2 clearance hole through the lever (plus through the rim)."""
    return (
        cq.Workplane("XY")
        .circle(LEVER_PIVOT_HOLE_D / 2)
        .extrude((y_end - y_start) + 1)
        .rotate((0, 0, 0), (1, 0, 0), -90)
        .translate((pivot_x, y_start - 0.5, LEVER_PIVOT_Z))
    )

# Local pivot boss — the lever is only 2 mm thick in z, but the M2 clearance
# hole is 2.3 mm wide, which would leave the hole open on the top and bottom
# faces. Add a thicker cylindrical boss around the pivot to give the hole
# real wall thickness in z.
LEVER_PIVOT_BOSS_OD  = 6.0     # 1.85 mm wall around the 2.3 mm hole
LEVER_PIVOT_BOSS_T   = 5.0     # z-thickness, replacing the 2 mm at the pivot

def _lever_pivot_boss(pivot_x, y0, y1):
    """Cylindrical boss centered on the pivot, axis along +y, spanning the
    lever's full grip width. Width 6 mm, thickness 5 mm in z."""
    return (
        cq.Workplane("XY")
        .circle(LEVER_PIVOT_BOSS_OD / 2)
        .extrude(y1 - y0)
        .rotate((0, 0, 0), (1, 0, 0), -90)
        .translate((pivot_x, y0, LEVER_PIVOT_Z))
    )

# Ratchet pawl contact — same "project the ring onto the lever" approach
# as the brake pad: the pawl footprint is the intersection of the toothed
# ring (r = FLANGE_INNER_ID/2..DRUM_ID/2 - PAWL_BRAKE_GAP) with the
# lever's y-strip, so the +y and -y side walls are flush with the lever
# edges. The bottom face is then carved to match the tooth ring's own
# sawtooth top surface, so the pawl nests into every valley it covers
# (providing the catch face at each tooth drop) and skims every tip it
# covers.
PAWL_BRAKE_GAP = 1.5    # radial gap between pawl outer edge and brake
                        # surface inner edge. Prevents pawl from dragging
                        # on the brake ring if there's any side play in
                        # the pivot / lever bearing.

# ── Dynamic ratchet tooth phase ──────────────────────────────────────────────
# Center a tooth boundary (HIGH→LOW step) at the angular midpoint of the
# ratchet pawl's footprint. The footprint is the intersection of the
# pawl's y-strip (inner lever edge to inner edge + LEVER_W) with the
# toothed ring (r_in=FLANGE_INNER_ID/2, r_out=DRUM_ID/2 - PAWL_BRAKE_GAP).
# Its angular midpoint is at the y-midline of the lever, r-midline of
# the ring. Computing the offset this way means any future move of the
# ratchet lever in y auto-updates the tooth phase.
_y_pawl_mid = HOUSING_W / 2 + LEVER_RIM_H + LEVER_W / 2
_r_pawl_mid = (FLANGE_INNER_ID / 2 + (DRUM_ID / 2 - PAWL_BRAKE_GAP)) / 2
_theta_pawl_mid_deg = math.degrees(
    math.asin(_y_pawl_mid / _r_pawl_mid)
)
_tooth_pitch_deg = 360.0 / RATCHET_TEETH
# Offset = the phase shift that places a tooth boundary (which, with
# offset=0, sits at i * pitch) right at _theta_pawl_mid_deg.
RATCHET_TOOTH_OFFSET_DEG = _theta_pawl_mid_deg % _tooth_pitch_deg

# Apply the ratchet teeth cut to main_body now that the phase is known.
main_body = main_body.cut(
    ratchet_cutter(RATCHET_TEETH,
                   r_in=FLANGE_INNER_ID / 2,
                   r_out=DRUM_ID / 2,
                   z_top=SPOOL_H,
                   depth=RATCHET_DEPTH,
                   theta_offset_deg=RATCHET_TOOTH_OFFSET_DEG)
)

def _ratchet_pawl_contact(y_near, y_far):
    """Pawl built as (lever-aligned annular-strip prism extending from
    valley floor up to the lever body bottom) MINUS (tooth material in
    that region). Result: sawtooth underside that fits the ring exactly,
    with vertical side walls flush with the lever y-edges."""
    import math
    r_in  = FLANGE_INNER_ID / 2
    r_out = DRUM_ID / 2 - PAWL_BRAKE_GAP
    z_valley = SPOOL_H - RATCHET_DEPTH
    z_tip    = SPOOL_H
    # Footprint corners — intersections of y = const with the two ring radii
    x_in_near  = math.sqrt(r_in**2  - y_near**2)
    x_out_near = math.sqrt(r_out**2 - y_near**2)
    x_in_far   = math.sqrt(r_in**2  - y_far**2)
    x_out_far  = math.sqrt(r_out**2 - y_far**2)
    y_mid = (y_near + y_far) / 2
    x_in_mid  = math.sqrt(r_in**2  - y_mid**2)
    x_out_mid = math.sqrt(r_out**2 - y_mid**2)

    prism = (
        cq.Workplane("XY").workplane(offset=z_valley)
        .moveTo(x_in_far, y_far)
        .lineTo(x_out_far, y_far)
        .threePointArc((x_out_mid, y_mid), (x_out_near, y_near))
        .lineTo(x_in_near, y_near)
        .threePointArc((x_in_mid, y_mid), (x_in_far, y_far))
        .close()
        .extrude(LEVER_Z_BOT - z_valley)
    )
    # Tooth material in z=z_valley..z_tip = full annulus minus the ratchet cutter
    tooth_material = (
        cyl(2 * r_out, z_tip - z_valley, z=z_valley)
        .cut(cyl(2 * r_in, z_tip - z_valley, z=z_valley))
        .cut(ratchet_cutter(RATCHET_TEETH,
                            r_in=r_in, r_out=r_out,
                            z_top=z_tip, depth=RATCHET_DEPTH,
                            theta_offset_deg=RATCHET_TOOTH_OFFSET_DEG))
    )
    return prism.cut(tooth_material)

# Ratchet lever — arm geometry is placeholder (handle-end half unchanged);
# new pawl contact sits at its engaged position, currently disconnected
# from the lever body (to be joined by the arm in the next subtask).
ratchet_lever = (
    _lever_body(RATCHET_LEVER_X_PAWL_SIDE, RATCHET_LEVER_X_HANDLE,
                RATCHET_LEVER_Y0, RATCHET_LEVER_Y1)
    .union(_ratchet_pawl_contact(RATCHET_LEVER_Y0, RATCHET_LEVER_Y1))
    .union(_lever_pivot_boss(RATCHET_PIVOT_X, RATCHET_LEVER_Y0, RATCHET_LEVER_Y1))
    .union(_lever_rim(RATCHET_PIVOT_X, RATCHET_LEVER_Y0, direction=+1))
    # Lever stop pin — projects into the rim gap toward the housing
    .union(_stop_pin(RATCHET_PIVOT_X, STOP_LEVER_PIN_ALPHA_RATCHET_DEG,
                     _LEVER_INNER_Y - STOP_PIN_H, _LEVER_INNER_Y,
                     hole_y=+(HOUSING_W / 2 + LEVER_RIM_H / 2)))
    .cut(_lever_pivot_hole(RATCHET_PIVOT_X,
                           RATCHET_LEVER_Y0 - LEVER_RIM_H,
                           RATCHET_LEVER_Y1))
)

# Brake pad contact — the portion of the brake annulus (r = 73..80) that
# lies directly under the lever's y-footprint. Think of it as the lever's
# projection dropped onto the brake ring:
#   - y sides are straight chords at the lever edges (y=-12 and y=-23).
#     The y=-12 edge is 1 mm from the housing face at y=-11 (set by the
#     lever rim gap), which is the clearance the user asked for.
#   - r sides are true arcs at the inner (r=73) and outer (r=80) ring radii,
#     so the pad exactly fills the lever-aligned strip of the brake surface.
#   - Bottom face sits at z = SPOOL_H (flat on the brake ring).
#   - Top face at LEVER_Z_BOT (arm connection in the next subtask).

def _brake_pad_contact(y_near, y_far):
    """Pad carved from the brake annulus by the strip y ∈ [y_far, y_near]
    in the +x half-plane. Straight edges at constant y (parallel to the
    housing); arc edges at the inner/outer ring radii. Uses threePointArc
    with an explicit midpoint to keep the arc direction unambiguous."""
    import math
    r_in  = DRUM_ID / 2
    r_out = FLANGE_OD / 2
    # 4 corners — intersections of y = const with each ring
    x_in_near  = math.sqrt(r_in**2  - y_near**2)
    x_out_near = math.sqrt(r_out**2 - y_near**2)
    x_in_far   = math.sqrt(r_in**2  - y_far**2)
    x_out_far  = math.sqrt(r_out**2 - y_far**2)
    # Arc midpoints at the y halfway between the two chords
    y_mid = (y_near + y_far) / 2
    x_in_mid  = math.sqrt(r_in**2  - y_mid**2)
    x_out_mid = math.sqrt(r_out**2 - y_mid**2)
    # Printed pad block at its REST position minus the rubber thickness:
    # the printed bottom sits at PAD_REST_Z_BOT + BRAKE_RUBBER_T, leaving
    # BRAKE_RUBBER_T of room below for the glued-on rubber pad. Total
    # height (rubber + printed block) = LEVER_Z_BOT − PAD_REST_Z_BOT, so
    # the kinematics are unchanged.
    pad_bot_z = PAD_REST_Z_BOT + BRAKE_RUBBER_T
    return (
        cq.Workplane("XY").workplane(offset=pad_bot_z)
        .moveTo(x_in_far, y_far)
        .lineTo(x_out_far, y_far)
        .threePointArc((x_out_mid, y_mid), (x_out_near, y_near))
        .lineTo(x_in_near, y_near)
        .threePointArc((x_in_mid, y_mid), (x_in_far, y_far))
        .close()
        .extrude(LEVER_Z_BOT - pad_bot_z)
    )

# Brake lever — arm geometry is placeholder (handle-end half unchanged);
# new flat pad sits at its engaged position on the brake ring, currently
# disconnected from the lever body.
brake_lever = (
    _lever_body(BRAKE_PIVOT_X, BRAKE_LEVER_X_HANDLE,
                BRAKE_LEVER_Y0, BRAKE_LEVER_Y1)
    .union(_brake_pad_contact(BRAKE_LEVER_Y1, BRAKE_LEVER_Y0))
    .union(_lever_pivot_boss(BRAKE_PIVOT_X, BRAKE_LEVER_Y0, BRAKE_LEVER_Y1))
    .union(_lever_rim(BRAKE_PIVOT_X, BRAKE_LEVER_Y1, direction=-1))
    # Lever stop pin — projects into the rim gap toward the housing
    .union(_stop_pin(BRAKE_PIVOT_X, STOP_LEVER_PIN_ALPHA_BRAKE_DEG,
                     -_LEVER_INNER_Y, -_LEVER_INNER_Y + STOP_PIN_H,
                     hole_y=-(HOUSING_W / 2 + LEVER_RIM_H / 2)))
    .cut(_lever_pivot_hole(BRAKE_PIVOT_X,
                           BRAKE_LEVER_Y0,
                           BRAKE_LEVER_Y1 + LEVER_RIM_H))
)

# ────────────────────────────────────────────────────────────────────────────
# TORSION SPRING (dummy / visualization only — not printed)
# Two springs, one per pivot. Coil approximated as a single-loop torus
# (0.5 mm wire, 4 mm ID, 5 mm OD) seated in the 1 mm rim gap around the
# lever rim. Each leg is a straight radial wire at the same angular
# position as its stop pin — the leg passes straight through the pin's
# diametral through-hole (STOP_PIN_HOLE_D). Inside the hole, the leg
# wire pushes tangentially against the hole walls as the spring tries
# to twist; that transmits torque into the pin, which is anchored to the
# lever (or housing). The pin's solid outer body still provides the
# mechanical stop via cylinder-to-cylinder tangent contact with the
# opposing pin. Shown in rest state.
# ────────────────────────────────────────────────────────────────────────────

SPRING_COIL_MAJOR_R = 1.875   # mean coil radius (ID 3, OD 4.5 per Lee
                              # Spring LTM*050E — rod 3 mm, coil OD 4.5 mm,
                              # wire 0.51 mm → mean r = (1.5 + 2.25)/2)
SPRING_LEG_LEN      = STOP_PIN_R + STOP_PIN_D / 2 - SPRING_COIL_MAJOR_R + 1.0
                              # start at coil centerline, reach 1 mm past
                              # the pin's outer edge so the exit side is
                              # visible

def _torsion_spring(pivot_x, y_center, lever_leg_alpha_deg, housing_leg_alpha_deg):
    """Dummy torsion spring at the given pivot, y centered in the rim gap.
    Coil axis along +y. Each leg is a straight radial cylinder from the
    coil centerline outward, passing through the matching stop pin's
    diametral through-hole and exiting 1 mm past the pin. Returned as a
    Compound (coil + two legs); no boolean union because the parts only
    touch tangentially. Visual reference only."""
    coil = cq.Solid.makeTorus(
        SPRING_COIL_MAJOR_R, SPRING_WIRE_R,
        pnt=cq.Vector(pivot_x, y_center, LEVER_PIVOT_Z),
        dir=cq.Vector(0, 1, 0),
    )
    parts = [coil]
    for alpha_deg in (lever_leg_alpha_deg, housing_leg_alpha_deg):
        alpha_rad = math.radians(alpha_deg)
        cos_a = math.cos(alpha_rad)
        sin_a = math.sin(alpha_rad)
        start = cq.Vector(
            pivot_x + SPRING_COIL_MAJOR_R * cos_a,
            y_center,
            LEVER_PIVOT_Z + SPRING_COIL_MAJOR_R * sin_a,
        )
        leg = cq.Solid.makeCylinder(
            SPRING_WIRE_R, SPRING_LEG_LEN,
            pnt=start,
            dir=cq.Vector(cos_a, 0, sin_a),
        )
        parts.append(leg)
    return cq.Compound.makeCompound(parts)

# Spring y centered in the rim gap (midway between housing face and lever
# inner face). Legs share α with the pins, so use the pin alphas directly.
_SPRING_Y_GAP_CENTER = (HOUSING_W / 2 + _LEVER_INNER_Y) / 2

ratchet_spring = _torsion_spring(
    RATCHET_PIVOT_X,
    +_SPRING_Y_GAP_CENTER,
    STOP_LEVER_PIN_ALPHA_RATCHET_DEG,
    STOP_HOUSING_PIN_ALPHA_RATCHET_DEG,
)
brake_spring = _torsion_spring(
    BRAKE_PIVOT_X,
    -_SPRING_Y_GAP_CENTER,
    STOP_LEVER_PIN_ALPHA_BRAKE_DEG,
    STOP_HOUSING_PIN_ALPHA_BRAKE_DEG,
)

# ────────────────────────────────────────────────────────────────────────────
# Orientation flip — mirror every part about z = SPOOL_H / 2 so that the
# tooth flange is at low z (facing -z) and the smooth flange is at high z.
# Everything above was designed with +z toward the teeth; this flip gives
# us the user's preferred convention where pulling the levers in +z is the
# direction from tooth-side to smooth-side.
# ────────────────────────────────────────────────────────────────────────────

def _flip_z(part):
    return part.mirror(mirrorPlane="XY", basePointVector=(0, 0, SPOOL_H / 2))

main_body     = _flip_z(main_body)
axle          = _flip_z(axle)
lever_housing = _flip_z(lever_housing)
ratchet_lever = _flip_z(ratchet_lever)
brake_lever   = _flip_z(brake_lever)
ratchet_spring = _flip_z(ratchet_spring)
brake_spring   = _flip_z(brake_spring)
brake_housing_pin = _flip_z(brake_housing_pin)
# Bearing cap: flip about its OWN center (z = CAP_H/2 = 4) so the standalone
# part still fits in a clean 0..8 z range, just with the lip and pocket
# swapped. Assembly position moves from z=CAP_SEAT_Z0 (=30) to z=0 since
# the cap seat in the flipped main_body is now at z=0..8.
bearing_cap   = bearing_cap.mirror(mirrorPlane="XY", basePointVector=(0, 0, CAP_H / 2))

# ────────────────────────────────────────────────────────────────────────────
# Export — individual parts for printing, + a combined assembly STEP for
# dimensional verification (bearings & spring not modelled, since they're
# purchased parts).
# ────────────────────────────────────────────────────────────────────────────

cq.exporters.export(main_body,    "spool_main_body.step")
print("Wrote spool_main_body.step")

cq.exporters.export(bearing_cap,  "bearing_cap.step")
print("Wrote bearing_cap.step")

cq.exporters.export(axle,         "axle.step")
print("Wrote axle.step")

cq.exporters.export(lever_housing,"lever_housing.step")
print("Wrote lever_housing.step")

cq.exporters.export(ratchet_lever, "ratchet_lever.step")
print("Wrote ratchet_lever.step")

cq.exporters.export(brake_lever,   "brake_lever.step")
print("Wrote brake_lever.step")

cq.exporters.export(ratchet_spring, "ratchet_spring.step")
print("Wrote ratchet_spring.step  (dummy — purchased torsion spring)")

cq.exporters.export(brake_spring,   "brake_spring.step")
print("Wrote brake_spring.step  (dummy — purchased torsion spring)")

cq.exporters.export(brake_housing_pin, "brake_housing_pin.step")
print("Wrote brake_housing_pin.step  (print separately — glue-install)")

assembly = (
    cq.Assembly(name="retractable_cable_spool")
    .add(main_body,     name="main_body")
    .add(bearing_cap,   name="bearing_cap",    loc=cq.Location((0, 0, 0)))
    .add(axle,          name="axle")
    .add(lever_housing, name="lever_housing")
    .add(ratchet_lever, name="ratchet_lever")
    .add(brake_lever,   name="brake_lever")
    .add(ratchet_spring, name="ratchet_spring")
    .add(brake_spring,   name="brake_spring")
    .add(brake_housing_pin, name="brake_housing_pin")
)
assembly.save("assembly.step")
print("Wrote assembly.step")
