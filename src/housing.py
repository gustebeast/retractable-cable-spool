"""Housing — U-bracket holding the spool and carrying both levers + the guide bearing.

Also defines the two separately-printed brake pins (``brake_housing_pin``,
``brake_housing_rest_pin``) since they share helpers with the housing-side
hole cuts.
"""

import math
import cadquery as cq

from .dimensions import (
    AXLE_D, AXLE_EXTRA_LEVER, AXLE_EXTRA_PANCAKE, AXLE_PRINT_D,
    BEARING_OD, BEARING_W,
    BOOL_OVERSHOOT,
    DRUM_OD,
    FLANGE_OD,
    HOUSING_GAP_LEVER, HOUSING_GAP_PANCAKE,
    M2_HEAD_RECESS_D, M2_HEAD_RECESS_H,
    M2_INSERT_DEPTH, M2_INSERT_PILOT_D, M2_SHAFT_CLR_D,
    PANCAKE_CROSS_PIN_Z, PLATE_T,
    SPOOL_H,
)
from .helpers import cyl, cone_solid

# ────────────────────────────────────────────────────────────────────────────
# HOUSING — U-bracket keyed only at the top of the axle.
#
# Two 11 mm plates (top + bottom) connected by a 2 mm spine. The plates are
# thick enough to carry the mounted-cantilever bending load directly — no
# local axle/pivot columns. Each plate sits HOUSING_GAP_LEVER /
# HOUSING_GAP_PANCAKE away from the nearest bearing; thickness grows OUTWARD
# (away from the spool) so the bearing gap is unchanged.
#
# Lever pivots and axle cross-pins are M2 screws driven in y (perpendicular
# to plate thickness), so the 11 mm plate just provides material for the
# clearance hole / heat-set insert. Stop-pin support bosses (top plate,
# inward-facing) and the guide-bearing sleeve (bottom plate, inward-facing)
# remain — they project INTO the lever / pancake space and don't depend on
# plate thickness.
# ────────────────────────────────────────────────────────────────────────────

HOUSING_W         =  22.0    # matches bearing OD
HOUSING_CLR       =  10.0    # radial clearance past flange OD
HOUSING_SPINE_T   =   2.0    # spine thickness (x direction)
HOUSING_HOLE_CLR  =   0.15   # axle hole slip fit (radial)

SPINE_X_INNER     = FLANGE_OD / 2 + HOUSING_CLR         # 90 — 10 mm past flange OD
SPINE_X_OUTER     = SPINE_X_INNER + HOUSING_SPINE_T     # 92
# Wrap-around: back spine on the opposite side of the housing, mirroring
# the front (mounting) spine. Plates extend across the full width from
# -SPINE_X_INNER to +SPINE_X_INNER, with a spine box at each end.
# With the back spine removed, plates are trimmed asymmetrically:
#   - Lever-side plate starts at LEVER_HOUSING_X_TAIL = 57: 2 mm past the
#     -x edge of the brake-lever pivot boss (BRAKE_PIVOT_X − boss_radius
#     = 62 − 3 = 59). Everything inboard of that is gone, including the
#     lever-side axle cross-pin (axle is now held only by the pancake-
#     side cross-pin + the bottom bearing).
#   - Pancake-side plate starts at PANCAKE_HOUSING_X_TAIL = -6: 2 mm past
#     the axle hole's -x edge (axle radius 4 → -x edge at -4). Keeps the
#     axle hole + cross-pin.
LEVER_HOUSING_X_TAIL   = 57.0
PANCAKE_HOUSING_X_TAIL = -6.0

# Lever pivot Z — preserved at the original position (2.5 mm into the plate
# from its inner face) so the lever kinematics stay unchanged when the
# plate thickens from 2 mm + 5 mm column to a uniform 11 mm slab.
_LEVER_PIVOT_OFFSET_FROM_INNER = 2.5

# Lever pivot x-positions — defined here so subsequent helpers can reference
# them.
# Pulling direction: user pushes the bottom face of the lever toward the
# spool (in +Z direction, since the lever sits below the spool body at
# negative Z). For that pull to DISENGAGE the ratchet and ENGAGE the brake:
#   RATCHET is class-1 (pivot BETWEEN pawl and handle; opposite motions).
#     Pawl catch edge at x ≈ _RATCHET_PAWL_X_INNER (= 60.4); handle at
#     x = SPINE_X_INNER + LEVER_GRIP_OVERHANG (= 115). Handle drops →
#     pawl rises off teeth. Pivot position is TUNED so that by the
#     moment the pawl clears the tooth tip, the brake pad has descended
#     exactly its rest-lift distance (2 mm). The ~76 value below comes
#     from that kinematic constraint (matched experimentally), not from
#     a clean geometric ratio.
#   BRAKE is class-2 (pad and handle BOTH on +x of pivot; same motions).
#     Pad midpoint at x ≈ DRUM_OD/2 - PAWL_BRAKE_GAP - rubber/2 (≈74.47);
#     handle at LEVER_GRIP_OVERHANG past the spine. Handle drops → pad
#     descends onto the brake ring. Pivot position TUNED for the
#     handle-throw to match the ratchet's, paired with the chosen lever
#     ratio.
RATCHET_PIVOT_X   = 70.0   # was 76 — moved inboard to clear the new lever-side
                           # guide bearing, which sits at x≈73..81 (between the
                           # levers, riding on the brake track at z=0). Throw
                           # ratio increases (handle/pawl ≈ 4.7 vs ~2.5 before),
                           # i.e., the user moves the handle a bit more for the
                           # same pawl rise. Kinematic match to the brake side
                           # has shifted; revisit if engagement timing matters.
BRAKE_PIVOT_X     = 62.0   # was 55 — moved to maintain the matched ROM
                           # constraint when the ratchet pivot moved 76 → 70.
                           # New ratio (pawl_arm / pad_arm = 9.55 / 11.5 = 0.83)
                           # is the same +11% deviation from ideal that the
                           # original (76, 55) design had, so engagement timing
                           # feel matches the prior prototype. Engagement angle
                           # is now ~9.5° vs the old ~6° (more lever throw to
                           # the same engagement state, but ratchet/brake still
                           # transition together).

# Derived z-coordinates: LEVER side sits below the spool body (Z < 0),
# PANCAKE side sits above (Z > SPOOL_H). _Z_IN is the spool-facing face
# of each plate; _Z_OUT is the away-from-spool face.
_LEVER_PLATE_Z_IN   = -HOUSING_GAP_LEVER                            # -10 — plate inner face (lever side)
_LEVER_PLATE_Z_OUT  = _LEVER_PLATE_Z_IN - PLATE_T                     # -21 — plate outer face
PANCAKE_PLATE_Z_IN   = SPOOL_H + HOUSING_GAP_PANCAKE                 # 53 — plate inner face (pancake side)
PANCAKE_PLATE_Z_OUT  = PANCAKE_PLATE_Z_IN + PLATE_T                     # 64 — outer face


def _top_plate():
    """11 mm slab — lever-side plate. Trimmed to start at
    LEVER_HOUSING_X_TAIL (= 57, just past the brake-pivot boss's -x edge)."""
    return (
        cq.Workplane("XZ")
        .moveTo(LEVER_HOUSING_X_TAIL, _LEVER_PLATE_Z_IN)
        .lineTo(SPINE_X_INNER,        _LEVER_PLATE_Z_IN)
        .lineTo(SPINE_X_INNER,        _LEVER_PLATE_Z_OUT)
        .lineTo(LEVER_HOUSING_X_TAIL, _LEVER_PLATE_Z_OUT)
        .close()
        .extrude(HOUSING_W / 2, both=True)
    )


def _bot_plate():
    """11 mm slab — pancake-side plate. Trimmed to start at
    PANCAKE_HOUSING_X_TAIL (= -6, 2 mm past the axle hole's -x edge)."""
    return (
        cq.Workplane("XZ")
        .moveTo(PANCAKE_HOUSING_X_TAIL, PANCAKE_PLATE_Z_IN)
        .lineTo(SPINE_X_INNER,          PANCAKE_PLATE_Z_IN)
        .lineTo(SPINE_X_INNER,          PANCAKE_PLATE_Z_OUT)
        .lineTo(PANCAKE_HOUSING_X_TAIL, PANCAKE_PLATE_Z_OUT)
        .close()
        .extrude(HOUSING_W / 2, both=True)
    )

# Round axle hole — slip fit through the full plate thickness, sized to the
# printed axle Ø (not bearing-bore nominal). The cross-pin (further down)
# keys the axle against rotation; rotational keying via the hole shape
# isn't needed.
def _axle_round_hole(z_base, height):
    return cyl(AXLE_PRINT_D + 2 * HOUSING_HOLE_CLR, height, z=z_base)

# Lever-side plate: NO axle hole — the trimmed plate doesn't reach x=0
# anymore, so there's no plate material at the axle's z-range to cut.
top_plate = _top_plate()
# Pancake-side plate: still needs the axle hole at x=0.
bot_plate = (
    _bot_plate()
    .cut(_axle_round_hole(PANCAKE_PLATE_Z_IN, PLATE_T))
)

spine = (
    cq.Workplane("XY").workplane(offset=_LEVER_PLATE_Z_OUT)
    .center((SPINE_X_INNER + SPINE_X_OUTER) / 2, 0)
    .box(HOUSING_SPINE_T, HOUSING_W,
         PANCAKE_PLATE_Z_OUT - _LEVER_PLATE_Z_OUT,
         centered=(True, True, False))
)

# Back spine + back-spine cable hole removed — the source cable now exits
# the housing through the lever-side end of the axle (cap + cable hole, to
# be added). Plates are trimmed to HOUSING_X_TAIL = -10 instead of -90.

# Wall-mount stud — single clearance hole through the front spine, on the
# face that sits against the mounting surface (+x face). Install method:
# 10-24 wood-screw stud (McMaster 90915A641) drives into the mounting
# board, the housing slides on, and a 10-24 flange nut (McMaster 90389A112,
# flange OD 12.7 mm) tightens on the +x face. Single hole at the centroid
# of the front face — y = 0, z = midpoint between plate outer faces. (Eventual
# top-mount conversion will replace this with a hole through the top of
# the axle column instead; for now this just consolidates the previous
# 2-hole diagonal pattern into one centered hole.)
MOUNT_SCREW_CLR_D    = 5.3                                    # #10 clearance
MOUNT_SCREW_Z        = (_LEVER_PLATE_Z_OUT + PANCAKE_PLATE_Z_OUT) / 2

def _mount_hole(y, z):
    """#10 clearance hole through the spine, axis along x. Enters from the
    +x face (mounting face); extends slightly past both faces for clean
    booleans."""
    x_start = SPINE_X_INNER - BOOL_OVERSHOOT                  # 0.5 mm inside
    x_len   = HOUSING_SPINE_T + 2 * BOOL_OVERSHOOT             # through + overshoot each side
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
LEVER_PIVOT_Z         = _LEVER_PLATE_Z_IN - _LEVER_PIVOT_OFFSET_FROM_INNER   # -12.5 — preserves lever kinematics from the 2 mm + 5 mm column era
# M2 hardware constants — defined once at the top of the file under the
# axle/cross-pin section. Aliased here for the lever-pivot code which
# was originally written before the shared constants existed.
LEVER_SCREW_CLR_D     = M2_SHAFT_CLR_D
LEVER_INSERT_PILOT_D  = M2_INSERT_PILOT_D
LEVER_INSERT_DEPTH    = M2_INSERT_DEPTH
LEVER_INSERT_CHAMFER_H = (LEVER_INSERT_PILOT_D - LEVER_SCREW_CLR_D) / 2  # 0.4 — 45° chamfer
                                # at the inner end of the pilot so the
                                # pilot→clearance step prints as a taper
                                # instead of a horizontal overhang ledge.

SCREW_HEAD_RECESS_D   = M2_HEAD_RECESS_D
SCREW_HEAD_RECESS_H   = M2_HEAD_RECESS_H     # depth of the counterbore on the screw-
                                # ENTRY face (opposite the insert). Two
                                # purposes: (1) hides the screw head
                                # below the housing surface, (2) buys 2
                                # mm of additional reach so the 20 mm
                                # screw can fully engage the insert at
                                # the far face of the 22 mm-wide housing.

def _pivot_clearance(x):
    """M2 through-hole (axis along y), diameter 2.5 mm, full housing width."""
    return (
        cq.Workplane("XY")
        .circle(LEVER_SCREW_CLR_D / 2)
        .extrude(HOUSING_W + 2)
        .rotate((0, 0, 0), (1, 0, 0), -90)
        .translate((x, -HOUSING_W / 2 - 1, LEVER_PIVOT_Z))
    )

def _screw_head_counterbore(x, z, entry_face_sign):
    """4 mm × 2 mm counterbore on a y face of the housing for the M2 cap-
    screw head. entry_face_sign = +1 cuts from the +y face, -1 from -y.
    Sized to recess the head below the surface AND give the screw shaft
    2 mm of extra reach to fully engage the insert on the opposite face."""
    cb = (
        cq.Workplane("XY")
        .circle(SCREW_HEAD_RECESS_D / 2)
        .extrude(SCREW_HEAD_RECESS_H)
    )
    if entry_face_sign > 0:
        return cb.rotate((0, 0, 0), (1, 0, 0), -90) \
                 .translate((x, HOUSING_W / 2 - SCREW_HEAD_RECESS_H, z))
    return cb.rotate((0, 0, 0), (1, 0, 0), +90) \
             .translate((x, -HOUSING_W / 2 + SCREW_HEAD_RECESS_H, z))


def _pivot_insert_pilot(x, insert_face_sign, recess_depth=0.0):
    """M2 insert pilot hole, drilled from one face of the housing (+y or -y,
    per insert_face_sign = +1 or -1). Straight 3.2 mm pilot for 5 mm, then a
    45° cone taper from 3.2 → 2.3 at the inner end so the pilot→clearance
    step prints as a self-supporting slope instead of a horizontal ledge.
    recess_depth shifts the pilot mouth inward from the face — used when
    the same face also has a counterbore for the screw head."""
    pilot = (
        cq.Workplane("XY")
        .circle(LEVER_INSERT_PILOT_D / 2)
        .extrude(LEVER_INSERT_DEPTH)
        .union(cone_solid(LEVER_INSERT_PILOT_D, LEVER_SCREW_CLR_D,
                          LEVER_INSERT_CHAMFER_H, LEVER_INSERT_DEPTH))
    )
    if insert_face_sign > 0:
        return pilot.rotate((0, 0, 0), (1, 0, 0), +90) \
                    .translate((x, HOUSING_W / 2 - recess_depth, LEVER_PIVOT_Z))
    else:
        return pilot.rotate((0, 0, 0), (1, 0, 0), -90) \
                    .translate((x, -HOUSING_W / 2 + recess_depth, LEVER_PIVOT_Z))


# ── Axle cross-pin (housing-side cuts) ───────────────────────────────────
# Mirror of the lever pivot pattern, applied to each axle column. A single
# M2 cap screw passes through the column on +y, through the axle, and into
# a heat-set insert in the column on -y.
def _axle_pin_clearance(z_center):
    """M2 through-hole at a given z, axis along y, full housing y-width."""
    return (
        cq.Workplane("XY")
        .circle(LEVER_SCREW_CLR_D / 2)
        .extrude(HOUSING_W + 2)
        .rotate((0, 0, 0), (1, 0, 0), -90)
        .translate((0, -HOUSING_W / 2 - 1, z_center))
    )


def _axle_pin_insert_pilot(z_center, insert_face_sign, recess_depth=0.0):
    """Heat-set insert pilot. recess_depth shifts the pilot's mouth inward
    from the housing face by that many mm — used when the same face also
    has a counterbore for the screw head, so the insert sits BEHIND the
    counterbore instead of conflicting with it."""
    pilot = (
        cq.Workplane("XY")
        .circle(LEVER_INSERT_PILOT_D / 2)
        .extrude(LEVER_INSERT_DEPTH)
        .union(cone_solid(LEVER_INSERT_PILOT_D, LEVER_SCREW_CLR_D,
                          LEVER_INSERT_CHAMFER_H, LEVER_INSERT_DEPTH))
    )
    if insert_face_sign > 0:
        return pilot.rotate((0, 0, 0), (1, 0, 0), +90) \
                    .translate((0, HOUSING_W / 2 - recess_depth, z_center))
    else:
        return pilot.rotate((0, 0, 0), (1, 0, 0), -90) \
                    .translate((0, -HOUSING_W / 2 + recess_depth, z_center))

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
# are needed. The housing-pin angles below are computed from STOP_PIN_D and
# STOP_PIN_R so they auto-track changes to either; the angular separation
# at first contact is the chord/circle relation
#   2 · STOP_PIN_R · sin(Δθ/2) = STOP_PIN_D
# (pin outer edges tangent → centers separated by STOP_PIN_D).
STOP_PIN_D       = 4.0     # mm, pin OD. Mirrors the lever-pivot bolt-clearance
                           # geometry (which prints cleanly): 4 mm OD around a
                           # 2.5 mm hole gives 0.75 mm walls — printable with
                           # a 0.4 mm nozzle and strong enough not to break
                           # off. (Earlier 2 mm OD with a 1 mm cross hole left
                           # 0.5 mm walls, which collapsed during printing.)
STOP_PIN_HOLE_D  = 2.0     # mm, through-hole in pin for the spring leg. Way
                           # bigger than needed for the 0.51 mm wire — sized
                           # so the printed hole reliably forms (small holes
                           # in cylindrical walls are unreliable on FDM).
                           # On the lever pins this is the wall-to-wall
                           # opening of the rectangular section of the
                           # house-shaped (rotated teardrop) hole; the 45°
                           # peak adds STOP_PIN_HOLE_D/2 of length on top of
                           # that. Hole is perpendicular to pin axis, along
                           # the radial-from-pivot direction.
STOP_PIN_H       = 3.0     # mm, y-extent. With LEVER_RIM_H = 4.5 mm gap,
                           # each pin extends 3 mm from its own face into
                           # the gap, leaving 1.5 mm of air between its tip
                           # and the opposing surface. Pin overlap region
                           # = 1.5 mm in the middle of the gap.
STOP_PIN_R       = 4.5     # mm, radial offset from pivot. Pin inner edge at
                           # r=2.5 (= R − D/2) clears the spring coil outer
                           # at r=2.25 by 0.25 mm. Bumped from 3.5 when pin
                           # OD grew from 2 → 4 mm.
SPRING_WIRE_R    = 0.25    # radius of the spring wire (0.5 mm wire)

# Angular separation between two pin centers when their cylindrical OD
# surfaces just touch (chord = STOP_PIN_D on a circle of radius STOP_PIN_R).
_STOP_CONTACT_SEP_DEG = math.degrees(2 * math.asin(STOP_PIN_D / (2 * STOP_PIN_R)))

# Lever pin angular positions — at the pin (and therefore leg) α:
#   Ratchet: α=180° (pawl side of pivot)
#   Brake:   α=0°   (pad/handle side of pivot)
STOP_LEVER_PIN_ALPHA_RATCHET_DEG   = 180.0
STOP_LEVER_PIN_ALPHA_BRAKE_DEG     =   0.0
# Each lever has TWO housing pins — one is the SPRING pin (with through-
# hole for the spring leg, also acts as the limit of the spring-direction
# motion), the other is the REST pin (catches the lever pin at the design
# rest position so the spring's restoring force can't over-rotate it).
# For each, we set a TRAVEL_DEG that's the angular travel between rest
# (lever pin touching the rest pin) and the spring direction's max
# rotation (lever pin touching the spring pin).
RATCHET_OUTER_TRAVEL_DEG = 12.0   # disengagement travel — user pulls the
                                  # ratchet lever CW until lever pin
                                  # meets the spring pin
BRAKE_INNER_TRAVEL_DEG   = 12.0   # engagement travel — user pushes the
                                  # brake lever CW until lever pin meets
                                  # the spring pin (then rubber-pad
                                  # compression takes over)

# SPRING pins — positioned so the lever pin's outer edge meets the spring
# pin's outer edge at the intended end-of-travel angle:
#   Ratchet: spring pin sits CW of lever-pin-rest by (TRAVEL + SEP) so
#            after Δθ = −TRAVEL CW rotation the pins are just touching.
#   Brake:   spring pin sits CW of lever-pin-rest by (TRAVEL + SEP), same
#            relationship, mirrored to the brake's layout (engagement is
#            CW rotation for the brake just as disengagement is CW for
#            the ratchet — both spring pins are CW of lever pin).
STOP_HOUSING_PIN_ALPHA_RATCHET_DEG = (
    STOP_LEVER_PIN_ALPHA_RATCHET_DEG
    - RATCHET_OUTER_TRAVEL_DEG
    - _STOP_CONTACT_SEP_DEG
)
STOP_HOUSING_PIN_ALPHA_BRAKE_DEG = (
    STOP_LEVER_PIN_ALPHA_BRAKE_DEG
    - BRAKE_INNER_TRAVEL_DEG
    - _STOP_CONTACT_SEP_DEG
)

# REST pins — sit at touching distance CCW of the lever pin's rest
# position. Catch the lever pin at rest so the spring's restoring force
# can't over-rotate the lever past its design rest.
STOP_REST_PIN_ALPHA_RATCHET_DEG = (
    STOP_LEVER_PIN_ALPHA_RATCHET_DEG + _STOP_CONTACT_SEP_DEG
)
STOP_REST_PIN_ALPHA_BRAKE_DEG = (
    STOP_LEVER_PIN_ALPHA_BRAKE_DEG + _STOP_CONTACT_SEP_DEG
)

STOP_PIN_BOSS_WALL_T   = 1.0   # wall thickness around the brake pin's
                               # square+rounded-corner hole. Boss outer
                               # profile = pin profile grown radially by
                               # this amount, so the boss is a "fatter"
                               # version of the pin shape.
STOP_PIN_SUPPORT_D = 5.0    # OD of the raised support ridge coaxial with
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
BRAKE_PIN_INSERT_DEPTH = HOUSING_W / 2 - STOP_PIN_H   # 8 mm — total pin length
                                 # = HOUSING_W/2 (= 11 mm). The deeper shank
                                 # gives a much stiffer cantilever in the
                                 # column than the original 2 mm anchor.
BRAKE_PIN_HOLE_CLR     = 0.30    # diametral slip clearance (0.15 mm per side)
                                 # Was 0.30, but the square keying needs
                                 # more slack than a round hole to drop
                                 # in by hand: corners print fat, walls
                                 # bow inward. 0.25 mm of radial slack
                                 # per side.
BRAKE_PIN_HOLE_BASE_CLR = STOP_PIN_HOLE_D / 2 + 0.4   # 0.9 mm — distance from
                                 # the brake pin's BASE to the spring-leg
                                 # through-hole's centerline. Leaves 0.4 mm
                                 # of solid pin material between the hole's
                                 # far edge and the housing face.

# Lever–housing gap. The torsion-spring coil itself acts as the spacer:
# its body bears directly on both the lever inner face and the housing
# column outer face, with the M2 bolt threading through the coil ID and
# clamping the stack. No printed mandrel or rim — the coil ID (3 mm) wraps
# the M2 bolt (2 mm OD) directly with 0.5 mm radial slop. LEVER_RIM_H is
# therefore set by the spring body length.
LEVER_RIM_H          = 4.5     # axial gap between lever inner face and
                               # housing column outer face. Holds the
                               # SPRING_BODY_LEN-tall coil PLUS a
                               # LEVER_BOSS_EXTENSION-tall stub of bare
                               # lever pivot boss that physically fills
                               # the rest of the gap (so the boss
                               # bottoms out against the housing rather
                               # than the spring carrying any axial
                               # load). Sized so STOP_PIN_H = 3 mm
                               # leaves 1.5 mm of clear pin travel
                               # between lever and housing pin tips.
                               # Stop-pin geometry with this gap:
                               #   pin H = 3 mm each → overlap = 6 − 4.5
                               #   = 1.5 mm in the middle of the gap;
                               #   tip-to-opposite-face clearance = 1.5 mm
                               #   each side.

# Offset of each pin's spring-leg through-hole from the pin's BASE (the
# face the pin grows out of). Set so the hole's near edge is tangent to
# the base face — i.e. the hole sits as far toward the base as it can go
# without breaking through. Matches the spring coil itself, which is
# fully flush with both faces; the leg therefore enters its pin hole
# right at the face it emerges from.
STOP_PIN_HOLE_OFFSET = STOP_PIN_HOLE_D / 2

# Lever-pin spring-leg hole has a different positioning rule than the
# housing pin: it's a house-shaped (rotated teardrop) hole with the 45°
# peak pointing toward the pin's gap-end face. Two constraints set the
# geometry:
#   1. Peak ends LEVER_STOP_PIN_HOLE_PEAK_CLEARANCE mm shy of the pin's
#      tip face (keeps that much solid pin material at the tip).
#   2. Rectangle's far edge sits flush with the lever's inner face (so
#      the hole doesn't bite into the lever body).
# Together these fix the rectangle's y-length:
#   rect_y_len = STOP_PIN_H − peak_clearance − peak_h
# where peak_h = STOP_PIN_HOLE_D / 2 (45° peak with hole half-width).
LEVER_STOP_PIN_HOLE_PEAK_CLEARANCE = 0.8

# Lever stop-pin hole rectangle's y-length, derived so the far edge sits
# flush with the lever inner face while the peak still leaves
# PEAK_CLEARANCE before the pin's gap-end face.
LEVER_STOP_PIN_HOLE_RECT_Y_LEN = (
    STOP_PIN_H - LEVER_STOP_PIN_HOLE_PEAK_CLEARANCE - STOP_PIN_HOLE_D / 2
)
# Distance from the pin's gap-end face to the hole center (Y direction,
# toward the lever interior). Equals peak_clearance + half_y_rect + peak_h.
LEVER_STOP_PIN_HOLE_Y_FROM_GAP_END = (
    LEVER_STOP_PIN_HOLE_PEAK_CLEARANCE
    + LEVER_STOP_PIN_HOLE_RECT_Y_LEN / 2
    + STOP_PIN_HOLE_D / 2
)

# Tiny pin-into-lever-body overlap for clean booleans (interior overlap,
# not visible from outside). The lever pin's outer cylinder is no longer
# tangent to the lever body's flat top/bottom faces because STOP_PIN_D=4
# > LEVER_T=2, so no separate boss feature is needed.
_PIN_BOSS_OVERLAP = 0.5

def stop_pin_solid(pivot_x: float, alpha_deg: float,
                   y_from: float, y_to: float) -> cq.Workplane:
    """Plain cylindrical pin, no through-hole. Axis along y, OD STOP_PIN_D.
    Pin center at (x, z) = pivot + STOP_PIN_R·(cos α, sin α); pin extends
    in y from y_from to y_to."""
    alpha_rad = math.radians(alpha_deg)
    cos_a = math.cos(alpha_rad)
    sin_a = math.sin(alpha_rad)
    x = pivot_x + STOP_PIN_R * cos_a
    z = LEVER_PIVOT_Z - STOP_PIN_R * sin_a
    return (
        cq.Workplane("XY")
        .circle(STOP_PIN_D / 2)
        .extrude(y_to - y_from)
        .rotate((0, 0, 0), (1, 0, 0), -90)
        .translate((x, y_from, z))
    )

def stop_pin_hole(pivot_x: float, alpha_deg: float, hole_y: float,
                  cover_d: float = STOP_PIN_D,
                  teardrop: bool = False) -> cq.Workplane:
    """Diametral spring-leg through-hole CUTTER for a stop pin. Radial
    direction (cos α, 0, sin α), centered at y = hole_y. cover_d is the
    largest enclosing diameter the hole must span.

    When teardrop=False, makes a round cylindrical hole (OD STOP_PIN_HOLE_D).

    When teardrop=True, makes a printable teardrop: half-circle bottom +
    45° peaked top, so each printed layer has only a 45° overhang at the
    top and the hole forms reliably as a horizontal feature. Peak points
    in +z so the hole prints self-supporting. teardrop=True requires the
    hole axis to lie in the xy plane (sin α = 0), which holds for the
    lever pins at α = 0° and 180°. Housing pins (sin α ≠ 0) must use
    teardrop=False."""
    alpha_rad = math.radians(alpha_deg)
    cos_a = math.cos(alpha_rad)
    sin_a = math.sin(alpha_rad)
    x = pivot_x + STOP_PIN_R * cos_a
    z = LEVER_PIVOT_Z - STOP_PIN_R * sin_a
    hole_len = cover_d + 0.4

    if not teardrop:
        hole_start = cq.Vector(
            x - (cover_d / 2 + 0.2) * cos_a,
            hole_y,
            z - (cover_d / 2 + 0.2) * sin_a,
        )
        return cq.Workplane("XY").add(cq.Solid.makeCylinder(
            STOP_PIN_HOLE_D / 2, hole_len,
            pnt=hole_start,
            dir=cq.Vector(cos_a, 0, sin_a),
        ))

    assert abs(sin_a) < 1e-9, (
        "Teardrop stop_pin_hole only supports hole axes in xy plane "
        "(sin α = 0); use teardrop=False for housing pins.")
    r = STOP_PIN_HOLE_D / 2
    half_y_rect = LEVER_STOP_PIN_HOLE_RECT_Y_LEN / 2  # half of rectangle's y length
    peak_h = r                  # 45° peak — peak length equals hole half-width
    # House-shaped profile, rotated 90° from a "peak-up" orientation so the
    # peak points along the PIN AXIS (Y direction) toward the gap end of
    # the pin (= the housing side). Rectangular section: 2*r wall-to-wall
    # in z (= STOP_PIN_HOLE_D), and LEVER_STOP_PIN_HOLE_RECT_Y_LEN in y;
    # plus a 45° peak extending peak_h further along y toward the gap.
    #
    # peak_sign: −1 for ratchet (peak in −y, toward gap at lower y),
    #            +1 for brake (peak in +y, toward gap at higher y).
    # Determined by the sign of hole_y (positive for the +y lever, negative
    # for the −y lever).
    peak_sign = -1 if hole_y > 0 else +1
    profile = (
        cq.Workplane("YZ")
        .moveTo(-peak_sign * half_y_rect, -r)               # corner away from peak, bottom
        .lineTo(-peak_sign * half_y_rect, +r)               # corner away from peak, top
        .lineTo(+peak_sign * half_y_rect, +r)               # corner near peak, top
        .lineTo(+peak_sign * (half_y_rect + peak_h), 0)     # peak
        .lineTo(+peak_sign * half_y_rect, -r)               # corner near peak, bottom
        .close()
    )
    teardrop_solid = profile.extrude(hole_len)
    # YZ workplane extrudes in +x. Center the cutter on the pin's xz axis
    # so it spans pin_x ± hole_len/2 — covers both ends of the hole through
    # the pin regardless of whether α=0° or α=180°.
    return teardrop_solid.translate((x - hole_len / 2, hole_y, z))

def _stop_pin_support(pivot_x, alpha_deg, *, pin_z_sign=None,
                       trim_z_post=None, trim_keep_above=True,
                       neg_x_shift_local=None, pos_x_shift_local=None,
                       rect_local=None):
    """Raised support ridge coaxial with a housing stop pin, spanning the
    full column y width.

    pin_z_sign=None (default): a STOP_PIN_SUPPORT_D round cylinder. Used
    for the in-place printed ratchet pins, which are plain cylinders.

    pin_z_sign=±1: a "fatter pin" boss whose XZ profile matches the
    brake pin's square+rounded-corner shape, scaled outward by
    STOP_PIN_BOSS_WALL_T. The boss orients its rounded corner the same
    way as the pin it surrounds (so the boss's outer face mirrors the
    pin's outer face, giving a visually-coherent housing).

    rect_local: optional (width_x, width_z, center_dx, center_dz)
    tuple. Builds a simple axis-aligned RECTANGLE in the boss's local
    frame instead of the square+rounded-corner profile — gives 90°
    corners everywhere. The rectangle has dimensions width_x×width_z
    and its center sits at (center_dx, center_dz) relative to the
    pin location in local frame. Use the center offset to shift the
    boss outline to align with the pin's recess.

    neg_x_shift_local / pos_x_shift_local: optional (dx, dz) tuples
    for legacy parallelogram-style asymmetric shifts on the -X / +X
    sides (kept for backward compatibility but rect_local is
    preferred).

    trim_z_post / trim_keep_above: if given, cut the boss with a
    horizontal half-space at z=trim_z_post (see comments on the
    housing call sites)."""
    alpha_rad = math.radians(alpha_deg)
    x = pivot_x + STOP_PIN_R * math.cos(alpha_rad)
    z = LEVER_PIVOT_Z - STOP_PIN_R * math.sin(alpha_rad)
    if pin_z_sign is None:
        return (
            cq.Workplane("XY")
            .circle(STOP_PIN_SUPPORT_D / 2)
            .extrude(HOUSING_W)
            .rotate((0, 0, 0), (1, 0, 0), -90)
            .translate((x, -HOUSING_W / 2, z))
        )
    boss_r = STOP_PIN_D / 2 + STOP_PIN_BOSS_WALL_T
    # Build the XZ profile.
    if rect_local is not None:
        wx, wz, cx, cz = rect_local
        profile = (
            cq.Workplane("XZ")
            .center(cx, cz)
            .rect(wx, wz)
        )
    elif neg_x_shift_local is None and pos_x_shift_local is None:
        profile = brake_pin_xz_profile(boss_r, pin_z_sign)
    else:
        s = pin_z_sign
        nx, nz = neg_x_shift_local or (0.0, 0.0)
        px, pz = pos_x_shift_local or (0.0, 0.0)
        profile = (
            cq.Workplane("XZ")
            .moveTo(+boss_r + px, -s * boss_r + pz)
            .lineTo(+boss_r + px, +s * boss_r + pz)
            .lineTo(-boss_r + nx, +s * boss_r + nz)
            .lineTo(-boss_r + nx, -s * boss_r + nz)
            .close()
        )
    boss = (
        profile
        .extrude(HOUSING_W / 2, both=True)
        .rotate((0, 0, 0), (0, 1, 0), alpha_deg)
        .translate((x, 0, z))
    )
    if trim_z_post is not None:
        # trim_z_post is in world coords; cut directly against it.
        big = 1000.0
        if trim_keep_above:
            # Keep z >= trim_z_post. Cut away the half-space BELOW.
            cutter = (
                cq.Workplane("XY")
                .workplane(offset=trim_z_post - big)
                .rect(big, big)
                .extrude(big)
            )
        else:
            # Keep z <= trim_z_post. Cut away the half-space ABOVE.
            cutter = (
                cq.Workplane("XY")
                .workplane(offset=trim_z_post)
                .rect(big, big)
                .extrude(big)
            )
        boss = boss.cut(cutter)
    return boss

def brake_pin_xz_profile(r: float, rounded_z_sign: int) -> cq.Workplane:
    """2D cross-section in the XZ plane: 2r×2r square with one corner
    rounded into a quarter-circle of radius r. Rounded corner sits at
    (-r, +r) for rounded_z_sign=+1 or (-r, -r) for rounded_z_sign=-1
    (always on the -x side, the side facing the lever pin). The rounded
    corner is the ONLY surface that sees contact with the lever pin —
    making it round gives clean line/point contact identical to a plain
    cylindrical pin. The remaining three square sides serve as keying:
    the matching square hole locks the pin in one rotational orientation
    when glued in.

    Spring pin and rest pin sit on opposite sides of the lever pin's
    rest position, so they need rounded corners on opposite sides of
    their respective local frames — modeled as separately-printed parts."""
    s = rounded_z_sign  # ±1
    sqrt2 = math.sqrt(2.0)
    # Walk the boundary; cadquery doesn't care about CW/CCW orientation
    # for a single closed wire, so use a fixed walk pattern across both
    # signs of s.
    return (
        cq.Workplane("XZ")
        .moveTo(+r, -s * r)
        .lineTo(+r, +s * r)
        .lineTo(0,  +s * r)                                  # tangent point on top edge
        .threePointArc((-r / sqrt2, s * r / sqrt2), (-r, 0))  # rounded corner
        .lineTo(-r, -s * r)
        .close()
    )


def _brake_hole_local(rounded_z_sign):
    """Housing-hole cutter in the same LOCAL frame as _brake_pin_local().
    Same square+rounded-corner profile as the pin, oversized in every
    direction by BRAKE_PIN_HOLE_CLR/2 (= 0.25 mm radial). Spans only the
    portion BURIED in the housing (insert shank); the exposed portion
    doesn't exist in the hole."""
    r       = STOP_PIN_D / 2 + BRAKE_PIN_HOLE_CLR / 2
    L_exp   = STOP_PIN_H
    L_ins   = BRAKE_PIN_INSERT_DEPTH
    eps     = 0.05

    return (
        brake_pin_xz_profile(r, rounded_z_sign)
        # Negative extrude — see _brake_pin_local for the explanation.
        .extrude(-(L_ins + eps))
        .translate((0, L_exp - eps, 0))
    )

def brake_pin_place(part: cq.Workplane,
                    alpha: float | None = None) -> cq.Workplane:
    """Shared transform from LOCAL frame to WORLD for the brake pin / hole.
    Local +x (radial) → world (cos α, 0, sin α) via rotation around Y by
    -α. Local +y (pin axis, exposed→deep) maps to world +y (from y=-13
    at exposed tip toward y=-8.25 at cone apex). The default α is the
    spring pin's angle; pass alpha= for the rest pin's angle."""
    if alpha is None:
        alpha = STOP_HOUSING_PIN_ALPHA_BRAKE_DEG
    alpha_rad = math.radians(alpha)
    x_pin = BRAKE_PIVOT_X + STOP_PIN_R * math.cos(alpha_rad)
    z_pin = LEVER_PIVOT_Z - STOP_PIN_R * math.sin(alpha_rad)
    y_tip = -HOUSING_W / 2 - STOP_PIN_H
    return (part
            .rotate((0, 0, 0), (0, 1, 0), alpha)
            .translate((x_pin, y_tip, z_pin)))

def _build_housing_skeleton():
    """U-bracket + pivot bosses + stop pins + lever-pivot/cross-pin/mount
    holes. Guide-bearing area is added on top in ``_build_housing()`` once
    the guide intermediates have been defined further down the module."""
    return (
        # ── UNIONS FIRST ────────────────────────────────────────────────────
        top_plate.union(bot_plate).union(spine)
        # Support bosses around each housing stop pin (full housing y width).
        .union(_stop_pin_support(RATCHET_PIVOT_X, STOP_HOUSING_PIN_ALPHA_RATCHET_DEG))
        # Top (spring pin) boss: axis-aligned rectangle in the boss's local
        # frame. -X wall is "inside housing", extended 0.110 mm outward in
        # local (= 0.1 mm in world toward the axle hole). Other 3 walls at
        # ±3.05 in local — 0.8 mm wall thickness around the 4.5 mm pin hole.
        .union(_stop_pin_support(BRAKE_PIVOT_X,   STOP_HOUSING_PIN_ALPHA_BRAKE_DEG,
                                 pin_z_sign=-1,
                                 rect_local=(6.287, 6.10, -0.0935, 0.0)))
        # Second support boss for the REST pin on each pivot.
        .union(_stop_pin_support(RATCHET_PIVOT_X, STOP_REST_PIN_ALPHA_RATCHET_DEG))
        # Bottom (rest pin) boss: -X wall stays housing-flush. Local +Z
        # wall (face closer to the ratchet lever axle) is bumped out
        # 0.215 mm for 1.015 mm wall thickness; +X and -Z at standard 0.8.
        .union(_stop_pin_support(BRAKE_PIVOT_X,   STOP_REST_PIN_ALPHA_BRAKE_DEG,
                                 pin_z_sign=+1,
                                 rect_local=(7.88, 6.315, -0.89, +0.1075)))
        # Ratchet housing stop pins (printed in-place).
        .union(stop_pin_solid(RATCHET_PIVOT_X, STOP_HOUSING_PIN_ALPHA_RATCHET_DEG,
                               HOUSING_W / 2 - _PIN_BOSS_OVERLAP,
                               HOUSING_W / 2 + STOP_PIN_H))
        .union(stop_pin_solid(RATCHET_PIVOT_X, STOP_REST_PIN_ALPHA_RATCHET_DEG,
                               HOUSING_W / 2 - _PIN_BOSS_OVERLAP,
                               HOUSING_W / 2 + STOP_PIN_H))
        # ── ALL CUTS LAST ───────────────────────────────────────────────
        # Lever pivots: clearance through + insert on one face. No head
        # counterbore — the screw IS the lever's pivot axle.
        .cut(_pivot_clearance(RATCHET_PIVOT_X))
        .cut(_pivot_insert_pilot(RATCHET_PIVOT_X, insert_face_sign=+1))
        # Brake pivot: insert on -y (same side as the brake stop-pin
        # holes); screw enters from +y with full housing width to reach.
        .cut(_pivot_clearance(BRAKE_PIVOT_X))
        .cut(_pivot_insert_pilot(BRAKE_PIVOT_X, insert_face_sign=-1))
        # Brake separately-printed pin insertion holes.
        .cut(brake_pin_place(_brake_hole_local(rounded_z_sign=-1)))
        .cut(brake_pin_place(_brake_hole_local(rounded_z_sign=+1),
                              alpha=STOP_REST_PIN_ALPHA_BRAKE_DEG))
        # Ratchet spring-leg through-hole (cut AFTER the support boss
        # exists so the hole spans the full STOP_PIN_SUPPORT_D).
        .cut(stop_pin_hole(RATCHET_PIVOT_X, STOP_HOUSING_PIN_ALPHA_RATCHET_DEG,
                            hole_y=+(HOUSING_W / 2 + STOP_PIN_HOLE_OFFSET),
                            cover_d=STOP_PIN_SUPPORT_D))
        # Axle cross-pin hole — pancake side only. The lever-side cross-pin
        # was dropped: with the lever-side plate trimmed to x=57 there's
        # no plate material at x=0 to anchor the screw, and the axle is
        # already constrained on the lever side by the bottom 608 bearing.
        .cut(_axle_pin_clearance(PANCAKE_CROSS_PIN_Z))
        .cut(_axle_pin_insert_pilot(PANCAKE_CROSS_PIN_Z, insert_face_sign=-1))
        # Wall-mount stud hole through the spine.
        .cut(_mount_hole(0, MOUNT_SCREW_Z))
        # Pancake cross-pin head counterbore (LAST so boss material in its
        # volume is removed).
        .cut(_screw_head_counterbore(0, PANCAKE_CROSS_PIN_Z, entry_face_sign=+1))
    )

# Final housing assembly: U-bracket skeleton (this module) + pancake-side
# guide-bearing (housing_guide.py) + lever-side guide-bearing
# (housing_lever_guide.py).
def _build_housing():
    from . import housing_guide, housing_lever_guide
    h = _build_housing_skeleton()
    h = housing_guide.apply_to_housing(h)
    h = housing_lever_guide.apply_to_housing(h)
    return h


housing = _build_housing()

