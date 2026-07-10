"""Mount bracket — separately printed L-shaped piece that fastens to two
wood beams (one horizontal above the housing, one vertical on +X of the
housing) and holds the housing+spool assembly via two M2 screws.

The bracket has two thick "chunks" set into pockets in the housing
(top + side), each holding two wood screws (Ø4 shaft, Ø9 cone-recessed
head at the housing-side face) and one M2 (Ø2.3, perpendicular to the
wood screws, with a heat-set insert on the chunk's +Y end). A 1.7 mm-
thick × 12.4 mm-wide connecting strip wraps around the housing's top-
right corner, joining the two chunks so the bracket prints and installs
as one piece.

Install workflow:
  1. Bracket attached to the wood beams via the four wood screws (heads
     accessible from the chunks' housing-side faces while the bracket is
     not yet inside the housing).
  2. Heat-set M2 inserts into the +Y end of each chunk.
  3. Housing (with spool inside) slides onto the bracket; M2 screws enter
     the housing's -Y outer face, pass through the housing wall and the
     bracket chunk, and engage the heat-set inserts. To remove the
     housing+spool, back out the two M2 screws — the bracket stays
     attached to the wood.
"""

import cadquery as cq

from .dimensions import (
    AXLE_PRINT_D,
    BOOL_OVERSHOOT,
    FIT_CLR,
    M2_SHAFT_CLR_D,
    STRUCT_WALL,
)
from .housing import (
    HOUSING_W,
    PANCAKE_PLATE_Z_OUT,
    SPINE_X_OUTER,
)

# ── Bracket dimensions ──────────────────────────────────────────────────────
WOOD_SCREW_SHAFT_D   = 4.0
WOOD_SCREW_HEAD_D    = 9.3   # countersunk-head pocket Ø *at the bracket's
                             # housing-side face* (the visible hole Ø, sized
                             # to fully recess the screw head). The cone's
                             # wide end is extrapolated linearly past the
                             # face by BOOL_OVERSHOOT for a clean boolean
                             # break — see _wood_screw_cut_at.
WOOD_SCREW_HEAD_H    = 4.0   # cone-recessed head depth at chunk's housing-side face
WOOD_SCREW_SHAFT_H   = STRUCT_WALL  # straight-cylinder length of the shaft hole past
                                    # the cone, before the chunk ends. Kept short
                                    # (= one structural wall) so the chunk's overlap
                                    # with the housing pocket stays shallow — less
                                    # overhang to support when printing the housing.

CHUNK_LENGTH         = 22.0                                  # along the two-wood-screw axis

# Housing-pocket extensions (bracket chunks themselves stay at CHUNK_LENGTH).
# See _bracket_envelope docstring for why both ends can't be a tight fit.
TOP_POCKET_EXTRA_X   = 1.0                                   # 22 → 23 mm pocket along X
SIDE_POCKET_EXTRA_Z  = 8.0                                   # 22 → 30 mm pocket along Z
                                                             # (6.6 mm needed kinematically; +1.4 mm safety)
CHUNK_WIDTH          = 13.0                                  # Y dimension (M2 axis) — matches
                                                             # the guide-wheel pocket Y width
                                                             # (WHEEL_PAINTED_OD + 2·WHEEL_POCKET_Y_CLR)
CHUNK_DEPTH          = WOOD_SCREW_HEAD_H + WOOD_SCREW_SHAFT_H  # 5.7 — cone + short shaft only
# Thin chunk wall between the wood-screw cone's wide-end edge and the chunk's
# ±Y face at the housing-side face — replaced with a 45° chamfer to make the
# housing pocket's roof more printable (the +Y / -Y wall transitions from the
# floor at 45° instead of a flat horizontal overhang). Length = the chunk's
# long-axis extent (21.85 mm after FIT_CLR shrink).
POCKET_CHAMFER_H     = 0.85

STRIP_W              = CHUNK_WIDTH    # 13.0, in Y (full bracket width)
STRIP_T              = STRUCT_WALL    # 1.7 mm depth into housing outer surface

# Wood-screw spacing along the chunk length: 2 heads + three equal gaps in
# the chunk's CHUNK_LENGTH. Hole centers sit _WS_OFFSET_FROM_END from each
# end of the chunk along its long axis. (CHUNK_Y_CENTER is defined below,
# after STRIP_Y_LO and CHUNK_Y_HI are set.)
_WS_GAP              = (CHUNK_LENGTH - 2 * WOOD_SCREW_HEAD_D) / 3
_WS_OFFSET_FROM_END  = _WS_GAP + WOOD_SCREW_HEAD_D / 2

# ── Chunk positions ─────────────────────────────────────────────────────────
# Top chunk: -X edge sits 2 mm past the main spool axle's +X surface (axle
# centered at x=0 with radius AXLE_PRINT_D/2).
TOP_CHUNK_X_LO       = AXLE_PRINT_D / 2 + 2.0              # 5.85
TOP_CHUNK_X_HI       = TOP_CHUNK_X_LO + CHUNK_LENGTH       # 27.85
TOP_CHUNK_Z_HI       = PANCAKE_PLATE_Z_OUT                 # flush with housing top face
TOP_CHUNK_Z_LO       = TOP_CHUNK_Z_HI - CHUNK_DEPTH        # depth into pancake plate
# Side chunk: sits on the front-housing +X face between the ratchet pivot
# (z≈20) and the pancake plate (z≈53), clear of both lever pivots and the
# axle cross-pin. (Was anchored to the now-removed lever-side guide wheel.)
SIDE_CHUNK_Z_LO      = 30.0
SIDE_CHUNK_Z_HI      = SIDE_CHUNK_Z_LO + CHUNK_LENGTH      # 52
SIDE_CHUNK_X_HI      = SPINE_X_OUTER                       # flush with housing +X face
SIDE_CHUNK_X_LO      = SIDE_CHUNK_X_HI - CHUNK_DEPTH       # depth into spine

# ── Strip path (1.7 mm-deep groove cut from housing outer surface) ──────────
STRIP_Y_LO           = -STRIP_W / 2
STRIP_Y_HI           = +STRIP_W / 2
# The CHUNK pockets stop STRIP_T short of the strip's +Y end so the chunk's
# +Y "roof" is flush with the -Y start of the chamfer wedge. This makes
# the strip-side chamfer extend continuously to the chunk's -Z wall (the
# chunk's deep portion just gets a flat-bridge roof at CHUNK_Y_HI, which
# PA6-GF handles fine).
CHUNK_Y_HI           = STRIP_Y_HI - STRIP_T

# Y-center of the wood-screw cones — midpoint of the chunk's actual Y
# extent [STRIP_Y_LO, CHUNK_Y_HI] (= the 11 mm "block" left behind after
# the +Y strip chamfer eats into the top 1.7 mm), NOT the 13 mm full
# strip width. Centering on the chunk block keeps equal material on
# both sides of each wood-screw head pocket.
CHUNK_Y_CENTER       = (STRIP_Y_LO + CHUNK_Y_HI) / 2         # -0.85
# Top strip — along +Z face of housing, from top chunk to corner.
TOP_STRIP_X_LO       = TOP_CHUNK_X_LO                      # 2 (chunk -X edge)
TOP_STRIP_X_HI       = SPINE_X_OUTER                       # 92.5 (housing +X edge / corner)
TOP_STRIP_Z_LO       = TOP_CHUNK_Z_HI - STRIP_T            # PANCAKE_PLATE_Z_OUT − 1.7
TOP_STRIP_Z_HI       = TOP_CHUNK_Z_HI
# Side strip — along +X face of housing, from corner to side chunk.
SIDE_STRIP_X_LO      = SIDE_CHUNK_X_HI - STRIP_T           # SPINE_X_OUTER − 1.7
SIDE_STRIP_X_HI      = SIDE_CHUNK_X_HI
SIDE_STRIP_Z_LO      = SIDE_CHUNK_Z_LO                     # -3 (chunk -Z edge)
SIDE_STRIP_Z_HI      = PANCAKE_PLATE_Z_OUT                 # housing top corner

# ── M2 axes — along Y. Head pocket in housing's -Y outer face; insert in chunk +Y end ──
# Centered along the chunk's LENGTH axis (between the two wood screws) and along its DEPTH axis.
TOP_M2_X_CENTER      = (TOP_CHUNK_X_LO + TOP_CHUNK_X_HI) / 2     # 13
TOP_M2_Z_CENTER      = (TOP_CHUNK_Z_LO + TOP_CHUNK_Z_HI) / 2     # depth-center
SIDE_M2_X_CENTER     = (SIDE_CHUNK_X_LO + SIDE_CHUNK_X_HI) / 2   # depth-center
SIDE_M2_Z_CENTER     = (SIDE_CHUNK_Z_LO + SIDE_CHUNK_Z_HI) / 2   # 8


# ── Helpers ─────────────────────────────────────────────────────────────────
def _box(x_lo, x_hi, y_lo, y_hi, z_lo, z_hi):
    return (cq.Workplane("XY")
            .box(x_hi - x_lo, y_hi - y_lo, z_hi - z_lo, centered=False)
            .translate((x_lo, y_lo, z_lo)))


# 45° chamfers at the +Y end of every cavity face the cut creates. The
# housing prints in the -Y → +Y direction, so the +Y wall of each cavity
# (a flat rectangle in the XZ plane) is the print's "ceiling" and would
# be an unsupported bridge over the cavity. Each chamfer ramps the
# cavity's DEEP wall (the wall farthest from the housing outer surface)
# up to the outer surface over a 45° slope ending at y=+CHUNK_WIDTH/2,
# so the cavity collapses to a line right where the +Y wall begins —
# everywhere along the L-shaped cut.
def _y_chamfer_wedge_yz(*, x_lo, x_hi, z_floor, z_ceil, y_threshold):
    """Triangular wedge in YZ extruded along X (used for the TOP chunk
    and the TOP strip — their deep dimension is Z, their length is X)."""
    return (cq.Workplane("YZ")
            .workplane(offset=x_lo - BOOL_OVERSHOOT)
            .moveTo(y_threshold, z_floor)
            .lineTo(STRIP_W / 2, z_floor)
            .lineTo(STRIP_W / 2, z_ceil)
            .close()
            .extrude(x_hi - x_lo + 2 * BOOL_OVERSHOOT))


def _y_chamfer_wedge_xy(*, x_floor, x_ceil, z_lo, z_hi, y_threshold):
    """Triangular wedge in XY extruded along Z (used for the SIDE chunk
    and the SIDE strip — their deep dimension is X, their length is Z)."""
    return (cq.Workplane("XY")
            .workplane(offset=z_lo - BOOL_OVERSHOOT)
            .moveTo(x_floor, y_threshold)
            .lineTo(x_floor, STRIP_W / 2)
            .lineTo(x_ceil, STRIP_W / 2)
            .close()
            .extrude(z_hi - z_lo + 2 * BOOL_OVERSHOOT))


def _top_strip_x_chamfer_cut():
    """The 45° X-axis chamfer, applied SUBTRACTIVELY to the housing
    plate material above the TOP strip. Bracket envelope already ends
    at CHUNK_Y_HI=+4.8 so the plate at y=+4.8..+STRIP_W/2 ×
    z=TOP_STRIP_Z_LO..TOP_STRIP_Z_HI is intact natural housing material;
    this cut removes the upper-left triangle in YZ from that material,
    leaving the lower triangle as the chamfered shape. Spans the full
    TOP strip X (TOP_STRIP_X_LO..TOP_STRIP_X_HI=SPINE_X_OUTER) — all
    the way to the +X edge."""
    y_threshold = CHUNK_Y_HI                # +4.8 (bracket +Y end)
    # No -X overshoot: the cut starts exactly at TOP_STRIP_X_LO (the
    # bracket's -X edge) so we don't carve into housing plate material
    # past the bracket. +X overshoot is OK (cuts air past SPINE_X_OUTER).
    return (cq.Workplane("YZ")
            .workplane(offset=TOP_STRIP_X_LO)
            .moveTo(y_threshold, TOP_STRIP_Z_LO)
            .lineTo(y_threshold, TOP_STRIP_Z_HI)
            .lineTo(STRIP_W / 2, TOP_STRIP_Z_HI)
            .close()
            .extrude(TOP_STRIP_X_HI - TOP_STRIP_X_LO + BOOL_OVERSHOOT))


def _side_strip_z_chamfer_cut():
    """The 45° Z-axis chamfer, applied SUBTRACTIVELY to the housing
    plate material above the SIDE strip. Same pattern as the TOP cut
    but in XY (extruded along Z). Removes the upper-right triangle in
    XY from the plate at y=+4.8..+STRIP_W/2 × x=SIDE_STRIP_X_LO..
    SIDE_STRIP_X_HI=SPINE_X_OUTER. Spans the full SIDE strip Z
    (SIDE_STRIP_Z_LO..SIDE_STRIP_Z_HI=PANCAKE_PLATE_Z_OUT) — all the
    way to the +Z edge of the housing."""
    y_threshold = CHUNK_Y_HI                # +4.8 (bracket +Y end)
    # No -Z overshoot: the cut starts exactly at SIDE_STRIP_Z_LO (the
    # bracket's -Z edge) so we don't carve into housing spine material
    # below the bracket. +Z overshoot is OK (cuts air past
    # PANCAKE_PLATE_Z_OUT).
    return (cq.Workplane("XY")
            .workplane(offset=SIDE_STRIP_Z_LO)
            .moveTo(SIDE_STRIP_X_LO, y_threshold)
            .lineTo(SIDE_STRIP_X_HI, y_threshold)
            .lineTo(SIDE_STRIP_X_HI, STRIP_W / 2)
            .close()
            .extrude(SIDE_STRIP_Z_HI - SIDE_STRIP_Z_LO + BOOL_OVERSHOOT))


def _corner_chamfer_tetrahedron():
    """The chamfer cut's third segment — a tilted 45° plane wrapping the
    corner cube. The plane (x − y + z = 148.5) contains exactly the same
    edge as the TOP strip chamfer plane (z = y + 57.7) along x = 90.8
    and the same edge as the SIDE strip chamfer plane (x = y + 86)
    along z = 62.5, so the three chamfer surfaces form ONE continuous
    45° surface across the entire L. Implemented as a tetrahedron — the
    natural shape of a half-space cut from the corner cube section.

    Vertices: V1 = apex at (90.8, +4.8, 62.5) (chamfer start);
    base triangle V2=(90.8,+6.5,62.5), V3=(90.8,+6.5,64.2),
    V4=(92.5,+6.5,62.5) (chamfer end). Loft from base triangle down to
    a degenerate (tiny) triangle around V1."""
    apex_y = STRIP_W / 2 - STRIP_T
    base_y = STRIP_W / 2
    eps = 1e-3
    return (cq.Workplane("XZ")
            .workplane(offset=-base_y)
            .moveTo(SIDE_STRIP_X_LO, TOP_STRIP_Z_LO)         # V2
            .lineTo(SIDE_STRIP_X_HI, TOP_STRIP_Z_LO)         # V4
            .lineTo(SIDE_STRIP_X_LO, TOP_STRIP_Z_HI)         # V3
            .close()
            .workplane(offset=base_y - apex_y)
            .moveTo(SIDE_STRIP_X_LO - eps, TOP_STRIP_Z_LO - eps)
            .lineTo(SIDE_STRIP_X_LO + eps, TOP_STRIP_Z_LO - eps)
            .lineTo(SIDE_STRIP_X_LO - eps, TOP_STRIP_Z_LO + eps)
            .close()
            .loft(combine=True))


def _y_chamfer_wedges():
    """The +Y chamfer cut. THREE PIECES that together form ONE continuous
    45° surface across the L-shaped strip path:

      • TOP strip wedge — chamfer in YZ along the TOP strip length
        (x in [TOP_STRIP_X_LO, SIDE_STRIP_X_LO]).
      • SIDE strip wedge — chamfer in XY along the SIDE strip length
        (z in [SIDE_STRIP_Z_LO, TOP_STRIP_Z_LO]).
      • Corner tetrahedron — the third segment, a tilted 45° plane that
        wraps the corner cube. Its chamfer plane meets the TOP wedge's
        plane along x=90.8 and the SIDE wedge's plane along z=62.5,
        so all three are coplanar at the boundaries — no notch, no seam.

    Each strip wedge is truncated at the corner cube boundary (otherwise
    its extruded prism would cut into the other strip at the corner cube
    and leave a stray triangle of bracket material). The corner
    tetrahedron handles the corner cube area.

    Only the STRIP wedges (1.7 mm depth) get the 45° chamfer — the
    CHUNK pockets' deeper +Y faces are left flat (22 × 6.6 mm bridges
    PA6-GF handles fine) so the wood-screw counterbores aren't eaten."""
    top_strip_depth = TOP_STRIP_Z_HI - TOP_STRIP_Z_LO            # 1.7
    side_strip_depth = SIDE_STRIP_X_HI - SIDE_STRIP_X_LO         # 1.7

    top_strip_wedge = _y_chamfer_wedge_yz(
        x_lo=TOP_STRIP_X_LO, x_hi=SIDE_STRIP_X_LO,
        z_floor=TOP_STRIP_Z_LO, z_ceil=TOP_STRIP_Z_HI,
        y_threshold=STRIP_W / 2 - top_strip_depth,
    )
    side_strip_wedge = _y_chamfer_wedge_xy(
        x_floor=SIDE_STRIP_X_LO, x_ceil=SIDE_STRIP_X_HI,
        z_lo=SIDE_STRIP_Z_LO, z_hi=TOP_STRIP_Z_LO,
        y_threshold=STRIP_W / 2 - side_strip_depth,
    )
    return (top_strip_wedge
            .union(side_strip_wedge)
            .union(_corner_chamfer_tetrahedron()))


def _wood_screw_cut_at(x_center, y_center, z_face, direction):
    """One wood-screw cutter: Ø WOOD_SCREW_SHAFT_D shaft through full
    CHUNK_DEPTH plus a Ø WOOD_SCREW_HEAD_D → Ø WOOD_SCREW_SHAFT_D cone-
    recessed head at the chunk's housing-side face. The cone is
    constructed so its diameter AT z_face is exactly WOOD_SCREW_HEAD_D
    (the visible hole Ø); the wide end is extrapolated linearly to
    BOOL_OVERSHOOT past the face for a clean boolean break.
    `direction` is (dx, dy, dz) pointing AWAY from the housing-side face
    (so the screw threads toward the wood)."""
    dx, dy, dz = direction
    pnt = cq.Vector(x_center - dx * BOOL_OVERSHOOT,
                    y_center - dy * BOOL_OVERSHOOT,
                    z_face   - dz * BOOL_OVERSHOOT)
    shaft = cq.Workplane("XY").add(cq.Solid.makeCylinder(
        WOOD_SCREW_SHAFT_D / 2, CHUNK_DEPTH + 2 * BOOL_OVERSHOOT,
        pnt=pnt, dir=cq.Vector(*direction),
    ))
    # Wide-end Ø chosen so the cone's Ø at z_face is exactly
    # WOOD_SCREW_HEAD_D after the BOOL_OVERSHOOT extension. Linear
    # extrapolation of the (HEAD_D → SHAFT_D over HEAD_H) cone:
    cone_slope_per_mm = (WOOD_SCREW_HEAD_D - WOOD_SCREW_SHAFT_D) / WOOD_SCREW_HEAD_H
    cone_wide_d = WOOD_SCREW_HEAD_D + BOOL_OVERSHOOT * cone_slope_per_mm
    head_cone = cq.Workplane("XY").add(cq.Solid.makeCone(
        cone_wide_d / 2, WOOD_SCREW_SHAFT_D / 2,
        WOOD_SCREW_HEAD_H + BOOL_OVERSHOOT,
        pnt=pnt, dir=cq.Vector(*direction),
    ))
    return shaft.union(head_cone)


def _m2_chunk_cut(x_center, z_center):
    """Ø M2_SHAFT_CLR_D through-hole along Y across the full chunk width.
    The screw head and the heat-set insert both live in housing material
    (-Y and +Y walls respectively, same pattern as the axle cross-pin);
    the chunk is sandwiched between them with only this clearance hole."""
    return cq.Workplane("XY").add(cq.Solid.makeCylinder(
        M2_SHAFT_CLR_D / 2,
        CHUNK_WIDTH + 2 * BOOL_OVERSHOOT,
        pnt=cq.Vector(x_center, -CHUNK_WIDTH / 2 - BOOL_OVERSHOOT, z_center),
        dir=cq.Vector(0, 1, 0),
    ))


def _chamfered_top_strip_prism(x_lo=None, x_hi=None, z_lo=None, z_hi=None,
                               y_lo=None, y_chamfer_base=None, y_top=None):
    """TOP strip as a single chamfered prism — trapezoidal YZ cross-section
    extruded along X. +Y face ramps from y=y_chamfer_base at z=z_lo up to
    y=y_top at z=z_hi, matching the housing's concave chamfer above the
    strip pocket. Bounds default to the pocket dimensions; the bracket
    build passes shrunk bounds for FIT_CLR clearance on its 6 small faces."""
    x_lo = TOP_STRIP_X_LO if x_lo is None else x_lo
    x_hi = TOP_STRIP_X_HI if x_hi is None else x_hi
    z_lo = TOP_STRIP_Z_LO if z_lo is None else z_lo
    z_hi = TOP_STRIP_Z_HI if z_hi is None else z_hi
    y_lo = STRIP_Y_LO if y_lo is None else y_lo
    y_chamfer_base = CHUNK_Y_HI if y_chamfer_base is None else y_chamfer_base
    y_top = STRIP_W / 2 if y_top is None else y_top
    return (cq.Workplane("YZ")
            .workplane(offset=x_lo)
            .moveTo(y_lo,           z_lo)
            .lineTo(y_chamfer_base, z_lo)
            .lineTo(y_top,          z_hi)
            .lineTo(y_lo,           z_hi)
            .close()
            .extrude(x_hi - x_lo))


def _chamfered_side_strip_prism(x_lo=None, x_hi=None, z_lo=None, z_hi=None,
                                y_lo=None, y_chamfer_base=None, y_top=None):
    """SIDE strip as a single chamfered prism — trapezoidal XY cross-section
    extruded along Z. Bounds default to the pocket dimensions; the bracket
    build passes shrunk bounds for FIT_CLR clearance on its 6 small faces."""
    x_lo = SIDE_STRIP_X_LO if x_lo is None else x_lo
    x_hi = SIDE_STRIP_X_HI if x_hi is None else x_hi
    z_lo = SIDE_STRIP_Z_LO if z_lo is None else z_lo
    z_hi = SIDE_STRIP_Z_HI if z_hi is None else z_hi
    y_lo = STRIP_Y_LO if y_lo is None else y_lo
    y_chamfer_base = CHUNK_Y_HI if y_chamfer_base is None else y_chamfer_base
    y_top = STRIP_W / 2 if y_top is None else y_top
    return (cq.Workplane("XY")
            .workplane(offset=z_lo)
            .moveTo(x_lo, y_lo)
            .lineTo(x_hi, y_lo)
            .lineTo(x_hi, y_top)
            .lineTo(x_lo, y_chamfer_base)
            .close()
            .extrude(z_hi - z_lo))


# ── Build the bracket ───────────────────────────────────────────────────────
# Four bodies built and cut INDEPENDENTLY (each cutter only touches the
# body it passes through), then unioned at the end. The two strips are
# each a single chamfered prism — the convex counterpart to the housing's
# concave 45° chamfer cuts above the strip pockets. In the corner cube
# where the two prisms overlap, their +Y surfaces meet at a diagonal
# seam under the housing's single chamfer plane (small cosmetic gap, no
# interference; verified algebraically). Per-prism cuts avoid the OCCT
# face-graph issues we hit when one boolean walked across a unioned
# multi-prism shape.
def _build_bracket():
    # Shrink the bracket's 6 non-flush perimeter faces by FIT_CLR (0.15 mm)
    # so it drops into the housing pocket with a uniform slip-fit gap.
    # Preserved (no shrink) — the 4 large faces:
    #   • TOP +Z (flush with housing top exterior) and TOP −Z (against the
    #     pocket's deep wall — bracket bottoms out on this surface in z)
    #   • FRONT +X (flush with housing side exterior) and FRONT −X (against
    #     the pocket's deep wall — bracket bottoms out on this surface in x)
    # Shrunk by FIT_CLR — the 6 small perimeter faces:
    #   • TOP −X end, TOP +Y, TOP −Y
    #   • FRONT −Z end, FRONT +Y, FRONT −Y
    # The +Y face is partly flat (over the chunks) and partly chamfered (over
    # the strips); both portions shift inward together so the chamfer angle
    # is preserved while the chunk_y_hi and strip top-y both shrink by s.
    # M2 / wood-screw centers and faces stay at original positions so they
    # remain aligned with the housing's matching holes.
    s = FIT_CLR
    chunk_y_hi_b = CHUNK_Y_HI - s        # shrunk +Y for chunks and chamfer base
    strip_y_lo_b = STRIP_Y_LO + s        # shrunk -Y (bilateral with above)
    strip_y_top_b = STRIP_W / 2 - s      # shrunk strip +Y at the flush face

    # 1. TOP chunk + its cuts (wood screws into +Z beam, M2 along Y).
    top_chunk = _box(TOP_CHUNK_X_LO + s, TOP_CHUNK_X_HI,
                     strip_y_lo_b, chunk_y_hi_b,
                     TOP_CHUNK_Z_LO, TOP_CHUNK_Z_HI)
    top_chunk = top_chunk.edges("|X and <Z").chamfer(POCKET_CHAMFER_H)
    # Inset the +Y vertical face of the wood-screw block from y=4.75 to
    # y=4.55 (0.2 mm). XZ-plane cut over the +Y face's z range PLUS the
    # bottom chamfer (down to TOP_CHUNK_Z_LO) so the chamfer's upper part
    # is consumed too — the chamfer ends cleanly at z=58.55 / y=4.55 with
    # no leftover horizontal step at z=58.75. Strip above is untouched.
    py_face_inset_cut = _box(
        TOP_CHUNK_X_LO + s - BOOL_OVERSHOOT, TOP_CHUNK_X_HI + BOOL_OVERSHOOT,
        chunk_y_hi_b - 0.2, chunk_y_hi_b + BOOL_OVERSHOOT,
        TOP_CHUNK_Z_LO, TOP_STRIP_Z_LO)
    top_chunk = top_chunk.cut(py_face_inset_cut)
    # 3. TOP strip — chamfered prism with shrunk -X and ±Y bounds.
    top_strip = _chamfered_top_strip_prism(
        x_lo=TOP_STRIP_X_LO + s,
        y_lo=strip_y_lo_b,
        y_chamfer_base=chunk_y_hi_b,
        y_top=strip_y_top_b,
    )
    for x in (TOP_CHUNK_X_LO + _WS_OFFSET_FROM_END,
              TOP_CHUNK_X_HI - _WS_OFFSET_FROM_END):
        ws = _wood_screw_cut_at(x, CHUNK_Y_CENTER, TOP_CHUNK_Z_LO, (0, 0, 1))
        top_chunk = top_chunk.cut(ws)
        top_strip = top_strip.cut(ws)

    # 2. SIDE chunk + its cuts (wood screws into +X beam).
    side_chunk = _box(SIDE_CHUNK_X_LO, SIDE_CHUNK_X_HI,
                      strip_y_lo_b, chunk_y_hi_b,
                      SIDE_CHUNK_Z_LO + s, SIDE_CHUNK_Z_HI)
    side_chunk = side_chunk.edges("|Z and <X").chamfer(POCKET_CHAMFER_H)
    # Inset the +Y vertical face of the side wood-screw block from y=4.75
    # to y=4.55 (0.2 mm). Same treatment as the top chunk, rotated 90°:
    # cut over the chunk's z range and the -X chamfer (x from SIDE_CHUNK_X_LO
    # up through the chamfer), ending at SIDE_STRIP_X_LO so the side strip
    # is untouched.
    side_py_face_inset_cut = _box(
        SIDE_CHUNK_X_LO, SIDE_STRIP_X_LO,
        chunk_y_hi_b - 0.2, chunk_y_hi_b + BOOL_OVERSHOOT,
        SIDE_CHUNK_Z_LO + s - BOOL_OVERSHOOT, SIDE_CHUNK_Z_HI + BOOL_OVERSHOOT)
    side_chunk = side_chunk.cut(side_py_face_inset_cut)
    # 4. SIDE strip — chamfered prism with shrunk -Z and ±Y bounds.
    side_strip = _chamfered_side_strip_prism(
        z_lo=SIDE_STRIP_Z_LO + s,
        y_lo=strip_y_lo_b,
        y_chamfer_base=chunk_y_hi_b,
        y_top=strip_y_top_b,
    )
    for z in (SIDE_CHUNK_Z_LO + _WS_OFFSET_FROM_END,
              SIDE_CHUNK_Z_HI - _WS_OFFSET_FROM_END):
        ws = _wood_screw_cut_at(SIDE_CHUNK_X_LO, CHUNK_Y_CENTER, z, (1, 0, 0))
        side_chunk = side_chunk.cut(ws)
        side_strip = side_strip.cut(ws)

    # Union everything FIRST, then cut the M2 holes — otherwise the strip
    # solids fill back in part of each M2 cylinder when they're union'd, and
    # the hole gets a flat spot where the strip overlaps the chunk's M2 axis.
    bracket = (top_chunk
               .union(side_chunk)
               .union(top_strip)
               .union(side_strip))
    bracket = bracket.cut(_m2_chunk_cut(TOP_M2_X_CENTER, TOP_M2_Z_CENTER))
    bracket = bracket.cut(_m2_chunk_cut(SIDE_M2_X_CENTER, SIDE_M2_Z_CENTER))
    return bracket


def _bracket_envelope():
    """The chunks + strips solid (without the bracket's own internal cuts) —
    used to carve a matching pocket into the housing.

    The HOUSING POCKETS for the two chunks are deliberately LONGER than
    the bracket chunks themselves so the housing can slide onto the
    rigidly-wall-mounted bracket without binding (the bracket's two
    chunks can't be tight-fit into two perpendicular pockets at the
    same time without one having extra play in its long axis):
      • TOP chunk pocket: extended TOP_POCKET_EXTRA_X mm in +X direction
        (22 → 23 mm along chunk's long axis). Small slop for fit ease.
      • SIDE chunk pocket: extended SIDE_POCKET_EXTRA_Z mm in +Z direction
        (22 → 30 mm along chunk's long axis). Installation sequence:
        housing starts with the bracket sitting at HIGHER +Z (the top
        chunk is further from its top-pocket floor); then the housing
        slides down in -Z while the top chunk drops into the top
        pocket. During that slide the side chunk has to sit HIGHER in
        the side pocket than its final position — so the pocket needs
        extra room in +Z (upward, toward the housing top), not -Z.
    The bracket itself is unchanged — only the housing pockets grow."""
    top_chunk  = _box(TOP_CHUNK_X_LO, TOP_CHUNK_X_HI + TOP_POCKET_EXTRA_X,
                      STRIP_Y_LO, CHUNK_Y_HI,
                      TOP_CHUNK_Z_LO, TOP_CHUNK_Z_HI)
    top_chunk  = top_chunk.edges("|X and <Z").chamfer(POCKET_CHAMFER_H)
    side_chunk = _box(SIDE_CHUNK_X_LO, SIDE_CHUNK_X_HI,
                      STRIP_Y_LO, CHUNK_Y_HI,
                      SIDE_CHUNK_Z_LO, SIDE_CHUNK_Z_HI + SIDE_POCKET_EXTRA_Z)
    side_chunk = side_chunk.edges("|Z and <X").chamfer(POCKET_CHAMFER_H)
    top_strip  = _box(TOP_STRIP_X_LO, TOP_STRIP_X_HI,
                      STRIP_Y_LO, CHUNK_Y_HI,
                      TOP_STRIP_Z_LO, TOP_STRIP_Z_HI)
    side_strip = _box(SIDE_STRIP_X_LO, SIDE_STRIP_X_HI,
                      STRIP_Y_LO, CHUNK_Y_HI,
                      SIDE_STRIP_Z_LO, SIDE_STRIP_Z_HI)
    return (top_chunk
            .union(side_chunk)
            .union(top_strip)
            .union(side_strip))


def cut_from_housing(housing):
    """Carve the bracket pocket out of the housing AND apply the standard M2
    anchor (self-tap now, heat-set insert later) at each chunk's centerline.
    The pattern matches the axle cross-pin exactly: full-Y Ø2.2 self-tap bore,
    Ø4.1 × 2 head counterbore on +Y, Ø3.3 × 3.5 insert pocket on -Y. The
    bracket chunk fills the middle of this hole with a Ø2.4 clearance
    pass-through, so the assembled M2 clamps the chunk between the two housing
    walls (head on +Y, anchor in the -Y wall). Orientation matches the axle
    cross-pin so the head pocket sits on the print's top face (no flat-bottomed
    overhang) and the insert pocket opens on the build-plate side.

    CAVEAT: the -Y anchor wall is only 4.6 mm here, so the pocket leaves just
    1.1 mm of Ø2.2 self-tap below it — under the M2_MIN_BITE (2.0 mm) rule of
    thumb. The screw holds on the first build, but this is the project's
    weakest self-tap; fitting the insert is the intended long-term state. To
    reach a full bite the -Y wall needs ~1 mm more material (see CLAUDE.md)."""
    from . import housing as _h
    out = housing.cut(_bracket_envelope(), clean=False)
    # Chamfer the housing plate material above the TOP strip. The
    # bracket envelope already ends at CHUNK_Y_HI=+4.8 so the plate at
    # y=+4.8..+STRIP_W/2 × z=TOP_STRIP_Z_LO..TOP_STRIP_Z_HI is intact
    # natural plate material — this cut takes a 45° wedge out of it,
    # leaving the chamfered shape.
    out = out.cut(_top_strip_x_chamfer_cut(), clean=False)
    out = out.cut(_side_strip_z_chamfer_cut(), clean=False)
    for (x, z) in [(TOP_M2_X_CENTER, TOP_M2_Z_CENTER),
                   (SIDE_M2_X_CENTER, SIDE_M2_Z_CENTER)]:
        out = (out
               .cut(_h._axle_pin_clearance(z, x_center=x), clean=False)
               .cut(_h._screw_head_counterbore(x, z, entry_face_sign=+1), clean=False)
               .cut(_h._axle_pin_insert_pilot(z, insert_face_sign=-1, x_center=x), clean=False))
    # Note: previously added a single-layer sacrificial bridge disc inside
    # each M2 clearance hole (drilled out post-print) to give the printer's
    # first layer above the bracket pocket a solid roof to bridge from.
    # Removed because the print needs supports inside the bracket pocket
    # anyway for a clean roof surface, and with supports present the
    # bridge layer just blocks the M2 hole during installation.
    return out


mount_bracket = _build_bracket()
