"""DUMMY purchased parts — assembly-only fit-check models (cadkit rule:
no standalone STEP for springs/bearings; they appear only in assembly.step).

Both are modelled IN PLACE (world/assembly coordinates from params).
"""

import cadquery as cq

from .helpers import cone_solid, cyl, heal
from .params import (
    FRAME_R_OUT, FRAME_TOP_Z1, MOUNT_TAIL_TIP_X,
    COUCH_TOP_BEAM_W, COUCH_FRONT_T, COUCH_FRONT_H,
    BEARING_OD, BEARING_ID, BEARING_W, BEARING_ORACE_ID, BEARING_IRACE_OD,
    BEARING_INNER_CH,
    BRG_POCKET_Z0, BOT_BRG_Z0,
    SPRING_FLANGE_OD, SPRING_FLANGE_T, SPRING_BODY_OD, SPRING_H,
    SPRING_BORE_TOP_D, SPRING_BORE_BOT_D, SPRING_Z0, SPRING_Z1,
)

# cosmetic groove depth so the three bearing bands (outer race / seal zone /
# inner race) read in the viewer — not a real bearing dimension
_RACE_GROOVE = 0.6


def _build_bearing_608(z0):
    """608: full annulus with a shallow face groove between the two race
    face annuli on each side, base at z0. TWO in the design now: the TOP
    one in the frame pocket (inner race carries the hanging axle), the
    BOTTOM one in the separator's underside pocket riding the frame
    floor's stub (outer race co-rotates with the disk)."""
    b = cyl(BEARING_OD, BEARING_W, z=0).cut(cyl(BEARING_ID, BEARING_W + 1.0, z=-0.5))
    for z in (0.0, BEARING_W - _RACE_GROOVE):
        b = b.cut(cyl(BEARING_ORACE_ID, _RACE_GROOVE, z=z)
                  .cut(cyl(BEARING_IRACE_OD, _RACE_GROOVE + 1.0, z=z - 0.5)))
    # the REAL ring-edge chamfers (spec r_s ≈ 0.3, all four edges) — the
    # bottom seat cone face-mates the OD-top one; a sharp-cornered dummy
    # falsely interpenetrates it (probe-caught)
    ch = BEARING_INNER_CH
    b = b.cut(cyl(BEARING_OD + 2.0, ch, z=BEARING_W - ch)
              .cut(cone_solid(BEARING_OD, BEARING_OD - 2.0 * ch,
                              ch, BEARING_W - ch)))
    b = b.cut(cyl(BEARING_OD + 2.0, ch, z=0.0)
              .cut(cone_solid(BEARING_OD - 2.0 * ch, BEARING_OD, ch, 0.0)))
    b = b.cut(cone_solid(BEARING_ID, BEARING_ID + 2.0 * ch,
                         ch, BEARING_W - ch))
    b = b.cut(cone_solid(BEARING_ID + 2.0 * ch, BEARING_ID, ch, 0.0))
    return heal(b.translate((0.0, 0.0, z0)))


def _build_spring():
    """The UNMODIFIED spring, installed UPSIDE DOWN (user #878: the axle
    spins in v3, flipping the wind chirality): Ø SPRING_BODY_OD coil
    body between two Ø SPRING_FLANGE_OD × SPRING_FLANGE_T flanges,
    SPRING_H total. The arbor holes DIFFER (user-measured), and the
    flip puts the BIG one on top: Ø13.4 through the TOP flange + body
    (the mortise collar's entry, from above), Ø8.7 through the bottom
    flange (the bare tenon post only). Top flange against the frame
    underside (the body gets fixed to the frame later)."""
    s = cyl(SPRING_BODY_OD, SPRING_H, z=SPRING_Z0)
    s = s.union(cyl(SPRING_FLANGE_OD, SPRING_FLANGE_T, z=SPRING_Z0))
    s = s.union(cyl(SPRING_FLANGE_OD, SPRING_FLANGE_T,
                    z=SPRING_Z1 - SPRING_FLANGE_T))
    s = s.cut(cyl(SPRING_BORE_BOT_D, SPRING_H + 1.0, z=SPRING_Z0 - 0.5))
    s = s.cut(cyl(SPRING_BORE_TOP_D,
                  (SPRING_Z1 + 0.5) - (SPRING_Z0 + SPRING_FLANGE_T),
                  z=SPRING_Z0 + SPRING_FLANGE_T))
    return heal(s)


def _box(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane("XY").workplane(offset=z0)
            .center((x0 + x1) / 2.0, (y0 + y1) / 2.0)
            .rect(x1 - x0, y1 - y0).extrude(z1 - z0))


def _build_couch_beams():
    """The couch install site (user): a 30 sq TOP beam along x resting
    on the frame's top face, centred on the axle, and a FRONT beam
    along y butted flush against the assembly's +x face (levers +
    cable exit), spanning the frame's y extent, 77 tall with its top
    flush with the top beam's top. Its face sits at the mount's
    GUIDE-TAIL tips (MOUNT_TAIL_TIP_X — flush on the desk build, the
    rotate-on set-back past the face in the couch version). Assembly-
    only, for the install fit-check."""
    top = _box(-FRAME_R_OUT - COUCH_FRONT_T, FRAME_R_OUT + COUCH_FRONT_T,
               -COUCH_TOP_BEAM_W / 2.0, COUCH_TOP_BEAM_W / 2.0,
               FRAME_TOP_Z1, FRAME_TOP_Z1 + COUCH_TOP_BEAM_W)
    z_top = FRAME_TOP_Z1 + COUCH_TOP_BEAM_W
    front = _box(MOUNT_TAIL_TIP_X, MOUNT_TAIL_TIP_X + COUCH_FRONT_T,
                 -FRAME_R_OUT, FRAME_R_OUT,
                 z_top - COUCH_FRONT_H, z_top)
    return top, front


bearing_608 = _build_bearing_608(BRG_POCKET_Z0)
bearing_608_bottom = _build_bearing_608(BOT_BRG_Z0)
spring = _build_spring()
couch_top_beam, couch_front_beam = _build_couch_beams()
