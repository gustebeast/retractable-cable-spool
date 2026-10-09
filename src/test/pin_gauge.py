"""PIN / TPU CLEARANCE GAUGE (user #1029) — the one thing the #1029
layout cannot be sure of on paper.

The pin's depth is set by its WEDGE in the lever, not by a hard bottom,
so where it comes to rest moves with print error — PIN_KEY_FLOAT either
way. The clip rides on the pin's head, so it moves with it, and if the
pin seats deep enough the clip's inboard face walks into the beam's −y
flank. CLIP_GAP is drawn at 3.2 to swallow that, which leaves 1.6 in the
worst case ON PAPER. This coupon measures it in PETG-GF.

THREE small parts, all DERIVED from the real ones (cadkit coupon rule —
never re-model; tune the design and the gauge follows):

  · pin_gauge    — the real lever_pin with a thin LIP added exactly where
                   the TPU clip's inboard face lands. Assemble, then look
                   at the gap between that lip and the frame's face: it
                   IS the clip's clearance, made visible. A feeler gauge
                   or a 1.6 offcut reads it directly.
  · lever_gauge  — the real ratchet_lever, intersected with a cylinder
                   about the pivot: just the boss, its bore and the
                   wedge's rib, so the pin seats exactly as it will in
                   the lever.
  · frame_gauge  — the frame's own bores (frame._pin_bores_side) cut out
                   of a HOUSE section (square + 45° gable, user's call):
                   the column's blind bore at one end, the beam's through
                   bore at the other, the real faces in between. It
                   prints upright with the bores horizontal, exactly as
                   frame_bottom does, so the bores distort the same way.

WHAT TO LOOK FOR: push the pin in until the wedge is tight. The lip
should stand clear of the frame's −y face. If it touches, the clip would
be pinched — raise CLIP_GAP.
"""

import cadquery as cq

from ..frame import frame_bottom, _box, _ARCH_Z0_R, _SB_Y0
from ..helpers import heal
from ..levers import lever_pin, ratchet_lever, PIN_TIP_Y, _key_slot
from ..params import (
    NOZZLE_D, LEVER_PIVOT_X, RATCHET_PIVOT_Z, LEVER_COL_Y1, BEAM_SIZE,
    BEAM_IR, FRAME_R_OUT,
    CLIP_FACE_Y, CLIP_GAP, CLIP_HUB_R, CLIP_SIDE, PIN_COL_Y0,
    LEVER_BOSS_OD, RATCHET_LEV_Y0, RATCHET_LEV_Y1, LEVER_PIN_BODY_L,
    LEVER_PIN_D, PIN_SHANK_UNDER,
    PRINT_ORIENTATION,
)

GAUGE_LIP_T = NOZZLE_D               # 0.8 — the lip is one bead thick: it
                                     # only has to be visible, not strong
GAUGE_HOUSE_W = 6 * NOZZLE_D         # 4.8 — meat around the bore, each side


def _pin_gauge():
    """The real pin + a LIP at the clip's inboard face, on a 45° cone.

    The pin prints standing on its head, so local +x runs DOWNWARD on the
    bed: the lip's far-x face is the one hanging in air, and bare it is a
    2.45 annulus of overhang (user #1034). The cone grows from the shaft
    up to the lip's OD on that side, which is self-supporting and leaves
    the lip's near-x face — the one you measure from — flat and sharp."""
    x_lip = PIN_TIP_Y + CLIP_FACE_Y + CLIP_GAP      # the clip's inner face
    r_shank = LEVER_PIN_D / 2.0 - PIN_SHANK_UNDER / 2.0
    lip = (cq.Workplane("YZ").workplane(offset=x_lip)
           .circle(CLIP_HUB_R).extrude(GAUGE_LIP_T))
    cone = (cq.Workplane("YZ").workplane(offset=x_lip + GAUGE_LIP_T)
            .circle(CLIP_HUB_R)
            .workplane(offset=CLIP_HUB_R - r_shank).circle(r_shank).loft())
    # re-cut the KEY SLOT through the lot: the lip and its cone bridge
    # the slot where they sit, and the cone's far end then leaves a wall
    # across it — a flat facing straight down in this print (#1034)
    return heal(lever_pin.union(lip).union(cone)
                .cut(_key_slot(LEVER_PIN_BODY_L + 0.5)))


def _lever_gauge():
    """The real lever's pivot boss — bore, wedge rib and all."""
    keep = cq.Workplane(obj=cq.Solid.makeCylinder(
        LEVER_BOSS_OD / 2.0 + 2.0, (RATCHET_LEV_Y1 - RATCHET_LEV_Y0) + 2.0,
        cq.Vector(LEVER_PIVOT_X, RATCHET_LEV_Y0 - 1.0, RATCHET_PIVOT_Z),
        cq.Vector(0, 1, 0)))
    return heal(ratchet_lever.intersect(keep))


# the crop: the fork column and the arch over it — the real frame's own
# lever mount, not a stand-in (user #1031). Top is the arch's run into
# the beam, bottom is far enough under the pivot to carry the thrust
# bosses; the pin's bores live in between.
GAUGE_Z1 = _ARCH_Z0_R + (_SB_Y0 - BEAM_SIZE / 2.0) / 2.0 + 2 * NOZZLE_D
GAUGE_Z0 = RATCHET_PIVOT_Z - LEVER_BOSS_OD        # under the bosses
GAUGE_BASE_T = 2 * NOZZLE_D                       # 1.6 — the raft that ties
                                   # the column and the beam together (user
                                   # #1031: cropped alone they stand free)


def _frame_gauge():
    """The REAL frame_bottom, cropped to the lever mount and given a
    1.6 base (user #1031). The crop keeps the fork column, the 45° arch
    over it, both thrust bosses and the pin's two bores; the base is a
    plain slab under the whole footprint, because the column and the
    beam are separate uprights at that height and would otherwise print
    as two loose towers.
    Its −y face ends EXACTLY on the beam's flank, so the gap you see
    between the pin's lip and that face IS the clip's clearance — no
    mental arithmetic."""
    y0, y1 = -CLIP_FACE_Y, LEVER_COL_Y1
    # starts AT the beam's inner radius (user #1032): reaching further
    # −x only caught a slab of the wall band, which holds nothing here
    x0, x1 = BEAM_IR, FRAME_R_OUT + 1.0
    crop = _box(x0, x1, y0, y1, GAUGE_Z0, GAUGE_Z1)
    g = frame_bottom.intersect(crop)
    base = _box(x0, x1, y0, y1, GAUGE_Z0 - GAUGE_BASE_T, GAUGE_Z0)
    return heal(g.union(base))


def _pin_in_place():
    """The gauge pin at its seated position — the same transform
    levers.lever_pin_in_place uses, applied to the lipped pin."""
    return (_pin_gauge()
            .rotate((0, 0, 0), (1, 0, 0), 180.0)
            .rotate((0, 0, 0), (0, 0, 1), CLIP_SIDE * 90.0)
            .translate((LEVER_PIVOT_X, PIN_TIP_Y, RATCHET_PIVOT_Z)))


# THE PRINTS — one STEP each, so any of the three can be reprinted on its
# own, in the orientation its real counterpart prints in: the frame's crop
# on its new base, the lever's boss on its face, the pin on its head.
# A COUPON IS A PRINTED PART, so it declares its bed face like one — and
# it must be the SAME face as the part it stands in for, or the coupon
# tests a print the real part never makes (cadkit #1070). So each coupon
# reads the REAL part's declaration rather than restating the rotation —
# they cannot drift apart, which is what used to happen.
GAUGE_PRINTS = (
    ("test_gauge_frame.step", _frame_gauge(),
     PRINT_ORIENTATION["frame_bottom"]),
    ("test_gauge_lever.step", _lever_gauge(),
     PRINT_ORIENTATION["ratchet_lever"]),
    ("test_gauge_pin.step", _pin_gauge(), PRINT_ORIENTATION["lever_pin"]),
)

# THE VIEWER — the same three where they actually meet, each its own
# component, so the lip's gap to the frame's face is visible in the tab
GAUGE_ASSEMBLED = (("gauge_frame", _frame_gauge()),
                   ("gauge_lever", _lever_gauge()),
                   ("gauge_pin", _pin_in_place()))
