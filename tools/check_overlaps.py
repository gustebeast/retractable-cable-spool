"""Overlap gate for the v3 assembly (cadkit.overlap_check engine).

  py -3.12 -m tools.check_overlaps        # exit code = unintended pairs; 0 = clean
  py -3.12 -m tools.check_overlaps --all  # also list the intended contacts
"""

from __future__ import annotations

import argparse
import sys

from cadkit.overlap_check import run

from src.frame import frame_top, frame_bottom
from src.mount import mount_ring
from src.wall import wall_top, wall_lock
from src.axle import axle_top, axle_separator
from src.lid import lid
from src.levers import (ratchet_lever, brake_lever, brake_pad_tpu,
                        lever_pin_in_place)
from src.params import RATCHET_PIVOT_Z, BRAKE_PIVOT_Z
from src.dummies import bearing_608, bearing_608_bottom, spring


def collect_components():
    return [
        ("frame_top",   frame_top),
        ("frame_bottom", frame_bottom),
        ("wall_top",    wall_top),
        *[(f"wall_lock_{a}",
           wall_lock.rotate((0, 0, 0), (0, 0, 1), a))
          for a in (0, 90, 180, 270)],
        ("mount_ring",  mount_ring),
        ("axle_top",    axle_top),
        ("axle_separator", axle_separator),
        ("lid",         lid),
        ("ratchet_lever", ratchet_lever),
        ("brake_lever",   brake_lever),
        ("brake_pad_tpu", brake_pad_tpu),
        ("ratchet_pin", lever_pin_in_place(RATCHET_PIVOT_Z, +1)),
        ("brake_pin",   lever_pin_in_place(BRAKE_PIVOT_Z, -1)),
        ("bearing_608", bearing_608),
        ("bearing_608_bottom", bearing_608_bottom),
        ("spring",      spring),
    ]


# Designed contacts (face-on-face seats — should carry no real common volume,
# whitelisted so numeric grazing never fails the gate):
_INTENDED = {
    frozenset(p) for p in [
        ("bearing_608", "frame_top"),    # outer race in the pocket, on the lip
        ("bearing_608", "axle_top"),     # shaft in the bore; lip on the inner race
        ("spring", "frame_top"),         # top flange against the frame underside
        ("axle_top", "axle_separator"),     # tenon glued in the mortise
        ("lid", "axle_separator"),  # bayonet seated: lid on the wall top,
                                    # tenons in the retained cavities
        ("frame_top", "frame_bottom"),   # arc joints seated on the beam tops
        ("wall_top", "frame_bottom"),
        ("bearing_608_bottom", "axle_separator"),  # outer race in the disk's
                                                   # underside pocket + seat
        ("bearing_608_bottom", "frame_bottom"),    # bore on the stub, inner
                                                   # race on the boss shoulder
        ("ratchet_lever", "axle_separator"),  # pawl MESHED in the teeth at
                                              # rest (exact-negative edge)
        ("brake_pad_tpu", "brake_lever"),     # hook slide joint seated
        ("ratchet_pin", "frame_bottom"),      # keyed diamond bores
        ("brake_pin",   "frame_bottom"),
        ("ratchet_pin", "ratchet_lever"),     # keyed lever pockets
        ("brake_pin",   "brake_lever"),
    ]
}
# each lock strip: in its beam channel, resting on wall_top, capped by
# frame_top's underside
for _a in (0, 90, 180, 270):
    for _p in ("frame_bottom", "wall_top", "frame_top"):
        _INTENDED.add(frozenset((f"wall_lock_{_a}", _p)))
# the mount ring: seated in the four arms' arc channels (tenons at the
# stops, spine in the slots/V-grooves — face contacts)
_INTENDED.add(frozenset(("mount_ring", "frame_top")))


def intended(a: str, b: str) -> bool:
    return frozenset((a, b)) in _INTENDED


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--all", action="store_true", help="also list intended contacts")
    p.add_argument("--jobs", type=int, default=None)
    args = p.parse_args()
    comps = [(n, wp.val()) for n, wp in collect_components()]
    return run(comps, intended, jobs=args.jobs, show_all=args.all)


if __name__ == "__main__":
    sys.exit(main())
