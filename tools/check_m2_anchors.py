"""Measure the REAL self-tap bite at every M2 anchor in the built parts.

A nominal bore `depth` cannot see a thin wall: the bracket screws bore straight
through the housing (nominal bite ~20 mm) but only cross 4.6 mm of material, so
their true bite is 1.1 mm. Only the finished solid knows. `fasteners.measured_bite`
walks the solid just past each pocket and returns the contiguous run where the
Ø2.2 bore is really surrounded by plastic.

    py -3.12 -m tools.check_m2_anchors     # exit 1 if any un-acknowledged site
                                           # falls below M2.min_bite

Sites whose bite is knowingly short carry a `short_bite` note here — the same
escape `cut_anchor()` demands, kept in one greppable place.
"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "freecad"))

from fasteners import M2, measured_bite  # noqa: E402
from src import housing as H  # noqa: E402
from src import mount_bracket as MB
from src.dimensions import PANCAKE_CROSS_PIN_Z

# (label, part, mouth_pt, direction, short_bite_reason or None)
SITES = [
    ("ratchet pivot", "housing",
     (H.RATCHET_PIVOT_X, H.HOUSING_W / 2 + H.HOUSING_BOSS_EXT, H.RATCHET_PIVOT_Z),
     (0, -1, 0), None),
    ("brake pivot", "brake_pin_chunk",
     (H.BRAKE_PIVOT_X, -(H.HOUSING_W / 2 + H.HOUSING_BOSS_EXT), H.BRAKE_PIVOT_Z),
     (0, 1, 0), None),
    ("axle cross-pin", "housing",
     (0.0, H.HOUSING_W / 2, PANCAKE_CROSS_PIN_Z),
     (0, -1, 0), None),
    ("bracket TOP M2", "housing",
     (MB.TOP_M2_X_CENTER, -H.HOUSING_W / 2, MB.TOP_M2_Z_CENTER),
     (0, 1, 0),
     "the -Y anchor wall is only 4.6 mm; widening it would move the bracket's "
     "mating face. Weakest self-tap in the project — the insert is the intended "
     "long-term state here."),
    ("bracket SIDE M2", "housing",
     (MB.SIDE_M2_X_CENTER, -H.HOUSING_W / 2, MB.SIDE_M2_Z_CENTER),
     (0, 1, 0),
     "same 4.6 mm -Y wall as the TOP M2."),
]


def main() -> None:
    parts = {"housing": H.housing.val(), "brake_pin_chunk": H.brake_pin_chunk.val()}
    print(f"M2: min_bite = {M2.min_bite} mm (5 threads x {M2.pitch} pitch), "
          f"anchor_min_wall = {M2.anchor_min_wall} mm\n")
    failures = 0
    for label, part, pnt, direction, short in SITES:
        bite = measured_bite(parts[part], pnt, direction, M2)
        if bite >= M2.min_bite:
            status = "OK"
        elif short:
            status = "SHORT (acknowledged)"
        else:
            status = "FAIL"
            failures += 1
        print(f"  {label:18s} {part:16s} bite = {bite:5.2f} mm   {status}")
        if short and bite < M2.min_bite:
            print(f"      ^ {short}")
    print()
    if failures:
        print(f"{failures} anchor(s) below min_bite with no short_bite note.")
    else:
        print("All anchors clear min_bite, or are explicitly acknowledged.")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
