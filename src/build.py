"""Retractable Cable Spool - main build script.

Run from the repo root:
  py -3.12 -m src.build              # build all parts + assembly
  py -3.12 -m src.build --part NAME  # build only one part (faster iteration)
  py -3.12 -m src.build --list       # list available part names

Produces all printed-part STEPs and an assembled assembly.step in cwd.

Build order:
  spool             → main_body (initial)
  caps              → bearing_cap_bottom + cap-key grooves
  axle              → axle
  housing           → housing, back_axle, back_axle_block
  housing_pins      → brake_housing_pin, brake_housing_rest_pin
  housing_guide     → guide_wheel (pancake-side cable-pinch wheel)
  housing_lever_guide → lever_guide_wheel (lever-side cable-pinch wheel)
  levers            → ratchet_lever, brake_lever
  mount_bracket     → mount_bracket
  viz               → ratchet_spring, brake_spring, brake_pad_rubber,
                      dummy 608 bearings (assembly-only — not exported)

Then this module composes the final main_body by applying caps' cap-key
groove cuts and levers' ratchet-teeth + drum-cable-hole cuts. The
per-module ``apply_to_main_body(body)`` functions are pure — no side
effects on imported modules.
"""

import argparse
import pathlib
import sys

import cadquery as cq

from .helpers import heal
from .dimensions import SPOOL_H

from . import spool
from . import caps
from . import axle as _axle_mod
from . import housing as _housing_mod
from . import housing_pins as _housing_pins_mod
from . import housing_guide as _guide_mod
from . import housing_lever_guide as _lever_guide_mod
from . import levers as _lev_mod
from . import mount_bracket as _bracket_mod
from . import viz as _viz_mod

# Compose the final main_body by applying the cap-key grooves and the
# lever-dependent cuts (ratchet teeth, drum cable hole) explicitly.
main_body = caps.apply_to_main_body(spool.main_body)
main_body = _lev_mod.apply_to_main_body(main_body)
# V-notch helical cable groove — cut LAST so the fragile swept geometry
# never has to survive another boolean. The drum leaves spool.py as a
# plain thick cylindrical shell; this cut carves the angled outer wall.
main_body = main_body.cut(spool._helical_v_groove_cut())

# Re-bind module-level part variables for export.
bearing_cap_bottom     = caps.bearing_cap_bottom
axle                   = _axle_mod.axle
housing                = _housing_mod.housing
back_axle              = _housing_mod.back_axle
back_axle_block        = _housing_mod.back_axle_block
brake_housing_pin      = _housing_pins_mod.brake_housing_pin
brake_housing_rest_pin = _housing_pins_mod.brake_housing_rest_pin
guide_wheel              = _guide_mod.guide_wheel
lever_guide_wheel        = _lever_guide_mod.lever_guide_wheel
mount_bracket            = _bracket_mod.mount_bracket
ratchet_lever          = _lev_mod.ratchet_lever
brake_lever            = _lev_mod.brake_lever
ratchet_spring         = _viz_mod.ratchet_spring
brake_spring           = _viz_mod.brake_spring
brake_pad_rubber       = _viz_mod.brake_pad_rubber
bearing_top            = _viz_mod.bearing_top
bearing_bottom         = _viz_mod.bearing_bottom
back_axle_bearing      = _viz_mod.back_axle_bearing

# ────────────────────────────────────────────────────────────────────────────
# Export — individual parts for printing, + a combined assembly STEP for
# dimensional verification (bearings & spring not modelled, since they're
# purchased parts).
# ────────────────────────────────────────────────────────────────────────────


main_body = heal(main_body)
axle = heal(axle)
housing = heal(housing)
ratchet_lever = heal(ratchet_lever)
brake_lever   = heal(brake_lever)
# Springs accumulate near-tangent boolean fuses between rings/legs/spheres
# — heal them too so strict STEP importers (Onshape, etc.) accept the
# resulting compound without face-tolerance complaints.
ratchet_spring = heal(ratchet_spring)
brake_spring   = heal(brake_spring)
brake_pad_rubber = heal(brake_pad_rubber)
back_axle = heal(back_axle)
back_axle_block = heal(back_axle_block)

# Bearing cap is built in its assembled position; for a clean single-
# part STEP export it stays as-is (already at z=0..CAP_H). The
# assembly uses this positioned version.
bearing_cap_bottom_export = bearing_cap_bottom

# Map of part name → (workplane, output filename, optional note).
PARTS = {
    "main_body":              (main_body,                  "spool_main_body.step",        None),
    "bearing_cap_bottom":     (bearing_cap_bottom_export,  "bearing_cap_bottom.step",     None),
    "axle":                   (axle,                       "axle.step",                   None),
    "housing":                (housing,                    "housing.step",                None),
    "ratchet_lever":          (ratchet_lever,              "ratchet_lever.step",          None),
    "brake_lever":            (brake_lever,                "brake_lever.step",            None),
    # Springs and the rubber pad are purchased/applied parts — they're
    # included in assembly.step for visualization only, no need to export
    # them as individual STEP files for printing.
    "brake_housing_pin":      (brake_housing_pin,          "brake_housing_pin.step",      "print separately — glue-install"),
    "brake_housing_rest_pin": (brake_housing_rest_pin,     "brake_housing_rest_pin.step", "print separately — glue-install"),
    "back_axle":              (back_axle,                  "back_axle.step",              "cable-retention guide axle (rides in the back 608 bearing)"),
    "back_axle_block":        (back_axle_block,            "back_axle_block.step",        "sliding bearing carrier (tenon clamps into the housing mortise)"),
    "guide_wheel":               (guide_wheel,              "guide_wheel.step",               "pancake-side printed guide wheel; user paints rubber on OD"),
    "lever_guide_wheel":         (lever_guide_wheel,        "lever_guide_wheel.step",         "lever-side printed guide wheel; user paints rubber on OD"),
    "mount_bracket":             (mount_bracket,            "mount_bracket.step",             "L-shaped wood-screw mount; housing M2-clamps to it"),
}


def _export(name):
    obj, path, note = PARTS[name]
    cq.exporters.export(obj, path)
    suffix = f"  ({note})" if note else ""
    print(f"Wrote {path}{suffix}")


# Visualization aid — when True, both levers are rotated to their fully-
# actuated (ENGAGED) position in assembly.step: ratchet pawl raised clear
# of the teeth (RATCHET_OUTER_TRAVEL_DEG of CW throw), brake pad descended
# onto the brake ring (BRAKE_INNER_TRAVEL_DEG of CCW throw). Per the
# matched-ROM design these happen simultaneously, so showing both engaged
# is the meaningful "actuated" snapshot of the mechanism. Set False to
# show both levers at REST instead.
SHOW_LEVERS_ENGAGED = True


def _ratchet_lever_for_assembly():
    if not SHOW_LEVERS_ENGAGED:
        return ratchet_lever
    from .housing import RATCHET_PIVOT_X, LEVER_PIVOT_Z
    from .levers import RATCHET_OUTER_TRAVEL_DEG
    # User pulls the handle UP (+z) to disengage; class-1 lever rotates so
    # the lever pin (on the -X side of the pivot) moves DOWN. With +Y
    # rotation taking +X toward -Z, that's NEGATIVE Y rotation.
    angle = -RATCHET_OUTER_TRAVEL_DEG
    return (
        ratchet_lever
        .translate((-RATCHET_PIVOT_X, 0, -LEVER_PIVOT_Z))
        .rotate((0, 0, 0), (0, 1, 0), angle)
        .translate((RATCHET_PIVOT_X, 0, LEVER_PIVOT_Z))
    )


def _brake_lever_for_assembly():
    if not SHOW_LEVERS_ENGAGED:
        return brake_lever
    return _brake_engaged_rotation(brake_lever)


def _brake_pad_rubber_for_assembly():
    if not SHOW_LEVERS_ENGAGED:
        return brake_pad_rubber
    # The rubber pad is a separate viz-only solid added to the assembly
    # alongside the lever — it has its own copy of the lever's rest-pose
    # pre-rotation in viz.py. To follow the lever into the engaged pose
    # it needs the same rotation applied here.
    return _brake_engaged_rotation(brake_pad_rubber)


def _brake_engaged_rotation(part):
    """Rotate `part` from the brake's REST pose to its ENGAGED pose.

    Both the printed brake lever and the rubber pad are built at the
    ENGAGED design pose and then pre-rotated +BRAKE_INNER_TRAVEL_DEG to
    reach the REST (print) pose (see _brake_pad_contact in levers.py and
    _brake_pad_rubber in viz.py). Applying -BRAKE_INNER_TRAVEL_DEG here
    undoes that pre-rotation and returns the part to the engaged pose —
    the brake pad rises onto the brake ring (+Z) at the same lever
    throw that the ratchet pawl clears the teeth."""
    from .housing import BRAKE_PIVOT_X, LEVER_PIVOT_Z
    from .levers import BRAKE_INNER_TRAVEL_DEG
    angle = -BRAKE_INNER_TRAVEL_DEG
    return (
        part
        .translate((-BRAKE_PIVOT_X, 0, -LEVER_PIVOT_Z))
        .rotate((0, 0, 0), (0, 1, 0), angle)
        .translate((BRAKE_PIVOT_X, 0, LEVER_PIVOT_Z))
    )


# ── Build counter — a 3D number floating well above the assembly, bumped
# on every full build. Lets you see at a glance in Onshape that a fresh
# build landed (the number ticks up), without poking at any real part.
# Stored in tools/build_counter.txt (gitignored); starts at 1 if missing.
_BUILD_COUNTER_FILE = pathlib.Path(__file__).resolve().parent.parent / "tools" / "build_counter.txt"


def _bump_build_counter() -> int:
    try:
        n = int(_BUILD_COUNTER_FILE.read_text().strip()) + 1
    except (OSError, ValueError):
        n = 1
    try:
        _BUILD_COUNTER_FILE.write_text(f"{n}\n")
    except OSError:                                         # noqa: BLE001 — counter is best-effort
        pass
    return n


def _build_counter_model(n: int):
    """Upright extruded number, centred at X=0 and floating ~50 mm above
    the top of the spool (well clear of the housing). Returns None if the
    text engine isn't available — a font hiccup must not break the build."""
    try:
        return (
            cq.Workplane("XZ")
            .center(0, SPOOL_H + 50)
            .text(str(n), 30, 6)
        )
    except Exception:                                       # noqa: BLE001
        return None


def _export_assembly():
    build_n = _bump_build_counter()
    assembly = (
        cq.Assembly(name="retractable_cable_spool")
        .add(main_body,     name="main_body")
        .add(bearing_cap_bottom, name="bearing_cap_bottom", loc=cq.Location((0, 0, 0)))
        .add(bearing_bottom,     name="bearing_bottom",     loc=cq.Location((0, 0, 0)))
        .add(bearing_top,        name="bearing_top",        loc=cq.Location((0, 0, 0)))
        .add(back_axle_bearing,  name="back_axle_bearing",  loc=cq.Location((0, 0, 0)))
        .add(back_axle_block,    name="back_axle_block",    loc=cq.Location((0, 0, 0)))
        .add(back_axle,          name="back_axle",          loc=cq.Location((0, 0, 0)))
        .add(guide_wheel,            name="guide_wheel",            loc=cq.Location((0, 0, 0)))
        .add(lever_guide_wheel,      name="lever_guide_wheel",      loc=cq.Location((0, 0, 0)))
        .add(mount_bracket,          name="mount_bracket",          loc=cq.Location((0, 0, 0)))
        .add(axle,          name="axle")
        .add(housing, name="housing")
        .add(_ratchet_lever_for_assembly(), name="ratchet_lever")
        .add(_brake_lever_for_assembly(),   name="brake_lever")
        .add(ratchet_spring, name="ratchet_spring")
        .add(brake_spring,   name="brake_spring")
        .add(_brake_pad_rubber_for_assembly(), name="brake_pad_rubber")
        .add(brake_housing_pin, name="brake_housing_pin")
        .add(brake_housing_rest_pin, name="brake_housing_rest_pin")
    )
    counter = _build_counter_model(build_n)
    if counter is not None:
        assembly.add(counter, name="build_counter")
    assembly.save("assembly.step")
    print(f"Wrote assembly.step  [build #{build_n}]"
          + ("  [levers ENGAGED]" if SHOW_LEVERS_ENGAGED else ""),
          flush=True)
    _push_onshape()


def _push_onshape() -> None:
    """Best-effort upload of assembly.step to Onshape via
    tools/onshape_push.py. No-op unless tools/onshape_credentials.json
    (or the ONSHAPE_* env vars) are configured. Never fatal to the build."""
    import os, subprocess
    script = pathlib.Path(__file__).resolve().parent.parent / "tools" / "onshape_push.py"
    if not script.exists():
        return
    files = [f for f in ("assembly.step",) if pathlib.Path(f).exists()]
    if not files:
        return
    try:
        subprocess.run([sys.executable, str(script), *files],
                       check=False, cwd=os.getcwd())
    except Exception as e:                                  # noqa: BLE001 — push must never break the build
        print(f"[onshape] push skipped: {e}", file=sys.stderr)


def main() -> None:
    p = argparse.ArgumentParser(prog="src.build", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--part", help="Build only this part (skips assembly).")
    p.add_argument("--list", action="store_true", help="List part names and exit.")
    args = p.parse_args()

    if args.list:
        print("assembly")
        for name in PARTS:
            print(name)
        return

    if args.part:
        if args.part == "assembly":
            _export_assembly()
            return
        if args.part not in PARTS:
            print(f"unknown part: {args.part!r}. Use --list to see options.",
                  file=sys.stderr)
            sys.exit(2)
        _export(args.part)
        return

    for name in PARTS:
        _export(name)
    _export_assembly()


main()
