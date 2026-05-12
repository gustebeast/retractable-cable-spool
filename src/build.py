"""Retractable Cable Spool - main build script.

Run from the repo root:
  py -3.12 -m src.build              # build all parts + assembly
  py -3.12 -m src.build --part NAME  # build only one part (faster iteration)
  py -3.12 -m src.build --list       # list available part names

Produces all printed-part STEPs and an assembled assembly.step in cwd.

Build order:
  spool          → main_body (initial)
  caps           → bearing caps + pancake_spool
  axle           → axle
  housing        → housing
  housing_pins   → brake_housing_pin, brake_housing_rest_pin
  guide_axle     → guide_axle_part
  levers         → ratchet_lever, brake_lever
  viz            → ratchet_spring, brake_spring, dummy 608 bearings

Then this module composes the final main_body by applying caps' cap-key
groove cuts and levers' ratchet-teeth + drum-cable-hole cuts. The
per-module ``apply_to_main_body(body)`` functions are pure — no side
effects on imported modules.
"""

import argparse
import sys

import cadquery as cq

from .helpers import heal

from . import spool
from . import caps
from . import axle as _axle_mod
from . import housing as _housing_mod
from . import housing_pins as _housing_pins_mod
from . import housing_guide as _guide_mod
from . import housing_lever_guide as _lever_guide_mod
from . import levers as _lev_mod
from . import viz as _viz_mod

# Compose the final main_body by applying the cap-key grooves and the
# lever-dependent cuts (ratchet teeth, drum cable hole) explicitly.
main_body = caps.apply_to_main_body(spool.main_body)
# Heal before the lever cuts — the drum's groove-following inner relief
# (built in spool.py) leaves geometry that's valid but fragile, and the
# cable-transit slot cut (which passes right through the relieved drum
# wall) silently no-ops on it otherwise.
main_body = heal(main_body)
main_body = _lev_mod.apply_to_main_body(main_body)

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
    "ratchet_spring":         (ratchet_spring,             "ratchet_spring.step",         "dummy — purchased torsion spring"),
    "brake_spring":           (brake_spring,               "brake_spring.step",           "dummy — purchased torsion spring"),
    "brake_pad_rubber":       (brake_pad_rubber,           "brake_pad_rubber.step",       "dummy — 3.3 mm rubber pad bonded to printed brake pad"),
    "brake_housing_pin":      (brake_housing_pin,          "brake_housing_pin.step",      "print separately — glue-install"),
    "brake_housing_rest_pin": (brake_housing_rest_pin,     "brake_housing_rest_pin.step", "print separately — glue-install"),
    "back_axle":              (back_axle,                  "back_axle.step",              "cable-retention guide axle (rides in the back 608 bearing)"),
    "back_axle_block":        (back_axle_block,            "back_axle_block.step",        "sliding bearing carrier (tenon clamps into the housing mortise)"),
    "guide_wheel":               (guide_wheel,              "guide_wheel.step",               "pancake-side printed guide wheel; user paints rubber on OD"),
    "lever_guide_wheel":         (lever_guide_wheel,        "lever_guide_wheel.step",         "lever-side printed guide wheel; user paints rubber on OD"),
}


def _export(name):
    obj, path, note = PARTS[name]
    cq.exporters.export(obj, path)
    suffix = f"  ({note})" if note else ""
    print(f"Wrote {path}{suffix}")


# Visualization aid — set True to rotate the ratchet lever to its
# fully-disconnected position (= max CW throw at RATCHET_OUTER_TRAVEL_DEG)
# in the assembly.step. Useful for eyeballing pawl-to-tooth clearance.
SHOW_RATCHET_DISCONNECTED = False


def _ratchet_lever_for_assembly():
    if not SHOW_RATCHET_DISCONNECTED:
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


def _export_assembly():
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
        .add(axle,          name="axle")
        .add(housing, name="housing")
        .add(_ratchet_lever_for_assembly(), name="ratchet_lever")
        .add(brake_lever,   name="brake_lever")
        .add(ratchet_spring, name="ratchet_spring")
        .add(brake_spring,   name="brake_spring")
        .add(brake_pad_rubber, name="brake_pad_rubber")
        .add(brake_housing_pin, name="brake_housing_pin")
        .add(brake_housing_rest_pin, name="brake_housing_rest_pin")
    )
    assembly.save("assembly.step")
    print("Wrote assembly.step"
          + ("  [ratchet shown DISCONNECTED]" if SHOW_RATCHET_DISCONNECTED else ""),
          flush=True)
    _push_onshape()


def _push_onshape() -> None:
    """Best-effort upload of assembly.step to an Onshape blob element via
    tools/onshape_push.py. No-op unless tools/onshape_credentials.json (or
    the ONSHAPE_* env vars) are configured. Never fatal to the build."""
    import os, pathlib, subprocess
    script = pathlib.Path(__file__).resolve().parent.parent / "tools" / "onshape_push.py"
    if not script.exists():
        return
    try:
        subprocess.run([sys.executable, str(script), "assembly.step"],
                       check=False, cwd=os.getcwd())
    except Exception as e:                                  # noqa: BLE001 — push must never break the build
        print(f"[onshape] push step skipped: {e}", file=sys.stderr)


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
