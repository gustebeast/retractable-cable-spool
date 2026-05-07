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
from . import guide_axle as _guide_mod
from . import lever_guide_axle as _lever_guide_mod
from . import levers as _lev_mod
from . import viz as _viz_mod

# Compose the final main_body by applying the cap-key grooves and the
# lever-dependent cuts (ratchet teeth, drum cable hole) explicitly.
main_body = caps.apply_to_main_body(spool.main_body)
main_body = _lev_mod.apply_to_main_body(main_body)

# Re-bind module-level part variables for export.
bearing_cap_bottom     = caps.bearing_cap_bottom
bearing_cap_top        = caps.bearing_cap_top
axle                   = _axle_mod.axle
housing                = _housing_mod.housing
brake_housing_pin      = _housing_pins_mod.brake_housing_pin
brake_housing_rest_pin = _housing_pins_mod.brake_housing_rest_pin
guide_axle_part        = _guide_mod.guide_axle_part
lever_guide_axle_part  = _lever_guide_mod.lever_guide_axle_part
ratchet_lever          = _lev_mod.ratchet_lever
brake_lever            = _lev_mod.brake_lever
ratchet_spring         = _viz_mod.ratchet_spring
brake_spring           = _viz_mod.brake_spring
bearing_top            = _viz_mod.bearing_top
bearing_bottom         = _viz_mod.bearing_bottom
bearing_guide          = _viz_mod.bearing_guide
bearing_lever_guide    = _viz_mod.bearing_lever_guide

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
# bearing_cap, levers, pin, spring viz are simple enough to skip

# Bearing caps are built in their assembled positions; for clean single-
# part STEP exports, translate each to a z=0 origin. The assembly uses
# the original positioned versions.
bearing_cap_bottom_export = bearing_cap_bottom
bearing_cap_top_export    = bearing_cap_top.translate((0, 0, -caps._top_cap_body_z0))

# Map of part name → (workplane, output filename, optional note).
PARTS = {
    "main_body":              (main_body,                  "spool_main_body.step",        None),
    "bearing_cap_bottom":     (bearing_cap_bottom_export,  "bearing_cap_bottom.step",     None),
    "bearing_cap_top":        (bearing_cap_top_export,     "bearing_cap_top.step",        None),
    "axle":                   (axle,                       "axle.step",                   None),
    "housing":                (housing,                    "housing.step",                None),
    "ratchet_lever":          (ratchet_lever,              "ratchet_lever.step",          None),
    "brake_lever":            (brake_lever,                "brake_lever.step",            None),
    "ratchet_spring":         (ratchet_spring,             "ratchet_spring.step",         "dummy — purchased torsion spring"),
    "brake_spring":           (brake_spring,               "brake_spring.step",           "dummy — purchased torsion spring"),
    "brake_housing_pin":      (brake_housing_pin,          "brake_housing_pin.step",      "print separately — glue-install"),
    "brake_housing_rest_pin": (brake_housing_rest_pin,     "brake_housing_rest_pin.step", "print separately — glue-install"),
    "guide_axle":             (guide_axle_part,            "guide_axle.step",             "separate axle — press-fits into pancake-side guide-bearing boss"),
    "lever_guide_axle":       (lever_guide_axle_part,      "lever_guide_axle.step",       "separate axle — press-fits into lever-side guide-bearing boss"),
}


def _export(name):
    obj, path, note = PARTS[name]
    cq.exporters.export(obj, path)
    suffix = f"  ({note})" if note else ""
    print(f"Wrote {path}{suffix}")


def _export_assembly():
    assembly = (
        cq.Assembly(name="retractable_cable_spool")
        .add(main_body,     name="main_body")
        .add(bearing_cap_bottom, name="bearing_cap_bottom", loc=cq.Location((0, 0, 0)))
        .add(bearing_cap_top,    name="bearing_cap_top",    loc=cq.Location((0, 0, 0)))
        .add(bearing_bottom,     name="bearing_bottom",     loc=cq.Location((0, 0, 0)))
        .add(bearing_top,        name="bearing_top",        loc=cq.Location((0, 0, 0)))
        .add(bearing_guide,      name="bearing_guide",      loc=cq.Location((0, 0, 0)))
        .add(bearing_lever_guide, name="bearing_lever_guide", loc=cq.Location((0, 0, 0)))
        .add(guide_axle_part,    name="guide_axle",         loc=cq.Location((0, 0, 0)))
        .add(lever_guide_axle_part, name="lever_guide_axle", loc=cq.Location((0, 0, 0)))
        .add(axle,          name="axle")
        .add(housing, name="housing")
        .add(ratchet_lever, name="ratchet_lever")
        .add(brake_lever,   name="brake_lever")
        .add(ratchet_spring, name="ratchet_spring")
        .add(brake_spring,   name="brake_spring")
        .add(brake_housing_pin, name="brake_housing_pin")
        .add(brake_housing_rest_pin, name="brake_housing_rest_pin")
    )
    assembly.save("assembly.step")
    print("Wrote assembly.step")


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
