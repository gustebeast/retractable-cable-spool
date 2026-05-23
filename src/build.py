"""Retractable Cable Spool - main build script.

Run from the repo root:
  py -3.12 -m src.build              # build all parts + assembly
  py -3.12 -m src.build --part NAME  # build only one part (faster iteration)
  py -3.12 -m src.build --list       # list available part names

Produces all printed-part STEPs and an assembled assembly.step in cwd.

Build order:
  spool             → main_body (initial)
  caps              → bearing_cap_top + cap-key grooves
  axle              → axle
  housing           → housing
  housing_pins      → brake_housing_pin, brake_housing_rest_pin
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
from . import levers as _lev_mod
from . import mount_bracket as _bracket_mod
from . import cable_rim as _cable_rim_mod
from . import cable_retainer as _cable_retainer_mod
from . import viz as _viz_mod

# Compose the final main_body by applying the cap-key grooves and the
# lever-dependent cuts (ratchet teeth, drum cable hole) explicitly.
main_body = caps.apply_to_main_body(spool.main_body)
main_body = _lev_mod.apply_to_main_body(main_body)
# (Helical V-groove cut removed — pancake rewrite has no drum/helix.)

# Re-bind module-level part variables for export.
bearing_cap_top        = caps.bearing_cap_top
cable_top_rim          = _cable_rim_mod.cable_top_rim
cable_retainer         = _cable_retainer_mod.cable_retainer
axle                   = _axle_mod.axle
housing                = _housing_mod.housing
brake_housing_pin      = _housing_pins_mod.brake_housing_pin
brake_housing_rest_pin = _housing_pins_mod.brake_housing_rest_pin
mount_bracket            = _bracket_mod.mount_bracket
ratchet_lever          = _lev_mod.ratchet_lever
brake_lever            = _lev_mod.brake_lever
ratchet_spring         = _viz_mod.ratchet_spring
brake_spring           = _viz_mod.brake_spring
brake_pad_rubber       = _viz_mod.brake_pad_rubber
bearing_top            = _viz_mod.bearing_top
bearing_bottom         = _viz_mod.bearing_bottom

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

# Bearing cap is built in its assembled position (top cap seat,
# z=PANCAKE_CAP_SEAT_Z0..SPOOL_H). Exported as-is.
bearing_cap_top_export = bearing_cap_top

# Map of part name → (workplane, output filename, optional note).
PARTS = {
    "main_body":              (main_body,                  "spool_main_body.step",        None),
    "bearing_cap_top":        (bearing_cap_top_export,     "bearing_cap_top.step",        None),
    "cable_top_rim":          (cable_top_rim,              "cable_top_rim.step",          None),
    "cable_retainer":         (cable_retainer,             "cable_retainer.step",         None),
    "axle":                   (axle,                       "axle.step",                   None),
    "housing":                (housing,                    "housing.step",                None),
    "ratchet_lever":          (ratchet_lever,              "ratchet_lever.step",          None),
    "brake_lever":            (brake_lever,                "brake_lever.step",            None),
    # Springs and the rubber pad are purchased/applied parts — they're
    # included in assembly.step for visualization only, no need to export
    # them as individual STEP files for printing.
    "brake_housing_pin":      (brake_housing_pin,          "brake_housing_pin.step",      "print separately — glue-install"),
    "brake_housing_rest_pin": (brake_housing_rest_pin,     "brake_housing_rest_pin.step", "print separately — glue-install"),
    "mount_bracket":             (mount_bracket,            "mount_bracket.step",             "L-shaped wood-screw mount; housing M2-clamps to it"),
}


def _export(name):
    obj, path, note = PARTS[name]
    cq.exporters.export(obj, path)
    suffix = f"  ({note})" if note else ""
    print(f"Wrote {path}{suffix}")


# Visualization aid — selects which lever pose to render in assembly.step:
#   "rest"        : both levers at their printed/rest pose (handles horizontal,
#                   pawl seated in valley, brake pad clear of track).
#   "match"       : both levers at θ_pull = THETA_MATCH_DEG (pawl just clears
#                   the tooth tip). With A1 relaxed, this is informative for
#                   the RATCHET but the brake is NOT at first contact here.
#   "transitions" : each lever at ITS OWN transition angle — ratchet at
#                   THETA_MATCH_DEG (just leaving contact), brake at
#                   THETA_BRAKE_CONTACT_DEG (just making contact). Shows the
#                   two transition moments side-by-side even though they
#                   no longer happen at the same θ_pull.
#   "engaged"     : θ_pull = RATCHET_OUTER_TRAVEL_DEG (full pull) — pawl
#                   fully clear of teeth, rubber compressed into the track.
LEVERS_POSE = "rest"


def _ratchet_theta_pull_deg():
    """θ_pull (degrees) to apply to the ratchet lever in the current pose."""
    if LEVERS_POSE == "rest":
        return 0.0
    if LEVERS_POSE in ("match", "transitions"):
        from .levers import THETA_MATCH_DEG
        return THETA_MATCH_DEG
    if LEVERS_POSE == "engaged":
        from .levers import RATCHET_OUTER_TRAVEL_DEG
        return RATCHET_OUTER_TRAVEL_DEG
    raise ValueError(f"Unknown LEVERS_POSE: {LEVERS_POSE!r}")


def _brake_theta_pull_deg():
    """θ_pull (degrees) to apply to the brake lever (and its pad rubber)
    in the current pose. With A1 relaxed, the brake's transition angle
    differs from the ratchet's, so the 'transitions' pose returns the
    brake-specific contact angle here."""
    if LEVERS_POSE == "rest":
        return 0.0
    if LEVERS_POSE == "match":
        from .levers import THETA_MATCH_DEG
        return THETA_MATCH_DEG
    if LEVERS_POSE == "transitions":
        from .levers import THETA_BRAKE_CONTACT_DEG
        return THETA_BRAKE_CONTACT_DEG
    if LEVERS_POSE == "engaged":
        from .levers import RATCHET_OUTER_TRAVEL_DEG
        return RATCHET_OUTER_TRAVEL_DEG
    raise ValueError(f"Unknown LEVERS_POSE: {LEVERS_POSE!r}")


def _ratchet_lever_for_assembly():
    theta = _ratchet_theta_pull_deg()
    if theta == 0.0:
        return ratchet_lever
    from .housing import RATCHET_PIVOT_X, LEVER_PIVOT_Z
    # Assembly rotation is -θ_pull around +Y at the ratchet pivot.
    return (
        ratchet_lever
        .translate((-RATCHET_PIVOT_X, 0, -LEVER_PIVOT_Z))
        .rotate((0, 0, 0), (0, 1, 0), -theta)
        .translate((RATCHET_PIVOT_X, 0, LEVER_PIVOT_Z))
    )


def _brake_lever_for_assembly():
    if _brake_theta_pull_deg() == 0.0:
        return brake_lever
    return _brake_pulled_rotation(brake_lever)


def _brake_pad_rubber_for_assembly():
    if _brake_theta_pull_deg() == 0.0:
        return brake_pad_rubber
    # The rubber pad is a separate viz-only solid; it ships with the same
    # rest-pose pre-rotation as the printed pad (see _brake_pad_rubber in
    # viz.py), so it follows the printed lever into the pulled pose under
    # the same transform.
    return _brake_pulled_rotation(brake_pad_rubber)


def _brake_pulled_rotation(part):
    """Rotate `part` from the brake's REST pose to its current θ_pull.

    Both the printed brake lever and the rubber pad are built at the
    parallel design pose and then pre-rotated +PAD_PARALLEL_THETA_PULL_DEG
    to reach the REST (print) pose (see _brake_pad_contact in levers.py
    and _brake_pad_rubber in viz.py). Applying -θ_pull here drives the
    part from rest toward (and past) the parallel pose."""
    theta = _brake_theta_pull_deg()
    from .housing import BRAKE_PIVOT_X, LEVER_PIVOT_Z
    return (
        part
        .translate((-BRAKE_PIVOT_X, 0, -LEVER_PIVOT_Z))
        .rotate((0, 0, 0), (0, 1, 0), -theta)
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
        .add(bearing_cap_top,    name="bearing_cap_top",    loc=cq.Location((0, 0, 0)))
        # Cable top rim placed 6 mm above the TOP of the bottom rim
        # (bottom rim top = RIM_H = 14, so its bottom face sits at z=20).
        # Translation baked into the geometry (not via the assembly loc) so
        # the STEP export unambiguously shows it at height. Slides freely on
        # the hub for now — no height stop.
        .add(cable_top_rim.translate((0, 0, spool.RIM_H + 6)),
             name="cable_top_rim", loc=cq.Location((0, 0, 0)))
        # Fixed (housing-attached) cable-retention cage — already modelled at
        # its working position (cable-channel Z-band), floating for now.
        .add(cable_retainer, name="cable_retainer", loc=cq.Location((0, 0, 0)))
        .add(bearing_bottom,     name="bearing_bottom",     loc=cq.Location((0, 0, 0)))
        .add(bearing_top,        name="bearing_top",        loc=cq.Location((0, 0, 0)))
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
          + (f"  [levers {LEVERS_POSE.upper()}]" if LEVERS_POSE != "rest" else ""),
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
