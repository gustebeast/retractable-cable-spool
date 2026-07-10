"""Retractable Cable Spool - main build script.

Run from the repo root:
  py -3.12 -m src.build              # build all parts + assembly
  py -3.12 -m src.build --part NAME  # build only one part (faster iteration)
  py -3.12 -m src.build --list       # list available part names

Produces all printed-part STEPs and an assembled assembly.step in cwd.

Build order:
  spool             → main_body (initial)
  caps              → bearing_cap_top + cap-key grooves
  axle              → axle_top, axle_bottom
  housing           → housing (L-bracket; carries both lever pivots)
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

# Shared helpers from the Archive/3D cadkit/ folder: color() (cq.Color from hex
# strings, 0..255 tuples, and SVG/X11 names), export_step() (names each STEP
# product after its file), and show() (opens/refreshes the FreeCAD viewer hub).
# See cadkit/README.md.
from cadkit.cq_colors import color
from cadkit.step_export import export_step
from cadkit.freecad import show

from .helpers import heal, cyl
from .dimensions import SPOOL_H


# ── Dark-mode-friendly assembly palette ────────────────────────────────────
# Picked to sit on a dark FreeCAD background (~#1e1e1e) without disappearing
# and to keep functionally-related parts colour-coordinated:
#   • spool body family (the rotating drum that wraps the cable) — warm tans
#   • housing family (stationary L-bracket) — slate blues
#   • brake parts (lever + its pin chunk) — muted teal-green
#   • ratchet parts (lever + its pin chunk) — warm coral
#   • bearings — silvery
#   • axle halves — bronze
#   • viz-only (springs, rubber pad) — muted utility colours
COLOR = {
    "main_body":         "#C4A56B",  # warm tan — spool body
    "cable_top_rim":     "#D6BC8C",  # lighter tan — top of spool
    "cable_retainer":    "#C7A55C",  # gold — retention cage
    "cable_stop":        "#C7A55C",  # gold — cable C-clamp stop

    "housing":           "#6B8AAB",  # slate blue — main structural
    "mount_bracket":     "#4F6478",  # deep slate — wood-mount L

    "axle_top":          "#B97D43",  # bronze — pancake half
    "axle_bottom":       "#B97D43",  # bronze — lever half (matches)
    "bearing_cap_top":   "#8FA8C0",  # lighter slate — removable cap

    "bearing_bottom":    "#B4BAC1",  # silver — 608
    "bearing_top":       "#B4BAC1",  # silver — 608

    "ratchet_lever":     "#DC7A6E",
    "brake_pin_chunk":   "#6DAB94",  # teal-green — brake group
    "brake_lever":       "#6DAB94",

    "guide_wheel":       "#E1C75D",  # yellow accent — small/visible

    "ratchet_spring":    "#95A4B4",  # steel — wire viz
    "brake_spring":      "#95A4B4",
    "brake_pad_rubber":  "#3A3A3A",  # charcoal — rubber

    "build_counter":     "#F0A878",  # salmon — accent
}

from . import spool
from . import caps
from . import axle as _axle_mod
from . import housing as _housing_mod
from . import levers as _lev_mod
from . import mount_bracket as _bracket_mod
from . import cable_rim as _cable_rim_mod
from . import cable_retainer as _cable_retainer_mod
from . import cable_stop as _cable_stop_mod
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
cable_stop             = _cable_stop_mod.cable_stop
axle_top               = _axle_mod.axle_top
axle_bottom            = _axle_mod.axle_bottom
housing                = _housing_mod.housing
brake_pin_chunk        = _housing_mod.brake_pin_chunk
guide_wheel            = _housing_mod.guide_wheel
guide_wheel_plus       = _housing_mod.guide_wheel_plus
mount_bracket            = _bracket_mod.mount_bracket
ratchet_lever          = _lev_mod.ratchet_lever
brake_lever            = _lev_mod.brake_lever
ratchet_spring         = _viz_mod.ratchet_spring
brake_spring           = _viz_mod.brake_spring
brake_pad_rubber       = _viz_mod.brake_pad_rubber
bearing_top            = _viz_mod.bearing_top
bearing_bottom         = _viz_mod.bearing_bottom

# Drill the M2 guide-wheel-axle clearance hole through the cable retainer's
# tenon — on BOTH the -X and +X sides (one hole per guide wheel). The axle
# screws pass through the housing along the line the retainer's tenon TIP now
# reaches (RETAIN_TENON_DEPTH lands the tip on the screw axis). The screw sits
# HALF in the tenon (this groove) and half in the housing, pinning the joint —
# so the bore is the SNUG axle-thread Ø (GUIDE_AXLE_SHAFT_D, the same the screw
# self-taps in the housing), NOT a loose clearance Ø. The cut is a full
# cylinder at the screw axis, but since the tenon tip ends at that axis only
# its inner half is removed, leaving the half-round pin groove. Done here (not
# in cable_retainer.py) to avoid a housing → retainer circular import.
def _guide_axle_clr_hole(x_center, z_head):
    """Snug Ø GUIDE_AXLE_SHAFT_D axle bore starting at z_head, at the given X.
    Each side's z_head matches the housing's own axle bore: the -X head seats
    on the back-leg floor (GUIDE_AXLE_HEAD_Z), the +X head at the bottom of
    its Ø4.1 access bore (GUIDE_AXLE_HEAD_Z_PLUS)."""
    return (
        cyl(_housing_mod.GUIDE_AXLE_SHAFT_D, _housing_mod.GUIDE_AXLE_HOLE_LEN,
            z=z_head)
        .translate((x_center, 0, 0))
    )
cable_retainer = (cable_retainer
                  .cut(_guide_axle_clr_hole(_housing_mod.GUIDE_WHEEL_CX,
                                             _housing_mod.GUIDE_AXLE_HEAD_Z))
                  .cut(_guide_axle_clr_hole(-_housing_mod.GUIDE_WHEEL_CX,
                                             _housing_mod.GUIDE_AXLE_HEAD_Z_PLUS)))

# ────────────────────────────────────────────────────────────────────────────
# Export — individual parts for printing, + a combined assembly STEP for
# dimensional verification (bearings & spring not modelled, since they're
# purchased parts).
# ────────────────────────────────────────────────────────────────────────────


main_body = heal(main_body)
housing = heal(housing)
brake_pin_chunk = heal(brake_pin_chunk)
cable_stop = heal(cable_stop)
guide_wheel = heal(guide_wheel)
guide_wheel_plus = heal(guide_wheel_plus)
ratchet_lever = heal(ratchet_lever)
brake_lever   = heal(brake_lever)
# Springs accumulate near-tangent boolean fuses between rings/legs/spheres
# — heal them too so strict STEP importers accept the resulting compound
# without face-tolerance complaints.
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
    "cable_stop":             (cable_stop,                 "cable_stop.step",             "C-clamp cable stop -- pinches onto the cable at the wood plate so it can't pull back through (M2 + heat-set insert; Ø5 bore / Ø15 body at full clamp)"),
    "axle_top":               (axle_top,                   "axle_top.step",               "pancake-side half — mortise"),
    "axle_bottom":            (axle_bottom,                "axle_bottom.step",            "lever-side half — tenon"),
    "housing":                (housing,                    "housing.step",                None),
    "brake_pin_chunk":        (brake_pin_chunk,            "brake_pin_chunk.step",        "glue-on -Y corner carrying the brake pivot + stop pins"),
    # Single STEP for the guide wheel — print TWO of these. guide_wheel_plus
    # is the +X mirror used in the assembly view only (an XZ flip of the same
    # part), so it doesn't need its own STEP file.
    "guide_wheel":            (guide_wheel,                "guide_wheel.step",            "Ø14 Z-axle wheel — print TWO; one bears on the brake rim opposite the brake lever, the other rides in the +X (lever-side) guide pocket"),
    "ratchet_lever":          (ratchet_lever,              "ratchet_lever.step",          None),
    "brake_lever":            (brake_lever,                "brake_lever.step",            None),
    # Springs and the rubber pad are purchased/applied parts — they're
    # included in assembly.step for visualization only, no need to export
    # them as individual STEP files for printing.
    "mount_bracket":             (mount_bracket,            "mount_bracket.step",             "L-shaped wood-screw mount; housing M2-clamps to it"),
}


def _export(name):
    obj, path, note = PARTS[name]
    export_step(obj, path)
    suffix = f"  ({note})" if note else ""
    print(f"Wrote {path}{suffix}")


# Visualization aid — selects which lever pose to render in assembly.step:
#   "rest"        : both levers at their printed/rest pose (ratchet pawl
#                   seated in the teeth = engaged; brake pad lifted off
#                   the band).
#   "engaged"     : both levers fully pulled (handles toward +X) — ratchet
#                   pawl swung clear of the teeth (disengaged), brake pad
#                   pressed onto the band (engaged). θ_pull = full travel.
#   "brake_midway": brake at the midpoint between first-contact and full
#                   engagement (= BRAKE_PAD_MOUNT_TILT_DEG of pull), so
#                   the pad's mount face sweeps to vertical → parallel to
#                   the band. For visual validation of the tilt geometry.
#                   Ratchet stays at rest.
LEVERS_POSE = "rest"


def _ratchet_theta_pull_deg():
    if LEVERS_POSE in ("rest", "brake_midway"):
        return 0.0
    if LEVERS_POSE == "engaged":
        from .housing import RATCHET_OUTER_TRAVEL_DEG
        return RATCHET_OUTER_TRAVEL_DEG
    raise ValueError(f"Unknown LEVERS_POSE: {LEVERS_POSE!r}")


def _brake_theta_pull_deg():
    if LEVERS_POSE == "rest":
        return 0.0
    if LEVERS_POSE == "engaged":
        from .housing import BRAKE_INNER_TRAVEL_DEG
        return BRAKE_INNER_TRAVEL_DEG
    if LEVERS_POSE == "brake_midway":
        from .levers import BRAKE_PAD_MOUNT_TILT_DEG
        return BRAKE_PAD_MOUNT_TILT_DEG
    raise ValueError(f"Unknown LEVERS_POSE: {LEVERS_POSE!r}")


def _pulled(part, pivot_x, pivot_z, theta):
    """Assembly rotation: -θ_pull about +Y at the lever's pivot."""
    if theta == 0.0:
        return part
    return (part
            .translate((-pivot_x, 0, -pivot_z))
            .rotate((0, 0, 0), (0, 1, 0), -theta)
            .translate((pivot_x, 0, pivot_z)))


def _ratchet_lever_for_assembly():
    from .housing import RATCHET_PIVOT_X, RATCHET_PIVOT_Z
    return _pulled(ratchet_lever, RATCHET_PIVOT_X, RATCHET_PIVOT_Z,
                   _ratchet_theta_pull_deg())


def _brake_lever_for_assembly():
    from .housing import BRAKE_PIVOT_X, BRAKE_PIVOT_Z
    return _pulled(brake_lever, BRAKE_PIVOT_X, BRAKE_PIVOT_Z,
                   _brake_theta_pull_deg())


def _brake_pad_rubber_for_assembly():
    from .housing import BRAKE_PIVOT_X, BRAKE_PIVOT_Z
    return _pulled(brake_pad_rubber, BRAKE_PIVOT_X, BRAKE_PIVOT_Z,
                   _brake_theta_pull_deg())


# ── Build counter — a 3D number floating well above the assembly, bumped
# on every full build. Lets you see at a glance in the viewer that a fresh
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
        .add(main_body,     name="main_body",     color=color(COLOR["main_body"]))
        .add(bearing_cap_top,    name="bearing_cap_top",    color=color(COLOR["bearing_cap_top"]),
             loc=cq.Location((0, 0, 0)))
        # Cable top rim placed CABLE_RIM_AIR_GAP above the TOP of the bottom rim
        # (= spool.CABLE_TOP_RIM_BASE_Z). This is its max realistic spacing for a
        # thick cable; the spring-strip screw hole tracks the lid's top off this.
        # Translation baked into the geometry (not via the assembly loc) so the
        # STEP export unambiguously shows it at height. Slides freely on the hub.
        .add(cable_top_rim.translate((0, 0, spool.CABLE_TOP_RIM_BASE_Z)),
             name="cable_top_rim", color=color(COLOR["cable_top_rim"]),
             loc=cq.Location((0, 0, 0)))
        # Fixed (housing-attached) cable-retention cage — already modelled at
        # its working position (cable-channel Z-band), floating for now.
        .add(cable_retainer, name="cable_retainer", color=color(COLOR["cable_retainer"]),
             loc=cq.Location((0, 0, 0)))
        # Cable stop C-clamp — a loose accessory (clamps onto the cable at
        # the wood plate), shown floating 100 mm below the assembly.
        .add(cable_stop, name="cable_stop", color=color(COLOR["cable_stop"]),
             loc=cq.Location((0, 0, -100)))
        .add(bearing_bottom,     name="bearing_bottom",     color=color(COLOR["bearing_bottom"]),
             loc=cq.Location((0, 0, 0)))
        .add(bearing_top,        name="bearing_top",        color=color(COLOR["bearing_top"]),
             loc=cq.Location((0, 0, 0)))
        .add(mount_bracket,          name="mount_bracket",          color=color(COLOR["mount_bracket"]),
             loc=cq.Location((0, 0, 0)))
        .add(axle_top,      name="axle_top",      color=color(COLOR["axle_top"]))
        .add(axle_bottom,   name="axle_bottom",   color=color(COLOR["axle_bottom"]))
        .add(housing, name="housing", color=color(COLOR["housing"]))
        .add(brake_pin_chunk, name="brake_pin_chunk", color=color(COLOR["brake_pin_chunk"]))
        .add(guide_wheel, name="guide_wheel", color=color(COLOR["guide_wheel"]))
        .add(guide_wheel_plus, name="guide_wheel_plus",
             color=color(COLOR["guide_wheel"]))
        .add(_ratchet_lever_for_assembly(), name="ratchet_lever",
             color=color(COLOR["ratchet_lever"]))
        .add(_brake_lever_for_assembly(),   name="brake_lever",
             color=color(COLOR["brake_lever"]))
        # Lever pivot spacers: glued to each lever's outer Y face at the M2
        # pivot screw axis. Placed at the housing's pivot X/Z, offset outward
        # in Y by HOUSING_W/2 + lever thickness so the spacer's inner face
        # sits flush on the lever's outer face. Rotated to point along Y.
        .add(ratchet_spring, name="ratchet_spring", color=color(COLOR["ratchet_spring"]))
        .add(brake_spring,   name="brake_spring",   color=color(COLOR["brake_spring"]))
        .add(_brake_pad_rubber_for_assembly(), name="brake_pad_rubber",
             color=color(COLOR["brake_pad_rubber"]))
    )
    counter = _build_counter_model(build_n)
    if counter is not None:
        assembly.add(counter, name="build_counter", color=color(COLOR["build_counter"]))
    assembly.save("assembly.step")
    print(f"Wrote assembly.step  [build #{build_n}]"
          + (f"  [levers {LEVERS_POSE.upper()}]" if LEVERS_POSE != "rest" else ""),
          flush=True)
    show("assembly.step")


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
