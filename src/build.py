"""Build — THE retractable cable spool (v3).

Run:  py -3.12 -m src.build   (from the project root)

Parts so far: frame_top (plus spider with the 608 pocketed into its own
thickness), the two-half rotating axle hanging on its top lip, and the
608 bearing + unmodified spring as assembly-only dummies. The spring-body
frame mount, drum, and the axial cable chamber follow.
"""

import pathlib

import cadquery as cq

from cadkit.step_export import export_step
from cadkit.cq_colors import color
from cadkit.freecad import show

from .frame import frame_top, frame_bottom
from .mount import mount_ring
from .axle import axle_top, axle_separator
from .lid import lid
from .levers import (
    ratchet_lever, brake_lever, brake_pad_tpu, lever_pin,
    lever_pin_in_place, brake_pad_tall_demo, BRAKE_CONTACT_DEG,
    RATCHET_OPEN_DEG, KIN,
)
from .params import RATCHET_PIVOT_Z, BRAKE_PIVOT_Z, LEVER_PIVOT_X
from .dummies import bearing_608, bearing_608_bottom, spring

# ── Viewer pose (revertable) — both levers at REST (canonical pose). Set
# BRAKE_POSE_DEG = BRAKE_CONTACT_DEG to preview the pad flush on the band;
# RATCHET_POSE_DEG likewise for a pulled pawl. BRAKE_PAD_DEMO_H forces an
# oversized viewer-only pad (None = the real one).
BRAKE_PAD_DEMO_H = None
BRAKE_POSE_DEG = 0.0
RATCHET_POSE_DEG = RATCHET_OPEN_DEG   # REVERT TO 0.0 — user asked to see
                                      # the ratchet FULLY OPEN, parked on
                                      # the new over-pull stop shelf


def _brake_pose(part):
    if not BRAKE_POSE_DEG:
        return part
    return part.rotate((LEVER_PIVOT_X, 0, BRAKE_PIVOT_Z),
                       (LEVER_PIVOT_X, 1, BRAKE_PIVOT_Z), -BRAKE_POSE_DEG)


def _ratchet_pose(part):
    if not RATCHET_POSE_DEG:
        return part
    return part.rotate((LEVER_PIVOT_X, 0, RATCHET_PIVOT_Z),
                       (LEVER_PIVOT_X, 1, RATCHET_PIVOT_Z), -RATCHET_POSE_DEG)

OUT = pathlib.Path(__file__).resolve().parent
VIEWER = OUT.parent / "assembly.step"

COLOR = {
    "frame_top":   "#4F6D8F",  # slate — top plus (v2's frame_top colour)
    "frame_bottom": "#6B8AAB", # lighter slate — beams + fused wall band +
                               # spoked floor + lever forks (one print)
    "mount_ring": "#7D8F69",   # sage — desk-mount twist-lock ring
    "axle_top":    "#A6786B",  # clay — axle top half (lip + tenon)
    "axle_separator": "#B0654B",  # rust — bottom axle + separator disk, one
                                  # printed part (v2's separator colour)
    "lid":         "#7FA37F",  # green — spool-chamber ceiling (v2 ceiling colour)
    "ratchet_lever": "#C9825F",  # copper — ratchet lever (+y side)
    "brake_lever":   "#B45A5A",  # brick — brake lever (−y side)
    "tpu":           "#141414",  # black — 95A TPU (pad + torsion pins;
                                 # colour reserved for TPU)
    "bearing_608": "#9FA8B2",  # steel — dummy 608 (assembly-only)
    "spring":      "#6E7B8B",  # gunmetal — dummy spring, flanges on (assembly-only)
    "build_num":   "#F0A878",  # salmon — floating build number
}

# One STEP per PRINTED part (dummies are assembly-only, cadkit rule).
PARTS = [
    ("frame_top",   frame_top,   "frame_top.step"),
    ("frame_bottom", frame_bottom, "frame_bottom.step"),
    ("mount_ring",  mount_ring,  "mount_ring.step"),
    ("axle_top",    axle_top,    "axle_top.step"),
    ("axle_separator", axle_separator, "axle_separator.step"),
    ("lid",         lid,         "lid.step"),
    ("ratchet_lever", ratchet_lever, "ratchet_lever.step"),
    ("brake_lever",   brake_lever,   "brake_lever.step"),
    ("lever_pin_tpu", lever_pin,     "lever_pin_tpu.step"),
    ("brake_pad_tpu", brake_pad_tpu, "brake_pad_tpu.step"),
]


def _bump_build_counter():
    f = OUT / "build_n.txt"
    try:
        n = int(f.read_text().strip()) + 1
    except (OSError, ValueError):
        n = 1
    try:
        f.write_text(str(n))
    except OSError:
        pass
    return n


def _build_number_model(n):
    """Floating build number above the assembly (all-time running count —
    the counter carried over from v2, which carried it from v1)."""
    try:
        return (cq.Workplane("XZ").text(str(n), 12, 2)
                .translate((0.0, 0.0, 40.0)))
    except Exception:                                       # noqa: BLE001
        return None


def main():
    for _name, part, fname in PARTS:
        export_step(part, str(OUT / fname))

    build_n = _bump_build_counter()

    asm = (cq.Assembly(name="retractable_cable_spool")
           .add(frame_top,   name="frame_top",   color=color(COLOR["frame_top"]))
           .add(frame_bottom, name="frame_bottom",
                color=color(COLOR["frame_bottom"])))
    # the desk-mount twist-lock ring, SEATED flush in the arms' arc channels
    asm.add(mount_ring, name="mount_ring", color=color(COLOR["mount_ring"]))
    asm = (asm
           .add(axle_top,    name="axle_top",    color=color(COLOR["axle_top"]))
           .add(axle_separator, name="axle_separator", color=color(COLOR["axle_separator"]))
           # lid SEATED on the drum wall (bayonet rotated to its stops)
           .add(lid, name="lid", color=color(COLOR["lid"]))
           # levers at REST (pawl meshed; pad off the band) + the 95A TPU
           # parts: torsion-bar pivot pins (twist-extruded working pose)
           # and the brake pad
           .add(_ratchet_pose(ratchet_lever),
                name="ratchet_lever", color=color(COLOR["ratchet_lever"]))
           .add(_brake_pose(brake_lever),
                name="brake_lever",   color=color(COLOR["brake_lever"]))
           .add(lever_pin_in_place(RATCHET_PIVOT_Z, +1,
                                   pull_deg=RATCHET_POSE_DEG),
                name="ratchet_pin_tpu", color=color(COLOR["tpu"]))
           .add(lever_pin_in_place(BRAKE_PIVOT_Z, -1,
                                   pull_deg=BRAKE_POSE_DEG),
                name="brake_pin_tpu",   color=color(COLOR["tpu"]))
           .add(_brake_pose(brake_pad_tall_demo(BRAKE_PAD_DEMO_H)
                            if BRAKE_PAD_DEMO_H else brake_pad_tpu),
                name="brake_pad_tpu", color=color(COLOR["tpu"]))
           .add(bearing_608, name="bearing_608", color=color(COLOR["bearing_608"]))
           .add(bearing_608_bottom, name="bearing_608_bottom",
                color=color(COLOR["bearing_608"]))
           .add(spring,      name="spring",      color=color(COLOR["spring"])))

    num = _build_number_model(build_n)
    if num is not None:
        asm.add(num, name="build_number", color=color(COLOR["build_num"]))

    asm.save(str(VIEWER), mode="default")
    print(f"Wrote {VIEWER.name}  [build #{build_n}]", flush=True)
    show(str(VIEWER))


main()
