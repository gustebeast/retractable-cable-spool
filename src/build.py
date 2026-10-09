"""Build — THE retractable cable spool (v3).

Run:  py -3.12 -m src.build   (from the project root)

Parts so far: frame_top (plus spider with the 608 pocketed into its own
thickness), the two-half rotating axle hanging on its top lip, and the
608 bearing + unmodified spring as assembly-only dummies. The spring-body
frame mount, drum, and the axial cable chamber follow.
"""

import pathlib

import cadquery as cq

from cadkit.step_export import export_print
from cadkit.orientation import require_declared, unconfirmed
from cadkit.cq_colors import color
from cadkit.web import show

from .frame import (frame_top, frame_bottom, horn_cap, lock_pin_tpu,
                    lock_pins_in_place)
from .mount import mount_ring, MOUNT_MATE_DEG
from .axle import axle_top, axle_separator, bearing_nub
from .lid import lid
from .test.pin_gauge import GAUGE_PRINTS, GAUGE_ASSEMBLED
from .levers import (
    ratchet_lever, lever_pin, ratchet_clip_tpu,
    lever_pin_in_place, return_clip_in_place,
    RATCHET_OPEN_DEG, KIN,
)
from .params import (RATCHET_PIVOT_Z, LEVER_PIVOT_X, CLIP_SIDE,
                     COUCH_TOP_PAD, PRINT_ORIENTATION)
from .dummies import bearing_608, bearing_608_bottom, spring

# ── Viewer pose (revertable) — the lever at REST (canonical pose). Set
# RATCHET_POSE_DEG to preview a pulled pawl.
RATCHET_POSE_DEG = 0.0
# UNINSTALL pose (viewer-only, revertable): twist the whole hanging
# assembly about z relative to the FIXED mount (and the couch beams) by
# this many degrees — CW viewed +z is the uninstall direction; at
# MOUNT_MATE_DEG the arc tenons have just cleared their channels and the
# spool can drop away. 0.0 = seated (canonical). Set to MOUNT_MATE_DEG to
# see where the lever/exit face hits the couch's front beam.
UNINSTALL_TWIST_DEG = 0.0


def _twist(part):
    """The uninstall twist — everything that hangs off the mount."""
    if not UNINSTALL_TWIST_DEG:
        return part
    return part.rotate((0, 0, 0), (0, 0, 1), -UNINSTALL_TWIST_DEG)


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
    "mount_ring": "#7D8F69",   # sage — desk-mount twist-lock ring (deep
                               # screw pads since #947 — the old separate
                               # stack spacer is fused in)
    "horn_cap":   "#C4A35A",   # ochre — cable-horn tunnel cap (closes
                               # the bore over the laid-in cable)
    "axle_top":    "#A6786B",  # clay — axle top half (lip + tenon)
    "axle_separator": "#B0654B",  # rust — bottom axle + separator disk, one
                                  # printed part (v2's separator colour)
    "lid":         "#7FA37F",  # green — spool-chamber ceiling (v2 ceiling colour)
    "ratchet_lever": "#C9825F",  # copper — ratchet lever (+y side)
    "tpu":           "#141414",  # black — 95A TPU (pad + lock pins;
                                 # colour reserved for TPU)
    "lever_pin":   "#8C7B6B",  # taupe — printed PETG pivot pins (#988;
                               # they were black TPU torsion axles)
    "bearing_608": "#9FA8B2",  # steel — dummy 608 (assembly-only)
    "spring":      "#6E7B8B",  # gunmetal — dummy spring, flanges on (assembly-only)
    "build_num":   "#F0A878",  # salmon — floating build number
    "coupon":      "#7FA37A",  # sage — test coupons (off to the side)
}

# ── PRINT POSE for the per-part STEP exports (user #935, cadkit's
# print_pose since #937): each file lands in Bambu already
# print-oriented — rotated onto its declared bed face, dropped to z=0,
# centred on xy. EXPORT-ONLY: the assembly / viewer keeps every part
# as-modeled.
#
# The bed faces themselves live in params.PRINT_ORIENTATION, because the
# JOINT SPECS derive from the same declarations (cadkit #1070) — a part's
# build direction is one fact and it is stated once. `export_print`
# refuses a part whose orientation is not a PrintOrientation, so there is
# no longer a code path where a missing bed face looks like a deliberate
# "models as printed".

# One STEP per PRINTED part (dummies are assembly-only, cadkit rule).
PARTS = [
    ("frame_top",   frame_top,   "frame_top.step"),
    ("frame_bottom", frame_bottom, "frame_bottom.step"),
    ("mount_ring",  mount_ring,  "mount_ring.step"),
    ("horn_cap",    horn_cap,    "horn_cap.step"),
    ("axle_top",    axle_top,    "axle_top.step"),
    ("axle_separator", axle_separator, "axle_separator.step"),
    ("bearing_nub",  bearing_nub,  "bearing_nub.step"),
    ("lid",         lid,         "lid.step"),
    ("ratchet_lever", ratchet_lever, "ratchet_lever.step"),
    ("lever_pin",     lever_pin,     "lever_pin.step"),
    ("ratchet_clip_tpu", ratchet_clip_tpu, "ratchet_clip_tpu.step"),
    ("lock_pin_tpu",  lock_pin_tpu,  "lock_pin_tpu.step"),
]


# Test coupons: exported to the project root and shown in the assembly
# off to the side (+y), so they rebuild with every run.
COUPONS = GAUGE_PRINTS        # (fname, solid, PrintOrientation) — the pin /
                              # lever / frame gauge: does the TPU clear?
COUPON_SHIFT = (0.0, 140.0, 0.0)


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


# EVERY printed part declares its bed face — no silent default. An
# omission and a deliberate "models as printed" are different claims, and
# PRINT_ROT.get() could not tell them apart (#1070). This also catches a
# RENAMED part, whose old entry would otherwise sit there matching nobody.
require_declared({n for n, _p, _f in PARTS}, PRINT_ORIENTATION)


def main():
    for _name, part, fname in PARTS:
        export_print(part, PRINT_ORIENTATION[_name], str(OUT / fname))
    # TEST COUPONS — printed in the real part's orientation, exported
    # into src/test/ beside their modules (user #1033; cadkit's own rule
    # says the project root — the test_ prefix still marks them)
    (OUT / "test").mkdir(exist_ok=True)
    for fname, part, orient in COUPONS:
        export_print(part, orient, str(OUT / "test" / fname))

    build_n = _bump_build_counter()

    # 🚩 an UNCONFIRMED bed face is a guess nobody has checked. Say so on
    # every build: the bed face decides which faces are overhangs, which
    # slots bridge and which way every joint on the part prints, so work
    # done on top of a guess can be silently wrong (cadkit AGENTS.md).
    _flagged = unconfirmed(PRINT_ORIENTATION)
    if _flagged:
        print("UNCONFIRMED print orientation (confirm with the user BEFORE "
              "working on these parts): " + ", ".join(_flagged), flush=True)

    T = _twist                      # uninstall pose (viewer-only, see flag)
    asm = (cq.Assembly(name="retractable_cable_spool")
           .add(T(frame_top),   name="frame_top",   color=color(COLOR["frame_top"]))
           .add(T(frame_bottom), name="frame_bottom",
                color=color(COLOR["frame_bottom"])))
    # the desk-mount twist-lock ring, SEATED flush in the arms' arc channels
    # (couch version: the part is unchanged, it seats COUCH_TOP_PAD
    # higher on frame_top's thickened top face — assembly placement only)
    asm.add(mount_ring.translate((0.0, 0.0, COUCH_TOP_PAD)),
            name="mount_ring", color=color(COLOR["mount_ring"]))
    # the TPU frame-lock pin, fully inserted through the lock stack
    # the TPU frame-lock pins, fully inserted — one per lock site (#983)
    for k, pin in enumerate(lock_pins_in_place):
        asm.add(T(pin), name="lock_pin_tpu" + ("" if k == 0 else f"_{k + 1}"),
                color=color(COLOR["tpu"]))
    # horn cap SEATED on the trough half (its as-modeled place)
    asm.add(T(horn_cap), name="horn_cap", color=color(COLOR["horn_cap"]))
    asm = (asm
           .add(T(axle_top),    name="axle_top",    color=color(COLOR["axle_top"]))
           .add(T(axle_separator), name="axle_separator", color=color(COLOR["axle_separator"]))
           # lid SEATED on the drum wall (bayonet rotated to its stops)
           .add(T(lid), name="lid", color=color(COLOR["lid"]))
           # the lever at REST (pawl meshed), its pin and its TPU clip
           .add(T(_ratchet_pose(ratchet_lever)),
                name="ratchet_lever", color=color(COLOR["ratchet_lever"]))
           .add(T(lever_pin_in_place(RATCHET_PIVOT_Z, CLIP_SIDE,
                                     pull_deg=RATCHET_POSE_DEG)),
                name="ratchet_pin", color=color(COLOR["lever_pin"]))
           # the TPU return clips (#1004), at REST (leaf on its post)
           .add(T(return_clip_in_place(RATCHET_PIVOT_Z, CLIP_SIDE)),
                name="ratchet_return_clip", color=color(COLOR["tpu"]))
           .add(T(bearing_nub), name="bearing_nub",
                color=color(COLOR["axle_separator"]))
           .add(T(bearing_608), name="bearing_608", color=color(COLOR["bearing_608"]))
           .add(T(bearing_608_bottom), name="bearing_608_bottom",
                color=color(COLOR["bearing_608"]))
           .add(T(spring),      name="spring",      color=color(COLOR["spring"])))

    # the gauge, ASSEMBLED (the print STEP lays them out flat instead)
    for _gn, _gp in GAUGE_ASSEMBLED:
        asm.add(_gp.translate(COUPON_SHIFT), name=_gn,
                color=color(COLOR["coupon"] if _gn != "gauge_pin"
                            else COLOR["lever_pin"]))

    num = _build_number_model(build_n)
    if num is not None:
        asm.add(num, name="build_number", color=color(COLOR["build_num"]))

    asm.save(str(VIEWER), mode="default")
    print(f"Wrote {VIEWER.name}  [build #{build_n}]", flush=True)
    show(str(VIEWER))


if __name__ == "__main__":       # guarded so audits can import src.* freely
    main()                       # (cadkit.bead_check walks every module)
