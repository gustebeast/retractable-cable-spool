"""Build — THE retractable cable spool.
Assembles the modular design and shows it in the shared FreeCAD hub as
`retractable_cable_spool` — the repo-root STEP, the project's main tab.

Run:  py -3.12 -m src.build   (from the project root)

Parts so far: spring chamber (center) + top bearing cap + the 12 mm middle
brake/ratchet rim seated at half height + the 4-beam housing with its spool/tray
wall ring. The rest of the sliding stack (spool ceiling/wall, tray floor) follows.
"""

import pathlib

import cadquery as cq

from cadkit.step_export import export_step
from cadkit.cq_colors import color
from cadkit.freecad import show

from .spring_housing import (
    spring_housing, spring_housing_cap, spring_gate_insert, spring_strip_end,
    spring_gate_coupon,
)
from .separator import separator
from .frame import frame_bottom, frame_top
from .wall import wall_bottom, wall_top
from .chambers import cable_ceiling, cable_floor, height_clamp
from .axle import axle_bottom, axle_top, axle_pin, pin_in_place, winding_tool
from .mount import mount_bracket, mount_lock_tpu, lock_in_place
from .lever_rig import lever_test_rig, lever_test_wall
from .t_coupon import t_test_mortise, t_test_tenon
from .levers import (
    ratchet_lever, brake_lever, brake_pad_tpu,
    lever_pin, lever_pin_in_place, KIN, brake_pad_tall_demo,
    BRAKE_CONTACT_DEG,
)
from .params import (
    ROD_Z_BOT, RATCHET_PIVOT_Z, BRAKE_PIVOT_Z, HEX_DRIVE_Z0, LEVER_PIVOT_X,
    LEVER_TRAVEL_DEG,
)

# ── Viewer pose (revertable) ─────────────────────────────────────────────────
# Both levers at REST (canonical pose). Set BRAKE_POSE_DEG = BRAKE_CONTACT_DEG
# to preview the brake flush on the band; RATCHET_POSE_DEG likewise for a
# pulled ratchet (its wall stop is at 15°). The TEST RIG group below poses
# its own copies independently (ratchet meshed, brake at contact).
# BRAKE_PAD_DEMO_H forces an oversized viewer-only pad (None = the real one).
BRAKE_PAD_DEMO_H = None
BRAKE_POSE_DEG = 0.0
RATCHET_POSE_DEG = 0.0


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


# The RIG's brake pose, independent of the main viewer's flags — 0.0 = rest
# (pad off the band on its rest tab); BRAKE_CONTACT_DEG shows the flush mate.
RIG_BRAKE_POSE_DEG = 0.0


def _rig_contact(part):
    if not RIG_BRAKE_POSE_DEG:
        return part
    return part.rotate((LEVER_PIVOT_X, 0, BRAKE_PIVOT_Z),
                       (LEVER_PIVOT_X, 1, BRAKE_PIVOT_Z), -RIG_BRAKE_POSE_DEG)
from .params import (
    RIM_SEAT_Z, CEIL_Z0, FLOOR_Z0, CEIL_CLAMP_Z0, FLOOR_CLAMP_Z0,
)

OUT = pathlib.Path(__file__).resolve().parent
# The main viewer STEP lives at the REPO ROOT under the project's own name —
# its FreeCAD tab reads "retractable_cable_spool" (the project's main tab).
VIEWER = OUT.parent / "retractable_cable_spool.step"

COLOR = {
    "spring_housing":     "#C4A56B",  # warm tan — spring housing (the backbone)
    "spring_housing_cap": "#8FA88F",  # sage — removable cap (now at −Z)
    "separator":          "#B0654B",  # rust — brake/ratchet separator
    "cable_ceiling":      "#7FA37F",  # green — spool-side cable cap (+Z)
    "cable_floor":        "#5F8F6F",  # deeper green — tray-side cable cap (−Z)
    "height_clamp":       "#E0A040",  # amber — C-clamp height lock
    "frame_bottom":       "#6B8AAB",  # slate — bottom plus + beams (wall mortises, tenons)
    "frame_top":          "#4F6D8F",  # deeper slate — top plus (slide-on mortise channels)
    "wall_bottom":        "#9B8AA6",  # muted violet — containment ring, lower half (−Z→+Z)
    "wall_top":           "#7E6E8F",  # deeper violet — upper half (prints INVERTED, +Z→−Z)
    "axle_top":           "#A6786B",  # clay — axle top half (mirrored base collar half)
    "axle_bottom":        "#8B5E52",  # deeper clay — axle bottom half (mirrored base lip half)
    "axle_pin":           "#D9C55F",  # brass — diagonal plastic rod pin
    "winding_tool":       "#708090",  # steel grey — pre-wind bar tool (on the hex drive)
    "mount_bracket":      "#96A48B",  # olive grey — desk/wall mount bar (tenons down)
    "test_rig":           "#7FA3AD",  # slate teal — TPU-lever test stand (below)
    "test_wall":          "#A87C68",  # clay — the rig's slide-in toothed spool stand-in
    "ratchet_lever":      "#C9825F",  # copper — ratchet lever (+y side)
    "brake_lever":        "#B45A5A",  # brick — brake lever (−y side)
    "tpu":                "#141414",  # black — 95A TPU parts (brake pad + torsion pins;
                                      # colour reserved for TPU)
    "build_num":          "#F0A878",  # salmon — floating build number
}

# One STEP per printed part (viewer-toggleable, slicer-ready).
PARTS = [
    ("spring_housing",     spring_housing,     "spring_housing.step"),
    ("spring_housing_cap", spring_housing_cap, "spring_housing_cap.step"),
    ("spring_gate_insert", spring_gate_insert, "spring_gate_insert.step"),
    ("test_spring_gate",   spring_gate_coupon, "test_spring_gate.step"),
    ("separator",          separator,          "separator.step"),
    ("cable_ceiling",      cable_ceiling,      "cable_ceiling.step"),
    ("cable_floor",        cable_floor,        "cable_floor.step"),
    ("height_clamp",       height_clamp,       "height_clamp.step"),
    ("frame_bottom",       frame_bottom,       "frame_bottom.step"),
    ("frame_top",          frame_top,          "frame_top.step"),
    ("wall_bottom",        wall_bottom,        "wall_bottom.step"),
    ("wall_top",           wall_top,           "wall_top.step"),
    ("axle_top",           axle_top,           "axle_top.step"),
    ("axle_bottom",        axle_bottom,        "axle_bottom.step"),
    ("axle_pin",           axle_pin,           "axle_pin.step"),
    ("winding_tool",       winding_tool,       "winding_tool.step"),
    ("mount_bracket",      mount_bracket,      "mount_bracket.step"),
    ("mount_lock_tpu",     mount_lock_tpu,     "mount_lock_tpu.step"),
    ("ratchet_lever",      ratchet_lever,      "ratchet_lever.step"),
    ("brake_lever",        brake_lever,        "brake_lever.step"),
    ("lever_pin_tpu",      lever_pin,          "lever_pin_tpu.step"),
    ("brake_pad_tpu",      brake_pad_tpu,      "brake_pad_tpu.step"),
    ("test_lever_rig",     lever_test_rig,     "test_lever_rig.step"),
    ("test_lever_wall",    lever_test_wall,    "test_lever_wall.step"),
    ("test_t_mortise",     t_test_mortise,     "test_t_mortise.step"),
    ("test_t_tenon",       t_test_tenon,       "test_t_tenon.step"),
]

# Show the TPU-lever test rig BELOW the main assembly (with posed
# lever/pin/pad copies mounted, so fit reads at a glance) — flip to False
# to hide it once the TPU validation is done.
SHOW_LEVER_RIG = True
RIG_OFF = (0.0, 0.0, -110.0)          # rig top ≈ −56, well under the hex drive
# T-joint print-fit coupon (production wall↔beam joint at full lengths),
# shown seated, rotated to the −X side at the rig's level
SHOW_T_COUPON = True
COUPON_OFF = (0.0, 0.0, -110.0)
# The steel spring strip's end posed in the anchor (viewer model only)
SHOW_SPRING_STRIP = True
# Spring-gate print-fit coupon at the test-part level
SHOW_SPRING_GATE_COUPON = True


def _bump_build_counter():
    f = OUT / "build_n.txt"
    n = 1
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
    """Floating build number ABOVE the assembly, upright so it reads from the
    front. The counter was seeded with the old spool's 583 builds (this design continues
    the same running total), so this is the true all-time build count."""
    try:
        return (cq.Workplane("XZ").text(str(n), 12, 2)
                .translate((0.0, 0.0, 80.0)))
    except Exception:                                       # noqa: BLE001
        return None


def main():
    for _name, part, fname in PARTS:
        export_step(part, str(OUT / fname))

    build_n = _bump_build_counter()

    asm = (cq.Assembly(name="retractable_cable_spool")
           .add(spring_housing,     name="spring_housing",
                color=color(COLOR["spring_housing"]))
           .add(spring_housing_cap, name="spring_housing_cap",
                color=color(COLOR["spring_housing_cap"]))
           # spring-gate insert SEATED through both gate blocks (hardware-
           # free spring retention; modelled in place inside the cavity)
           .add(spring_gate_insert, name="spring_gate_insert",
                color=color(COLOR["axle_pin"]))
           # separator seats on the spring-housing cone notch at half height
           .add(separator.translate((0, 0, RIM_SEAT_Z)),
                name="separator",    color=color(COLOR["separator"]))
           # cable caps either side of the separator (empty/collapsed nominal)
           .add(cable_ceiling.translate((0, 0, CEIL_Z0)),
                name="cable_ceiling", color=color(COLOR["cable_ceiling"]))
           .add(cable_floor.translate((0, 0, FLOOR_Z0)),
                name="cable_floor",   color=color(COLOR["cable_floor"]))
           # a height_clamp on the far face of each cap (locks it against the cable)
           .add(height_clamp.translate((0, 0, CEIL_CLAMP_Z0)),
                name="ceiling_clamp", color=color(COLOR["height_clamp"]))
           .add(height_clamp.translate((0, 0, FLOOR_CLAMP_Z0)),
                name="floor_clamp",   color=color(COLOR["height_clamp"]))
           .add(frame_bottom,    name="frame_bottom",   color=color(COLOR["frame_bottom"]))
           # top plus seated on the beam tenons (slid on −x→+x, modelled seated)
           .add(frame_top,       name="frame_top",      color=color(COLOR["frame_top"]))
           # wall halves seated in the beams' dovetail mortises (modelled in
           # place, stacked at the split plane; top half prints inverted)
           .add(wall_bottom,     name="wall_bottom",    color=color(COLOR["wall_bottom"]))
           .add(wall_top,        name="wall_top",       color=color(COLOR["wall_top"]))
           # axle halves (modelled in place, glue joint engaged) + the two
           # diagonal pins (one per plus crossing; same printed part twice)
           .add(axle_top,        name="axle_top",       color=color(COLOR["axle_top"]))
           .add(axle_bottom,     name="axle_bottom",    color=color(COLOR["axle_bottom"]))
           .add(pin_in_place(),  name="axle_pin_top",   color=color(COLOR["axle_pin"]))
           .add(pin_in_place(ROD_Z_BOT),
                name="axle_pin_bottom",                 color=color(COLOR["axle_pin"]))
           # winding tool shown ENGAGED on the hex drive below the frame
           # (bottom-aligned with the hex tip)
           .add(winding_tool.translate((0, 0, HEX_DRIVE_Z0)),
                name="winding_tool",  color=color(COLOR["winding_tool"]))
           # mount bracket SEATED (X-install shown: tenons butting the
           # mortise stops) + the TPU lock in the +Y cross-arm end mortise
           .add(mount_bracket,   name="mount_bracket",  color=color(COLOR["mount_bracket"]))
           .add(lock_in_place(), name="mount_lock_tpu", color=color(COLOR["tpu"]))
           # levers (modelled at REST: pawl meshed, brake pad off the band)
           # + the 95A TPU parts (black): torsion-bar pivot pins + brake pad
           .add(_ratchet_pose(ratchet_lever),
                name="ratchet_lever",  color=color(COLOR["ratchet_lever"]))
           .add(_brake_pose(brake_lever),
                name="brake_lever",    color=color(COLOR["brake_lever"]))
           .add(lever_pin_in_place(RATCHET_PIVOT_Z, +1, pull_deg=RATCHET_POSE_DEG),
                name="ratchet_pin_tpu",                 color=color(COLOR["tpu"]))
           .add(lever_pin_in_place(BRAKE_PIVOT_Z, -1, pull_deg=BRAKE_POSE_DEG),
                name="brake_pin_tpu",                   color=color(COLOR["tpu"]))
           .add(_brake_pose(brake_pad_tall_demo(BRAKE_PAD_DEMO_H)
                            if BRAKE_PAD_DEMO_H else brake_pad_tpu),
                name="brake_pad_tpu",  color=color(COLOR["tpu"])))
    if SHOW_SPRING_GATE_COUPON:
        # spring-gate print-fit coupon (standing wall slice + both gate
        # blocks), parked at the +Y side of the test-part level
        asm.add(spring_gate_coupon.rotate((0, 0, 0), (0, 0, 1), 90.0)
                                  .translate((0.0, 0.0, -110.0)),
                name="spring_gate_coupon", color=color(COLOR["test_rig"]))

    if SHOW_SPRING_STRIP:
        # the STEEL STRIP's end shown installed (viewer model only, not a
        # printed part): neck in the key's slit under the stop, head
        # bearing on the block, taper + body wrapping CW toward the coil
        asm.add(spring_strip_end, name="spring_strip_end",
                color=color("#8A8F99"))

    if SHOW_T_COUPON:
        # T-joint print-fit coupon (PRODUCTION wall↔beam joint, full
        # lengths), shown SEATED on the −X side of the rig level
        for nm, part, ck in (
            ("t_coupon_mortise", t_test_mortise, "test_rig"),
            ("t_coupon_tenon",   t_test_tenon,   "test_wall"),
        ):
            asm.add(part.rotate((0, 0, 0), (0, 0, 1), 180.0)
                        .translate(COUPON_OFF),
                    name=nm, color=color(COLOR[ck]))

    if SHOW_LEVER_RIG:
        # rig poses are independent of the main viewer's: the RATCHET copy
        # sits at REST, MESHED into the rig's teeth (that engagement is what
        # the rig exists to show); the brake copy stays at contact
        for nm, part, ck in (
            ("rig_stand",       lever_test_rig,                    "test_rig"),
            ("rig_wall",        lever_test_wall,                   "test_wall"),
            ("rig_ratchet",     ratchet_lever,                     "ratchet_lever"),
            ("rig_brake",       _rig_contact(brake_lever),         "brake_lever"),
            ("rig_ratchet_pin", lever_pin_in_place(RATCHET_PIVOT_Z, +1),
                                                                   "tpu"),
            ("rig_brake_pin",   lever_pin_in_place(
                BRAKE_PIVOT_Z, -1, pull_deg=RIG_BRAKE_POSE_DEG),   "tpu"),
            ("rig_brake_pad",   _rig_contact(brake_pad_tpu),       "tpu"),
        ):
            asm.add(part.translate(RIG_OFF), name=nm,
                    color=color(COLOR[ck]))

    num = _build_number_model(build_n)
    if num is not None:
        asm.add(num, name="build_number", color=color(COLOR["build_num"]))

    asm.save(str(VIEWER), mode="default")
    print(f"Wrote {VIEWER.name}  [build #{build_n}]", flush=True)
    show(str(VIEWER))


main()
