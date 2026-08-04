"""Build — THE retractable cable spool.
Assembles the modular design and shows it in the shared FreeCAD hub as
`retractable_cable_spool` — the repo-root STEP, the project's main tab.

Run:  py -3.12 -m v2.build   (from the project root)

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
    spring_housing, spring_housing_cap, spring_gate_insert,
    spring_gate_coupon,
)
from .separator import separator, test_brake_knurl, test_hub_fit
from .frame import frame_bottom, frame_top
from .wall import wall_bottom, wall_top, wall_collar
from .chambers import cable_ceiling, tpu_grip_ring, grip_ring_seated
from .axle import (
    axle_bottom, axle_top, axle_pin, pin_in_place, winding_tool,
    cap_grip_ring_tpu, cap_ring_seated,
)
# the posed end-to-end cable is a ~2 m swept spline — by far the most
# expensive solid in build AND scan. Revertable flag (user's call): flip
# True to show it in the viewer / include it in scans.
SHOW_CABLE = False
if SHOW_CABLE:
    from .cable_path import cable_path
from .coil_cup import coil_cup
from .mount import mount_bracket, mount_lock_tpu, lock_in_place
# (lever_rig RETIRED from the build: its stand anchored to the old shallow
# frame's BOT_RIB/climb internals, which the axial chamber moved; its TPU
# pivot + pad validation prints are long done. Module kept on disk.)
from .t_coupon import t_test_mortise, t_test_tenon
from .frame_joint_coupon import (
    test_frame_joint_bottom, test_frame_joint_top, test_mount_strip, _Z0 as _FJC_Z0,
)
from .levers import (
    ratchet_lever, brake_lever, brake_pad_tpu,
    lever_pin, lever_pin_in_place, KIN, brake_pad_tall_demo,
    BRAKE_CONTACT_DEG,
)
from .params import (
    ROD_Z_BOT, RATCHET_PIVOT_Z, BRAKE_PIVOT_Z, DRIVE_Z0, LEVER_PIVOT_X,
    LEVER_TRAVEL_DEG,
)

# ── Viewer pose (revertable) ─────────────────────────────────────────────────
# Both levers at REST (canonical pose). Set BRAKE_POSE_DEG = BRAKE_CONTACT_DEG
# to preview the brake flush on the band; RATCHET_POSE_DEG likewise for a
# pulled ratchet (its wall stop is at 15°). The TEST RIG group below poses
# its own copies independently (ratchet meshed, brake at contact).
# BRAKE_PAD_DEMO_H forces an oversized viewer-only pad (None = the real one).
# POSE (revertable): both levers at REST. Set BRAKE_POSE_DEG =
# BRAKE_CONTACT_DEG to show the full-band pad flush on the brake rim.
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
    RIM_SEAT_Z, CEIL_Z0, CEIL_CLAMP_Z0,
)

OUT = pathlib.Path(__file__).resolve().parent
# The main viewer STEP is the cadkit-conventional root `assembly.step` (the
# shared View Assembly launcher expects exactly that name); the FreeCAD tab
# reads the PROJECT FOLDER's name, so it still shows retractable-cable-spool.
VIEWER = OUT.parent / "assembly.step"

COLOR = {
    "spring_housing":     "#C4A56B",  # warm tan — spring housing (the backbone)
    "spring_housing_cap": "#8FA88F",  # sage — removable cap (at +Z)
    "separator":          "#B0654B",  # rust — brake/ratchet separator
    "cable_ceiling":      "#7FA37F",  # green — spool-side cable cap (+Z)
    "height_clamp":       "#E0A040",  # amber — (name kept for the poses)
    "frame_bottom":       "#6B8AAB",  # slate — bottom plus + beams (wall mortises, tenons)
    "frame_top":          "#4F6D8F",  # deeper slate — top plus (slide-on mortise channels)
    "wall_bottom":        "#9B8AA6",  # muted violet — containment ring, lower half (−Z→+Z)
    "wall_top":           "#7E6E8F",  # deeper violet — middle piece (prints INVERTED, +Z→−Z)
    "wall_collar":        "#B4A5C2",  # pale violet — top collar ring + bare tenon posts
    "axle_top":           "#A6786B",  # clay — axle top half (mirrored base collar half)
    "axle_bottom":        "#8B5E52",  # deeper clay — axle bottom half (mirrored base lip half)
    "axle_pin":           "#D9C55F",  # brass — diagonal plastic rod pin
    "winding_tool":       "#708090",  # steel grey — pre-wind bar tool (on the square drive)
    "cable":              "#D97B29",  # jacket orange — the posed working cable (4 ft, viewer only)
    "coil_cup":           "#6F9B8E",  # sea green — the axial service-loop chamber cup
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
    ("grip_ring_tpu",      tpu_grip_ring,      "grip_ring_tpu.step"),
    ("frame_bottom",       frame_bottom,       "frame_bottom.step"),
    ("frame_top",          frame_top,          "frame_top.step"),
    ("wall_bottom",        wall_bottom,        "wall_bottom.step"),
    ("wall_top",           wall_top,           "wall_top.step"),
    ("wall_collar",        wall_collar,        "wall_collar.step"),
    ("axle_top",           axle_top,           "axle_top.step"),
    ("axle_bottom",        axle_bottom,        "axle_bottom.step"),
    ("axle_pin",           axle_pin,           "axle_pin.step"),
    ("cap_grip_ring_tpu",  cap_grip_ring_tpu,  "cap_grip_ring_tpu.step"),
    ("coil_cup",           coil_cup,           "coil_cup.step"),
    ("winding_tool",       winding_tool,       "winding_tool.step"),
    ("mount_bracket",      mount_bracket,      "mount_bracket.step"),
    ("mount_lock_tpu",     mount_lock_tpu,     "mount_lock_tpu.step"),
    ("ratchet_lever",      ratchet_lever,      "ratchet_lever.step"),
    ("brake_lever",        brake_lever,        "brake_lever.step"),
    ("lever_pin_tpu",      lever_pin,          "lever_pin_tpu.step"),
    ("brake_pad_tpu",      brake_pad_tpu,      "brake_pad_tpu.step"),
    ("test_t_mortise",     t_test_mortise,     "test_t_mortise.step"),
    ("test_t_tenon",       t_test_tenon,       "test_t_tenon.step"),
    ("test_brake_knurl",   test_brake_knurl,   "test_brake_knurl.step"),
    ("test_hub_fit",       test_hub_fit,       "test_hub_fit.step"),
    ("test_frame_joint_bottom", test_frame_joint_bottom, "test_frame_joint_bottom.step"),
    ("test_frame_joint_top",    test_frame_joint_top,    "test_frame_joint_top.step"),
    ("test_mount_strip",        test_mount_strip,        "test_mount_strip.step"),
]

# Show the TPU-lever test rig BELOW the main assembly (with posed
# lever/pin/pad copies mounted, so fit reads at a glance) — flip to False
# to hide it once the TPU validation is done.
SHOW_LEVER_RIG = False                # RETIRED with the axial chamber (the rig
                                      # anchored to the old shallow frame; its
                                      # TPU validation prints are done)
RIG_OFF = (0.0, 0.0, -110.0)          # rig top ≈ −56, well under the hex drive
# T-joint print-fit coupon (production wall↔beam joint at full lengths),
# shown seated, rotated to the −X side at the rig's level
SHOW_T_COUPON = True
COUPON_OFF = (0.0, 0.0, -110.0)
# Spring-gate print-fit coupon at the test-part level
SHOW_SPRING_GATE_COUPON = True
# Brake-band knurl grip coupon (rub the TPU pad on it) at the test level
SHOW_KNURL_COUPON = True
# Frame top<->bottom four-arc-joint coupon, shown SEATED on its own level
SHOW_FRAME_JOINT_COUPON = True
# Separator->housing hub-fit ring, on its own level below the frame coupon
SHOW_HUB_FIT_COUPON = True


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
           # spool-side cable cap (the tray-side cable_floor left with the
           # flat tray design — the axial coil chamber replaces it)
           .add(cable_ceiling.translate((0, 0, CEIL_Z0)),
                name="cable_ceiling", color=color(COLOR["cable_ceiling"]))
           # TPU grip ring over the ceiling (locks it against the cable
           # pack; posed SEATED — lip line-on-line, pointing down at it)
           .add(grip_ring_seated(True).translate((0, 0, CEIL_CLAMP_Z0)),
                name="ceiling_grip_ring", color=color(COLOR["tpu"]))
           .add(frame_bottom,    name="frame_bottom",   color=color(COLOR["frame_bottom"]))
           # top plus seated on the beam tenons (slid on −x→+x, modelled seated)
           .add(frame_top,       name="frame_top",      color=color(COLOR["frame_top"]))
           # wall halves seated in the beams' dovetail mortises (modelled in
           # place, stacked at the split plane; top half prints inverted)
           .add(wall_bottom,     name="wall_bottom",    color=color(COLOR["wall_bottom"]))
           .add(wall_top,        name="wall_top",       color=color(COLOR["wall_top"]))
           .add(wall_collar,     name="wall_collar",    color=color(COLOR["wall_collar"]))
           # axle halves (modelled in place, glue joint engaged) + the two
           # diagonal pins (one per plus crossing; same printed part twice)
           .add(axle_top,        name="axle_top",       color=color(COLOR["axle_top"]))
           .add(axle_bottom,     name="axle_bottom",    color=color(COLOR["axle_bottom"]))
           .add(pin_in_place(),  name="axle_pin_top",   color=color(COLOR["axle_pin"]))
           .add(pin_in_place(ROD_Z_BOT),
                name="axle_pin_bottom",                 color=color(COLOR["axle_pin"])))
    if SHOW_CABLE:
        asm.add(cable_path, name="cable_path", color=color(COLOR["cable"]))
    asm = (asm
           # cap-retention TPU ring SEATED flush under the housing cap's
           # bottom face (bite=0 pose — line-on-line on the shaft)
           .add(cap_ring_seated(spring_housing_cap.val().BoundingBox().zmin),
                name="cap_grip_ring_tpu", color=color(COLOR["tpu"]))
           # the axial-loop chamber cup, resting on the lowered bottom plus
           .add(coil_cup,        name="coil_cup",       color=color(COLOR["coil_cup"]))
           # winding tool shown ENGAGED on the square drive below the frame
           # (bottom-aligned with the drive tip)
           .add(winding_tool.translate((0, 0, DRIVE_Z0)),
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

    if SHOW_FRAME_JOINT_COUPON:
        # both pieces posed SEATED (mated), on their own level below the rig
        asm.add(test_frame_joint_bottom.translate((0.0, 0.0, -230.0)),
                name="test_frame_joint_bottom", color=color(COLOR["test_rig"]))
        asm.add(test_frame_joint_top.translate((0.0, 0.0, -230.0)),
                name="test_frame_joint_top", color=color(COLOR["frame_top"]))
        # the mount strip posed SEATED in the coupon's X channels (world
        # pose shifted into the coupon's base frame)
        asm.add(test_mount_strip.translate((0.0, 0.0, -230.0 - _FJC_Z0)),
                name="test_mount_strip", color=color(COLOR["mount_bracket"]))

    if SHOW_HUB_FIT_COUPON:
        asm.add(test_hub_fit.translate((0.0, 0.0, -280.0)),
                name="test_hub_fit", color=color(COLOR["separator"]))

    if SHOW_KNURL_COUPON:
        # knurl grip coupon: the band arc rotated to the −Y side of the
        # test-part level, clear of the rig (+X) and gate coupon (+Y)
        asm.add(test_brake_knurl.rotate((0, 0, 0), (0, 0, 1), -90.0)
                                .translate((0.0, 0.0, -110.0)),
                name="test_brake_knurl", color=color(COLOR["test_rig"]))

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


    num = _build_number_model(build_n)
    if num is not None:
        asm.add(num, name="build_number", color=color(COLOR["build_num"]))

    asm.save(str(VIEWER), mode="default")
    print(f"Wrote {VIEWER.name}  [build #{build_n}]", flush=True)
    show(str(VIEWER))


main()
