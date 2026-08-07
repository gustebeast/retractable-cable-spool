"""Bead-grid checker: every PRINTED length must be a whole multiple of the nozzle.

  py -3.12 -m tools.check_beads             # report off-grid constants
  py -3.12 -m tools.check_beads --all       # also list what is exempt, and why
  py -3.12 -m tools.check_beads --only params,frame

WHY: this build runs a 0.8 mm nozzle. A wall of 1.75 mm is 2.19 beads -- the
slicer (Arachne) cannot lay 2.19 beads, so it either widens two beads to 0.88
or squeezes in a starved third. Fine on a cosmetic face, NOT fine on a
load-bearing ledge, where the improvised bead is exactly where a part
delaminates. Sizing on the grid means what you drew is what gets extruded.

THE RULE (user): every length is either N * BEAD, or another feature +/- N * BEAD.

Two different tests (cadkit.bead_check owns the logic):

  BARE LITERAL   (WALL_T = 2.0)          -> the VALUE must be on the grid.
  DERIVED        (Y_HI = AXLE_Y + 3.0)   -> the OFFSETS must be on the grid;
                                            the RESULT inherits its datum and
                                            may sit anywhere.

THREE THINGS ARE LEGITIMATELY OFF-GRID, each with a reason in EXEMPT below:

  hardware  -- a dummy modelling a REAL object (608 bearing, the spring, the
               user's measured cable). Rounding these lies about fit.
  clearance -- gaps, not material: no bead is ever laid across one.
  layout    -- tuned knobs pinned by model asserts, domain-set dims.

Anything else off-grid is a finding. Exit code = number of findings.
This file is the project DRIVER; the walker itself is cadkit.bead_check.
"""

from __future__ import annotations

import pathlib

from cadkit import bead_check

SRC = pathlib.Path(__file__).resolve().parent.parent / "src"

# ── exemptions ──────────────────────────────────────────────────────────────
# name -> (category, reason). A bare name matches in EVERY module; "mod.NAME"
# (spelled mod__NAME below) pins it to one. Keep the reason specific.
EXEMPT: dict[str, tuple[str, str]] = {}


def _ex(cat: str, **kv) -> None:
    for k, v in kv.items():
        EXEMPT[k.replace("__", ".")] = (cat, v)


# Real objects. These are measurements, not choices.
_ex("hardware",
    BEARING_OD="608 bearing OD 22.0",
    BEARING_ID="608 bearing bore 8.0",
    BEARING_W="608 bearing width 7.0",
    BEARING_ORACE_ID="608 outer-race seal-line annulus ID (v2-measured)",
    BEARING_IRACE_OD="608 inner-race face annulus OD (v2-measured)",
    BEARING_INNER_CH="608 ring-edge chamfer r_s min ~0.3 (spec)",
    SPRING_FLANGE_OD="spring flange Ø78.3 (measured)",
    SPRING_FLANGE_T="spring flange thickness (measured)",
    SPRING_BODY_OD="spring coil body Ø (measured)",
    SPRING_H="spring total height (measured)",
    SPRING_BORE_TOP_D="spring top-flange hole Ø13.4 (measured)",
    SPRING_BORE_BOT_D="spring bottom-flange hole Ø8.7 (user-measured)",
    SPRING_STRIP_T="spring strip steel 0.2 (v2-measured)",
    SPRING_STRIP_W="spring strip width 22 (v2-measured)",
    CABLE_D="working cable Ø3.85 (user-measured)",
    CABLE_CAPACITY="1 m of cable -- the payload, not printed",
    CABLE_BEND_R_MIN="cable safe bend radius (spec)",
    AXLE_D="the 608's nominal bore Ø8",
    STRIP_BODY_W="strip tip geometry (user-measured)",
    STRIP_TAPER_L="strip tip geometry (user-measured)",
    STRIP_NECK_W="strip tip geometry (user-measured)",
    STRIP_NECK_L="strip tip geometry (user-measured)",
    STRIP_HEAD_W="strip tip geometry (user-measured)",
    STRIP_HEAD_L="strip tip geometry (user-measured)",
    SEP_PASS_W="Ø13 connector must pass (hardware envelope)",
    ENTRY_PORT_W="Ø13 connector must pass (= SEP_PASS_W)",
    WOOD_SCREW_SHAFT_D="v2-validated wood screw shaft Ø4",
    WOOD_SCREW_HEAD_D="wood screw countersunk head Ø9.3 (measured)",
    WOOD_SCREW_HEAD_H="wood screw head cone depth (measured)",
    )

_ex("hardware",
    _RACE_GROOVE="cosmetic race-band groove on the 608 dummy (viewer-only)",
    )

# Gaps, not material -- no bead is laid across a clearance.
_ex("clearance",
    _ARCH_CLR="fork arch roof off the lever's swept ceiling (air, v2)",
    PAWL_CLEAR_MM="pawl-to-tooth-tip swing clearance target (kinematics)",
    BRAKE_REST_MM="pad-to-band rest gap target (kinematics)",
    RSTOP_X0="shelf inset inside the wing's open-pose footprint (margin)",
    RSTOP_X1="shelf inset inside the wing's open-pose footprint (margin)",
    RWING_X0="0.05 extra air over the rim-clearance limit",
    _GUARD="handle-face static guard gap (assert bound)",
    BOSS_BORE_ID="rotating-diamond envelope clearance (0.2/side)",
    BRAKE_STOP_GAP="designed TPU squish interference at plumb (user)",
    BRG_SEAT_CONE_Z0="outer race seats 0.1 below nominal (seat allowance)",
    CABLE_EXIT_Z0="exit-window margin past the chamber floor (cut edge)",
    CABLE_EXIT_Z1="exit-window margin past the lid plane (cut edge)",
    COIL_PITCH="0.1 wrap-on-wrap slack over the cable diameter",
    ENTRY_PORT_SILL="connector lift off the chamber floor (v2)",
    LEVER_SIDE_CLR="air each side of the lever plates",
    LOOP_D_LOOSE="0.7/side breathing gap off the wall bore",
    PAD_JOINT_CLR="TPU-in-PETG snug fit (v2 print-proven)",
    PIN_SQ_FRAME_CLR="TPU axle in frame keyway/bore, per side",
    PIN_SQ_LEVER_CLR="TPU axle in the lever pocket, per side",
    )

# Tuned knobs pinned by model asserts, kinematic datums, cutter margins.
_ex("layout",
    _SLIT_Y_OVERSHOOT="slit cutter overshoot into open air (nothing printed)",
    BRAKE_FLUSH_TOL_MM="flush-check solver tolerance (>= pad-width sagitta)",
    _EPS="1e-6 numerical epsilon, not a length",
    BRAKE_PIVOT_Z="kinematic pivot datum (v2's 5.5-below-band rule) -- a position",
    RATCHET_PIVOT_Z="kinematic pivot datum (v2's 10-above-band rule) -- a position",
    BRAKE_RUBBER_T="v1/v2 print-proven TPU pad slab (squish-tuned)",
    CH_BOT_Z="chamber AIR depth: sill lift + 1.5x connector-house gable (a void)",
    HORN_CH_W="the 1.5 is the bore/cable RATIO; the result IS snapped up",
    KNURL_DEPTH="half-bead ridge texture field (v2 print-proven)",
    LEVER_HANDLE_W="pinned: snap-up trips A_PAD_HANDLE_CLR and the pivot has "
                   "no packaging room left (see the params.py comment)",
    PIN_SQ_S="largest side passing the frame packaging asserts (pinned knob)",
    TOP_CONE_Z1="pinned by the flange-hole-zone assert (cone top under the "
                "frame face)",
    )

_ex("clearance",
    FIT_CLR="slip fit 0.15/side (v2 print-proven)",
    BEARING_CLR="bearing pocket diametral clearance 0.2",
    TENON_SHOULDER_CLR="rotating gap under the spring",
    SEP_SPRING_CLR="separator-spring gap",
    ANCH_FLANGE_CLR="anchor wall off the flange edges",
    ANCH_BAY_BIAS="bay-centre bias inside the strip's float band",
    LIDJ_Z_CLR="lid bayonet z-sandwich 0.20 (user-tightened)",
    LIDJ_SEAT_CLR="bayonet seating clearance (= JOINT_CLR)",
    LIDJ_ENTRY_OVER="entry-pocket overshoot past the tenon (gap)",
    TOP_JOINT_SEAT_CLR="frame arc-joint seating clearance (= JOINT_CLR)",
    TOP_ENTRY_OVER="frame arc-channel entry overshoot (gap)",
    JOINT_CLR="cadkit joint lateral clearance (policy)",
    JOINT_BACK_CLR="cadkit joint depth-face clearance (policy)",
    HORNJ_Z_CLR="horn cap rail z-sandwich 0.20 (lid precedent)",
    )


if __name__ == "__main__":
    raise SystemExit(bead_check.cli(
        src=SRC, package="src", exempt=EXEMPT, default_nozzle=0.8))
