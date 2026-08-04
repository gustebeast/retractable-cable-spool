"""Validate 20 mm M2 cap screws fit every M2 site.

Definition (per user spec):
  A = distance from screw-head BOTTOM (counterbore floor where the head
      seats) to the BOTTOM of the fitted insert (the far end of the
      heat-set insert, opposite from the screw-entry side -- the deepest
      point the screw tip can reach by threading through the insert).
  B = clearance beyond the insert bottom (additional bore + air) before
      hitting solid material that would stop the screw.

For a 20 mm screw:
  - A == 20.00  : screw bottoms exactly at the insert's far end, head fully seats.
  - A <  20.00  : screw exceeds insert length; the (20 - A) mm tip excess
                  must fit into B mm of clearance, else head won't seat.
  - A >  20.00  : screw is too short to reach the insert bottom (only
                  engages the outer threads; usually still OK if engagement
                  >= ~1xD = 2 mm, but the head won't fully torque down).
"""
from v2.dimensions import M2_HEAD_RECESS_H, M2_INSERT_DEPTH, DRUM_OD
from src import wheel as W
from src import housing as H
from src import housing_guide as PG, housing_lever_guide as LG

SCREW_LEN = 20.0
INSERT_PHYSICAL_LEN = 2.5   # per dimensions.py: "2.5 mm insert + 1 mm margin"


def report(name, A, beyond, axis, notes=""):
    if A > SCREW_LEN + 0.001:
        v = f"FAIL: A={A:.2f} mm > {SCREW_LEN:.1f} (screw too SHORT by {A-SCREW_LEN:.2f} mm)"
    else:
        excess = SCREW_LEN - A
        if excess <= 0.001:
            v = f"OK (exact): A={A:.2f} mm, head seats just as tip bottoms"
        elif excess <= beyond + 0.001:
            v = f"OK: A={A:.2f}, screw tip projects {excess:.2f} mm past insert, have {beyond:.2f} mm clearance"
        else:
            v = (f"FAIL: A={A:.2f}, screw tip projects {excess:.2f} mm past insert, "
                 f"only {beyond:.2f} mm clearance (short by {excess-beyond:.2f} mm)")
    print(f"  {name:<42}  axis={axis}  {v}")
    if notes:
        print(f"      {notes}")


print(f"SCREW_LEN={SCREW_LEN} mm,  INSERT_PHYSICAL_LEN={INSERT_PHYSICAL_LEN} mm\n")

# ---- Housing Y-axis M2 sites -----------------------------------------------
# Counterbore on +Y face (H.SCREW_HEAD_RECESS_H deep), pilot on -Y face
# (M2_INSERT_DEPTH = 3.5 deep), insert pressed flush with -Y face occupies
# the outer 2.5 mm of the pilot. The screw enters from +Y and threads through
# the insert in -Y direction; "insert bottom" = -Y face exterior plane.
HW = H.HOUSING_W
head_bot_y     = HW / 2 - H.SCREW_HEAD_RECESS_H           # head seats here
neg_face_y     = -HW / 2                                   # insert far end
insert_near_y  = -HW / 2 + INSERT_PHYSICAL_LEN             # insert near end (inside housing)
pilot_tip_y    = -HW / 2 + M2_INSERT_DEPTH                 # deepest pilot point
A_housing = head_bot_y - neg_face_y           # head bottom -> insert FAR end
# Beyond insert bottom (going more -Y): we're outside the housing -> open air.
# So clearance is effectively unbounded for screw-tip protrusion past -Y face.
beyond_housing = float("inf")

print(f"=== Housing Y-axis M2 sites (HOUSING_W={HW:.1f} mm) ===")
print(f"  head_bot_y={head_bot_y:.2f}  insert_far_y={neg_face_y:.2f}  "
      f"insert_near_y={insert_near_y:.2f}  pilot_deepest_y={pilot_tip_y:.2f}")
print(f"  A = head_bot - insert_far = {A_housing:.2f} mm")
print(f"  Beyond insert: insert FAR end is at -Y exterior face --> open air (clearance unbounded)")
for name in ("ratchet lever pivot",
             "brake lever pivot",
             "pancake axle cross-pin",
             "mount_bracket TOP M2",
             "mount_bracket SIDE M2"):
    report(name, A_housing, beyond_housing, axis="Y")
print()

# ---- Guide-wheel M2 sites (X-axis through slab + pocket + slab) ------------
# m2_hole_cut(head_side="outboard"):
#   head pocket: outer MOUNT_OUTER_T (2 mm) of outboard slab (Ø MOUNT_HEAD_HOLE_D)
#   insert pocket: outer MOUNT_OUTER_T (2 mm) of inboard slab (Ø M2_INSERT_PILOT_D = 3.3)
#   tip_blind: cylinder Ø M2_SHAFT_CLR_D from inboard_outer_x - TIP_CLEARANCE_DEPTH
#              to inboard_outer_x -- BUT it's cut into the spool-gap air, so it
#              doesn't actually create geometry; it's a no-op cutter.
# Insert is 2.5 mm long, pocket is only 2.0 mm deep at Ø3.3 -- the insert's inner
# 0.5 mm has nowhere to seat (the bore narrows to Ø2.3 shaft clearance past
# MOUNT_OUTER_T). Effective insert occupies the 2 mm pocket only.
half      = W.WHEEL_W / 2 + W.HUB_H + W.HUB_AXIAL_CLR     # = 4.2
inb_out   = -(half + W.MOUNT_T)                            # inboard slab outer face (relative to x_c)
out_out   = +(half + W.MOUNT_T)                            # outboard slab outer face
head_bot  = out_out - W.MOUNT_OUTER_T                      # head seats here
insert_far = inb_out                                        # insert far end = inboard slab outer face
# Insert PHYSICAL fit: pocket only 2 mm, insert is 2.5 mm; if it sat flush at
# inb_out it would need 2.5 mm of Ø3.3 bore but the pilot only provides 2.0 mm.
# Realistically the installer would either (a) leave 0.5 mm of insert proud, or
# (b) press it in until the inner 0.5 mm cuts into the Ø2.3 shaft bore (won't
# go in without damage). Treat the insert FAR end as inb_out (outer face) and
# flag the depth issue separately.
A_wheel = head_bot - insert_far
# Beyond insert (more -X): out into the spool-gap air. Tip travels until it
# hits the spool's pancake rim (at radius DRUM_OD/2) or, on the lever side,
# the toothed rim (also bounded by DRUM_OD/2 outer extent).
print(f"=== Guide-wheel M2 sites (X-axis through wheel mount) ===")
print(f"  head_bot_dx={head_bot:.2f}  insert_far_dx={insert_far:.2f}")
print(f"  A = head_bot - insert_far = {A_wheel:.2f} mm   (relative to wheel x_center)")
print(f"  NOTE: insert pocket is {W.MOUNT_OUTER_T} mm deep but heat-set insert is "
      f"{INSERT_PHYSICAL_LEN} mm long -- insert can't fully seat without enlarging "
      f"the Ø{W.MOUNT_INSERT_HOLE_D} pilot deeper.")

for label, mod in (("pancake guide wheel", PG), ("lever guide wheel", LG)):
    wxc = mod.WHEEL_X_CENTER
    head_bot_abs = wxc + head_bot
    insert_far_abs = wxc + insert_far
    tip_abs = head_bot_abs - SCREW_LEN
    # Past the inboard slab outer face at z=WHEEL_Z_CENTER: at the wheel's
    # z, the pancake/lever plate fully covers the radial space between the
    # wheel pocket and the spool drum (the pocket only opens a small notch).
    # So past the slab outer face is SOLID PLATE MATERIAL -- zero clearance
    # unless the tip exits the plate's lateral footprint.
    # The wheel pocket extends a few mm wider than the wheel OD in Y; the
    # screw is at y=0 (wheel center y), so the tip is well within the plate.
    beyond = 0.0   # solid plate material immediately past insert
    notes = (f"wheel_x_center={wxc:.2f}  insert_far_x={insert_far_abs:.2f}  "
             f"tip_x={tip_abs:.2f}  (tip is inside pancake/lever plate material)")
    report(label, A_wheel, beyond, axis="X", notes=notes)
