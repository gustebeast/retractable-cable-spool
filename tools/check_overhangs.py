"""PRINT-POSE OVERHANG GATE — every printed part, in the orientation it
actually prints (build.PRINT_ROT), scanned for downward-facing material
the printer would have to bridge or support.

    py -3.12 -m tools.check_overhangs

THE RULE (user, project doctrine): a face is a problem when BOTH
  · it faces downward steeper than 45° off horizontal — the
    self-supporting limit this project designs to — AND
  · its BRIDGE SPAN is more than one bead (NOZZLE_D).
A bridge of a bead or less is considered SUPPORTED: the nozzle lays a
single strand across it and it is a dull flat, not a droop. That is why
the octagon arc joints' one-bead cavity roofs (frame_top ↔ mount_ring)
pass, and why the separator's dulled mortise tip passes — both are
deliberate one-bead flats.

SPAN is measured as 2 × area / perimeter, which is the width of the face
however it is shaped: a w × L strip gives w when L ≫ w, an annulus of
width w gives w, and a disc of radius R gives R. Bounding boxes cannot
do this — an arc-shaped one-bead roof has a bbox the size of its chord.

Faces within FIRST_LAYER of the bed are skipped: they ARE the bed, or
near enough that the first layers carry them.
"""

import math

from cadkit.step_export import print_pose

from src.build import PARTS, PRINT_ROT, COUPONS
from src.params import NOZZLE_D

ANGLE_LIMIT = 45.5          # degrees off horizontal; 45 + slicer slack
FIRST_LAYER = 0.5           # mm above the bed to ignore
SPAN_LIMIT = NOZZLE_D       # one bead bridges fine


def _span(face):
    """The face's bridge width: 2 × area / perimeter."""
    per = sum(e.Length() for e in face.Edges())
    return 2.0 * face.Area() / per if per > 1e-9 else 0.0


def scan(part, rotate):
    """[(area, span, angle, x, y, z)] for every unsupported face."""
    posed = print_pose(part, rotate)
    z_bed = posed.val().BoundingBox().zmin
    out = []
    for f in posed.val().Faces():
        try:
            n = f.normalAt(f.Center())
        except Exception:          # degenerate face — no normal to probe
            continue
        if n.z >= -1e-6:
            continue
        ang = math.degrees(math.asin(min(1.0, -n.z)))
        c = f.Center()
        if ang <= ANGLE_LIMIT or c.z <= z_bed + FIRST_LAYER:
            continue
        s = _span(f)
        if s > SPAN_LIMIT + 1e-9:
            out.append((f.Area(), s, ang, c.x, c.y, c.z))
    out.sort(reverse=True)
    return out


def main():
    print(f"overhang limit = {ANGLE_LIMIT:.0f} deg off horizontal, "
          f"bridges over {SPAN_LIMIT} mm ({NOZZLE_D} nozzle = one bead)")
    bad = 0
    scanned = [(n, p, PRINT_ROT.get(n)) for n, p, _f in PARTS]
    # COUPONS get printed too, so they get gated too (#1034: the pin
    # gauge's lip shipped a 2.45 overhang because they were skipped)
    scanned += [(f.replace(".step", ""), p, rot) for f, p, rot in COUPONS]
    for name, part, rot in scanned:
        faces = scan(part, rot)
        if not faces:
            print(f"  {name:<18} clean")
            continue
        bad += len(faces)
        total = sum(f[0] for f in faces)
        print(f"  {name:<18} {len(faces)} face(s), {total:.2f} mm2")
        for a, s, ang, x, y, z in faces[:6]:
            print(f"       {a:7.2f} mm2  span {s:5.2f}  {ang:3.0f} deg   "
                  f"print xyz ({x:.1f}, {y:.1f}, {z:.1f})")
    print()
    if bad:
        print(f"== {bad} UNSUPPORTED face(s) — each needs a 45 deg, a bead, "
              f"or a deliberate exception ==")
    else:
        print("clean: every downward face is 45 deg or bridges one bead")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
