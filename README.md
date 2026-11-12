# Retractable Cable Spool

A 3D-printable retractable cable spool. A power spring winds the drum; a
ratchet holds the cable out at any length; a one-click pull on the lever
releases it. It mounts under a couch or a desk and routes a USB-A-to-C
source cable to the powered end.

This is **v3**, the rotating-axle design, and it lives entirely in `src/`.
(v1 and v2 are in git history only. The old `demo.png` in this directory
is a v1 render and does not show this design.)

## Building

```bash
py -3.12 -m pip install cadquery
py -3.12 -m src.build
```

That writes one STEP per printed part into `src/`, bumps the build counter
in `src/build_n.txt`, and hands the assembly to the web viewer. There is no
part-selection CLI — the whole model builds in about a minute.

Or run the wrapper `./build.sh`, and double-click `View Assembly.cmd` to
reopen the last build in the viewer.

### Gates

Four checks, all expected to pass before anything is printed:

```bash
py -3.12 -m tools.check_overlaps     # parts intersecting where they shouldn't
py -3.12 -m tools.check_beads        # every printed length a whole number of beads
py -3.12 -m tools.check_overhangs    # every downward face 45 deg, or a one-bead bridge
py -3.12 tools/check_geometry.py     # geometry regression vs tools/baseline.tsv
```

`check_geometry.py --update-baseline` blesses a deliberate geometry change.
Run it after any intentional change, and *not* after a refactor that was
supposed to change nothing — that is exactly what it is there to catch.

## Install site

`COUCH_VERSION` in `src/params.py` is one switch that picks the whole
build, because the cable differs per site as well as the geometry:

| | `True` (couch) | `False` (desk) |
|---|---|---|
| cable | Ø3.3, 7 ft | Ø4.0, 4 ft |
| frame-top pad | 10.4 | 2.4 |

It currently builds the **couch** version. Flip the flag and rebuild; never
hand-edit the gated numbers, or switching back silently keeps the wrong
spool.

## Printed parts

Twelve parts. **PETG-GF through a 0.8 mm nozzle**, except the two TPU parts.
**No support material anywhere** — every overhang is 45° or steeper and
every bridge is one bead wide, which is what `check_overhangs` enforces.
Each part's bed face is declared in `PRINT_ORIENTATION` in `src/params.py`
and the STEPs are exported already rotated into it, so the slicer needs no
manual orientation.

| Part | What it is |
|---|---|
| `frame_top.step` | The plus-spider top frame. Hosts the upper 608 in a pocket in its own thickness, and carries the lever mount and the cable horn. |
| `frame_bottom.step` | The lower frame: floor, wall band, the four beams, and the lower bearing seat. Joins `frame_top` with arc tenons at the four stations. |
| `mount_ring.step` | The fixed part. Screws to the couch or desk, then the whole spool rotates on and locks with the two TPU pins. |
| `horn_cap.step` | Closes the cable horn's guide mouth. |
| `axle_top.step` | The rotating shaft. Installs from below, up through the spring and into the 608. |
| `axle_separator.step` | The separator disc: carries the ratchet star, the spring's tenon post, and the drum. |
| `bearing_nub.step` | Seats the lower 608 against the separator. |
| `lid.step` | The drum lid, with the spoke windows. |
| `ratchet_lever.step` | The pawl and the handle, in one part. Pull to release. |
| `lever_pin.step` | The lever's pivot pin. Keyed by a wedge, not a D-flat. |
| `ratchet_clip_tpu.step` | **95A TPU.** The lever's return spring — a bending leaf on the pivot post. |
| `lock_pin_tpu.step` | **95A TPU.** The mount lock. One printed part, installed at both lock sites. |

Print the two TPU parts in black — black filament is reserved for TPU in
this project, so the material is obvious in hand.

## Purchased parts

Deliberately almost nothing. The lever return springs were retired at
build #1004 in favour of the printed TPU leaf, so no springs need buying.

- **2× 608 bearing** — 8 mm bore, 22 mm OD, 7 mm wide. Any ZZ or RS
  version. One in `frame_top`'s pocket, one on the `bearing_nub`.
- **1× power spring** — the mainspring of a **Stanley PowerLock 35'** tape
  measure, pulled out of the cassette whole and installed **unmodified**,
  upside down (the Ø13.4 flange hole ends up on top). Ø78.3 flanges, Ø56.3
  body, 32.0 tall overall. Buy the tape measure — the spring is not sold
  separately, and every dimension in `src/params.py` was measured off that
  cassette. (Earlier notes said "Stanley 30-455", a 25' PowerLock; that was
  wrong.)
- **4× wood screw** — Ø4.0 shaft, one per mount arm, into the couch frame
  or the underside of the desk.
- **The cable** — per the install site, above.

## Tuning

`src/params.py` is the single source for every dimension; the part modules
read it and never restate a number. Three conventions it enforces:

- **The bead grid.** Every printed length is a whole number of 0.8 mm
  beads. `check_beads` gates it, and its exemption table names each
  constant that is legitimately off-grid (clearances, moduli, angles) with
  the reason.
- **Bridges max 0.8 mm,** one documented exception: the separator above the
  bearing.
- **One statement of one fact.** A number that can be derived is derived.
  Where a constant is retired, its history stays as prose so the reasoning
  survives without a dead name in the namespace.

Print-fit history is recorded in comments at each constant, keyed to the
build number the finding came from — those are the record of what the
printer actually did, and they are worth reading before changing a fit.
