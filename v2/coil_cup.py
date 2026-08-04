"""COIL CUP — the stationary chamber for the AXIAL service loop (the
free-floating torsion coil that replaced the failed flat tray; see params'
AXIAL block for the physics).

One printed piece, resting on the lowered bottom plus (its floor sits
directly on the arm tops — gravity + the wall skirt's 0.15 gap above cap
it in z; the axle collar centres it; four beam-flank RIB pairs lock its
rotation against the port's cable reaction):

  · FLOOR — solid disc CUP_Z0..CH_BOT_Z, the only stationary full-radius
    plane under the rotating parts (everything rotating stays above it);
  · SLEEVE — Ø CUP_SLEEVE_OD inner guide rising to just under the coil
    ceiling: bounds the TIGHT coil's inward drift (0.5/side design gap —
    a floating coil should never actually ride it);
  · COLLAR — boss around the axle's slip bore: radial location + the
    axle's bottom-end steady now lives here too;
  · WALL — the chamber's outer bore, continuing the containment cylinder
    (WALL_IR..WALL_OR) from the plus up to 0.15 under the wall skirt;
  · RIBS — two flanks per beam azimuth hugging each beam's sides;
  · PORT — house-shaped hole near the floor at the back (CUP_PORT_AZ):
    the stationary source lead enters here (single fixed hole — design
    goal 1), threads up the chamber and the separator pass.

Prints floor-down, fully support-free: plate on the bed, vertical wall/
sleeve/collar/ribs, house-gable port ceiling.

INSTALL: slide up the axle BEFORE frame_bottom (the plus then rises under
it); pre-wind at the drive below the plus, pin, then the wall stack. The
cable threads the port before the wall goes on.
"""

import math

import cadquery as cq

from v2.dimensions import NOZZLE  # bead-multiple sizing unit
from v2.helpers import cyl, heal
from .params import (
    WALL_IR, WALL_OR, BEAM_SIZE,
    CH_BOT_Z, CH_TOP_Z, CUP_Z0, CUP_FLOOR_T,
    CUP_SLEEVE_OD, CUP_SLEEVE_T, CUP_COLLAR_T, CUP_COLLAR_H, CUP_RIB_T,
    CUP_PORT_AZ_DEG, CUP_PORT_W, CUP_PORT_SILL,
    LOOP_D_LOOSE,
    AXLE_BORE_D, WALL_ZB,
)

CUP_TOP_Z = WALL_ZB - 0.15                        # 8.65 — wall top: 0.15 under the
                                                  # wall skirt = the z cap


def _port():
    """House-shaped through hole (vertical sides + 45° gable — the gable is
    self-supporting in this upright print) at the back azimuth, sill just
    off the chamber floor. The prism is AIMED ALONG the cable's entry
    tangent (the line from outside the port grazing the LOOSE coil's
    bottom wrap) — a radial hole would clip that oblique crossing."""
    r_bot = LOOP_D_LOOSE / 2.0
    t_az = math.radians(CUP_PORT_AZ_DEG
                        - math.degrees(math.acos(r_bot / WALL_IR)))
    touch = cq.Vector(r_bot * math.cos(t_az), r_bot * math.sin(t_az), 0.0)
    n = cq.Vector(-math.sin(t_az), math.cos(t_az), 0.0)   # entry-line direction
    cross = touch + n * math.sqrt(WALL_IR ** 2 - r_bot ** 2)
    w = CUP_PORT_W
    rect_top = CUP_PORT_SILL + 8.0
    apex = rect_top + w / 2.0
    plane = cq.Plane(origin=(cross.x - n.x * 12.0, cross.y - n.y * 12.0, 0.0),
                     xDir=(-math.cos(t_az), -math.sin(t_az), 0.0),
                     normal=(n.x, n.y, 0.0))
    return (cq.Workplane(plane)
            .polyline([(-w / 2, CUP_PORT_SILL), (w / 2, CUP_PORT_SILL),
                       (w / 2, rect_top), (0.0, apex), (-w / 2, rect_top)])
            .close().extrude(24.0))


def _ribs():
    """Two CUP_RIB_T-square flank ribs per beam azimuth, full height, riding
    each beam's side faces (0.3 clear): the cup's rotation lock — and their
    tops (0.15 under the wall skirt) are the cup's z-cap. NOT at the +X
    beam: the hanging lever handles sweep to r 79.9 there (probe-caught at
    10 mm³) — three azimuths lock rotation fine."""
    out = None
    for az in (90.0, 180.0, 270.0):
        for s in (+1.0, -1.0):
            y0 = s * (BEAM_SIZE / 2.0 + 0.3)
            r = (cq.Workplane("XY").workplane(offset=CUP_Z0)
                 .polyline([(WALL_OR - 0.5, y0),
                            (WALL_OR + CUP_RIB_T, y0),
                            (WALL_OR + CUP_RIB_T, y0 + s * CUP_RIB_T),
                            (WALL_OR - 0.5, y0 + s * CUP_RIB_T)])
                 .close().extrude(CUP_TOP_Z - CUP_Z0)
                 .rotate((0, 0, 0), (0, 0, 1), az))
            out = r if out is None else out.union(r)
    return out


def _build():
    # floor plate: full disc, bed face of the print
    c = cyl(2 * WALL_OR, CUP_FLOOR_T, z=CUP_Z0)
    # chamber wall (the containment bore) up to the skirt gap
    c = c.union(cyl(2 * WALL_OR, CUP_TOP_Z - CUP_Z0, z=CUP_Z0)
                .cut(cyl(2 * WALL_IR, CUP_TOP_Z - CUP_Z0 + 1.0, z=CUP_Z0 - 0.5)))
    # inner guide sleeve to just under the coil ceiling
    c = c.union(cyl(CUP_SLEEVE_OD, (CH_TOP_Z - 1.0) - CUP_Z0, z=CUP_Z0)
                .cut(cyl(CUP_SLEEVE_OD - 2 * CUP_SLEEVE_T,
                         (CH_TOP_Z - 1.0) - CUP_Z0 + 1.0, z=CUP_Z0 - 0.5)))
    # axle collar boss above the plate
    c = c.union(cyl(AXLE_BORE_D + 2 * CUP_COLLAR_T,
                    CUP_FLOOR_T + CUP_COLLAR_H, z=CUP_Z0))
    # the axle slip bore through plate + collar
    c = c.cut(cyl(AXLE_BORE_D, CUP_FLOOR_T + CUP_COLLAR_H + 1.0, z=CUP_Z0 - 0.5))
    c = c.union(_ribs())
    c = c.cut(_port())
    return heal(c)


coil_cup = _build()
