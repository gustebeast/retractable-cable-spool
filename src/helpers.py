"""Geometric helpers (carried over from v2 — pure functions, no state)."""

from __future__ import annotations

import math

import cadquery as cq


def chord_x(r: float, y: float) -> float:
    """x of a circle's chord at height |y| — where a ring of radius `r`
    (about the z axis) sits in x at a given y. The guarded form of
    sqrt(r² − y²): 0 past the rim, so callers can subtract a weld/bury
    margin without a domain error. One home for the formula the band,
    horn and lever code all need (the #890 membrane-lesson math)."""
    return math.sqrt(max(r * r - y * y, 0.0))


def chevron_plan(x0: float, x1: float, xa: float, y0: float, y1: float,
                 inset: float = 0.0) -> tuple:
    """Plan outline of a slot whose +x end closes as a CHEVRON: the
    rectangle [x0,x1]x[y0,y1] with a nose running out to the apex `xa`
    on the centreline.

    WHY a nose: a slot cut through a part that the printer builds as a
    45 deg front sweeping +x (anything corbelled off a wall) has its
    far wall appear all at once — one layer bridging the slot's whole
    width, drooping into the bore. A chevron makes that closure
    DIAGONAL instead, so each layer adds only a sliver.

    `inset` shrinks the outline uniformly (every edge moves in by that
    much, the apex by inset / sin of its half-angle) — the printed pin
    is its own slot inset by the fit clearance."""
    yc = (y0 + y1) / 2.0
    dx, dy = xa - x1, (y1 - y0) / 2.0
    L = math.hypot(dx, dy)
    xk = x1 - inset * (L - dx) / dy          # the nose's base corners
    xp = xa - inset * L / dy                 # the apex itself
    return ((x0 + inset, y0 + inset), (xk, y0 + inset), (xp, yc),
            (xk, y1 - inset), (x0 + inset, y1 - inset))


def cyl(d: float, h: float, z: float = 0.0) -> cq.Workplane:
    """Solid cylinder, diameter d, height h, base at z."""
    return cq.Workplane("XY").workplane(offset=z).circle(d / 2).extrude(h)


def cone_solid(d_bottom: float, d_top: float, h: float, z_base: float) -> cq.Workplane:
    """Solid truncated cone: d_bottom at z_base, d_top at z_base+h (filled from axis)."""
    return (
        cq.Workplane("XY").workplane(offset=z_base)
        .circle(d_bottom / 2)
        .workplane(offset=h)
        .circle(d_top / 2)
        .loft()
    )


def drop_stray_shells(wp: cq.Workplane) -> cq.Workplane:
    """Rebuild any solid that carries stranded OPEN shells. A boolean fuse
    occasionally leaves an orphan internal FACE inside the body as a
    second, non-closed shell — OCC still reports the solid valid and
    heal() keeps it, but slicers flag it ('the following shells are not
    closed', Bambu on axle_separator, build #871: a 1-face shell at one
    lid-bayonet tenon's fuse seam). Keeps the OUTER shell and any CLOSED
    inner shells (real voids); drops open ones. The overlap gate audits
    every component for this."""
    from OCP.BRep import BRep_Tool
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid
    from OCP.BRepClass3d import BRepClass3d
    from OCP.TopAbs import TopAbs_SHELL
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS

    def _n_open(shape):
        n, ex = 0, TopExp_Explorer(shape, TopAbs_SHELL)
        while ex.More():
            if not BRep_Tool.IsClosed_s(TopoDS.Shell_s(ex.Current())):
                n += 1
            ex.Next()
        return n

    # audit the WHOLE shape: the stray may be INSIDE a solid, or a loose
    # open shell riding beside it in a compound (the #871 case)
    if _n_open(wp.val().wrapped) == 0:
        return wp
    rebuilt = []
    for sol in wp.solids().vals():
        closed = []
        ex = TopExp_Explorer(sol.wrapped, TopAbs_SHELL)
        while ex.More():
            sh = TopoDS.Shell_s(ex.Current())
            if BRep_Tool.IsClosed_s(sh):
                closed.append(sh)
            ex.Next()
        if _n_open(sol.wrapped) == 0:
            rebuilt.append(sol)
            continue
        outer = BRepClass3d.OuterShell_s(TopoDS.Solid_s(sol.wrapped))
        mk = BRepBuilderAPI_MakeSolid(outer)
        for sh in closed:
            if not sh.IsSame(outer):
                mk.Add(sh)
        rebuilt.append(cq.Solid(mk.Solid()))
    # rebuild from the cleaned SOLIDS ONLY — loose shells/faces are dropped
    out = cq.Workplane("XY")
    for s in rebuilt:
        out = out.add(s)
    return out


def heal(wp: cq.Workplane) -> cq.Workplane:
    """Run OCCT's ShapeFix + ShapeUpgrade on a Workplane's underlying shape to
    clean up minor face/edge tolerance issues and merge same-domain adjacent
    faces before STEP export (see v2/helpers.py for the full rationale)."""
    from OCP.ShapeFix import ShapeFix_Shape  # type: ignore[import]
    from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain  # type: ignore[import]
    from OCP.TopAbs import TopAbs_COMPOUND
    shape = wp.val().wrapped
    fixer = ShapeFix_Shape(shape)
    fixer.SetPrecision(1e-4)
    fixer.SetMaxTolerance(1e-3)
    fixer.Perform()
    fixed = fixer.Shape()
    try:
        unifier = ShapeUpgrade_UnifySameDomain(fixed, True, True, True)
        unifier.Build()
        unified = unifier.Shape()
    except Exception:
        unified = fixed
    # DEFENSIVE (v3 print finding, build #827): on a heavily-perforated
    # solid the fix/unify pass can silently DEGRADE the result to a bare
    # closed SHELL (0 solids, volume still reads) — never return a shape
    # with fewer solids than came in.
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer

    def _n_solids(s):
        if s.ShapeType() == TopAbs_SOLID:
            return 1
        ex, n = TopExp_Explorer(s, TopAbs_SOLID), 0
        while ex.More():
            n += 1
            ex.Next()
        return n

    n_in = _n_solids(shape)
    for candidate in (unified, fixed, shape):
        if _n_solids(candidate) >= min(n_in, 1):
            break
    if candidate.ShapeType() == TopAbs_COMPOUND:
        wrapped = cq.Compound(candidate)
    else:
        wrapped = cq.Solid(candidate)
    return cq.Workplane("XY").add(wrapped)
