"""Tessellate a STEP file and report mesh-level edge health.

Reports counts mirroring what slicers flag:
  - non-manifold edges: shared by > 2 triangles (the bad ones)
  - boundary (open) edges: shared by exactly 1 triangle (holes in the mesh)
  - manifold edges: shared by exactly 2 triangles (the good ones)

Tessellation tolerance can be tightened/loosened with --linear / --angular;
absolute counts shift with tolerance, but the RELATIVE trend across builds
(before/after a code change) is the useful metric.

Usage:
  py -3.12 tools/count_nonmanifold.py housing.step
  py -3.12 tools/count_nonmanifold.py *.step
"""

from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path

import cadquery as cq
import trimesh


def _tessellate_to_trimesh(step_path: Path, linear: float, angular: float) -> trimesh.Trimesh:
    wp = cq.importers.importStep(str(step_path))
    shape = wp.val()
    # cadquery's Shape.tessellate returns (verts, faces) — already at the
    # requested mesh tolerances.
    verts, faces = shape.tessellate(linear, angular)
    return trimesh.Trimesh(
        vertices=[(v.x, v.y, v.z) for v in verts],
        faces=faces,
        process=True,  # merge duplicate vertices so edge counts are accurate
    )


def _edge_health(mesh: trimesh.Trimesh) -> tuple[int, int, int]:
    """Return (n_manifold, n_boundary, n_nonmanifold)."""
    # Build undirected edge keys (sorted vertex index pairs), one per face-edge.
    edges = mesh.edges_sorted
    counts = Counter(map(tuple, edges))
    manifold = boundary = nonmanifold = 0
    for c in counts.values():
        if c == 2:
            manifold += 1
        elif c == 1:
            boundary += 1
        else:
            nonmanifold += 1
    return manifold, boundary, nonmanifold


def main() -> None:
    p = argparse.ArgumentParser(prog="count_nonmanifold", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("paths", nargs="+", help="STEP file(s) to analyze")
    p.add_argument("--linear", type=float, default=0.01,
                   help="Linear tessellation tolerance in mm (default 0.01)")
    p.add_argument("--angular", type=float, default=0.1,
                   help="Angular tessellation tolerance in rad (default 0.1)")
    args = p.parse_args()

    print(f"{'file':<30}  {'manifold':>10}  {'boundary':>9}  {'non-manif':>10}")
    print("-" * 65)
    worst = 0
    for raw in args.paths:
        path = Path(raw)
        if not path.exists():
            print(f"{path.name:<30}  (not found)", file=sys.stderr)
            continue
        mesh = _tessellate_to_trimesh(path, args.linear, args.angular)
        m, b, nm = _edge_health(mesh)
        worst = max(worst, nm)
        flag = "  *" if nm else ""
        print(f"{path.name:<30}  {m:>10}  {b:>9}  {nm:>10}{flag}")
    if worst:
        sys.exit(1)


if __name__ == "__main__":
    main()
