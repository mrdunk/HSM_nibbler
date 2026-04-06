#!/usr/bin/env python3
"""
Visualise the difference between what Pocket.calculated_area_total tracks
(full circles placed by the planner) and what the output path actually cuts
(arcs and CUT lines buffered by overlap/2).

Layers drawn (bottom to top):
  orange  — calculated_area_total (what the planner thinks was cut)
  green   — actual cut area (arcs + CUT lines buffered by overlap/2)
  blue    — pocket outline
  red     — starting circle outline

Orange regions NOT covered by green reveal where the planner over-estimated
the cut area, leaving real material uncut between arc passes.

Usage:
    python3 tests/integration/show_area_diff.py test_cases/arcs.dxf --outdir /tmp/HSM
    python3 tests/integration/show_area_diff.py test_cases/*.dxf --overlap 1.6 --winding CW --outdir /tmp/HSM
"""

import argparse
import os
import sys
from glob import glob

import ezdxf
import matplotlib.patches as patches  # type: ignore
import matplotlib.pyplot as plt  # type: ignore
import numpy as np
from shapely.geometry import MultiPolygon, Polygon  # type: ignore
from shapely.ops import unary_union  # type: ignore

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from hsm_nibble import dxf, geometry
from hsm_nibble.arc_utils import ArcData, LineData


def _draw_polygon(ax, poly, facecolor, edgecolor=None, alpha=1.0, linewidth=0.5, fill=True):
    if poly is None or poly.is_empty:
        return
    if poly.geom_type == "Polygon":
        poly = MultiPolygon([poly])
    if poly.geom_type != "MultiPolygon":
        return
    for geom in poly.geoms:
        if len(geom.exterior.coords) < 3:
            continue
        if fill:
            patch = patches.Polygon(
                np.array(geom.exterior.xy).T,
                facecolor=facecolor, edgecolor=edgecolor or facecolor,
                alpha=alpha, linewidth=linewidth)
            ax.add_patch(patch)
            for interior in geom.interiors:
                hole = patches.Polygon(
                    np.array(interior.xy).T, facecolor="white", edgecolor="white",
                    linewidth=linewidth)
                ax.add_patch(hole)
        else:
            ax.plot(*geom.exterior.xy, c=edgecolor or facecolor, linewidth=linewidth)
            for interior in geom.interiors:
                ax.plot(*interior.xy, c=edgecolor or facecolor, linewidth=linewidth)


def process(filepath, overlap, winding, outdir):
    filename = os.path.basename(filepath)
    print(f"  {filename}  overlap={overlap}  winding={winding.name}")

    dxf_data = ezdxf.readfile(filepath)
    modelspace = dxf_data.modelspace()
    shape = dxf.dxf_to_polygon(modelspace).geoms[-1].buffer(0)

    toolpath = geometry.Pocket(shape, overlap, winding, generate=False, debug=False)

    # Compute what the path actually cuts; collect arc/CUT paths for drawing.
    actual_cut = Polygon()
    cut_paths = []
    for element in toolpath.path:
        if isinstance(element, ArcData):
            actual_cut = actual_cut.union(element.path.buffer(overlap / 2))
            cut_paths.append(element.path)
        elif isinstance(element, LineData) and element.move_style == geometry.MoveStyle.CUT:
            actual_cut = actual_cut.union(element.path.buffer(overlap / 2))
            cut_paths.append(element.path)

    planned_cut = toolpath.calculated_area_total

    # Starting circle.
    start_circle = toolpath.voronoi.start_point.buffer(toolpath.start_radius)

    fig, ax = plt.subplots(1, 1, dpi=300)

    # Layer 1: planned area (orange) — what the planner tracked.
    _draw_polygon(ax, planned_cut, facecolor="orange", alpha=0.6)

    # Layer 2: actual cut area (green) — what the path really cuts.
    _draw_polygon(ax, actual_cut, facecolor="green", alpha=0.6)

    # Layer 3: pocket outline (blue, no fill).
    _draw_polygon(ax, toolpath.polygon, facecolor=None, edgecolor="blue",
                  linewidth=0.3, fill=False)

    # Layer 4: starting circle outline (red, no fill).
    _draw_polygon(ax, start_circle, facecolor=None, edgecolor="red",
                  linewidth=0.5, fill=False)

    # Layer 5: arc and CUT paths (black lines).
    for path in cut_paths:
        ax.plot(*path.xy, c="black", linewidth=0.15)

    ax.axis("equal")
    ax.set_title(
        f"{filename}  overlap={overlap}  {winding.name}\n"
        "orange=planned  green=actual  red=start circle",
        fontsize=4)

    os.makedirs(outdir, exist_ok=True)
    stem = os.path.splitext(filename)[0]
    out = os.path.join(outdir, f"{stem}_{overlap}_{winding.name}_area_diff.png")
    plt.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print(f"  saved: {out}")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input_paths", nargs="+", metavar="DXF",
                        help="DXF file(s) or glob patterns")
    parser.add_argument("--overlap", type=float, default=1.6,
                        help="Step/overlap distance (default 1.6)")
    parser.add_argument("--winding", choices=["CW", "CCW", "CLOSEST"], default="CW",
                        help="Winding direction (default CW)")
    parser.add_argument("--outdir", required=True, metavar="DIR",
                        help="Directory to write PNG images into")
    args = parser.parse_args()

    winding = geometry.ArcDir[args.winding]

    filepaths = []
    for pat in args.input_paths:
        filepaths.extend(glob(pat))
    filepaths = sorted(set(filepaths))

    if not filepaths:
        print("No DXF files found.")
        sys.exit(1)

    for fp in filepaths:
        try:
            process(fp, args.overlap, winding, args.outdir)
        except Exception as exc:
            print(f"  ERROR {fp}: {exc}")
            raise


if __name__ == "__main__":
    main()
