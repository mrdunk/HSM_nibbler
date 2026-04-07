#!/usr/bin/env python3
"""
Visualise the difference between what Pocket.calculated_area_total tracks
(full circles placed by the planner) and what the output path actually cuts
(arcs and CUT lines buffered by overlap/2).

Layers drawn (bottom to top):
  orange  — calculated_area_total (what the planner thinks was cut)
  green   — actual cut area (arcs + CUT lines buffered by overlap/2)
  red     — planned but not cut (orange minus green): gaps where the planner
            over-estimated coverage, leaving real material uncut
  blue    — pocket outline
  purple  — starting circle outline (outline only)
  black   — arc and CUT paths
  cyan    — rapid moves (RAPID_INSIDE and RAPID_OUTSIDE)

Usage:
    python3 tests/integration/show_area_diff.py test_cases/arcs.dxf --outdir /tmp/HSM
    python3 tests/integration/show_area_diff.py test_cases/arcs.dxf --interactive
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

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from hsm_nibble import dxf, geometry
from hsm_nibble.arc_utils import ArcData, LineData


def _draw_polygon(ax, poly, facecolor, edgecolor=None, alpha=1.0, linewidth=0.5, fill=True):
    """Draw a Shapely Polygon/MultiPolygon onto ax. Returns list of added artists."""
    artists = []
    if poly is None or poly.is_empty:
        return artists
    if poly.geom_type == "Polygon":
        poly = MultiPolygon([poly])
    if poly.geom_type != "MultiPolygon":
        return artists
    for geom in poly.geoms:
        if len(geom.exterior.coords) < 3:
            continue
        if fill:
            patch = patches.Polygon(
                np.array(geom.exterior.xy).T,
                facecolor=facecolor, edgecolor=edgecolor or facecolor,
                alpha=alpha, linewidth=linewidth)
            ax.add_patch(patch)
            artists.append(patch)
            for interior in geom.interiors:
                hole = patches.Polygon(
                    np.array(interior.xy).T, facecolor="white", edgecolor="white",
                    linewidth=linewidth)
                ax.add_patch(hole)
                artists.append(hole)
        else:
            line, = ax.plot(*geom.exterior.xy, c=edgecolor or facecolor, linewidth=linewidth)
            artists.append(line)
            for interior in geom.interiors:
                line, = ax.plot(*interior.xy, c=edgecolor or facecolor, linewidth=linewidth)
                artists.append(line)
    return artists


def build_figure(filepath, overlap, winding):
    """Compute toolpath and build a matplotlib figure. Returns (fig, layer_map)."""
    filename = os.path.basename(filepath)
    print(f"  {filename}  overlap={overlap}  winding={winding.name}")

    dxf_data = ezdxf.readfile(filepath)
    modelspace = dxf_data.modelspace()
    shape = dxf.dxf_to_polygon(modelspace).geoms[-1].buffer(0)

    toolpath = geometry.Pocket(shape, overlap, winding, generate=False, debug=False)

    # Compute what the path actually cuts; collect arc/CUT and rapid paths for drawing.
    actual_cut = Polygon()
    cut_paths = []
    rapid_paths = []
    for element in toolpath.path:
        if isinstance(element, ArcData):
            actual_cut = actual_cut.union(element.path.buffer(overlap / 2))
            cut_paths.append(element.path)
        elif isinstance(element, LineData) and element.move_style == geometry.MoveStyle.CUT:
            actual_cut = actual_cut.union(element.path.buffer(overlap / 2))
            cut_paths.append(element.path)
        elif isinstance(element, LineData):
            rapid_paths.append(element.path)

    planned_cut = toolpath.path_planner.calculated_area_total
    start_circle = toolpath.voronoi.start_point.buffer(toolpath.start_radius)

    fig, ax = plt.subplots(1, 1, dpi=150)

    layer_artists = {}

    layer_artists["orange: planned"] = _draw_polygon(
        ax, planned_cut, facecolor="orange", alpha=0.6)

    layer_artists["green: actual cut"] = _draw_polygon(
        ax, actual_cut, facecolor="green", alpha=0.6)

    gap = planned_cut.difference(actual_cut)
    layer_artists["red: planned but not cut"] = _draw_polygon(
        ax, gap, facecolor="red", alpha=0.8)

    layer_artists["blue: pocket outline"] = _draw_polygon(
        ax, toolpath.polygon, facecolor=None, edgecolor="blue", linewidth=0.3, fill=False)

    layer_artists["purple: start circle"] = _draw_polygon(
        ax, start_circle, facecolor=None, edgecolor="purple", linewidth=0.5, fill=False)

    arc_lines = []
    for path in cut_paths:
        line, = ax.plot(*path.xy, c="black", linewidth=0.15)
        arc_lines.append(line)
    layer_artists["black: arcs/cuts"] = arc_lines

    rapid_lines = []
    for path in rapid_paths:
        line, = ax.plot(*path.xy, c="cyan", linewidth=0.2, linestyle="--")
        rapid_lines.append(line)
    layer_artists["cyan: rapids"] = rapid_lines

    ax.axis("equal")
    ax.set_title(f"{filename}  overlap={overlap}  {winding.name}", fontsize=6)

    return fig, ax, layer_artists, filename


def show_interactive(filepath, overlap, winding):
    """Open an interactive window with toggleable layers."""
    fig, ax, layer_artists, _ = build_figure(filepath, overlap, winding)

    # Build legend with one proxy per layer; map legend line → artist list.
    proxy_lines = []
    legend_to_artists = {}
    for label, artists in layer_artists.items():
        if not artists:
            continue
        colour = label.split(":")[0].strip()
        proxy, = ax.plot([], [], color=colour, label=label, linewidth=4)
        proxy_lines.append(proxy)
        legend_to_artists[proxy] = artists

    leg = ax.legend(loc="upper right", fontsize=6, framealpha=0.8)
    leg.set_draggable(True)

    # Make legend entries pickable and thicker so they're easy to click.
    for legline in leg.get_lines():
        legline.set_picker(8)
        legline.set_linewidth(6)

    # Map each legend line back to the matching proxy, then to the artists.
    legline_to_artists = {}
    for legline, proxy in zip(leg.get_lines(), proxy_lines):
        legline_to_artists[legline] = legend_to_artists[proxy]

    def on_pick(event):
        artists = legline_to_artists.get(event.artist)
        if artists is None:
            return
        visible = not artists[0].get_visible()
        for artist in artists:
            artist.set_visible(visible)
        event.artist.set_alpha(1.0 if visible else 0.3)
        fig.canvas.draw()

    fig.canvas.mpl_connect("pick_event", on_pick)
    plt.show()


def save_to_file(filepath, overlap, winding, outdir):
    """Render and save a PNG."""
    fig, ax, layer_artists, filename = build_figure(filepath, overlap, winding)

    ax.set_title(
        f"{filename}  overlap={overlap}  {winding.name}\n"
        "orange=planned  green=actual  red=gap  purple=start circle  cyan=rapids",
        fontsize=4)

    os.makedirs(outdir, exist_ok=True)
    stem = os.path.splitext(filename)[0]
    out = os.path.join(outdir, f"{stem}_{overlap}_{winding.name}_area_diff.png")
    plt.savefig(out, bbox_inches="tight", dpi=300)
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

    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--outdir", metavar="DIR",
                      help="Save PNG images to this directory")
    mode.add_argument("--interactive", action="store_true",
                      help="Open an interactive window with toggleable layers "
                           "(only the first matched file is shown)")

    args = parser.parse_args()
    winding = geometry.ArcDir[args.winding]

    filepaths = sorted(set(fp for pat in args.input_paths for fp in glob(pat)))
    if not filepaths:
        print("No DXF files found.")
        sys.exit(1)

    if args.interactive:
        show_interactive(filepaths[0], args.overlap, winding)
    else:
        for fp in filepaths:
            try:
                save_to_file(fp, args.overlap, winding, args.outdir)
            except Exception as exc:
                print(f"  ERROR {fp}: {exc}")
                raise


if __name__ == "__main__":
    main()
