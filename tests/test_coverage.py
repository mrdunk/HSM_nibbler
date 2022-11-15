#!/usr/bin/env python3

"""
Experimenting with CNC machining toolpaths.
Run code against .dxf test patterns.
"""

from typing import List, NamedTuple, Optional

import argparse
import os
from glob import glob
import signal
import sys
import time

import ezdxf
import matplotlib.pyplot as plt  # type: ignore
import matplotlib.patches as patches  # type: ignore
from shapely.geometry import LineString, MultiLineString, MultiPolygon, Polygon  # type: ignore
from shapely.ops import linemerge, unary_union  # type: ignore
from tabulate import tabulate

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from hsm_nibble import dxf
from hsm_nibble import geometry

break_count: int = 0
results = []


Result = NamedTuple("Result", [
    ("filename", str),
    ("overlap", float),
    ("winding", geometry.ArcDir),
    ("cut_ratio", float),
    ("crash_ratio", float),
    ("dangerous_crash_count", int),
])

Error = NamedTuple("Error", [
    ("filename", str),
    ("overlap", float),
    ("winding", geometry.ArcDir),
    ("message", str),
])


def signal_handler(_, __):
    """
    Count number of times ^C has bee pressed.
    Exit gracefully after the 1st.
    Actually break after the 2nd.
    """
    global break_count
    global results

    break_count += 1
    if break_count >= 2:
        print(tabulate(results, headers="keys"))
        sys.exit(0)
    print('Ctrl+C pressed. Finishing existing test. Ctrl+C again to quit immediately.')


def init_argparse() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        #usage="%(prog)s [OPTION] [FILE]...",
        description="A program to exercise the CAM HSM 'peeling' algorithm."
    )
    parser.add_argument(
        "-a", "--showarcs",
        action='store_true',
        help="Show the path the endmill takes.")
    parser.add_argument(
        "-f", "--tofile",
        nargs='?',
        const="/tmp/HSM",
        metavar="PATH",
        help="Save a .png of the cut area in the specified directory. "
        "If no directory specified, '/tmp/HSM/' will be used."
    )
    parser.add_argument(
        "-s", "--toscreen",
        action='store_true',
        help="Display cut area on screen")
    parser.add_argument(
        'input_paths',
        nargs='*',
        default=["./test_cases/*.dxf", "../test_cases/*.dxf"],
        metavar="INPUT_PATH",
        help="Path to source .dxf files. "
        "If not specified, will try to find the 'test_cases' directory.")
    return parser


def draw(
        combined_path: List[LineString],
        combined_rapids: List[LineString],
        pocket: Polygon,
        cut_area: Polygon,
        crash_area: Polygon,
        filename: str,
        overlap: float,
        winding: geometry.ArcDir,
        output_path: Optional[str],
        output_display: bool
) -> None:
    """Display cut path."""
    fig, ax = plt.subplots(1, 1, dpi=1600)

    def polygon(poly: Polygon, colour: str, fill: bool = True, linewidth=1) -> None:
        """Draw a Shaplely Polygon on matplotlib.pyplot. """
        if poly.type != "MultiPolygon":
            if poly.type != "Polygon":
                return
            if len(poly.exterior.coords) < 3:
                return
            for interior in poly.interiors:
                if len(interior.coords) < 3:
                    return
            poly = MultiPolygon([poly])
        for geom in poly.geoms:
            if len(geom.exterior.coords) >= 3:
                if fill:
                    patch = patches.Polygon(
                        np.array(geom.exterior.xy).T, fc=colour)
                    ax.add_patch(patch)
                else:
                    ax.plot(*geom.exterior.xy, c=colour, linewidth=linewidth)
                for interior in geom.interiors:
                    if len(interior.coords) >= 3:
                        if fill:
                            patch = patches.Polygon(
                                np.array(interior.xy).T, fc="white")
                            ax.add_patch(patch)
                        else:
                            ax.plot(*interior.xy, c=colour, linewidth=linewidth)

    polygon(cut_area, "green")
    polygon(pocket, "blue", fill=False, linewidth=0.1)
    polygon(crash_area, "red")

    for path in combined_path:
        ax.plot(*path.xy, c="black", linewidth=0.01)
    for path in combined_rapids:
        ax.plot(*path.xy, c="red", linewidth=0.03)

    ax.axis('equal')
    if output_display:
        plt.show()

    if output_path:
        try:
            os.mkdir(output_path)
        except FileExistsError:
            pass
        except OSError:
            print(f"Could not create directory: {output_path}")
            return

        new_filename = os.path.splitext(filename)[0]
        new_filename = f"{new_filename}_{overlap}_{winding.name}.png"

        plt.savefig(os.path.join(output_path, new_filename))

    plt.cla()
    plt.clf()
    plt.close(fig)
    plt.close("all")

def test_file(
        filepath: str,
        overlap: float,
        winding: geometry.ArcDir,
        show_arcs: bool,
        output_path: Optional[str],
        output_display: bool
) -> Result:
    """Run HSM algorithm on test file. Look for inconsistencies."""
    print(f"Trying: {filepath=}\t{overlap=}\t{winding=}")

    filename = filepath.split("/")[-1]

    dxf_data = ezdxf.readfile(filepath)
    modelspace = dxf_data.modelspace()

    shape = dxf.dxf_to_polygon(modelspace).geoms[-1]
    shape = shape.buffer(0)

    time_run = time.time()
    toolpath = geometry.Pocket(shape, overlap, winding, generate=False)
    time_run -= time.time()

    cut_area = Polygon()
    crash_area = Polygon()

    combined_path = []
    combined_rapids = []

    cut_area_parts = []
    crash_area_parts = []
    for element in toolpath.path:
        if type(element).__name__ == "Arc":
            new_cut = element.path.buffer(overlap / 2)
            cut_area = cut_area.union(new_cut)
            #cut_area_parts.append(new_cut)

            if show_arcs:
                combined_path.append(element.path)
    #cut_area = unary_union(cut_area_parts)

    #for element in toolpath.path:
        if type(element).__name__ == "Line":
            if element.move_style == geometry.MoveStyle.RAPID_INSIDE:
                crash = element.path.buffer(overlap / 2).difference(cut_area)
                #crash_area = crash_area.union(crash)
                crash_area_parts.append(crash)
                if show_arcs:
                    combined_path.append(element.path)
            elif show_arcs:
                combined_rapids.append(element.path)
    crash_area = unary_union(crash_area_parts)

    uncut_area = toolpath.polygon.difference(cut_area)

    cut_ratio = round(1.0 - uncut_area.area / toolpath.polygon.area, 4)
    crash_ratio = round(crash_area.area / toolpath.polygon.area, 4)

    eroded_crash_area = crash_area.buffer(-overlap / 20)
    if eroded_crash_area.type == "Polygon":
        eroded_crash_area = MultiPolygon([eroded_crash_area])
    dangerous_crash_count = len(eroded_crash_area.geoms)

    if output_path or output_display:
        draw(
            combined_path,
            combined_rapids,
            toolpath.polygon,
            cut_area,
            crash_area,
            filename,
            overlap,
            winding,
            output_path,
            output_display)

    return Result(
        filename,
        overlap,
        winding,
        cut_ratio,
        crash_ratio,
        dangerous_crash_count
    )


def main():
    """
    A program to exercise the CAM HSM 'peeling' algorithm.
    Run with --help parameter for usage info.
    """
    global results
    errors = []

    signal.signal(signal.SIGINT, signal_handler)

    parser = init_argparse()
    args = parser.parse_args()

    show_arcs = args.showarcs
    input_paths = args.input_paths
    output_path = args.tofile
    output_display = args.toscreen

    filepaths = []
    for path in input_paths:
        filepaths += glob(path)
    windings = [geometry.ArcDir.CW, geometry.ArcDir.CCW]
    #windings = [geometry.ArcDir.CW, ]
    overlaps = [0.4, 0.8, 1.6, 3.2, 6.4]
    #overlaps = [0.4, 1.6]
    count = 0
    total_count = len(filepaths) * len(windings) * len(overlaps)
    for filepath in filepaths:
        for overlap in overlaps:
            for winding in windings:
                count += 1
                try:
                    results.append(test_file(
                        filepath, overlap, winding, show_arcs, output_path, output_display))
                    print(f"{count} of {total_count}")
                    print(results[-1])
                    print()
                except Exception as error:
                    print(error)
                    print(f"during: {filepath}\t{overlap}\t{winding}")
                    errors.append(Error(filepath, overlap, winding, error))
                    #raise error

                if break_count:
                    break
            if break_count:
                break
        if break_count:
            break

    print(tabulate(results, headers="keys"))
    print(tabulate(errors, headers="keys"))
    return 0


if __name__ == "__main__":
    main()
