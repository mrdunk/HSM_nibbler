#!/usr/bin/env python3

"""
Experimenting with CNC machining toolpaths.
Run code against .dxf test patterns.
"""

from typing import NamedTuple, Optional

import argparse
import os
from glob import glob
import signal
import sys
import time

import ezdxf
import numpy as np  # type: ignore
import matplotlib.pyplot as plt  # type: ignore
import matplotlib.patches as patches  # type: ignore
from shapely.geometry import MultiPolygon, Polygon  # type: ignore
from tabulate import tabulate

import dxf
import geometry

break_count: int = 0

Result = NamedTuple("Result", [
    ("filename", str),
    ("overlap", float),
    ("winding", geometry.ArcDir),
    ("cut_ratio", float),
    ("crash_ratio", float),
])


def signal_handler(_, __):
    """
    Count number of times ^C has bee pressed.
    Exit gracefully after the 1st.
    Actually break after the 2nd.
    """
    global break_count
    break_count += 1
    if break_count >= 2:
        sys.exit(0)
    print('Ctrl+C pressed. Finishing existing test. Ctrl+C again to quit immediately.')


def init_argparse() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        #usage="%(prog)s [OPTION] [FILE]...",
        description="A program to exercise the CAM HSM 'peeling' algorithm."
    )
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
    _, ax = plt.subplots(1, 1, dpi=800)

    def polygon(poly: Polygon, colour: str, fill: bool = True) -> None:
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
                    ax.plot(*geom.exterior.xy, c=colour)
                for interior in geom.interiors:
                    if len(interior.coords) >= 3:
                        if fill:
                            patch = patches.Polygon(
                                np.array(interior.xy).T, fc="white")
                            ax.add_patch(patch)
                        else:
                            ax.plot(*interior.xy, c=colour)

    polygon(cut_area, "green")
    polygon(crash_area, "red")
    polygon(pocket, "blue", fill=False)

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


def test_file(
        filepath: str,
        overlap: float,
        winding: geometry.ArcDir,
        output_path: Optional[str],
        output_display: bool
) -> Result:
    """Run HSM algorithm on test file. Look for inconsistencies."""
    print(f"Trying: {filepath=}\t{overlap=}\t{winding=}")

    filename = filepath.split("/")[-1]

    dxf_data = ezdxf.readfile(filepath)
    modelspace = dxf_data.modelspace()
    shape = dxf.dxf_to_polygon(modelspace)

    time_run = time.time()
    toolpath = geometry.ToolPath(shape, overlap, winding, generate=False)
    time_run -= time.time()

    cut_area = toolpath.start_point.buffer(toolpath.start_radius + overlap / 2)
    crash_area = Polygon()

    for element in toolpath.path:
        if type(element).__name__ == "Arc":
            cut_area = cut_area.union(element.path.buffer(overlap / 2))
        elif type(element).__name__ == "Line":
            if element.safe:
                crash = element.path.buffer(overlap / 2).difference(cut_area)
                crash_area = crash_area.union(crash)

    uncut_area = toolpath.polygon.difference(cut_area)

    cut_ratio = round(uncut_area.area / toolpath.polygon.area, 4)
    crash_ratio = round(crash_area.area / toolpath.polygon.area, 4)

    if output_path or output_display:
        draw(
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
        crash_ratio
    )


def main():
    """
    A program to exercise the CAM HSM 'peeling' algorithm.
    Run with --help parameter for usage info.
    """
    signal.signal(signal.SIGINT, signal_handler)

    parser = init_argparse()
    args = parser.parse_args()

    input_paths = args.input_paths
    output_path = args.tofile
    output_display = args.toscreen

    results = []

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
                    results.append(
                        test_file(filepath, overlap, winding, output_path, output_display))
                    print(f"{count} of {total_count}")
                    print(results[-1])
                    print()
                except Exception as error:
                    print(error)
                    print(f"during: {filepath}\t{overlap}\t{winding}")
                    raise error

                if break_count:
                    break
            if break_count:
                break
        if break_count:
            break

    print(tabulate(results, headers="keys"))
    return 0


if __name__ == "__main__":
    main()
