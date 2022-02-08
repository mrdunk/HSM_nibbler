#!/usr/bin/env python3
"""
Experimenting with CNC machining toolpaths.
Run code against .dxf test patterns.
"""

from typing import NamedTuple, Tuple

from glob import glob
import signal
import sys
from tabulate import tabulate
import time

import ezdxf

import dxf
import geometry

break_count: int = 0

Result = NamedTuple("Result", [
    ("filename", str),
    ("overlap", float),
    ("winding", geometry.ArcDir),
    ("uncut_area", float),
    ("uncut_ratio", float),
    ("arc_count", int),
    ("arc_attempt_ratio", float),
    ("arc_fail_ratio", float),
    ("path_fail_ratio", float),
    ("time_per_arc", float),
    ("worst_undersize_arc", Tuple[float, float]),
    ("worst_oversize_arc", Tuple[float, float]),
    ])

def signal_handler(sig, frame):
    global break_count
    break_count += 1
    if break_count >= 2:
        sys.exit(0)
    print('Ctrl+C pressed. Finishing existing test. Ctrl+C again to quit immediately.')

def describe():
    print("uncut_area:")
    print("\tThe area enclosed by the drawn path is subtracted from the "
            "desired geometry shape. The remainder is then eroded by the overlap / 2. "
            "(Lower is better)")
    print("uncut_ratio:")
    print("\tThe uncut_area divided by the number of arcs to give a "
            "normalized ratio. (Lower is better)")
    print("arc_count:")
    print("\tThe number of arcs in the calculated path.")
    print("arc_attempt_ratio:")
    print("\tThe average number of arc calculations before arriving "
            "as an arc that fits. (Lower is better.")
    print("arc_fail_ratio:")
    print("\tThe ratio of failed attempts to find an exact fit arc. "
            "(Lower is better)")
    print("path_fail_ratio:")
    print("\tThe number of failures to draw any more arcs along a "
            "voronoi edge. Any number over 0 should be investigated. (Lower is better)")
    print("time_per_arc:")
    print("\tThe average time taken to calculate an arc position. "
            "(Lower is better)")
    print("worst_undersize_arc:")
    print("\tThe worst fitting arc smaller than desired. "
            "Displayed in the form: (actual, desired). Not particularly serious "
            "if arc_fail_ratio is low. "
            "(None or less difference between is better)")
    print("worst_oversize_arc:")
    print("\tThe worst fitting arc larger than desired. "
            "Displayed in the form: (actual, desired). "
            "Investigate any occurrence of more than a few percent. "
            "(None or less difference between is better)")
    print()

def test_file(filepath: str, overlap: float, winding: geometry.ArcDir) -> Result:
    print(f"Trying: {filepath=}\t{overlap=}\t{winding=}")

    filename = filepath.split("/")[-1]

    dxf_data = ezdxf.readfile(filepath)
    modelspace = dxf_data.modelspace()
    shape = dxf.dxf_to_polygon(modelspace)
    
    time_run = time.time()
    toolpath = geometry.ToolPath(shape, overlap, winding)
    time_run -= time.time()

    polygon_remaining = toolpath.polygon.difference(toolpath.cut_area_total)
    polygon_erroded = polygon_remaining.buffer(-overlap / 2)

    polygon_area = round(toolpath.polygon.area, 4)
    uncut_area = round(polygon_erroded.area, 4)
    uncut_ratio = round(uncut_area/polygon_area, 4)

    arc_count = len(toolpath.path_data)
    arc_attempt_ratio = round(toolpath.loop_count / arc_count, 4)
    arc_fail_ratio = round(toolpath.arc_fail_count / arc_count, 4)
    path_fail_ratio = round(toolpath.path_fail_count / arc_count, 4)
    time_per_arc = round(time_run / arc_count, 4)
    if toolpath.worst_undersize_arc is None:
        worst_undersize_arc = None
    else:
        worst_undersize_arc = (
                round(toolpath.worst_undersize_arc[0], 2), toolpath.worst_undersize_arc[1])
    if toolpath.worst_oversize_arc is None:
        worst_oversize_arc = None
    else:
        worst_oversize_arc = (
                round(toolpath.worst_oversize_arc[0], 2), toolpath.worst_oversize_arc[1])

    return Result(
            filename,
            overlap,
            winding,
            uncut_area,
            uncut_ratio,
            arc_count,
            arc_attempt_ratio,
            arc_fail_ratio,
            path_fail_ratio,
            time_per_arc,
            worst_undersize_arc,
            worst_oversize_arc
            ) 

def main(argv):
    signal.signal(signal.SIGINT, signal_handler)

    path = "../**/*.dxf"
    if len(argv) >= 2:
        path = argv[1]

    results = []

    filepaths = glob(path)
    #windings = [geometry.ArcDir.CW, geometry.ArcDir.CCW]
    windings = [geometry.ArcDir.CW,]
    #overlaps = [0.4, 0.8, 1.6, 3.2, 6.4]
    overlaps = [0.4, 1.6, 6.4]
    count = 0
    total_count = len(filepaths) * len(windings) * len(overlaps)
    for filepath in filepaths:
        for overlap in overlaps:
            for winding in windings:
                count += 1
                try:
                    results.append(test_file(filepath, overlap, winding))
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
    describe()
    print(tabulate(results, headers="keys"))

if __name__ == "__main__":
    main(sys.argv)

