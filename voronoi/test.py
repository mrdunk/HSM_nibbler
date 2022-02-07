#!/usr/bin/env python3
""" Experimenting with CNC machining toolpaths. """

from typing import NamedTuple

from glob import glob
import sys

import ezdxf

import dxf
import geometry

Result = NamedTuple("Result", [
    ("filename", str),
    ("overlap", float),
    ("winding", geometry.ArcDir),
    ("nominal_uncut_area", float),
    ("uncut_area", float),
    ("uncut_ratio", float)
    ])

def test_file(filepath: str, overlap: float, winding: geometry.ArcDir) -> Result:
    print(f"{filepath=}\t{overlap=}\t{winding=}")

    dxf_data = ezdxf.readfile(filepath)
    modelspace = dxf_data.modelspace()
    shape = dxf.dxf_to_polygon(modelspace)
    toolpath = geometry.ToolPath(shape, overlap, winding)

    polygon_remaining = toolpath.polygon.difference(toolpath.cut_area_total)
    polygon_erroded = polygon_remaining.buffer(-overlap)

    nominal_uncut_area = round(polygon_remaining.area, 2)
    uncut_area = round(polygon_erroded.area, 2)
    polygon_area = round(toolpath.polygon.area, 2)

    return Result(
            filepath,
            overlap,
            winding,
            nominal_uncut_area,
            uncut_area,
            uncut_area/polygon_area
            ) 

def main(argv):
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
                    print()
                    print(f"{count} of {total_count}")
                    print(results[-1])
                    print()
                except Exception as error:
                    print("*****")
                    print(error)
                    print("*****")
    for result in results:
        print(result)

if __name__ == "__main__":
    main(sys.argv)

