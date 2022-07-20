#!/usr/bin/env python3
"""
Experimenting with CNC machining toolpaths.
This program is a demo which uses the main library file on test .dxf CAD files.
"""

from typing import List, Tuple

import sys
import math
import ezdxf
import matplotlib.pyplot as plt    # type: ignore
from shapely.geometry import Point
import dxf
import geometry


def display_outline(shape, colour="blue"):
    """ Display the outline of the shape to be cut. """
    x, y = shape.exterior.xy
    plt.plot(x, y, c=colour, linewidth=2)

    for interior in shape.interiors:
        x, y = interior.xy
        plt.plot(x, y, c=colour, linewidth=2)

def display_voronoi(toolpath, colour="red"):
    """ Display the voronoi edges. These are equidistant from the shape's edges. """
    for edge in toolpath.voronoi.edges.values():
        x = []
        y = []
        for point in edge.coords:
            x.append(point[0])
            y.append(point[1])
        plt.plot(x, y, c="red", linewidth=4)
        plt.plot(x[0], y[0], 'x', c=colour)
        plt.plot(x[-1], y[-1], 'x', c=colour)

def display_visited_voronoi_edges(toolpath, colour="black"):
    """ 
    Display the voronoi edges that were used to calculate cut geometry.
    This should match the output of display_voronoi(...).
    """
    for edge in toolpath.visited_edges:
        edge_coords = toolpath.voronoi.edges[edge].coords
        x = []
        y = []
        for point in edge_coords:
            x.append(point[0])
            y.append(point[1])
        plt.plot(x, y, c=colour, linewidth=1)
        plt.plot(x[0], y[0], 'x', c=colour)
        plt.plot(x[-1], y[-1], 'x', c=colour)

def display_toolpath(toolpath, cut_colour="green", rapid_inside_colour="blue", rapid_outside_colour="orange"):
    # Display path.
    for element in toolpath.path:
        if type(element).__name__ == "Arc":
            x, y = element.path.xy
            if element.debug:
                plt.plot(x, y, c=element.debug, linewidth=3)
            else:
                plt.plot(x, y, c=cut_colour, linewidth=1)

        elif type(element).__name__ == "Line":
            x, y = element.path.xy
            if element.move_style == geometry.MoveStyle.RAPID_INSIDE:
                plt.plot(x, y, linestyle='--', c=rapid_inside_colour, linewidth=1)
            elif element.move_style == geometry.MoveStyle.RAPID_OUTSIDE:
                plt.plot(x, y, c=rapid_outside_colour, linewidth=1)
            else:
                assert element.move_style == geometry.MoveStyle.CUT
                plt.plot(x, y, linestyle='--', c=cut_colour, linewidth=1)


def generate_tool_path(shape, step_size, inner=True):
    """ Calculate the toolpath. """

    if inner:
        previous_cut = shape.buffer(-1)

        toolpath = geometry.RefineInnerPocket(
                shape,
                previous_cut,
                step_size,
                geometry.ArcDir.Closest,
                generate=True,
                #starting_point=Point(30, 30),
                debug=True)
    else:
        previous_cut = shape.buffer(10).difference(shape.buffer(5))

        toolpath = geometry.RefineOuter(
                shape,
                previous_cut,
                step_size,
                geometry.ArcDir.Closest,
                generate=True,
                debug=True)

    display_outline(previous_cut)

    timeslice = 100  # ms
    for index, progress in enumerate(toolpath.get_arcs(timeslice)):
        print(index, progress)
        # toolpath.path contains the currently generated path data at this point.
    return toolpath

def main(argv):
    """
    Example program making use of HSM "peeling" CAM.
    """
    if len(argv) < 2:
        print("Incorrect command line arguments.")
        print(f"Use:\n   {argv[0]} FILENAME [STEP_SIZE]")
        sys.exit(0)
    filename = argv[1]

    try:
        dxf_data = ezdxf.readfile(filename)
    except IOError:
        print(f'Not a DXF file or a generic I/O error.')
        sys.exit(2)
    except ezdxf.DXFStructureError:
        print(f'Invalid or corrupted DXF file.')
        sys.exit(3)

    if len(argv) > 2:
        step_size = float(argv[2])
    else:
        step_size = 1

    print(f"filename: {filename}\n step_size: {step_size}\n")

    modelspace = dxf_data.modelspace()
    shape = dxf.dxf_to_polygon(modelspace).geoms[-1]

    #toolpath = generate_tool_path(shape, step_size, inner=False)
    toolpath = generate_tool_path(shape, step_size, inner=True)

    display_outline(shape)
    display_toolpath(toolpath)
    display_voronoi(toolpath)
    # display_visited_voronoi_edges(toolpath)

    plt.gca().set_aspect('equal')
    plt.show()

if __name__ == "__main__":
    main(sys.argv)
