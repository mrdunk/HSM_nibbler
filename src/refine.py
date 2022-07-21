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
from shapely.geometry import MultiPolygon, Point, Polygon
import dxf
import geometry


def display_start_point(toolpath, colour="purple"):
    plt.plot(toolpath.start_point.x, toolpath.start_point.y, marker='o', c=colour, markersize=10)

def display_outline(shapes, colour="blue"):
    """ Display the outline of the shape to be cut. """
    if shapes.type != "MultiPolygon":
        shapes = MultiPolygon([shapes])
    for shape in shapes.geoms:
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


def generate_tool_path(shapes, step_size, inner=True):
    """ Calculate the toolpath. """

    if inner:
        previous_cut = shapes.geoms[0].buffer(-5)

        toolpath = geometry.RefineInnerPocket(
                shapes,
                previous_cut,
                step_size,
                geometry.ArcDir.Closest,
                generate=True,
                #starting_point=Point(30, 30),
                debug=True)
    else:
        previous_cut = Polygon()
        for shape in shapes.geoms:
            shape_poly = Polygon(shape.exterior)
            previous_cut = previous_cut.union(
                    shape_poly.buffer(5).difference(shape_poly.buffer(3)))

        toolpath = geometry.RefineOuter(
                shapes,
                previous_cut,
                step_size,
                geometry.ArcDir.Closest,
                #starting_point=Point(44, 0),
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
    shapes = dxf.dxf_to_polygon(modelspace)

    toolpath = generate_tool_path(shapes, step_size, inner=False)
    #toolpath = generate_tool_path(shapes, step_size, inner=True)

    display_outline(shapes)
    display_toolpath(toolpath)
    display_voronoi(toolpath)
    # display_visited_voronoi_edges(toolpath)
    display_start_point(toolpath)

    plt.gca().set_aspect('equal')
    plt.show()

if __name__ == "__main__":
    main(sys.argv)
