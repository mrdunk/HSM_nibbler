#!/usr/bin/env python3
"""
Experimenting with CNC machining toolpaths.
This program is a demo which uses the main library file on test .dxf CAD files.
"""

import sys

import ezdxf
import matplotlib.pyplot as plt    # type: ignore
from shapely.geometry import box, LinearRing, LineString, MultiPolygon, Point, Polygon  # type: ignore

import dxf
import geometry


def display_outline(shape, colour="blue"):
    """ Display the outline of the shape to be cut. """
    shapes = shape
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

def display_start_point(toolpath, colour="purple"):
    plt.plot(toolpath.start_point.x, toolpath.start_point.y, marker='o', c=colour, markersize=10)

def main(argv):
    """
    Example program making use of HSM "peeling" CAM.
    """
    if len(argv) < 2:
        print("Incorrect command line arguments.")
        print(f"Use:\n   {argv[0]} FILENAME [STEP_sIZE]")
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

    material_bounds = shapes.buffer(20 * step_size).bounds
    material = box(*material_bounds)

    already_cut = material
    for shape in shapes.geoms:
        already_cut = already_cut.difference(Polygon(shape.exterior))

    #toolpath = geometry.OutsidePocketSimple(
    #        shapes[0],
    #        step_size,
    #        geometry.ArcDir.Closest,
    #        generate=True)
    toolpath = geometry.Pocket(
            shapes,
            step_size,
            geometry.ArcDir.Closest,
            already_cut=already_cut,
            generate=True,
            starting_point_tactic = geometry.StartPointTactic.PERIMETER,
            debug=True)

    timeslice = 100  # ms
    for index, progress in enumerate(toolpath.get_arcs(timeslice)):
        print(index, progress)
        # toolpath.path contains the currently generated path data at this point.

    #display_outline(shapes)
    display_toolpath(toolpath)
    display_voronoi(toolpath)
    # display_visited_voronoi_edges(toolpath)
    display_start_point(toolpath)

    plt.gca().set_aspect('equal')
    plt.show()

if __name__ == "__main__":
    main(sys.argv)
