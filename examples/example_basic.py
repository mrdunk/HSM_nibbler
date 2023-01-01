#!/usr/bin/env python3
"""
Experimenting with CNC machining toolpaths.
This program is a demo which uses the main library file on test .dxf CAD files.

The demo shows how to calculate a HSM path on the inside of the shape to be cut.
"""

from typing import List, Tuple

import os
import sys
import math
import ezdxf
import matplotlib.pyplot as plt    # type: ignore
from shapely.affinity import rotate  # type: ignore
from shapely.geometry import Point, LineString  # type: ignore

# This line is required if you want to use the local version of the code.
# If you have installed HSM_nibble via PIP it is not required.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from hsm_nibble import dxf
from hsm_nibble import geometry

#import warnings
#from shapely.errors import ShapelyDeprecationWarning
#warnings.filterwarnings("error", category=ShapelyDeprecationWarning)

def print_entity(entity: ezdxf.entities.DXFGraphic, indent: int = 0):
    """ Display some debug information about a DXF file. """
    dxf_attributes = ["start", "end", "center", "radius", "count"]
    collection_attributes = ["points"]
    other_attributes = ["virtual_entities"]

    padding = " " * indent
    print(f"{padding}{entity}")
    print(f"{padding}  type: {entity.dxftype()}")

    for attribute in dxf_attributes:
        if hasattr(entity.dxf, attribute):
            print(f"{padding}  {attribute}: {getattr(entity.dxf, attribute)}")

    for attribute in collection_attributes:
        if hasattr(entity, attribute):
            generator = getattr(entity, attribute)
            with generator() as collection:
                print(f"{padding}  {attribute}: {collection}")

    for attribute in other_attributes:
        if hasattr(entity, attribute):
            got = getattr(entity, attribute)
            print(f"{padding}  {attribute}: {list(got())}")

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

def display_entry_point(toolpath):
    if toolpath.starting_radius is None:
        # Entry circle not set.
        return

    starting_circle = geometry.create_circle(
            toolpath.start_point, toolpath.starting_radius).path
    x, y = starting_circle.xy

    colour = "red"
    if toolpath.max_starting_radius >= toolpath.starting_radius:
        colour = "green"

    plt.plot(x, y, linestyle=':', c=colour, linewidth=2)

    if toolpath.max_starting_radius < toolpath.starting_radius:
        # Requested entry circle doesn't fit in the part.
        # Instead max_starting_radius contains the best fit radius.
        starting_circle = geometry.create_circle(
                toolpath.start_point, toolpath.max_starting_radius).path
        x, y = starting_circle.xy
        plt.plot(x, y, linestyle='--', c="red", linewidth=1)

    if toolpath.starting_angle is not None:
        # This implies the entry circle overlaps the cutting path.
        # The contained angle is the direction to the start of the cutting path
        # where it is intersected by the entry circle.
        radius = min(toolpath.starting_radius, toolpath.max_starting_radius)
        line = LineString([
            toolpath.start_point,
            [toolpath.start_point.x, toolpath.start_point.y + radius]
            ])
        line = rotate(line, -toolpath.starting_angle, origin=toolpath.start_point, use_radians=True)

        x, y = line.xy
        plt.plot(x, y, linestyle='--', c="green", linewidth=2)

def display_starting_circle(toolpath, colour="orange"):
    starting_circle = geometry.create_circle(
            toolpath.start_point, toolpath.start_radius).path
    x, y = starting_circle.xy
    plt.plot(x, y, c=colour, linewidth=4)

def display_toolpath(toolpath, cut_colour="green", rapid_inside_colour="blue", rapid_outside_colour="orange"):
    # Display path.
    last_element = None

    for element in toolpath.path:
        if last_element is not None:
            assert last_element.end.equals_exact(element.start, 6)
            assert Point(last_element.path.coords[-1]
                    ).equals_exact(Point(element.path.coords[0]), 6)
        last_element = element

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


def generate_tool_path(shape, step_size):
    """ Calculate the toolpath. """
    toolpath = geometry.Pocket(
            shape,
            step_size,
            #geometry.ArcDir.Closest,
            geometry.ArcDir.CW,
            #geometry.ArcDir.CCW,
            generate=True,
            #starting_point=Point(-39.9, 11.8),
            #starting_radius=2.5,
            debug=True)

    timeslice = 1000  # ms
    for index, progress in enumerate(toolpath.get_arcs(timeslice)):
        print(index, round(progress * 1000) / 1000)
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
    #shape = dxf.dxf_to_polygon(modelspace).geoms[0]

    toolpath = generate_tool_path(shape, step_size)
    # Call toolpath.calculate_path() to scrap the existing and regenerate toolpath.

    display_outline(shape)
    display_entry_point(toolpath)
    #display_starting_circle(toolpath)
    display_toolpath(toolpath)
    display_voronoi(toolpath)
    # display_visited_voronoi_edges(toolpath)

    plt.gca().set_aspect('equal')
    plt.show()

if __name__ == "__main__":
    main(sys.argv)
