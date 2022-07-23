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


def spiral(center: Point, radius: float, step_size: float) -> List:
    arcs: List[Arc] = []
    loop: float = 0.25
    offset: List[float] = [0.0, 0.0]
    while loop * step_size < radius:
        orientation = round(loop * 4) % 4
        if orientation == 0:
            start_angle = 0
            offset[1] -= step_size / 4
            section_radius = loop * step_size
        elif orientation == 1:
            start_angle = math.pi / 2
            offset[0] -= step_size / 4
            section_radius = loop * step_size
        elif orientation == 2:
            start_angle = math.pi
            offset[1] += step_size / 4
            section_radius = loop * step_size
        elif orientation == 3:
            start_angle = 3 * math.pi / 2
            offset[0] += step_size / 4
            section_radius = loop * step_size
        else:
            raise

        section_center = Point(center.x + offset[0], center.y + offset[1])
        arcs.append(geometry.create_arc(
            section_center, section_radius, start_angle, math.pi / 2))

        loop += 0.25  # 1/4 turn.
        
    arcs.append(geometry.create_arc(center, radius, 0, math.pi * 2 -0.001))

    return arcs

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

def display_starting_circle(toolpath, colour="orange"):
    starting_circle = geometry.create_circle(
            toolpath.start_point, toolpath.start_radius).path
    x, y = starting_circle.xy
    plt.plot(x, y, c=colour, linewidth=4)

def display_starting_spiral(toolpath, colour="green"):
    """
    Draw a starting entry spiral.
    Just for demo purposes; not provided by HSM library.
    """
    starting_arcs = spiral(toolpath.start_point, toolpath.start_radius, toolpath.step)
    for element in starting_arcs:
        x, y = element.path.xy
        plt.plot(x, y, c=colour, linewidth=1)

    return

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


def generate_tool_path(shape, step_size):
    """ Calculate the toolpath. """
    toolpath = geometry.Pocket(
            shape,
            step_size,
            geometry.ArcDir.Closest,
            generate=True,
            #starting_point=Point(-39.9, 11.8),
            debug=True)
    #toolpath = geometry.InsidePocket(
    #        shape, step_size, geometry.ArcDir.CW, generate=True, debug=True)

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
    #shape = dxf.dxf_to_polygon(modelspace).geoms[0]

    toolpath = generate_tool_path(shape, step_size)
    # Call toolpath.calculate_path() to scrap the existing and regenerate toolpath.

    display_outline(shape)
    display_starting_spiral(toolpath)
    #display_starting_circle(toolpath)
    display_toolpath(toolpath)
    display_voronoi(toolpath)
    # display_visited_voronoi_edges(toolpath)

    plt.gca().set_aspect('equal')
    plt.show()

if __name__ == "__main__":
    main(sys.argv)
