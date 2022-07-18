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

    #material = LineString([(-100, -100), (-100, 150), (200, 150), (200, -100)])
    #material = LineString([(10, 10), (10, 200), (200, 200), (200, 10)])
    material_bounds = shapes.buffer(4 * step_size).bounds
    material = box(*material_bounds)

    #material_bounds = shape.buffer(4 * step_size).bounds
    #longest = max(abs(material_bounds[0] - material_bounds[2]),
    #        abs(material_bounds[1] - material_bounds[3]))
    #material = shape.centroid.buffer(2 * longest / 3).exterior

    # Generate tool path.
    #toolpath = geometry.OutsidePocket(shapes, material, step_size, geometry.ArcDir.CW, generate=True)
    #toolpath = geometry.OutsidePocket(shapes, material, step_size, geometry.ArcDir.Closest, generate=True)
    #toolpath = geometry.OutsidePocketSimple(shapes[0], step_size, geometry.ArcDir.CW, generate=True)
    #toolpath = geometry.OutsidePocketSimple(shapes[0], step_size, geometry.ArcDir.Closest, generate=True)
    toolpath = geometry.OuterPeel(
            shapes, step_size, geometry.ArcDir.Closest, generate=True, debug=True)


    # Display shape to be cut
    for shape in shapes.geoms:
        x, y = toolpath.voronoi.polygon.exterior.xy
        plt.plot(x, y, linestyle='--', c="blue", linewidth=2)

        multi = toolpath.polygon
        if multi.type != "MultiPolygon":
            multi = MultiPolygon([multi])
        for poly in multi.geoms:
            x, y = poly.exterior.xy
            #plt.plot(x, y, c="blue", linewidth=2)

            for interior in poly.interiors:
                x, y = interior.xy
                #plt.plot(x, y, c="orange", linewidth=2)


    # Draw arcs via generator.
    timeslice = 100  # ms
    for index, progress in enumerate(toolpath.get_arcs(timeslice)):
        print(index, progress)
        #if index == 100:
        #    break

        # You have access to toolpath.path here.
        # Draw what's there so far; it will ot change position in the buffer.

    # Call toolpath.calculate_path() to scrap the existing and regenerate toolpath.

    # Display voronoi edges.
    for edge in toolpath.voronoi.edges.values():
        x = []
        y = []
        for point in edge.coords:
            x.append(point[0])
            y.append(point[1])
        plt.plot(x, y, c="red", linewidth=2)
        plt.plot(x[0], y[0], 'x', c="red")
        plt.plot(x[-1], y[-1], 'x', c="red")

    # Starting circle.
    #starting_circle = geometry.create_circle(toolpath.start_point, toolpath.start_radius).path
    #x, y = starting_circle.xy
    #plt.plot(x, y, c="orange", linewidth=1)

    # Display path.
    for element in toolpath.path:
        if type(element).__name__ == "Arc":
            x, y = element.path.xy
            if element.debug:
                plt.plot(x, y, c=element.debug, linewidth=3)
            else:
                plt.plot(x, y, c="green", linewidth=1)
                pass
            #plt.plot(element.origin.x, element.origin.y, "o")

        elif type(element).__name__ == "Line":
            x, y = element.path.xy
            if element.move_style == geometry.MoveStyle.RAPID_INSIDE:
                plt.plot(x, y, linestyle='--', c="blue", linewidth=1)
                pass
            elif element.move_style == geometry.MoveStyle.RAPID_OUTSIDE:
                plt.plot(x, y, c="orange", linewidth=1)
                pass
            else:
                assert element.move_style == geometry.MoveStyle.CUT
                plt.plot(x, y, linestyle='--', c="green", linewidth=1)

    plt.plot(toolpath.start_point.x, toolpath.start_point.y, 'o', c="black")

    plt.gca().set_aspect('equal')
    plt.show()

if __name__ == "__main__":
    main(sys.argv)
