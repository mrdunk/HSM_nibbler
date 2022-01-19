#!/usr/bin/env python3
""" Experimenting with CNC machining toolpaths. """

from typing import Any, Dict, List, Optional, Tuple

import ezdxf
import math
import matplotlib.pyplot as plt    # type: ignore
import random
import sys

import geometry
import operations

DxfPolyline = ezdxf.entities.lwpolyline.LWPolyline


def print_entity(entity: ezdxf.entities.DXFGraphic, indent: int = 0):
    dxf_attributes = ["start", "end", "center", "radius", "count"]
    collection_attributes = ["points"]

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


def display_polyline(line: DxfPolyline, color: str = "black", width: float = 1) -> None:
    x = []
    y = []
    with line.points('xyseb') as points:
        for point in points:
            x.append(point[0])
            y.append(point[1])
    x.append(x[0])
    y.append(y[0])
    plt.plot(x, y, c=color, linewidth=width)


def main(argv):
    if len(argv) < 2:
        print("Incorrect command line arguments.")
        print(f"Use:\n   {argv[0]} FILENAME [RANDOM_SEED]")
        sys.exit(0)
    filename = argv[1]

    try:
        doc = ezdxf.readfile(filename)
    except IOError:
        print(f'Not a DXF file or a generic I/O error.')
        sys.exit(2)
    except ezdxf.DXFStructureError:
        print(f'Invalid or corrupted DXF file.')
        sys.exit(3)

    random_seed = None
    if len(argv) > 2:
        random_seed = argv[2]
    else:
        random_seed = random.randint(1, 999999)

    random.seed(int(random_seed))
    
    print(f"{filename=}\n{random_seed=}\n")


    modelspace = doc.modelspace()

    line = modelspace.query('LWPOLYLINE').first
    start = geometry.start_point(line)

    #arc = geometry.Arc.from_angle(start, 1.5 * math.pi, math.pi, 10)
    #arc = geometry.ArcLineString.from_points(start, (start[0] + 50, start[1] + 50), (start[0] + 50, start[1] - 50))
    #arc = geometry.Arc.from_angle(start, math.pi, math.pi, 10)
    #arc = geometry.Arc.from_points(start, (start[0] + 50, start[1] + 50), (start[0] + 50, start[1] + 50))
    #x, y = arc.xy
    #plt.plot(arc.start.x, arc.start.y, 'X', c="red")
    #plt.plot(arc.end.x, arc.end.y, 'X', c="red")
    #plt.plot(x[:10], y[:10], c="blue", linewidth=3)
    #plt.plot(x, y, c="green", linewidth=2)

    #print_entity(line)
    #print(start)

    marker = plt.plot(start[0], start[1], 'o', label='Start', c="black")
    plt.setp(marker[0], markersize=5)
    legend = plt.legend(loc='upper left', shadow=True)
    display_polyline(line, color="green")

    #operations.HsmPath(start, line)
    #hole = geometry.HelicalHole(line, start, 2, 3)
    #x, y = hole.xy
    #plt.plot(x, y, c="green", linewidth=2)
    hsm = operations.HsmPath(start, line)
    for shape in hsm.output:
        x, y = shape.xy
        plt.plot(x, y, c="green")

    plt.show()


if __name__ == "__main__":
    main(sys.argv)
