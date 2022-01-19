#!/usr/bin/env python3
""" Experimenting with CNC machining toolpaths. """

from typing import Any, Dict, List, Optional, Tuple

import ezdxf
import math
import matplotlib.pyplot as plt    # type: ignore
import random
import sys

import geometry


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


def main(argv):
    if len(argv) < 2:
        print("Incorrect command line arguments.")
        print(f"Use:\n   {argv[0]} FILENAME [RANDOM_SEED]")
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

    random_seed = None
    if len(argv) > 2:
        random_seed = argv[2]
        if random_seed.startswith("random_seed="):
            random_seed = random_seed.split("=")[1]
        random_seed = int(random_seed)
    else:
        random_seed = random.randint(1, 999999)

    random.seed(random_seed)

    print(f"{filename=}\n{random_seed=}\n")


    modelspace = dxf_data.modelspace()
    dxf_polylines = modelspace.query('LWPOLYLINE')
    shape = geometry.dxf_to_polygon(dxf_polylines)
    
    #start = geometry.start_point(shape)
    
    x, y = shape.exterior.xy
    plt.plot(x, y, c="blue", linewidth=2)

    for interior in shape.interiors:
        x, y = interior.xy
        plt.plot(x, y, c="blue", linewidth=2)

    v = geometry.Voronoi(shape)
    start = v.fatest_point()
    v.walk(start)

    #hole = geometry.HelicalHole(shape, start, 3, 3)
    #x, y = hole.xy
    #plt.plot(x, y, c="green")
    
    marker = plt.plot(start.x, start.y, 'o', c="black")

    plt.gca().set_aspect('equal')
    plt.show()


if __name__ == "__main__":
    main(sys.argv)
