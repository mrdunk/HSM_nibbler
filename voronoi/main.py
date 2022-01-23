#!/usr/bin/env python3
""" Experimenting with CNC machining toolpaths. """

import random
import sys

import ezdxf
import matplotlib.pyplot as plt    # type: ignore

import dxf
import geometry


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

    #print()
    #for entity in modelspace:
    #    print_entity(entity)
    #    print()

    shape = dxf.dxf_to_polygon(modelspace)

    x, y = shape.exterior.xy
    plt.plot(x, y, c="blue", linewidth=2)

    for interior in shape.interiors:
        x, y = interior.xy
        plt.plot(x, y, c="blue", linewidth=2)


    tp = geometry.ToolPath(shape, 5)

    for vertex, edges in tp.voronoi.vertex_to_edges.items():
        for edge_index in edges:
            edge = tp.voronoi.edges[edge_index]
            x = []
            y = []
            for point in edge.coords:
                x.append(point[0])
                y.append(point[1])
            plt.plot(x, y, c="red", linewidth=2)

    for element in tp.joined_path_data:
        if type(element).__name__ == "Arc":
            x, y = element.path.xy
            plt.plot(x, y, c="green", linewidth=1)
        elif type(element).__name__ == "Line":
            x, y = element.path.xy
            if element.safe:
                plt.plot(x, y, linestyle='--', c="purple", linewidth=1)
            else:
                plt.plot(x, y, linestyle='--', c="orange", linewidth=1)


    plt.plot(tp.start.x, tp.start.y, 'o', c="black")


    plt.gca().set_aspect('equal')
    plt.show()

if __name__ == "__main__":
    main(sys.argv)
