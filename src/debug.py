#!/usr/bin/env python3
import matplotlib.pyplot as plt    # type: ignore
from shapely.geometry import MultiPolygon  # type: ignore

def display(polygons):
    for multi in polygons:
        if multi.type != "MultiPolygon":
            multi = MultiPolygon([multi])
        for polygon in multi.geoms:
            for ring in [polygon.exterior] + list(polygon.interiors):
                x, y = ring.xy
                plt.plot(x, y, linewidth=1)

    plt.gca().set_aspect('equal')
    plt.show()

