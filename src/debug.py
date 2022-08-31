#!/usr/bin/env python3
"""
Debugging functions.

Recognises the following environment variables:
    HSM_DEBUG: Set to enable any debug output. Default = not set.
    HSM_DEBUG_RES: (Optional) Resolution of output image in dpi. Default = 1000 dpi.
    HSM_DEBUG_FILENAME: (Optional) Specify output image filename. Default = /tmp/hsm.png
    HSM_DEBUG_SCREEN: (Optional) Attempt to output image to screen. Default = not set.
"""

import os

from typing import List, Optional, Union, Any

try:
    import matplotlib.pyplot as plt    # type: ignore
except ImportError:
    pass

from shapely.geometry import MultiPolygon, Polygon  # type: ignore

try:
    from voronoi_centers import VoronoiCenters  # type: ignore
except ImportError:
    from cam.voronoi_centers import VoronoiCenters  # type: ignore

def display(polygons: Optional[List[Union[Polygon, MultiPolygon]]] = None,
            voronoi: Optional[VoronoiCenters] = None) -> None:
    """
    Use matplotlib.pyplot to display outline of polygons and voronoi diagram.
    """
    if not os.environ.get("HSM_DEBUG"):
        return

    if not plt:
        print("Error: matplotlib.pyplot not imported.")
        return

    resolution = 1000
    filename = "/tmp/hsm.png"
    screen = False

    if os.environ.get("HSM_DEBUG_RES"):
        resolution = int(os.environ.get("HSM_DEBUG_RES"))

    if os.environ.get("HSM_DEBUG_FILENAME"):
        filename = os.environ.get("HSM_DEBUG_FILENAME")

    if os.environ.get("HSM_DEBUG_SCREEN"):
        screen = True

    print(f"Printing debug image: {filename} at resolution: {resolution}dpi")

    if polygons is None:
        polygons = []

    for multi in polygons:
        if multi.type != "MultiPolygon":
            multi = MultiPolygon([multi])
        for polygon in multi.geoms:
            for ring in [polygon.exterior] + list(polygon.interiors):
                x, y = ring.xy
                plt.plot(x, y, linewidth=0.01)

    if voronoi:
        for edge in voronoi.edges.values():
            x = []
            y = []
            for point in edge.coords:
                x.append(point[0])
                y.append(point[1])
            plt.plot(x, y, c="red", linewidth=0.1)
            plt.plot(x[0], y[0], 'o', c='red', markersize=0.1)
            plt.plot(x[-1], y[-1], 'o', c='red', markersize=0.1)

    plt.gca().set_aspect('equal')
    if filename:
        plt.savefig(filename, dpi=resolution, bbox_inches='tight')
    if screen:
        plt.show()

