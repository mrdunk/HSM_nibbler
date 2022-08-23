#!/usr/bin/env python3
"""
Debugging functions.
"""

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
    if not plt:
        print("Error: matplotlib.pyplot not imported.")
        return

    if polygons is None:
        polygons = []

    for multi in polygons:
        if multi.type != "MultiPolygon":
            multi = MultiPolygon([multi])
        for polygon in multi.geoms:
            for ring in [polygon.exterior] + list(polygon.interiors):
                x, y = ring.xy
                plt.plot(x, y, linewidth=1)

    if voronoi:
        for edge in voronoi.edges.values():
            x = []
            y = []
            for point in edge.coords:
                x.append(point[0])
                y.append(point[1])
            plt.plot(x, y, c="red", linewidth=2)
            plt.plot(x[0], y[0], 'x', c='red')
            plt.plot(x[-1], y[-1], 'x', c='red')

    plt.gca().set_aspect('equal')
    plt.show()

