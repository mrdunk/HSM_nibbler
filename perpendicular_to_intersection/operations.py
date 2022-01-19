"""
Interesting random seeds:
    687914 : Causes a circle to be drawn parallel to a material edge. This
      causes a "LineString" intersection.
    629983 : An intersection on the join between 2 arcs make it trigger twice.
"""

from typing import Any, List, Optional

from ezdxf.entities.lwpolyline import LWPolyline as DxfPolyline
import math
import matplotlib.pyplot as plt                  # type: ignore
from shapely.coords import CoordinateSequence    # type: ignore
from shapely.geometry import LineString, MultiLineString, Point, Polygon      # type: ignore
from shapely.ops import linemerge                # type ignore

import geometry

class CncPath:
    def __init__(self, start: Point, end: Point, origin: Point, mid_on_arc: Point) -> None:
        self.start: Point = start
        self.end: Point = end
        self.origin: Point = origin
        self.mid_on_arc: Point = mid_on_arc

    def __str__(self) -> str:
        #plt.plot(self.start.x, self.start.y, 'o', c="green")
        #plt.plot(self.end.x, self.end.y, 'o', c="cyan")
        #plt.plot(self.origin.x, self.origin.y, 'o', c="magenta")
        return (f"{id(self)}  "
                f"start: {self.start.type}{self.start.coords[0]},  "
                f"end: {self.end.type}{self.end.coords[0]},  "
                f"origin: {self.origin.type}{self.origin.coords[0]}")

    def mid(self) -> Point:
        """ Midway between start and end points. """
        return Point((self.start.x + self.end.x) / 2, (self.start.y + self.end.y) / 2)

    def angle(self) -> float:
        """ Angle between start - end mid point and origin. """
        return geometry.angle_between(self.mid(), self.origin)


def circle_center(c1: Point, c2: Point, c3: Point) -> Point:
    c = (c1.x - c2.x) ** 2 + (c1.y - c2.y) ** 2
    a = (c2.x - c3.x) ** 2 + (c2.y - c3.y) ** 2
    b = (c3.x - c1.x) ** 2 + (c3.y - c1.y) ** 2
    s = 2 * (a * b + b * c + c * a) - (a * a + b * b + c * c)
    px = (a * (b + c - a) * c1.x + b * (c + a - b) * c2.x + c * (a + b - c) * c3.x) / s
    py = (a * (b + c - a) * c1.y + b * (c + a - b) * c2.y + c * (a + b - c) * c3.y) / s
    return Point(px, py)


class HsmPath:
    def __init__(self, origin: Point, material: DxfPolyline, overlap: float = 1):
        self.origin = Point(origin)
        self.material = material
        self.polygon: Polygon = geometry.populate_polygon(material)
        self.polygon_path: Polygon = Polygon()
        self.overlap = overlap

        self.output: List[LineString] = []
        self.open_jobs: List[CncPath] = []

        hole_start_radius = overlap

        self.hole = geometry.HelicalHole(material, origin, overlap, hole_start_radius)
        self.output.append(self.hole)

        self._tidy_hole()

        self._process_jobs()


    def _tidy_hole(self) -> None:
        hole_intersection = self.hole.intersection
        arc = geometry.ArcLineString.from_points(
                self.origin, hole_intersection, hole_intersection)

        new_intersections = arc.intersection(self.polygon)
        if new_intersections.type in ["LineString", "Point"]:
            new_intersections = [new_intersections]
        for new_intersection in new_intersections:
            #print("_tidy_hole", new_intersection.type)
            if new_intersection.type == "LineString" and len(new_intersection.coords) > 2:
                self.output.append(new_intersection)
                length = new_intersection.length
                mid_on_arc = new_intersection.interpolate(length / 2)
                self.open_jobs.append(CncPath(
                            Point(new_intersection.coords[0]),
                            Point(new_intersection.coords[-1]),
                            self.origin,
                            mid_on_arc))
                self.polygon_path = self.polygon_path.union(Polygon(new_intersection))

    def _process_jobs(self) -> None:
        count = 0
        while self.open_jobs:
            if count % 100 == 0:
                print(count)
            count += 1

            job = self.open_jobs.pop()
            #print("_process_jobs", job)

            # Move the next arc out by the overlap value.
            angle = geometry.angle_between(job.origin, job.mid_on_arc)
            #new_mid_on_arc = geometry.move_point(job.mid_on_arc, self.overlap * 3, angle)
            new_mid_on_arc = Point(
                    job.mid_on_arc.x + math.sin(angle) * self.overlap * 7,
                    job.mid_on_arc.y + math.cos(angle) * self.overlap * 7)

            new_arc_origin = circle_center(job.start, new_mid_on_arc, job.end)
            new_arc_origin = Point(
                    (new_arc_origin.x + 9 * job.origin.x) / 10,
                    (new_arc_origin.y + 9 * job.origin.y) / 10)

            #plt.plot(job.mid_on_arc.x, job.mid_on_arc.y, 'o', c="yellow")
            #plt.plot(new_mid_on_arc.x, new_mid_on_arc.y, 'x', c="black")
            #plt.plot(new_arc_origin.x, new_arc_origin.y, 'x', c="magenta")

            arc = geometry.ArcLineString.from_points(new_arc_origin, job.start, job.end)
            #x, y = arc.xy
            #plt.plot(x, y, c="red", linewidth=1)

            intersections = arc.intersection(self.polygon)
            if intersections.type in ["LineString", "Point"]:
                intersections = [intersections]
            for intersection in intersections:
                #print("_process_jobs", intersection.type)
                if intersection.type == "LineString" and len(intersection.coords) > 2:
                    area_ramaining = Polygon(intersection).difference(self.polygon_path).area
                    if area_ramaining > 0.01:
                        length = intersection.length
                        mid_on_arc = intersection.interpolate(length / 2)
                        self.open_jobs.append(CncPath(
                            Point(intersection.coords[0]),
                            Point(intersection.coords[-1]),
                            new_arc_origin,
                            mid_on_arc))
                        self.polygon_path = self.polygon_path.union(Polygon(intersection))
                        self.output.append(intersection)
                    else:
                        print("**")
            #self.output.append(arc)



