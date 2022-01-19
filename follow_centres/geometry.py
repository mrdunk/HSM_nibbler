from typing import List, Optional, Tuple

from ezdxf.entities.lwpolyline import LWPolyline as DxfPolyline
from ezdxf.query import EntityQuery as DxfPolylines
import math
import matplotlib.pyplot as plt                            # type: ignore
import random
from shapely.ops import nearest_points
from shapely.geometry import LineString, MultiLineString, Point, Polygon, LinearRing, MultiPoint    # type: ignore
from shapely.ops import linemerge, split, voronoi_diagram
import time

def timing(f):
    def wrap(*args, **kwargs):
        time1 = time.time()
        ret = f(*args, **kwargs)
        time2 = time.time()
        print('{:s} function took {:.3f} ms'.format(f.__name__, (time2-time1)*1000.0))

        return ret
    return wrap

def angle_between(origin: Point, point: Point) -> float:
    return math.atan2((point.x - origin.x), (point.y - origin.y))
    #return math.atan2((point.y - origin.y), (point.x - origin.x))

def move_point(point: Point, distance: float, angle: float) -> Point:
    return Point(
                point.x + math.cos(angle) * distance,
                point.y - math.sin(angle) * distance)

@timing
def dxf_to_polygon(polylines: DxfPolylines) -> Polygon:
    """ Convert .dxf polyline into shapley Polygon. """
    rings = []
    for polyline in polylines:
        point_list = []
        with polyline.points('xy') as points:
            for point in points:
                point_list.append(Point(point[:2]))
        rings.append(LinearRing(point_list))

    return Polygon(rings[0], holes = [rings[1]])

def start_point(polygon: Polygon) -> Tuple[int, int]:
    x = polygon.bounds[0] - 1
    y = polygon.bounds[1] - 1
    point = Point(x, y)

    while not point.within(polygon):
        x = random.randint(int(polygon.bounds[0]), int(polygon.bounds[2]))
        y = random.randint(int(polygon.bounds[1]), int(polygon.bounds[3]))
        point = Point(x, y)

        #if point.within(polygon):
        #    marker = plt.plot(x, y, 'x', c="green")
        #else:
        #    marker = plt.plot(x, y, 'x', c="red")
        marker = plt.plot(x, y, 'x', c="red")
        plt.setp(marker[0], markersize=5)

    return Point(x, y)

class ArcLineString(LineString):
    def __init__(self, points):
        LineString.__init__(self, points)

    @classmethod
    def from_points(cls, origin_: Point, start_: Point, end_: Point):
        origin = Point(origin_)
        start = Point(start_)
        end = Point(end_)

        start_angle = angle_between(origin, start)
        end_angle = angle_between(origin, end)
        radius = origin.distance(start)

        return cls.from_angle(origin, start_angle, end_angle, radius)

    @classmethod
    def from_angle(
            cls,
            origin: Point,
            start_angle: float,
            end_angle: float,
            radius: float,
            steps: int = 200
            ):
        cls.origin = Point(origin)
        cls.start_angle = start_angle % (math.pi * 2)
        cls.end_angle = end_angle % (math.pi * 2)
        cls.radius = radius

        tmp_start_angle = cls.start_angle
        if tmp_start_angle >= cls.end_angle:
            tmp_start_angle -= math.pi * 2
        step_size = 2 * math.pi / steps

        points = []
        while tmp_start_angle < cls.end_angle:
            points.append(cls.polar_point(cls.origin, tmp_start_angle, radius))
            tmp_start_angle += step_size

        last_point = cls.polar_point(cls.origin, cls.end_angle, radius)
        if points[-1] != last_point:
            points.append(last_point)

        cls.start_point = points[0]
        cls.end_point = points[-1]

        # Close the loop.
        #points.append(cls.origin)
        #points.append(points[0])

        return cls(points)

    @staticmethod
    def polar_point(origin: Point, angle: float,  radius: float) -> Point:
        return Point(
                origin.x + math.sin(angle) * radius,
                origin.y + math.cos(angle) * radius)

class HelicalHole(LineString):
    def __init__(
            self,
            polygon: Polygon,
            origin: Point,
            overlap: float,
            start_radius: float) -> LineString:
        self.origin = origin
        self.overlap = overlap

        steps = 4
        angle_chunk = math.pi * 2 / steps
        head_angle = 0.0
        tail_angle = 0.0
        radius = start_radius / 2
        new_origin = Point(origin)

        path = None

        while True:
            head_angle = tail_angle + angle_chunk
            head_angle %= (math.pi * 2)
            tail_angle %= (math.pi * 2)
            if head_angle < tail_angle:
                head_angle += math.pi * 2

            radius += overlap / steps / 2

            new_origin = move_point(new_origin, overlap / steps / 2, head_angle)

            arc = ArcLineString.from_angle(new_origin, tail_angle, head_angle, radius)

            if path:
                #print(head_angle / math.pi, tail_angle / math.pi, path.type, arc.type)
                path = LineString(list(path.coords) + list(arc.coords))
            else:
                path = arc

            if not path.within(polygon) or path.touches(polygon.boundary):
                break

            tail_angle = head_angle

        linestring = path.intersection(polygon)
        if linestring.type == "MultiLineString":
            linestring_parts = []
            for part in linestring.geoms:
                linestring_parts += list(part.coords)
            linestring = LineString(linestring_parts)

        self.intersection = Point(linestring.coords[-1])

        LineString.__init__(self, linestring)

class Voronoi:
    @timing
    def __init__(self, polygon: Polygon) -> None:
        self.polygon = polygon
        self.step = 2

        #remainder = LineString(polygon.exterior.coords)
        outlines = [polygon.exterior] + list(polygon.interiors)
        faceted_outlines = []
        for outline in outlines:
            remainder = LineString(outline)
            parts = []
            while remainder.length > self.step:
                split_point = remainder.interpolate(self.step)
                tmp = split(remainder, split_point.buffer(0.001))
                if len(tmp.geoms) == 3:
                    short, _, remainder = tmp.geoms
                    parts += short.coords[:-1]
                    #plt.plot(split_point.x, split_point.y, "o",  c="black")

            parts.append(split_point)
            parts.append(parts[0])
            faceted_outlines.append(LineString(parts))
            #x, y = faceted.xy
            #plt.plot(x, y, c="red")


        combined_edges = {}
        all_points = [point for outline in faceted_outlines for point in outline.coords]
        print(len(all_points))
        edges = voronoi_diagram(MultiPoint(all_points), edges=True)
        for edge in edges.geoms:
            for line in edge.geoms:
                if line.distance(polygon.exterior) > self.step and line.within(polygon):
                #if line.within(polygon):
                    start = line.coords[0]
                    end = line.coords[-1]
                    assert start != end
                    content = (start, end) if start < end else (end, start)
                    combined_edges.setdefault(self._point_as_key(start), []
                            ).append(content)
                    combined_edges.setdefault(self._point_as_key(end), []
                            ).append(content)

                    x, y = line.xy
                    plt.plot(x, y, c="red")

        self.combined_edges = combined_edges

    @timing
    def fatest_point(self) -> Point:
        closest_dist = 0
        closest_point = None

        for point, branches in self.combined_edges.items():
            for branch in branches:
                if self._compare_point(point, branch[0]):
                    dist = Point(point).distance(self.polygon.boundary)
                    if dist > closest_dist:
                        closest_dist = dist
                        closest_point = point
        return Point(closest_point)

    @timing
    def walk(self, start: Optional[Point] = None) -> None:
        cutaway = Polygon()

        recent = None
        progress = True
        location = (start.x, start.y) or tuple(self.combined_edges.keys())[0]
        plt.plot(location[0], location[1], "o",  c="black")

        while progress:
            progress = None
            options = self.combined_edges[location]
            for option in options:
                other_end = self._other_end(location, option)
                if other_end != recent:
                    x = [location[0], other_end[0]]
                    y = [location[1], other_end[1]]
                    plt.plot(x, y, linewidth=5)

                    nearest = nearest_points(Point(location), self.polygon.boundary)
                    radius = Point(location).distance(nearest[1])
                    arc = Point(location).buffer(radius).boundary
                    arc_section = arc.difference(cutaway)

                    cutaway = cutaway.union(Polygon(arc))

                    if arc_section.type == "LineString":
                        arc_section = [arc_section]
                    for thing in arc_section:
                        x, y = thing.xy
                        plt.plot(x, y, c="green")


                    progress = True
                    recent = location
                    location = other_end
                    break

    def _compare_point(self, a, b) -> bool:
        return self._point_as_key(a) == self._point_as_key(b)

    def _other_end(self, point, line) -> bool:
        if self._compare_point(point, line[0]):
            return line[1]
        elif self._compare_point(point, line[1]):
            return line[0]
        return None

    def _join_intersections(self):
        coords = list(self.combined_edges.keys())
        for coord in coords:
            edgelet = self.combined_edges[coord]
            if len(edgelet) == 1:
                # A line end.
                plt.plot(coord[0], coord[1], "o", c="black")
            elif len(edgelet) == 2:
                # A candidate to be joined.
                assert len(edgelet[0]) == 2
                assert len(edgelet[1]) == 2
                new_edge = []
                for candiate_coord in list(edgelet[0]) + list(edgelet[1]):
                    if coord != self._point_as_key(candiate_coord):
                        new_edge.append(candiate_coord)
                assert len(new_edge) == 2
                if new_edge[0] > new_edge[1]:
                    new_edge = [new_edge[1], new_edge[0]]
                assert new_edge[0] < new_edge[1]
                del self.combined_edges[coord]

                # Update other entries referenced by coord.
                self._replace_coord(coord, new_edge)

        print()
        print(len(self.combined_edges))
        for key, value in self.combined_edges.items():
            print(key, ":", value)
            for edge in value:
                if edge[0] == key:
                    x = [edge[0][0], edge[1][0]]
                    y = [edge[0][1], edge[1][1]]
                    plt.plot(x, y)

    def _replace_coord(self, bad_coord, replacment):
        key_a = self._point_as_key(replacment[0])
        key_b = self._point_as_key(replacment[1])
        for key in [key_a, key_b]:
            candidate_edges = self.combined_edges.get(key)
            if candidate_edges is None:
                continue
            for index, candidate in enumerate(candidate_edges):
                for test_coord in candidate:
                    if self._point_as_key(test_coord) == bad_coord:
                        print(bad_coord, ":", replacment, ">", candidate_edges[index])
                        candidate_edges[index] = replacment


    def _point_as_key(self, coord: Tuple[float, float]) -> Tuple[float, float]:
        #return (round(coord[0], 2), round(coord[1], 2))
        return coord
