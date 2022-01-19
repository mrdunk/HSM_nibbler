from typing import List, Tuple

from ezdxf.entities.lwpolyline import LWPolyline as DxfPolyline
import math
import matplotlib.pyplot as plt                            # type: ignore
import random
from shapely.geometry import LineString, MultiLineString, Point, Polygon    # type: ignore
from shapely.ops import linemerge

def angle_between(origin: Point, point: Point) -> float:
    return math.atan2((point.x - origin.x), (point.y - origin.y))
    #return math.atan2((point.y - origin.y), (point.x - origin.x))

def move_point(point: Point, distance: float, angle: float) -> Point:
    return Point(
                point.x + math.cos(angle) * distance,
                point.y - math.sin(angle) * distance)

def populate_polygon(line: DxfPolyline) -> Polygon:
    """ Convert .dxf polyline into shapley Polygon. """
    point_list = []
    with line.points('xy') as points:
        for point in points:
            point_list.append(Point(point[:2]))

    return Polygon(point_list)

def start_point(line: DxfPolyline) -> Tuple[int, int]:
    polygon = populate_polygon(line)
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

    return (x, y)

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
            material: DxfPolyline,
            origin: Point,
            overlap: float,
            start_radius: float) -> LineString:
        self.origin = origin
        self.overlap = overlap

        polygon: Polygon = populate_polygon(material)
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
            for part in linestring:
                linestring_parts += list(part.coords)
            linestring = LineString(linestring_parts)

        self.intersection = Point(linestring.coords[-1])

        LineString.__init__(self, linestring)
