from typing import Dict, Generator, List, NamedTuple, Optional, Set, Tuple, Union

from enum import Enum
import math
import time

from shapely.geometry import LineString, MultiLineString, Point, Polygon
from shapely.ops import linemerge, nearest_points
from shapely.validation import make_valid

import pyvoronoi

Vertex = Tuple[float, float]

# Number of tries before we give up trying to find a best-fit arc and just go
# with the best we have found so far.
ITERATION_COUNT = 50

# Whether to visit short voronoi edges first (True) or try to execute longer
# branches first.
# TODO: We could use more long range path planning that take the shortest total
# path into account.
#BREADTH_FIRST = True
BREADTH_FIRST = False

# Number of decimal places resolution for coordinates.
ROUND_DP = 4

# Resolution of voronoi algorithm.
# See C++ Boost documentation.
#VORONOI_RES = 10000000
VORONOI_RES = 10**(ROUND_DP + 1)

# When arc sizes drop below a certain point, we need to reduce the step size or
# forward motion due to the distance between arcs (step) becomes more than the
# arc diameter.
# This constant is the minimum arc radius size at which we start reducing step size.
# Expressed as a multiple of the step size.
#CORNER_ZOOM = 4
#CORNER_ZOOM = 8
CORNER_ZOOM = 0
CORNER_ZOOM_EFFECT = 0.75
#CORNER_ZOOM_EFFECT = 3


class ArcDir(Enum):
    CW = 0
    CCW = 1
    # Closest = 2  # TODO


def timing(func):
    def wrap(*args, **kwargs):
        time1 = time.time()
        ret = func(*args, **kwargs)
        time2 = time.time()
        print('{:s} function took {:.3f} ms'.format(func.__name__, (time2-time1)*1000.0))

        return ret
    return wrap

def round_coord(value: Tuple[float, float]) -> Tuple[float, float]:
    return (round(value[0], ROUND_DP), round(value[1], ROUND_DP))


ArcData = NamedTuple("Arc", [
    ("origin", Point),
    ("radius", float),
    ("start", Optional[Point]),
    ("end", Optional[Point]),
    ("start_angle", float),
    ("span_angle", float),
    # TODO: ("widest_at", Optional[Point]),
    # TODO: ("start_DOC", float),
    # TODO: ("end_DOC", float),
    # TODO: ("widest_DOC", float),
    ("path", LineString),
    ("debug", str)
    ])

LineData = NamedTuple("Line", [
    ("start", Optional[Point]),
    ("end", Optional[Point]),
    ("path", LineString),
    ("safe", bool)
    ])

def join_arcs(start: ArcData, end: ArcData, safe_area: Polygon) -> LineData:
    path = LineString([start.path.coords[-1], end.path.coords[0]])
    safe = path.covered_by(safe_area)
    return LineData(start.start, end.end, path, safe)

def create_circle(
        origin: Point,
        radius: float,
        winding_dir: ArcDir,
        path: Optional[LineString]=None) -> ArcData:
    span_angle = 2 * math.pi
    if winding_dir == ArcDir.CCW:
        span_angle = -span_angle
    if path is None:
        return ArcData(
                origin, radius, None, None, 0, span_angle, origin.buffer(radius).boundary, "")
    # Warning: For this branch no check is done to ensure path has correct
    # winding direction.
    return ArcData(origin, radius, None, None, 0, span_angle, path, "")

def create_arc_from_path(
        origin: Point,
        winding_dir: ArcDir,
        path_: LineString,
        debug: str = None
        ) -> ArcData:
    # Make copy of path since we may need to modify it.
    path = LineString(path_)

    start = Point(path.coords[0])
    end = Point(path.coords[-1])
    mid = path.interpolate(0.5, normalized=True)
    radius = origin.distance(start)
    #print(round(radius, 1), round(origin.distance(mid), 1), round(origin.distance(end), 1))
    assert abs((origin.distance(mid) - radius) / radius) < 0.01
    assert abs((origin.distance(end) - radius) / radius) < 0.01

    start_angle = math.atan2(start.x - origin.x, start.y - origin.y)
    end_angle = math.atan2(end.x - origin.x, end.y - origin.y)
    mid_angle = math.atan2(mid.x - origin.x, mid.y - origin.y)

    ds = (start_angle - mid_angle) % (2 * math.pi)
    de = (mid_angle - end_angle) % (2 * math.pi)
    if ((ds > 0 and de > 0 and winding_dir == ArcDir.CCW) or
            (ds < 0 and de < 0 and winding_dir == ArcDir.CW)):
        # Needs reversed.
        path = LineString(path.coords[::-1])
        start = Point(path.coords[0])
        end = Point(path.coords[-1])
        start_angle, end_angle = end_angle, start_angle

    if winding_dir == ArcDir.CW:
        span_angle = (end_angle - start_angle) % (2 * math.pi)
    elif winding_dir == ArcDir.CCW:
        span_angle = -((start_angle - end_angle) % (2 * math.pi))

    return ArcData(origin, radius, start, end, start_angle, span_angle, path, debug)


def arcs_from_circle_diff(
        circle: ArcData,
        polygon: Polygon,
        winding_dir: ArcDir,
        debug: str = None
        ) -> List[ArcData]:
    """ Return any sections of circle that do not overlap polygon. """
    line_diff = circle.path.difference(polygon)
    if not line_diff:
        return []
    if line_diff.type == "MultiLineString":
        line_diff = linemerge(line_diff)
    if line_diff.type != "MultiLineString":
        line_diff = MultiLineString([line_diff])

    arcs = []
    for arc in line_diff.geoms:
        path = LineString(arc.coords)
        arcs.append(create_arc_from_path(circle.origin, winding_dir, path, debug))
    return arcs


class Voronoi:
    @timing
    def __init__(self, polygon: Polygon, tolerence: float = 0) -> None:
        self.polygon = polygon
        self.tolerence = tolerence

        self.max_dist = max((self.polygon.bounds[2] - self.polygon.bounds[0]),
                (self.polygon.bounds[3] - self.polygon.bounds[1])) + 1

        # Collect polygon segments to populate voronoi with.
        geom_primatives = []
        outlines = [polygon.exterior] + list(polygon.interiors)
        for outline in outlines:
            prev_point = None
            first_point = None
            for point in outline.coords:
                if prev_point is None:
                    first_point = point
                    prev_point = point
                elif point == prev_point:
                    continue
                else:
                    geom_primatives.append((prev_point, point))
                    prev_point = point
            if prev_point != first_point:
                geom_primatives.append((prev_point, first_point))

        # Generate voronoi diagram.
        pv = pyvoronoi.Pyvoronoi(VORONOI_RES)
        for segment in geom_primatives:
            pv.AddSegment(segment)
        pv.Construct()

        edges = pv.GetEdges()
        vertices = pv.GetVertices()
        cells = pv.GetCells()

        # Parse voronoi diagram. Store data as shapely LineSegment.
        self.edges: Dict[int, LineString] = {}
        self.vertex_to_edges: Dict[Vertex, List[int]] = {}
        self.edge_to_vertex: Dict[int, Tuple[Vertex, Vertex]] = {}

        self.edge_count = 0
        visited_edges: Set[int] = set()
        for edge_index, edge in enumerate(edges):
            if edge_index in visited_edges:
                continue
            visited_edges.add(edge_index)
            visited_edges.add(edge.twin)

            start_vert = vertices[edge.start]
            end_vert = vertices[edge.end]
            if edge.twin and edge.is_primary:
                if (Point(round_coord((start_vert.X, start_vert.Y))).intersects(polygon) and
                        Point(round_coord((end_vert.X, end_vert.Y))).intersects(polygon)):
                    if edge.is_linear:
                        line = LineString((
                            round_coord((start_vert.X, start_vert.Y)),
                            round_coord((end_vert.X, end_vert.Y))
                            ))
                        self._store_edge(line)
                    else:
                        cell = cells[edge.cell]
                        cell_twin = cells[edges[edge.twin].cell]

                        geom_points = []
                        geom_edges = []
                        if cell.contains_point:
                            geom_points.append(pv.RetrieveScaledPoint(cell))
                        if cell_twin.contains_point:
                            geom_points.append(pv.RetrieveScaledPoint(cell_twin))
                        if cell.contains_segment:
                            geom_edges.append(pv.RetriveScaledSegment(cell))
                        if cell_twin.contains_segment:
                            geom_edges.append(pv.RetriveScaledSegment(cell_twin))
                        assert len(geom_edges) > 0
                        assert len(geom_points) + len(geom_edges) == 2

                        if len(geom_points) == 1 and len(geom_edges) == 1:
                            start_point = Point(round_coord((start_vert.X, start_vert.Y)))
                            end_point = Point(round_coord((end_vert.X, end_vert.Y)))
                            max_distance = start_point.distance(end_point) / 10 + 0.01
                            points = (round_coord(point)
                                for point in pv.DiscretizeCurvedEdge(edge_index, max_distance))
                            self._store_edge(LineString(points))
                        else:
                            # A parabola between 2 lines (as opposed to 1 line and one point)
                            # leaves the DiscretizeCurvedEdge() function broken sometimes.
                            # Let's just assume a straight line edge in these cases.
                            print("BORKED VORONOI", geom_points, geom_edges)
                            line = LineString((
                                round_coord((start_vert.X, start_vert.Y)),
                                round_coord((end_vert.X, end_vert.Y))
                                ))
                            self._store_edge(line)
                        continue

        # The widest_gap() method checks vertices for the widest point.
        # If we remove this vertex subsequent calls will not work.
        # If we cached the value instead, client code will not be able to use the
        # returned vertex value as an index into this class's data structures.
        widest_point, _ = self.widest_gap()

        self._drop_irrelevant_edges()
        self._combine_edges([widest_point.coords[0]])

        #self._check_data()

    def _check_data(self) -> None:
        """ Sanity check data structures. """
        for edge_i, edge in self.edges.items():
            assert edge_i in self.edge_to_vertex
            vertices = self.edge_to_vertex[edge_i]
            assert len(vertices) == 2
            assert edge.coords[0] in vertices
            assert edge.coords[-1] in vertices
            assert edge.coords[0] != edge.coords[-1] # Loop
            assert vertices[0] in self.vertex_to_edges
            assert vertices[1] in self.vertex_to_edges

        for edge_i, vertices in self.edge_to_vertex.items():
            assert edge_i in self.edges
            assert len(vertices) == 2
            assert edge_i in self.edges
            assert vertices[0] in self.vertex_to_edges
            assert vertices[1] in self.vertex_to_edges

        for vertex, edges_i in self.vertex_to_edges.items():
            assert len(set(edges_i)) == len(edges_i) # Loop
            for edge_i in edges_i:
                assert edge_i in self.edges
                vertices = self.edge_to_vertex[edge_i]
                assert vertex in vertices

    def _store_edge(self, edge: LineString, replace_index=None) -> None:
        if edge.length == 0:
            return

        edge_index = replace_index
        if edge_index is None:
            edge_index = self.edge_count
            self.edge_count += 1
        vert_index_a = (edge.coords[0][0], edge.coords[0][1])
        vert_index_b = (edge.coords[-1][0], edge.coords[-1][1])

        self.edges[edge_index] = edge
        if edge_index not in self.vertex_to_edges:
            self.vertex_to_edges.setdefault(vert_index_a, []).append(edge_index)
        if edge_index not in self.vertex_to_edges:
            self.vertex_to_edges.setdefault(vert_index_b, []).append(edge_index)
        self.edge_to_vertex[edge_index] = (vert_index_a, vert_index_b)

    def _remove_edge(self, edge_index: int) -> None:
        vert_a, vert_b = self.edge_to_vertex[edge_index]
        neibours_a = self.vertex_to_edges[vert_a]
        neibours_b = self.vertex_to_edges[vert_b]

        neibours_a.remove(edge_index)
        neibours_b.remove(edge_index)
        if not neibours_a:
            del self.vertex_to_edges[vert_a]
        if not neibours_b:
            del self.vertex_to_edges[vert_b]
        del self.edge_to_vertex[edge_index]
        del self.edges[edge_index]

    def distance_from_geom(self, point: Point) -> float:
        """
        Distance form nearest edge. Note this edge may be the outer boundary or
        the edge of an island.
        """
        radius = self.max_dist
        for ring in [self.polygon.exterior] + list(self.polygon.interiors):
            nearest = nearest_points(point, ring)
            dist = point.distance(nearest[1])
            radius = min(radius, dist)
        return radius

    def _combine_edges(self, dont_merge: List[Tuple[float, float]]) -> None:
        """ For any vertex with only 2 edges meeting there, combine the edges. """
        to_merge = []
        for vertex, edges_i in self.vertex_to_edges.items():
            if len(edges_i) == 2 and vertex not in dont_merge:
                to_merge.append(vertex)

        while to_merge:
            candidate = to_merge.pop()
            edges_i = self.vertex_to_edges[candidate]
            assert len(edges_i) == 2

            if edges_i[0] not in self.edges or edges_i[1] not in self.edges:
                # Already done.
                continue

            edge_i_a = edges_i[0]
            edge_i_b = edges_i[1]

            edge_a = self.edges[edge_i_a].coords
            edge_b = self.edges[edge_i_b].coords

            if edge_a[0] in [edge_b[0], edge_b[-1]] and edge_a[-1] in [edge_b[0], edge_b[-1]]:
                # Loop with no branches. "Circle on a stick."
                # It wold be possible to combine the last 2 segments but that
                # would confuse linemerge(...) as it would not know which pair
                # of ends to make the beginning and end of the LineString.
                continue

            self._remove_edge(edge_i_a)
            self._remove_edge(edge_i_b)

            edge_combined = linemerge([edge_a, edge_b])
            self._store_edge(edge_combined, edge_i_a)

    def _drop_irrelevant_edges(self) -> None:
        """
        If any geometry resulting from a  voronoi edge will be covered by the
        geometry of some other voronoi edge it is deemed irrelevant and pruned
        by this function.
        """
        to_prune = set()
        for index, edge in self.edges.items():
            vert_a, vert_b = self.edge_to_vertex[index]
            neibours_a = self.vertex_to_edges[vert_a]
            neibours_b = self.vertex_to_edges[vert_b]

            if len(neibours_a) == 1:
                if abs(self.distance_from_geom(Point(vert_b)) - edge.length) < self.tolerence / 2:
                    to_prune.add(index)
                if edge.length < self.tolerence:
                    to_prune.add(index)
            if len(neibours_b) == 1:
                if abs(self.distance_from_geom(Point(vert_a)) - edge.length) < self.tolerence / 2:
                    to_prune.add(index)
                if edge.length < self.tolerence:
                    to_prune.add(index)

        for index in to_prune:
            self._remove_edge(index)

    @timing
    def widest_gap(self) -> Tuple[Point, float]:
        """
        Find the point inside self.polygon but furthest from an edge.

        Returns:
            (Point, float): Point at center of widest point.
                            Radius of circle that fits in widest point.
        """
        max_dist = max((self.polygon.bounds[2] - self.polygon.bounds[0]),
                (self.polygon.bounds[3] - self.polygon.bounds[1])) + 1

        widest_dist = 0
        widest_point = None
        for vertex in self.vertex_to_edges:
            nearest_dist = max_dist
            for ring in [self.polygon.exterior] + list(self.polygon.interiors):
                dist = Point(vertex).distance(ring)
                if dist < nearest_dist:
                    nearest_dist = dist
            if nearest_dist > widest_dist:
                widest_dist = nearest_dist
                widest_point = Point(vertex)
        return (widest_point, widest_dist)


class ToolPath:
    @timing
    def __init__(self, polygon: Polygon, step: float, winding_dir: ArcDir) -> None:
        self.polygon: Polygon = polygon
        self.step: float = step
        self.winding_dir: ArcDir = winding_dir

        self._validate_poly()

        self.remainder: float = 0.0
        self.last_radius = None
        self.voronoi = Voronoi(self.polygon, tolerence = self.step)
        self.start_point: Point
        self.start_radius: float
        self.start_point, self.start_radius = self.voronoi.widest_gap()
        self.cut_area_total = Polygon()
        self.last_circle: Optional[ArcData] = None
        self.arc_fail_count: int = 0
        self.path_fail_count: int = 0
        self.loop_count: int = 0

        self.visited_edges: Set[int] = set()
        self.open_paths = {}

        self.path_data: List[ArcData] = self._walk()
        self.joined_path_data: List[Union[ArcData, LineData]] = self._join_arcs()

    def _validate_poly(self) -> None:
        # TODO: This probably belongs in the Voronoi class.

        fixed = make_valid(self.polygon)
        while fixed.type == "MultiPolygon":
            fixed = fixed.geoms[0]
            fixed = make_valid(fixed)
            # TODO: Should we just throw an exception here?
            # The geometry is not valid. Knowing which piece to work on is a
            # crap shoot.
            # It should be up to the client code to make the decision and correctly
            # format the input polygon.
        if fixed.type == "Polygon":
            self.polygon = fixed

        self.polygon = self.polygon.simplify(0.1)

    def _chose_path_remainder(
            self, closest: Optional[Tuple[float, float]] = None) -> Optional[Tuple[float, float]]:
        """
        Choose a vertex with an un-traveled voronoi edge leading from it.

        Returns:
            A vertex that has un-traveled edges leading from it.
        """
        shortest = self.voronoi.max_dist + 1
        closest_vertex: Optional[Tuple[float, float]] = None
        closest_edge: Optional[int] = None
        to_cleanup = []
        for edge_i, vertex in self.open_paths.items():
            if edge_i in self.visited_edges:
                to_cleanup.append(edge_i)
            else:
                if closest_vertex is None:
                    shortest = self.voronoi.max_dist
                    closest_vertex = vertex
                    closest_edge = edge_i
                else:
                    if closest:
                        dist = Point(vertex).distance(Point(closest))
                    else:
                        dist = 0

                    if dist < shortest:
                        closest_vertex = vertex
                        closest_edge = edge_i
                        shortest = dist

        for edge_i in to_cleanup:
            self.open_paths.pop(edge_i)

        if closest_edge is not None:
            self.open_paths.pop(closest_edge)

        self.last_circle = None
        return closest_vertex

        #start_vertex = None
        #while start_vertex is None and len(self.open_paths) > 0:
        #    edge_i, start_vertex = self.open_paths.popitem()

        #    if edge_i in self.visited_edges:
        #        edge_i = -1
        #        start_vertex = None
        #self.last_circle = None
        #return start_vertex

    @classmethod
    def _colapse_dupe_points(cls, line: LineString) -> Optional[LineString]:
        points = []
        last_point = None
        for point in line.coords:
            if last_point == point:
                continue
            points.append(point)
            last_point = point
        if len(points) < 2:
            return None
        return LineString(points)

    @classmethod
    def _extrapolate_line(cls, extra: float, line: LineString) -> LineString:
        """
        Extend a line at both ends in the same direction as the end coordinate
        pair implies.
        """
        coord_0, coord_1 = line.coords[:2]
        coord_m2, coord_m1 = line.coords[-2:]
        ratio_begin = extra / LineString([coord_0, coord_1]).length
        ratio_end = extra / LineString([coord_m2, coord_m1]).length
        coord_begin = Point(
                coord_0[0] + (coord_0[0] - coord_1[0]) * ratio_begin,
                coord_0[1] + (coord_0[1] - coord_1[1]) * ratio_begin)
        coord_end = Point(
                coord_m1[0] + (coord_m1[0] - coord_m2[0]) * ratio_end,
                coord_m1[1] + (coord_m1[1] - coord_m2[1]) * ratio_end)
        return LineString([coord_begin] + list(line.coords) + [coord_end])

    @classmethod
    def _pid(cls, kp: float, ki: float, kd: float, seed: float
            ) -> Generator[float, Tuple[float, float], None]:
        """
        A PID algorithm used for recursively estimating the position of the best
        fit arc.

        Arguments:
            kp: Propotional multiplier.
            ki: Integral multiplier.
            kd: Derivative multiplier.
        Yields:
            Arguments:
                target: Target step size.
                current: step size resulting from the previous iteration result.
            next distance.
        Returns:
            Never exits Yield loop.
        """
        error_previous: float = 0.0
        integral: float = 0.0
        value: float = seed

        while True:
            target, current = yield value

            error = target - current
            #error = current - target

            prportional = kp * error
            integral += ki * error
            differential = kd * (error - error_previous)

            value = seed + prportional + integral + differential

            error_previous = error

    def _arc_at_distance(self, distance: float, voronoi_edge: LineString) -> Tuple[Point, float]:
        """
        Calculate the center point and radius of the largest arc that fits at a
        set distance along a voronoi edge.
        """
        pos = voronoi_edge.interpolate(distance)
        radius = self.voronoi.distance_from_geom(pos)

        return (pos, radius)

    @classmethod
    def _furthest_spacing_arcs(cls, arcs: List[ArcData], last_circle: ArcData) -> float:
        """
        Calculate maximum step_over between 2 arcs.
        """
        #return self._furthest_spacing_shapely(arcs, Polygon(last_circle.path))

        spacing = -1

        for arc in arcs:
            spacing = max(spacing,
                    last_circle.origin.hausdorff_distance(arc.path) - last_circle.radius)
        return spacing

    @classmethod
    def _furthest_spacing_shapely(
            cls, arcs: List[ArcData], previous: LineString) -> float:
        """
        Calculate maximum step_over between 2 arcs.

        TODO: Current implementation is expensive. Not sure how shapely's distance
        method works but it is likely "O(N)", making this implementation N^2.
        We can likely reduce that to O(N*log(N)) with a binary search.

        Arguments:
            arcs: The new arcs.
            previous: The previous cut path geometry we are testing the arks against.

        Returns:
            The step distance.
        """
        spacing = 0
        polygon = previous
        for arc in arcs:
            # This is expensive but yields good results.
            # Probably want to do a binary search version?

            #for index in range(0, len(arc.path.coords), 1):
            #    coord = arc.path.coords[index]
            #    spacing = max(spacing, Point(coord).distance(polygon))

            for progress in range(32):
                spacing = max(
                        spacing,
                        arc.path.interpolate(float(progress) / 32, True).distance(polygon))

        return spacing

    def _calculate_arc(
            self,
            voronoi_edge: LineString,
            start_distance: float,
            debug: bool = False
            ) -> Tuple[float, List[ArcData]]:
        """
        Calculate the arc that best fits within the path geometry.

        A given point on the voronoi_edge is equidistant between the edges of the
        desired cut path. We can calculate this distance and it forms the radius
        of an arc touching the cut path edges.
        We need the furthers point on that arc to be step distance away from
        the previous arc. It is hard to calculate a point on the voronoi_edge that
        results in the correct spacing between the new and previous arc.

        Rather than work out the new arc centre position with maths, it is quicker
        and easier to use trial and error, moving the proposed centre repeatedly
        and seeing if the arc fits.

        Arguments:
            voronoi_edge: The line of mid-way points between the edges of desired
              cut path.
            start_distance: The distance along voronoi_edge to start trying to
              find an arc that fits.
        Returns:
            A tuple containing:
                1. Distance along voronoi edge of the final arc.
                2. A collection of ArcData objects containing relevant information
                about the arcs generated with an origin the specified distance
                allong the voronoi edge.
        """
        pid = self._pid(0.75, 0, 0, 0)
        #pid = self._pid(0.19, 0.04, 0.12, 0)
        #pid = self._pid(0.9, 0.01, 0.01, 0)
        #pid = self._pid(0, 0.001, 0.3, 0)
        pid.send(None)  # type: ignore

        color_overide = None

        desired_step = self.step
        if (voronoi_edge.length - start_distance) < desired_step:
            desired_step = (voronoi_edge.length - start_distance)

        distance = start_distance + desired_step

        # Strange shapely bug causes ...interpolate(distance) to give wrong
        # results if distance == 0.
        if distance == 0:
            distance = 0.001

        count: int = 0
        circle: Optional[ArcData] = None
        arcs: List[ArcData] = []
        progress: float = 1.0
        best_progress: float = 0.0
        best_distance: float = 0.0
        dist_offset: int = 100000
        log = ""

        # Extrapolate line beyond it's actual distance to give the PID algorithm
        # room to overshoot while converging on an optimal position for the new arc.
        edge_extended: LineString = self._extrapolate_line(dist_offset, voronoi_edge)
        assert abs(edge_extended.length - (voronoi_edge.length + 2 * dist_offset)) < 0.0001

        # Loop multiple times, trying to converge on a distance along the voronoi
        # edge that provides the correct step size.
        while count < ITERATION_COUNT:
            count += 1

            # Propose an arc.
            pos, radius = self._arc_at_distance(distance + dist_offset, edge_extended)
            if radius <= 0:
                # The voronoi edge has met the part geometry.
                # Nothing more to do.
                return (distance, [])
            circle = create_circle(pos, radius, self.winding_dir)

            # Compare proposed arc to cut area.
            # We are only interested in sections that have not been cut yet.
            arcs = arcs_from_circle_diff(
                    circle, self.cut_area_total, self.winding_dir, color_overide)
            if not arcs:
                # arc is entirely hidden by previous cut geometry.
                # Nothing more to do here.
                return (voronoi_edge.length, [])

            if not self.cut_area_total:
                # The very first circle in the whole path.
                # No calculation required. Just draw it.
                break

            # Progress is measured as the furthest point he proposed arc is
            # from the previous one. We are aiming for proposed == desired_step.
            if self.last_circle:
                progress = self._furthest_spacing_arcs(arcs, self.last_circle)
            else:
                progress = self._furthest_spacing_shapely(arcs, self.cut_area_total)

            desired_step = self.step
            if radius < CORNER_ZOOM:
                # Limit step size as the arc radius gets very small.
                #plt.plot(pos.x, pos.y, 'x', c="black")
                multiplier = (CORNER_ZOOM - radius) / CORNER_ZOOM
                desired_step = self.step - CORNER_ZOOM_EFFECT * multiplier

            if (abs(desired_step - progress) < abs(desired_step - best_progress) and
                    distance > 0 and best_distance <= voronoi_edge.length):
                best_progress = progress
                best_distance = distance

            if abs(desired_step - progress) < desired_step / 20:
                # Good enough fit.
                break

            modifier = pid.send((desired_step, progress))
            distance += modifier

            #log += f"\t{start_distance}\t{distance=}\t{progress=}\t{best_progress=}\t{modifier=}\n"

        log += f"\t{progress=}\t{best_progress=}\t{best_distance=}\t{voronoi_edge.length=}\n"
        log += "\t--------"

        if best_distance > voronoi_edge.length:
            if abs(voronoi_edge.length - best_distance) < desired_step / 20:
                # Ignore trivial edge ends.
                return (distance, [])
            best_distance = voronoi_edge.length

        if distance != best_distance:
            distance = best_distance
            progress = best_progress
            pos, radius = self._arc_at_distance(distance + dist_offset, edge_extended)
            circle = create_circle(Point(pos), radius, self.winding_dir)
            arcs = arcs_from_circle_diff(
                    circle, self.cut_area_total, self.winding_dir, color_overide)

        distance_remain = voronoi_edge.length - distance
        if count == ITERATION_COUNT:
            self.arc_fail_count += 1
            print("\tDid not find an arc that fits. Spacing/Desired: "
                    f"{round(progress, 3)}/{desired_step}"
                    "\tdistance remaining: "
                    f"{round(distance_remain, 3)}")

        if count == ITERATION_COUNT or debug:
            print(log)

        self.loop_count += count

        assert circle is not None
        self.last_circle = circle
        self.cut_area_total = self.cut_area_total.union(Polygon(circle.path))

        return (distance, arcs)

    def _join_branches(self, start_vertex: Tuple[float, float]) -> LineString:
        """
        Walk a section of the voronoi edge tree, creating a combined edge as we
        go.

        Returns:
            A LineString object of the combined edges.
        """
        vertex = start_vertex

        line_coords: List[Tuple[float, float]] = []

        while True:
            branches = self.voronoi.vertex_to_edges[vertex]
            candidate = None
            longest = 0
            for branch in branches:
                if branch not in self.visited_edges:
                    self.open_paths[branch] = vertex
                    length = self.voronoi.edges[branch].length
                    if candidate is None:
                        candidate = branch
                    elif BREADTH_FIRST and length < longest:
                        candidate = branch
                    elif not BREADTH_FIRST and length > longest:
                        candidate = branch

                    longest = max(longest, length)

            if candidate is None:
                break

            self.visited_edges.add(candidate)
            edge_coords = self.voronoi.edges[candidate].coords
            if not line_coords:
                if vertex == edge_coords[-1]:
                    edge_coords = edge_coords[::-1]
            else:
                if line_coords[-1] != edge_coords[0]:
                    edge_coords = edge_coords[::-1]
            line_coords += edge_coords

            vertices = self.voronoi.edge_to_vertex[candidate]
            assert vertex in vertices
            if vertex == vertices[0]:
                vertex = vertices[1]
            else:
                vertex = vertices[0]

        line = LineString(line_coords)

        return self._colapse_dupe_points(line)

    def _walk(self) -> List[ArcData]:
        """
        Iterate through voronoi edges.
        For each edge, calculate arc positions along the edge.
        """
        path_data: List[ArcData] = []
        start_vertex: Optional[Tuple[float, float]] = self.start_point.coords[0]

        path_count = 0
        while start_vertex is not None:
            combined_edge = self._join_branches(start_vertex)
            if not combined_edge:
                start_vertex = self._chose_path_remainder()
                continue

            path_count += 1
            print(f"_walk\t{start_vertex}\t{combined_edge.length=}")

            dist = 0.0
            stuck_count = int(combined_edge.length * 10 / self.step + 10)
            closest = 2 * combined_edge.length
            while dist < combined_edge.length and stuck_count:
                if stuck_count % 100 == 1:
                    print(stuck_count)
                stuck_count -= 1
                dist, arcs = self._calculate_arc(combined_edge, dist)

                if abs(combined_edge.length - dist) > closest:
                    # Getting worse.
                    break
                closest = abs(combined_edge.length - dist)

                path_data += arcs

            if stuck_count <= 0:
                print(f"stuck: {round(dist, 2)} / {round(combined_edge.length, 2)}")
                self.path_fail_count += 1

            start_vertex = self._chose_path_remainder(combined_edge.coords[-1])

        assert self.visited_edges == set(self.voronoi.edges.keys())
        assert not self.open_paths
        print(f"{self.arc_fail_count=}\t {self.loop_count=}")
        print(f"{self.path_fail_count=}\t {path_count}")
        return path_data

    @timing
    def _join_arcs(self) -> List[Union[ArcData, LineData]]:
        joined_path_data: List[Union[ArcData, LineData]] = []
        last_arc = None
        for arc in self.path_data:
            if last_arc is not None:
                joined_path_data.append(join_arcs(last_arc, arc, self.cut_area_total))
            joined_path_data.append(arc)
            last_arc = arc

        return joined_path_data



