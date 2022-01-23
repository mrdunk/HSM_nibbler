from typing import Dict, Generator, List, NamedTuple, Optional, Set, Tuple, Union

from collections import namedtuple
import time

from shapely.geometry import LineString, MultiLineString, Point, Polygon
from shapely.ops import linemerge, nearest_points

import matplotlib.pyplot as plt
import pyvoronoi

Vertex = Tuple[float, float]

# Number of tries before we give up trying to find a best-fit arc and just go
# with the best we have found so far.
ITERATION_COUNT = 50

# Resolution of vorinoi algorithm.
# See C++ Boost documentation.
VORONOI_RES = 1000

# Whether to visit short voronoi edges first (True) or try to execute longer
# branches first.
# TODO: We could use more long range path planning that take the shortest total
# path into account.
BREADTH_FIRST = True
#BREADTH_FIRST = False

# Number of decimal places resolution for coordinates.
ROUND_DP = 4

# When arc sizes drop below a certain point, we need to reduce the step size or
# forward motion due to the distance between arcs (step) becomes more than the
# arc diameter.
# This constant is the minimum arc radius size at which we start reducing step size.
# Expressed as a multiple of the step size.
CORNER_ZOOM = 4
CORNER_ZOOM_EFFECT = 2


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
    # TODO: ("widest_at", Optional[Point]),
    # TODO: ("start_DOC", float),
    # TODO: ("end_DOC", float),
    # TODO: ("widest_DOC", float),
    ("path", LineString)
    ])

LineData = NamedTuple("Line", [
    ("start", Optional[Point]),
    ("end", Optional[Point]),
    ("path", LineString),
    ("safe", bool)
    ])

def JoinArcs(start: ArcData, end: ArcData, safe_area: Polygon) -> LineData:
    path = LineString([start.path.coords[0], end.path.coords[-1]])
    safe = path.covered_by(safe_area)
    return LineData(start.start, end.end, path, safe)

def Circle(origin: Point, radius: float, path: Optional[LineString]=None) -> ArcData:
    if path is None:
        return ArcData(origin, radius, None, None, origin.buffer(radius).boundary)
    return ArcData(origin, radius, None, None, path)

def Arc(origin: Point, start: Point, end: Point, path: Optional[LineString]=None) -> ArcData:
    assert origin.distance(start) == origin.distance(end)
    radius = origin.distance(start)
    if path is None:
        line = origin.buffer(radius).split(start)
        line_section = line.split(end)[0]
        return ArcData(origin, radius, start, end, line_section)
    return ArcData(origin, radius, start, end, path)

def arcs_from_circle_diff(circle: ArcData, polygon: Polygon) -> List[ArcData]:
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
        start = Point(arc.coords[0])
        end = Point(arc.coords[-1])
        path = LineString(arc.coords)
        arcs.append(ArcData(circle.origin, circle.radius, start, end, path))
    return arcs


class Voronoi:
    @timing
    def __init__(self, polygon: Polygon, tolerence: float = 0) -> None:
        self.polygon = polygon
        self.tolerence = tolerence

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
                else:
                    geom_primatives.append((prev_point, point))
                    prev_point = point
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
                            max_distance = start_point.distance(end_point) / 10
                            points = (round_coord(point)
                                for point in pv.DiscretizeCurvedEdge(edge_index, max_distance))
                            self._store_edge(LineString(points))
                        else:
                            # A parabola between 2 lines (as opposed to 1 line and one point)
                            # leaves the DiscretizeCurvedEdge() function broken sometimes.
                            # Let's just assume a straight line edge in these cases.
                            line = LineString((
                                round_coord((start_vert.X, start_vert.Y)),
                                round_coord((end_vert.X, end_vert.Y))
                                ))
                            self._store_edge(line)
                        continue

        self._drop_irrelevant_edges()

    def _store_edge(self, edge: LineString) -> None:
        index_a = (edge.coords[0][0], edge.coords[0][1])
        index_b = (edge.coords[-1][0], edge.coords[-1][1])

        if index_a == index_b:
            return

        self.edges[self.edge_count] = edge
        self.vertex_to_edges.setdefault(index_a, []).append(self.edge_count)
        self.vertex_to_edges.setdefault(index_b, []).append(self.edge_count)
        self.edge_to_vertex[self.edge_count] = (index_a, index_b)
        self.edge_count += 1

    def _prune_edge(self, edge_index: int) -> None:
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

    def _drop_irrelevant_edges(self) -> None:
        """
        The voronoi edge meets the edge of the cut at a change in angle of the
        cut. If that corner is open enough to fit a circle, it will get cut as
        part of one of the other voronoi edges so we don't need a vorinoi edge
        into the corner.
        """
        to_prune = []
        for index, edge in self.edges.items():
            vert_a, vert_b = self.edge_to_vertex[index]
            neibours_a = self.vertex_to_edges[vert_a]
            neibours_b = self.vertex_to_edges[vert_b]

            center = None
            if len(neibours_a) == 1:
                center = edge.centroid
            if len(neibours_b) == 1:
                center = edge.centroid

            radius = edge.length / 2
            if (center is not None 
                    and self.polygon.contains(center.buffer(radius - self.tolerence / 2))):
                to_prune.append(index)

        for index in to_prune:
            self._prune_edge(index)

    @timing
    def widest_gap(self) -> Point:
        """ Find the point inside self.polygon but furthest from an edge. """
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
        return widest_point


class ToolPath:
    @timing
    def __init__(self, polygon: Polygon, step: float) -> None:
        self.polygon: Polygon = polygon
        self.step: float = step
        self.remainder: float = 0.0
        self.last_radius = None
        self.max_dist = max((self.polygon.bounds[2] - self.polygon.bounds[0]),
                (self.polygon.bounds[3] - self.polygon.bounds[1])) + 1
        self.voronoi = Voronoi(polygon, tolerence = self.step)
        self.start: Point = self.voronoi.widest_gap()
        self.cut_area_total = Polygon()
        self.last_circle: Optional[ArcData] = None
        self.fail_count: int = 0
        self.loop_count: int = 0

        self.visited_edges: Set[int] = set()
        start_vertex = (self.start.x, self.start.y)

        self.open_paths = {edge: start_vertex for edge in self.voronoi.vertex_to_edges[start_vertex]}

        self.path_data: List[ArcData] = self._walk()
        self.joined_path_data: List[Union[ArcData, LineData]] = self._join_arcs()

    def _chose_path(self, start_vertex) -> Tuple[Tuple[float, float], int]:
        """
        Chose a path onwards from start_vertex.

        globals:
            BREADTH_FIRST = True:
                Try to pick the shortest option without branches.
            BREADTH_FIRST = False:
                Try to pick the longest option without branches.

        Returns:
            Index of the edge onwards form start_vertex.
        """
        choices = self.voronoi.vertex_to_edges[start_vertex]

        # Work out which of the branches onwards from start_vertex is has
        # the longest distance to it's next branch.
        shortest_len = self.max_dist + 1
        shortest_edge = -1
        longest_len = 0
        longest_edge = -1
        for edge_index in choices:
            if edge_index in self.visited_edges:
                continue
            self.open_paths[edge_index] = start_vertex
            edge_length = self._join_edges(start_vertex, edge_index)[0].length
            if edge_length > longest_len:
                longest_len = edge_length
                longest_edge = edge_index
            elif edge_length < shortest_len:
                shortest_len = edge_length
                shortest_edge = edge_index

        if BREADTH_FIRST and shortest_edge >= 0:
            self.open_paths.pop(shortest_edge, None)
            return (start_vertex, shortest_edge)
        elif longest_edge >= 0:
            self.open_paths.pop(longest_edge, None)
            return (start_vertex, longest_edge)

        # There is no adjoining path onwards.
        # Pick one from the list of open branches.
        edge_i = -1
        while edge_i < 0 and len(self.open_paths) > 0:
            edge_i = list(self.open_paths.keys())[0]
            start_vertex = self.open_paths[edge_i]
            del self.open_paths[edge_i]

            if edge_i in self.visited_edges:
                edge_i = -1
        self.last_circle = None
        return (start_vertex, edge_i)

    def _distance_from_geom(self, point: Point) -> float:
        radius = self.max_dist
        for ring in [self.polygon.exterior] + list(self.polygon.interiors):
            nearest = nearest_points(point, ring)
            dist = point.distance(nearest[1])
            radius = min(radius, dist)
        return radius

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
            next distance 
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
        radius = self._distance_from_geom(pos)

        return (pos, radius)

    def _furthest_spacing_arcs(self, arcs: List[ArcData], last_circle: ArcData) -> float:
        """
        Calculate maximum step_over between 2 arcs.
        """
        #return self._furthest_spacing_shapely(arcs, last_circle.path)

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
        for arc in arcs:
            # This is expensive but yields good results.
            # Probably want to do a binary search version?
            for index in range(0, len(arc.path.coords), 2):
                coord = arc.path.coords[index]
                spacing = max(spacing, Point(coord).distance(previous))

        return spacing

    def _calculate_arc(
            self,
            voronoi_edge: LineString,
            start_distance: float,
            debug: bool = False) -> Tuple[float, List[ArcData]]:
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

        step = self.step
        final_run = False
        if (voronoi_edge.length - start_distance) < step:
            step = (voronoi_edge.length - start_distance)
            final_run = True
        distance = start_distance + step

        # Strange shapely bug causes ...interpolate(distance) to give wrong
        # results if distance == 0.
        if distance == 0:
            distance = 0.01

        count: int = 0
        circle: Optional[ArcData] = None
        arcs: List[ArcData] = []
        progress: float = 1.0
        best_progress: float = 0.0
        best_distance: float = 0.0
        dist_offset: int = 100000
        desired_step = step
        log = ""

        # Extrapolate line beyond it's actual distance to give the PID algorithm
        # room to overshoot while converging on an optimal position for the new arc.
        edge_extended: LineString = self._extrapolate_line(dist_offset, voronoi_edge)
        assert abs(edge_extended.length - (voronoi_edge.length + 2 * dist_offset)) < 0.0001

        # Loop multiple times, trying to converge on a distance along the vorinoi
        # edge that provides the correct step size.
        while count < ITERATION_COUNT:
            count += 1

            # Propose an arc.
            pos, radius = self._arc_at_distance(distance + dist_offset, edge_extended)
            if radius <= 0:
                # The voronoi edge has met the part geometry.
                # Nothing more to do.
                return (distance, [])
            circle = Circle(pos, radius)

            # Compare proposed arc to cut area.
            # We are only interested in sections that have not been cut yet.
            arcs = arcs_from_circle_diff(circle, self.cut_area_total)
            if not arcs:
                # arc is entirely hidden by previous cut geometry.
                # Nothing more to do here.
                return (voronoi_edge.length, [])

            if not self.cut_area_total:
                # The very first circle in the whole path.
                # No calculation required. Just draw it.
                break

            # Progress is measured as the furthest point he proposed arc is
            # from the previous one. We are aiming for proposed == step.
            if self.last_circle:
                progress = self._furthest_spacing_arcs(arcs, self.last_circle)
            else:
                progress = self._furthest_spacing_shapely(arcs, self.cut_area_total)

            desired_step = step
            if radius < CORNER_ZOOM:
                # Limit step size as the arc radius gets very small.
                #plt.plot(pos.x, pos.y, 'x', c="black")
                modifier = (radius + CORNER_ZOOM_EFFECT - 1) / (CORNER_ZOOM * CORNER_ZOOM_EFFECT)
                desired_step = max(step * 0.2, step * modifier)

            if (abs(desired_step - progress) < abs(desired_step - best_progress) and
                    distance > 0 and best_distance <= voronoi_edge.length):
                best_progress = progress
                best_distance = distance

            if abs(desired_step - progress) < desired_step / 20:
                # Good enough fit.
                break

            if abs(best_distance - voronoi_edge.length) < desired_step / 20 and progress < desired_step:
                # Have reached end of voronoi edge. Pointless going further.
                break

            modifier = pid.send((desired_step, progress))
            distance += modifier

            #log += f"\t{progress=}\t{best_progress=}\t{modifier=}\n"

        log += f"\t{progress=}\t{best_progress=}\t{best_distance=}\t{voronoi_edge.length=}\n"
        log += "\t--------"

        if best_distance > voronoi_edge.length:
            best_distance = voronoi_edge.length
        if best_distance <= 0:
            best_distance = 0.0001
            log += "\tclamp distance to 0\n"

        if distance != best_distance:
            distance = best_distance
            progress = best_progress
            pos, radius = self._arc_at_distance(distance + dist_offset, edge_extended)
            circle = Circle(Point(pos), radius)
            arcs = arcs_from_circle_diff(circle, self.cut_area_total)

        if debug:
            # Recalculate view of arcs without cut path masked out.
            assert circle
            assert self.last_circle is not None
            arcs = arcs_from_circle_diff(circle, Polygon(self.last_circle))

        distance_remain = voronoi_edge.length - distance
        if count == ITERATION_COUNT:
            self.fail_count += 1
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

        if final_run and abs(distance_remain) < desired_step:
            # Close enough to end.
            distance = voronoi_edge.length

        return (distance, arcs)

    def _join_edges(
            self,
            start: Tuple[float, float],
            edge_index: int
            ) -> Tuple[LineString, Set[int]]:
        """
        Traverse voronoi edges, combining any sections with no branches.

        Returns:
            Tuple containing:
                A LineString object of the combined edges.
                A list of indexes into self.edges of the edges traversed.
        """
        assert start in self.voronoi.edge_to_vertex[edge_index]

        line_parts: List[LineString] = []
        traversed_edges: Set[int] = set()
        head = start
        next_edge_i = edge_index
        while next_edge_i not in self.visited_edges:
            edge = self.voronoi.edges[next_edge_i]
            edge_coords = edge.coords
            if edge_coords[0] != head:
                edge_coords = edge_coords[::-1]

            if line_parts and edge_coords[-1] == line_parts[0].coords[0]:
                # Loop. Stop before saving.
                break

            traversed_edges.add(next_edge_i)
            line_parts.append(edge)

            branches = self.voronoi.vertex_to_edges[edge_coords[-1]]
            if len(branches) != 2:
                break

            if next_edge_i == branches[0]:
                next_edge_i = branches[1]
            else:
                next_edge_i = branches[0]

            if next_edge_i in traversed_edges:
                break

            head = edge_coords[-1]

        line = linemerge(MultiLineString(line_parts))
        if line.coords[0] != start:
            line = LineString(line.coords[::-1])

        #assert line.coords[0] == start
        return (line, traversed_edges)

    def _walk(self) -> List[ArcData]:
        """
        Iterate through vorinoi edges.
        For each edge, calculate arc positions along the edge.
        """
        path_data: List[ArcData] = []
        start_vertex = (self.start.x, self.start.y)
        start_vertex, edge_i = self._chose_path(start_vertex)

        while edge_i >= 0:
            print(f"_walk\t{edge_i}\t{start_vertex}")
            combined_edge, traversed_edges = self._join_edges(start_vertex, edge_i)
            start_vertex = combined_edge.coords[0]
            end_vertex = combined_edge.coords[-1]
            dist = 0.0
            stuck_count = int(combined_edge.length * 20 / self.step + 10)
            while dist < combined_edge.length and stuck_count:
                stuck_count -= 1
                debug = edge_i in []
                dist, arc = self._calculate_arc(combined_edge, dist, debug=debug)
                path_data += arc

            if stuck_count <= 0:
                print(f"stuck: {round(dist, 2)} / {round(combined_edge.length, 2)}")

            self.visited_edges |= traversed_edges
            #assert end_vertex != start_vertex
            start_vertex = end_vertex

            start_vertex, edge_i = self._chose_path(start_vertex)

            #if not self.last_circle:
            #    self.last_circle = self.cut_area_total

        print(f"{self.fail_count=}\t {self.loop_count=}")
        return path_data

    @timing
    def _join_arcs(self) -> List[Union[ArcData, LineData]]:
        joined_path_data: List[Union[ArcData, LineData]] = []
        last_arc = None
        for arc in self.path_data:
            if last_arc is not None:
                joined_path_data.append(JoinArcs(last_arc, arc, self.cut_area_total))
            joined_path_data.append(arc)
            last_arc = arc

        return joined_path_data



