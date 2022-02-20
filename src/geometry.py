"""
A CAM library for generating HSM "peeling" toolpaths from supplied geometry.
"""

# pylint: disable=attribute-defined-outside-init

from typing import Dict, Generator, List, NamedTuple, Optional, Set, Tuple, Union

from enum import Enum
import math
import time

from shapely.geometry import LineString, MultiLineString, Point, Polygon  # type: ignore
from shapely.ops import linemerge  # type: ignore

try:
    from voronoi_centers import VoronoiCenters  # type: ignore
    from helpers import log  # type: ignore
except ImportError:
    from cam.voronoi_centers import VoronoiCenters  # type: ignore
    from cam.helpers import log  # type: ignore


# Number of tries before we give up trying to find a best-fit arc and just go
# with the best we have found so far.
ITERATION_COUNT = 50

# Whether to visit short voronoi edges first (True) or try to execute longer
# branches first.
# TODO: We could use more long range path planning that take the shortest total
# path into account.
#BREADTH_FIRST = True
BREADTH_FIRST = False

# When arc sizes drop below a certain point, we need to reduce the step size or
# forward motion due to the distance between arcs (step) becomes more than the
# arc diameter.
# This constant is the minimum arc radius size at which we start reducing step size.
# Expressed as a multiple of the step size.
#CORNER_ZOOM = 4
#CORNER_ZOOM = 8
CORNER_ZOOM = 0  # Disabled.
CORNER_ZOOM_EFFECT = 0.75
#CORNER_ZOOM_EFFECT = 3


class ArcDir(Enum):
    CW = 0
    CCW = 1
    Closest = 2


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
    ("debug", Optional[str])
])

LineData = NamedTuple("Line", [
    ("start", Optional[Point]),
    ("end", Optional[Point]),
    ("path", LineString),
    ("safe", bool)
])


def join_arcs(start: ArcData, end: ArcData, safe_area: Polygon) -> LineData:
    """
    Generate CAM tool path to join the end of one arc to the beginning of the next.
    """
    path = LineString([start.end, end.start])
    safe = path.covered_by(safe_area)
    return LineData(start.end, end.start, path, safe)


def create_circle(origin: Point, radius: float) -> ArcData:
    """
    Generate a circle that will be split into arcs to be part of the toolpath later.
    """
    span_angle = 2 * math.pi
    return ArcData(
        origin, radius, None, None, 0, span_angle, origin.buffer(radius).boundary, "")


def create_arc_from_path(
        origin: Point,
        winding_dir: ArcDir,
        path_: LineString,
        debug: str = None
) -> ArcData:
    """
    Save data for the arc sections of the path.
    This is called a lot so any optimizations here save us time.
    """
    # Make copy of path since we may need to modify it.
    path = LineString(path_)

    start_coord = path.coords[0]
    end_coord = path.coords[-1]
    start = Point(start_coord)
    end = Point(end_coord)
    mid = path.interpolate(0.5, normalized=True)
    radius = origin.distance(start)
    #assert abs((origin.distance(mid) - radius) / radius) < 0.01
    #assert abs((origin.distance(end) - radius) / radius) < 0.01

    # Breaking these out once rather than separately inline later saves us ~7%
    # CPU time overall.
    org_x, org_y = origin.xy
    start_x, start_y = start_coord
    mid_x, mid_y = mid.xy
    end_x, end_y = end_coord

    start_angle = math.atan2(start_x - org_x[0], start_y - org_y[0])
    end_angle = math.atan2(end_x - org_x[0], end_y - org_y[0])
    mid_angle = math.atan2(mid_x[0] - org_x[0], mid_y[0] - org_y[0])

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
        arcs.append(create_arc_from_path(
            circle.origin, winding_dir, arc, debug))
    return arcs


def _colapse_dupe_points(line: LineString) -> Optional[LineString]:
    """
    Filter out duplicate points.
    TODO: Profile whether a .simplify(0) would be quicker?
    """
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


class ToolPath:
    """
    A CAM library to generate a HSM "peeling" pocketing toolpath.
    """

    def __init__(
            self,
            polygon: Polygon,
            step: float,
            winding_dir: ArcDir,
            generate: bool = False,
            voronoi: Optional[VoronoiCenters] = None
    ) -> None:
        self.step: float = step
        self.winding_dir: ArcDir = winding_dir
        self.generate = generate

        if voronoi is None:
            self.voronoi = VoronoiCenters(polygon)
        else:
            self.voronoi = voronoi
        self.polygon: Polygon = self.voronoi.polygon

        self.calculate_path()

    def _reset(self) -> None:
        """ Cleanup and/or initialise everything. """
        self.start_point: Point
        self.start_radius: float
        self.start_point, self.start_radius = self.voronoi.widest_gap()
        self.current_winding = self.winding_dir
        self._set_winding()

        # Used to detect when an arc is too close to the edge to be worthwhile.
        self.dilated_polygon_boundaries = []
        for ring in [self.polygon.exterior] + list(self.polygon.interiors):
            self.dilated_polygon_boundaries.append(ring.buffer(self.step / 4))

        # Assume starting circle is already cut.
        self.last_circle: Optional[ArcData] = create_circle(
            self.start_point, self.start_radius)
        self.cut_area_total = Polygon(self.last_circle.path)
        self.last_arc: Optional[ArcData] = None

        self.arc_fail_count: int = 0
        self.path_fail_count: int = 0
        self.loop_count: int = 0
        self.worst_oversize_arc: Optional[Tuple[float, float]] = None
        self.worst_undersize_arc: Optional[Tuple[float, float]] = None

        self.visited_edges: Set[int] = set()
        self.open_paths: Dict[int, Tuple[float, float]] = {}

        self.path: List[Union[ArcData, LineData]] = []
        self.joined_path_data = self.path  # TODO: Deprecated.
        self.pending_arc_queues: List[List[ArcData]] = []

        self.path_len_progress: float = 0.0
        self.path_len_total: float = 0.0
        for edge in self.voronoi.edges.values():
            self.path_len_total += edge.length

    def _set_winding(self) -> None:
        if self.winding_dir == ArcDir.Closest:
            if self.current_winding == ArcDir.CW:
                self.current_winding = ArcDir.CCW
            else:
                self.current_winding = ArcDir.CW

    def calculate_path(self) -> None:
        """ Reset path and restart from beginning. """
        # Create the generator.
        generator = self.get_arcs()

        if not self.generate:
            # Don't want to use it as a generator so set it running.
            try:
                next(generator)
            except StopIteration:
                pass

    def _choose_next_path(
            self,
            current_pos: Optional[Tuple[float, float]] = None
    ) -> Optional[Tuple[float, float]]:
        """
        Choose a vertex with an un-traveled voronoi edge leading from it.

        Returns:
            A vertex that has un-traveled edges leading from it.
        """
        # Cleanup.
        for edge_i in self.visited_edges:
            if edge_i in self.open_paths:
                self.open_paths.pop(edge_i)

        shortest = self.voronoi.max_dist + 1
        closest_vertex: Optional[Tuple[float, float]] = None
        closest_edge: Optional[int] = None
        for edge_i, vertex in self.open_paths.items():
            if current_pos:
                dist = Point(vertex).distance(Point(current_pos))
            else:
                dist = 0

            if closest_vertex is None:
                shortest = dist
                closest_vertex = vertex
                closest_edge = edge_i
                if not current_pos:
                    break
            elif dist < shortest:
                closest_vertex = vertex
                closest_edge = edge_i
                shortest = dist

        if closest_edge is not None:
            self.open_paths.pop(closest_edge)

        self.last_circle = None
        return closest_vertex

    @classmethod
    def _extrapolate_line(cls, extra: float, line: LineString) -> LineString:
        """
        Extend a line at both ends in the same direction it points.
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
            kp: Proportional multiplier.
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

    def _furthest_spacing_arcs(self, arcs: List[ArcData], last_circle: ArcData) -> float:
        """
        Calculate maximum step_over between 2 arcs.
        """
        #return self._furthest_spacing_shapely(arcs, last_circle.path)
        spacing = -self.voronoi.max_dist

        for arc in arcs:
            spacing = max(spacing,
                          last_circle.origin.hausdorff_distance(arc.path) - last_circle.radius)

            #for index in range(0, len(arc.path.coords), 1):
            #    coord = arc.path.coords[index]
            #    spacing = max(spacing,
            #            Point(coord).distance(last_circle.origin) - last_circle.radius)

        return abs(spacing)

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
        spacing = -1
        polygon = previous
        for arc in arcs:
            if not arc.path:
                continue

            # This is expensive but yields good results.
            # Probably want to do a binary search version?

            for index in range(0, len(arc.path.coords), 1):
                coord = arc.path.coords[index]
                #spacing = max(spacing, Point(coord).distance(polygon))
                spacing = max(spacing, polygon.distance(Point(coord)))

        return spacing

    def _calculate_arc(
            self,
            voronoi_edge: LineString,
            start_distance: float,
            min_distance: float,
            debug: bool = False
    ) -> Tuple[float, List[ArcData]]:
        """
        Calculate the arc that best fits within the path geometry.

        A given point on the voronoi_edge is equidistant between the edges of the
        desired cut path. We can calculate this distance and it forms the radius
        of an arc touching the cut path edges.
        We need the furthest point on that arc to be desired_step distance away from
        the previous arc. It is hard to calculate a point on the voronoi_edge that
        results in the correct spacing between the new and previous arc.

        The constraints for the new arc are:
        1) The arc must go through the point on the voronoi edge desired_step
          distance from the previous arc's intersection with the voronoi edge.
        2) The arc must be a tangent to the edge of the cut pocket.
          Or put another way: The distance from the center of the arc to the edge
          of the cut pocket should be the same as the distance from the center of
          the arc to the point described in 1).

        Rather than work out the new arc centre position with maths, it is quicker
        and easier to use a binary search, moving the proposed centre repeatedly
        and seeing if the arc fits.

        Arguments:
            voronoi_edge: The line of mid-way points between the edges of desired
              cut path.
            start_distance: The distance along voronoi_edge to start trying to
              find an arc that fits.
            min_distance: Do not return arcs below this distance; The algorithm
              is confused and traveling backwards.
        Returns:
            A tuple containing:
                1. Distance along voronoi edge of the final arc.
                2. A collection of ArcData objects containing relevant information
                about the arcs generated with an origin the specified distance
                allong the voronoi edge.
        """
        # A full PID algorithm is provided for estimating the center of the arc
        # but so far all test cases have been solved with only the Proportional
        # parameter enabled.
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
        # if distance == 0:
        #    distance = 0.001

        count: int = 0
        circle: Optional[ArcData] = None
        arcs: List[ArcData] = []
        progress: float = 0.0
        best_progress: float = 0.0
        best_distance: float = 0.0
        dist_offset: int = 100000
        log_arc = ""

        # Extrapolate line beyond it's actual distance to give the PID algorithm
        # room to overshoot while converging on an optimal position for the new arc.
        edge_extended: LineString = self._extrapolate_line(
            dist_offset, voronoi_edge)
        assert abs(edge_extended.length -
                   (voronoi_edge.length + 2 * dist_offset)) < 0.0001

        assert self.cut_area_total

        # Loop multiple times, trying to converge on a distance along the voronoi
        # edge that provides the correct step size.
        while count < ITERATION_COUNT:
            count += 1

            # Propose an arc.
            pos, radius = self._arc_at_distance(
                distance + dist_offset, edge_extended)
            circle = create_circle(pos, radius)

            # Compare proposed arc to cut area.
            # We are only interested in sections that have not been cut yet.
            arcs = arcs_from_circle_diff(
                circle, self.cut_area_total, self.current_winding, color_overide)
            if not arcs:
                # arc is entirely hidden by previous cut geometry.
                # Don't record it as an arc that needs drawn.
                arcs = [circle]
                self.last_circle = circle
                return(distance, [])

            # Progress is measured as the furthest point he proposed arc is
            # from the previous one. We are aiming for proposed == desired_step.
            if self.last_circle:
                progress = self._furthest_spacing_arcs(arcs, self.last_circle)
            else:
                progress = self._furthest_spacing_shapely(arcs, self.cut_area_total)

            desired_step = self.step
            if radius < CORNER_ZOOM:
                # Limit step size as the arc radius gets very small.
                multiplier = (CORNER_ZOOM - radius) / CORNER_ZOOM
                desired_step = self.step - CORNER_ZOOM_EFFECT * multiplier

            if (abs(desired_step - progress) < abs(desired_step - best_progress)
                    #and distance > 0
                    ):
                # Better fit.
                best_progress = progress
                best_distance = distance

                if abs(desired_step - progress) < desired_step / 20:
                    # Good enough fit.
                    best_progress = progress
                    best_distance = distance
                    break

            modifier = pid.send((desired_step, progress))
            distance += modifier

            #log_arc += f"\t{start_distance}\t{distance}\t{progress}\t{best_progress}\t{modifier}\n"

        log_arc += f"progress: {best_progress}\tdistance: {best_distance}\tlength: {voronoi_edge.length}\n"
        log_arc += "\t--------"

        if count == ITERATION_COUNT:
            color_overide = "red"
            if distance < min_distance:
                # Moving the wrong way along the voronoi edge.
                # Only happens when we've been to the end of an edge already.
                return (voronoi_edge.length, [])

        if best_distance > voronoi_edge.length:
            best_distance = voronoi_edge.length

        if distance != best_distance or progress != best_progress or color_overide is not None:
            distance = best_distance
            progress = best_progress
            pos, radius = self._arc_at_distance(
                distance + dist_offset, edge_extended)
            circle = create_circle(Point(pos), radius)
            arcs = arcs_from_circle_diff(
                circle, self.cut_area_total, self.current_winding, color_overide)

        distance_remain = voronoi_edge.length - distance
        if count == ITERATION_COUNT:
            # Log some debug data.
            self.arc_fail_count += 1
            log("\tDid not find an arc that fits. Spacing/Desired: "
                f"{round(progress, 3)}/{desired_step}"
                "\tdistance remaining: "
                f"{round(distance_remain, 3)}")
            if progress < desired_step:
                if (self.worst_undersize_arc is None or
                        abs(progress - desired_step) >
                        abs(self.worst_undersize_arc[0] - self.worst_undersize_arc[1])):
                    self.worst_undersize_arc = (progress, desired_step)
            if progress > desired_step:
                if (self.worst_oversize_arc is None or
                        abs(progress - desired_step) >
                        abs(self.worst_oversize_arc[0] - self.worst_oversize_arc[1])):
                    self.worst_oversize_arc = (progress, desired_step)

        if count == ITERATION_COUNT or debug:
            log(log_arc)

        self.loop_count += count

        assert circle is not None
        self.last_circle = circle
        self.cut_area_total = self.cut_area_total.union(Polygon(circle.path))

        self._set_winding()

        filtered_arcs = []
        for arc in arcs:
            if self._filter_arc(arc):
                filtered_arcs.append(arc)
        return (distance, filtered_arcs)

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
                line_coords = edge_coords
                if start_vertex != line_coords[0]:
                    line_coords = line_coords[::-1]
            else:
                if line_coords[-1] == edge_coords[-1]:
                    edge_coords = edge_coords[::-1]
                assert line_coords[0] == start_vertex
                assert line_coords[-1] == edge_coords[0]
                line_coords = list(line_coords) + list(edge_coords)

            vertex = line_coords[-1]

        line = LineString(line_coords)

        return _colapse_dupe_points(line)

    def _arcs_to_path(self, arcs: List[ArcData]) -> None:
        """
        Process list list of arcs, calculate tool path to join one to the next
        and apply them to the self.path parameter.

        Note: This function modifies the arcs parameter in place.
        """
        while arcs:
            arc = arcs.pop(0)
            if arc is None:
                continue
            if self.last_arc is not None:
                self.path.append(join_arcs(self.last_arc, arc, self.cut_area_total))
                self.path.append(arc)
            self.last_arc = arc

    def _get_arcs(self, timeslice: int = 0):
        # TODO: Deprecated. remove.
        return self.get_arcs(timeslice)

    def get_arcs(self, timeslice: int = 0):
        """
        A generator method to create the path.

        Class instance properties:
            self.generate: bool: Whether or not to yield.
                False: Do not yield. Generate all data in one shot.
                True: Yield an estimated ratio of path completion.

        Arguments:
            timeslice: int: How long to generate arcs for before yielding (ms).
        """
        start_time = round(time.time() * 1000)  # ms
        self._reset()

        assert self.last_circle
        self._queue_arcs([
            create_arc_from_path(
                self.start_point, self.current_winding, self.last_circle.path)
            ])

        start_vertex: Optional[Tuple[float, float]
                               ] = self.start_point.coords[0]

        while start_vertex is not None:
            combined_edge = self._join_branches(start_vertex)
            if not combined_edge:
                start_vertex = self._choose_next_path()
                continue

            dist = 0.0
            best_dist = dist
            stuck_count = int(combined_edge.length * 10 / self.step + 10)
            while dist < combined_edge.length and stuck_count > 0:
                stuck_count -= 1
                dist, new_arcs = self._calculate_arc(combined_edge, dist, best_dist)

                self.path_len_progress -= best_dist
                self.path_len_progress += dist

                if dist < best_dist:
                    # Getting worse not better or staying the same.
                    # This can happen legitimately but is an indication we may be
                    # stuck.
                    stuck_count = int(stuck_count / 2)
                else:
                    best_dist = dist

                self._queue_arcs(new_arcs)

                if timeslice >= 0 and self.generate:
                    now = round(time.time() * 1000)  # (ms)
                    if start_time + timeslice < now:
                        yield min(0.999, self.path_len_progress / self.path_len_total)
                        start_time = round(time.time() * 1000)  # (ms)

            self._flush_arc_queues()

            if stuck_count <= 0:
                print(
                    f"stuck: {round(dist, 2)} / {round(combined_edge.length, 2)}")
                self.path_fail_count += 1

            start_vertex = self._choose_next_path(combined_edge.coords[-1])

        if timeslice and self.generate:
            yield 1.0

        assert not self.open_paths
        log(f"loop_count: {self.loop_count}")
        log(f"arc_fail_count: {self.arc_fail_count}")
        log(f"len(path): {len(self.path)}")
        log(f"path_fail_count: {self.path_fail_count}")

    def _flush_arc_queues(self) -> None:
        while self.pending_arc_queues:
            to_process = self.pending_arc_queues.pop(-1)
            self._arcs_to_path(to_process)

    def _queue_arcs(self, new_arcs: List[ArcData]) -> None:
        """
        When an arc intersects with an area that has already been cut the arc may
        get split into multiple pieces.
        When we cone to join the arcs we want to join the "left" arcs to each other
        and the "right" arcs to each other. (There may be more than 2 sets as well.)
        To do this we need to store arcs in separate queues. Each queue contains
        arcs that should be joined to each other.
        """

        if len(new_arcs) != len(self.pending_arc_queues):
            # Number of arcs has changed.
            # A path of spilt arcs has either started or ended.
            self._flush_arc_queues()

        if not new_arcs:
            return

        if not self.pending_arc_queues:
            # Create empty queues.
            for _ in new_arcs:
                self.pending_arc_queues.append([])

        if len(new_arcs) == 1:
            # Nothing complicated to do. Only one line of arcs so only one queue.
            self.pending_arc_queues[0] += new_arcs
        else:
            if len(self.pending_arc_queues[0]) == 0:
                # Queues are all empty. Doesn't matter what goes where.
                for arc_i, arc in enumerate(new_arcs):
                    self.pending_arc_queues[arc_i].append(arc)
                return

            # Need to put each arc in the queue with nearest predecessor.
            for arc in new_arcs:
                closest_queue = None
                closest_dist = None
                for queue in self.pending_arc_queues:
                    dist = arc.path.distance(queue[0].path)
                    if closest_dist is None or dist < closest_dist:
                        closest_dist = dist
                        closest_queue = queue
                assert closest_queue is not None
                closest_queue.append(arc)

            # If queues have ended up different sizes, we have failed to identify
            # the closest arcs to the queues.
            # Flush everything to be safe.
            for queue_index in range(1, len(self.pending_arc_queues)):
                queue = self.pending_arc_queues[queue_index]
                if len(queue) != len(self.pending_arc_queues[0]):
                    self._flush_arc_queues()

    def _filter_arc(self, arc: ArcData) -> Optional[ArcData]:
        """
        Remove any arc that is very close to the edge of the part in it's entirety.
        """
        if len(arc.path.coords) < 3:
            return None
        poly_arc = Polygon(arc.path)
        for ring in self.dilated_polygon_boundaries:
            if ring.contains(Polygon(poly_arc)):
                return None
        return arc

