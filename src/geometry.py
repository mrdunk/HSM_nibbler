"""
A CAM library for generating HSM "peeling" toolpaths from supplied geometry.
"""

# pylint: disable=attribute-defined-outside-init

from typing import Dict, Generator, List, NamedTuple, Optional, Set, Tuple, Union

from enum import Enum
import math
import time

from shapely.affinity import rotate  # type: ignore
from shapely.geometry import box, LinearRing, LineString, MultiLineString, MultiPoint, MultiPolygon, Point, Polygon  # type: ignore
from shapely.ops import linemerge, split, unary_union  # type: ignore

try:
    from debug import display
    from voronoi_centers import (
            round_coord, start_point_perimeter, start_point_widest, VoronoiCenters)  # type: ignore
    from helpers import log  # type: ignore
except ImportError:
    try:
        from cam.debug import display  # type: ignore
    except ImportError:
        pass
    from cam.voronoi_centers import (  # type: ignore
            round_coord, start_point_perimeter, start_point_widest, VoronoiCenters)
    from cam.helpers import log  # type: ignore

# Filter arcs that are entirely within this distance of a pocket edge.
SKIP_EDGE_ARCS = 1 / 20

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
# This constant is the minimum arc radius, expressed as a multiple of the overlap
# size, size at which we start reducing step size.
CORNER_ZOOM = 2.0
# This constant is how much effect the feature will have. A value of "1" will
# keep the distance between each arc center proportional to the arc size.
CORNER_ZOOM_EFFECT = 1.0


class ArcDir(Enum):
    CW = 0
    CCW = 1
    Closest = 2

class MoveStyle(Enum):
    RAPID_OUTSIDE = 0
    RAPID_INSIDE = 1
    CUT = 2

class StartPointTactic(Enum):
    PERIMETER = 0  # Starting point hint for outer peel. On the outer perimeter.
    PERIMITER = 0  # Typo. TODO: remove.
    WIDEST = 1     # Starting point hint for inner pockets.


ArcData = NamedTuple("Arc", [
    ("origin", Point),
    ("radius", Optional[float]),
    ("start", Optional[Point]),
    ("end", Optional[Point]),
    ("start_angle", Optional[float]),
    ("span_angle", Optional[float]),
    ("winding_dir", Optional[ArcDir]),
    # TODO: ("widest_at", Optional[Point]),
    # TODO: ("start_DOC", float),
    # TODO: ("end_DOC", float),
    # TODO: ("widest_DOC", float),
    ("path", LineString),
    ("debug", Optional[str])
])

LineData = NamedTuple("Line", [
    ("start", Point),
    ("end", Point),
    ("path", LineString),
    ("move_style", MoveStyle),
])


def clean_linear_ring(ring: LinearRing) -> LinearRing:
    """ Remove duplicate points in a LinearRing. """
    new_ring = []
    prev_point = None
    first_point = None
    for point in ring.coords:
        if first_point is None:
            first_point = point
        if point == prev_point:
            continue
        else:
            new_ring.append(point)
            prev_point = point
    assert prev_point == first_point  # This is a loop.

    return LinearRing(new_ring)

def clean_polygon(polygon: Polygon) -> Polygon:
    exterior = clean_linear_ring(polygon.exterior)
    holes = []
    for hole in polygon.interiors:
        holes.append(clean_linear_ring(hole))

    return Polygon(exterior, holes=holes)

def clean_multipolygon(multi: MultiPolygon) -> MultiPolygon:
    if multi.type != "MultiPolygon":
        multi = MultiPolygon([multi])
    polygons = []
    for polygon in multi.geoms:
        polygons.append(clean_polygon(polygon))

    return MultiPolygon(polygons)

def create_circle(origin: Point, radius: float, winding_dir: Optional[ArcDir] = None) -> ArcData:
    """
    Generate a circle that will be split into arcs to be part of the toolpath later.
    """
    span_angle = 2 * math.pi
    return ArcData(
        origin, radius, None, None, 0, span_angle, winding_dir, origin.buffer(radius).boundary, "")

def create_arc(
        origin: Point,
        radius: float,
        start_angle: float,
        span_angle: float,
        winding_dir: ArcDir) -> Optional[ArcData]:
    """
    Generate a arc.

    Args:
        origin: Center of arc.
        radius: Radius of arc.
        start_angle: Angle from vertical. (Clockwise)
        span_angle: Angular length of arc.
    """
    if radius == 0:
        return None

    start_angle = start_angle % (2 * math.pi)

    circle_path = origin.buffer(radius).boundary

    line_up = LineString([origin, Point(origin.x, origin.y + radius * 2)])
    circle_path = split(circle_path, line_up)
    points = circle_path.geoms[1].coords[:] + circle_path.geoms[0].coords[:]
    circle_path = LineString(points)

    right_border = rotate(line_up, -span_angle, origin=origin, use_radians=True)
    arc_paths = split(circle_path, right_border).geoms[0]

    arc_paths = rotate(arc_paths, -start_angle, origin=origin, use_radians=True)
    return ArcData(
            origin,
            radius,
            Point(arc_paths.coords[0]),
            Point(arc_paths.coords[-1]),
            start_angle,
            span_angle,
            winding_dir,
            arc_paths,
            "")

def create_arc_from_path(
        origin: Point,
        path: LineString,
        radius: float,
        winding_dir: Optional[ArcDir] = None,
        debug: str = None
) -> ArcData:
    """
    Save data for the arc sections of the path.
    """
    start = path.coords[0]
    end = path.coords[0]
    start_angle = None
    span_angle = None

    return ArcData(origin, radius, start, end, start_angle, span_angle, winding_dir, path, debug)

def mirror_arc(
        arc_data: ArcData,
        winding_dir: ArcDir
        ) -> ArcData:
    """
    Mirror X axis of an arc about the origin point.
    """
    arc_paths = LineString([(-point[0], point[1]) for point in arc_data.path.coords][::-1])
    origin = Point(-arc_data.origin.x, arc_data.origin.y)
    return ArcData(
            origin,
            arc_data.radius,
            Point(arc_paths.coords[0]),
            Point(arc_paths.coords[-1]),
            -arc_data.start_angle % (math.pi * 2),
            -arc_data.span_angle,
            winding_dir,
            arc_paths,
            arc_data.debug)

def complete_arc(
        arc_data: ArcData,
        winding_dir_: ArcDir = None
        ) -> Optional[ArcData]:
    """
    Calculate start_angle, span_angle and radius.
    Fix start, end and path direction based on winding_dir.
    This is called a lot so any optimizations here save us time.
    Given some properties of an arc, calculate the others.
    """
    winding_dir = winding_dir_
    if winding_dir is None or winding_dir == ArcDir.Closest:
        winding_dir = ArcDir.CW

    # Make copy of path since we may need to modify it.
    path = LineString(arc_data.path)
    if path.length == 0.0:
        return None

    start_coord = path.coords[0]
    end_coord = path.coords[-1]
    mid = path.interpolate(0.5, normalized=True)

    # Breaking these out once rather than separately inline later saves us ~7%
    # CPU time overall.
    org_x, org_y = arc_data.origin.xy
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
        start_angle, end_angle = end_angle, start_angle
        start_coord, end_coord = end_coord, start_coord

    if winding_dir == ArcDir.CW:
        span_angle = (end_angle - start_angle) % (2 * math.pi)
    elif winding_dir == ArcDir.CCW:
        span_angle = (-(start_angle - end_angle) % (2 * math.pi)) - (2 * math.pi)

    if span_angle == 0.0:
        span_angle = 2 * math.pi

    radius = arc_data.radius or arc_data.origin.distance(Point(path.coords[0]))

    return ArcData(
            arc_data.origin,
            radius,
            Point(start_coord),
            Point(end_coord),
            start_angle % (2 * math.pi),
            span_angle,
            winding_dir,
            path,
            arc_data.debug)


def arcs_from_circle_diff(
        circle: ArcData,
        already_cut: Polygon,
        debug: str = None
        ) -> List[ArcData]:
    """ Return any sections of circle that do not overlap already_cut. """
    if not already_cut:
        return [circle]
    if circle is None:
        return None

    line_diff = circle.path.difference(already_cut)
    if not line_diff:
        return []
    if line_diff.type == "MultiLineString":
        line_diff = linemerge(line_diff)
    if line_diff.type != "MultiLineString":
        line_diff = MultiLineString([line_diff])

    arcs = []
    assert circle.radius is not None
    for arc in line_diff.geoms:
        arcs.append(create_arc_from_path(
            circle.origin, arc, circle.radius, winding_dir=circle.winding_dir, debug=debug))
            #circle.origin, arc, circle.radius, debug=debug))
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


class BaseGeometry:
    # TODO: Can we get by with only cut_area_total2?
    # Cut area so far, calculated by appending full circles.
    cut_area_total: Polygon
    # Cut area so far, calculated by appending arc segments, dilated by (step / 2).
    cut_area_total2: Polygon

    starting_cut_area: Polygon

    def __init__(
            self,
            to_cut: Polygon,
            step: float,
            winding_dir: ArcDir,
            already_cut: Polygon = None,
    ) -> None:
        self.polygon: Polygon = to_cut
        self.step: float = step
        self.winding_dir: ArcDir = winding_dir
        if already_cut is None:
            already_cut = Polygon()
        self.starting_cut_area = already_cut
        self.path: List[Union[ArcData, LineData]] = []

    def _reset(self) -> None:
        self.pending_arc_queues: List[List[ArcData]] = []
        self.last_arc: Optional[ArcData] = None
        self.dilated_polygon = self.polygon.buffer(self.step / 20)

        self.cut_area_total = self.starting_cut_area
        self.cut_area_total2 = self.starting_cut_area.buffer(self.step / 2)

    def _flush_arc_queues(self) -> None:
        while self.pending_arc_queues:
            to_process = self.pending_arc_queues.pop(0)
            self._arcs_to_path(to_process)

    def _check_queue_overlap(self, arc: ArcData, queue_index: int) -> bool:
        """
        Arcs should only be added if the area inside them has been cleared.
        This checks for overlaps between the specified arc and those that are
        to be cut /after/ it.
        Returns:
            True: If no overlap occurs.
            False: If the proposed arc is blocked for this queue by as yet uncut arcs.
        """
        dilated_arc = arc.path.buffer(self.step)
        for queue in self.pending_arc_queues[queue_index + 1:]:
            for existing_arc in queue:
                if existing_arc.path.intersects(dilated_arc):
                    return False
        return True

    def _queue_arcs(self, new_arcs: List[ArcData]) -> None:
        """
        When an arc intersects with an area that has already been cut the arc may
        get split into multiple pieces.
        When we cone to join the arcs we want to join the "left" arcs to each other
        and the "right" arcs to each other. (There may be more than 2 sets as well.)
        To do this we need to store arcs in separate queues. Each queue contains
        arcs that should be joined to each other.
        """

        # Need to put each arc in the queue with nearest predecessor.
        modified_queues_indexes = set()

        if len(new_arcs) == 1 and len(self.pending_arc_queues) == 1:
            # Optimization to save expensive repeated distance calculations.
            # Only a single queue and a single arc so we don't need complicated
            # queues.
            # Just add the arc to the single queue.
            # Since there are no other queues depending on our remaining one,
            # it is safe to drain it,
            self.pending_arc_queues[0].append(new_arcs[0])
            to_process = self.pending_arc_queues.pop(0)
            self._arcs_to_path(to_process)
            return
        else:
            closest_queue: Optional[List[ArcData]]
            for arc in new_arcs:
                #arc = complete_arc(arc, self.winding_dir)
                closest_queue = None
                closest_queue_index = None
                closest_dist = self.step
                for queue_index, queue in enumerate(self.pending_arc_queues):
                    if self.winding_dir == ArcDir.Closest:
                        combined_dist = 1
                        seperate_dist = 0
                    else:
                        assert arc.start is not None
                        assert queue[-1].start is not None
                        assert queue[-1].end is not None

                        seperate_dist = (
                                queue[-1].start.distance(queue[-1].end) + arc.start.distance(arc.end))
                        combined_dist = (
                                queue[-1].start.distance(arc.end) + queue[-1].end.distance(arc.start))

                    seperation = arc.path.distance(queue[-1].path)
                    if seperation < closest_dist or combined_dist < seperate_dist:
                        if self._check_queue_overlap(arc, queue_index):
                            closest_dist = seperation
                            closest_queue = queue
                            closest_queue_index = queue_index
                if closest_queue is None:
                    # Not close to any predecessor. Create new queue.
                    closest_queue = []
                    closest_queue_index = len(self.pending_arc_queues)
                    self.pending_arc_queues.append(closest_queue)
                closest_queue.append(arc)
                modified_queues_indexes.add(closest_queue_index)
                assert closest_queue_index is not None
                assert closest_queue is self.pending_arc_queues[closest_queue_index]
                assert arc in closest_queue

        # Queues need processed in the order they were created: FIFO.
        # It is only safe to process the oldest queue (index: 0) as any younger
        # queue may be a child of it.
        # TODO: If we really wanted to, whenever there are any un-modified queues,
        # we could process the contents of all the older queues even though they
        # are still being appended to before processing the un-modifies queue(s).
        if modified_queues_indexes and 0 not in modified_queues_indexes:
            to_process = self.pending_arc_queues.pop(0)
            self._arcs_to_path(to_process)

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

            winding_dir = self.winding_dir
            if winding_dir == ArcDir.Closest:
                if self.last_arc is None:
                    winding_dir = ArcDir.CW
                else:
                    # TODO: We could improve this: Rather that taking the opposite
                    # of the last arc, we could work out the closest end based on
                    # the last drawn arc.
                    if self.last_arc.winding_dir == ArcDir.CCW:
                        winding_dir = ArcDir.CW
                    else:
                        winding_dir = ArcDir.CCW

                    assert self.last_arc.start is not None
                    assert self.last_arc.end is not None

                    if self.last_arc.end.distance(arc.start) < self.last_arc.end.distance(arc.end):
                        winding_dir = ArcDir.CW
                    if self.last_arc.start.distance(arc.end) < self.last_arc.end.distance(arc.end):
                        winding_dir = ArcDir.CW

                arc_ = complete_arc(arc, winding_dir)
                if arc_ is None:
                    continue
                arc = arc_

            assert arc.path.length > 0
            assert len(arc.path.coords) >= 2
            assert arc.span_angle != 0

            if self.last_arc is not None:
                self.path += self.join_arcs(arc)
            self.path.append(arc)
            # This union takes up ~5% of processing time of the whole algorithm.
            # TODO: Only truncated arcs really need the whole check in 'join_arcs(...)'.
            # We could tag arcs that need the detailed check and use shapely's
            # unary_union(...) here for the others.
            self.cut_area_total2 = self.cut_area_total2.union(arc.path.buffer(self.step / 2))

            self.last_arc = arc

    def join_arcs(self, next_arc: ArcData) -> List[LineData]:
        """
        Generate CAM tool path to join the end of one arc to the beginning of the next.
        """
        assert self.last_arc
        lines = []
        path = LineString([self.last_arc.end, next_arc.start])
        dilation = (self.step / 2) - (self.step / 20)

        inside_pocket = (
                path.covered_by(self.dilated_polygon)
                or path.covered_by(self.cut_area_total2.buffer(-dilation))
                )

        if inside_pocket:
            # Whole path is inside pocket and is already cut.
            not_cut_path_area = (path.buffer(self.step / 2).
                    difference(self.cut_area_total2).buffer(dilation))
            split_path = split(path, not_cut_path_area)

            for part in split_path.geoms:
                assert part.type == "LineString"

                move_style = MoveStyle.RAPID_INSIDE
                if part.intersects(not_cut_path_area):
                    move_style = MoveStyle.CUT

                lines.append(LineData(
                    Point(part.coords[0]), Point(part.coords[-1]), part, move_style))
            # Shapely paths are not particularly accurate.
            # Clamp endpoints on actual arcs.
            lines[0] = LineData(
                    self.last_arc.end,
                    lines[0].end,
                    lines[0].path,
                    lines[0].move_style)
            lines[-1] = LineData(
                    lines[-1].start,
                    next_arc.start,
                    lines[-1].path,
                    lines[-1].move_style)
        else:
            # Path is not entirely inside pocket or crosses uncut area.
            move_style = MoveStyle.RAPID_OUTSIDE
            lines.append(LineData(self.last_arc.end, next_arc.start, path, move_style))

        return lines

    def _split_arcs(self, full_arcs: List[ArcData]) -> List[ArcData]:
        """
        When an arcs pass through an already cut area, it should be trimmed to
        remove the already cut section.
        It's important to keep the sequence of the order of the resulting sub arcs
        the same as the originals so the final path can pass through them in order.
        """
        split_arcs = []
        for full_arc in full_arcs:
            new_arcs = arcs_from_circle_diff(full_arc, self.cut_area_total)
            if not new_arcs:
                continue

            new_arcs = list(filter(
                None, [
                    complete_arc(new_arc, new_arc.winding_dir)
                    for new_arc in new_arcs if new_arc.path.length]))

            arc_set = set()
            for new_arc_index, new_arc in enumerate(new_arcs):
                arc_set.add((full_arc.path.project(new_arc.start), new_arc_index))

            for _, new_arc_index in sorted(arc_set):
                split_arcs.append(new_arcs[new_arc_index])

        return split_arcs


class Pocket(BaseGeometry):
    """
    A CAM library to generate a HSM "peeling" pocketing toolpath.
    """

    start_point: Point
    max_starting_radius: float
    starting_angle: Optional[float] = None
    last_arc: Optional[ArcData]
    last_circle: Optional[ArcData]
    debug: bool = False

    def __init__(
            self,
            to_cut: Polygon,
            step: float,
            winding_dir: ArcDir,
            already_cut: Polygon = None,
            generate: bool = False,
            voronoi: Optional[VoronoiCenters] = None,
            starting_point_tactic: StartPointTactic = StartPointTactic.WIDEST,
            starting_point: Point = None,
            starting_radius: float = None,
            do_starting_circle: bool = True,  # TODO: Remove
            debug: bool = False,
    ) -> None:
        """
        Arguments:
            to_cut: The area this pocket should cover.
            step: The distance between cutting path passes.
            winding_dir: Tactic for choosing clockwise or anticlockwise path sections.
            already_cut: An area where no cutting path should be generated but
                also no care needs to be taken when intersecting it. Also rapid moves
                may pass through it.
            generate: Whether to use the get_arcs(...) parameter as a generator
                function (True) or calculate the whole path at the start (False).
            voronoi: Optionally pass in a voronoi diagram detailing points equidistant
                from the part edges. If not provided, one will be created.
            starting_point_tactic: Tactic used to pick the very start of the cutting
                path.
            starting_point: Override the automatically calculated cutting path start point.
                Must be withing either the `to_cut` area or the `already_cut` area if
                one was specified.
            starting_radius: Optionally set the required space at the start_point.
                Used to make sure an entry helix has enough space at the start_point
                when machining.

        Properties:
            start_point: The start of the cutting path. If one is not specified with
                the starting_point parameter, this will be calculated automatically.
            max_starting_radius: The maximum radius a circle at start_point could
                be without overlapping the part's edges.
            starting_angle: `None` if the entry circle is completely within the already_cut
                area. Otherwise it contains the angle to the start of the cutting path
                where the entry circle intersects the cutting path.
        """
        if already_cut is None:
            already_cut = Polygon()
        else:
            already_cut = already_cut
        complete_pocket = already_cut.union(to_cut)
        to_cut = complete_pocket.difference(already_cut.buffer(step / 20))

        clean_multipolygon(to_cut)
        clean_multipolygon(complete_pocket)
        clean_multipolygon(already_cut)

        self.starting_cut_area = already_cut
        self.starting_radius = starting_radius
        self.generate = generate
        self.do_starting_circle = do_starting_circle
        self.debug = debug

        # Calculate voronoi diagram for finding points equidistant between part edges.
        if voronoi is None:
            if starting_point is not None:
                voronoi = VoronoiCenters(complete_pocket, starting_point=starting_point)
            elif starting_point_tactic == StartPointTactic.WIDEST:
                voronoi = start_point_widest(
                        self.starting_radius, step, complete_pocket, self.starting_cut_area)
            elif starting_point_tactic == StartPointTactic.PERIMETER:
                voronoi = start_point_perimeter(
                        self.starting_radius or step, complete_pocket, self.starting_cut_area)

            assert voronoi is not None
        self.voronoi = voronoi

        # Calculate entry hole settings.
        self.max_starting_radius = voronoi.max_starting_radius
        if starting_radius and self.max_starting_radius < starting_radius:
            print(f"Warning: Starting radius of {starting_radius} overlaps boundaries "
                  f"at {voronoi.start_point}")
            print(f" Largest possible at his location: {round(self.max_starting_radius, 3)}")
        if self.starting_radius is not None:
            radius = min(self.starting_radius, self.max_starting_radius)
            start_circle = voronoi.start_point.buffer(radius)
            self.starting_cut_area = self.starting_cut_area.union(start_circle)

        super().__init__(to_cut, step, winding_dir, already_cut=self.starting_cut_area)

        # display(polygons=[self.starting_cut_area, to_cut], voronoi=self.voronoi)

        self._reset()
        self.calculate_path()

    def _reset(self) -> None:
        """ Cleanup and/or initialise everything. """
        super()._reset()

        self.start_point: Point = self.voronoi.start_point
        self.start_radius: float = self.voronoi.start_distance or 0

        self.arc_fail_count: int = 0
        self.path_fail_count: int = 0
        self.loop_count: int = 0

        self.visited_edges: Set[int] = set()
        self.open_paths: Dict[int, Tuple[float, float]] = {}

        self.path_len_progress: float = 0.0
        self.path_len_total: float = 0.0
        for edge in self.voronoi.edges.values():
            self.path_len_total += edge.length

        # Used to detect when an arc is too close to the edge to be worthwhile.
        self.dilated_polygon_boundaries = []
        multi = self.polygon
        if multi.type != "MultiPolygon":
            multi = MultiPolygon([multi])
        for poly in multi.geoms:
            for ring in [poly.exterior] + list(poly.interiors):
                self.dilated_polygon_boundaries.append(
                        ring.buffer(self.step * SKIP_EDGE_ARCS))

        if self.do_starting_circle:
            entry_circle = EntryCircle(
                    self.polygon,
                    self.start_point,
                    self.start_radius,
                    self.step,
                    self.winding_dir,
                    already_cut=self.cut_area_total,
                    path=self.path)
            entry_circle.spiral()
            entry_circle.circle()

            if self.starting_radius is not None:
                radius = min(self.starting_radius, self.max_starting_radius)
                if (self.path and
                        self.start_point.buffer(radius).distance(
                            self.path[0].start) < self.step / 20):
                    assert self.path[0].start
                    dx = self.path[0].start.x - self.start_point.x
                    dy = self.path[0].start.y - self.start_point.y
                    self.starting_angle = math.atan2(dx, dy)

            self.last_arc = entry_circle.last_arc

        # Assume starting circle is already cut.
        self.last_circle: Optional[ArcData] = create_circle(
            self.start_point, self.start_radius)
        self.cut_area_total = self.cut_area_total.union(
                Polygon(self.last_circle.path))
        self.cut_area_total2 = self.cut_area_total2.union(
                Polygon(self.last_circle.path).buffer(self.step))

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
    def _converge(cls, kp: float) -> Generator[float, Tuple[float, float], None]:
        """
        Algorithm used for recursively estimating the position of the best fit arc.

        Arguments:
            kp: Proportional multiplier.
        Yields:
            Arguments:
                target: Target step size.
                current: step size resulting from the previous iteration result.
            next distance.
        Returns:
            Never exits Yield loop.
        """
        error: float = 0.0
        value: float = 0.0

        while True:
            target, current = yield value

            error = target - current
            prportional = kp * error
            value = prportional

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
        # A generator to converge on desired spacing.
        #converge = self._converge(0.75)
        converge = self._converge(0.76)
        converge.send(None)  # type: ignore

        coverage_algos = [
                #self._converge(0.75),
                self._converge(0.76),
                self._converge(0.74),
                self._converge(0.78),
                self._converge(0.72),
                self._converge(0.7),
                self._converge(0.8),
                ]

        color_overide = None

        desired_step = min(self.step, (voronoi_edge.length - start_distance))

        distance = start_distance + desired_step

        count: int = 0
        circle: Optional[ArcData] = None
        arcs: List[ArcData] = []
        progress: float = 0.0
        best_progress: float = 0.0
        best_distance: float = 0.0
        dist_offset: int = 100000
        corner_zoom = CORNER_ZOOM * self.step

        # Extrapolate line beyond it's actual distance to give the algorithm
        # room to overshoot while converging on an optimal position for the new arc.
        edge_extended: LineString = self._extrapolate_line(
            dist_offset, voronoi_edge)
        assert abs(edge_extended.length -
                   (voronoi_edge.length + 2 * dist_offset)) < 0.0001

        assert self.cut_area_total
        assert self.cut_area_total.is_valid

        # Loop multiple times, trying to converge on a distance along the voronoi
        # edge that provides the correct step size.
        while count <= ITERATION_COUNT:
            count += 1

            # Propose an arc.
            pos, radius = self._arc_at_distance(
                distance + dist_offset, edge_extended)
            circle = create_circle(pos, radius, self.winding_dir)

            # Compare proposed arc to cut area.
            # We are only interested in sections that have not been cut yet.
            arcs = self._split_arcs([circle])
            if not arcs:
                # arc is entirely hidden by previous cut geometry.

                if best_progress > 0:
                    # Has made some progress.
                    count = ITERATION_COUNT
                    color_overide = "orange"
                    break

                # Has not found any useful arc yet.
                # Don't record it as an arc that needs drawn.
                self.last_circle = circle
                return(distance, [])

            # Progress is measured as the furthest point the proposed arc is
            # from the previous one. We are aiming for proposed == desired_step.
            if self.last_circle:
                progress = self._furthest_spacing_arcs(arcs, self.last_circle)
            else:
                progress = self._furthest_spacing_shapely(arcs, self.cut_area_total)

            desired_step = min(self.step, (voronoi_edge.length - start_distance))
            if radius < corner_zoom:
                # Limit step size as the arc radius gets very small.
                multiplier = (corner_zoom - radius) / corner_zoom
                desired_step = self.step - self.step * CORNER_ZOOM_EFFECT * multiplier

            if abs(desired_step - progress) < abs(desired_step - best_progress):
                # Better fit.
                best_progress = progress
                best_distance = distance

                if abs(desired_step - progress) < desired_step / 20:
                    # Good enough fit.
                    best_progress = progress
                    best_distance = distance
                    break

            modifier = converge.send((desired_step, progress))
            distance += modifier

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
            circle = create_circle(Point(pos), radius, self.winding_dir)
            arcs = self._split_arcs([circle])

        if count == ITERATION_COUNT and self.debug:
            # Log some debug data.
            distance_remain = voronoi_edge.length - distance
            self.arc_fail_count += 1
            log("\tDid not find an arc that fits. Spacing/Desired: "
                f"{round(progress, 3)}/{desired_step}"
                "\tdistance remaining: "
                f"{round(distance_remain, 3)}")

        self.loop_count += count

        assert circle is not None
        self.last_circle = circle
        self.cut_area_total = self.cut_area_total.union(Polygon(circle.path))

        filtered_arcs = []
        for arc in arcs:
            if self._filter_arc(arc):
                filtered_arcs.append(arc)

        return (distance, filtered_arcs)

    def _join_voronoi_branches(self, start_vertex: Tuple[float, float]) -> LineString:
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

    def _get_arcs(self, timeslice: int = 0):
        # TODO: Deprecated. Remove.
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

        start_vertex: Optional[Tuple[float, float]
                               ] = self.start_point.coords[0]

        while start_vertex is not None:
            # This outer loop iterates through the voronoi vertexes, looking for
            # a voronoi edge that has not yet had arcs calculated for it.
            combined_edge = self._join_voronoi_branches(start_vertex)
            if not combined_edge:
                start_vertex = self._choose_next_path()
                continue

            dist = 0.0
            best_dist = dist
            stuck_count = int(combined_edge.length * 10 / self.step + 10)
            while abs(dist - combined_edge.length) > self.step / 20 and stuck_count > 0:
                # This inner loop travels along a voronoi edge, trying to fit arcs
                # that are the correct distance apart.
                stuck_count -= 1
                dist, new_arcs = self._calculate_arc(combined_edge, dist, best_dist)

                self.path_len_progress -= best_dist
                self.path_len_progress += dist

                if dist < best_dist and False:
                    # Getting worse not better or staying the same.
                    # This can happen legitimately but is an indication the algorthm
                    # may be stuck.
                    stuck_count = int(stuck_count / 2)
                else:
                    best_dist = dist
                self._queue_arcs(new_arcs)

                if timeslice >= 0 and self.generate:
                    now = round(time.time() * 1000)  # (ms)
                    if start_time + timeslice < now:
                        yield min(0.999, self.path_len_progress / self.path_len_total)
                        start_time = round(time.time() * 1000)  # (ms)

            if stuck_count <= 0:
                print(
                    f"stuck: {round(dist, 2)} / {round(combined_edge.length, 2)}")
                self.path_fail_count += 1

            start_vertex = self._choose_next_path(combined_edge.coords[-1])

            self._flush_arc_queues()

        if timeslice and self.generate:
            yield 1.0

        assert not self.open_paths
        log(f"loop_count: {self.loop_count}")
        log(f"arc_fail_count: {self.arc_fail_count}")
        log(f"len(path): {len(self.path)}")
        log(f"path_fail_count: {self.path_fail_count}")


    def _filter_arc(self, arc: ArcData) -> Optional[ArcData]:
        """
        Remove any arc that is very close to the edge of the part in it's entirety.
        """
        if len(arc.path.coords) < 3:
            return None

        if arc.path.length <= self.step / 20:
            # Arc too short to care about.
            return None

        if not arc.path.intersects(self.polygon):
            return None

        poly_arc = Polygon(arc.path)
        for ring in self.dilated_polygon_boundaries:
            if ring.contains(poly_arc):
                return None

        return arc

    def _start_point_perimeter(
            self, polygons: MultiPolygon, already_cut: Polygon, step: float) -> VoronoiCenters:
        """
        Recalculate the start point to be inside the cut area, adjacent to the perimeter.
        """
        starting_radius = self.starting_radius or step
        voronoi = VoronoiCenters(polygons, preserve_edge=True)

        perimiter_point = voronoi.start_point
        assert perimiter_point is not None
        voronoi_edge_index = voronoi.vertex_to_edges[perimiter_point.coords[0]]
        assert len(voronoi_edge_index) == 1
        voronoi_edge = voronoi.edges[voronoi_edge_index[0]]
        cut_edge_section = already_cut.intersection(voronoi_edge)
        if cut_edge_section.length > starting_radius * 2:
            new_start_point = cut_edge_section.interpolate(0.5, normalized=True)

            voronoi = VoronoiCenters(polygons, starting_point=new_start_point)
        else:
            # Can't fit starting_radius here.
            # Look for widest point in already_cut area.
            cut_voronoi = VoronoiCenters(already_cut, preserve_widest=True)
            voronoi = VoronoiCenters(polygons, starting_point=cut_voronoi.start_point)
            del cut_voronoi

        return voronoi


class OutsidePocketSimple(Pocket):
    """
    For old branch of DerpCAM compatibility.
    TODO: Remove this class.
    """
    def __init__(
            self,
            polygon: Polygon,
            step: float,
            winding_dir: ArcDir,
            generate: bool = False,
    ) -> None:
        outer_bound = []
        for index, (p, m) in enumerate(zip(polygon.bounds, polygon.exterior.bounds)):
            padding = m - p
            if index < 2:
                padding = min(-4 * step, padding)
            else:
                padding = max(4 * step, padding)
            outer_bound.append(m + padding)
        outer_box = box(*outer_bound)

        to_cut = Polygon(outer_box)
        for interior in polygon.interiors:
            to_cut = to_cut.difference(Polygon(interior))

        already_cut = Polygon(outer_box)
        already_cut = already_cut.difference(Polygon(polygon.exterior))

        super().__init__(
                to_cut,
                step,
                winding_dir,
                already_cut=already_cut,
                generate=True,
                starting_point_tactic=StartPointTactic.PERIMETER,
                do_starting_circle=False,
                debug=True)


class EntryCircle(BaseGeometry):
    center: Point
    radius: float
    start_angle: float
    path: List[Union[ArcData, LineData]]
    cut_area_total2 = Polygon

    def __init__(
            self,
            to_cut: Polygon,
            center: Point,
            radius: float,
            step: float,
            winding_dir: ArcDir,
            start_angle: float = 0,
            already_cut: Optional[Polygon] = None,
            path: Optional[List[Union[ArcData, LineData]]] = None) :
        super().__init__(to_cut, step, winding_dir, already_cut)

        self.center = center
        self.radius = radius
        self.start_angle = start_angle

        if path is None:
            path = []
        self.path = path

        self.cut_area_total2 = self.starting_cut_area.buffer(self.step / 2)

        self._reset()

    def spiral(self):
        loop: float = 0.25
        offset: List[float] = [0.0, 0.0]
        new_arcs = []
        while loop * self.step <= self.radius:
            orientation = round(loop * 4) % 4
            if orientation == 0:
                start_angle = 0
                offset[1] -= self.step / 4
                section_radius = loop * self.step
            elif orientation == 1:
                start_angle = math.pi / 2
                offset[0] -= self.step / 4
                section_radius = loop * self.step
            elif orientation == 2:
                start_angle = math.pi
                offset[1] += self.step / 4
                section_radius = loop * self.step
            elif orientation == 3:
                start_angle = 3 * math.pi / 2
                offset[0] += self.step / 4
                section_radius = loop * self.step
            else:
                raise

            section_center = Point(self.center.x + offset[0], self.center.y + offset[1])
            new_arc = create_arc(
                    section_center, section_radius, start_angle, math.pi / 2, self.winding_dir)
            if section_radius > self.radius:
                mask = self.center.buffer(self.radius)
                masked_arc = new_arc.path.intersection(mask)
                new_arc = create_arc_from_path(Point(offset), masked_arc, section_radius)
            new_arcs.append(new_arc)

            loop += 0.25  # 1/4 turn.
        if self.winding_dir == ArcDir.CCW:
            new_arcs = [mirror_arc(new_arc, self.winding_dir) for new_arc in new_arcs]
        #for new_arc in new_arcs:
        #    print(
        #            new_arc.winding_dir,
        #            new_arc.start_angle / math.pi, new_arc.span_angle / math.pi,
        #            new_arc.start, new_arc.end)

        sorted_arcs = self._split_arcs(new_arcs)

        #print()
        #for new_arc in sorted_arcs:
        #    print(
        #            new_arc.winding_dir,
        #            new_arc.start_angle / math.pi, new_arc.span_angle / math.pi,
        #            new_arc.start, new_arc.end)
        
        self._queue_arcs(sorted_arcs)
        #self._queue_arcs(new_arcs)
        self._flush_arc_queues()

    def circle(self):
        new_arc = create_arc(self.center, self.radius, 0, math.pi * 2 -0.00000001, self.winding_dir)
        sorted_arcs = self._split_arcs([new_arc])
        self._queue_arcs(sorted_arcs)
        self._flush_arc_queues()

