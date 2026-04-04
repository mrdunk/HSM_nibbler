"""
    A CAM library for generating HSM "peeling" toolpaths from supplied geometry.

    Copyright (C) <2022>  <duncan law>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

    mrdunk@gmail.com
"""

# pylint: disable=attribute-defined-outside-init

from typing import Dict, List, Optional, Set, Tuple, Union

import math
import time

from shapely.geometry import LinearRing, LineString, MultiLineString, MultiPolygon, Point, Polygon  # type: ignore

from hsm_nibble.arc_utils import (  # type: ignore
    ArcData, ArcDir, LineData, MoveStyle, StartPointTactic,
    arcs_from_circle_diff, complete_arc, create_arc, create_arc_from_path,
    create_circle, filter_arc, mirror_arc, split_arcs,
)
from hsm_nibble.arc_fitter import (  # type: ignore
    ProportionalController, arc_at_distance, find_best_arc_distance,
    ITERATION_COUNT, CORNER_ZOOM, CORNER_ZOOM_EFFECT,
)
from hsm_nibble.path_assembler import PathAssembler, split_line_by_poly  # type: ignore
from hsm_nibble.debug import Display
from hsm_nibble.voronoi_centers import VoronoiCenters  # type: ignore
from hsm_nibble.helpers import log  # type: ignore
DEBUG_DISPLAY = Display()


# Filter arcs that are entirely within this distance of a pocket edge.
SKIP_EDGE_ARCS = 1 / 20

# Whether to visit short voronoi edges first (True) or try to execute longer
# branches first.
# TODO: We could use more long range path planning that take the shortest total
# path into account.
#BREADTH_FIRST = True
BREADTH_FIRST = False


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
    if multi.geom_type != "MultiPolygon":
        multi = MultiPolygon([multi])
    polygons = []
    for polygon in multi.geoms:
        polygons.append(clean_polygon(polygon))

    return MultiPolygon(polygons)

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
    # Area we have calculated arcs for. Calculated by appending full circles.
    calculated_area_total: Polygon

    starting_cut_area: Polygon

    def __init__(
            self,
            to_cut: Union[Polygon, MultiPolygon],
            step: float,
            winding_dir: ArcDir,
            already_cut: Polygon = None,
            assembler: Optional[PathAssembler] = None,
    ) -> None:
        self.polygon: Union[Polygon, MultiPolygon] = to_cut
        self.step: float = step
        self.winding_dir: ArcDir = winding_dir
        if already_cut is None:
            already_cut = Polygon()
        self.starting_cut_area = already_cut

        self.calculated_area_total = self.starting_cut_area.buffer(0)
        cut_area_initial = self.starting_cut_area.buffer(0)
        # Not the same instance.
        assert self.calculated_area_total is not cut_area_initial

        self._filter_input_geometry()
        self.dilated_starting_polygon = self.polygon.buffer(self.step / 10)

        if assembler is None:
            self.assembler = PathAssembler(
                step, winding_dir, cut_area_initial, self.dilated_starting_polygon)
        else:
            self.assembler = assembler

    # Properties that delegate to the assembler so the rest of the code can
    # continue to use self.path, self.last_arc, self.cut_area_total.

    @property
    def path(self) -> List[Union[ArcData, LineData]]:
        return self.assembler.path

    @path.setter
    def path(self, value: List[Union[ArcData, LineData]]) -> None:
        self.assembler.path = value

    @property
    def last_arc(self) -> Optional[ArcData]:
        return self.assembler.last_arc

    @last_arc.setter
    def last_arc(self, value: Optional[ArcData]) -> None:
        self.assembler.last_arc = value

    @property
    def cut_area_total(self) -> Polygon:
        return self.assembler.cut_area_total

    @cut_area_total.setter
    def cut_area_total(self, value: Polygon) -> None:
        self.assembler.cut_area_total = value

    @property
    def pending_arc_queues(self) -> List[List[ArcData]]:
        return self.assembler.pending_arc_queues

    def _filter_input_geometry(self) -> None:
        """Normalise and validate the input polygon before path generation.

        Accepts either a Polygon or MultiPolygon and ensures self.polygon is
        always a MultiPolygon on exit.  Steps performed:

        1. Coerce a bare Polygon into a single-element MultiPolygon.
        2. Drop any non-Polygon sub-geometries (e.g. LineStrings or Points
           that can appear after a boolean difference).
        3. Repair each sub-polygon with buffer(0) if it is not valid.
        4. Discard sub-polygons that vanish when inset by step/10 — these
           are too small to contribute a meaningful toolpath arc.
        5. Repair the reassembled MultiPolygon if overlapping sub-polygons
           have made it invalid.
        """
        if self.polygon.geom_type == "Polygon":
            geoms = [self.polygon]
        else:
            geoms = [g for g in self.polygon.geoms if g.geom_type == "Polygon"]

        # Repair each sub-polygon individually, then filter out tiny ones.
        repaired = []
        for poly in geoms:
            if not poly.is_valid:
                poly = poly.buffer(0)
            # buffer(0) on a self-intersecting polygon can return a MultiPolygon;
            # flatten it so no geometry is silently dropped.
            if poly.geom_type == "MultiPolygon":
                candidates = list(poly.geoms)
            elif poly.geom_type == "Polygon":
                candidates = [poly]
            else:
                continue
            for candidate in candidates:
                if candidate.buffer(-self.step / 10).area <= 0:
                    continue
                repaired.append(candidate)

        self.polygon = MultiPolygon(repaired)

        # Repair the combined MultiPolygon if overlaps are present.
        if not self.polygon.is_valid:
            self.polygon = self.polygon.buffer(0)


    def _flush_arc_queues(self) -> None:
        self.assembler.flush_arc_queues()

    def _queue_arcs(self, new_arcs: List[ArcData]) -> None:
        self.assembler.queue_arcs(new_arcs)

    def join_arcs(self, next_arc: ArcData) -> List[LineData]:
        return self.assembler.join_arcs(next_arc)

    def _split_arcs(self, full_arcs: List[ArcData]) -> List[ArcData]:
        return split_arcs(full_arcs, self.calculated_area_total)


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
            already_cut: Optional[Polygon] = None,
            generate: bool = False,
            voronoi: Optional[VoronoiCenters] = None,
            starting_point_tactic: StartPointTactic = StartPointTactic.WIDEST,
            starting_point: Optional[Point] = None,
            starting_radius: Optional[float] = None,
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
        complete_pocket = already_cut.union(to_cut)
        to_cut = complete_pocket.difference(already_cut)

        clean_multipolygon(to_cut)
        clean_multipolygon(complete_pocket)
        clean_multipolygon(already_cut)

        self.starting_cut_area = already_cut
        self.starting_radius = starting_radius
        self.generate = generate
        self.debug = debug

        # Calculate voronoi diagram for finding points equidistant between part edges.
        if voronoi is None:
            if starting_point is not None:
                voronoi = VoronoiCenters(complete_pocket, starting_point=starting_point)
            elif starting_point_tactic == StartPointTactic.WIDEST:
                voronoi = VoronoiCenters.for_widest_start(
                        self.starting_radius, step, complete_pocket, already_cut)
            elif starting_point_tactic == StartPointTactic.PERIMETER:
                voronoi = VoronoiCenters.for_perimeter_start(
                        self.starting_radius or step, complete_pocket, already_cut)

            assert voronoi is not None
        self.voronoi = voronoi

        # Calculate entry hole settings.
        self.max_starting_radius = voronoi.max_starting_radius
        if starting_radius and self.max_starting_radius < starting_radius:
            print(f"Warning: Starting radius of {starting_radius} overlaps boundaries "
                  f"at {voronoi.start_point}")
            print(f" Largest possible at this location: {round(self.max_starting_radius, 3)}")
        if self.starting_radius is not None:
            radius = min(self.starting_radius, self.max_starting_radius)
            start_circle = voronoi.start_point.buffer(radius)
            already_cut = already_cut.union(start_circle)

        super().__init__(to_cut, step, winding_dir, already_cut=already_cut)

        self._reset()
        self.calculate_path()

    def _reset(self) -> None:
        """ Cleanup and/or initialise everything. """

        self.start_point: Point = self.voronoi.start_point
        self.start_radius: float = self.voronoi.start_distance or 0

        self.arc_fail_count: int = 0
        self.path_fail_count: int = 0
        self.loop_count: int = 0

        self.visited_edges: Set[int] = set()
        self.open_paths: Dict[int, Tuple[float, float]] = {}

        self.path_len_progress: float = 0.0
        self.path_len_total: float = 0.0
        for edge in self.voronoi.graph.edges.values():
            self.path_len_total += edge.length

        # Used to detect when an arc is too close to the edge to be worthwhile.
        self.dilated_polygon_boundaries = []
        multi = self.polygon
        if multi.geom_type != "MultiPolygon":
            multi = MultiPolygon([multi])
        for poly in multi.geoms:
            for ring in [poly.exterior] + list(poly.interiors):
                self.dilated_polygon_boundaries.append(
                        ring.buffer(self.step * SKIP_EDGE_ARCS))

        # Generate first circle from which other arcs expand.
        # Pass the shared assembler so EntryCircle appends directly to our path.
        entry_circle = EntryCircle(
                self.polygon,
                self.start_point,
                self.start_radius,
                self.step,
                self.winding_dir,
                already_cut=self.calculated_area_total,
                assembler=self.assembler)
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

        self.last_circle: Optional[ArcData] = create_circle(
            self.start_point, self.start_radius)
        self.calculated_area_total = self.calculated_area_total.union(
                Polygon(self.last_circle.path))

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

    def done_generating(self):
        if DEBUG_DISPLAY:
            DEBUG_DISPLAY.display(
                    polygons={
                        "previously_cut": self.starting_cut_area,
                        "to_cut": self.polygon,
                        "to_cut_dilated": self.dilated_starting_polygon,
                        #"cut_progress": self.calculated_area_total,
                        },
                    voronoi=self.voronoi,
                    path=self.path
                    )

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

    def _arc_at_distance(self, distance: float, voronoi_edge: LineString) -> Tuple[Point, float]:
        return arc_at_distance(distance, voronoi_edge, self.voronoi.distance_from_geom)

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
        assert self.calculated_area_total
        assert self.calculated_area_total.is_valid

        controller = ProportionalController()
        best_distance, best_circle, hidden_at_start, iteration_count, backwards = \
            find_best_arc_distance(
                voronoi_edge=voronoi_edge,
                start_distance=start_distance,
                min_distance=min_distance,
                step=self.step,
                winding_dir=self.winding_dir,
                calculated_area=self.calculated_area_total,
                last_circle=self.last_circle,
                distance_from_geom=self.voronoi.distance_from_geom,
                max_dist=self.voronoi.max_dist,
                controller=controller,
            )

        if hidden_at_start:
            self.last_circle = best_circle
            return (best_distance, [])

        if backwards:
            return (voronoi_edge.length, [])

        if iteration_count == ITERATION_COUNT and self.debug:
            distance_remain = voronoi_edge.length - best_distance
            self.arc_fail_count += 1
            log("\tDid not find an arc that fits. "
                "\tdistance remaining: "
                f"{round(distance_remain, 3)}")

        self.loop_count += iteration_count

        assert best_circle is not None
        self.last_circle = best_circle

        arcs = self._split_arcs([best_circle])
        self.calculated_area_total = self.calculated_area_total.union(
            Polygon(best_circle.path))

        filtered_arcs = [arc for arc in arcs if self._filter_arc(arc)]

        return (best_distance, filtered_arcs)

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
            branches = self.voronoi.graph.vertex_to_edges[vertex]
            candidate = None
            longest = 0
            for branch in branches:
                if branch not in self.visited_edges:
                    self.open_paths[branch] = vertex
                    length = self.voronoi.graph.edges[branch].length
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
            edge_coords = self.voronoi.graph.edges[candidate].coords

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

        self.done_generating()

    def _filter_arc(self, arc: ArcData) -> Optional[ArcData]:
        return filter_arc(arc, self.polygon, self.dilated_polygon_boundaries, self.step)

    def _start_point_perimeter(
            self, polygons: MultiPolygon, already_cut: Polygon, step: float) -> VoronoiCenters:
        """
        Recalculate the start point to be inside the cut area, adjacent to the perimeter.
        """
        starting_radius = self.starting_radius or step
        voronoi = VoronoiCenters(polygons, preserve_edge=True)

        perimiter_point = voronoi.start_point
        assert perimiter_point is not None
        voronoi_edge_index = voronoi.graph.vertex_to_edges[perimiter_point.coords[0]]
        assert len(voronoi_edge_index) == 1
        voronoi_edge = voronoi.graph.edges[voronoi_edge_index[0]]
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


class EntryCircle(BaseGeometry):
    center: Point
    radius: float
    start_angle: float

    def __init__(
            self,
            to_cut: Polygon,
            center: Point,
            radius: float,
            step: float,
            winding_dir: ArcDir,
            start_angle: float = 0,
            already_cut: Optional[Polygon] = None,
            assembler: Optional[PathAssembler] = None) :
        super().__init__(to_cut, step, winding_dir, already_cut, assembler=assembler)

        self.center = center
        self.radius = radius
        self.start_angle = start_angle

    def spiral(self):
        loop: float = 0.25
        offset: List[float] = [0.0, 0.0]
        new_arcs = []
        mask = self.center.buffer(self.radius)
        not_done = True
        while loop * self.step <= self.radius and not_done:
            orientation = round(loop * 4) % 4
            if orientation == 0:
                start_angle: float = 0
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
                raise RuntimeError(f"Unexpected orientation: {orientation}")

            section_center = Point(self.center.x + offset[0], self.center.y + offset[1])
            new_arc = create_arc(
                    section_center, section_radius, start_angle, math.pi / 2, ArcDir.CW)
            assert new_arc is not None

            if new_arc.path.intersects(mask.exterior):
                # Spiral has crossed bounding circle.
                masked_arc = new_arc.path.intersection(mask)
                if masked_arc.geom_type == "MultiLineString":
                    longest = None
                    for arc in list(masked_arc.geoms):
                        if not longest or arc.length > longest.length:
                            longest = arc
                    masked_arc = longest
                if masked_arc.geom_type == "GeometryCollection":
                    longest = None
                    for arc in list(masked_arc.geoms):
                        if not longest or arc.length > longest.length:
                            longest = arc
                    masked_arc = longest

                assert new_arc.radius is not None
                new_arc = create_arc_from_path(
                        new_arc.origin, masked_arc, new_arc.radius, ArcDir.CW)
                new_arc = complete_arc(new_arc, new_arc.winding_dir)
                assert new_arc is not None

                not_done = False

            if self.winding_dir == ArcDir.CCW:
                new_arc = mirror_arc(self.center.x, new_arc)

            new_arcs.append(new_arc)
            loop += 0.25  # 1/4 turn.

        #self._queue_arcs(new_arcs)

        sorted_arcs = self._split_arcs([a for a in new_arcs if a is not None])
        sorted_arcs = [a for a in [complete_arc(s) for s in sorted_arcs] if a is not None]
        self._queue_arcs(sorted_arcs)

        self._flush_arc_queues()

    def circle(self):
        if self.winding_dir == ArcDir.CCW:
            angle = -math.pi * 1.99999
        else:
            angle = math.pi * 1.99999

        new_arc = create_arc(self.center, self.radius, 0, angle, self.winding_dir)
        assert new_arc is not None
        sorted_arcs = self._split_arcs([new_arc])
        sorted_arcs = [a for a in [complete_arc(s) for s in sorted_arcs] if a is not None]
        self._queue_arcs(sorted_arcs)
        self._flush_arc_queues()
