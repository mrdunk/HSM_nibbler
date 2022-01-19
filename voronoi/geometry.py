from typing import Dict, Generator, List, Optional, Set, Tuple, Union

import time

from shapely.geometry import LineString, MultiLineString, Point, Polygon 
from shapely.ops import linemerge, nearest_points

import matplotlib.pyplot as plt
import pyvoronoi

Vertex = Tuple[float, float]

ITERATION_COUNT = 50
VORONOI_RES = 1000
#BREADTH_FIRST = True
BREADTH_FIRST = False
CURVE_STEP_LEN = 0.5
ROUND_DP = 4


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
    def __init__(self, polygon: Polygon) -> None:
        self.polygon = polygon
        self.step = 1
        self.remainder = 0
        self.last_radius = None
        self.max_dist = max((self.polygon.bounds[2] - self.polygon.bounds[0]),
                (self.polygon.bounds[3] - self.polygon.bounds[1])) + 1
        self.voronoi = Voronoi(polygon, tolerence = self.step)
        self.start: Point = self.voronoi.widest_gap()
        self.cut_area_total = Polygon()
        self.last_circle = Polygon()
        self.fail_count = 0
        self.loop_count = 0

        self.visited_edges: Set[int] = set()
        start_vertex = (self.start.x, self.start.y)

        self.open_paths = {edge: start_vertex for edge in self.voronoi.vertex_to_edges[start_vertex]}

        self._walk()

    def _chose_path(self, start_vertex) -> Tuple[Tuple[float, float], int]:
        """
        Chose a path onwards from start_vertex.
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
        self.last_circle = LineString()
        return (start_vertex, edge_i)

    def _distance_from_geom(self, point: Point) -> float:
        radius = self.max_dist
        for ring in [self.polygon.exterior] + list(self.polygon.interiors):
            nearest = nearest_points(point, ring)
            dist = point.distance(nearest[1])
            if dist < radius:
                radius = dist
        return radius

    @classmethod
    def _extrapolate_line(cls, extra: float, line: LineString) -> LineString:
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
            pos = voronoi_edge.interpolate(distance)
            radius = self._distance_from_geom(pos)
            
            return (pos, radius)

    @classmethod
    def _furthest_spacing(cls, a: Union[LineString, MultiLineString], b: LineString) -> float:
        a_normal = a
        # Merge lines if we can but ultimately use MultiLineString so we
        # deal with a single type.
        if a_normal.type == "MultiLineString":
            a_normal = linemerge(a_normal)
        if a_normal.type != "MultiLineString":
            a_normal = MultiLineString([a_normal])

        spacing = 0
        for section in a_normal.geoms:
            #start = Point(section.coords[0])
            #mid = section.interpolate(0.5, normalized=True)
            #end = Point(section.coords[-1])
            #spacing = max(spacing, start.distance(b))
            #spacing = max(spacing, mid.distance(b))
            #spacing = max(spacing, end.distance(b))
            for index in range(0, len(section.coords), 2):
                coord = section.coords[index]
                spacing = max(spacing, Point(coord).distance(b))

        return spacing

    def _calculate_arc(
            self,
            voronoi_edge: LineString,
            start_distance: float,
            debug: bool = False) -> float:
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
        """
        pid = self._pid(0.75, 0, 0, 0)
        #pid = self._pid(0.19, 0.04, 0.12, 0)
        #pid = self._pid(0.9, 0.01, 0.01, 0)
        #pid = self._pid(0, 0.001, 0.3, 0)
        pid.send(None)  # type: ignore

        color = "cyan"
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
        circle: Optional[Polygon] = None
        arc: Union[LineString, MultiLineString, None] = None
        arc_section: Optional[LineString] = None
        progress: float = 1.0
        best_progress: float = 0.0
        best_distance: float = 0.0
        dist_offset: int = 100000
        edge_extended: LineString = self._extrapolate_line(dist_offset, voronoi_edge)
        #edge_extended = voronoi_edge
        assert abs(edge_extended.length - (voronoi_edge.length + 2 * dist_offset)) < 0.0001

        log = f"\t{voronoi_edge.length=} {edge_extended.length=}\n"
        while count < ITERATION_COUNT:
            count += 1

            # Propose an arc.
            pos, radius = self._arc_at_distance(distance + dist_offset, edge_extended)
            if radius <= 0:
                # The voronoi edge has met the part geometry.
                # Nothing more to do.
                color = "yellow"
                return distance
            circle = Point(pos).buffer(radius)
            arc = circle.boundary

            if not self.last_circle:
                # The very first circle in the whole path.
                # No calculation required. Just draw it.
                break

            # Compare proposed arc to cut area.
            # We are only interested in sections that have not been cut yet.
            arc_section = arc.difference(self.cut_area_total)
            if not arc_section:
                # arc is entirely hidden by previous cut geometry.
                # Nothing more to do here.
                color = "orange"
                return distance

            # Progress is measured as the furthest point he proposed arc is
            # from the previous one. We are aiming for proposed == step.
            progress = self._furthest_spacing(arc_section, self.last_circle.boundary)
            if (abs(step - progress) < abs(step - best_progress) and
                    distance > 0 and best_distance <= voronoi_edge.length):
                best_progress = progress
                best_distance = distance

            if abs(step - progress) < step / 20:
                # Good enough fit.
                color = "green"
                break

            if abs(best_distance - voronoi_edge.length) < step / 20 and progress < self.step:
                # Have reached end of voronoi edge. Pointless going further.
                color = "green"
                break

            modifier = pid.send((step, progress))
            distance += modifier

            #log += f"\t{progress=}\t{best_progress=}\t{modifier=}\n"

        log += f"\t{progress=}\t{best_progress=}\t{best_distance=}\t{voronoi_edge.length=}\n"
        log += f"\t{color=}\n"
        log += "\t--------"

        if best_distance > voronoi_edge.length:
            best_distance = voronoi_edge.length
            color = "black"
        if best_distance <= 0:
            best_distance = 0.0001
            color = "blue"
            log += "\tclamp distance to 0\n"

        if distance != best_distance:
            distance = best_distance
            progress = best_progress
            pos, radius = self._arc_at_distance(distance + dist_offset, edge_extended)
            circle = Point(pos).buffer(radius)
            arc = circle.boundary

        # Recalculate arc_section with cut path masked out.
        assert arc is not None
        if debug:
            arc_section = arc.difference(self.last_circle)
        else:
            arc_section = arc.difference(self.cut_area_total)

        assert arc_section is not None
        if arc_section.type == "MultiLineString":
            arc_section = linemerge(arc_section)
        if arc_section:
            if arc_section.type != "MultiLineString":
                arc_section = MultiLineString([arc_section])
            for section in arc_section.geoms:
                x, y = section.xy
                plt.plot(x, y, c=color, linewidth=1)

        if count == ITERATION_COUNT:
            self.fail_count += 1
            print("\tDid not find an arc that fits. Spacing/Desired: "
                    f"{round(progress, 3)}/{step}")

        if count == ITERATION_COUNT or debug:
            print(log)

        self.loop_count += count

        self.last_circle = circle
        self.cut_area_total = self.cut_area_total.union(circle)

        if final_run and progress > step * 0.9:
            # Close enough to end.
            distance = voronoi_edge.length

        return distance

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

        assert line.coords[0] == start
        return (line, traversed_edges)

    def _walk(self) -> None:
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
                dist = self._calculate_arc(
                        combined_edge, dist, debug=(edge_i in []))

            if stuck_count <= 0:
                print(f"stuck: {round(dist, 2)} / {round(combined_edge.length, 2)}")

            plt.plot(start_vertex[0], start_vertex[1], "o", c="yellow")
            plt.plot(end_vertex[0], end_vertex[1], "o", c="cyan", markersize=4)

            self.visited_edges |= traversed_edges
            assert end_vertex != start_vertex
            start_vertex = end_vertex

            start_vertex, edge_i = self._chose_path(start_vertex)

            if not self.last_circle:
                self.last_circle = self.cut_area_total

        print(f"{self.fail_count=}\t {self.loop_count=}")
