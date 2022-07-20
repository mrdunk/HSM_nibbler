"""
A wrapper for pyvoronoi that calculates the paths along the center of polygon
data. Any point on the voronoi path will be equidistant to 2 or more polygon edges.
While pyvoronoi does not natively handle arcs and circles, this wrapper attempts
to prune voronoi edges that are produces by circles that have been split into
line segments.
"""
from typing import Dict, List, Optional, Set, Tuple, Union

import math

from shapely.geometry.base import BaseGeometry  # type: ignore
from shapely.geometry import box, LineString, Point, Polygon  # type: ignore
from shapely.ops import linemerge, nearest_points  # type: ignore
from shapely.validation import make_valid  # type: ignore

import pyvoronoi  # type: ignore

try:
    from helpers import log  # type: ignore
except ImportError:
    from cam.helpers import log  # type: ignore

Vertex = Tuple[float, float]

# Number of decimal places resolution for coordinates.
ROUND_DP = 5

# Resolution of voronoi algorithm.
# See C++ Boost documentation.
# Set it to 1 better than the geometry resolution.
VORONOI_RES = 10**(ROUND_DP + 1)

# A small number for comparing things that should touch if not for floating-point error.
#EPS = 5.96e-08
EPS = 1 / (10 ** ROUND_DP)


def round_coord(value: Tuple[float, float], dp: int = ROUND_DP) -> Tuple[float, float]:
    return (round(value[0], dp), round(value[1], dp))


class VoronoiCenters:
    start_point: Point = None
    start_distance: float = None

    def __init__(
            self,
            polygon: Polygon,
            tolerence: float = 0.1,
            preserve_widest: bool = False,
            preserve_edge: bool = False,
            starting_point: Point = None
            ) -> None:
        """
        Arguments:
            polygon: The geometer that we wish to generate a voronoi diagram inside.
            tolerence: Parameter used for pruning unwanted voronoi edges. This should
                be approximately the same as the maximum expected jitter in
                geometry coordinates.
        """
        self.polygon = polygon
        self._validate_poly()
        self.tolerence = tolerence

        self.max_dist = max((self.polygon.bounds[2] - self.polygon.bounds[0]),
                            (self.polygon.bounds[3] - self.polygon.bounds[1])) + 1

        # Collect polygon segments to populate voronoi with.
        geom_primatives: List[Tuple[Vertex, Vertex]] = []
        outlines = [self.polygon.exterior] + list(self.polygon.interiors)
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
            assert prev_point == first_point  # This is a loop.

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
                p_start = Point(round_coord((start_vert.X, start_vert.Y)))
                p_end = Point(round_coord((end_vert.X, end_vert.Y)))
                if (p_start.distance(self.polygon) <= EPS and
                        p_end.distance(self.polygon) <= EPS):
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
                            geom_points.append(
                                pv.RetrieveScaledPoint(cell_twin))
                        if cell.contains_segment:
                            geom_edges.append(pv.RetriveScaledSegment(cell))
                        if cell_twin.contains_segment:
                            geom_edges.append(
                                pv.RetriveScaledSegment(cell_twin))
                        assert len(geom_edges) > 0
                        assert len(geom_points) + len(geom_edges) == 2

                        if len(geom_points) == 1 and len(geom_edges) == 1:
                            start_point = Point(round_coord(
                                (start_vert.X, start_vert.Y)))
                            end_point = Point(round_coord(
                                (end_vert.X, end_vert.Y)))
                            max_distance = start_point.distance(end_point) / 10 + 0.01
                            points = (round_coord(point)
                                      for point in
                                      pv.DiscretizeCurvedEdge(edge_index, max_distance))
                            self._store_edge(LineString(points))
                        else:
                            # A parabola between 2 lines (as opposed to 1 line and one point)
                            # leaves the DiscretizeCurvedEdge() function broken sometimes.
                            # Let's just assume a straight line edge in these cases.
                            # This is a particular problem when duplicate points in
                            # input data create a line of zero length.
                            log("BORKED VORONOI: \t"
                                "geom_points: {geom_points}\tgeom_edges: {geom_edges}")
                            line = LineString((
                                round_coord((start_vert.X, start_vert.Y)),
                                round_coord((end_vert.X, end_vert.Y))
                            ))
                            self._store_edge(line)
                        continue

        preserve = []
        if preserve_widest:
            # The widest_gap() method checks vertices for the widest point.
            # If we remove this vertex subsequent calls will not work.
            # If we cached the value instead, client code will not be able to use the
            # returned vertex value as an index into this class's data structures.
            self.start_point, self.start_distance = self.widest_gap()
            if self.start_point is not None:
                preserve.append(self.start_point.coords[0])
        if preserve_edge:
            # The vertex_on_perimiter() method picks a vertex that touches the
            # perimeter. We might not want to clean that up if we want to cut in from
            # the edge.
            self.start_point = self.vertex_on_perimiter()
            preserve.append(self.start_point.coords[0])
            self.start_distance = 0

        self._drop_irrelevant_edges(preserve)

        if starting_point is not None:
            # Set the starting point.
            self.start_point, self.start_distance = self.set_starting_point(starting_point)
            preserve.append(self.start_point.coords[0])

        self._split_on_pocket_edge()
        self._combine_edges(preserve)

        self._check_data()

    def _check_data(self) -> None:
        """ Sanity check data structures. """
        for edge_i, edge in self.edges.items():
            assert edge_i in self.edge_to_vertex
            vertices = self.edge_to_vertex[edge_i]
            assert len(vertices) == 2
            assert edge.coords[0] in vertices
            assert edge.coords[-1] in vertices
            assert edge.coords[0] != edge.coords[-1]  # Loop
            assert vertices[0] in self.vertex_to_edges
            assert vertices[1] in self.vertex_to_edges

        for edge_i, vertices in self.edge_to_vertex.items():
            assert edge_i in self.edges
            assert len(vertices) == 2
            assert edge_i in self.edges
            assert vertices[0] in self.vertex_to_edges
            assert vertices[1] in self.vertex_to_edges

        for vertex, edges_i in self.vertex_to_edges.items():
            assert len(set(edges_i)) == len(edges_i)  # Loop
            for edge_i in edges_i:
                assert edge_i in self.edges
                vertices = self.edge_to_vertex[edge_i]
                assert vertex in vertices

    def _validate_poly(self) -> None:
        """
        Make sure input geometry is sane.
        Do some quick fixes on common issues.
        """
        fixed = make_valid(self.polygon)
        while fixed.type == "MultiPolygon":
            bigest = None
            size = 0
            for geom in fixed.geoms:
                poly_area = Polygon(geom).area
                if poly_area > size:
                    size = poly_area
                    bigest = geom
            fixed = bigest
            fixed = make_valid(fixed)
            # TODO: Should we just throw an exception here?
            # There is more than one choice for the outer geometry loop.
            # Knowing which piece to work on is a crap shoot.
            # It should be up to the client code to make the decision and correctly
            # format the input polygon.
        if fixed.type == "Polygon":
            self.polygon = fixed

        # Any jitter in input geometry may make points appear out of sequence
        # leading to invalid geometry.
        # See here for what valid geometry looks like:
        # https://shapely.readthedocs.io/en/latest/manual.html#polygons
        # .simplify(...) will fix issues caused by input coordinate jitter.
        self.polygon = self.polygon.simplify(0.05)

    def _store_edge(self, edge: LineString, replace_index=None) -> int:
        """
        Store a vorinoi edge and associated vertices in out internal data structures.
        """
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
            self.vertex_to_edges.setdefault(
                vert_index_a, []).append(edge_index)
        if edge_index not in self.vertex_to_edges:
            self.vertex_to_edges.setdefault(
                vert_index_b, []).append(edge_index)
        self.edge_to_vertex[edge_index] = (vert_index_a, vert_index_b)

        return edge_index

    def _remove_edge(self, edge_index: int) -> None:
        """
        Remove a vorinoi edge and associated vertices in out internal data structures.
        """
        if len(self.edges) <= 1:
            # Special case when cleaning up edges and self.polygon is a simple circle.
            return
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

    def distance_from_geom(self, point: BaseGeometry) -> float:
        """
        Distance form nearest geometry edge. Note this edge may be the outer
        boundary or the edge of an island.
        """
        distance = self.max_dist
        for ring in [self.polygon.exterior] + list(self.polygon.interiors):
            distance = min(distance, ring.distance(point))
        return distance

    def _combine_edges(self, dont_merge: List[Tuple[float, float]]) -> None:
        """
        For any voronoi vertex with only 2 edges meeting there, combine the
        voronoi edges, removing the vertex.
        """
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

    def _split_on_pocket_edge(self) -> None:
        """
        When an polygon island touches the exterior at a single point the vorinoi
        diagram will have a vertex at that point. This leads code like _combine_edges
        to presume the edge passes through this vertex.
        It is unlikely this is the desired behavior so this method "shrinks" the
        voronoi edges slightly, creating 2 very close vertexes which stop the
        voronoi edges from touching.
        """
        split_at = []
        for vertex, edges_i in self.vertex_to_edges.items():
            point = Point(vertex)
            if len(edges_i) == 2 and self.distance_from_geom(point) <= EPS:
                split_at.append(vertex)

        while split_at:
            vertex = split_at.pop()
            edges_i = self.vertex_to_edges[vertex]

            assert len(edges_i) == 2

            edge_i_0 = edges_i[0]
            edge_0 = self.edges[edge_i_0]
            if edge_0.coords[0] == vertex:
                edge_0 = LineString(edge_0.coords[::-1])
            vertex_updated_0 = edge_0.interpolate(edge_0.length - EPS)

            edge_i_1 = edges_i[1]
            edge_1 = self.edges[edge_i_1]
            if edge_1.coords[0] == vertex:
                edge_1 = LineString(edge_1.coords[::-1])
            vertex_updated_1 = edge_1.interpolate(edge_1.length - EPS)

            self._remove_edge(edge_i_0)
            self._remove_edge(edge_i_1)

            self._store_edge(LineString([edge_0.coords[0], vertex_updated_0]), edge_i_0)
            self._store_edge(LineString([edge_1.coords[0], vertex_updated_1]), edge_i_1)

    def _follow_cleanup_candidates(self, vertex: Vertex) -> Set[int]:
        """
        Voronoi edges that touch self.polygon are sometimes not needed and are
        candidates for pruning.
        eg: When the outer geometry contains an arc section, the arc will have been
        split into straight line facets. A voronoi edge will touch the point where
        these facets join. We do not need these voronoi edges so they should be pruned.

        Arguments:
            vertex: The starting point of an edge that may need pruned.
        Returns:
            A set of indexes into self.edges that need pruned.
        """
        vertex_start: Vertex = vertex
        last_edge_angle: Optional[float] = None
        visited_edges: Set[int] = set()
        to_return: Set[int] = set()
        working = True
        while working:
            working = False
            edges_i = self.vertex_to_edges[vertex_start]
            if last_edge_angle is None and len(edges_i) != 1:
                # Only interested in starting at the end of an edge.
                return set()

            for edge_i in edges_i:
                if edge_i in visited_edges:
                    continue
                visited_edges.add(edge_i)

                vertices = self.edge_to_vertex[edge_i]
                vertex_end = vertices[0] if vertex_start != vertices[0] else vertices[1]

                edge_angle = math.atan2(
                    (vertex_start[0] - vertex_end[0]), (vertex_start[1] - vertex_end[1]))

                if abs(LineString([vertex, vertex_end]).length -
                       self.distance_from_geom(Point(vertex_end))) >= self.tolerence:
                    # The closest point on the outer geometry is closer that the end of the
                    # end of the edges we are tracking.
                    # This implies vertex_end is the far side of the arc center from the
                    # first section.
                    # Not a candidate for cleanup.
                    # TODO: We could also compare how parallel the line between vertex
                    # and nearest pint is to the edges we are examining. If they are
                    # roughly parallel we should still consider this edge for cleanup.
                    continue

                if (last_edge_angle is None or
                        abs(edge_angle - last_edge_angle) < 0.3 or
                        (len(edges_i) == 2 and len(self.vertex_to_edges[vertex_end]) == 2)):
                    # This voronoi edge section is one of the following:
                    # 1. The first to be considered.
                    # 2. Roughly co-linear with the previous section.
                    # 3. Joins exactly one other edge at either end.
                    to_return.add(edge_i)
                    vertex_start = vertex_end
                    working = True
                    last_edge_angle = edge_angle
                    break

        return to_return

    def _drop_irrelevant_edges(self, preserve: List[Tuple[float, float]]) -> None:
        """
        If any geometry resulting from a voronoi edge will be covered by the
        geometry of some other voronoi edge it is deemed irrelevant and pruned
        by this function.
        """
        to_prune: Set[int] = set()
        for edge_i in self.edges:
            vert_a, vert_b = self.edge_to_vertex[edge_i]
            if vert_a not in preserve and vert_b not in preserve:
                to_prune |= self._follow_cleanup_candidates(vert_a)
                to_prune |= self._follow_cleanup_candidates(vert_b)

        for edge_i in to_prune:
            self._remove_edge(edge_i)

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

    def vertex_on_perimiter(self) -> Point:
        """
        Find a point as close to the perimeter as possible.
        """
        bounding_box = box(*self.polygon.bounds).exterior
        closest = None
        distance = self.max_dist
        for vertex in self.vertex_to_edges:
            if bounding_box.intersects(Point(vertex)):
                return Point(vertex)
            dist = bounding_box.distance(Point(vertex))
            if dist < distance:
                distance = dist
                closest = Point(vertex)
        return closest

    def set_starting_point(self, point: Point) -> Tuple[Point, float]:
        """
        Set a starting point within the polygon from which to start iterating over
        the voronoi diagram.
        If this point is not on the existing voronoi diagram it will need a new edge
        to be added to the diagram to connect this starting point.
        """
        closest_edge = None
        closest_index = None
        closest_dist = None
        for edge_index, edge in self.edges.items():
            dist = point.distance(edge)
            proposed_new_vertex = round_coord(nearest_points(edge, point)[0].coords[0])
            proposed_new_edge = LineString([point, proposed_new_vertex])
            if closest_dist is None or dist < closest_dist:
                if proposed_new_edge.within(self.polygon):
                    closest_dist = dist
                    closest_index = edge_index
                    closest_edge = edge

        # Add a new vertex on the voronoi graph.
        new_vertex = round_coord(nearest_points(closest_edge, point)[0].coords[0])

        if new_vertex not in self.vertex_to_edges:
            v1, v2 = self.edge_to_vertex[closest_index]
            new_edge_1 = LineString([v1, new_vertex])
            new_edge_2 = LineString([v2, new_vertex])

            assert new_edge_1.length > 0
            assert new_edge_2.length > 0

            self._remove_edge(closest_index)
            
            self._store_edge(new_edge_1)
            self._store_edge(new_edge_2)

        # Add a new edge to the voronoi graph.
        # This will home the new starting point.
        new_edge_3 = LineString([point, new_vertex])

        if new_edge_3.length == 0:
            return Point(new_vertex), self.distance_from_geom(Point(new_vertex))

        self._store_edge(new_edge_3)

        return point, self.distance_from_geom(point)


