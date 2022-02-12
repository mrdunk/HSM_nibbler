from typing import Dict, Generator, List, NamedTuple, Optional, Set, Tuple, Union

import math

from shapely.geometry import LineString, MultiLineString, Point, Polygon
from shapely.ops import linemerge, nearest_points
from shapely.validation import make_valid

import pyvoronoi

try:
    from helpers import log
except ImportError:
    from cam.helpers import log

Vertex = Tuple[float, float]

# Number of decimal places resolution for coordinates.
ROUND_DP = 5

# Resolution of voronoi algorithm.
# See C++ Boost documentation.
# Set it to 1 better than the geometry resolution.
VORONOI_RES = 10**(ROUND_DP + 1)

# A tiny number for comparing things that should touch if not for floating-point error.
#EPS = 5.96e-08
EPS = 1 / (10 ** ROUND_DP)

def round_coord(value: Tuple[float, float], dp: int = ROUND_DP) -> Tuple[float, float]:
    return (round(value[0], dp), round(value[1], dp))


class VoronoiCenters:
    def __init__(self, polygon: Polygon, tolerence: float = 0.1) -> None:
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
                            log(f"BORKED VORONOI: \t{geom_points=}\t{geom_edges=}")
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

    def _validate_poly(self) -> None:
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

        self.polygon = self.polygon.simplify(0.05)

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

    def _follow_cleanup_candidates(self, vertex: Vertex) -> Set[int]:
        """
        Voronoi edges that touch self.polygon are sometimes not needed and should be pruned.
        eg: When the outer geometry contains an arc section, the arc will have been
        split into straight line facets. A voronoi edge will touch the point where
        these facets join. We do not need these voronoi edges so they should be pruned.

        Arguments:
            vertex: The starting point of an edge that may need pruned.
        Returns:
            A set of indexes into self.edges that need pruned.
        """
        vertex_start = vertex
        last_edge_angle = None
        visited_edges = set()
        to_return = []
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

                if last_edge_angle is None:
                    # This is the first edge section to be considered.
                    # All future angles should be compared against this one for
                    # co-linearity.
                    assert len(edges_i) == 1
                    last_edge_angle = edge_angle

                if abs(LineString([vertex, vertex_end]).length -
                        self.distance_from_geom(Point(vertex_end))) > self.tolerence:
                    # This vertex_end is the far side of the arc center from the first section.
                    # Not a candidate for cleanup.
                    continue

                if last_edge_angle is None or abs(edge_angle - last_edge_angle) < 0.3:
                    # This voronoi edge section is roughly co-linear with the first
                    # one so add it to the cleanup list.
                    to_return.append(edge_i)
                    vertex_start = vertex_end
                    working = True
                    break

        return set(to_return)

    def _drop_irrelevant_edges(self) -> None:
        """
        If any geometry resulting from a voronoi edge will be covered by the
        geometry of some other voronoi edge it is deemed irrelevant and pruned
        by this function.
        """
        to_prune = set()
        for index, edge in self.edges.items():
            vert_a, vert_b = self.edge_to_vertex[index]
            to_prune |= self._follow_cleanup_candidates(vert_a)
            to_prune |= self._follow_cleanup_candidates(vert_b)

        for index in to_prune:
            self._remove_edge(index)

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

