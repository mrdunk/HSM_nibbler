#!/usr/bin/env python3

import unittest
import os, sys
import math

from shapely.geometry import LineString, MultiPolygon, Point, Polygon  # type: ignore

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import hsm_nibble.voronoi_centers as voronoi_centers


def _make_graph(edges, edge_to_vertex, vertex_to_edges, edge_count=None):
    """Build a VoronoiGraph from raw dicts for use in tests."""
    g = voronoi_centers.VoronoiGraph()
    g.edges = edges
    g.edge_to_vertex = edge_to_vertex
    g.vertex_to_edges = vertex_to_edges
    g.edge_count = edge_count if edge_count is not None else len(edges)
    return g


class TestBoilerplate(unittest.TestCase):
    def setUp(self):
        self.vc = voronoi_centers.VoronoiCenters(Polygon())
        self.vc.graph = _make_graph(
            edge_to_vertex={
                1: ((10.1, 10.1), (10.1, 20.1)),
                2: ((20.2, 10.1), (10.1, 10.1)),
                3: ((30.3, 20.2), (20.2, 20.2)),
                4: ((20.2, 20.2), (20.2, 10.1)),
                5: ((20.2, 20.2), (10.1, 20.1)),
                6: ((10.1, 20.1), (-10, 20)),
            },
            vertex_to_edges={
                (10.1, 10.1): [1, 2],
                (20.2, 10.1): [2, 4],
                (20.2, 20.2): [4, 5, 3],
                (10.1, 20.1): [1, 6, 5],
                (30.3, 20.2): [3,],
                (-10, 20): [6,],
            },
            edges={},  # filled below using edge_to_vertex
        )
        self.vc.graph.edges = {
            k: LineString(v) for k, v in self.vc.graph.edge_to_vertex.items()
        }
        self.vc.graph.check()

    def test_one(self):
        pass


class TestDistanceFromGeom(unittest.TestCase):
    def _make_vc(self, polygon):
        vc = voronoi_centers.VoronoiCenters(Polygon())
        vc.polygon = polygon
        vc.max_dist = 1000
        return vc

    def test_polygon_exterior(self):
        # Point at centre of a 10x10 square — distance to edge should be 5.
        poly = Polygon([(0,0),(10,0),(10,10),(0,10)])
        vc = self._make_vc(poly)
        self.assertAlmostEqual(vc.distance_from_geom(Point(5, 5)), 5.0)

    def test_polygon_with_hole(self):
        # Annular polygon: outer 20x20, inner 10x10 hole centred at origin.
        outer = [(-10,-10),(10,-10),(10,10),(-10,10)]
        inner = [(-5,-5),(5,-5),(5,5),(-5,5)]
        poly = Polygon(outer, [inner])
        vc = self._make_vc(poly)
        # Point at (0,0): distance to inner ring edge is 5, outer edge is 10.
        self.assertAlmostEqual(vc.distance_from_geom(Point(0, 0)), 5.0)

    def test_multipolygon(self):
        # Two separate squares. Point near the first square's edge.
        poly1 = Polygon([(0,0),(10,0),(10,10),(0,10)])
        poly2 = Polygon([(20,0),(30,0),(30,10),(20,10)])
        multi = MultiPolygon([poly1, poly2])
        vc = self._make_vc(multi)
        # Point at (1,5): distance to left edge of poly1 is 1.
        self.assertAlmostEqual(vc.distance_from_geom(Point(1, 5)), 1.0)


class TestWidestGap(unittest.TestCase):
    def _make_vc(self, polygon, vertices):
        vc = voronoi_centers.VoronoiCenters(Polygon())
        vc.polygon = polygon
        vc.max_dist = 1000
        vc.graph.vertex_to_edges = {v: [] for v in vertices}
        return vc

    def test_centre_wins(self):
        poly = Polygon([(0,0),(10,0),(10,10),(0,10)])
        vc = self._make_vc(poly, [(5,5), (1,1), (9,1)])
        point, dist = vc.widest_gap()
        self.assertEqual(point, Point(5, 5))
        self.assertAlmostEqual(dist, 5.0)

    def test_returned_distance(self):
        poly = Polygon([(0,0),(10,0),(10,10),(0,10)])
        vc = self._make_vc(poly, [(2,2)])
        _, dist = vc.widest_gap()
        self.assertAlmostEqual(dist, 2.0)

    def test_polygon_with_hole(self):
        # Without the hole, (0,0) would win (distance 10 to outer ring).
        # With the hole, (7,0) wins (distance 3 > distance 2 for (0,0)).
        outer = [(-10,-10),(10,-10),(10,10),(-10,10)]
        inner = [(-2,-2),(2,-2),(2,2),(-2,2)]
        poly = Polygon(outer, [inner])
        vc = self._make_vc(poly, [(0,0), (7,0)])
        point, dist = vc.widest_gap()
        self.assertEqual(point, Point(7, 0))
        self.assertAlmostEqual(dist, 3.0)


class TestStoreEdge(unittest.TestCase):
    def setUp(self):
        self.graph = voronoi_centers.VoronoiGraph()

    def test_stores_edge(self):
        # A normal edge should be recorded in all three data structures.
        self.graph.store(LineString([(0, 0), (1, 1)]))

        self.assertIn(0, self.graph.edges)
        self.assertIn((0.0, 0.0), self.graph.vertex_to_edges)
        self.assertIn((1.0, 1.0), self.graph.vertex_to_edges)
        self.assertEqual(self.graph.edge_to_vertex[0], ((0.0, 0.0), (1.0, 1.0)))

    def test_zero_length_edge_ignored(self):
        # An edge whose start and end are the same point has zero length and
        # should be silently dropped.
        self.graph.store(LineString([(5, 5), (5, 5)]))

        self.assertEqual(self.graph.edges, {})

    def test_replace_index(self):
        # Storing with replace_index= overwrites the existing edge without
        # bumping edge_count.
        self.graph.store(LineString([(0, 0), (1, 1)]))
        self.assertEqual(self.graph.edge_count, 1)

        replacement = LineString([(0, 0), (2, 2)])
        self.graph.store(replacement, replace_index=0)

        self.assertEqual(self.graph.edge_count, 1)   # count unchanged
        self.assertIs(self.graph.edges[0], replacement)


class TestRemoveEdge(unittest.TestCase):
    def _make_two_edge_graph(self):
        # Graph: A --edge0-- B --edge1-- C
        A, B, C = (0.0, 0.0), (5.0, 0.0), (10.0, 0.0)
        g = _make_graph(
            edges={0: LineString([A, B]), 1: LineString([B, C])},
            edge_to_vertex={0: (A, B), 1: (B, C)},
            vertex_to_edges={A: [0], B: [0, 1], C: [1]},
        )
        return g, A, B, C

    def test_removes_edge_and_orphaned_vertex(self):
        # Removing edge0 should delete its dead-end vertex A entirely.
        g, A, B, C = self._make_two_edge_graph()
        g.remove(0)

        self.assertNotIn(0, g.edges)
        self.assertNotIn(0, g.edge_to_vertex)
        self.assertNotIn(A, g.vertex_to_edges)

    def test_shared_vertex_remains(self):
        # The vertex shared by both edges must survive after one edge is removed.
        g, A, B, C = self._make_two_edge_graph()
        g.remove(0)

        self.assertIn(B, g.vertex_to_edges)

    def test_single_edge_not_removed(self):
        # When only one edge exists the guard prevents removal to avoid
        # leaving an empty graph.
        A, B = (0.0, 0.0), (1.0, 0.0)
        g = _make_graph(
            edges={0: LineString([A, B])},
            edge_to_vertex={0: (A, B)},
            vertex_to_edges={A: [0], B: [0]},
        )
        g.remove(0)

        self.assertIn(0, g.edges)


class TestVertexesToEdge(unittest.TestCase):
    """Uses the shared boilerplate graph."""

    def setUp(self):
        self.graph = _make_graph(
            vertex_to_edges={
                (10.1, 10.1): [1, 2],
                (20.2, 10.1): [2, 4],
                (20.2, 20.2): [4, 5, 3],
                (10.1, 20.1): [1, 6, 5],
                (30.3, 20.2): [3],
                (-10, 20):    [6],
            },
            edge_to_vertex={
                1: ((10.1, 10.1), (10.1, 20.1)),
                2: ((20.2, 10.1), (10.1, 10.1)),
                3: ((30.3, 20.2), (20.2, 20.2)),
                4: ((20.2, 20.2), (20.2, 10.1)),
                5: ((20.2, 20.2), (10.1, 20.1)),
                6: ((10.1, 20.1), (-10, 20)),
            },
            edges={},
        )

    def test_connected_vertices_returns_edge(self):
        # (10.1, 10.1) and (10.1, 20.1) are the endpoints of edge 1.
        result = self.graph.vertexes_to_edge((10.1, 10.1), (10.1, 20.1))
        self.assertEqual(result, 1)

    def test_unconnected_vertices_returns_none(self):
        # (10.1, 10.1) and (30.3, 20.2) have no direct edge between them.
        result = self.graph.vertexes_to_edge((10.1, 10.1), (30.3, 20.2))
        self.assertIsNone(result)


class TestCombineEdges(unittest.TestCase):
    def _make_chain_vc(self):
        # Graph:  A --0-- B --1-- C --2-- D
        #                         |
        #                         3
        #                         |
        #                         E
        #
        # Only B has exactly 2 edges and is the merge candidate.
        # C has 3 edges so it is not in the merge queue.
        # When B is processed: removing edge 0 leaves 3 edges (guard won't fire),
        # removing edge 1 leaves 2 edges (guard still won't fire).
        A = (0.0, 0.0)
        B = (5.0, 0.0)
        C = (10.0, 0.0)
        D = (15.0, 0.0)
        E = (10.0, -5.0)
        vc = voronoi_centers.VoronoiCenters(Polygon())
        vc.graph = _make_graph(
            edges={
                0: LineString([A, B]),
                1: LineString([B, C]),
                2: LineString([C, D]),
                3: LineString([C, E]),
            },
            edge_to_vertex={0: (A, B), 1: (B, C), 2: (C, D), 3: (C, E)},
            vertex_to_edges={A: [0], B: [0, 1], C: [1, 2, 3], D: [2], E: [3]},
        )
        return vc, A, B, C, D, E

    def test_two_edge_vertex_merged(self):
        # After combining, B is gone and A—C are joined into a single edge.
        vc, A, B, C, D, _E = self._make_chain_vc()
        vc._combine_edges(set())

        self.assertNotIn(B, vc.graph.vertex_to_edges)
        self.assertIn(A, vc.graph.vertex_to_edges)
        self.assertIn(C, vc.graph.vertex_to_edges)

    def test_preserve_set_prevents_merge(self):
        # When B is in the preserve set the merge must not happen.
        vc, A, B, C, D, _E = self._make_chain_vc()
        vc._combine_edges({B})

        self.assertIn(B, vc.graph.vertex_to_edges)
        self.assertEqual(len(vc.graph.edges), 4)

    def test_loop_on_stick_not_merged(self):
        # A—B and B—A share both endpoints. _combine_edges must leave them alone
        # because linemerge cannot determine which end is the start.
        A, B = (0.0, 0.0), (5.0, 0.0)
        vc = voronoi_centers.VoronoiCenters(Polygon())
        vc.graph = _make_graph(
            edges={0: LineString([A, B]), 1: LineString([B, A])},
            edge_to_vertex={0: (A, B), 1: (B, A)},
            vertex_to_edges={A: [0, 1], B: [0, 1]},
        )
        vc._combine_edges(set())

        self.assertEqual(len(vc.graph.edges), 2)


class TestRemoveTrivial(unittest.TestCase):
    def _make_vc_with_short_deadend(self):
        # Graph: A --long edge-- B --tiny edge-- C
        # C is a dead end and the B—C edge is below EPS*10.
        short = voronoi_centers.EPS * 5   # shorter than EPS*10
        A = (0.0, 0.0)
        B = (10.0, 0.0)
        C = (10.0 + short, 0.0)
        vc = voronoi_centers.VoronoiCenters(Polygon())
        vc.graph = _make_graph(
            edges={0: LineString([A, B]), 1: LineString([B, C])},
            edge_to_vertex={0: (A, B), 1: (B, C)},
            vertex_to_edges={A: [0], B: [0, 1], C: [1]},
        )
        return vc, A, B, C

    def test_short_deadend_removed(self):
        # The tiny dead-end edge and its orphaned vertex C should be pruned.
        vc, A, B, C = self._make_vc_with_short_deadend()
        vc._remove_trivial(set())

        self.assertNotIn(1, vc.graph.edges)
        self.assertNotIn(C, vc.graph.vertex_to_edges)

    def test_short_edge_preserved_by_preserve_set(self):
        # When C's coordinate is in the preserve set the edge must survive.
        vc, A, B, C = self._make_vc_with_short_deadend()
        vc._remove_trivial({C})

        self.assertIn(1, vc.graph.edges)

    def test_duplicate_edges_collapsed(self):
        # Two short edges between the same pair of vertices should be reduced to one.
        short = voronoi_centers.EPS * 5
        A = (0.0, 0.0)
        B = (short, 0.0)
        vc = voronoi_centers.VoronoiCenters(Polygon())
        vc.graph = _make_graph(
            edges={0: LineString([A, B]), 1: LineString([A, B])},
            edge_to_vertex={0: (A, B), 1: (A, B)},
            vertex_to_edges={A: [0, 1], B: [0, 1]},
        )
        vc._remove_trivial(set())

        self.assertEqual(len(vc.graph.edges), 1)


class TestVertexOnPerimiter(unittest.TestCase):
    def _make_vc(self, vertices):
        vc = voronoi_centers.VoronoiCenters(Polygon())
        vc.polygon = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
        vc.max_dist = 1000
        vc.graph.vertex_to_edges = {v: [] for v in vertices}
        return vc

    def test_vertex_on_bounding_box_returned_immediately(self):
        # (0, 5) lies on the left edge of the bounding box and should be
        # returned without checking any other vertex.
        vc = self._make_vc([(0, 5), (5, 5)])
        result = vc.vertex_on_perimiter()
        self.assertEqual(result, Point(0, 5))

    def test_nearest_vertex_returned_when_none_on_box(self):
        # Neither vertex touches the bounding box.
        # (9, 5) is distance 1 from the right edge; (2, 5) is distance 2 from
        # the left edge.  The closer one should win.
        vc = self._make_vc([(2, 5), (9, 5)])
        result = vc.vertex_on_perimiter()
        self.assertEqual(result, Point(9, 5))


class TestSetStartingPoint(unittest.TestCase):
    def _make_vc(self):
        # Minimal graph: one horizontal edge from (2,2) to (8,2) inside a
        # 10x10 square.
        A, B = (2.0, 2.0), (8.0, 2.0)
        vc = voronoi_centers.VoronoiCenters(Polygon())
        vc.polygon = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
        vc.max_dist = 1000
        vc.graph = _make_graph(
            edges={0: LineString([A, B])},
            edge_to_vertex={0: (A, B)},
            vertex_to_edges={A: [0], B: [0]},
        )
        return vc

    def test_point_outside_polygon_raises(self):
        vc = self._make_vc()
        with self.assertRaises(voronoi_centers.PointOutsidePart):
            vc.set_starting_point(Point(15, 5))

    def test_existing_vertex_used_directly(self):
        # (2, 2) is already a vertex — no new edge should be added.
        vc = self._make_vc()
        edge_count_before = len(vc.graph.edges)

        vc.set_starting_point(Point(2, 2))

        self.assertEqual(vc.start_point, Point(2, 2))
        self.assertEqual(len(vc.graph.edges), edge_count_before)

    def test_new_vertex_added_to_graph(self):
        # (5, 2) lies on the edge but is not a vertex — a new connecting edge
        # should be inserted.
        vc = self._make_vc()
        edge_count_before = len(vc.graph.edges)

        vc.set_starting_point(Point(5, 2))

        self.assertEqual(vc.start_point, Point(5, 2))
        self.assertGreater(len(vc.graph.edges), edge_count_before)


if __name__ == '__main__':
    unittest.main()
