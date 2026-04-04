#!/usr/bin/env python3

import unittest
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from shapely.geometry import LineString, Point, Polygon

from hsm_nibble.arc_utils import ArcDir
from hsm_nibble.path_assembler import PathAssembler
from hsm_nibble.path_planner import PathPlanner
from hsm_nibble.voronoi_centers import VoronoiGraph


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

STEP = 3.0


def _make_graph(*edges):
    """Build a VoronoiGraph from a sequence of LineString objects."""
    g = VoronoiGraph()
    for edge in edges:
        g.store(edge)
    return g


class _MockVoronoi:
    """Minimal stand-in for VoronoiCenters, sufficient for PathPlanner tests."""
    def __init__(self, graph, start_point=None, max_dist=100.0, distance_from_geom=None):
        self.graph = graph
        self.start_point = start_point or Point(0, 0)
        self.max_dist = max_dist
        self.distance_from_geom = distance_from_geom or (lambda pt: 3.0)


def _assembler(step=STEP):
    dilated = Polygon([(-50, -50), (50, -50), (50, 50), (-50, 50)])
    return PathAssembler(step, ArcDir.CW, Polygon(), dilated)


def _planner(graph, start_point=None, step=STEP):
    voronoi = _MockVoronoi(graph, start_point=start_point or Point(0, 0))
    assembler = _assembler(step)
    polygon = Polygon([(-50, -50), (50, -50), (50, 50), (-50, 50)])
    return PathPlanner(
        voronoi=voronoi,
        step=step,
        winding_dir=ArcDir.CW,
        assembler=assembler,
        polygon=polygon,
        dilated_polygon_boundaries=[],
    )


# ---------------------------------------------------------------------------
# _select_next_vertex
# ---------------------------------------------------------------------------

class TestSelectNextVertex(unittest.TestCase):

    def test_returns_none_when_branch_starts_empty(self):
        graph = _make_graph(LineString([(0, 0), (5, 0)]))
        planner = _planner(graph)
        result = planner._select_next_vertex()
        self.assertIsNone(result)

    def test_returns_single_available_vertex(self):
        graph = _make_graph(LineString([(0, 0), (5, 0)]))
        planner = _planner(graph)
        planner.branch_starts[0] = (5.0, 0.0)
        result = planner._select_next_vertex()
        self.assertEqual(result, (5.0, 0.0))

    def test_branch_starts_empty_after_returning_only_entry(self):
        graph = _make_graph(LineString([(0, 0), (5, 0)]))
        planner = _planner(graph)
        planner.branch_starts[0] = (5.0, 0.0)
        planner._select_next_vertex()
        self.assertEqual(planner.branch_starts, {})

    def test_prunes_visited_edges_from_branch_starts(self):
        graph = _make_graph(
            LineString([(0, 0), (5, 0)]),
            LineString([(5, 0), (10, 0)]),
        )
        planner = _planner(graph)
        planner.branch_starts[0] = (0.0, 0.0)
        planner.branch_starts[1] = (5.0, 0.0)
        planner.visited_edges.add(0)  # mark edge 0 as visited
        result = planner._select_next_vertex()
        # Edge 0 should be pruned; only edge 1's vertex returned.
        self.assertEqual(result, (5.0, 0.0))
        self.assertNotIn(0, planner.branch_starts)

    def test_returns_closest_vertex_to_current_pos(self):
        graph = _make_graph(
            LineString([(0, 0), (5, 0)]),
            LineString([(0, 0), (0, 20)]),
        )
        planner = _planner(graph)
        planner.branch_starts[0] = (5.0, 0.0)    # 5 units from (8, 0)
        planner.branch_starts[1] = (0.0, 20.0)   # ~21 units from (8, 0)
        result = planner._select_next_vertex(current_pos=(8.0, 0.0))
        self.assertEqual(result, (5.0, 0.0))

    def test_resets_last_circle(self):
        from hsm_nibble.arc_utils import create_circle
        graph = _make_graph(LineString([(0, 0), (5, 0)]))
        planner = _planner(graph)
        planner.last_circle = create_circle(Point(0, 0), 2.0)
        planner._select_next_vertex()
        self.assertIsNone(planner.last_circle)


# ---------------------------------------------------------------------------
# _merge_voronoi_edges
# ---------------------------------------------------------------------------

class TestMergeVoronoiEdges(unittest.TestCase):

    def test_single_edge_returns_linestring(self):
        graph = _make_graph(LineString([(0, 0), (5, 0)]))
        planner = _planner(graph, start_point=Point(0, 0))
        result = planner._merge_voronoi_edges((0.0, 0.0))
        self.assertIsNotNone(result)
        self.assertEqual(result.geom_type, "LineString")

    def test_single_edge_marked_visited(self):
        graph = _make_graph(LineString([(0, 0), (5, 0)]))
        planner = _planner(graph, start_point=Point(0, 0))
        planner._merge_voronoi_edges((0.0, 0.0))
        self.assertEqual(len(planner.visited_edges), 1)

    def test_chain_of_two_edges_combined(self):
        """A→B→C linear graph starting from A should return a single LineString A→C."""
        graph = _make_graph(
            LineString([(0, 0), (5, 0)]),   # edge 0: A→B
            LineString([(5, 0), (10, 0)]),  # edge 1: B→C
        )
        planner = _planner(graph, start_point=Point(0, 0))
        result = planner._merge_voronoi_edges((0.0, 0.0))
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.length, 10.0, places=5)
        self.assertEqual(len(planner.visited_edges), 2)

    def test_branch_records_sibling_in_branch_starts(self):
        """At a junction with two unvisited edges, the unwalked sibling is in branch_starts.

        branch_starts is pruned lazily (in _select_next_vertex), so after
        _merge_voronoi_edges it contains both walked and unwalked branches.
        The number of entries NOT yet visited should be exactly 1.
        """
        graph = _make_graph(
            LineString([(0, 0), (5, 0)]),   # edge 0: A→B
            LineString([(5, 0), (10, 0)]),  # edge 1: B→C (length 5)
            LineString([(5, 0), (5, 8)]),   # edge 2: B→D (length 8)
        )
        planner = _planner(graph, start_point=Point(0, 0))
        planner._merge_voronoi_edges((0.0, 0.0))
        unvisited = {k for k in planner.branch_starts if k not in planner.visited_edges}
        self.assertEqual(len(unvisited), 1)

    def test_no_edges_from_vertex_returns_none(self):
        """If start_vertex has no unvisited edges, return None."""
        graph = _make_graph(LineString([(0, 0), (5, 0)]))
        planner = _planner(graph, start_point=Point(0, 0))
        planner.visited_edges.add(0)  # mark the only edge as visited
        result = planner._merge_voronoi_edges((0.0, 0.0))
        self.assertIsNone(result)

    def test_prefers_longer_edge_at_junction(self):
        """BREADTH_FIRST=False: the longer branch should be walked first."""
        graph = _make_graph(
            LineString([(0, 0), (5, 0)]),   # edge 0: A→B
            LineString([(5, 0), (10, 0)]),  # edge 1: B→C length=5
            LineString([(5, 0), (5, 8)]),   # edge 2: B→D length=8 (longer)
        )
        planner = _planner(graph, start_point=Point(0, 0))
        result = planner._merge_voronoi_edges((0.0, 0.0))
        # Walked edges: 0 (A→B) then 2 (B→D). Total length = 5 + 8 = 13.
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.length, 5.0 + 8.0, places=5)


if __name__ == "__main__":
    unittest.main()
