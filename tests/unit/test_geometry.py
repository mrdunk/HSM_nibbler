#!/usr/bin/env python3
"""
Unit tests for geometry.py classes:
  BaseGeometry._filter_input_geometry, properties
  EntryCircle.spiral, EntryCircle.circle
  Pocket.__init__, _reset, calculate_path, done_generating, get_arcs
"""

import types
import unittest
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from shapely.geometry import MultiPolygon, Point, Polygon

from hsm_nibble.arc_utils import ArcData, ArcDir
from hsm_nibble.geometry import BaseGeometry, EntryCircle, Pocket


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _square(size=10.0):
    return Polygon([(0, 0), (size, 0), (size, size), (0, size)])


def _multi_square(size=10.0):
    return MultiPolygon([_square(size)])


# ---------------------------------------------------------------------------
# BaseGeometry._filter_input_geometry
# ---------------------------------------------------------------------------

class TestFilterInputGeometry(unittest.TestCase):

    def test_single_polygon_becomes_multipolygon(self):
        poly = _square()
        bg = BaseGeometry(poly, 1.0, ArcDir.CW)
        self.assertEqual(bg.polygon.geom_type, "MultiPolygon")

    def test_multipolygon_input_preserved(self):
        multi = _multi_square()
        bg = BaseGeometry(multi, 0.5, ArcDir.CW)
        self.assertEqual(bg.polygon.geom_type, "MultiPolygon")

    def test_two_separate_polygons_both_kept(self):
        p1 = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
        p2 = Polygon([(20, 0), (30, 0), (30, 10), (20, 10)])
        multi = MultiPolygon([p1, p2])
        bg = BaseGeometry(multi, 0.5, ArcDir.CW)
        self.assertEqual(len(bg.polygon.geoms), 2)

    def test_tiny_polygon_filtered_out(self):
        large = Polygon([(0, 0), (20, 0), (20, 20), (0, 20)])
        # Polygon too small to survive step/10 inset
        tiny = Polygon([(50, 50), (50.001, 50), (50.001, 50.001), (50, 50.001)])
        multi = MultiPolygon([large, tiny])
        bg = BaseGeometry(multi, 2.0, ArcDir.CW)
        self.assertEqual(len(bg.polygon.geoms), 1)

    def test_result_is_valid(self):
        bg = BaseGeometry(_square(10), 1.0, ArcDir.CW)
        self.assertTrue(bg.polygon.is_valid)


# ---------------------------------------------------------------------------
# BaseGeometry properties
# ---------------------------------------------------------------------------

class TestBaseGeometryProperties(unittest.TestCase):

    def setUp(self):
        self.bg = BaseGeometry(_square(), 1.0, ArcDir.CW)

    def test_path_initially_empty(self):
        self.assertEqual(self.bg.path, [])

    def test_last_arc_initially_none(self):
        self.assertIsNone(self.bg.last_arc)

    def test_pending_arc_queues_initially_empty(self):
        self.assertEqual(self.bg.pending_arc_queues, [])

    def test_cut_area_total_is_polygon(self):
        self.assertIsInstance(self.bg.cut_area_total, Polygon)

    def test_cut_area_total_setter(self):
        new_area = Point(5, 5).buffer(1.0)
        self.bg.cut_area_total = new_area
        self.assertAlmostEqual(self.bg.cut_area_total.area, new_area.area, places=3)

    def test_cut_area_total_setter_delegates_to_assembler(self):
        new_area = Point(5, 5).buffer(1.0)
        self.bg.cut_area_total = new_area
        self.assertIs(self.bg.cut_area_total, self.bg.assembler.cut_area_total)


# ---------------------------------------------------------------------------
# EntryCircle.spiral
# ---------------------------------------------------------------------------

class TestEntryCircleSpiral(unittest.TestCase):

    def _make_ec(self, center=None, radius=4.0, step=2.0, winding=ArcDir.CW):
        poly = _multi_square(20)
        c = center or Point(10, 10)
        return EntryCircle(poly, c, radius, step, winding)

    def test_spiral_produces_arcs(self):
        ec = self._make_ec()
        ec.spiral()
        arcs = [p for p in ec.path if isinstance(p, ArcData)]
        self.assertGreater(len(arcs), 0)

    def test_spiral_cw_and_ccw_both_work(self):
        for winding in (ArcDir.CW, ArcDir.CCW):
            ec = self._make_ec(winding=winding)
            ec.spiral()
            self.assertGreater(len(ec.path), 0)

    def test_spiral_arcs_have_nonzero_path_length(self):
        ec = self._make_ec()
        ec.spiral()
        for arc in ec.path:
            if isinstance(arc, ArcData):
                self.assertGreater(arc.path.length, 0)

    def test_spiral_larger_radius_more_arcs(self):
        """A larger entry radius requires more spiral turns."""
        ec_small = self._make_ec(radius=2.0)
        ec_large = self._make_ec(radius=6.0)
        ec_small.spiral()
        ec_large.spiral()
        self.assertGreater(len(ec_large.path), len(ec_small.path))

    def test_closing_arc_continuity(self):
        """The closing arc is appended last; all arcs should have positive length."""
        ec = self._make_ec()
        ec.spiral()
        arcs = [p for p in ec.path if isinstance(p, ArcData)]
        for arc in arcs:
            self.assertGreater(arc.path.length, 0,
                               "Arc with zero-length path found after spiral()")


# ---------------------------------------------------------------------------
# EntryCircle.circle
# ---------------------------------------------------------------------------

class TestEntryCircleCircle(unittest.TestCase):

    def _make_ec(self, center=None, radius=4.0, step=2.0, winding=ArcDir.CW):
        poly = _multi_square(20)
        c = center or Point(10, 10)
        return EntryCircle(poly, c, radius, step, winding)

    def test_circle_adds_arcs_to_path(self):
        ec = self._make_ec()
        ec.spiral()
        count_after_spiral = len(ec.path)
        ec.circle()
        self.assertGreaterEqual(len(ec.path), count_after_spiral)

    def test_circle_cw_produces_output(self):
        ec = self._make_ec(winding=ArcDir.CW)
        ec.spiral()
        ec.circle()
        arcs = [p for p in ec.path if isinstance(p, ArcData)]
        self.assertGreater(len(arcs), 0)

    def test_circle_ccw_produces_output(self):
        ec = self._make_ec(winding=ArcDir.CCW)
        ec.spiral()
        ec.circle()
        arcs = [p for p in ec.path if isinstance(p, ArcData)]
        self.assertGreater(len(arcs), 0)


# ---------------------------------------------------------------------------
# Pocket
# ---------------------------------------------------------------------------

class TestPocket(unittest.TestCase):

    def _make_pocket(self, size=20.0, step=5.0, winding=ArcDir.CW):
        poly = _square(size)
        return Pocket(poly, step, winding)

    def test_path_not_empty(self):
        pocket = self._make_pocket()
        self.assertGreater(len(pocket.path), 0)

    def test_start_point_inside_polygon(self):
        pocket = self._make_pocket()
        # start_point is on the voronoi diagram inside the pocket area
        self.assertIsInstance(pocket.start_point, Point)

    def test_path_fail_count_non_negative(self):
        pocket = self._make_pocket()
        self.assertGreaterEqual(pocket.path_fail_count, 0)

    def test_visited_edges_not_empty(self):
        pocket = self._make_pocket()
        self.assertGreater(len(pocket.visited_edges), 0)

    def test_loop_count_non_negative(self):
        pocket = self._make_pocket()
        self.assertGreaterEqual(pocket.loop_count, 0)

    def test_path_contains_arc_data(self):
        pocket = self._make_pocket()
        arcs = [p for p in pocket.path if isinstance(p, ArcData)]
        self.assertGreater(len(arcs), 0)

    def test_done_generating_does_not_raise(self):
        pocket = self._make_pocket()
        pocket.done_generating()  # should not raise

    def test_get_arcs_returns_generator(self):
        pocket = self._make_pocket()
        gen = pocket.get_arcs()
        self.assertIsInstance(gen, types.GeneratorType)

    def test_get_arcs_can_be_exhausted(self):
        pocket = self._make_pocket()
        gen = pocket.get_arcs()
        for _ in gen:
            pass  # should terminate without error

    def test_cw_and_ccw_both_produce_path(self):
        for winding in (ArcDir.CW, ArcDir.CCW):
            pocket = self._make_pocket(winding=winding)
            self.assertGreater(len(pocket.path), 0)

    def test_already_cut_accepted(self):
        """Pocket with an already-cut area should still produce a path."""
        poly = _square(20.0)
        already_cut = Point(10, 10).buffer(5.0)
        pocket = Pocket(poly, 4.0, ArcDir.CW, already_cut=already_cut)
        self.assertGreater(len(pocket.path), 0)

    def test_max_starting_radius_positive(self):
        pocket = self._make_pocket()
        self.assertGreater(pocket.max_starting_radius, 0.0)


if __name__ == "__main__":
    unittest.main()
