#!/usr/bin/env python3

import unittest
import os, sys
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from shapely.geometry import LineString, Point, Polygon

from hsm_nibble.arc_fitter import (
    ProportionalController,
    extrapolate_line,
    arc_at_distance,
    find_best_arc_distance,
    ITERATION_COUNT,
)
from hsm_nibble.arc_utils import ArcDir, create_circle


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _square_pocket(size=20.0):
    """Return a square polygon and a simple distance_from_geom callable."""
    poly = Polygon([(-size, -size), (size, -size), (size, size), (-size, size)])
    def distance_from_geom(pt):
        return poly.boundary.distance(pt)
    return poly, distance_from_geom


def _horizontal_edge(length=10.0):
    """Return a horizontal voronoi edge of the given length."""
    return LineString([(0, 0), (length, 0)])


# ---------------------------------------------------------------------------
# ProportionalController
# ---------------------------------------------------------------------------

class TestProportionalController(unittest.TestCase):

    def test_zero_error_gives_zero_step(self):
        ctrl = ProportionalController(kp=0.76)
        self.assertAlmostEqual(ctrl.step(target=5.0, current=5.0), 0.0)

    def test_positive_error_gives_positive_step(self):
        ctrl = ProportionalController(kp=0.76)
        result = ctrl.step(target=5.0, current=3.0)
        self.assertGreater(result, 0)
        self.assertAlmostEqual(result, 0.76 * 2.0)

    def test_negative_error_gives_negative_step(self):
        ctrl = ProportionalController(kp=0.76)
        result = ctrl.step(target=3.0, current=5.0)
        self.assertLess(result, 0)
        self.assertAlmostEqual(result, 0.76 * -2.0)

    def test_kp_scales_result(self):
        ctrl_a = ProportionalController(kp=1.0)
        ctrl_b = ProportionalController(kp=0.5)
        self.assertAlmostEqual(ctrl_a.step(10.0, 6.0), 2 * ctrl_b.step(10.0, 6.0))

    def test_default_kp(self):
        """Default kp is 0.76 — verify it matches explicit construction."""
        ctrl = ProportionalController()
        ctrl_explicit = ProportionalController(kp=0.76)
        self.assertAlmostEqual(
            ctrl.step(5.0, 2.0),
            ctrl_explicit.step(5.0, 2.0)
        )


# ---------------------------------------------------------------------------
# extrapolate_line
# ---------------------------------------------------------------------------

class TestExtrapolateLine(unittest.TestCase):

    def test_length_increases_by_2x_overshoot(self):
        line = _horizontal_edge(10.0)
        extended = extrapolate_line(3.0, line)
        self.assertAlmostEqual(extended.length, 16.0, places=5)

    def test_direction_preserved_at_start(self):
        """Extended start should be in the opposite direction of the first segment."""
        line = _horizontal_edge(10.0)
        extended = extrapolate_line(5.0, line)
        # First coord should be to the left of (0,0).
        self.assertAlmostEqual(extended.coords[0][1], 0.0)
        self.assertLess(extended.coords[0][0], 0.0)

    def test_direction_preserved_at_end(self):
        """Extended end should continue past the last point."""
        line = _horizontal_edge(10.0)
        extended = extrapolate_line(5.0, line)
        self.assertGreater(extended.coords[-1][0], 10.0)

    def test_original_coords_preserved(self):
        """Interior coords of the original line must appear in the extended line."""
        line = LineString([(0, 0), (5, 1), (10, 0)])
        extended = extrapolate_line(2.0, line)
        coords = list(extended.coords)
        self.assertIn((5.0, 1.0), coords)


# ---------------------------------------------------------------------------
# arc_at_distance
# ---------------------------------------------------------------------------

class TestArcAtDistance(unittest.TestCase):

    def _flat_distance_fn(self, fixed_radius=3.0):
        """Return a distance callable that always returns fixed_radius."""
        return lambda pt: fixed_radius

    def test_returns_point_and_radius(self):
        edge = _horizontal_edge(10.0)
        pos, radius = arc_at_distance(5.0, edge, self._flat_distance_fn(3.0))
        self.assertIsInstance(pos, Point)
        self.assertAlmostEqual(radius, 3.0)

    def test_midpoint_at_midway(self):
        """At distance 5 on a 10-unit horizontal edge, x should be ~5."""
        edge = _horizontal_edge(10.0)
        pos, _ = arc_at_distance(5.0, edge, self._flat_distance_fn())
        self.assertAlmostEqual(pos.x, 5.0, places=3)
        self.assertAlmostEqual(pos.y, 0.0, places=3)

    def test_negative_distance_handled_by_overshoot(self):
        """Negative distance should resolve via the extrapolated extension."""
        edge = _horizontal_edge(10.0)
        # Should not raise; overshoot allows negative distances.
        pos, radius = arc_at_distance(-1.0, edge, self._flat_distance_fn())
        self.assertIsInstance(pos, Point)

    def test_distance_beyond_edge_handled_by_overshoot(self):
        """Distance beyond edge length should resolve via the extrapolated extension."""
        edge = _horizontal_edge(10.0)
        pos, radius = arc_at_distance(15.0, edge, self._flat_distance_fn())
        self.assertIsInstance(pos, Point)


# ---------------------------------------------------------------------------
# find_best_arc_distance
# ---------------------------------------------------------------------------

class TestFindBestArcDistance(unittest.TestCase):
    """Tests for find_best_arc_distance against a simple square pocket."""

    STEP = 3.0

    def setUp(self):
        self.poly, self.dist_fn = _square_pocket(size=20.0)
        self.ctrl = ProportionalController()
        # A horizontal edge through the middle of the square.
        self.edge = LineString([(-10, 0), (10, 0)])

    def _empty_area(self):
        """An area with just the starting point so arcs are never hidden."""
        return Point(0, 0).buffer(0.01)

    def test_returns_five_tuple(self):
        area = self._empty_area()
        result = find_best_arc_distance(
            voronoi_edge=self.edge,
            start_distance=0.0,
            min_distance=0.0,
            step=self.STEP,
            winding_dir=ArcDir.CW,
            calculated_area=area,
            last_circle=None,
            distance_from_geom=self.dist_fn,
            max_dist=30.0,
            controller=self.ctrl,
        )
        self.assertEqual(len(result), 5)

    def test_converges_without_last_circle(self):
        """When last_circle is None the function should still return a valid arc."""
        area = self._empty_area()
        best_dist, best_circle, hidden, iters, backwards = find_best_arc_distance(
            voronoi_edge=self.edge,
            start_distance=0.0,
            min_distance=0.0,
            step=self.STEP,
            winding_dir=ArcDir.CW,
            calculated_area=area,
            last_circle=None,
            distance_from_geom=self.dist_fn,
            max_dist=30.0,
            controller=self.ctrl,
        )
        self.assertFalse(hidden)
        self.assertFalse(backwards)
        self.assertIsNotNone(best_circle)

    def test_converges_with_last_circle(self):
        """With a last_circle the step spacing should converge close to STEP."""
        # Use a flat distance function so circles stay small (radius=2) and
        # don't swallow the whole pocket. The voronoi edge is horizontal at y=0.
        flat_dist = lambda pt: 2.0
        area = self._empty_area()
        origin = Point(-5.0, 0.0)
        last_circle = create_circle(origin, 2.0, ArcDir.CW)
        area = area.union(Polygon(last_circle.path))

        best_dist, best_circle, hidden, iters, backwards = find_best_arc_distance(
            voronoi_edge=self.edge,
            start_distance=0.0,
            min_distance=0.0,
            step=self.STEP,
            winding_dir=ArcDir.CW,
            calculated_area=area,
            last_circle=last_circle,
            distance_from_geom=flat_dist,
            max_dist=30.0,
            controller=self.ctrl,
        )
        self.assertFalse(hidden)
        self.assertFalse(backwards)
        self.assertLess(iters, ITERATION_COUNT)

    def test_hidden_at_start_when_area_covers_start(self):
        """If the very first proposed arc is inside calculated_area, hidden_at_start=True."""
        # Cover the entire edge with calculated_area.
        full_area = self.poly
        best_dist, best_circle, hidden, iters, backwards = find_best_arc_distance(
            voronoi_edge=self.edge,
            start_distance=0.0,
            min_distance=0.0,
            step=self.STEP,
            winding_dir=ArcDir.CW,
            calculated_area=full_area,
            last_circle=None,
            distance_from_geom=self.dist_fn,
            max_dist=30.0,
            controller=self.ctrl,
        )
        self.assertTrue(hidden)
        self.assertFalse(backwards)

    def test_best_distance_within_edge(self):
        """best_distance must not exceed voronoi_edge.length."""
        area = self._empty_area()
        best_dist, _, _, _, _ = find_best_arc_distance(
            voronoi_edge=self.edge,
            start_distance=0.0,
            min_distance=0.0,
            step=self.STEP,
            winding_dir=ArcDir.CW,
            calculated_area=area,
            last_circle=None,
            distance_from_geom=self.dist_fn,
            max_dist=30.0,
            controller=self.ctrl,
        )
        self.assertLessEqual(best_dist, self.edge.length)

    def test_best_circle_is_arc_data(self):
        from hsm_nibble.arc_utils import ArcData
        area = self._empty_area()
        _, best_circle, _, _, _ = find_best_arc_distance(
            voronoi_edge=self.edge,
            start_distance=0.0,
            min_distance=0.0,
            step=self.STEP,
            winding_dir=ArcDir.CW,
            calculated_area=area,
            last_circle=None,
            distance_from_geom=self.dist_fn,
            max_dist=30.0,
            controller=self.ctrl,
        )
        self.assertIsInstance(best_circle, ArcData)

    def test_winding_direction_propagated(self):
        """best_circle should carry the requested winding direction."""
        area = self._empty_area()
        for wd in (ArcDir.CW, ArcDir.CCW):
            _, best_circle, _, _, _ = find_best_arc_distance(
                voronoi_edge=self.edge,
                start_distance=0.0,
                min_distance=0.0,
                step=self.STEP,
                winding_dir=wd,
                calculated_area=area,
                last_circle=None,
                distance_from_geom=self.dist_fn,
                max_dist=30.0,
                controller=self.ctrl,
            )
            self.assertEqual(best_circle.winding_dir, wd)


if __name__ == "__main__":
    unittest.main()
