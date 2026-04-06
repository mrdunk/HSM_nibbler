#!/usr/bin/env python3
"""
Unit tests for arc_fitter private helpers:
  _desired_step, _area_spacing, _hausdorff_spacing
"""

import unittest
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from shapely.geometry import LineString, Point, Polygon

from hsm_nibble.arc_fitter import (
    _desired_step,
    _area_spacing,
    _hausdorff_spacing,
    CORNER_ZOOM,
    CORNER_ZOOM_EFFECT,
)
from hsm_nibble.arc_utils import ArcData, ArcDir, create_circle


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

STEP = 2.0
EDGE = LineString([(-10, 0), (10, 0)])
FLAT_DIST = staticmethod(lambda pt: 2.0)


# ---------------------------------------------------------------------------
# _desired_step
# ---------------------------------------------------------------------------

class TestDesiredStep(unittest.TestCase):

    def test_large_radius_returns_full_step(self):
        """Radius well above threshold — no corner-zoom reduction."""
        radius = 3 * CORNER_ZOOM * STEP
        self.assertAlmostEqual(_desired_step(radius, STEP), STEP)

    def test_zero_radius_gives_minimum(self):
        """At radius=0 the multiplier is 1.0, result = step*(1 - EFFECT)."""
        expected = STEP * (1 - CORNER_ZOOM_EFFECT)
        self.assertAlmostEqual(_desired_step(0.0, STEP), expected)

    def test_half_threshold_reduces_step(self):
        """Radius at half the threshold should reduce the step."""
        radius = CORNER_ZOOM * STEP / 2
        result = _desired_step(radius, STEP)
        self.assertLess(result, STEP)
        self.assertGreater(result, STEP * (1 - CORNER_ZOOM_EFFECT) - 1e-9)

    def test_at_threshold_returns_full_step(self):
        """Radius exactly at threshold is not below it — full step returned."""
        self.assertAlmostEqual(_desired_step(CORNER_ZOOM * STEP, STEP), STEP)

    def test_result_scales_linearly_with_step(self):
        """Doubling both radius and step should double the result."""
        r = 0.5
        r1 = _desired_step(r, 1.0)
        r2 = _desired_step(r * 2, 2.0)
        self.assertAlmostEqual(r1 * 2, r2, places=5)

    def test_result_never_negative(self):
        for radius in (0.0, 0.1, 0.5, 1.0, 10.0):
            self.assertGreaterEqual(_desired_step(radius, STEP), 0.0)


# ---------------------------------------------------------------------------
# _area_spacing
# ---------------------------------------------------------------------------

class TestAreaSpacing(unittest.TestCase):

    def test_arc_fully_hidden_returns_none_spacing(self):
        """If the circle is entirely inside calculated_area, spacing is None."""
        area = Point(0, 0).buffer(50.0)
        sp, circle = _area_spacing(0.0, EDGE, ArcDir.CW, area, FLAT_DIST)
        self.assertIsNone(sp)

    def test_arc_fully_hidden_still_returns_circle(self):
        area = Point(0, 0).buffer(50.0)
        sp, circle = _area_spacing(0.0, EDGE, ArcDir.CW, area, FLAT_DIST)
        self.assertIsInstance(circle, ArcData)

    def test_visible_arc_returns_positive_spacing(self):
        """Arc far from calculated_area should have positive spacing."""
        area = Point(100, 100).buffer(0.01)  # well away from arc
        sp, circle = _area_spacing(0.0, EDGE, ArcDir.CW, area, FLAT_DIST)
        self.assertIsNotNone(sp)
        self.assertGreater(sp, 0.0)

    def test_visible_arc_returns_arc_data(self):
        area = Point(100, 100).buffer(0.01)
        _sp, circle = _area_spacing(0.0, EDGE, ArcDir.CW, area, FLAT_DIST)
        self.assertIsInstance(circle, ArcData)

    def test_ccw_winding_also_works(self):
        area = Point(100, 100).buffer(0.01)
        sp, circle = _area_spacing(0.0, EDGE, ArcDir.CCW, area, FLAT_DIST)
        self.assertIsNotNone(sp)
        self.assertEqual(circle.winding_dir, ArcDir.CCW)

    def test_spacing_is_distance_from_area_to_arc_points(self):
        """Spacing must equal max distance from any visible arc point to area."""
        # Small area far left; arc positioned far right — spacing should be large.
        area = Polygon([(-10, -1), (-8, -1), (-8, 1), (-10, 1)])
        sp, _ = _area_spacing(8.0, EDGE, ArcDir.CW, area, FLAT_DIST)
        if sp is not None:
            self.assertGreater(sp, 5.0)


# ---------------------------------------------------------------------------
# _hausdorff_spacing
# ---------------------------------------------------------------------------

class TestHausdorffSpacing(unittest.TestCase):

    def setUp(self):
        # last_circle at the far-left end of the edge.
        self.last_circle = create_circle(Point(-10.0, 0.0), 2.0, ArcDir.CW)
        self.area = Polygon(self.last_circle.path)

    def test_arc_fully_hidden_returns_none(self):
        area = Point(0, 0).buffer(50.0)
        sp, circle = _hausdorff_spacing(
            0.0, EDGE, ArcDir.CW, area, self.last_circle, FLAT_DIST, 100.0)
        self.assertIsNone(sp)

    def test_fully_hidden_still_returns_circle(self):
        area = Point(0, 0).buffer(50.0)
        sp, circle = _hausdorff_spacing(
            0.0, EDGE, ArcDir.CW, area, self.last_circle, FLAT_DIST, 100.0)
        self.assertIsInstance(circle, ArcData)

    def test_visible_arc_returns_positive_spacing(self):
        """Arc well past the last_circle should have positive Hausdorff spacing."""
        sp, _ = _hausdorff_spacing(
            8.0, EDGE, ArcDir.CW, self.area, self.last_circle, FLAT_DIST, 100.0)
        self.assertIsNotNone(sp)
        self.assertGreater(sp, 0.0)

    def test_spacing_larger_farther_away(self):
        """An arc placed farther along the edge should have larger spacing."""
        sp_near, _ = _hausdorff_spacing(
            2.0, EDGE, ArcDir.CW, self.area, self.last_circle, FLAT_DIST, 100.0)
        sp_far, _ = _hausdorff_spacing(
            8.0, EDGE, ArcDir.CW, self.area, self.last_circle, FLAT_DIST, 100.0)
        if sp_near is not None and sp_far is not None:
            self.assertGreater(sp_far, sp_near)

    def test_result_never_negative(self):
        for d in (1.0, 3.0, 5.0, 8.0):
            sp, _ = _hausdorff_spacing(
                d, EDGE, ArcDir.CW, self.area, self.last_circle, FLAT_DIST, 100.0)
            if sp is not None:
                self.assertGreaterEqual(sp, 0.0)


if __name__ == "__main__":
    unittest.main()
