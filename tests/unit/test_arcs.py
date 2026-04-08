#!/usr/bin/env python3

import unittest
import os, sys
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from shapely.geometry import LineString, Point, Polygon

import hsm_nibble.geometry as geometry
import hsm_nibble.arc_utils as arc_utils


ACCURACY = 0.05


class BaseTests(unittest.TestCase):
    def verify_arc(self, arc):
        self.verify_arc_points(arc)
        self.verify_arc_angles(arc)

    def verify_arc_points(self, arc):
        self.assertIsNotNone(arc.radius)
        self.assertIsNotNone(arc.start)
        self.assertIsNotNone(arc.end)
        self.assertIsNotNone(arc.path)
        self.assertIsNotNone(arc.origin)

        self.assertEqual(arc.start, Point(arc.path.coords[0]))
        self.assertEqual(arc.end, Point(arc.path.coords[-1]))

        for point in arc.path.coords:
            self.assertAlmostEqual(arc.radius, arc.origin.distance(Point(point)))

    def verify_arc_angles(self, arc):
        self.assertIsNotNone(arc.start_angle)
        self.assertIsNotNone(arc.span_angle)
        self.assertIsNotNone(arc.winding_dir)

        start_angle_observed = math.atan2(arc.start.x - arc.origin.x, arc.start.y - arc.origin.y)
        start_angle_observed = start_angle_observed % (math.pi * 2)
        end_angle_observed = math.atan2(arc.end.x - arc.origin.x, arc.end.y - arc.origin.y)
        end_angle_observed = end_angle_observed % (math.pi * 2)

        self.assertAlmostEqual(start_angle_observed, arc.start_angle)
        self.assertAlmostEqual(end_angle_observed, (arc.start_angle + arc.span_angle) % (2 * math.pi))

        self.assertAlmostEqual(arc.path.length, arc.radius * abs(arc.span_angle), places=1)

        if arc.winding_dir == geometry.ArcDir.CW:
            self.assertGreater(arc.span_angle, 0)
        elif arc.winding_dir == geometry.ArcDir.CCW:
            self.assertLess(arc.span_angle, 0)


class TestCircle(unittest.TestCase):
    def test_create(self):
        """ Check circles are create in a sane way. """
        origin = Point(10, 10)
        radius = 7

        circle = arc_utils.create_circle(origin = origin, radius = radius)

        self.assertEqual(circle.origin, origin)
        self.assertEqual(circle.radius, radius)
        self.assertIsNone(circle.start)
        self.assertIsNone(circle.end)
        self.assertEqual(circle.start_angle, 0)
        self.assertEqual(circle.span_angle, 2 * math.pi)
        self.assertIsNotNone(circle.path)
        self.assertEqual(circle.path.coords[0], circle.path.coords[-1])
        self.assertNotEqual(circle.path.coords[0], circle.path.coords[1])

        mid_point = circle.path.coords[int(len(circle.path.coords)/2)]
        length_to_mid = LineString([circle.path.coords[0], mid_point]).length
        self.assertEqual(length_to_mid, 2 * radius)

class TestCreateArc(BaseTests):
    def test_create_cw(self):
        """ Check create_arc() method works in a sane way for ClockWise arcs. """
        origin = Point(100, 100)
        radius = 5
        start_angle = geometry.math.pi / 2
        span_angle = geometry.math.pi / 2
        winding_dir = geometry.ArcDir.CW

        arc = arc_utils.create_arc(origin, radius, start_angle, span_angle, winding_dir)

        self.verify_arc(arc)

    def test_create_ccw(self):
        """ Check create_arc() method works in a sane way for CounterClockWise arcs. """
        origin = Point(-123, -123)
        radius = 5
        start_angle = 3 * geometry.math.pi / 2
        span_angle = -geometry.math.pi / 4
        winding_dir = geometry.ArcDir.CCW

        arc = arc_utils.create_arc(origin, radius, start_angle, span_angle, winding_dir)

        self.verify_arc(arc)

    def test_create_cw_full_circle(self):
        """ Check create_arc() method works in a sane way for CounterClockWise arcs. """
        origin = Point(123, 123)
        radius = 5
        start_angle = 0
        span_angle = math.pi * 2
        winding_dir = geometry.ArcDir.CW

        arc = arc_utils.create_arc(origin, radius, start_angle, span_angle, winding_dir)

        self.verify_arc(arc)

    def test_create_ccw_full_circle(self):
        """ Check create_arc() method works in a sane way for CounterClockWise arcs. """
        origin = Point(-100, -100)
        radius = 5
        start_angle = 0
        span_angle = -math.pi * 2
        winding_dir = geometry.ArcDir.CCW

        arc = arc_utils.create_arc(origin, radius, start_angle, span_angle, winding_dir)

        self.verify_arc(arc)


class TestArc(BaseTests):
    def test_create_arc_from_path_CW(self):
        """ Create a Clockwise arc. """
        origin = Point(123.45, 123.45)
        radius = 7
        winding_dir = geometry.ArcDir.CW

        # This path will start at the coordinates: (radius, 0)
        path = list(arc_utils.create_circle(origin, radius).path.coords)

        path = path[int(len(path) / 8) : int(7 * len(path) / 8)]

        arc = arc_utils.create_arc_from_path(origin, LineString(path), radius)
        self.verify_arc_points(arc)
        arc = arc_utils.complete_arc(arc, winding_dir)

        self.assertEqual(arc.origin, origin)
        self.assertEqual(arc.radius, radius)
        self.assertEqual(arc.winding_dir, winding_dir)

        # We start this arc at the coordinate point: (radius, 0) with the origin at (0, 0).
        # Angle rotated from full circle's start/end point:
        expected_start_angle = (1 / 8) * (2 * math.pi)
        # Add to full circle's start angle.
        expected_start_angle = (math.pi / 2) + expected_start_angle

        self.assertLess(abs(arc.start_angle - expected_start_angle), ACCURACY * math.pi * 2 )

        self.verify_arc(arc)

    def test_create_arc_from_path_CCW(self):
        """ Create a Counter-Clockwise arc. """
        origin = Point(-123.45, -123.45)
        radius = 7
        winding_dir = geometry.ArcDir.CCW

        # This path will start at the coordinates: (radius, 0)
        path = list(arc_utils.create_circle(origin, radius).path.coords)

        path = path[int(len(path) / 8) : int(7 * len(path) / 8)]

        arc = arc_utils.create_arc_from_path(origin, LineString(path), radius)
        self.verify_arc_points(arc)
        arc = arc_utils.complete_arc(arc, winding_dir)

        self.assertEqual(arc.origin, origin)
        self.assertEqual(arc.radius, radius)
        self.assertEqual(arc.winding_dir, winding_dir)

        # We start this arc at the coordinate point: (radius, 0) with the origin at (0, 0).
        # Angle rotated from full circle's start/end point:
        expected_start_angle = (1 / 8) * (2 * math.pi)
        # Add to full circle's start angle.
        expected_start_angle = (math.pi / 2) - expected_start_angle
        self.assertLess(abs(arc.start_angle - expected_start_angle), ACCURACY * math.pi * 2 )

        self.verify_arc(arc)


class TestMirrorArc(BaseTests):
    def test_mirror_arc_from_path_CW(self):
        """ Mirror a Clockwise arc. """
        origin = Point(100, 100)
        radius = 5
        span_angle = geometry.math.pi / 2
        winding_dir = geometry.ArcDir.CW
        mirror_axis = 123

        for start_angle_multiplier in range(8):
            start_angle = start_angle_multiplier * math.pi * 2 / 8
            arc = arc_utils.create_arc(origin, radius, start_angle, span_angle, winding_dir)
            self.verify_arc(arc)

            mirrored_arc = arc_utils.mirror_arc(mirror_axis, arc)
            self.verify_arc(mirrored_arc)
            self.assertEqual(mirrored_arc.winding_dir, geometry.ArcDir.CCW)

    def test_mirror_arc_from_path_CCW(self):
        """ Mirror a Counter Clockwise arc. """
        origin = Point(100, 100)
        radius = 5
        span_angle = -geometry.math.pi / 2
        winding_dir = geometry.ArcDir.CCW
        mirror_axis = 123

        for start_angle_multiplier in range(8):
            start_angle = start_angle_multiplier * math.pi * 2 / 8
            arc = arc_utils.create_arc(origin, radius, start_angle, span_angle, winding_dir)
            self.verify_arc(arc)

            mirrored_arc = arc_utils.mirror_arc(mirror_axis, arc)
            self.verify_arc(mirrored_arc)
            self.assertEqual(mirrored_arc.winding_dir, geometry.ArcDir.CW)

    def test_mirror_arc_from_path_CW_force_dir_no_change(self):
        """ Try to mirror a Clockwise arc but no change in direction specified."""
        origin = Point(100, 100)
        radius = 5
        span_angle = geometry.math.pi / 2
        winding_dir = geometry.ArcDir.CW
        mirror_axis = 123
        start_angle = math.pi * 2 / 8

        arc = arc_utils.create_arc(origin, radius, start_angle, span_angle, winding_dir)
        self.verify_arc(arc)

        mirrored_arc = arc_utils.mirror_arc(mirror_axis, arc, geometry.ArcDir.CW)

        self.assertIs(arc, mirrored_arc)

    def test_mirror_arc_from_path_CW_force_dir_change(self):
        """ Mirror a Clockwise arc. Force winding_dir to a set state."""
        origin = Point(100, 100)
        radius = 5
        span_angle = geometry.math.pi / 2
        winding_dir = geometry.ArcDir.CW
        mirror_axis = 123

        for start_angle_multiplier in range(8):
            start_angle = start_angle_multiplier * math.pi * 2 / 8
            arc = arc_utils.create_arc(origin, radius, start_angle, span_angle, winding_dir)
            self.verify_arc(arc)

            mirrored_arc = arc_utils.mirror_arc(mirror_axis, arc, geometry.ArcDir.CCW)
            self.verify_arc(mirrored_arc)

class TestCompleteArc(BaseTests):
    """Tests for complete_arc() edge cases not covered by TestArc."""

    def test_zero_length_path_returns_none(self):
        origin = Point(0, 0)
        radius = 5
        circle = arc_utils.create_circle(origin, radius)
        zero_path = LineString([circle.path.coords[0], circle.path.coords[0]])
        arc = arc_utils.create_arc_from_path(origin, zero_path, radius)
        # complete_arc should return None for a zero-length path.
        result = arc_utils.complete_arc(arc)
        self.assertIsNone(result)

    def test_cw_span_angle_positive(self):
        origin = Point(0, 0)
        radius = 5
        arc = arc_utils.create_arc(origin, radius, 0, math.pi / 2, arc_utils.ArcDir.CW)
        result = arc_utils.complete_arc(arc, arc_utils.ArcDir.CW)
        self.assertGreater(result.span_angle, 0)
        self.assertEqual(result.winding_dir, arc_utils.ArcDir.CW)

    def test_ccw_span_angle_negative(self):
        origin = Point(0, 0)
        radius = 5
        arc = arc_utils.create_arc(origin, radius, 0, -math.pi / 2, arc_utils.ArcDir.CCW)
        result = arc_utils.complete_arc(arc, arc_utils.ArcDir.CCW)
        self.assertLess(result.span_angle, 0)
        self.assertEqual(result.winding_dir, arc_utils.ArcDir.CCW)

    def test_closest_defaults_to_cw(self):
        origin = Point(0, 0)
        radius = 5
        arc = arc_utils.create_arc(origin, radius, 0, math.pi / 2, arc_utils.ArcDir.CW)
        result = arc_utils.complete_arc(arc, arc_utils.ArcDir.CLOSEST)
        self.assertEqual(result.winding_dir, arc_utils.ArcDir.CW)
        self.assertGreater(result.span_angle, 0)

    def test_reversal_when_direction_mismatched(self):
        """complete_arc should reverse the path if the existing direction contradicts winding_dir."""
        origin = Point(0, 0)
        radius = 5
        # Create a CW arc then ask complete_arc to treat it as CCW — it should reverse.
        arc = arc_utils.create_arc(origin, radius, 0, math.pi / 2, arc_utils.ArcDir.CW)
        result = arc_utils.complete_arc(arc, arc_utils.ArcDir.CCW)
        self.assertIsNotNone(result)
        self.assertEqual(result.winding_dir, arc_utils.ArcDir.CCW)
        self.assertLess(result.span_angle, 0)
        self.verify_arc(result)


class TestArcsFromCircleDiff(unittest.TestCase):

    def setUp(self):
        self.origin = Point(0, 0)
        self.radius = 10
        self.circle = arc_utils.create_circle(self.origin, self.radius)

    def test_empty_already_cut_returns_circle(self):
        result = arc_utils.arcs_from_circle_diff(self.circle, Polygon())
        self.assertEqual(len(result), 1)
        self.assertAlmostEqual(result[0].path.length, self.circle.path.length, places=1)

    def test_circle_entirely_inside_cut_returns_empty(self):
        already_cut = self.origin.buffer(self.radius * 2)
        result = arc_utils.arcs_from_circle_diff(self.circle, already_cut)
        self.assertEqual(result, [])

    def test_half_circle_cut_returns_one_arc(self):
        # Cut away the right half of the circle.
        already_cut = Polygon([(0, -20), (20, -20), (20, 20), (0, 20)])
        result = arc_utils.arcs_from_circle_diff(self.circle, already_cut)
        self.assertEqual(len(result), 1)
        # Remaining arc should be roughly half the circumference.
        self.assertAlmostEqual(
            result[0].path.length, math.pi * self.radius, delta=self.radius * 0.1)

    def test_two_cuts_returns_two_arcs(self):
        # Cut away two opposite wedges, leaving two separate arcs.
        cut1 = Polygon([(-1, -20), (1, -20), (1, 20), (-1, 20)])  # vertical strip
        cut2 = Polygon([(-20, -1), (20, -1), (20, 1), (-20, 1)])  # horizontal strip
        already_cut = cut1.union(cut2)
        result = arc_utils.arcs_from_circle_diff(self.circle, already_cut)
        self.assertGreater(len(result), 1)


class TestSplitArcs(unittest.TestCase):

    def setUp(self):
        self.origin = Point(0, 0)
        self.radius = 10
        self.circle = arc_utils.create_circle(self.origin, self.radius)

    def test_arc_outside_calculated_area_returned_unchanged(self):
        calculated_area = Polygon()
        result = arc_utils.split_arcs([self.circle], calculated_area)
        self.assertEqual(len(result), 1)

    def test_arc_entirely_inside_calculated_area_filtered_out(self):
        calculated_area = self.origin.buffer(self.radius * 2)
        result = arc_utils.split_arcs([self.circle], calculated_area)
        self.assertEqual(result, [])

    def test_partial_overlap_trims_arc(self):
        # Cut away the right half.
        calculated_area = Polygon([(0, -20), (20, -20), (20, 20), (0, 20)])
        result = arc_utils.split_arcs([self.circle], calculated_area)
        self.assertEqual(len(result), 1)
        self.assertLess(result[0].path.length, self.circle.path.length)

    def test_order_preserved_across_multiple_arcs(self):
        # Two circles at different positions — order in output should match input.
        circle2 = arc_utils.create_circle(Point(50, 0), self.radius)
        calculated_area = Polygon()
        result = arc_utils.split_arcs([self.circle, circle2], calculated_area)
        self.assertEqual(len(result), 2)

    def test_empty_input_returns_empty(self):
        result = arc_utils.split_arcs([], Polygon())
        self.assertEqual(result, [])


class TestNoRepeatedPoints(unittest.TestCase):
    """create_circle and create_arc must not produce paths with consecutive
    duplicate coordinates — Shapely 2.0.7 warns (invalid value in
    line_locate_point) when .project() is called on such a LineString."""

    def _assert_no_dupes(self, path):
        coords = list(path.coords)
        for a, b in zip(coords, coords[1:]):
            self.assertNotEqual(a, b, f"consecutive duplicate coord {a} in path")

    def test_create_circle_no_duplicate_points(self):
        circle = arc_utils.create_circle(Point(0, 0), 5.0, arc_utils.ArcDir.CW)
        self._assert_no_dupes(circle.path)

    def test_create_arc_full_circle_no_duplicate_points(self):
        arc = arc_utils.create_arc(Point(0, 0), 5.0, 0, 2 * math.pi, arc_utils.ArcDir.CW)
        self._assert_no_dupes(arc.path)

    def test_create_arc_partial_no_duplicate_points(self):
        arc = arc_utils.create_arc(Point(0, 0), 5.0, 0, math.pi, arc_utils.ArcDir.CW)
        self._assert_no_dupes(arc.path)

    def test_create_arc_partial_ccw_no_duplicate_points(self):
        arc = arc_utils.create_arc(Point(0, 0), 5.0, 0, math.pi, arc_utils.ArcDir.CCW)
        self._assert_no_dupes(arc.path)

    def test_create_arc_from_path_strips_duplicates(self):
        """create_arc_from_path (used by arcs_from_circle_diff) must strip
        consecutive duplicate coords from geometry-operation output."""
        p = (5.0, 0.0)
        path_with_dupe = LineString([p, p, (4.0, 3.0), (0.0, 5.0)])
        arc = arc_utils.create_arc_from_path(Point(0, 0), path_with_dupe, 5.0)
        self._assert_no_dupes(arc.path)

    def test_split_arcs_project_no_warning(self):
        """split_arcs calls full_arc.path.project() — must not warn on circles
        whose difference-derived sub-arcs have duplicate coords."""
        import warnings
        circle = arc_utils.create_circle(Point(0, 0), 5.0, arc_utils.ArcDir.CW)
        # Partially overlapping cut forces arcs_from_circle_diff to run.
        already_cut = Point(5.0, 0.0).buffer(1.0)
        with warnings.catch_warnings():
            warnings.simplefilter("error", RuntimeWarning)
            arc_utils.split_arcs([circle], already_cut)  # must not raise


class TestFilterArc(unittest.TestCase):

    def setUp(self):
        self.step = 1.0
        self.polygon = Point(0, 0).buffer(20)
        # A dilated boundary ring well away from the arc under test.
        self.dilated_boundaries = [Point(100, 100).buffer(5)]

    def _make_arc(self, origin, radius, step=None):
        """Helper: create a complete CW arc."""
        s = step or self.step
        arc = arc_utils.create_arc(origin, radius, 0, math.pi, arc_utils.ArcDir.CW)
        return arc_utils.complete_arc(arc, arc_utils.ArcDir.CW)

    def test_valid_arc_passes_through(self):
        arc = self._make_arc(Point(0, 0), 5)
        result = arc_utils.filter_arc(arc, self.polygon, self.dilated_boundaries, self.step)
        self.assertIsNotNone(result)
        self.assertIs(result, arc)

    def test_too_few_coords_filtered(self):
        # A two-point LineString has fewer than 3 coords.
        arc = arc_utils.create_arc_from_path(
            Point(0, 0), LineString([(0, 5), (0, -5)]), 5)
        result = arc_utils.filter_arc(arc, self.polygon, self.dilated_boundaries, self.step)
        self.assertIsNone(result)

    def test_too_short_filtered(self):
        arc = self._make_arc(Point(0, 0), 5)
        # step large enough that arc.path.length <= step/20 is impossible here,
        # so make step huge instead.
        result = arc_utils.filter_arc(arc, self.polygon, self.dilated_boundaries, step=10000)
        self.assertIsNone(result)

    def test_arc_not_intersecting_polygon_filtered(self):
        # Arc far outside the polygon.
        arc = self._make_arc(Point(200, 200), 5)
        result = arc_utils.filter_arc(arc, self.polygon, self.dilated_boundaries, self.step)
        self.assertIsNone(result)

    def test_arc_inside_dilated_boundary_filtered(self):
        # Place the dilated boundary to fully contain the arc.
        origin = Point(0, 0)
        radius = 5
        arc = self._make_arc(origin, radius)
        dilated_boundaries = [origin.buffer(radius * 2)]
        result = arc_utils.filter_arc(arc, self.polygon, dilated_boundaries, self.step)
        self.assertIsNone(result)


if __name__ == '__main__':
    unittest.main()

