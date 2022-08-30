#!/usr/bin/env python3

import unittest
import os, sys
import math

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from shapely.geometry import LineString, Point, Polygon

import geometry


ACCURACY = 0.05

class TestCircle(unittest.TestCase):
    def test_create(self):
        """ Check circles are create in a sane way. """
        origin = Point(10, 10)
        radius = 7

        circle = geometry.create_circle(origin = origin, radius = radius)

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

class TestCreateArc(unittest.TestCase):
    def test_create_cw(self):
        """ Check create_arc() method works in a sane way for ClockWise arcs. """
        origin = Point(10, 10)
        radius = 5
        start_angle = geometry.math.pi / 2
        span_angle = geometry.math.pi / 2
        winding_dir = geometry.ArcDir.CW

        arc = geometry.create_arc(origin, radius, start_angle, span_angle, winding_dir)

        # For CW arcs, span_angle should be positive.
        self.assertGreater(arc.span_angle, 0)

        self.assertEqual(arc.start, Point(arc.path.coords[0]))
        self.assertEqual(arc.end, Point(arc.path.coords[-1]))

        # Length of path close to accurate.
        path_len = math.pi * 2 * radius / 4  # 1/4 of a circle.
        self.assertLess(abs(path_len - arc.path.length), path_len / 1000)

        # Angle from origin to arc.start matches start_angle.
        start_angle_observed = math.atan2(arc.start.x - origin.x, arc.start.y - origin.y)
        start_angle_observed = start_angle_observed % (math.pi * 2)
        end_angle_observed = math.atan2(arc.end.x - origin.x, arc.end.y - origin.y)
        end_angle_observed = end_angle_observed % (math.pi * 2)

        self.assertEqual(start_angle_observed, start_angle)
        self.assertEqual(end_angle_observed, start_angle + span_angle)

    def test_create_ccw(self):
        """ Check create_arc() method works in a sane way for CounterClockWise arcs. """
        origin = Point(10, 10)
        radius = 5
        start_angle = 3 * geometry.math.pi / 2
        span_angle = -geometry.math.pi / 4
        winding_dir = geometry.ArcDir.CCW

        arc = geometry.create_arc(origin, radius, start_angle, span_angle, winding_dir)

        # For CCW arcs, span_angle should be negative.
        self.assertLess(arc.span_angle, 0)

        self.assertEqual(arc.start, Point(arc.path.coords[0]))
        self.assertEqual(arc.end, Point(arc.path.coords[-1]))

        # Length of path close to accurate.
        path_len = math.pi * 2 * radius / 8  # 1/4 of a circle.
        self.assertLess(abs(path_len - arc.path.length), path_len / 1000)

        # Angle from origin to arc.start matches start_angle.
        start_angle_observed = math.atan2(arc.start.x - origin.x, arc.start.y - origin.y)
        start_angle_observed = start_angle_observed % (math.pi * 2)
        end_angle_observed = math.atan2(arc.end.x - origin.x, arc.end.y - origin.y)
        end_angle_observed = end_angle_observed % (math.pi * 2)

        self.assertEqual(start_angle_observed, start_angle)
        self.assertEqual(end_angle_observed, start_angle + span_angle)


    def test_create_ccw_full_circle(self):
        """ Check create_arc() method works in a sane way for CounterClockWise arcs. """
        origin = Point(10, 10)
        radius = 5
        start_angle = 0
        span_angle = -math.pi * 1.9999999
        winding_dir = geometry.ArcDir.CCW

        arc = geometry.create_arc(origin, radius, start_angle, span_angle, winding_dir)

        # For CCW arcs, span_angle should be negative.
        self.assertLess(arc.span_angle, 0)

        self.assertEqual(arc.start, Point(arc.path.coords[0]))
        self.assertEqual(arc.end, Point(arc.path.coords[-1]))

        # Length of path close to accurate.
        path_len = math.pi * 2 * radius   # full circle.
        self.assertLess(abs(path_len - arc.path.length), path_len / 1000)

        # Angle from origin to arc.start matches start_angle.
        start_angle_observed = math.atan2(arc.start.x - origin.x, arc.start.y - origin.y)
        start_angle_observed = start_angle_observed % (math.pi * 2)
        end_angle_observed = math.atan2(arc.end.x - origin.x, arc.end.y - origin.y)
        end_angle_observed = end_angle_observed % (math.pi * 2)

        self.assertEqual(start_angle_observed, start_angle)
        self.assertLess(abs(end_angle_observed - (start_angle + span_angle) % (math.pi * 2)), 0.0001)


class TestArc(unittest.TestCase):
    def test_create_arc_from_path_CW(self):
        """ Create a Clockwise arc. """
        origin = Point(10, -0.11)
        radius = 7
        winding_dir = geometry.ArcDir.CW

        # This path will start at the coordinates: (radius, 0)
        path = list(geometry.create_circle(origin, radius).path.coords)

        path = path[int(len(path) / 8) : int(7 * len(path) / 8)]

        arc = geometry.create_arc_from_path(origin, LineString(path), radius)
        arc = geometry.complete_arc(arc, winding_dir)

        self.assertEqual(arc.origin, origin)
        self.assertEqual(arc.radius, radius)
        self.assertTrue(arc.start.equals_exact(Point(arc.path.coords[0]), 1e-6))
        self.assertTrue(arc.end.equals_exact(Point(arc.path.coords[-1]), 1e-6))

        # We start this arc at the coordinate point: (radius, 0) with the origin at (0, 0).
        # Angle rotated from full circle's start/end point:
        expected_start_angle = (1 / 8) * (2 * math.pi)
        # Add to full circle's start angle.
        expected_start_angle = (math.pi / 2) + expected_start_angle
        self.assertLess(abs(arc.start_angle - expected_start_angle), ACCURACY * math.pi * 2 )

        # Arc rotation should be 3/4 of a full circle for this arc.
        expected_span = (3 / 4) * (2 * math.pi)
        self.assertLess(abs(arc.span_angle - expected_span), ACCURACY * math.pi * 2)

        # Clockwise rotation should be positive.
        self.assertGreater(arc.span_angle, 0)

        # Arc length should be 3/4 of a full circle for this arc.
        path_len = arc.path.length
        expected_path_len = 2 * math.pi * radius * (3 / 4)
        self.assertLess(abs(arc.path.length - expected_path_len), ACCURACY * expected_path_len)

        # Radius equal to all points.
        for point in path:
            self.assertEqual(
                    round(arc.origin.distance(Point(path[0])), 6),
                    round(arc.origin.distance(Point(point)), 6))

    def test_create_arc_from_path_CCW(self):
        """ Create a Counter-Clockwise arc. """
        origin = Point(0, 0)
        radius = 7
        winding_dir = geometry.ArcDir.CCW

        # This path will start at the coordinates: (radius, 0)
        path = list(geometry.create_circle(origin, radius).path.coords)

        path = path[int(len(path) / 8) : int(7 * len(path) / 8)]

        arc = geometry.create_arc_from_path(origin, LineString(path), radius)
        arc = geometry.complete_arc(arc, winding_dir)

        self.assertEqual(arc.origin, origin)
        self.assertEqual(arc.radius, radius)
        self.assertTrue(arc.start.equals_exact(Point(arc.path.coords[0]), 1e-6))
        self.assertTrue(arc.end.equals_exact(Point(arc.path.coords[-1]), 1e-6))

        # We start this arc at the coordinate point: (radius, 0) with the origin at (0, 0).
        # Angle rotated from full circle's start/end point:
        expected_start_angle = (1 / 8) * (2 * math.pi)
        # Add to full circle's start angle.
        expected_start_angle = (math.pi / 2) - expected_start_angle
        self.assertLess(abs(arc.start_angle - expected_start_angle), ACCURACY * math.pi * 2 )

        # Arc rotation should be 3/4 of a full circle for this arc.
        expected_span = -(3 / 4) * (2 * math.pi)
        self.assertLess(abs(arc.span_angle - expected_span), ACCURACY * math.pi * 2)

        # Counter-clockwise rotation should be negative.
        self.assertLess(arc.span_angle, 0)

        # Arc length should be 3/4 of a full circle for this arc.
        path_len = arc.path.length
        expected_path_len = 2 * math.pi * radius * (3 / 4)
        self.assertLess(abs(arc.path.length - expected_path_len), ACCURACY * expected_path_len)

        # Radius equal to all points.
        for point in path:
            self.assertEqual(
                    round(arc.origin.distance(Point(path[0])), 6),
                    round(arc.origin.distance(Point(point)), 6))


if __name__ == '__main__':
    unittest.main()

