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

