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
        origin = Point(0, 0)
        radius = 7
        winding_dir = geometry.ArcDir.CW

        # This path will start at the coordinates: (radius, 0)
        path = list(geometry.create_circle(origin, radius).path.coords)

        path = path[int(len(path) / 8) : int(7 * len(path) / 8)]

        arc = geometry.create_arc_from_path(origin, winding_dir, LineString(path))

        self.assertEqual(arc.origin, origin)
        self.assertEqual(arc.radius, radius)
        self.assertEqual(arc.start, Point(arc.path.coords[0]))
        self.assertEqual(arc.end, Point(arc.path.coords[-1]))

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

    def test_create_arc_from_path_CCW(self):
        """ Create a Counter-Clockwise arc. """
        origin = Point(0, 0)
        radius = 7
        winding_dir = geometry.ArcDir.CCW

        # This path will start at the coordinates: (radius, 0)
        path = list(geometry.create_circle(origin, radius).path.coords)

        path = path[int(len(path) / 8) : int(7 * len(path) / 8)]

        arc = geometry.create_arc_from_path(origin, winding_dir, LineString(path))

        self.assertEqual(arc.origin, origin)
        self.assertEqual(arc.radius, radius)
        self.assertEqual(arc.start, Point(arc.path.coords[0]))
        self.assertEqual(arc.end, Point(arc.path.coords[-1]))

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


class TestJoinArcs(unittest.TestCase):
    def test_join(self):
        """ A simple join. No collision outside safe_area. """
        arc1 = geometry.ArcData(
                origin = Point(1, 1),
                radius = 0.5,
                start = Point(0, 1),
                end = Point(2, 1),
                start_angle = -math.pi / 2,
                span_angle = math.pi,
                path = LineString([(0, 1), (1, 1.5), (2, 1)]), # Not very round.
                debug = ""
                )
        arc2 = geometry.ArcData(
                origin = Point(1, 2),
                radius = 0.5,
                start = Point(0, 2),
                end = Point(2, 2),
                start_angle = -math.pi / 2,
                span_angle = math.pi,
                path = LineString([(0, 2), (1, 2.5), (2, 2)]), # Not very round.
                debug = ""
                )
        safe_area = Polygon([(-10, -10), (-10, 10), (10, 10), (10, -10)])
        line = geometry.join_arcs(arc1, arc2, safe_area)

        self.assertEqual(line.start, arc1.end)
        self.assertEqual(line.end, arc2.start)
        self.assertEqual(line.path, LineString([arc1.end, arc2.start]))
        self.assertTrue(line.safe)

if __name__ == '__main__':
    unittest.main()

