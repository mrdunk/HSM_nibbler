#!/usr/bin/env python3

import unittest
import os, sys
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from shapely.geometry import LineString, Point, Polygon

import hsm_nibble.geometry as geometry


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

class TestCreateArc(BaseTests):
    def test_create_cw(self):
        """ Check create_arc() method works in a sane way for ClockWise arcs. """
        origin = Point(100, 100)
        radius = 5
        start_angle = geometry.math.pi / 2
        span_angle = geometry.math.pi / 2
        winding_dir = geometry.ArcDir.CW

        arc = geometry.create_arc(origin, radius, start_angle, span_angle, winding_dir)

        self.verify_arc(arc)

    def test_create_ccw(self):
        """ Check create_arc() method works in a sane way for CounterClockWise arcs. """
        origin = Point(-123, -123)
        radius = 5
        start_angle = 3 * geometry.math.pi / 2
        span_angle = -geometry.math.pi / 4
        winding_dir = geometry.ArcDir.CCW

        arc = geometry.create_arc(origin, radius, start_angle, span_angle, winding_dir)

        self.verify_arc(arc)

    def test_create_cw_full_circle(self):
        """ Check create_arc() method works in a sane way for CounterClockWise arcs. """
        origin = Point(123, 123)
        radius = 5
        start_angle = 0
        span_angle = math.pi * 2
        winding_dir = geometry.ArcDir.CW

        arc = geometry.create_arc(origin, radius, start_angle, span_angle, winding_dir)

        self.verify_arc(arc)

    def test_create_ccw_full_circle(self):
        """ Check create_arc() method works in a sane way for CounterClockWise arcs. """
        origin = Point(-100, -100)
        radius = 5
        start_angle = 0
        span_angle = -math.pi * 2
        winding_dir = geometry.ArcDir.CCW

        arc = geometry.create_arc(origin, radius, start_angle, span_angle, winding_dir)

        self.verify_arc(arc)


class TestArc(BaseTests):
    def test_create_arc_from_path_CW(self):
        """ Create a Clockwise arc. """
        origin = Point(123.45, 123.45)
        radius = 7
        winding_dir = geometry.ArcDir.CW

        # This path will start at the coordinates: (radius, 0)
        path = list(geometry.create_circle(origin, radius).path.coords)

        path = path[int(len(path) / 8) : int(7 * len(path) / 8)]

        arc = geometry.create_arc_from_path(origin, LineString(path), radius)
        self.verify_arc_points(arc)
        arc = geometry.complete_arc(arc, winding_dir)

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
        path = list(geometry.create_circle(origin, radius).path.coords)

        path = path[int(len(path) / 8) : int(7 * len(path) / 8)]

        arc = geometry.create_arc_from_path(origin, LineString(path), radius)
        self.verify_arc_points(arc)
        arc = geometry.complete_arc(arc, winding_dir)

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
            arc = geometry.create_arc(origin, radius, start_angle, span_angle, winding_dir)
            self.verify_arc(arc)

            mirrored_arc = geometry.mirror_arc(mirror_axis, arc)
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
            arc = geometry.create_arc(origin, radius, start_angle, span_angle, winding_dir)
            self.verify_arc(arc)

            mirrored_arc = geometry.mirror_arc(mirror_axis, arc)
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

        arc = geometry.create_arc(origin, radius, start_angle, span_angle, winding_dir)
        self.verify_arc(arc)

        mirrored_arc = geometry.mirror_arc(mirror_axis, arc, geometry.ArcDir.CW)

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
            arc = geometry.create_arc(origin, radius, start_angle, span_angle, winding_dir)
            self.verify_arc(arc)

            mirrored_arc = geometry.mirror_arc(mirror_axis, arc, geometry.ArcDir.CCW)
            self.verify_arc(mirrored_arc)

if __name__ == '__main__':
    unittest.main()

