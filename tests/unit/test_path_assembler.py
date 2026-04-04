#!/usr/bin/env python3

import unittest
import os, sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from shapely.geometry import LineString, Point, Polygon

from hsm_nibble.arc_utils import ArcData, ArcDir, LineData, MoveStyle, create_arc, create_circle
from hsm_nibble.path_assembler import PathAssembler, split_line_by_poly


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

STEP = 3.0


def _square(size=20.0) -> Polygon:
    return Polygon([(-size, -size), (size, -size), (size, size), (-size, size)])


def _assembler(step=STEP, winding_dir=ArcDir.CW, cut_area=None, dilated=None) -> PathAssembler:
    if cut_area is None:
        cut_area = Polygon()
    if dilated is None:
        dilated = _square(30.0)
    return PathAssembler(step, winding_dir, cut_area, dilated)


def _make_arc(cx, cy, radius=1.5, winding_dir=ArcDir.CW) -> ArcData:
    """Return a complete (start/end/angles populated) arc centred at (cx, cy)."""
    from hsm_nibble.arc_utils import complete_arc
    circle = create_circle(Point(cx, cy), radius, winding_dir)
    arc = create_arc(Point(cx, cy), radius, 0, 1.5, winding_dir)
    assert arc is not None
    result = complete_arc(arc, winding_dir)
    assert result is not None
    return result


# ---------------------------------------------------------------------------
# split_line_by_poly
# ---------------------------------------------------------------------------

class TestSplitLineByPoly(unittest.TestCase):

    def test_line_not_crossing_poly_returns_single_segment(self):
        line = LineString([(0, 0), (1, 0)])
        poly = Polygon([(10, 10), (20, 10), (20, 20), (10, 20)])
        result = split_line_by_poly(line, poly)
        self.assertEqual(result.geom_type, "MultiLineString")
        self.assertEqual(len(result.geoms), 1)

    def test_line_crossing_poly_is_split(self):
        line = LineString([(-5, 0), (5, 0)])
        poly = Polygon([(-1, -1), (1, -1), (1, 1), (-1, 1)])
        result = split_line_by_poly(line, poly)
        self.assertEqual(result.geom_type, "MultiLineString")
        self.assertGreater(len(result.geoms), 1)

    def test_total_length_preserved(self):
        line = LineString([(-5, 0), (5, 0)])
        poly = Polygon([(-1, -1), (1, -1), (1, 1), (-1, 1)])
        result = split_line_by_poly(line, poly)
        total = sum(g.length for g in result.geoms)
        self.assertAlmostEqual(total, line.length, places=5)

    def test_accepts_multipolygon(self):
        from shapely.geometry import MultiPolygon
        line = LineString([(0, 0), (10, 0)])
        poly = MultiPolygon([
            Polygon([(1, -1), (2, -1), (2, 1), (1, 1)]),
            Polygon([(4, -1), (5, -1), (5, 1), (4, 1)]),
        ])
        result = split_line_by_poly(line, poly)
        self.assertGreater(len(result.geoms), 1)


# ---------------------------------------------------------------------------
# PathAssembler construction
# ---------------------------------------------------------------------------

class TestPathAssemblerInit(unittest.TestCase):

    def test_path_starts_empty(self):
        asm = _assembler()
        self.assertEqual(asm.path, [])

    def test_last_arc_starts_none(self):
        asm = _assembler()
        self.assertIsNone(asm.last_arc)

    def test_pending_queues_starts_empty(self):
        asm = _assembler()
        self.assertEqual(asm.pending_arc_queues, [])


# ---------------------------------------------------------------------------
# PathAssembler.queue_arcs / flush_arc_queues
# ---------------------------------------------------------------------------

class TestQueueAndFlush(unittest.TestCase):

    def test_single_arc_reaches_path_after_flush(self):
        """queue_arcs accumulates; flush_arc_queues drains."""
        asm = _assembler()
        arc = _make_arc(0, 0)
        asm.queue_arcs([arc])
        asm.flush_arc_queues()
        arc_data = [p for p in asm.path if isinstance(p, ArcData)]
        self.assertEqual(len(arc_data), 1)

    def test_second_arc_triggers_fast_path_drain(self):
        """When exactly one queue exists and one new arc arrives the fast path
        appends and immediately drains that queue."""
        asm = _assembler()
        arc1 = _make_arc(0, 0)
        arc2 = _make_arc(3, 0)
        asm.queue_arcs([arc1])   # Creates queue[0] with arc1.
        asm.queue_arcs([arc2])   # Fast path: appends arc2, drains queue[0].
        arc_data = [p for p in asm.path if isinstance(p, ArcData)]
        self.assertEqual(len(arc_data), 2)

    def test_flush_drains_pending_queues(self):
        asm = _assembler()
        arc1 = _make_arc(0, 0)
        arc2 = _make_arc(0, 10)  # Far away — goes to a second queue.
        # Force two queues by directly inserting them.
        asm.pending_arc_queues.append([arc1])
        asm.pending_arc_queues.append([arc2])
        asm.flush_arc_queues()
        self.assertEqual(asm.pending_arc_queues, [])
        arc_data = [p for p in asm.path if isinstance(p, ArcData)]
        self.assertEqual(len(arc_data), 2)

    def test_last_arc_updated_after_flush(self):
        asm = _assembler()
        arc = _make_arc(0, 0)
        asm.queue_arcs([arc])
        asm.flush_arc_queues()
        self.assertIsNotNone(asm.last_arc)


# ---------------------------------------------------------------------------
# PathAssembler.join_arcs
# ---------------------------------------------------------------------------

class TestJoinArcs(unittest.TestCase):

    def _two_arc_assembler(self):
        """Set up an assembler with one arc already on the path."""
        asm = _assembler()
        arc1 = _make_arc(0, 0)
        asm.queue_arcs([arc1])
        asm.flush_arc_queues()
        return asm

    def test_returns_list(self):
        asm = self._two_arc_assembler()
        arc2 = _make_arc(3, 0)
        result = asm.join_arcs(arc2)
        self.assertIsInstance(result, list)

    def test_short_inside_path_is_cut_move(self):
        """Two arcs that are inside the pocket and close together → CUT move."""
        # Put a big already-cut area so the joining line is 'inside pocket'.
        big_cut = _square(25.0)
        asm = _assembler(cut_area=big_cut, dilated=_square(30.0))
        arc1 = _make_arc(0, 0)
        asm.queue_arcs([arc1])
        asm.flush_arc_queues()

        arc2 = _make_arc(2, 0)  # Close — within step distance.
        lines = asm.join_arcs(arc2)
        move_styles = [l.move_style for l in lines]
        self.assertTrue(all(s == MoveStyle.CUT for s in move_styles))

    def test_outside_path_is_rapid_outside(self):
        """Two arcs far apart, outside the dilated polygon → RAPID_OUTSIDE."""
        # dilated polygon is small (5 units) so arcs at ±15 are outside it.
        asm = _assembler(cut_area=Polygon(), dilated=_square(5.0))
        arc1 = _make_arc(-15, 0)
        asm.queue_arcs([arc1])
        asm.flush_arc_queues()
        arc2 = _make_arc(15, 0)
        lines = asm.join_arcs(arc2)
        move_styles = [l.move_style for l in lines]
        self.assertIn(MoveStyle.RAPID_OUTSIDE, move_styles)

    def test_cut_area_grows_after_join(self):
        """cut_area_total should grow when arcs are joined via an outside move."""
        # Use a small dilated polygon so the connecting path triggers the
        # outside-pocket branch, which is where cut_area_total is updated.
        asm = _assembler(cut_area=Polygon(), dilated=_square(5.0))
        arc1 = _make_arc(-15, 0)
        asm.queue_arcs([arc1])
        asm.flush_arc_queues()
        area_before = asm.cut_area_total.area
        arc2 = _make_arc(15, 0)
        asm.join_arcs(arc2)
        self.assertGreater(asm.cut_area_total.area, area_before)


if __name__ == "__main__":
    unittest.main()
