#!/usr/bin/env python3
"""
Unit tests for PathAssembler private methods:
  _check_queue_overlap, _arcs_to_path
"""

import unittest
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from shapely.geometry import Point, Polygon

from hsm_nibble.arc_utils import ArcData, ArcDir, MoveStyle, complete_arc, create_arc, create_circle
from hsm_nibble.path_assembler import PathAssembler


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

STEP = 3.0


def _assembler(step=STEP, cut_area=None, dilated=None):
    if cut_area is None:
        cut_area = Polygon()
    if dilated is None:
        dilated = Polygon([(-50, -50), (50, -50), (50, 50), (-50, 50)])
    return PathAssembler(step, ArcDir.CW, cut_area, dilated)


def _make_arc(cx, cy, radius=1.5, winding_dir=ArcDir.CW):
    arc = create_arc(Point(cx, cy), radius, 0, 1.5, winding_dir)
    assert arc is not None
    result = complete_arc(arc, winding_dir)
    assert result is not None
    return result


# ---------------------------------------------------------------------------
# PathAssembler._check_queue_overlap
# ---------------------------------------------------------------------------

class TestCheckQueueOverlap(unittest.TestCase):

    def test_no_later_queues_returns_true(self):
        """With only one queue (index 0), there are no later queues to conflict."""
        asm = _assembler()
        arc1 = _make_arc(0, 0)
        asm.pending_arc_queues.append([arc1])
        arc2 = _make_arc(20, 0)  # far away — no overlap
        self.assertTrue(asm._check_queue_overlap(arc2, 0))

    def test_non_overlapping_later_queue_returns_true(self):
        """Later queue arc is far from candidate arc — no conflict."""
        asm = _assembler()
        asm.pending_arc_queues.append([_make_arc(0, 0)])   # queue 0
        asm.pending_arc_queues.append([_make_arc(50, 50)]) # queue 1 — far away
        new_arc = _make_arc(1, 0)  # close to queue 0, far from queue 1
        self.assertTrue(asm._check_queue_overlap(new_arc, 0))

    def test_overlapping_later_queue_returns_false(self):
        """Later queue contains an arc whose swept area intersects candidate."""
        asm = _assembler()
        asm.pending_arc_queues.append([_make_arc(0, 0)])  # queue 0
        # queue 1 contains an arc right on top of the candidate
        asm.pending_arc_queues.append([_make_arc(1, 0)])  # queue 1 — very close
        candidate = _make_arc(1.2, 0)  # overlaps queue 1's arc
        self.assertFalse(asm._check_queue_overlap(candidate, 0))

    def test_empty_later_queues_returns_true(self):
        asm = _assembler()
        asm.pending_arc_queues.append([_make_arc(0, 0)])  # queue 0
        asm.pending_arc_queues.append([])                  # queue 1 — empty
        self.assertTrue(asm._check_queue_overlap(_make_arc(5, 0), 0))


# ---------------------------------------------------------------------------
# PathAssembler._arcs_to_path
# ---------------------------------------------------------------------------

class TestArcsToPat(unittest.TestCase):

    def test_single_arc_appended_to_path(self):
        asm = _assembler()
        arc = _make_arc(0, 0)
        asm._arcs_to_path([arc])
        arc_data = [p for p in asm.path if isinstance(p, ArcData)]
        self.assertEqual(len(arc_data), 1)

    def test_multiple_arcs_all_appended(self):
        asm = _assembler()
        arcs = [_make_arc(0, 0), _make_arc(4, 0), _make_arc(8, 0)]
        asm._arcs_to_path(arcs)
        arc_data = [p for p in asm.path if isinstance(p, ArcData)]
        self.assertEqual(len(arc_data), 3)

    def test_connecting_moves_inserted_between_arcs(self):
        """With a pre-existing arc, _arcs_to_path should call join_arcs and insert moves."""
        big_cut = Polygon([(-50, -50), (50, -50), (50, 50), (-50, 50)])
        asm = _assembler(cut_area=big_cut)
        # Seed last_arc by putting one arc on the path directly.
        first_arc = _make_arc(0, 0)
        asm.path.append(first_arc)
        asm.last_arc = first_arc

        second_arc = _make_arc(4, 0)
        asm._arcs_to_path([second_arc])

        # Path should contain both arcs and at least one connecting move.
        from hsm_nibble.arc_utils import LineData
        arc_data = [p for p in asm.path if isinstance(p, ArcData)]
        line_data = [p for p in asm.path if isinstance(p, LineData)]
        self.assertEqual(len(arc_data), 2)
        self.assertGreater(len(line_data), 0)

    def test_last_arc_updated_after_processing(self):
        asm = _assembler()
        arc = _make_arc(0, 0)
        asm._arcs_to_path([arc])
        self.assertIsNotNone(asm.last_arc)
        self.assertIs(asm.last_arc, arc)

    def test_none_arc_in_list_is_skipped(self):
        """_arcs_to_path should tolerate None entries."""
        asm = _assembler()
        arc = _make_arc(0, 0)
        # The implementation checks `if arc is None: continue`
        asm._arcs_to_path([arc, None])
        arc_data = [p for p in asm.path if isinstance(p, ArcData)]
        self.assertEqual(len(arc_data), 1)

    def test_empty_list_leaves_path_unchanged(self):
        asm = _assembler()
        asm._arcs_to_path([])
        self.assertEqual(asm.path, [])

    def test_closest_winding_processes_without_error(self):
        """ArcDir.CLOSEST selects winding per-arc based on geometry — just verify no crash."""
        dilated = Polygon([(-50, -50), (50, -50), (50, 50), (-50, 50)])
        asm = PathAssembler(STEP, ArcDir.CLOSEST, Polygon(), dilated)
        arc1 = _make_arc(0, 0)
        arc2 = _make_arc(4, 0)
        asm._arcs_to_path([arc1, arc2])
        arc_data = [p for p in asm.path if isinstance(p, ArcData)]
        self.assertEqual(len(arc_data), 2)
        # Each arc must have a concrete (non-CLOSEST) winding direction after processing.
        for arc in arc_data:
            self.assertIn(arc.winding_dir, (ArcDir.CW, ArcDir.CCW))


if __name__ == "__main__":
    unittest.main()
