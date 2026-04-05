#!/usr/bin/env python3
"""
Hard smoke test: run the toolpath generator against square_circle.dxf at a
smaller overlap (1.6) and assert zero dangerous crashes.

This test currently FAILS (dangerous_crash_count=1) — intentionally. It acts
as a TDD red test to drive fixing the path-planning crash in the narrow region
between the circle boundary and rectangle corner. The crash is caused by a
RAPID_INSIDE move crossing uncut material during a branch-to-branch transition.

Once the path-planning crash is fixed, this test should pass with no changes.
"""

import os
import sys
import unittest

import ezdxf
from shapely.geometry import MultiPolygon, Polygon  # type: ignore
from shapely.ops import unary_union  # type: ignore

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from hsm_nibble import dxf
from hsm_nibble import geometry
from hsm_nibble.arc_utils import ArcData, LineData

DXF_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "test_cases", "square_circle.dxf"
)

OVERLAP = 1.6
WINDING = geometry.ArcDir.CW

MAX_DANGEROUS_CRASH_COUNT = 0


class TestSmokeHard(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        dxf_data = ezdxf.readfile(DXF_PATH)
        modelspace = dxf_data.modelspace()
        shape = dxf.dxf_to_polygon(modelspace).geoms[-1].buffer(0)
        cls.toolpath = geometry.Pocket(shape, OVERLAP, WINDING, generate=False, debug=False)

        cut_area = Polygon()
        crash_area_parts = []
        cls.disjointed_path_count = 0
        last_element = None
        for element in cls.toolpath.path:
            if last_element is not None:
                if (not last_element.end.equals_exact(element.start, 6) or
                        not last_element.path.coords[-1] == element.path.coords[0]):
                    cls.disjointed_path_count += 1
            last_element = element

            if isinstance(element, ArcData):
                cut_area = cut_area.union(element.path.buffer(OVERLAP / 2))
            if isinstance(element, LineData):
                if element.move_style == geometry.MoveStyle.RAPID_INSIDE:
                    crash_area_parts.append(
                        element.path.buffer(OVERLAP / 2).difference(cut_area))

        crash_area = unary_union(crash_area_parts)
        eroded = crash_area.buffer(-OVERLAP / 20)
        if eroded.geom_type == "Polygon":
            eroded = MultiPolygon([eroded])
        cls.dangerous_crash_count = len(eroded.geoms) if not eroded.is_empty else 0

    def test_path_not_empty(self):
        self.assertGreater(len(self.toolpath.path), 0)

    def test_dangerous_crash_count(self):
        self.assertEqual(
            self.dangerous_crash_count, MAX_DANGEROUS_CRASH_COUNT,
            f"dangerous_crash_count {self.dangerous_crash_count} exceeds threshold {MAX_DANGEROUS_CRASH_COUNT}"
        )

    def test_disjointed_path_count(self):
        self.assertEqual(
            self.disjointed_path_count, 0,
            f"disjointed_path_count {self.disjointed_path_count} — path has gaps"
        )


if __name__ == "__main__":
    unittest.main()
