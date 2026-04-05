#!/usr/bin/env python3
"""
Smoke test: run the toolpath generator against a single representative .dxf
file and assert quality metrics are within acceptable bounds.

Intended to be run after each migration step to catch regressions quickly.
Uses a single winding direction and a large overlap to keep execution fast.

Coverage metrics (crash_ratio, dangerous_crash_count) come from simulating
rapid moves and checking whether they cross uncut material — a proxy for
dangerous tool engagement on the CNC machine.  At overlap=1.6 these should
be essentially zero; small-overlap cases (0.4, 0.8) have known pre-existing
crashes that are not tested here.
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
    "test_cases", "involute.dxf"
)

# Large overlap = fewer arcs = faster execution.
OVERLAP = 3.2
WINDING = geometry.ArcDir.CW

# Quality thresholds.
MAX_ARC_FAIL_RATIO = 0.01
MAX_PATH_FAIL_RATIO = 0.01
# At OVERLAP=3.2 the path should be fully covered and have no dangerous rapids.
MIN_CUT_RATIO = 0.999
MAX_CRASH_RATIO = 0.001
MAX_DANGEROUS_CRASH_COUNT = 0


class TestSmoke(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        dxf_data = ezdxf.readfile(DXF_PATH)
        modelspace = dxf_data.modelspace()
        # test_coverage uses .geoms[-1] + buffer(0) to get a clean polygon.
        shape = dxf.dxf_to_polygon(modelspace).geoms[-1].buffer(0)
        cls.toolpath = geometry.Pocket(shape, OVERLAP, WINDING, generate=False, debug=True)

        # Build cut_area and crash_area the same way test_coverage.py does.
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
        uncut_area = cls.toolpath.polygon.difference(cut_area)

        cls.cut_ratio = 1.0 - uncut_area.area / cls.toolpath.polygon.area
        cls.crash_ratio = crash_area.area / cls.toolpath.polygon.area

        eroded = crash_area.buffer(-OVERLAP / 20)
        if eroded.geom_type == "Polygon":
            eroded = MultiPolygon([eroded])
        cls.dangerous_crash_count = len(eroded.geoms) if not eroded.is_empty else 0

    def test_path_not_empty(self):
        self.assertGreater(len(self.toolpath.path), 0)

    def test_arc_fail_ratio(self):
        arc_count = len(self.toolpath.path)
        if arc_count == 0:
            self.skipTest("No arcs generated")
        arc_fail_ratio = self.toolpath.arc_fail_count / arc_count
        self.assertLess(
            arc_fail_ratio, MAX_ARC_FAIL_RATIO,
            f"arc_fail_ratio {arc_fail_ratio:.4f} exceeds threshold {MAX_ARC_FAIL_RATIO}"
        )

    def test_path_fail_ratio(self):
        arc_count = len(self.toolpath.path)
        if arc_count == 0:
            self.skipTest("No arcs generated")
        path_fail_ratio = self.toolpath.path_fail_count / arc_count
        self.assertLess(
            path_fail_ratio, MAX_PATH_FAIL_RATIO,
            f"path_fail_ratio {path_fail_ratio:.4f} exceeds threshold {MAX_PATH_FAIL_RATIO}"
        )

    def test_cut_ratio(self):
        self.assertGreaterEqual(
            self.cut_ratio, MIN_CUT_RATIO,
            f"cut_ratio {self.cut_ratio:.4f} below threshold {MIN_CUT_RATIO}"
        )

    def test_crash_ratio(self):
        self.assertLessEqual(
            self.crash_ratio, MAX_CRASH_RATIO,
            f"crash_ratio {self.crash_ratio:.4f} exceeds threshold {MAX_CRASH_RATIO}"
        )

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
