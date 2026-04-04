#!/usr/bin/env python3
"""
Smoke test: run the toolpath generator against a single representative .dxf
file and assert quality metrics are within acceptable bounds.

Intended to be run after each migration step to catch regressions quickly.
Uses a single winding direction and a large overlap to keep execution fast.
"""

import os
import sys
import unittest

import ezdxf
from shapely.geometry import Polygon  # type: ignore

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from hsm_nibble import dxf
from hsm_nibble import geometry

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


class TestSmoke(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        dxf_data = ezdxf.readfile(DXF_PATH)
        modelspace = dxf_data.modelspace()
        shape = dxf.dxf_to_polygon(modelspace)
        cls.toolpath = geometry.Pocket(shape, OVERLAP, WINDING, generate=False, debug=True)

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


if __name__ == "__main__":
    unittest.main()
