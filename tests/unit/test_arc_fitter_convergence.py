#!/usr/bin/env python3
"""
Regression tests for find_best_arc_distance using real convergence-failure
inputs captured from involute.dxf (OVERLAP=3.2, CW).

Each fixture module in tests/fixtures/convergence_failure_N.py was generated
by running tests/fixtures/capture_convergence_failures.py, which intercepts
calls where the convergence loop hits ITERATION_COUNT and serialises the inputs.

Assertions:
  - The function completes without raising.
  - It returns a usable arc (not None, positive path length).
  - hidden_at_start and backwards are False (these are true convergence misses,
    not the hidden/backwards early-exit cases).
  - iteration_count == ITERATION_COUNT — documents the known failure.
    If this assertion starts failing it means convergence improved; update the
    test to assertLess(iters, ITERATION_COUNT) and celebrate.
"""

import importlib
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from shapely.wkt import loads as wkt_loads
from shapely.geometry import Point

from hsm_nibble.arc_fitter import find_best_arc_distance, ITERATION_COUNT, ProportionalController
from hsm_nibble.arc_utils import ArcData, ArcDir


FIXTURE_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "fixtures"
)


def _load_fixture(index):
    """Load fixture module and reconstruct find_best_arc_distance arguments."""
    spec = importlib.util.spec_from_file_location(
        f"convergence_failure_{index}",
        os.path.join(FIXTURE_DIR, f"convergence_failure_{index}.py")
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    voronoi_edge = wkt_loads(mod.VORONOI_EDGE_WKT)
    calculated_area = wkt_loads(mod.CALCULATED_AREA_WKT)
    pocket_polygon = wkt_loads(mod.POCKET_POLYGON_WKT)
    max_dist = mod.MAX_DIST
    winding_dir = ArcDir[mod.WINDING_DIR]

    distance_from_geom = lambda pt: min(max_dist, pocket_polygon.boundary.distance(pt))

    last_circle = None
    if mod.LAST_CIRCLE is not None:
        d = mod.LAST_CIRCLE
        last_circle = ArcData(
            origin=wkt_loads(d["origin_wkt"]),
            radius=d["radius"],
            start=wkt_loads(d["start_wkt"]) if d["start_wkt"] else None,
            end=wkt_loads(d["end_wkt"]) if d["end_wkt"] else None,
            start_angle=d["start_angle"],
            span_angle=d["span_angle"],
            winding_dir=ArcDir[d["winding_dir"]] if d["winding_dir"] else None,
            path=wkt_loads(d["path_wkt"]),
            debug=d["debug"],
        )

    return dict(
        voronoi_edge=voronoi_edge,
        start_distance=mod.START_DISTANCE,
        min_distance=mod.MIN_DISTANCE,
        step=mod.STEP,
        winding_dir=winding_dir,
        calculated_area=calculated_area,
        last_circle=last_circle,
        distance_from_geom=distance_from_geom,
        max_dist=max_dist,
    )


def _count_fixtures():
    return sum(
        1 for f in os.listdir(FIXTURE_DIR)
        if f.startswith("convergence_failure_") and f.endswith(".py")
    )


class TestConvergenceFailureFixtures(unittest.TestCase):
    """One test method per captured failure fixture."""

    def _run_fixture(self, index):
        kwargs = _load_fixture(index)
        best_dist, best_circle, hidden, iters, backwards = find_best_arc_distance(
            **kwargs, controller=ProportionalController()
        )
        self.assertFalse(hidden, f"fixture {index}: unexpected hidden_at_start")
        self.assertFalse(backwards, f"fixture {index}: unexpected backwards")
        self.assertIsNotNone(best_circle, f"fixture {index}: best_circle is None")
        self.assertGreater(best_circle.path.length, 0,
                           f"fixture {index}: best_circle has zero-length path")
        # Known non-convergence — update to assertLess if convergence improves:
        self.assertEqual(iters, ITERATION_COUNT,
                         f"fixture {index}: convergence improved! iters={iters}")

    def test_failure_0(self): self._run_fixture(0)
    def test_failure_1(self): self._run_fixture(1)


if __name__ == "__main__":
    unittest.main()
