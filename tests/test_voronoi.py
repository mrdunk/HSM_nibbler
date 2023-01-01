#!/usr/bin/env python3

import unittest
import os, sys
import math

from shapely.geometry import LineString, Polygon  # type: ignore

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import hsm_nibble.voronoi_centers as voronoi_centers

class TestBoilerplate(unittest.TestCase):
    def setUp(self):
        self.vc = voronoi_centers.VoronoiCenters()
        self.vc.vertex_to_edges = {
                (10.1, 10.1): [1, 2],
                (20.2, 10.1): [2, 4],
                (20.2, 20.2): [4, 5, 3],
                (10.1, 20.1): [1, 6, 5],
                (30.3, 20.2): [3,],
                (-10, 20): [6,],
                }
        self.vc.edge_to_vertex = {
                1: ((10.1, 10.1), (10.1, 20.1)),
                2: ((20.2, 10.1), (10.1, 10.1)),
                3: ((30.3, 20.2), (20.2, 20.2)),
                4: ((20.2, 20.2), (20.2, 10.1)),
                5: ((20.2, 20.2), (10.1, 20.1)),
                6: ((10.1, 20.1), (-10, 20)),
                }
        self.vc.edges = {
                1: LineString(self.vc.edge_to_vertex[1]),
                2: LineString(self.vc.edge_to_vertex[2]),
                3: LineString(self.vc.edge_to_vertex[3]),
                4: LineString(self.vc.edge_to_vertex[4]),
                5: LineString(self.vc.edge_to_vertex[5]),
                6: LineString(self.vc.edge_to_vertex[6]),
                }
        self.vc._check_data()

    def test_one(self):
        pass


if __name__ == '__main__':
    unittest.main()

