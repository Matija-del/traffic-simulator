import unittest
import numpy as np

from traffic_sim.geometry import calc_sector, s_diffs

class TestGeometryFunctions(unittest.TestCase):
    def test_calc_sector(self):
        # 4 sectors, positions spaced evenly from 0 to 2pi
        nump = 4
        positions = np.array([0.1, np.pi / 2 + 0.1, np.pi + 0.1, 1.5 * np.pi + 0.1])
        sectors = calc_sector(positions, nump)
        np.testing.assert_array_equal(sectors, np.array([0, 1, 2, 3]))

    def test_s_diffs_wrap_around(self):
        # Two cars, one at 0.5 rad, one at 2pi - 0.5 rad (approx 5.78)
        positions = np.array([0.5, 2 * np.pi - 0.5])
        diffs = s_diffs(positions)
        # Expected:
        # For car 0: positions[0] - positions[1] = 0.5 - (2pi - 0.5) < 0 -> (2pi + 0.5 - 2pi + 0.5) = 1.0
        self.assertAlmostEqual(diffs[0], 1.0)