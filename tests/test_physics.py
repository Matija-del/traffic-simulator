import unittest
import numpy as np

from traffic_sim.physics import calc_v, calc_s, sec_v_diffs

class TestPhysicsFunctions(unittest.TestCase):
    def test_sec_v_diffs(self):
        speeds = np.array([20.0, 30.0])
        limit = np.array([25.0, 25.0])
        diffs = sec_v_diffs(speeds, limit)
        np.testing.assert_array_equal(diffs, np.array([5.0, -5.0]))

    def test_calc_v_kinematics(self):
        v = np.array([10.0, 15.0])
        a = np.array([2.0, -2.0])
        dt = 0.5
        new_v = calc_v(v, a, dt)
        np.testing.assert_array_equal(new_v, np.array([11.0, 14.0]))

    def test_calc_s_positioning(self):
        pos = np.array([0.0])
        vel = np.array([10.0])
        acc = np.array([2.0])
        radius = 10.0
        dt = 1.0
        # s_new = pos + (v*dt + 0.5*a*dt^2) / radius = 0 + (10 + 1)/10 = 1.1
        new_pos = calc_s(pos, vel, acc, radius, dt)
        self.assertAlmostEqual(new_pos[0], 1.1)





if __name__ == "__main__":
    unittest.main()