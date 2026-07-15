import unittest
import numpy as np

from traffic_sim.road import Road


class TestRoadClassValidation(unittest.TestCase):
    def test_invalid_constructor_arguments(self):
        with self.assertRaises(ValueError):
            Road("FailRoad", radius=-10, number_of_parts=10, number_of_cars=5, v_initial=20, reaction_factors=0.5)
        with self.assertRaises(ValueError):
            Road("FailRoad", radius=10, number_of_parts=0, number_of_cars=5, v_initial=20, reaction_factors=0.5)
        with self.assertRaises(ValueError):
            Road("FailRoad", radius=10, number_of_parts=10, number_of_cars=-1, v_initial=20, reaction_factors=0.5)

    def test_radius_property_setter(self):
        road = Road("SuccessRoad", radius=100.0, number_of_parts=4, number_of_cars=3, v_initial=20,
                    reaction_factors=0.5)
        self.assertEqual(road.radius, 100.0)

        road.radius = 150.0
        self.assertEqual(road.radius, 150.0)

        with self.assertRaises(ValueError):
            road.radius = -10.0