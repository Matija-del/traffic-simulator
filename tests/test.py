import pytest
import numpy as np

from traffic_sim.constants import CarIndex, PartIndex, SegmentTypes
from traffic_sim.road import Road
from traffic_sim.parts import Parts
from traffic_sim.linked_road import LinkedRoad
import traffic_sim.physics as phys
from traffic_sim.geometry import *

# ==============================================================================
# Physics Tests
# ==============================================================================

def test_sign_and_reactor():
    a = np.array([5.0, 2.0])
    b = np.array([3.0, 4.0])
    
    np.testing.assert_array_equal(phys.sign(a, b), [1, -1])
    
    # Test reactor response shape and non-zero values
    react = phys.reactor(a, b)
    assert react.shape == (2,)
    assert react[0] > 0
    assert react[1] < 0


def test_v_diffs():
    speeds = np.array([10.0, 15.0])
    diffs = phys.v_diffs(speeds)
    expected = np.array([[0.0, -5.0], [5.0, 0.0]])
    np.testing.assert_array_almost_equal(diffs, expected)


def test_v_check_clamping():
    v = np.array([-5.0, 50.0, 120.0])
    v_sect = np.array([100.0, 100.0, 100.0])
    clamped = phys.v_check(v, v_sect)
    
    # Speeds clipped between 0 and 1.1 * v_sect (110.0)
    expected = np.array([0.0, 50.0, 110.0])
    np.testing.assert_array_almost_equal(clamped, expected)


def test_calc_v_and_s():
    vel = np.array([10.0])
    acc = np.array([2.0])
    dt = 0.5
    radius = 100.0
    pos = np.array([0.0])

    new_vel = phys.calc_v(vel, acc, dt=dt)
    assert new_vel[0] == 11.0

    # pos + (vel * dt + 0.5 * acc * dt^2) / radius
    # 0.0 + (10.0 * 0.5 + 0.5 * 2.0 * 0.25) / 100.0 = (5.0 + 0.25) / 100.0 = 0.0525
    new_s = phys.calc_s(pos, vel, acc, radius, dt=dt)
    assert pytest.approx(new_s[0], 0.0001) == 0.0525


# ==============================================================================
# Road Class Tests
# ==============================================================================

def test_road_init_valid():
    road = Road(name="Test Loop", radius=100.0, number_of_cars=5, v_initial=20.0)
    assert road.name == "Test Loop"
    assert road.radius == 100.0
    assert road.numc == 5
    assert road.cars.shape == (7, 5)


def test_road_init_invalid():
    with pytest.raises(ValueError):
        Road(name="Invalid Radius", radius=-10.0, number_of_cars=5, v_initial=20.0)

    with pytest.raises(ValueError):
        Road(name="Invalid Cars", radius=100.0, number_of_cars=0, v_initial=20.0)


def test_road_radius_setter():
    road = Road(name="Test Loop", radius=100.0, number_of_cars=5, v_initial=20.0)
    road.radius = 150.0
    assert road.radius == 150.0

    with pytest.raises(ValueError):
        road.radius = -5.0


# ==============================================================================
# Parts Class Tests
# ==============================================================================

def test_parts_init():
    parts_track = Parts(name="Sectors Loop", radius=50.0, number_of_parts=4, number_of_cars=3, v_initial=15.0)
    assert parts_track.nump == 4
    assert parts_track.parts.shape == (2, 4)
    np.testing.assert_array_equal(parts_track.parts[PartIndex.MAX_SPEEDS], [15.0, 15.0, 15.0, 15.0])


def test_parts_invalid_parts_count():
    with pytest.raises(ValueError):
        Parts(name="Invalid Sector Track", radius=50.0, number_of_parts=0, number_of_cars=3, v_initial=15.0)


# ==============================================================================
# LinkedRoad Tests
# ==============================================================================

def test_linked_road_building():
    linked = LinkedRoad(name="Dynamic Track", number_of_cars=4, v_initial=25.0)
    
    # Initialized via default _get_segments() adding 4 segment nodes (8 sub-nodes)
    assert linked.number_of_segments == 8
    assert linked.segments_data.shape == (3, 8)
    assert linked.lenght > 0
    assert linked.radius > 0


def test_linked_road_speed_limit_change():
    linked = LinkedRoad(name="Dynamic Track", number_of_cars=4, v_initial=25.0)
    
    linked.change_speed_limit(segment=0, new_speed=12.0)
    assert linked.segments_data[PartIndex.MAX_SPEEDS][0] == 12.0

    with pytest.raises(ValueError):
        linked.change_speed_limit(segment=99, new_speed=10.0)

    with pytest.raises(TypeError):
        linked.change_speed_limit(segment=0, new_speed="fast")
