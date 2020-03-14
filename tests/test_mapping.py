import pytest
from crazyslam.mapping import *


@pytest.fixture
def params():
    return init_params_dict(
        size=30,
        resolution=10,
    )

@pytest.fixture
def empty_map(params):
    return create_empty_map(params)

def test_init_params_dict():
    ref = {
        "resolution": 10,
        "size": 30,
        "origin": (150, 150),
    }
    assert ref == init_params_dict(30, 10)
    assert ref == init_params_dict(30, 10, (150, 150))

def test_create_empty_map(params, empty_map):
    assert (empty_map == create_empty_map(params)).all()

def test_discretize(params):
    pos = np.array([
        [0, 0],
        [10, 10],
        [-10, 10],
        [-130, 140]
    ]).T
    ref = np.array([
        [150, 150],
        [250, 250],
        [50, 250],
        [0, 299],
    ]).T
    assert (discretize(pos, params) == ref).all()

def test_target_cell():
    state = np.array([10, 10, 0])
    sensor_range = np.array([1, 2, 5, 10])
    sensor_bearing = np.array([0, np.pi / 4, np.pi / 2, np.pi])
    targets = np.array([
        [11.0, 10.0],
        [11.414213562373096, 8.585786437626904],
        [10.0, 5.0],
        [0.0, 9.999999999999998],
    ]).T
    assert np.isclose(
        target_cell(state, sensor_range, sensor_bearing),
        targets,
    ).all()

def test_update_grid_map():
    params = init_params_dict(size=23, resolution=1)
    map = create_empty_map(params)
    assert params["origin"] == (11, 11)
    state = np.array([0, 0, 0])
    ranges = np.array([1, 2, 3, 5])
    angles = np.array([0, np.pi / 2, np.pi, 3*np.pi / 2])
    map = update_grid_map(map, ranges, angles, state, params)
    assert map[11, 11] < 0 # check if position is free
    assert map[12, 11] > 0 # first target
    assert map[11, 9]  > 0 and (map[11, 10:12] < 0).all() # second target + path
    assert map[8, 11]  > 0 and (map[9:12, 11] < 0).all() # third target + path
    assert map[11, 16] > 0 and (map[11, 12:16] < 0).all() # third target + path
