import pytest
from crazyslam.localization import *
from crazyslam.mapping import *

def test_init_random_particles():
    assert init_random_particles(10).shape[0] == 4
    assert init_random_particles(10).shape[1] == 10

def test_normalize_weights():
    particles = init_random_particles(10)
    assert np.isclose(np.sum(normalize_weights(particles[-1, :])), 1)

def test_get_correlation_score():
    params = init_params_dict(11, 1)
    map = create_empty_map(params)
    map[[2, 6, 8], [1, 1, 5]] = 10
    map[[1, 7, 8], [1, 2, 3]] = -5
    correlation_matrix = np.array([
        [0, -1],
        [0,  1],
    ])
    target_cells = np.array([
        [2, 8, 1],
        [1, 5, 7],
    ])
    assert get_correlation_score(map, target_cells, correlation_matrix) == 1

def test_update_particle_weights():
    params = init_params_dict(11, 1)
    map = create_empty_map(params)
    map[[2, 6, 8], [1, 1, 5]] = 10
    map[[1, 7, 8], [1, 2, 3]] = -5
    correlation_matrix = np.array([
        [0, -1],
        [0,  1],
    ])
    particles = np.array([
        [  1,   2],
        [  0,   0],
        [  0,   1],
        [0.5, 0.5],
    ])
    ranges = np.array([2, 4])
    angles = np.array([0, np.pi / 2])
    test_particles = update_particle_weights(
        particles,
        correlation_matrix,
        map,
        params,
        ranges,
        angles,
    )
    assert test_particles[-1, 0] > test_particles[-1, 1]

def test_get_best_particle():
    particles = np.array([
        [1, 7, 2, 7],
        [0, 4, 2, 7],
        [1, 3, 2, 7],
        [1, 7, 99, 13],
    ])
    assert np.all(get_best_particle(particles) == np.array([2, 2, 2, 99]))

def test_get_state_estimate():
    params = init_params_dict(11, 1)
    map = create_empty_map(params)
    map[[2, 6, 8], [1, 1, 5]] = 10
    map[[1, 7, 8], [1, 2, 3]] = -5
    correlation_matrix = np.array([
        [0, -1],
        [0,  1],
    ])
    particles = np.array([
        [  1,   2],
        [  0,   0],
        [  0,   1],
        [0.5, 0.5],
    ])
    ranges = np.array([2, 4])
    angles = np.array([0, np.pi / 2])
    system_noise_variance = np.diag([1e-10, 1e-10, 1e-10])

    state_estimate = get_state_estimate(
        particles,
        system_noise_variance,
        correlation_matrix,
        map,
        params,
        ranges,
        angles,
        resample_threshold=0
    )[0]
    assert np.all(
        np.abs(state_estimate - particles[:3, 0]) <
        np.abs(state_estimate - particles[:3, 1])
    )
