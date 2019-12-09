"""SLAM module

This module implements ...
"""


import numpy as np
from crazyslam.mapping import update_grid_map, create_empty_map
from crazyslam.localization import get_state_estimate


class SLAM():
    """
    """

    def __init__(
        self,
        params,
        n_particles,
        current_state,
        system_noise_variance,
        correlation_matrix,
    ):
        """
        """
        self.map = create_empty_map(params)
        self.params = params
        self.n_particles = n_particles
        self.system_noise_variance = system_noise_variance
        self.correlation_matrix = correlation_matrix
        self.resampling_threshold = (n_particles * 10) // 100
        self.current_state = current_state
        self.particles = particles = np.zeros((4, n_particles))
        self.particles[:3, :] = current_state.reshape((3, 1)) \
            * np.ones((3, n_particles))
        self.particles[3, :] = (1/500) * np.ones((1, n_particles))

    def update_state(self, ranges, angles, motion_update):
        """
        """
        self.map = update_grid_map(
            self.map,
            ranges,
            angles,
            self.current_state,
            self.params,
        )

        # motion model update
        self.particles[:3, :] += motion_update.reshape((3, 1)) \
            * np.ones((3, self.n_particles))

        self.current_state, self.particles = get_state_estimate(
            self.particles,
            self.system_noise_variance,
            self.correlation_matrix,
            self.map,
            self.params,
            ranges,
            angles,
            self.resampling_threshold
        )

        return self.current_state
