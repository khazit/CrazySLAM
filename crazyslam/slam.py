"""SLAM module

This module implements a SLAM algorithm to correct noisy motion updates and
state estimates while also mapping the environment.
"""


import numpy as np
from crazyslam.mapping import update_grid_map, create_empty_map
from crazyslam.localization import get_state_estimate


class SLAM():
    """
    SLAM agent. Initialized at the beginning of the flight.

    The main goal of this class is to store all the useful variables for
    the SLAM algorithm.

    Attributes:
        map: Occupancy grid map
        params: Grid map parameters dictionary
        n_particles: Number of particles for the Particle Filter
        system_noise_variance: Variance for noise generation
        correlation_matrix: Matrix for computing the correlation scores
        resampling_threshold: Threshold for resampling
        current_state: Current state (i.e. particle with the highest score)
        particles: Set of state estimates and their corresponding weight
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
        Initialize a SLAM agent.

        Store all arguments and initialize the particles with current_state
        as a first state estimate.
        """
        self.map = create_empty_map(params)
        self.params = params
        self.n_particles = n_particles
        self.system_noise_variance = system_noise_variance
        self.correlation_matrix = correlation_matrix
        self.resampling_threshold = (n_particles * 10) // 100
        self.current_state = current_state
        self.particles = np.zeros((4, n_particles))
        self.particles[:3, :] = current_state.reshape((3, 1)) \
            * np.ones((3, n_particles))
        self.particles[3, :] = (1/500) * np.ones((1, n_particles))

    def update_state(self, ranges, angles, motion_update):
        """
        Update state estimate. One iteration of the SLAM algorithm

        Args:
            ranges: Set on range inputs from sensor
            angles: Scan angles
            motion_update: Update to apply to the current state

        Returns:
            Updated state estimate

        """
        # map update
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

        # state update
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
