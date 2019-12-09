import argparse
from tqdm import tqdm
from scipy.io import loadmat
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from crazyslam.slam import SLAM
from crazyslam.mapping import init_params_dict, discretize


parser = argparse.ArgumentParser()
parser.add_argument(
    "--n_data_points",
    default=4,
    help="Number of data points to use for each scan",
)
parser.add_argument(
    "--n_particles",
    default=100,
    help="Number of particles in the particle filter",
)


if __name__ == '__main__':
    args = parser.parse_args()

    # Load data
    data = loadmat("data/localization_data.mat")
    ranges = data["ranges"]
    angles = data["scanAngles"]
    states = np.array(data["pose"])
    selected_idx = np.linspace(
        0,
        len(angles)-1,
        int(args.n_data_points),
        dtype="int32"
    )
    angles = angles[selected_idx, :]
    ranges = ranges[selected_idx, :]

    # Add noise to the ground truth
    motion_updates = np.diff(states, axis=1, prepend=np.zeros((3, 1)))
    noise = np.concatenate([
        np.random.normal(
            loc=0,
            scale=0.02,
            size=states.shape[1]*2
        ).reshape((2, -1)),
        np.random.normal(
            loc=0,
            scale=0.02,
            size=states.shape[1]
        ).reshape((1, -1))
    ], axis=0)
    motion_updates += noise
    states_noise = np.cumsum(motion_updates, axis=1)

    # Useful values
    system_noise_variance = np.diag([0.2, 0.2, 0.2])
    correlation_matrix = np.array([
        [0, -1],
        [-1, 10],
    ])
    slam_states = np.zeros_like(states_noise)

    # Init the SLAM agent
    slam_agent = SLAM(
        params=init_params_dict(size=70, resolution=10),
        n_particles=int(args.n_particles),
        current_state=states_noise[:, 0],
        system_noise_variance=system_noise_variance,
        correlation_matrix=correlation_matrix,
    )

    # Main loop
    for t in tqdm(range(states_noise.shape[1])):
        slam_states[:, t]  = slam_agent.update_state(
            ranges[:, t],
            angles,
            motion_updates[:, t],
        )

    slam_map = slam_agent.map
    idx_slam = discretize(slam_states[:2, :], slam_agent.params)
    idx_noise = discretize(states_noise[:2, :], slam_agent.params)

    plt.figure(figsize=(11, 11))
    plt.imshow(slam_map, cmap="gray")
    plt.plot(idx_slam[1, :], idx_slam[0, :], "-r", label="slam")
    plt.plot(idx_noise[1, :], idx_noise[0, :], "-y", label="noise")
    plt.legend()
    plt.show()
