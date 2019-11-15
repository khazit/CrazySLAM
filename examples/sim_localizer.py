from crazyslam.localization import *
from crazyslam.mapping import *
from tqdm import tqdm
from scipy.io import loadmat
import matplotlib.pyplot as plt


if __name__ == '__main__':
    # Load data
    data = loadmat("crazyslam/data/localization_data.mat")
    ranges = data["ranges"]
    angles = data["scanAngles"]
    selected_idx = np.linspace(0, len(angles)-1, 100, dtype="int32")
    angles = angles[selected_idx, :]
    ranges = ranges[selected_idx, :]
    gt_map = data["M"].T - 0.5

    # Init useful variables
    count = 3700
    n_particles = 300
    params = init_params_dict(size=30, resolution=25, origin=(684, 571))
    particles = init_random_particles(n_particles)
    pose = np.zeros((3, count))
    system_noise_variance = np.diag([1e-3, 1e-3, 1e-5])
    correlation_matrix = np.array([
        [0, -1],
        [-1, 10],
    ])
    resampling_threshold = 100

    # Main loop
    for t in tqdm(range(count)):
        pose[:, t], particles = get_state_estimate(
            particles,
            system_noise_variance,
            correlation_matrix,
            gt_map,
            params,
            ranges[:, t],
            angles,
            resampling_threshold,
        )

    # Visualization
    position = discretize(pose[:2, :], params)
    plt.figure()
    plt.imshow(gt_map, cmap="gray")
    plt.plot(position[1, :], position[0, :], "bo", markersize=1, label="estimate")
    plt.show()
