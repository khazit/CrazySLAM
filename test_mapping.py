from crazyslam.mapping import *

if __name__ == '__main__':
    from scipy.io import loadmat
    import matplotlib.pyplot as plt
    from tqdm import tqdm

    data = loadmat("crazyslam/data/mapping_data.mat")
    states = np.array(data["pose"])
    ranges = np.array(data["ranges"])
    angles = np.array(data["scanAngles"])
    timestamp = np.array(data["t"])

    selected_idx = np.linspace(0, len(angles)-1, 100, dtype="int32")
    angles = angles[selected_idx, :]
    ranges = ranges[selected_idx, :]

    params = init_params_dict(70, 10)
    occupancy_grid = create_empty_map(params)

    # for each timestamp
    for i in tqdm(range(states.shape[1])):
        occupancy_grid = update_grid_map(
            occupancy_grid,
            ranges[:, i],
            angles,
            states[:, i],
            params
        )

    plt.figure()
    plt.imshow(occupancy_grid, cmap="gray")
    plt.show()
