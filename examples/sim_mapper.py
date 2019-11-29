import argparse
from tqdm import tqdm
from scipy.io import loadmat
import matplotlib.pyplot as plt
import matplotlib
from crazyslam.mapping import *


parser = argparse.ArgumentParser()
parser.add_argument(
    "--n_data_points",
    default=4,
    help="Number of data points to use for each scan",
)


if __name__ == '__main__':
    args = parser.parse_args()

    data = loadmat("data/mapping_data.mat")
    states = np.array(data["pose"])
    ranges = np.array(data["ranges"])
    angles = np.array(data["scanAngles"])
    timestamp = np.array(data["t"])

    selected_idx = np.linspace(
        0,
        len(angles)-1,
        int(args.n_data_points),
        dtype="int32"
    )
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

    matplotlib.rc('xtick', labelsize=5)
    matplotlib.rc('ytick', labelsize=5)
    plt.figure()
    plt.imshow(occupancy_grid, cmap="gray")
    plt.title("{} data points at each scan".format(args.n_data_points))
    plt.show()
