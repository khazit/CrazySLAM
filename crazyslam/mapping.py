"""Mapping module

This module ...
"""


import numpy as np
from math import floor, cos, sin
from skimage.draw import line as bresenham


def init_params_dict(size, resolution):
    """Initialize the parameters dictionary given a map size and resolution

    Args:
        size: Size of the square map in meters
        resolution: Number of cells to subdivide 1 meter into

    Returns:
        Parameters dictionary:
            "resolution": Resolution
            "size": Size
            "origin": Starting coordinates of the Crazyflie (middle of the map)
    """
    params = {
        "resolution": resolution,
        "size": size,
        "origin": None,
    }
    params["origin"] = (
        params["resolution"]*params["size"]//2,
        params["resolution"]*params["size"]//2,
    )
    return params


def create_empty_map(params):
    """Return an empty map of size params.size

    Map is a square matrix of n = params.size * params.resolution
    The x-axis is pointing downward and the y-axis towards the right

    Args:
        params: Dict of parameters

    Returns:
        Square numpy array
    """
    return np.zeros((
        params["size"]*params["resolution"],
        params["size"]*params["resolution"],
    ))


def discretize(position, params):
    """Discretize the vehicule position

    Given a (x, y) tuple of GLOBAL coordinates, compute the corresponding
    indexes on the grid map.
    The (0, 0) coordinates are put in the middle of the map.


    Args:
        position: Tuple of (x, y) GLOBAL coordinates
        params: Dict of parameters

    Returns:
        Tuple of (x, y) GLOBAL index coordinates

    """
    return (
        floor((position[0]) * params["resolution"]) + params["origin"][0],
        floor((position[1]) * params["resolution"]) + params["origin"][1],
    )


def target_cell(state, sensor_range, sensor_bearing):
    """Find the (x, y) GLOBAL coordinates of the observed point

    NOTE: target point could be out of range (not in the map)

    Args:
        state: (x, y, alpha) state of the vehicule in the GLOBAL frame
        sensor_range: Observed range
        sensor_bearing: Sensor heading

    Returns:
        Tuple of (x, y) GLOBAL coordinates
    """
    return (
        (sensor_range * cos(state[2]+sensor_bearing)) + state[0],
        (-sensor_range * sin(state[2]+sensor_bearing)) + state[1],
    )


def bresenham_line(start, end):
    """Find the cells that should be selected to form a straight line

    Use scikit-image implementation of the Bresenham line algorithm

    Args:
        start: (x, y) GLOBAL index coordinates of the starting point
        end: (x, y) GLOBAL index coordinates of the ending point

    Returns:
        List of (x, y) index coordinates that form the straight line
    """
    tmp = bresenham(start[0], start[1], end[0], end[1])
    return list(zip(tmp[0], tmp[1]))[1:-1]  # start and end points are removed


def update_grid_map(grid, ranges, angles, state, params):
    """Update the grid map given a new set on sensor data

    Args:
        grid: Grid map to be updated
        ranges: Set of range inputs from the sensor
        angles: Angles at which the range points are captured
        state: State estimate (x, y, yaw)
        params: Parameters dictionary

    Returns:
        Updated occupancy grid map
    """
    LOG_ODD_MAX = 100
    LOG_ODD_MIN = -30
    LOG_ODD_OCCU = 1
    LOG_ODD_FREE = 0.3
    for idx, d in enumerate(ranges):
        # compute the vehicule position
        position = discretize(state[:2], params)
        # compute the target position
        target = target_cell(state, d, angles[idx])
        target = discretize(target, params)
        # find the affected cells
        path = bresenham_line(position, target)
        # update log odds
        grid[position] += LOG_ODD_FREE
        for cell in path:
            grid[cell] += LOG_ODD_FREE
        grid[target] -= LOG_ODD_OCCU
    return np.clip(grid, a_max=LOG_ODD_MAX, a_min=LOG_ODD_MIN)


if __name__ == '__main__':
    from scipy.io import loadmat
    import matplotlib.pyplot as plt

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
    for i in range(states.shape[1]):
        occupancy_grid = update_grid_map(
            occupancy_grid,
            ranges[:, i],
            angles,
            states[:, i],
            params
        )

    plt.figure()
    plt.imshow(occupancy_grid, cmap="gray_r")
    plt.show()
