#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import numpy as np
from scipy.io import loadmat
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from skimage.draw import line as bresenham

#%matplotlib notebook


# ## Data
# ---

# In[ ]:


data = loadmat("mapping_data.mat")
data


# In[ ]:


states = np.array(data["pose"])
ranges = np.array(data["ranges"])
angles = np.array(data["scanAngles"])
timestamp = np.array(data["t"])

selected_idx= np.linspace(0, len(angles)-1, 4, dtype="int32")
angles = angles[selected_idx, :]
ranges = ranges[selected_idx, :]


# In[ ]:


ranges.shape


# ## Visualization
# ---
# ### State

# In[ ]:


plt.figure(figsize=(14, 6))
plt.subplot(121)
plt.plot(states[1], states[0])
plt.xlabel("y (m)")
plt.gca().invert_yaxis()
plt.ylabel("x (m)")
plt.title("Position")

plt.subplot(122)
plt.plot(timestamp[0], states[2])
plt.xlabel("t")
plt.ylabel("rad")
plt.title("Heading")
plt.show()


# #### Sensor bearing

# In[ ]:


plt.figure()
plt.plot(angles)
plt.ylabel("rad")
plt.title("Sensor angles")
plt.show()


# ## Algorithm
# ---

# In[ ]:


from math import floor, cos, sin, degrees, copysign, pi


# In[ ]:


params = {
    "resolution": 10,  # number of cells to subdivide 1 meter
    "size": 70,        # size of the square map in meters
    "origin": None,  
}
params["origin"] = (
    params["resolution"]*params["size"]//2,
    params["resolution"]*params["size"]//2,
)
params


# ### Utility functions

# In[ ]:


def create_empty_map(params):
    """Return an empty maps of size defined by params
    
    Map is a square of n = size in meters * resolution
    The x-axis is pointing downward and the y-axis towards the right 
    
    Args:
        params: Dict of parameters
        
    Returns:
        Square numpy array
    """
    return np.zeros((params["size"]*params["resolution"], params["size"]*params["resolution"]))


# In[ ]:


def discretize(position, params):
    """Discretize the vehicule position
    
    Given a (x, y) tuple of GLOBAL coordinates, compute the corresponding
    indexes on the grid map
    
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

discretize((0, 0), params)


# ### FInd observed cell

# In[ ]:





# In[ ]:


def target_cell(state, sensor_range, sensor_bearing):
    """Find the (x, y) GLOBAL coordinates of the observed point
    
    IMPORTANT: target point could be out of range (not in the map)
    
    Args:
        state: (x, y, alpha) state of the vehicule in the GLOBAL frame
        sensor_range: Observed range
        sensor_bearing: Sensor heading
        
    Returns:
        Tuple of (x, y) GLOBAL coordinates
    """
    return (
        ( sensor_range * cos(state[2]+sensor_bearing)) + state[0],
        (-sensor_range * sin(state[2]+sensor_bearing)) + state[1],
    )


# In[ ]:


params_tmp = {
    "resolution": 5,  # number of cells to subdivide 1 meter
    "size": 10,        # size of the square map in meters
}
params_tmp["origin"] = (
    params_tmp["resolution"]*params_tmp["size"]//2,
    params_tmp["resolution"]*params_tmp["size"]//2,
)

print(params_tmp)
tmp_map = create_empty_map(params_tmp)

state = [0, 0, 0]
position = discretize(state[:2], params_tmp)

plt.figure(figsize=(8, 7))
plt.imshow(tmp_map)
plt.plot(position[1], position[0], 'wo', markersize=15, label="vehicule")

first_target = discretize(target_cell(state, ranges[1, 0], angles[0]), params_tmp)
plt.plot(first_target[1], first_target[0], 'co', markersize=10, label="first")

last_target = discretize(target_cell(state, ranges[1, -1], angles[-1]), params_tmp)
plt.plot(last_target[1], last_target[0], 'ro', markersize=10, label="last")


for i in range(len(angles)):
    target = discretize(target_cell(state, ranges[0, i], angles[i]), params_tmp)
    plt.plot(target[1], target[0], 'yo', markersize=1)

plt.legend()
plt.show()


# ### Bresenham's line algorithm

# In[ ]:


def bresenham_line(start, end):
    """Find the points that should be selected in order to form a straight line between two points.
    
    Use scikit-image implementation of the Bresenham line algorithm
    
    Args:
        start: (x, y) GLOBAL index coordinates of the starting point of the line
        end: (x, y) GLOBAL index coordinates of the ending point of the line
    
    Returns:
        List of (x, y) index coordinates that form the straight line
    """
    tmp = bresenham(start[0], start[1], end[0], end[1])
    return list(zip(tmp[0], tmp[1]))[1:-1] # start and end points are removed


# In[ ]:


get_ipython().run_cell_magic('time', '', '\ngrid_bresenham = np.zeros((40, 80))\nstart = (5, 10)\nend = (34, 75)\n\nline = bresenham_line(start, end)')


# In[ ]:


grid_bresenham[start[0], start[1]] = 3
grid_bresenham[end[0], end[1]] = 4
for indexes in line:
    grid_bresenham[indexes[0], indexes[1]] = 2

plt.figure(figsize=(15, 5))
plt.imshow(grid_bresenham)
plt.colorbar()
plt.show()


# ### Path visualization

# In[ ]:


def occupancy_grid_map_path(ranges, angles, states, params):
    occupancy_grid = create_empty_map(params)
    for i in range(states.shape[1]):
        x, y = discretize(tuple(states[:2, i]), params) 
        occupancy_grid[x, y] = 1 
    return occupancy_grid


# In[ ]:


grid_map = occupancy_grid_map_path(ranges, angles, states, params)


# In[ ]:


plt.figure(figsize=(20, 18))
plt.imshow(grid_map)

start = discretize(states[:2, 0], params)
plt.plot(start[1], start[0], 'ro', markersize=10, label="start")

for i in range(states.shape[1]):
    if i%150 == 0:
        # arrow base = vehicule position
        x_arrow, y_arrow = discretize(tuple(states[:2, i]), params)
        
        # end point of the arrow
        dx_arrow = states[0, i]+cos(states[2, i])
        dy_arrow = states[1, i]+sin(states[2, i]) 
        dx_arrow, dy_arrow = discretize((dx_arrow, dy_arrow), params)
        
        # plot the arrow
        plt.text(
            dy_arrow, 
            dx_arrow, 
            "{:4.2f}".format(states[2, i]), 
            color="c", 
            fontdict={"size" : 13} 
        )
        plt.arrow(y_arrow, x_arrow, dy_arrow-y_arrow, dx_arrow-x_arrow, color="white", width=1.5)
        
end = discretize(states[:2, -1], params)
plt.plot(end[1], end[0], 'co', markersize=10, label="end")

plt.colorbar()
plt.legend()
plt.show()


# ### Main function
# ---

# In[ ]:


def occupancy_grid_map(ranges, angles, states, params):
    LOG_ODD_MAX  = 100
    LOG_ODD_MIN  = -30
    LOG_ODD_OCCU = 1
    LOG_ODD_FREE = 0.3
    occupancy_grid = create_empty_map(params)
    
    # for each timestamp
    for i in range(states.shape[1]):
        # for each scan 
        for idx, d in enumerate(ranges[:, i]):
            # compute the measured position
            target = target_cell(states[:, i], d, angles[idx])
            target = discretize(target, params)
            # find the affected cells
            position = discretize(states[:2, i], params)
            path = bresenham_line(position, target)
            # update log odds
            for cell in path:
                occupancy_grid[cell] += LOG_ODD_FREE
            occupancy_grid[target] -= LOG_ODD_OCCU
    
    return np.clip(occupancy_grid, a_max=LOG_ODD_MAX, a_min=LOG_ODD_MIN)


# In[ ]:


grid_map = occupancy_grid_map(ranges, angles, states, params)


# In[ ]:


plt.figure(figsize=(40, 30))
plt.imshow(grid_map, cmap="gray_r")
plt.colorbar()
plt.show()

