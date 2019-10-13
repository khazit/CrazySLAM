#!/usr/bin/env python
# coding: utf-8

# In[310]:


import numpy as np
from scipy.io import loadmat
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from skimage.draw import line as bresenham

#%matplotlib notebook


# ## Data
# ---

# In[311]:


data = loadmat("practice.mat")
data


# In[312]:


states = np.array(data["pose"])
ranges = np.array(data["ranges"])
angles = np.array(data["scanAngles"])
timestamp = np.array(data["t"])

selected_idx= np.linspace(0, len(angles), 200, dtype="uint8")
angles = angles[selected_idx, :]
ranges = ranges[selected_idx, :]


# In[313]:


ranges.shape


# ## Visualization
# ---
# ### State

# In[314]:


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

# In[315]:


plt.figure()
plt.plot(angles)
plt.ylabel("rad")
plt.title("Sensor angles")
plt.show()


# ## Algorithm
# ---

# In[316]:


from math import floor, cos, sin, degrees, copysign


# In[317]:


params = {
    "resolution": 10,  # number of cells to subdivide 1 meter
    "size": 30,        # size of the square map in meters
    "origin": (0, 0),  # (x, y)
    "min": (           # helps discretize position states
        states[0, :].min(), 
        states[1, :].min(),
    )
}

params


# ### Utility functions

# In[318]:


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


# In[319]:


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
        floor((position[0]-params["min"][0]) * params["resolution"]), 
        floor((position[1]-params["min"][1]) * params["resolution"]),
    )


# ### FInd observed cell

# In[320]:


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
        (sensor_range * cos(state[2]+sensor_bearing)) + state[0],
        (sensor_range * sin(state[2]+sensor_bearing)) + state[1],
    )


# In[321]:


params_tmp = {
    "resolution": 20,  # number of cells to subdivide 1 meter
    "size": 30,        # size of the square map in meters
    "origin": (0, 0),  # (x, y)
    "min": (           # helps discretize position states
        0, 
        0,
    )
}


tmp_map = create_empty_map(params_tmp)

state = [10, 10, -4.59]
sensor_range = 10
sensor_bearing = -1

position = discretize(state[:2], params_tmp)
target = discretize(target_cell(state, sensor_range, sensor_bearing), params_tmp)

plt.figure(figsize=(8, 7))
plt.imshow(tmp_map)
plt.plot(position[1], position[0], 'wo', markersize=15, label="vehicule")
plt.plot(target[1], target[0], 'yo', markersize=15, label="target")
plt.legend()
plt.show()


# ### Bresenham's line algorithm

# In[322]:


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


# In[323]:


get_ipython().run_cell_magic('time', '', '\ngrid_bresenham = np.zeros((40, 80))\nstart = (5, 10)\nend = (34, 75)\n\nline = bresenham_line(start, end)')


# In[324]:


grid_bresenham[start[0], start[1]] = 3
grid_bresenham[end[0], end[1]] = 4
for indexes in line:
    grid_bresenham[indexes[0], indexes[1]] = 2

plt.figure(figsize=(15, 5))
plt.imshow(grid_bresenham)
plt.colorbar()
plt.show()


# ### Path visualization

# In[325]:


def occupancy_grid_map_path(ranges, angles, states, params):
    occupancy_grid = create_empty_map(params)
    for i in range(states.shape[1]):
        x, y = discretize(tuple(states[:2, i]), params) 
        occupancy_grid[x, y] = 1 
    return occupancy_grid


# In[326]:


grid_map = occupancy_grid_map_path(ranges, angles, states, params)


# In[327]:


plt.figure(figsize=(16, 14))
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

# In[341]:


def occupancy_grid_map(ranges, angles, states, params):
    LOG_ODD_MAX  = 25
    LOG_ODD_MIN  = -5
    LOG_ODD_OCCU = 0.9
    LOG_ODD_FREE = 0.7
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


# In[342]:


grid_map = occupancy_grid_map(ranges, angles, states, params)


# In[344]:


plt.figure(figsize=(20, 15))
plt.imshow(grid_map, cmap="gray")
plt.colorbar()
plt.show()


# In[ ]:




