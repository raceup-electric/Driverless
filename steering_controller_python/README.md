## Overview
The `simulation.py` script is used to the steering controller based on predefined paths. It employs a Stanley controller for path tracking and visualizes the results using matplotlib.

## Dependencies
- **numpy**: For numerical operations.
- **matplotlib**: For plotting simulation results.
- **scipy**: Specifically, `loadmat` for loading MATLAB `.mat` files.
- **ControllerStanley**: A custom module that implements the Stanley control method.
- **Plot**: A custom module for plotting utilities.

# Data Loading

Data for the simulation is loaded from MATLAB files. Uncomment the section corresponding to the data you want to use:

# Vi-Grade Autocross Long Test Track:

```bash
data = loadmat('workspace.mat')
xRef = data['xRTcc'].squeeze()  # Use .squeeze() to reduce any singleton dimensions
yRef = data['yRTcc'].squeeze()
```
# Simulated Circuit by Jad:

```bash
data = loadmat("waypoints_sim.mat")
xRef = data['waypoints'][:, 0]
yRef = data['waypoints'][:, 1]
```

# 2023 Data Acquisition Test Track with Jad's Delaunay Triangulation:

```bash
data = loadmat("waypoints_real_delaunay.mat")
xRef = data['waypoints'][:, 0]
yRef = data['waypoints'][:, 1]
```