import numpy as np
import matplotlib.pyplot as plt

def plot_vehicle_path(x_vehicle, y_vehicle, x_ref, y_ref, steering_angles, time_points):
    """
    Plots the path of the vehicle along with the reference path, a 2-meter wide border around it,
    and steering angles over time.
    
    Parameters:
        x_vehicle (list): X coordinates of the vehicle's path.
        y_vehicle (list): Y coordinates of the vehicle's path.
        x_ref (np.array): X coordinates of the reference path.
        y_ref (np.array): Y coordinates of the reference path.
        steering_angles (list): Steering angles at each time point.
        time_points (list): Simulation time points.
    """
    plt.figure(figsize=(12, 6))
    
    # Compute tangents and normals for the reference path
    tangents = np.diff(np.column_stack((x_ref, y_ref)), axis=0)
    normals = np.stack([-tangents[:, 1], tangents[:, 0]], axis=1)
    norm_lengths = np.linalg.norm(normals, axis=1)
    normals /= norm_lengths[:, np.newaxis]  # Normalize

    # Compute the offset paths
    offset_distance = 2
    x_outer = x_ref[:-1] + normals[:, 0] * offset_distance
    y_outer = y_ref[:-1] + normals[:, 1] * offset_distance
    x_inner = x_ref[:-1] - normals[:, 0] * offset_distance
    y_inner = y_ref[:-1] - normals[:, 1] * offset_distance

    # Plot the reference and vehicle paths
    plt.subplot(1, 2, 1)
    plt.plot(x_ref, y_ref, 'g--', label='Reference Path')
    plt.plot(x_outer, y_outer, 'y--', label='Outer Border')
    plt.plot(x_inner, y_inner, 'y--', label='Inner Border')
    plt.plot(x_vehicle, y_vehicle, 'b-', label='Vehicle Path')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Vehicle and Reference Paths')
    plt.legend()
    plt.axis('equal')

    # Plot the steering angles
    plt.subplot(1, 2, 2)
    plt.plot(time_points, steering_angles, 'r-')
    plt.xlabel('Time (s)')
    plt.ylabel('Steering Angle (radians)')
    plt.title('Steering Angle Over Time')
    plt.grid(True)

    plt.tight_layout()
    plt.show()

# Now you can call this function with your data.
