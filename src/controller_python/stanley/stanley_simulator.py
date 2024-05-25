import numpy as np
import Plot
import ControllerStanley as ControllerStanley
from scipy.io import loadmat
import matplotlib.pyplot as plt

# Load the MATLAB file
# -------------------------------------------------------------------------------------------------
data = loadmat('workspace.mat')
xRef = data['xRTcc'].squeeze()  # Use .squeeze() to reduce any singleton dimensions
yRef = data['yRTcc'].squeeze()
# -------------------------------------------------------------------------------------------------
# data = loadmat("waypoints_sim.mat")
# xRef = data['waypoints'][:, 0]
# yRef = data['waypoints'][:, 1]
# -------------------------------------------------------------------------------------------------
# data = loadmat("waypoints_real_delaunay.mat")
# xRef = data['waypoints'][:, 0]
# yRef = data['waypoints'][:, 1]
# -------------------------------------------------------------------------------------------------
x_vehicle = []
y_vehicle = []
steering_angles = []
time_points = []

# Instantiate the controller
controller = ControllerStanley.StanleyController()
controller.update_path(xRef, yRef)

# Simulation parameters
X = xRef[0]/50  # Initial X position of the vehicle
Y = yRef[0]/50  # Initial Y position slightly off the path
yaw = 0  # Initial heading angle, radians
vel = 20    # Initial velocity of the vehicle in m/s
clk = 0    # Initial clock time
dt = 0.01   # Time step in seconds
max_time = 500  # Maximum simulation time in seconds
distance_threshold = 1  # meters, adjust as needed for accuracy

print("Initial Position:", X, Y)
print(f"Target Path from X={xRef[0]} to X={xRef[-1]}, Y={yRef[0]} to Y={yRef[-1]}")

# Run the simulation
while clk < max_time:
    steering_angle = controller.controller_stanley(X, Y, yaw, vel, clk)
    steering_angles.append(steering_angle)
    steering_angle = steering_angle * 28/8.55254

    time_points.append(clk)
    print(f"At time {clk:.2f}s, Steering Angle: {steering_angle:.2f} radians")

    # Compute the wheel angle based on the steering angle
    # wheels_angle = (np.pi / 180) * 1163 * 0.01273 * steering_angle * np.pi / 180

    # # Calculate the angular velocity
    # if wheels_angle == 0:
    #     w = 0
    # else:
    #     w = vel / (1.535 / np.tan(wheels_angle))
    
    # # Update the vehicle's yaw (orientation)
    # yaw += w * dt
    
    # Update vehicle's position
    # X += vel * np.cos(yaw) * dt
    # Y += vel * np.sin(yaw) * dt
    # x_vehicle.append(X)
    # y_vehicle.append(Y)
    
    yaw += vel * np.tan(steering_angle) * dt / 2.0
    X += vel * np.cos(yaw) * dt
    Y += vel * np.sin(yaw) * dt
    x_vehicle.append(X)
    y_vehicle.append(Y)
    
    print(f"Updated Position: X={X:.2f}, Y={Y:.2f}, Yaw={yaw:.2f} radians")
    
    if clk > 0.5:
        distance_to_end = np.sqrt((X - xRef[-1])**2 + (Y - yRef[-1])**2)
        if distance_to_end <= distance_threshold:
            print(f"Car has reached the end of the track at X={X:.2f}, Y={Y:.2f}")
            break
    
    clk += dt  # Increment clock time by dt seconds

Plot.plot_vehicle_path(x_vehicle, y_vehicle, xRef, yRef, steering_angles, time_points)
