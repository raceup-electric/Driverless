import numpy as np
from scipy.io import loadmat
import matplotlib.pyplot as plt
import Plot
from PurePursuitController import PurePursuitController

# Load the MATLAB file
data = loadmat('workspace.mat')
xRef = data['xRTcc'].squeeze()  # Use .squeeze() to reduce any singleton dimensions
yRef = data['yRTcc'].squeeze()

# Simulation parameters
X = xRef[0]/50  # Initial X position of the vehicle
Y = yRef[0]/50  # Initial Y position slightly off the path
Theta = 0       # Initial yaw angle
vel = 15        # Initial velocity of the vehicle in m/s
clk = 0         # Initial clock time
dt = 0.01       # Time step in seconds
max_time = 500  # Maximum simulation time in seconds
distance_threshold = 1  # Meters, adjust as needed for accuracy
yaw = 0

L = 1.535       # Width of axle 
tire_to_steering = 28/8.55254  # Ratio from tire angle to steering wheel angle
lookahead = 10

targetdir = 0

x_vehicle = []
y_vehicle = []
steering_angles = []
time_points = []

controller = PurePursuitController(lookahead_distance=lookahead)
controller.update_path(xRef, yRef)

print("Initial Position:", X, Y)
print(f"Target Path from X={xRef[0]} to X={xRef[-1]}, Y={yRef[0]} to Y={yRef[-1]}")

# Run the simulation
while clk < max_time:
    # Assuming controller_stanley method exists and updates the yaw angle based on the vehicle's current state
    targetdir = controller.compute_target(X, Y, Theta, vel, clk)

    steering_angle = np.arctan(np.sin(targetdir) * 2 * L) + tire_to_steering
        
    steering_angles.append(steering_angle)
    
    yaw += vel * np.tan(steering_angle) * dt / 2.0  # Update yaw based on the steering angle
    X += vel * np.cos(yaw) * dt
    Y += vel * np.sin(yaw) * dt
    x_vehicle.append(X)
    y_vehicle.append(Y)
    time_points.append(clk)
    
    Theta = yaw

    print(f"At time {clk:.2f}s, Steering Angle: {steering_angle:.2f} radians, Updated Position: X={X:.2f}, Y={Y:.2f}, Yaw={yaw:.2f} radians")

    # Check if the vehicle has reached the end of the path
    if clk > 0.5:
        distance_to_end = np.sqrt((X - xRef[-1])**2 + (Y - yRef[-1])**2)
        if distance_to_end <= distance_threshold:
            print(f"Car has reached the end of the track at X={X:.2f}, Y={Y:.2f}")
            break

    clk += dt  # Increment clock time by dt seconds

# Plot the vehicle path
Plot.plot_vehicle_path(x_vehicle, y_vehicle, xRef, yRef, steering_angles, time_points)
