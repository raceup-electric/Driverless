import numpy as np
import ControllerStanley
import Plot
import matplotlib.pyplot as plt
import from
from scipy.io import loadmat




def __init__(self):
    self.planned_path = self.create_subscription(
        PlannedTrajectory,
        "planned_trajectory",
        self.__trajectoryCallback,
        200)

def __trajectoryCallback(self, planned_trajectory: PlannedTrajectory):

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
    controller = ControllerStanley.ControllerStanley()
    controller.update_path(xRef, yRef)

    # Simulation parameters
    X = xRef[0]   # Initial X position of the vehicle
    Y = yRef[0]   # Initial Y position slightly off the path
    Theta = 0  # Initial heading angle, radians
    vel = 9    # Initial velocity of the vehicle in m/s
    clk = 0    # Initial clock time
    dt = 0.1   # Time step in seconds


    print("Initial Position:", X, Y)
    print(f"Target Path from X={xRef[0]} to X={xRef[-1]}, Y={yRef[0]} to Y={yRef[-1]}")

    # Distance threshold to determine if the car has reached the end of the track
    distance_threshold = 0.3  # meters, adjust as needed for accuracy

    # Run the simulation
    while True:
        steering_angle = controller.controller_stanley(planned_trajectory.target_x, planned_trajectory.target_y, Theta, vel, clk)
        steering_angles.append(steering_angle)
        time_points.append(clk)
        print(f"At time {clk:.2f}s, Steering Angle: {steering_angle:.2f} radians")

        # Update vehicle state based on motion model
        Theta += vel * np.tan(steering_angle) * dt / 2.0
        X += vel * np.cos(Theta) * dt
        Y += vel * np.sin(Theta) * dt
        x_vehicle.append(X)
        y_vehicle.append(Y)

        # Check if the car is close to the end of the track
        distance_to_end = np.sqrt((X - xRef[-1])**2 + (Y - yRef[-1])**2)
        if distance_to_end <= distance_threshold:
            print(f"Car has reached the end of the track at X={X:.2f}, Y={Y:.2f}")
            break

        clk += dt  # Increment clock time by dt seconds

        print(f"Updated Position: X={X:.2f}, Y={Y:.2f}, Theta={Theta:.2f} radians")

    Plot.plot_vehicle_path(x_vehicle, y_vehicle, xRef, yRef, steering_angles, time_points)
