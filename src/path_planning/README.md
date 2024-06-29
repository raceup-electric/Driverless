
# Path Planning

Node responsible for path planning in the driverless vehicle system. It receives PoseGraph messages from SLAM and calculates the trajectory for the steering angle calculation of the controller. The first lap is on 'Exploration mode', the second is on 'Optimization mode' (the trajectory is optimized with a min-curve criterion).

## Dependencies

List of dependencies required to run this node:
- trajectory_planning_helpers == 0.76
- numpy >= 1.18.1, <= 1.21.1
- scipy >= 1.3.3
- quadprog == 0.1.7
- matplotlib >= 3.3.1

To install the dependencies:
```bash
	pip3 intall dependency_name == version	
```

## Installation

Instructions for installing dependencies (if any) and setting up the environment for running this node.

## Usage

Instructions on how to run this node:

1. Clone this repository:

    ```bash
    git clone https://github.com/this_is_just_a_sample
    ```

2. Navigate to the directory of this node:

    ```bash
    cd your_repository/path/to/this/node
    ```

3. Build the ROS workspace:

    ```bash
    colcon build
    ```

4. Source the ROS setup file:

    ```bash
    source install/setup.bash
    ```

5. Launch the node:

    ```bash
    ros2 run package_name node_executable
    ```

   Replace `package_name` with the name of the ROS package containing this node, and `launch_file.launch` with the launch file for this node.

## Parameters

Explanation of any configurable parameters and their default values.

## ROS Topics

List of ROS topics published and subscribed by this node, along with their message types:

### Publishers:
		- cones
		- distances
### Subscribed to:
		- SLAM 
		- 
### Subscribers:
		- Path Planning
		-  
## Changelog

### Version X.Y.Z (YYYY-MM-DD)

- Change 1
- Change 2
- ...

### Version X.Y.Z-1 (YYYY-MM-DD)

- Change 1
- Change 2
- ...
