# ROS2 driver for Flir/PointGrey Bumblebee2 stereo cameras
This repository contains a ROS2 package that interfaces with Flir/PointGrey Bumblebee2 stereo cameras (only the 1024x768 version is supported, with basic configuration options). It is actually a wrapper around a basic, ROS agnostic driver (bumblebee2_driver).

## Dependencies
-bumblebee2_driver

After cloning this repository, you should move root directory (bumblebee2_ros_driver/) and clone the bumblebee2_driver repository:

https://github.com/albertopretto/bumblebee2_driver


## Setup

Following other parameters and default values are listed here and can be changed in `params/bumblebee2.yaml`.
``` 
    shutter: 379 # Camera shutter
    gain: 118 # Camera gain
    sched_priority: 99 # Real-time priority assigned to the process.

```

Build the package in your workspace:

    colcon build --packages-select bumblebee2_ros_driver

Source setup.bash in your workspace:

    . install/setup.bash
    
Run it:

    ros2 run bumblebee2_ros_driver bumblebee2

If you want to change the the camera shutter or the camera gain, run with:

    ros2 run bumblebee2_ros_driver bumblebee2 --ros-args -p shutter:=300 -p gain:=200

To start the same node using saved parameter values, use the --params-file option, e.g.:

    ros2 run bumblebee2_ros_driver bumblebee2 --ros-args --params-file src/bumblebee2_ros_driver/params/bumblebee2.yaml

   
