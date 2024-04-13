# ROS2 driver for legacy 3rd generation MTi devices
This repository contains a ROS2 package that interfaces with XSens 3rd generation MTi devices (without supporting internal GPS data, if available). It is actually a wrapper around a basic, ROS agnostic driver (xsens_mti_driver). The orientation is calculated from accelerometer and magnetometer measurements and is currently unfiltered (the conversion code has been taken from https://github.com/hiwad-aziz/ros2_mpu9250_driver )

## Dependencies
-xsens_mti_driver

After cloning this repository, you should move root directory (xsens_mti_ros_driver/) and clone the xsens_mti_driver repository:

https://github.com/albertopretto/xsens_mti_driver


## Setup

Following other parameters and default values are listed here and can be changed in `params/xsens_mti.yaml`.
``` 
    sample_frequency: 100 # [Hz]
    calibrated: True # Use calibrated IMU data with factory calibration
    low_latency_serial: True # Acquire (and timestamp) as soon as a new packet is available
    sched_priority: 99 # Real-time priority assigned to the process
```

Build the package in your workspace:

    colcon build --packages-select xsens_mti_ros_driver

Source setup.bash in your workspace:

    . install/setup.bash
    
Run it:

    ros2 run xsens_mti_ros_driver xsens_mti

If you want to change the sample frequency e.g., to 200 Hz, run with (frequencies higher than 200 Hz are not recommended for computational efficiency):

    ros2 run xsens_mti_ros_driver xsens_mti --ros-args -p sample_frequency:=200
