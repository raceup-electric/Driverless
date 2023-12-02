#!/bin/bash 
source /opt/ros/foxy/setup.bash

BASEDIR=$(dirname "$0")
cd "$BASEDIR"
. install/setup.bash

ros2 launch launch/path_planning_launch_test.py
ros2 launch src/path_planning/path_planning/optimization_service.py