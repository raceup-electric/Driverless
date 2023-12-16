# HOW BUMBLEBEE WORK

## HOW TO INSTALL
- 1) pull the directory in your eufs_WS main directory.
- 2) source the WS using "source/install.bash" (everytime you open a new terminal remember to source).

- 3) use the command "colcon build" to build the package.
- 4) if there are some errors try to resolve it, if the problem is that a package is unable to find use: "sudo apt install <package name>" or "sudo apt install ros-foxy<package name>" (all the (underline)_ have to be changed in (dash)-).
- 4b) if the build continue to fail, make sure that the followin line of the graph_base_slam/CMakeList.txt are commented:
  - line 43 "link_directories(${G2O_LIBRARY_DIRS})".
  - line 55 "find dependencies".
  - line 65 "ind_package(g2o REQUIRED)".
  - line 89 "find_package(graph_based_slam REQUIRED)".
  - line 131 "target_link_libraries(ks_kinematics g2o::core g2o::types_slam2d g2o::solver_eigen)".
  - line 310 "ament_export_include_directories("~/Documenti/thesis/FSD_SLAM_navigation/eufs_sim_folder/src/slam_module/src/graph_based_slam/DeepNetPorting/include/")".

## HOW TO READ DATA FROM ROSBAG USING BUMBLEBEE2

test_bumblebee2 subscribe to StereoImage topic and read the data using custom msgs(all the message used are from ufs_msgs).

- 1) you have to start reading the rosbag using this command "ros2 bag play <nameOfTheBagFile.db3>".
- 2) from the main directory of the WS,you have to use this command: "ros2 run bumblebee2_ros_driver test_bumblebee2".
- 3) if all works properly now you should see two windows (one for camera) with the image.

## HOW TO COLLECT DATA FOR ROSBAG USING BUMBLEBEE2

bumblebee2 publish StereoImage topic and using custom msgs(all the message used are from ufs_msgs).

- 1) from the main directory of the WS,you have to use this command: "ros2 run bumblebee2_ros_driver bumblebee2".
- 2) if all works properly bumblebee2 should publish the StereoCamera data
