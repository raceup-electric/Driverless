

1.18.1 =< numpy =< 1.21.1
scipy>=1.3.3
quadprog==0.1.7
matplotlib>=3.3.1


SETUP INFORMATION
------ SETUP ------	
1. Clone the repo
	git clone https://github.com/Beomar97/path-planning.git
	
2. Clone in path_planning/src/ the custom messages
	git clone https://github.com/Beomar97/ros-msgs.git
	
3. Remove the old fszhaw_msgs and rename the new folder  fszhaw_msgs

4. Add to path_planning/src/fszhaw_msgs/fs_msgs/msg:
	ConeArrayWithCovariance.msg
	ConeWithCovariance.msg

5. Include in path_planning/src/fszhaw_msgs/fs_msgs/CMakeLists.txt:
	msg/ConeArrayWithCovariance.msg
  	msg/ConeWithCovariance.msg

6. Repeat 4. and 5. for path_planning/src/fszhaw_msgs/fszhaw_msgs/


------ LAUNCH THE PROGRAM ------
1. Initialize submodules (fszhaw_msgs) (Currently only available in the ZHAW network)
	git submodule update --init

2. Install Python 3 dependencies necessary for ROS 2
	sudo apt install python3-pip python3-rosdep2 python3-colcon-common-extensions python3-pip
	
3. Initialize rosdep installation
	rosdep update

4. Source ROS distribution
	source /opt/ros/foxy/setup.bash

5. Install dependencies (from workspace root path_planning)
	rosdep install -i --from-path src --rosdistro foxy -y

6. Install dependencies not available via rosdep (e.g. trajectory-planning-helpers)
	pip3 install trajectory_planning_helpers==0.76

-- optional (if given some error on custom messages): remove in path_planning the following folders:
	-build
	-install
	-log

7. Build packages
	colcon build

8. Source setup files
	source install/setup.bash

9. Run package
	ros2 launch launch/path_planning_launch.py <parameter_name>:=<parameter_value>
