This repository contains a SLAM module for Formula Student Driverless racing car of UNIPD.

It consists in a graph-based SLAM implementation, using a backend based on pose graph optimization through g2o.

Since it works closely together with eufs_sim enviroment, you need to be careful to place it in the same workspace as eufs_sim and eufs_msgs packages, or otherwise to correctly set dependencies, includes and related stuff.

As said before, it requires g2o framework for graph optimization. The official repository, with instructions for installation, can be found [here](https://github.com/RainerKuemmerle/g2o). For convenience, the fundamental steps are reported:  
<br/>

			git clone https://github.com/RainerKuemmerle/g2o
			sudo apt-get install cmake libeigen3-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
			cd g2o
			mkdir build
			cd build
			cmake ../
			make
	
Dependencies, includes and other paths must be set according to your installation folder. The CMakeLists.txt file of this package assumes that g2o is installed in your home directory.  

If during the building of graph based slam package you encounter some error related to missing include files, undefined dependencies or similar, please check carefully the instructions in your CMakeLists.txt: the problem is probably related to linking to libraries of g2o.




# Graph-based SLAM module components

### Execute mapping module, with graph visualization

To activate the basic graph SLAM module, and see the resulting map containing cones positions and car poses, run the following commands:
 
        ros2 run graph_based_slam graph_SLAM
        ros2 run graph_based_slam graph_visualizer

If you want to execute the incremental version of the mapping module, please run 
       
        ros2 run graph_based_slam incremental_graph_SLAM
        ros2 run graph_based_slam graph_visualizer

### Execute the perception components

To execute the cones detection module using YOLO on data acquired by the camera, please run 
		
		ros2 run graph_based_slam yolo_detector

Be careful that you may need to do some modifications inside the source code of this node, depending if you want to run the detection on pre-saved images, or directly on the ROS topic published by the camera.

To execute the cones detection module by exploiting PointClouds coming from the LIDAR, please run

		ros2 run graph_based_slam lidar_detection
		
		
### Draft code, not tested

Some executables are just a first draft version of particular functionalities of the code, and they may not have been tested or used yet.
 
Such components are ``` deskew_lidar```, ```convert_GPS ``` and ```stereo_matching```.
