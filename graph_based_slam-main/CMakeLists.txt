cmake_minimum_required(VERSION 3.5)
project(graph_based_slam)

set(CMAKE_BUILD_TYPE "RELEASE")

LIST(APPEND CMAKE_PREFIX_PATH "/home/navid/libtorch/share/cmake/Torch")


# suppress confusing developers warning
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
  set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


#SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,/usr/local/lib")
#set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

#---------------------
#add_subdirectory(DeepNetPorting) #add this on target_link_libraries of the executable
#set(yolo_LIB_DIR "~/Documenti/thesis/FSD_SLAM_navigation/eufs_sim_folder/src/slam_module/src/graph_based_slam/DeepNetPorting/include")
include_directories("/home/navid/eufs_ws/slam_module/src/graph_based_slam/DeepNetPorting/include/")
#---------------------


# needed to avoid a compilation error (g2o doesn't find Eigen/Core)
INCLUDE_DIRECTORIES(/usr/include/eigen3)

INCLUDE_DIRECTORIES(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS} )
#link_directories(${G2O_LIBRARY_DIRS})

# needed to avoid a compilation error (g2o doesn't find cs.h)
INCLUDE_DIRECTORIES(/usr/include/suitesparse)

include_directories(
  include
  ${TORCH_INCLUDE_DIRECTORIES}

)

#SET(g2o_root /home/navid/g2o/)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(g2o REQUIRED)
find_package(g2o REQUIRED COMPONENTS
g2o::core
g2o::types_slam2d
g2o::solver_eigen
g2o::solver_csparse
)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED)
find_package(geodesy REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(can_msgs REQUIRED)
# find_package(Torch REQUIRED)
# find_package(CUDA REQUIRED)
find_package(bumblebee2_ros_driver REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(kvaser_reader_driver REQUIRED)
find_package(graph_based_slam REQUIRED)
find_package(builtin_interfaces)


#-----------------------------------
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
# set(TensorRT_DIR "/usr/src/tensorrt")
# #find_package(TensorRT REQUIRED)
# #find_package(torchtrt REQUIRED)
# set(TorchTensorRT_INCLUDE_DIR "~/torch_tensorrt/include")
# set(TorchTensorRT_LIBS "~/torch_tensorrt/lib/libtorchtrt.so;~/torch_tensorrt/lib/libtorchtrt_plugins.so;~/torch_tensorrt/lib/libtorchtrt_runtime.so")
#-------------------------------

# find_library(RT_LIB rt REQUIRED)
# find_library(yolo_lib REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_library(graph_handling SHARED src/GraphHandling.cpp)
ament_target_dependencies(graph_handling rclcpp g2o)
target_link_libraries(graph_handling g2o::core g2o::types_slam2d g2o::solver_eigen g2o::solver_csparse)

add_library(node_data SHARED src/NodeData.cpp)
ament_target_dependencies(node_data rclcpp g2o geometry_msgs)
target_link_libraries(node_data g2o::core g2o::types_slam2d)

add_library(cone_data SHARED src/ConeData.cpp)
ament_target_dependencies(cone_data rclcpp g2o geometry_msgs)
target_link_libraries(cone_data g2o::core g2o::types_slam2d node_data)

add_library(pose_data SHARED src/PoseData.cpp)
ament_target_dependencies(pose_data rclcpp g2o geometry_msgs)
target_link_libraries(pose_data g2o::core g2o::types_slam2d node_data)

add_library(ks_kinematics SHARED src/ks_kinematics.cpp)
ament_target_dependencies(ks_kinematics rclcpp)
#target_link_libraries(ks_kinematics g2o::core g2o::types_slam2d g2o::solver_eigen)

add_library(utilities SHARED src/utilities.cpp)
ament_target_dependencies(utilities rclcpp)

# add_library(can_functions SHARED src/can_functions.cpp)
#ament_target_dependencies(can_functions rclcpp)
#target_link_libraries(can_functions g2o::core g2o::types_slam2d node_data)


# add executables


add_executable(graph_SLAM src/GraphSLAM.cpp)
ament_target_dependencies(graph_SLAM rclcpp rclcpp_components std_msgs eufs_msgs nav_msgs visualization_msgs geometry_msgs tf2 tf2_ros tf2_msgs tf2_sensor_msgs tf2_geometry_msgs ackermann_msgs)
target_link_libraries(graph_SLAM graph_handling node_data cone_data pose_data ks_kinematics utilities ${G2O_SOLVER_CSPARSE})

add_executable(lidar_detection src/lidar_detection.cpp)
ament_target_dependencies(lidar_detection rclcpp std_msgs nav_msgs eufs_msgs geometry_msgs visualization_msgs pcl_ros tf2 tf2_ros tf2_msgs tf2_sensor_msgs tf2_geometry_msgs)
target_link_libraries(lidar_detection ${PCL_LIBRARIES})

add_executable(deskew_lidar src/deskew_lidar.cpp)
ament_target_dependencies(deskew_lidar rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs pcl_ros tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV)
target_link_libraries(deskew_lidar ${PCL_LIBRARIES})

add_executable(graph_visualizer src/GraphVisualizer.cpp)
ament_target_dependencies(graph_visualizer rclcpp std_msgs visualization_msgs)

add_executable(convert_GPS src/convert_GPS.cpp)
ament_target_dependencies(convert_GPS rclcpp rclcpp_components std_msgs sensor_msgs geometry_msgs geodesy geographic_msgs)

add_executable(incremental_graph_SLAM src/incrementalGraphSLAM.cpp)
ament_target_dependencies(incremental_graph_SLAM rclcpp rclcpp_components std_msgs eufs_msgs nav_msgs visualization_msgs geometry_msgs tf2 tf2_ros tf2_msgs tf2_sensor_msgs tf2_geometry_msgs ackermann_msgs)
target_link_libraries(incremental_graph_SLAM graph_handling node_data cone_data pose_data ks_kinematics utilities ${G2O_SOLVER_CSPARSE} )

add_executable(odometry_estimation src/odometry_estimation.cpp)
ament_target_dependencies(odometry_estimation rclcpp std_msgs tf2_eigen sensor_msgs kvaser_reader_driver graph_based_slam eufs_msgs builtin_interfaces visualization_msgs)

# add_executable(kvaser_reader src/kvaser_reader.cpp)
# ament_target_dependencies(kvaser_reader rclcpp rclcpp_components can_msgs std_msgs)
# target_link_libraries(kvaser_reader canlib can_functions ${RT_LIB})

# add_executable(kvaser_parser src/kvaser_parser.cpp)
# target_link_libraries(kvaser_parser canlib can_functions ${RT_LIB})
# ament_target_dependencies(kvaser_parser rclcpp std_msgs can_msgs)

####################################################################################
# add_executable(stereo_matching src/opencv_stereo_matching.cpp)
# ament_target_dependencies(stereo_matching rclcpp std_msgs OpenCV)
####################################################################################

# add_executable(yolo_detector src/yolo_cones_detection.cpp)
# target_include_directories(yolo_detector PUBLIC 
# ${TorchTensorRT_INCLUDE_DIR})
# target_link_libraries(yolo_detector ${TORCH_LIBRARIES})
# target_link_libraries(yolo_detector ~/torch_tensorrt/lib/libtorchtrt_runtime.so)
# target_link_libraries(yolo_detector ~/libtorch/lib/libtorch_cuda.so)
# target_link_libraries(yolo_detector 
#  ~/Documenti/thesis/FSD_SLAM_navigation/eufs_sim_folder/src/slam_module/src/graph_based_slam/DeepNetPorting/build/libyolo_lib.so
# ~/libtorch/lib/libtorch.so
# ~/libtorch/lib/libtorch_cpu.so
# #~/libtorch/lib/libtorch_cuda.so
# ~/libtorch/lib/libtorch_global_deps.so
# ~/libtorch/lib/libbackend_with_compiler.so
# ~/libtorch/lib/libc10.so
# ~/libtorch/lib/libc10_cuda.so
# )
# ament_target_dependencies(yolo_detector rclcpp std_msgs OpenCV Torch bumblebee2_ros_driver)



add_executable(extract_gt_cones src/gt_cones.cpp)
ament_target_dependencies(extract_gt_cones rclcpp OpenCV)


ament_export_libraries(graph_handling)
ament_export_libraries(node_data)
ament_export_libraries(cone_data)
ament_export_libraries(pose_data)
ament_export_libraries(ks_kinematics)
ament_export_libraries(utilities)
# ament_export_libraries(can_functions)


install(TARGETS
  graph_handling
  node_data
  cone_data
  pose_data
  ks_kinematics
  utilities
  # can_functions
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  graph_SLAM
  incremental_graph_SLAM
  lidar_detection
  deskew_lidar
  convert_GPS
  graph_visualizer
  # kvaser_reader
  # kvaser_parser
  # yolo_detector
  # stereo_matching
  extract_gt_cones
  odometry_estimation
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# install(DIRECTORY ~/Documenti/thesis/FSD_SLAM_navigation/eufs_sim_folder/src/slam_module/src/graph_based_slam/DeepNetPorting/include/
#   DESTINATION include
# )

install(
  DIRECTORY launch
   DESTINATION share/${PROJECT_NAME}
)

# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}
# )


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GraphEdge.msg"
  "msg/GraphNode.msg"
  "msg/PoseGraph.msg"
  "msg/UtmGPS.msg"
  "msg/WheelSpeeds.msg"
  # "msg/CanData.msg"
  DEPENDENCIES std_msgs
)

rosidl_target_interfaces(graph_visualizer
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(graph_SLAM
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(convert_GPS
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(incremental_graph_SLAM
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# rosidl_target_interfaces(kvaser_reader
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")

# rosidl_target_interfaces(kvaser_parser 
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")


# check if this is correct
# target_include_directories(graph_SLAM
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
#ament_export_include_directories("~/Documenti/thesis/FSD_SLAM_navigation/eufs_sim_folder/src/slam_module/src/graph_based_slam/DeepNetPorting/include/")
ament_export_dependencies(rosidl_default_runtime)


ament_package()
