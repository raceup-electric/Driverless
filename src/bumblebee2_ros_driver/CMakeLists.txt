cmake_minimum_required(VERSION 3.8)
project(bumblebee2_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_subdirectory(bumblebee2_driver)

# find dependencies

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME} "msg/StereoImage.msg" DEPENDENCIES std_msgs )
#rosidl_generate_interfaces(${PROJECT_NAME}_msg "msg/StereoImage.msg" LIBRARY_NAME ${PROJECT_NAME} DEPENDENCIES std_msgs )

add_executable(bumblebee2 src/bumblebee2.cpp)
target_link_libraries(bumblebee2 bumblebee2_driver)
rosidl_target_interfaces(bumblebee2 ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(bumblebee2 rclcpp std_msgs)

add_executable(test_bumblebee2 src/test_bumblebee2.cpp)
target_link_libraries(test_bumblebee2 bumblebee2_driver)
rosidl_target_interfaces(test_bumblebee2 ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(test_bumblebee2 rclcpp std_msgs sensor_msgs cv_bridge)

target_compile_features(bumblebee2 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(test_bumblebee2 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  bumblebee2
  test_bumblebee2
  DESTINATION lib/${PROJECT_NAME})

ament_package()
