cmake_minimum_required(VERSION 3.8)
project(xsens_mti_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_subdirectory(xsens_mti_driver)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(xsens_mti src/xsens_mti.cpp)
target_link_libraries(xsens_mti xsens_mti_driver)
ament_target_dependencies(xsens_mti rclcpp sensor_msgs)

target_compile_features(xsens_mti PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
#target_compile_features(xsens_mti_sample_listener PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  xsens_mti
#  xsens_mti_sample_listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
