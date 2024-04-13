cmake_minimum_required(VERSION 3.8)
project(kvaser_reader_driver)

#set(CMAKE_BUILD_TYPE "RELEASE")
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

# include folder
include_directories(
  include
)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)


find_library(RT_LIB rt REQUIRED)


# add link_libraries
add_library(can_functions SHARED src/can_functions.cpp)

# add executables
add_executable(kvaser_reader src/kvaser_reader.cpp)
target_link_libraries(kvaser_reader canlib can_functions ${RT_LIB})
ament_target_dependencies(kvaser_reader rclcpp std_msgs can_msgs)

# add executables
add_executable(kvaser_parser src/kvaser_parser.cpp)
target_link_libraries(kvaser_parser canlib can_functions ${RT_LIB})
ament_target_dependencies(kvaser_parser rclcpp std_msgs can_msgs)


ament_export_libraries(can_functions)

install(TARGETS
  can_functions
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
  kvaser_reader
  kvaser_parser
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CanData.msg"
  "msg/CarInfo.msg"
  "msg/AmkVelocity.msg"
  DEPENDENCIES std_msgs
)


rosidl_target_interfaces(kvaser_reader ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(kvaser_parser ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
