cmake_minimum_required(VERSION 3.5)
project(object_on_lanelet_checker)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(tier4_perception_msgs REQUIRED)
find_package(autoware_auto_mapping_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)

find_package(autoware_auto_perception_msgs REQUIRED)

find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(lanelet2_extension REQUIRED)

# Executable
add_executable(object_on_lanelet_checker_node src/object_on_lanelet_checker.cpp)

ament_target_dependencies(object_on_lanelet_checker_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tier4_perception_msgs
  autoware_auto_mapping_msgs
  autoware_planning_msgs
  autoware_auto_perception_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  tf2_ros
  tf2_geometry_msgs
  lanelet2_extension
)

install(TARGETS
  object_on_lanelet_checker_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

