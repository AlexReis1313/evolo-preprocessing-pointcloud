cmake_minimum_required(VERSION 3.5)
project(pointcloud_to_laserscan)

# Automatically find and add dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Find any specific additional dependencies not automatically found
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)

# Node 1: Shared library for pointcloud_to_laserscan
ament_auto_add_library(pointcloud_to_laserscan SHARED
  src/pointcloud_to_laserscan_node.cpp
  src/filtering_params.cpp)

rclcpp_components_register_node(pointcloud_to_laserscan
  PLUGIN "pointcloud_to_laserscan::PointCloudToLaserScanNode"
  EXECUTABLE pointcloud_to_laserscan_node)

# Node 2: Standalone executable for base_footprint_publisher
ament_auto_add_executable(base_footprint_publisher
  src/base_footprint_publisher.cpp)

# Install launch folder
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Testing support
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
