cmake_minimum_required(VERSION 3.5)
project(bb_dataass_tracking)

find_package(ament_cmake_auto REQUIRED)
find_package(pcl_ros REQUIRED)
#find_package(laser_geometry REQUIRED)



ament_auto_find_build_dependencies()

ament_auto_add_library(bb_dataass_tracking SHARED 
                  src/bounding_box_node.cpp
                  src/RotatingCalipers.cpp
                  src/kalmanFilter.cpp
                  src/Hungarian.cpp)

rclcpp_components_register_node(bb_dataass_tracking
  PLUGIN "bb_dataass_tracking::BoundingBoxNode"
  EXECUTABLE bounding_box_node)  # ✅ Make sure the executable is created

ament_auto_add_executable(bounding_box_node_main src/main.cpp)  # ✅ Add main.cpp as an executable


#ament_target_dependencies(pointcloud_to_laserscan_node laser_geometry)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)