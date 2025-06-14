cmake_minimum_required(VERSION 3.5)
project(pointcloud_preprocessing)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


ament_auto_add_library(pointcloud_preprocessing SHARED
  src/pointcloud_preprocessing_node.cpp
  src/filtering_params.cpp)

rclcpp_components_register_node(pointcloud_preprocessing
  PLUGIN "pointcloud_preprocessing::PointCloudPreProcessingNode"
  EXECUTABLE pointcloud_preprocessing_node)



ament_auto_add_executable(base_footprint_publisher
  src/base_footprint_publisher.cpp)

# Install launch folder
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
#intall config folder
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
