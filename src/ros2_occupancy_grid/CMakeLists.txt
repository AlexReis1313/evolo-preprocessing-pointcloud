cmake_minimum_required(VERSION 3.5)
project(clustering_segmentation)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) # Add this line
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_ros REQUIRED)



add_executable(occupancy_grid src/occupancy_grid_node.cpp
                              src/occupancy_grid.cpp
                              src/euclidean_spatial.cpp
                              )
                              

add_executable(occupancy_grid_mqtt src/mqtt_occupancy_bridge.cpp)


ament_target_dependencies(occupancy_grid rclcpp nav_msgs sensor_msgs Eigen3 tf2_geometry_msgs OpenCV pcl_ros)
ament_target_dependencies(occupancy_grid_mqtt rclcpp nav_msgs sensor_msgs Eigen3 tf2_geometry_msgs OpenCV pcl_ros)

target_include_directories(
  occupancy_grid
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}> $<INSTALL_INTERFACE:include>)

install(TARGETS occupancy_grid DESTINATION lib/${PROJECT_NAME})

target_include_directories(
  occupancy_grid_mqtt
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}> $<INSTALL_INTERFACE:include>)

install(TARGETS occupancy_grid_mqtt DESTINATION lib/${PROJECT_NAME})
ament_package()
