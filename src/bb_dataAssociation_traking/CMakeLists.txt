cmake_minimum_required(VERSION 3.5)
project(bb_dataass_tracking)

# Auto finds dependencies declared in package.xml
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Explicitly find OpenCV (you can use OpenCV4 if needed)
find_package(OpenCV REQUIRED)

# Add your library with all sources
ament_auto_add_library(bb_dataass_tracking SHARED 
    src/bounding_box_node.cpp
    src/RotatingCalipers.cpp
    src/kalmanFilter.cpp
    src/Hungarian.cpp
)

# Link OpenCV to the library
target_include_directories(bb_dataass_tracking PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(bb_dataass_tracking ${OpenCV_LIBS})

# Register the component
rclcpp_components_register_node(bb_dataass_tracking
  PLUGIN "bb_dataass_tracking::BoundingBoxNode"
  EXECUTABLE bounding_box_node)

# Add the main executable
ament_auto_add_executable(bounding_box_node_main src/main.cpp)

# Link OpenCV to the executable if needed
target_include_directories(bounding_box_node_main PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(bounding_box_node_main ${OpenCV_LIBS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
