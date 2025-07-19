# Lidar Perception Pipeline for Hydrofoiling USV - ROS2 Humble
## Introduction
This project is an obstacle detection and tracking
framework designed for a hydrofoiling Unmaned Surface Vehicle (USV), using a LiDAR sensor and a ego-vehicle localization estimate given by an INS.
This work serves as a perception layer for a future obstacle avoidance and path planning implementation.
The project processes raw point cloud data, filters it and represents the obstacles in the environment either with an Occupancy Grid Map or a set of Bounding Boxes with a velocity estimate, or even a combination of both.
The code is integrated with ROS2 Humble, it expects to receive the appropriate Tfs and an input raw point cloud. Outputs are the filtered point cloud, marker arrays and occupancy grid maps, all published with standard ROS2 topics. Througough testing has been made to prove that the algorithm runs in real-time.

A complete  master thesis report of this project can be read here (link still missing). It includes a literature review of related scintific work, a complete explanation and justification of the chosen methods, and an analysis and discussion of results.

### Evolo Hydrofoiling USV:
The project is purposely made for a surface maritime environment, where point clouds often detect water surface reflctions. The test platform was a hydrofoiling USV called Evolo, which often reaches high Roll and Pitch angles, that were also dealt with by this perception pipeline.
<p align="center">
  <img src="media/Evolo_onwater.JPG" width="45%" />
  <img src="media/EvoloComponents.png" width="40%" />
</p>

### Flowchart definining high-level pipeline:
This package preprocesses raw 3D LiDAR data into a filtered 2d representation. It then creates a rolling local occupancy grid map with static obstacles and it performs multi-object tracking with dynamic obstacles. 
<p align="center">
    <img src="media/pipeline.png" width="60%" />
</p>

# Collected Maritime Surface Vehicle data/input
<p align="center">
    <img src="media/collectionPointclouds.png" width="60%" />
</p>

# Outputs:
The package outputs several representations of a filtered point cloud. 
You can choose if this filtered data is used to build a rolling local occupancy grid map, if it gets clustered and tracked with multi-object tracking. There is also an option to segment the clusters with dynamic and static labels, then tracking the dynamic clussters and building the local map with the static ones.
### LiDAR Point Cloud Filtering and Preprocessing
Preprocessing steps:
1. Transformation of the point cloud from the LiDAR sensor's frame to the vehicle's *base footprint* frame.
2. IMU-informed RANSAC to estimate the water surface plane, followed by removal of all points at or below the surface of the water.
3. Dynamic Radius Outlier Filtering.
4. Region-based pass-through filter for points too high up to be considered obstacles.
5. Data reduction from a 3D point cloud to a 2D Laser Scan.
6. Time decay: Temporal accumulation of 2D Laser Scans to construct a more complete 2D point cloud, gathered over a specified time horizon.

<p align="center">
    <img src="media/preprocessing_img_git.png" width="40%" />
</p>

### Rolling Local Occupancy Grid Mapping 
<p align="center">
    <img src="media/localMappingPipeline.png" width="80%" />
</p>

<p align="center">
  <img src="media/mapping_generic.png" width="50%" />
</p>



### Multi-object tracking with Kalman Filter
<p align="center">
    <img src="media/tracking pipeline.png" width="70%" />
</p>

<p align="center">
  <img src="media/tracking second main result.png" width="50%" />
</p>

### Full pipeline with static-dynamic obstacle segmentation

<p align="center">
  <img src="media/SeB_1" width="45%" />
</p>

## ROS2 Integration and System Arquitecture



The implementation follows a **modular design**, where each high-level process is encapsulated within its own **ROS 2 node**. This approach enables:

- Parallel execution of nodes using multi-threading  
- Clear input-output interfaces for each node  
- Easier debugging through well-defined communication over ROS 2 topics  

The full ROS 2 node graph for the proposed pipeline (including static-dynamic segmentation) is shown below:

<p align="center">
  <img src="media/ROSNODES_graph.png" alt="ROS 2 Node Graph" width="70%" />
</p>

> _Node graph illustrating the modular pipeline design with static-dynamic segmentation._

The system is composed of the following main nodes:

- **`/base_footprint_publisher_node`**  
  Uses localization data from the SBG Systems INS to publish the vehicle’s `base_footprint` coordinate frame.

- **`/pointcloud_preprocessing_filtering`**  
  Transforms the raw LiDAR point cloud into the `base_footprint` frame, filters the data, and publishes several representations of the filtered point cloud.

- **`/mapping_and_clustering_node`**  
  Performs clustering on time-accumulated laser scans received via `/filtered/ls/laserscan_accumulated`. It creates a segmentation map from the filtered points (from `/filtered/ls/laserscan`), segments clusters into static or dynamic classes using this map, and routes the data accordingly. It also publishes an occupancy grid map based on the static-classified points. Odometry from the `base_footprint` frame is used to propagate the rolling occupancy maps.

- **`/object_tracking_node`**  
  Handles data association, computes 2D bounding boxes, and manages a Kalman filter for each tracked object.


# Build

    git clone https://github.com/AlexReis1313/evolo-preprocessing-pointcloud.git
    cd /evolo-preprocessing-pointcloud/
    colcon build
    source the workspace
# Run
### Lidar Preprocessing Node - essential
Takes in raw LiDAR Point CLouds in sensor frame (or base_link), filters it and outputs several 2d representations in base_footprint.
Also produces base_footprint

    ros2 launch pointcloud_preprocessing pointcloud_preprocessing_launch_evolo.py
### Clustering and local occupancy grid map

    ros2 run clustering_segmentation clustering_segmentation 
    
### Tracking

    ros2 run bb_dataass_tracking bounding_box_node_main 

### Other commands - not always used

    ros2 launch pointcloud_preprocessing mapping_launch.py
    ros2 launch lidar_cluster euclidean_spatial.launch.py topic:=/filtered/ls/pointcloud/accumulated
    
### Send lightweight occupancy grid map over MQTT
    
    ros2 run occupancy_grid occupancy_grid_mqtt
    ros2 launch str_json_mqtt_bridge waraps_bridge.launch

For alternative vehicle parameters, use one of the following options:

    ros2 launch pointcloud_preprocessing pointcloud_preprocessing_launch_boat.py
    ros2 launch pointcloud_preprocessing pointcloud_preprocessing_launch_sim.py

# Youtube examples of this Package

https://www.youtube.com/playlist?list=PLS_kAMaDCf634gqgQ6XRzQY9NJVkBzv5d

# External used code
### ROS 2 pointcloud_to_laserscan package
This reository uses the pointcloud_to_laserscan ROS 2 package that provides components to convert `sensor_msgs/msg/PointCloud2` messages to `sensor_msgs/msg/LaserScan` messages and back.
Source code can be found here https://github.com/ros-perception/pointcloud_to_laserscan 
### ROS2 ros2_occupancy_grid package
This repository adapted and used the local ocupancy grid map implementation in https://github.com/hiwad-aziz/ros2_occupancy_grid

