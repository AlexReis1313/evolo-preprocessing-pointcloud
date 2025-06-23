#ifndef EUCLIDEANSPATIAL_H
#define EUCLIDEANSPATIAL_H

// Non-grid (spatial) euclidean cluster filter for point cloud data
// based on https://github.com/autowarefoundation/autoware.universe/blob/main/perception/euclidean_cluster/lib/euclidean_cluster.cpp
// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include <unordered_set>
#include <pcl/filters/extract_indices.h>
#include "occupancy_grid/occupancy_grid.h"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <memory> // For std::unique_ptr and std::enable_shared_from_this


using namespace std::chrono_literals;
using std::placeholders::_1;


class EuclideanSpatial {
public:
    explicit EuclideanSpatial(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar); 
    void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);
    void lidarAndMapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg, std::unique_ptr<OccupancyGrid> & grid_map_, tf2::Transform & robot_pose_inOCGMapFrame, bool & DynamicStatic_segmentation);

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_;

  rclcpp::Node::SharedPtr node_;

  float tolerance_ = 4.0;
  int min_cluster_size_ = 4;
  int max_cluster_size_ = 2500;
  bool use_height_ = false;
  bool verbose1 = false, verbose2 = false;

};

#endif // EUCLIDEANSPATIAL_H