// Non-grid (spatial) euclidean cluster filter for point cloud data
// based on https://github.com/autowarefoundation/autoware.universe/blob/main/perception/euclidean_cluster/lib/euclidean_cluster.cpp
// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "euclidean_spatial.hpp"


  
EuclideanSpatial::EuclideanSpatial(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar)
    : pub_lidar_(pub_lidar) {
    


  }

void EuclideanSpatial::lidarAndMapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg, std::unique_ptr<OccupancyGrid> & grid_map_, tf2::Transform & robot_pose_inOCGMapFrame){

    // Convert to PCL data type
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_nonfiltered(new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>); // not PointXYZI
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_msg, *pointcloud_nonfiltered);
    std::vector<pcl::PointCloud<pcl::PointXYZI>> clusters;

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pointcloud_nonfiltered, *pointcloud, indices);
    pointcloud_ptr = pointcloud;

    
    // create tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(pointcloud_ptr);
    // clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> pcl_euclidean_cluster;
    pcl_euclidean_cluster.setClusterTolerance(tolerance_);
    pcl_euclidean_cluster.setMinClusterSize(min_cluster_size_);
    pcl_euclidean_cluster.setMaxClusterSize(max_cluster_size_);
    pcl_euclidean_cluster.setSearchMethod(tree);
    pcl_euclidean_cluster.setInputCloud(pointcloud_ptr);
    pcl_euclidean_cluster.extract(cluster_indices);


    pcl::PointIndices::Ptr clustered_indices(new pcl::PointIndices);
    // get number of clusters
  


    // build output
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    int intensity = 1; //intensity zero is for clusteres that will be part of the static map
    float total_counter;
    float occ_counter;
     //bool dynamic_cluster =true;
    int cluster_intensity;
    for (const auto &cluster : cluster_indices)
    {
      occ_counter =0.0;
      total_counter = 0.0;
      for (const auto &point_idx : cluster.indices)
      {
        Point2d<double> point = {pointcloud->points[point_idx].x, pointcloud->points[point_idx].y};
        if(grid_map_->checkOccupancy(point, robot_pose_inOCGMapFrame)){
          occ_counter +=1.0;
        }
        total_counter+=1.0;
      }
      double fraction = occ_counter/total_counter;
      if ( fraction < 0.5 ){//less than 70% of the points are within occupied part of the map
         //dynamic_cluster= true; //cluster is dynamic, will be tracked
        intensity++;
        cluster_intensity=intensity;
      }else{
         //dynamic_cluster=false; //cluster is static - will be mapped in an occupancy grid map
        cluster_intensity = 0;
      }
      //std::cout << " this cluster is dynamic? "<< dynamic_cluster << "with counters (%, occ, total)" << fraction << " occ:"<<occ_counter << " total:"<<total_counter << std::endl;

      for (const auto &point_idx : cluster.indices)
      {
        // convert pointcloud->points[point_idx] to PointXYZI
        pcl::PointXYZI pxyzi;
        pxyzi.x = pointcloud->points[point_idx].x;
        pxyzi.y = pointcloud->points[point_idx].y;
        pxyzi.z = pointcloud->points[point_idx].z;
        pxyzi.intensity = cluster_intensity;
        cloud_cluster->points.push_back(pxyzi);
        clustered_indices->indices.push_back(point_idx);

      }
      clusters.push_back(*cloud_cluster);
      clusters.back().width = cloud_cluster->points.size();
      clusters.back().height = 1;
      clusters.back().is_dense = false;

 
    }

    // 2. Use pcl::ExtractIndices to extract the *non-clustered* points
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(pointcloud_ptr);
    extract.setIndices(clustered_indices);
    extract.setNegative(true); // This is key: get the points *not* in any cluster

    pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*outlier_cloud);
    // 3. Append outliers with intensity 0 to cloud_cluster
    for (const auto& pt : outlier_cloud->points) {
        pcl::PointXYZI pxyzi;
        pxyzi.x = pt.x;
        pxyzi.y = pt.y;
        pxyzi.z = pt.z;
        pxyzi.intensity = 0;
        cloud_cluster->points.push_back(pxyzi);
    }

    
  
  cloud_cluster->width = cloud_cluster->points.size();
  cloud_cluster->height = 1;
  cloud_cluster->is_dense = true;

  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*cloud_cluster, output_msg);
  // Add the same frame_id as the input, it is not included in pcl PointXYZ
  output_msg.header.frame_id = input_msg->header.frame_id;
  output_msg.header.stamp = input_msg->header.stamp;

  // Publish the data as a ROS message
  pub_lidar_->publish(output_msg);
  }


