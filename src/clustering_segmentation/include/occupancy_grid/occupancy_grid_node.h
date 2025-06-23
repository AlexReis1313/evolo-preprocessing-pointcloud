#ifndef OCCUPANCYGRIDNODE_H
#define OCCUPANCYGRIDNODE_H

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "occupancy_grid/occupancy_grid.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <tf2/utils.h>
#include "euclidean_spatial.hpp"

                                                          //because we want euclidean cluster.cpp to have the same node and create a publisher
class OccupancyGridNode : public rclcpp::Node{

 public:
  OccupancyGridNode();

 private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_static_map_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_, laser_scan_static_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  void handleOdom(const nav_msgs::msg::Odometry::SharedPtr odom);
  void handleLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan);
  void handleStaticLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan);

  void lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);
  void paramLaunch();

  std::vector<Point2d<double>> convertPolarScantoCartesianScan(
      const sensor_msgs::msg::LaserScan::SharedPtr laser_scan);
  nav_msgs::msg::Odometry prev_odom_{};
  std::unique_ptr<OccupancyGrid> grid_map_, grid_map_static_;
  tf2::Transform robot_pose_inOCGMapFrame;
  EuclideanSpatial clustering;
  //std::unique_ptr<ClusteringMappingParams> params_;
  bool saveTimeMetric_ = false;
  bool timeMetric = true; //verbose the time metrics
  std::string timing_file = "Segmentation_timingFile.csv";
  std::ofstream timeoutFile_;
  bool staticMapping = true; //if it subscribes to staticPoints and builds a occupancy grid map - used just for output
  std::string staticPoints = "static/pointcloud";
  bool clustering_bool = true; //if it subscribes to lidar accumulated point cloud, clusteres them and publishes the clusters as clusteres points
  std::string points_in_topic = "filtered/ls/pointcloud/accumulated";
  std::string points_out_topic = "clustered_points";
  bool DynamicStatic_segmentation=true; //if true, builds map from filtered/ls/pointcloud with slow occupancy gain and uses it to classify clusteres into dynamic and static before publishing them
                                        //if false, builds map from filtered/ls/pointcloud with very fast occupancy
  
};

#endif  // OCCUPANCYGRIDNODE_H