#ifndef MQTTOCCUPANCYBRIDGE_H
#define MQTTOCCUPANCYBRIDGE_H

#include "std_msgs/msg/string.hpp"
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
#include "sensor_msgs/point_cloud2_iterator.hpp"
//#include "sensor_msgs/point_cloud2_modifier.hpp"

class MqttOccupancyBridgeNode : public rclcpp::Node{

 public:
  MqttOccupancyBridgeNode();

 private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_json_;

  
  void mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr input_msg);

  std::string map_in_topic = "local_costmap/costmap";
  std::string points_out_topic = "Evolo/mqtt/map";
  std::string string_out_topic = "sensor/pointcloud";

  double throttle_hz_ = 2.0;
  rclcpp::Time prev_time;
  bool first_time_; 
};

#endif  // MQTTOCCUPANCYBRIDGE_H