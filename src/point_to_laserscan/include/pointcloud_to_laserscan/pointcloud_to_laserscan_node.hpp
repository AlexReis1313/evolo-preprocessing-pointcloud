/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

 #ifndef POINTCLOUD_TO_LASERSCAN__POINTCLOUD_TO_LASERSCAN_NODE_HPP_
 #define POINTCLOUD_TO_LASERSCAN__POINTCLOUD_TO_LASERSCAN_NODE_HPP_
 
 #include <atomic>
 #include <memory>
 #include <string>
 #include <thread>
 
 #include "message_filters/subscriber.h"
 #include "tf2_ros/buffer.h"
 #include "tf2_ros/message_filter.h"
 #include "tf2_ros/transform_listener.h"
 #include "tf2_ros/transform_broadcaster.h"
 
 
 
 #include "rclcpp/rclcpp.hpp"
 #include "sensor_msgs/msg/laser_scan.hpp"
 #include "sensor_msgs/msg/point_cloud2.hpp"
 
 #include "pointcloud_to_laserscan/visibility_control.h"
 #include "laser_geometry/laser_geometry.hpp"
 #include <chrono>
 #include <functional>
 #include <limits>
 #include <memory>
 #include <string>
 #include <thread>
 #include <utility>
 
 #include "sensor_msgs/point_cloud2_iterator.hpp"
 #include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
 #include "tf2_ros/create_timer_ros.h"
 #include "tf2_ros/transform_broadcaster.h"
 #include <tf2/LinearMath/Quaternion.h>
 #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
 #include "pcl_ros/transforms.hpp"
 #include <pcl_conversions/pcl_conversions.h>
 #include <pcl/filters/radius_outlier_removal.h>
 #include <pcl/filters/conditional_removal.h>
 #include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


 namespace pointcloud_to_laserscan
 {
 typedef tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2> MessageFilter;
 
 /**
 * Class to process incoming pointclouds into laserscans.
 * Some initial code was pulled from the defunct turtlebot pointcloud_to_laserscan implementation.
 */
 class PointCloudToLaserScanNode : public rclcpp::Node
 {
 public:
   POINTCLOUD_TO_LASERSCAN_PUBLIC
   explicit PointCloudToLaserScanNode(const rclcpp::NodeOptions & options);
 
   ~PointCloudToLaserScanNode() override;
 
 private:
   void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg);
 
   void subscriptionListenerThreadLoop();
   sensor_msgs::msg::LaserScan::UniquePtr mergeLaserScans(const sensor_msgs::msg::LaserScan::UniquePtr & short_scan,
     sensor_msgs::msg::LaserScan::UniquePtr && long_scan);
   sensor_msgs::msg::LaserScan::UniquePtr computeLaserScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
     double min_range,
     double max_range,
     double min_height,
     double max_height,
     double angle_min,
     double angle_max,
     double angle_increment,
     double scan_time,
     double inf_epsilon,
     bool use_inf,
     std::string target_frame);
     std::pair<sensor_msgs::msg::LaserScan::UniquePtr, sensor_msgs::msg::LaserScan::UniquePtr> 
        LaserScan2radialMap(const sensor_msgs::msg::LaserScan::UniquePtr &scan, 
                    double angle_min, double angle_max, double angle_increment);
    void filterCloud( pcl::PointCloud<pcl::PointXYZI> & cloud_in,
                    const double & min_height,
                    pcl::PointCloud<pcl::PointXYZI> & cloud_out);
    pcl::PointCloud<pcl::PointXYZI>::Ptr  adaptiveRadiusFilter(
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                      float scale = 0.0125f, float min_radius = 0.05f,             // Radius = scale * range
                      int min_neighbors = 3);
 
   std::unique_ptr<tf2_ros::Buffer> tf2_;
   std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
   std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
   message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_;
   std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> pub_;
   std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> pub_radialmap_;
   std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> pub_radialmapVisual_;
   std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pc_;
   std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_OriginalPC_;


   laser_geometry::LaserProjection Laser2PCprojector_;


   //std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> pub_short_;
   //std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> pub_long_;
 
   std::unique_ptr<MessageFilter> message_filter_;
 
   std::thread subscription_listener_thread_;
   std::atomic_bool alive_{true};
 
   // ROS Parameters
   int input_queue_size_,nr_neighbours_;
   std::string target_frame_, fixed_frame_, cloud_frame_,baseLink_frame_;
   double tolerance_;
   double angle_visual_outputmap_, min_height_shortrange_, max_height_shortrange_, min_height_longrange_, max_height_longrange_, angle_min_, angle_max_,angle_min_map_, angle_max_map_, angle_increment_, scan_time_, range_min_,
   range_transition_, range_max_, inf_epsilon_,b_neighboursRadius_, m_neighboursRadius_;
   bool use_inf_;

   
   
 };
 
 }  // namespace pointcloud_to_laserscan
 
 #endif  // POINTCLOUD_TO_LASERSCAN__POINTCLOUD_TO_LASERSCAN_NODE_HPP_
 