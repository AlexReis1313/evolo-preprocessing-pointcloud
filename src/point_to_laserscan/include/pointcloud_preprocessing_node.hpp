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
 * Author: Alexandre Reis. Uses code from Paul Bovbel 
 */

#ifndef POINTCLOUD_PREPROCESSING_NODE_HPP_
#define POINTCLOUD_PREPROCESSING_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "message_filters/subscriber.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"


#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/node.hpp"  
#include "rclcpp/parameter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "visibility_control.h"
#include "laser_geometry/laser_geometry.hpp"
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <deque>
#include <cmath>


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
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "filtering_params.hpp"

namespace pointcloud_preprocessing
{
typedef tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2> MessageFilter;

/**
 * Class to process incoming pointclouds into laserscans.
 * Some initial code was pulled from the defunct turtlebot pointcloud_to_laserscan implementation.
 */
class PointCloudPreProcessingNode : public rclcpp::Node
{
public:
POINTCLOUD_TO_LASERSCAN_PUBLIC
explicit PointCloudPreProcessingNode(const rclcpp::NodeOptions & options);

~PointCloudPreProcessingNode() override;

private:

struct TimedCloud { //this is needed because pcl clouds do not save the time stamp form the ros msg
      rclcpp::Time timestamp;
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
      double counter; //it is a double because it will be compared to a double

TimedCloud(const rclcpp::Time& ts, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cld)
    : timestamp(ts), cloud(cld) {
      counter=0.0;
    }
};

std::deque<TimedCloud> clouds_queu_laserscan_;
std::deque<TimedCloud> clouds_queu_projectedPc_;

void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg);

void subscriptionListenerThreadLoop();
sensor_msgs::msg::LaserScan::UniquePtr mergeLaserScans(const sensor_msgs::msg::LaserScan::UniquePtr & short_scan, sensor_msgs::msg::LaserScan::UniquePtr && long_scan);
sensor_msgs::msg::LaserScan::UniquePtr computeLaserScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg);
std::pair<sensor_msgs::msg::LaserScan::UniquePtr, sensor_msgs::msg::LaserScan::UniquePtr> LaserScan2radialMap(const sensor_msgs::msg::LaserScan::UniquePtr &scan);
void filterCloud( pcl::PointCloud<pcl::PointXYZI> & cloud_in, const double & min_height,pcl::PointCloud<pcl::PointXYZI> & cloud_out,pcl::PointCloud<pcl::PointXYZI> & cloud_outREJ);
std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr , pcl::PointCloud<pcl::PointXYZI>::Ptr>  adaptiveRadiusFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,float scale = 0.0125f, float min_radius = 0.05f,            // Radius = scale * range
                    int min_neighbors = 3);
bool detectWaterPlane( const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out);

void project(pcl::PointCloud<pcl::PointXYZI> & cloud_in, pcl::PointCloud<pcl::PointXYZI> & cloud_out_projected);
void accumulate(
  const pcl::PointCloud<pcl::PointXYZI> & cloud_in,
  pcl::PointCloud<pcl::PointXYZI> & cloud_out_accumulated,
  const rclcpp::Time & currentTime,
  const double & time_decay,geometry_msgs::msg::TransformStamped & world_fix_transform,
  geometry_msgs::msg::TransformStamped & inverse_world_fix_transform, std::deque<TimedCloud> cloud_queue);

void detectWaterPlane(
  const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
  pcl::PointCloud<pcl::PointXYZI>::Ptr &plane_inliers_output,
  Eigen::Vector4f &plane_equation_output);

std::unique_ptr<tf2_ros::Buffer> tf2_;
std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> pub_;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> pub_radialmap_;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> pub_radialmapVisual_;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pc_;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_FilteredPC_;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_projectedPC_;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_accumulatedPC_laserscan_;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_projected_AccumulatedPC_;
std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>>  marker_pub_;
void publish_plane_marker(float a, float b, float c, float d);
bool checkWaterPlane(float a, float b, float c, float d, float threshold_degrees);

rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

laser_geometry::LaserProjection Laser2PCprojector_;
std::ofstream timeoutFile_;


std::unique_ptr<MessageFilter> message_filter_;
std::thread subscription_listener_thread_;
std::atomic_bool alive_{true};

// ROS Parameters
std::unique_ptr<PointCloudPreProcessingParams> params_;
rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  
};

}  
#endif  // POINTCLOUD_PREPROCESSING_NODE_HPP_
