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

 #include "pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp"

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

 
namespace pointcloud_to_laserscan
{

PointCloudToLaserScanNode::PointCloudToLaserScanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_to_laserscan", options)
{
  target_frame_ = this->declare_parameter("target_frame", "");
  fixed_frame_ =this->declare_parameter("fixed_frame", "");
  cloud_frame_=this->declare_parameter("cloud_frame", "");
  tolerance_ = this->declare_parameter("transform_tolerance", 0.01);
  // TODO(hidmic): adjust default input queue size based on actual concurrency levels
  // achievable by the associated executor
  input_queue_size_ = this->declare_parameter(
    "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
  min_height_shortrange_ = this->declare_parameter("min_height_shortrange", std::numeric_limits<double>::min());
  max_height_shortrange_ = this->declare_parameter("max_height_shortrange", std::numeric_limits<double>::max());
  angle_min_ = this->declare_parameter("angle_min", -M_PI);
  angle_max_ = this->declare_parameter("angle_max", M_PI);
  angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
  scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0);
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
  use_inf_ = this->declare_parameter("use_inf", true);
  min_height_longrange_ = this->declare_parameter("min_height_longrange", std::numeric_limits<double>::min());
  max_height_longrange_ = this->declare_parameter("max_height_longrange", std::numeric_limits<double>::max());
  range_transition_ = this->declare_parameter("range_transition", 0.0);

  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
  pub_short_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scanner/scan/short", rclcpp::SensorDataQoS());
  pub_long_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scanner/scan/long", rclcpp::SensorDataQoS());

  vgicpRegistrationClass vgicpRegistration_;

  using std::placeholders::_1;
  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty()) {
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
    message_filter_ = std::make_unique<MessageFilter>(
      sub_, *tf2_, cloud_frame_, input_queue_size_,
      this->get_node_logging_interface(),
      this->get_node_clock_interface());
    message_filter_->registerCallback(
      std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  } else {  // otherwise setup direct subscription
    sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  }

  subscription_listener_thread_ = std::thread(
    std::bind(&PointCloudToLaserScanNode::subscriptionListenerThreadLoop, this));


  }

PointCloudToLaserScanNode::~PointCloudToLaserScanNode()
{
  alive_.store(false);
  subscription_listener_thread_.join();
}

void PointCloudToLaserScanNode::subscriptionListenerThreadLoop()
{
  rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();

  const std::chrono::milliseconds timeout(100);
  while (rclcpp::ok(context) && alive_.load()) {
    int subscription_count = pub_->get_subscription_count() +
      pub_->get_intra_process_subscription_count()+pub_short_->get_subscription_count() +
      pub_short_->get_intra_process_subscription_count() + pub_long_->get_subscription_count() +
      pub_long_->get_intra_process_subscription_count();
    if (subscription_count > 0) {
      if (!sub_.getSubscriber()) {
        RCLCPP_INFO(
          this->get_logger(),
          "Got a subscriber to laserscan, starting pointcloud subscriber");
        rclcpp::SensorDataQoS qos;
        qos.keep_last(input_queue_size_);
        sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile());
      }
    } else if (sub_.getSubscriber()) {
      RCLCPP_INFO(
        this->get_logger(),
        "No subscribers to laserscan, shutting down pointcloud subscriber");
      sub_.unsubscribe();
    }
    rclcpp::Event::SharedPtr event = this->get_graph_event();
    this->wait_for_graph_change(event, timeout);
  }
  sub_.unsubscribe();
}

void PointCloudToLaserScanNode::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  

  // Transform cloud if necessary
  //if (target_frame_ != cloud_msg->header.frame_id) {
    try {
        
      geometry_msgs::msg::TransformStamped transform_stamped, noattitude_transform;
  
  
      transform_stamped = tf2_->lookupTransform(fixed_frame_, cloud_msg->header.frame_id, tf2::TimePointZero);
      // adavanced version transform_stamped = tf2_->lookupTransform(fixed_frame_, cloud_msg->header.frame_id, tf2::TimePointZero);
      tf2::Quaternion q_orig, q_new;
      tf2::convert(transform_stamped.transform.rotation, q_orig);
  
      double roll, pitch, yaw;
      tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);
  
      q_new.setRPY(0, 0, yaw);
  
      noattitude_transform.header.stamp = cloud_msg->header.stamp;
      noattitude_transform.header.frame_id = fixed_frame_;
      noattitude_transform.child_frame_id = target_frame_;
      noattitude_transform.transform.translation.x = transform_stamped.transform.translation.x;
      noattitude_transform.transform.translation.y = transform_stamped.transform.translation.y;
      noattitude_transform.transform.translation.z = transform_stamped.transform.translation.z;
      noattitude_transform.transform.rotation = tf2::toMsg(q_new);
  
      tf_broadcaster_->sendTransform(noattitude_transform);
      noattitude_transform.transform.translation.x =-transform_stamped.transform.translation.x;//changed, era 0
      noattitude_transform.transform.translation.y =-transform_stamped.transform.translation.y;//changed
      noattitude_transform.transform.translation.z =0;
      q_new.setRPY(roll, pitch, 0);
      noattitude_transform.transform.rotation = tf2::toMsg(q_new);
        
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud, pcl_cloud_transformed, pcl_cloud_transformed2;
      pcl::fromROSMsg(*cloud_msg, pcl_cloud);
  
      pcl_ros::transformPointCloud(pcl_cloud, pcl_cloud_transformed, noattitude_transform);
  
       //ICP cloud
      if (vgicpRegistration_.firstTime_){
        vgicpRegistration_.setLastCloud(pcl_cloud_transformed);
      } else{
        vgicpRegistration_.swapNewLastCloud();
        vgicpRegistration_.setNewCloud(pcl_cloud_transformed);
        vgicpRegistration_.computeRegistration();
        pcl_cloud_transformed= vgicpRegistration_.getNewTransformedCloud();
      }
  
      /*  //new
      noattitude_transform.transform.translation.x =transform_stamped.transform.translation.x;//changed, era 0
      noattitude_transform.transform.translation.y =transform_stamped.transform.translation.y;//changed
      noattitude_transform.transform.translation.z =0;
      q_new.setRPY(0, 0, 0);
      noattitude_transform.transform.rotation = tf2::toMsg(q_new);
      pcl_ros::transformPointCloud(pcl_cloud_transformed, pcl_cloud_transformed2, noattitude_transform);
      */
      //stop new
      auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(pcl_cloud_transformed, *cloud); //should be non2 version
      cloud_msg = cloud;
    
  
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
      return;
    }
    //}
  
   

 
  auto short_scan_msg = PointCloudToLaserScanNode::computeLaserScan(
    cloud_msg, range_min_, range_transition_, min_height_shortrange_, max_height_shortrange_,
    angle_min_, angle_max_, angle_increment_, scan_time_, inf_epsilon_, use_inf_, target_frame_);
  auto long_scan_msg = PointCloudToLaserScanNode::computeLaserScan(
    cloud_msg, range_transition_, range_max_, min_height_longrange_, max_height_longrange_,
      angle_min_, angle_max_, angle_increment_, scan_time_, inf_epsilon_, use_inf_,target_frame_);
  
  //pub_short_->publish(std::move(short_scan_msg));
  //pub_long_->publish(std::move(long_scan_msg));

  auto copy_long_scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>(*long_scan_msg);

  auto merged_scan_msg = PointCloudToLaserScanNode::mergeLaserScans(short_scan_msg,  std::move(copy_long_scan_msg));
  
  pub_->publish(std::move(merged_scan_msg));
  
  
}
sensor_msgs::msg::LaserScan::UniquePtr PointCloudToLaserScanNode::computeLaserScan(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
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
  std::string target_frame )
{
// build laserscan output
auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
scan_msg->header = cloud_msg->header;
if (!target_frame.empty()) {
  scan_msg->header.frame_id = target_frame_;
}

scan_msg->angle_min = angle_min;
scan_msg->angle_max = angle_max;
scan_msg->angle_increment = angle_increment;
scan_msg->time_increment = 0.0;
scan_msg->scan_time = scan_time;
scan_msg->range_min = min_range;
scan_msg->range_max = max_range;

// determine amount of rays to create
uint32_t ranges_size = std::ceil(
  (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

// determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
if (use_inf) {
  scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
} else {
  scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon);
}

// Iterate through pointcloud
for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
{
if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
  RCLCPP_DEBUG(
    this->get_logger(),
    "rejected for nan in point(%f, %f, %f)\n",
    *iter_x, *iter_y, *iter_z);
  continue;
}

if (*iter_z > max_height || *iter_z < min_height) {
  RCLCPP_DEBUG(
    this->get_logger(),
    "rejected for height %f not in range (%f, %f)\n",
    *iter_z, min_height, max_height);
  continue;
}

double range = hypot(*iter_x, *iter_y);
if (range < scan_msg->range_min) {
  RCLCPP_DEBUG(
    this->get_logger(),
    "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
    range, scan_msg->range_min, *iter_x, *iter_y, *iter_z);
  continue;
}
if (range > scan_msg->range_max) {
  RCLCPP_DEBUG(
    this->get_logger(),
    "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
    range, scan_msg->range_max, *iter_x, *iter_y, *iter_z);
  continue;
}

double angle = atan2(*iter_y, *iter_x);
if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
  RCLCPP_DEBUG(
    this->get_logger(),
    "rejected for angle %f not in range (%f, %f)\n",
    angle, scan_msg->angle_min, scan_msg->angle_max);
  continue;
}

// overwrite range at laserscan ray if new range is smaller
int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
if (range < scan_msg->ranges[index]) {
  scan_msg->ranges[index] = range;
}
}

return scan_msg;


}


sensor_msgs::msg::LaserScan::UniquePtr PointCloudToLaserScanNode::mergeLaserScans(
  const sensor_msgs::msg::LaserScan::UniquePtr & short_scan,
  sensor_msgs::msg::LaserScan::UniquePtr && long_scan)
  {


    long_scan->range_min = std::min(long_scan->range_min, short_scan->range_min);
    for (int i=0; i<long_scan->ranges.size(); i++){
      long_scan->ranges[i] = std::min(long_scan->ranges[i], short_scan->ranges[i]);
    }
    return std::move(long_scan);
  }


}  // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::PointCloudToLaserScanNode)
