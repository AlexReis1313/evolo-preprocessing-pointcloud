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


 
namespace pointcloud_to_laserscan
{

PointCloudToLaserScanNode::PointCloudToLaserScanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_to_laserscan", options)
{
  baseLink_frame_ = this->declare_parameter("base_link", "base_link");
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
  angle_min_ = this->declare_parameter("angle_min_laserscan", -M_PI);
  angle_max_ = this->declare_parameter("angle_max_laserscan", M_PI);
  angle_min_map_ = this->declare_parameter("angle_min_map", -M_PI/2);
  angle_max_map_ = this->declare_parameter("angle_max_map", M_PI/2);
  angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
  scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0);
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
  use_inf_ = this->declare_parameter("use_inf", true);
  min_height_longrange_ = this->declare_parameter("min_height_longrange", std::numeric_limits<double>::min());
  max_height_longrange_ = this->declare_parameter("max_height_longrange", std::numeric_limits<double>::max());
  range_transition_ = this->declare_parameter("range_transition", 0.0);
  angle_visual_outputmap_ =this->declare_parameter("angle_increment_output_map", M_PI / 180.0);
  //params for the radius search removal of the pointcloud
  b_neighboursRadius_ =this->declare_parameter("minimum_radius_paramB", 0.05);
  m_neighboursRadius_ =this->declare_parameter("minimum_radius_paramM", 0.0125);
  nr_neighbours_ =this->declare_parameter("minimum_neighbours", 3);


 

  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
  //pub_short_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scanner/scan/short", rclcpp::SensorDataQoS());
  //pub_long_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scanner/scan/long", rclcpp::SensorDataQoS());
  pub_radialmap_ = this->create_publisher<sensor_msgs::msg::LaserScan>("map/radial", rclcpp::SensorDataQoS());
  pub_radialmapVisual_=this->create_publisher<sensor_msgs::msg::LaserScan>("map/radial/visual", rclcpp::SensorDataQoS());
  pub_pc_= this->create_publisher<sensor_msgs::msg::PointCloud2>("scanner/scan/pointcloud", rclcpp::SensorDataQoS());
  pub_OriginalPC_= this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_pointcloud", rclcpp::SensorDataQoS());

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
      pub_->get_intra_process_subscription_count()+pub_radialmap_->get_subscription_count() +  pub_radialmap_->get_intra_process_subscription_count()
      + pub_pc_->get_subscription_count() +  pub_pc_->get_intra_process_subscription_count(); //+ pub_long_->get_subscription_count() +pub_long_->get_intra_process_subscription_count();
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
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud, pcl_cloud_transformed,pcl_cloud_transformed2, pcl_cloud_filtered;


  // Transform cloud if necessary
  //if (target_frame_ != cloud_msg->header.frame_id) {
  try {
    
    geometry_msgs::msg::TransformStamped transform_stamped, noattitude_transform,transform_cloud2base;
    
    transform_stamped = tf2_->lookupTransform(fixed_frame_, baseLink_frame_, tf2::TimePointZero);

    transform_cloud2base = tf2_->lookupTransform(cloud_msg->header.frame_id, baseLink_frame_, tf2::TimePointZero);
    // adavanced version transform_stamped = tf2_->lookupTransform(fixed_frame_, cloud_msg->header.frame_id, tf2::TimePointZero);
    tf2::Quaternion q_orig, q_new;
    tf2::convert(transform_stamped.transform.rotation, q_orig);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);

    q_new.setRPY(0, 0, yaw );

    noattitude_transform.header.stamp = cloud_msg->header.stamp;
    noattitude_transform.header.frame_id = fixed_frame_;
    noattitude_transform.child_frame_id = target_frame_;
    noattitude_transform.transform.translation.x = transform_stamped.transform.translation.x;
    noattitude_transform.transform.translation.y = transform_stamped.transform.translation.y;
    noattitude_transform.transform.translation.z = transform_stamped.transform.translation.z;
    noattitude_transform.transform.rotation = tf2::toMsg(q_new);

    tf_broadcaster_->sendTransform(noattitude_transform); //this transform is rotated by 180 degrees by the z axis wrt the original pointcloud frame_id
    noattitude_transform.transform.translation.x =0;
    noattitude_transform.transform.translation.y =0;
    noattitude_transform.transform.translation.z =0;
    q_new.setRPY(roll, pitch, 0 );
    noattitude_transform.transform.rotation = tf2::toMsg(q_new);
      
    pcl::fromROSMsg(*cloud_msg, pcl_cloud);
    pcl_ros::transformPointCloud(pcl_cloud, pcl_cloud_transformed, transform_cloud2base);

    pcl_ros::transformPointCloud(pcl_cloud_transformed, pcl_cloud_transformed2, noattitude_transform);
   
    PointCloudToLaserScanNode::filterCloud(pcl_cloud_transformed2, min_height_shortrange_,pcl_cloud_filtered);

    //pcl::toROSMsg(pcl_cloud_transformed, *cloud);
    pcl::toROSMsg(pcl_cloud_filtered,*cloud);
    
    //here, cloud_msg (ROS PointCLoud) and pcl_cloud_transformed (pcl XYZI pointcloud) have the same information
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
  sensor_msgs::msg::PointCloud2 merged_point_cloud;

  //https://wiki.ros.org/laser_geometry
  Laser2PCprojector_.projectLaser(*merged_scan_msg, merged_point_cloud);

  //auto clustered_point_cloud = PointCloudToLaserScanNode::DbscanCluster(merged_scan_msg, height_2dscan)
  auto [radialMap, radialMapVisual] = PointCloudToLaserScanNode::LaserScan2radialMap(merged_scan_msg, angle_min_map_, angle_max_map_, angle_visual_outputmap_);


  sensor_msgs::msg::PointCloud2 transformed_cloud = *cloud_msg; // Create a modifiable copy
  transformed_cloud.header.frame_id = target_frame_; // Set new frame_id
  
  pub_OriginalPC_->publish(transformed_cloud);
  
  pub_radialmapVisual_->publish(std::move(radialMapVisual));
  pub_radialmap_->publish(std::move(radialMap));

  pub_pc_->publish(merged_point_cloud);
  pub_->publish(std::move(merged_scan_msg));
  
  
}

void PointCloudToLaserScanNode::filterCloud(pcl::PointCloud<pcl::PointXYZI> & cloud_in, const double & min_height, pcl::PointCloud<pcl::PointXYZI> & cloud_out){

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(cloud_in, cloud_in, indices);

    pcl::PointCloud<pcl::PointXYZI> filtered;
    float min_intensity, max_intensity;
    min_intensity = std::numeric_limits<float>::max();
    max_intensity = std::numeric_limits<float>::lowest();

    for (const auto& pt : cloud_in.points) {
        if (!std::isfinite(pt.intensity)) continue;
        if (pt.intensity < min_intensity) min_intensity = pt.intensity;
        if (pt.intensity > max_intensity) max_intensity = pt.intensity;
    }
    float intensity_threshold = (max_intensity - min_intensity )*0.1 + min_intensity;
    for (const auto& pt : cloud_in.points) {
            if (pt.intensity >= intensity_threshold || pt.z >= min_height-3.0) { //remove points that are under the water and with low intensity water
                filtered.points.push_back(pt);
            }
        }
    filtered.width = filtered.points.size();
    filtered.height = 1;
    filtered.is_dense = true;

    //pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_ptr(new pcl::PointCloud<pcl::PointXYZI>(cloud_in));
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_ptr(new pcl::PointCloud<pcl::PointXYZI>(filtered));
        /*
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
    outrem.setInputCloud(filtered_ptr);
    outrem.setRadiusSearch(0.45);  // Radius for neighbor search 0.8
    outrem.setMinNeighborsInRadius(2);  // Keep points with >= 3 neighbors 3
    outrem.filter(cloud_out);
        */
    auto output = PointCloudToLaserScanNode::adaptiveRadiusFilter(filtered_ptr, m_neighboursRadius_, b_neighboursRadius_, nr_neighbours_);
    cloud_out=*output;

  }



pcl::PointCloud<pcl::PointXYZI>::Ptr  PointCloudToLaserScanNode::adaptiveRadiusFilter(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
  float scale , float min_radius ,             // Radius = scale * range
  int min_neighbors )
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  
  kdtree.setInputCloud(input_cloud);

  std::vector<int> indices;
  std::vector<float> distances;
  float distance_to_origin, radius;

  for (const auto& pt : input_cloud->points) {
      //if (!pcl::isFinite(pt)) continue;

      distance_to_origin = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      radius = min_radius + (scale * distance_to_origin);
      /*RCLCPP_INFO(
        this->get_logger(),
        "nr neighbours");
      char buffer[64];
      
      int aux = kdtree.radiusSearch(pt, radius, indices, distances); 
      int ret = snprintf(buffer, sizeof buffer, "%i", aux);
        RCLCPP_INFO(
          this->get_logger(),
          buffer);*/
      if (kdtree.radiusSearch(pt, radius, indices, distances) >= min_neighbors) {
          output_cloud->points.push_back(pt);
      }
  }

  output_cloud->width = output_cloud->points.size();
  output_cloud->height = 1;
  output_cloud->is_dense = true;

  return output_cloud;
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

//angle_min += M_PI; //because the filtered point cloud frame is rotated by 180 degrees in relation to the cloud msg frame
//angle_max += M_PI;

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

 
std::pair<sensor_msgs::msg::LaserScan::UniquePtr, sensor_msgs::msg::LaserScan::UniquePtr> 
  PointCloudToLaserScanNode::LaserScan2radialMap(
    const sensor_msgs::msg::LaserScan::UniquePtr &scan,
    double angle_min, double angle_max, double angle_increment)
  {
    auto radialMapScan = std::make_unique<sensor_msgs::msg::LaserScan>();
    auto radialMapScanVisual = std::make_unique<sensor_msgs::msg::LaserScan>();
  
    // Copy metadata from input scan
    radialMapScan->header = scan->header;
    radialMapScan->header.frame_id = scan->header.frame_id;
    radialMapScan->angle_min = angle_min;
    radialMapScan->angle_max = angle_max;
    radialMapScan->angle_increment = angle_increment; //big angle increment 
    radialMapScan->time_increment = scan->time_increment;
    radialMapScan->scan_time = scan->scan_time;
    radialMapScan->range_min = scan->range_min;
    radialMapScan->range_max = scan->range_max;


    radialMapScanVisual->header = radialMapScan->header;
    radialMapScanVisual->header.frame_id = radialMapScan->header.frame_id;
    radialMapScanVisual->angle_min = angle_min;
    radialMapScanVisual->angle_max = angle_max;
    radialMapScanVisual->angle_increment = scan->angle_increment; //fine course angle increment
    radialMapScanVisual->time_increment = scan->time_increment;
    radialMapScanVisual->scan_time = scan->scan_time;
    radialMapScanVisual->range_min = scan->range_min;
    radialMapScanVisual->range_max = scan->range_max;

    
    // Compute the number of output radial map bins
    int number_of_cells = static_cast<int>((angle_max - angle_min) / angle_increment);
    radialMapScan->ranges.resize(number_of_cells, radialMapScan->range_max);

    // Iterate through each bin in the new radial map
    for (int i = 0; i < number_of_cells; ++i)
    {
      double cell_angle_min = angle_min + i * angle_increment;
      double cell_angle_max = cell_angle_min + angle_increment;
      float lowest_range = radialMapScan->range_max;
        
        // Iterate through the original scan ranges to find points in the current bin
      for (size_t j = 0; j < scan->ranges.size(); ++j)
        {
          double current_angle = scan->angle_min + j * scan->angle_increment;
          
          if (current_angle >= cell_angle_min && current_angle < cell_angle_max)
          {
              if (scan->ranges[j] >= scan->range_min && scan->ranges[j] <= scan->range_max)
              {
                       lowest_range = std::min(lowest_range, scan->ranges[j]);
              }
          }
      radialMapScan->ranges[i] = lowest_range;

        }
              
    }

    
    int visual_cells = static_cast<int>((angle_max - angle_min) / scan->angle_increment);
    radialMapScanVisual->ranges.resize(visual_cells, radialMapScan->range_max);
    
    // Assign values from radialMapScan to radialMapScanVisual
    for (int i = 0; i < visual_cells; ++i)
    {
        double current_angle = angle_min + i * scan->angle_increment;
        int radial_index = static_cast<int>((current_angle - angle_min) / angle_increment);
        radialMapScanVisual->ranges[i] = radialMapScan->ranges[radial_index];
    }
    
    return {std::move(radialMapScan), std::move(radialMapScanVisual)};
}


}  // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::PointCloudToLaserScanNode)
