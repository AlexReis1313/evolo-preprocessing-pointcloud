#include "pointcloud_preprocessing_node.hpp"
#include "timing_metrics.cpp"

namespace pointcloud_preprocessing
{

PointCloudPreProcessingNode::PointCloudPreProcessingNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_preprocessing", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting node");

  //reading the ros2 params for this node and saving them in the params_ class
  params_ = std::make_unique<PointCloudPreProcessingParams>(this);
  
  //original PC, transformed to base_footprint and filtered with RANSAC and adaptiveRadiusFilter
  pub_FilteredPC_= this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered/pc/pointcloud", rclcpp::SensorDataQoS()); 
  //filtered Point Cloud flatten to 2d laserscan
  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("filtered/ls/laserscan", rclcpp::SensorDataQoS()); 
  //same as last topic but in PointCloud format
  pub_pc_= this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered/ls/pointcloud", rclcpp::SensorDataQoS());
  //same as last topic but with time decay
  pub_accumulatedPC_laserscan_= this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered/ls/pointcloud/accumulated", rclcpp::SensorDataQoS()); 
  
  //complete 3d pointcloud projected to 2d pointcloud - all points have z=0 in base_footprint frame
  pub_projectedPC_= this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered/pc/pointcloud/Projected", rclcpp::SensorDataQoS());
  pub_projected_AccumulatedPC_= this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered/pc/pointcloud/Projected/accumulated", rclcpp::SensorDataQoS()); //time decay version

  //radial map - filtered/ls/laserscan transformed into a low resolution 2d laserscan that reprsents lower range found in original ls for each angle interval
  pub_radialmap_ = this->create_publisher<sensor_msgs::msg::LaserScan>("map/radial", rclcpp::SensorDataQoS()); //1 point per angle interval
  pub_radialmapVisual_=this->create_publisher<sensor_msgs::msg::LaserScan>("map/radial/visual", rclcpp::SensorDataQoS()); //many points of same range per angle interval - to view on rviz

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("estimatedWaterPlane", 10); //ransac produced water plane
  RCLCPP_INFO(this->get_logger(), "Started publishers");

  using std::placeholders::_1;
  // if pointcloud target frame specified, we need to filter by transform availability
  if (!params_->target_frame_.empty()) {
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
    message_filter_ = std::make_unique<MessageFilter>(
      sub_, *tf2_, params_->cloud_frame_, params_->input_queue_size_,
      this->get_node_logging_interface(),
      this->get_node_clock_interface());
    message_filter_->registerCallback(
      std::bind(&PointCloudPreProcessingNode::cloudCallback, this, _1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  } else {  // otherwise setup direct subscription
    sub_.registerCallback(std::bind(&PointCloudPreProcessingNode::cloudCallback, this, _1));
  }

  subscription_listener_thread_ = std::thread(
    std::bind(&PointCloudPreProcessingNode::subscriptionListenerThreadLoop, this));

  // Register callback for dynamic parameters updates
  callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&PointCloudPreProcessingParams::onParameterChange, params_.get(), std::placeholders::_1)
  );
}

PointCloudPreProcessingNode::~PointCloudPreProcessingNode()
{
  alive_.store(false);
  subscription_listener_thread_.join();
}

void PointCloudPreProcessingNode::subscriptionListenerThreadLoop()
{
  rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();

  // This node only subscribes to LiDAR point cloud and computes all filtering when some other node is subscribing to any of this node's outputs
  // This function performs that logic, and was adapted from https://github.com/ros-perception/pointcloud_to_laserscan 
  const std::chrono::milliseconds timeout(100);
  while (rclcpp::ok(context) && alive_.load()) {
    int subscription_count = pub_->get_subscription_count() +pub_->get_intra_process_subscription_count() +
      pub_radialmap_->get_subscription_count() +  pub_radialmap_->get_intra_process_subscription_count() +
      pub_pc_->get_subscription_count() +  pub_pc_->get_intra_process_subscription_count() +
      pub_radialmap_->get_subscription_count() +  pub_radialmap_->get_intra_process_subscription_count() +
      pub_FilteredPC_->get_subscription_count() +  pub_FilteredPC_->get_intra_process_subscription_count() +
      pub_radialmapVisual_->get_subscription_count() +  pub_radialmapVisual_->get_intra_process_subscription_count() + 
      pub_accumulatedPC_laserscan_->get_subscription_count() + pub_accumulatedPC_laserscan_->get_intra_process_subscription_count() +
      pub_projectedPC_->get_subscription_count() + pub_projectedPC_->get_intra_process_subscription_count() +
      pub_projected_AccumulatedPC_->get_subscription_count() + pub_projected_AccumulatedPC_->get_intra_process_subscription_count(); 

    if (subscription_count > 0) {
      if (!sub_.getSubscriber()) {
        RCLCPP_INFO(this->get_logger(), "Got a subscriber to Point Cloud pre-processing, starting LiDAR pointcloud subscriber");
        rclcpp::SensorDataQoS qos;
        qos.keep_last(params_->input_queue_size_);
        sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile());
      }
    } else if (sub_.getSubscriber()) {
      RCLCPP_INFO(this->get_logger(),"No subscribers to laserscan, shutting down pointcloud subscriber");
      sub_.unsubscribe();
    }
    rclcpp::Event::SharedPtr event = this->get_graph_event();
    this->wait_for_graph_change(event, timeout);
  }
  sub_.unsubscribe();
}



//cloudCallback is effectively the main function of this program
void PointCloudPreProcessingNode::cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  ScopedTimer timer_calback("Entire Lidar Cloud callback",this, params_->timeMetric);

  try {
      //init variables
      ScopedTimer timer_transform("Transforming original cloud to base_footprint",this, params_->timeMetric);

      auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      auto cloudREJ = std::make_shared<sensor_msgs::msg::PointCloud2>();
      auto cloud_projected2D_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      auto laserScanPC_timeDecay_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      auto cloud_projected2D_timeDecay_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      pcl::PointCloud<pcl::PointXYZI> pcl_cloud, pcl_cloud_transformed, pcl_cloud_filtered, rejected_pointcloud;

      // Get transform from cloud frame to base_footprint
      // Convert ROS message to PCL point cloud
      // Transform point cloud to base_footprint frame
      pcl::fromROSMsg(*cloud_msg, pcl_cloud);
      
      if(params_->simulation_mode_){ //logic that deals with bad clock and use_sim_time that may come from using Unity Sim
        geometry_msgs::msg::TransformStamped transform = 
            tf2_->lookupTransform(params_->target_frame_, cloud_msg->header.frame_id, tf2_ros::fromMsg(cloud_msg->header.stamp));
        pcl_ros::transformPointCloud(pcl_cloud, pcl_cloud_transformed, transform);
      }else{ //normal evolo
        geometry_msgs::msg::TransformStamped transform = 
          tf2_->lookupTransform(params_->target_frame_, cloud_msg->header.frame_id, tf2::TimePointZero);
        pcl_ros::transformPointCloud(pcl_cloud, pcl_cloud_transformed, transform);
      }
      timer_transform.stopClock(); //prints out time
     
      // Apply filtering: filters out points with negative z and low intensity (water reflections) and does adaptiveRadiusFilter
      //pcl_cloud_filtered is the output, rejected_pointcloud has all the points that were filtered out (for debug purposes) - it is not currently used
      PointCloudPreProcessingNode::filterCloud(pcl_cloud_transformed, params_->min_height_shortrange_, pcl_cloud_filtered, rejected_pointcloud);

      // Convert back to ROS message and update headers to reflect the new frame
      pcl::toROSMsg(pcl_cloud_filtered, *cloud);
      cloud->header.frame_id = params_->target_frame_;
      cloud->header.stamp = cloud_msg->header.stamp; //this is the origintal PC, transformed to base_footprint frame and filtered (z and low intensity (water reflections) and adaptiveRadiusFilter)
      ScopedTimer pctoLsTimer("PC 2 LS",this, params_->timeMetric);

      //the following code tranansforms and filters a point cloud into a laser scan - point coordinate filter for further removal of water reflections. A 2d laser scan is outputed, taking closest non-water point to the lidar, in each horizontal angle
      //short_scan_msg is a 2d laserscan of points in a close range. In this range, water reflections filtering is agressive due to there being more water reflections.
      //  and IMU attitude without accumulated error because of long distance - this is especially needed for when ransac fails
      //long_scan_msg is a 2d laserscan of points in long range. In this range, small IMU attitude errors accumulate into big z coordinate diferences, so filtering for water reflections is much less agressive
      // merged_scan_msg merges short_scan_msg and long_scan_msg, taking closeste point from both of them, in each horizontal angle
      // merged_point_cloud has the same information as merged_scan_msg but in 3D Point cloud message format, instead of 2D laser scan. It still only has points in one horizontal plane
      params_->setUpParamsShortRange();
      auto short_scan_msg = PointCloudPreProcessingNode::computeLaserScan(cloud);
      params_->setUpParamsLongRange();
      auto long_scan_msg = PointCloudPreProcessingNode::computeLaserScan(cloud);
      auto merged_scan_msg = PointCloudPreProcessingNode::mergeLaserScans(short_scan_msg,  std::move(long_scan_msg));
      merged_scan_msg->header.stamp = cloud_msg->header.stamp;
      sensor_msgs::msg::PointCloud2 merged_point_cloud;
      Laser2PCprojector_.projectLaser(*merged_scan_msg, merged_point_cloud);  //https://wiki.ros.org/laser_geometry

      //The following code takes the merged_scan_msg 2d laserscan and convert it into lower resolution radialMap. The resolution is defined by params_->angle_increment_, it is normally 12 measurements for a 360degree scan
      // for each angle interval, the closeste laserscan from merged_scan_msg is used as range for radialMap
      // radialMapVisual has the same information of radialMap but includes much higher resolution, effectively representing each of the ~12 segments as an arc of points, all with the same range defined in radialMap
      // radialMapVisual is mostly used for RVIZ visualization
      // radialMap is used to communicate regions where obstacles are to the USV's path controller
      auto [radialMap, radialMapVisual] = PointCloudPreProcessingNode::LaserScan2radialMap(merged_scan_msg);
      pctoLsTimer.stopClock();
      //project the filtered and transformed cloud into a fixed world frame -> needed to accumulate points in time and get a time decay effect
      //accumulate the merged_point_cloud over a time horizon, similar to rviz time decay - usefull to get more features to cluster with
      //project() zeros the z of all points in the filtered point cloud, getting a 2d one. This is also accumulated over time
      ScopedTimer accuProjTimer("Accumulating and projecting",this, params_->timeMetric);

      geometry_msgs::msg::TransformStamped world_fix_transform = 
          tf2_->lookupTransform(params_->fixed_frame_,params_->target_frame_, tf2::TimePointZero);  
      geometry_msgs::msg::TransformStamped inverse_world_fix_transform = 
          tf2_->lookupTransform(params_->target_frame_, params_->fixed_frame_,tf2::TimePointZero);
      pcl::PointCloud<pcl::PointXYZI> cloud_projected2D,cloud_projected2D_timeDecay, pcl_merged_cloud,laserScanPC_timeDecay;; 
      pcl::fromROSMsg(merged_point_cloud, pcl_merged_cloud); // causes [pointcloud_to_laserscan_node-2] Failed to find match for field 'intensity'.
      PointCloudPreProcessingNode::accumulate(pcl_merged_cloud, laserScanPC_timeDecay,rclcpp::Time(cloud_msg->header.stamp), params_->timeDecay_, world_fix_transform, inverse_world_fix_transform,clouds_queu_laserscan_);
      PointCloudPreProcessingNode::project(pcl_cloud_filtered ,cloud_projected2D);
      PointCloudPreProcessingNode::accumulate(cloud_projected2D, cloud_projected2D_timeDecay,rclcpp::Time(cloud_msg->header.stamp), params_->timeDecay_,world_fix_transform,inverse_world_fix_transform,clouds_queu_projectedPc_);
      //setup ros msgs to be published
      accuProjTimer.stopClock();
      pcl::toROSMsg(laserScanPC_timeDecay,*laserScanPC_timeDecay_msg);
      pcl::toROSMsg(cloud_projected2D, *cloud_projected2D_msg); //debugging
      pcl::toROSMsg(cloud_projected2D_timeDecay, *cloud_projected2D_timeDecay_msg);
      if(params_->outputOnFixedFrame_){
        cloud_projected2D_timeDecay_msg->header.frame_id = params_->fixed_frame_;
        laserScanPC_timeDecay_msg->header.frame_id = params_->fixed_frame_;
      }else{
        cloud_projected2D_timeDecay_msg->header.frame_id = params_->target_frame_;
        laserScanPC_timeDecay_msg->header.frame_id = params_->target_frame_;
      }
      cloud_projected2D_msg->header.frame_id = params_->target_frame_;    
      laserScanPC_timeDecay_msg->header.stamp = cloud_msg->header.stamp;
      cloud_projected2D_msg->header.stamp = cloud_msg->header.stamp;
      cloud_projected2D_timeDecay_msg->header.stamp = cloud_msg->header.stamp;
      merged_point_cloud.header.frame_id = params_->target_frame_;
      merged_point_cloud.header.stamp =cloud_msg->header.stamp;
     

      //publishing everything
      pub_FilteredPC_->publish(std::move(*cloud));
      pub_radialmapVisual_->publish(std::move(radialMapVisual));
      pub_radialmap_->publish(std::move(radialMap));
      pub_pc_->publish(merged_point_cloud);
      pub_->publish(std::move(merged_scan_msg));
      pub_accumulatedPC_laserscan_->publish(std::move(*laserScanPC_timeDecay_msg));
      pub_projectedPC_->publish(std::move(*cloud_projected2D_msg));
      pub_projected_AccumulatedPC_->publish(std::move(*cloud_projected2D_timeDecay_msg));
  
  } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to transform point cloud from " << cloud_msg->header.frame_id 
          << " to base_footprint: " << ex.what());
      return;
  }
}

void PointCloudPreProcessingNode::project(pcl::PointCloud<pcl::PointXYZI> & cloud_in,pcl::PointCloud<pcl::PointXYZI> & cloud_out_projected){

  cloud_out_projected = cloud_in;
  for (auto& pt : cloud_out_projected.points) {
         pt.z = 0.0f;
        }
  
  cloud_out_projected.width = cloud_out_projected.points.size();
  cloud_out_projected.height = 1;
  cloud_out_projected.is_dense = true;

}


void PointCloudPreProcessingNode::accumulate(const pcl::PointCloud<pcl::PointXYZI> & cloud_in, pcl::PointCloud<pcl::PointXYZI> & cloud_out_accumulated,
    const rclcpp::Time & currentTime, const double & time_decay,geometry_msgs::msg::TransformStamped & world_fix_transform,geometry_msgs::msg::TransformStamped & inverse_world_fix_transform ,std::deque<TimedCloud> cloud_queue){ 

  pcl::PointCloud<pcl::PointXYZI> cloud_in_transformed, cloud_out_untrasnformed;
  if (cloud_in.empty()) {
    RCLCPP_WARN(this->get_logger(), "Input point cloud is empty, skipping transformation.");
       
  } else{
    pcl_ros::transformPointCloud(cloud_in, cloud_in_transformed, world_fix_transform); //making it crash process has died [pid 43347, exit code -8, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(cloud_in_transformed));
    // Store with timestamp
    cloud_queue.push_back(TimedCloud(currentTime, cloud_ptr));
    // Remove clouds older than time_decay                                                     this counter is if time for some reason goes back, for example playing a rosbag in loop with use_sim_time true
    while (!cloud_queue.empty()  &&  (  ((currentTime - cloud_queue.front().timestamp).seconds() > time_decay)  ||   (cloud_queue.front().counter >  time_decay*10 )  ) ) {
              cloud_queue.pop_front();                                            
          }    
    cloud_out_accumulated.clear();
    if(params_->outputOnFixedFrame_){
      // Concatenate remaining clouds
      for (auto &timed_cloud : cloud_queue) {
        cloud_out_accumulated += *(timed_cloud.cloud);
        timed_cloud.counter +=1.0; //double because it is compared to a double time_decay * 10Hz
      }
    }else{
      cloud_out_untrasnformed.clear();
      // Concatenate remaining clouds
      for (const auto &timed_cloud : cloud_queue) {
          cloud_out_untrasnformed += *(timed_cloud.cloud);
      }
      pcl_ros::transformPointCloud(cloud_out_untrasnformed, cloud_out_accumulated,inverse_world_fix_transform );
    }
  }
}

bool PointCloudPreProcessingNode::detectWaterPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out){

    
    // 1. Filter points within short range and low height - candidates for water (range ~ 30m, z < ~1m)
    pcl::PointCloud<pcl::PointXYZI>::Ptr ransac_candidates(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr ransac_non_candidates(new pcl::PointCloud<pcl::PointXYZI>());

    for (const auto &pt : cloud_in->points)
    {
        float range = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        if (range > 1 && range < params_->ransac_range_candidates_ &&  pt.z < params_->ransac_Maxheight_candidates_ &&  pt.z > params_->ransac_Minheight_candidates_)
        {
          ransac_candidates->points.push_back(pt);
        }else {
          ransac_non_candidates->points.push_back(pt); 
        }
    }
    if (ransac_candidates->size()<15)//we need 3 points to define plane, but we want at least a few inliers to know it is representative
    {
        //std::cerr << "[detectGroundPlane] Not enought points found to estimate plane.\n";
        return false;
    }

    // 2. Set up RANSAC plane estimation
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true); // this makes the coeficients be the least squares of all inliers
    seg.setModelType(pcl::SACMODEL_PLANE); //modle to fit is a plane ax + by + cz + d = 0
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(params_->ransac_threshold_inliers_);  // in meters
    seg.setInputCloud(ransac_candidates);
    seg.segment(*inliers, *coefficients); //performs ransac and Least Squares estimate of plane with inliers

    if (inliers->indices.size()<10)
    {
        std::cerr << "[detectGroundPlane] No plane inliers found.\n";
        return false;
    }
    // outlier extractor
    pcl::PointCloud<pcl::PointXYZI>::Ptr outliers(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(true);  // <-- This means extract *non-inliers* = outliers
    extract.filter(*outliers);

    // Store plane coefficients: ax + by + cz + d = 0
    if (coefficients->values.size() == 4)
    {
        float a = coefficients->values[0];
        float b = coefficients->values[1];
        float c = coefficients->values[2];
        float d = coefficients->values[3];
        float threshold = 10.0; //planes more inclined than this angle in degrees will not be considered
        if (!checkWaterPlane(a,b,c,d, threshold)){
          return false;
          //RCLCPP_INFO(this->get_logger(), "Plane equation not valid");
        }
        publish_plane_marker( a, b, c, d); //publishes blue plane to be seen in rviz that shows water estimation
        float plane_norm = std::sqrt(a*a + b*b + c*c);
        *outliers += *ransac_non_candidates; //merge the ransac candidates that do not bellong to water surface with non ransac candidates
        //then we filter from this big group and only keep points that are above the water or too far away to make considerations
        for (const auto& point : outliers->points) {
            // Compute plane equation value
            float plane_value = a * point.x + b * point.y + c * point.z + d ;
            float plane_dist = std::abs(plane_value)/plane_norm; //distance from plane to point
            // Keep point only if it's on or above the plane (>= 0)
            float range = std::sqrt(point.x * point.x + point.y * point.y);
            if ((plane_value > 0 && plane_dist > params_->ransac_filter_height_) || range > params_->range_transition_) {
                cloud_out->points.push_back(point);
            }
        }
    }
    // Set metadata
    cloud_out->width = static_cast<uint32_t>(cloud_out->points.size());
    cloud_out->height = 1;
    cloud_out->is_dense = true;
    return true;
}

bool PointCloudPreProcessingNode::checkWaterPlane(float a, float b, float c, float d, float threshold_degrees)
{
    float norm = std::sqrt(a * a + b * b + c * c);
    if (norm == 0.0f) {
        return false; // invalid normal vector
    }
    //we are computinh angle with horizontal plane, wich has normal vector (0,0,1). Dot produc is then just 0 + 0 + c*1. norm_horizontal=1. norm_plane=norm
    //cost_theta=|dot product|/norm1*norm2
    float cos_theta = std::abs(c) / norm;
    float angle_radians = std::acos(cos_theta);
    float angle_degrees = angle_radians * 180.0f / M_PI;
    //RCLCPP_INFO(this->get_logger(), "Plane angle: %f", angle_degrees);

    if( angle_degrees > threshold_degrees){
     return false;
    }
    return true;
}

void PointCloudPreProcessingNode::filterCloud(
    pcl::PointCloud<pcl::PointXYZI> &cloud_in,
    const double &min_height,
    pcl::PointCloud<pcl::PointXYZI> &cloud_out,
    pcl::PointCloud<pcl::PointXYZI> &cloud_outREJ)
{
  ScopedTimer filterTimer("Filtering",this, params_->timeMetric);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(cloud_in, cloud_in, indices);
  // Use shared_ptr for all filtered stages
  pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(cloud_in);

  // RANSAC plane detection (if enabled)
  if (params_->useRansac_) {
      ScopedTimer filterTimer("ransac",this, params_->timeMetric);
      pcl::PointCloud<pcl::PointXYZI>::Ptr ransac_filtered(new pcl::PointCloud<pcl::PointXYZI>);
      bool ransacSuccess = detectWaterPlane(current_cloud, ransac_filtered);
      if (ransacSuccess) {
          current_cloud = ransac_filtered;
      }
      // If RANSAC fails, we just continue with current_cloud (still points to cloud_in)
  }

  // Intensity filtering (if enabled) - not usually used
  if (params_->filterBy_intensity_) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_filtered(new pcl::PointCloud<pcl::PointXYZI>);

      float min_intensity = std::numeric_limits<float>::max();
      float max_intensity = std::numeric_limits<float>::lowest();

      for (const auto &pt : current_cloud->points) {
          if (!std::isfinite(pt.intensity)) continue;
          min_intensity = std::min(min_intensity, pt.intensity);
          max_intensity = std::max(max_intensity, pt.intensity);
      }
      float intensity_threshold = (max_intensity - min_intensity) * 0.1f + min_intensity;
      for (const auto &pt : current_cloud->points) {
          if (pt.intensity >= intensity_threshold || pt.z >= min_height - 3.0) {
              intensity_filtered->points.push_back(pt);
          }
      }
      intensity_filtered->width = intensity_filtered->points.size();
      intensity_filtered->height = 1;
      intensity_filtered->is_dense = true;
      current_cloud = intensity_filtered;
  }

  // Adaptive radius filtering (if not in sim mode - simulation does not show water reflections and cannot estimate plane)
  if(!params_->simulation_mode_){
    ScopedTimer timer_tranaform("adaptiveRadiusFilter",this, params_->timeMetric);

    auto [output , rejected_output]= PointCloudPreProcessingNode::adaptiveRadiusFilter(current_cloud, params_->m_neighboursRadius_, params_->b_neighboursRadius_, params_->nr_neighbours_);
    cloud_out=*output;
    cloud_outREJ=*rejected_output;
  }else{
    cloud_out=cloud_in;
  }
}
    
std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr , pcl::PointCloud<pcl::PointXYZI>::Ptr> PointCloudPreProcessingNode::adaptiveRadiusFilter(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
  float scale , float min_radius ,             // Radius = scale * range
  int min_neighbors )
{
  // only keeps points that have at least min_neighbors in a radius search. That radius is proportional
  // to the lidar range at that point - this compensates for LiDAR sparsity at long ranges
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_rejectedcloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  
  kdtree.setInputCloud(input_cloud);
  std::vector<int> indices;
  std::vector<float> distances;
  float distance_to_origin, radius;
  for (const auto& pt : input_cloud->points) {
      distance_to_origin = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      radius = min_radius + (scale * distance_to_origin);
      
      if (kdtree.radiusSearch(pt, radius, indices, distances) >= min_neighbors) {
          output_cloud->points.push_back(pt);
      }
      else{
        output_rejectedcloud->points.push_back(pt);

      }
  }
  output_cloud->width = output_cloud->points.size();
  output_cloud->height = 1;
  output_cloud->is_dense = true;
  output_rejectedcloud->width = output_rejectedcloud->points.size();
  output_rejectedcloud->height = 1;
  output_rejectedcloud->is_dense = true;
  return {output_cloud,output_rejectedcloud};
}

sensor_msgs::msg::LaserScan::UniquePtr PointCloudPreProcessingNode::computeLaserScan(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg )
{
// build laserscan output
//function created by Paul Bovbel, Tully Foote and Michel Hidalgo. Source code found here: https://github.com/ros-perception/pointcloud_to_laserscan 
auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
scan_msg->header = cloud_msg->header;
scan_msg->header.frame_id = params_->target_frame_;
scan_msg->angle_min = params_->angle_min_;
scan_msg->angle_max = params_->angle_max_;
scan_msg->angle_increment = params_->angle_increment_;
scan_msg->time_increment = 0.0;
scan_msg->scan_time = params_->scan_time_;
scan_msg->range_min = params_->range_min_laserscan_;
scan_msg->range_max = params_->range_max_laserscan_;

// determine amount of rays to create
uint32_t ranges_size = std::ceil(
  (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

// determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
if (params_->use_inf_) {
  scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
} else {
  scan_msg->ranges.assign(ranges_size, scan_msg->range_max + params_->inf_epsilon_);
}
scan_msg->intensities.assign(ranges_size, 0.0f); 

// Iterate through pointcloud
for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
  iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z"), iter_intensity(*cloud_msg, "intensity");
  iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z,++iter_intensity)
  {
  if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "rejected for nan in point(%f, %f, %f)\n",
      *iter_x, *iter_y, *iter_z);
    continue;
    }

  if (*iter_z > params_->max_height_ || *iter_z < params_->min_height_) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "rejected for height %f not in range (%f, %f)\n",
      *iter_z, params_->min_height_, params_->max_height_);
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
    scan_msg->intensities[index]=  *iter_intensity;
  }
}

return scan_msg;


}


sensor_msgs::msg::LaserScan::UniquePtr PointCloudPreProcessingNode::mergeLaserScans(
  const sensor_msgs::msg::LaserScan::UniquePtr & short_scan,
  sensor_msgs::msg::LaserScan::UniquePtr && long_scan)
  {
    //merges 2 laserscans into one. Assumes both have equal resolution. Takes minimum range between 2 inputs at each angle resolution
    long_scan->range_min = std::min(long_scan->range_min, short_scan->range_min);
    for (int i=0; i<long_scan->ranges.size(); i++){
      //long_scan->ranges[i] = std::min(long_scan->ranges[i], short_scan->ranges[i]);
      if (short_scan->ranges[i] < long_scan->ranges[i]) {
            long_scan->ranges[i] = short_scan->ranges[i];
            long_scan->intensities[i] = short_scan->intensities[i];
      }
    }
    return std::move(long_scan);
  }

 
std::pair<sensor_msgs::msg::LaserScan::UniquePtr, sensor_msgs::msg::LaserScan::UniquePtr> 
  PointCloudPreProcessingNode::LaserScan2radialMap( const sensor_msgs::msg::LaserScan::UniquePtr &scan)
  {
    //creates low resolution ranges radial map(in the form of a laserscan) from an input laserscan 
    //at each angle interval params_->angle_increment_, this keeps the point with the least range
    // also outputs radialMapScanVisual which has high resolution and is meant to be viewd in RVIZ. For each angle interval,
    // this has several points all with same range as radialMapScan range
    auto radialMapScan = std::make_unique<sensor_msgs::msg::LaserScan>();
    auto radialMapScanVisual = std::make_unique<sensor_msgs::msg::LaserScan>();
  
    // Copy metadata from input scan
    radialMapScan->header = scan->header;
    radialMapScan->header.frame_id = scan->header.frame_id;
    radialMapScan->angle_min = params_->angle_min_map_;
    radialMapScan->angle_max = params_->angle_max_map_;
    radialMapScan->angle_increment = params_->angle_increment_; //big angle increment 
    radialMapScan->time_increment = scan->time_increment;
    radialMapScan->scan_time = scan->scan_time;
    radialMapScan->range_min = scan->range_min;
    radialMapScan->range_max = scan->range_max;

    radialMapScanVisual->header = radialMapScan->header;
    radialMapScanVisual->header.frame_id = radialMapScan->header.frame_id;
    radialMapScanVisual->angle_min = params_->angle_min_map_;
    radialMapScanVisual->angle_max = params_->angle_max_map_;
    radialMapScanVisual->angle_increment = scan->angle_increment; //fine course angle increment
    radialMapScanVisual->time_increment = scan->time_increment;
    radialMapScanVisual->scan_time = scan->scan_time;
    radialMapScanVisual->range_min = scan->range_min;
    radialMapScanVisual->range_max = scan->range_max;

    
    // Compute the number of output radial map bins
    int number_of_cells = static_cast<int>((params_->angle_max_map_ - params_->angle_min_map_) / params_->angle_increment_);
    radialMapScan->ranges.resize(number_of_cells, radialMapScan->range_max);

    // Iterate through each bin in the new radial map
    for (int i = 0; i < number_of_cells; ++i)
    {
      double cell_angle_min = params_->angle_min_map_ + i * params_->angle_increment_;
      double cell_angle_max = cell_angle_min + params_->angle_increment_;
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

    //computes radialMapScanVisual
    int visual_cells = static_cast<int>((params_->angle_max_map_ - params_->angle_min_map_) / scan->angle_increment);
    radialMapScanVisual->ranges.resize(visual_cells, radialMapScan->range_max);
    // Assign values from radialMapScan to radialMapScanVisual
    for (int i = 0; i < visual_cells; ++i)
    {
        double current_angle = params_->angle_min_map_ + i * scan->angle_increment;
        int radial_index = static_cast<int>((current_angle - params_->angle_min_map_) / params_->angle_increment_);
        radialMapScanVisual->ranges[i] = radialMapScan->ranges[radial_index];
    }
    
    return {std::move(radialMapScan), std::move(radialMapScanVisual)};
}



void PointCloudPreProcessingNode::publish_plane_marker(float a, float b, float c, float d)
{
    const std::string frame_id = "base_footprint";
    int marker_id = 0;
    // Normalize plane normal
    Eigen::Vector3f normal(a, b, c);
    normal.normalize();

    // Create a marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    //marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "plane";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Center point on the plane (pick arbitrary x and y, solve for z)
    float size = params_->range_transition_ * 2; // size of the visualized plane in meters
    Eigen::Vector3f point_on_plane;

    // Avoid divide by zero
    if (c != 0) {
        point_on_plane = Eigen::Vector3f(0, 0, -d / c);
    } else if (b != 0) {
        point_on_plane = Eigen::Vector3f(0, -d / b, 0);
    } else {
        point_on_plane = Eigen::Vector3f(-d / a, 0, 0);
    }

    marker.pose.position.x = point_on_plane.x();
    marker.pose.position.y = point_on_plane.y();
    marker.pose.position.z = point_on_plane.z();

    // Compute quaternion from normal
    Eigen::Vector3f default_normal(0, 0, 1); // cube's default normal
    Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(default_normal, normal);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    // Size of the visualized plane (width, height, and small thickness)
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = 0.01;

    // Color
    marker.color.r = 0.0f;
    marker.color.g = 0.1f;
    marker.color.b = 0.6f;
    marker.color.a = 0.4f;

    marker.lifetime = rclcpp::Duration::from_seconds(1); // 0 = forever
   
    marker_pub_->publish(marker);
}


}  // namespace pointcloud_preprocessing

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessing::PointCloudPreProcessingNode)
