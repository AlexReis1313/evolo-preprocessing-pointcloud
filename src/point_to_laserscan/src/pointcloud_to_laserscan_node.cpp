#include "pointcloud_to_laserscan_node.hpp"


 
namespace pointcloud_to_laserscan
{

PointCloudToLaserScanNode::PointCloudToLaserScanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_to_laserscan", options)
{
  //reading the ros2 params for this node and saving them in the params_ class
  params_ = std::make_unique<PointCloudToLaserScanParams>(this);
 

  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("filtered/ls/laserscan", rclcpp::SensorDataQoS()); //filtered_laserscan
  pub_pc_= this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered/ls/pointcloud", rclcpp::SensorDataQoS());
  pub_accumulatedPC_laserscan_= this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered/ls/pointcloud/accumulated", rclcpp::SensorDataQoS());
  //pub_accumulatedPC_= this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered/3d/pointcloud/accumulated"), rclcpp::SensorDataQoS());

  //pub_short_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scanner/scan/short", rclcpp::SensorDataQoS());
  //pub_long_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scanner/scan/long", rclcpp::SensorDataQoS());
  pub_radialmap_ = this->create_publisher<sensor_msgs::msg::LaserScan>("map/radial", rclcpp::SensorDataQoS());
  pub_radialmapVisual_=this->create_publisher<sensor_msgs::msg::LaserScan>("map/radial/visual", rclcpp::SensorDataQoS());
  pub_OriginalPC_= this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered/pc/pointcloud", rclcpp::SensorDataQoS());
  pub_REJ_= this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered/pc/rejected_pointcloud", rclcpp::SensorDataQoS());
  
  pub_projectedPC_= this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered/pc/pointcloud/Projected", rclcpp::SensorDataQoS());
  pub_projected_AccumulatedPC_= this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered/pc/pointcloud/Projected/accumulated", rclcpp::SensorDataQoS());

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("estimatedWaterPlane", 10);


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

  // This node only subscribes to LiDAR point cloud and computes all filtering when some other node is subscribing to any of this node's outputs
  // This function performs that logic, and was copied from https://github.com/ros-perception/pointcloud_to_laserscan 
  const std::chrono::milliseconds timeout(100);
  while (rclcpp::ok(context) && alive_.load()) {
    int subscription_count = pub_->get_subscription_count() +
      pub_->get_intra_process_subscription_count()+pub_radialmap_->get_subscription_count() +  pub_radialmap_->get_intra_process_subscription_count()
      + pub_pc_->get_subscription_count() +  pub_pc_->get_intra_process_subscription_count() +
      pub_radialmap_->get_subscription_count() +  pub_radialmap_->get_intra_process_subscription_count()
      + pub_OriginalPC_->get_subscription_count() +  pub_OriginalPC_->get_intra_process_subscription_count() +
      pub_radialmapVisual_->get_subscription_count() +  pub_radialmapVisual_->get_intra_process_subscription_count() + 
      pub_accumulatedPC_laserscan_->get_subscription_count() + pub_accumulatedPC_laserscan_->get_intra_process_subscription_count() ; //+ pub_long_->get_subscription_count() +pub_long_->get_intra_process_subscription_count();
    if (subscription_count > 0) {
      if (!sub_.getSubscriber()) {
        RCLCPP_INFO(
          this->get_logger(),
          "Got a subscriber to Point Cloud pre-processing, starting LiDAR pointcloud subscriber");
        rclcpp::SensorDataQoS qos;
        qos.keep_last(params_->input_queue_size_);
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



//cloudCallback is effectively the main function of this program
void PointCloudToLaserScanNode::cloudCallback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{


  try {
      //init variables
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
      
      if(params_->simulation_mode_){

        geometry_msgs::msg::TransformStamped transform = 
            tf2_->lookupTransform(params_->target_frame_, cloud_msg->header.frame_id, tf2_ros::fromMsg(cloud_msg->header.stamp));

        pcl_ros::transformPointCloud(pcl_cloud, pcl_cloud_transformed, transform);


      }else{ //normal evolo
        geometry_msgs::msg::TransformStamped transform = 
          tf2_->lookupTransform(params_->target_frame_, cloud_msg->header.frame_id, tf2::TimePointZero);
        pcl_ros::transformPointCloud(pcl_cloud, pcl_cloud_transformed, transform);

      }


      // Apply filtering: filters out points with negative z and low intensity (water reflections) and does adaptiveRadiusFilter
      //pcl_cloud_filtered is the output, rejected_pointcloud has all the points that were filtered out (for debug purposes)
      PointCloudToLaserScanNode::filterCloud(pcl_cloud_transformed, params_->min_height_shortrange_, pcl_cloud_filtered, rejected_pointcloud);

      // Convert back to ROS message and update headers to reflect the new frame

      pcl::toROSMsg(pcl_cloud_filtered, *cloud);
      //pcl::toROSMsg(rejected_pointcloud, *cloudREJ);
      //RCLCPP_INFO( this->get_logger(), " before stamp:  %f n",  rclcpp::Time(cloud_msg->header.stamp).seconds());
      cloud->header.frame_id = params_->target_frame_;
      cloud->header.stamp = cloud_msg->header.stamp;
      
      //this is the origintal PC, transformed to base_footprint frame and filtered (z and low intensity (water reflections) and adaptiveRadiusFilter)
      //RCLCPP_INFO( this->get_logger(), " after stamp:  %f n",  rclcpp::Time(cloud_msg->header.stamp).seconds());

      //filters point cloud into a laser scan - point coordinate filter for water reflections. A 2d laser scan is outputed, taking closest non-water point to the lidar, in each horizontal angle
      //short_scan_msg is a 2d laserscan of points in a close range. In this range, water reflections filtering is agressive due to there being more water reflections
      //  and IMU attitude without accumulated error because of long distance
      //long_scan_msg is a 2d laserscan of points in long range. In this range, small IMU attitude errors accumulate into big z coordinate diferences, so filtering for water reflections is much less agressive
      // merged_scan_msg merges short_scan_msg and long_scan_msg, taking closeste point from both of them, in each horizontal angle
      // merged_point_cloud has the same information as merged_scan_msg but in 3D Point cloud message format, instead of 2D laser scan. It still only has points in one horizonta plane

      params_->setUpParamsShortRange();
      auto short_scan_msg = PointCloudToLaserScanNode::computeLaserScan(cloud);
      params_->setUpParamsLongRange();
      auto long_scan_msg = PointCloudToLaserScanNode::computeLaserScan(cloud);
      auto merged_scan_msg = PointCloudToLaserScanNode::mergeLaserScans(short_scan_msg,  std::move(long_scan_msg));
      merged_scan_msg->header.stamp = cloud_msg->header.stamp;
      sensor_msgs::msg::PointCloud2 merged_point_cloud;

      Laser2PCprojector_.projectLaser(*merged_scan_msg, merged_point_cloud);  //https://wiki.ros.org/laser_geometry

      //Take the merged_scan_msg 2d laserscan and convert it into lower resolution radialMap. The resolution is defined by params_->angle_increment_, it is normally 12 measurements for a 360degree scan
      // for each angle interval, the closeste laserscan from merged_scan_msg is used as range for radialMap
      // radialMapVisual has the same information of radialMap but includes much higher resolution, effectively representing each of the ~12 segments as an arc of points, all with the same range defined in radialMap
      // radialMapVisual is mostly used for RVIZ visualization
      // radialMap is used to communicate regions where obstacles are to the USV's path controller

      auto [radialMap, radialMapVisual] = PointCloudToLaserScanNode::LaserScan2radialMap(merged_scan_msg);

      //project the filtered and transformed to base_footprint cloud into the 2D horizontal place: used to test clustering methods
      //accumulate the merged_point_cloud over a time horizon, similar to rviz time decay - usefull to get more features to cluster with
      //accumulate the projected clouds over the same time window: used to test clustering methods

      geometry_msgs::msg::TransformStamped world_fix_transform = 
          tf2_->lookupTransform(params_->fixed_frame_,params_->target_frame_, tf2::TimePointZero); //change this for real evolo 
      geometry_msgs::msg::TransformStamped inverse_world_fix_transform = 
          tf2_->lookupTransform(params_->target_frame_, params_->fixed_frame_,tf2::TimePointZero);//chang ethis for real_evolo
      pcl::PointCloud<pcl::PointXYZI> cloud_projected2D,cloud_projected2D_timeDecay, pcl_merged_cloud,laserScanPC_timeDecay;; 
      pcl::fromROSMsg(merged_point_cloud, pcl_merged_cloud); // causes [pointcloud_to_laserscan_node-2] Failed to find match for field 'intensity'.

      PointCloudToLaserScanNode::accumulate(pcl_merged_cloud, laserScanPC_timeDecay,rclcpp::Time(cloud_msg->header.stamp), params_->timeDecay_, world_fix_transform, inverse_world_fix_transform,clouds_queu_laserscan_);
      PointCloudToLaserScanNode::project(pcl_cloud_filtered ,cloud_projected2D);
      PointCloudToLaserScanNode::accumulate(cloud_projected2D, cloud_projected2D_timeDecay,rclcpp::Time(cloud_msg->header.stamp), params_->timeDecay_,world_fix_transform,inverse_world_fix_transform,clouds_queu_projectedPc_);

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
      pub_OriginalPC_->publish(std::move(*cloud));
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

void PointCloudToLaserScanNode::project(pcl::PointCloud<pcl::PointXYZI> & cloud_in,pcl::PointCloud<pcl::PointXYZI> & cloud_out_projected){

  cloud_out_projected = cloud_in;
  for (auto& pt : cloud_out_projected.points) {
         pt.z = 0.0f;
        }
  
  cloud_out_projected.width = cloud_out_projected.points.size();
  cloud_out_projected.height = 1;
  cloud_out_projected.is_dense = true;

}
/*
void PointCloudToLaserScanNode::accumulate(const pcl::PointCloud<pcl::PointXYZI> & cloud_in, pcl::PointCloud<pcl::PointXYZI> & cloud_out_accumulated,
    const rclcpp::Time & currentTime, const double & time_decay,geometry_msgs::msg::TransformStamped & world_fix_transform,geometry_msgs::msg::TransformStamped & inverse_world_fix_transform ,std::deque<TimedCloud> cloud_queue){ 
    // method receives Pcl:point clouds XYZI and converts them to XYZ to be able to call the main accumulate function
    pcl::PointCloud<pcl::PointXYZI> converted_cloud;
    converted_cloud.reserve(cloud_in.size());

    for (const auto& pt : cloud_in.points) {
        converted_cloud.emplace_back(pt.x, pt.y, pt.z);
    }
    converted_cloud.width = converted_cloud.size();
    converted_cloud.height = 1;
    converted_cloud.is_dense = cloud_in.is_dense;
    accumulate(converted_cloud,cloud_out_accumulated,currentTime,time_decay,world_fix_transform,inverse_world_fix_transform, cloud_queue);

    }*/

void PointCloudToLaserScanNode::accumulate(const pcl::PointCloud<pcl::PointXYZI> & cloud_in, pcl::PointCloud<pcl::PointXYZI> & cloud_out_accumulated,
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
bool PointCloudToLaserScanNode::detectWaterPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out){

    //ransac_range_candidates_ ;
    // ransac_height_candidates_;
    // ransac_threshold_inliers_;
    // ransac_filter_height_ ;
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

    // 2. Set up RANSAC plane segmentation
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true); // this makes the coeficients be the least squares of all inliers
    seg.setModelType(pcl::SACMODEL_PLANE); //modle to fit is a plane ax + by + cz + d = 0)
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(params_->ransac_threshold_inliers_);  // in meters
    seg.setInputCloud(ransac_candidates);
    seg.segment(*inliers, *coefficients); //performs ransac an Least Squares

    if (inliers->indices.size()<10)
    {
        std::cerr << "[detectGroundPlane] No plane inliers found.\n";
        return false;
    }

    // 3. Extract plane inliers
    /*pcl::ExtractIndices<pcl::PointXYZI> extract;
    plane_inliers_out.reset(new pcl::PointCloud<pcl::PointXYZI>());
    extract.setInputCloud(ransac_candidates);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane_inliers_out);*/

    pcl::PointCloud<pcl::PointXYZI>::Ptr outliers(new pcl::PointCloud<pcl::PointXYZI>);

    // Create the extractor
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(true);  // <-- This means extract *non-inliers* = outliers
    extract.filter(*outliers);

    // 4. Store plane coefficients: ax + by + cz + d = 0
    if (coefficients->values.size() == 4)
    {
        /*plane_coefficients_out = Eigen::Vector4f(
            coefficients->values[0],
            coefficients->values[1],
            coefficients->values[2],
            coefficients->values[3]);*/
        float a = coefficients->values[0];
        float b = coefficients->values[1];
        float c = coefficients->values[2];
        float d = coefficients->values[3];
        float threshold = 10.0; //planes more inclined than this will not be considered
        if (!checkWaterPlane(a,b,c,d, threshold)){
          return false;
          //RCLCPP_INFO(this->get_logger(), "Plane equation not valid");

        }
        publish_plane_marker( a, b, c, d);
        float plane_norm = std::sqrt(a*a + b*b + c*c);
        //d_planeFilter= (d/plane_norm) + params_->ransac_filter_height_;
        //RCLCPP_INFO( this->get_logger(), " Plane %f x + %f y + %fz +%f =0\n",  a, b, c, d);

        // We only keep points above surface of the water that do not bellong to it
        /*for (const auto& point : outliers->points) {
            // Compute plane equation value
            float plane_dist = a * point.x + b * point.y + c * point.z + d;

            // Keep point only if it's on or above the plane (>= 0)
            if (plane_dist > 0) {
                cloud_out->points.push_back(point);
            }*/
        //alternitevely we could:
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

    }else
    {
        std::cerr << "[detectGroundPlane] Invalid plane coefficients.\n";
    }
    //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());

    
    
    // Set metadata
    cloud_out->width = static_cast<uint32_t>(cloud_out->points.size());
    cloud_out->height = 1;
    cloud_out->is_dense = true;
    return true;

}

bool PointCloudToLaserScanNode::checkWaterPlane(float a, float b, float c, float d, float threshold_degrees)
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
/*
void PointCloudToLaserScanNode::filterCloud(pcl::PointCloud<pcl::PointXYZI> & cloud_in, const double & min_height, pcl::PointCloud<pcl::PointXYZI> & cloud_out
  ,pcl::PointCloud<pcl::PointXYZI> & cloud_outREJ){

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_ptr;
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(cloud_in, cloud_in, indices);

  if (params_->useRansac){

    bool ransacSuccess = detectWaterPlane(cloud_in, filtered_ptr)
    if (!ransacSuccess){filtered_ptr = &cloud_in}
  } else{filtered_ptr = &cloud_in}
  
  
  if(params_->filterBy_intensity_){
    //remove points that are under the water and with low intensity water
    

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
            if (pt.intensity >= intensity_threshold || pt.z >= min_height-3.0) { 
                filtered.points.push_back(pt);
            }
        }
    filtered.width = filtered.points.size();
    filtered.height = 1;
    filtered.is_dense = true;

    filtered_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(filtered); //filter by intensity
  }
  else{
    //do not use filter by intensity
    filtered_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(cloud_in);
 

  }

  
        // This uses standard outlier removal instead of addaptive radius filter
    //pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
    //outrem.setInputCloud(filtered_ptr);
    //outrem.setRadiusSearch(0.45);  // Radius for neighbor search 0.8
    //outrem.setMinNeighborsInRadius(2);  // Keep points with >= 3 neighbors 3
    //outrem.filter(cloud_out);
        
    
    
    
    //adaptive radius filter
    auto [output , rejected_output]= PointCloudToLaserScanNode::adaptiveRadiusFilter(filtered_ptr, params_->m_neighboursRadius_, params_->b_neighboursRadius_, params_->nr_neighbours_);
    cloud_out=*output;
    cloud_outREJ=*rejected_output;


  }*/


void PointCloudToLaserScanNode::filterCloud(
    pcl::PointCloud<pcl::PointXYZI> &cloud_in,
    const double &min_height,
    pcl::PointCloud<pcl::PointXYZI> &cloud_out,
    pcl::PointCloud<pcl::PointXYZI> &cloud_outREJ)
{
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(cloud_in, cloud_in, indices);
  // Use shared_ptr for all filtered stages
  pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(cloud_in);

  // 1. RANSAC plane detection (if enabled)
  if (params_->useRansac_) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr ransac_filtered(new pcl::PointCloud<pcl::PointXYZI>);
      bool ransacSuccess = detectWaterPlane(current_cloud, ransac_filtered);
      if (ransacSuccess) {
          current_cloud = ransac_filtered;
      }
      // If RANSAC fails, we just continue with current_cloud (still points to cloud_in)
  }

  // 2. Intensity filtering (if enabled)
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
  // 3. Adaptive radius filtering 
  if(!params_->simulation_mode_){
    auto [output , rejected_output]= PointCloudToLaserScanNode::adaptiveRadiusFilter(current_cloud, params_->m_neighboursRadius_, params_->b_neighboursRadius_, params_->nr_neighbours_);
    cloud_out=*output;
    cloud_outREJ=*rejected_output;
  }else{
    cloud_out=cloud_in;

  }


}
    




  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr , pcl::PointCloud<pcl::PointXYZI>::Ptr> PointCloudToLaserScanNode::adaptiveRadiusFilter(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
  float scale , float min_radius ,             // Radius = scale * range
  int min_neighbors )
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_rejectedcloud(new pcl::PointCloud<pcl::PointXYZI>);
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

sensor_msgs::msg::LaserScan::UniquePtr PointCloudToLaserScanNode::computeLaserScan(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg )
{
// build laserscan output
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

//angle_min += M_PI; //because the filtered point cloud frame is rotated by 180 degrees in relation to the cloud msg frame
//angle_max += M_PI;

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


sensor_msgs::msg::LaserScan::UniquePtr PointCloudToLaserScanNode::mergeLaserScans(
  const sensor_msgs::msg::LaserScan::UniquePtr & short_scan,
  sensor_msgs::msg::LaserScan::UniquePtr && long_scan)
  {


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
  PointCloudToLaserScanNode::LaserScan2radialMap( const sensor_msgs::msg::LaserScan::UniquePtr &scan)
  {
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



void PointCloudToLaserScanNode::publish_plane_marker(float a, float b, float c, float d)
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


}  // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::PointCloudToLaserScanNode)
