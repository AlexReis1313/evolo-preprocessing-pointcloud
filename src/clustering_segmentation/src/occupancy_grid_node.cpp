#include "occupancy_grid/occupancy_grid_node.h"
#include "timing_metrics.cpp"




using namespace std::chrono_literals;
using std::placeholders::_1;
int size_of_map_=200; //m
double grid_size_=2.0;


OccupancyGridNode::OccupancyGridNode()
    : Node("occupancy_grid"),  grid_map_{std::make_unique<OccupancyGrid>(size_of_map_, grid_size_)}, grid_map_static_{std::make_unique<OccupancyGrid>(size_of_map_, grid_size_)}, clustering{this->create_publisher<sensor_msgs::msg::PointCloud2>("clustered_points", 10)}
{

  this->paramLaunch();
  std::cout <<"Created all nodes"<<std::endl;
  // create publisher
  publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancyMap", 10);

  // create subscribers
  if(clustering_bool){
    sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_in_topic, rclcpp::SensorDataQoS().keep_last(1), std::bind(&OccupancyGridNode::lidarCallback, this, std::placeholders::_1));
  }

  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("footprint_odom", 10, std::bind(&OccupancyGridNode::handleOdom, this, std::placeholders::_1));
  
  laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>( "filtered/ls/laserscan", rclcpp::SensorDataQoS().keep_last(2), std::bind(&OccupancyGridNode::handleLaserScan, this, std::placeholders::_1));      

  if (staticMapping){
      laser_scan_static_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>( staticPoints, rclcpp::SensorDataQoS().keep_last(2), std::bind(&OccupancyGridNode::handleStaticLaserScan, this, std::placeholders::_1));      
      publisher_static_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancyMap/Static", 10);

  }

  robot_pose_inOCGMapFrame.setIdentity();

  if(saveTimeMetric_){
        timeoutFile_.open(timing_file, std::ios::app);
        //std::ofstream timeoutFile_(timing_file, std::ios::app); // append mode

        if (!timeoutFile_.is_open()) {
            RCLCPP_WARN(this->get_logger(), "Failed to open %s for writing.", timing_file.c_str());
            saveTimeMetric_ = false;
        } else{
            timeoutFile_ << "\n\nNode was restarted\n";

        }
            
    }


}
void OccupancyGridNode::paramLaunch(){

    clustering_bool = this->declare_parameter("clustering", false);
    DynamicStatic_segmentation = this->declare_parameter("DynamicStatic_clusters_segmentation", false);

    staticMapping = this->declare_parameter("static_mapping", false);
    staticPoints = this->declare_parameter("static_points_topic", std::string("static/laserscan"));
    points_in_topic = this->declare_parameter("clustering_points_topic_in", std::string("filtered/ls/pointcloud/accumulated"));
    points_out_topic = this->declare_parameter("clustering_points_topic_out", std::string("clustered_points"));
    timeMetric = this->declare_parameter("PrintTimeMetric", false);
    saveTimeMetric_ = this->declare_parameter("SaveTimeMetric", false);
    timing_file = this->declare_parameter("timingMetrics_file", std::string("Segmentation_timingFile.csv"));

    RCLCPP_INFO(this->get_logger(), "Parameters loaded.");
}

void OccupancyGridNode::handleOdom(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  //std::cout << "handleOdom" << std::endl;

  // transform based on current and previous odom data
  ScopedTimer handleodom_timer("[segmentation], HandleOdom",this, timeMetric,saveTimeMetric_,timeoutFile_ );


  tf2::Vector3 pos_prev(
    prev_odom_.pose.pose.position.x,
    prev_odom_.pose.pose.position.y,
    0.0
  );
  tf2::Quaternion q_prev;
  tf2::fromMsg(prev_odom_.pose.pose.orientation, q_prev);
  tf2::Transform tf_prev(q_prev, pos_prev);

  // Convert current pose to tf2::Transform
  tf2::Vector3 pos_now(
    odom->pose.pose.position.x,
    odom->pose.pose.position.y,
    0.0
  );
  tf2::Quaternion q_now;
  tf2::fromMsg(odom->pose.pose.orientation, q_now);
  tf2::Transform tf_now(q_now, pos_now);

  // Compute relative transform: T_rel = T_prev^-1 * T_now
  tf2::Transform tf_rel = tf_prev.inverse() * tf_now;

  
  double translation_norm = tf_rel.getOrigin().length();

  if ( translation_norm > grid_size_*4){
    // Extract translation and yaw from tf_rel
    tf2::Vector3 translation = tf_rel.getOrigin();
    tf2::Quaternion rotation = tf_rel.getRotation();
    double roll, pitch, yaw;
    tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
    grid_map_->update(translation.x(), translation.y(), yaw); 
    // update previous odometry data
    prev_odom_ = *odom;
    // fill msg and publish grid
    auto message = nav_msgs::msg::OccupancyGrid();
    grid_map_->toRosMsg(message,robot_pose_inOCGMapFrame );
    publisher_->publish(message);

    if (staticMapping){

      grid_map_static_->update(translation.x(), translation.y(), yaw); 
      auto message = nav_msgs::msg::OccupancyGrid();
      grid_map_static_->toRosMsg(message,robot_pose_inOCGMapFrame );
      publisher_static_map_->publish(message);
    }
    robot_pose_inOCGMapFrame.setIdentity();

  }else{
    robot_pose_inOCGMapFrame= tf_rel;
  }
}

void OccupancyGridNode::lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg){
  ScopedTimer lidarCallback_timer("[segmentation], Clustering",this, timeMetric,saveTimeMetric_,timeoutFile_ );

  clustering.lidarAndMapCallback(input_msg,  grid_map_, robot_pose_inOCGMapFrame, DynamicStatic_segmentation);
  
}



void OccupancyGridNode::handleStaticLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan)
{
  //RCLCPP_INFO(this->get_logger(), "Handling laser scan data...");
  ScopedTimer laserscanCallback_timer("[segmentation], Update Static Map",this, timeMetric,saveTimeMetric_,timeoutFile_ );

  // update grid based on new laser scan data
  std::vector<Point2d<double>> scan_cartesian = convertPolarScantoCartesianScan(laser_scan);
  bool bayesFilterSelector=false;
  grid_map_static_->update(scan_cartesian, robot_pose_inOCGMapFrame, bayesFilterSelector); //false selectes the bayesian filter to be used to update cell values - one where occupancy is fast to update and free cell status takes some time to be achieved
  ScopedTimer sweping_timer("[segmentation], Sweping to smoth map",this, timeMetric,saveTimeMetric_,timeoutFile_ );
  grid_map_static_->fillFreeBetweenOccupied();

  // fill msg and publish grid
  auto message = nav_msgs::msg::OccupancyGrid();
  grid_map_static_->toRosMsg(message,robot_pose_inOCGMapFrame);
  message.header.stamp = laser_scan->header.stamp;
  message.header.frame_id ="base_footprint";

  publisher_static_map_->publish(message);
}

void OccupancyGridNode::handleLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan)
{
  //RCLCPP_INFO(this->get_logger(), "Handling laser scan data...");

  ScopedTimer laserscanCallback_timer("[segmentation], Update Map",this, timeMetric,saveTimeMetric_,timeoutFile_ );

  // update grid based on new laser scan data
  std::vector<Point2d<double>> scan_cartesian = convertPolarScantoCartesianScan(laser_scan);
  grid_map_->update(scan_cartesian, robot_pose_inOCGMapFrame, DynamicStatic_segmentation);//DynamicStatic_segmentation selectes the bayesian filter to be used to update cell values
  ScopedTimer sweping_timer("[segmentation], Sweping to smoth map",this, timeMetric,saveTimeMetric_,timeoutFile_ );
  grid_map_->fillFreeBetweenOccupied();

  // fill msg and publish grid
  auto message = nav_msgs::msg::OccupancyGrid();
  grid_map_->toRosMsg(message,robot_pose_inOCGMapFrame);
  message.header.stamp = laser_scan->header.stamp;
  message.header.frame_id ="base_footprint";

  publisher_->publish(message);
}

std::vector<Point2d<double>> OccupancyGridNode::convertPolarScantoCartesianScan(
    const sensor_msgs::msg::LaserScan::SharedPtr laser_scan)
{
  ScopedTimer polartocartesian("[segmentation], polar to cartesian",this, timeMetric,saveTimeMetric_,timeoutFile_ );

  std::vector<Point2d<double>> scan_cartesian;
  scan_cartesian.reserve(laser_scan->ranges.size());
  float angle = laser_scan->angle_min;
  Point2d<double> cartesian_point;
  for (float range : laser_scan->ranges) {
    if (range == std::numeric_limits<double>::infinity() ){
      range=size_of_map_; 
    }
    cartesian_point.x = range * cos(angle);
    cartesian_point.y = range * sin(angle);
    scan_cartesian.push_back(cartesian_point);
    angle += laser_scan->angle_increment;
  }
  return scan_cartesian;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGridNode>());
  rclcpp::shutdown();
  return 0;
}