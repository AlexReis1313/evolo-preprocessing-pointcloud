#include "occupancy_grid/occupancy_grid_node.h"




using namespace std::chrono_literals;
using std::placeholders::_1;
int size_of_map_=200; //m
double grid_size_=1.0;
OccupancyGridNode::OccupancyGridNode()
    : Node("occupancy_grid"), grid_map_{std::make_unique<OccupancyGrid>(size_of_map_, grid_size_)}
{
  // create publisher
  publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("ocg", 10);
  // create subscribers
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "footprint_odom", 10, std::bind(&OccupancyGridNode::handleOdom, this, std::placeholders::_1));
  laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "filtered/ls/laserscan", rclcpp::SensorDataQoS().keep_last(2),
      std::bind(&OccupancyGridNode::handleLaserScan, this, std::placeholders::_1));      
  robot_pose_inOCGMapFrame.setIdentity();

}

void OccupancyGridNode::handleOdom(const nav_msgs::msg::Odometry::SharedPtr odom)
{
 // RCLCPP_INFO(this->get_logger(), "Handling odometry data...");
  //std::cout << "handleOdom" << std::endl;

  // transform based on current and previous odom data


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

  /*double delta_x = odom->pose.pose.position.x - prev_odom_.pose.pose.position.x;
  double delta_y = odom->pose.pose.position.y - prev_odom_.pose.pose.position.y;
  tf2::Quaternion q_now, q_prev;
  tf2::fromMsg(odom->pose.pose.orientation, q_now);
  tf2::fromMsg(prev_odom_.pose.pose.orientation, q_prev);

  double yaw_now = tf2::getYaw(q_now);
  double yaw_prev = tf2::getYaw(q_prev);
  double delta_yaw = yaw_now - yaw_prev;


  double cos_prev_yaw = cos(yaw_prev);
  double sin_prev_yaw = sin(yaw_prev);
  
  // Rotate delta into robot frame
  double local_dx =  cos_prev_yaw * delta_y - sin_prev_yaw * delta_x;
  double local_dy = sin_prev_yaw * delta_y + cos_prev_yaw * delta_x;*/
  //local dx is negative, it should be positive, what is up?
  double translation_norm = tf_rel.getOrigin().length();

  if ( translation_norm > grid_size_*5){
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
    robot_pose_inOCGMapFrame.setIdentity();
  }else{
    robot_pose_inOCGMapFrame= tf_rel;
  }

 

  
}

void OccupancyGridNode::handleLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan)
{
  //RCLCPP_INFO(this->get_logger(), "Handling laser scan data...");

  // update grid based on new laser scan data
  std::vector<Point2d<double>> scan_cartesian = convertPolarScantoCartesianScan(laser_scan);
  grid_map_->update(scan_cartesian, robot_pose_inOCGMapFrame);
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