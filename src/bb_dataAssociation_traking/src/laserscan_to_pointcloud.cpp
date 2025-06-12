
//this a helper function taken from the pointcloud_to_laserscan ros2 package, that is also used in the pointcloud_preprocess node

//parameters to output laserscan
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <functional>
#include <limits>
#include <utility>
#include <cmath>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"



double angle_min_ = -3.14159;
double angle_max_ = 3.14159;
double angle_increment_=0.003067;  // M_PI/360.0
double scan_time_=0.1;
double range_min_laserscan_ = 0;
double range_max_laserscan_=3000;
bool use_inf_ = true;
sensor_msgs::msg::LaserScan::UniquePtr computeLaserScan(
const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg )
{
    // build laserscan output
    auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
    scan_msg->header = cloud_msg->header;


    scan_msg->angle_min = angle_min_;
    scan_msg->angle_max = angle_max_;
    scan_msg->angle_increment = angle_increment_;
    scan_msg->time_increment = 0.0;
    scan_msg->scan_time = scan_time_;
    scan_msg->range_min = range_min_laserscan_;
    scan_msg->range_max = range_max_laserscan_;

    //angle_min += M_PI; //because the filtered point cloud frame is rotated by 180 degrees in relation to the cloud msg frame
    //angle_max += M_PI;

    // determine amount of rays to create
    uint32_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

    // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
    if (use_inf_) {
        scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());

    } else {
        scan_msg->ranges.assign(ranges_size, scan_msg->range_max );
    }
    scan_msg->intensities.assign(ranges_size, 0.0f); 

    // Iterate through pointcloud
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
    iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z"), iter_intensity(*cloud_msg, "intensity");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z,++iter_intensity)
    {
        if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
        continue;
        }


        double range = hypot(*iter_x, *iter_y);
        if (range < scan_msg->range_min) {
        continue;
        }
        if (range > scan_msg->range_max) {

        continue;
        }

        double angle = atan2(*iter_y, *iter_x);
        if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {

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