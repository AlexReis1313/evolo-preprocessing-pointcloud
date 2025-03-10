#ifndef VGICP_REGISTRATION_HPP
#define VGICP_REGISTRATION_HPP

#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/benchmark/read_points.hpp>

#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "pcl_ros/transforms.hpp"
#include <pcl_conversions/pcl_conversions.h>

using namespace small_gicp;

class vgicpRegistrationClass {
private:
    geometry_msgs::msg::TransformStamped ICP_output_transform_;
    RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> reg_;
    pcl::PointCloud<pcl::PointXYZ> last_cloud_, new_cloud_, new_cloud_transformed_;

public:
    void setLastCloud(const pcl::PointCloud<pcl::PointXYZ>& last_cloud);
    void setNewCloud(const pcl::PointCloud<pcl::PointXYZ>& new_cloud);
    void swapNewLastCloud();
    void computeRegistration();
    pcl::PointCloud<pcl::PointXYZ> getNewTransformedCloud() const;
    const geometry_msgs::msg::TransformStamped& getTransformation4NewCloud() const;
    bool firstTime_;

};

#endif // VGICP_REGISTRATION_HPP