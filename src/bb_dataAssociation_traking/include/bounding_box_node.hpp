#ifndef BOUNDING_BOX_NODE_HPP
#define BOUNDING_BOX_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <eigen3/Eigen/Dense>
#include <unordered_map>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "RotatingCalipers.h"
#include <deque>
#include "pcl_ros/transforms.hpp"


struct KalmanFilter
{
    double x;
    double y;
    double velx;
    double vely;


};

class FixedSizeQueue {
    public:
        FixedSizeQueue(size_t max_size) : max_size_(max_size) {}
    
        void push(double value) {
            if (buffer_.size() >= max_size_) {
                buffer_.pop_front();  // Remove oldest element
            }
            buffer_.push_back(value);  // Add new element
        }
    
        void print() const {
            for (double num : buffer_) {
                std::cout << num << " ";
            }
            std::cout << std::endl;
        }
    
    private:
        std::deque<double> buffer_;
        size_t max_size_;
    };

struct trackedObject
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr last_cluster;
    MinAreaRect rectangle;
    double height;
    // KalmanFilter filter;
    FixedSizeQueue height_queue;
    trackedObject(): height_queue(100){}; //A class inside a struct needs to be first declared and then initialized in a constructor for the struct
};

class BoundingBoxNode : public rclcpp::Node {
public:
    BoundingBoxNode();

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void pointCloud3DBuffer(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void defineHeight(trackedObject& object);
    void pca2DBoundingBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, visualization_msgs::msg::Marker& marker);
    MinAreaRect rotatingCaliper2DBoundingBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, visualization_msgs::msg::Marker& marker);
    std::vector<Point> convertPCLCloudToCalipersInput(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    std::list<trackedObject> trackedObjectsList;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud2D_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud3D_sub_;
    sensor_msgs::msg::PointCloud2::SharedPtr last_3Dcloud_;
    bool has_received_3dcloud_=false;
    bool draw_height_=false;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_pub_;
};

#endif // BOUNDING_BOX_NODE_HPP
