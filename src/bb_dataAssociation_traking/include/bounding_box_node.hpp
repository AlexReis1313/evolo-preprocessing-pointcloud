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
#include "kalmanFilter.hpp"
#include "Hungarian.h"
#include <vector>
#include <list>


//GLOBAL variables
int num_states = 10;
int num_sensors = 5;
extern Eigen::MatrixXd motion_model;
extern Eigen::MatrixXd measurement_model;


            
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

struct objectTracker
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr last_cluster;
    MinAreaRect rectangle;
    double height;
    KalmanFilter kf;
    Eigen::VectorXd costVector;
    FixedSizeQueue height_queue;
    bool updateStepKF = true;
    unsigned int ocludedCounter = 0;
    unsigned int newObjectCounter = 0;
    const int newObjectThreshold=5;
    const int pruneThreshold= 20;
    bool newObject = true;  //if true, the object is newly seen and not yet considered as existent. 
                            //Only when newObjectCounter>=newObjectThreshold, this bool=false and the object is considered for obstacle avoidance
      
    objectTracker():
        height_queue(100), //A class inside a struct needs to be first declared and then initialized in a constructor for the struct
        kf(num_states, num_sensors, motion_model, measurement_model) {}
    
};

class BoundingBoxNode : public rclcpp::Node {
public:
    BoundingBoxNode();

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void pointCloud3DBuffer(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void predictKalmanFilters(float currentTime);
    void defineHeight(objectTracker& object);
    void pca2DBoundingBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, visualization_msgs::msg::Marker& marker);
    void pubKfMarkerArrays(std::string frame_id);
    void updateKalmanFilters();
    void DataAssociate();
    std::vector<int> findMissingNumbers(const std::vector<int>& assignment, int m);
    void defineCosts(objectTracker& object);
    double costFuntion(const objectTracker& object, const objectTracker& trackedObject);



    
    MinAreaRect rotatingCaliper2DBoundingBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, visualization_msgs::msg::Marker& marker);
    std::vector<Point> convertPCLCloudToCalipersInput(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    std::list<objectTracker> trackedObjectsList;
    std::list<objectTracker> toEraseObjectsList;
    std::list<objectTracker> currentObjectsList;
	HungarianAlgorithm HungAlgo;
	std::vector<int> assignment_;
    std::vector< std::vector<double> > costMatrix_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud2D_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud3D_sub_;
    sensor_msgs::msg::PointCloud2::SharedPtr last_3Dcloud_;
    bool has_received_3dcloud_=false;
    bool draw_height_=false;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr kf_bbox_pub_;

};

#endif // BOUNDING_BOX_NODE_HPP
