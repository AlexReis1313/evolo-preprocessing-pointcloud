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

#include "message_filters/subscriber.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "pcl_ros/transforms.hpp"
#include <pcl/filters/filter.h>
#include <opencv2/core/types.hpp>







//GLOBAL variables
int num_states = 10;
int num_sensors = 5;
float metric_time_horizon = 3.0;
double acell_cov =0.5;
double pose_cov = 0.05;
double cost_threshold_ = 15;
extern Eigen::MatrixXd motion_model;
extern Eigen::MatrixXd measurement_model;
extern Eigen::MatrixXd process_noise;


            
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

struct TimedPrediction {
        double predicted_time;
        Eigen::VectorXd predicted_state;
    };

struct objectTracker
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr last_cluster;
    MinAreaRect rectangle;
    double height;
    KalmanFilter kf;
    std::vector<double> costVector;
    FixedSizeQueue height_queue;
    bool updateStepKF = true;
    unsigned int ocludedCounter = 0;
    unsigned int newObjectCounter = 0;
    int newObjectThreshold=5;
    int pruneThreshold= 20;
    bool newObject = true;  //if true, the object is newly seen and not yet considered as existent. 
                            //Only when newObjectCounter>=newObjectThreshold, this bool=false and the object is considered for obstacle avoidance
    std::deque<TimedPrediction> future_predictions;

    objectTracker():
        height_queue(100), //A class inside a struct needs to be first declared and then initialized in a constructor for the struct
        kf(num_states, num_sensors, motion_model, measurement_model, process_noise,acell_cov ,pose_cov) {}
    
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
    void initiateTrackedObjects();
    void computeMetrics(double& currentTime);
    TimedPrediction getPrediction(objectTracker& object,double& currentTime);
    float computeIoU(float x1, float y1, float len1, float wid1, float angle1,
        float x2, float y2, float len2, float wid2, float angle2);

    std::vector<int> findMissingNumbers(const std::vector<int>& assignment, int m);
    void defineCosts(objectTracker& object);
    double costFuntion(const objectTracker& object, const objectTracker& trackedObject);



    
    MinAreaRect rotatingCaliper2DBoundingBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, visualization_msgs::msg::Marker& marker);
    std::vector<Point> convertPCLCloudToCalipersInput(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    std::vector<objectTracker> trackedObjectsList;
    std::vector<objectTracker> toEraseObjectsList;
    std::vector<objectTracker> currentObjectsList;
	HungarianAlgorithm HungAlgo;
	std::vector<int> assignment_;
    std::vector< std::vector<double> > costMatrix_;

    std::unique_ptr<tf2_ros::Buffer> tf2_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud2D_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud3D_sub_;
    sensor_msgs::msg::PointCloud2::SharedPtr last_3Dcloud_;
    bool has_received_3dcloud_=false;
    bool draw_height_=false;
    std::string fixed_frame_="map_gt";

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr kf_bbox_pub_;

};

#endif // BOUNDING_BOX_NODE_HPP
