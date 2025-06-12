///////////////////////////////////////////////////////////////////////////////
// bounding_box_node.hpp: Header file for Class BoundingBoxNode.
// 
// by Alexandre Reis, 2025
// 

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
#include "sensor_msgs/msg/laser_scan.hpp"

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
#include <opencv2/imgproc.hpp> 
#include "tf2_eigen/tf2_eigen.hpp"
#include <pcl/common/transforms.h>





//GLOBAL variables
const int num_states = 10;
const int num_sensors = 5;
float metric_time_horizon = 3.0;
extern Eigen::MatrixXd motion_model;
extern Eigen::MatrixXd measurement_model;
extern Eigen::MatrixXd process_noise;
//Tunning parameters
double acell_cov_R =0.5; //R matrix is proporcional to this value and dt - used as motion model noise cov - PROCESS NOISE
double pose_cov_Q = 0.3; //Q matrix is proporcional to this value - measurement covariance of pose states
double boundingBox_cov_Q = 6.0; //Q matrix is proporcional to this value - measurement covariance of bounding box states
double min_velocity_threshold_ = 1.3; //m/s
int newObjectThreshold_ = 15;   //number of times an object has to be seen before tracker output starts
double cost_threshold_ = 6;   //cost threshold to associate cluster to object                
double cov_limit_factor_=50;   // if a tracked object has more cov than this, it will be deleted
int pruneThreshold_ = 35; //if an object is not seen for 40 consecutive point clouds, it will be deleted - this leavs ~4seconds where ocluded objects get propagated
bool save_metrics_txt_ = false;
std::string metrics_file = "boundingBoxMetrics.txt";
std::string fixed_frame_ = "odom";
bool timeMetric_ = true;




struct TimedPrediction {
        rclcpp::Time predicted_time;
        Eigen::VectorXd predicted_state;
    };

struct CovarianceInfo {
    Eigen::Matrix2f covariance;
    Eigen::Matrix2f inverse;
    Eigen::Vector2f meanxy;
    float determinant;
};
struct objectTracker
{
    int id = -1;//undefined
    pcl::PointCloud<pcl::PointXYZI>::Ptr last_cluster;
    MinAreaRect rectangle;
    KalmanFilter kf;
    std::vector<double> costVector;
    CovarianceInfo covInfo;
    bool hasPublished_last_cluster = false;
    bool updateStepKF = true;
    unsigned int ocludedCounter = 0;
    unsigned int newObjectCounter = 0;
    int newObjectThreshold=newObjectThreshold_;
    int pruneThreshold= pruneThreshold_;
    bool newObject = true;  //if true, the object is newly seen and not yet considered as existent. 
                            //Only when newObjectCounter>=newObjectThreshold, this bool=false and the object is considered for obstacle avoidance
    std::deque<TimedPrediction> future_predictions;

    objectTracker(double x_init, double y_init):
        kf(x_init,y_init,num_states, num_sensors, motion_model, measurement_model, process_noise,acell_cov_R ,pose_cov_Q,boundingBox_cov_Q , cov_limit_factor_) {}
    
};




class BoundingBoxNode : public rclcpp::Node {
public:
    BoundingBoxNode();
    ~BoundingBoxNode(); 

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void predictKalmanFilters(rclcpp::Time currentTime);
    void defineHeight(objectTracker& object);
    void pca2DBoundingBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, visualization_msgs::msg::Marker& marker);
    void pubKfMarkerArrays(std::string frame_id);
    void updateKalmanFilters();
    void DataAssociate();
    void initiateTrackedObjects();
    void computeMetrics(rclcpp::Time& currentTime);
    void correctBBorientation(objectTracker& trackedObject);
    void saveMetricsTxt(const objectTracker& trackedObject);
    void publishNonTrackedPC(std::string frame_id, rclcpp::Time stamp, geometry_msgs::msg::TransformStamped transform_stamped);
    double costFuntion_VANILA(const objectTracker& object, const objectTracker& trackedObject);
    double costFuntion_IOU(const objectTracker& object, const objectTracker& trackedObject);
    double costFuntion_BACHY(const objectTracker& object, const objectTracker& trackedObject);
    double costFuntion_BACHY_covBB(const objectTracker& object, const objectTracker& trackedObject);
    double costFuntion_BACHY_IOU(const objectTracker& object, const objectTracker& trackedObject);
    double costFuntion_BACHY_IOU_eucledian(const objectTracker& object, const objectTracker& trackedObject);

    Eigen::Matrix2f approximateCovarianceFromBoundingBox(float width, float length, float heading);
    CovarianceInfo computeCovarianceInfo(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void pupBBMarkerArray(std::string frame_id);


    TimedPrediction getPrediction(objectTracker& object,rclcpp::Time& currentTime);
    float computeIoU(float x1, float y1, float len1, float wid1, float angle1,
        float x2, float y2, float len2, float wid2, float angle2);

    std::vector<int> findMissingNumbers(const std::vector<int>& assignment, int m);
    void defineCosts(objectTracker& object);


    rclcpp::Time last_iteration_time_;
    MinAreaRect rotatingCaliper2DBoundingBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, visualization_msgs::msg::Marker& marker);
    std::vector<Point> convertPCLCloudToCalipersInput(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    
    std::vector<objectTracker> trackedObjectsList;
    std::vector<objectTracker> toEraseObjectsList;
    std::vector<objectTracker> currentObjectsList;
	HungarianAlgorithm HungAlgo;
	std::vector<int> assignment_;
    std::vector< std::vector<double> > costMatrix_;
    std::ofstream outfile_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr nonTrackedPc_;

    std::unique_ptr<tf2_ros::Buffer> tf2_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud2D_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud3D_sub_;
    sensor_msgs::msg::PointCloud2::SharedPtr last_3Dcloud_;
    bool has_received_3dcloud_=false;
    unsigned int id_counter_ = 0;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>>  pub_NonTRacked_pc_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr kf_bbox_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corrected_bbox_pub_;


    

};

#endif // BOUNDING_BOX_NODE_HPP
