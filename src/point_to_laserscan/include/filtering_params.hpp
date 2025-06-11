#ifndef POINTCLOUD_PREPROCESSING__FILTERING_PARAMS_HPP_
#define POINTCLOUD_PREPROCESSING__FILTERING_PARAMS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <limits>
#include <cmath>
#include <thread>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/node.hpp"  
#include "rclcpp/parameter.hpp"




namespace pointcloud_preprocessing
{

class PointCloudPreProcessingParams
{
public:

    // Constructor that declares and initializes all parameters
    explicit PointCloudPreProcessingParams(rclcpp::Node* node);
    void setUpParamsShortRange();
    void setUpParamsLongRange ();

    //params to compute laserscans
    double min_height_;
    double max_height_;
    double range_min_laserscan_;
    double range_max_laserscan_;

    
    // Frame parameters
    std::string baseLink_frame_;
    std::string target_frame_;
    std::string fixed_frame_;
    std::string cloud_frame_;
    
    // Transform and queue parameters
    double tolerance_;
    int input_queue_size_;
    bool simulation_mode_;
    
    // Height range parameters
    double min_height_shortrange_;
    double max_height_shortrange_;
    double min_height_longrange_;
    double max_height_longrange_;
    
    // Angle parameters
    double angle_min_;
    double angle_max_;
    double angle_min_map_;
    double angle_max_map_;
    double angle_increment_;
    double angle_visual_outputmap_;
    
    // Scan parameters
    double scan_time_;
    double range_min_;
    double range_max_;
    double range_transition_;
    double inf_epsilon_;
    bool use_inf_;
    
    // Radius search parameters
    double b_neighboursRadius_;
    double m_neighboursRadius_;
    int nr_neighbours_;
    bool filterBy_intensity_;

    //time decay to accumulate pc
    double timeDecay_;
    bool outputOnFixedFrame_;

    //ransac params
    double ransac_range_candidates_ ;
    double ransac_Maxheight_candidates_;
    double ransac_threshold_inliers_;
    double ransac_filter_height_ ;
    double ransac_Minheight_candidates_;
    bool useRansac_ = true;

    // to print time of each computation
    bool timeMetric;
    
    rclcpp::Node* node_;
    rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter>& parameters);


};

}  // namespace pointcloud_to_laserscan

#endif  // POINTCLOUD_PREPROCESSING__FILTERING_PARAMS_HPP_