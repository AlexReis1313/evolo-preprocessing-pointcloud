#include "filtering_params.hpp"

namespace pointcloud_to_laserscan
{

PointCloudToLaserScanParams::PointCloudToLaserScanParams(rclcpp::Node* node)
{
    // Frame parameters
    baseLink_frame_ = node->declare_parameter("base_link", "base_link");
    target_frame_ = node->declare_parameter("target_frame", "");
    fixed_frame_ = node->declare_parameter("fixed_frame", "");
    cloud_frame_ = node->declare_parameter("cloud_frame", "");
    
    // Transform and queue parameters
    tolerance_ = node->declare_parameter("transform_tolerance", 0.01);
    input_queue_size_ = node->declare_parameter(
        "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
    simulation_mode_ = node->declare_parameter("simulation_mode", false);
    
    // Height range parameters (short range)
    min_height_shortrange_ = node->declare_parameter(
        "min_height_shortrange", std::numeric_limits<double>::min());
    max_height_shortrange_ = node->declare_parameter(
        "max_height_shortrange", std::numeric_limits<double>::max());
    
    // Height range parameters (long range)
    min_height_longrange_ = node->declare_parameter(
        "min_height_longrange", std::numeric_limits<double>::min());
    max_height_longrange_ = node->declare_parameter(
        "max_height_longrange", std::numeric_limits<double>::max());
    
    // Angle parameters
    angle_min_ = node->declare_parameter("angle_min_laserscan", -M_PI);
    angle_max_ = node->declare_parameter("angle_max_laserscan", M_PI);
    angle_min_map_ = node->declare_parameter("angle_min_map", -M_PI/2);
    angle_max_map_ = node->declare_parameter("angle_max_map", M_PI/2);
    angle_increment_ = node->declare_parameter("angle_increment", M_PI / 180.0);
    angle_visual_outputmap_ = node->declare_parameter(
        "angle_increment_output_map", M_PI / 180.0);
    
    // Scan parameters
    scan_time_ = node->declare_parameter("scan_time", 1.0 / 30.0);
    range_min_ = node->declare_parameter("range_min", 0.0);
    range_max_ = node->declare_parameter("range_max", std::numeric_limits<double>::max());
    range_transition_ = node->declare_parameter("range_transition", 0.0);
    inf_epsilon_ = node->declare_parameter("inf_epsilon", 1.0);
    use_inf_ = node->declare_parameter("use_inf", true);
    
    // Radius search parameters
    b_neighboursRadius_ = node->declare_parameter("minimum_radius_paramB", 0.05);
    m_neighboursRadius_ = node->declare_parameter("minimum_radius_paramM", 0.0125);
    nr_neighbours_ = node->declare_parameter("minimum_neighbours", 3);
    filterBy_intensity_= node->declare_parameter("filter_by_intensity", false);

    //time decay to accumulate pc
    timeDecay_= node->declare_parameter("time_decay", 2.0);
    outputOnFixedFrame_ = node->declare_parameter("TimeDecay_output_On_Fixed_Frame", false);

    //ransac parameters
    ransac_range_candidates_ =node->declare_parameter("ransac_range_candidates", 30.0);
    ransac_Maxheight_candidates_=node->declare_parameter("ransac_Maxheight_candidates", 1.0);
    ransac_Minheight_candidates_=node->declare_parameter("ransac_Minheight_candidates", -1.5);
    useRansac_=node->declare_parameter("use_Ransac", false);
    ransac_threshold_inliers_=node->declare_parameter("ransac_threshold_inliers", 0.3);
    ransac_filter_height_ =node->declare_parameter("ransac_filter_height", 0.5);

}

void PointCloudToLaserScanParams::setUpParamsShortRange(){
// this prepares the parameters to compute a laserscan for short ranges
    min_height_ = min_height_shortrange_;
    max_height_ = max_height_shortrange_;
    range_min_laserscan_ = range_min_;
    range_max_laserscan_ = range_transition_;


}

void PointCloudToLaserScanParams::setUpParamsLongRange(){

    min_height_ = min_height_longrange_;
    max_height_ = max_height_longrange_;
    range_min_laserscan_ = range_transition_;
    range_max_laserscan_ = range_max_;
    
}

}  // namespace pointcloud_to_laserscan