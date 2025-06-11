#include "filtering_params.hpp"

namespace pointcloud_preprocessing
{

PointCloudPreProcessingParams::PointCloudPreProcessingParams(rclcpp::Node* node)
{
    timeMetric = node->declare_parameter("print_time_metric", false);

    // Frame parameters
    baseLink_frame_ = node->declare_parameter("base_link", "base_link");
    target_frame_ = node->declare_parameter("target_frame", "base_footprint");
    fixed_frame_ = node->declare_parameter("fixed_frame", "odom");
    cloud_frame_ = node->declare_parameter("cloud_frame", "os_sensor");
    
    // Transform and queue parameters
    tolerance_ = node->declare_parameter("transform_tolerance", 0.01);
    input_queue_size_ = node->declare_parameter(
        "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
    simulation_mode_ = node->declare_parameter("simulation_mode", false);
    
    // Height range parameters (short range)
    min_height_shortrange_ = node->declare_parameter(
        "min_height_shortrange", -0.2);
    max_height_shortrange_ = node->declare_parameter(
        "max_height_shortrange", 4.0);
    
    // Height range parameters (long range)
    min_height_longrange_ = node->declare_parameter(
        "min_height_longrange", -6.0);
    max_height_longrange_ = node->declare_parameter(
        "max_height_longrange", 6.0);
    
    // Angle parameters
    angle_min_ = node->declare_parameter("angle_min_laserscan", -M_PI);
    angle_max_ = node->declare_parameter("angle_max_laserscan", M_PI);
    angle_min_map_ = node->declare_parameter("angle_min_map", -M_PI/2);
    angle_max_map_ = node->declare_parameter("angle_max_map", M_PI/2);

    angle_increment_ = node->declare_parameter("angle_increment", M_PI / 360.0);
    angle_visual_outputmap_ = node->declare_parameter(
        "angle_increment_output_map", M_PI / 6.0);
    
    // Scan parameters
    scan_time_ = node->declare_parameter("scan_time", 0.1);
    range_min_ = node->declare_parameter("range_min", 1.0);
    range_max_ = node->declare_parameter("range_max", std::numeric_limits<double>::max());
    range_transition_ = node->declare_parameter("range_transition", 30.0);
    inf_epsilon_ = node->declare_parameter("inf_epsilon", 1.0); //when use_inf is false, max range will be reported as max_range +inf_epsilon_
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

void PointCloudPreProcessingParams::setUpParamsShortRange(){
// this prepares the parameters to compute a laserscan for short ranges
    min_height_ = min_height_shortrange_;
    max_height_ = max_height_shortrange_;
    range_min_laserscan_ = range_min_;
    range_max_laserscan_ = range_transition_;


}

void PointCloudPreProcessingParams::setUpParamsLongRange(){

    min_height_ = min_height_longrange_;
    max_height_ = max_height_longrange_;
    range_min_laserscan_ = range_transition_;
    range_max_laserscan_ = range_max_;
    
}
rcl_interfaces::msg::SetParametersResult PointCloudPreProcessingParams::onParameterChange(
    const std::vector<rclcpp::Parameter>& parameters)
{
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    
    for (const auto& param : parameters)
    {
        const std::string& name = param.get_name();

        try
        {
            if (name == "base_link") baseLink_frame_ = param.as_string();
            else if (name == "target_frame") target_frame_ = param.as_string();
            else if (name == "fixed_frame") fixed_frame_ = param.as_string();
            else if (name == "cloud_frame") cloud_frame_ = param.as_string();
            else if (name == "transform_tolerance") tolerance_ = param.as_double();
            else if (name == "queue_size") input_queue_size_ = param.as_int();
            else if (name == "simulation_mode") simulation_mode_ = param.as_bool();
            else if (name == "min_height_shortrange") min_height_shortrange_ = param.as_double();
            else if (name == "max_height_shortrange") max_height_shortrange_ = param.as_double();
            else if (name == "min_height_longrange") min_height_longrange_ = param.as_double();
            else if (name == "max_height_longrange") max_height_longrange_ = param.as_double();
            else if (name == "angle_min_laserscan") angle_min_ = param.as_double();
            else if (name == "angle_max_laserscan") angle_max_ = param.as_double();
            else if (name == "angle_min_map") angle_min_map_ = param.as_double();
            else if (name == "angle_max_map") angle_max_map_ = param.as_double();
            else if (name == "angle_increment") angle_increment_ = param.as_double();
            else if (name == "angle_increment_output_map") angle_visual_outputmap_ = param.as_double();
            else if (name == "scan_time") scan_time_ = param.as_double();
            else if (name == "range_min") range_min_ = param.as_double();
            else if (name == "range_max") range_max_ = param.as_double();
            else if (name == "range_transition") range_transition_ = param.as_double();
            else if (name == "inf_epsilon") inf_epsilon_ = param.as_double();
            else if (name == "use_inf") use_inf_ = param.as_bool();
            else if (name == "minimum_radius_paramB") b_neighboursRadius_ = param.as_double();
            else if (name == "minimum_radius_paramM") m_neighboursRadius_ = param.as_double();
            else if (name == "minimum_neighbours") nr_neighbours_ = param.as_int();
            else if (name == "filter_by_intensity") filterBy_intensity_ = param.as_bool();
            else if (name == "time_decay") timeDecay_ = param.as_double();
            else if (name == "TimeDecay_output_On_Fixed_Frame") outputOnFixedFrame_ = param.as_bool();
            else if (name == "ransac_range_candidates") ransac_range_candidates_ = param.as_double();
            else if (name == "ransac_Maxheight_candidates") ransac_Maxheight_candidates_ = param.as_double();
            else if (name == "ransac_Minheight_candidates") ransac_Minheight_candidates_ = param.as_double();
            else if (name == "use_Ransac") useRansac_ = param.as_bool();
            else if (name == "ransac_threshold_inliers") ransac_threshold_inliers_ = param.as_double();
            else if (name == "ransac_filter_height") ransac_filter_height_ = param.as_double();
            else {
                // Optional: reject unknown parameters
                RCLCPP_WARN(rclcpp::get_logger("ParamMonitor"), "Unknown parameter changed: %s", name.c_str());
            }
        }
        catch (const rclcpp::ParameterTypeException &e)
        {
            result.successful = false;
            result.reason = "Failed to set parameter '" + name + "': " + std::string(e.what());
            return result;
        }
    }

    return result;
}


}  // namespace pointcloud_to_laserscan