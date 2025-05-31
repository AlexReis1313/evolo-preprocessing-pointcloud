#ifndef POINTCLOUD_TO_LASERSCAN__FILTERING_PARAMS_HPP_
#define POINTCLOUD_TO_LASERSCAN__FILTERING_PARAMS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <limits>
#include <cmath>
#include <thread>

namespace pointcloud_to_laserscan
{

class PointCloudToLaserScanParams
{
public:

    // Constructor that declares and initializes all parameters
    explicit PointCloudToLaserScanParams(rclcpp::Node* node);
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

    //ransac params
    double ransac_range_candidates_ ;
    double ransac_height_candidates_;
    double ransac_threshold_inliers_;
    double ransac_filter_height_ ;

};

}  // namespace pointcloud_to_laserscan

#endif  // POINTCLOUD_TO_LASERSCAN__POINTCLOUD_TO_LASERSCAN_PARAMS_HPP_