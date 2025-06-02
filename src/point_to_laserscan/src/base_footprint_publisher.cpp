#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class BaseFootprintPublisher : public rclcpp::Node
{
public:
    BaseFootprintPublisher() : Node("base_footprint_publisher")
    {
        baseLink_frame_ = this->declare_parameter("base_link", "base_link");
        target_frame_ = this->declare_parameter("target_frame", "");
        fixed_frame_ = this->declare_parameter("fixed_frame", "");
        
        // Initialize tf2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Initialize transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // Timer at 50 Hz (0.02 seconds)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&BaseFootprintPublisher::publishBaseFootprint, this));
        
        RCLCPP_INFO(this->get_logger(), "Base footprint publisher started at 50Hz");
    }

private:
    void publishBaseFootprint()
    {
        try
        {
            // Get transform from odom to base_link
            geometry_msgs::msg::TransformStamped transform = 
                tf_buffer_->lookupTransform(this->fixed_frame_, this->baseLink_frame_, tf2::TimePointZero);
                                            //odom          base_link

            // Create new transform for base_footprint
            geometry_msgs::msg::TransformStamped base_footprint_transform;
            base_footprint_transform.header.stamp = transform.header.stamp;
            base_footprint_transform.header.frame_id = this->fixed_frame_; //odom
            base_footprint_transform.child_frame_id = this->target_frame_;//base_footprint
            
            // Copy x and y translation, set z to 0
            base_footprint_transform.transform.translation.x = transform.transform.translation.x;
            base_footprint_transform.transform.translation.y = transform.transform.translation.y;
            base_footprint_transform.transform.translation.z = transform.transform.translation.z;
            
            // Extract yaw from base_link orientation
            tf2::Quaternion base_link_quat(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            );
            
            // Convert to RPY and extract only yaw
            tf2::Matrix3x3 m(base_link_quat);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            // Create new quaternion with only yaw (roll=0, pitch=0)
            tf2::Quaternion footprint_quat;
            footprint_quat.setRPY(0, 0, yaw);
            
            base_footprint_transform.transform.rotation.x = footprint_quat.x();
            base_footprint_transform.transform.rotation.y = footprint_quat.y();
            base_footprint_transform.transform.rotation.z = footprint_quat.z();
            base_footprint_transform.transform.rotation.w = footprint_quat.w();
            
            // Broadcast the transform
            tf_broadcaster_->sendTransform(base_footprint_transform);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Could not get transform from odom to base_link: %s", ex.what());
        }
    }
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string baseLink_frame_;
    std::string target_frame_;
    std::string fixed_frame_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseFootprintPublisher>());
    rclcpp::shutdown();
    return 0;
}