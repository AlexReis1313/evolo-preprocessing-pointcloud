#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "nav_msgs/msg/odometry.hpp"


class BaseFootprintPublisher : public rclcpp::Node
{
public:
    BaseFootprintPublisher() : Node("base_footprint_publisher")
    {
        baseLink_frame_ = this->declare_parameter("base_link", "base_link");
        target_frame_ = this->declare_parameter("target_frame", "");
        fixed_frame_ = this->declare_parameter("fixed_frame", "");
        zero_height_ = this->declare_parameter("zero_heigh_footprint", true);

        // Initialize tf2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        count_=0;
        // Initialize transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        odom_pub_ =  this->create_publisher<nav_msgs::msg::Odometry>("footprint_odom",10);
        
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
            count_=count_+1;
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
            if (zero_height_){
                base_footprint_transform.transform.translation.z = 0;

            }else{
                base_footprint_transform.transform.translation.z = transform.transform.translation.z;

            }
            
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

            if (count_>1){ //reduces the odom publish rate in half, usefull for map updates as there is no need for such high publish rate
                count_=0;

            
                
                // --- Publish Odometry Message ---
                nav_msgs::msg::Odometry odom_msg;
                odom_msg.header.stamp = base_footprint_transform.header.stamp;
                odom_msg.header.frame_id = this->fixed_frame_; // odom
                odom_msg.child_frame_id = this->target_frame_; // base_footprint

                // Set the pose
                odom_msg.pose.pose.position.x = base_footprint_transform.transform.translation.x;
                odom_msg.pose.pose.position.y = base_footprint_transform.transform.translation.y;
                odom_msg.pose.pose.position.z = base_footprint_transform.transform.translation.z;
                odom_msg.pose.pose.orientation = base_footprint_transform.transform.rotation;

                // Calculate Twist (linear and angular velocities)
                if (last_transform_.header.stamp.sec != 0) { // Check if it's not the first transform
                    double dt = (base_footprint_transform.header.stamp.sec - last_transform_.header.stamp.sec) +
                                (base_footprint_transform.header.stamp.nanosec - last_transform_.header.stamp.nanosec) / 1e9;

                    if (dt > 0) {
                        // Linear velocity
                        odom_msg.twist.twist.linear.x = (base_footprint_transform.transform.translation.x - last_transform_.transform.translation.x) / dt;
                        odom_msg.twist.twist.linear.y = (base_footprint_transform.transform.translation.y - last_transform_.transform.translation.y) / dt;
                        odom_msg.twist.twist.linear.z = (base_footprint_transform.transform.translation.z - last_transform_.transform.translation.z) / dt;

                        // Angular velocity (only yaw component for base_footprint)
                        tf2::Quaternion current_quat(
                            base_footprint_transform.transform.rotation.x,
                            base_footprint_transform.transform.rotation.y,
                            base_footprint_transform.transform.rotation.z,
                            base_footprint_transform.transform.rotation.w
                        );
                        tf2::Quaternion last_quat(
                            last_transform_.transform.rotation.x,
                            last_transform_.transform.rotation.y,
                            last_transform_.transform.rotation.z,
                            last_transform_.transform.rotation.w
                        );

                        tf2::Matrix3x3 current_m(current_quat);
                        tf2::Matrix3x3 last_m(last_quat);
                        double current_roll, current_pitch, current_yaw;
                        double last_roll, last_pitch, last_yaw;
                        current_m.getRPY(current_roll, current_pitch, current_yaw);
                        last_m.getRPY(last_roll, last_pitch, last_yaw);

                        odom_msg.twist.twist.angular.z = (current_yaw - last_yaw) / dt;
                    }
                }

                // Set covariance (using default values or known values if available)
                // For simplicity, using a diagonal covariance matrix here.
                // In a real application, these values should be tuned based on sensor characteristics.
                odom_msg.pose.covariance[0] = 0.01;  // x
                odom_msg.pose.covariance[7] = 0.01;  // y
                odom_msg.pose.covariance[14] = 0.01; // z
                odom_msg.pose.covariance[21] = 0.001; // roll
                odom_msg.pose.covariance[28] = 0.001; // pitch
                odom_msg.pose.covariance[35] = 0.01; // yaw

                odom_msg.twist.covariance[0] = 0.01;  // linear x
                odom_msg.twist.covariance[7] = 0.01;  // linear y
                odom_msg.twist.covariance[14] = 0.01; // linear z
                odom_msg.twist.covariance[21] = 0.001; // angular roll
                odom_msg.twist.covariance[28] = 0.001; // angular pitch
                odom_msg.twist.covariance[35] = 0.01; // angular yaw

                // Publish the odometry message
                odom_pub_->publish(odom_msg);

                // Store the current transform for the next iteration
                last_transform_ = base_footprint_transform;
            }

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
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string baseLink_frame_;
    std::string target_frame_;
    std::string fixed_frame_;
    bool zero_height_;
    int count_=0;

    geometry_msgs::msg::TransformStamped last_transform_; // To store the previous transform for velocity calculation

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseFootprintPublisher>());
    rclcpp::shutdown();
    return 0;
}