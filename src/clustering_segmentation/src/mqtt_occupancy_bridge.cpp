#include "mqtt_occupancy_bridge.hpp"

#include <nlohmann/json.hpp>  // For JSON handling, install via apt or add to your CMakeLists
using json = nlohmann::json;

MqttOccupancyBridgeNode::MqttOccupancyBridgeNode()
: Node("mqtt_occupancy_bridge")
{
    first_time_ = true;
    // Subscriber for occupancy grid
    sub_map = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_in_topic, 10,
    std::bind(&MqttOccupancyBridgeNode::mapCallback, this, std::placeholders::_1)
    );

    // Publisher for point cloud
    //pub_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    //points_out_topic, 10   );
    pub_json_ = this->create_publisher<std_msgs::msg::String>(
        string_out_topic, 10
    );
    
}

void MqttOccupancyBridgeNode::mapCallback(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr input_msg)
{   
    rclcpp::Time currentTime =rclcpp::Time(input_msg->header.stamp);
    
    if ( !first_time_ &&
        (currentTime - prev_time).seconds() < 1/throttle_hz_ ){ //pass this msg
        return;
    }
    //else
    first_time_ = false;
    prev_time = currentTime;

    // Extract map info
    const auto &info = input_msg->info;
    const float resolution = info.resolution;
    const uint32_t width = info.width;
    const uint32_t height = info.height;
    const float origin_x = info.origin.position.x;
    const float origin_y = info.origin.position.y;

    /*// Create an iterator for the "xyz" fields
    // This iterator allows you to assign a float array of size 3 (x, y, z) directly
    pcl::PointCloud<pcl::PointXYZ> cloud_;

    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            const size_t idx = y * width + x;
            const int8_t occ = input_msg->data[idx];
            if (occ > 30) {
                pcl::PointXYZ pt;
                pt.x = origin_x + (x + 0.5f) * resolution;
                pt.y = origin_y + (y + 0.5f) * resolution;
                pt.z = 0.0f; //2D map, so Z is 0
                cloud_.points.push_back(pt);

                
            }
        }
    }
    cloud_.width = cloud_.points.size();
    cloud_.height = 1;
    cloud_.is_dense = true;
    auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud_, *cloud_msg);
    cloud_msg->header=input_msg->header;
    pub_lidar_->publish(std::move(*cloud_msg));*/
    json json_points = json::array();

    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            const size_t idx = y * width + x;
            const int8_t occ = input_msg->data[idx];
            if (occ > 30) {
                float px = origin_x + (x + 0.5f) * resolution;
                float py = origin_y + (y + 0.5f) * resolution;

                json_points.push_back({px,py});
            }
        }
    }

    std_msgs::msg::String msg;           
    msg.data = json_points.dump();       // Convert JSON array to string
    pub_json_->publish(msg);     

    }

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MqttOccupancyBridgeNode>());
  rclcpp::shutdown();
  return 0;
}