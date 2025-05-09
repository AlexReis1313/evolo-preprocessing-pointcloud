#include "rclcpp/rclcpp.hpp"
#include "bounding_box_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BoundingBoxNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}