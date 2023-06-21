#include "ros2_mqtt_bridge/dynamic_bridge.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<ros2_mqtt_bridge::RCLNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;   
}