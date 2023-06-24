#include "ros2_mqtt_bridge/dynamic_bridge.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> ros2_node = std::make_shared<ros2_mqtt_bridge::RCLNode>();
    rclcpp::executors::SingleThreadedExecutor ros2_single_executor;
    while(rclcpp::ok()) {
        ros2_single_executor.spin_node_once(ros2_node);
    }
    rclcpp::shutdown();
    return 0;   
}