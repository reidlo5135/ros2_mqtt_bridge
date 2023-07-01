#include "ros2_mqtt_bridge/dynamic_bridge.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> rcl_node_ptr = std::make_shared<ros2_mqtt_bridge::RCLNode>();
    rclcpp::executors::SingleThreadedExecutor rcl_single_thread_executor;
    while(rclcpp::ok()) {
        rcl_single_thread_executor.spin_node_once(rcl_node_ptr);
    }
    rclcpp::shutdown();
    return 0;   
}