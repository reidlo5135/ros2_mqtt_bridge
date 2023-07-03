/**
 * @file main.cpp
 * @author reidlo(naru5135@wavem.net)
 * @date 2023-06-27
 * @brief main program file
*/

/**
 * @brief include/ros2_mqtt_bridge/dynamic_bridge.hpp include area
*/

#include "ros2_mqtt_bridge/dynamic_bridge.hpp"

/**
 * @brief function for initialize rclcpp and execute main program with rclcpp::Node
 * @see rclcpp
 * @see rclcpp::Node
 * @see rclcpp::executors
*/
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