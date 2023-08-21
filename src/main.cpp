// Copyright [2023] [wavem-reidlo]
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> rcl_node_ptr = std::make_shared<ros2_mqtt_bridge::Bridge>();

    signal(SIGINT, &ros2_mqtt_bridge::Bridge::signal_handler);
    signal(SIGTSTP, &ros2_mqtt_bridge::Bridge::signal_handler);

    rclcpp::executors::SingleThreadedExecutor rcl_single_thread_executor;
    while (rclcpp::ok())
    {
        rcl_single_thread_executor.spin_node_once(rcl_node_ptr);
    }
    rclcpp::shutdown();
    return 0;
}