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
 * @file bridge_tester.hpp
 * @author reidlo(naru5135@wavem.net)
 * @date 2023-07-17
 * @brief header file for bridge_tester.cpp
*/

#ifndef ROS2_MQTT_BRIDGE_BRIDGE_TESTER__HPP
#define ROS2_MQTT_BRIDGE_BRIDGE_TESTER__HPP

#include "ros2_mqtt_definer/define_container.hpp"
#include "ros2_mqtt_bridge/dynamic_bridge.hpp"

namespace ros2_mqtt_bridge {

    class RCLMQTTBridgeTester final : public rclcpp::Node {
        private :
            /**
             * @brief Shared Pointer for rclcpp::Node
            */
            rclcpp::Node::SharedPtr rcl_node_ptr_;
            
            /**
             * @brief shared pointer for ros2_mqtt_bridge::RCLConnection
             * @see ros2_mqtt_bridge::RCLConnection
            */
            std::shared_ptr<ros2_mqtt_bridge::RCLConnection> rcl_connection_manager_ptr_;
            
            /**
             * @brief shared pointer for rclcpp::TimerBase
            */
            rclcpp::TimerBase::SharedPtr rcl_timer_base_ptr_;

            /**
             * @brief Shared Pointer for rclcpp::Publisher(topic : /chatter)
            */
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rcl_chatter_publisher_ptr_;

            /**
             * @brief Shared Pointer for rclcpp::Publisher(topic : /map)
            */
            rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr rcl_map_publisher_ptr_;

            /**
             * @brief Shared Pointer for rclcpp::Publisher(topic : /robot_pose)
            */
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr rcl_robot_pose_publisher_ptr_;

            /**
             * @brief Shared Pointer for rclcpp::Publisher(topic : /scan)
            */
            rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr rcl_scan_publisher_ptr_;

            /**
             * @brief Shared Pointer for rclcpp::Subscription(topic : /tf)
            */
            rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr rcl_tf_publisher_ptr_;

            /**
             * @brief Shared Pointer for rclcpp::Publisher(topic : /tf_static)
            */
            rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr rcl_tf_static_publisher_ptr_;

            /**
             * @brief Shared Pointer for rclcpp::Publisher(topic : /cmd_vel)
            */
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr rcl_cmd_vel_publisher_ptr_;

            /**
             * @brief Shared Pointer for rclcpp::Publisher(topic : /odom)
            */
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rcl_odom_publisher_ptr_;

            /**
             * @brief Shared Pointer for rclcpp::Publisher(topic : /can/control_hardware)
            */
            rclcpp::Publisher<can_msgs::msg::ControlHardware>::SharedPtr rcl_can_control_hardware_publisher_ptr_;

            /**
             * @brief Shared Pointer for rclcpp::Publisher(topic : /ublox_fix)
            */
            rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr rcl_ublox_fix_publisher_ptr_;

            /**
             * @brief function for test "ros2_mqtt_bridge"#bridge_mqtt_to_rcl()
             * @return void
            */
            void bridge_test_mqtt_to_rcl();

            std_msgs::msg::Header::UniquePtr rcl_build_std_msgs_header(const char * frame_id);

            void flag_rcl_publish(const char * target_rcl_topic);

            void rcl_publish_chatter();

            void rcl_publish_map();

            void rcl_publish_robot_pose();

            void rcl_publish_scan();

            void rcl_publish_tf();

            void rcl_publish_tf_static();

            void rcl_publish_cmd_vel();

            void rcl_publish_odom();

            void rcl_publish_can_control_hardware();

            void rcl_publish_ublox_fix();

            /**
             * @brief function for test "ros2_mqtt_bridge"#bridge_rcl_to_mqtt()
             * @return void
            */
            void bridge_test_rcl_to_mqtt();
        public :
            /**
             * Create a new this class' instance
             * @brief Default Constructor
            */
            explicit RCLMQTTBridgeTester();

            /**
             * Destroy this class' instance
             * @brief Default Destructor
            */
            virtual ~RCLMQTTBridgeTester();

            /**
             * function for handle signal_input when program exit
             * @param signal_input The signal_input of input
             * @return void
             * @see signal_input.h
            */
            static void signal_handler(int signal_input);
    };
}

#endif