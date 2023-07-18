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
 * @file dynamic_bridge.hpp
 * @author reidlo(naru5135@wavem.net)
 * @date 2023-06-27
 * @brief header file for dynamic_bridge.cpp
*/

#ifndef ROS2_MQTT_BRIDGE_DYNAMIC_BRIDGE__HPP
#define ROS2_MQTT_BRIDGE_DYNAMIC_BRIDGE__HPP

#include "ros2_mqtt_definer/define_container.hpp"
#include "ros2_mqtt_message_converter/message_converter.hpp"

/**
 * @brief namespace for define RCLMQTTBridgeManager, RCLConnectionManager, RCLNode classes
*/

namespace ros2_mqtt_bridge {

    class RCLNode;
    class RCLConnectionManager;
    
    /**
     * @class RCLMQTTBridgeManager
     * @brief final class for implements mqtt::callback what defines MQTT connections
    */
    class RCLMQTTBridgeManager final : public virtual mqtt::callback {
        private :
            /**
             * @brief mqtt::async_client instance
             * @see mqtt::async_client
            */
            mqtt::async_client mqtt_async_client_;

            /**
             * @brief Shared Pointer for rclcpp::Node
            */
            rclcpp::Node::SharedPtr rcl_node_ptr_;

            /**
             * @brief shared pointer for ros2_mqtt_bridge::RCLMQTTBridgeManager
             * @see ros2_mqtt_bridge::RCLMQTTBridgeManager
            */
            std::shared_ptr<ros2_mqtt_bridge::RCLMQTTBridgeManager> rcl_mqtt_bridge_manager_ptr_;

            /**
             * @brief shared pointer for ros2_mqtt_bridge::RCLConnectionManager
             * @see ros2_mqtt_bridge::RCLConnectionManager
            */
            std::shared_ptr<ros2_mqtt_bridge::RCLConnectionManager> rcl_connection_manager_ptr_;

            /**
             * @brief shared pointer for rclcpp::TimerBase
            */
            rclcpp::TimerBase::SharedPtr rcl_timer_base_ptr_;

            /**
             * @brief shared pointer for ros2_mqtt_bridge::StdMessageConverter
             * @see ros2_mqtt_bridge::StdMessageConverter
            */
            std::shared_ptr<ros2_mqtt_bridge::StdMessageConverter> rcl_std_msgs_converter_ptr_;

            /**
             * @brief shared pointer for ros2_mqtt_bridge::StdMessageConverter
             * @see ros2_mqtt_bridge::StdMessageConverter
            */
            std::shared_ptr<ros2_mqtt_bridge::GeometryMessageConverter> rcl_geometry_msgs_converter_ptr_;

            /**
             * @brief shared pointer for ros2_mqtt_bridge::StdMessageConverter
             * @see ros2_mqtt_bridge::StdMessageConverter
            */
            std::shared_ptr<ros2_mqtt_bridge::SensorMessageConverter> rcl_sensor_msgs_converter_ptr_;

            /**
             * @brief shared pointer for ros2_mqtt_bridge::NavMessageConverter
             * @see ros2_mqtt_bridge::NavMessageConverter
            */
            std::shared_ptr<ros2_mqtt_bridge::NavMessageConverter> rcl_nav_msgs_converter_ptr_;

            /**
             * @brief shared pointer for ros2_mqtt_bridge::TFMessageConverter
             * @see ros2_mqtt_bridge::TFMessageConverter
            */
            std::shared_ptr<ros2_mqtt_bridge::TFMessageConverter> rcl_tf_msgs_converter_ptr_;

            /**
             * @brief shared pointer for ros2_mqtt_bridge::NavigateToPoseActionConverter
             * @see ros2_mqtt_bridge::NavigateToPoseActionConverter
            */
            std::shared_ptr<ros2_mqtt_bridge::NavigateToPoseActionConverter> rcl_navigate_to_pose_action_converter_ptr_;

            /**
             * @brief shared pointer for ros2_mqtt_bridge::CanMessageConverter
             * @see ros2_mqtt_bridge::CanMessageConverter
            */
            std::shared_ptr<ros2_mqtt_bridge::CanMessageConverter> rcl_can_msgs_converter_ptr_;
            
            /**
             * @brief Shared Pointer for rclcpp::Publisher(topic : /chatter)
            */
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rcl_chatter_publisher_ptr_;
            
            /**
             * @brief Shared Pointer for rclcpp::Subscription(topic : /chatter)
            */
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rcl_chatter_subscription_ptr_;

            /**
             * @brief Shared Pointer for rclcpp::Subscription(topic : /map)
            */
            rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr rcl_map_subscription_ptr_;

            /**
             * @brief Shared Pointer for rclcpp::Subscription(topic : /robot_pose)
            */
            rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr rcl_robot_pose_subscription_ptr_;

            /**
             * @brief Shared Pointer for rclcpp::Subscription(topic : /scan)
            */
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr rcl_scan_subscription_ptr_;

            /**
             * @brief Shared Pointer for rclcpp::Subscription(topic : /tf)
            */
            rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr rcl_tf_subscription_ptr_;

            /**
             * @brief Shared Pointer for rclcpp::Subscription(topic : /tf_static)
            */
            rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr rcl_tf_static_subscription_ptr_;

            /**
             * @brief Shared Pointer for rclcpp::Publisher(topic : /cmd_vel)
            */
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr rcl_cmd_vel_publisher_ptr_;

            /**
             * @brief Shared Pointer for rclcpp::Subscription(topic : /cmd_vel)
            */
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr rcl_cmd_vel_subscription_ptr_;

            /**
             * @brief Shared Pointer for rclcpp::Subscription(topic : /odom)
            */
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rcl_odom_subscription_ptr_;

            /**
             * @brief Shared Pointer for rclcpp::Publisher(topic : /can/control_hardware)
            */
            rclcpp::Publisher<can_msgs::msg::ControlHardware>::SharedPtr rcl_can_control_hardware_publisher_ptr_;

            /**
             * @brief Shared Pointer for rclcpp::Subscription(topic : /ublox_fix)
            */
            rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr rcl_ublox_fix_subscription_ptr_;

            /**
             * @brief Shared Pointer for rclcpp::Subscription(topic : /imu/data)
            */
            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr rcl_imu_data_subscription_ptr_;

            /**
             * @brief mutex for bridge_rcl_to_mqtt
            */
            std::mutex rcl_lambda_mutex_;

            /**
             * @brief function for MQTT connection
             * @return void
            */
            void mqtt_connect();

            /**
             * @brief overrided function for handle when MQTT connection lost
             * @param mqtt_connection_lost_cause lost cause of MQTT connection
             * @return void
            */
            void connection_lost(const std::string & mqtt_connection_lost_cause) override;

            /**
             * @brief overrided function for handle when MQTT message arrived
             * @param mqtt_message callback for MQTT message
             * @return void
            */
            void message_arrived(mqtt::const_message_ptr mqtt_message) override;

            /**
             * @brief overrided function for handle when MQTT delivery completed
             * @param mqtt_delivered_token callback for MQTT delivery token
             * @return void
            */
            void delivery_complete(mqtt::delivery_token_ptr mqtt_delivered_token) override;

            /**
            * @brief function for compare MQTT topic and RCL topic before bridge MQTT to RCL
            * @param mqtt_topic target MQTT topic
            * @param rcl_target_topic target RCL topic
            * @return is_mqtt_topic_equals_rcl_topic bool
            */
            bool bridge_mqtt_to_rcl_topic_cmp(const std::string & mqtt_topic, const char * rcl_target_topic);

            /**
            * @brief function for compare MQTT topic and RCL topic before bridge MQTT to RCL
            * @param rcl_subscription_topic target MQTT topic
            * @param rcl_target_topic target RCL topic
            * @return is_rcl_subscription_topic_equals_rcl_target_topic bool
            */
            bool bridge_mqtt_to_rcl_topic_cmp(const char * rcl_subscription_topic, const char * rcl_target_topic);

            /**
             * @brief function for compare RCL subscription's message type and RCL target message type before bridge RCL to MQTT
             * @param rcl_subscription_msgs_type target RCL subscription's message type const std::string &
             * @param rcl_target_msgs_type target RCL message type const std::string &
             * @return is_rcl_subscription_message_type_equals_rcl_target_message_type bool
            */
            bool bridge_mqtt_to_rcl_msgs_type_cmp(const std::string & rcl_subscription_msgs_type, const char * rcl_target_msgs_type);

            /**
             * @brief function for flag and log foudate MQTT to RCL
             * @param rcl_subscription_topic target RCL subscription topic const char *
             * @param rcl_subscription_msgs_type target RCL subscription message type const std::string &
             * @return void
            */
            void flag_foundate_mqtt_to_rcl(const char * rcl_subscription_topic, const std::string & rcl_subscription_msgs_type);

            /**
             * @brief function for compare RCL publisher's topic and RCL target topic before bridge RCL to MQTT
             * @param rcl_publisher_topic target RCL publisher topic const char *
             * @param rcl_target_topic target RCL publisher topic const char *
             * @return is_rcl_publisher_topic_equals_rcl_target_topic bool
            */
            bool bridge_rcl_to_mqtt_topic_cmp(const char * rcl_publisher_topic, const char * rcl_target_topic);

            /**
             * @brief function for compare RCL publisher's message type and RCL target message type before bridge RCL to MQTT
             * @param rcl_publisher_msgs_type target RCL publisher's message type const std::string &
             * @param rcl_target_msgs_type target RCL message type const std::string &
             * @return is_rcl_publisher_message_type_equals_rcl_target_message_type bool
            */
            bool bridge_rcl_to_mqtt_msgs_type_cmp(const std::string & rcl_publisher_msgs_type, const char * rcl_target_msgs_type);

            /**
             * @brief function for flag and log bridge MQTT to RCL
             * @param mqtt_topic target MQTT topic
             * @param mqtt_payload target MQTT payload
             * @return void
            */
            void flag_bridge_mqtt_to_rcl(const std::string & mqtt_topic, const std::string & mqtt_payload);

            /**
             * @brief function for flag and log bridge RCL to MQTT
             * @param rcl_publisher_topic target RCL publisher topic
             * @param rcl_json_string target parsed RCL json styled string
             * @return void
            */
            void flag_bridge_rcl_to_mqtt(const char * rcl_publisher_topic, const std::string & rcl_json_string);
        public :
            /**
             * Create a new this class' instance
             * @brief Default Constructor
            */
            explicit RCLMQTTBridgeManager(rclcpp::Node::SharedPtr rcl_node_ptr, std::shared_ptr<ros2_mqtt_bridge::RCLConnectionManager> rcl_connection_manager_ptr);

            /**
             * Destroy this class' instance
             * @brief Default Destructor
            */
            virtual ~RCLMQTTBridgeManager();

            /**
             * @brief function for MQTT publish
             * @param mqtt_topic target MQTT topic
             * @param mqtt_payload target MQTT payload
             * @return void
            */
            void mqtt_publish(const char * mqtt_topic, std::string mqtt_payload);

            /**
             * @brief function for MQTT subscribe
             * @param mqtt_topic target MQTT topic
             * @return void
            */
            void mqtt_subscribe(const char * mqtt_topic);

            /**
             * @brief template function for log map
             * @param target_map target std::map
             * @param target_flag log target
             * @return void
            */
            template<typename begin_type, typename end_type>
            void flag_map(std::map<begin_type, end_type> target_map, const char * target_flag);

            /**
             * function for foundate MQTT to RCL by grant MQTT subscription with current RCL publishers
             * @param rcl_current_subscriptions_map target RCL current publishers map
             * @return void
            */
            void foundate_mqtt_to_rcl(std::map<std::string, std::string> rcl_current_subscriptions_map);

            /**
             * @brief function for bridge MQTT to RCL after delivered mqtt_subscription callback data
             * @param mqtt_topic target MQTT topic
             * @param mqtt_payload received MQTT payload
             * @return void
            */
            void bridge_mqtt_to_rcl(const std::string & mqtt_topic, const std::string & mqtt_payload);

            /**
             * @brief function for bridge RCL to MQTT after get current topic and types
             * @return void
            */
            void bridge_rcl_to_mqtt();
    };

    /**
     * @class RCLConnectionManager
     * @brief final class for defines template functions what implements rclcpp ROS2 connections
    */
    class RCLConnectionManager final {
        public :
            /**
             * Create a new this class' instance
             * @brief Default Constructor
            */
            inline explicit RCLConnectionManager() {

            };

            /**
             * Destroy this class' instance
             * @brief Default Destructor
            */
            inline virtual ~RCLConnectionManager() {

            };

            /**
             * @brief inline template function for initialize rclcpp::Publisher and register into rclcpp::Node
             * @param rcl_node_ptr rclcpp::Node's shared pointer
             * @param rcl_topic_name target ROS2 topic
             * @return typename rclcpp::Publisher<rcl_message_type>::SharedPtr
            */
            template<typename rcl_message_type>
            inline typename rclcpp::Publisher<rcl_message_type>::SharedPtr register_publisher(
                rclcpp::Node::SharedPtr rcl_node_ptr,
                const std::string & rcl_topic_name
            ) {
                RCLCPP_NODE_POINTER_VALIDATION(rcl_node_ptr, RCL_PUBLISHER_FLAG);

                typename rclcpp::Publisher<rcl_message_type>::SharedPtr rcl_publisher_ptr = rcl_node_ptr->create_publisher<rcl_message_type>(
                    rcl_topic_name,
                    rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS))
                );

                RCLCPP_REGISTER_INFO(rcl_node_ptr->get_logger(), RCL_PUBLISHER_FLAG, rcl_topic_name.c_str());

                return rcl_publisher_ptr;
            };

            /**
             * @brief inline template function for initialize rclcpp::Subscription and register into rclcpp::Node
             * @param rcl_node_ptr rclcpp::Node's shared pointer
             * @param rcl_topic_name target ROS2 topic
             * @param rcl_subscription_callback callback function for ROS2 subscription callback
             * @return typename rclcpp::Subscription<rcl_message_type>::SharedPtr
            */
            template<typename rcl_message_type>
            inline typename rclcpp::Subscription<rcl_message_type>::SharedPtr register_subscription(
                rclcpp::Node::SharedPtr rcl_node_ptr,
                const std::string & rcl_topic_name,
                std::function<void(std::shared_ptr<rcl_message_type>)> rcl_subscription_callback
            ) {
                RCLCPP_NODE_POINTER_VALIDATION(rcl_node_ptr, RCL_SUBSCRIPTION_FLAG);

                typename rclcpp::Subscription<rcl_message_type>::SharedPtr rcl_subscription_ptr = rcl_node_ptr->create_subscription<rcl_message_type>(
                    rcl_topic_name,
                    rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
                    std::function<void(std::shared_ptr<rcl_message_type>)>(rcl_subscription_callback)
                );

                RCLCPP_REGISTER_INFO(rcl_node_ptr->get_logger(), RCL_SUBSCRIPTION_FLAG, rcl_topic_name.c_str());

                return rcl_subscription_ptr;
            };

            /**
             * @brief inline template function for initialize rclcpp::Client and register into rclcpp::Node
             * @param rcl_node_ptr rclcpp::Node's shared pointer
             * @param rcl_target_service_server_name target ROS2 target service server name
             * @param rcl_service_callback callback function for ROS2 service callback
             * @return typename rclcpp::Client<rcl_message_type>::SharedPtr
            */
            template<typename rcl_message_type>
            inline typename rclcpp::Client<rcl_message_type>::SharedPtr register_service_client(
                rclcpp::Node::SharedPtr rcl_node_ptr,
                const std::string & rcl_target_service_server_name 
            ) {
                RCLCPP_NODE_POINTER_VALIDATION(rcl_node_ptr, RCL_SUBSCRIPTION_FLAG);
                RCLCPP_REGISTER_INFO(rcl_node_ptr->get_logger(), RCL_SERVICE_CLIENT_FLAG, rcl_target_service_server_name.c_str());

                typename rclcpp::Client<rcl_message_type>::SharedPtr rcl_service_client_ptr = rcl_node_ptr->create_client<rcl_message_type>(
                    rcl_target_service_server_name
                );

                return rcl_service_client_ptr;
            };

            /**
             * @brief inline template function for initialize rclcpp::Service and register into rclcpp::Node
             * @param rcl_node_ptr rclcpp::Node's shared pointer
             * @param rcl_service_server_name target ROS2 service server name
             * @param rcl_service_callback callback function for ROS2 service callback
             * @return typename rclcpp::Service<rcl_message_type>::SharedPtr
            */
            template<typename rcl_message_type>
            inline typename rclcpp::Service<rcl_message_type>::SharedPtr register_service_server(
                rclcpp::Node::SharedPtr rcl_node_ptr,
                const std::string & rcl_service_server_name,
                std::function<void(std::shared_ptr<rcl_message_type>)> rcl_service_callback
            ) {
                RCLCPP_NODE_POINTER_VALIDATION(rcl_node_ptr, RCL_SUBSCRIPTION_FLAG);
                RCLCPP_REGISTER_INFO(rcl_node_ptr->get_logger(), RCL_SERVICE_CLIENT_FLAG, rcl_service_server_name.c_str());

                typename rclcpp::Service<rcl_message_type>::SharedPtr rcl_service_server_ptr = rcl_node_ptr->create_service<rcl_message_type>(
                    rcl_service_server_name,
                    rcl_service_callback
                );

                return rcl_service_server_ptr;
            };
    };

    /**
     * @class RCLNode
     * @brief final class for initialize rclcpp::Node what defines rclcpp::Node
    */
    class RCLNode final : public rclcpp::Node {
        private :
            /**
             * @brief Shared Pointer for rclcpp::Node
            */
            rclcpp::Node::SharedPtr rcl_node_ptr_;

            /**
             * @brief shared pointer for ros2_mqtt_bridge::RCLMQTTBridgeManager
             * @see ros2_mqtt_bridge::RCLMQTTBridgeManager
            */
            std::shared_ptr<ros2_mqtt_bridge::RCLMQTTBridgeManager> rcl_mqtt_bridge_manager_ptr_;

            /**
             * @brief shared pointer for ros2_mqtt_bridge::RCLConnectionManager
             * @see ros2_mqtt_bridge::RCLConnectionManager
            */
            std::shared_ptr<ros2_mqtt_bridge::RCLConnectionManager> rcl_connection_manager_ptr_;
        public :
            /**
             * Create a new this class' instance
             * @brief Default Constructor
            */
            explicit RCLNode();

            /**
             * Destroy this class' instance
             * @brief Default Destructor
            */
            virtual ~RCLNode();

            /**
             * function for handle signal_input when program exit
             * @param signal_input The signal_input of input
             * @return void
             * @see signal_input.h
            */
            static void sig_handler(int signal_input);
    };
}

#endif