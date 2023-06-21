#ifndef ROS2_MQTT_BRIDGE_DYNAMIC_BRIDGE_HPP
#define ROS2_MQTT_BRIDGE_DYNAMIC_BRIDGE_HPP

#include <stdio.h>
#include <map>
#include <memory>
#include <cstring>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <mqtt/async_client.h>
#include <rcutils/logging_macros.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>

#define ROS_NODE_NAME "ros2_mqtt_bridge"
#define ROS_DEAULT_QOS 10

#define MQTT_ADDRESS    "tcp://localhost:1883"
#define MQTT_CLIENT_ID    "ros2_mqtt_bridge"
#define MQTT_DEFAULT_QOS         0
#define MQTT_N_RETRY_ATTEMPTS 5

using std::placeholders::_1;

namespace ros2_mqtt_bridge {
    class RCLNode;
    class DynamicBridge;
    
    class BridgeManager : public virtual mqtt::callback {
        private :
            mqtt::async_client mqtt_async_client_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ros2_chatter_publisher_ptr_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ros2_chatter_subscription_ptr_;
            rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr ros2_header_subscription_ptr_;
            void mqtt_connect();
            void connection_lost(const std::string& mqtt_connection_lost_cause) override;
            void message_arrived(mqtt::const_message_ptr mqtt_message) override;
            void delivery_complete(mqtt::delivery_token_ptr mqtt_delivered_token) override;
        public :
            BridgeManager(rclcpp::Node::SharedPtr ros2_node_ptr, std::shared_ptr<ros2_mqtt_bridge::DynamicBridge> ros2_dynamic_bridge_ptr);
            virtual ~BridgeManager();
            void mqtt_publish(const char * mqtt_topic, std::string mqtt_payload);
            void mqtt_subscribe(const char * mqtt_topic);
            void mqtt_grant_subscription();
    };

    class DynamicBridge {
        public :
            DynamicBridge(){};
            virtual ~DynamicBridge(){};

            template<typename message_type>
            typename rclcpp::Publisher<message_type>::SharedPtr register_publisher(
                rclcpp::Node::SharedPtr ros2_node_ptr,
                const std::string& ros2_topic_name
            ) {
                if(ros2_node_ptr == nullptr) {
                    RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, "register publisher ros2 node pointer is null \n");
                }

                RCLCPP_INFO(ros2_node_ptr->get_logger(), "register publisher %s", ros2_topic_name.c_str());

                typename rclcpp::Publisher<message_type>::SharedPtr ros2_publisher_ptr = ros2_node_ptr->create_publisher<message_type>(
                    ros2_topic_name,
                    rclcpp::QoS(rclcpp::KeepLast(ROS_DEAULT_QOS))
                );

                return ros2_publisher_ptr;
            }

            template<typename message_type>
            typename rclcpp::Subscription<message_type>::SharedPtr register_subscription(
                rclcpp::Node::SharedPtr ros2_node_ptr,
                const std::string& ros2_topic_name,
                std::function<void(std::shared_ptr<message_type>)> ros2_subscription_callback
            ) {
                if(ros2_node_ptr == nullptr) {
                    RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, "register subscription ros2 node pointer is null \n");
                }

                RCLCPP_INFO(ros2_node_ptr->get_logger(), "register subscription %s", ros2_topic_name.c_str());

                typename rclcpp::Subscription<message_type>::SharedPtr ros2_subscription_ptr = ros2_node_ptr->create_subscription<message_type>(
                    ros2_topic_name,
                    rclcpp::QoS(rclcpp::KeepLast(ROS_DEAULT_QOS)),
                    std::function<void(std::shared_ptr<message_type>)>(ros2_subscription_callback)
                );

                return ros2_subscription_ptr;
            }
    };

    class RCLNode : public rclcpp::Node {
        private :
            rclcpp::Node::SharedPtr ros2_node_ptr_;
            std::shared_ptr<ros2_mqtt_bridge::BridgeManager> bridge_manager_ptr_;
            std::shared_ptr<ros2_mqtt_bridge::DynamicBridge> ros2_dynamic_bridge_ptr_;
            std::mutex bridge_mutex_;
        public :
            RCLNode();
            virtual ~RCLNode();
            void get_current_topic_and_types();
    };
}

#endif