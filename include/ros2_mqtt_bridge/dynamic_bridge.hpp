#ifndef ROS2_MQTT_BRIDGE_DYNAMIC_BRIDGE__HPP
#define ROS2_MQTT_BRIDGE_DYNAMIC_BRIDGE__HPP

#include <stdio.h>
#include <map>
#include <memory>
#include <cstring>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <mqtt/async_client.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <rcutils/logging_macros.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>

static constexpr const char * RCL_NODE_NAME = "ros2_mqtt_bridge";

static constexpr const int & RCL_DEFAULT_QOS = 10;

static constexpr const char * RCL_CHATTER_TOPIC = "/chatter";

static constexpr const char * MQTT_ADDRESS = "tcp://localhost:1883";

static constexpr const char * MQTT_CLIENT_ID = "ros2_mqtt_bridge";

static constexpr const int & MQTT_DEFAULT_QOS = 0;

static constexpr const int & MQTT_RETRY_ATTEMPTS = 5;

/**
 * @brief define macros area
*/
#define RCLCPP_LINE_INFO() \
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "LINE : [%d]\n", __LINE__)

#define RCLCPP_LINE_ERROR() \
    RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "LINE : [%d]\n", __LINE__)

/**
 * @brief using namespaces area
*/
using std::placeholders::_1;
using std::placeholders::_2;

namespace ros2_mqtt_bridge {
    class RCLNode;
    class DynamicBridge;
    
    class BridgeManager : public virtual mqtt::callback {
        private :
            mqtt::async_client mqtt_async_client_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rcl_chatter_publisher_ptr_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rcl_chatter_subscription_ptr_;
            void mqtt_connect();
            void connection_lost(const std::string & mqtt_connection_lost_cause) override;
            void message_arrived(mqtt::const_message_ptr mqtt_message) override;
            void delivery_complete(mqtt::delivery_token_ptr mqtt_delivered_token) override;
        public :
            explicit BridgeManager(rclcpp::Node::SharedPtr rcl_node_ptr, std::shared_ptr<ros2_mqtt_bridge::DynamicBridge> rcl_dynamic_bridge_ptr);
            virtual ~BridgeManager();
            void mqtt_publish(const char * mqtt_topic, std::string mqtt_payload);
            void mqtt_subscribe(const char * mqtt_topic);
    };

    class DynamicBridge {
        public :
            explicit DynamicBridge(){};
            virtual ~DynamicBridge(){};

            template<typename message_type>
            typename rclcpp::Publisher<message_type>::SharedPtr register_publisher(
                rclcpp::Node::SharedPtr rcl_node_ptr,
                const std::string & rcl_topic_name
            ) {
                if(rcl_node_ptr == nullptr) {
                    RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "rcl register publisher ros2 node pointer is null \n");
                    return nullptr;
                }

                RCLCPP_INFO(rcl_node_ptr->get_logger(), "rcl register publisher [%s]", rcl_topic_name.c_str());

                typename rclcpp::Publisher<message_type>::SharedPtr rcl_publisher_ptr = rcl_node_ptr->create_publisher<message_type>(
                    rcl_topic_name,
                    rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS))
                );

                return rcl_publisher_ptr;
            }

            template<typename message_type>
            typename rclcpp::Subscription<message_type>::SharedPtr register_subscription(
                rclcpp::Node::SharedPtr rcl_node_ptr,
                const std::string& rcl_topic_name,
                std::function<void(std::shared_ptr<message_type>)> rcl_subscription_callback
            ) {
                if(rcl_node_ptr == nullptr) {
                    RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "ROS2 register subscription ros2 node pointer is null \n");
                    return nullptr;
                }

                RCLCPP_INFO(rcl_node_ptr->get_logger(), "ROS2 register subscription [%s]", rcl_topic_name.c_str());

                typename rclcpp::Subscription<message_type>::SharedPtr rcl_subscription_ptr = rcl_node_ptr->create_subscription<message_type>(
                    rcl_topic_name,
                    rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
                    std::function<void(std::shared_ptr<message_type>)>(rcl_subscription_callback)
                );

                return rcl_subscription_ptr;
            }
    };

    class RCLNode : public rclcpp::Node {
        private :
            rclcpp::Node::SharedPtr rcl_node_ptr_;
            std::shared_ptr<ros2_mqtt_bridge::BridgeManager> bridge_manager_ptr_;
            std::shared_ptr<ros2_mqtt_bridge::DynamicBridge> rcl_dynamic_bridge_ptr_;
            rclcpp::TimerBase::SharedPtr rcl_poll_timer_ptr_;
            std::mutex bridge_mutex_;
        public :
            explicit RCLNode();
            virtual ~RCLNode();
            void get_current_topic_and_types();
    };
}

#endif