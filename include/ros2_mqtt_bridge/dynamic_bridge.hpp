/**
 * @file dynamic_bridge.hpp
 * @author reidlo(naru5135@wavem.net)
 * @date 2023-06-27
 * @brief header file for dynamic_bridge.cpp
*/

#ifndef ROS2_MQTT_BRIDGE_DYNAMIC_BRIDGE__HPP
#define ROS2_MQTT_BRIDGE_DYNAMIC_BRIDGE__HPP

/**
 * @brief default cpp header files include area
*/
#include <stdio.h>
#include <map>
#include <memory>
#include <cstring>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include <signal.h>

/**
 * @brief mqtt header file include area
*/
#include <mqtt/async_client.h>

/**
 * @brief rclcpp header files include area
*/
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <rcutils/logging_macros.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>

/**
 * @brief static const instance for define name of rclcpp::Node
*/
static constexpr const char * RCL_NODE_NAME = "ros2_mqtt_bridge";

/**
 * @brief static const instance for define default value of rclcpp::QoS
*/
static constexpr const int & RCL_DEFAULT_QOS = 10;

/**
 * @brief static const instance for define topic of rclcpp::Subscription(topic : /chatter)
*/
static constexpr const char * RCL_CHATTER_TOPIC = "/chatter";

/**
 * @brief static const instance for define address of MQTT
*/
static constexpr const char * MQTT_ADDRESS = "tcp://localhost:1883";

/**
 * @brief static const instance for define client id of MQTT
*/
static constexpr const char * MQTT_CLIENT_ID = "ros2_mqtt_bridge";

/**
 * @brief static const instance for define QoS of MQTT
*/
static constexpr const int & MQTT_DEFAULT_QOS = 0;

/**
 * @brief static const instance for define retry attempts of MQTT
*/
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

/**
 * @brief namespace for define BridgeManager, DynamicBridge, RCLNode classes
*/

namespace ros2_mqtt_bridge {

    class RCLNode;
    class DynamicBridge;
    
    /**
     * @brief final class for implements mqtt::callback what defines MQTT connections
    */
    class BridgeManager final : public virtual mqtt::callback {
        private :
            /**
             * @brief mqtt::async_client instance
             * @see mqtt::async_client
            */
            mqtt::async_client mqtt_async_client_;

            /**
             * @brief Shared Pointer for rclcpp::Publisher(topic : /chatter)
            */
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rcl_chatter_publisher_ptr_;

            /**
             * @brief Shared Pointer for rclcpp::Subscription(topic : /chatter)
            */
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rcl_chatter_subscription_ptr_;

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
        public :
            /**
             * Create a new this class' instance
             * @brief Default Constructor
            */
            explicit BridgeManager(rclcpp::Node::SharedPtr rcl_node_ptr, std::shared_ptr<ros2_mqtt_bridge::DynamicBridge> rcl_dynamic_bridge_ptr);

            /**
             * Destroy this class' instance
             * @brief Default Destructor
            */
            virtual ~BridgeManager();

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
    };

    /**
     * @brief final class for defines template functions what implements rclcpp ROS2 connections
    */
    class DynamicBridge final {
        public :
            /**
             * Create a new this class' instance
             * @brief Default Constructor
            */
            explicit DynamicBridge(){};

            /**
             * Destroy this class' instance
             * @brief Default Destructor
            */
            virtual ~DynamicBridge(){};

            /**
             * @brief template function for initialize rclcpp::Publisher and register into rclcpp::Node
             * @param rcl_node_ptr rclcpp::Node's shared pointer
             * @param rcl_topic_name target ROS2 topic
             * @return typename rclcpp::Publisher<message_type>::SharedPtr
            */
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

            /**
             * @brief template function for initialize rclcpp::Subscription and register into rclcpp::Node
             * @param rcl_node_ptr rclcpp::Node's shared pointer
             * @param rcl_topic_name target ROS2 topic
             * @param rcl_subscription_callback callback function for ROS2 subscription callback
             * @return typename rclcpp::Subscription<message_type>::SharedPtr
            */
            template<typename message_type>
            typename rclcpp::Subscription<message_type>::SharedPtr register_subscription(
                rclcpp::Node::SharedPtr rcl_node_ptr,
                const std::string & rcl_topic_name,
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

    /**
     * @brief final class for initialize rclcpp::Node what defines rclcpp::Node
    */
    class RCLNode final : public rclcpp::Node {
        private :
            /**
             * @brief Shared Pointer for rclcpp::Node
            */
            rclcpp::Node::SharedPtr rcl_node_ptr_;

            /**
             * @brief shared pointer for ros2_mqtt_bridge::BridgeManager
             * @see ros2_mqtt_bridge::BridgeManager
            */
            std::shared_ptr<ros2_mqtt_bridge::BridgeManager> bridge_manager_ptr_;

            /**
             * @brief shared pointer for ros2_mqtt_bridge::DynamicBridge
             * @see ros2_mqtt_bridge::DynamicBridge
            */
            std::shared_ptr<ros2_mqtt_bridge::DynamicBridge> rcl_dynamic_bridge_ptr_;

            /**
             * @brief shared pointer for rclcpp::TimberBase
            */
            rclcpp::TimerBase::SharedPtr rcl_poll_timer_ptr_;

            /**
             * @brief mutex for bridge
            */
            std::mutex bridge_mutex_;
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
             * Function for handle signal when program exit
             * @param sig The signal of input
             * @return void
             * @see signal.h
            */
            static void sig_handler(int sig);

            /**
             * @brief function for get current ROS2 topics and types
             * @return void
            */
            void get_current_topics_and_types();
    };
}

#endif