#ifndef ROS2_MQTT_BRIDGE_DYNAMIC_BRIDGE_HPP
#define ROS2_MQTT_BRIDGE_DYNAMIC_BRIDGE_HPP

#include <map>
#include <memory>
#include <cstring>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/scope_exit.hpp>
#include <rcutils/get_env.h>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

namespace ros2_mqtt_bridge {
    class DynamicBridge {
        public :
            template<typename message_type>
            typename rclcpp::Publisher<message_type>::SharedPtr register_publisher(rclcpp::Node::SharedPtr ros2_node_ptr, const std::string& ros2_topic_name) {
                if(ros2_node_ptr == nullptr) {
                    printf("ROS2 node pointer is null \n");
                }

                RCLCPP_INFO(ros2_node_ptr->get_logger(), "register publisher %s", ros2_topic_name.c_str());

                typename rclcpp::Publisher<message_type>::SharedPtr ros2_publisher_ptr = ros2_node_ptr->create_publisher<message_type>(
                    ros2_topic_name,
                    rclcpp::QoS(rclcpp::KeepLast(10))
                );

                return ros2_publisher_ptr;
            }

            template<typename message_type>
            typename rclcpp::Subscription<message_type>::SharedPtr register_subscription(rclcpp::Node::SharedPtr ros2_node_ptr, const std::string& ros2_topic_name) {
                if(ros2_node_ptr == nullptr) {
                    printf("ROS2 node pointer is null \n");
                }

                RCLCPP_INFO(ros2_node_ptr->get_logger(), "register subscription %s", ros2_topic_name.c_str());

                typename rclcpp::Subscription<message_type>::SharedPtr ros2_subscription_ptr = ros2_node_ptr->create_subscription<message_type>(
                    ros2_topic_name,
                    rclcpp::QoS(rclcpp::KeepLast(10)),
                    [this, ros2_node_ptr](const std::shared_ptr<message_type> callback) {
                        RCLCPP_INFO(ros2_node_ptr->get_logger(), "callback : ");
                    }
                );

                return ros2_subscription_ptr;
            }
    };

    class RCLNode : public rclcpp::Node {
        private :
            rclcpp::Node::SharedPtr ros2_node_ptr_;
            std::shared_ptr<ros2_mqtt_bridge::DynamicBridge> ros2_dynamic_bridge_ptr_;
            std::mutex bridge_mutex_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ros2_chatter_publisher_ptr_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ros2_chatter_subscription_ptr_;
        public :
            RCLNode();
            virtual ~RCLNode();
            void get_current_topic_and_types();
    };
}


#endif