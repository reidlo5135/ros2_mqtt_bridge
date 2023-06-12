#ifndef ROS2_MQTT_BRIDGE_FACTORY_INTERFACE_HPP
#define ROS2_MQTT_BRIDGE_FACTORY_INTERFACE_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"

namespace ros2_mqtt_bridge {

    class FactoryInterface {
        public:
            FactoryInterface(){};
            virtual ~FactoryInterface(){};

            virtual rclcpp::PublisherBase::SharedPtr create_ros2_publisher(
                rclcpp::Node::SharedPtr ros2_node_ptr,
                const std::string& mqtt_topic,
                const std::string& ros2_topic
            ) = 0;

            virtual rclcpp::SubscriptionBase::SharedPtr create_ros2_subscription(
                rclcpp::Node::SharedPtr ros2_node_ptr,
                const std::string& ros2_topic,
                const std::string& mqtt_topic,
                const std::string& ros2_message_type,
                rclcpp::QoS ros2_qos,
                rclcpp::PublisherBase::SharedPtr ros2_publisher_ptr = nullptr
            ) = 0;
    };
}

#endif