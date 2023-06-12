#ifndef ROS2_MQTT_BRIDGE_DYNAMIC_BRIDGE_HPP
#define ROS2_MQTT_BRIDGE_DYNAMIC_BRIDGE_HPP

#include <map>
#include <memory>
#include <cstring>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/node.hpp"
#include "rclcpp/scope_exit.hpp"
#include "rcutils/get_env.h"
#include "std_msgs/msg/string.hpp"
#include "factory/factory.hpp"
#include "factory/factory_interface.hpp"

std::mutex g_bridge_mutex;

namespace ros2_mqtt_bridge {
    
    struct BridgeMqttToRos2 {
        rclcpp::PublisherBase::SharedPtr ros2_publisher_ptr;
    };

    struct BridgeRos2ToMqtt {
        rclcpp::SubscriptionBase::SharedPtr ros2_subscription_ptr;        
    };

    struct BridgeHandle {
        BridgeMqttToRos2 bridge_mqtt_to_ros2;
        BridgeRos2ToMqtt bridge_ros2_to_mqtt;
    };

    bool get_ros_to_mqtt_mapping(const std::string& ros2_topic, const std::string& mqtt_topic);
    bool get_mqtt_ros_mapping(const std::string& mqtt_topic, const std::string& ros2_topic);

    std::multimap<std::string, std::string> get_all_message_mappings();

    std::shared_ptr<FactoryInterface> get_factory(const std::string& ros2_message_type);

    BridgeMqttToRos2 create_bridge_mqtt_to_ros2(
        rclcpp::Node::SharedPtr ros2_node_ptr,
        const std::string& mqtt_topic,
        const std::string& ros2_message_type,
        const std::string& ros2_topic,
        rclcpp::QoS ros2_qos
    );

    BridgeRos2ToMqtt create_bridge_ros2_to_mqtt(
        rclcpp::Node::SharedPtr ros2_node_ptr,
        const std::string& mqtt_topic,
        const std::string& ros2_message_type,
        const std::string& ros2_topic,
        rclcpp::QoS ros2_qos        
    );
}

bool find_command_option(const std::vector<std::string>& args, const std::string& option) {
    return std::find(args.begin(), args.end(), option) != args.end();
}

bool get_flag_option(const std::vector<std::string>& args, const std::string& option) {
    auto it = std::find(args.begin(), args.end(), option);
    return it != args.end();
}

#endif