#ifndef ROS_MQTT_BRIDGE_FACTORY_HPP
#define ROS_MQTT_BRIDGE_FACTORY_HPP

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rmw/rmw.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"
#include "factory/factory_interface.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace ros2_mqtt_bridge {    

    template<typename ROS2_MSG_TYPE>
    class Factory {
        public :
            Factory(const std::string& ros2_message_type) 
            : ros2_message_type_(ros2_message_type)
            {};
            virtual ~Factory(){};

            rclcpp::PublisherBase::SharedPtr create_ros2_publisher(
                rclcpp::Node::SharedPtr ros2_node_ptr,
                const std::string& mqtt_topic
                const std::string& ros2_topic
            ) {
                RCLCPP_INFO(ros2_node_ptr->get_logger(), "creating %s publisher", ros2_topic);
                return ros2_node_ptr->create_publisher<ROS2_MSG_TYPE>(ros2_topic, rclcpp::QoS(rclcpp::KeepLast(10)));
            }

            rclcpp::SubscriptionBase::SharedPtr create_ros2_subscription(
                rclcpp::Node::SharedPtr ros2_node_ptr,
                const std::string& ros2_topic,
                const std::string& mqtt_topic,
                rclcpp::QoS ros2_qos,
                rclcpp::PublisherBase::SharedPtr ros2_publisher_ptr = nullptr
            ) {
                std::function<void(const typename ROS2_MSG_TYPE::SharedPtr ros2_msg, const rclcpp::MessageInfo& ros2_msg_info)> callback;
                callback = std::bind(
                    &Factory<ROS2_MSG_TYPE>::ros2_callback,
                    _1,
                    _2,
                    ros2_node_ptr->get_logger(),
                    ros2_publisher_ptr
                );

                rclcpp::SubscriptionOptions ros2_subscription_options;
                ros2_subscription_options.ignore_local_publications = true;

                return ros2_node_ptr->create_subscription<ROS2_MSG_TYPE>(
                    ros2_topic,
                    ros2_qos,
                    callback,
                    ros2_subscription_options
                );
            }

            std::string ros2_message_type_;

        protected:
            static void ros2_callback(
                typename ROS2_MSG_TYPE::SharedPtr ros2_msg,
                const rclcpp::MessageInfo& ros2_msg_info,
                const std::string& ros2_topic,
                rclcpp::Logger ros2_logger,
                rclcpp::PublisherBase::SharedPtr ros2_publisher_ptr = nullptr
            ) {
                if(ros2_publisher_ptr) {
                    bool rmw_result = false;

                    rmw_ret_t rmw_ret = rmw_compare_gids_equal(
                        &ros2_msg_info.get_rmw_message_info().publisher_gid,
                        &ros2_publisher_ptr->get_gid(),
                        &rmw_result
                    );
                    
                    if (rmw_ret == RMW_RET_OK) {
                        if (rmw_result) {
                            return;
                        }
                    } else {
                        std::string rmw_error_msg = std::string("Failed to compare gids : ") + rmw_get_error_string().str;
                        rmw_reset_error();
                        throw std::runtime_error(rmw_error_msg);
                    }
                }
            }
    };
}

#endif