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
 * @file define_container.hpp
 * @author reidlo(naru5135@wavem.net)
 * @date 2023-06-27
 * @brief header file for dynamic_bridge.cpp and message_converter.cpp
*/

#ifndef ROS2_MQTT_BRIDGE_DEFINE_CONTAINER__HPP
#define ROS2_MQTT_BRIDGE_DEFINE_CONTAINER__HPP

/**
 * @brief default cpp header files include area
*/
#include <iostream>
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <map>
#include <memory>
#include <string>
#include <cstring>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include <signal.h>
#include <functional>
#include <array>
#include <jsoncpp/json/json.h>

/**
 * @brief mqtt header file include area
*/
#include <mqtt/async_client.h>

/**
 * @brief rclcpp header files include area
*/
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/executor.hpp>
#include <rcutils/logging_macros.h>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <tf2_msgs/msg/tf_message.hpp>

/**
 * @brief static const instance for define name of rclcpp::Node
*/
static constexpr const char * RCL_NODE_NAME = "ros2_mqtt_bridge";

/**
 * @brief static const instance for define default value of rclcpp::QoS
*/
static constexpr const int & RCL_DEFAULT_QOS = 10;

/**
 * @brief static const instance for define flag when rcl exited
*/
static constexpr const int & RCL_EXIT_FLAG = 0;

/**
 * @brief static const instance for define flag of rclcpp::Publisher
*/
static constexpr const char * RCL_PUBLISHER_FLAG = "publisher";

/**
 * @brief static const instance for define flag of rclcpp::Subscription
*/
static constexpr const char * RCL_SUBSCRIPTION_FLAG = "subscription";

/**
 * @brief static const instance for define flag of rclcpp::Client
*/
static constexpr const char * RCL_SERVICE_CLIENT_FLAG = "service - client";

/**
 * @brief static const instance for define flag of rclcpp::Service
*/
static constexpr const char * RCL_SERVICE_SERVER_FLAG = "service - server";

/**
 * @brief static const instance for define flag of rclcpp_action::Client
*/
static constexpr const char * RCL_ACTION_CLIENT_FLAG = "action - client";

/**
 * @brief static const instance for define flag of rclcpp_action::Server
*/
static constexpr const char * RCL_ACTION_SERVER_FLAG = "action - server";

/**
 * @brief static const instance for define ignore topic(topic : /parameter_events)
*/
static constexpr const char * RCL_PARAMETER_EVENTS_TOPIC = "/parameter_events";

/**
 * @brief static const instance for define ignore topic(topic : /rosout)
*/
static constexpr const char * RCL_ROSOUT_TOPIC = "/rosout";

/**
 * @brief static const instance for define topic(topic : /chatter)
*/
static constexpr const char * RCL_CHATTER_TOPIC = "/chatter";

/**
 * @brief static const instance for define topic(topic : /odom)
*/
static constexpr const char * RCL_ODOM_TOPIC = "/odom";

/**
 * @brief static const instance for define topic(topic : /imu/data)
*/
static constexpr const char * RCL_IMU_DATA_TOPIC = "/imu/data";

/**
 * @brief static const instance for define topic(topic : /scan)
*/
static constexpr const char * RCL_SCAN_TOPIC = "/scan";

static constexpr const char * RCL_JSON_STRING_DEFAULT = "nullstr";

static constexpr const double & RCL_JSON_DOUBLE_DEFAULT = 0.0;

static constexpr const char * RCL_JSON_HEADER = "Header";

static constexpr const char * RCL_JSON_STRING = "String";

static constexpr const char * RCL_JSON_HEADER_FRAME_ID = "frame_id";

static constexpr const char * RCL_JSON_HEADER_SEQ = "seq";

static constexpr const char * RCL_JSON_HEADER_SEC = "sec";

static constexpr const char * RCL_JSON_HEADER_NANOSEC = "nanosec";

static constexpr const char * RCL_JSON_HEADER_STAMP = "stamp";

static constexpr const char * RCL_JSON_STRING_DATA = "data";

static constexpr const char * RCL_JSON_POINT = "point";

static constexpr const char * RCL_JSON_POINT_X = "x";

static constexpr const char * RCL_JSON_POINT_Y = "y";

static constexpr const char * RCL_JSON_POINT_Z = "z";

static constexpr const char * RCL_JSON_QUATERNION = "quaternion";

static constexpr const char * RCL_JSON_QUATERNION_X = "x";

static constexpr const char * RCL_JSON_QUATERNION_Y = "y";

static constexpr const char * RCL_JSON_QUATERNION_Z = "z";

static constexpr const char * RCL_JSON_QUATERNION_W = "w";

static constexpr const char * RCL_JSON_POSE = "pose";

static constexpr const char * RCL_JSON_POSITION = "position";

static constexpr const char * RCL_JSON_ORIENTATION = "orientation";

static constexpr const char * RCL_JSON_VECTOR3 = "vector3";

static constexpr const char * RCL_JSON_VECTOR3_X = "x";

static constexpr const char * RCL_JSON_VECTOR3_Y = "y";

static constexpr const char * RCL_JSON_VECTOR3_Z = "z";

static constexpr const char * RCL_JSON_TWIST = "twist";

static constexpr const char * RCL_JSON_LINEAR = "linear";

static constexpr const char * RCL_JSON_ANGULAR = "angular";

static constexpr const char * RCL_JSON_POSE_COVARIANCE = "covariance";

static constexpr const int & RCL_JSON_POSE_COVARIANCE_SIZE_DEFAULT = 36;

static constexpr const char * RCL_JSON_POSE_STAMPED = "pose_stamped";

static constexpr const char * RCL_JSON_POSE_WITH_COVARIANCE = "pose_with_covariance";

static constexpr const char * RCL_JSON_POSE_WITH_COVARIANCE_STAMPED = "pose_with_covariance_stamped";

/**
 * @brief static const instance for define message type of std_msgs::msg
*/
static constexpr const char * RCL_STD_MSGS_TYPE = "std_msgs/msg/";

/**
 * @brief static const instance for define message type of geometry_msgs::msg
*/
static constexpr const char * RCL_GEOMETRY_MSGS_TYPE = "geometry_msgs/msg/";

/**
 * @brief static const instance for define message type of nav_msgs::msg
*/
static constexpr const char * RCL_NAV_MSGS_TYPE = "nav_msgs/msg/";

/**
 * @brief static const instance for define message type of nav2_msgs::msg
*/
static constexpr const char * RCL_NAV2_MSGS_TYPE = "nav2_msgs/msg/";

/**
 * @brief static const instance for define message type of sensor_msgs::msg
*/
static constexpr const char * RCL_SENSOR_MSGS_TYPE = "sensor_msgs/msg/";

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

#define RCLCPP_LINE_WARN() \
    RCUTILS_LOG_WARN_NAMED(RCL_NODE_NAME, "LINE : [%d]\n", __LINE__)

#define RCLCPP_NODE_POINTER_VALIDATION(rcl_node_ptr, rcl_target_connection) \
    if(rcl_node_ptr == nullptr) {RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "rcl register [%s] - rclcpp::Node pointer is nullptr\n", rcl_target_connection);RCLCPP_LINE_ERROR();return nullptr;}

#define RCLCPP_REGISTER_INFO(rcl_node_ptr, rcl_target_connection, rcl_target_connection_name) \
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "rcl register [%s] named with [%s]", rcl_target_connection, rcl_target_connection_name);RCLCPP_LINE_INFO();

/**
 * @brief using namespaces area
*/
using std::placeholders::_1;
using std::placeholders::_2;

#endif