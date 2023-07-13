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

/**
 * @brief static const instance for define jsoncpp string default value
*/
static constexpr const char * RCL_JSON_STRING_DEFAULT = "nullstr";

/**
 * @brief static const instance for define jsoncpp double default value
*/
static constexpr const double & RCL_JSON_DOUBLE_DEFAULT = 0.0;

/**
 * @brief static const instance for define jsoncpp float default value
*/
static constexpr const float & RCL_JSON_FLOAT_DEFAULT = 0.0f;

/**
 * @brief static const instance for define jsoncpp std_msgs::msg::String flag
*/
static constexpr const char * RCL_JSON_STRING_FLAG = "String";

/**
 * @brief static const instance for define jsoncpp std_msgs::msg::String string key
*/
static constexpr const char * RCL_JSON_STRING_KEY = "string";

/**
 * @brief static const instance for define jsoncpp std_msgs::msg::String data key
*/
static constexpr const char * RCL_JSON_STRING_DATA = "data";

/**
 * @brief static const instance for define jsoncpp child_frame_id key
*/
static constexpr const char * RCL_JSON_CHILD_FRAME_ID = "child_frame_id";

/**
 * @brief static const instance for define jsoncpp std_msgs::msg::Header flag
*/
static constexpr const char * RCL_JSON_HEADER_FLAG = "Header";

/**
 * @brief static const instance for define jsoncpp std_msgs::msg::Header header key
*/
static constexpr const char * RCL_JSON_HEADER_KEY = "header";

/**
 * @brief static const instance for define jsoncpp std_msgs::msg::Header frame_id key
*/
static constexpr const char * RCL_JSON_HEADER_FRAME_ID = "frame_id";

/**
 * @brief static const instance for define jsoncpp std_msgs::msg::Header seq key
*/
static constexpr const char * RCL_JSON_HEADER_SEQ = "seq";

/**
 * @brief static const instance for define jsoncpp std_msgs::msg::Header sec key
*/
static constexpr const char * RCL_JSON_HEADER_SEC = "sec";

/**
 * @brief static const instance for define jsoncpp std_msgs::msg::Header nanosec key
*/
static constexpr const char * RCL_JSON_HEADER_NANOSEC = "nanosec";

/**
 * @brief static const instance for define jsoncpp std_msgs::msg::Header stamp key
*/
static constexpr const char * RCL_JSON_HEADER_STAMP = "stamp";

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::Point point key
*/
static constexpr const char * RCL_JSON_POINT = "point";

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::Point point x key
*/
static constexpr const char * RCL_JSON_POINT_X = "x";

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::Point point y key
*/
static constexpr const char * RCL_JSON_POINT_Y = "y";

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::Point point z key
*/
static constexpr const char * RCL_JSON_POINT_Z = "z";

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::Quaternion quaternion key
*/
static constexpr const char * RCL_JSON_QUATERNION = "quaternion";

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::Quaternion quaternion x key
*/
static constexpr const char * RCL_JSON_QUATERNION_X = "x";

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::Quaternion quaternion y key
*/
static constexpr const char * RCL_JSON_QUATERNION_Y = "y";

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::Quaternion quaternion z key
*/
static constexpr const char * RCL_JSON_QUATERNION_Z = "z";

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::Quaternion quaternion w key
*/
static constexpr const char * RCL_JSON_QUATERNION_W = "w";

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::Pose pose key
*/
static constexpr const char * RCL_JSON_POSE = "pose";

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::Pose position key
*/
static constexpr const char * RCL_JSON_POSITION = "position";

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::Pose orientation key
*/
static constexpr const char * RCL_JSON_ORIENTATION = "orientation";

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::Vector3 vector3 key
*/
static constexpr const char * RCL_JSON_VECTOR3 = "vector3";

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::Vector3 vector3 x key
*/
static constexpr const char * RCL_JSON_VECTOR3_X = "x";

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::Vector3 vector3 y key
*/
static constexpr const char * RCL_JSON_VECTOR3_Y = "y";

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::Vector3 vector3 z key
*/
static constexpr const char * RCL_JSON_VECTOR3_Z = "z";

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::Twist twist key
*/
static constexpr const char * RCL_JSON_TWIST = "twist";

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::Twist linear key
*/
static constexpr const char * RCL_JSON_TWIST_LINEAR = "linear";

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::Twist angular key
*/
static constexpr const char * RCL_JSON_TWIST_ANGULAR = "angular";

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::TwistWithCovariance flag
*/
static constexpr const char * RCL_JSON_TWIST_WITH_COVARIANCE_FLAG = "TwistWithCovariance";

/**
 * @brief static const instance for define jsoncpp covariance key
*/
static constexpr const char * RCL_JSON_COVARIANCE = "covariance";

/**
 * @brief static const instance for define size of jsoncpp pose covariance
*/
static constexpr const int & RCL_JSON_POSE_COVARIANCE_SIZE_DEFAULT = 36;

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::PoseStamped pose_stamped key
*/
static constexpr const char * RCL_JSON_POSE_STAMPED = "pose_stamped";

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::PoseWithCovariance pose_with_covariance key
*/
static constexpr const char * RCL_JSON_POSE_WITH_COVARIANCE = "pose_with_covariance";

/**
 * @brief static const instance for define jsoncpp geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance_stamped key
*/
static constexpr const char * RCL_JSON_POSE_WITH_COVARIANCE_STAMPED = "pose_with_covariance_stamped";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::LaserScan flag
*/
static constexpr const char * RCL_JSON_LASER_SCAN_FLAG = "LaserScan";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::LaserScan angle_min key
*/
static constexpr const char * RCL_JSON_LASER_SCAN_ANGLE_MIN = "angle_min";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::LaserScan angle_max key
*/
static constexpr const char * RCL_JSON_LASER_SCAN_ANGLE_MAX = "angle_max";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::LaserScan angle_increment key
*/
static constexpr const char * RCL_JSON_LASER_SCAN_ANGLE_INCREMENT = "angle_increment";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::LaserScan time_increment key
*/
static constexpr const char * RCL_JSON_LASER_SCAN_TIME_INCREMENT = "time_increment";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::LaserScan scan_time key
*/
static constexpr const char * RCL_JSON_LASER_SCAN_SCAN_TIME = "scan_time";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::LaserScan range_min key
*/
static constexpr const char * RCL_JSON_LASER_SCAN_RANGE_MIN = "range_min";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::LaserScan range_max key
*/
static constexpr const char * RCL_JSON_LASER_SCAN_RANGE_MAX = "range_max";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::LaserScan ranges key
*/
static constexpr const char * RCL_JSON_LASER_SCAN_RANGES = "ranges";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::LaserScan intensities key
*/
static constexpr const char * RCL_JSON_LASER_SCAN_INTENSITIES = "intensities";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::Imu flag
*/
static constexpr const char * RCL_JSON_IMU_FLAG = "Imu";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::Imu angular_velocity key
*/
static constexpr const char * RCL_JSON_ANGULAR_VELOCITY = "angular_velocity";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::Imu linear_acceleration key
*/
static constexpr const char * RCL_JSON_LINEAR_ACCELERATION = "linear_acceleration";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::Imu orientation_covariance key
*/
static constexpr const char * RCL_JSON_ORIENTATION_COVARIANCE = "orientation_covariance";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::Imu angular_velocity_covariance key
*/
static constexpr const char * RCL_JSON_ANGULAR_VELOCITY_COVARIANCE = "angular_velocity_covariance";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::Imu linear_acceleration_covariance key
*/
static constexpr const char * RCL_JSON_LINEAR_ACCELERATION_COVARIANCE = "linear_acceleration_covariance";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::NavSatFix flag
*/
static constexpr const char * RCL_JSON_NAV_SAT_STATUS = "NavSatStatus";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::NavSatStatus flag
*/
static constexpr const char * RCL_JSON_NAV_SAT_STATUS_STATUS_NO_FIX = "STATUS_NO_FIX";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::NavSatStatus STATUS_FIX key
*/
static constexpr const char * RCL_JSON_NAV_SAT_STATUS_STATUS_FIX = "STATUS_FIX";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::NavSatStatus STATUS_SBAS_FIX key
*/
static constexpr const char * RCL_JSON_NAV_SAT_STATUS_STATUS_SBAS_FIX = "STATUS_SBAS_FIX";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::NavSatStatus STATUS_GBAS_FIX key
*/
static constexpr const char * RCL_JSON_NAV_SAT_STATUS_STATUS_GBAS_FIX = "STATUS_GBAS_FIX";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::NavSatStatus status key
*/
static constexpr const char * RCL_JSON_NAV_SAT_STATUS_STATUS = "status";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::NavSatStatus SERVICE_GPS key
*/
static constexpr const char * RCL_JSON_NAV_SAT_STATUS_SERVICE_GPS = "SERVICE_GPS";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::NavSatStatus SERVICE_GLONASS key
*/
static constexpr const char * RCL_JSON_NAV_SAT_STATUS_SERVICE_GLONASS = "SERVICE_GLONASS";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::NavSatStatus SERVICE_COMPASS key
*/
static constexpr const char * RCL_JSON_NAV_SAT_STATUS_SERVICE_COMPASS = "SERVICE_COMPASS";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::NavSatStatus SERVICE_GALILEO key
*/
static constexpr const char * RCL_JSON_NAV_SAT_STATUS_SERVICE_GALILEO = "SERVICE_GALILEO";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::NavSatStatus service key
*/
static constexpr const char * RCL_JSON_NAV_SAT_STATUS_SERVICE = "service";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::NavSatFix flag
*/
static constexpr const char * RCL_JSON_NAV_SAT_FIX_FLAG = "NavSatFix";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::NavSatFix status key
*/
static constexpr const char * RCL_JSON_NAV_SAT_FIX_STATUS = "status";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::NavSatFix latitude key
*/
static constexpr const char * RCL_JSON_NAV_SAT_FIX_LATITUDE = "latitude";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::NavSatFix altitude key
*/
static constexpr const char * RCL_JSON_NAV_SAT_FIX_ALTITUDE = "altitude";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::NavSatFix position_covariance key
*/
static constexpr const char * RCL_JSON_NAV_SAT_FIX_POSITION_COVARIANCE = "position_covariance";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::NavSatFix COVARIANCE_TYPE_UNKNOWN key
*/
static constexpr const char * RCL_JSON_NAV_SAT_FIX_COVARIANCE_TYPE_UNKNOWN = "COVARIANCE_TYPE_UNKNOWN";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::NavSatFix COVARIANCE_TYPE_APPROXIMATED key
*/
static constexpr const char * RCL_JSON_NAV_SAT_FIX_COVARIANCE_TYPE_APPROXIMATED = "COVARIANCE_TYPE_APPROXIMATED";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::NavSatFix COVARIANCE_TYPE_DIAGONAL_KNOWN key
*/
static constexpr const char * RCL_JSON_NAV_SAT_FIX_COVARIANCE_TYPE_DIAGONAL_KNOWN = "COVARIANCE_TYPE_DIAGONAL_KNOWN";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::NavSatFix COVARIANCE_TYPE_KNOWN key
*/
static constexpr const char * RCL_JSON_NAV_SAT_FIX_COVARIANCE_TYPE_KNOWN = "COVARIANCE_TYPE_KNOWN";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::NavSatFix position_covariance_type key
*/
static constexpr const char * RCL_JSON_NAV_SAT_FIX_POSITION_COVARIANCE_TYPE = "position_covariance_type";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState flag
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_FLAG = "BatteryState";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState POWER_SUPPLY_STATUS_UNKNOWN key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_STATUS_UNKNOWN = "POWER_SUPPLY_STATUS_UNKNOWN";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState POWER_SUPPLY_STATUS_CHARGING key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_STATUS_CHARGING = "POWER_SUPPLY_STATUS_CHARGING";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState POWER_SUPPLY_STATUS_DISCHARGING key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_STATUS_DISCHARGING = "POWER_SUPPLY_STATUS_DISCHARGING";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState POWER_SUPPLY_STATUS_NOT_CHARGING key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_STATUS_NOT_CHARGING = "POWER_SUPPLY_STATUS_NOT_CHARGING";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState POWER_SUPPLY_STATUS_FULL key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_STATUS_FULL = "POWER_SUPPLY_STATUS_FULL";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState POWER_SUPPLY_HEALTH_UNKNOWN key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_HEALTH_UNKNOWN = "POWER_SUPPLY_HEALTH_UNKNOWN";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState POWER_SUPPLY_HEALTH_GOOD key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_HEALTH_GOOD = "POWER_SUPPLY_HEALTH_GOOD";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState POWER_SUPPLY_HEALTH_OVERHEAT key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_HEALTH_OVERHEAT = "POWER_SUPPLY_HEALTH_OVERHEAT";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState POWER_SUPPLY_HEALTH_DEAD key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_HEALTH_DEAD = "POWER_SUPPLY_HEALTH_DEAD";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState POWER_SUPPLY_HEALTH_OVERVOLTAGE key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_HEALTH_OVERVOLTAGE = "POWER_SUPPLY_HEALTH_OVERVOLTAGE";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState POWER_SUPPLY_HEALTH_UNSPEC_FAILURE key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = "POWER_SUPPLY_HEALTH_UNSPEC_FAILURE";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState POWER_SUPPLY_HEALTH_COLD key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_HEALTH_COLD = "POWER_SUPPLY_HEALTH_COLD";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = "POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = "POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState POWER_SUPPLY_TECHNOLOGY_UNKNOWN key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_UNKNOWN = "POWER_SUPPLY_TECHNOLOGY_UNKNOWN";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState POWER_SUPPLY_TECHNOLOGY_NIMH key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_NIMH = "POWER_SUPPLY_TECHNOLOGY_NIMH";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState POWER_SUPPLY_TECHNOLOGY_LION key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_LION = "POWER_SUPPLY_TECHNOLOGY_LION";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState POWER_SUPPLY_TECHNOLOGY_LIPO key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_LIPO = "POWER_SUPPLY_TECHNOLOGY_LIPO";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState POWER_SUPPLY_TECHNOLOGY_LIFE key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_LIFE = "POWER_SUPPLY_TECHNOLOGY_LIFE";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState POWER_SUPPLY_TECHNOLOGY_NICD key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_NICD = "POWER_SUPPLY_TECHNOLOGY_NICD";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState POWER_SUPPLY_TECHNOLOGY_LIMN key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_LIMN = "POWER_SUPPLY_TECHNOLOGY_LIMN";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState voltage key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_VOLTAGE = "voltage";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState temperature key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_TEMPERATURE = "temperature";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState current key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_CURRENT = "current";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState charge key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_CHARGE = "charge";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState capacity key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_CAPACITY = "capacity";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState design_capacity key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_DESIGN_CAPACITY = "design_capacity";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState percentage key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_PERCENTAGE = "percentage";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState power_supply_status key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_STATUS = "power_supply_status";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState power_supply_health key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_HEALTH = "power_supply_health";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState power_supply_technology key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY = "power_supply_technology";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState present key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_PRESENT = "present";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState cell_voltage key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_CELL_VOLTAGE = "cell_voltage";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState cell_temperature key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_CELL_TEMPERATURE = "cell_temperature";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState location key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_LOCATION = "location";

/**
 * @brief static const instance for define jsoncpp sensor_msgs::msg::BatteryState serial_number key
*/
static constexpr const char * RCL_JSON_BATTERY_STATE_SERIAL_NUMBER = "serial_number";

/**
 * @brief static const instance for define jsoncpp nav_msgs::msg::Odometry flag
*/
static constexpr const char * RCL_JSON_ODOMETRY_FLAG = "Odometry";

/**
 * @brief static const instance for define message type of std_msgs::msg
*/
static constexpr const char * RCL_STD_MSGS_TYPE = "std_msgs/msg/";

/**
 * @brief static const instance for define message type of geometry_msgs::msg
*/
static constexpr const char * RCL_GEOMETRY_MSGS_TYPE = "geometry_msgs/msg/";

/**
 * @brief static const instance for define message type of sensor_msgs::msg
*/
static constexpr const char * RCL_SENSOR_MSGS_TYPE = "sensor_msgs/msg/";

/**
 * @brief static const instance for define message type of nav_msgs::msg
*/
static constexpr const char * RCL_NAV_MSGS_TYPE = "nav_msgs/msg/";

/**
 * @brief static const instance for define message type of nav2_msgs::msg
*/
static constexpr const char * RCL_NAV2_MSGS_TYPE = "nav2_msgs/msg/";

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