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
 * @file bridge_tester.cpp
 * @author reidlo(naru5135@wavem.net)
 * @date 2023-06-27
 * @brief implementation file for rclcpp::Node what test "ros2_mqtt_bridge"
*/

/**
 * @brief include/ros2_mqtt_bridge/bridge_tester.hpp include area
*/
#include "ros2_mqtt_bridge/bridge_tester.hpp"

/**
* Create a new this class' instance and extends rclcpp::Node
* @brief Default Constructor
* @see rclcpp::Node
*/
ros2_mqtt_bridge::RCLMQTTBridgeTester::RCLMQTTBridgeTester()
: Node(RCL_TESTER_NODE_NAME) {
    rcl_node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});

    rcl_connection_manager_ptr_ = std::make_shared<ros2_mqtt_bridge::RCLConnectionManager>();

    rcl_timer_base_ptr_ = rcl_node_ptr_->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&ros2_mqtt_bridge::RCLMQTTBridgeTester::bridge_test_rcl_to_mqtt, this)
    );

    rcl_chatter_publisher_ptr_ = rcl_connection_manager_ptr_->register_publisher<std_msgs::msg::String>(
        rcl_node_ptr_,
        RCL_CHATTER_TOPIC
    );

    rcl_map_publisher_ptr_ = rcl_connection_manager_ptr_->register_publisher<nav_msgs::msg::OccupancyGrid>(
        rcl_node_ptr_,
        RCL_MAP_TOPIC
    );

    rcl_robot_pose_publisher_ptr_ = rcl_connection_manager_ptr_->register_publisher<geometry_msgs::msg::Pose>(
        rcl_node_ptr_,
        RCL_ROBOT_POSE_TOPIC
    );

    rcl_scan_publisher_ptr_ = rcl_connection_manager_ptr_->register_publisher<sensor_msgs::msg::LaserScan>(
        rcl_node_ptr_,
        RCL_SCAN_TOPIC
    );

    rcl_tf_publisher_ptr_ = rcl_connection_manager_ptr_->register_publisher<tf2_msgs::msg::TFMessage>(
        rcl_node_ptr_,
        RCL_TF_TOPIC
    );

    rcl_tf_static_publisher_ptr_ = rcl_connection_manager_ptr_->register_publisher<tf2_msgs::msg::TFMessage>(
        rcl_node_ptr_,
        RCL_TF_STATIC_TOPIC
    );

    rcl_cmd_vel_publisher_ptr_ = rcl_connection_manager_ptr_->register_publisher<geometry_msgs::msg::Twist>(
        rcl_node_ptr_,
        RCL_CMD_VEL_TOPIC
    );

    rcl_odom_publisher_ptr_ = rcl_connection_manager_ptr_->register_publisher<nav_msgs::msg::Odometry>(
        rcl_node_ptr_,
        RCL_ODOM_TOPIC
    );

    rcl_can_control_hardware_publisher_ptr_ = rcl_connection_manager_ptr_->register_publisher<can_msgs::msg::ControlHardware>(
        rcl_node_ptr_,
        RCL_CAN_CONTROL_HARDWARE_TOPIC
    );

    rcl_ublox_fix_publisher_ptr_ = rcl_connection_manager_ptr_->register_publisher<sensor_msgs::msg::NavSatFix>(
        rcl_node_ptr_,
        RCL_UBLOX_FIX_TOPIC
    );
}

/**
* Destroy this class' instance
* @brief Default Destructor
*/
ros2_mqtt_bridge::RCLMQTTBridgeTester::~RCLMQTTBridgeTester() {

}

void ros2_mqtt_bridge::RCLMQTTBridgeTester::bridge_test_mqtt_to_rcl() {

}

std_msgs::msg::Header::UniquePtr ros2_mqtt_bridge::RCLMQTTBridgeTester::rcl_build_std_msgs_header(const char * frame_id) {
    builtin_interfaces::msg::Time::UniquePtr rcl_built_in_interfaces_time_ptr = std::make_unique<builtin_interfaces::msg::Time>();

    const std::chrono::_V2::system_clock::time_point & now = std::chrono::system_clock::now();
    const int64_t & seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    const int32_t & current_time = static_cast<int32_t>(seconds);

    rcl_built_in_interfaces_time_ptr->set__sec(current_time);
    rcl_built_in_interfaces_time_ptr->set__nanosec((current_time * 1e-9));

    std_msgs::msg::Header::UniquePtr rcl_std_msgs_header_ptr = std::make_unique<std_msgs::msg::Header>();

    rcl_std_msgs_header_ptr->set__frame_id(frame_id);
    rcl_std_msgs_header_ptr->set__stamp(*rcl_built_in_interfaces_time_ptr);
    
    return rcl_std_msgs_header_ptr;
}

void ros2_mqtt_bridge::RCLMQTTBridgeTester::flag_rcl_publish(const char * target_rcl_topic) {
    RCLCPP_INFO(
        rcl_node_ptr_->get_logger(),
        "RCL published to [%s]",
        target_rcl_topic
    );
    RCLCPP_LINE_INFO();
}

void ros2_mqtt_bridge::RCLMQTTBridgeTester::rcl_publish_chatter() {
    std_msgs::msg::String::UniquePtr rcl_std_msgs_string_ptr = std::make_unique<std_msgs::msg::String>();

    const char * rcl_std_msgs_string_data = "HELLO";
    rcl_std_msgs_string_ptr->set__data(rcl_std_msgs_string_data);

    rcl_chatter_publisher_ptr_->publish(std::move(*rcl_std_msgs_string_ptr));

    this->flag_rcl_publish("/chatter");
}

void ros2_mqtt_bridge::RCLMQTTBridgeTester::rcl_publish_map() {
    nav_msgs::msg::MapMetaData::UniquePtr rcl_nav_msgs_map_meta_data_ptr = std::make_unique<nav_msgs::msg::MapMetaData>();
    nav_msgs::msg::OccupancyGrid::UniquePtr rcl_nav_msgs_occupancy_grid_ptr = std::make_unique<nav_msgs::msg::OccupancyGrid>();

    std_msgs::msg::Header::UniquePtr rcl_std_msgs_header_ptr = this->rcl_build_std_msgs_header("occupancy_grid");
    rcl_nav_msgs_map_meta_data_ptr->set__width(5);
    rcl_nav_msgs_map_meta_data_ptr->set__height(5);
    std::vector<int8_t, std::allocator<int8_t>> rcl_nav_msgs_occupancy_grid_data_vec = {0, -1, 100, 0, 100};
}

void ros2_mqtt_bridge::RCLMQTTBridgeTester::rcl_publish_robot_pose() {

}

void ros2_mqtt_bridge::RCLMQTTBridgeTester::rcl_publish_scan() {

}

void ros2_mqtt_bridge::RCLMQTTBridgeTester::rcl_publish_tf() {

}

void ros2_mqtt_bridge::RCLMQTTBridgeTester::rcl_publish_tf_static() {

}

void ros2_mqtt_bridge::RCLMQTTBridgeTester::rcl_publish_cmd_vel() {

}

void ros2_mqtt_bridge::RCLMQTTBridgeTester::rcl_publish_odom() {

}

void ros2_mqtt_bridge::RCLMQTTBridgeTester::rcl_publish_can_control_hardware() {

}

void ros2_mqtt_bridge::RCLMQTTBridgeTester::rcl_publish_ublox_fix() {

}

void ros2_mqtt_bridge::RCLMQTTBridgeTester::bridge_test_rcl_to_mqtt() {
    this->rcl_publish_chatter();
}

/**
* Function for handle signal_input when program exit
* @param signal_input The signal_input of input
* @return void
* @see signal_input.h
*/
void ros2_mqtt_bridge::RCLMQTTBridgeTester::sig_handler(int signal_input) {
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "stopped with SIG [%i]", signal_input);
    RCLCPP_LINE_INFO();
	signal(signal_input, SIG_IGN);
	exit(RCL_EXIT_FLAG);
}

int main(int argc, const char * const * argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> rcl_node_ptr = std::make_shared<ros2_mqtt_bridge::RCLMQTTBridgeTester>();

    signal(SIGINT, &ros2_mqtt_bridge::RCLMQTTBridgeTester::sig_handler);
    signal(SIGTSTP, &ros2_mqtt_bridge::RCLMQTTBridgeTester::sig_handler);

    rclcpp::executors::SingleThreadedExecutor rcl_single_thread_executor;
    while(rclcpp::ok()) {
        rcl_single_thread_executor.spin_node_once(rcl_node_ptr);
    }
    rclcpp::shutdown();
    return 0;
}