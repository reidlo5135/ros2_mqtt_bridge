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
 * @file message_converter.hpp
 * @author reidlo(naru5135@wavem.net)
 * @date 2023-06-27
 * @brief header file for convert message between ROS2 - MQTT
*/

#ifndef ROS2_MQTT_BRIDGE_MESSAGE_CONVERTER__HPP
#define ROS2_MQTT_BRIDGE_MESSAGE_CONVERTER__HPP

#include "ros2_mqtt_definer/define_container.hpp"

namespace ros2_mqtt_bridge {

    class StdMessageConverter {
        public :
            explicit StdMessageConverter(){};
            virtual ~StdMessageConverter(){};

            Json::Value std_msgs_header_to_json(const std_msgs::msg::Header::SharedPtr rcl_std_msgs_header_ptr) {
                Json::Value std_msgs_header_json;

                try {
                    std_msgs_header_json["frame_id"] = rcl_std_msgs_header_ptr->frame_id;
                    std_msgs_header_json["seq"] = rcl_std_msgs_header_ptr->stamp.sec;
                    std_msgs_header_json["stamp"] = rcl_std_msgs_header_ptr->stamp.sec + rcl_std_msgs_header_ptr->stamp.nanosec * 1e-9;
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "error occurred during convert [%sheader] to json\n\twith : [%s]", RCL_STD_MSGS_TYPE, json_expn.what());
                    RCLCPP_LINE_ERROR();
                }

                return std_msgs_header_json;
            };

            const std::string & std_msgs_string_to_json_string(const std_msgs::msg::String::SharedPtr rcl_std_msgs_string_ptr) {
                Json::Value std_msgs_string_json;

                try {
                    std_msgs_string_json["data"] = rcl_std_msgs_string_ptr->data;
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "error occurred during convert [%sstring] to json\n\twith : [%s]", RCL_STD_MSGS_TYPE, json_expn.what());
                    RCLCPP_LINE_ERROR();
                }

                const std::string & std_msgs_string_json_string = Json::StyledWriter().write(std_msgs_string_json);

                return std_msgs_string_json_string;
            };

            std_msgs::msg::String::UniquePtr json_to_std_msgs_string(const std::string & raw_json_string) {
                Json::Value std_string_json;
                Json::Reader json_reader;
                std_msgs::msg::String::UniquePtr rcl_std_msgs_string_ptr = std::make_unique<std_msgs::msg::String>();
                
                try {
                    bool is_parsing_succeeded = json_reader.parse(raw_json_string, std_string_json);

                    if(is_parsing_succeeded) {
                        RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "parsing JSON raw json string to [%sstring] completed : ", RCL_STD_MSGS_TYPE);
                        RCLCPP_LINE_INFO();
                        rcl_std_msgs_string_ptr->set__data(std_string_json.get("data", "nullstr").asString());
                    } else {
                        RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "error occurred during parse [%sstring] to json\n\twith : [%s]", RCL_STD_MSGS_TYPE, json_reader.getFormatedErrorMessages());
                        RCLCPP_LINE_ERROR();
                    }
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "error occurred during convert [%sstring] to json\n\twith : [%s]", RCL_STD_MSGS_TYPE, json_expn.what());
                    RCLCPP_LINE_ERROR();
                }

                return rcl_std_msgs_string_ptr;
            };

            std_msgs::msg::Header::UniquePtr json_to_std_msgs_header(Json::Value raw_header_json) {
                std_msgs::msg::Header::UniquePtr rcl_std_msgs_header_ptr = std::make_unique<std_msgs::msg::Header>();
                builtin_interfaces::msg::Time::UniquePtr rcl_builtin_interfaces_stamp_ptr = std::make_unique<builtin_interfaces::msg::Time>();

                try {
                    rcl_std_msgs_header_ptr->set__frame_id(raw_header_json.get("frame_id", "nullstr").asString());

                    rcl_builtin_interfaces_stamp_ptr->set__sec(raw_header_json.get("sec", 0.0).asDouble());
                    rcl_builtin_interfaces_stamp_ptr->set__nanosec(raw_header_json.get("nanosec", 0.0).asDouble());

                    rcl_std_msgs_header_ptr->set__stamp(std::move(*rcl_builtin_interfaces_stamp_ptr));
                } catch(Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "error occurred during convert [%sheader] to json\n\twith : [%s]", RCL_STD_MSGS_TYPE, json_expn.what());
                    RCLCPP_LINE_ERROR();
                }

                return rcl_std_msgs_header_ptr;
            }
    };
}

#endif