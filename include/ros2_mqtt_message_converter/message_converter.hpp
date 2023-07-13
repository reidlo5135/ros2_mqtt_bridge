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

    /**
     * @class StdMessageConverter
     * @brief final class for implements converting functions between std_msgs::msg - Json::Value
    */
    class StdMessageConverter final {
        public :
            /**
             * Create a new this class' instance
             * @brief Default Constructor
            */
            inline explicit StdMessageConverter() {

            };

            /**
             * Destroy this class' instance
             * @brief Default Destructor
            */
            inline virtual ~StdMessageConverter() {

            };

            /**
             * @brief inline function for convert std_msgs::msg::Header into Json::Value
             * @param rcl_header_ptr target std_msgs::msg::Header::SharedPtr data
             * @return header_json Json::Value
            */
            inline Json::Value header_to_json(const std_msgs::msg::Header rcl_header) {
                Json::Value header_json;

                try {
                    header_json[RCL_JSON_HEADER_FRAME_ID] = rcl_header.frame_id;
                    header_json[RCL_JSON_HEADER_SEQ] = rcl_header.stamp.sec;
                    header_json[RCL_JSON_HEADER_STAMP] = rcl_header.stamp.sec + rcl_header.stamp.nanosec * 1e-9;
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert [%s%s] to json\n\twith : [%s]", 
                        RCL_STD_MSGS_TYPE, 
                        RCL_JSON_HEADER, 
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                return header_json;
            };

            /**
             * @brief inline function for convert std_msgs::msg::String into Json style string
             * @param rcl_string_ptr target std_msgs::msg::String::SharedPtr data
             * @return string_json_string const std::string &
            */
            inline const std::string & string_to_json_string(const std_msgs::msg::String::SharedPtr rcl_string_ptr) {
                Json::Value string_json;

                try {
                    string_json[RCL_JSON_STRING_DATA] = rcl_string_ptr->data;
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert [%s%s] to json\n\twith : [%s]",
                        RCL_STD_MSGS_TYPE,
                        RCL_JSON_STRING,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                const std::string & string_json_string = Json::StyledWriter().write(string_json);

                return string_json_string;
            };

            /**
             * @brief inline function for convert Json style string into std_msgs::msg::String::UniquePtr
             * @param raw_json_string target Json style string
             * @return rcl_std_msgs_string_ptr std_msgs::msg::String::UniquePtr
            */
            inline std_msgs::msg::String::UniquePtr json_to_string(const std::string & raw_json_string) {
                Json::Value string_json;
                Json::Reader json_reader;
                std_msgs::msg::String::UniquePtr rcl_std_msgs_string_ptr = std::make_unique<std_msgs::msg::String>();
                
                try {
                    bool is_parsing_succeeded = json_reader.parse(raw_json_string, string_json);

                    if(is_parsing_succeeded) {
                        RCUTILS_LOG_INFO_NAMED(
                            RCL_NODE_NAME, 
                            "parsing JSON raw json string to [%s%s] completed : ", 
                            RCL_STD_MSGS_TYPE, 
                            RCL_JSON_STRING
                        );
                        RCLCPP_LINE_INFO();

                        const std::string & string_json_data = string_json.get(RCL_JSON_STRING_DATA, RCL_JSON_STRING_DEFAULT).asString();
                        rcl_std_msgs_string_ptr->set__data(string_json_data);
                    } else {
                        const std::string & json_formatted_error_message = json_reader.getFormatedErrorMessages();

                        RCUTILS_LOG_ERROR_NAMED(
                            RCL_NODE_NAME, 
                            "error occurred during parse json to [%s%s]\n\twith : [%s]",
                            RCL_STD_MSGS_TYPE,
                            RCL_JSON_STRING,
                            json_formatted_error_message.c_str()
                        );
                        RCLCPP_LINE_ERROR();
                    }
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert json to [%s%s]\n\twith : [%s]",
                        RCL_STD_MSGS_TYPE,
                        RCL_JSON_STRING,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                return rcl_std_msgs_string_ptr;
            };

            /**
             * @brief inline function for convert Json::Value into std_msgs::msg::Header::UniquePtr
             * @param raw_header_json target Json::Value
             * @return rcl_std_msgs_header_ptr std_msgs::msg::Header::UniquePtr
            */
            inline std_msgs::msg::Header::UniquePtr json_to_header(const Json::Value & raw_header_json) {
                std_msgs::msg::Header::UniquePtr rcl_std_msgs_header_ptr = std::make_unique<std_msgs::msg::Header>();
                builtin_interfaces::msg::Time::UniquePtr rcl_builtin_interfaces_stamp_ptr = std::make_unique<builtin_interfaces::msg::Time>();

                try {
                    const std::string & raw_header_json_frame_id = raw_header_json.get(RCL_JSON_HEADER_FRAME_ID, RCL_JSON_STRING_DEFAULT).asString();
                    rcl_std_msgs_header_ptr->set__frame_id(raw_header_json_frame_id);

                    const double & raw_header_json_sec = raw_header_json.get(RCL_JSON_HEADER_SEC, RCL_JSON_DOUBLE_DEFAULT).asDouble();
                    rcl_builtin_interfaces_stamp_ptr->set__sec(raw_header_json_sec);

                    const double & raw_header_json_nanosec = raw_header_json.get(RCL_JSON_HEADER_NANOSEC, RCL_JSON_DOUBLE_DEFAULT).asDouble();
                    rcl_builtin_interfaces_stamp_ptr->set__nanosec(raw_header_json_nanosec);

                    rcl_std_msgs_header_ptr->set__stamp(std::move(*rcl_builtin_interfaces_stamp_ptr));
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert json to [%s%s]\n\twith : [%s]",
                        RCL_STD_MSGS_TYPE,
                        RCL_JSON_HEADER,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                return rcl_std_msgs_header_ptr;
            }
    };

    /**
     * @class GeometryMessageConverter
     * @brief final class for implements converting functions between geometry::msg - Json::Value
    */
    class GeometryMessageConverter final {
        private :
            /**
             * @brief shared pointer for ros2_mqtt_bridge::StdMessageConverter
             * @see ros2_mqtt_bridge::StdMessageConverter
            */
            std::shared_ptr<ros2_mqtt_bridge::StdMessageConverter> rcl_std_msgs_json_converter_ptr_;
        public :
            /**
             * Create a new this class' instance
             * @brief Default Constructor
            */
            inline explicit GeometryMessageConverter() {
                rcl_std_msgs_json_converter_ptr_ = std::make_shared<ros2_mqtt_bridge::StdMessageConverter>();
            };
            
            /**
             * Destroy this class' instance
             * @brief Default Destructor
            */
            inline virtual ~GeometryMessageConverter() {

            };

            /**
             * @brief inline function for convert geometry_msgs::msg::Point into Json::Value
             * @param rcl_point target geometry_msgs::msg::Point
             * @return point_json Json::Value
            */
            inline Json::Value point_to_json(const geometry_msgs::msg::Point rcl_point) {
                Json::Value point_json;

                try {
                    point_json[RCL_JSON_POINT_X] = rcl_point.x;
                    point_json[RCL_JSON_POINT_Y] = rcl_point.y;
                    point_json[RCL_JSON_POINT_Z] = rcl_point.z;
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert [%s%s] to json\n\twith : [%s]",
                        RCL_GEOMETRY_MSGS_TYPE,
                        RCL_JSON_POINT,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                return point_json;
            };

            /**
             * @brief inline function for convert Json::Value into geometry_msgs::msg::Point::UniquePtr
             * @param raw_point_json target const Json::Value &
             * @return rcl_point_ptr geometry_msgs::msg::Point::UniquePtr
            */
            inline geometry_msgs::msg::Point::UniquePtr json_to_point(const Json::Value & raw_point_json) {
                geometry_msgs::msg::Point::UniquePtr rcl_point_ptr = std::make_unique<geometry_msgs::msg::Point>();

                try {
                    const double & raw_point_json_x = raw_point_json.get(RCL_JSON_POINT_X, RCL_JSON_DOUBLE_DEFAULT).asDouble();
                    rcl_point_ptr->set__x(raw_point_json_x);

                    const double & raw_point_json_y = raw_point_json.get(RCL_JSON_POINT_Y, RCL_JSON_DOUBLE_DEFAULT).asDouble();
                    rcl_point_ptr->set__x(raw_point_json_y);

                    const double & raw_point_json_z = raw_point_json.get(RCL_JSON_POINT_Z, RCL_JSON_DOUBLE_DEFAULT).asDouble();
                    rcl_point_ptr->set__x(raw_point_json_z);
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert json to [%s%s]\n\twith : [%s]",
                        RCL_GEOMETRY_MSGS_TYPE,
                        RCL_JSON_POINT,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                return rcl_point_ptr;
            };

            /**
             * @brief inline function for convert geometry_msgs::msg::Quaternion into Json::Value
             * @param rcl_quaternion target geometry_msgs::msg::Quaternion
             * @return quaternion_json Json::Value
            */
            inline Json::Value quaternion_to_json(const geometry_msgs::msg::Quaternion rcl_quaternion) {
                Json::Value quaternion_json;

                try {
                    quaternion_json[RCL_JSON_QUATERNION_X] = rcl_quaternion.x;
                    quaternion_json[RCL_JSON_QUATERNION_Y] = rcl_quaternion.y;
                    quaternion_json[RCL_JSON_QUATERNION_Z] = rcl_quaternion.z;
                    quaternion_json[RCL_JSON_QUATERNION_W] = rcl_quaternion.w;
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert [%s%s] to json\n\twith : [%s]", 
                        RCL_GEOMETRY_MSGS_TYPE,
                        RCL_JSON_QUATERNION,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                return quaternion_json;
            };

            /**
             * @brief inline function for convert Json::Value into geometry_msgs::msg::Quaternion::UniquePtr
             * @param raw_quaternion_json target const Json::Value &
             * @return rcl_quaternion_ptr geometry_msgs::msg::Quaternion::UniquePtr
            */
            inline geometry_msgs::msg::Quaternion::UniquePtr json_to_quaternion(const Json::Value & raw_quaternion_json) {
                geometry_msgs::msg::Quaternion::UniquePtr rcl_quaternion_ptr = std::make_unique<geometry_msgs::msg::Quaternion>();

                try {
                    const double & raw_quaternion_json_x = raw_quaternion_json.get(RCL_JSON_QUATERNION_X, RCL_JSON_DOUBLE_DEFAULT).asDouble();
                    rcl_quaternion_ptr->set__x(raw_quaternion_json_x);

                    const double & raw_quaternion_json_y = raw_quaternion_json.get(RCL_JSON_QUATERNION_Y, RCL_JSON_DOUBLE_DEFAULT).asDouble();
                    rcl_quaternion_ptr->set__x(raw_quaternion_json_y);

                    const double & raw_quaternion_json_z = raw_quaternion_json.get(RCL_JSON_QUATERNION_Z, RCL_JSON_DOUBLE_DEFAULT).asDouble();
                    rcl_quaternion_ptr->set__x(raw_quaternion_json_z);

                    const double & raw_quaternion_json_w = raw_quaternion_json.get(RCL_JSON_QUATERNION_W, RCL_JSON_DOUBLE_DEFAULT).asDouble();
                    rcl_quaternion_ptr->set__x(raw_quaternion_json_w);
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert json to [%s%s]\n\twith : [%s]",
                        RCL_GEOMETRY_MSGS_TYPE,
                        RCL_JSON_QUATERNION,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                return rcl_quaternion_ptr;
            };

            /**
             * @brief inline function for convert geometry_msgs::msg::Pose into Json::Value
             * @param rcl_pose_ptr target geometry_msgs::msg::Pose::SharedPtr
             * @return pose_json_string const std::string &
            */
            inline const std::string & pose_to_json_string(const geometry_msgs::msg::Pose::SharedPtr rcl_pose_ptr) {
                Json::Value pose_json;

                try {
                    pose_json[RCL_JSON_POSITION] = this->point_to_json(rcl_pose_ptr->position);
                    pose_json[RCL_JSON_ORIENTATION] = this->quaternion_to_json(rcl_pose_ptr->orientation);
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert [%s%s] to json\n\twith : [%s]",
                        RCL_GEOMETRY_MSGS_TYPE,
                        RCL_JSON_POSE,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                const std::string & pose_json_string = Json::StyledWriter().write(pose_json);

                return pose_json_string;
            };

            /**
             * @brief inline function for convert Json::Value into geometry_msgs::msg::Pose::UniquePtr
             * @param raw_pose_json target const Json::Value &
             * @return rcl_pose_ptr geometry_msgs::msg::Pose::UniquePtr
            */
            inline geometry_msgs::msg::Pose::UniquePtr json_to_pose(const Json::Value & raw_pose_json) {
                geometry_msgs::msg::Pose::UniquePtr rcl_pose_ptr = std::make_unique<geometry_msgs::msg::Pose>();
                geometry_msgs::msg::Point::UniquePtr rcl_point_ptr;
                geometry_msgs::msg::Quaternion::UniquePtr rcl_quaternion_ptr;

                try {
                    Json::Value point_json = raw_pose_json.get(RCL_JSON_POINT, Json::Value::null);
                    bool is_point_json_null = point_json.isNull();

                    if(!is_point_json_null) {
                        rcl_point_ptr = this->json_to_point(point_json);
                        rcl_pose_ptr->set__position(*rcl_point_ptr);
                    } else {
                        RCUTILS_LOG_ERROR_NAMED(
                            RCL_NODE_NAME,
                            "pose point json isn null..."
                        );
                        RCLCPP_LINE_ERROR();
                    }

                    Json::Value orientation_json = raw_pose_json.get(RCL_JSON_QUATERNION, Json::Value::null);
                    bool is_orientation_json_null = orientation_json.isNull();

                    if(!is_orientation_json_null) {
                        rcl_quaternion_ptr = this->json_to_quaternion(orientation_json);
                        rcl_pose_ptr->set__orientation(*rcl_quaternion_ptr);
                    } else {
                        RCUTILS_LOG_ERROR_NAMED(
                            RCL_NODE_NAME,
                            "pose orientation json isn null..."
                        );
                        RCLCPP_LINE_ERROR();
                    }
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert json to [%s%s]\n\twith : [%s]",
                        RCL_GEOMETRY_MSGS_TYPE,
                        RCL_JSON_POSE,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }
                
                return rcl_pose_ptr;
            };

            /**
             * @brief inline function for convert geometry_msgs::msg::Vector3 into Json::Value
             * @param rcl_vector3 target geometry_msgs::msg::Vector3
             * @return vector3_json Json::Value
            */
            inline Json::Value vector3_to_json(const geometry_msgs::msg::Vector3 rcl_vector3) {
                Json::Value vector3_json;

                try {
                    vector3_json[RCL_JSON_VECTOR3_X] = rcl_vector3.x;
                    vector3_json[RCL_JSON_VECTOR3_Y] = rcl_vector3.y;
                    vector3_json[RCL_JSON_VECTOR3_Z] = rcl_vector3.z;
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert [%s%s] to json\n\twith : [%s]",
                        RCL_GEOMETRY_MSGS_TYPE,
                        RCL_JSON_VECTOR3,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                return vector3_json;
            };

            /**
             * @brief inline function for convert geometry_msgs::msg::Twist into Json style string
             * @param rcl_twist_ptr target geometry_msgs::msg::Twist::SharedPtr
             * @return twist_json_string const std::string &
            */
            inline const std::string & twist_to_json_string(const geometry_msgs::msg::Twist::SharedPtr rcl_twist_ptr) {
                Json::Value twist_json;

                try {
                    twist_json[RCL_JSON_LINEAR] = this->vector3_to_json(rcl_twist_ptr->linear);
                    twist_json[RCL_JSON_ANGULAR] = this->vector3_to_json(rcl_twist_ptr->angular);
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert [%s%s] to json\n\twith : [%s]",
                        RCL_GEOMETRY_MSGS_TYPE,
                        RCL_JSON_TWIST,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                const std::string & twist_json_string = Json::StyledWriter().write(twist_json);

                return twist_json_string;
            };

            /**
             * @brief inline function for convert Json::Value into geometry_msgs::msg::Vector3::UniquePtr
             * @param raw_vector3_json const Json::Value &
             * @return rcl_vector3_ptr geometry_msgs::msg::Vector3::SharedPtr
            */
            inline geometry_msgs::msg::Vector3::UniquePtr json_to_vector3(const Json::Value & raw_vector3_json) {
                geometry_msgs::msg::Vector3::UniquePtr rcl_vector3_ptr = std::make_unique<geometry_msgs::msg::Vector3>();

                try {
                    const double & raw_vector3_x = raw_vector3_json.get(RCL_JSON_VECTOR3_X, RCL_JSON_DOUBLE_DEFAULT).asDouble();
                    rcl_vector3_ptr->set__x(raw_vector3_x);

                    const double & raw_vector3_y = raw_vector3_json.get(RCL_JSON_VECTOR3_Y, RCL_JSON_DOUBLE_DEFAULT).asDouble();
                    rcl_vector3_ptr->set__y(raw_vector3_y);

                    const double & raw_vector3_z = raw_vector3_json.get(RCL_JSON_VECTOR3_Z, RCL_JSON_DOUBLE_DEFAULT).asDouble();
                    rcl_vector3_ptr->set__z(raw_vector3_z);
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert [%s%s] to json\n\twith : [%s]",
                        RCL_GEOMETRY_MSGS_TYPE,
                        RCL_JSON_VECTOR3,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                return rcl_vector3_ptr;
            };

            /**
             * @brief inline function for convert Json style string into geometry_msgs::msg::Twist::UniquePtr
             * @param raw_twist_json_string target const std::string &
             * @return rcl_twist_ptr geometry_msgs::msg::Twist::UniquePtr
            */
            inline geometry_msgs::msg::Twist::UniquePtr json_to_twist(const std::string & raw_twist_json_string) {
                Json::Value twist_json;
                Json::Reader json_reader;
                geometry_msgs::msg::Twist::UniquePtr rcl_twist_ptr = std::make_unique<geometry_msgs::msg::Twist>();

                try {
                    bool is_twist_json_parsing_succeeded = json_reader.parse(raw_twist_json_string, twist_json);

                    if(is_twist_json_parsing_succeeded) {
                        RCUTILS_LOG_INFO_NAMED(
                            RCL_NODE_NAME,
                            "parsing twist json completed : [%s]",
                            twist_json.asCString()
                        );
                        RCLCPP_LINE_INFO();

                        Json::Value linear_json = twist_json.get(RCL_JSON_LINEAR, Json::Value::null);
                        bool is_linear_json_null = linear_json.isNull();

                        if(!is_linear_json_null) {
                            RCUTILS_LOG_INFO_NAMED(
                                RCL_NODE_NAME,
                                "twist linear json : [%s]",
                                linear_json.asCString()
                            );
                            RCLCPP_LINE_INFO();

                            rcl_twist_ptr->set__linear(*(this->json_to_vector3(linear_json)));
                        } else {
                            RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "linear json is null");
                            RCLCPP_LINE_ERROR();
                        }

                        Json::Value angular_json = twist_json.get(RCL_JSON_ANGULAR, Json::Value::null);
                        bool is_angular_json_null = angular_json.isNull();

                        if(!is_angular_json_null) {
                            RCUTILS_LOG_INFO_NAMED(
                                RCL_NODE_NAME,
                                "twist angular json : [%s]",
                                angular_json.asCString()
                            );
                            RCLCPP_LINE_INFO();

                            rcl_twist_ptr->set__angular(*(this->json_to_vector3(angular_json)));
                        } else {
                            RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "anuglar json is null");
                            RCLCPP_LINE_ERROR();
                        }
                    } else {
                        const std::string & json_formatted_error_message = json_reader.getFormatedErrorMessages();

                        RCUTILS_LOG_ERROR_NAMED(
                            RCL_NODE_NAME,
                            "parsing twist json error : %s",
                            json_formatted_error_message.c_str()
                        );
                        RCLCPP_LINE_ERROR();
                    }
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert [%s%s] to json\n\twith : [%s]",
                        RCL_GEOMETRY_MSGS_TYPE,
                        RCL_JSON_TWIST,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                return rcl_twist_ptr;
            };

            /**
             * @brief inline function for Json style string into geometry_msgs::msg::PoseStamped::UniquePtr
             * @param raw_pose_stamped_json_string target const std::string &
             * @return rcl_pose_stamped_ptr geometry_msgs::msg::PoseStamped::UniquePtr
            */
            inline geometry_msgs::msg::PoseStamped::UniquePtr json_to_pose_stamped(const std::string & raw_pose_stapmed_json_string) {
                Json::Value pose_stamped_json;
                Json::Reader json_reader;
                geometry_msgs::msg::PoseStamped::UniquePtr rcl_pose_stamped_ptr = std::make_unique<geometry_msgs::msg::PoseStamped>();

                try {
                    bool is_pose_stamped_json_parsing_succeded = json_reader.parse(raw_pose_stapmed_json_string, pose_stamped_json);

                    if(is_pose_stamped_json_parsing_succeded) {
                        RCUTILS_LOG_INFO_NAMED(
                            RCL_NODE_NAME,
                            "parsing pose stamped json completed : [%s]",
                            pose_stamped_json.asCString()
                        );
                        RCLCPP_LINE_INFO();
                        
                        Json::Value header_json = pose_stamped_json.get(RCL_JSON_HEADER_KEY, Json::Value::null);
                        bool is_header_json_null = header_json.isNull();

                        if(!is_header_json_null) {
                            std_msgs::msg::Header::UniquePtr rcl_header_ptr = rcl_std_msgs_json_converter_ptr_->json_to_header(header_json);
                            rcl_pose_stamped_ptr->set__header(*rcl_header_ptr);
                        } else {
                            RCUTILS_LOG_ERROR_NAMED(
                                RCL_NODE_NAME, 
                                "pose stamped header json is null"
                            );
                            RCLCPP_LINE_ERROR();
                        }
                        
                        Json::Value pose_json = pose_stamped_json.get(RCL_JSON_POSE, Json::Value::null);
                        bool is_pose_json_null = pose_json.isNull();

                        if(!is_pose_json_null) {
                            geometry_msgs::msg::Pose::UniquePtr rcl_pose_ptr = this->json_to_pose(pose_json);
                            rcl_pose_stamped_ptr->set__pose(*rcl_pose_ptr);
                        } else {
                            RCUTILS_LOG_ERROR_NAMED(
                                RCL_NODE_NAME,
                                "pose stamped pose json is null"
                            );
                            RCLCPP_LINE_ERROR();
                        }
                    } else {
                        RCUTILS_LOG_ERROR_NAMED(
                            RCL_NODE_NAME,
                            "parsing pose stamped json failed..."
                        );
                        RCLCPP_LINE_ERROR();
                    }
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert json to [%s%s]\n\twith : [%s]",
                        RCL_GEOMETRY_MSGS_TYPE,
                        RCL_JSON_POSE_STAMPED,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                return rcl_pose_stamped_ptr;
            };

            /**
             * @brief inline function for extract pose covariance array from Json::Value
             * @param raw_pose_covariance_json target const Json::Value &
             * @return rcl_pose_covariance_array
            */
            inline std::array<double, 36UL> extract_pose_covariance_array_from_json(const Json::Value & raw_pose_covariance_json) {
                Json::Value pose_covariance_json = raw_pose_covariance_json[RCL_JSON_POSE_COVARIANCE];
                std::array<double, 36UL> rcl_pose_covariance_array;

                try {
                    bool is_pose_covariance_json_array = pose_covariance_json.isArray();

                    if(is_pose_covariance_json_array) {
                        bool is_pose_covariance_default_size = (pose_covariance_json.size() == RCL_JSON_POSE_COVARIANCE_SIZE_DEFAULT);
                        if(is_pose_covariance_default_size) {
                            for(int i=0;i<RCL_JSON_POSE_COVARIANCE_SIZE_DEFAULT;i++) {
                                rcl_pose_covariance_array[i] = pose_covariance_json[i].asDouble();
                            }
                        } else {
                            RCUTILS_LOG_ERROR_NAMED(
                                RCL_NODE_NAME,
                                "pose covariance array's size is not default size : [%d]",
                                RCL_JSON_POSE_COVARIANCE_SIZE_DEFAULT
                            );
                            RCLCPP_LINE_ERROR();
                        }
                    } else {
                        RCUTILS_LOG_ERROR_NAMED(
                            RCL_NODE_NAME,
                            "pose covariance json is not array type..."
                        );
                        RCLCPP_LINE_ERROR();
                    }
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert json to [%s%s]\n\twith : [%s]",
                        RCL_GEOMETRY_MSGS_TYPE,
                        RCL_JSON_POSE_COVARIANCE,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                return rcl_pose_covariance_array;
            };

            /**
             * @brief inline function for convert Json style string into geometry_msgs::msg::PoseWithCovariance::UniquePtr
             * @param raw_pose_with_covariance_string target const std::string &
             * @return rcl_pose_with_covariance_ptr geometry_msgs::msg::PoseWithCovariance::UniquePtr
            */
            inline geometry_msgs::msg::PoseWithCovariance::UniquePtr json_to_pose_with_covariance(const std::string & raw_pose_with_covariance_string) {
                Json::Value pose_with_covariance_json;
                Json::Reader json_reader;
                geometry_msgs::msg::PoseWithCovariance::UniquePtr rcl_pose_with_covariance_ptr = std::make_unique<geometry_msgs::msg::PoseWithCovariance>();

                try {
                    bool is_pose_with_covariance_parsing_succeeded = json_reader.parse(raw_pose_with_covariance_string, pose_with_covariance_json);

                    if(is_pose_with_covariance_parsing_succeeded) {
                        Json::Value pose_json = pose_with_covariance_json.get(RCL_JSON_POSE, Json::Value::null);
                        bool is_pose_json_null = pose_json.isNull();

                        std::array<double, 36UL> rcl_pose_covariance_array = this->extract_pose_covariance_array_from_json(pose_json);
                        rcl_pose_with_covariance_ptr->set__covariance(rcl_pose_covariance_array);

                        if(!is_pose_json_null) {
                            Json::Value pose_pose_json = pose_json.get(RCL_JSON_POSE, Json::Value::null);
                            bool is_pose_pose_json_null = pose_pose_json.isNull();

                            if(!is_pose_pose_json_null) {
                                geometry_msgs::msg::Pose::UniquePtr rcl_pose_ptr = this->json_to_pose(pose_pose_json);
                                rcl_pose_with_covariance_ptr->set__pose(*rcl_pose_ptr);
                            } else {
                                RCUTILS_LOG_ERROR_NAMED(
                                    RCL_NODE_NAME,
                                    "pose with covariance pose pose json is null"
                                );
                                RCLCPP_LINE_ERROR();
                            }
                        } else {
                            RCUTILS_LOG_ERROR_NAMED(
                                RCL_NODE_NAME,
                                "pose with covariance pose json is null"
                            );
                            RCLCPP_LINE_ERROR();
                        }
                    } else {
                        RCUTILS_LOG_ERROR_NAMED(
                            RCL_NODE_NAME,
                            "parsing pose with covariance json failed..."
                        );
                        RCLCPP_LINE_ERROR();
                    }
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert json to [%s%s]\n\twith : [%s]",
                        RCL_GEOMETRY_MSGS_TYPE,
                        RCL_JSON_POSE_WITH_COVARIANCE,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                return rcl_pose_with_covariance_ptr;
            };

            /**
             * @brief inline function for convert Json style string into geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr
             * @param raw_pose_with_covariance_stamped_string target const std::string &
             * @return rcl_pose_with_covariance_stamped_ptr geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr
            */
            inline geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr json_to_pose_with_covariance_stamped(const std::string & raw_pose_with_covariance_stamped_string) {
                Json::Value pose_with_covariance_stamped_json;
                Json::Reader json_reader;
                geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr rcl_pose_with_covariance_stamped_ptr = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();

                try {
                    bool is_pose_with_covariance_parsing_succeeded = json_reader.parse(raw_pose_with_covariance_stamped_string, pose_with_covariance_stamped_json);

                    if(is_pose_with_covariance_parsing_succeeded) {
                        Json::Value header_json = pose_with_covariance_stamped_json.get(RCL_JSON_HEADER_KEY, Json::Value::null);
                        bool is_header_json_null = header_json.isNull();

                        if(!is_header_json_null) {
                            std_msgs::msg::Header::UniquePtr rcl_header_ptr = rcl_std_msgs_json_converter_ptr_->json_to_header(header_json);
                            rcl_pose_with_covariance_stamped_ptr->set__header(*rcl_header_ptr);
                        } else {
                            RCUTILS_LOG_ERROR_NAMED(
                                RCL_NODE_NAME,
                                "pose with covariance stamped header json is null"
                            );
                            RCLCPP_LINE_ERROR();
                        }

                        Json::Value pose_json = pose_with_covariance_stamped_json.get(RCL_JSON_POSE, Json::Value::null);
                        bool is_pose_json_null = pose_json.isNull();

                        if(!is_pose_json_null) {
                            Json::Value pose_pose_json = pose_json.get(RCL_JSON_POSE, Json::Value::null);
                            bool is_pose_pose_json_null = pose_pose_json.isNull();

                            if(!is_pose_pose_json_null) {
                                geometry_msgs::msg::Pose::UniquePtr rcl_pose_ptr = this->json_to_pose(pose_pose_json);
                                rcl_pose_with_covariance_stamped_ptr->pose.set__pose(*rcl_pose_ptr);
                            } else {
                                RCUTILS_LOG_ERROR_NAMED(
                                    RCL_NODE_NAME,
                                    "pose with covariance stamped pose pose json is null"
                                );
                                RCLCPP_LINE_ERROR();
                            }
                        } else {
                            RCUTILS_LOG_ERROR_NAMED(
                                RCL_NODE_NAME,
                                "pose with covariance stamped pose json is null"
                            );
                            RCLCPP_LINE_ERROR();
                        }

                        std::array<double, 36UL> rcl_pose_covariance_array = this->extract_pose_covariance_array_from_json(pose_json);
                        rcl_pose_with_covariance_stamped_ptr->pose.set__covariance(rcl_pose_covariance_array);
                    } else {
                        RCUTILS_LOG_ERROR_NAMED(
                            RCL_NODE_NAME,
                            "parsing pose with covariance stamped json failed..."
                        );
                        RCLCPP_LINE_ERROR();
                    }
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert json to [%s%s]\n\twith : [%s]",
                        RCL_GEOMETRY_MSGS_TYPE,
                        RCL_JSON_POSE_WITH_COVARIANCE_STAMPED,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                return rcl_pose_with_covariance_stamped_ptr;
            };
    };

    class SensorMessageConverter final {
        private :
            /**
             * @brief shared pointer for ros2_mqtt_bridge::StdMessageConverter
             * @see ros2_mqtt_bridge::StdMessageConverter
            */
            std::shared_ptr<ros2_mqtt_bridge::StdMessageConverter> rcl_std_msgs_json_converter_ptr_;
        public :
            /**
             * Create a new this class' instance
             * @brief Default Constructor
            */
            inline explicit SensorMessageConverter() {
                rcl_std_msgs_json_converter_ptr_ = std::make_shared<ros2_mqtt_bridge::StdMessageConverter>();
            };

            /**
             * Destroy this class' instance
             * @brief Default Destrcutor
            */
            inline virtual ~SensorMessageConverter() {
                
            };

            inline const std::string & laser_scan_to_json_string(const sensor_msgs::msg::LaserScan::SharedPtr rcl_laser_scan_ptr) {
                Json::Value laser_scan_json;

                try {
                    laser_scan_json[RCL_JSON_HEADER_KEY] = rcl_std_msgs_json_converter_ptr_->header_to_json(rcl_laser_scan_ptr->header);
                    laser_scan_json[RCL_JSON_LASER_SCAN_ANGLE_MIN] = rcl_laser_scan_ptr->angle_min;
                    laser_scan_json[RCL_JSON_LASER_SCAN_ANGLE_MAX] = rcl_laser_scan_ptr->angle_max;
                    laser_scan_json[RCL_JSON_LASER_SCAN_ANGLE_INCREMENT] = rcl_laser_scan_ptr->angle_increment;
                    laser_scan_json[RCL_JSON_LASER_SCAN_TIME_INCREMENT] = rcl_laser_scan_ptr->time_increment;
                    laser_scan_json[RCL_JSON_LASER_SCAN_SCAN_TIME] = rcl_laser_scan_ptr->scan_time;

                    laser_scan_json[RCL_JSON_LASER_SCAN_RANGE_MIN] = rcl_laser_scan_ptr->range_min;
                    laser_scan_json[RCL_JSON_LASER_SCAN_RANGE_MAX] = rcl_laser_scan_ptr->range_max;
                    
                    const std::vector<float> & rcl_laser_scan_ranges = rcl_laser_scan_ptr->ranges;
                    for (const float & range : rcl_laser_scan_ranges) {
                        laser_scan_json[RCL_JSON_LASER_SCAN_RANGES].append(range);
                    }

                    const std::vector<float> & rcl_laser_scan_intensities = rcl_laser_scan_ptr->intensities;
                    for (const float & intense : rcl_laser_scan_intensities) {
                        laser_scan_json[RCL_JSON_LASER_SCAN_INTENSITIES].append(intense);
                    }
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert [%s%s] to json\n\twith : [%s]",
                        RCL_SENSOR_MSGS_TYPE,
                        RCL_JSON_LASER_SCAN,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                const std::string & laser_scan_json_str = Json::StyledWriter().write(laser_scan_json);
                return laser_scan_json_str;
            };

            inline std::vector<float> extract_laser_scan_array_from_json(const Json::Value & raw_laser_scan_array_json, const char * target_array_key) {
                Json::Value scan_array_json = raw_laser_scan_array_json[target_array_key];
                std::vector<float> rcl_scan_array;

                try {
                    bool is_scan_array_json_array = scan_array_json.isArray();

                    if(is_scan_array_json_array) {
                        const int & scan_array_json_size = scan_array_json.size();

                        for(int i=0;i<scan_array_json_size;i++) {
                            const float & scan_array_element = scan_array_json[i].asFloat();
                            rcl_scan_array.push_back(scan_array_element);
                        }
                    } else {
                        RCUTILS_LOG_ERROR_NAMED(
                            RCL_NODE_NAME,
                            "scan array json is not array type..."
                        );
                        RCLCPP_LINE_ERROR();
                    }
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert json to [%s%s]\n\twith : [%s]",
                        RCL_SENSOR_MSGS_TYPE,
                        target_array_key,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                return rcl_scan_array;
            };

            /**
             * @brief inline function for convert Json::Value into sensor_msgs::msg::LaserScan::UniquePtr
             * @param raw_laser_scan_json target const Json::Value &
             * @return rcl_laser_scan_ptr sensor_msgs::msg::LaserScan::UniquePtr
            */
            inline sensor_msgs::msg::LaserScan::UniquePtr json_to_laser_scan(const Json::Value & raw_laser_scan_json) {
                sensor_msgs::msg::LaserScan::UniquePtr rcl_laser_scan_ptr = std::make_unique<sensor_msgs::msg::LaserScan>();

                try {
                    const float & raw_laser_scan_json_angle_min = raw_laser_scan_json.get(RCL_JSON_LASER_SCAN_ANGLE_MIN, RCL_JSON_FLOAT_DEFAULT).asFloat();
                    rcl_laser_scan_ptr->set__angle_min(raw_laser_scan_json_angle_min);

                    const float & raw_laser_scan_json_angle_max = raw_laser_scan_json.get(RCL_JSON_LASER_SCAN_ANGLE_MAX, RCL_JSON_FLOAT_DEFAULT).asFloat();
                    rcl_laser_scan_ptr->set__angle_max(raw_laser_scan_json_angle_max);

                    const float & raw_laser_scan_json_angle_increment = raw_laser_scan_json.get(RCL_JSON_LASER_SCAN_ANGLE_INCREMENT, RCL_JSON_FLOAT_DEFAULT).asFloat();
                    rcl_laser_scan_ptr->set__angle_increment(raw_laser_scan_json_angle_increment);

                    const float & raw_laser_scan_json_time_increment = raw_laser_scan_json.get(RCL_JSON_LASER_SCAN_TIME_INCREMENT, RCL_JSON_FLOAT_DEFAULT).asFloat();
                    rcl_laser_scan_ptr->set__time_increment(raw_laser_scan_json_time_increment);

                    const float & raw_laser_scan_json_scan_time = raw_laser_scan_json.get(RCL_JSON_LASER_SCAN_SCAN_TIME, RCL_JSON_FLOAT_DEFAULT).asFloat();
                    rcl_laser_scan_ptr->set__scan_time(raw_laser_scan_json_scan_time);

                    const float & raw_laser_scan_json_range_min = raw_laser_scan_json.get(RCL_JSON_LASER_SCAN_RANGE_MIN, RCL_JSON_FLOAT_DEFAULT).asFloat();
                    rcl_laser_scan_ptr->set__range_min(raw_laser_scan_json_range_min);

                    const float & raw_laser_scan_json_range_max = raw_laser_scan_json.get(RCL_JSON_LASER_SCAN_RANGE_MAX, RCL_JSON_FLOAT_DEFAULT).asFloat();
                    rcl_laser_scan_ptr->set__range_max(raw_laser_scan_json_range_max);

                    const std::vector<float> & raw_laser_scan_json_ranges = this->extract_laser_scan_array_from_json(raw_laser_scan_json, RCL_JSON_LASER_SCAN_RANGES);
                    rcl_laser_scan_ptr->set__ranges(raw_laser_scan_json_ranges);

                    const std::vector<float> & raw_laser_scan_json_intensities = this->extract_laser_scan_array_from_json(raw_laser_scan_json, RCL_JSON_LASER_SCAN_INTENSITIES);
                    rcl_laser_scan_ptr->set__intensities(raw_laser_scan_json_intensities);
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert json to [%s%s]\n\twith : [%s]",
                        RCL_SENSOR_MSGS_TYPE,
                        RCL_JSON_LASER_SCAN,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                return rcl_laser_scan_ptr;
            };

            /**
             * @brief inline function for convert sensor_msgs::msg::NavSatStatus::UniquePtr into Json::Value
             * @param rcl_nav_sat_status target sensor_msgs::msg::NavSatStatus
             * @return nav_sat_status_json Json::Value
            */
            inline Json::Value nav_sat_status_to_json(const sensor_msgs::msg::NavSatStatus rcl_nav_sat_status) {
                Json::Value nav_sat_status_json;

                try {
                    nav_sat_status_json[RCL_JSON_NAV_SAT_STATUS_STATUS_NO_FIX] = rcl_nav_sat_status.STATUS_NO_FIX;
                    nav_sat_status_json[RCL_JSON_NAV_SAT_STATUS_STATUS_FIX] = rcl_nav_sat_status.STATUS_FIX;
                    nav_sat_status_json[RCL_JSON_NAV_SAT_STATUS_STATUS_SBAS_FIX] = rcl_nav_sat_status.STATUS_SBAS_FIX;
                    nav_sat_status_json[RCL_JSON_NAV_SAT_STATUS_STATUS_GBAS_FIX] = rcl_nav_sat_status.STATUS_GBAS_FIX;
                    nav_sat_status_json[RCL_JSON_NAV_SAT_STATUS_STATUS] = rcl_nav_sat_status.status;

                    nav_sat_status_json[RCL_JSON_NAV_SAT_STATUS_SERVICE_GPS] = rcl_nav_sat_status.SERVICE_GPS;
                    nav_sat_status_json[RCL_JSON_NAV_SAT_STATUS_SERVICE_GLONASS] = rcl_nav_sat_status.SERVICE_GLONASS;
                    nav_sat_status_json[RCL_JSON_NAV_SAT_STATUS_SERVICE_COMPASS] = rcl_nav_sat_status.SERVICE_COMPASS;
                    nav_sat_status_json[RCL_JSON_NAV_SAT_STATUS_SERVICE_GALILEO] = rcl_nav_sat_status.SERVICE_GALILEO;
                    nav_sat_status_json[RCL_JSON_NAV_SAT_STATUS_SERVICE] = rcl_nav_sat_status.service;
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert json to [%s%s]\n\twith : [%s]",
                        RCL_SENSOR_MSGS_TYPE,
                        RCL_JSON_NAV_SAT_STATUS,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                return nav_sat_status_json;
            };

            /**
             * @brief inline function for convert std::array<double, 9UL> into Json::Value
             * @param target_double_9UL_array const std::array<double, 9UL>
             * @return parsed_array_json Json::Value
            */
            inline Json::Value nav_sat_fix_double_9UL_array_to_array_json(const std::array<double, 9UL> target_double_9UL_array) {
                Json::Value parsed_array_json;
                
                try {
                    const int & target_array_size = target_double_9UL_array.size();

                    for(int i=0;i<target_array_size;i++) {
                        parsed_array_json[i] = target_double_9UL_array[i];
                    }
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert [%s%s] to array json\n\twith : [%s]",
                        RCL_SENSOR_MSGS_TYPE,
                        RCL_JSON_NAV_SAT_FIX,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                return parsed_array_json;
            };

            /**
             * @brief inline function for convert sensor_msgs::msg::NavSatFix::SharedPtr into json styled string
             * @param rcl_nav_sat_fix_ptr target sensro_msgs::msg::NavSatFix::SharedPtr
             * @return nav_sat_fix_json Json::Value
            */
            inline const std::string & nav_sat_fix_to_json_string(const sensor_msgs::msg::NavSatFix::SharedPtr rcl_nav_sat_fix_ptr) {
                Json::Value nav_sat_fix_json;

                try {
                    nav_sat_fix_json[RCL_JSON_HEADER_KEY] = rcl_std_msgs_json_converter_ptr_->header_to_json(rcl_nav_sat_fix_ptr->header);
                    nav_sat_fix_json[RCL_JSON_NAV_SAT_FIX_STATUS] = this->nav_sat_status_to_json(rcl_nav_sat_fix_ptr->status);
                    nav_sat_fix_json[RCL_JSON_NAV_SAT_FIX_LATITUDE] = rcl_nav_sat_fix_ptr->latitude;
                    nav_sat_fix_json[RCL_JSON_NAV_SAT_FIX_ALTITUDE] = rcl_nav_sat_fix_ptr->altitude;
                    nav_sat_fix_json[RCL_JSON_NAV_SAT_FIX_POSITION_COVARIANCE] = this->nav_sat_fix_double_9UL_array_to_array_json(rcl_nav_sat_fix_ptr->position_covariance);
                    nav_sat_fix_json[RCL_JSON_NAV_SAT_FIX_COVARIANCE_TYPE_UNKNOWN] = rcl_nav_sat_fix_ptr->COVARIANCE_TYPE_UNKNOWN;
                    nav_sat_fix_json[RCL_JSON_NAV_SAT_FIX_COVARIANCE_TYPE_APPROXIMATED] = rcl_nav_sat_fix_ptr->COVARIANCE_TYPE_APPROXIMATED;
                    nav_sat_fix_json[RCL_JSON_NAV_SAT_FIX_COVARIANCE_TYPE_DIAGONAL_KNOWN] = rcl_nav_sat_fix_ptr->COVARIANCE_TYPE_DIAGONAL_KNOWN;
                    nav_sat_fix_json[RCL_JSON_NAV_SAT_FIX_COVARIANCE_TYPE_KNOWN] = rcl_nav_sat_fix_ptr->COVARIANCE_TYPE_KNOWN;
                    nav_sat_fix_json[RCL_JSON_NAV_SAT_FIX_POSITION_COVARIANCE_TYPE] = rcl_nav_sat_fix_ptr->position_covariance_type;
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert json to [%s%s]\n\twith : [%s]",
                        RCL_SENSOR_MSGS_TYPE,
                        RCL_JSON_NAV_SAT_FIX,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                const std::string & nav_sat_fix_json_string = Json::StyledWriter().write(nav_sat_fix_json);

                return nav_sat_fix_json_string;
            };

            /**
             * @brief inline function for convert std::vector<float> into Json::Value
             * @param target_float_vector std::vector<float>
             * @return parsed_array_json Json::Value
            */
            inline Json::Value battery_state_float_vector_to_array_json(const std::vector<float> & target_float_vector) {
                Json::Value parsed_array_json;

                try {
                    const int & target_float_vector_size = target_float_vector.size();

                    for(int i=0;i<target_float_vector_size;i++) {
                        parsed_array_json[i] = target_float_vector[i];
                    }
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert [%s%s] to array json\n\twith : [%s]",
                        RCL_SENSOR_MSGS_TYPE,
                        RCL_JSON_BATTERY_STATE,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }

                return parsed_array_json;
            };

            inline const std::string & battery_state_to_json_string(const sensor_msgs::msg::BatteryState::SharedPtr rcl_battery_state_ptr) {
                Json::Value battery_state_json;

                try {
                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_STATUS_UNKNOWN] = rcl_battery_state_ptr->POWER_SUPPLY_STATUS_UNKNOWN;
                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_STATUS_CHARGING] = rcl_battery_state_ptr->POWER_SUPPLY_STATUS_CHARGING;
                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_STATUS_DISCHARGING] = rcl_battery_state_ptr->POWER_SUPPLY_STATUS_DISCHARGING;
                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_STATUS_NOT_CHARGING] = rcl_battery_state_ptr->POWER_SUPPLY_STATUS_NOT_CHARGING;
                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_STATUS_FULL] = rcl_battery_state_ptr->POWER_SUPPLY_STATUS_FULL;

                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_HEALTH_UNKNOWN] = rcl_battery_state_ptr->POWER_SUPPLY_HEALTH_UNKNOWN;
                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_HEALTH_GOOD] = rcl_battery_state_ptr->POWER_SUPPLY_HEALTH_GOOD;
                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_HEALTH_OVERHEAT] = rcl_battery_state_ptr->POWER_SUPPLY_HEALTH_OVERHEAT;
                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_HEALTH_DEAD] = rcl_battery_state_ptr->POWER_SUPPLY_HEALTH_DEAD;
                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_HEALTH_OVERVOLTAGE] = rcl_battery_state_ptr->POWER_SUPPLY_HEALTH_OVERVOLTAGE;
                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_HEALTH_UNSPEC_FAILURE] = rcl_battery_state_ptr->POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_HEALTH_COLD] = rcl_battery_state_ptr->POWER_SUPPLY_HEALTH_COLD;
                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE] = rcl_battery_state_ptr->POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE;
                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE] = rcl_battery_state_ptr->POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;

                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_UNKNOWN] = rcl_battery_state_ptr->POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_NIMH] = rcl_battery_state_ptr->POWER_SUPPLY_TECHNOLOGY_NIMH;
                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_LION] = rcl_battery_state_ptr->POWER_SUPPLY_TECHNOLOGY_LION;
                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_LIPO] = rcl_battery_state_ptr->POWER_SUPPLY_TECHNOLOGY_LIPO;
                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_LIFE] = rcl_battery_state_ptr->POWER_SUPPLY_TECHNOLOGY_LIFE;
                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_NICD] = rcl_battery_state_ptr->POWER_SUPPLY_TECHNOLOGY_NICD;
                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_LIMN] = rcl_battery_state_ptr->POWER_SUPPLY_TECHNOLOGY_LIMN;

                    battery_state_json[RCL_JSON_HEADER_KEY] = rcl_std_msgs_json_converter_ptr_->header_to_json(rcl_battery_state_ptr->header);
                    
                    battery_state_json[RCL_JSON_BATTERY_STATE_VOLTAGE] = rcl_battery_state_ptr->voltage;
                    battery_state_json[RCL_JSON_BATTERY_STATE_TEMPERATURE] = rcl_battery_state_ptr->temperature;
                    battery_state_json[RCL_JSON_BATTERY_STATE_CURRENT] = rcl_battery_state_ptr->current;
                    battery_state_json[RCL_JSON_BATTERY_STATE_CHARGE] = rcl_battery_state_ptr->charge;
                    battery_state_json[RCL_JSON_BATTERY_STATE_CAPACITY] = rcl_battery_state_ptr->capacity;
                    battery_state_json[RCL_JSON_BATTERY_STATE_DESIGN_CAPACITY] = rcl_battery_state_ptr->design_capacity;
                    battery_state_json[RCL_JSON_BATTERY_STATE_PERCENTAGE] = rcl_battery_state_ptr->percentage;
                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_STATUS] = rcl_battery_state_ptr->power_supply_status;
                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_HEALTH] = rcl_battery_state_ptr->power_supply_health;
                    battery_state_json[RCL_JSON_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY] = rcl_battery_state_ptr->power_supply_technology;
                    battery_state_json[RCL_JSON_BATTERY_STATE_PRESENT] = rcl_battery_state_ptr->present;

                    battery_state_json[RCL_JSON_BATTERY_STATE_CELL_VOLTAGE] = this->battery_state_float_vector_to_array_json(rcl_battery_state_ptr->cell_voltage);
                    battery_state_json[RCL_JSON_BATTERY_STATE_CELL_TEMPERATURE] = this->battery_state_float_vector_to_array_json(rcl_battery_state_ptr->cell_temperature);

                    battery_state_json[RCL_JSON_BATTERY_STATE_LOCATION] = rcl_battery_state_ptr->location;
                    battery_state_json[RCL_JSON_BATTERY_STATE_SERIAL_NUMBER] = rcl_battery_state_ptr->serial_number;
                } catch(const Json::Exception & json_expn) {
                    RCUTILS_LOG_ERROR_NAMED(
                        RCL_NODE_NAME,
                        "error occurred during convert json to [%s%s]\n\twith : [%s]",
                        RCL_SENSOR_MSGS_TYPE,
                        RCL_JSON_BATTERY_STATE,
                        json_expn.what()
                    );
                    RCLCPP_LINE_ERROR();
                }
            };
    };
}

#endif