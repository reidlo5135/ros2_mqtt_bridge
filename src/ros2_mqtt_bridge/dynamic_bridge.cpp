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
 * @file dynamic_bridge.cpp
 * @author reidlo(naru5135@wavem.net)
 * @date 2023-06-27
 * @brief implementation file for rclcpp::Node
*/

/**
 * @brief include/ros2_mqtt_bridge/dynamic_bridge.hpp include area
*/

#include "ros2_mqtt_bridge/dynamic_bridge.hpp"

/**
 * Create a new this class' instance and extends mqtt::async_client
 * @brief Default Constructor
 * @see mqtt::async_client
*/
ros2_mqtt_bridge::RCLMQTTBridgeManager::RCLMQTTBridgeManager(
    rclcpp::Node::SharedPtr rcl_node_ptr, 
    std::shared_ptr<ros2_mqtt_bridge::RCLConnectionManager> rcl_connection_manager_ptr
) : rcl_node_ptr_(rcl_node_ptr),
rcl_connection_manager_ptr_(rcl_connection_manager_ptr),
mqtt_async_client_(MQTT_ADDRESS, MQTT_CLIENT_ID) {

    rcl_std_msgs_converter_ptr_ = std::make_shared<ros2_mqtt_bridge::StdMessageConverter>();
    rcl_geometry_msgs_converter_ptr_ = std::make_shared<ros2_mqtt_bridge::GeometryMessageConverter>();
    rcl_sensor_msgs_converter_ptr_ = std::make_shared<ros2_mqtt_bridge::SensorMessageConverter>();
    rcl_nav_msgs_converter_ptr_ = std::make_shared<ros2_mqtt_bridge::NavMessageConverter>();
    rcl_tf_msgs_converter_ptr_ = std::make_shared<ros2_mqtt_bridge::TFMessageConverter>();
    rcl_navigate_to_pose_action_converter_ptr_ = std::make_shared<ros2_mqtt_bridge::NavigateToPoseActionConverter>();
    rcl_can_msgs_converter_ptr_ = std::make_shared<ros2_mqtt_bridge::CanMessageConverter>();

    /**
     * invoke for establish MQTT connection
    */
    this->mqtt_connect();

    rcl_timer_base_ptr_ = rcl_node_ptr_->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&ros2_mqtt_bridge::RCLMQTTBridgeManager::bridge_rcl_to_mqtt, this)
    );
}

/**
* Destroy this class' instance
* @brief Default Destructor
*/
ros2_mqtt_bridge::RCLMQTTBridgeManager::~RCLMQTTBridgeManager() {

}

/**
* @brief function for MQTT connection
* @return void
*/
void ros2_mqtt_bridge::RCLMQTTBridgeManager::mqtt_connect() {
    try {
        /**
         * build MQTT connection options
         * - clean session : true
        */
        mqtt::connect_options mqtt_connect_opts;
        mqtt_connect_opts.set_clean_session(true);

        /**
         * establish MQTT connection with mqtt_connect_opts and wait for 60 seconds
         * - when succeeded to connect, set MQTT callback by this class
         * - when failed to connect, retry to establish MQTT connection and wait for 30 seconds
        */
        mqtt_async_client_.connect(mqtt_connect_opts)->wait_for(std::chrono::seconds(60));
        if(mqtt_async_client_.is_connected()) {
            RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "MQTT connection success");
            RCLCPP_LINE_INFO();
            mqtt_async_client_.set_callback(*this);
        } else {
            RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "MQTT connection failed... trying to reconnect");
            RCLCPP_LINE_INFO();
            mqtt_async_client_.connect(mqtt_connect_opts)->wait_for(std::chrono::seconds(30));
        }
    } catch (const mqtt::exception & mqtt_expn) {
        RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "MQTT connection error [%s]", mqtt_expn.what());
    }
}

/**
* @brief overrided function for handle when MQTT connection lost
* @param mqtt_connection_lost_cause lost cause of MQTT connection
* @return void
*/
void ros2_mqtt_bridge::RCLMQTTBridgeManager::connection_lost(const std::string & mqtt_connection_lost_cause) {
    /**
     * log cause when MQTT connection lost
    */
    RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "MQTT connection lost with cause [%s]", mqtt_connection_lost_cause.c_str());
    RCLCPP_LINE_ERROR();
}

/**
* @brief overrided function for handle when MQTT message arrived
* @param mqtt_message callback for MQTT message
* @return void
*/
void ros2_mqtt_bridge::RCLMQTTBridgeManager::message_arrived(mqtt::const_message_ptr mqtt_message) {
    /**
     * initialize MQTT topic and payload when message arrived
    */
    const std::string & mqtt_topic = mqtt_message->get_topic();
    const std::string & mqtt_payload = mqtt_message->to_string();

    this->bridge_mqtt_to_rcl(mqtt_topic, mqtt_payload);
}

/**
* @brief overrided function for handle when MQTT delivery completed
* @param mqtt_delivered_token callback for MQTT delivery token
* @return void
*/
void ros2_mqtt_bridge::RCLMQTTBridgeManager::delivery_complete(mqtt::delivery_token_ptr mqtt_delivered_token) {
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "MQTT delivery completed with [%s]", mqtt_delivered_token);
    RCLCPP_LINE_INFO();
}

/**
* @brief function for MQTT publish
* @param mqtt_topic target MQTT topic
* @param mqtt_payload target MQTT payload
* @return void
*/
void ros2_mqtt_bridge::RCLMQTTBridgeManager::mqtt_publish(const char * mqtt_topic, std::string mqtt_payload) {
    try {
        /**
         * build MQTT publish message
         * - MQTT_DEFAULT_QOS : 0
        */
		mqtt::message_ptr mqtt_publish_message_ptr = mqtt::make_message(mqtt_topic, mqtt_payload);
		mqtt_publish_message_ptr->set_qos(MQTT_DEFAULT_QOS);

        /**
         * return value of MQTT publishing result
         * - blocks the current thread until the action this token is associated with has completed.
        */
		mqtt::delivery_token_ptr mqtt_delivery_token_ptr = mqtt_async_client_.publish(mqtt_publish_message_ptr);
        mqtt_delivery_token_ptr->wait();

        /**
         * log error when MQTT publishing has not succeeded
        */
        const int & mqtt_delivered_token_return_code = mqtt_delivery_token_ptr->get_return_code();
        const bool & is_mqtt_delivered_failed = mqtt_delivered_token_return_code != mqtt::SUCCESS;

        if (is_mqtt_delivered_failed) {
            RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "MQTT publishing error with code [%d]", mqtt_delivery_token_ptr->get_return_code());
            RCLCPP_LINE_ERROR();
        }
	} catch (const mqtt::exception& mqtt_expn) {
		RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "MQTT publishing error with code [%s]", mqtt_expn.what());
        RCLCPP_LINE_ERROR();
    }
}

/**
* @brief function for MQTT subscribe
* @param mqtt_topic target MQTT topic
* @return void
*/
void ros2_mqtt_bridge::RCLMQTTBridgeManager::mqtt_subscribe(const char * mqtt_topic) {
    try {
        printf("\n");
        /**
         * grant MQTT subscription
         * - mqtt_topic : target MQTT publisher topic
         * - MQTT_DEFAULT_QOS : 0
        */
        RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "MQTT grant subscription with [%s]", mqtt_topic);
        RCLCPP_LINE_INFO();
		mqtt_async_client_.subscribe(mqtt_topic, MQTT_DEFAULT_QOS);
	} catch (const mqtt::exception & mqtt_expn) {
        RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "MQTT grant subscription error [%s]", mqtt_expn.what());
        RCLCPP_LINE_ERROR();
	}
}

/**
* @brief template function for log map
* @param target_map target std::map
* @param target_flag log target
* @return void
*/
template<typename begin_type, typename end_type>
void ros2_mqtt_bridge::RCLMQTTBridgeManager::flag_map(std::map<begin_type, end_type> target_map, const char * target_flag) {
    typename std::map<begin_type, end_type>::iterator target_map_iterator = target_map.begin();

    for (target_map_iterator;target_map_iterator != target_map.end();++target_map_iterator) {
        RCLCPP_INFO(
            rcl_node_ptr_->get_logger(),
            "RCL current [%ss] map\n\ttopic : [%s]\n\ttype : [%s]",
            target_flag,
            target_map_iterator->first.c_str(),
            target_map_iterator->second.c_str()
        );
        RCLCPP_LINE_INFO();
    }
}

/**
* @brief function for compare MQTT topic and RCL topic before bridge MQTT to RCL
* @param mqtt_topic target MQTT topic
* @param rcl_topic target RCL topic
* @return is_mqtt_topic_equals_rcl_topic bool
*/
bool ros2_mqtt_bridge::RCLMQTTBridgeManager::bridge_mqtt_to_rcl_topic_cmp(const std::string & mqtt_topic, const char * rcl_topic) {
    const char * cmqtt_topic = mqtt_topic.c_str();

    const bool & is_mqtt_topic_equals_rcl_topic = (strcmp(cmqtt_topic, rcl_topic) == 0);

    return is_mqtt_topic_equals_rcl_topic;
}

/**
* @brief function for compare MQTT topic and RCL topic before bridge MQTT to RCL
* @param rcl_subscription_topic target MQTT topic
* @param rcl_target_topic target RCL topic
* @return is_rcl_subscription_topic_equals_rcl_target_topic bool
*/
bool ros2_mqtt_bridge::RCLMQTTBridgeManager::bridge_mqtt_to_rcl_topic_cmp(const char * rcl_subscription_topic, const char * rcl_target_topic) {
    const bool & is_rcl_subscription_topic_equals_rcl_target_topic = (strcmp(rcl_subscription_topic, rcl_target_topic) == 0);

    return is_rcl_subscription_topic_equals_rcl_target_topic;
}

/**
* @brief function for compare RCL subscription's message type and RCL target message type before bridge RCL to MQTT
* @param rcl_subscription_msgs_type target RCL subscription's message type const std::string &
* @param rcl_target_msgs_type target RCL message type const std::string &
* @return is_rcl_subscription_message_type_equals_rcl_target_message_type bool
*/
bool ros2_mqtt_bridge::RCLMQTTBridgeManager::bridge_mqtt_to_rcl_msgs_type_cmp(const std::string & rcl_subscription_msgs_type, const char * rcl_target_msgs_type) {
    const bool & is_rcl_subscription_message_type_equals_rcl_target_message_type = (rcl_subscription_msgs_type.find(rcl_target_msgs_type) != std::string::npos);

    return is_rcl_subscription_message_type_equals_rcl_target_message_type;
}

/**
 * @brief function for flag and log foudate MQTT to RCL
 * @param rcl_subscription_topic target RCL subscription topic const char *
 * @param rcl_subscription_msgs_type target RCL subscription message type const std::string &
 * @return void
*/
void ros2_mqtt_bridge::RCLMQTTBridgeManager::flag_foundate_mqtt_to_rcl(const char * rcl_subscription_topic, const std::string & rcl_subscription_msgs_type) {
    RCLCPP_INFO(
        rcl_node_ptr_->get_logger(),
        "registered [%s] publisher with type : [%s]",
        rcl_subscription_topic,
        rcl_subscription_msgs_type.c_str()
    );
    RCLCPP_LINE_INFO();
}

/**
* @brief function for foundate MQTT to RCL by grant MQTT subscription with current RCL publishers
* @param rcl_current_publishers_map target RCL current publishers map
* @return void
*/
void ros2_mqtt_bridge::RCLMQTTBridgeManager::foundate_mqtt_to_rcl(std::map<std::string, std::string> rcl_current_subscriptions_map) {
    std::map<std::string, std::string>::iterator rcl_current_subscriptions_map_iterator = rcl_current_subscriptions_map.begin();

    for (rcl_current_subscriptions_map_iterator;rcl_current_subscriptions_map_iterator != rcl_current_subscriptions_map.end();++rcl_current_subscriptions_map_iterator) {
        const char * rcl_subscription_topic = rcl_current_subscriptions_map_iterator->first.c_str();
        const std::string & rcl_subscription_msgs_type = rcl_current_subscriptions_map_iterator->second;
        this->mqtt_subscribe(rcl_subscription_topic);
        
        const bool & is_rcl_std_msgs_type = this->bridge_mqtt_to_rcl_msgs_type_cmp(rcl_subscription_msgs_type, RCL_STD_MSGS_TYPE);
        const bool & is_rcl_geometry_msgs_type = this->bridge_mqtt_to_rcl_msgs_type_cmp(rcl_subscription_msgs_type, RCL_GEOMETRY_MSGS_TYPE);
        const bool & is_rcl_can_msgs_type = this->bridge_mqtt_to_rcl_msgs_type_cmp(rcl_subscription_msgs_type, RCL_CAN_MSGS_TYPE);

        if(is_rcl_std_msgs_type) {
            const bool & is_rcl_chatter_subscription = this->bridge_mqtt_to_rcl_topic_cmp(rcl_subscription_topic, RCL_CHATTER_TOPIC);
            
            if(is_rcl_chatter_subscription) {
                rcl_chatter_publisher_ptr_ = rcl_connection_manager_ptr_->register_publisher<std_msgs::msg::String>(
                    rcl_node_ptr_,
                    RCL_CHATTER_TOPIC
                );

                this->flag_foundate_mqtt_to_rcl(rcl_subscription_topic, rcl_subscription_msgs_type);
            } else {
                RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "RCL [%s] subscriptions ended");
                RCLCPP_LINE_ERROR();
            }
        } else if(is_rcl_geometry_msgs_type) {
            const bool & is_rcl_cmd_vel_subscription = this->bridge_mqtt_to_rcl_topic_cmp(rcl_subscription_topic, RCL_CMD_VEL_TOPIC);

            if(is_rcl_cmd_vel_subscription) {
                rcl_cmd_vel_publisher_ptr_ = rcl_connection_manager_ptr_->register_publisher<geometry_msgs::msg::Twist>(
                    rcl_node_ptr_,
                    RCL_CMD_VEL_TOPIC
                );

                this->flag_foundate_mqtt_to_rcl(rcl_subscription_topic, rcl_subscription_msgs_type);
            } else {
                RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "RCL [%s] subscriptions ended");
                RCLCPP_LINE_ERROR();
            }
        } else if(is_rcl_can_msgs_type) {
            const bool & is_rcl_can_control_hardware_subscription = this->bridge_mqtt_to_rcl_topic_cmp(rcl_subscription_topic, RCL_CAN_CONTROL_HARDWARE_TOPIC);

            if(is_rcl_can_control_hardware_subscription) {
                rcl_can_control_hardware_publisher_ptr_ = rcl_connection_manager_ptr_->register_publisher<can_msgs::msg::ControlHardware>(
                    rcl_node_ptr_,
                    RCL_CAN_CONTROL_HARDWARE_TOPIC
                );

                this->flag_foundate_mqtt_to_rcl(rcl_subscription_topic, rcl_subscription_msgs_type);
            } else {
                RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "RCL [%s] subscriptions ended");
                RCLCPP_LINE_ERROR();
            }
        } else {
            RCLCPP_INFO(rcl_node_ptr_->get_logger(), "foundate MQTT to RCL ended..");
            RCLCPP_LINE_INFO();
        }
    }
}

/**
* @brief function for flag and log bridge MQTT to RCL
* @param mqtt_topic target MQTT topic
* @param mqtt_payload target MQTT payload
* @return void
*/
void ros2_mqtt_bridge::RCLMQTTBridgeManager::flag_bridge_mqtt_to_rcl(const std::string & mqtt_topic, const std::string & mqtt_payload) {
    RCLCPP_INFO(rcl_node_ptr_->get_logger(), "bridge MQTT to RCL with\n\ttopic : [%s]\n\tpayload : [%s]", mqtt_topic.c_str(), mqtt_payload.c_str());
    RCLCPP_LINE_INFO();
}

/**
* @brief function for bridge MQTT to RCL after delivered mqtt_subscription callback data
* @param mqtt_topic target MQTT topic
* @param mqtt_payload received MQTT payload
* @return void
*/
void ros2_mqtt_bridge::RCLMQTTBridgeManager::bridge_mqtt_to_rcl(const std::string & mqtt_topic, const std::string & mqtt_payload) {
    RCLCPP_INFO(
        rcl_node_ptr_->get_logger(),
        "bridge MQTT to RCL message arrived \n\ttopic : [%s]\n\tpayload : [%s]",
        mqtt_topic.c_str(),
        mqtt_payload.c_str()
    );
    RCLCPP_LINE_INFO();

    const bool & is_mqtt_chatter_publisher = this->bridge_mqtt_to_rcl_topic_cmp(mqtt_topic, RCL_CHATTER_TOPIC);
    const bool & is_mqtt_cmd_vel_publisher = this->bridge_mqtt_to_rcl_topic_cmp(mqtt_topic, RCL_CMD_VEL_TOPIC);
    const bool & is_mqtt_can_control_hardware_publisher = this->bridge_mqtt_to_rcl_topic_cmp(mqtt_topic, RCL_CAN_CONTROL_HARDWARE_TOPIC);

    // if(is_mqtt_chatter_publisher) {
    //     this->flag_bridge_mqtt_to_rcl(mqtt_topic, mqtt_payload);
        
    //     std_msgs::msg::String::UniquePtr rcl_std_msgs_string_ptr = rcl_std_msgs_converter_ptr_->json_to_std_msgs_string(mqtt_payload);
    //     const std_msgs::msg::String && rcl_std_msgs_string_ptr_moved = std::move(*rcl_std_msgs_string_ptr);

    //     rcl_chatter_publisher_ptr_->publish(rcl_std_msgs_string_ptr_moved);
    // } 
    if(is_mqtt_cmd_vel_publisher) {
        this->flag_bridge_mqtt_to_rcl(mqtt_topic, mqtt_payload);

        geometry_msgs::msg::Twist::UniquePtr rcl_geometry_msgs_twist_ptr = rcl_geometry_msgs_converter_ptr_->json_to_twist(mqtt_payload);
        const geometry_msgs::msg::Twist && rcl_geometry_msgs_twist_ptr_moved = std::move(*rcl_geometry_msgs_twist_ptr);

        rcl_cmd_vel_publisher_ptr_->publish(rcl_geometry_msgs_twist_ptr_moved);
    } else if(is_mqtt_can_control_hardware_publisher) {
        this->flag_bridge_mqtt_to_rcl(mqtt_topic, mqtt_payload);

        can_msgs::msg::ControlHardware::UniquePtr rcl_can_msgs_control_hardware_ptr = rcl_can_msgs_converter_ptr_->json_to_can_control_hardware(mqtt_payload);
        const can_msgs::msg::ControlHardware && rcl_can_msgs_control_hardware_ptr_moved = std::move(*rcl_can_msgs_control_hardware_ptr);

        rcl_can_control_hardware_publisher_ptr_->publish(rcl_can_msgs_control_hardware_ptr_moved);
    } else {
        RCLCPP_WARN(
            rcl_node_ptr_->get_logger(),
            "bridge MQTT to RCL ended..."
        );
        RCLCPP_LINE_WARN();
    }
}

/**
* @brief function for compare RCL publisher's topic and RCL target topic before bridge RCL to MQTT
* @param rcl_publisher_topic target RCL publisher topic const char *
* @param rcl_target_topic target RCL publisher topic const char *
* @return is_rcl_publisher_topic_equals_rcl_target_topic bool
*/
bool ros2_mqtt_bridge::RCLMQTTBridgeManager::bridge_rcl_to_mqtt_topic_cmp(const char * rcl_publisher_topic, const char * rcl_target_topic) {
    const bool & is_rcl_publisher_topic_equals_rcl_target_topic = (strcmp(rcl_publisher_topic, rcl_target_topic) == 0);

    return is_rcl_publisher_topic_equals_rcl_target_topic;
}

/**
* @brief function for compare RCL publisher's message type and RCL target message type before bridge RCL to MQTT
* @param rcl_publisher_msgs_type target RCL publisher's message type const std::string &
* @param rcl_target_msgs_type target RCL message type const char *
* @return is_rcl_publisher_message_type_equals_rcl_target_message_type bool
*/
bool ros2_mqtt_bridge::RCLMQTTBridgeManager::bridge_rcl_to_mqtt_msgs_type_cmp(const std::string & rcl_publisher_msgs_type, const char * rcl_target_msgs_type) {
    const bool & is_rcl_publisher_message_type_equals_rcl_target_message_type = (rcl_publisher_msgs_type.find(rcl_target_msgs_type) != std::string::npos);

    return is_rcl_publisher_message_type_equals_rcl_target_message_type;
}

/**
* @brief function for flag and log bridge RCL to MQTT
* @param rcl_publisher_topic target RCL publisher topic
* @param rcl_json_string target parsed RCL json styled string
* @return void
*/
void ros2_mqtt_bridge::RCLMQTTBridgeManager::flag_bridge_rcl_to_mqtt(const char * rcl_publisher_topic, const std::string & rcl_json_string) {
    const char * crcl_json_string = rcl_json_string.c_str();

    RCLCPP_INFO(
        rcl_node_ptr_->get_logger(), 
        "RCL to MQTT [%s] subscription callback : [%s]", 
        rcl_publisher_topic, 
        crcl_json_string    
    );
    RCLCPP_LINE_INFO();
}

/**
* @brief function for bridge_rcl_to_mqtt rcl and mqtt after get current topic and types
* @return void
*/
void ros2_mqtt_bridge::RCLMQTTBridgeManager::bridge_rcl_to_mqtt() {
    /**
     * maps for save rcl connections
     * - rcl_publishers_map : for rcl publishers
     * - rcl_subscriptions_map : for rcl subscriptions
     * - rcl_service_map : for rcl services
    */
    std::map<std::string, std::string> rcl_previous_publishers_map;
    std::map<std::string, std::string> rcl_previous_subscriptions_map;
    std::map<std::string, std::string> rcl_publishers_map;
    std::map<std::string, std::string> rcl_subscriptions_map;
    std::map<std::string, std::map<std::string, std::string>> rcl_services_map;

    /**
     * sets for save already ignored rcl topics
     * - rcl_already_ignored_topics_set : for topics(publishers, subscriptions)
     * - rcl_already_ignored_services_set : for services
    */
    std::set<std::string> rcl_already_ignored_topics_set;
    std::set<std::string> rcl_already_ignored_services_set;

    /**
     * lamba function for bridge rcl to mqtt rcl to mqtt get current rcl topics and types
    */
    std::function<void()> rcl_bridge_lambda = [
        this,
        &rcl_previous_publishers_map,
        &rcl_previous_subscriptions_map,
        &rcl_publishers_map,
        &rcl_subscriptions_map,
        &rcl_services_map,
        &rcl_already_ignored_topics_set,
        &rcl_already_ignored_services_set
    ]() -> void {
        const std::map<std::string, std::vector<std::string, std::allocator<std::string>>> & rcl_topics_and_types_map = rcl_node_ptr_->get_topic_names_and_types();

        std::set<std::string> rcl_ignored_topics_set;
        rcl_ignored_topics_set.insert(RCL_PARAMETER_EVENTS_TOPIC);
        rcl_ignored_topics_set.insert(RCL_ROSOUT_TOPIC);

        std::map<std::string, std::string> rcl_current_publishers_map;
        std::map<std::string, std::string> rcl_current_subscriptions_map;

        for(const std::pair<const std::string, std::vector<std::string, std::allocator<std::string>>> & rcl_topic_and_types_pair : rcl_topics_and_types_map) {
            if (rcl_ignored_topics_set.find(rcl_topic_and_types_pair.first) != rcl_ignored_topics_set.end()) {
                continue;
            }

            const std::string & rcl_topic_name = rcl_topic_and_types_pair.first;
            const std::string & rcl_topic_type = rcl_topic_and_types_pair.second[0];

            if (rcl_topic_and_types_pair.second.size() > 1) {
                if (rcl_already_ignored_topics_set.count(rcl_topic_name) == 0) {
                    std::string types = "";
                    
                    for (const std::string & type : rcl_topic_and_types_pair.second) {
                        types += type + ", ";
                    }

                    const char * rcl_ignoring_topic = rcl_topic_name.c_str();
                    const int & rcl_ignoring_type_length = (types.length() - 2);
                    const char * rcl_ignoring_type = types.substr(0, rcl_ignoring_type_length).c_str();

                    RCLCPP_WARN(
                        rcl_node_ptr_->get_logger(),
                        "ignoring topic [%s], which has more than one type: [%s]\n",
                        rcl_ignoring_topic,
                        rcl_ignoring_type
                    );
                    RCLCPP_LINE_WARN();

                    rcl_already_ignored_topics_set.insert(rcl_topic_name);
                }

                continue;
            }

            const size_t & rcl_publisher_count = rcl_node_ptr_->count_publishers(rcl_topic_name);
            const size_t & rcl_subscription_count = rcl_node_ptr_->count_subscribers(rcl_topic_name);

            if (rcl_publisher_count) {
                RCLCPP_INFO(
                    rcl_node_ptr_->get_logger(),
                    "RCL found [%s]\n\ttopic : [%s]\n\ttype : [%s]",
                    RCL_PUBLISHER_FLAG,
                    rcl_topic_name.c_str(),
                    rcl_topic_type.c_str()
                );
                RCLCPP_LINE_INFO();

                rcl_current_publishers_map[rcl_topic_name] = rcl_topic_type;
            } else {
                RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "RCL [%s]s is empty...", RCL_PUBLISHER_FLAG);
                RCLCPP_LINE_ERROR();
            }

            this->flag_map<std::string, std::string>(rcl_current_publishers_map, RCL_PUBLISHER_FLAG);

            if (rcl_subscription_count) {
                RCLCPP_INFO(
                    rcl_node_ptr_->get_logger(),
                    "RCL found [%s]\n\ttopic : [%s]\n\ttype : [%s]",
                    RCL_SUBSCRIPTION_FLAG,
                    rcl_topic_name.c_str(),
                    rcl_topic_type.c_str()
                );
                RCLCPP_LINE_INFO();

                rcl_current_subscriptions_map[rcl_topic_name] = rcl_topic_type;
            } else {
                RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "RCL [%s]s is empty...", RCL_SUBSCRIPTION_FLAG);
                RCLCPP_LINE_ERROR();
            }

            this->flag_map<std::string, std::string>(rcl_current_subscriptions_map, RCL_SUBSCRIPTION_FLAG);
            this->foundate_mqtt_to_rcl(rcl_current_subscriptions_map);

            RCLCPP_INFO(
                rcl_node_ptr_->get_logger(),
                "RCL current status \n\ttopic : [%s]\n\ttypes : [%s]\n\tcount : [%zu publishers, %zu subscriptions]",
                rcl_topic_name.c_str(), 
                rcl_topic_type.c_str(),
                rcl_publisher_count,
                rcl_subscription_count
            );
            RCLCPP_LINE_INFO();

            std::map<std::string, std::string>::iterator rcl_current_publishers_map_iterator = rcl_current_publishers_map.begin();

            for(rcl_current_publishers_map_iterator;rcl_current_publishers_map_iterator != rcl_current_publishers_map.end(); ++rcl_current_publishers_map_iterator) {
                const char * rcl_publisher_topic = rcl_current_publishers_map_iterator->first.c_str();
                const std::string & rcl_publisher_msgs_type = rcl_current_publishers_map_iterator->second;

                RCLCPP_INFO(
                    rcl_node_ptr_->get_logger(),
                    "RCL current publisher status\n\ttopic : [%s]\n\ttype : [%s]",
                    rcl_publisher_topic,
                    rcl_publisher_msgs_type.c_str()
                );
                RCLCPP_LINE_INFO();
                
                const bool & is_rcl_std_msgs_type = this->bridge_rcl_to_mqtt_msgs_type_cmp(rcl_publisher_msgs_type, RCL_STD_MSGS_TYPE);
                const bool & is_rcl_nav_msgs_type = this->bridge_rcl_to_mqtt_msgs_type_cmp(rcl_publisher_msgs_type, RCL_NAV_MSGS_TYPE);
                const bool & is_rcl_geometry_msgs_type = this->bridge_rcl_to_mqtt_msgs_type_cmp(rcl_publisher_msgs_type, RCL_GEOMETRY_MSGS_TYPE);
                const bool & is_rcl_sensor_msgs_type = this->bridge_rcl_to_mqtt_msgs_type_cmp(rcl_publisher_msgs_type, RCL_SENSOR_MSGS_TYPE);
                const bool & is_rcl_tf2_msgs_type = this->bridge_rcl_to_mqtt_msgs_type_cmp(rcl_publisher_msgs_type, RCL_TF2_MSGS_TYPE);
                const bool & is_rcl_can_msgs_type = this->bridge_rcl_to_mqtt_msgs_type_cmp(rcl_publisher_msgs_type, RCL_CAN_MSGS_TYPE);

                if(is_rcl_std_msgs_type) {
                    const bool & is_rcl_chatter_publisher = this->bridge_rcl_to_mqtt_topic_cmp(rcl_publisher_topic, RCL_CHATTER_TOPIC);

                    RCLCPP_INFO(rcl_node_ptr_->get_logger(), "RCL publisher is [%s] publisher : [%d]", RCL_CHATTER_TOPIC, is_rcl_chatter_publisher);
                    RCLCPP_LINE_INFO();

                    if(is_rcl_chatter_publisher) {
                        using rcl_message_type_t = std_msgs::msg::String;

                        RCLCPP_INFO(rcl_node_ptr_->get_logger(), "RCL [%s] publisher is type [%sstring]", rcl_publisher_topic, RCL_STD_MSGS_TYPE);
                        RCLCPP_LINE_INFO();

                        const std::function<void(std::shared_ptr<rcl_message_type_t>)> & rcl_chatter_callback = [this, rcl_publisher_topic](const std_msgs::msg::String::SharedPtr rcl_chatter_callback_data_ptr) {
                            const std::string & std_msgs_string_json_string = this->rcl_std_msgs_converter_ptr_->std_msgs_string_to_json_string(rcl_chatter_callback_data_ptr);
                            this->flag_bridge_rcl_to_mqtt(rcl_publisher_topic, std_msgs_string_json_string);
                            this->mqtt_publish(rcl_publisher_topic, std_msgs_string_json_string);
                        };

                        rcl_chatter_subscription_ptr_ = rcl_connection_manager_ptr_->register_subscription<rcl_message_type_t>(rcl_node_ptr_, rcl_publisher_topic, rcl_chatter_callback);
                    } else {
                        RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "RCL [%s] publishers ended");
                        RCLCPP_LINE_ERROR();
                    }
                } else if(is_rcl_nav_msgs_type) {
                    const bool & is_rcl_map_publisher = this->bridge_rcl_to_mqtt_topic_cmp(rcl_publisher_topic, RCL_MAP_TOPIC);
                    const bool & is_rcl_odom_publisher = this->bridge_rcl_to_mqtt_topic_cmp(rcl_publisher_topic, RCL_ODOM_TOPIC);

                    if(is_rcl_map_publisher) {
                        using rcl_message_type_t = nav_msgs::msg::OccupancyGrid;

                        const std::function<void(std::shared_ptr<rcl_message_type_t>)> & rcl_map_callback = [this, rcl_publisher_topic](const nav_msgs::msg::OccupancyGrid::SharedPtr rcl_map_callback_data_ptr) {
                            const std::string & occupancy_grid_json_string = this->rcl_nav_msgs_converter_ptr_->occupancy_grid_to_json_string(rcl_map_callback_data_ptr);
                            this->flag_bridge_rcl_to_mqtt(rcl_publisher_topic, occupancy_grid_json_string);
                            this->mqtt_publish(rcl_publisher_topic, occupancy_grid_json_string);
                        };

                        rcl_map_subscription_ptr_ = rcl_connection_manager_ptr_->register_subscription<rcl_message_type_t>(rcl_node_ptr_, rcl_publisher_topic, rcl_map_callback);
                    } else if(is_rcl_odom_publisher) {
                        using rcl_message_type_t = nav_msgs::msg::Odometry;

                        const std::function<void(std::shared_ptr<rcl_message_type_t>)> & rcl_odom_callback = [this, rcl_publisher_topic](const nav_msgs::msg::Odometry::SharedPtr rcl_odom_callback_data_ptr) {
                            const std::string & odometry_json_string = this->rcl_nav_msgs_converter_ptr_->odometry_to_json_string(rcl_odom_callback_data_ptr);
                            this->flag_bridge_rcl_to_mqtt(rcl_publisher_topic, odometry_json_string);
                            this->mqtt_publish(rcl_publisher_topic, odometry_json_string);
                        };

                        rcl_odom_subscription_ptr_ = rcl_connection_manager_ptr_->register_subscription<rcl_message_type_t>(rcl_node_ptr_, rcl_publisher_topic, rcl_odom_callback);
                    } else {
                        RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "RCL [%s] publishers ended");
                        RCLCPP_LINE_ERROR();
                    }
                } else if(is_rcl_sensor_msgs_type) {
                    const bool & is_rcl_scan_publisher = this->bridge_rcl_to_mqtt_topic_cmp(rcl_publisher_topic, RCL_SCAN_TOPIC);
                    const bool & is_rcl_ublox_fix_publisher = this->bridge_mqtt_to_rcl_topic_cmp(rcl_publisher_topic, RCL_UBLOX_FIX_TOPIC);

                    if(is_rcl_scan_publisher) {
                        using rcl_message_type_t = sensor_msgs::msg::LaserScan;

                        const std::function<void(std::shared_ptr<rcl_message_type_t>)> & rcl_scan_callback = [this, rcl_publisher_topic](const sensor_msgs::msg::LaserScan::SharedPtr rcl_scan_callback_data_ptr) {
                            const std::string & scan_json_string = this->rcl_sensor_msgs_converter_ptr_->laser_scan_to_json_string(rcl_scan_callback_data_ptr);
                            this->flag_bridge_rcl_to_mqtt(rcl_publisher_topic, scan_json_string);
                            this->mqtt_publish(rcl_publisher_topic, scan_json_string);
                        };

                        rcl_scan_subscription_ptr_ = rcl_connection_manager_ptr_->register_subscription<rcl_message_type_t>(rcl_node_ptr_, rcl_publisher_topic, rcl_scan_callback);
                    } else if(is_rcl_ublox_fix_publisher) {
                        using rcl_message_type_t = sensor_msgs::msg::NavSatFix;

                        const std::function<void(std::shared_ptr<rcl_message_type_t>)> & rcl_ublox_fix_callback = [this, rcl_publisher_topic](const sensor_msgs::msg::NavSatFix::SharedPtr rcl_ublox_fix_callback_data_ptr) {
                            const std::string & ublox_fix_json_string = this->rcl_sensor_msgs_converter_ptr_->nav_sat_fix_to_json_string(rcl_ublox_fix_callback_data_ptr);
                            this->flag_bridge_rcl_to_mqtt(rcl_publisher_topic, ublox_fix_json_string);
                            this->mqtt_publish(rcl_publisher_topic, ublox_fix_json_string);
                        };

                        rcl_ublox_fix_subscription_ptr_ = rcl_connection_manager_ptr_->register_subscription<rcl_message_type_t>(rcl_node_ptr_, rcl_publisher_topic, rcl_ublox_fix_callback);
                    } else {
                        RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "RCL [%s] publishers ended");
                        RCLCPP_LINE_ERROR();
                    }
                } else if(is_rcl_tf2_msgs_type) {
                    const bool & is_rcl_tf_publisher = this->bridge_rcl_to_mqtt_topic_cmp(rcl_publisher_topic, RCL_TF_TOPIC);
                    const bool & is_rcl_tf_static_publisher = this->bridge_rcl_to_mqtt_topic_cmp(rcl_publisher_topic, RCL_TF_STATIC_TOPIC);

                    if(is_rcl_tf_publisher) {
                        using rcl_message_type_t = tf2_msgs::msg::TFMessage;

                        const std::function<void(std::shared_ptr<rcl_message_type_t>)> & rcl_tf_callback = [this, rcl_publisher_topic](const tf2_msgs::msg::TFMessage::SharedPtr rcl_tf_callback_data_ptr) {
                            const std::string & tf_json_string = this->rcl_tf_msgs_converter_ptr_->tf_message_to_json_string(rcl_tf_callback_data_ptr);
                            this->flag_bridge_rcl_to_mqtt(rcl_publisher_topic, tf_json_string);
                            this->mqtt_publish(rcl_publisher_topic, tf_json_string);
                        };

                        rcl_tf_subscription_ptr_ = rcl_connection_manager_ptr_->register_subscription<rcl_message_type_t>(rcl_node_ptr_, rcl_publisher_topic, rcl_tf_callback);
                    } else if(is_rcl_tf_static_publisher) {
                        using rcl_message_type_t = tf2_msgs::msg::TFMessage;

                        const std::function<void(std::shared_ptr<rcl_message_type_t>)> & rcl_tf_static_callback = [this, rcl_publisher_topic](const tf2_msgs::msg::TFMessage::SharedPtr rcl_tf_static_callback_data_ptr) {
                            const std::string & tf_static_json_string = this->rcl_tf_msgs_converter_ptr_->tf_message_to_json_string(rcl_tf_static_callback_data_ptr);
                            this->flag_bridge_rcl_to_mqtt(rcl_publisher_topic, tf_static_json_string);
                            this->mqtt_publish(rcl_publisher_topic, tf_static_json_string);
                        };

                        rcl_tf_subscription_ptr_ = rcl_connection_manager_ptr_->register_subscription<rcl_message_type_t>(rcl_node_ptr_, rcl_publisher_topic, rcl_tf_static_callback);
                    } else {
                        RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "RCL [%s] publishers ended");
                        RCLCPP_LINE_ERROR();
                    }
                } else if(is_rcl_geometry_msgs_type) {
                    const bool & is_rcl_robot_pose_publisher = this->bridge_rcl_to_mqtt_topic_cmp(rcl_publisher_topic, RCL_ROBOT_POSE_TOPIC);
                    const bool & is_rcl_cmd_vel_publisher = this->bridge_rcl_to_mqtt_topic_cmp(rcl_publisher_topic, RCL_CMD_VEL_TOPIC);

                    if(is_rcl_robot_pose_publisher) {
                        using rcl_message_type_t = geometry_msgs::msg::Pose;

                        const std::function<void(std::shared_ptr<rcl_message_type_t>)> & rcl_robot_pose_callback = [this, rcl_publisher_topic](const geometry_msgs::msg::Pose::SharedPtr rcl_robot_pose_callback_data_ptr) {
                            const std::string & robot_pose_json_string = this->rcl_geometry_msgs_converter_ptr_->pose_to_json_string(rcl_robot_pose_callback_data_ptr);
                            this->flag_bridge_rcl_to_mqtt(rcl_publisher_topic, robot_pose_json_string);
                            this->mqtt_publish(rcl_publisher_topic, robot_pose_json_string);
                        };

                        rcl_robot_pose_subscription_ptr_ = rcl_connection_manager_ptr_->register_subscription<rcl_message_type_t>(rcl_node_ptr_, rcl_publisher_topic, rcl_robot_pose_callback);
                    } else if(is_rcl_cmd_vel_publisher) {
                        using rcl_message_type_t = geometry_msgs::msg::Twist;

                        const std::function<void(std::shared_ptr<rcl_message_type_t>)> & rcl_cmd_vel_callback = [this, rcl_publisher_topic](const geometry_msgs::msg::Twist::SharedPtr rcl_cmd_vel_callback_data_ptr) {
                            const std::string & cmd_vel_json_string = this->rcl_geometry_msgs_converter_ptr_->twist_to_json_string(rcl_cmd_vel_callback_data_ptr);
                            this->flag_bridge_rcl_to_mqtt(rcl_publisher_topic, cmd_vel_json_string);
                            this->mqtt_publish(rcl_publisher_topic, cmd_vel_json_string);
                        };

                        rcl_cmd_vel_subscription_ptr_ = rcl_connection_manager_ptr_->register_subscription<rcl_message_type_t>(rcl_node_ptr_, rcl_publisher_topic, rcl_cmd_vel_callback);
                    } else {
                        RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "RCL [%s] publishers ended");
                        RCLCPP_LINE_ERROR();
                    }
                } else {
                    RCLCPP_WARN(rcl_node_ptr_->get_logger(), "bridge RCL to MQTT ended..");
                    RCLCPP_LINE_WARN();
                }
            }

            {
                std::lock_guard<std::mutex> rcl_bridge_lock_guard(rcl_lambda_mutex_);

                for (const std::pair<const std::string, const std::string> & rcl_publisher : rcl_publishers_map) {
                    const std::string & rcl_publisher_topic = rcl_publisher.first;
                    const std::string & rcl_publisher_type = rcl_publisher.second;
                    
                    const bool & is_rcl_publisher_already_established = (rcl_previous_publishers_map.find(rcl_publisher_topic) != rcl_previous_publishers_map.end());
                    if (is_rcl_publisher_already_established) {
                        RCLCPP_WARN(
                            rcl_node_ptr_->get_logger(),
                            "RCL to MQTT / this connection has already established\n\ttopic : [%s]\n\ttype : [%s]",
                            rcl_publisher_topic,
                            rcl_publisher_type
                        );
                        RCLCPP_LINE_WARN();
                        continue;
                    }

                    rcl_previous_publishers_map[rcl_publisher_topic] = rcl_publisher_type;
                }

                for (const std::pair<const std::string, const std::string> & rcl_subscription : rcl_subscriptions_map) {
                    const std::string & rcl_subscription_topic = rcl_subscription.first;
                    const std::string & rcl_subscription_type = rcl_subscription.second;

                    const bool & is_rcl_subscription_already_established = (rcl_previous_subscriptions_map.find(rcl_subscription_topic) != rcl_previous_subscriptions_map.end());
                    if (is_rcl_subscription_already_established) {
                        RCLCPP_WARN(
                            rcl_node_ptr_->get_logger(),
                            "MQTT to RCL / this connection has already established\n\ttopic : [%s]\n\ttype : [%s]",
                            rcl_subscription_topic,
                            rcl_subscription_type
                        );
                        RCLCPP_LINE_WARN();
                        continue;
                    }

                    rcl_previous_subscriptions_map[rcl_subscription_topic] = rcl_subscription_type;
                }

                rcl_publishers_map = rcl_current_publishers_map;
                rcl_subscriptions_map = rcl_current_subscriptions_map;
            }
        }
    };

    rcl_previous_publishers_map.clear();
    rcl_previous_subscriptions_map.clear();
    rcl_bridge_lambda();
}

/**
* Create a new this class' instance and extends rclcpp::Node
* @brief Default Constructor
* @see rclcpp::Node
*/
ros2_mqtt_bridge::RCLNode::RCLNode()
: Node(RCL_NODE_NAME) {
    rcl_node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    rcl_connection_manager_ptr_ = std::make_shared<ros2_mqtt_bridge::RCLConnectionManager>();
    rcl_mqtt_bridge_manager_ptr_ = std::make_shared<ros2_mqtt_bridge::RCLMQTTBridgeManager>(rcl_node_ptr_, rcl_connection_manager_ptr_);
}

/**
* Destroy this class' instance
* @brief Default Destructor
*/
ros2_mqtt_bridge::RCLNode::~RCLNode() {

}

/**
* Function for handle signal_input when program exit
* @param signal_input The signal_input of input
* @return void
* @see signal_input.h
*/
void ros2_mqtt_bridge::RCLNode::sig_handler(int signal_input) {
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "stopped with SIG [%i]", signal_input);
    RCLCPP_LINE_INFO();
	signal(signal_input, SIG_IGN);
	exit(RCL_EXIT_FLAG);
}