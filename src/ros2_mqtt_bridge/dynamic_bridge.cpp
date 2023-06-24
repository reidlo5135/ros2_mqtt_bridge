#include "ros2_mqtt_bridge/dynamic_bridge.hpp"

ros2_mqtt_bridge::BridgeManager::BridgeManager(rclcpp::Node::SharedPtr ros2_node_ptr, std::shared_ptr<ros2_mqtt_bridge::DynamicBridge> ros2_dynamic_bridge_ptr)
: mqtt_async_client_(MQTT_ADDRESS, MQTT_CLIENT_ID) {
    this->mqtt_connect();
    ros2_chatter_publisher_ptr_ = ros2_dynamic_bridge_ptr->register_publisher<std_msgs::msg::String>(ros2_node_ptr, "/chatter");

    std::function<void(std::shared_ptr<std_msgs::msg::String>)> chatter_callback = [this, ros2_node_ptr](const std_msgs::msg::String::SharedPtr chatter_callback_data) {
        RCLCPP_INFO(ros2_node_ptr->get_logger(), "chatter callback : %s", chatter_callback_data->data.c_str());
        this->mqtt_publish("/chatter", chatter_callback_data->data.c_str());
    };

    ros2_chatter_subscription_ptr_ = ros2_dynamic_bridge_ptr->register_subscription<std_msgs::msg::String>(ros2_node_ptr, "/chatter", chatter_callback);

    std::function<void(std::shared_ptr<std_msgs::msg::Header>)> header_callback = [this, ros2_node_ptr](const std_msgs::msg::Header::SharedPtr header_callback_data) {
        RCLCPP_INFO(ros2_node_ptr->get_logger(), "header callback : %s", header_callback_data->frame_id.c_str());
    };

    ros2_header_subscription_ptr_ = ros2_dynamic_bridge_ptr->register_subscription<std_msgs::msg::Header>(ros2_node_ptr, "/header", header_callback);
}

ros2_mqtt_bridge::BridgeManager::~BridgeManager() {

}

void ros2_mqtt_bridge::BridgeManager::mqtt_connect() {
    try {
        mqtt::connect_options mqtt_connect_opts;
        mqtt_connect_opts.set_clean_session(true);
        mqtt_async_client_.connect(mqtt_connect_opts)->wait_for(std::chrono::seconds(60));
        if(mqtt_async_client_.is_connected()) {
            RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, "MQTT connection success");
            mqtt_async_client_.set_callback(*this);
        } else {
            RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, "MQTT connection failed... trying to reconnect");
            mqtt_async_client_.connect(mqtt_connect_opts)->wait_for(std::chrono::seconds(30));
        }
    } catch (const mqtt::exception& mqtt_expn) {
        RCUTILS_LOG_ERROR_NAMED(ROS_NODE_NAME, "MQTT connection error [%s]", mqtt_expn.what());
    }
}

void ros2_mqtt_bridge::BridgeManager::connection_lost(const std::string& mqtt_connection_lost_cause) {
    RCUTILS_LOG_ERROR_NAMED(ROS_NODE_NAME, "MQTT connection lost with cause [%s]", mqtt_connection_lost_cause.c_str());
}

void ros2_mqtt_bridge::BridgeManager::message_arrived(mqtt::const_message_ptr mqtt_message) {
    const std::string& mqtt_topic = mqtt_message->get_topic();
    const std::string& mqtt_payload = mqtt_message->to_string();

    RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, "MQTT message arrived \n\t topic [%s] \n\t payload [%s]", mqtt_topic.c_str(), mqtt_payload.c_str());
}

void ros2_mqtt_bridge::BridgeManager::delivery_complete(mqtt::delivery_token_ptr mqtt_delivered_token) {
    RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, "delivery completed with [%s]", mqtt_delivered_token);
}

void ros2_mqtt_bridge::BridgeManager::mqtt_publish(const char * mqtt_topic, std::string mqtt_payload) {
    try {
		mqtt::message_ptr mqtt_publish_message_ptr = mqtt::make_message(mqtt_topic, mqtt_payload);
		mqtt_publish_message_ptr->set_qos(MQTT_DEFAULT_QOS);
		mqtt::delivery_token_ptr mqtt_delivery_token_ptr = mqtt_async_client_.publish(mqtt_publish_message_ptr);
        mqtt_delivery_token_ptr->wait();
        if (mqtt_delivery_token_ptr->get_return_code() != mqtt::SUCCESS) {
            RCUTILS_LOG_ERROR_NAMED(ROS_NODE_NAME, "MQTT publishing error with code [%i]", mqtt_delivery_token_ptr->get_return_code());
        }
	} catch (const mqtt::exception& mqtt_expn) {
		RCUTILS_LOG_ERROR_NAMED(ROS_NODE_NAME, "MQTT publishing error with code [%s]", mqtt_expn.what());
    }
}

void ros2_mqtt_bridge::BridgeManager::mqtt_subscribe(const char * mqtt_topic) {
    try {
        printf("\n");
        RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, "MQTT grant subscription with [%s]", mqtt_topic);
		mqtt_async_client_.subscribe(mqtt_topic, MQTT_DEFAULT_QOS);
	} catch (const mqtt::exception& mqtt_expn) {
        RCUTILS_LOG_ERROR_NAMED(ROS_NODE_NAME, "MQTT grant subscription error [%s]", mqtt_expn.what());
	}
}

ros2_mqtt_bridge::RCLNode::RCLNode()
: Node(ROS_NODE_NAME) {
    ros2_node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    ros2_dynamic_bridge_ptr_ = std::make_shared<ros2_mqtt_bridge::DynamicBridge>();
    bridge_manager_ptr_ = std::make_shared<ros2_mqtt_bridge::BridgeManager>(ros2_node_ptr_, ros2_dynamic_bridge_ptr_);
    this->get_current_topic_and_types();
}

ros2_mqtt_bridge::RCLNode::~RCLNode() {

}

void ros2_mqtt_bridge::RCLNode::get_current_topic_and_types() {
    std::map<std::string, std::string> ros2_publishers;
    std::map<std::string, std::string> ros2_subscriptions;
    std::map<std::string, std::map<std::string, std::string>> ros2_services;

    std::set<std::string> ros2_already_ignored_topics;
    std::set<std::string> ros2_already_ignored_services;

    std::function<void()> ros2_poll = [
        this,
        &ros2_publishers,
        &ros2_subscriptions,
        &ros2_services,
        &ros2_already_ignored_topics,
        &ros2_already_ignored_services
    ]() -> void {
        std::map<std::string, std::vector<std::string, std::allocator<std::string>>> ros2_topics = ros2_node_ptr_->get_topic_names_and_types();

        std::set<std::string> ros2_ignored_topics;
        ros2_ignored_topics.insert("/parameter_events");
        ros2_ignored_topics.insert("/rosout");

        std::map<std::string, std::string> ros2_current_publishers;
        std::map<std::string, std::string> ros2_current_subscriptions;

        for(std::pair<const std::string, std::vector<std::string, std::allocator<std::string>>> ros2_topic_and_types : ros2_topics) {
            if (ros2_ignored_topics.find(ros2_topic_and_types.first) != ros2_ignored_topics.end()) {
                continue;
            }

            const std::string& ros2_topic_name = ros2_topic_and_types.first;
            std::string& ros2_topic_type = ros2_topic_and_types.second[0];

            if (ros2_topic_and_types.second.size() > 1) {
                if (ros2_already_ignored_topics.count(ros2_topic_name) == 0) {
                    std::string types = "";
                    for (std::string type : ros2_topic_and_types.second) {
                        types += type + ", ";
                    }

                    RCLCPP_WARN(
                        ros2_node_ptr_->get_logger(),
                        "warning: ignoring topic '%s', which has more than one type: [%s]\n",
                        ros2_topic_name.c_str(),
                        types.substr(0, types.length() - 2).c_str()
                    );

                    ros2_already_ignored_topics.insert(ros2_topic_name);
                }

                continue;
            }

            const char * mqtt_subscription_topic = ros2_topic_name.c_str();
            bridge_manager_ptr_->mqtt_subscribe(mqtt_subscription_topic);
           
            size_t ros2_publisher_count = ros2_node_ptr_->count_publishers(ros2_topic_name);
            size_t ros2_subscription_count = ros2_node_ptr_->count_subscribers(ros2_topic_name);

            if (ros2_publisher_count) {
                ros2_current_publishers[ros2_topic_name] = ros2_topic_type;
            }

            if (ros2_subscription_count) {
                ros2_current_subscriptions[ros2_topic_name] = ros2_topic_type;
            }


            RCLCPP_INFO(
                ros2_node_ptr_->get_logger(),
                "ROS2 dynamic bridge current status \n\ttopic : [%s]\n\ttypes : [%s]\n\tcount : [%zu publishers, %zu subscriptions]\n",
                ros2_topic_name.c_str(), 
                ros2_topic_type.c_str(),
                ros2_publisher_count,
                ros2_subscription_count
            );

            {
                std::lock_guard<std::mutex> lock(bridge_mutex_);
                ros2_publishers = ros2_current_publishers;
                ros2_subscriptions = ros2_current_subscriptions;
            }
        }
    };

    ros2_poll();
}