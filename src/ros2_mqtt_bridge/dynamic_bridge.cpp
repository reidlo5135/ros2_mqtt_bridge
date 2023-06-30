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
ros2_mqtt_bridge::BridgeManager::BridgeManager(rclcpp::Node::SharedPtr rcl_node_ptr, std::shared_ptr<ros2_mqtt_bridge::DynamicBridge> rcl_dynamic_bridge_ptr)
: mqtt_async_client_(MQTT_ADDRESS, MQTT_CLIENT_ID) {
    this->mqtt_connect();
    rcl_chatter_publisher_ptr_ = rcl_dynamic_bridge_ptr->register_publisher<std_msgs::msg::String>(rcl_node_ptr, RCL_CHATTER_TOPIC);

    std::function<void(std::shared_ptr<std_msgs::msg::String>)> rcl_chatter_callback = [this, rcl_node_ptr](const std_msgs::msg::String::SharedPtr rcl_chatter_callback_data) {
        RCLCPP_INFO(rcl_node_ptr->get_logger(), "rcl [%s] subscription callback : [%s]", RCL_CHATTER_TOPIC, rcl_chatter_callback_data->data.c_str());
        RCLCPP_LINE_INFO();
        this->mqtt_publish(RCL_CHATTER_TOPIC, rcl_chatter_callback_data->data.c_str());
    };

    rcl_chatter_subscription_ptr_ = rcl_dynamic_bridge_ptr->register_subscription<std_msgs::msg::String>(rcl_node_ptr, RCL_CHATTER_TOPIC, rcl_chatter_callback);
}

/**
* Destroy this class' instance
* @brief Default Destructor
*/
ros2_mqtt_bridge::BridgeManager::~BridgeManager() {

}

/**
* @brief function for MQTT connection
* @return void
*/
void ros2_mqtt_bridge::BridgeManager::mqtt_connect() {
    try {
        mqtt::connect_options mqtt_connect_opts;
        mqtt_connect_opts.set_clean_session(true);
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
void ros2_mqtt_bridge::BridgeManager::connection_lost(const std::string & mqtt_connection_lost_cause) {
    RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "MQTT connection lost with cause [%s]", mqtt_connection_lost_cause.c_str());
    RCLCPP_LINE_ERROR();
}

/**
* @brief overrided function for handle when MQTT message arrived
* @param mqtt_message callback for MQTT message
* @return void
*/
void ros2_mqtt_bridge::BridgeManager::message_arrived(mqtt::const_message_ptr mqtt_message) {
    const std::string & mqtt_topic = mqtt_message->get_topic();
    const std::string & mqtt_payload = mqtt_message->to_string();

    RCUTILS_LOG_INFO_NAMED(
        RCL_NODE_NAME,
        "MQTT message arrived \n\ttopic : [%s]\n\tpayload : [%s]",
        mqtt_topic.c_str(),
        mqtt_payload.c_str()
    );
    RCLCPP_LINE_INFO();
}

/**
* @brief overrided function for handle when MQTT delivery completed
* @param mqtt_delivered_token callback for MQTT delivery token
* @return void
*/
void ros2_mqtt_bridge::BridgeManager::delivery_complete(mqtt::delivery_token_ptr mqtt_delivered_token) {
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "delivery completed with [%s]", mqtt_delivered_token);
    RCLCPP_LINE_INFO();
}

/**
* @brief function for MQTT publish
* @param mqtt_topic target MQTT topic
* @param mqtt_payload target MQTT payload
* @return void
*/
void ros2_mqtt_bridge::BridgeManager::mqtt_publish(const char * mqtt_topic, std::string mqtt_payload) {
    try {
		mqtt::message_ptr mqtt_publish_message_ptr = mqtt::make_message(mqtt_topic, mqtt_payload);
		mqtt_publish_message_ptr->set_qos(MQTT_DEFAULT_QOS);
		mqtt::delivery_token_ptr mqtt_delivery_token_ptr = mqtt_async_client_.publish(mqtt_publish_message_ptr);
        mqtt_delivery_token_ptr->wait();
        if (mqtt_delivery_token_ptr->get_return_code() != mqtt::SUCCESS) {
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
void ros2_mqtt_bridge::BridgeManager::mqtt_subscribe(const char * mqtt_topic) {
    try {
        printf("\n");
        RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "MQTT grant subscription with [%s]", mqtt_topic);
        RCLCPP_LINE_INFO();
		mqtt_async_client_.subscribe(mqtt_topic, MQTT_DEFAULT_QOS);
	} catch (const mqtt::exception & mqtt_expn) {
        RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "MQTT grant subscription error [%s]", mqtt_expn.what());
        RCLCPP_LINE_ERROR();
	}
}

/**
* Create a new this class' instance and extends rclcpp::Node
* @brief Default Constructor
* @see rclcpp::Node
*/
ros2_mqtt_bridge::RCLNode::RCLNode()
: Node(RCL_NODE_NAME) {
    rcl_node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    rcl_dynamic_bridge_ptr_ = std::make_shared<ros2_mqtt_bridge::DynamicBridge>();
    bridge_manager_ptr_ = std::make_shared<ros2_mqtt_bridge::BridgeManager>(rcl_node_ptr_, rcl_dynamic_bridge_ptr_);
    this->get_current_topics_and_types();
}

/**
* Destroy this class' instance
* @brief Default Destructor
*/
ros2_mqtt_bridge::RCLNode::~RCLNode() {

}

/**
* Function for handle signal when program exit
* @param sig The signal of input
* @return void
* @see signal.h
*/
void ros2_mqtt_bridge::RCLNode::sig_handler(int sig) {
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "\n ros2_mqtt_bridge stopped with SIG [%i] \n", sig);
	signal(sig, SIG_IGN);
	exit(0);
}

/**
* @brief function for get current ROS2 topics and types
* @return void
*/
void ros2_mqtt_bridge::RCLNode::get_current_topics_and_types() {
    std::map<std::string, std::string> rcl_publishers;
    std::map<std::string, std::string> rcl_subscriptions;
    std::map<std::string, std::map<std::string, std::string>> rcl_services;

    std::set<std::string> rcl_already_ignored_topics;
    std::set<std::string> rcl_already_ignored_services;

    std::function<void()> ros2_poll = [
        this,
        &rcl_publishers,
        &rcl_subscriptions,
        &rcl_services,
        &rcl_already_ignored_topics,
        &rcl_already_ignored_services
    ]() -> void {
        std::map<std::string, std::vector<std::string, std::allocator<std::string>>> rcl_topics = rcl_node_ptr_->get_topic_names_and_types();

        std::set<std::string> rcl_ignored_topics;
        rcl_ignored_topics.insert("/parameter_events");
        rcl_ignored_topics.insert("/rosout");

        std::map<std::string, std::string> rcl_current_publishers;
        std::map<std::string, std::string> rcl_current_subscriptions;

        for(std::pair<const std::string, std::vector<std::string, std::allocator<std::string>>> rcl_topic_and_types : rcl_topics) {
            if (rcl_ignored_topics.find(rcl_topic_and_types.first) != rcl_ignored_topics.end()) {
                continue;
            }

            const std::string & rcl_topic_name = rcl_topic_and_types.first;
            std::string & rcl_topic_type = rcl_topic_and_types.second[0];

            if (rcl_topic_and_types.second.size() > 1) {
                if (rcl_already_ignored_topics.count(rcl_topic_name) == 0) {
                    std::string types = "";
                    for (std::string type : rcl_topic_and_types.second) {
                        types += type + ", ";
                    }

                    RCLCPP_WARN(
                        rcl_node_ptr_->get_logger(),
                        "warning: ignoring topic '%s', which has more than one type: [%s]\n",
                        rcl_topic_name.c_str(),
                        types.substr(0, types.length() - 2).c_str()
                    );

                    rcl_already_ignored_topics.insert(rcl_topic_name);
                }

                continue;
            }

            const char * mqtt_subscription_topic = rcl_topic_name.c_str();
            bridge_manager_ptr_->mqtt_subscribe(mqtt_subscription_topic);

            size_t rcl_publisher_count = rcl_node_ptr_->count_publishers(rcl_topic_name);
            size_t rcl_subscription_count = rcl_node_ptr_->count_subscribers(rcl_topic_name);

            if (rcl_publisher_count) {
                rcl_current_publishers[rcl_topic_name] = rcl_topic_type;
            }

            if (rcl_subscription_count) {
                rcl_current_subscriptions[rcl_topic_name] = rcl_topic_type;
            }


            RCLCPP_INFO(
                rcl_node_ptr_->get_logger(),
                "ROS2 dynamic bridge current status \n\ttopic : [%s]\n\ttypes : [%s]\n\tcount : [%zu publishers, %zu subscriptions]\n",
                rcl_topic_name.c_str(), 
                rcl_topic_type.c_str(),
                rcl_publisher_count,
                rcl_subscription_count
            );
            RCLCPP_LINE_INFO();

            {
                std::lock_guard<std::mutex> lock(bridge_mutex_);
                rcl_publishers = rcl_current_publishers;
                rcl_subscriptions = rcl_current_subscriptions;
            }
        }
    };

    ros2_poll();
}