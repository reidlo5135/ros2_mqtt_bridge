#include "ros2_mqtt_bridge/dynamic_bridge.hpp"

ros2_mqtt_bridge::RCLNode::RCLNode()
: Node("ros2_mqtt_bridge") {
    ros2_node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    ros2_dynamic_bridge_ptr_ = std::make_shared<ros2_mqtt_bridge::DynamicBridge>();

    ros2_chatter_publisher_ptr_ = ros2_dynamic_bridge_ptr_->register_publisher<std_msgs::msg::String>(ros2_node_ptr_, "/chatter");
    ros2_chatter_subscription_ptr_ = ros2_dynamic_bridge_ptr_->register_subscription<std_msgs::msg::String>(ros2_node_ptr_, "/chatter");
}

ros2_mqtt_bridge::RCLNode::~RCLNode() {

}

void ros2_mqtt_bridge::RCLNode::get_current_topic_and_types() {
    std::map<std::string, std::string> ros2_publishers;
    std::map<std::string, std::string> ros2_subscriptions;
    std::map<std::string, std::map<std::string, std::string>> ros2_services;

    std::set<std::string> ros2_already_ignored_topics;
    std::set<std::string> ros2_already_ignored_services;

    auto ros2_poll = [
        this,
        &ros2_publishers,
        &ros2_subscriptions,
        &ros2_services,
        &ros2_already_ignored_topics,
        &ros2_already_ignored_services
    ]() -> void {
        std::map<std::string, std::vector<std::string, std::allocator<std::string>>> ros2_topics = ros2_node_ptr_->get_topic_names_and_types();

        std::set<std::string> ros2_ignored_topics;
        ros2_ignored_topics.insert("parameter_events");

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

            size_t ros2_publisher_count = ros2_node_ptr_->count_publishers(ros2_topic_name);
            size_t ros2_subscription_count = ros2_node_ptr_->count_subscribers(ros2_topic_name);

            if (ros2_publisher_count) {
                ros2_current_publishers[ros2_topic_name] = ros2_topic_type;
            }

            if (ros2_subscription_count) {
                ros2_current_subscriptions[ros2_topic_name] = ros2_topic_type;
            }


            // RCLCPP_INFO(
            //     ros2_node_ptr->get_logger(),
            //     "ROS 2: %s (%s) [%zu publishers, %zu subscriptions]\n",
            //     ros2_topic_name.c_str(), 
            //     ros2_topic_type.c_str(),
            //     ros2_publisher_count,
            //     ros2_subscription_count
            // );

            {
                std::lock_guard<std::mutex> lock(bridge_mutex_);
                ros2_publishers = ros2_current_publishers;
                ros2_subscriptions = ros2_current_subscriptions;
            }
        }
    };

    auto ros2_poll_timer = ros2_node_ptr_->create_wall_timer(std::chrono::seconds(1), ros2_poll);
}