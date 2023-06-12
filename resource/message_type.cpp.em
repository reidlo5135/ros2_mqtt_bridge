
#include "@(ros2_package_name)_factories.hpp"

namespace ros2_mqtt_bridge {

std::shared_ptr<FactoryInterface> get_factory_@(ros2_package_name)(const std::string & ros2_message_type) {
    @[if not ros2_msg_types]@
    (void)ros2_message_type;
    @[else]@
    std::shared_ptr<FactoryInterface> factory;
    @[end if]@
    @[for m in ros2_msg_types]@
    factory = get_factory_@(ros2_package_name)__msg__@(m.message_name)(ros2_message_type);
    if (factory) {
        return factory;
    }
    @[end for]@
    return std::shared_ptr<FactoryInterface>();
    }
}