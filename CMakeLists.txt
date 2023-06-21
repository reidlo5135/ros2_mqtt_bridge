cmake_minimum_required(VERSION 3.5)
project(ros2_mqtt_bridge)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  include/${PROJECT_NAME}
  /usr/local/include
  /usr/local/include/mqtt
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_library(PAHO_MQTT_CPP_LIB paho-mqttpp3 PATHS /usr/local/lib REQUIRED)

add_executable(
  dynamic_bridge
  src/ros2_mqtt_bridge/dynamic_bridge.cpp
  src/main.cpp
)

ament_target_dependencies(
  dynamic_bridge
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
)

target_link_libraries(
  dynamic_bridge
  ${PAHO_MQTT_CPP_LIB} -lpaho-mqtt3as
)

install(TARGETS
  dynamic_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()