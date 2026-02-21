cmake_minimum_required(VERSION 3.20)
project(rpi_motor_node)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Motor Subscriber Node (direct GPIO control - not used if ESP32 controls motors)
add_executable(motor_subscriber_node motor_subscriber_node.cpp)
ament_target_dependencies(motor_subscriber_node rclcpp std_msgs)
install(TARGETS motor_subscriber_node DESTINATION lib/${PROJECT_NAME})

# Motor ESP32 Bridge Node (sends JSON commands to ESP32 via Serial/UART)
add_executable(motor_esp32_bridge_node motor_esp32_bridge_node.cpp)
ament_target_dependencies(motor_esp32_bridge_node rclcpp std_msgs)
install(TARGETS motor_esp32_bridge_node DESTINATION lib/${PROJECT_NAME})

ament_package()

