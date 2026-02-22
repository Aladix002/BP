cmake_minimum_required(VERSION 3.20)
project(prp_project)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SDL2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Find curl for HTTP communication
find_package(CURL REQUIRED)

# Main = manual_teleop + lidar_auto + cmd_mux + motor_driver; /scan z ldlidar_ros2
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/nodes/manual_teleop_node.cpp
    src/nodes/lidar_auto_node.cpp
    src/nodes/lidar_sectors.cpp
    src/nodes/cmd_mux_node.cpp
    src/nodes/motor_driver_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include /opt/ros/jazzy/include)
target_link_libraries(${PROJECT_NAME} ${CURL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs sensor_msgs)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

