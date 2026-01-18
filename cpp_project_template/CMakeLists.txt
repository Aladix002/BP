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

# Find curl for HTTP communication
find_package(CURL REQUIRED)

set(SRC
        include/nodes/io_node.hpp
        src/nodes/io_node.cpp
        include/nodes/motor.hpp
        src/nodes/motor.cpp
        include/nodes/kinematics_odometry.hpp
        src/nodes/kinematics_odometry.cpp
        include/nodes/line.hpp
        src/nodes/line.cpp
        include/nodes/button.hpp
        src/nodes/button.cpp
        src/nodes/lidar.cpp
        include/nodes/lidar.hpp
        src/nodes/imu_node.cpp
        include/nodes/imu_node.hpp
        include/nodes/planar_imu_integrator.hpp
        src/nodes/planar_imu_integrator.cpp
        src/nodes/pid.cpp
        include/nodes/pid.hpp
        src/nodes/camera.cpp
        include/nodes/camera.hpp
        include/nodes/esp32_bridge.hpp
        src/nodes/esp32_bridge.cpp
        include/nodes/lidar_bridge.hpp
        src/nodes/lidar_bridge.cpp
)

set(AMENT_LIBS rclcpp std_msgs sensor_msgs geometry_msgs cv_bridge image_transport)

add_executable(${PROJECT_NAME} src/main.cpp src/nodes/io_node.cpp src/nodes/motor.cpp src/nodes/kinematics_odometry.cpp ${SRC})
target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS}
        ${SDL2_INCLUDE_DIRS}
        /opt/ros/jazzy/include
)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${SDL2_LIBS} ${CURL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

