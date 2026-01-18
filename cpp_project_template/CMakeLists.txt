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
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
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
        include/nodes/rpi_camera_publisher.hpp
        src/nodes/rpi_camera_publisher.cpp
        include/nodes/libcamera_publisher.hpp
        src/nodes/libcamera_publisher.cpp
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
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs sensor_msgs geometry_msgs cv_bridge image_transport visualization_msgs)

# Standalone camera publisher executable (OpenCV/V4L2 - doesn't work on Pi 5)
add_executable(rpi_camera_publisher src/rpi_camera_main.cpp src/nodes/rpi_camera_publisher.cpp)
target_include_directories(rpi_camera_publisher PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS}
        /opt/ros/jazzy/include
)
target_link_libraries(rpi_camera_publisher ${OpenCV_LIBS})
ament_target_dependencies(rpi_camera_publisher rclcpp sensor_msgs cv_bridge std_msgs)

# Libcamera publisher executable (uses cam command - works on Pi 5)
add_executable(libcamera_publisher src/libcamera_main.cpp src/nodes/libcamera_publisher.cpp)
target_include_directories(libcamera_publisher PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS}
        /opt/ros/jazzy/include
)
target_link_libraries(libcamera_publisher ${OpenCV_LIBS})
ament_target_dependencies(libcamera_publisher rclcpp sensor_msgs cv_bridge std_msgs)

install(TARGETS ${PROJECT_NAME} rpi_camera_publisher libcamera_publisher DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

