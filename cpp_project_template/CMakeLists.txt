cmake_minimum_required(VERSION 3.20)
project(waverower)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ── Hlavný executable: motor HAT + manuálne/auto ovládanie ──────────────────
add_executable(waverower
    src/main.cpp
    src/nodes/motor_hat_i2c.cpp
    src/nodes/wasd_motor_hat.cpp
)
target_include_directories(waverower PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(waverower
    rclcpp geometry_msgs sensor_msgs
)

# ── Camera node: compressed image + voliteľný YOLO ──────────────────────────
add_executable(waverower_camera
    src/main_camera.cpp
    src/nodes/camera.cpp
)
target_include_directories(waverower_camera PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(waverower_camera
    rclcpp sensor_msgs std_msgs cv_bridge OpenCV ament_index_cpp
)

# ── Inštalácia ───────────────────────────────────────────────────────────────
install(TARGETS
    waverower
    waverower_camera
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY doc     DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/waverower_manual_teleop.sh DESTINATION share/${PROJECT_NAME}/scripts)

ament_package()
