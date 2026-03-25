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
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

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

# ── Optical flow node: zarovnávanie jazdy podľa kamery (Lucas-Kanade) ────────
add_executable(optical_flow
    src/main_optical_flow.cpp
    src/nodes/optical_flow.cpp
)
target_include_directories(optical_flow PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(optical_flow
    rclcpp geometry_msgs sensor_msgs cv_bridge
)
target_link_libraries(optical_flow ${OpenCV_LIBS})

# ── Dense optical flow node: zarovnávanie jazdy podľa kamery (Farneback) ─────
add_executable(optical_flow_dense
    src/main_optical_flow_dense.cpp
    src/nodes/optical_flow_dense.cpp
)
target_include_directories(optical_flow_dense PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(optical_flow_dense
    rclcpp geometry_msgs sensor_msgs cv_bridge
)
target_link_libraries(optical_flow_dense ${OpenCV_LIBS})

# ── Inštalácia ───────────────────────────────────────────────────────────────
install(TARGETS
    waverower
    optical_flow
    optical_flow_dense
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY web_ui  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/waverower_manual_teleop.sh DESTINATION share/${PROJECT_NAME}/scripts)

ament_package()
