cmake_minimum_required(VERSION 3.20)
project(waverower)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowBall.action"
)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# motor HAT executable (meno waverower drzi rosidl)
add_executable(waverower_motor
  src/main.cpp
  src/nodes/motor_hat_i2c.cpp
  src/nodes/motor_hat.cpp
)
target_include_directories(waverower_motor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(waverower_motor
  rclcpp geometry_msgs sensor_msgs
)

# optical flow (Lucas-Kanade)
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

# dense optical flow (Farneback)
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

# instalacia: executables do lib/<pkg>/
install(TARGETS waverower_motor optical_flow optical_flow_dense
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY web_ui DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/waverower_manual_teleop.sh DESTINATION share/${PROJECT_NAME}/scripts)
install(PROGRAMS scripts/imu_serial_fusion_bridge.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/imu_kalman_filter.py        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/lidar_wander.py             DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/robot_mode_switch.py         DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/ball_follower.py            DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/ball_follower_action.py     DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/ball_follow_bt_runner.py    DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/ball_follower_base.py         DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/cmd_vel_odometry.py         DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
