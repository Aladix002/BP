cmake_minimum_required(VERSION 3.20)
project(waverover)

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

# motor drive executable
add_executable(waverover_motor
  src/main.cpp
  src/nodes/pca9685.cpp
  src/nodes/drive_node.cpp
)
target_include_directories(waverover_motor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(waverover_motor
  rclcpp geometry_msgs sensor_msgs
)

# instalacia: executables do lib/<pkg>/
install(TARGETS waverover_motor
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY web_ui DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/imu_serial_fusion_bridge.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/cmd_vel_odom.py             DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/simple_nav_node.py          DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/lidar_wander.py             DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/robot_mode_switch.py         DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/ball_follower_action.py     DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/ball_follow_bt_runner.py    DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/optical_flow.py             DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tools/visualisation/log_imu_wz_csv.py     DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tools/visualisation/log_imu_drive_csv.py    DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tools/visualisation/plot_imu_drive_csv.py   DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tools/visualisation/log_optical_flow_csv.py   DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tools/visualisation/plot_optical_flow_csv.py   DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tools/visualisation/log_optical_flow_debug_csv.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tools/visualisation/plot_optical_flow_debug_csv.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tools/visualisation/log_ball_follow_csv.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tools/visualisation/plot_ball_follow_csv.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/ball_follower_base.py         DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
