cmake_minimum_required(VERSION 3.20)
project(waverower)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
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

# Main = manual + lidar + behavioral_tree_auto + cmd_mux + motor_driver + camera + imu
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/nodes/manual.cpp
    src/nodes/lidar.cpp
    src/nodes/behavioral_tree_auto.cpp
    src/nodes/cmd_mux.cpp
    src/nodes/motor_driver.cpp
    src/nodes/camera.cpp
    src/nodes/imu.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include /opt/ros/jazzy/include)
target_link_libraries(${PROJECT_NAME} ${CURL_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(${PROJECT_NAME} ament_index_cpp rclcpp std_msgs sensor_msgs cv_bridge)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

