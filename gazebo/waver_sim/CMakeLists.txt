cmake_minimum_required(VERSION 3.5)
project(waver_sim)

# Balik nekompiluje C++ kod.
# Iba instalujeme staticke subory (config, urdf/xacro, launch),
# aby ich vedel ROS2 launch najst cez share/waver_sim.
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config description launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
