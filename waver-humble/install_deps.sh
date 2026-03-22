#!/usr/bin/env bash
# Install ROS 2 Jazzy + Gazebo Harmonic dependencies for the WaveRover simulation.
# Run once as: bash install_deps.sh

set -e

sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image \
  ros-jazzy-ros-gz-interfaces \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-rviz-plugins \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-twist-mux \
  ros-jazzy-rviz2

# Python build tools (needed if using miniforge/conda Python)
pip install catkin_pkg empy lark

echo ""
echo "Dependencies installed. Build the workspace:"
echo "  cd $(dirname \$0)"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  colcon build --symlink-install"
echo "  source install/setup.bash"
