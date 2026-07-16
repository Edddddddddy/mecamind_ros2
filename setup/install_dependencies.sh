#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-pytest \
  ros-jazzy-ros-gz \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-tf2-tools \
  ros-jazzy-rqt-image-view \
  ros-jazzy-rqt-graph \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro

echo "MecaMind ROS2 第一次课依赖安装完成。"
