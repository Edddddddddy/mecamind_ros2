#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

if [ -f "$HOME/ros2_ai_stage3_ws/install/setup.bash" ]; then
  source "$HOME/ros2_ai_stage3_ws/install/setup.bash"
elif [ -f "/tmp/ros2_ai_3_2_manual_ws/install/setup.bash" ]; then
  source /tmp/ros2_ai_3_2_manual_ws/install/setup.bash
fi

URDF_FILE=$(ros2 pkg prefix my_robot_description)/share/my_robot_description/urdf/diff_drive.urdf
RVIZ_CONFIG=$(ros2 pkg prefix my_robot_description)/share/my_robot_description/rviz/display.rviz

echo "Checking $URDF_FILE"
check_urdf "$URDF_FILE"

# display_xacro.launch.py 声明了 model / rviz_config 参数（xacro 也能展开纯 URDF）
ros2 launch my_robot_description display_xacro.launch.py \
  model:="$URDF_FILE" \
  rviz_config:="$RVIZ_CONFIG"

