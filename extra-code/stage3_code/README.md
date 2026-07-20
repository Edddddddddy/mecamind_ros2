# Stage 3 Source Code

This folder contains the runnable examples for Stage 3 courseware. Each lesson
is tested independently and has a matching manual report under `self_test`.

## 3.1 TF and Transform Math

Package: `tf_math_demo`

Build in a temporary workspace:

```bash
mkdir -p /tmp/ros2_ai_3_1_manual_ws/src
cp -r /mnt/f/ros2_ai/stage3_code/tf_math_demo /tmp/ros2_ai_3_1_manual_ws/src/
cd /tmp/ros2_ai_3_1_manual_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tf_math_demo
source install/setup.bash
```

Run the pure math demo:

```bash
ros2 run tf_math_demo transform_math_demo
```

Run the TF point transform demo after TurtleBot3 TF is being published:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_description robot_state_publisher.launch.py

# another terminal
source /opt/ros/jazzy/setup.bash
source /tmp/ros2_ai_3_1_manual_ws/install/setup.bash
ros2 run tf_math_demo tf_point_transform_demo
```

## 3.2 URDF Basics

Package: `my_robot_description`

Build in a temporary workspace:

```bash
mkdir -p /tmp/ros2_ai_3_2_manual_ws/src
cp -r /mnt/f/ros2_ai/stage3_code/my_robot_description /tmp/ros2_ai_3_2_manual_ws/src/
cd /tmp/ros2_ai_3_2_manual_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select my_robot_description
source install/setup.bash
```

Validate URDF files:

```bash
check_urdf $(ros2 pkg prefix my_robot_description)/share/my_robot_description/urdf/first_box.urdf
check_urdf $(ros2 pkg prefix my_robot_description)/share/my_robot_description/urdf/my_robot.urdf
check_urdf $(ros2 pkg prefix my_robot_description)/share/my_robot_description/urdf/diff_drive.urdf
```

Launch the visual demo for a specific plain URDF (`display.launch.py` always
loads the full `my_robot.urdf.xacro`; use `display_xacro.launch.py model:=...`
to display the 3.2 URDF files):

```bash
ros2 launch my_robot_description display_xacro.launch.py \
  model:=$(ros2 pkg prefix my_robot_description)/share/my_robot_description/urdf/diff_drive.urdf
```

Or run the helper script:

```bash
ros2 run my_robot_description display_robot.sh
```

## 3.3 Xacro Advanced Modeling

Package: `my_robot_description`

The same package now also contains modular Xacro files:

```text
urdf/my_robot.urdf.xacro
urdf/my_robot/properties.xacro
urdf/my_robot/materials.xacro
urdf/my_robot/inertia_macros.xacro
urdf/my_robot/macros.xacro
urdf/my_robot/base.xacro
urdf/my_robot/sensors.xacro
urdf/my_robot/wheels.xacro
urdf/my_robot/gazebo_materials.xacro
urdf/exercise_properties.urdf.xacro
```

Expand and validate:

```bash
xacro $(ros2 pkg prefix my_robot_description)/share/my_robot_description/urdf/my_robot.urdf.xacro -o /tmp/my_robot_from_xacro.urdf
check_urdf /tmp/my_robot_from_xacro.urdf

xacro $(ros2 pkg prefix my_robot_description)/share/my_robot_description/urdf/exercise_properties.urdf.xacro -o /tmp/exercise_properties.urdf
check_urdf /tmp/exercise_properties.urdf
```

Launch the Xacro visual demo:

```bash
ros2 launch my_robot_description display_xacro.launch.py
```

## 3.4 Simulation-Ready Robot Description

Package: `my_robot_description`

The same package is extended with collision geometry, inertial tags, Gazebo
material hints, a production `display.launch.py`, and a minimal launch exercise.

Build and expand:

```bash
mkdir -p /tmp/ros2_ai_3_4_manual_ws/src
cp -r /mnt/f/ros2_ai/stage3_code/my_robot_description /tmp/ros2_ai_3_4_manual_ws/src/
cd /tmp/ros2_ai_3_4_manual_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select my_robot_description
source install/setup.bash

xacro $(ros2 pkg prefix my_robot_description)/share/my_robot_description/urdf/my_robot.urdf.xacro -o /tmp/my_robot_sim_ready.urdf
check_urdf /tmp/my_robot_sim_ready.urdf
grep -c '<collision>' /tmp/my_robot_sim_ready.urdf
grep -c '<inertial>' /tmp/my_robot_sim_ready.urdf
```

Launch with or without GUI:

```bash
ros2 launch my_robot_description display.launch.py
ros2 launch my_robot_description display.launch.py use_gui:=false use_rviz:=false
ros2 launch my_robot_description minimal_display.launch.py
```

## 3.5 P2 Milestone 1 Mobile Robot URDF

Package: `my_robot_description`

The final Stage 3 model adds a rear caster wheel and verification script. The
expanded model has 6 links, 5 joints, 5 collisions, 5 inertials, and Gazebo
material hints for all physical links.

Build and verify:

```bash
mkdir -p /tmp/ros2_ai_3_5_manual_ws/src
cp -r /mnt/f/ros2_ai/stage3_code/my_robot_description /tmp/ros2_ai_3_5_manual_ws/src/
cd /tmp/ros2_ai_3_5_manual_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select my_robot_description
source install/setup.bash

ros2 run my_robot_description verify_urdf.sh
ros2 launch my_robot_description display.launch.py
```
