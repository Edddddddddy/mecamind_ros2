# Stage 3 配套源码

本目录是第三阶段课件的可运行示例，每讲都可以独立验证，对应的手动测试报告在 `self_test/stage3/`。所有源码已添加面向初学者的中文注释。

下面所有命令中的 `REPO` 请先设置为本仓库的实际路径：

```bash
REPO=/mnt/f/ros2_ai   # 改成你的课程仓库实际路径（如克隆在 WSL 内则形如 ~/ros2_ai）
```

## 3.1 TF 与坐标变换数学

包：`tf_math_demo`

在临时工作空间中构建：

```bash
mkdir -p /tmp/ros2_ai_3_1_manual_ws/src
cp -r "$REPO"/stage3_code/tf_math_demo /tmp/ros2_ai_3_1_manual_ws/src/
cd /tmp/ros2_ai_3_1_manual_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tf_math_demo
source install/setup.bash
```

运行纯数学演示（齐次变换矩阵、链式相乘）：

```bash
ros2 run tf_math_demo transform_math_demo
```

在 TurtleBot3 的 TF 发布之后，运行 TF 点变换演示：

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_description robot_state_publisher.launch.py

# 另开一个终端
source /opt/ros/jazzy/setup.bash
source /tmp/ros2_ai_3_1_manual_ws/install/setup.bash
ros2 run tf_math_demo tf_point_transform_demo
```

## 3.2 URDF 基础

包：`my_robot_description`

在临时工作空间中构建：

```bash
mkdir -p /tmp/ros2_ai_3_2_manual_ws/src
cp -r "$REPO"/stage3_code/my_robot_description /tmp/ros2_ai_3_2_manual_ws/src/
cd /tmp/ros2_ai_3_2_manual_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select my_robot_description
source install/setup.bash
```

校验 URDF 文件：

```bash
check_urdf $(ros2 pkg prefix my_robot_description)/share/my_robot_description/urdf/first_box.urdf
check_urdf $(ros2 pkg prefix my_robot_description)/share/my_robot_description/urdf/my_robot.urdf
check_urdf $(ros2 pkg prefix my_robot_description)/share/my_robot_description/urdf/diff_drive.urdf
```

可视化指定的普通 URDF 文件（`display.launch.py` 固定加载完整的
`my_robot.urdf.xacro`；想显示 3.2 的 URDF 文件请用
`display_xacro.launch.py model:=...`）：

```bash
ros2 launch my_robot_description display_xacro.launch.py \
  model:=$(ros2 pkg prefix my_robot_description)/share/my_robot_description/urdf/diff_drive.urdf
```

或者运行辅助脚本：

```bash
ros2 run my_robot_description display_robot.sh
```

## 3.3 Xacro 进阶建模

包：`my_robot_description`

同一个包内包含模块化的 Xacro 文件：

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

展开并校验：

```bash
xacro $(ros2 pkg prefix my_robot_description)/share/my_robot_description/urdf/my_robot.urdf.xacro -o /tmp/my_robot_from_xacro.urdf
check_urdf /tmp/my_robot_from_xacro.urdf

xacro $(ros2 pkg prefix my_robot_description)/share/my_robot_description/urdf/exercise_properties.urdf.xacro -o /tmp/exercise_properties.urdf
check_urdf /tmp/exercise_properties.urdf
```

启动 Xacro 可视化演示：

```bash
ros2 launch my_robot_description display_xacro.launch.py
```

## 3.4 仿真准备与机器人描述包

包：`my_robot_description`

同一个包在本讲扩展了碰撞几何（collision）、惯性标签（inertial）、
Gazebo 材质提示、正式版 `display.launch.py` 和最小 Launch 练习。

构建并展开：

```bash
mkdir -p /tmp/ros2_ai_3_4_manual_ws/src
cp -r "$REPO"/stage3_code/my_robot_description /tmp/ros2_ai_3_4_manual_ws/src/
cd /tmp/ros2_ai_3_4_manual_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select my_robot_description
source install/setup.bash

xacro $(ros2 pkg prefix my_robot_description)/share/my_robot_description/urdf/my_robot.urdf.xacro -o /tmp/my_robot_sim_ready.urdf
check_urdf /tmp/my_robot_sim_ready.urdf
grep -c '<collision>' /tmp/my_robot_sim_ready.urdf
grep -c '<inertial>' /tmp/my_robot_sim_ready.urdf
```

带/不带 GUI 启动：

```bash
ros2 launch my_robot_description display.launch.py
ros2 launch my_robot_description display.launch.py use_gui:=false use_rviz:=false
ros2 launch my_robot_description minimal_display.launch.py
```

## 3.5 P2 里程碑 1：移动机器人 URDF

包：`my_robot_description`

Stage 3 的最终模型增加了后万向轮（caster）和校验脚本。展开后的模型
包含 6 个 link、5 个 joint、5 个 collision、5 个 inertial，所有物理
link 都带 Gazebo 材质提示。

构建并验证：

```bash
mkdir -p /tmp/ros2_ai_3_5_manual_ws/src
cp -r "$REPO"/stage3_code/my_robot_description /tmp/ros2_ai_3_5_manual_ws/src/
cd /tmp/ros2_ai_3_5_manual_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select my_robot_description
source install/setup.bash

ros2 run my_robot_description verify_urdf.sh
ros2 launch my_robot_description display.launch.py
```
