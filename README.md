# MecaMind ROS2 具身智能麦克纳姆机器人项目

独立 ROS 2 Jazzy 课程工程，支持：

- Gazebo Harmonic 麦克纳姆轮仿真（课堂主线，UI/headless）。
- 内置轻量 2D 仿真（低配置与自动测试兜底）。
- LiDAR、IMU、RGB Camera、里程计和 TF。
- 后续六次课逐步接入 SLAM、Nav2、YOLO、视觉跟随、语音和大模型。

 

## 环境

- Windows 10 + WSL2 Ubuntu 24.04（支持 WSL GUI）
- ROS 2 Jazzy
- Gazebo Harmonic

检查环境：

```bash
bash setup/check_environment.sh
```

安装缺失依赖：

```bash
bash setup/install_dependencies.sh
```

## 构建

```bash
cd /home/lqf/ros2_ai/mecamind_ros2
bash scripts/build.sh
source install/setup.bash
```

## 第一次课运行

Gazebo UI：

```bash
ros2 launch mecamind_bringup gazebo.launch.py use_gui:=true
```

Gazebo 无界面：

```bash
ros2 launch mecamind_bringup gazebo.launch.py use_gui:=false
```

轻量仿真兜底：

```bash
ros2 launch mecamind_bringup lite_sim.launch.py
```

另一终端启动全向键盘遥控：

```bash
source /opt/ros/jazzy/setup.bash
source /home/lqf/ros2_ai/mecamind_ros2/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

键盘小写 `i/,/j/l` 用于前后和旋转；大写 `I/J/L/<` 用于全向平移。

## CP1 验收

```bash
bash scripts/test_cp1.sh
```

该脚本在 headless Gazebo 中检查 `/clock`、`/scan`、`/imu/data`、`/odom`，发送前进、横移、旋转命令并确认里程计发生变化。

## 一键清理测试残留

默认只清理本项目启动的 ROS 2 和 Gazebo 进程：

```bash
bash scripts/cleanup_ros2.sh
```

先预览当前用户的全部 ROS 2/Gazebo 残留：

```bash
bash scripts/cleanup_ros2.sh --all --dry-run
```

确认后执行全量清理：

```bash
bash scripts/cleanup_ros2.sh --all
```

`--all` 会终止当前 Linux 用户启动的其他 ROS 2、RViz 和 Gazebo 进程，请勿在其他 ROS 工程正在运行时使用。
