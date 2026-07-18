# MecaMind ROS2 具身智能麦克纳姆机器人项目

独立 ROS 2 Jazzy 课程工程，支持：

- Gazebo Harmonic 麦克纳姆轮仿真（课堂主线，UI/headless）。
- 内置轻量 2D 仿真（低配置与自动测试兜底）。
- LiDAR、IMU、RGB Camera、里程计和 TF。
- 后续六次课逐步接入 SLAM、Nav2、YOLO、视觉跟随、语音和大模型。

> 路径约定：下文假设项目位于 `~/mecamind_ros2`。若你放在其他目录，请自行替换。

## 获取代码与课程 Tag

克隆仓库后，**第一节课请切换到 tag `lesson1.0`**，保证与课堂资料版本一致：

```bash
git clone git@gitlab.0voice.com:2604_vip/10.3-mecamind_ros2.git ~/mecamind_ros2
cd ~/mecamind_ros2
git fetch --tags
git checkout lesson1.0
```

说明：

- `git fetch --tags` 成功时可能没有输出，属正常；用 `git tag -l` 确认本地已有 `lesson1.0`。
- 查看远端 tag：`git ls-remote --tags origin`
- 后续课程会继续打 `lesson2.0`、`lesson3.0` …，按当节课说明切换即可。
- 若要回到最新开发分支：`git checkout main && git pull`

讲义：

- 第一节课：`docs/10.3.1_项目总览与仿真环境搭建.md`
- 第二节课：`docs/10.3.2_SLAM建图与地图管理.md`

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
cd ~/mecamind_ros2
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
source ~/mecamind_ros2/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

键盘小写 `i/,/j/l` 用于前后和旋转；大写 `I/J/L/<` 用于全向平移。

## CP1 验收

```bash
bash scripts/test_cp1.sh
```

该脚本在 headless Gazebo 中检查 `/clock`、`/scan`、`/imu/data`、`/odom`，发送前进、横移、旋转命令并确认里程计发生变化。

## 第二次课：SLAM 建图

统一建图入口（默认 Gazebo + RViz，手动遥控）：

```bash
ros2 launch mecamind_bringup mapping.launch.py
```

遥控需 remap 到安全门入口：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_nav
```

半自动建图（路线驱动 + 自动存图）：

```bash
ros2 launch mecamind_bringup mapping.launch.py auto_route:=true
```

轻量仿真兜底：

```bash
ros2 launch mecamind_bringup mapping.launch.py backend:=lite auto_route:=true
```

地图默认写到项目内 `maps/`（见 `maps/README.md`）。质量分析示例：

```bash
WORLD="$(ros2 pkg prefix mecamind_tools)/share/mecamind_tools/worlds/three_room_house.world"
ros2 run mecamind_tools mecamind_map_quality_analyzer maps/mecamind_three_room_map.yaml \
  --world "$WORLD" --json
```

## CP2 验收

```bash
bash scripts/test_cp2.sh
```

默认使用轻量后端做无界面半自动建图、存图与质量验收。Gazebo headless：

```bash
MECAMIND_CP2_BACKEND=gazebo bash scripts/test_cp2.sh
```

成功标志：`MECAMIND_CP2_OK`。跑验收前建议先 `bash scripts/cleanup_ros2.sh`，避免残留仿真抢话题。

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
