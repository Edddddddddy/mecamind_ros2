# Stage 1 代码与 LAB 材料

本目录提供 **1.3 LAB** 与 **1.5 LAB** 的参考实现，可直接复制到工作空间或按路径加载。所有源码已添加面向初学者的中文注释。

下面所有命令中的 `REPO` 请先设置为本仓库的实际路径：

```bash
REPO=/mnt/f/ros2_ai   # 改成你的课程仓库实际路径（如克隆在 WSL 内则形如 ~/ros2_ai）
```

## 目录结构

```
stage1_code/
├── README.md
├── hello_ros2/                  # 1.3 LAB — 完整 ament_python 包（可 colcon build）
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── resource/hello_ros2
│   ├── hello_ros2/
│   │   └── talker_node.py
│   └── test/
│       └── test_talker_node.py
├── config/
│   └── turtlebot3_basic.rviz   # 1.5 LAB — TurtleBot3 RViz 预设
├── scripts/
│   └── check_env.sh            # 1.3 — 环境诊断
└── vscode/
    └── settings.json           # 1.3 — VS Code 配置模板
```

## 对应讲义

| 材料 | 讲义 | 说明 |
|------|------|------|
| `hello_ros2/` | 1.3 讲次 8 LAB | 参考答案、退出保护与最小单测 |
| `config/turtlebot3_basic.rviz` | 1.5 讲次 4 LAB | Fixed Frame=`odom`，含 Grid + TF + RobotModel + LaserScan |
| `scripts/check_env.sh` | 1.3 环境排查 | ROS、GUI、工具与可选工作空间诊断 |
| `vscode/settings.json` | 1.3 讲次 5 | VS Code 工作区模板 |

## hello_ros2 — 构建与运行

将包复制到工作空间后构建（路径按你的 clone 位置调整）：

```bash
cp -r "$REPO"/stage1_code/hello_ros2 ~/ros2_ws/src/

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select hello_ros2
colcon test --packages-select hello_ros2 --event-handlers console_cohesion+
colcon test-result --verbose
source install/setup.bash
ros2 run hello_ros2 talker
```

## turtlebot3_basic.rviz — 1.5 LAB 使用

启动 Gazebo 仿真后，加载预设（无需手动点选 Display）：

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
# 终端 1：ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 终端 2：加载课程仓库预设
ros2 run rviz2 rviz2 -d "$REPO"/stage1_code/config/turtlebot3_basic.rviz
```

也可复制到工作空间备用：

```bash
cp "$REPO"/stage1_code/config/turtlebot3_basic.rviz ~/ros2_ws/turtlebot3.rviz
ros2 run rviz2 rviz2 -d ~/ros2_ws/turtlebot3.rviz
```

## 环境诊断

```bash
bash "$REPO"/stage1_code/scripts/check_env.sh

# 工作空间不是 ~/ros2_ws 时，可显式传入路径
bash "$REPO"/stage1_code/scripts/check_env.sh /path/to/your_ws
```
