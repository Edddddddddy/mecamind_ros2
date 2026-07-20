# Stage 2 配套源码

本目录是第二阶段课件的配套源码。每个子目录都是可以放入 ROS 2 工作空间 `src/` 下构建的参考包，包名与课件中的命令保持一致。所有源码已添加面向初学者的中文注释。

## 课件与包的对应关系

| 课件 | 包 | 内容 |
|---|---|---|
| 2.1 ROS2 节点基础 | `demos`、`py_first_pkg`、`cpp_first_pkg` | 最小节点、OOP 节点、Python/C++ 建包 |
| 2.2 话题通信 | `py_pubsub`、`cpp_pubsub`、`turtle_circle` | Publisher、Subscriber、Turtlesim 话题实验 |
| 2.3 服务通信 | `py_srv`、`cpp_srv`、`turtle_spawn` | Service 服务端/客户端、Turtlesim 生成海龟实验 |
| 2.4 动作通信 | `count_until_interfaces`、`py_action_tutorial`、`cpp_action_tutorial` | CountUntil / MoveToPosition / TimerDelay 三个 Action |
| 2.5 自定义接口 | `hardware_interfaces`、`py_hardware_tutorial`、`cpp_hardware_tutorial` | 自定义 HardwareStatus msg 与 ComputeRectangleArea srv |
| 2.6 Launch 与参数 | `my_robot_bringup`、`my_py_pkg`、`my_cpp_pkg` | Launch 文件、YAML 参数、Python/C++ 参数节点 |
| 2.7 Turtlesim 通信实战 | `my_robot_interfaces`、`turtlesim_catch_them_all`、`my_robot_bringup` | P1 项目：多节点抓海龟游戏 |

## 构建示例

```bash
REPO=/mnt/f/ros2_ai   # 改成你的课程仓库实际路径（如克隆在 WSL 内则形如 ~/ros2_ai）

mkdir -p ~/ros2_ws/src
cp -r "$REPO"/stage2_code/{py_first_pkg,cpp_first_pkg,py_pubsub,cpp_pubsub,turtle_circle,py_srv,cpp_srv,turtle_spawn,count_until_interfaces,py_action_tutorial,cpp_action_tutorial,hardware_interfaces,py_hardware_tutorial,cpp_hardware_tutorial,my_py_pkg,my_cpp_pkg,my_robot_interfaces,turtlesim_catch_them_all,my_robot_bringup} ~/ros2_ws/src/

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

说明：`demos/` 目录是两个独立演示脚本（`python3` 直接运行），不是 colcon 包，无需复制到工作空间。

Turtlesim 相关包还需要安装：

```bash
sudo apt install ros-jazzy-turtlesim
```

 