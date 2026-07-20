# Stage 2 Source Code

本目录是第二阶段课件的配套源码。每个子目录都是可以放入 ROS 2 工作空间 `src/` 下构建的参考包，包名尽量与课件命令保持一致。

## Package Map

| Courseware | Packages | Purpose |
|---|---|---|
| 2.1 ROS2 节点基础 | `demos`, `py_first_pkg`, `cpp_first_pkg` | 最小节点、OOP 节点、Python/C++ package |
| 2.2 话题通信 | `py_pubsub`, `cpp_pubsub`, `turtle_circle` | Publisher、Subscriber、Turtlesim topic lab |
| 2.3 服务通信 | `py_srv`, `cpp_srv`, `turtle_spawn` | Service server/client、Turtlesim spawn lab |
| 2.4 动作通信 | `count_until_interfaces`, `py_action_tutorial`, `cpp_action_tutorial` | CountUntil / MoveToPosition / TimerDelay actions |
| 2.5 自定义接口 | `hardware_interfaces`, `py_hardware_tutorial`, `cpp_hardware_tutorial` | HardwareStatus msg and ComputeRectangleArea srv |
| 2.6 Launch 与参数 | `my_robot_bringup` plus related nodes | Launch, YAML parameters, one-command bringup |
| 2.7 Turtlesim 通信实战 | `my_robot_interfaces`, `turtlesim_catch_them_all`, `my_robot_bringup` | P1 final multi-node Turtlesim game |

## Build Example

```bash
mkdir -p ~/ros2_ws/src
cp -r REPO/stage2_code/{py_pubsub,cpp_pubsub,py_srv,cpp_srv,count_until_interfaces,py_action_tutorial,cpp_action_tutorial,hardware_interfaces,py_hardware_tutorial,cpp_hardware_tutorial,my_robot_interfaces,turtlesim_catch_them_all,my_robot_bringup} ~/ros2_ws/src/

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

把 `REPO` 替换为本仓库路径。Turtlesim 相关包还需要安装：

```bash
sudo apt install ros-jazzy-turtlesim ros-jazzy-ros2launch
```

## Self Test

本仓库提供 Windows 可运行的静态检查脚本：

```powershell
powershell -ExecutionPolicy Bypass -File self_test/stage2/run_static_checks.ps1
```

真实 ROS 2 运行验证建议在 Ubuntu 24.04 + ROS 2 Jazzy 环境中执行，截图清单见 `self_test/stage2/STAGE2_SELF_TEST_REPORT.md`。

