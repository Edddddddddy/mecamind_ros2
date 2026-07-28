# MecaMind ROS2 具身智能麦克纳姆机器人项目

独立 ROS 2 Jazzy 课程工程，支持：

- Gazebo Harmonic 麦克纳姆轮仿真（课堂主线，UI/headless）。
- 内置轻量 2D 仿真（低配置与自动测试兜底）。
- LiDAR、IMU、RGB Camera、里程计和 TF。
- 后续六次课逐步接入 SLAM、Nav2、YOLO、视觉跟随、语音和大模型。

> 路径约定：下文假设项目位于 `~/mecamind_ros2`。若你放在其他目录，请自行替换。

## 获取代码与课程 Tag

克隆仓库后，按当节课切换到对应 tag，保证与课堂资料版本一致：

```bash
git clone git@gitlab.0voice.com:2604_vip/10.3-mecamind_ros2.git ~/mecamind_ros2
cd ~/mecamind_ros2
git fetch --tags
git checkout lesson1.0   # 第一节课；其后按表切换
```

| 课程 | Tag | 说明 |
| --- | --- | --- |
| 第一节课 | `lesson1.0` | 项目总览与仿真环境搭建 |
| 第二节课 | `lesson2.0` | SLAM 建图与地图管理 |
| 第三节课 | `lesson3.0` | Nav2 导航系统与路径规划 |
| 第四节课 | `lesson4.0` | YOLO 目标检测与 ROS 2 节点接入 |
| 第五节课 | `lesson5.0`（待打 tag） | 多线程推理与视觉跟随控制 |

第四节课示例：

```bash
git fetch --tags
git checkout lesson4.0
```

说明：

- `git fetch --tags` 成功时可能没有输出，属正常；用 `git tag -l` 确认本地已有对应 tag。
- 查看远端 tag：`git ls-remote --tags origin`
- 第五、六节讲义已就绪；冻结版本时再打 `lesson5.0` / `lesson6.0`。
- 若要回到最新开发分支：`git checkout main && git pull`

讲义（仓库发布 PDF；本地若有 Markdown 源稿同名即可）：

- 第一节课：`docs/10.3.1_项目总览与仿真环境搭建.pdf`
- 第二节课：`docs/10.3.2_SLAM建图与地图管理.pdf`
- 第三节课：`docs/10.3.3_Nav2导航系统与路径规划.pdf`
- 第四节课：`docs/10.3.4_YOLO目标检测与ROS2节点接入.pdf`
- 第五节课：`docs/10.3.5_多线程推理与视觉跟随控制.pdf`
- 第五节课源码走读：`docs/10.3.5_工程版源码走读.pdf`
- 第五节课白板：`docs/10.3.5_多线程推理与视觉跟随控制.excalidraw`（预览图同名 `.png`）
- 第六节课：`docs/10.3.6_语音交互大模型与项目验收.pdf`（源稿同名 `.md`）

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

第四课若要跑 YOLO / ONNX（可选，不装也能用 HSV 兜底完成课堂主线）：

```bash
bash setup/install_dependencies.sh --with-ml
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

## 第三次课：Nav2 导航

统一导航入口（默认 Gazebo + RViz，加载打包好的三居室地图并自动激活 Nav2）：

```bash
ros2 launch mecamind_bringup navigation.launch.py
```

轻量仿真兜底：

```bash
ros2 launch mecamind_bringup navigation.launch.py backend:=lite use_gui:=false
```

命名目标导航与多点巡航（另开终端）：

```bash
# 去客厅（支持中文别名，如 "卧室"）
ros2 topic pub --once /mecamind/task_command std_msgs/msg/String \
  '{data: "{\"intent\": \"navigate\", \"target\": \"living_room\"}"}'

# 多点巡航 / 运动中取消
ros2 topic pub --once /mecamind/task_command std_msgs/msg/String '{data: "{\"intent\": \"patrol\"}"}'
ros2 topic pub --once /mecamind/task_command std_msgs/msg/String '{data: "{\"intent\": \"stop\"}"}'

# 任务状态心跳
ros2 topic echo /mecamind/mission_state
```

使用自己第二课建的地图：`navigation.launch.py map_file:=$HOME/mecamind_ros2/maps/mecamind_cp2_gazebo_map.yaml`。

## CP3 验收

```bash
bash scripts/test_cp3.sh                                # 轻量后端，约 3 分钟
MECAMIND_CP3_BACKEND=gazebo bash scripts/test_cp3.sh    # Gazebo headless，10~20 分钟
```

自动完成"启动导航栈 → 三个命名目标 → 运动中取消"全流程，成功标志：`MECAMIND_CP3_OK`，报告写入 `maps/mecamind_cp3_<backend>_nav_report.json`。

## 第四次课：YOLO 目标检测

先编译感知相关包（若尚未编译）：

```bash
cd ~/mecamind_ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  mecamind_interfaces mecamind_perception mecamind_tools mecamind_bringup
source install/setup.bash
```

**零依赖主线**（默认合成画面 + HSV 兜底，不装 PyTorch / ultralytics 也能看见绿框）：

```bash
ros2 launch mecamind_bringup perception.launch.py
```

看检测结果（另开终端）：

```bash
ros2 topic echo /mecamind/detections --once
ros2 topic echo /mecamind/perception_metrics --once
ros2 topic echo /mecamind/perception_event --once
```

**可选：安装 YOLO / ONNX 依赖**（想跑真神经网络时）：

```bash
./setup/install_dependencies.sh --with-ml
```

等价于：

```bash
pip install --user --break-system-packages "setuptools<80" ultralytics onnx onnxruntime
```

Ubuntu 24.04 系统 Python 受 PEP 668 保护，直接 `pip install xxx` 常会报 `externally-managed-environment`，请用上面的课程命令。装好后自检：

```bash
python3 -c "import ultralytics, onnx, onnxruntime; print('ML ok')"
ls -lh models/yolov8n.pt models/yolov8n.onnx
```

权重与演示素材已放在仓库：`models/`、`assets/yolo_demo.mp4`、`assets/bus.jpg`。若本地没有 `.onnx`，可自行导出：

```bash
python3 scripts/export_yolo_onnx.py
```

真实 YOLO（视频源；**不要**用 synthetic 考 YOLO——红方块不是 COCO 类别）：

```bash
# ultralytics + .pt
ros2 launch mecamind_bringup perception.launch.py \
  source:=video \
  video_path:=$HOME/mecamind_ros2/assets/yolo_demo.mp4 \
  backend:=ultralytics \
  model_path:=$HOME/mecamind_ros2/models/yolov8n.pt

# onnxruntime + .onnx
ros2 launch mecamind_bringup perception.launch.py \
  source:=video \
  video_path:=$HOME/mecamind_ros2/assets/yolo_demo.mp4 \
  backend:=onnx \
  model_path:=$HOME/mecamind_ros2/models/yolov8n.onnx
```

有 Gazebo / 真机相机时：`source:=camera camera_topic:=/camera/image_raw`。无图形界面可加 `with_ui:=false`。

## CP4 验收

```bash
bash scripts/test_cp4.sh
# 默认 synthetic + auto → 落到 hsv，零依赖可过

MECAMIND_CP4_SOURCE=video \
MECAMIND_CP4_VIDEO=assets/yolo_demo.mp4 \
MECAMIND_CP4_BACKEND=onnx \
MECAMIND_CP4_MODEL=models/yolov8n.onnx \
bash scripts/test_cp4.sh

MECAMIND_CP4_SOURCE=video \
MECAMIND_CP4_VIDEO=assets/yolo_demo.mp4 \
MECAMIND_CP4_BACKEND=ultralytics \
bash scripts/test_cp4.sh
```

成功标志：`MECAMIND_CP4_OK`，报告写入 `maps/mecamind_cp4_report.json`。跑验收前建议先 `bash scripts/cleanup_ros2.sh`。

## 第五次课：多线程推理与视觉跟随

### 课堂主演示（一条命令）

Gazebo + 车载相机 + HSV + 客厅南侧红柱绕圈：

```bash
cd ~/mecamind_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash scripts/cleanup_ros2.sh
ros2 launch mecamind_bringup follow.launch.py \
  backend:=gazebo use_gui:=true \
  source:=camera infer_backend:=hsv \
  moving_target:=true target_label:=person
```

约 10 秒后自动打开跟随；也可手动：

```bash
ros2 topic pub --once /mecamind/follow_enable std_msgs/msg/Bool '{data: true}'
```

可选检测窗口：`python3 scripts/show_detection_image.py`  
结束：`Ctrl+C`，再 `bash scripts/cleanup_ros2.sh`。

### 首次 / 改源码后先编译

```bash
cd ~/mecamind_ros2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select \
  mecamind_interfaces mecamind_perception mecamind_tools \
  mecamind_bringup mecamind_gazebo --symlink-install
source install/setup.bash
```

### 零依赖冒烟（lite + 合成红块 + HSV）

```bash
bash scripts/cleanup_ros2.sh
ros2 launch mecamind_bringup follow.launch.py \
  backend:=lite source:=synthetic infer_backend:=hsv
```

## CP5 验收

```bash
bash scripts/cleanup_ros2.sh
bash scripts/test_cp5.sh
# 默认 lite + synthetic + hsv，零依赖可过

MECAMIND_CP5_BACKEND=gazebo bash scripts/test_cp5.sh
```

成功标志：`MECAMIND_CP5_OK`，报告写入 `maps/mecamind_cp5_report.json`。

说明：三室户世界已放大到约 **11.7m × 8.85m**（约 1.5 倍），跟随控制器为完整 **PID**。
若你仍使用旧地图做导航，请用第二节课流程**重新建图**；命名目标/巡航点已按新尺寸更新。

## 第六次课：语音产品核心（唤醒 / 流式 ASR / TTS 播放）

### 密钥（不要提交 git）

复制模板并填入 DashScope API Key：

```bash
cp src/mecamind_tools/config/mecamind_aliyun.example.yaml \
   src/mecamind_tools/config/mecamind_aliyun.local.yaml
# 编辑 local 文件，写入 aliyun.api_key
# 也可：export DASHSCOPE_API_KEY=sk-...
```

`*.local.yaml` 已在 `.gitignore` 中，**禁止**把真实密钥写进 README / launch / example。

云端 ASR/TTS 依赖：

```bash
pip install --user --break-system-packages -U dashscope
```

### 启动连续听（默认）

```bash
cd ~/mecamind_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mecamind_tools mecamind_aliyun_voice.launch.py
```

默认能力：

- 关键词唤醒：`小度小度`（参数 `wake_words`；内置 `小度/小杜/小渡/小肚` 同音与标点误转写兜底）
- 能量门限连续听 + 流式 ASR（`paraformer-realtime-v2`）
- 任务解析（默认阿里云 LLM，失败自动降级规则）
- TTS 合成 + 本机自动播放（`ffplay` / `paplay` / `aplay`）
- 模糊指令会先请你「确认 / 取消」

按键说话兜底（无唤醒）：

```bash
ros2 launch mecamind_tools mecamind_aliyun_voice.launch.py listen_mode:=ptt
ros2 service call /mecamind/record_voice std_srvs/srv/Trigger
```

无麦克风时可用文字路径：

```bash
ros2 topic pub --once /mecamind/voice_command std_msgs/msg/String "{data: '去卧室'}"
ros2 topic pub --once /mecamind/voice_command std_msgs/msg/String "{data: '确认'}"
ros2 topic pub --once /mecamind/voice_command std_msgs/msg/String "{data: '取消'}"
```

### 语音核心验收

```bash
bash scripts/test_voice_core.sh
```

成功标志：`MECAMIND_VOICE_CORE_OK`（文本确认闭环 + 密钥可读；不强制真麦）。

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
