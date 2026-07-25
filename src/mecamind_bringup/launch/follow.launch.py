"""第五课（10.3.5）视觉跟随 + 速度仲裁的统一启动入口。

【这个文件是干什么的】
一条命令拉起"仿真后端 + 视觉链路 + 跟随控制 + 速度仲裁 + 安全门"：

    ros2 launch mecamind_bringup follow.launch.py backend:=lite
    ros2 launch mecamind_bringup follow.launch.py backend:=gazebo source:=camera
    # Gazebo 房间内红色圆柱往返 + 车载相机 + HSV（推荐课堂演示）：
    ros2 launch mecamind_bringup follow.launch.py \\
      backend:=gazebo source:=camera infer_backend:=hsv moving_target:=true

【速度链路（本课核心，与第三课的区别是多了仲裁器）】

    遥控 ───────────── /cmd_vel_teleop ──┐
    跟随控制器 ─────── /cmd_vel_follow ──┤（estop > teleop > follow > nav）
    Nav2（本课不启） ─ /cmd_vel_nav_in ──┴─► cmd_vel_arbiter
                                              │ /cmd_vel_nav
                                              ▼
                                          safety_gate ──► /cmd_vel ──► 后端

仲裁器决定"听谁的"，安全门决定"听了之后撞不撞墙"，两层各管一事。

【视觉链路】
detector(synthetic/camera/video) -> /mecamind/detections
  -> json_bridge -> /mecamind/raw_detections
  -> perception_filter -> /mecamind/perception_event
  -> vision_follow_controller -> /cmd_vel_follow

【上课操作提示】
- 约 10s 后自动打开跟随；也可手动：
  ros2 topic pub --once /mecamind/follow_enable std_msgs/msg/Bool '{data: true}'
- Gazebo 简化演示：红柱只在客厅南北短直线慢速往返，先保证跟得住。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("mecamind_bringup")
    gazebo_share = get_package_share_directory("mecamind_gazebo")

    backend = LaunchConfiguration("backend")
    use_gui = LaunchConfiguration("use_gui")
    source = LaunchConfiguration("source")
    infer_backend = LaunchConfiguration("infer_backend")
    model_path = LaunchConfiguration("model_path")
    video_path = LaunchConfiguration("video_path")
    target_label = LaunchConfiguration("target_label")
    moving_target = LaunchConfiguration("moving_target")

    is_gazebo = IfCondition(PythonExpression(["'", backend, "' == 'gazebo'"]))
    is_lite = IfCondition(PythonExpression(["'", backend, "' == 'lite'"]))
    # 可动红柱只在 Gazebo 世界有意义；lite 无对应实体
    use_moving_target = IfCondition(
        PythonExpression(
            ["'", backend, "' == 'gazebo' and '", moving_target, "' == 'true'"]
        )
    )

    gazebo_backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{gazebo_share}/launch/gazebo_sim.launch.py"),
        launch_arguments={
            "use_gui": use_gui,
            # 跟随专用世界：小车直接出生在客厅起点后方朝南
            "world": f"{gazebo_share}/worlds/three_room_house_follow.sdf",
        }.items(),
        condition=is_gazebo,
    )
    lite_backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{bringup_share}/launch/lite_sim.launch.py"),
        condition=is_lite,
    )

    # ---- 视觉链路（复用第四课的三个节点）----
    detector = Node(
        package="mecamind_perception",
        executable="mecamind_detector",
        name="mecamind_detector",
        output="screen",
        parameters=[
            {"source": source},
            {"backend": infer_backend},
            {"model_path": model_path},
            {"video_path": video_path},
            {"target_label": target_label},
        ],
    )
    json_bridge = Node(
        package="mecamind_perception",
        executable="mecamind_detection_json_bridge",
        name="mecamind_detection_json_bridge",
        output="screen",
    )
    perception_filter = Node(
        package="mecamind_tools",
        executable="mecamind_perception_filter",
        name="mecamind_perception_filter",
        output="screen",
        parameters=[{"target_label": target_label}],
    )

    # ---- Gazebo：客厅南侧空地绕圈（不朝车头对撞）----
    follow_target_mover = Node(
        package="mecamind_tools",
        executable="mecamind_follow_target_mover",
        name="mecamind_follow_target_mover",
        output="screen",
        condition=use_moving_target,
        parameters=[
            {"use_sim_time": True},
            {"world_name": "three_room_house"},
            {"model_name": "follow_target"},
            # 圆心(-2.3,-1.2) r≈1.1 六边形；比原 0.75 圈更大，仍避开小车
            {
                "waypoints_xy": (
                    "[-2.30,-0.10, -3.25,-0.65, -2.85,-2.15, "
                    "-2.30,-2.30, -1.75,-2.15, -1.35,-0.65]"
                )
            },
            {"z": 0.38},
            {"speed_mps": 0.12},
            {"rate_hz": 4.0},
            {"startup_delay_sec": 6.0},
            {"wait_for_follow_enable": True},
            {"follow_start_delay_sec": 2.0},
            {"ping_pong": False},
            {"closed_loop": True},
            {"reposition_robot": True},
            {"robot_x": -2.0},
            {"robot_y": 1.95},
            {"robot_yaw": -1.5708},
        ],
    )

    # ---- 跟随控制器 ----
    follow_controller = Node(
        package="mecamind_tools",
        executable="mecamind_vision_follow_controller",
        name="mecamind_vision_follow_controller",
        output="screen",
        parameters=[
            {"output_cmd_topic": "/cmd_vel_follow"},
            # 比 0.28 再近约 1/3（画面更宽），过近仍大力后退防穿模
            {"desired_width": 0.36},
            {"max_linear": 0.24},
            {"max_angular": 1.0},
            {"kp_linear": 1.15},
            {"kp_angular": 2.2},
            {"search_on_loss": True},
            {"search_angular": 0.45},
        ],
    )

    # ~10s 后自动打开跟随（连发，避免丢消息）
    auto_enable_follow = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-lc",
                    "for i in 1 2 3 4 5; do ros2 topic pub --once /mecamind/follow_enable std_msgs/msg/Bool '{data: true}'; sleep 0.8; done",
                ],
                output="screen",
            )
        ],
    )

    # ---- 速度仲裁器：estop > teleop > follow > nav ----
    arbiter = Node(
        package="mecamind_perception",
        executable="mecamind_cmd_vel_arbiter",
        name="mecamind_cmd_vel_arbiter",
        output="screen",
        parameters=[
            {"output_topic": "/cmd_vel_nav"},
            {"teleop_topic": "/cmd_vel_teleop"},
            {"follow_topic": "/cmd_vel_follow"},
            {"nav_topic": "/cmd_vel_nav_in"},
        ],
    )

    # ---- 安全门：仲裁之后仍要过避障关（与第三课同一节点、同一接线）----
    safety_gate = Node(
        package="mecamind_tools",
        executable="mecamind_safety_gate",
        name="mecamind_follow_safety_gate",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"scan_topic": "/scan"},
            {"input_cmd_topic": "/cmd_vel_nav"},
            {"output_cmd_topic": "/cmd_vel"},
            {"scan_timeout_sec": 2.0},
            {"cmd_timeout_sec": 1.5},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="lite",
                description="Simulation backend: gazebo or lite.",
            ),
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument(
                "source",
                default_value="synthetic",
                description="Detector image source: camera / video / synthetic.",
            ),
            DeclareLaunchArgument(
                "infer_backend",
                default_value="auto",
                description="Inference backend: auto / ultralytics / onnx / hsv.",
            ),
            DeclareLaunchArgument("model_path", default_value="yolov8n.pt"),
            DeclareLaunchArgument("video_path", default_value=""),
            DeclareLaunchArgument("target_label", default_value="person"),
            DeclareLaunchArgument(
                "moving_target",
                default_value="true",
                description="Gazebo only: drive the red follow_target cylinder left/right.",
            ),
            gazebo_backend,
            lite_backend,
            detector,
            json_bridge,
            perception_filter,
            follow_target_mover,
            follow_controller,
            arbiter,
            safety_gate,
            auto_enable_follow,
        ]
    )
