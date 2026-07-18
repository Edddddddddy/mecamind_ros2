"""Unified SLAM mapping entry for lesson 10.3.2.

Both simulation backends expose the same interface (/scan, /odom, /cmd_vel,
TF odom->base_footprint), so SLAM Toolbox and the mapping tools below do not
care which backend is running:

- backend:=gazebo  Gazebo Harmonic (main classroom path, supports use_gui)
- backend:=lite    built-in lightweight 2D simulator (fallback / CI path)

Manual mapping (default): drive with teleop, then save the map yourself.
Semi-automatic mapping (auto_route:=true): a route driver traverses the
three-room house, saves the map and writes a route report, then shuts down.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("mecamind_bringup")
    gazebo_share = get_package_share_directory("mecamind_gazebo")
    tools_share = get_package_share_directory("mecamind_tools")

    backend = LaunchConfiguration("backend")
    use_gui = LaunchConfiguration("use_gui")
    with_rviz = LaunchConfiguration("with_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    auto_route = LaunchConfiguration("auto_route")
    map_save_path = LaunchConfiguration("map_save_path")
    route_report_path = LaunchConfiguration("route_report_path")
    route_file = LaunchConfiguration("route_file")
    shutdown_on_route_complete = LaunchConfiguration("shutdown_on_route_complete")

    is_gazebo = IfCondition(PythonExpression(["'", backend, "' == 'gazebo'"]))
    is_lite = IfCondition(PythonExpression(["'", backend, "' == 'lite'"]))
    auto_route_and_shutdown = IfCondition(
        PythonExpression(
            [
                "'",
                auto_route,
                "'.lower() in ('true', '1') and '",
                shutdown_on_route_complete,
                "'.lower() in ('true', '1')",
            ]
        )
    )

    gazebo_backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{gazebo_share}/launch/gazebo_sim.launch.py"),
        launch_arguments={"use_gui": use_gui}.items(),
        condition=is_gazebo,
    )
    # Spawn the lite robot at the origin so it matches the Gazebo spawn pose
    # and the shared mapping route file.
    lite_backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{bringup_share}/launch/lite_sim.launch.py"),
        launch_arguments={
            "initial_x": "0.0",
            "initial_y": "0.0",
            "initial_yaw": "0.0",
        }.items(),
        condition=is_lite,
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            f"{tools_share}/config/mecamind_slam_toolbox_params.yaml",
            {
                "use_sim_time": True,
                "autostart": True,
                "use_lifecycle_manager": False,
                "scan_topic": "/scan",
            },
        ],
    )

    slam_activator = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="mecamind_tools",
                executable="mecamind_lifecycle_activator",
                output="screen",
                parameters=[
                    {"node_names": ["slam_toolbox"]},
                    {"publish_initial_pose": False},
                ],
            )
        ],
    )

    # Teleop or the route driver publish to /cmd_vel_nav; the gate forwards a
    # collision-checked command to /cmd_vel, which both backends consume.
    safety_gate = Node(
        package="mecamind_tools",
        executable="mecamind_safety_gate",
        name="mecamind_mapping_safety_gate",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"scan_topic": "/scan"},
            {"input_cmd_topic": "/cmd_vel_nav"},
            {"output_cmd_topic": "/cmd_vel"},
            # Gazebo RTF 偏低时激光墙钟间隔变长，避免误判 scan_stale 刹死。
            {"scan_timeout_sec": 2.0},
            {"cmd_timeout_sec": 1.5},
        ],
    )

    route_driver_node = Node(
        package="mecamind_tools",
        executable="mecamind_mapping_route_driver",
        name="mecamind_mapping_route_driver",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"route_file": route_file},
            {"map_save_path": map_save_path},
            {"route_report_path": route_report_path},
            {"cmd_topic": "/cmd_vel_nav"},
            {"scan_topic": "/scan"},
            {"startup_delay_sec": 12.0},
            {"waypoint_timeout_sec": 80.0},
            {"waypoint_stall_sec": 30.0},
            {
                "shutdown_after_route": ParameterValue(
                    shutdown_on_route_complete, value_type=bool
                )
            },
        ],
    )

    route_driver = TimerAction(
        period=10.0,
        actions=[route_driver_node],
        condition=IfCondition(auto_route),
    )

    shutdown_when_route_done = RegisterEventHandler(
        OnProcessExit(
            target_action=route_driver_node,
            on_exit=[EmitEvent(event=Shutdown(reason="mapping route complete"))],
        ),
        condition=auto_route_and_shutdown,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="mecamind_mapping_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(with_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="gazebo",
                description="Simulation backend: gazebo or lite.",
            ),
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("with_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=f"{tools_share}/rviz/mecamind_mapping.rviz",
            ),
            DeclareLaunchArgument(
                "auto_route",
                default_value="false",
                description="Run the semi-automatic mapping route driver.",
            ),
            DeclareLaunchArgument(
                "map_save_path",
                default_value="maps/mecamind_three_room_map",
                description="Map save prefix, relative to the launch directory.",
            ),
            DeclareLaunchArgument("route_report_path", default_value=""),
            DeclareLaunchArgument(
                "route_file",
                default_value=f"{tools_share}/config/mecamind_mapping_route.yaml",
            ),
            DeclareLaunchArgument("shutdown_on_route_complete", default_value="true"),
            gazebo_backend,
            lite_backend,
            slam_toolbox,
            slam_activator,
            safety_gate,
            route_driver,
            shutdown_when_route_done,
            rviz_node,
        ]
    )
