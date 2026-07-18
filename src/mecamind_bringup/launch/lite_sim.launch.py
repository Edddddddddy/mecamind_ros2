from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    tools_share = get_package_share_directory("mecamind_tools")
    world = LaunchConfiguration("world")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=f"{tools_share}/worlds/three_room_house.world",
            ),
            DeclareLaunchArgument("initial_x", default_value="-3.2"),
            DeclareLaunchArgument("initial_y", default_value="-2.4"),
            DeclareLaunchArgument("initial_yaw", default_value="0.0"),
            Node(
                package="mecamind_tools",
                executable="mecamind_simulator_node",
                name="mecamind_lite_simulator",
                output="screen",
                remappings=[
                    ("/scan_raw", "/scan"),
                    ("/imu/data_raw", "/imu/data"),
                    ("/controller/cmd_vel", "/cmd_vel"),
                ],
                parameters=[
                    {"world_path": world},
                    {"initial_x": ParameterValue(initial_x, value_type=float)},
                    {"initial_y": ParameterValue(initial_y, value_type=float)},
                    {"initial_yaw": ParameterValue(initial_yaw, value_type=float)},
                ],
            ),
        ]
    )
