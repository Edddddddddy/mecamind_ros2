from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_share = get_package_share_directory("mecamind_gazebo")
    description_share = get_package_share_directory("mecamind_description")
    ros_gz_share = get_package_share_directory("ros_gz_sim")

    world = LaunchConfiguration("world")
    use_gui = LaunchConfiguration("use_gui")
    display = LaunchConfiguration("display")
    world_default = f"{gazebo_share}/worlds/three_room_house.sdf"
    model_path = f"{gazebo_share}/models"
    display_default = (
        ":0"
        if Path("/tmp/.X11-unix/X0").exists()
        else os.environ.get("DISPLAY", "")
    )
    robot_description = Command(
        ["xacro ", f"{description_share}/urdf/mecamind_mecanum.urdf.xacro"]
    )

    gz_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{ros_gz_share}/launch/gz_sim.launch.py"),
        launch_arguments={"gz_args": ["-r -v 2 ", world]}.items(),
        condition=IfCondition(use_gui),
    )
    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{ros_gz_share}/launch/gz_sim.launch.py"),
        launch_arguments={"gz_args": ["-r -s -v 2 ", world]}.items(),
        condition=UnlessCondition(use_gui),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("world", default_value=world_default),
            DeclareLaunchArgument(
                "display",
                default_value=display_default,
                description="X11 display used by Gazebo GUI; WSLg normally uses :0.",
            ),
            SetEnvironmentVariable("DISPLAY", display),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                [model_path, ":", EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
            ),
            gz_with_gui,
            gz_headless,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"robot_description": robot_description, "use_sim_time": True}
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="mecamind_gz_bridge",
                output="screen",
                parameters=[
                    {
                        "config_file": f"{gazebo_share}/config/bridge.yaml",
                        "use_sim_time": True,
                    }
                ],
            ),
        ]
    )
