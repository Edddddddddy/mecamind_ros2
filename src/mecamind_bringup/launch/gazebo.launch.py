from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gazebo_share = get_package_share_directory("mecamind_gazebo")
    use_gui = LaunchConfiguration("use_gui")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_gui",
                default_value="true",
                description="Start the Gazebo graphical client when true.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    f"{gazebo_share}/launch/gazebo_sim.launch.py"
                ),
                launch_arguments={"use_gui": use_gui}.items(),
            ),
        ]
    )
