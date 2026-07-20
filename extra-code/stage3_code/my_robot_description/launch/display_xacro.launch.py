# 文件说明：用 Substitution 方式显示任意 xacro 模型的 launch 文件。
# 演示概念：Command 替换（运行时执行 `xacro <文件>` 命令展开模型）、
# ParameterValue、FindPackageShare/PathJoinSubstitution。
# 与 display.launch.py 的区别：模型路径可整体作为参数传入，
# 展开推迟到 launch 运行时，不需要在 Python 里 import xacro。
# 运行：ros2 launch my_robot_description display_xacro.launch.py
#       可选 model:=/绝对路径/xxx.urdf.xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # PathJoinSubstitution + FindPackageShare：
    # 在 launch 运行时才拼出 share 目录下的默认文件路径
    default_model = PathJoinSubstitution(
        [FindPackageShare("my_robot_description"), "urdf", "my_robot.urdf.xacro"]
    )
    default_rviz = PathJoinSubstitution(
        [FindPackageShare("my_robot_description"), "rviz", "my_robot.rviz"]
    )

    # LaunchConfiguration 是"占位符"，运行时取对应参数的实际值
    model = LaunchConfiguration("model")
    rviz_config = LaunchConfiguration("rviz_config")
    # Command 替换：launch 启动时执行 shell 命令 `xacro <model>`，
    # 把命令输出（展开后的 URDF 文本）作为参数值；
    # ParameterValue(value_type=str) 明确告诉 launch 这是字符串
    robot_description = ParameterValue(Command(["xacro ", model]), value_type=str)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=default_model,
                description="Absolute path to the xacro file to display.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="Absolute path to the RViz config file.",
            ),
            # robot_state_publisher：根据 robot_description 和
            # /joint_states 广播各 link 的 TF 变换
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            # 滑块窗口：手动发布 /joint_states 驱动可动关节
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            # RViz2，-d 加载保存好的显示配置
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
