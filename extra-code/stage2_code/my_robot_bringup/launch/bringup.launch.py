# bringup.launch.py —— 综合 Launch 示例：一条命令启动整套应用
# 演示：从 YAML 加载参数、Launch 参数（with_teleop 开关）、
# 条件启动（IfCondition）、话题 remapping。
# 运行：ros2 launch my_robot_bringup bringup.launch.py
# 关闭键盘遥控：... bringup.launch.py with_teleop:=false
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ROS 2 约定的 Launch 入口函数，返回要启动的动作列表
    # 定位本包安装目录下的参数文件（YAML 里按节点名分组）
    pkg_share = get_package_share_directory("my_robot_bringup")
    config_file = os.path.join(pkg_share, "config", "bringup.yaml")
    # LaunchConfiguration：引用名为 with_teleop 的 Launch 参数，
    # 其值在启动时才确定（见下方 DeclareLaunchArgument）
    with_teleop = LaunchConfiguration("with_teleop")

    # Node 各字段含义：
    #   package    节点所在的包名
    #   executable 可执行文件名（console_scripts 里的命令名）
    #   name       运行时节点名（可覆盖代码里的默认名）
    #   output     "screen" 表示日志直接打印到终端
    #   parameters 参数来源，可以是 YAML 文件或字典
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim",
        output="screen",
        parameters=[config_file],
    )

    # condition=IfCondition(...)：Launch 参数为 true 才启动此节点
    # prefix 把节点接到真实终端上，键盘按键才能被读取
    teleop_node = Node(
        package="turtlesim",
        executable="turtle_teleop_key",
        name="teleop",
        output="screen",
        emulate_tty=True,
        prefix="bash -lc 'exec </dev/tty; exec \"$@\"' bash",
        condition=IfCondition(with_teleop),
    )

    # remappings：话题重映射，把代码里的 /chatter
    # 在运行时改名为 /my_topic，不用改源代码
    param_publisher_node = Node(
        package="my_py_pkg",
        executable="param_publisher",
        name="param_publisher",
        remappings=[("/chatter", "/my_topic")],
        parameters=[config_file],
        output="screen",
    )

    return LaunchDescription([
        # 声明 Launch 参数：名称、默认值、说明文字
        DeclareLaunchArgument(
            "with_teleop",
            default_value="true",
            description="Start the interactive keyboard teleop node",
        ),
        turtlesim_node,
        teleop_node,
        param_publisher_node,
    ])
