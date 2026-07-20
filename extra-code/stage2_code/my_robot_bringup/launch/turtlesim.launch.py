# turtlesim.launch.py —— 最简单的 Launch 文件示例
# 启动一个 turtlesim 节点并直接在 Launch 里设置节点参数，
# 其中背景红色分量通过 Launch 参数 bg_r 可在命令行覆盖。
# 运行：ros2 launch my_robot_bringup turtlesim.launch.py
# 改背景色：... turtlesim.launch.py bg_r:=200
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明 Launch 参数 bg_r，命令行不传时用默认值 69
    bg_r_arg = DeclareLaunchArgument("bg_r", default_value="69")

    # parameters 直接传字典设置节点参数：
    # background_r 引用 Launch 参数（启动时才求值），
    # 另外两个分量写成固定值
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim",
        parameters=[{
            "background_r": LaunchConfiguration("bg_r"),
            "background_g": 204,
            "background_b": 239,
        }],
    )

    return LaunchDescription([
        bg_r_arg,
        turtlesim_node,
    ])
