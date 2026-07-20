# number_app.launch.py —— 共享参数文件的 Launch 示例
# 同时启动 Python 版和 C++ 版两个参数化发布节点，
# 演示：两个不同语言的节点可以共用同一份 YAML 参数文件。
# 运行：ros2 launch my_robot_bringup number_app.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 定位本包安装目录下的共享参数文件
    pkg_share = get_package_share_directory("my_robot_bringup")
    config_file = os.path.join(pkg_share, "config", "number_app.yaml")

    # Python 节点：来自 my_py_pkg 包；YAML 中按节点名
    # param_publisher 分组的参数会加载到它身上
    python_publisher = Node(
        package="my_py_pkg",
        executable="param_publisher",
        name="param_publisher",
        parameters=[config_file],
        output="screen",
    )

    # C++ 节点：executable 同名但来自 my_cpp_pkg 包；
    # 用 name 改成 cpp_param_publisher 以免节点名冲突
    cpp_publisher = Node(
        package="my_cpp_pkg",
        executable="param_publisher",
        name="cpp_param_publisher",
        parameters=[config_file],
        output="screen",
    )

    return LaunchDescription([python_publisher, cpp_publisher])
