# 文件说明：最简可视化 launch 文件（无 GUI 滑块窗口）。
# 演示概念：OpaqueFunction——在 launch 运行时执行 Python 函数，
# 从而能用 .perform(context) 取出参数的字符串值再做路径拼接。
# 运行：ros2 launch my_robot_description minimal_display.launch.py
#       可选 model:=diff_drive.urdf（urdf 目录下的文件名）
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


# OpaqueFunction 在运行时调用本函数，context 里存着参数实际值
def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("my_robot_description")
    # perform(context)：把 LaunchConfiguration 占位符解析成字符串
    model = LaunchConfiguration("model").perform(context)
    xacro_path = os.path.join(pkg_share, "urdf", model)
    rviz_config = os.path.join(pkg_share, "rviz", "my_robot.rviz")

    # 用 xacro 库把模型文件展开成 URDF 字符串
    robot_description = xacro.process_file(xacro_path).toxml()

    # robot_state_publisher：把 link 位姿以 TF 广播出去
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # 无 GUI 版关节状态发布器：发布默认值 0，保证 TF 树完整
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # RViz2 可视化
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return [robot_state_publisher_node, joint_state_publisher_node, rviz_node]


def generate_launch_description():
    # 声明 model 参数：只填文件名，目录固定为本包 urdf/
    model_arg = DeclareLaunchArgument(
        "model",
        default_value="my_robot.urdf.xacro",
        description="Xacro file name under the urdf directory.",
    )

    # OpaqueFunction 把节点创建推迟到运行时的 launch_setup 里
    return LaunchDescription([model_arg, OpaqueFunction(function=launch_setup)])
