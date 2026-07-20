# 文件说明：显示 my_robot 模型的标准 launch 文件。
# 演示概念：DeclareLaunchArgument、IfCondition/UnlessCondition、
# 在 launch 阶段用 xacro Python API 展开模型文件。
# 启动 robot_state_publisher + joint_state_publisher(_gui) + RViz2。
# 运行：ros2 launch my_robot_description display.launch.py
# 可选参数：use_gui:=false use_rviz:=false
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


# ros2 launch 会自动调用这个函数，返回要启动的动作列表
def generate_launch_description():
    # 找到本包安装后的 share 目录（urdf/rviz 等资源都装在这里）
    pkg_share = get_package_share_directory("my_robot_description")

    # 声明 launch 参数：命令行可用 use_gui:=false 覆盖默认值
    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        description="Start joint_state_publisher_gui for interactive joint control.",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz2 with the saved robot display configuration.",
    )

    # 在 launch 阶段直接用 xacro 库把 .xacro 展开成 URDF 字符串
    # （另一种做法见 display_xacro.launch.py：运行时用 Command 展开）
    xacro_path = os.path.join(pkg_share, "urdf", "my_robot.urdf.xacro")
    robot_description = xacro.process_file(xacro_path).toxml()
    rviz_config = os.path.join(pkg_share, "rviz", "my_robot.rviz")

    # robot_state_publisher：读取 robot_description 参数，
    # 订阅 /joint_states，把每个 link 的位姿以 TF 形式广播出去
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": False,
            }
        ],
    )

    # 带滑块窗口的关节状态发布器：拖动滑块即可转动轮子
    # condition 使它只在 use_gui:=true 时启动
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("use_gui")),
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # 无界面版本：use_gui:=false 时改用它发布默认关节状态，
    # 保证 TF 树完整（否则轮子 link 会缺少变换）
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=UnlessCondition(LaunchConfiguration("use_gui")),
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # RViz2 可视化工具，-d 指定预先保存好的显示配置文件
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # 把参数声明和所有 Node 汇总成一个 LaunchDescription 返回
    return LaunchDescription(
        [
            use_gui_arg,
            use_rviz_arg,
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            joint_state_publisher_node,
            rviz_node,
        ]
    )
