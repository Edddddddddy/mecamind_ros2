# dual_robots.launch.py —— namespace（命名空间）演示
# 同一个 turtlesim 节点启动两份，靠不同 namespace 隔离：
# 话题分别变成 /robot1/turtle1/... 和 /robot2/turtle1/...，
# 节点名和话题都不冲突。键盘遥控只接到 robot1 上。
# 运行：ros2 launch my_robot_bringup dual_robots.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 两个节点 package/executable/name 完全相同，
    # 仅 namespace 不同，即可在同一系统中共存
    robot1 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim",
        namespace="robot1",
    )

    robot2 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim",
        namespace="robot2",
    )

    # 遥控节点也放进 robot1 命名空间，
    # 它发布的 cmd_vel 就只会控制 robot1 的海龟
    teleop1 = Node(
        package="turtlesim",
        executable="turtle_teleop_key",
        name="teleop",
        namespace="robot1",
        output="screen",
        emulate_tty=True,
        prefix="bash -lc 'exec </dev/tty; exec \"$@\"' bash",
    )

    return LaunchDescription([robot1, robot2, teleop1])
