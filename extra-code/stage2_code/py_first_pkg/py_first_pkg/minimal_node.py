#!/usr/bin/env python3
# ============================================================
# py_first_pkg 包里的最小节点（非 OOP 写法）
# 演示概念：ROS 2 程序的最小骨架 init -> Node -> spin
# 运行方式（先 colcon build 并 source）：
#   ros2 run py_first_pkg py_minimal
# ============================================================
import rclpy
from rclpy.node import Node


def main(args=None):
    # 初始化 rclpy，是所有 ROS 2 Python 程序的第一步
    rclpy.init(args=args)
    # 直接实例化 Node 类，节点名为 py_minimal
    node = Node("py_minimal")
    node.get_logger().info("Hello from py_first_pkg!")
    # spin 进入事件循环并阻塞，保持节点存活直到 Ctrl+C
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
