#!/usr/bin/env python3
# ============================================================
# 最小 ROS 2 节点示例（非 OOP 写法）
# 演示概念：节点(Node)的创建、日志输出、spin 事件循环
# 这是一个独立脚本，不属于任何包，直接运行：
#   python3 minimal_node.py
# ============================================================
import rclpy  # rclpy 是 ROS 2 的 Python 客户端库
from rclpy.node import Node


def main(args=None):
    # 初始化 rclpy：建立与 ROS 2 底层通信(DDS)的连接，
    # 任何 ROS 2 操作之前都必须先调用它
    rclpy.init(args=args)
    # 创建一个名为 my_first_node 的节点
    # （节点是 ROS 2 程序的基本单元）
    node = Node("my_first_node")
    node.get_logger().info("Hello ROS 2!")
    # spin 让节点"转起来"：进入事件循环，阻塞在这里等待并
    # 处理回调（定时器、消息等），直到按 Ctrl+C 退出
    rclpy.spin(node)
    # 退出前的收尾：先销毁节点释放资源，再关闭 rclpy
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
