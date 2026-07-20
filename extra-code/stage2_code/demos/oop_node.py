#!/usr/bin/env python3
# ============================================================
# OOP（面向对象）风格的 ROS 2 节点示例
# 演示概念：继承 Node 类、Timer 定时器回调
# 这是独立演示脚本，直接运行：
#   python3 oop_node.py
# ============================================================
import rclpy
from rclpy.node import Node


# 推荐写法：自定义类继承 Node，把节点的状态（如计数器）
# 和行为（回调函数）都封装在类里
class MyNode(Node):
    def __init__(self):
        # 调用父类构造函数，传入节点名 py_test
        super().__init__("py_test")
        self.counter_ = 0
        self.get_logger().info("Hello world")
        # 创建 Timer：每 1.0 秒触发一次 timer_callback，
        # 回调由 spin 的事件循环调度执行
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # 每秒被 Timer 触发一次，打印递增的计数
        self.get_logger().info("Hello " + str(self.counter_))
        self.counter_ += 1


def main(args=None):
    rclpy.init(args=args)  # 初始化 ROS 2 通信
    node = MyNode()
    # spin 阻塞运行事件循环，Timer 回调在这里被反复调用
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
