#!/usr/bin/env python3
# ============================================================
# py_first_pkg 包里的 OOP 风格节点
# 演示概念：继承 Node、Timer 回调、优雅退出的收尾写法
# 运行方式（先 colcon build 并 source）：
#   ros2 run py_first_pkg py_oop
# ============================================================
import rclpy
from rclpy.node import Node


# OOP 写法：继承 Node，状态和回调封装在同一个类中
class OopNode(Node):
    def __init__(self):
        super().__init__("py_oop")  # 节点名为 py_oop
        self.counter_ = 0
        self.get_logger().info("OOP node started")
        # 每 1.0 秒触发一次回调（由 spin 事件循环调度）
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # 定时回调：打印递增的计数
        self.get_logger().info("Hello " + str(self.counter_))
        self.counter_ += 1


def main(args=None):
    rclpy.init(args=args)
    node = OopNode()
    # try/finally 保证 Ctrl+C 退出时也能正确清理资源：
    # spin 阻塞运行，KeyboardInterrupt 被捕获后走 finally
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 收尾：销毁节点再关闭 rclpy，顺序不能反
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
