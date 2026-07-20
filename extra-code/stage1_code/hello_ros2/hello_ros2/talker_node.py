#!/usr/bin/env python3
# 文件说明：Stage 1 最小发布者（Publisher）示例节点。
# 演示概念：Node（节点）、Publisher、Timer 定时回调、rclpy.spin。
# 运行方式（先 colcon build 并 source install/setup.bash）：
#   ros2 run hello_ros2 talker
# 验证方式：另开终端执行 ros2 topic echo /chatter 查看消息。
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


# 独立函数构造消息，方便单元测试（不需要启动 ROS 2 就能测）
def build_greeting_message() -> String:
    msg = String()
    msg.data = "Hello ROS 2!"
    return msg


class TalkerNode(Node):
    def __init__(self):
        # 调用父类构造函数，"talker" 是节点在 ROS 图中的名字
        super().__init__("talker")
        # 创建 Publisher：消息类型 String，话题名 "chatter"，
        # 10 是 QoS 队列深度——订阅方来不及处理时最多缓存 10 条，
        # 超出后最旧的消息会被丢弃
        self.publisher_ = self.create_publisher(String, "chatter", 10)
        # 创建 Timer：每 1.0 秒触发一次 timer_callback
        # （由 spin 的事件循环调度，不是新开线程）
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Talker node started!")

    # Timer 到期时被 spin 调用：组装消息并发布到 chatter 话题
    def timer_callback(self):
        msg = build_greeting_message()
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')


def main(args=None):
    # 初始化 rclpy（ROS 2 Python 客户端库），必须在创建节点前调用
    rclpy.init(args=args)
    node = TalkerNode()
    try:
        # spin 让节点进入事件循环：阻塞在这里等待并分发
        # Timer/订阅等回调，直到 Ctrl+C 或外部关停
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 退出前清理资源：销毁节点，再关闭 rclpy 上下文
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
