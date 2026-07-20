# ============================================================
# Topic 通信示例：Subscriber（订阅者）节点
# 演示概念：create_subscription、消息回调触发机制
# 订阅 /chatter 话题并打印收到的每条消息，
# 配合 publisher_node 一起运行观察效果
# 运行方式：ros2 run py_pubsub subscriber_node
# ============================================================
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class SubscriberNode(Node):
    """Subscribe to /chatter and print each received String message."""

    def __init__(self):
        # 创建 Subscription：消息类型和话题名必须与 Publisher
        # 一致才能通信；末尾的 10 是 QoS 队列深度
        # （回调来不及处理时最多缓存 10 条）
        super().__init__("subscriber_node")
        self.subscription_ = self.create_subscription(
            String, "chatter", self.callback_message, 10)
        self.get_logger().info("Subscriber node started, listening on /chatter")

    def callback_message(self, msg: String):
        # 回调函数：每当 /chatter 上有新消息到达、且节点在
        # spin 中时被自动触发，msg 就是收到的消息
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    # spin 保持节点存活并处理消息回调；异常捕获保证收尾执行
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
