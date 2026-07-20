# ============================================================
# Topic 通信示例：Publisher（发布者）节点
# 演示概念：create_publisher、Timer 定时发布、String 消息
# 每秒向 /chatter 话题发布一条消息，配合 subscriber_node 使用
# 运行方式：ros2 run py_pubsub publisher_node
# ============================================================
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String  # 标准字符串消息类型


class PublisherNode(Node):
    """Publish one String message to /chatter every second."""

    def __init__(self):
        super().__init__("publisher_node")
        # 创建 Publisher：参数依次是消息类型、话题名、
        # QoS 队列深度 10（网络不畅时最多缓存 10 条待发消息）
        self.publisher_ = self.create_publisher(String, "chatter", 10)
        self.counter_ = 0
        # Publisher 本身不会自动发消息，
        # 用 Timer 每 1.0 秒调用一次发布函数
        self.timer_ = self.create_timer(1.0, self.publish_message)
        self.get_logger().info("Publisher node started, publishing at 1 Hz")

    def publish_message(self):
        # 构造消息对象，填入 data 字段，再通过 publish 发出去
        msg = String()
        msg.data = f"Hello ROS 2! [{self.counter_}]"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter_ += 1


def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    # spin 阻塞运行事件循环；Ctrl+C（KeyboardInterrupt）或
    # 外部关闭（ExternalShutdownException）都会走 finally 收尾
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # rclpy 可能已被外部关闭，先检查再 shutdown 避免报错
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
