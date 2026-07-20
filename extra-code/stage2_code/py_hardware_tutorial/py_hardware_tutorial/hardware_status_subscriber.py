# ============================================================
# 自定义消息接口示例：Subscriber（订阅者）
# 演示概念：订阅自定义 msg 并根据字段内容做判断和告警
# 订阅 /hardware_status，打印温度并在电机未就绪时告警，
# 配合 hardware_status_publisher 一起运行
# 运行方式：
#   ros2 run py_hardware_tutorial hardware_status_subscriber
# ============================================================
import rclpy
from hardware_interfaces.msg import HardwareStatus
from rclpy.node import Node


class HardwareStatusSubscriber(Node):
    def __init__(self):
        super().__init__("hardware_status_subscriber")
        # 消息类型和话题名必须与 Publisher 一致；
        # 每收到一条消息就触发一次 callback_status
        self.subscription_ = self.create_subscription(
            HardwareStatus, "hardware_status", self.callback_status, 10)
        self.get_logger().info("HardwareStatus subscriber started")

    def callback_status(self, msg: HardwareStatus):
        # 把毫摄氏度换算成摄氏度用于显示
        temp_celsius = msg.temperature / 1000.0
        self.get_logger().info(
            f"Received: {temp_celsius:.1f} C, "
            f"motors_ready={msg.are_motors_ready}, "
            f'debug="{msg.debug_message}"')
        # 根据消息字段做业务判断：未就绪时用 warn 级别日志
        if not msg.are_motors_ready:
            self.get_logger().warn("Motors not ready!")


def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusSubscriber()
    # spin 保持节点存活并持续处理消息回调
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
