# ============================================================
# 自定义消息接口示例：Publisher（发布者）
# 演示概念：使用自定义 msg（hardware_interfaces/HardwareStatus）
# 每秒发布一次模拟的硬件状态（温度、电机是否就绪）
# 运行方式：
#   ros2 run py_hardware_tutorial hardware_status_publisher
# ============================================================
import rclpy
# 自定义消息类型，由 hardware_interfaces 包定义并生成
from hardware_interfaces.msg import HardwareStatus
from rclpy.node import Node


class HardwareStatusPublisher(Node):
    def __init__(self):
        super().__init__("hardware_status_publisher")
        # 自定义消息的用法与标准消息完全一样，
        # 末尾 10 是 QoS 队列深度
        self.publisher_ = self.create_publisher(HardwareStatus, "hardware_status", 10)
        # 模拟温度，单位是毫摄氏度（42000 = 42.0 C）
        self.temperature_ = 42000
        self.timer_ = self.create_timer(1.0, self.publish_status)
        self.get_logger().info("HardwareStatus publisher started")

    def publish_status(self):
        # 逐个填写自定义消息的三个字段再发布；
        # 温度每次上涨，超过 80000（80 C）后电机标记为未就绪
        msg = HardwareStatus()
        msg.temperature = self.temperature_
        msg.are_motors_ready = self.temperature_ < 80000
        msg.debug_message = f"Temperature: {self.temperature_ / 1000.0:.1f} C"
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Publishing: temp={msg.temperature}, '
            f'ready={msg.are_motors_ready}, msg="{msg.debug_message}"')
        self.temperature_ += 500


def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusPublisher()
    # spin 阻塞运行事件循环，Ctrl+C 后走 finally 收尾
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
