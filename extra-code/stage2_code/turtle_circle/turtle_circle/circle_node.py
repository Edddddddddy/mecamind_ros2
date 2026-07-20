# circle_node.py —— 话题发布入门示例：让海龟画圆
# 用定时器周期性地向 /turtle1/cmd_vel 发布固定的 Twist 速度：
# 线速度 + 角速度恒定不变，运动轨迹自然就是一个圆。
# 运行（需先启动 turtlesim）：ros2 run turtle_circle circle_node
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CircleNode(Node):
    """Publish Twist commands that make turtle1 move in a circle."""

    def __init__(self):
        super().__init__("circle_node")
        # 发布者：消息类型 Twist，话题 /turtle1/cmd_vel，队列长度 10
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        # 定时器：每 0.1 秒（10Hz）调用一次发布回调
        self.timer_ = self.create_timer(0.1, self.publish_velocity)
        self.get_logger().info("Circle node started: linear.x=1.0, angular.z=0.5")

    def publish_velocity(self):
        # 恒定的前进速度 + 转向速度 => 圆周运动
        # （圆半径 = linear.x / angular.z = 2.0）
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.5
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CircleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前发一条全零 Twist，让海龟停下而不是保持最后速度
        if rclpy.ok():
            try:
                stop_msg = Twist()
                node.publisher_.publish(stop_msg)
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
