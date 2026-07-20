# 文件说明：用真实 TF 系统做点坐标变换的演示节点。
# 演示概念：TF Buffer/TransformListener、lookup_transform、
# tf_buffer.transform() 直接变换带 frame 信息的消息。
# 运行前需先启动 TurtleBot3 的 robot_state_publisher
# （提供 base_link -> base_scan 的 TF），然后执行：
#   ros2 run tf_math_demo tf_point_transform_demo
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

# 这个 import 看似没用到，实际是注册"PointStamped 如何被
# TF 变换"的转换函数，缺了它 tf_buffer.transform 会报错
import tf2_geometry_msgs  # noqa: F401 - registers PointStamped transform support


class TfPointTransformDemo(Node):
    def __init__(self) -> None:
        super().__init__("tf_point_transform_demo")
        # Buffer 缓存收到的所有 TF 变换；TransformListener
        # 在后台订阅 /tf 和 /tf_static 话题并写入 Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def transform_once(self, timeout_sec: float = 8.0) -> bool:
        # PointStamped = 点坐标 + header（所在 frame 和时间戳）
        # frame_id 声明这个点是在 base_scan 坐标系下表示的；
        # stamp 取 Time()（零时刻）表示"用最新可用的变换"
        point = PointStamped()
        point.header.frame_id = "base_scan"
        point.header.stamp = Time().to_msg()
        point.point.x = 2.0
        point.point.y = 0.5
        point.point.z = 0.0

        deadline = time.monotonic() + timeout_sec
        last_error = None

        # 节点刚启动时 Buffer 是空的，所以循环重试：
        # spin_once 处理一批 TF 消息，然后尝试变换，
        # 直到成功或超过 timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                # transform()：把 PointStamped 直接变换到目标
                # frame（base_link），数学细节由 TF 库完成
                point_base = self.tf_buffer.transform(
                    point,
                    "base_link",
                    timeout=Duration(seconds=0.2),
                )
                # lookup_transform：查询两个 frame 之间的原始
                # 变换（平移+旋转），用于打印对照
                transform = self.tf_buffer.lookup_transform(
                    "base_link",
                    "base_scan",
                    Time(),
                    timeout=Duration(seconds=0.2),
                )
                translation = transform.transform.translation
                self.get_logger().info(
                    "TF base_link -> base_scan translation: "
                    f"x={translation.x:.3f}, y={translation.y:.3f}, z={translation.z:.3f}"
                )
                p = point_base.point
                self.get_logger().info(
                    "Point base_scan(2.00, 0.50, 0.00) -> "
                    f"base_link({p.x:.2f}, {p.y:.2f}, {p.z:.2f})"
                )
                return True
            except TransformException as exc:
                # TF 还没就绪（变换尚未收到），记下错误继续等
                last_error = exc

        # 超时仍失败：通常是 robot_state_publisher 没启动
        self.get_logger().error(
            "Failed to transform base_scan point to base_link. "
            "Start TurtleBot3 robot_state_publisher first. "
            f"Last error: {last_error}"
        )
        return False


def main() -> None:
    # 一次性任务节点：不用 spin 常驻，执行完变换就退出
    rclpy.init()
    node = TfPointTransformDemo()
    try:
        ok = node.transform_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    # 失败时以非零退出码结束，方便脚本判断结果
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
