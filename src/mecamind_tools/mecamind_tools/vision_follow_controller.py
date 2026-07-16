from __future__ import annotations

from dataclasses import dataclass
import json
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


@dataclass(frozen=True)
class FollowCommand:
    linear_x: float
    angular_z: float
    active: bool
    reason: str


def compute_follow_command(
    cx: float,
    target_width: float,
    desired_width: float = 0.25,
    center_deadband: float = 0.05,
    max_linear: float = 0.20,
    max_angular: float = 0.8,
) -> FollowCommand:
    error_x = cx - 0.5
    angular = 0.0 if abs(error_x) < center_deadband else -2.0 * error_x
    angular = max(-max_angular, min(max_angular, angular))

    width_error = desired_width - target_width
    linear = max(-max_linear * 0.4, min(max_linear, width_error * 0.8))
    if target_width <= 0.0:
        linear = 0.0
    return FollowCommand(linear, angular, True, "target_confirmed")


def should_publish_follow(enabled: bool, confirmed: bool) -> bool:
    return bool(enabled) and bool(confirmed)


class VisionFollowController(Node):
    def __init__(self) -> None:
        super().__init__("mecamind_vision_follow_controller")
        self.declare_parameter("input_topic", "/mecamind/perception_event")
        self.declare_parameter("output_cmd_topic", "/controller/cmd_vel_nav")
        self.declare_parameter("enable_topic", "/mecamind/follow_enable")
        self.declare_parameter("desired_width", 0.25)
        self.declare_parameter("event_timeout_sec", 0.70)
        self.declare_parameter("max_linear", 0.20)
        self.declare_parameter("max_angular", 0.8)

        self._enabled = False
        self._last_event_time = 0.0
        self.create_subscription(String, str(self.get_parameter("input_topic").value), self._event_cb, 10)
        self.create_subscription(Bool, str(self.get_parameter("enable_topic").value), self._enable_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("output_cmd_topic").value), 10)
        self.timer = self.create_timer(0.1, self._watchdog)
        self.get_logger().info("MecaMind vision follow controller ready (follow disabled by default)")

    def _enable_cb(self, msg: Bool) -> None:
        was_enabled = self._enabled
        self._enabled = bool(msg.data)
        if was_enabled and not self._enabled:
            self._last_event_time = 0.0
            self.cmd_pub.publish(Twist())
            self.get_logger().info("Vision follow disabled")
        elif not was_enabled and self._enabled:
            self.get_logger().info("Vision follow enabled")

    def _event_cb(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Invalid perception event: {exc}")
            return
        if not should_publish_follow(self._enabled, bool(event.get("confirmed", False))):
            return
        command = compute_follow_command(
            float(event.get("cx", 0.5)),
            float(event.get("width", 0.0)),
            desired_width=float(self.get_parameter("desired_width").value),
            max_linear=float(self.get_parameter("max_linear").value),
            max_angular=float(self.get_parameter("max_angular").value),
        )
        twist = Twist()
        twist.linear.x = command.linear_x
        twist.angular.z = command.angular_z
        self._last_event_time = time.monotonic()
        self.cmd_pub.publish(twist)

    def _watchdog(self) -> None:
        if not self._enabled:
            return
        if self._last_event_time <= 0.0:
            return
        if time.monotonic() - self._last_event_time <= float(self.get_parameter("event_timeout_sec").value):
            return
        self._last_event_time = 0.0
        self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionFollowController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
