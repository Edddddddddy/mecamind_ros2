from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import time
from typing import Iterable, Tuple

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


@dataclass(frozen=True)
class SectorDistances:
    front: float
    left: float
    right: float
    rear: float


@dataclass(frozen=True)
class SafetyDecision:
    level: str
    reason: str
    scale_x: float
    scale_y: float
    allow_forward: bool
    allow_left: bool
    allow_right: bool
    sectors: SectorDistances

    def to_json(self) -> str:
        data = asdict(self)
        data["sectors"] = asdict(self.sectors)
        return json.dumps(data, ensure_ascii=False)


def _angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _sector_min(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    center: float,
    half_width: float,
    range_min: float = 0.02,
    range_max: float = 20.0,
) -> float:
    values = []
    for index, value in enumerate(ranges):
        if not math.isfinite(value) or value < range_min or value > range_max:
            continue
        angle = angle_min + index * angle_increment
        if abs(_angle_diff(angle, center)) <= half_width:
            values.append(float(value))
    return min(values) if values else float("inf")


def analyze_scan(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    front_warn_dist: float = 0.45,
    front_stop_dist: float = 0.25,
    side_warn_dist: float = 0.28,
    side_stop_dist: float = 0.16,
    sector_half_width: float = 0.45,
) -> SafetyDecision:
    sectors = SectorDistances(
        front=_sector_min(ranges, angle_min, angle_increment, 0.0, sector_half_width),
        left=_sector_min(ranges, angle_min, angle_increment, math.pi / 2.0, sector_half_width),
        right=_sector_min(ranges, angle_min, angle_increment, -math.pi / 2.0, sector_half_width),
        rear=_sector_min(ranges, angle_min, angle_increment, math.pi, sector_half_width),
    )

    allow_forward = sectors.front >= front_stop_dist
    allow_left = sectors.left >= side_stop_dist
    allow_right = sectors.right >= side_stop_dist
    level = "clear"
    reason = "all_sectors_clear"
    scale_x = 1.0
    scale_y = 1.0

    if not allow_forward:
        level = "stop"
        reason = "front_stop"
        scale_x = 0.0
    elif sectors.front < front_warn_dist:
        level = "warn"
        reason = "front_warn"
        span = max(0.001, front_warn_dist - front_stop_dist)
        scale_x = max(0.25, min(1.0, (sectors.front - front_stop_dist) / span))

    if not allow_left or not allow_right:
        level = "stop" if level == "clear" else level
        reason = "side_stop" if reason == "all_sectors_clear" else reason
        scale_y = 0.0
    elif sectors.left < side_warn_dist or sectors.right < side_warn_dist:
        if level == "clear":
            level = "warn"
            reason = "side_warn"
        scale_y = 0.5

    return SafetyDecision(
        level=level,
        reason=reason,
        scale_x=scale_x,
        scale_y=scale_y,
        allow_forward=allow_forward,
        allow_left=allow_left,
        allow_right=allow_right,
        sectors=sectors,
    )


def gate_twist(command: Twist, decision: SafetyDecision) -> Twist:
    gated = Twist()
    gated.linear.x = command.linear.x
    gated.linear.y = command.linear.y
    gated.angular.z = command.angular.z

    if command.linear.x > 0.0:
        gated.linear.x = command.linear.x * decision.scale_x
    if command.linear.y > 0.0 and not decision.allow_left:
        gated.linear.y = 0.0
    elif command.linear.y < 0.0 and not decision.allow_right:
        gated.linear.y = 0.0
    else:
        gated.linear.y = command.linear.y * decision.scale_y
    return gated


def command_timed_out(
    last_command_time: float,
    now: float,
    timeout_sec: float,
    command_active: bool,
) -> bool:
    return command_active and timeout_sec > 0.0 and now - last_command_time > timeout_sec


class SafetyGateNode(Node):
    def __init__(self) -> None:
        super().__init__("mecamind_safety_gate")
        self.declare_parameter("scan_topic", "/scan_raw")
        self.declare_parameter("input_cmd_topic", "/controller/cmd_vel_nav")
        self.declare_parameter("output_cmd_topic", "/controller/cmd_vel")
        self.declare_parameter("state_topic", "/mecamind/safety_state")
        self.declare_parameter("front_warn_dist", 0.45)
        self.declare_parameter("front_stop_dist", 0.25)
        self.declare_parameter("side_warn_dist", 0.28)
        self.declare_parameter("side_stop_dist", 0.16)
        self.declare_parameter("scan_timeout_sec", 0.60)
        self.declare_parameter("cmd_timeout_sec", 0.60)

        self._last_scan: LaserScan | None = None
        self._last_scan_wall_time = 0.0
        self._last_cmd_wall_time = time.monotonic()
        self._command_active = False

        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._scan_cb,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("input_cmd_topic").value),
            self._cmd_cb,
            10,
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("output_cmd_topic").value),
            10,
        )
        self.state_pub = self.create_publisher(
            String,
            str(self.get_parameter("state_topic").value),
            10,
        )
        self.watchdog_timer = self.create_timer(0.05, self._command_watchdog)
        self.get_logger().info("MecaMind safety gate ready")

    def _scan_cb(self, msg: LaserScan) -> None:
        self._last_scan = msg
        self._last_scan_wall_time = time.monotonic()

    def _decision(self) -> SafetyDecision:
        timeout = float(self.get_parameter("scan_timeout_sec").value)
        if self._last_scan is None or time.monotonic() - self._last_scan_wall_time > timeout:
            return SafetyDecision(
                level="stop",
                reason="scan_stale",
                scale_x=0.0,
                scale_y=0.0,
                allow_forward=False,
                allow_left=False,
                allow_right=False,
                sectors=SectorDistances(float("inf"), float("inf"), float("inf"), float("inf")),
            )
        scan = self._last_scan
        return analyze_scan(
            scan.ranges,
            float(scan.angle_min),
            float(scan.angle_increment),
            front_warn_dist=float(self.get_parameter("front_warn_dist").value),
            front_stop_dist=float(self.get_parameter("front_stop_dist").value),
            side_warn_dist=float(self.get_parameter("side_warn_dist").value),
            side_stop_dist=float(self.get_parameter("side_stop_dist").value),
        )

    def _cmd_cb(self, msg: Twist) -> None:
        self._last_cmd_wall_time = time.monotonic()
        decision = self._decision()
        gated = gate_twist(msg, decision)
        self._command_active = any(
            abs(value) > 1.0e-6
            for value in (gated.linear.x, gated.linear.y, gated.angular.z)
        )
        self.cmd_pub.publish(gated)
        state = String()
        state.data = decision.to_json()
        self.state_pub.publish(state)

    def _command_watchdog(self) -> None:
        timeout = float(self.get_parameter("cmd_timeout_sec").value)
        if not command_timed_out(
            self._last_cmd_wall_time,
            time.monotonic(),
            timeout,
            self._command_active,
        ):
            return
        self._command_active = False
        self.cmd_pub.publish(Twist())
        decision = self._decision()
        timeout_state = SafetyDecision(
            level="stop",
            reason="cmd_timeout",
            scale_x=0.0,
            scale_y=0.0,
            allow_forward=False,
            allow_left=False,
            allow_right=False,
            sectors=decision.sectors,
        )
        self.state_pub.publish(String(data=timeout_state.to_json()))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        finally:
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
