from __future__ import annotations

import json
import math
import os
import subprocess
import time
from pathlib import Path
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

from .nav_utils import load_yaml


def write_occupancy_grid_pair(save_base: Path, msg: OccupancyGrid) -> None:
    """Write nav2-compatible PGM/YAML from an OccupancyGrid (map_saver Y-flip)."""
    width = int(msg.info.width)
    height = int(msg.info.height)
    if width <= 0 or height <= 0 or len(msg.data) < width * height:
        raise ValueError("OccupancyGrid is empty or incomplete")

    pixels = bytearray(width * height)
    for row in range(height):
        src_row = height - 1 - row  # map_saver: image row 0 = world y_max
        for col in range(width):
            value = int(msg.data[src_row * width + col])
            if value < 0:
                pixel = 205
            elif value >= 65:
                pixel = 0
            elif value <= 25:
                pixel = 254
            else:
                # Keep mid values visually between free/occupied.
                pixel = int(round(255 - value * 255 / 100.0))
            pixels[row * width + col] = pixel

    pgm_path = save_base.with_suffix(".pgm")
    yaml_path = save_base.with_suffix(".yaml")
    pgm_path.write_bytes(
        f"P5\n{width} {height}\n255\n".encode("ascii") + bytes(pixels)
    )
    meta = {
        "image": pgm_path.name,
        "mode": "trinary",
        "resolution": float(msg.info.resolution),
        "origin": [
            float(msg.info.origin.position.x),
            float(msg.info.origin.position.y),
            0.0,
        ],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    yaml_path.write_text(yaml.dump(meta, default_flow_style=False, sort_keys=False), encoding="utf-8")


def _default_route_file() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory

        return str(Path(get_package_share_directory("mecamind_tools")) / "config" / "mecamind_mapping_route.yaml")
    except Exception:
        return "config/mecamind_mapping_route.yaml"


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _angle_diff(a: float, b: float) -> float:
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


def _as_bool(value: object) -> bool:
    if isinstance(value, str):
        return value.lower() in ("1", "true", "yes", "on")
    return bool(value)


def build_route_report(
    route_file: str,
    phase: str,
    route_size: int,
    waypoint_results: List[dict],
    robot_pose: dict,
    auto_map_path: str,
    manual_map_path: str,
    auto_map_saved: bool,
    manual_map_saved: bool,
) -> dict:
    """Build the machine-readable evidence emitted by a mapping route run."""
    reached = sum(item.get("status") == "reached" for item in waypoint_results)
    skipped = sum(item.get("status") == "skipped" for item in waypoint_results)
    return {
        "schema_version": 1,
        "phase": phase,
        "route_file": route_file,
        "route_size": route_size,
        "processed_waypoints": len(waypoint_results),
        "reached_waypoints": reached,
        "skipped_waypoints": skipped,
        "all_waypoints_reached": route_size > 0 and reached == route_size,
        "auto_map_path": auto_map_path,
        "manual_map_path": manual_map_path,
        "auto_map_saved": auto_map_saved,
        "manual_map_saved": manual_map_saved,
        "robot_pose": robot_pose,
        "waypoints": waypoint_results,
    }


class MecaMindMappingRouteDriver(Node):
    """Drive a safe waypoint route through the three-room house for SLAM mapping."""

    def __init__(self) -> None:
        super().__init__("mecamind_mapping_route_driver")
        self.declare_parameter("route_file", _default_route_file())
        self.declare_parameter("startup_delay_sec", 8.0)
        self.declare_parameter("goal_tolerance", 0.18)
        self.declare_parameter("yaw_tolerance", 0.20)
        self.declare_parameter("hold_sec", 0.4)
        self.declare_parameter("linear_gain", 0.7)
        self.declare_parameter("angular_gain", 1.4)
        self.declare_parameter("max_linear_x", 0.28)
        self.declare_parameter("max_linear_y", 0.22)
        self.declare_parameter("max_angular_z", 1.0)
        self.declare_parameter("obstacle_stop_dist", 0.28)
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("auto_save_map", True)
        self.declare_parameter("checkpoint_save_interval_sec", 90.0)
        self.declare_parameter("checkpoint_keep_snapshots", False)
        self.declare_parameter("waypoint_timeout_sec", 50.0)
        self.declare_parameter("waypoint_stall_sec", 18.0)
        self.declare_parameter("waypoint_min_progress", 0.08)
        self.declare_parameter("shutdown_after_route", True)
        self.declare_parameter("manual_save_path", "")
        self.declare_parameter("route_report_path", "")
        self.declare_parameter("status_topic", "/mecamind/mapping_status")
        self.declare_parameter("map_save_path", "maps/mecamind_three_room_map")

        self.route_file = str(self.get_parameter("route_file").value)
        config = load_yaml(self.route_file)
        self.route: List[dict] = list(config.get("waypoints", []))
        if not self.route:
            raise ValueError(f"No waypoints found in route file: {self.route_file}")

        self.startup_delay_sec = float(self.get_parameter("startup_delay_sec").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        self.hold_sec = float(self.get_parameter("hold_sec").value)
        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.angular_gain = float(self.get_parameter("angular_gain").value)
        self.max_linear_x = float(self.get_parameter("max_linear_x").value)
        self.max_linear_y = float(self.get_parameter("max_linear_y").value)
        self.max_angular_z = float(self.get_parameter("max_angular_z").value)
        self.obstacle_stop_dist = float(self.get_parameter("obstacle_stop_dist").value)
        self.checkpoint_save_interval_sec = float(self.get_parameter("checkpoint_save_interval_sec").value)
        self.checkpoint_keep_snapshots = _as_bool(
            self.get_parameter("checkpoint_keep_snapshots").value
        )
        self.waypoint_timeout_sec = float(self.get_parameter("waypoint_timeout_sec").value)
        self.waypoint_stall_sec = float(self.get_parameter("waypoint_stall_sec").value)
        self.waypoint_min_progress = float(self.get_parameter("waypoint_min_progress").value)
        self.shutdown_after_route = _as_bool(self.get_parameter("shutdown_after_route").value)
        self.auto_map_path = Path(
            os.path.expanduser(str(self.get_parameter("map_save_path").value))
        ).expanduser()
        manual_save_path = str(self.get_parameter("manual_save_path").value).strip()
        self.manual_map_path = (
            Path(os.path.expanduser(manual_save_path)).expanduser()
            if manual_save_path
            else self.auto_map_path.with_name(f"{self.auto_map_path.name}_manual")
        )
        report_path = str(self.get_parameter("route_report_path").value).strip()
        self.route_report_path = (
            Path(os.path.expanduser(report_path)).expanduser()
            if report_path
            else self.auto_map_path.with_name(f"{self.auto_map_path.name}_route_report.json")
        )

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.laser_ranges: Optional[List[float]] = None
        self.scan_angle_min = -math.pi
        self.scan_angle_increment = 0.0

        # 使用仿真时钟做路线超时；Gazebo RTF < 1 时墙钟会误判 no_progress。
        self._boot_time = -1.0
        self._running = False
        self._route_idx = 0
        self._hold_until = 0.0
        self._map_saved = False
        self._manual_map_saved = False
        self._route_finished = False
        self._phase = "waiting_for_sensors"
        self._waypoint_results: List[dict] = []
        self._last_status_publish = 0.0
        self._last_checkpoint_save = -1.0
        self._checkpoint_index = 0
        self._active_route_idx = -1
        self._waypoint_started_at = -1.0
        self._last_waypoint_progress = -1.0
        self._best_waypoint_dist = float("inf")

        self.odom_sub = self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value), self._scan_cb, 10
        )
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._latest_map: Optional[OccupancyGrid] = None
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self._map_cb, map_qos
        )
        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_topic").value), 10)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            status_qos,
        )
        self.save_map_service = self.create_service(Trigger, "~/save_map", self._save_map_cb)
        self.status_service = self.create_service(Trigger, "~/status", self._status_cb)
        self.timer = self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f"Mapping route driver ready: {len(self.route)} waypoints from {self.route_file}"
        )
        self._publish_status(force=True)

    def _odom_cb(self, msg: Odometry) -> None:
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _scan_cb(self, msg: LaserScan) -> None:
        self.laser_ranges = list(msg.ranges)
        self.scan_angle_min = float(msg.angle_min)
        self.scan_angle_increment = float(msg.angle_increment)

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def _front_distance(self) -> float:
        if not self.laser_ranges:
            return float("inf")
        increment = abs(self.scan_angle_increment)
        if increment < 1e-9:
            increment = (2.0 * math.pi) / len(self.laser_ranges)
        values = []
        for index, value in enumerate(self.laser_ranges):
            angle = self.scan_angle_min + index * increment
            delta = _angle_diff(angle, 0.0)
            if abs(delta) <= max(0.10, (len(self.laser_ranges) // 24) * increment):
                if 0.02 < value < 12.0:
                    values.append(value)
        clean = values
        return min(clean) if clean else float("inf")

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _start_if_ready(self, now: float) -> None:
        if self._running:
            return
        if self._map_saved:
            return
        if self.laser_ranges is None:
            return
        if self._boot_time < 0.0:
            if now <= 0.0:
                return
            self._boot_time = now
            self._last_checkpoint_save = now
            self._waypoint_started_at = now
            self._last_waypoint_progress = now
        if now - self._boot_time < self.startup_delay_sec:
            return
        self._running = True
        self._phase = "automatic_mapping"
        self.get_logger().info("Starting mapping route traversal")
        self._publish_status(force=True)

    def _tick(self) -> None:
        if self._route_finished:
            self._publish_status()
            return
        now = self._now_sec()
        self._publish_status()
        self._start_if_ready(now)
        if not self._running:
            return

        self._try_checkpoint_save(now)

        if self._route_idx >= len(self.route):
            self._finish_route()
            return

        if now < self._hold_until:
            self.cmd_pub.publish(Twist())
            return

        target = self.route[self._route_idx]
        tx = float(target["x"])
        ty = float(target["y"])
        name = str(target.get("name", f"wp_{self._route_idx}"))

        dx = tx - self.robot_x
        dy = ty - self.robot_y
        dist = math.hypot(dx, dy)
        if self._active_route_idx != self._route_idx:
            self._active_route_idx = self._route_idx
            self._waypoint_started_at = now
            self._last_waypoint_progress = now
            self._best_waypoint_dist = dist
        elif dist + self.waypoint_min_progress < self._best_waypoint_dist:
            self._best_waypoint_dist = dist
            self._last_waypoint_progress = now

        target_tolerance = float(target.get("tolerance", self.goal_tolerance))
        if dist <= target_tolerance:
            target_yaw = target.get("yaw")
            if target_yaw is not None:
                yaw_error = _angle_diff(float(target_yaw), self.robot_yaw)
                if abs(yaw_error) > self.yaw_tolerance:
                    twist = Twist()
                    twist.angular.z = _clamp(yaw_error * self.angular_gain, self.max_angular_z)
                    self.cmd_pub.publish(twist)
                    return
            self.get_logger().info(f"Reached {name} at ({tx:.2f}, {ty:.2f})")
            self._record_waypoint(name, "reached", "goal_tolerance", dist, now)
            self._route_idx += 1
            self._active_route_idx = -1
            self._hold_until = now + float(target.get("dwell_sec", self.hold_sec))
            self.cmd_pub.publish(Twist())
            return

        skip_reason = self._skip_reason(now)
        if skip_reason:
            if skip_reason == "timeout":
                self.get_logger().warn(
                    f"Skipping {name}: timeout after {self.waypoint_timeout_sec:.0f}s, dist={dist:.2f}m"
                )
            else:
                self.get_logger().warn(
                    f"Skipping {name}: no progress for {self.waypoint_stall_sec:.0f}s, dist={dist:.2f}m"
                )
            self._record_waypoint(name, "skipped", skip_reason, dist, now)
            self._route_idx += 1
            self._active_route_idx = -1
            self._hold_until = now + self.hold_sec
            self.cmd_pub.publish(Twist())
            return

        body_x = math.cos(self.robot_yaw) * dx + math.sin(self.robot_yaw) * dy
        body_y = -math.sin(self.robot_yaw) * dx + math.cos(self.robot_yaw) * dy
        heading = math.atan2(dy, dx)
        yaw_error = _angle_diff(heading, self.robot_yaw)

        twist = Twist()
        twist.linear.x = _clamp(body_x * self.linear_gain, self.max_linear_x)
        twist.linear.y = _clamp(body_y * self.linear_gain, self.max_linear_y)
        twist.angular.z = _clamp(yaw_error * self.angular_gain, self.max_angular_z)

        front = self._front_distance()
        if front < self.obstacle_stop_dist:
            twist.linear.x = min(0.0, twist.linear.x)
            twist.linear.y = min(0.0, twist.linear.y)

        self.cmd_pub.publish(twist)

    def _skip_reason(self, now: float) -> str:
        if self.waypoint_timeout_sec > 0.0 and now - self._waypoint_started_at > self.waypoint_timeout_sec:
            return "timeout"
        if self.waypoint_stall_sec > 0.0 and now - self._last_waypoint_progress > self.waypoint_stall_sec:
            return "no_progress"
        return ""

    def _record_waypoint(self, name: str, status: str, reason: str, dist: float, now: float) -> None:
        self._waypoint_results.append(
            {
                "index": self._route_idx,
                "name": name,
                "status": status,
                "reason": reason,
                "remaining_distance": round(dist, 3),
                "elapsed_sec": round(now - self._waypoint_started_at, 3),
                "robot_pose": self._robot_pose(),
            }
        )
        self._publish_status(force=True)

    def _finish_route(self) -> None:
        if self._route_finished:
            self.cmd_pub.publish(Twist())
            return
        self.get_logger().info("Route complete, saving map")
        self.cmd_pub.publish(Twist())
        self._map_saved = self._try_save_map(self.auto_map_path)
        self._running = False
        self._route_finished = True
        if self._map_saved:
            self._phase = "complete" if self.shutdown_after_route else "manual_assist_ready"
            self.get_logger().info(
                "Mapping route complete"
                if self.shutdown_after_route
                else "Automatic route complete; manual assist is ready"
            )
        else:
            self._phase = "save_failed"
            self.get_logger().error("Mapping route finished, but the automatic map save failed")
        self._write_route_report()
        self._publish_status(force=True)
        if self.shutdown_after_route:
            rclpy.try_shutdown()

    def _try_checkpoint_save(self, now: float) -> None:
        if self.checkpoint_save_interval_sec <= 0.0:
            return
        if now - self._last_checkpoint_save < self.checkpoint_save_interval_sec:
            return
        self._last_checkpoint_save = now
        self.get_logger().info("Saving route mapping checkpoint")
        save_base = None
        if self.checkpoint_keep_snapshots:
            self._checkpoint_index += 1
            save_base = self.auto_map_path.with_name(
                f"{self.auto_map_path.name}_checkpoint_{self._checkpoint_index:02d}"
            )
        self._try_save_map(save_base)

    def _try_save_map(self, save_base: Optional[Path] = None) -> bool:
        if not bool(self.get_parameter("auto_save_map").value):
            return False
        if save_base is None:
            save_base = self.auto_map_path
        save_base.parent.mkdir(parents=True, exist_ok=True)

        # 优先直接写缓存的 /map（不依赖 map_saver_cli 订阅时机）。
        if self._latest_map is not None:
            try:
                write_occupancy_grid_pair(save_base, self._latest_map)
                self.get_logger().info(f"Map saved to {save_base}.pgm/.yaml")
                return True
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Direct map write failed: {exc}")

        for attempt in range(1, 4):
            try:
                completed = subprocess.run(
                    [
                        "ros2",
                        "run",
                        "nav2_map_server",
                        "map_saver_cli",
                        "-f",
                        str(save_base),
                        "--ros-args",
                        "-p",
                        "map_subscribe_transient_local:=true",
                        "-p",
                        "use_sim_time:=true",
                    ],
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=45,
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Map save failed to start (try {attempt}): {exc}")
                continue

            if completed.returncode == 0:
                self.get_logger().info(f"Map saved to {save_base}.pgm/.yaml")
                return True
            self.get_logger().warn(
                f"map_saver_cli try {attempt} failed: "
                f"{completed.stderr.strip() or completed.stdout.strip()}"
            )
            time.sleep(1.0)

        self.get_logger().error(f"Map save failed for {save_base}")
        return False

    def _robot_pose(self) -> dict:
        return {
            "x": round(self.robot_x, 3),
            "y": round(self.robot_y, 3),
            "yaw": round(self.robot_yaw, 3),
        }

    def _status_payload(self) -> dict:
        report = build_route_report(
            route_file=self.route_file,
            phase=self._phase,
            route_size=len(self.route),
            waypoint_results=self._waypoint_results,
            robot_pose=self._robot_pose(),
            auto_map_path=str(self.auto_map_path),
            manual_map_path=str(self.manual_map_path),
            auto_map_saved=self._map_saved,
            manual_map_saved=self._manual_map_saved,
        )
        report["current_waypoint_index"] = min(self._route_idx, len(self.route))
        report["manual_assist_available"] = self._phase in (
            "manual_assist_ready",
            "manual_map_saved",
        )
        return report

    def _publish_status(self, force: bool = False) -> None:
        now = time.time()
        if not force and now - self._last_status_publish < 1.0:
            return
        self._last_status_publish = now
        try:
            self.status_pub.publish(String(data=json.dumps(self._status_payload(), ensure_ascii=False)))
        except Exception as exc:  # noqa: BLE001 - ROS context can close during SIGINT
            if rclpy.ok():
                self.get_logger().warn(f"Mapping status publish failed: {exc}")

    def _write_route_report(self) -> None:
        try:
            self.route_report_path.parent.mkdir(parents=True, exist_ok=True)
            self.route_report_path.write_text(
                json.dumps(self._status_payload(), ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            self.get_logger().info(f"Route report saved to {self.route_report_path}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Route report save failed: {exc}")

    def _save_map_cb(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.cmd_pub.publish(Twist())
        save_path = self.manual_map_path if self._route_finished else self.auto_map_path
        response.success = self._try_save_map(save_path)
        if response.success:
            if self._route_finished:
                self._manual_map_saved = True
                self._phase = "manual_map_saved"
            response.message = f"Map saved to {save_path}.pgm/.yaml"
            self._write_route_report()
            self._publish_status(force=True)
        else:
            response.message = f"Map save failed for {save_path}"
        return response

    def _status_cb(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success = True
        response.message = json.dumps(self._status_payload(), ensure_ascii=False)
        return response

    def mark_interrupted(self) -> None:
        if self._route_finished:
            return
        self._phase = "interrupted"
        self._write_route_report()
        self._publish_status(force=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MecaMindMappingRouteDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.mark_interrupted()
    finally:
        try:
            if rclpy.ok():
                try:
                    node.cmd_pub.publish(Twist())
                except Exception:
                    pass
            if rclpy.ok() and not node._map_saved:
                node._try_save_map()
        finally:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
            finally:
                try:
                    rclpy.try_shutdown()
                except Exception:
                    pass


if __name__ == "__main__":
    main()
