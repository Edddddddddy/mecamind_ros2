from __future__ import annotations

from collections import deque
import math
from pathlib import Path
import random

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .nav_utils import quaternion_from_yaw
from .sim_core import SimPose, SimTwist, MecaMindSimModel
from .world_geometry import load_world_geometry


def _default_world_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory

        return str(Path(get_package_share_directory("mecamind_tools")) / "worlds" / "three_room_house.world")
    except Exception:
        return "worlds/three_room_house.world"


def _to_time(seconds: float):
    sec = int(seconds)
    nanosec = int((seconds - sec) * 1_000_000_000)
    from builtin_interfaces.msg import Time

    msg = Time()
    msg.sec = sec
    msg.nanosec = nanosec
    return msg


class MecaMindSimulatorNode(Node):
    def __init__(self) -> None:
        super().__init__("mecamind_simulator")
        self.declare_parameter("world_path", _default_world_path())
        self.declare_parameter("update_rate_hz", 20.0)
        self.declare_parameter("initial_x", -3.2)
        self.declare_parameter("initial_y", -2.4)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("robot_radius", 0.22)
        self.declare_parameter("max_linear_x", 0.45)
        self.declare_parameter("max_linear_y", 0.35)
        self.declare_parameter("max_angular_z", 1.5)
        self.declare_parameter("max_accel_x", 0.9)
        self.declare_parameter("max_accel_y", 0.9)
        self.declare_parameter("max_accel_z", 1.8)
        self.declare_parameter("sensor_offset_x", 0.16)
        self.declare_parameter("sensor_offset_y", 0.0)
        self.declare_parameter("lidar_range_max", 8.0)
        self.declare_parameter("lidar_beams", 360)
        self.declare_parameter("lidar_angle_min", -math.pi)
        self.declare_parameter("lidar_angle_max", math.pi)
        self.declare_parameter("publish_clock", True)
        self.declare_parameter("random_seed", 8)
        self.declare_parameter("lidar_noise_std", 0.0)
        self.declare_parameter("lidar_dropout_prob", 0.0)
        self.declare_parameter("odom_xy_noise_std", 0.0)
        self.declare_parameter("odom_yaw_noise_std", 0.0)
        self.declare_parameter("cmd_latency_sec", 0.0)

        world_path = str(self.get_parameter("world_path").value)
        geometry = load_world_geometry(world_path)
        self.model = MecaMindSimModel(
            geometry=geometry,
            initial_pose=SimPose(
                float(self.get_parameter("initial_x").value),
                float(self.get_parameter("initial_y").value),
                float(self.get_parameter("initial_yaw").value),
            ),
            robot_radius=float(self.get_parameter("robot_radius").value),
            max_linear_x=float(self.get_parameter("max_linear_x").value),
            max_linear_y=float(self.get_parameter("max_linear_y").value),
            max_angular_z=float(self.get_parameter("max_angular_z").value),
            max_accel_x=float(self.get_parameter("max_accel_x").value),
            max_accel_y=float(self.get_parameter("max_accel_y").value),
            max_accel_z=float(self.get_parameter("max_accel_z").value),
            sensor_offset_x=float(self.get_parameter("sensor_offset_x").value),
            sensor_offset_y=float(self.get_parameter("sensor_offset_y").value),
            lidar_range_max=float(self.get_parameter("lidar_range_max").value),
            lidar_beams=int(self.get_parameter("lidar_beams").value),
            lidar_angle_min=float(self.get_parameter("lidar_angle_min").value),
            lidar_angle_max=float(self.get_parameter("lidar_angle_max").value),
        )

        self.update_rate = float(self.get_parameter("update_rate_hz").value)
        self.dt = 1.0 / self.update_rate
        self.publish_clock = bool(self.get_parameter("publish_clock").value)
        self._last_command = SimTwist()
        self._last_debug_log = 0.0
        self._rng = random.Random(int(self.get_parameter("random_seed").value))
        self._pending_commands: deque[tuple[float, SimTwist]] = deque()
        self._reported_pose = self.model.pose

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan_raw", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data_raw", 10)
        self.cmd_sub = self.create_subscription(Twist, "/controller/cmd_vel", self._cmd_cb, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_transforms()
        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"MecaMind simulator ready: {geometry.name} "
            f"({len(geometry.boxes)} boxes, {self.model.lidar_beams} beams)"
        )

    def _publish_static_transforms(self) -> None:
        static_transforms = []

        def make_transform(parent: str, child: str, x: float, y: float, z: float, yaw: float = 0.0):
            msg = TransformStamped()
            msg.header.frame_id = parent
            msg.child_frame_id = child
            msg.transform.translation.x = x
            msg.transform.translation.y = y
            msg.transform.translation.z = z
            msg.transform.rotation = quaternion_from_yaw(yaw)
            return msg

        static_transforms.append(make_transform("base_footprint", "base_link", 0.0, 0.0, 0.08))
        static_transforms.append(make_transform("base_link", "lidar_link", 0.16, 0.0, 0.16))
        static_transforms.append(make_transform("base_link", "imu_link", 0.0, 0.0, 0.10))
        self.static_tf_broadcaster.sendTransform(static_transforms)

    def _cmd_cb(self, msg: Twist) -> None:
        command = SimTwist(msg.linear.x, msg.linear.y, msg.angular.z)
        self._last_command = command
        latency = max(0.0, float(self.get_parameter("cmd_latency_sec").value))
        if latency <= 1.0e-6:
            self.model.set_command(command.vx, command.vy, command.wz)
            return
        self._pending_commands.append((self.model.sim_time + latency, command))

    def _tick(self) -> None:
        self._apply_delayed_commands()
        self.model.step(self.dt)
        self._reported_pose = self._make_reported_pose()
        if self.model.sim_time - self._last_debug_log >= 5.0:
            self._last_debug_log = self.model.sim_time
            self.get_logger().info(
                "motion: pose=(%.2f, %.2f, %.2f) "
                "cmd=(%.2f, %.2f, %.2f) vel=(%.2f, %.2f, %.2f)"
                % (
                    self.model.pose.x,
                    self.model.pose.y,
                    self.model.pose.yaw,
                    self._last_command.vx,
                    self._last_command.vy,
                    self._last_command.wz,
                    self.model.velocity.vx,
                    self.model.velocity.vy,
                    self.model.velocity.wz,
                )
            )
        stamp = _to_time(self.model.sim_time)
        self._publish_clock(stamp)
        self._publish_odom(stamp)
        self._publish_scan(stamp)
        self._publish_imu(stamp)
        self._publish_tf(stamp)

    def _apply_delayed_commands(self) -> None:
        while self._pending_commands and self._pending_commands[0][0] <= self.model.sim_time:
            _, command = self._pending_commands.popleft()
            self.model.set_command(command.vx, command.vy, command.wz)

    def _make_reported_pose(self) -> SimPose:
        xy_std = max(0.0, float(self.get_parameter("odom_xy_noise_std").value))
        yaw_std = max(0.0, float(self.get_parameter("odom_yaw_noise_std").value))
        if xy_std <= 1.0e-9 and yaw_std <= 1.0e-9:
            return self.model.pose
        return SimPose(
            self.model.pose.x + self._rng.gauss(0.0, xy_std),
            self.model.pose.y + self._rng.gauss(0.0, xy_std),
            self.model.pose.yaw + self._rng.gauss(0.0, yaw_std),
        )

    def _publish_clock(self, stamp) -> None:
        if not self.publish_clock:
            return
        msg = Clock()
        msg.clock = stamp
        self.clock_pub.publish(msg)

    def _publish_tf(self, stamp) -> None:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = self._reported_pose.x
        transform.transform.translation.y = self._reported_pose.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = quaternion_from_yaw(self._reported_pose.yaw)
        self.tf_broadcaster.sendTransform(transform)

    def _publish_odom(self, stamp) -> None:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"
        msg.pose.pose.position.x = self._reported_pose.x
        msg.pose.pose.position.y = self._reported_pose.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = quaternion_from_yaw(self._reported_pose.yaw)
        msg.twist.twist.linear.x = self.model.velocity.vx
        msg.twist.twist.linear.y = self.model.velocity.vy
        msg.twist.twist.angular.z = self.model.velocity.wz
        self.odom_pub.publish(msg)

    def _publish_scan(self, stamp) -> None:
        ranges = self.model.scan_ranges()
        noise_std = max(0.0, float(self.get_parameter("lidar_noise_std").value))
        dropout_prob = max(0.0, min(1.0, float(self.get_parameter("lidar_dropout_prob").value)))
        if noise_std > 1.0e-9 or dropout_prob > 1.0e-9:
            noisy_ranges = []
            for value in ranges:
                if self._rng.random() < dropout_prob:
                    noisy_ranges.append(float("inf"))
                    continue
                noisy = value + self._rng.gauss(0.0, noise_std)
                noisy_ranges.append(max(0.12, min(self.model.lidar_range_max, noisy)))
            ranges = noisy_ranges
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = "lidar_link"
        msg.angle_min = self.model.lidar_angle_min
        msg.angle_max = self.model.lidar_angle_max
        msg.angle_increment = self.model.scan_increment()
        msg.time_increment = self.dt / max(1, self.model.lidar_beams)
        msg.scan_time = self.dt
        msg.range_min = 0.12
        msg.range_max = self.model.lidar_range_max
        msg.ranges = [float(value) for value in ranges]
        self.scan_pub.publish(msg)

    def _publish_imu(self, stamp) -> None:
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = "imu_link"
        msg.orientation = quaternion_from_yaw(self._reported_pose.yaw)
        msg.angular_velocity.z = self.model.velocity.wz
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 0.0
        self.imu_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MecaMindSimulatorNode()
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
