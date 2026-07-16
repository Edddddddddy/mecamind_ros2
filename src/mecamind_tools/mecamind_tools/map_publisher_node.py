from __future__ import annotations

from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .world_geometry import load_world_geometry


def _default_world_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory

        return str(Path(get_package_share_directory("mecamind_tools")) / "worlds" / "three_room_house.world")
    except Exception:
        return "worlds/three_room_house.world"


class MecaMindMapPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("mecamind_map_publisher")
        self.declare_parameter("world_path", _default_world_path())
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("padding", 0.20)
        self.declare_parameter("publish_period_sec", 5.0)

        world_path = str(self.get_parameter("world_path").value)
        self.geometry = load_world_geometry(world_path)
        self.spec = self.geometry.build_occupancy_grid(
            resolution=float(self.get_parameter("resolution").value),
            padding=float(self.get_parameter("padding").value),
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(OccupancyGrid, "/map", qos)
        self.timer = self.create_timer(float(self.get_parameter("publish_period_sec").value), self._publish)
        self._publish()
        self.get_logger().info(
            f"MecaMind map publisher ready: {self.geometry.name} "
            f"({self.spec.width}x{self.spec.height} @ {self.spec.resolution:.2f}m)"
        )

    def _publish(self) -> None:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.info.resolution = self.spec.resolution
        msg.info.width = self.spec.width
        msg.info.height = self.spec.height
        msg.info.origin.position.x = self.spec.origin_x
        msg.info.origin.position.y = self.spec.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = list(self.spec.data)
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MecaMindMapPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
