from __future__ import annotations

import math
import subprocess
import time
from collections import deque
from typing import List, Optional, Set, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener

from .nav_utils import quaternion_from_yaw


GridCell = Tuple[int, int]


class FrontierExplorer(Node):
    """Frontier-based exploration node for MecaMind mapping demos."""

    def __init__(self) -> None:
        super().__init__("mecamind_frontier_explorer")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("min_frontier_cells", 12)
        self.declare_parameter("goal_timeout_sec", 90.0)
        self.declare_parameter("goal_reached_radius", 0.45)
        self.declare_parameter("frontier_blacklist_radius", 0.65)
        self.declare_parameter("map_save_basename", "")

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.min_frontier_cells = int(self.get_parameter("min_frontier_cells").value)
        self.goal_timeout = float(self.get_parameter("goal_timeout_sec").value)
        self.goal_reached_radius = float(self.get_parameter("goal_reached_radius").value)
        self.blacklist_radius = float(self.get_parameter("frontier_blacklist_radius").value)

        self.latest_map: Optional[OccupancyGrid] = None
        self.goal_handle = None
        self.goal_sent_time = None
        self.current_goal_xy: Optional[Tuple[float, float]] = None
        self.blacklist: List[Tuple[float, float]] = []
        self._map_saved = False

        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigate_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        map_topic = str(self.get_parameter("map_topic").value)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, map_topic, self._map_callback, map_qos)
        self.timer = self.create_timer(2.0, self._tick)
        self.get_logger().info("Frontier explorer waiting for map and Nav2 action server")

    def _map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def _tick(self):
        if self.latest_map is None:
            self.get_logger().info("Waiting for /map ...")
            return
        if not self.navigate_client.server_is_ready():
            self.get_logger().info("Waiting for navigate_to_pose action server ...")
            return
        if self.goal_handle is not None:
            self._check_goal_timeout()
            return

        robot_xy = self._robot_xy()
        if robot_xy is None:
            return
        frontier = self._choose_frontier(robot_xy)
        if frontier is None:
            self.get_logger().info("No usable frontier found; exploration appears complete")
            self._save_map_if_requested()
            return
        self._send_goal(frontier, robot_xy)

    def _robot_xy(self) -> Optional[Tuple[float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(f"TF {self.map_frame}->{self.base_frame} unavailable: {exc}")
            return None
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def _choose_frontier(self, robot_xy: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        grid = self.latest_map
        assert grid is not None
        width = grid.info.width
        height = grid.info.height
        data = grid.data

        frontiers: Set[GridCell] = set()
        for y in range(1, height - 1):
            row = y * width
            for x in range(1, width - 1):
                value = data[row + x]
                if 0 <= value <= 20 and self._has_unknown_neighbor(data, width, x, y):
                    frontiers.add((x, y))

        clusters = self._cluster_frontiers(frontiers)
        candidates = []
        for cluster in clusters:
            if len(cluster) < self.min_frontier_cells:
                continue
            world_xy = self._cluster_centroid_world(cluster)
            if self._is_blacklisted(world_xy):
                continue
            distance = math.hypot(world_xy[0] - robot_xy[0], world_xy[1] - robot_xy[1])
            if distance < self.goal_reached_radius:
                continue
            candidates.append((distance, len(cluster), world_xy))

        if not candidates:
            return None
        candidates.sort(key=lambda item: (item[0], -item[1]))
        return candidates[0][2]

    @staticmethod
    def _has_unknown_neighbor(data, width: int, x: int, y: int) -> bool:
        offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        return any(data[(y + dy) * width + (x + dx)] == -1 for dx, dy in offsets)

    @staticmethod
    def _cluster_frontiers(frontiers: Set[GridCell]) -> List[List[GridCell]]:
        clusters: List[List[GridCell]] = []
        while frontiers:
            start = frontiers.pop()
            cluster = [start]
            queue = deque([start])
            while queue:
                x, y = queue.popleft()
                for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                    item = (nx, ny)
                    if item in frontiers:
                        frontiers.remove(item)
                        cluster.append(item)
                        queue.append(item)
            clusters.append(cluster)
        return clusters

    def _cluster_centroid_world(self, cluster: List[GridCell]) -> Tuple[float, float]:
        grid = self.latest_map
        assert grid is not None
        mean_x = sum(cell[0] for cell in cluster) / len(cluster)
        mean_y = sum(cell[1] for cell in cluster) / len(cluster)
        origin = grid.info.origin.position
        resolution = grid.info.resolution
        return (
            origin.x + (mean_x + 0.5) * resolution,
            origin.y + (mean_y + 0.5) * resolution,
        )

    def _is_blacklisted(self, point: Tuple[float, float]) -> bool:
        return any(
            math.hypot(point[0] - bad[0], point[1] - bad[1]) < self.blacklist_radius
            for bad in self.blacklist
        )

    def _send_goal(self, point: Tuple[float, float], robot_xy: Tuple[float, float]):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = point[0]
        goal.pose.pose.position.y = point[1]
        yaw = math.atan2(point[1] - robot_xy[1], point[0] - robot_xy[0])
        goal.pose.pose.orientation = quaternion_from_yaw(yaw)

        self.current_goal_xy = point
        self.goal_sent_time = self.get_clock().now()
        self.get_logger().info(f"Sending frontier goal: x={point[0]:.2f}, y={point[1]:.2f}")
        future = self.navigate_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Frontier goal rejected")
            if self.current_goal_xy:
                self.blacklist.append(self.current_goal_xy)
            self.goal_handle = None
            self.current_goal_xy = None
            return
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Frontier goal reached")
        else:
            self.get_logger().warn(f"Frontier goal failed with status={result.status}")
            if self.current_goal_xy:
                self.blacklist.append(self.current_goal_xy)
        self.goal_handle = None
        self.goal_sent_time = None
        self.current_goal_xy = None

    def _check_goal_timeout(self):
        if self.goal_sent_time is None or self.goal_handle is None:
            return
        elapsed = (self.get_clock().now() - self.goal_sent_time).nanoseconds / 1e9
        if elapsed <= self.goal_timeout:
            return
        self.get_logger().warn("Frontier goal timed out; canceling and blacklisting")
        if self.current_goal_xy:
            self.blacklist.append(self.current_goal_xy)
        self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.goal_sent_time = None
        self.current_goal_xy = None

    def _save_map_if_requested(self) -> None:
        basename = str(self.get_parameter("map_save_basename").value).strip()
        if not basename or self._map_saved:
            return
        self.get_logger().info(f"Saving exploration map to {basename}.pgm/.yaml")
        try:
            completed = subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", basename],
                check=False,
                text=True,
                capture_output=True,
                timeout=15,
            )
        except Exception as exc:  # noqa: BLE001 - logged for teaching diagnostics
            self.get_logger().error(f"Map save command failed to start: {exc}")
            return
        if completed.returncode == 0:
            self.get_logger().info("Map save completed")
            self._map_saved = True
        else:
            self.get_logger().error(completed.stderr.strip() or completed.stdout.strip())


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
