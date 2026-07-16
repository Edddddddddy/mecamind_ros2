from __future__ import annotations

import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from .nav_utils import initial_pose_from_dict, load_yaml, pose_stamped_from_dict


def _default_waypoints_file() -> str:
    return (
        get_package_share_directory("mecamind_tools")
        + "/config/mecamind_patrol_waypoints.yaml"
    )


def _wait_for_task(navigator: BasicNavigator, timeout_sec: float, label: str) -> bool:
    start = time.monotonic()
    while not navigator.isTaskComplete():
        if timeout_sec > 0 and time.monotonic() - start > timeout_sec:
            navigator.get_logger().warn(f"{label} timed out after {timeout_sec:.1f}s; canceling")
            navigator.cancelTask()
            return False
        time.sleep(0.2)
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info(f"{label} succeeded")
        return True
    if result == TaskResult.CANCELED:
        navigator.get_logger().warn(f"{label} canceled")
    else:
        navigator.get_logger().error(f"{label} failed with result={result}")
    return False


def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()
    navigator.declare_parameter("waypoints_file", "")

    path = str(navigator.get_parameter("waypoints_file").value) or _default_waypoints_file()
    config = load_yaml(path)
    now = navigator.get_clock().now().to_msg()

    initial_pose = initial_pose_from_dict(config["initial_pose"], now)
    navigator.setInitialPose(initial_pose)
    navigator.get_logger().info(f"Loaded patrol waypoints from {path}")

    try:
        navigator.waitUntilNav2Active()
    except TypeError:
        navigator.waitUntilNav2Active(localizer="amcl")

    loop_count = int(config.get("loop_count", 1))
    timeout_sec = float(config.get("goal_timeout_sec", 120.0))
    wait_sec = float(config.get("wait_between_goals_sec", 0.0))
    waypoints = config.get("waypoints", [])

    for loop_index in range(loop_count):
        navigator.get_logger().info(f"Starting patrol loop {loop_index + 1}/{loop_count}")
        for item in waypoints:
            name = item.get("name", "unnamed")
            goal = pose_stamped_from_dict(item, navigator.get_clock().now().to_msg())
            navigator.get_logger().info(
                f"Navigating to {name}: x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}"
            )
            navigator.goToPose(goal)
            _wait_for_task(navigator, timeout_sec, f"Waypoint {name}")
            if wait_sec > 0:
                time.sleep(wait_sec)

    navigator.get_logger().info("Patrol complete")
    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
