from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from .nav_utils import quaternion_from_yaw


class LifecycleActivator(Node):
    def __init__(self) -> None:
        super().__init__("mecamind_lifecycle_activator")
        self.declare_parameter(
            "node_names",
            [
                "amcl",
                "controller_server",
                "smoother_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
                "waypoint_follower",
                "velocity_smoother",
            ],
        )
        self.declare_parameter("service_timeout_sec", 20.0)
        self.declare_parameter("startup_delay_sec", 1.0)
        self.declare_parameter("publish_initial_pose", True)
        self.declare_parameter("initial_pose_frame_id", "map")
        self.declare_parameter("initial_pose_x", -3.2)
        self.declare_parameter("initial_pose_y", -2.4)
        self.declare_parameter("initial_pose_yaw", 0.0)
        self.declare_parameter("initial_pose_publish_count", 5)
        self.declare_parameter("initial_pose_period_sec", 0.5)
        self.declare_parameter("post_initial_pose_delay_sec", 2.0)

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos)

    def activate_all(self) -> bool:
        delay = float(self.get_parameter("startup_delay_sec").value)
        if delay > 0:
            self.get_logger().info(f"Waiting {delay:.1f}s before lifecycle activation")
            time.sleep(delay)
        node_names = [str(name) for name in self.get_parameter("node_names").value]
        ok = True
        for node_name in node_names:
            node_ok = self.configure_and_activate(node_name)
            ok = node_ok and ok
            if node_ok and node_name == "amcl" and bool(self.get_parameter("publish_initial_pose").value):
                self.publish_initial_pose()
        if ok:
            self.get_logger().info("All requested lifecycle nodes are active")
        else:
            self.get_logger().error("One or more lifecycle nodes failed to activate")
        return ok

    def publish_initial_pose(self) -> None:
        count = int(self.get_parameter("initial_pose_publish_count").value)
        period = float(self.get_parameter("initial_pose_period_sec").value)
        x = float(self.get_parameter("initial_pose_x").value)
        y = float(self.get_parameter("initial_pose_y").value)
        yaw = float(self.get_parameter("initial_pose_yaw").value)
        frame_id = str(self.get_parameter("initial_pose_frame_id").value)

        for index in range(count):
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.orientation = quaternion_from_yaw(yaw)
            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.0685
            self.initial_pose_pub.publish(msg)
            self.get_logger().info(
                f"Published initial pose {index + 1}/{count}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
            )
            time.sleep(period)

        delay = float(self.get_parameter("post_initial_pose_delay_sec").value)
        if delay > 0:
            self.get_logger().info(f"Waiting {delay:.1f}s for AMCL map->odom transform")
            time.sleep(delay)

    def configure_and_activate(self, node_name: str) -> bool:
        state = self.get_state(node_name)
        if state is None:
            return False
        if state == State.PRIMARY_STATE_ACTIVE:
            self.get_logger().info(f"{node_name} already active")
            return True
        if state == State.PRIMARY_STATE_UNCONFIGURED:
            if not self.change_state(node_name, Transition.TRANSITION_CONFIGURE):
                return False
            state = self.get_state(node_name)
        if state == State.PRIMARY_STATE_INACTIVE:
            return self.change_state(node_name, Transition.TRANSITION_ACTIVATE)
        self.get_logger().error(f"{node_name} is in unsupported state id={state}")
        return False

    def get_state(self, node_name: str):
        client = self.create_client(GetState, f"/{node_name}/get_state")
        timeout = float(self.get_parameter("service_timeout_sec").value)
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f"Timed out waiting for /{node_name}/get_state")
            return None
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            self.get_logger().error(f"Failed to get lifecycle state for {node_name}")
            return None
        state = future.result().current_state.id
        self.get_logger().info(f"{node_name} state={future.result().current_state.label} [{state}]")
        return state

    def change_state(self, node_name: str, transition_id: int) -> bool:
        client = self.create_client(ChangeState, f"/{node_name}/change_state")
        timeout = float(self.get_parameter("service_timeout_sec").value)
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f"Timed out waiting for /{node_name}/change_state")
            return False
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        result = future.result()
        if result is not None and result.success:
            self.get_logger().info(f"{node_name} transition {transition_id} succeeded")
            return True
        self.get_logger().error(f"{node_name} transition {transition_id} failed")
        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LifecycleActivator()
    try:
        ok = node.activate_all()
        raise SystemExit(0 if ok else 1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
