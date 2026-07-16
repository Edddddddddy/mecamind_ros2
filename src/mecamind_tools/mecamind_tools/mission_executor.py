from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from typing import Any, Dict, List, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .nav_utils import load_yaml, pose_stamped_from_dict


@dataclass(frozen=True)
class MissionState:
    mode: str
    last_intent: str = ""
    target: str = ""
    detail: str = ""

    def to_json(self) -> str:
        return json.dumps(asdict(self), ensure_ascii=False)


def normalize_goal_key(name: str) -> str:
    return name.strip().lower().replace(" ", "_").replace("-", "_")


def load_named_goals(path: str) -> Dict[str, Dict[str, Any]]:
    data = load_yaml(path)
    goals = data.get("goals", {})
    if not isinstance(goals, dict):
        raise ValueError(f"named goals root.goals must be a mapping: {path}")
    result: Dict[str, Dict[str, Any]] = {}
    for key, value in goals.items():
        if not isinstance(value, dict):
            continue
        result[normalize_goal_key(str(key))] = value
    return result


def resolve_named_goal(
    goals: Dict[str, Dict[str, Any]],
    target: str,
) -> Optional[Dict[str, Any]]:
    key = normalize_goal_key(target)
    if key in goals:
        return goals[key]
    aliases = {
        "客厅": "living_room",
        "卧室": "bedroom",
        "厨房": "kitchen",
        "大厅": "hall_entry",
        "门口": "hall_entry",
    }
    mapped = aliases.get(target.strip()) or aliases.get(key)
    if mapped and mapped in goals:
        return goals[mapped]
    return None


def load_patrol_waypoints(path: str) -> List[Dict[str, Any]]:
    data = load_yaml(path)
    waypoints = data.get("waypoints", [])
    if not isinstance(waypoints, list):
        raise ValueError(f"patrol waypoints must be a list: {path}")
    return [item for item in waypoints if isinstance(item, dict)]


def next_mission_mode(intent: str) -> str:
    normalized = intent.strip().lower()
    if normalized in {"stop", "cancel"}:
        return "idle"
    if normalized == "navigate":
        return "navigate"
    if normalized == "patrol":
        return "patrol"
    if normalized == "follow":
        return "follow"
    return "idle"


class MissionExecutorNode(Node):
    """Execute MecaMind task_command intents against Nav2 / follow enable."""

    def __init__(self) -> None:
        super().__init__("mecamind_mission_executor")
        self.declare_parameter("task_command_topic", "/mecamind/task_command")
        self.declare_parameter("mission_state_topic", "/mecamind/mission_state")
        self.declare_parameter("follow_enable_topic", "/mecamind/follow_enable")
        self.declare_parameter("cmd_vel_topic", "/controller/cmd_vel_nav")
        self.declare_parameter("named_goals_file", "")
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("goal_timeout_sec", 120.0)
        self.declare_parameter("navigate_action", "navigate_to_pose")

        goals_file = str(self.get_parameter("named_goals_file").value)
        waypoints_file = str(self.get_parameter("waypoints_file").value)
        if not goals_file:
            raise RuntimeError("named_goals_file parameter is required")
        if not waypoints_file:
            raise RuntimeError("waypoints_file parameter is required")

        self._goals = load_named_goals(goals_file)
        self._waypoints = load_patrol_waypoints(waypoints_file)
        self._goal_timeout = float(self.get_parameter("goal_timeout_sec").value)

        self._mode = "idle"
        self._last_intent = ""
        self._target = ""
        self._detail = "ready"
        self._patrol_index = 0
        self._goal_handle = None
        self._goal_sent_time: Optional[float] = None
        self._pending_send: Optional[Dict[str, Any]] = None
        self._goal_serial = 0

        action_name = str(self.get_parameter("navigate_action").value)
        self.navigate_client = ActionClient(self, NavigateToPose, action_name)
        self.state_pub = self.create_publisher(
            String, str(self.get_parameter("mission_state_topic").value), 10
        )
        self.follow_pub = self.create_publisher(
            Bool, str(self.get_parameter("follow_enable_topic").value), 10
        )
        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("task_command_topic").value),
            self._task_cb,
            10,
        )
        self.create_timer(0.5, self._tick)
        self._publish_follow(False)
        self._publish_state()
        self.get_logger().info(
            f"MecaMind mission executor ready with {len(self._goals)} named goals "
            f"and {len(self._waypoints)} patrol waypoints"
        )

    def _publish_follow(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = enabled
        self.follow_pub.publish(msg)

    def _publish_state(self) -> None:
        state = MissionState(
            mode=self._mode,
            last_intent=self._last_intent,
            target=self._target,
            detail=self._detail,
        )
        out = String()
        out.data = state.to_json()
        self.state_pub.publish(out)

    def _set_state(self, mode: str, detail: str, target: str = "") -> None:
        self._mode = mode
        self._detail = detail
        if target:
            self._target = target
        self._publish_state()

    def _task_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Invalid task_command JSON: {exc}")
            return
        intent = str(data.get("intent", "unknown")).strip().lower()
        target = str(data.get("target", "")).strip()
        requires_confirmation = bool(data.get("requires_confirmation", False))
        self._last_intent = intent
        self._target = target

        if requires_confirmation and intent not in {"stop", "cancel"}:
            self._set_state("idle", "awaiting_confirmation", target)
            return

        mode = next_mission_mode(intent)
        if mode == "idle":
            self._stop_all("stopped" if intent == "stop" else "canceled")
            return
        if mode == "follow":
            self._enter_follow()
            return
        if mode == "navigate":
            self._enter_navigate(target)
            return
        if mode == "patrol":
            self._enter_patrol()
            return
        self._set_state("idle", f"unsupported_intent:{intent}", target)

    def _stop_all(self, detail: str) -> None:
        self._publish_follow(False)
        self._cancel_nav()
        self.cmd_pub.publish(Twist())
        self._patrol_index = 0
        self._pending_send = None
        self._set_state("idle", detail)

    def _enter_follow(self) -> None:
        self._cancel_nav()
        self._pending_send = None
        self._patrol_index = 0
        self._publish_follow(True)
        self._set_state("follow", "follow_enabled")

    def _enter_navigate(self, target: str) -> None:
        goal = resolve_named_goal(self._goals, target)
        if goal is None:
            self._publish_follow(False)
            self._set_state("idle", f"unknown_goal:{target}", target)
            return
        self._publish_follow(False)
        self._cancel_nav()
        self._patrol_index = 0
        self._pending_send = {"kind": "navigate", "pose": goal, "name": normalize_goal_key(target)}
        self._set_state("navigate", f"queued:{normalize_goal_key(target)}", target)

    def _enter_patrol(self) -> None:
        if not self._waypoints:
            self._set_state("idle", "empty_patrol")
            return
        self._publish_follow(False)
        self._cancel_nav()
        self._patrol_index = 0
        first = self._waypoints[0]
        name = str(first.get("name", "wp0"))
        self._pending_send = {"kind": "patrol", "pose": first, "name": name}
        self._set_state("patrol", f"queued:{name}")

    def _cancel_nav(self) -> None:
        # Invalidate callbacks from a goal that is being preempted. This also
        # covers a send request whose response has not arrived yet.
        self._goal_serial += 1
        handle = self._goal_handle
        self._goal_handle = None
        self._goal_sent_time = None
        if handle is None:
            return
        try:
            handle.cancel_goal_async()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Cancel navigate goal failed: {exc}")

    def _tick(self) -> None:
        # Publish a low-rate heartbeat so late-joining classroom tools and smoke
        # tests can always observe the current state, not only transitions.
        self._publish_state()
        if self._mode not in {"navigate", "patrol"}:
            return
        if self._goal_handle is not None:
            self._poll_active_goal()
            return
        if self._pending_send is None:
            return
        if not self.navigate_client.server_is_ready():
            self._detail = "waiting_for_nav2"
            self._publish_state()
            return
        self._send_pending()

    def _send_pending(self) -> None:
        pending = self._pending_send
        if pending is None:
            return
        stamp = self.get_clock().now().to_msg()
        pose = pose_stamped_from_dict(pending["pose"], stamp)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._pending_send = None
        self._goal_serial += 1
        serial = self._goal_serial
        send_future = self.navigate_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda future, name=pending["name"], kind=pending["kind"], goal_serial=serial: (
                self._goal_response(future, name, kind, goal_serial)
            )
        )
        self._detail = f"sending:{pending['name']}"
        self._publish_state()

    def _goal_response(self, future, name: str, kind: str, serial: int) -> None:
        if serial != self._goal_serial:
            return
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Navigate goal send failed: {exc}")
            self._set_state("idle", f"send_failed:{name}")
            return
        if handle is None or not handle.accepted:
            self._set_state("idle", f"rejected:{name}")
            return
        self._goal_handle = handle
        self._goal_sent_time = self.get_clock().now().nanoseconds / 1e9
        self._detail = f"active:{name}"
        self._publish_state()
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda result_future, goal_name=name, goal_kind=kind, goal_serial=serial: self._goal_finished(
                result_future, goal_name, goal_kind, goal_serial
            )
        )

    def _poll_active_goal(self) -> None:
        if self._goal_sent_time is None or self._goal_timeout <= 0.0:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._goal_sent_time < self._goal_timeout:
            return
        self.get_logger().warn("Navigate goal timed out; canceling")
        self._cancel_nav()
        self._publish_follow(False)
        self.cmd_pub.publish(Twist())
        self._set_state("idle", "goal_timeout")

    def _goal_finished(self, future, name: str, kind: str, serial: int) -> None:
        if serial != self._goal_serial:
            return
        self._goal_handle = None
        self._goal_sent_time = None
        try:
            result = future.result()
            status = result.status
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Goal result error: {exc}")
            self._set_state("idle", f"result_error:{name}")
            return

        if self._mode == "idle":
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._set_state("idle", f"failed:{name}")
            return

        if kind == "navigate":
            self._set_state("idle", f"reached:{name}", name)
            return

        self._patrol_index += 1
        if self._patrol_index >= len(self._waypoints):
            self._set_state("idle", "patrol_complete")
            return
        nxt = self._waypoints[self._patrol_index]
        nxt_name = str(nxt.get("name", f"wp{self._patrol_index}"))
        self._pending_send = {"kind": "patrol", "pose": nxt, "name": nxt_name}
        self._set_state("patrol", f"queued:{nxt_name}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionExecutorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
