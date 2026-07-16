from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .aliyun_clients import AliyunLlmClient, AliyunTaskPlan


@dataclass(frozen=True)
class TaskCommand:
    intent: str
    target: str = ""
    requires_confirmation: bool = False
    reply: str = ""

    def to_json(self) -> str:
        return json.dumps(asdict(self), ensure_ascii=False)


def parse_task_command(text: str) -> TaskCommand:
    normalized = text.strip().lower()
    if not normalized:
        return TaskCommand("unknown", requires_confirmation=True, reply="I did not hear a command.")
    if any(word in normalized for word in ("stop", "停", "急停", "停车")):
        return TaskCommand("stop", reply="Stopping.")
    if any(word in normalized for word in ("cancel", "取消")):
        return TaskCommand("cancel", reply="Canceled.")
    if any(word in normalized for word in ("patrol", "巡逻")):
        return TaskCommand("patrol", reply="Starting patrol.")
    if any(word in normalized for word in ("follow", "跟随")):
        return TaskCommand("follow", reply="Following target.")
    if any(word in normalized for word in ("map", "建图")):
        return TaskCommand("mapping", reply="Starting mapping.")

    match = re.search(r"(?:go to|navigate to|去|导航到)\s*([\w\u4e00-\u9fff_-]+)", normalized)
    if match:
        target = match.group(1)
        return TaskCommand("navigate", target=target, reply=f"Navigating to {target}.")
    return TaskCommand("unknown", target=text.strip(), requires_confirmation=True, reply="Please confirm the task.")


def task_command_from_aliyun_plan(plan: AliyunTaskPlan) -> TaskCommand:
    return TaskCommand(
        intent=plan.intent,
        target=plan.target,
        requires_confirmation=plan.requires_confirmation,
        reply=plan.reply,
    )


class TaskSchedulerNode(Node):
    def __init__(self) -> None:
        super().__init__("mecamind_task_scheduler")
        self.declare_parameter("input_topic", "/mecamind/voice_command")
        self.declare_parameter("output_topic", "/mecamind/task_command")
        self.declare_parameter("reply_topic", "/mecamind/robot_reply")
        self.declare_parameter("provider", "rule")
        self.declare_parameter("llm_model", "qwen-plus")
        self.declare_parameter("dashscope_base_url", "https://dashscope.aliyuncs.com/compatible-mode/v1")
        self.declare_parameter("api_key_env", "DASHSCOPE_API_KEY")
        self.declare_parameter("llm_timeout_sec", 15.0)
        self.create_subscription(String, str(self.get_parameter("input_topic").value), self._cb, 10)
        self.pub = self.create_publisher(String, str(self.get_parameter("output_topic").value), 10)
        self.reply_pub = self.create_publisher(String, str(self.get_parameter("reply_topic").value), 10)
        self.llm_client = AliyunLlmClient(
            model=str(self.get_parameter("llm_model").value),
            api_key_env=str(self.get_parameter("api_key_env").value),
            base_url=str(self.get_parameter("dashscope_base_url").value),
            timeout_sec=float(self.get_parameter("llm_timeout_sec").value),
        )
        self.get_logger().info("MecaMind task scheduler ready")

    def _cb(self, msg: String) -> None:
        command = self._parse_command(msg.data)
        output = String()
        output.data = command.to_json()
        self.pub.publish(output)
        if command.reply:
            reply = String()
            reply.data = command.reply
            self.reply_pub.publish(reply)

    def _parse_command(self, text: str) -> TaskCommand:
        provider = str(self.get_parameter("provider").value).strip().lower()
        if provider not in ("aliyun", "llm"):
            return parse_task_command(text)
        try:
            return task_command_from_aliyun_plan(self.llm_client.parse_task(text))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Aliyun LLM task parsing failed, falling back to rules: {exc}")
            return parse_task_command(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskSchedulerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
