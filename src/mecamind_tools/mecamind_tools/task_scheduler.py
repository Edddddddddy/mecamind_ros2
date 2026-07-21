"""任务调度器：把语音识别出的自然语言指令解析成结构化任务命令。

在系统中的角色：语音链路的"大脑"，位于 ASR 和具体执行模块之间。
- 订阅 /mecamind/voice_command（String）：ASR 节点识别出的用户原话；
- 发布 /mecamind/task_command（JSON String）：结构化任务命令
  {intent, target, requires_confirmation, reply}，供导航/跟随/建图等
  执行模块消费；
- 发布 /mecamind/robot_reply（String）：给用户的口头回复，
  由 TTS 节点合成语音播报，形成"你说 -> 它答"的交互闭环。

支持两种解析方式（通过 provider 参数切换）：
1. "rule"（默认）：本地关键词/正则规则解析，零延迟、零成本、不依赖网络，
   适合课堂演示和离线环境；
2. "aliyun"/"llm"：调用通义千问大模型解析，能理解更口语化的表达
   （如"帮我去卧室看看"），失败时自动降级回规则解析——
   这个"云端优先、本地兜底"的降级策略是本文件最值得学习的设计。

初学者重点阅读：
1. parse_task_command —— 关键词规则解析（最朴素的 NLU）；
2. TaskSchedulerNode._parse_command —— LLM 失败自动降级的写法；
3. TaskCommand —— 全系统统一的任务命令数据结构。
"""

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
    """结构化任务命令：全系统统一的"任务"表达格式。

    与 aliyun_clients.AliyunTaskPlan 字段一致，但属于本模块自己的类型——
    刻意做这层转换是为了让下游模块只依赖 TaskCommand，不感知
    "命令是规则解析的还是 LLM 解析的"。字段含义：
    - intent: stop/cancel/patrol/follow/mapping/navigate/unknown；
    - target: 导航目标名（房间/路点）；
    - requires_confirmation: True 表示指令含糊，执行前需要用户确认；
    - reply: 播报给用户听的回复语。
    """

    intent: str
    target: str = ""
    requires_confirmation: bool = False
    reply: str = ""

    def to_json(self) -> str:
        """序列化成 JSON 字符串以便通过 String topic 发布。

        ensure_ascii=False 让中文目标名以原文而非 \\uXXXX 转义输出，
        方便用 ros2 topic echo 直接肉眼调试。
        """
        return json.dumps(asdict(self), ensure_ascii=False)


def parse_task_command(text: str) -> TaskCommand:
    """规则版指令解析：靠关键词匹配 + 正则提取目标，不依赖任何外部服务。

    设计要点：
    - 匹配顺序即优先级：stop（安全相关）放最前面——一句话里同时
      出现"停"和其他词时，必须优先理解为停止；
    - 中英文关键词都收录，配合 ASR 中英混识的输出；
    - navigate 用正则捕获"去/导航到/go to"后面的词作为目标名，
      字符类 [\\w\\u4e00-\\u9fff_-] 同时覆盖英文单词字符和 CJK 汉字区间；
    - 任何规则都没命中时返回 unknown 并置 requires_confirmation=True，
      把原话放进 target 留给人工确认——"听不懂就问"比"猜着执行"安全。
    """
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
    """把 LLM 客户端返回的 AliyunTaskPlan 转成本模块的 TaskCommand。

    纯字段搬运，存在的意义是隔离依赖方向：下游只 import 本模块的
    TaskCommand，将来换掉 LLM 供应商时只需要改这一个转换函数。
    """
    return TaskCommand(
        intent=plan.intent,
        target=plan.target,
        requires_confirmation=plan.requires_confirmation,
        reply=plan.reply,
    )


class TaskSchedulerNode(Node):
    """任务调度节点：订阅语音文本，解析成任务命令并广播，同时回话。

    provider 参数决定解析器：
    - "rule"（默认）：只用本地规则；
    - "aliyun" 或 "llm"：先试 LLM，失败自动降级到规则。
    LLM 客户端在构造时就创建好（即使 provider 是 rule 也创建），
    这样运行中把 provider 参数切到 llm 时无需重启节点。
    """

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
        """收到语音文本：解析 -> 发布任务命令 -> 有回复语则同时发去 TTS。"""
        command = self._parse_command(msg.data)
        output = String()
        output.data = command.to_json()
        self.pub.publish(output)
        # reply 为空就不发（不打扰 TTS），非空才播报。
        if command.reply:
            reply = String()
            reply.data = command.reply
            self.reply_pub.publish(reply)

    def _parse_command(self, text: str) -> TaskCommand:
        """根据 provider 参数选择解析器，LLM 失败时自动降级到规则解析。

        每次回调都重新读 provider 参数（而不是缓存在 __init__ 里），
        所以运行中用 ros2 param set 切换解析方式立刻生效。
        降级只记 warn 日志——网络抖动导致的 LLM 失败是预期内情况，
        不应该让整条语音链路瘫痪。
        """
        provider = str(self.get_parameter("provider").value).strip().lower()
        if provider not in ("aliyun", "llm"):
            return parse_task_command(text)
        try:
            return task_command_from_aliyun_plan(self.llm_client.parse_task(text))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Aliyun LLM task parsing failed, falling back to rules: {exc}")
            return parse_task_command(text)


def main(args=None) -> None:
    """节点入口：标准的 init -> spin -> 清理 生命周期。"""
    rclpy.init(args=args)
    node = TaskSchedulerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
