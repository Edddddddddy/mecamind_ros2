"""视觉跟随控制器：根据感知事件计算速度指令，让机器人跟着目标走。

在系统中的角色：感知 -> 控制 链路的最后一环。
- 订阅 /mecamind/perception_event（JSON）：perception_filter 输出的
  已确认目标事件（含归一化坐标 cx 和目标宽度 width）；
- 订阅 /mecamind/follow_enable（Bool）：跟随功能总开关，默认关闭，
  由任务调度/状态机在用户说"跟随我"之后打开；
- 发布 /cmd_vel_follow（Twist）：速度指令，送往第 5 课速度仲裁器
  （再经安全门到 /cmd_vel）；勿直接抢 Nav2 的控制器入口。

控制思想（教学版比例控制 P；计划书称 PID，课堂先掌握 P）：
- 转向：目标偏离画面中心多少（cx - 0.5），就按 kp_angular 反向转，
  让目标回到画面中央；
- 前进/后退：目标在画面里的宽度反映距离远近——目标越远看起来越小。
  用"期望宽度 - 实际宽度"的偏差按 kp_linear 给前进速度，实现"保持跟随距离"。

安全设计是本文件的重点，初学者务必理解这三层保护：
1. 使能开关：follow_enable 不打开，收到再多目标事件也不动；
2. confirmed 门槛：只响应过滤节点确认过的目标，不追误检；
3. 看门狗超时：目标事件断流超过 event_timeout_sec 秒就发零速刹停,
   防止"最后一条指令是前进"时目标丢失导致机器人一直往前冲。
   可选 search_on_loss：丢目标后改为原地慢转搜索，便于重新看见目标。

初学者重点阅读：
1. compute_follow_command —— 控制律本体（纯函数，可单独测试）；
2. _watchdog —— 超时刹停/搜索看门狗；
3. _enable_cb —— 关闭跟随时为什么要立刻发零速。
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


@dataclass(frozen=True)
class FollowCommand:
    """一次控制律计算的输出：线速度、角速度、是否有效及原因说明。

    用 dataclass 而不是直接返回 Twist，是为了让 compute_follow_command
    保持纯 Python（不依赖 ROS 消息类型），方便脱离 ROS 做单元测试。
    """

    linear_x: float
    angular_z: float
    active: bool
    reason: str


def compute_follow_command(
    cx: float,
    target_width: float,
    desired_width: float = 0.25,
    center_deadband: float = 0.05,
    max_linear: float = 0.20,
    max_angular: float = 0.8,
    kp_angular: float = 2.0,
    kp_linear: float = 0.8,
) -> FollowCommand:
    """跟随控制律：由目标水平位置和视觉宽度算出 (线速度, 角速度)。

    这是一个教学版比例控制器（P 控制器），分两个独立通道：

    转向通道：
    - 误差 = cx - 0.5（目标偏离画面中心的量，右偏为正）；
    - 死区 center_deadband 内不转向，避免目标在中心附近小幅抖动时
      机器人不停地左右摆（震荡）；
    - 比例增益 -kp_angular：负号是因为图像坐标系里目标偏右（误差为正）时，
      机器人应向右转，而 ROS 的 angular.z 正方向是逆时针（左转），
      所以要取反；
    - 最后限幅到 [-max_angular, max_angular]，防止误差大时输出危险的角速度。

    前进通道（远快近慢）：
    - 用目标在画面里的宽度近似距离：越远看起来越小；
    - proximity = target_width / desired_width：<1 偏远、=1 合适、>1 过近；
    - 偏远时线速度随距离增大（最大 max_linear）；过近时明显后退（防顶柱/穿模）；
    - kp_linear 作为灵敏度（>1 更积极追赶）；
    - 转向偏差大时自动压低线速度，避免边转边冲把目标甩出画面；
    - target_width <= 0 时距离未知，线速度为 0。

    源码位置：本函数；讲义 §2.2 / 源码走读第 3 章。
    """
    error_x = cx - 0.5
    angular = 0.0 if abs(error_x) < center_deadband else -kp_angular * error_x
    angular = max(-max_angular, min(max_angular, angular))

    if target_width <= 0.0:
        return FollowCommand(0.0, angular, True, "no_width")

    proximity = target_width / max(1e-3, desired_width)
    if proximity < 1.0:
        # 越远（proximity 越小）越快；kp_linear 放大追赶积极性
        linear = max_linear * (1.0 - proximity) * max(0.5, kp_linear)
        linear = min(max_linear, linear)
    else:
        # 过近：明显后退，避免顶到/穿进目标
        overshoot = min(2.0, proximity - 1.0)
        linear = -max_linear * 0.95 * min(1.0, overshoot / 0.6)

    # 大转向时压线速度，先对准再加速
    turn = abs(error_x)
    if turn > center_deadband:
        turn_scale = max(0.25, 1.0 - (turn - center_deadband) / 0.40)
        linear *= turn_scale

    return FollowCommand(linear, angular, True, "target_confirmed")


def should_publish_follow(enabled: bool, confirmed: bool) -> bool:
    """双条件闸门：跟随开关已打开 且 目标已被过滤节点确认，才允许发速度。

    抽成独立函数是为了让这条安全规则可以被单元测试覆盖，
    并在代码里有一个显眼的"发速度前必须过这道闸"的落点。
    """
    return bool(enabled) and bool(confirmed)


class VisionFollowController(Node):
    """视觉跟随节点：事件驱动发速度 + 定时看门狗兜底刹停。

    两个回调协同工作：
    - _event_cb（事件驱动）：每收到一条确认目标事件就算一次控制律并发速度；
    - _watchdog（10 Hz 定时器）：发现事件断流超时就发零速。
    速度指令默认发到 /cmd_vel_follow，由 cmd_vel_arbiter 与遥控/导航仲裁后再进安全门。
    """

    def __init__(self) -> None:
        super().__init__("mecamind_vision_follow_controller")
        self.declare_parameter("input_topic", "/mecamind/perception_event")
        self.declare_parameter("output_cmd_topic", "/cmd_vel_follow")
        self.declare_parameter("enable_topic", "/mecamind/follow_enable")
        self.declare_parameter("desired_width", 0.25)
        self.declare_parameter("event_timeout_sec", 0.70)
        self.declare_parameter("max_linear", 0.20)
        self.declare_parameter("max_angular", 0.8)
        self.declare_parameter("center_deadband", 0.05)
        self.declare_parameter("kp_angular", 2.0)
        self.declare_parameter("kp_linear", 0.8)
        # 丢目标后原地慢转搜索（柱子遮挡、拐弯出画面时的恢复手段）
        self.declare_parameter("search_on_loss", True)
        self.declare_parameter("search_angular", 0.55)

        # 安全默认值：启动即禁用，必须显式发 True 到 enable topic 才会动。
        self._enabled = False
        # 最近一次发出速度指令的单调时钟时间；0.0 表示"当前没有在跟随"。
        self._last_event_time = 0.0
        self._searching = False
        self.create_subscription(String, str(self.get_parameter("input_topic").value), self._event_cb, 10)
        self.create_subscription(Bool, str(self.get_parameter("enable_topic").value), self._enable_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("output_cmd_topic").value), 10)
        # 看门狗周期 0.1 s，远小于默认超时 0.7 s，保证超时检测足够及时。
        self.timer = self.create_timer(0.1, self._watchdog)
        self.get_logger().info("MecaMind vision follow controller ready (follow disabled by default)")

    def _enable_cb(self, msg: Bool) -> None:
        """处理跟随开关。关键点：关闭的瞬间必须立刻发一帧零速 Twist()。

        如果只是停止发新指令，底盘会保持最后一条速度继续运动
        （多数速度控制器在没有新指令时短暂维持旧值），所以"关掉"
        必须是一个主动刹停动作，而不是被动的"不再说话"。
        """
        was_enabled = self._enabled
        self._enabled = bool(msg.data)
        if was_enabled and not self._enabled:
            self._last_event_time = 0.0
            self._searching = False
            self.cmd_pub.publish(Twist())  # 全零 Twist = 刹停
            self.get_logger().info("Vision follow disabled")
        elif not was_enabled and self._enabled:
            self.get_logger().info("Vision follow enabled")

    def _event_cb(self, msg: String) -> None:
        """收到感知事件：过安全闸门 -> 算控制律 -> 发布速度指令。"""
        # 步骤 1：解析事件 JSON，坏数据只警告不崩溃。
        try:
            event = json.loads(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Invalid perception event: {exc}")
            return
        # 步骤 2：安全闸门——未使能或目标未确认，直接忽略这条事件。
        if not should_publish_follow(self._enabled, bool(event.get("confirmed", False))):
            return
        # 步骤 3：用事件里的 cx 和 width 计算速度。缺字段时的兜底值
        # （cx=0.5 居中、width=0.0 距离未知）都会让控制律输出零速，安全。
        command = compute_follow_command(
            float(event.get("cx", 0.5)),
            float(event.get("width", 0.0)),
            desired_width=float(self.get_parameter("desired_width").value),
            center_deadband=float(self.get_parameter("center_deadband").value),
            max_linear=float(self.get_parameter("max_linear").value),
            max_angular=float(self.get_parameter("max_angular").value),
            kp_angular=float(self.get_parameter("kp_angular").value),
            kp_linear=float(self.get_parameter("kp_linear").value),
        )
        # 步骤 4：换算成 ROS Twist 消息并发布，同时刷新看门狗时间戳。
        # 用 time.monotonic() 而不是 time.time()：单调时钟不受系统时间
        # 跳变（NTP 校时等）影响，做超时判断必须用它。
        twist = Twist()
        twist.linear.x = command.linear_x
        twist.angular.z = command.angular_z
        self._last_event_time = time.monotonic()
        self._searching = False
        self.cmd_pub.publish(twist)

    def _watchdog(self) -> None:
        """看门狗（10 Hz）：跟随中若事件断流超时，刹停或慢转搜索。

        三个提前返回分别对应：未使能（本来就不该动）、尚未开始跟随
        （_last_event_time 为 0 且未在搜索）、事件仍然新鲜。
        超时后进入搜索态：search_on_loss 时持续发慢转，否则发一次零速。
        """
        if not self._enabled:
            return
        timeout = float(self.get_parameter("event_timeout_sec").value)
        # 已在搜索：持续慢转，直到再次看到 confirmed 目标
        if self._searching:
            if bool(self.get_parameter("search_on_loss").value):
                twist = Twist()
                twist.angular.z = float(self.get_parameter("search_angular").value)
                self.cmd_pub.publish(twist)
            return
        if self._last_event_time <= 0.0:
            return
        if time.monotonic() - self._last_event_time <= timeout:
            return
        self._last_event_time = 0.0
        if bool(self.get_parameter("search_on_loss").value):
            self._searching = True
            twist = Twist()
            twist.angular.z = float(self.get_parameter("search_angular").value)
            self.cmd_pub.publish(twist)
            self.get_logger().info("Target lost: start in-place search rotate", throttle_duration_sec=2.0)
        else:
            self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    """节点入口：标准的 init -> spin -> 清理 生命周期。"""
    rclpy.init(args=args)
    node = VisionFollowController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
