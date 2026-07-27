"""视觉跟随控制器：根据感知事件计算速度指令，让机器人跟着目标走。

在系统中的角色：感知 -> 控制 链路的最后一环。
- 订阅 /mecamind/perception_event（JSON）：perception_filter 输出的
  已确认目标事件（含归一化坐标 cx 和目标宽度 width）；
- 订阅 /mecamind/follow_enable（Bool）：跟随功能总开关，默认关闭，
  由任务调度/状态机在用户说"跟随我"之后打开；
- 发布 /cmd_vel_follow（Twist）：速度指令，送往速度仲裁器
  （再经安全门到 /cmd_vel）；勿直接抢 Nav2 的控制器入口。

控制思想（完整 PID）：
- 转向通道：误差 = cx - 0.5，输出 angular = Kp·e + Ki·∫e + Kd·de/dt；
- 距离通道：误差 = desired_width - target_width（正=偏远应前进），
  同样用 PID；过近时误差为负，自然后退。
- 死区、积分限幅、输出限幅防止震荡与积分饱和。

安全设计：
1. 使能开关：follow_enable 不打开则不动；
2. confirmed 门槛：只追过滤节点确认过的目标；
3. 看门狗超时：断流后刹停或原地慢转搜索；丢目标时清零积分。
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
class PidState:
    """单通道 PID 记忆：积分项与上一拍误差。"""

    integral: float = 0.0
    prev_error: float = 0.0


@dataclass(frozen=True)
class FollowCommand:
    """一次控制律计算的输出：线速度、角速度、是否有效及更新后的 PID 状态。"""

    linear_x: float
    angular_z: float
    active: bool
    reason: str
    angular_state: PidState = PidState()
    linear_state: PidState = PidState()


def _pid_step(
    error: float,
    state: PidState,
    kp: float,
    ki: float,
    kd: float,
    dt: float,
    i_limit: float,
) -> tuple[float, PidState]:
    """标准位置式 PID 一步；dt<=0 时退化为纯 P。"""
    if dt <= 1e-6:
        return kp * error, PidState(0.0, error)
    integral = state.integral + error * dt
    if i_limit > 0.0:
        integral = max(-i_limit, min(i_limit, integral))
    derivative = (error - state.prev_error) / dt
    output = kp * error + ki * integral + kd * derivative
    return output, PidState(integral=integral, prev_error=error)


def compute_follow_command(
    cx: float,
    target_width: float,
    desired_width: float = 0.25,
    center_deadband: float = 0.05,
    max_linear: float = 0.20,
    max_angular: float = 0.8,
    kp_angular: float = 2.0,
    ki_angular: float = 0.12,
    kd_angular: float = 0.06,
    kp_linear: float = 0.8,
    ki_linear: float = 0.05,
    kd_linear: float = 0.04,
    i_limit_angular: float = 0.45,
    i_limit_linear: float = 0.30,
    dt: float = 0.1,
    angular_state: PidState | None = None,
    linear_state: PidState | None = None,
) -> FollowCommand:
    """跟随控制律：PID 转向 + PID 距离保持。

    转向通道：
    - 误差 = cx - 0.5；死区内误差置 0，并缓慢衰减积分，减轻中心抖动；
    - 输出取反：图像偏右（正误差）应对应 ROS 顺时针（负 angular.z）。

    距离通道：
    - 误差 = desired_width - target_width（目标偏小/偏远 → 正误差 → 前进）；
    - 大转向时压低线速度，先对准再加速。
    """
    ang_state = angular_state or PidState()
    lin_state = linear_state or PidState()

    error_x = cx - 0.5
    if abs(error_x) < center_deadband:
        error_x = 0.0
        # 死区内衰减积分，避免饱和后出死区猛打方向
        ang_state = PidState(integral=ang_state.integral * 0.9, prev_error=0.0)

    angular_raw, ang_state = _pid_step(
        error_x,
        ang_state,
        kp=kp_angular,
        ki=ki_angular,
        kd=kd_angular,
        dt=dt,
        i_limit=i_limit_angular,
    )
    angular = max(-max_angular, min(max_angular, -angular_raw))

    if target_width <= 0.0:
        return FollowCommand(0.0, angular, True, "no_width", ang_state, lin_state)

    width_error = desired_width - target_width
    linear_raw, lin_state = _pid_step(
        width_error,
        lin_state,
        kp=kp_linear,
        ki=ki_linear,
        kd=kd_linear,
        dt=dt,
        i_limit=i_limit_linear,
    )
    # 过近时允许接近满速后退，略放宽负向限幅
    linear = max(-max_linear * 0.95, min(max_linear, linear_raw))

    turn = abs(cx - 0.5)
    if turn > center_deadband:
        turn_scale = max(0.25, 1.0 - (turn - center_deadband) / 0.40)
        linear *= turn_scale

    return FollowCommand(linear, angular, True, "target_confirmed", ang_state, lin_state)


def should_publish_follow(enabled: bool, confirmed: bool) -> bool:
    """双条件闸门：跟随开关已打开 且 目标已被过滤节点确认，才允许发速度。"""
    return bool(enabled) and bool(confirmed)


class VisionFollowController(Node):
    """视觉跟随节点：事件驱动发速度 + 定时看门狗兜底刹停。"""

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
        self.declare_parameter("ki_angular", 0.12)
        self.declare_parameter("kd_angular", 0.06)
        self.declare_parameter("kp_linear", 0.8)
        self.declare_parameter("ki_linear", 0.05)
        self.declare_parameter("kd_linear", 0.04)
        self.declare_parameter("i_limit_angular", 0.45)
        self.declare_parameter("i_limit_linear", 0.30)
        self.declare_parameter("search_on_loss", True)
        self.declare_parameter("search_angular", 0.55)

        self._enabled = False
        self._last_event_time = 0.0
        self._last_cmd_mono = 0.0
        self._searching = False
        self._ang_state = PidState()
        self._lin_state = PidState()
        self.create_subscription(String, str(self.get_parameter("input_topic").value), self._event_cb, 10)
        self.create_subscription(Bool, str(self.get_parameter("enable_topic").value), self._enable_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("output_cmd_topic").value), 10)
        self.timer = self.create_timer(0.1, self._watchdog)
        self.get_logger().info(
            "MecaMind vision follow controller ready (PID; follow disabled by default)"
        )

    def _reset_pid(self) -> None:
        self._ang_state = PidState()
        self._lin_state = PidState()
        self._last_cmd_mono = 0.0

    def _enable_cb(self, msg: Bool) -> None:
        """关闭跟随时立刻发零速，并清零 PID 积分。"""
        was_enabled = self._enabled
        self._enabled = bool(msg.data)
        if was_enabled and not self._enabled:
            self._last_event_time = 0.0
            self._searching = False
            self._reset_pid()
            self.cmd_pub.publish(Twist())
            self.get_logger().info("Vision follow disabled")
        elif not was_enabled and self._enabled:
            self._reset_pid()
            self.get_logger().info("Vision follow enabled (PID)")

    def _event_cb(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Invalid perception event: {exc}")
            return
        if not should_publish_follow(self._enabled, bool(event.get("confirmed", False))):
            return

        now = time.monotonic()
        dt = 0.1 if self._last_cmd_mono <= 0.0 else max(0.02, min(0.5, now - self._last_cmd_mono))
        command = compute_follow_command(
            float(event.get("cx", 0.5)),
            float(event.get("width", 0.0)),
            desired_width=float(self.get_parameter("desired_width").value),
            center_deadband=float(self.get_parameter("center_deadband").value),
            max_linear=float(self.get_parameter("max_linear").value),
            max_angular=float(self.get_parameter("max_angular").value),
            kp_angular=float(self.get_parameter("kp_angular").value),
            ki_angular=float(self.get_parameter("ki_angular").value),
            kd_angular=float(self.get_parameter("kd_angular").value),
            kp_linear=float(self.get_parameter("kp_linear").value),
            ki_linear=float(self.get_parameter("ki_linear").value),
            kd_linear=float(self.get_parameter("kd_linear").value),
            i_limit_angular=float(self.get_parameter("i_limit_angular").value),
            i_limit_linear=float(self.get_parameter("i_limit_linear").value),
            dt=dt,
            angular_state=self._ang_state,
            linear_state=self._lin_state,
        )
        self._ang_state = command.angular_state
        self._lin_state = command.linear_state
        self._last_cmd_mono = now

        twist = Twist()
        twist.linear.x = command.linear_x
        twist.angular.z = command.angular_z
        self._last_event_time = now
        self._searching = False
        self.cmd_pub.publish(twist)

    def _watchdog(self) -> None:
        if not self._enabled:
            return
        timeout = float(self.get_parameter("event_timeout_sec").value)
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
        self._reset_pid()
        if bool(self.get_parameter("search_on_loss").value):
            self._searching = True
            twist = Twist()
            twist.angular.z = float(self.get_parameter("search_angular").value)
            self.cmd_pub.publish(twist)
            self.get_logger().info("Target lost: start in-place search rotate", throttle_duration_sec=2.0)
        else:
            self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionFollowController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
