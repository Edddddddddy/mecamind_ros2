"""Gazebo 红色跟随目标驱动：客厅南侧空地绕圈（第 5 课演示专用）。

【在系统中的角色】
只服务「课堂看得见」的演示，不参与控制律本身。
- 通过 `gz service .../set_pose` 周期性改写模型位姿；
- 默认驱动模型名 `follow_target`（红柱）；
- 可选同时把小车摆回出生位姿（防仿真漂移/叠实例搞乱初始化）。

【为什么绕圈而不是朝车直线往返】
直线若落在车头方向，红柱会顶到/穿进车身，观感差也难跟。
改为车南方空地单向闭环：始终在视野前方，并与小车保持距离。

默认几何（可被 launch 参数覆盖）：
  圆心约 (-2.3, -1.2)、半径约 1.1m 的六边形近似圆
  小车 (-2.0, 1.95) 朝南旁观

【时序】
1. startup_delay：等 Gazebo 服务起来；
2. _place_start：确认小车/红柱出生位姿；
3. 等到 /mecamind/follow_enable=true（可关）；
4. follow_start_delay 后再沿折线推进弧长 s。

初学者重点阅读：
1. closed_loop 如何把折线首尾闭合；
2. _on_timer 里弧长 s 的推进与 set_pose；
3. _follow_enable_cb 上升沿为何要重置 _placed（重新摆位再开跑）。
"""

from __future__ import annotations

import ast
import math
import shutil
import subprocess
import time
from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

Point = Tuple[float, float]


def _parse_waypoints(raw: object) -> List[Point]:
    """解析 waypoints_xy：支持扁平 [x0,y0,x1,y1,...] 或嵌套 [[x,y],...]。"""
    if isinstance(raw, str):
        data = ast.literal_eval(raw)
    else:
        data = raw
    if not isinstance(data, (list, tuple)) or len(data) < 4:
        raise ValueError(f"waypoints 至少需要 2 个点，收到: {raw!r}")
    if isinstance(data[0], (list, tuple)):
        pts = [(float(p[0]), float(p[1])) for p in data]
    else:
        if len(data) % 2 != 0:
            raise ValueError(f"扁平 waypoints 长度必须为偶数: {raw!r}")
        pts = [(float(data[i]), float(data[i + 1])) for i in range(0, len(data), 2)]
    if len(pts) < 2:
        raise ValueError("waypoints 至少 2 个点")
    return pts


def _polyline_length(pts: Sequence[Point]) -> float:
    """折线总弧长（米）。"""
    total = 0.0
    for i in range(1, len(pts)):
        x0, y0 = pts[i - 1]
        x1, y1 = pts[i]
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def _point_on_polyline(pts: Sequence[Point], dist: float) -> Point:
    """沿折线弧长 dist 取点；超出末端则钳在终点。"""
    if dist <= 0.0:
        return pts[0]
    traveled = 0.0
    for i in range(1, len(pts)):
        x0, y0 = pts[i - 1]
        x1, y1 = pts[i]
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg < 1e-6:
            continue
        if traveled + seg >= dist:
            t = (dist - traveled) / seg
            return (x0 + t * (x1 - x0), y0 + t * (y1 - y0))
        traveled += seg
    return pts[-1]


class FollowTargetMover(Node):
    """定时器驱动：沿 waypoints 折线（可闭环）推进红柱位姿。"""

    def __init__(self) -> None:
        super().__init__("mecamind_follow_target_mover")
        self.declare_parameter("world_name", "three_room_house")
        self.declare_parameter("model_name", "follow_target")
        # 客厅南侧空地绕圈（1.5x 放大后的更大半径）
        self.declare_parameter(
            "waypoints_xy",
            "[-3.45,-0.15, -4.875,-0.975, -4.275,-3.225, -3.45,-3.45, -2.625,-3.225, -2.025,-0.975]",
        )
        self.declare_parameter("z", 0.38)
        self.declare_parameter("speed_mps", 0.14)
        self.declare_parameter("rate_hz", 4.0)
        self.declare_parameter("enabled", True)
        self.declare_parameter("ping_pong", False)
        self.declare_parameter("closed_loop", True)
        self.declare_parameter("startup_delay_sec", 6.0)
        self.declare_parameter("wait_for_follow_enable", True)
        self.declare_parameter("follow_enable_topic", "/mecamind/follow_enable")
        self.declare_parameter("follow_start_delay_sec", 2.0)
        self.declare_parameter("reposition_robot", True)
        self.declare_parameter("robot_name", "mecamind_mecanum")
        self.declare_parameter("robot_x", -3.0)
        self.declare_parameter("robot_y", 2.925)
        self.declare_parameter("robot_z", 0.10)
        self.declare_parameter("robot_yaw", -math.pi / 2.0)

        self._gz_bin = shutil.which("gz")
        if not self._gz_bin:
            self.get_logger().error("未找到 gz 可执行文件，无法驱动跟随目标")

        try:
            self._waypoints = _parse_waypoints(self.get_parameter("waypoints_xy").value)
        except ValueError as exc:
            self.get_logger().error(f"waypoints 无效，回退默认绕圈: {exc}")
            self._waypoints = [
                (-3.45, -0.15),
                (-4.875, -0.975),
                (-4.275, -3.225),
                (-3.45, -3.45),
                (-2.625, -3.225),
                (-2.025, -0.975),
            ]
        # closed_loop=True 时把起点追加到末尾，才能用 fmod(s) 真正绕圈
        self._path_pts = list(self._waypoints)
        if bool(self.get_parameter("closed_loop").value):
            if math.hypot(
                self._path_pts[0][0] - self._path_pts[-1][0],
                self._path_pts[0][1] - self._path_pts[-1][1],
            ) > 1e-3:
                self._path_pts.append(self._path_pts[0])
        self._path_len = max(0.1, _polyline_length(self._path_pts))
        self._s = 0.0  # 当前弧长位置
        self._direction = 1.0  # ping_pong 时用；闭环模式保持 +1
        self._busy = False  # set_pose 进行中，避免重入
        self._placed = False
        self._place_tries = 0
        self._follow_armed = not bool(self.get_parameter("wait_for_follow_enable").value)
        self._follow_armed_wall: Optional[float] = (
            time.monotonic() if self._follow_armed else None
        )
        self._boot_wall = time.monotonic()
        self._last_tick_wall = self._boot_wall

        self.create_subscription(
            Bool,
            str(self.get_parameter("follow_enable_topic").value),
            self._follow_enable_cb,
            10,
        )
        rate = max(1.0, float(self.get_parameter("rate_hz").value))
        self._timer = self.create_timer(1.0 / rate, self._on_timer)
        mode = "绕圈" if bool(self.get_parameter("closed_loop").value) else "往返"
        pts = " -> ".join(f"({x:.2f},{y:.2f})" for x, y in self._waypoints)
        self.get_logger().info(
            f"follow_target_mover {mode}: {pts} | "
            f"len={self._path_len:.2f}m speed={float(self.get_parameter('speed_mps').value):.2f}m/s "
            f"| 等跟随使能={bool(self.get_parameter('wait_for_follow_enable').value)}"
        )

    def _follow_enable_cb(self, msg: Bool) -> None:
        """跟随使能上升沿：重置轨迹并从出生点重新摆位后再开跑。

        注意：只在 False→True 时重置；重复的 True 不重置，避免自动连发
        把已经在绕圈的红柱反复拽回起点。
        """
        if bool(msg.data):
            if self._follow_armed:
                return
            self._follow_armed = True
            self._follow_armed_wall = time.monotonic()
            self._s = 0.0
            self._direction = 1.0
            self._placed = False
            self._place_tries = 0
            self.get_logger().info("跟随已使能：摆好小车/红柱后开始绕圈")
        else:
            if self._follow_armed:
                self.get_logger().info("跟随已关闭：红柱暂停")
            self._follow_armed = False
            self._follow_armed_wall = None

    def _on_timer(self) -> None:
        """心跳：先摆位 → 等使能 → 沿折线推进并 set_pose。"""
        if (
            not bool(self.get_parameter("enabled").value)
            or not self._gz_bin
            or self._busy
        ):
            return
        if time.monotonic() - self._boot_wall < float(
            self.get_parameter("startup_delay_sec").value
        ):
            return

        # 启动后先摆一次出生位姿（世界文件也会出生，这里再确认）
        if not self._placed:
            self._place_start()
            return

        if bool(self.get_parameter("wait_for_follow_enable").value) and not self._follow_armed:
            return

        start_delay = float(self.get_parameter("follow_start_delay_sec").value)
        if (
            self._follow_armed_wall is not None
            and time.monotonic() - self._follow_armed_wall < start_delay
        ):
            return

        now = time.monotonic()
        dt = max(0.0, min(now - self._last_tick_wall, 0.5))
        self._last_tick_wall = now
        speed = max(0.02, float(self.get_parameter("speed_mps").value))
        self._s += self._direction * speed * dt

        # ping_pong 仅在非闭环时生效；闭环用 fmod 单向循环
        if bool(self.get_parameter("ping_pong").value) and not bool(
            self.get_parameter("closed_loop").value
        ):
            if self._s >= self._path_len:
                self._s = self._path_len
                self._direction = -1.0
            elif self._s <= 0.0:
                self._s = 0.0
                self._direction = 1.0
        elif self._s >= self._path_len:
            self._s = math.fmod(self._s, self._path_len)

        x, y = _point_on_polyline(self._path_pts, self._s)
        ok = self._set_pose(
            str(self.get_parameter("model_name").value),
            x,
            y,
            float(self.get_parameter("z").value),
        )
        if ok:
            self.get_logger().info(
                f"红柱 s={self._s:.2f}/{self._path_len:.2f} -> ({x:.2f},{y:.2f})",
                throttle_duration_sec=4.0,
            )

    def _place_start(self) -> None:
        """把小车/红柱摆到演示出生点；失败会重试若干次。"""
        self._place_tries += 1
        ok_robot = True
        if bool(self.get_parameter("reposition_robot").value):
            ok_robot = self._set_pose(
                str(self.get_parameter("robot_name").value),
                float(self.get_parameter("robot_x").value),
                float(self.get_parameter("robot_y").value),
                float(self.get_parameter("robot_z").value),
                yaw=float(self.get_parameter("robot_yaw").value),
            )
        x0, y0 = self._waypoints[0]
        ok_target = self._set_pose(
            str(self.get_parameter("model_name").value),
            x0,
            y0,
            float(self.get_parameter("z").value),
        )
        if (ok_robot and ok_target) or self._place_tries >= 8:
            self._placed = True
            self._s = 0.0
            self._direction = 1.0
            self._last_tick_wall = time.monotonic()
            rx = float(self.get_parameter("robot_x").value)
            ry = float(self.get_parameter("robot_y").value)
            self.get_logger().info(
                f"出生位姿已确认：小车=({rx:.1f},{ry:.2f})朝南，红柱=({x0:.2f},{y0:.2f}) "
                f"(robot_ok={ok_robot}, target_ok={ok_target})"
            )

    def _set_pose(
        self,
        name: str,
        x: float,
        y: float,
        z: float,
        yaw: Optional[float] = None,
    ) -> bool:
        """调用 Gazebo Harmonic 的 /world/<name>/set_pose 服务。

        成功判据：进程返回码 0 且 stdout 含 `data: true`。
        忙碌/超时时返回 False，由上层重试或跳过本拍。
        """
        world = str(self.get_parameter("world_name").value)
        req = f'name: "{name}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}'
        if yaw is not None:
            half = 0.5 * yaw
            req += (
                f", orientation: {{x: 0, y: 0, "
                f"z: {math.sin(half):.6f}, w: {math.cos(half):.6f}}}"
            )
        cmd = [
            self._gz_bin,
            "service",
            "-s",
            f"/world/{world}/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "1500",
            "--req",
            req,
        ]
        self._busy = True
        try:
            result = subprocess.run(
                cmd, check=False, capture_output=True, text=True, timeout=2.5
            )
        except (subprocess.TimeoutExpired, OSError) as exc:
            self.get_logger().warning(
                f"set_pose 失败: {exc}", throttle_duration_sec=5.0
            )
            return False
        finally:
            self._busy = False
        ok = result.returncode == 0 and "data: true" in (result.stdout or "")
        if not ok:
            detail = (result.stderr or result.stdout or "").strip()
            self.get_logger().warning(
                f"set_pose 未成功: {detail[:160]}",
                throttle_duration_sec=5.0,
            )
        return ok


def main(args: Optional[list] = None) -> None:
    """节点入口。"""
    rclpy.init(args=args)
    node = FollowTargetMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
